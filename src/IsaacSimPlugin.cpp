/* IsaacSim mc_rtc global plugin (client side of mc_isaac).
 *
 * init():   read the configuration (+ MC_ISAAC_* overrides of the mc_isaac command), start/wait for the server,
 *           build the scene spec from gc.robots() and their <key>.yaml descriptions (USDs uploaded by sha256),
 *           POST /scene, connect the lockstep bridge, map Isaac joints <-> mc_rtc joints.
 * before(): last Isaac state -> mc_rtc sensors (encoders, joint velocities/torques, FloatingBase, IMUs, force sensors).
 * after():  mc_rtc output robots -> Isaac commands (q, alpha, tau in Isaac joint order), one bridge "step" request
 *           = Timestep / physics_dt PhysX substeps, then the reply is handled (Isaac events, panel commands, marker
 *           edits, server logs).
 * The server protocol is documented at the top of server/mc_isaac_server.py.
 */

#include "IsaacSimPlugin.h"

#include "Net.h"
#include "Sha256.h"

#include <mc_control/GlobalPluginMacros.h>
#include <mc_rtc/gui.h>
#include <mc_rtc/logging.h>

#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <thread>

namespace fs = std::filesystem;

namespace mc_isaac
{

namespace
{

constexpr const char * ACTIVATION_ENV = "MC_ISAAC_ACTIVE";
constexpr int MIN_SERVER_VERSION[3] = {0, 12, 0};
/** Per robot state layout after q, alpha, tau: root pose (x y z qw qx qy qz) + root velocity (world lin, world ang) */
constexpr size_t ROOT_STATE_SIZE = 13;
/** Per IMU: world orientation (qw qx qy qz), angular velocity and proper linear acceleration in the sensor frame */
constexpr size_t IMU_STATE_SIZE = 10;
/** Per force sensor: force then couple, in the sensor frame */
constexpr size_t FORCE_STATE_SIZE = 6;

bool server_version_ok(const std::string & version)
{
  int v[3] = {0, 0, 0};
  std::istringstream ss(version);
  char dot;
  ss >> v[0] >> dot >> v[1] >> dot >> v[2];
  for(int i = 0; i < 3; ++i)
  {
    if(v[i] != MIN_SERVER_VERSION[i]) { return v[i] > MIN_SERVER_VERSION[i]; }
  }
  return true;
}

std::string read_file(const std::string & path)
{
  std::ifstream file(path, std::ios::binary);
  if(!file) { mc_rtc::log::error_and_throw("[mc_isaac] cannot read {}", path); }
  std::ostringstream ss;
  ss << file.rdbuf();
  return ss.str();
}

/** USD prim names only accept [A-Za-z0-9_] and must not start with a digit */
std::string usd_identifier(const std::string & name)
{
  std::string out = name;
  for(auto & c : out)
  {
    if(!std::isalnum(static_cast<unsigned char>(c))) { c = '_'; }
  }
  if(out.empty() || std::isdigit(static_cast<unsigned char>(out[0]))) { out = "r_" + out; }
  return out;
}

/** Run a program (no shell) with inherited stdout/stderr, returns its exit code */
int run_command(const std::vector<std::string> & cmd)
{
  std::vector<char *> argv;
  for(const auto & arg : cmd) { argv.push_back(const_cast<char *>(arg.c_str())); }
  argv.push_back(nullptr);
  pid_t pid = fork();
  if(pid < 0) { return -1; }
  if(pid == 0)
  {
    execvp(argv[0], argv.data());
    _exit(127);
  }
  int status = 0;
  while(waitpid(pid, &status, 0) < 0)
  {
    if(errno != EINTR) { return -1; }
  }
  return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
}

std::string join(const std::vector<std::string> & cmd)
{
  std::string out;
  for(const auto & arg : cmd) { out += (out.empty() ? "" : " ") + arg; }
  return out;
}

bool is_local_host(const std::string & host)
{
  return host == "127.0.0.1" || host == "localhost" || host == "::1";
}

} // namespace

IsaacSimPlugin::~IsaacSimPlugin()
{
  if(launchedServer_ && !keepAlive_)
  {
    mc_rtc::log::info("[mc_isaac] Stopping the Isaac server started by mc_isaac (keep_alive: false)");
    bridge_.close();
    run_command({launcher(), "--stop", "--host", host_, "--http-port", std::to_string(httpPort_), "--name", dockerName_});
  }
}

std::string IsaacSimPlugin::launcher() const
{
  const auto installed = fs::path(MC_ISAAC_BIN_DIR) / "mc_isaac_server";
  return fs::exists(installed) ? installed.string() : "mc_isaac_server";
}

std::string IsaacSimPlugin::serverState()
{
  try
  {
    auto response = http_request(host_, httpPort_, "GET", "/status");
    if(response.status == 200)
    {
      return mc_rtc::Configuration::fromData(response.body)("state", std::string("unknown"));
    }
  }
  catch(const std::exception &)
  {
  }
  return "unreachable";
}

void IsaacSimPlugin::launchServer()
{
  if(launchMode_ == "none" || serverState() != "unreachable") { return; }
  if(!is_local_host(host_))
  {
    mc_rtc::log::warning("[mc_isaac] server.launch.mode is {} but the server host {} is not local, not launching it",
                         launchMode_, host_);
    return;
  }
  const auto docker = launchConfig_("docker", mc_rtc::Configuration{});
  const auto apptainer = launchConfig_("apptainer", mc_rtc::Configuration{});
  const auto local = launchConfig_("local", mc_rtc::Configuration{});
  const std::string docker_image = docker("image", std::string{});
  const std::string sif = apptainer("image", std::string{});
  std::string mode = launchMode_;
  if(mode == "auto") { mode = !docker_image.empty() ? "docker" : (!sif.empty() ? "apptainer" : "local"); }

  std::vector<std::string> cmd = {launcher(), "--host", host_, "--http-port", std::to_string(httpPort_),
                                  "--bridge-port", std::to_string(bridgePort_), "--detach", "--timeout",
                                  std::to_string(static_cast<int>(connectTimeout_))};
  if(headless_) { cmd.push_back("--headless"); }
  const std::string stream = launchConfig_("stream", std::string("none"));
  if(stream != "none")
  {
    cmd.insert(cmd.end(), {"--stream", stream});
    if(launchConfig_.has("stream_fps"))
    {
      cmd.insert(cmd.end(), {"--stream-fps", std::to_string(static_cast<double>(launchConfig_("stream_fps")))});
    }
    if(launchConfig_.has("stream_size"))
    {
      cmd.insert(cmd.end(), {"--stream-size", static_cast<std::string>(launchConfig_("stream_size"))});
    }
  }
  auto add_common = [&](const mc_rtc::Configuration & c)
  {
    const std::string python = c("python", std::string{});
    if(!python.empty() && python != "auto") { cmd.insert(cmd.end(), {"--python", python}); }
    for(const auto & arg : c("container_args", std::vector<std::string>{})) { cmd.push_back("--container-arg=" + arg); }
  };
  if(mode == "docker")
  {
    if(docker_image.empty()) { mc_rtc::log::error_and_throw("[mc_isaac] server.launch.docker.image is not set"); }
    cmd.insert(cmd.end(), {"--docker", docker_image, "--name", dockerName_});
    if(docker("host_vulkan", false)) { cmd.push_back("--host-vulkan"); }
    add_common(docker);
  }
  else if(mode == "apptainer")
  {
    if(sif.empty()) { mc_rtc::log::error_and_throw("[mc_isaac] server.launch.apptainer.image is not set"); }
    cmd.insert(cmd.end(), {"--sif", sif});
    add_common(apptainer);
  }
  else if(mode == "local")
  {
    const std::string python = local("python", std::string("auto"));
    cmd.push_back("--local");
    if(!python.empty() && python != "auto") { cmd.push_back(python); }
  }
  else
  {
    mc_rtc::log::error_and_throw("[mc_isaac] Unknown server.launch.mode '{}' (none, docker, apptainer, local, auto)",
                                 launchMode_);
  }
  mc_rtc::log::info("[mc_isaac] Starting the Isaac server ({}): {}", mode, join(cmd));
  const int code = run_command(cmd);
  if(code != 0)
  {
    mc_rtc::log::error_and_throw("[mc_isaac] mc_isaac_server failed (exit code {}), see its output above", code);
  }
  launchedServer_ = true;
}

void IsaacSimPlugin::init(mc_control::MCGlobalController & gc, const mc_rtc::Configuration & config)
{
  const char * env = std::getenv(ACTIVATION_ENV);
  if(env == nullptr || std::string(env) != "1")
  {
    mc_rtc::log::info("[mc_isaac] IsaacSim plugin inactive (start mc_rtc with the mc_isaac command to use Isaac Sim)");
    return;
  }
  active_ = true;
  loadConfig(config);
  // real-time settings given to mc_rtc_ticker by the mc_isaac command
  if(const char * sync = std::getenv("MC_ISAAC_SYNC")) { tickerSync_ = std::string(sync) != "0"; }
  if(const char * ratio = std::getenv("MC_ISAAC_SYNC_RATIO")) { tickerTargetRatio_ = std::atof(ratio); }
  if(tickerTargetRatio_ <= 0.0) { tickerTargetRatio_ = 1.0; }
  if(autoPhysicsDt_)
  {
    // largest divisor of the controller timestep not above 5 ms (IsaacLab's default simulation dt)
    physicsDt_ = gc.timestep() / std::ceil(gc.timestep() / 0.005 - 1e-9);
  }
  const double ratio = gc.timestep() / physicsDt_;
  nSubsteps_ = static_cast<int>(std::lround(ratio));
  if(nSubsteps_ < 1 || std::fabs(ratio - nSubsteps_) > 1e-6)
  {
    mc_rtc::log::error_and_throw("[mc_isaac] mc_rtc Timestep ({}) must be a multiple of simulation.physics_dt ({})",
                                 gc.timestep(), physicsDt_);
  }
  launchServer();
  waitServer();
  // only forward server messages produced from now on
  logNext_ = std::numeric_limits<int>::max();
  fetchLogs();
  loadScene(gc);
  fetchLogs();
  connectBridge(gc);
  connectMarkers(gc);
  addGui(gc);
  mc_rtc::log::success("[mc_isaac] Isaac Sim ready: {} robot(s), {} x {} ms physics steps per {} ms control step",
                       robots_.size(), nSubsteps_, physicsDt_ * 1e3, gc.timestep() * 1e3);
  if(!streamUrl_.empty())
  {
    mc_rtc::log::success("[mc_isaac] Isaac browser stream (view only): open {} in a web browser", streamUrl_);
  }
}

void IsaacSimPlugin::applyCommandLine()
{
  auto env = [](const char * name) -> std::optional<std::string>
  {
    const char * value = std::getenv(name);
    if(value == nullptr) { return std::nullopt; }
    return std::string(value);
  };
  if(auto server = env("MC_ISAAC_SERVER"))
  {
    const auto colon = server->rfind(':');
    host_ = server->substr(0, colon);
    if(colon != std::string::npos) { httpPort_ = std::stoi(server->substr(colon + 1)); }
  }
  if(auto launch = env("MC_ISAAC_LAUNCH")) { launchMode_ = *launch; }
  if(env("MC_ISAAC_HEADLESS") == std::optional<std::string>("1")) { headless_ = true; }
  if(auto stream = env("MC_ISAAC_STREAM")) { launchConfig_.add("stream", *stream); }
  if(auto dt = env("MC_ISAAC_PHYSICS_DT"))
  {
    autoPhysicsDt_ = *dt == "auto";
    if(!autoPhysicsDt_) { physicsDt_ = std::stod(*dt); }
  }
  if(env("MC_ISAAC_TORQUE_CONTROL") == std::optional<std::string>("1")) { torqueControl_ = true; }
  if(env("MC_ISAAC_RELOAD_SCENE") == std::optional<std::string>("1")) { forceSceneReload_ = true; }
  if(env("MC_ISAAC_SHOW_COLLISIONS") == std::optional<std::string>("1")) { showCollisions_ = true; }
  if(env("MC_ISAAC_MARKERS") == std::optional<std::string>("0")) { markersEnabled_ = false; }
  if(env("MC_ISAAC_PRINT_TIMINGS") == std::optional<std::string>("1")) { printTimings_ = true; }
  if(auto gui = env("MC_ISAAC_GUI")) { guiMode_ = *gui; }
}

void IsaacSimPlugin::loadConfig(const mc_rtc::Configuration & config)
{
  if(config.has("server"))
  {
    auto server = config("server");
    server("host", host_);
    server("http_port", httpPort_);
    server("bridge_port", bridgePort_);
    server("connect_timeout", connectTimeout_);
    if(server.has("launch"))
    {
      launchConfig_ = server("launch");
      launchConfig_("mode", launchMode_);
      launchConfig_("keep_alive", keepAlive_);
      launchConfig_("headless", headless_);
      dockerName_ = launchConfig_("docker", mc_rtc::Configuration{})("name", std::string(dockerName_));
    }
  }
  if(config.has("simulation"))
  {
    auto simulation = config("simulation");
    if(simulation.has("physics_dt"))
    {
      const auto dt = simulation("physics_dt");
      autoPhysicsDt_ = dt.isString() && static_cast<std::string>(dt) == "auto";
      if(!autoPhysicsDt_) { physicsDt_ = static_cast<double>(dt); }
    }
    simulation("interpolate_commands", interpolate_);
    simulation("torque_control", torqueControl_);
    simulation("print_timings", printTimings_);
    simulation("print_timings_period", printPeriod_);
  }
  config("markers", markersEnabled_);
  config("gui", guiMode_);
  applyCommandLine();
  if(guiMode_ != "none" && guiMode_ != "minimal" && guiMode_ != "full")
  {
    mc_rtc::log::error_and_throw("[mc_isaac] gui must be none, minimal or full (not {})", guiMode_);
  }
  if(!autoPhysicsDt_ && physicsDt_ <= 0.0)
  {
    mc_rtc::log::error_and_throw("[mc_isaac] simulation.physics_dt must be positive or auto");
  }

  descriptionDirs_.clear();
  if(const char * home = std::getenv("HOME")) { descriptionDirs_.push_back(fs::path(home) / ".config/mc_rtc/mc_isaac"); }
  for(const auto & dir : config("description_paths", std::vector<std::string>{})) { descriptionDirs_.push_back(dir); }
  descriptionDirs_.push_back(MC_ISAAC_SHARE_DIR);
  // descriptions installed alongside mc_rtc (e.g. by robot module packages)
  if(fs::path(MC_RTC_ISAAC_SHARE_DIR) != fs::path(MC_ISAAC_SHARE_DIR)) { descriptionDirs_.push_back(MC_RTC_ISAAC_SHARE_DIR); }
  descriptionOverrides_ = config.has("robots") ? config("robots") : mc_rtc::Configuration{};
  excludedRobots_ = config("exclude_robots", std::vector<std::string>{});
  camera_ = config("camera", mc_rtc::Configuration{});
}

bool IsaacSimPlugin::findDescription(const std::vector<std::string> & keys,
                                     mc_rtc::Configuration & description,
                                     std::string & source) const
{
  for(const auto & key : keys)
  {
    if(descriptionOverrides_.has(key))
    {
      description = descriptionOverrides_(key);
      source = "plugin configuration (robots: " + key + ")";
      return true;
    }
    for(const auto & dir : descriptionDirs_)
    {
      const auto path = dir / (key + ".yaml");
      if(!fs::is_regular_file(path)) { continue; }
      description = mc_rtc::Configuration(path.string());
      // relative USD paths are relative to the description file
      if(description.has("usd"))
      {
        const fs::path usd = static_cast<std::string>(description("usd"));
        if(usd.is_relative()) { description.add("usd", (dir / usd).lexically_normal().string()); }
      }
      source = path.string();
      return true;
    }
  }
  return false;
}

void IsaacSimPlugin::waitServer()
{
  using clock = std::chrono::steady_clock;
  const auto start = clock::now();
  std::string last_state;
  while(true)
  {
    std::string state = "unreachable";
    std::string version, isaac_version;
    try
    {
      auto response = http_request(host_, httpPort_, "GET", "/status");
      if(response.status == 200)
      {
        auto status = mc_rtc::Configuration::fromData(response.body);
        state = status("state", std::string("unknown"));
        version = status("server_version", std::string("0.0.0"));
        isaac_version = status("isaac_version", std::string("?"));
        // the bridge of this server (it may not use the configured port, e.g. with --server HOST:PORT)
        if(status.has("bridge_port")) { bridgePort_ = status("bridge_port"); }
        if(status.has("stream_url")) { streamUrl_ = static_cast<std::string>(status("stream_url")); }
        const std::string wanted_stream = launchConfig_("stream", std::string("none"));
        const bool display_requested = std::getenv("MC_ISAAC_HEADLESS") || std::getenv("MC_ISAAC_STREAM");
        if(state == "ready" && !launchedServer_ && display_requested
           && (status("headless", false) != headless_ || status("stream", std::string("none")) != wanted_stream))
        {
          mc_rtc::log::warning("[mc_isaac] The running Isaac server uses headless: {}, stream: {} (requested {}, {}): "
                               "restart it (mc_isaac_server --stop) to change the display mode",
                               status("headless", false), status("stream", std::string("none")), headless_,
                               wanted_stream);
        }
      }
    }
    catch(const std::exception &)
    {
    }
    if(state == "ready")
    {
      if(!server_version_ok(version))
      {
        mc_rtc::log::error_and_throw("[mc_isaac] Isaac server {} is too old (>= {}.{}.{} required), restart it "
                                     "(mc_isaac_server --stop, then start it again)",
                                     version, MIN_SERVER_VERSION[0], MIN_SERVER_VERSION[1], MIN_SERVER_VERSION[2]);
      }
      mc_rtc::log::success("[mc_isaac] Connected to Isaac server {} (Isaac Sim {}) at {}:{}", version, isaac_version,
                           host_, httpPort_);
      serverVersion_ = version;
      isaacVersion_ = isaac_version;
      if(!streamUrl_.empty()) { mc_rtc::log::info("[mc_isaac] Isaac browser stream: {}", streamUrl_); }
      return;
    }
    if(state != last_state)
    {
      mc_rtc::log::info("[mc_isaac] Waiting for the Isaac server at {}:{} (state: {})", host_, httpPort_, state);
      last_state = state;
    }
    if(std::chrono::duration<double>(clock::now() - start).count() > connectTimeout_)
    {
      mc_rtc::log::error_and_throw("[mc_isaac] No ready Isaac server at {}:{} after {}s (state: {}). Start one with "
                                   "mc_isaac_server --docker <image> | --sif <file> | --local",
                                   host_, httpPort_, connectTimeout_, state);
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

std::string IsaacSimPlugin::uploadAsset(const std::string & path)
{
  const std::string data = read_file(path);
  const std::string sha = sha256_hex(data);
  if(http_request(host_, httpPort_, "HEAD", "/upload/" + sha).status == 200) { return sha; }
  const std::string filename = fs::path(path).filename().string();
  mc_rtc::log::info("[mc_isaac] Uploading {} ({:.1f} MB) to the Isaac server", filename, data.size() / 1e6);
  auto response = http_request(host_, httpPort_, "PUT", "/upload/" + sha + "?filename=" + filename, data,
                               "application/octet-stream", 300.0);
  if(response.status != 200)
  {
    mc_rtc::log::error_and_throw("[mc_isaac] Upload of {} failed ({}): {}", path, response.status, response.body);
  }
  return sha;
}

void IsaacSimPlugin::loadScene(mc_control::MCGlobalController & gc)
{
  const auto & main_robot = gc.controller().robot().name();
  // MainRobot may be given with the loader alias (e.g. Honda_RightHand_UR10e) instead of the module name
  std::string main_alias;
  const auto & gc_config = gc.configuration().config;
  if(gc_config.has("MainRobot") && !gc_config("MainRobot").isArray() && !gc_config("MainRobot").isObject())
  {
    main_alias = static_cast<std::string>(gc_config("MainRobot"));
  }

  mc_rtc::Configuration spec;
  spec.add("physics_dt", physicsDt_);
  if(!camera_.empty()) { spec.add("camera", camera_); }
  bool ground = false;
  auto robots = spec.array("robots");
  for(const auto & robot : gc.robots())
  {
    const auto & module = robot.module().name;
    if(std::find(excludedRobots_.begin(), excludedRobots_.end(), robot.name()) != excludedRobots_.end())
    {
      mc_rtc::log::info("[mc_isaac] {}: excluded (exclude_robots), not simulated", robot.name());
      continue;
    }
    std::vector<std::string> keys = {module, robot.name()};
    if(robot.name() == main_robot && !main_alias.empty()) { keys.push_back(main_alias); }
    mc_rtc::Configuration desc;
    std::string source;
    if(!findDescription(keys, desc, source))
    {
      std::string searched;
      for(const auto & dir : descriptionDirs_) { searched += "\n  - " + dir.string(); }
      if(robot.name() == main_robot)
      {
        mc_rtc::log::error_and_throw("[mc_isaac] No Isaac description for the main robot {} (module {}), searched {}.yaml "
                                     "in:{}\nInstall its <robot>_isaac_description package or add its folder to "
                                     "description_paths in the IsaacSim plugin configuration",
                                     robot.name(), module, module, searched);
      }
      mc_rtc::log::warning("[mc_isaac] No Isaac description for robot {} (module {}), it is not simulated",
                           robot.name(), module);
      continue;
    }
    mc_rtc::log::info("[mc_isaac] {}: description {}", robot.name(), source);
    if(desc.has("builtin"))
    {
      const std::string builtin = desc("builtin");
      if(builtin == "ground_plane") { ground = true; }
      else
      {
        mc_rtc::log::warning("[mc_isaac] Unknown builtin '{}' for robot {} ({})", builtin, robot.name(), source);
      }
      continue;
    }
    if(!desc.has("usd")) { mc_rtc::log::error_and_throw("[mc_isaac] No usd entry in {}", source); }
    const std::string usd_path = desc("usd");

    IsaacRobot isaac;
    isaac.mcName = robot.name();
    isaac.isaacName = usd_identifier(robot.name());

    auto entry = robots.object();
    entry.add("name", isaac.isaacName);
    auto usd = entry.add("usd");
    usd.add("sha256", uploadAsset(usd_path));
    usd.add("filename", fs::path(usd_path).filename().string());
    // files referenced by the USD (textures, sublayers), paths relative to the USD folder
    const auto extra_files = desc("extra_files", std::vector<std::string>{});
    if(!extra_files.empty())
    {
      auto extra = usd.array("extra");
      for(const auto & rel : extra_files)
      {
        auto file = extra.object();
        file.add("path", rel);
        file.add("sha256", uploadAsset((fs::path(usd_path).parent_path() / rel).string()));
      }
    }
    const bool floating = robot.mb().nrJoints() > 0 && robot.mb().joint(0).dof() == 6;
    entry.add("fixed", desc("fixed", !floating));
    // robots without actuated joints (objects, environments) are simulated as a single rigid body
    entry.add("rigid", desc("rigid", robot.module().ref_joint_order().empty()));
    if(desc.has("mass")) { entry.add("mass", static_cast<double>(desc("mass"))); }
    const auto & X_0_root = robot.posW();
    // sva rotations are transposed with respect to Eigen's convention
    const Eigen::Quaterniond ori(X_0_root.rotation().transpose());
    const auto & t = X_0_root.translation();
    entry.add("pos", std::vector<double>{t.x(), t.y(), t.z()});
    entry.add("quat", std::vector<double>{ori.w(), ori.x(), ori.y(), ori.z()});
    auto init_q = entry.add("init_q");
    for(const auto & joint : robot.module().ref_joint_order())
    {
      if(!robot.hasJoint(joint)) { continue; }
      const auto & qj = robot.mbc().q[robot.jointIndexByName(joint)];
      if(qj.size() == 1) { init_q.add(joint, qj[0]); }
    }
    if(desc.has("drives")) { entry.add("drives", desc("drives")); }
    if(desc.has("self_collisions")) { entry.add("self_collisions", static_cast<bool>(desc("self_collisions"))); }
    if(desc.has("solver_iterations")) { entry.add("solver_iterations", desc("solver_iterations")); }
    entry.add("torque_control", torqueControl_);
    // every mc_rtc body sensor but FloatingBase is simulated as an IMU (gyro + accelerometer) on its parent link
    auto imus = entry.array("imus");
    for(const auto & sensor : robot.bodySensors())
    {
      if(sensor.name() == "FloatingBase") { continue; }
      const auto & X_b_s = sensor.X_b_s();
      const Eigen::Quaterniond rot(X_b_s.rotation().transpose());
      auto imu = imus.object();
      imu.add("name", sensor.name());
      imu.add("link", sensor.parentBody());
      imu.add("pos", std::vector<double>{X_b_s.translation().x(), X_b_s.translation().y(), X_b_s.translation().z()});
      imu.add("quat", std::vector<double>{rot.w(), rot.x(), rot.y(), rot.z()});
    }
    // force sensors are read from the PhysX joint reaction of their parent body (must be a separate USD link)
    auto force_sensors = entry.array("force_sensors");
    for(const auto & fs : robot.forceSensors())
    {
      const auto & X_p_f = fs.X_p_f();
      const Eigen::Quaterniond rot(X_p_f.rotation().transpose());
      auto f = force_sensors.object();
      f.add("name", fs.name());
      f.add("link", fs.parentBody());
      f.add("pos", std::vector<double>{X_p_f.translation().x(), X_p_f.translation().y(), X_p_f.translation().z()});
      f.add("quat", std::vector<double>{rot.w(), rot.x(), rot.y(), rot.z()});
    }
    robots_.push_back(isaac);
  }
  spec.add("ground", ground);
  if(showCollisions_) { spec.add("show_collisions", true); }
  specJson_ = spec.dump();

  auto response = http_request(host_, httpPort_, "POST", forceSceneReload_ ? "/scene?force=1" : "/scene", specJson_,
                               "application/json", 600.0);
  if(response.status != 200)
  {
    mc_rtc::log::error_and_throw("[mc_isaac] Scene loading failed ({}): {}", response.status, response.body);
  }
  const auto scene = mc_rtc::Configuration::fromData(response.body);
  mc_rtc::log::info("[mc_isaac] Scene {}", scene("reused", false) ? "unchanged, reset only" : "loaded");
}

void IsaacSimPlugin::connectBridge(mc_control::MCGlobalController & gc)
{
  bridge_.connect(host_, bridgePort_, 60.0);
  std::vector<double> unused;
  mc_rtc::Configuration hello_request;
  hello_request.add("type", "hello");
  const auto hello = bridge_.request(hello_request, {}, unused);
  const auto described = hello("robots");
  if(described.size() != robots_.size())
  {
    mc_rtc::log::error_and_throw("[mc_isaac] Isaac scene has {} robots, expected {}", described.size(), robots_.size());
  }
  for(size_t r = 0; r < robots_.size(); ++r)
  {
    auto & isaac = robots_[r];
    const auto desc = described[r];
    if(static_cast<std::string>(desc("name")) != isaac.isaacName)
    {
      mc_rtc::log::error_and_throw("[mc_isaac] Unexpected robot order in the Isaac scene");
    }
    isaac.joints = desc("joints").operator std::vector<std::string>();
    isaac.fixedBase = desc("fixed_base");
    isaac.qCmd = desc("init_q").operator std::vector<double>();
    isaac.imus = desc("imus", std::vector<std::string>{});
    isaac.imuValues.assign(isaac.imus.size() * IMU_STATE_SIZE, 0.0);
    isaac.forceSensors = desc("force_sensors", std::vector<std::string>{});
    isaac.forceValues.assign(isaac.forceSensors.size() * FORCE_STATE_SIZE, 0.0);
    const size_t n = isaac.joints.size();
    isaac.alphaCmd.assign(n, 0.0);
    isaac.tauCmd.assign(n, 0.0);

    const auto & robot = gc.robots().robot(isaac.mcName);
    isaac.mbcIndex.assign(n, -1);
    for(size_t i = 0; i < n; ++i)
    {
      if(robot.hasJoint(isaac.joints[i]))
      {
        isaac.mbcIndex[i] = static_cast<int>(robot.jointIndexByName(isaac.joints[i]));
      }
      else
      {
        mc_rtc::log::warning("[mc_isaac] {}: Isaac joint {} unknown to mc_rtc, held at its initial position",
                             isaac.mcName, isaac.joints[i]);
      }
    }
    const auto & rjo = robot.module().ref_joint_order();
    isaac.refToIsaac.assign(rjo.size(), -1);
    for(size_t k = 0; k < rjo.size(); ++k)
    {
      auto it = std::find(isaac.joints.begin(), isaac.joints.end(), rjo[k]);
      if(it != isaac.joints.end()) { isaac.refToIsaac[k] = static_cast<int>(it - isaac.joints.begin()); }
      else
      {
        mc_rtc::log::warning("[mc_isaac] {}: mc_rtc joint {} not simulated by Isaac", isaac.mcName, rjo[k]);
      }
    }
    mc_rtc::log::info("[mc_isaac] {}: {} Isaac joints, fixed base: {}", isaac.mcName, n, isaac.fixedBase);
  }
  requestState();
}

void IsaacSimPlugin::requestState()
{
  mc_rtc::Configuration header;
  header.add("type", "state");
  std::vector<double> data;
  const auto reply = bridge_.request(header, {}, data);
  simTime_ = reply("sim_time", 0.0);
  parseState(data);
}

void IsaacSimPlugin::parseState(const std::vector<double> & data)
{
  size_t offset = 0;
  for(auto & isaac : robots_)
  {
    const size_t n = isaac.joints.size();
    const size_t robot_size = 3 * n + ROOT_STATE_SIZE + IMU_STATE_SIZE * isaac.imus.size()
                              + FORCE_STATE_SIZE * isaac.forceSensors.size();
    if(data.size() < offset + robot_size)
    {
      mc_rtc::log::error_and_throw("[mc_isaac] Isaac state too short ({} values)", data.size());
    }
    const double * d = data.data() + offset;
    isaac.q.assign(d, d + n);
    isaac.alpha.assign(d + n, d + 2 * n);
    isaac.tau.assign(d + 2 * n, d + 3 * n);
    const double * root = d + 3 * n;
    isaac.pos = Eigen::Vector3d(root[0], root[1], root[2]);
    isaac.ori = Eigen::Quaterniond(root[3], root[4], root[5], root[6]);
    isaac.linVel = Eigen::Vector3d(root[7], root[8], root[9]);
    isaac.angVel = Eigen::Vector3d(root[10], root[11], root[12]);
    isaac.imuValues.assign(root + ROOT_STATE_SIZE, root + ROOT_STATE_SIZE + IMU_STATE_SIZE * isaac.imus.size());
    const double * forces = root + ROOT_STATE_SIZE + IMU_STATE_SIZE * isaac.imus.size();
    isaac.forceValues.assign(forces, forces + FORCE_STATE_SIZE * isaac.forceSensors.size());
    offset += robot_size;
  }
}

void IsaacSimPlugin::reset(mc_control::MCGlobalController & gc)
{
  if(!active_) { return; }
  mc_rtc::Configuration header;
  header.add("type", "reset");
  std::vector<double> unused;
  bridge_.request(header, {}, unused);
  for(auto & isaac : robots_)
  {
    std::fill(isaac.alphaCmd.begin(), isaac.alphaCmd.end(), 0.0);
    std::fill(isaac.tauCmd.begin(), isaac.tauCmd.end(), 0.0);
  }
  requestState();
  // the controller may have been switched: GUI elements belong to the controller
  addGui(gc);
  connectMarkers(gc);
  // mc_rtc_ticker leaves step-by-step mode to run its reset step: pause again after that step (see after())
  pauseAfterReset_ = pausedBeforeReset_;
  mc_rtc::log::info("[mc_isaac] Isaac scene reset{}", pauseAfterReset_ ? " (staying in step-by-step mode)" : "");
}

void IsaacSimPlugin::handleReply(mc_control::MCGlobalController & gc,
                                 const mc_rtc::Configuration & reply,
                                 bool ticker_running)
{
  timelinePlaying_ = reply("timeline_playing", true);
  // Isaac buttons pressed by the user (the server mirrors the ticker state on the timeline itself)
  const std::string event = reply("isaac_event", std::string{});
  if(event == "pause" && ticker_running)
  {
    mc_rtc::log::info("[mc_isaac] Paused from the Isaac window");
    tickerRequest(gc, "Step by step");
  }
  else if(event == "play" && !ticker_running)
  {
    mc_rtc::log::info("[mc_isaac] Resumed from the Isaac window");
    tickerRequest(gc, "Step by step");
  }
  if(reply("reloaded", false))
  {
    mc_rtc::log::warning("[mc_isaac] Isaac physics was restarted (Stop button), resetting the controller");
    tickerRequest(gc, "Reset");
  }
  reply("physics_ms", physicsMs_);
  reply("render_ms", renderMs_);
  if(reply("log_next", 0) > logNext_) { fetchLogs(); }
  if(reply.has("commands"))
  {
    const auto commands = reply("commands");
    for(size_t i = 0; i < commands.size(); ++i) { handlePanelCommand(gc, commands[i]); }
  }
  if(reply.has("gui_requests") && markers_)
  {
    // mc_rtc elements moved with the Isaac transform gizmo
    const auto requests = reply("gui_requests");
    for(size_t i = 0; i < requests.size(); ++i)
    {
      markers_->request(requests[i]("id"), requests[i]("pose").operator std::vector<double>());
    }
  }
  if(guiMirror_)
  {
    // the server rebuilt its mirror (first connection, gui mode change): send everything again
    if(reply("gui_resync", false)) { guiMirror_->resync(); }
    if(reply.has("gui_widget_requests"))
    {
      // user actions in the mirrored GUI, applied like the requests of mc-rtc-magnum
      const auto requests = reply("gui_widget_requests");
      for(size_t i = 0; i < requests.size(); ++i)
      {
        const auto request = requests[i];
        const std::string key = request("key");
        if(request.has("data"))
        {
          const auto data = request("data");
          guiMirror_->widgetRequest(key, &data);
        }
        else { guiMirror_->widgetRequest(key, nullptr); }
      }
    }
  }
}

void IsaacSimPlugin::connectMarkers(mc_control::MCGlobalController & gc)
{
  auto gui = gc.controller().gui();
  if(!gui) { return; }
  guiMirror_ = nullptr;
  if(guiMode_ == "full")
  {
    auto mirror = std::make_unique<GuiMirror>();
    guiMirror_ = mirror.get();
    markers_ = std::move(mirror);
  }
  else { markers_ = std::make_unique<Markers>(); }
  markers_->connect(gc.server(), *gui);
}

void IsaacSimPlugin::addMarkers(mc_rtc::Configuration & header)
{
  header.add("gui_mode", guiMode_);
  const auto now = std::chrono::steady_clock::now();
  if(!markers_ || now - lastMarkers_ < std::chrono::milliseconds(50)) { return; }
  lastMarkers_ = now;
  if(markersEnabled_ || guiMirror_) { markers_->update(); }
  if(markersEnabled_) { header.add("markers", markers_->toConfig()); }
  else
  {
    // empty marker set: the server clears the viewport
    auto empty = header.add("markers");
    empty.add("lines", std::vector<double>{});
  }
  if(guiMirror_ && now - lastGui_ >= std::chrono::milliseconds(100))
  {
    lastGui_ = now;
    if(auto gui = guiMirror_->collect()) { header.add("gui", *gui); }
  }
}

void IsaacSimPlugin::updateRatio(bool running)
{
  const auto now = std::chrono::steady_clock::now();
  if(!running)
  {
    // a pause would count as slow simulation: restart the window
    ratioWall_ = {};
    return;
  }
  if(ratioWall_ == std::chrono::steady_clock::time_point{})
  {
    ratioWall_ = now;
    ratioSim_ = simTime_;
    return;
  }
  const double wall = std::chrono::duration<double>(now - ratioWall_).count();
  if(wall < 1.0) { return; }
  simRealRatio_ = (simTime_ - ratioSim_) / wall;
  ratioWall_ = now;
  ratioSim_ = simTime_;
}

void IsaacSimPlugin::addTickerState(mc_control::MCGlobalController & gc, mc_rtc::Configuration & header)
{
  auto ticker = header.add("ticker");
  ticker.add("step_by_step", !gc.running);
  ticker.add("sync", tickerSync_);
  ticker.add("target_ratio", tickerTargetRatio_);
  ticker.add("dt", gc.timestep());
  ticker.add("markers", markersEnabled_);
}

void IsaacSimPlugin::handlePanelCommand(mc_control::MCGlobalController & gc, const mc_rtc::Configuration & command)
{
  const std::string cmd = command("cmd", std::string{});
  if(cmd == "toggle_pause") { tickerRequest(gc, "Step by step"); }
  else if(cmd == "steps") { tickerSteps(gc, static_cast<size_t>(std::max(1, command("n", 1)))); }
  else if(cmd == "step_ms")
  {
    const double ms = command("ms", 0.0);
    tickerSteps(gc, static_cast<size_t>(std::max(1L, std::lround(ms * 1e-3 / gc.timestep()))));
  }
  else if(cmd == "toggle_sync")
  {
    tickerRequest(gc, "Synchronize");
    tickerSync_ = !tickerSync_;
  }
  else if(cmd == "ratio")
  {
    double ratio = command("value", 1.0);
    if(ratio <= 0.0) { ratio = 1.0 / 1024.0; }
    mc_rtc::Configuration data;
    data.add("value", ratio);
    tickerRequest(gc, "Target ratio", data("value"));
    tickerTargetRatio_ = ratio;
  }
  else if(cmd == "ratio_x2")
  {
    tickerRequest(gc, "x2");
    tickerTargetRatio_ *= 2.0;
  }
  else if(cmd == "ratio_div2")
  {
    tickerRequest(gc, "/2");
    tickerTargetRatio_ /= 2.0;
  }
  else if(cmd == "reset") { tickerRequest(gc, "Reset"); }
  else if(cmd == "stop")
  {
    mc_rtc::log::info("[mc_isaac] Stop requested from the Isaac window");
    tickerRequest(gc, "Stop");
  }
  else if(cmd == "reload") { reloadRequested_ = true; }
  else if(cmd == "toggle_markers") { markersEnabled_ = !markersEnabled_; }
  else { mc_rtc::log::warning("[mc_isaac] Unknown command from the Isaac panel: {}", cmd); }
}

void IsaacSimPlugin::tickerSteps(mc_control::MCGlobalController & gc, size_t n)
{
  if(gc.running)
  {
    mc_rtc::log::warning("[mc_isaac] Steps are only available in step-by-step mode (pause first)");
    return;
  }
  // mc_rtc_ticker only has +1, +5, +10, +50 and +100 steps buttons, labelled in ms
  for(size_t steps : std::initializer_list<size_t>{100, 50, 10, 5, 1})
  {
    const auto ms = static_cast<size_t>(std::ceil(static_cast<double>(steps * 1000) * gc.timestep()));
    for(; n >= steps; n -= steps) { tickerRequest(gc, fmt::format("+{}ms", ms)); }
  }
}

void IsaacSimPlugin::tickerRequest(mc_control::MCGlobalController & gc,
                                   const std::string & name,
                                   const mc_rtc::Configuration & data)
{
  auto gui = gc.controller().gui();
  if(!gui || !gui->hasElement({"Ticker"}, name))
  {
    mc_rtc::log::warning("[mc_isaac] No '{}' element in the Ticker GUI tab (not running mc_rtc_ticker?)", name);
    return;
  }
  gui->handleRequest({"Ticker"}, name, data);
}

void IsaacSimPlugin::fetchLogs()
{
  try
  {
    auto response = http_request(host_, httpPort_, "GET", "/logs?since=" + std::to_string(logNext_));
    if(response.status != 200) { return; }
    const auto data = mc_rtc::Configuration::fromData(response.body);
    const auto logs = data("logs");
    for(size_t i = 0; i < logs.size(); ++i)
    {
      mc_rtc::log::info("[Isaac] {}", static_cast<std::string>(logs[i]("msg")));
    }
    logNext_ = data("next", 0);
  }
  catch(const std::exception & e)
  {
    mc_rtc::log::warning("[mc_isaac] Cannot fetch the Isaac server logs: {}", e.what());
  }
}

void IsaacSimPlugin::addGui(mc_control::MCGlobalController & gc)
{
  auto gui = gc.controller().gui();
  if(!gui) { return; }
  gui->removeElements(this);
  using namespace mc_rtc::gui;
  // same controls as the mc_isaac panel of the Isaac window; run by the next after(), outside of the GUI handling
  auto command = [this](const std::string & cmd)
  {
    return [this, cmd]()
    {
      mc_rtc::Configuration c;
      c.add("cmd", cmd);
      guiCommands_.push_back(c);
    };
  };
  auto steps = [this](int n)
  {
    return [this, n]()
    {
      mc_rtc::Configuration c;
      c.add("cmd", "steps");
      c.add("n", n);
      guiCommands_.push_back(c);
    };
  };
  auto step_label = [&gc](int n) { return fmt::format("+{}ms", static_cast<int>(std::ceil(n * 1000 * gc.timestep() - 1e-9))); };
  gui->addElement(
      this, {"IsaacSim"},
      Label("Server", [this]() { return fmt::format("{}:{} (mc_isaac_server {})", host_, httpPort_, serverVersion_); }),
      Label("Isaac Sim", [this]() { return isaacVersion_; }),
      Label("Stream", [this]() { return streamUrl_.empty() ? std::string("none") : streamUrl_; }),
      Label("Scene",
            [this]()
            {
              std::string names;
              for(const auto & r : robots_) { names += (names.empty() ? "" : ", ") + r.mcName; }
              return fmt::format("{} robot(s): {}", robots_.size(), names);
            }),
      Label("Physics",
            [this]() { return fmt::format("{} x {:.3f} ms per control step", nSubsteps_, physicsDt_ * 1e3); }),
      Label("Sim time [s]", [this]() { return fmt::format("{:.3f}", simTime_); }),
      Label("Physics time per control step [ms]", [this]() { return fmt::format("{:.2f}", physicsMs_); }),
      Label("Isaac render time per frame [ms]", [this]() { return fmt::format("{:.1f}", renderMs_); }),
      Label("Sim/real ratio",
            [this]()
            {
              return fmt::format("{:.2f} (target {:.3g}, real time {})", simRealRatio_, tickerTargetRatio_,
                                 tickerSync_ ? "ON" : "OFF");
            }),
      Label("Isaac timeline", [this]() { return std::string(timelinePlaying_ ? "playing" : "paused"); }),
      Checkbox(
          "Paused (step by step)", [&gc]() { return !gc.running; }, command("toggle_pause")));
  gui->addElement(this, {"IsaacSim"}, ElementsStacking::Horizontal, Button(step_label(1), steps(1)),
                  Button(step_label(5), steps(5)), Button(step_label(10), steps(10)), Button(step_label(50), steps(50)),
                  Button(step_label(100), steps(100)));
  gui->addElement(this, {"IsaacSim"}, ElementsStacking::Horizontal,
                  NumberInput(
                      "Step duration [ms]", [this]() { return stepMs_; }, [this](double ms) { stepMs_ = ms; }),
                  Button("Step",
                         [this]()
                         {
                           mc_rtc::Configuration c;
                           c.add("cmd", "step_ms");
                           c.add("ms", stepMs_);
                           guiCommands_.push_back(c);
                         }));
  gui->addElement(
      this, {"IsaacSim"}, Checkbox(
                              "Real time", [this]() { return tickerSync_; }, command("toggle_sync")),
      NumberInput(
          "Target ratio", [this]() { return tickerTargetRatio_; },
          [this](double ratio)
          {
            mc_rtc::Configuration c;
            c.add("cmd", "ratio");
            c.add("value", ratio);
            guiCommands_.push_back(c);
          }));
  gui->addElement(this, {"IsaacSim"}, ElementsStacking::Horizontal, Button("Ratio x2", command("ratio_x2")),
                  Button("Ratio /2", command("ratio_div2")));
  gui->addElement(this, {"IsaacSim"}, ElementsStacking::Horizontal, Button("Reset controller", command("reset")),
                  Button("Stop mc_rtc", command("stop")));
  gui->addElement(
      this, {"IsaacSim"},
      Checkbox(
          "Show mc_rtc markers in Isaac", [this]() { return markersEnabled_; },
          [this]() { markersEnabled_ = !markersEnabled_; }),
      Checkbox(
          "Print timings", [this]() { return printTimings_; },
          [this]()
          {
            printTimings_ = !printTimings_;
            timings_ = Timings{};
          }),
      Button("Reload Isaac scene", [this]() { reloadRequested_ = true; }));
}

void IsaacSimPlugin::reloadScene(mc_control::MCGlobalController & gc)
{
  mc_rtc::log::info("[mc_isaac] Reloading the Isaac scene");
  auto response = http_request(host_, httpPort_, "POST", "/scene?force=1", specJson_, "application/json", 600.0);
  if(response.status != 200)
  {
    mc_rtc::log::error_and_throw("[mc_isaac] Scene reload failed ({}): {}", response.status, response.body);
  }
  fetchLogs();
  requestState();
  timelinePlaying_ = true;
  tickerRequest(gc, "Reset");
}

void IsaacSimPlugin::before(mc_control::MCGlobalController & gc)
{
  if(!active_ || !gc.running) { return; }
  beforeStart_ = std::chrono::steady_clock::now();
  for(const auto & isaac : robots_)
  {
    const auto & robot = gc.robots().robot(isaac.mcName);
    const size_t n_ref = robot.module().ref_joint_order().size();
    if(n_ref > 0)
    {
      std::vector<double> q = robot.encoderValues();
      std::vector<double> alpha = robot.encoderVelocities();
      std::vector<double> tau = robot.jointTorques();
      q.resize(n_ref, 0.0);
      alpha.resize(n_ref, 0.0);
      tau.resize(n_ref, 0.0);
      for(size_t k = 0; k < n_ref; ++k)
      {
        const int i = isaac.refToIsaac[k];
        if(i < 0) { continue; }
        q[k] = isaac.q[i];
        alpha[k] = isaac.alpha[i];
        tau[k] = isaac.tau[i];
      }
      gc.setEncoderValues(isaac.mcName, q);
      gc.setEncoderVelocities(isaac.mcName, alpha);
      gc.setJointTorques(isaac.mcName, tau);
    }
    // Isaac gives the root body state: only usable when the FloatingBase sensor is attached to the root body
    if(!isaac.fixedBase && robot.hasBodySensor("FloatingBase")
       && robot.bodySensor("FloatingBase").parentBody() == robot.mb().body(0).name())
    {
      gc.setSensorPositions(isaac.mcName, {{"FloatingBase", isaac.pos}});
      gc.setSensorOrientations(isaac.mcName, {{"FloatingBase", isaac.ori}});
      gc.setSensorLinearVelocities(isaac.mcName, {{"FloatingBase", isaac.linVel}});
      gc.setSensorAngularVelocities(isaac.mcName, {{"FloatingBase", isaac.angVel}});
    }
    for(size_t i = 0; i < isaac.imus.size(); ++i)
    {
      const double * v = isaac.imuValues.data() + i * IMU_STATE_SIZE;
      const auto & name = isaac.imus[i];
      gc.setSensorOrientations(isaac.mcName, {{name, Eigen::Quaterniond(v[0], v[1], v[2], v[3])}});
      gc.setSensorAngularVelocities(isaac.mcName, {{name, Eigen::Vector3d(v[4], v[5], v[6])}});
      gc.setSensorLinearAccelerations(isaac.mcName, {{name, Eigen::Vector3d(v[7], v[8], v[9])}});
    }
    if(!isaac.forceSensors.empty())
    {
      std::map<std::string, sva::ForceVecd> wrenches;
      for(size_t i = 0; i < isaac.forceSensors.size(); ++i)
      {
        const double * w = isaac.forceValues.data() + i * FORCE_STATE_SIZE;
        wrenches[isaac.forceSensors[i]] =
            sva::ForceVecd(Eigen::Vector3d(w[3], w[4], w[5]), Eigen::Vector3d(w[0], w[1], w[2]));
      }
      gc.setWrenches(isaac.mcName, wrenches);
    }
  }
  beforeEnd_ = std::chrono::steady_clock::now();
}

void IsaacSimPlugin::after(mc_control::MCGlobalController & gc)
{
  if(!active_) { return; }
  if(!guiCommands_.empty())
  {
    const auto commands = std::move(guiCommands_);
    guiCommands_.clear();
    for(const auto & c : commands) { handlePanelCommand(gc, c); }
  }
  updateRatio(gc.running);
  // in step-by-step mode, mc_rtc_ticker calls the plugins with gc.running == false between steps
  pausedBeforeReset_ = !gc.running;
  if(pauseAfterReset_ && gc.running)
  {
    pauseAfterReset_ = false;
    tickerRequest(gc, "Step by step");
  }
  if(reloadRequested_)
  {
    reloadRequested_ = false;
    reloadScene(gc);
    return;
  }
  if(!gc.running)
  {
    // ticker paused: no step, but keep following the Isaac window (Play button, Stop) at ~20 Hz
    const auto now = std::chrono::steady_clock::now();
    if(now - lastPing_ < std::chrono::milliseconds(50)) { return; }
    lastPing_ = now;
    mc_rtc::Configuration ping;
    ping.add("type", "ping");
    ping.add("running", false);
    addTickerState(gc, ping);
    addMarkers(ping);
    std::vector<double> unused;
    try
    {
      handleReply(gc, bridge_.request(ping, {}, unused), false);
    }
    catch(const std::exception & e)
    {
      mc_rtc::log::error_and_throw("[mc_isaac] Lost the Isaac server: {}", e.what());
    }
    return;
  }
  std::vector<double> payload;
  const auto afterStart = std::chrono::steady_clock::now();
  for(auto & isaac : robots_)
  {
    const auto & robot = gc.controller().outputRobots().robot(isaac.mcName);
    const auto & mbc = robot.mbc();
    for(size_t i = 0; i < isaac.joints.size(); ++i)
    {
      const int m = isaac.mbcIndex[i];
      if(m < 0 || mbc.q[m].size() != 1) { continue; }
      isaac.qCmd[i] = mbc.q[m][0];
      isaac.alphaCmd[i] = mbc.alpha[m][0];
      isaac.tauCmd[i] = torqueControl_ ? mbc.jointTorque[m][0] : 0.0;
    }
    payload.insert(payload.end(), isaac.qCmd.begin(), isaac.qCmd.end());
    payload.insert(payload.end(), isaac.alphaCmd.begin(), isaac.alphaCmd.end());
    payload.insert(payload.end(), isaac.tauCmd.begin(), isaac.tauCmd.end());
  }
  mc_rtc::Configuration header;
  header.add("type", "step");
  header.add("running", true);
  header.add("n_substeps", nSubsteps_);
  header.add("interpolate", interpolate_);
  addTickerState(gc, header);
  addMarkers(header);
  std::vector<double> state;
  mc_rtc::Configuration reply;
  double bridge_ms = 0.0;
  try
  {
    const auto t0 = std::chrono::steady_clock::now();
    reply = bridge_.request(header, payload, state);
    bridge_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    // the (key, non-const ref) overload of Configuration assigns in place when the key exists
    reply("sim_time", simTime_);
  }
  catch(const std::exception & e)
  {
    mc_rtc::log::error_and_throw("[mc_isaac] Lost the Isaac server: {}", e.what());
  }
  parseState(state);
  handleReply(gc, reply, true);
  if(printTimings_)
  {
    const double after_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - afterStart).count();
    accumulateTimings(reply, after_ms, bridge_ms, gc.timestep());
  }
}

void IsaacSimPlugin::accumulateTimings(const mc_rtc::Configuration & reply,
                                       double after_ms,
                                       double bridge_ms,
                                       double sim_dt)
{
  using ms = std::chrono::duration<double, std::milli>;
  const auto now = std::chrono::steady_clock::now();
  // a pause (or the first step) breaks the step sequence: restart the window
  const bool consecutive = timings_.steps > 0 && ms(beforeStart_ - lastBeforeStart_).count() < 500.0;
  lastBeforeStart_ = beforeStart_;
  if(!consecutive && timings_.steps > 0) { timings_ = Timings{}; }
  if(timings_.steps == 0) { timingsStart_ = beforeStart_; }
  auto & t = timings_;
  t.steps += 1;
  t.sim += sim_dt;
  t.before += ms(beforeEnd_ - beforeStart_).count();
  t.mcrtc += ms(now - beforeEnd_).count() - after_ms;
  t.after += after_ms;
  t.bridge += bridge_ms;
  const auto server = reply("timings", mc_rtc::Configuration{});
  t.server += server("server", 0.0);
  t.wait += server("wait", 0.0);
  t.apply += server("apply", 0.0);
  t.simulate += server("simulate", 0.0);
  t.state += server("state", 0.0);
  t.render += server("render", 0.0);
  t.renders += static_cast<size_t>(server("renders", 0));
  t.wall = ms(now - timingsStart_).count();
  if(t.wall >= 1000.0 * printPeriod_)
  {
    mc_rtc::log::info("{}", timingsSummary());
    timings_ = Timings{};
  }
}

std::string IsaacSimPlugin::timingsSummary() const
{
  const auto & t = timings_;
  if(t.steps < 2) { return "[mc_isaac] timings: not enough steps yet"; }
  const double n = static_cast<double>(t.steps);
  const double wall = t.wall / n;
  const double rest = wall - (t.before + t.mcrtc + t.after) / n;
  const double renders_per_s = 1000.0 * static_cast<double>(t.renders) / t.wall;
  return fmt::format(
      "[mc_isaac] timings over {:.1f} s, {} steps, sim/real {:.2f} (target {:.3g}, real time {}), {} x {:.1f} ms physics\n"
      "  per control step [ms]: wall {:.2f} = mc_rtc {:.2f} + plugin before {:.2f} + plugin after {:.2f} + rest {:.2f} "
      "(ticker sync sleep, logging)\n"
      "  plugin after: bridge {:.2f} = server {:.2f} (queue wait {:.2f}, apply cmds {:.2f}, physx {:.2f}, read state "
      "{:.2f}) + transport {:.2f}\n"
      "  Isaac rendering: {:.1f} renders/s x {:.1f} ms = {:.0f} ms/s on the server main thread (delays the steps)",
      t.wall / 1000.0, t.steps, t.sim * 1000.0 / t.wall, tickerTargetRatio_, tickerSync_ ? "ON" : "OFF", nSubsteps_,
      physicsDt_ * 1e3, wall, t.mcrtc / n, t.before / n, t.after / n, rest, t.bridge / n, t.server / n, t.wait / n,
      t.apply / n, t.simulate / n, t.state / n, (t.bridge - t.server) / n, renders_per_s,
      t.renders ? t.render / static_cast<double>(t.renders) : 0.0, renders_per_s * (t.renders ? t.render / static_cast<double>(t.renders) : 0.0));
}

mc_control::GlobalPlugin::GlobalPluginConfiguration IsaacSimPlugin::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = true;
  // called while the ticker is paused too: after() pings the server to follow the Isaac window and the panel
  out.should_always_run = true;
  return out;
}

} // namespace mc_isaac

EXPORT_MC_RTC_PLUGIN("IsaacSim", mc_isaac::IsaacSimPlugin)
