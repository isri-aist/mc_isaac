#pragma once

#include "Bridge.h"
#include "GuiMirror.h"
#include "Markers.h"

#include <mc_control/GlobalPlugin.h>

#include <Eigen/Geometry>

#include <chrono>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace mc_isaac
{

/** One mc_rtc robot simulated by the Isaac server. Vectors without suffix are in Isaac joint order. */
struct IsaacRobot
{
  std::string mcName;
  std::string isaacName;
  std::vector<std::string> joints;
  bool fixedBase = true;
  /** Isaac joint -> mc_rtc mbc joint index (-1: unknown to mc_rtc, held at its initial position). */
  std::vector<int> mbcIndex;
  /** mc_rtc refJointOrder index -> Isaac joint index (-1: not simulated, ticker value kept). */
  std::vector<int> refToIsaac;
  /** Last commands sent, initialized with the initial posture. */
  std::vector<double> qCmd, alphaCmd, tauCmd;
  /** Last state received from Isaac. */
  std::vector<double> q, alpha, tau;
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Quaterniond ori = Eigen::Quaterniond::Identity();
  Eigen::Vector3d linVel = Eigen::Vector3d::Zero();
  Eigen::Vector3d angVel = Eigen::Vector3d::Zero();
  /** mc_rtc body sensors simulated as IMUs, 10 values each (see IMU_STATE_SIZE) */
  std::vector<std::string> imus;
  std::vector<double> imuValues;
  /** mc_rtc force sensors, 6 values each (force, couple in the sensor frame) */
  std::vector<std::string> forceSensors;
  std::vector<double> forceValues;
};

/** Runs mc_rtc against the mc_isaac server in lockstep.
 *
 * Only active when started through the mc_isaac command (MC_ISAAC_ACTIVE=1), so it can stay in the Plugins list.
 * before(): Isaac state -> mc_rtc sensors (overrides the open-loop sensors of mc_rtc_ticker).
 * after(): mc_rtc output robots -> Isaac targets, then one lockstep step of Timestep / physics_dt substeps.
 * Nothing is stepped while gc.running is false (ticker paused).
 */
struct IsaacSimPlugin : public mc_control::GlobalPlugin
{
  ~IsaacSimPlugin() override;
  void init(mc_control::MCGlobalController & gc, const mc_rtc::Configuration & config) override;
  void reset(mc_control::MCGlobalController & gc) override;
  void before(mc_control::MCGlobalController & gc) override;
  void after(mc_control::MCGlobalController & gc) override;
  GlobalPluginConfiguration configuration() override;

private:
  /** Plugin configuration (IsaacSim.yaml) then command line overrides; description search folders */
  void loadConfig(const mc_rtc::Configuration & config);
  /** Overrides given by the mc_isaac command line (MC_ISAAC_* environment variables) */
  void applyCommandLine();
  /** Path of the mc_isaac_server launcher (installed next to the plugin, else from PATH) */
  std::string launcher() const;
  /** Server state from GET /status ("ready", "warming_up"...), "unreachable" if nothing answers */
  std::string serverState();
  /** Start the server with mc_isaac_server when server.launch.mode != none and nothing answers */
  void launchServer();
  /** First description found for the keys (robot module name, robot name, MainRobot alias), see descriptionDirs_ */
  bool findDescription(const std::vector<std::string> & keys,
                       mc_rtc::Configuration & description,
                       std::string & source) const;
  /** Wait until the server is ready (connect_timeout), check its version, read its bridge port and stream URL */
  void waitServer();
  /** Upload a file to the server cache unless already there (HEAD then PUT /upload/<sha256>), returns its sha256 */
  std::string uploadAsset(const std::string & path);
  /** Scene spec from gc.robots() and their descriptions (USD upload, drives, initial state, sensors), POST /scene */
  void loadScene(mc_control::MCGlobalController & gc);
  /** Connect the lockstep bridge, "hello": Isaac joint order and sensors -> mc_rtc joint/sensor mapping */
  void connectBridge(mc_control::MCGlobalController & gc);
  /** Read the current Isaac state without stepping (after connection, reset, reload) */
  void requestState();
  /** Split the state payload of all robots into robots_ (layout documented in the server) */
  void parseState(const std::vector<double> & data);
  /** Isaac timeline transitions -> Ticker pause/resume, Isaac Stop -> Ticker reset, new server logs -> console */
  void handleReply(mc_control::MCGlobalController & gc, const mc_rtc::Configuration & reply, bool ticker_running);
  /** Press a GUI element of mc_rtc_ticker's "Ticker" tab (single source of truth for pause/step/reset) */
  void tickerRequest(mc_control::MCGlobalController & gc,
                     const std::string & name,
                     const mc_rtc::Configuration & data = mc_rtc::Configuration{});
  /** Ticker state shown by the Isaac panel, sent with every bridge request */
  void addTickerState(mc_control::MCGlobalController & gc, mc_rtc::Configuration & header);
  /** Commands from the Isaac panel (pause, steps, real-time sync, reset, reload...) */
  void handlePanelCommand(mc_control::MCGlobalController & gc, const mc_rtc::Configuration & command);
  /** Run n controller steps with the ticker "+N ms" buttons (requires step-by-step mode) */
  void tickerSteps(mc_control::MCGlobalController & gc, size_t n);
  /** Print the server log lines produced since logNext_ as [Isaac] messages */
  void fetchLogs();
  /** IsaacSim tab of the mc_rtc GUI (status + the same controls as the Isaac panel), re-added on controller switch */
  void addGui(mc_control::MCGlobalController & gc);
  /** Rebuild the Isaac scene from the last spec (POST /scene?force=1), then reset the controller */
  void reloadScene(mc_control::MCGlobalController & gc);

private:
  bool active_ = false;
  std::string host_ = "127.0.0.1";
  int httpPort_ = 5050;
  int bridgePort_ = 5055;
  double connectTimeout_ = 120.0;
  mc_rtc::Configuration launchConfig_;
  std::string launchMode_ = "none";
  bool keepAlive_ = true;
  bool headless_ = false;
  std::string dockerName_ = "mc_isaac_server";
  bool launchedServer_ = false;
  double physicsDt_ = 0.001;
  bool autoPhysicsDt_ = true;
  bool interpolate_ = true;
  bool torqueControl_ = false;
  int nSubsteps_ = 1;
  double simTime_ = 0.0;
  double physicsMs_ = 0.0;
  std::string serverVersion_;
  std::string isaacVersion_;
  /** Scene spec sent to the server, resent with force=1 by the "Reload Isaac scene" button */
  std::string specJson_;
  bool reloadRequested_ = false;
  bool forceSceneReload_ = false;
  bool showCollisions_ = false;
  /** Ticker paused at the last after() call, i.e. when a reset is requested */
  bool pausedBeforeReset_ = false;
  bool pauseAfterReset_ = false;
  /** Ticker real-time settings as last set from mc_isaac (initial values from the mc_isaac command line);
   * changes made directly in the mc_rtc GUI Ticker tab are not seen. */
  bool tickerSync_ = true;
  double tickerTargetRatio_ = 1.0;
  bool timelinePlaying_ = true;
  int logNext_ = 0;
  std::chrono::steady_clock::time_point lastPing_;
  /** Timing breakdown (simulation.print_timings or the IsaacSim GUI tab): sums over the print period, in ms */
  struct Timings
  {
    size_t steps = 0;
    double sim = 0, wall = 0, before = 0, mcrtc = 0, after = 0, bridge = 0, server = 0, wait = 0, apply = 0,
           simulate = 0, state = 0, render = 0;
    size_t renders = 0;
  };
  bool printTimings_ = false;
  /** mc_rtc 3D GUI elements drawn in Isaac (sent at most every 50 ms) */
  std::unique_ptr<Markers> markers_;
  bool markersEnabled_ = true;
  std::chrono::steady_clock::time_point lastMarkers_;
  /** 2D GUI in the Isaac window: none | minimal (mc_isaac panel) | full (mirror of the whole mc_rtc GUI) */
  std::string guiMode_ = "minimal";
  /** markers_ when gui: full (the mirror also handles the 3D elements), else nullptr */
  GuiMirror * guiMirror_ = nullptr;
  std::chrono::steady_clock::time_point lastGui_;
  /** Commands of the mc_rtc IsaacSim tab, run by the next after() (not inside the GUI request handling) */
  std::vector<mc_rtc::Configuration> guiCommands_;
  double stepMs_ = 20.0;
  double renderMs_ = 0.0;
  /** sim/real time ratio over ~1 s windows */
  double simRealRatio_ = 0.0;
  double ratioSim_ = 0.0;
  std::chrono::steady_clock::time_point ratioWall_;
  /** Update simRealRatio_ (paused time is not counted) */
  void updateRatio(bool running);
  /** (Re)create the in-process GUI client: GuiMirror with gui: full, else Markers */
  void connectMarkers(mc_control::MCGlobalController & gc);
  /** Markers, GUI mirror and gui mode added to a bridge request */
  void addMarkers(mc_rtc::Configuration & header);
  double printPeriod_ = 5.0;
  Timings timings_;
  std::chrono::steady_clock::time_point beforeStart_, beforeEnd_, lastBeforeStart_, timingsStart_;
  /** Add one control step to timings_ (plugin side + server side timings of the reply), print every printPeriod_ */
  void accumulateTimings(const mc_rtc::Configuration & reply, double after_ms, double bridge_ms, double sim_dt);
  /** Per control step averages of timings_ (--print-timings output) */
  std::string timingsSummary() const;
  std::vector<IsaacRobot> robots_;
  /** <key>.yaml search order: ~/.config/mc_rtc/mc_isaac, description_paths, mc_isaac share (robots: overrides first) */
  std::vector<std::filesystem::path> descriptionDirs_;
  mc_rtc::Configuration descriptionOverrides_;
  /** mc_rtc robots not simulated (e.g. visual-only goal/ghost robots) */
  std::vector<std::string> excludedRobots_;
  /** Fixed scene camera (position, target, focal_length, viewport) used by the MJPEG stream / headless viewport */
  mc_rtc::Configuration camera_;
  /** Browser stream address reported by the server (empty without --stream mjpeg) */
  std::string streamUrl_;
  Bridge bridge_;
};

} // namespace mc_isaac
