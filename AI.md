# mc_isaac — developer / agent notes

Read this before changing mc_isaac and keep it up to date: design decisions, protocol, pitfalls, open issues.
User documentation is in `README.md`; the server protocol is also documented at the top of
`server/mc_isaac_server.py`. Keep entries short and factual, no change logs (git has the history).

Versions: server `0.12.0`, plugin requires `>= 0.12.0` (`MIN_SERVER_VERSION`). Bump both on protocol changes.

---

## 1. Goal and scope

`mc_isaac` = `mc_mujoco` for **Isaac Sim** (5.1 and 6.0): run mc_rtc controllers against Isaac Sim physics/rendering.

- Isaac Sim runs as a **persistent server** (docker image, apptainer `.sif` or local install), mc_rtc anywhere
  (native or another container) on the same machine. They only talk over TCP (HTTP 5050 + bridge 5055); assets are
  uploaded by content hash, no shared file system is assumed.
- Lockstep simulation (mc_rtc dt = N physics substeps), single environment. Parallel envs, ROS 2 / ros2_control
  backends: out of scope for now (Isaac Sim 6.0 `isaacsim.ros2.control` would be the starting point).
- Robots are described by data-only `<robot>_isaac_description` packages (USD + small yaml), like
  `<robot>_mj_description` for mc_mujoco. JVRC1 is embedded in mc_isaac for quick tests.

## 2. Architecture (decided, validated)

- mc_rtc side = **`IsaacSim` GlobalPlugin** (C++) run by the stock **`mc_rtc_ticker`**, started by the `mc_isaac`
  command (sets `MC_ISAAC_*` env variables, execs the ticker). The ticker provides loop, real-time sync/ratio,
  step-by-step, reset and its `Ticker` GUI tab, which stays the single source of truth for run/pause/step: the Isaac
  panel and timeline buttons only *press* Ticker GUI elements (`gui->handleRequest({"Ticker"}, name)`).
  - The plugin is inactive unless `MC_ISAAC_ACTIVE=1`, so it can stay in the `Plugins` list.
  - `before()` overrides the ticker's open-loop sensors (`Ticker::simulate_sensors()` runs before `gc.run()`) with
    the last Isaac state; `after()` sends the output robot commands and steps Isaac.
  - `should_always_run`: while paused (`gc.running == false`) `after()` pings the server at 20 Hz to follow Isaac
    buttons / panel commands / marker edits.
- Isaac side = **`server/mc_isaac_server.py`**: single file, stdlib + numpy (+ PIL for MJPEG) + Isaac Sim, no
  IsaacLab. `mc_isaac_server` (launcher, stdlib only) copies it into the container at each start, so server changes
  only need a server restart.
- Physics is stepped manually through `omni.physx` `simulate/fetch_results` and the PhysX **tensor API**
  (`omni.physics.tensors`, numpy frontend): articulation views for robots, rigid body views for objects. The Kit
  timeline never steps physics (`/app/player/playSimulations` false); it only mirrors run/pause.
- Viewport sync through **Fabric** (`omni.physx.fabric`, `update(0, 0)` before each render), USD write-back
  fallback (`--no-fabric`). Same choices as IsaacLab `use_fabric=True`.

## 3. Protocol summary

- HTTP: `GET /status` (state, versions, `bridge_port`, `stream_url` only with mjpeg), `GET /logs?since=ID`,
  `HEAD|PUT /upload/<sha256>?filename=` (asset cache `<cache>/<sha>/<file>`, sha verified), `POST /scene[?force=1]`
  (same spec hash → reset only), `POST /reset|/clear|/shutdown`, MJPEG `/`, `/stream`, `/snapshot.jpg`.
- Scene spec (JSON, built by the plugin from `gc.robots()` + descriptions): `physics_dt`, `ground`, `camera`,
  `show_collisions`, `robots: [{name, usd{filename, sha256, extra[{path, sha256}]}, fixed, rigid, mass, pos,
  quat(wxyz), init_q{joint: q}, drives[], self_collisions, solver_iterations, torque_control, imus[{name, link, pos,
  quat}], force_sensors[{name, link, pos, quat}]}]`.
- Bridge frame `!II` + JSON header + float64 LE payload. Messages `hello`, `step{n_substeps, interpolate, render}`,
  `state`, `reset`, `ping`. Every request carries `running` and `ticker{step_by_step, sync, target_ratio, dt,
  markers}`, `gui_mode` (none/minimal/full), at most every 50 ms `markers{lines, points, handles}` and, with gui:
  full, at most every 100 ms and only when changed `gui{tree, data, schemas, plots, plots_active, full}`. Replies:
  `timeline_playing`, `isaac_event` (play/pause pressed in Isaac), `reloaded` (Isaac Stop rebuilt the scene),
  `commands` (Isaac panel), `gui_requests` (handles moved with the gizmo), `gui_widget_requests` ([{key, data?}],
  mirrored GUI actions), `gui_resync` (send the full GUI again), `render_ms`, `log_next`, `timings`.
- State per robot: q, alpha, tau (Isaac joint order), root pose (x y z qw qx qy qz), root velocity (world linear of
  the root **origin**, world angular), 10 values per IMU (world quat, gyro, proper acceleration in the sensor frame),
  6 per force sensor (force, couple in the sensor frame). Rigid objects: pose + velocity only.
- mc_rtc `Configuration` cannot read JSON `null`: omit keys instead of sending null.

## 4. Implementation notes

- **Description lookup** (plugin): keys = robot module name, robot name, `MainRobot` alias; dirs = inline `robots:`
  → `~/.config/mc_rtc/mc_isaac` → `description_paths` → `<mc_isaac prefix>/share/mc_isaac` →
  `<mc_rtc prefix>/share/mc_isaac`. Relative `usd` paths are relative to the yaml. `builtin: ground_plane` = ground
  (mc_rtc `env/ground` is a robot, like in mc_mujoco). Main robot without description → error; others → warning.
- **Fixed base**: if the USD has no world joint, the server adds `mc_isaac_fixed_root` (FixedJoint world → root body)
  and moves `ArticulationRootAPI` to the parent if it was on the root body (IsaacLab `fix_root_link` approach).
- **Rigid objects** (robots without ref joints): ArticulationRootAPI removed, kinematic if fixed. PhysX refuses
  velocities on kinematic bodies (skip them at reset).
- **Drives**: tensor setters (`set_dof_stiffnesses/dampings/max_forces/max_velocities/armatures`), rad units =
  IsaacLab `ImplicitActuatorCfg`. USD drives must be of type **force** (see §5). `torque_control` → gains 0, mc_rtc
  torques as actuation forces. PhysX mimic joints (`PhysxMimicJointAPI`, e.g. Revo2 distal joints) get no drive.
- **Commands** are linearly interpolated over the substeps (mc_mujoco frameskip semantics);
  `physics_dt: auto` = largest divisor of the Timestep ≤ 5 ms.
- **Joint mapping** by name: Isaac order (breadth-first) ≠ mc_rtc `refJointOrder`. Isaac joints unknown to mc_rtc
  hold their initial position; mc_rtc joints not in Isaac keep the ticker values.
- **Sensors**: `FloatingBase` only when attached to the root body (PhysX root velocity is at the COM → converted to
  the origin). Other body sensors = IMU (acceleration by finite differences over a control step). Force sensors =
  negated `get_link_incoming_joint_force()` of the sensor parent link (wrench from the parent, link frame, at the
  joint = link origin for URDF imports), moved to the sensor pose: reads contact wrench + weight of the bodies
  after the sensor (feet Fz > 0 standing). Needs the sensor link to be a separate USD body (no fixed-joint merge).
  Validated on JVRC1: feet Fz sum = weight − feet weight.
- **Pause/reset**: Isaac Pause/Play → `isaac_event` → plugin toggles Ticker "Step by step"; `timeline.commit()` after
  play/pause (otherwise applied only at the next app update and back-to-back steps see a fake button press). The
  ticker leaves step-by-step to run its reset step: `reset()` sets `pauseAfterReset_` and the next running
  `after()` presses "Step by step" again. Isaac Stop invalidates the tensor views → server rebuilds the same scene,
  reply `reloaded` → Ticker Reset. Pause/Stop with no controller connected clears the scene.
- **Isaac panel** (omni.ui, server): UI callbacks run inside `app.update()`, so they never act directly: ticker
  commands are queued and sent with the next bridge reply, scene actions run from the main loop.
- **Markers**: `Markers` = in-process `mc_control::ControllerClient` connected to `gc.server()` (reads the published
  GUI buffer, no side effect; `update()` parses each publication once, by hash, so accumulated trajectories and plot
  points are not duplicated). Lines/points → `isaacsim.util.debug_draw`; editable Point3D/Transform/Rotation →
  sphere prims under `/World/mc_isaac_markers` that the user moves with the gizmo; the server returns their poses
  (0.5 s hold against mc_rtc overwrites) → GUI requests.
- **2D GUI modes** (`gui:` / `--gui`, sent as `gui_mode`): server shows the mc_isaac panel without client and with
  `minimal`, hides it with `none` and `full`; restored at disconnect. The mc_rtc `IsaacSim` tab always has the panel
  controls (callbacks queue commands run by the next `after()`, never inside the GUI request handling).
- **Full GUI mirror** (`gui: full`): plugin `GuiMirror` (subclass of Markers, so one GUI parse for both) records the
  2D elements of each parse in a category tree; element key = category path + name joined by `\x1f`; forms keep
  nested fields (object, generic array template, one-of options); `Schema` elements: the plugin loads the schema
  directory from `mc_rtc::JSON_SCHEMA_PATH` and resolves `$ref`/`allOf` (as mc_rtc-imgui), the server converts the
  JSON schema to form fields (same rules as mc_rtc-imgui `ObjectForm`: robot/body/surface/frame names → data combos).
  Server `GuiMirror`: one omni.ui window per top-level category docked next to Property, sub-categories as tab
  buttons (only the selected one built), each element in its own `ui.Frame` (values refreshed in place, frame rebuilt
  when the structure changes), 0.5 s hold after a user action, inputs Edit/Done like mc-rtc-magnum, forms keep the
  edited values (`_transfer`) when mc_rtc updates their defaults. Plots: points accumulated by the server (4000 per
  series), rendered with numpy into a `ui.ByteImageProvider` (lines, dashes, Point style = last point, polygons).
  Not mirrored: Robot/Visual elements (3D) and the interactive 3D markers of form point/rotation/transform inputs
  (edited numerically).
- **Display modes**: window (X11 cookie forwarded by the launcher), `--headless`, `--stream mjpeg` (replicator render
  product + rgb annotator, JPEG encoding thread; Replicator only captures while the timeline plays, so the stream
  freezes while paused), `--stream webrtc` (Isaac livestream extension, untested). Headless containers still get
  X forwarded when `DISPLAY` is set (hybrid-GPU laptops need it for the NVIDIA Vulkan/CUDA init).
- **Warm-up**: the server reports `ready` only after frames are consistently fast (RTX shader compilation can take
  minutes in a new container). Headless + mjpeg: a temporary render product forces the compilation at start.
- **Timings**: `print_timings` splits a control step into mc_rtc / plugin / bridge / server queue wait / apply /
  PhysX / state read / renders. Rendering shares the server main thread with stepping (`--stepping-fps`).
- **Converters**: `mc_isaac_mesh_to_usd` (stdlib, STL/OBJ → USDA rigid body, used for env/table);
  `mc_isaac_urdf_to_usd.py` (Isaac python, URDF importer + mc_rtc fixes, see §5; used for JVRC1).

## 5. Pitfalls (Isaac Sim 5.1)

- URDF importer: drive type defaults to **acceleration** (gains scaled by the joint inertia: a humanoid cannot
  stand); links without `<inertial>` get **1 kg**; imported PhysX mimic joints can leave followers swinging.
  `mc_isaac_urdf_to_usd.py` fixes all three (force drives, 1e-4 kg, mimic → plain drives commanded by mc_rtc).
  URDF effort limits are often placeholders (JVRC1: 100 N.m) → `max_effort` in the yaml.
- `timeline.play()/pause()` take effect at the next app update unless `timeline.commit()`.
- `physxfabric.update(0, 0)` does not show teleports (reset) before a physics step; `force_update` needs `(dt, t)`.
- Exceptions in the main loop must be caught: `app.close()` exits before Python prints the traceback.
- One-off `SimulationApp` scripts: set `PYTHONUNBUFFERED=1` (close() exits without flushing stdout). `pxr` outside a
  SimulationApp: `PYTHONPATH=<isaac>/extscache/omni.usd.libs-*` and `LD_LIBRARY_PATH=<that>/bin`.
- Quaternions: tensor API `xyzw`, USD `Gf.Quat` (real, imaginary), spec/mc_rtc `wxyz`, `sva` rotations are the
  transpose of Eigen's.
- `mc_rtc::Configuration::operator()(key, T&)` returns void and assigns in place when the key exists.
- `/persistent/physics/visualizationDisplayColliders` (2/0) shows collision shapes.
- Docker: options are stored in a container label; changed options require `--recreate`.
  Bare `--docker` selects the only tested image, `nvcr.io/nvidia/isaac-sim:5.1.0`. Its dedicated profile uses
  UID/GID 1234, NVIDIA's six mounts under `~/docker/isaac-sim` (caches survive recreation), and a read-only
  session-cookie bind at `/isaac-sim/.Xauthority`. New mount directories are sticky/shared-writable when the
  host UID differs; existing permissions are untouched. Other images retain the legacy profile (not validated).
- Docker X11: `docker cp` defaults to root ownership; a mode-0600 cookie is unreadable to NGC's non-root user.
  The official profile stages a readable cookie in a private host directory; the legacy profile copies it via
  a tar archive with mode 0444 inside the container, keeping the host copy mode 0600. Host Vulkan mounts are opt-in.
  Cookie discovery checks explicit/home files plus current-user GDM, LightDM, SDDM and Xwayland locations;
  `/run/user/<uid>` is the fallback when `XDG_RUNTIME_DIR` is unset. Never disable X11 access control automatically.
- omni.ui: `ui.Frame(build_fn)` builds lazily when drawn; a headless Kit never draws docked windows, so GUI tests
  must call the build functions inside `with window.frame:` explicitly. Setting a model value fires its callbacks
  (guard with a flag). `ControllerClient::default_impl` must be overridden (no-op) or every unhandled element warns.

## 6. Status

Validated by the user: lockstep stepping, scene from `gc.robots()` + description packages, asset upload/cache,
persistent server (fast restart), launch modes, floating base + IMU, objects/env table, Honda/G1/Revo2 packages
(G1 collisions, G1 Revo2 hand mount), Isaac panel, MJPEG headless stream, markers.
Implemented, awaiting user test: `mc_isaac` CLI options, JVRC1 example + force sensors (validated by the agent
end-to-end: standing, feet Fz sum 583 N), bridge port taken from `/status`, gui modes + full GUI mirror (validated by
the agent on a headless test server with the JVRC1 CoM controller: every element of the GUI built, Form/Schema
forms built and collected for every MetaTask schema, requests applied by mc_rtc, panel visibility per mode; not yet
seen in a real Isaac window).

## 7. Open issues / ideas

- After Pause → Reset the viewport only updates at the next step (Fabric not refreshed by teleports).
- MJPEG stream frozen while paused; debug-draw markers probably not visible in the stream; WebRTC untested.
- Desktop freezes on PRIME laptops while the Isaac window is focused (driver/compositor, also with IsaacLab).
- Mimic joints of the Revo2/G1 USDs still use PhysX mimic constraints (fine so far); regenerate with
  `mc_isaac_urdf_to_usd.py` if the followers misbehave.
- Possible: sim-only objects in the plugin yaml (mc_mujoco `objects:`), Ctrl+drag forces, ros2_control backend.

## 8. Working rules (from the user)

- Agents implement and run minimal checks (pyflakes, plugin build, short tests with a temporary server on other
  ports, e.g. `--http-port 5150 --bridge-port 5155 --name <tmp>`, removed afterwards); the user runs the real tests.
- Never stop or reuse the user's running server/containers for tests.
- No machine-specific paths or names in this repository (docs, code, comments).

## 9. References

- mc_rtc: https://jrl-umi3218.github.io/mc_rtc/ (Ticker: `src/mc_control/Ticker.cpp`, `utils/mc_rtc_ticker.cpp`)
- mc_mujoco: https://github.com/rohanpsingh/mc_mujoco (reference behaviour: `src/mj_sim.cpp`, `robots/`)
- Isaac Sim docs: https://docs.isaacsim.omniverse.nvidia.com/latest/ (URDF importer, tensor API, livestream)
- Isaac Sim containers: NGC `nvcr.io/nvidia/isaac-sim:<version>`
