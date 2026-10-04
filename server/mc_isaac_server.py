"""mc_isaac server: hosts Isaac Sim for the mc_rtc IsaacSim plugin.

Must be run with the python of an Isaac Sim installation (``python.sh``). Only depends on the Python standard
library, numpy, PIL (MJPEG stream) and Isaac Sim (5.1 / 6.0), so ``mc_isaac_server`` can copy it into any Isaac Sim
container at start.

Threads
    - main thread: owns Kit / USD / PhysX (not thread-safe). Its loop (``main``) runs the tasks queued by the other
      threads (``MainThreadExecutor``), the Isaac panel actions, and renders at most ``--max-fps``
      (``--stepping-fps`` while a controller steps).
    - HTTP thread pool (``make_handler``): status/logs, asset upload, scene load/reset/clear, MJPEG stream.
    - bridge thread (``BridgeServer``): one mc_rtc client at a time, lockstep: each request is executed on the main
      thread and answered before the next one is read.
    - MJPEG encoder thread (``MjpegStreamer``).

HTTP API (port 5050)
    GET  /status, /logs?since=ID              server state (JSON), log lines
    HEAD /upload/<sha256>                     200 if the asset is cached
    PUT  /upload/<sha256>?filename=NAME       store an asset (content-addressed, checked against the sha256)
    POST /scene[?force=1]                     build the scene from a JSON spec (reset only if the spec is unchanged)
    POST /reset, /clear, /shutdown
    GET  /, /stream, /snapshot.jpg            MJPEG stream (``--stream mjpeg`` only)

Bridge protocol (port 5055)
    Frame = ``!II`` (header length, payload length) + JSON header + float64 little-endian payload.
    Requests: ``hello`` (scene description), ``step`` (payload = commands, reply payload = state), ``state``,
    ``reset``, ``ping``. Requests also carry the mc_rtc ticker state (``running``, ``ticker``), the mc_rtc GUI
    markers, the requested 2D GUI (``gui_mode``) and, with gui: full, the mirrored GUI (``gui``, see GuiMirror);
    replies carry Isaac events (timeline buttons, panel commands, marker edits, GUI requests) and timings.
    Command payload, per robot: q_ref[n], alpha_ref[n], tau_ref[n].
    State payload, per robot: q[n], alpha[n], tau[n], root pose [x y z qw qx qy qz], root velocity [world linear of
    the root origin, world angular], 10 values per IMU, 6 values per force sensor (see ``describe``).
"""

import argparse
import collections
import functools
import hashlib
import io
import json
import os
import queue
import re
import signal
import socket
import struct
import threading
import time
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import numpy as np

SERVER_VERSION = "0.12.0"
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
FILENAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,200}$")
MAX_JSON_BODY = 16 * 1024 * 1024
# bridge framing: header length, payload length, JSON header, float64 little-endian payload
FRAME = struct.Struct("!II")


class LogBuffer:
    """Server log lines, printed and kept for ``GET /logs`` (the plugin forwards them to the mc_rtc console)."""

    def __init__(self, maxlen=2000):
        self._lock = threading.Lock()
        self._entries = collections.deque(maxlen=maxlen)
        self._next_id = 0

    def add(self, msg):
        with self._lock:
            self._entries.append((self._next_id, time.time(), msg))
            self._next_id += 1
        print(f"[mc_isaac_server] {msg}", flush=True)

    def since(self, first_id):
        """Log entries with id >= first_id and the next id (for GET /logs?since=)."""
        with self._lock:
            return [e for e in self._entries if e[0] >= first_id], self._next_id

    @property
    def next_id(self):
        with self._lock:
            return self._next_id


class ServerState:
    """Everything shared between the threads: the current scene, the connected client, the UI helpers.

    Methods below ``main thread only`` touch Kit/USD/PhysX and must be called through ``executor``.
    """

    def __init__(self, args):
        self.args = args
        self.state = "starting"  # starting | warming_up | ready | stopping
        self.isaac_version = None
        self.window = None
        self.start_time = time.time()
        self.shutdown_requested = threading.Event()
        self.logs = LogBuffer()
        self.executor = MainThreadExecutor(self.logs)
        self.cache = AssetCache(args.cache_dir)
        self.app = None
        self.scene = None
        self.last_render = 0.0
        self.last_step = 0.0
        self.render_ms = 0.0
        # renders since the last bridge reply (they delay the steps: same thread)
        self.render_total_ms = 0.0
        self.render_count = 0
        self.physics_ms = 0.0
        self.fabric_update = None
        self.bridge_client = None
        self._idle_timeline_playing = None
        self.last_spec = None
        # ticker state reported by the plugin with each request, None when no controller or an old plugin
        self.ticker = None
        # Isaac panel -> controller commands, sent with the next bridge reply
        self.client_commands = []
        # Isaac panel actions run by the main loop (not inside app.update(), which runs the UI callbacks)
        self.panel_actions = []
        self.panel = None
        self.streamer = None
        self.markers = MarkerView(self.logs.add)
        # 2D GUI requested by the connected plugin: None (no client), "none", "minimal" (panel) or "full" (mirror)
        self.gui_mode = None
        self.gui_mirror = None

    def status(self):
        """GET /status payload: server state, versions, display mode, bridge client, current scene."""
        scene = self.scene
        out = {
            "state": self.state,
            "server_version": SERVER_VERSION,
            "isaac_version": self.isaac_version,
            "headless": self.args.headless,
            "stream": self.args.stream,
            "window": self.window,
            "uptime": round(time.time() - self.start_time, 1),
            "bridge_port": self.args.bridge_port,
            "bridge_client": self.bridge_client,
            "render_ms": round(self.render_ms, 2),
            "scene": None if scene is None else {
                "robots": [r.name for r in scene.robots],
                "physics_dt": scene.physics_dt,
                "sim_time": scene.sim_time,
                "spec_hash": scene.spec_hash,
            },
        }
        if self.args.stream == "mjpeg":
            # omitted otherwise: mc_rtc's Configuration cannot read null values
            out["stream_url"] = f"http://{self.args.host}:{self.args.http_port}/"
        return out

    # ---- main thread only

    def render(self):
        """Render one frame (main thread): flush physics to Fabric, app.update(), feed the stream."""
        t0 = time.perf_counter()
        if self.fabric_update is not None and self.scene is not None:
            # physics is stepped manually, so Fabric is not refreshed by PhysX on its own
            self.fabric_update.update(0.0, 0.0)
        self.app.update()
        self.last_render = time.perf_counter()
        self.render_ms = 0.9 * self.render_ms + 0.1 * 1e3 * (self.last_render - t0)
        self.render_total_ms += 1e3 * (self.last_render - t0)
        self.render_count += 1
        if self.streamer is not None:
            self.streamer.capture()

    def render_period(self):
        """Minimum time between renders: --stepping-fps while a client steps, --max-fps idle."""
        stepping = time.perf_counter() - self.last_step < 0.2
        fps = self.args.stepping_fps if stepping else self.args.max_fps
        return 1.0 / fps if fps > 0 else 0.0

    def load_scene(self, spec, force=False):
        """POST /scene: build the scene of the spec, or only reset it when the spec is unchanged."""
        self.last_spec = spec
        if not force and self.scene is not None and self.scene.spec_hash == spec_hash(spec):
            self.scene.reset()
            self.logs.add("Same scene requested: reset only")
            return {**self.scene.describe(), "reused": True}
        if self.scene is not None:
            self.scene.close()
            self.scene = None
        self.logs.add(f"Loading scene ({len(spec.get('robots', []))} robot(s))...")
        t0 = time.time()
        self.scene = Scene(self, spec)
        self.logs.add(f"Scene loaded in {time.time() - t0:.1f}s")
        return {**self.scene.describe(), "reused": False}

    def clear_scene(self):
        """POST /clear and panel: empty stage, refused while a controller is connected."""
        if self.bridge_client is not None:
            raise RuntimeError(f"a controller is connected ({self.bridge_client}), stop it before clearing the scene")
        import omni.usd

        if self.scene is not None:
            self.scene.close()
            self.scene = None
        omni.usd.get_context().new_stage()
        self.render()
        self.logs.add("Scene cleared")
        return {"cleared": True}

    def watch_idle_timeline(self):
        """Isaac Pause/Stop pressed while no controller is connected: clear the scene."""
        if self.scene is None:
            self._idle_timeline_playing = None
            return
        playing = self.scene.timeline.is_playing()
        was_playing = self._idle_timeline_playing
        self._idle_timeline_playing = playing
        # transitions only, so a scene left paused by a disconnected controller is kept
        if self.bridge_client is None and was_playing and not playing:
            self.logs.add("Isaac paused/stopped with no controller connected: clearing the scene")
            self.clear_scene()

    def reset_scene(self):
        """POST /reset and panel: robots back to their initial state."""
        if self.scene is None:
            raise RuntimeError("no scene loaded")
        self.scene.reset()
        return {"sim_time": self.scene.sim_time}

    def apply_gui_mode(self, mode):
        """Show the mc_isaac panel (no client, minimal), nothing (none) or the mc_rtc GUI mirror (full)."""
        if mode == self.gui_mode:
            return
        self.gui_mode = mode
        if self.panel is not None:
            self.panel.window.visible = mode in (None, "minimal")
        if mode == "full" and self.gui_mirror is None and self.window:
            self.gui_mirror = GuiMirror(self)
        elif mode != "full" and self.gui_mirror is not None:
            self.gui_mirror.close()
            self.gui_mirror = None

    def handle_bridge(self, header, payload):
        """One bridge request (main thread): Isaac/mc_rtc state exchange around the request itself."""
        kind = header.get("type")
        if isinstance(header.get("ticker"), dict):
            self.ticker = header["ticker"]
        if "gui_mode" in header:
            self.apply_gui_mode(header["gui_mode"])
        if self.gui_mirror is not None and isinstance(header.get("gui"), dict):
            self.gui_mirror.receive(header["gui"])
        reloaded = False
        if self.scene is not None and self.scene.invalid():
            # Isaac Stop button tears down physics: rebuild the same scene, the client resets its controller
            self.logs.add("Isaac physics was stopped (Stop button?): reloading the scene")
            spec = self.scene.spec
            self.scene.close()
            self.scene = None
            self.scene = Scene(self, spec)
            reloaded = True
        # mirror the controller run/pause state on the Isaac timeline before stepping
        event = self.scene.sync_timeline(header.get("running")) if self.scene is not None else None
        reply, out = self._bridge_request(kind, header, payload)
        scene = self.scene
        if isinstance(header.get("markers"), dict):
            requests = self.markers.update(header["markers"], scene)
            if requests:
                reply["gui_requests"] = requests
        reply["timeline_playing"] = scene is not None and scene.timeline.is_playing()
        if event is not None:
            reply["isaac_event"] = event
        reply["log_next"] = self.logs.next_id
        reply["reloaded"] = reloaded
        timings = reply.setdefault("timings", {})
        timings["renders"] = self.render_count
        timings["render"] = self.render_total_ms
        self.render_count, self.render_total_ms = 0, 0.0
        if self.client_commands:
            reply["commands"], self.client_commands = self.client_commands, []
        reply["render_ms"] = self.render_ms
        if self.gui_mirror is not None:
            if self.gui_mirror.need_full:
                reply["gui_resync"] = True
            if self.gui_mirror.requests:
                reply["gui_widget_requests"], self.gui_mirror.requests = self.gui_mirror.requests, []
        return reply, out

    def run_panel_actions(self):
        """Run the scene actions queued by the Isaac panel (outside of the UI callbacks)."""
        actions, self.panel_actions = self.panel_actions, []
        for name, fn in actions:
            try:
                fn()
            except Exception as e:
                self.logs.add(f"ERROR: {name}: {type(e).__name__}: {e}")

    def reload_last_scene(self):
        """Panel "Reload scene" without controller: rebuild the last loaded spec."""
        if self.last_spec is None:
            raise RuntimeError("no scene was loaded since the server started")
        self.load_scene(self.last_spec, force=True)

    def _bridge_request(self, kind, header, payload):
        """Dispatch one bridge message (ping, hello, step, state, reset) to the scene."""
        scene = self.scene
        if kind == "ping":
            return {"ok": True, "scene": scene is not None, "sim_time": None if scene is None else scene.sim_time}, b""
        if scene is None:
            raise RuntimeError("no scene loaded (POST /scene first)")
        if kind == "hello":
            return {"ok": True, **scene.describe()}, b""
        if kind == "step":
            return scene.step(header, payload)
        if kind == "state":
            return {"ok": True, "sim_time": scene.sim_time}, scene.read_state()
        if kind == "reset":
            scene.reset()
            self.logs.add("Scene reset to its initial state")
            return {"ok": True, "sim_time": scene.sim_time}, b""
        raise ValueError(f"unknown message type {kind!r}")


class _Task:
    """Call queued for the Isaac main thread; run_pending fills result or error and sets done."""
    def __init__(self, fn):
        self.fn = fn
        self.done = threading.Event()
        self.result = None
        self.error = None


class MainThreadExecutor:
    """Runs requests from HTTP/bridge threads on the Isaac main thread (Kit, USD and PhysX are not thread-safe)."""

    def __init__(self, logs):
        self._tasks = queue.Queue()
        self._logs = logs

    def call(self, fn, timeout=None):
        """Run fn on the main thread and wait for its result (from the HTTP/bridge threads)."""
        task = _Task(fn)
        self._tasks.put(task)
        if not task.done.wait(timeout):
            raise TimeoutError("Isaac main thread did not process the request in time")
        if task.error:
            raise RuntimeError(task.error)
        return task.result

    def run_pending(self, timeout):
        """Main loop: run the next queued call, waiting at most timeout seconds for one."""
        try:
            task = self._tasks.get(timeout=timeout) if timeout > 0 else self._tasks.get_nowait()
        except queue.Empty:
            return
        try:
            task.result = task.fn()
        except Exception as e:
            task.error = f"{type(e).__name__}: {e}"
            self._logs.add(f"ERROR: {task.error}")
            traceback.print_exc()
        finally:
            task.done.set()


class AssetCache:
    """Content-addressed asset store: <root>/<sha256>/<filename>."""

    def __init__(self, root):
        self.root = root
        os.makedirs(root, exist_ok=True)

    @staticmethod
    def _check(sha, filename=None):
        if not SHA256_RE.match(sha or ""):
            raise ValueError("invalid sha256")
        if filename is not None and not FILENAME_RE.match(filename):
            raise ValueError(f"invalid filename {filename!r}")

    def has(self, sha):
        """True if the asset sha256 was already uploaded (HEAD /upload/<sha256>)."""
        self._check(sha)
        d = os.path.join(self.root, sha)
        return os.path.isdir(d) and bool(os.listdir(d))

    def path(self, sha, filename):
        """Cache path of an asset (validated sha256 and file name)."""
        self._check(sha, filename)
        return os.path.join(self.root, sha, filename)

    def store(self, sha, filename, stream, length):
        """PUT /upload: stream the body to the cache, checking its sha256 before publishing it."""
        self._check(sha, filename)
        tmp = os.path.join(self.root, f".{sha}.{threading.get_ident()}.part")
        digest = hashlib.sha256()
        try:
            with open(tmp, "wb") as f:
                remaining = length
                while remaining > 0:
                    chunk = stream.read(min(1 << 20, remaining))
                    if not chunk:
                        raise ValueError("truncated upload")
                    digest.update(chunk)
                    f.write(chunk)
                    remaining -= len(chunk)
            if digest.hexdigest() != sha:
                raise ValueError("sha256 mismatch")
            os.makedirs(os.path.join(self.root, sha), exist_ok=True)
            os.replace(tmp, self.path(sha, filename))
        finally:
            if os.path.exists(tmp):
                os.remove(tmp)

    def bundle(self, usd):
        """Main USD + extra files (textures, sublayers) symlinked at their relative paths, returns the USD path."""
        files = [(usd["filename"], usd["sha256"])]
        for extra in usd.get("extra", []):
            parts = extra["path"].split("/")
            if not parts or any(not FILENAME_RE.match(part) for part in parts):
                raise ValueError(f"invalid extra file path {extra['path']!r}")
            files.append((extra["path"], extra["sha256"]))
        for path, sha in files:
            if not os.path.isfile(self.path(sha, os.path.basename(path))):
                raise RuntimeError(f"asset {path} ({sha[:12]}) not uploaded")
        if len(files) == 1:
            return self.path(usd["sha256"], usd["filename"])
        bundle = os.path.join(self.root, "bundles", spec_hash(files))
        for path, sha in files:
            link = os.path.join(bundle, path)
            if not os.path.lexists(link):
                os.makedirs(os.path.dirname(link), exist_ok=True)
                os.symlink(self.path(sha, os.path.basename(path)), link)
        return os.path.join(bundle, usd["filename"])


# ------------------------------------------------------------------------------------------ scene


def spec_hash(spec):
    """Stable hash of a JSON-like object (scene specs, asset bundles)."""
    return hashlib.sha256(json.dumps(spec, sort_keys=True).encode()).hexdigest()


def configure_kit_for_stepping(log, use_fabric=True, vsync=False):
    """Kit settings for fast manual stepping (same choices as IsaacLab with use_fabric=True).

    Returns the PhysX Fabric interface used to flush physics results before rendering, or None (USD write-back).
    """
    import carb
    import omni.kit.app

    settings = carb.settings.get_settings()
    # app.update() must not block on vsync / Kit's 60 Hz main loop limiter: mc_isaac limits rendering itself
    settings.set_bool("/app/vsync", vsync)
    settings.set_bool("/app/runLoops/main/rateLimitEnabled", False)
    settings.set_bool("/physics/updateVelocitiesToUsd", False)
    settings.set_bool("/physics/updateParticlesToUsd", False)
    settings.set_bool("/physics/updateForceSensorsToUsd", False)
    fabric_update = None
    if use_fabric:
        manager = omni.kit.app.get_app().get_extension_manager()
        try:
            if not manager.is_extension_enabled("omni.physx.fabric"):
                manager.set_extension_enabled_immediate("omni.physx.fabric", True)
            from omni.physxfabric import get_physx_fabric_interface

            fabric_update = get_physx_fabric_interface()
        except Exception as e:
            log(f"WARNING: omni.physx.fabric unavailable ({e}), falling back to USD write-back (slower)")
    # without Fabric, the viewport only sees physics results written back to USD
    settings.set_bool("/physics/updateToUsd", fabric_update is None)
    log(f"Viewport sync: {'Fabric' if fabric_update else 'USD write-back'}")
    return fabric_update


def _find_prim(root, predicate):
    """First prim under root (included) matching predicate, or None."""
    from pxr import Usd

    for prim in Usd.PrimRange(root):
        if predicate(prim):
            return prim
    return None


def _has_world_joint(robot_prim):
    """True if the robot USD already attaches a body to the world (joint with a single body)."""
    from pxr import Usd, UsdPhysics

    for prim in Usd.PrimRange(robot_prim):
        if not prim.IsA(UsdPhysics.Joint):
            continue
        joint = UsdPhysics.Joint(prim)
        if joint.GetExcludeFromArticulationAttr().Get():
            continue
        if not joint.GetBody0Rel().GetTargets() or not joint.GetBody1Rel().GetTargets():
            return True
    return False


def _fix_base(stage, robot_prim, art_root, log):
    """Attach the articulation root body to the world with a fixed joint (like IsaacLab fix_root_link)."""
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    if _has_world_joint(robot_prim):
        return art_root
    root_body = art_root if art_root.HasAPI(UsdPhysics.RigidBodyAPI) else \
        _find_prim(art_root, lambda p: p.HasAPI(UsdPhysics.RigidBodyAPI))
    if root_body is None:
        raise RuntimeError(f"no rigid body found under articulation root {art_root.GetPath()}")
    world = UsdGeom.Xformable(root_body).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    pos = world.ExtractTranslation()
    rot = world.RemoveScaleShear().ExtractRotationQuat()
    joint = UsdPhysics.FixedJoint.Define(stage, robot_prim.GetPath().AppendChild("mc_isaac_fixed_root"))
    joint.CreateBody1Rel().SetTargets([root_body.GetPath()])
    joint.CreateLocalPos0Attr(Gf.Vec3f(*pos))
    joint.CreateLocalRot0Attr(Gf.Quatf(rot.GetReal(), Gf.Vec3f(*rot.GetImaginary())))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    joint.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    if art_root == root_body:
        # the fixed joint must belong to the articulation: move the root API above the root body
        parent = root_body.GetParent()
        art_root.RemoveAPI(UsdPhysics.ArticulationRootAPI)
        UsdPhysics.ArticulationRootAPI.Apply(parent)
        art_root = parent
    log(f"  fixed base: joint world -> {root_body.GetPath()}, articulation root {art_root.GetPath()}")
    return art_root


def _reference_model(stage, spec, cache, log):
    """/World/<name> Xform at the initial pose, with the uploaded USD referenced at /World/<name>/model."""
    from pxr import Gf, Sdf, UsdGeom

    name = spec["name"]
    if not Sdf.Path.IsValidIdentifier(name):
        raise ValueError(f"robot name {name!r} is not a valid USD identifier")
    usd = spec["usd"]
    usd_path = cache.bundle(usd)
    log(f"Robot {name}: {usd['filename']}" + (f" (+{len(usd['extra'])} files)" if usd.get("extra") else ""))
    robot = UsdGeom.Xform.Define(stage, f"/World/{name}")
    x, y, z = spec.get("pos", [0.0, 0.0, 0.0])
    qw, qx, qy, qz = spec.get("quat", [1.0, 0.0, 0.0, 0.0])
    robot.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(x, y, z))
    robot.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(qw, Gf.Vec3d(qx, qy, qz)))
    model = stage.DefinePrim(f"/World/{name}/model")
    if not model.GetReferences().AddReference(usd_path):
        raise RuntimeError(f"cannot reference {usd_path}")
    return robot.GetPrim()


def _apply_mass(body, spec, log):
    """Optional description mass of a rigid object."""
    from pxr import UsdPhysics

    if spec.get("mass") is not None:
        UsdPhysics.MassAPI.Apply(body).CreateMassAttr(float(spec["mass"]))
        log(f"  mass {spec['mass']} kg on {body.GetPath()}")


GRAVITY = np.array([0.0, 0.0, -9.81])


def quat_xyzw_to_matrix(q):
    """Rotation matrix of a quaternion given as x y z w (PhysX tensor API order)."""
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quat_wxyz(R):
    """Quaternion w x y z (mc_rtc order) of a rotation matrix."""
    w = np.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = np.copysign(np.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1.0 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1.0 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return np.array([w, x, y, z])


def origin_velocity(com_lin_vel, ang_vel, rotation, com_local):
    """PhysX gives the linear velocity of the center of mass, mc_rtc expects the one of the body origin."""
    return com_lin_vel - np.cross(ang_vel, rotation @ com_local)


class ImuSensor:
    """mc_rtc body sensor (gyro + accelerometer) on an articulation link, acceleration by finite differences."""

    def __init__(self, spec):
        self.name = spec["name"]
        self.link = spec["link"]
        self.pos = np.array(spec.get("pos", [0.0, 0.0, 0.0]))
        qw, qx, qy, qz = spec.get("quat", [1.0, 0.0, 0.0, 0.0])
        self.rot = quat_xyzw_to_matrix([qx, qy, qz, qw])
        self.index = None
        self.reset()

    def reset(self):
        self.prev_vel = None
        self.prev_time = None
        self.values = np.array([1.0, 0, 0, 0, 0, 0, 0, 0, 0, -GRAVITY[2]])

    def update(self, transforms, velocities, coms, sim_time):
        """New sensor values from the link poses/velocities of the last physics step."""
        if self.index is None:
            return
        tr, vel = transforms[self.index], velocities[self.index]
        R_link = quat_xyzw_to_matrix(tr[3:7])
        ang = vel[3:6]
        # velocity of the sensor point from the link center of mass velocity
        lin = vel[0:3] + np.cross(ang, R_link @ (self.pos - coms[self.index][:3]))
        acc = np.zeros(3)
        if self.prev_vel is not None and sim_time > self.prev_time:
            acc = (lin - self.prev_vel) / (sim_time - self.prev_time)
        self.prev_vel, self.prev_time = lin, sim_time
        R_sensor = R_link @ self.rot
        # accelerometers measure the proper acceleration (a - g); gyro and accelerometer are in the sensor frame
        self.values = np.concatenate([matrix_to_quat_wxyz(R_sensor), R_sensor.T @ ang, R_sensor.T @ (acc - GRAVITY)])


class ForceSensor:
    """mc_rtc force sensor on an articulation link (the sensor parent body), values in the sensor frame.

    PhysX gives the incoming joint wrench of each link: wrench applied by the parent link, in the link frame, at the
    joint (= link origin for URDF imports). Its opposite is what a real sensor at that joint measures: contact
    wrench + weight of the bodies after the sensor (standing feet: Fz > 0, free hand: its own weight, removed in
    mc_rtc by ``wrenchWithoutGravity``).
    """

    def __init__(self, spec):
        self.name = spec["name"]
        self.link = spec["link"]
        self.pos = np.array(spec.get("pos", [0.0, 0.0, 0.0]))
        qw, qx, qy, qz = spec.get("quat", [1.0, 0.0, 0.0, 0.0])
        self.rot = quat_xyzw_to_matrix([qx, qy, qz, qw])
        self.index = None
        self.values = np.zeros(6)  # force xyz, torque xyz in the sensor frame

    def update(self, wrenches):
        """New wrench from the incoming joint wrenches of all links (one row per link)."""
        if self.index is None:
            return
        force, torque = -wrenches[self.index, 0:3], -wrenches[self.index, 3:6]
        torque = torque - np.cross(self.pos, force)
        self.values = np.concatenate([self.rot.T @ force, self.rot.T @ torque])


class RobotHandle:
    """Articulated robot of the scene spec, driven through the PhysX tensor API (one articulation view).

    Position/velocity targets come from mc_rtc (PhysX joint drives = PD), torques are added as actuation forces.
    """

    def __init__(self, stage, spec, cache, log):
        from pxr import PhysxSchema, Usd, UsdPhysics

        self.spec = spec
        self.name = spec["name"]
        robot = _reference_model(stage, spec, cache, log)
        art_root = _find_prim(robot, lambda p: p.HasAPI(UsdPhysics.ArticulationRootAPI))
        if art_root is None:
            raise RuntimeError(f"no ArticulationRootAPI found in {spec['usd']['filename']}")
        if spec.get("fixed", True):
            art_root = _fix_base(stage, robot, art_root, log)
        self.root_path = str(art_root.GetPath())
        if spec.get("self_collisions") is not None:
            PhysxSchema.PhysxArticulationAPI.Apply(art_root).CreateEnabledSelfCollisionsAttr(bool(spec["self_collisions"]))
            log(f"  {self.name}: self-collisions {'enabled' if spec['self_collisions'] else 'disabled'}")
        iterations = spec.get("solver_iterations") or {}
        if iterations:
            articulation = PhysxSchema.PhysxArticulationAPI.Apply(art_root)
            if iterations.get("position") is not None:
                articulation.CreateSolverPositionIterationCountAttr(int(iterations["position"]))
            if iterations.get("velocity") is not None:
                articulation.CreateSolverVelocityIterationCountAttr(int(iterations["velocity"]))
            log(f"  {self.name}: solver iterations {iterations}")
        # PhysX mimic joints follow another joint and have no drive
        self.mimic_joints = {p.GetName() for p in Usd.PrimRange(robot)
                             if any(s.startswith("PhysxMimicJointAPI") for s in p.GetAppliedSchemas())}
        self.imus = [ImuSensor(imu) for imu in spec.get("imus", [])]
        self.force_sensors = [ForceSensor(fs) for fs in spec.get("force_sensors", [])]
        self.view = None

    def attach(self, sim_view, log):
        """After physics start: create the articulation view, apply drives, index links of the
        sensors, record the initial state (init_q from the spec).
        """
        self.view = sim_view.create_articulation_view(self.root_path)
        if self.view is None or self.view.count != 1:
            raise RuntimeError(f"cannot create articulation view for {self.root_path}")
        meta = self.view.shared_metatype
        self.joints = list(meta.dof_names)
        self.n = self.view.max_dofs
        self.fixed_base = bool(meta.fixed_base)
        self.idx = np.array([0], dtype=np.int32)
        log(f"  {self.name}: {self.n} dofs, fixed_base={self.fixed_base}")
        self._apply_drives(log)
        if self.spec.get("torque_control", False):
            zeros = np.zeros((1, self.n), dtype=np.float32)
            self.view.set_dof_stiffnesses(zeros, self.idx)
            self.view.set_dof_dampings(zeros, self.idx)
            log(f"  {self.name}: torque control (drive gains set to 0, mc_rtc torques applied as efforts)")
        try:
            # local center of mass pose of every link (x y z qx qy qz qw)
            self.coms = np.array(self.view.get_coms(), dtype=np.float64).reshape(self.view.max_links, 7)
        except Exception as e:
            log(f"  WARNING: cannot read link centers of mass ({e}), velocities given at the centers of mass")
            self.coms = np.zeros((self.view.max_links, 7))
        links = list(meta.link_names)
        for imu in self.imus:
            if imu.link in links:
                imu.index = links.index(imu.link)
                log(f"  {self.name}: IMU {imu.name} on {imu.link}")
            else:
                log(f"  WARNING: IMU {imu.name}: link {imu.link} not in {self.name}, sensor not simulated")
        for fs in self.force_sensors:
            if fs.link in links:
                fs.index = links.index(fs.link)
                log(f"  {self.name}: force sensor {fs.name} on {fs.link}")
            else:
                log(f"  WARNING: force sensor {fs.name}: link {fs.link} not in {self.name}, sensor not simulated")

        q0 = self.view.get_dof_positions()[0].astype(np.float64)
        for joint, value in self.spec.get("init_q", {}).items():
            if joint in self.joints:
                q0[self.joints.index(joint)] = value
            else:
                log(f"  WARNING: init_q joint {joint} not in {self.name}")
        self.init_q = q0
        self.init_root = self.view.get_root_transforms().copy()

    def _apply_drives(self, log):
        """Description ``drives`` groups (regex over joint names) -> PhysX drive parameters, in rad units."""
        setters = {
            "stiffness": (self.view.get_dof_stiffnesses, self.view.set_dof_stiffnesses),
            "damping": (self.view.get_dof_dampings, self.view.set_dof_dampings),
            "max_effort": (self.view.get_dof_max_forces, self.view.set_dof_max_forces),
            "max_velocity": (self.view.get_dof_max_velocities, self.view.set_dof_max_velocities),
            "armature": (self.view.get_dof_armatures, self.view.set_dof_armatures),
        }
        covered = np.zeros(self.n, dtype=bool)
        for group in self.spec.get("drives", []):
            pattern = re.compile(group["joints"])
            mask = np.array([bool(pattern.fullmatch(j)) for j in self.joints])
            if not mask.any():
                log(f"  WARNING: drive group {group['joints']!r} matches no joint")
                continue
            covered |= mask
            for key, (getter, setter) in setters.items():
                if group.get(key) is None:
                    continue
                values = np.array(getter(), dtype=np.float32)
                values[0, mask] = group[key]
                setter(values, self.idx)
            log(f"  drives {group['joints']!r}: {int(mask.sum())} joints")
        for j in np.flatnonzero(~covered):
            if self.joints[j] not in self.mimic_joints:
                log(f"  WARNING: joint {self.joints[j]} has no drive group, keeping USD gains")
        if self.mimic_joints:
            # a drive on a mimic follower would fight the mimic constraint
            mask = np.array([j in self.mimic_joints for j in self.joints])
            for getter, setter in (setters["stiffness"], setters["damping"]):
                values = np.array(getter(), dtype=np.float32)
                values[0, mask] = 0.0
                setter(values, self.idx)
            log(f"  {len(self.mimic_joints)} mimic joints (follow their leader joint, no drive)")

    def reset(self):
        """Initial joint positions, zero velocities and commands, initial root pose (floating base)."""
        zeros = np.zeros((1, self.n), dtype=np.float32)
        q = self.init_q[None].astype(np.float32)
        self.view.set_dof_positions(q, self.idx)
        self.view.set_dof_velocities(zeros, self.idx)
        self.view.set_dof_position_targets(q, self.idx)
        self.view.set_dof_velocity_targets(zeros, self.idx)
        self.view.set_dof_actuation_forces(zeros, self.idx)
        if not self.fixed_base:
            self.view.set_root_transforms(self.init_root, self.idx)
            self.view.set_root_velocities(np.zeros((1, 6), dtype=np.float32), self.idx)
        self.prev = (self.init_q.copy(), np.zeros(self.n), np.zeros(self.n))
        for imu in self.imus:
            imu.reset()
        for fs in self.force_sensors:
            fs.values[:] = 0.0

    def update_sensors(self, sim_time):
        """Force sensors and IMUs after the physics substeps of a control step."""
        if self.force_sensors:
            wrenches = np.asarray(self.view.get_link_incoming_joint_force(), dtype=np.float64).reshape(-1, 6)
            for fs in self.force_sensors:
                fs.update(wrenches)
        if not self.imus:
            return
        transforms = np.asarray(self.view.get_link_transforms(), dtype=np.float64).reshape(self.view.max_links, 7)
        velocities = np.asarray(self.view.get_link_velocities(), dtype=np.float64).reshape(self.view.max_links, 6)
        for imu in self.imus:
            imu.update(transforms, velocities, self.coms, sim_time)

    def apply_command(self, ratio, cmd):
        # linear interpolation between the previous and the new mc_rtc command over the physics substeps
        q, alpha, tau = (p + ratio * (c - p) for p, c in zip(self.prev, cmd))
        self.view.set_dof_position_targets(q[None].astype(np.float32), self.idx)
        self.view.set_dof_velocity_targets(alpha[None].astype(np.float32), self.idx)
        self.view.set_dof_actuation_forces(tau[None].astype(np.float32), self.idx)

    def read_state(self):
        """Bridge state of this robot (layout in Scene.describe)."""
        q = self.view.get_dof_positions()[0]
        alpha = self.view.get_dof_velocities()[0]
        try:
            tau = self.view.get_dof_projected_joint_forces()[0]
        except Exception:
            tau = np.zeros(self.n)
        tr = self.view.get_root_transforms()[0]  # x y z qx qy qz qw
        pose = [tr[0], tr[1], tr[2], tr[6], tr[3], tr[4], tr[5]]
        vel = np.array(self.view.get_root_velocities()[0], dtype=np.float64)  # world linear (CoM), world angular
        vel[0:3] = origin_velocity(vel[0:3], vel[3:6], quat_xyzw_to_matrix(tr[3:7]), self.coms[0][:3])
        imus = [imu.values for imu in self.imus]
        forces = [fs.values for fs in self.force_sensors]
        return np.concatenate([q, alpha, tau, pose, vel, *imus, *forces]).astype(np.float64)

    def describe(self):
        """hello reply entry: Isaac joint order, base type, initial posture, sensor names."""
        return {"name": self.name, "joints": self.joints, "fixed_base": self.fixed_base, "init_q": self.init_q.tolist(),
                "imus": [imu.name for imu in self.imus], "force_sensors": [fs.name for fs in self.force_sensors]}


class RigidObjectHandle:
    """Robot without joints (manipulated object, environment): one rigid body, same bridge layout with n = 0."""

    def __init__(self, stage, spec, cache, log):
        from pxr import PhysxSchema, Usd, UsdPhysics

        self.spec = spec
        self.name = spec["name"]
        self.fixed_base = bool(spec.get("fixed", False))
        robot = _reference_model(stage, spec, cache, log)
        # assets exported as articulations (single link) must be plain rigid bodies here
        for prim in Usd.PrimRange(robot):
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
                prim.RemoveAPI(PhysxSchema.PhysxArticulationAPI)
        body = _find_prim(robot, lambda p: p.HasAPI(UsdPhysics.RigidBodyAPI))
        if body is None:
            body = stage.GetPrimAtPath(f"/World/{self.name}/model")
            UsdPhysics.RigidBodyAPI.Apply(body)
        # fixed objects (tables, holders) are kinematic: they collide but never move
        UsdPhysics.RigidBodyAPI(body).CreateKinematicEnabledAttr(self.fixed_base)
        _apply_mass(body, spec, log)
        self.body_path = str(body.GetPath())
        self.joints = []
        self.n = 0
        self.init_q = np.zeros(0)
        self.view = None

    def attach(self, sim_view, log):
        """After physics start: rigid body view, initial pose, center of mass."""
        self.view = sim_view.create_rigid_body_view(self.body_path)
        if self.view is None or self.view.count != 1:
            raise RuntimeError(f"cannot create rigid body view for {self.body_path}")
        self.idx = np.array([0], dtype=np.int32)
        self.init_pose = self.view.get_transforms().copy()
        try:
            self.com = np.array(self.view.get_coms(), dtype=np.float64).reshape(-1)[:3]
        except Exception as e:
            log(f"  WARNING: cannot read the center of mass of {self.name} ({e}), velocity given at the center of mass")
            self.com = np.zeros(3)
        log(f"  {self.name}: rigid object, {'kinematic' if self.fixed_base else 'dynamic'}")

    def reset(self):
        """Initial pose (and zero velocity for dynamic objects)."""
        self.view.set_transforms(self.init_pose, self.idx)
        if not self.fixed_base:  # PhysX refuses velocities on kinematic bodies
            self.view.set_velocities(np.zeros((1, 6), dtype=np.float32), self.idx)
        self.prev = (np.zeros(0), np.zeros(0), np.zeros(0))

    def update_sensors(self, sim_time):
        pass

    def apply_command(self, ratio, cmd):
        pass

    def read_state(self):
        """Bridge state of an object: root pose and velocity (n = 0 joints)."""
        tr = self.view.get_transforms()[0]  # x y z qx qy qz qw
        pose = [tr[0], tr[1], tr[2], tr[6], tr[3], tr[4], tr[5]]
        vel = np.array(self.view.get_velocities()[0], dtype=np.float64)  # world linear (center of mass), world angular
        vel[0:3] = origin_velocity(vel[0:3], vel[3:6], quat_xyzw_to_matrix(tr[3:7]), self.com)
        return np.concatenate([pose, vel]).astype(np.float64)

    def describe(self):
        return {"name": self.name, "joints": [], "fixed_base": self.fixed_base, "init_q": [], "imus": []}


class Scene:
    """Stage built from a scene spec (POST /scene): physics scene, lights, ground, robots, camera.

    Physics is stepped manually (``step``), never by the Kit timeline: the timeline only mirrors the controller
    run/pause state so that Isaac's Play/Pause buttons can drive the mc_rtc ticker.
    """

    def __init__(self, state, spec):
        import carb
        import omni.physics.tensors as tensors
        import omni.physx
        import omni.timeline
        import omni.usd
        from pxr import Gf, PhysxSchema, UsdGeom, UsdLux, UsdPhysics

        self.state = state
        self.app = state.app
        log = state.logs.add
        self.spec = spec
        self.spec_hash = spec_hash(spec)
        self.physics_dt = float(spec.get("physics_dt", 0.001))
        if not 0.0 < self.physics_dt <= 0.1:
            raise ValueError(f"invalid physics_dt {self.physics_dt}")
        self.sim_time = 0.0
        self.robots = []
        self.timeline = omni.timeline.get_timeline_interface()

        self.timeline.stop()
        self.app.update()
        # physics is only stepped by mc_isaac, app.update() just renders
        carb.settings.get_settings().set_bool("/app/player/playSimulations", False)
        # PhysX debug display of the collision shapes: 2 = all, 0 = none
        carb.settings.get_settings().set_int("/persistent/physics/visualizationDisplayColliders",
                                             2 if spec.get("show_collisions", False) else 0)
        context = omni.usd.get_context()
        context.new_stage()
        self.app.update()
        stage = context.get_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, "/World").GetPrim())

        physics_scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
        physics_scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
        physics_scene.CreateGravityMagnitudeAttr(9.81)
        physx_scene = PhysxSchema.PhysxSceneAPI.Apply(physics_scene.GetPrim())
        physx_scene.CreateTimeStepsPerSecondAttr(int(round(1.0 / self.physics_dt)))
        physx_scene.CreateEnableGPUDynamicsAttr(False)
        physx_scene.CreateBroadphaseTypeAttr("MBP")
        physx_scene.CreateSolverTypeAttr("TGS")

        UsdLux.DomeLight.Define(stage, "/World/domeLight").CreateIntensityAttr(1000.0)
        sun = UsdLux.DistantLight.Define(stage, "/World/sunLight")
        sun.CreateIntensityAttr(2000.0)
        sun.AddRotateXYZOp().Set(Gf.Vec3f(45.0, 0.0, 30.0))
        if spec.get("ground", True):
            ground = UsdGeom.Cube.Define(stage, "/World/ground")
            ground.CreateSizeAttr(1.0)
            ground.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.05))
            ground.AddScaleOp().Set(Gf.Vec3f(100.0, 100.0, 0.1))
            ground.CreateDisplayColorAttr([Gf.Vec3f(0.45, 0.45, 0.45)])
            UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

        for robot_spec in spec.get("robots", []):
            handle = RigidObjectHandle if robot_spec.get("rigid", False) else RobotHandle
            self.robots.append(handle(stage, robot_spec, state.cache, log))
        if not self.robots:
            raise ValueError("scene has no robot")
        self.app.update()

        physx = omni.physx.get_physx_interface()
        physx.force_load_physics_from_usd()
        physx.start_simulation()
        self.timeline.play()
        self.expected_playing = True
        self.app.update()
        self.physx_sim = omni.physx.get_physx_simulation_interface()
        self.sim_view = tensors.create_simulation_view("numpy")
        self.sim_view.set_subspace_roots("/")
        for robot in self.robots:
            robot.attach(self.sim_view, log)
        self.reset()
        self.camera_path = self._setup_camera(stage, spec.get("camera") or {}, log)
        state.render()

    def _setup_camera(self, stage, camera, log):
        """Fixed camera of the scene spec: used by the MJPEG stream, and by the viewport when headless/streamed."""
        from pxr import Gf, UsdGeom

        path = "/World/mc_isaac_camera"
        position = Gf.Vec3d(*camera.get("position", [2.0, -2.0, 1.5]))
        target = Gf.Vec3d(*camera.get("target", [0.0, 0.0, 0.8]))
        prim = UsdGeom.Camera.Define(stage, path)
        prim.CreateFocalLengthAttr(float(camera.get("focal_length", 18.0)))
        prim.CreateClippingRangeAttr(Gf.Vec2f(0.01, 1000.0))
        # look-at gives world -> camera; USD cameras look along -Z with +Y up, like OpenGL
        view = Gf.Matrix4d().SetLookAt(position, target, Gf.Vec3d(0.0, 0.0, 1.0))
        UsdGeom.Xformable(prim).AddTransformOp().Set(view.GetInverse())
        args = self.state.args
        if camera.get("viewport", args.headless or args.stream == "webrtc"):
            try:
                from omni.kit.viewport.utility import get_active_viewport

                viewport = get_active_viewport()
                if viewport is not None:
                    viewport.camera_path = path
            except Exception as e:
                log(f"WARNING: cannot use the scene camera in the viewport: {e}")
        if self.state.streamer is not None:
            self.app.update()
            self.state.streamer.attach(path)
        log(f"  camera at {tuple(position)} looking at {tuple(target)}")
        return path

    def describe(self):
        """hello reply: robots and the command/state layouts."""
        return {
            "physics_dt": self.physics_dt,
            "sim_time": self.sim_time,
            "spec_hash": self.spec_hash,
            "robots": [r.describe() for r in self.robots],
            "command_layout": "per robot: q_ref[n], alpha_ref[n], tau_ref[n]",
            "state_layout": "per robot: q[n], alpha[n], tau[n], root_pose[x y z qw qx qy qz], root_vel[world lin (origin), "
                            "world ang], per IMU: ori[qw qx qy qz world], gyro[3 sensor], acc[3 sensor, proper], "
                            "per force sensor: force[3 sensor], torque[3 sensor]",
        }

    def reset(self):
        """All robots back to their initial state, simulation time 0."""
        for robot in self.robots:
            robot.reset()
        self.sim_time = 0.0

    def step(self, header, payload):
        """One mc_rtc control step: apply the commands, run n_substeps physics steps, return the new state."""
        n_substeps = int(header.get("n_substeps", 1))
        if not 1 <= n_substeps <= 10000:
            raise ValueError(f"invalid n_substeps {n_substeps}")
        if not getattr(self.sim_view, "is_valid", True):
            raise RuntimeError("physics was reset from Isaac (Stop button?), reload the scene")
        data = np.frombuffer(payload, dtype="<f8")
        expected = sum(3 * r.n for r in self.robots)
        if data.size != expected:
            raise ValueError(f"step payload has {data.size} values, expected {expected}")
        commands, offset = [], 0
        for robot in self.robots:
            commands.append(tuple(data[offset + k * robot.n: offset + (k + 1) * robot.n] for k in range(3)))
            offset += 3 * robot.n
        interpolate = header.get("interpolate", True)
        t0 = time.perf_counter()
        apply_s = simulate_s = 0.0
        for i in range(n_substeps):
            ratio = (i + 1) / n_substeps if interpolate else 1.0
            ta = time.perf_counter()
            for robot, cmd in zip(self.robots, commands):
                robot.apply_command(ratio, cmd)
            tb = time.perf_counter()
            self.physx_sim.simulate(self.physics_dt, self.sim_time)
            self.physx_sim.fetch_results()
            simulate_s += time.perf_counter() - tb
            apply_s += tb - ta
            self.sim_time += self.physics_dt
        t1 = time.perf_counter()
        for robot, cmd in zip(self.robots, commands):
            robot.prev = tuple(c.copy() for c in cmd)
            robot.update_sensors(self.sim_time)
        out = self.read_state()
        t2 = time.perf_counter()
        physics_ms = 1e3 * (t2 - t0)
        self.state.physics_ms = 0.9 * self.state.physics_ms + 0.1 * physics_ms
        self.state.last_step = time.perf_counter()
        if header.get("render", False):
            self.state.render()
        timings = {"apply": 1e3 * apply_s, "simulate": 1e3 * simulate_s, "state": 1e3 * (t2 - t1)}
        return {"ok": True, "sim_time": self.sim_time, "physics_ms": physics_ms, "timings": timings}, out

    def read_state(self):
        """Concatenated state of all robots, float64 little-endian bytes."""
        return np.concatenate([robot.read_state() for robot in self.robots]).astype("<f8").tobytes()

    def invalid(self):
        """True when Isaac's Stop button tore down the physics (the scene must be rebuilt)."""
        return self.timeline.is_stopped() or not getattr(self.sim_view, "is_valid", True)

    def sync_timeline(self, running):
        """Returns "play"/"pause" when the user pressed an Isaac timeline button, else applies the client state."""
        playing = self.timeline.is_playing()
        if playing != self.expected_playing:
            self.expected_playing = playing
            return "play" if playing else "pause"
        if running is not None and bool(running) != playing:
            if running:
                self.timeline.play()
            else:
                self.timeline.pause()
            # play()/pause() are only applied at the next app update otherwise: back-to-back steps would see the
            # old state and report it as an Isaac button press
            self.timeline.commit()
            self.expected_playing = bool(running)
        return None

    def close(self):
        """Stop the timeline and release the views (before a new stage is created)."""
        if self.state.streamer is not None:
            self.state.streamer.detach()
        self.timeline.stop()
        self.sim_view = None
        self.robots = []
        self.app.update()


# ------------------------------------------------------------------------------------------ panel


class MarkerView:
    """mc_rtc 3D GUI elements in the Isaac viewport (main thread only).

    Lines/points use Isaac's debug draw. Editable elements get a small sphere prim under /World/mc_isaac_markers that
    the user moves with the Isaac transform gizmo; its new pose is returned as a GUI request for the plugin.
    """

    ROOT = "/World/mc_isaac_markers"
    HOLD = 0.5  # s during which mc_rtc values do not overwrite a handle the user just moved

    def __init__(self, log):
        self.log = log
        self.draw = None
        self.handles = {}  # key -> {path, written pose, hold until}
        self.stage_id = None

    def _debug_draw(self):
        if self.draw is None:
            from isaacsim.core.utils.extensions import enable_extension

            enable_extension("isaacsim.util.debug_draw")
            from isaacsim.util.debug_draw import _debug_draw

            self.draw = _debug_draw.acquire_debug_draw_interface()
        return self.draw

    def clear(self):
        """Remove all lines, points and handles (controller disconnected)."""
        if self.draw is not None:
            self.draw.clear_lines()
            self.draw.clear_points()
        self._remove_handles(list(self.handles))

    def update(self, markers, scene):
        """Draw the markers of a bridge request; returns the GUI requests of the handles the user
        moved with the gizmo ([{id, pose}]).
        """
        draw = self._debug_draw()
        draw.clear_lines()
        draw.clear_points()
        if scene is None:
            self.handles = {}
            return []
        lines = np.asarray(markers.get("lines", []), dtype=np.float64).reshape(-1, 11)
        if len(lines):
            draw.draw_lines([tuple(p) for p in lines[:, 0:3]], [tuple(p) for p in lines[:, 3:6]],
                            [tuple(c) for c in lines[:, 6:10]], lines[:, 10].tolist())
        points = np.asarray(markers.get("points", []), dtype=np.float64).reshape(-1, 8)
        if len(points):
            draw.draw_points([tuple(p) for p in points[:, 0:3]], [tuple(c) for c in points[:, 3:7]],
                             points[:, 7].tolist())
        return self._update_handles(markers.get("handles", []))

    def _update_handles(self, handles):
        """Create/move/remove the handle spheres and detect the ones moved by the user."""
        import omni.usd
        from pxr import Gf, UsdGeom

        stage = omni.usd.get_context().get_stage()
        if id(stage) != self.stage_id:
            # new stage (scene reload/clear): the handle prims are gone
            self.stage_id, self.handles = id(stage), {}
        wanted = {h["id"]: h for h in handles}
        self._remove_handles([k for k in self.handles if k not in wanted])
        requests, now = [], time.perf_counter()
        cache = UsdGeom.XformCache()
        for key, h in wanted.items():
            pose = np.asarray(h["pose"], dtype=np.float64)
            pose[3:] /= np.linalg.norm(pose[3:]) or 1.0
            state = self.handles.get(key)
            prim = stage.GetPrimAtPath(state["path"]) if state else None
            if prim is None or not prim.IsValid():
                state = self._create_handle(stage, key, h["kind"])
                prim = stage.GetPrimAtPath(state["path"])
            else:
                m = cache.GetLocalToWorldTransform(prim)
                t, q = m.ExtractTranslation(), m.ExtractRotationQuat()
                current = np.array([t[0], t[1], t[2], q.GetReal(), *q.GetImaginary()])
                if state["written"] is not None and not _same_pose(current, state["written"]):
                    # moved with the Isaac gizmo: forward to mc_rtc, keep the user pose for a moment
                    requests.append({"id": key, "pose": current.tolist()})
                    state["written"], state["hold"] = current, now + self.HOLD
                    continue
            if now < state["hold"] or (state["written"] is not None and _same_pose(pose, state["written"])):
                continue
            xform = UsdGeom.Xformable(prim)
            ops = xform.GetOrderedXformOps()
            ops[0].Set(Gf.Vec3d(*pose[0:3]))
            ops[1].Set(Gf.Quatd(float(pose[3]), Gf.Vec3d(*pose[4:7])))
            state["written"] = pose
        return requests

    def _create_handle(self, stage, key, kind):
        """Sphere prim of an editable mc_rtc element (orange point, blue frame)."""
        from pxr import Gf, UsdGeom

        if not stage.GetPrimAtPath(self.ROOT).IsValid():
            UsdGeom.Xform.Define(stage, self.ROOT)
        path = f"{self.ROOT}/h_{re.sub(r'[^A-Za-z0-9_]', '_', key)}"
        sphere = UsdGeom.Sphere.Define(stage, path)
        sphere.CreateRadiusAttr(0.02 if kind == "point" else 0.03)
        sphere.CreateDisplayColorAttr([Gf.Vec3f(1.0, 0.55, 0.0) if kind == "point" else Gf.Vec3f(0.2, 0.6, 1.0)])
        sphere.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
        sphere.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
        state = {"path": path, "written": None, "hold": 0.0}
        self.handles[key] = state
        return state

    def _remove_handles(self, keys):
        """Delete the handle prims of the given keys."""
        try:
            import omni.usd

            stage = omni.usd.get_context().get_stage()
        except Exception:
            stage = None
        for key in keys:
            state = self.handles.pop(key, None)
            if state and stage is not None and id(stage) == self.stage_id:
                stage.RemovePrim(state["path"])


def _same_pose(a, b, eps=1e-5):
    """True if two poses [x y z qw qx qy qz] are equal up to eps."""
    a, b = np.asarray(a), np.asarray(b)
    # q and -q are the same rotation
    return np.allclose(a[:3], b[:3], atol=eps) and min(np.abs(a[3:] - b[3:]).max(), np.abs(a[3:] + b[3:]).max()) < eps


class MjpegStreamer:
    """View-only browser stream of the scene camera (render product + rgb annotator, JPEG encoded off the main thread)."""

    def __init__(self, fps, size, log):
        width, height = (int(v) for v in size.lower().split("x"))
        self.size = (width, height)
        self.period = 1.0 / fps if fps > 0 else 0.0
        self.log = log
        self.render_product = None
        self.annotator = None
        self.last_capture = 0.0
        self.raw = None
        self.raw_ready = threading.Event()
        self.frame_cond = threading.Condition()
        self.jpeg = None
        self.seq = 0
        threading.Thread(target=self._encode_loop, daemon=True).start()

    # ---- main thread

    def attach(self, camera_path):
        """Stream the given camera (render product + rgb annotator)."""
        import omni.replicator.core as rep

        self.detach()
        self.render_product = rep.create.render_product(camera_path, self.size)
        self.annotator = rep.AnnotatorRegistry.get_annotator("rgb")
        self.annotator.attach([self.render_product])
        self.log(f"MJPEG stream of {camera_path} ({self.size[0]}x{self.size[1]})")

    def detach(self):
        """Release the render product (scene closed)."""
        try:
            if self.annotator is not None:
                self.annotator.detach()
            if self.render_product is not None:
                self.render_product.destroy()
        except Exception as e:
            self.log(f"WARNING: stream detach: {e}")
        self.annotator = self.render_product = None

    def capture(self):
        """After a render: hand the last frame to the encoder thread (at most --stream-fps)."""
        now = time.perf_counter()
        if self.annotator is None or now - self.last_capture < self.period:
            return
        try:
            data = self.annotator.get_data()
        except Exception:
            # no frame rendered yet for a new render product
            return
        if data is None or getattr(data, "size", 0) == 0:
            return
        self.last_capture = now
        self.raw = np.ascontiguousarray(np.asarray(data)[..., :3])
        self.raw_ready.set()

    # ---- encoder thread / HTTP threads

    def _encode_loop(self):
        """Encoder thread: JPEG-encode the captured frames and wake the HTTP clients."""
        from PIL import Image

        while True:
            self.raw_ready.wait()
            self.raw_ready.clear()
            buffer = io.BytesIO()
            Image.fromarray(self.raw).save(buffer, "JPEG", quality=80)
            with self.frame_cond:
                self.jpeg = buffer.getvalue()
                self.seq += 1
                self.frame_cond.notify_all()

    def next_frame(self, last_seq, timeout=5.0):
        """Waits for a frame newer than last_seq; returns (seq, jpeg), jpeg None on timeout."""
        with self.frame_cond:
            self.frame_cond.wait_for(lambda: self.seq != last_seq, timeout=timeout)
            if self.seq == last_seq:
                return last_seq, None
            return self.seq, self.jpeg


STREAM_PAGE = """<!DOCTYPE html><html><head><title>mc_isaac</title>
<style>body{margin:0;background:#202020;color:#ccc;font-family:sans-serif}img{width:100%;height:auto;display:block}
p{margin:4px 8px;font-size:13px}</style></head><body>
<img src="/stream" alt="waiting for the first frame (a scene must be loaded)">
<p>mc_isaac_server @VERSION@ &mdash; view-only stream of the scene camera</p></body></html>"""


class IsaacPanel:
    """mc_isaac window in the Isaac Sim UI: server/scene status, mc_rtc ticker controls, scene actions.

    Ticker controls are sent to the plugin with the next bridge reply; the plugin presses the matching
    elements of the mc_rtc_ticker "Ticker" GUI tab, so mc_rtc stays the single source of truth.
    """

    STEP_BUTTONS = (1, 5, 10, 50, 100)  # same as the mc_rtc_ticker "+N ms" buttons (in controller steps)
    UPDATE_PERIOD = 0.2

    def __init__(self, state):
        import omni.ui as ui

        self.state = state
        self.last_update = 0.0
        self.ratio_sample = None
        self.ratio = None
        self.window = ui.Window("mc_isaac", width=400, height=560)
        header = {"font_size": 16, "color": 0xFF76B900}
        with self.window.frame:
            with ui.ScrollingFrame():
                with ui.VStack(spacing=4, height=0):
                    ui.Label("Isaac server", style=header, height=22)
                    self.l_server = ui.Label("", word_wrap=True)
                    self.l_scene = ui.Label("", word_wrap=True)
                    self.l_perf = ui.Label("")
                    ui.Spacer(height=6)
                    ui.Label("mc_rtc controller", style=header, height=22)
                    self.l_client = ui.Label("", word_wrap=True)
                    self.l_ticker = ui.Label("", word_wrap=True)
                    self.controls = []
                    with ui.HStack(height=26, spacing=4):
                        self.b_pause = self._button("Pause", {"cmd": "toggle_pause"})
                        self._button("Reset", {"cmd": "reset"})
                        self._button("Stop mc_rtc", {"cmd": "stop"})
                    with ui.HStack(height=26, spacing=4):
                        ui.Label("Step", width=40)
                        self.b_steps = [self._button(f"+{n}", {"cmd": "steps", "n": n}, paused_only=True)
                                        for n in self.STEP_BUTTONS]
                    with ui.HStack(height=26, spacing=4):
                        ui.Label("Step", width=40)
                        self.f_ms = ui.FloatField(width=70)
                        self.f_ms.model.set_value(20.0)
                        ui.Label("ms", width=24)
                        self.controls.append((self.f_ms, True))
                        self._button("Go", lambda: {"cmd": "step_ms", "ms": self.f_ms.model.get_value_as_float()},
                                     paused_only=True)
                    with ui.HStack(height=26, spacing=4):
                        self.b_sync = self._button("Real time: ?", {"cmd": "toggle_sync"})
                        self.f_ratio = ui.FloatField(width=60)
                        self.f_ratio.model.set_value(1.0)
                        self.controls.append((self.f_ratio, False))
                        self._button("Set ratio", lambda: {"cmd": "ratio", "value": self.f_ratio.model.get_value_as_float()})
                        self._button("x2", {"cmd": "ratio_x2"})
                        self._button("/2", {"cmd": "ratio_div2"})
                    with ui.HStack(height=26, spacing=4):
                        self.b_markers = self._button("mc_rtc markers: ?", {"cmd": "toggle_markers"})
                    ui.Spacer(height=6)
                    ui.Label("Scene", style=header, height=22)
                    with ui.HStack(height=26, spacing=4):
                        ui.Button("Reload scene", clicked_fn=self._reload)
                        self.b_reset_scene = ui.Button("Reset scene", clicked_fn=lambda: self._action(
                            "reset scene", self.state.reset_scene))
                        self.b_clear = ui.Button("Clear environment", clicked_fn=lambda: self._action(
                            "clear environment", self.state.clear_scene))
                    ui.Label("Clear / Reset scene only without controller (use the mc_rtc controls otherwise)",
                             word_wrap=True, style={"color": 0xFF909090})
        try:
            self.window.deferred_dock_in("Property")
        except Exception:
            pass
        self.update(force=True)

    def _button(self, text, command, paused_only=False):
        """Ticker control button: queues the command for the plugin (sent with the next reply)."""
        import omni.ui as ui

        def clicked():
            cmd = command() if callable(command) else dict(command)
            if self.state.ticker is not None:
                self.state.client_commands.append(cmd)

        button = ui.Button(text, clicked_fn=clicked)
        self.controls.append((button, paused_only))
        return button

    def _action(self, name, fn):
        """Queue a scene action run by the main loop."""
        self.state.panel_actions.append((name, fn))

    def _reload(self):
        """Reload through the controller when connected (it resets), else directly."""
        if self.state.ticker is not None:
            self.state.client_commands.append({"cmd": "reload"})
        else:
            self._action("reload scene", self.state.reload_last_scene)

    def update(self, force=False):
        """Refresh the labels and button states (main loop, every UPDATE_PERIOD)."""
        now = time.perf_counter()
        if not force and now - self.last_update < self.UPDATE_PERIOD:
            return
        self.last_update = now
        state, scene, ticker = self.state, self.state.scene, self.state.ticker
        self.l_server.text = (f"mc_isaac_server {SERVER_VERSION}, Isaac Sim {state.isaac_version}, "
                              f"http {state.args.http_port} / bridge {state.args.bridge_port}")
        if scene is None:
            self.l_scene.text = "No scene" + (" (Reload scene restores the last one)" if state.last_spec else "")
            self.ratio_sample = self.ratio = None
        else:
            self.l_scene.text = (f"{len(scene.robots)} robot(s): {', '.join(r.name for r in scene.robots)}\n"
                                 f"sim time {scene.sim_time:.3f} s, physics dt {scene.physics_dt * 1e3:.2f} ms")
            # sim / real time ratio over ~1 s windows
            if self.ratio_sample is None or now - self.ratio_sample[0] >= 1.0:
                if self.ratio_sample is not None:
                    self.ratio = (scene.sim_time - self.ratio_sample[1]) / (now - self.ratio_sample[0])
                self.ratio_sample = (now, scene.sim_time)
        self.l_perf.text = f"physics {state.physics_ms:.1f} ms per control step, render {state.render_ms:.1f} ms" + (
            f"\nsim/real ratio {self.ratio:.2f}" if self.ratio is not None else "") + (
            f" (target {float(ticker.get('target_ratio', 1.0)):.3g}, real time {'ON' if ticker.get('sync', True) else 'OFF'})"
            if ticker is not None else "")
        connected = state.bridge_client is not None
        self.l_client.text = f"connected ({state.bridge_client})" if connected else "not connected"
        has_ticker = ticker is not None
        paused = has_ticker and bool(ticker.get("step_by_step", False))
        if has_ticker:
            dt = float(ticker.get("dt", 0.005))
            self.l_ticker.text = (f"{'paused (step by step)' if paused else 'running'}, controller dt {dt * 1e3:.2f} ms, "
                                  f"target ratio {float(ticker.get('target_ratio', 1.0)):.3g}")
            for button, n in zip(self.b_steps, self.STEP_BUTTONS):
                button.text = f"+{int(np.ceil(n * 1000 * dt - 1e-9))}ms"
            self.b_pause.text = "Resume" if paused else "Pause"
            self.b_sync.text = f"Real time: {'ON' if ticker.get('sync', True) else 'OFF'}"
            self.b_markers.text = f"mc_rtc markers: {'ON' if ticker.get('markers', False) else 'OFF'}"
        else:
            self.l_ticker.text = "" if not connected else "ticker state not reported by the client"
        for widget, paused_only in self.controls:
            widget.enabled = has_ticker and (paused or not paused_only)
        self.b_clear.enabled = not connected and scene is not None
        self.b_reset_scene.enabled = not connected and scene is not None


# ------------------------------------------------------------------------------------------ mc_rtc GUI mirror
#
# gui: full (plugin option). The plugin (src/GuiMirror.cpp) sends with the bridge requests:
#   tree: {n, w: [widgets], c: [sub-categories]}, widget = {k: key, n: name, t: type, s: stack id, ...type data}
#   data: mc_rtc GUI data store (robots, bodies, surfaces...) used by the data combo inputs
#   schemas: {schema dir: {title: resolved JSON schema}} for the Schema elements
#   plots: [{id, title, axes, series, points: [did x y ...], polygons}] new points since the last send
# Every top-level category is a window docked next to Property, nested categories are tab buttons, plots go to the
# "mc_rtc plots" window. User actions become requests {key, data} sent with the next bridge reply, the plugin
# applies them like mc-rtc-magnum does. Widgets follow mc-rtc-magnum (mc_rtc-imgui): inputs are read-only until
# "Edit" is pressed, forms keep the user's values until sent. All of this runs on the main thread.

NO_DATA = object()
GUI_HEADER_STYLE = {"font_size": 16, "color": 0xFF76B900}
GUI_DIM_STYLE = {"color": 0xFF909090}
GUI_ERROR_STYLE = {"color": 0xFF5050FF}
GUI_TAB_STYLE = {"Button": {"background_color": 0xFF3A3A3A}}
GUI_TAB_SELECTED_STYLE = {"Button": {"background_color": 0xFF2E6B1F}}


def _abgr(rgba):
    """mc_rtc color [r, g, b, a] (0-1) -> omni.ui color 0xAABBGGRR."""
    r, g, b, a = (int(max(0.0, min(1.0, float(c))) * 255) for c in (list(rgba or []) + [1.0] * 4)[:4])
    return (a << 24) | (b << 16) | (g << 8) | r


def _fmt(value):
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return f"{float(value):.4f}"
    return str(value)


def _edit_text(value):
    return repr(float(value)) if isinstance(value, float) else str(value)


def _resolve_data(data, ref, lookup=None):
    """Values of a data combo: walk the GUI data store along ref ("$name" = value of the form field name)."""
    out = data
    for i, key in enumerate(ref or []):
        if isinstance(key, str) and key.startswith("$") and lookup is not None:
            key = lookup(key[1:])
            if not key:
                return [], f"fill {ref[i][1:]} first"
        if not isinstance(out, dict) or key not in out:
            return [], f"no {'/'.join(str(r) for r in ref[:i + 1])} in the GUI data"
        out = out[key]
    if isinstance(out, dict):
        return [str(k) for k in out], None
    if isinstance(out, list):
        return [str(v) for v in out], None
    return [], None


class GuiMirror:
    """Mirror of the whole mc_rtc GUI in the Isaac window (plugin option gui: full), like mc-rtc-magnum."""

    REFRESH_PERIOD = 0.1
    PLOT_PERIOD = 0.2

    def __init__(self, state):
        self.state = state
        self.tree = None
        self.data = {}
        self.schemas = {}
        # until the plugin sent everything once (tree, data store, schemas): reply gui_resync
        self.need_full = True
        self.dirty = False
        self.requests = []
        self.windows = {}
        # selected sub-category per category path, kept across rebuilds
        self.selected = {}
        self.plots = {}
        self.plots_window = None
        self.last_refresh = self.last_plots = 0.0
        state.logs.add("mc_rtc GUI mirrored in the Isaac window (gui: full)")

    def receive(self, payload):
        """Bridge request "gui" payload: store the tree / data / schemas, accumulate plot points."""
        if payload.get("full"):
            self.need_full = False
        if "tree" in payload:
            self.tree = payload["tree"]
            self.dirty = True
        if "data" in payload:
            self.data = payload["data"] if isinstance(payload["data"], dict) else {}
            self.dirty = True
        if payload.get("schemas"):
            self.schemas.update(payload["schemas"])
            self.dirty = True
        for plot in payload.get("plots") or []:
            self.plots.setdefault(plot["id"], PlotState(plot["id"])).update(plot)
        if "plots_active" in payload:
            active = set(payload["plots_active"] or [])
            for pid, plot in self.plots.items():
                plot.active = pid in active

    def request(self, key, data=NO_DATA):
        """Queue a user action on element key (sent with the next bridge reply)."""
        entry = {"key": key}
        if data is not NO_DATA:
            entry["data"] = data
        self.requests.append(entry)

    def refresh(self):
        """Main loop: apply the last GUI state to the windows (REFRESH_PERIOD) and the plots."""
        now = time.perf_counter()
        if self.dirty and self.tree is not None and now - self.last_refresh >= self.REFRESH_PERIOD:
            self.last_refresh = now
            self.dirty = False
            self._update_windows()
        if now - self.last_plots >= self.PLOT_PERIOD:
            self.last_plots = now
            if self.plots and self.plots_window is None:
                self.plots_window = PlotsWindow(self)
            elif not self.plots and self.plots_window is not None:
                self.plots_window.destroy()
                self.plots_window = None
            if self.plots_window is not None:
                self.plots_window.refresh()

    def _update_windows(self):
        """One window per top-level category: create, update or destroy them."""
        wanted = {}
        if self.tree.get("w"):
            wanted[""] = {"n": "", "w": self.tree["w"], "c": []}
        for category in sorted(self.tree.get("c", []), key=lambda c: c["n"]):
            wanted[category["n"]] = category
        for name in [n for n in self.windows if n not in wanted]:
            self.windows.pop(name).destroy()
        for order, (name, desc) in enumerate(wanted.items()):
            if name in self.windows:
                self.windows[name].update(desc)
            else:
                self.windows[name] = GuiWindow(self, name, desc, order)

    def close(self):
        """Destroy all mirror windows (gui mode changed or controller disconnected)."""
        for window in self.windows.values():
            window.destroy()
        self.windows = {}
        if self.plots_window is not None:
            self.plots_window.destroy()
            self.plots_window = None


class GuiWindow:
    """One top-level mc_rtc category, docked as a tab next to Property (like the mc_isaac panel)."""

    def __init__(self, mirror, name, desc, order):
        import omni.ui as ui

        self.window = ui.Window(f"mc_rtc {name}" if name else "mc_rtc", width=440, height=600)
        self.view = CategoryView(mirror, [name] if name else [])
        with self.window.frame:
            with ui.ScrollingFrame():
                self.view.build(desc)
        try:
            self.window.dock_order = order
            self.window.deferred_dock_in("Property", ui.DockPolicy.DO_NOTHING)
        except Exception:
            pass

    def update(self, desc):
        self.view.update(desc)

    def destroy(self):
        self.window.visible = False
        self.window.destroy()


def _category_signature(desc):
    """Elements and sub-categories of a category: a change rebuilds the category view."""
    return (tuple((w.get("k"), w.get("t"), w.get("s")) for w in desc.get("w", [])),
            tuple(sorted(c["n"] for c in desc.get("c", []))))


def _stacks(widgets):
    """Consecutive widgets with the same stack id (mc_rtc ElementsStacking) are laid out on one row."""
    groups = []
    for w in widgets:
        if groups and w.get("s", -1) != -1 and groups[-1][0].get("s", -1) == w.get("s"):
            groups[-1].append(w)
        else:
            groups.append([w])
    return groups


class CategoryView:
    """Widgets of a category (rebuilt when the element list changes, values updated in place otherwise) and its
    sub-categories as tab buttons (only the selected one is built)."""

    def __init__(self, mirror, path):
        self.mirror, self.path = mirror, path
        self.desc = None
        self.signature = None
        self.widgets = {}
        self.child = None
        self.frame = None

    def build(self, desc):
        """Create the category frame (built lazily by omni.ui) for desc."""
        import omni.ui as ui

        self.desc = desc
        self.frame = ui.Frame(build_fn=self._build)

    def _build(self):
        import omni.ui as ui

        desc = self.desc
        self.signature = _category_signature(desc)
        self.widgets, self.child = {}, None
        with ui.VStack(spacing=6, height=0):
            for group in _stacks(desc.get("w", [])):
                if len(group) == 1:
                    self._add(group[0])
                    continue
                with ui.HStack(spacing=8, height=0):
                    for w in group:
                        self._add(w)
            children = sorted(desc.get("c", []), key=lambda c: c["n"])
            if not children:
                return
            names = [c["n"] for c in children]
            key = tuple(self.path)
            if self.mirror.selected.get(key) not in names:
                self.mirror.selected[key] = names[0]
            selected = self.mirror.selected[key]
            ui.Spacer(height=2)
            with ui.VGrid(column_count=min(4, len(names)), row_height=24, height=0):
                for name in names:
                    ui.Button(name, style=GUI_TAB_SELECTED_STYLE if name == selected else GUI_TAB_STYLE,
                              clicked_fn=functools.partial(self._select, name))
            self.child = CategoryView(self.mirror, self.path + [selected])
            with ui.HStack(height=0):
                ui.Spacer(width=10)
                self.child.build(next(c for c in children if c["n"] == selected))

    def _add(self, desc):
        """Create and build the view of one element."""
        view = GUI_WIDGETS.get(desc.get("t"), UnsupportedView)(self.mirror, desc)
        self.widgets[view.key] = view
        view.build()

    def _select(self, name):
        """Tab button: show another sub-category."""
        self.mirror.selected[tuple(self.path)] = name
        self.frame.rebuild()

    def update(self, desc):
        """New state of the category: rebuild if its structure changed, else update in place."""
        self.desc = desc
        if self.frame is None:
            return
        if _category_signature(desc) != self.signature:
            self.frame.rebuild()
            return
        for w in desc.get("w", []):
            view = self.widgets.get(w.get("k"))
            if view is not None:
                view.update(w)
        if self.child is not None:
            selected = self.mirror.selected.get(tuple(self.path))
            child = next((c for c in desc.get("c", []) if c["n"] == selected), None)
            if child is not None:
                self.child.update(child)


class WidgetView:
    """One mc_rtc element in its own ui.Frame: values are refreshed in place (_refresh) or the frame is rebuilt.

    After a user action, mc_rtc values are ignored for HOLD seconds (the request is applied at the next controller
    step), then the widget shows mc_rtc's value again.
    """

    HOLD = 0.5

    def __init__(self, mirror, desc):
        self.mirror = mirror
        self.desc = desc
        self.shown = None
        self.key = desc.get("k", "")
        self.name = desc.get("n", "")
        self.frame = None
        self.hold_until = 0.0
        self.setting = False

    def build(self):
        """Create the element frame (built lazily by omni.ui)."""
        import omni.ui as ui

        self.frame = ui.Frame(build_fn=self._safe_build, height=0)

    def _safe_build(self):
        import omni.ui as ui

        self.shown = self.desc
        try:
            self._build(ui)
        except Exception as e:
            message = f"{self.name}: cannot display ({type(e).__name__}: {e})"
            ui.Label(message, style=GUI_ERROR_STYLE, word_wrap=True)
            self.mirror.state.logs.add(f"WARNING: mc_rtc GUI {self.desc.get('t')} {message}")

    def busy(self):
        """True while the user edits the widget: mc_rtc updates are not applied."""
        return False

    def update(self, desc):
        """New mc_rtc state of the element (ignored while held or busy)."""
        self.desc = desc
        if self.busy() or time.perf_counter() < self.hold_until or desc == self.shown:
            return
        try:
            refreshed = self.shown is not None and self._refresh(self.shown, desc)
        except Exception:
            refreshed = False
        if refreshed:
            self.shown = desc
        else:
            self.frame.rebuild()

    def _refresh(self, old, new):
        """Update the existing widgets from new; False requests a frame rebuild."""
        return False

    def request(self, data=NO_DATA):
        """Send a user action to mc_rtc, then hold mc_rtc's values for HOLD seconds."""
        self.mirror.request(self.key, data)
        self.hold_until = time.perf_counter() + self.HOLD
        self.shown = None

    def set_model(self, model, value):
        """Programmatic model change, without triggering the user callbacks."""
        self.setting = True
        try:
            model.set_value(value)
        finally:
            self.setting = False


class UnsupportedView(WidgetView):
    def _build(self, ui):
        ui.Label(f"{self.name} ({self.desc.get('t')} not supported)", style=GUI_DIM_STYLE)


class LabelView(WidgetView):
    def _text(self, desc):
        value = desc.get("v", "")
        return f"{self.name} {value}" if value != "" else self.name

    def _build(self, ui):
        self.label = ui.Label(self._text(self.desc), word_wrap=True, height=0)

    def _refresh(self, old, new):
        self.label.text = self._text(new)
        return True


class ArrayLabelView(WidgetView):
    def _build(self, ui):
        labels, values = self.desc.get("l") or [], self.desc.get("v") or []
        self.cells = []
        if len(values) > 6 and not labels:
            # long vectors: norm, full vector as tooltip (as mc-rtc-magnum)
            with ui.HStack(height=0, spacing=6):
                ui.Label(self.name, width=ui.Percent(35))
                self.norm = ui.Label("", tooltip="")
            self._set_norm(values)
            return
        ui.Label(self.name, height=0)
        with ui.HStack(height=0, spacing=4):
            for i, value in enumerate(values):
                with ui.VStack(height=0):
                    if labels:
                        ui.Label(labels[i] if i < len(labels) else "", style=GUI_DIM_STYLE)
                    self.cells.append(ui.Label(_fmt(value)))

    def _set_norm(self, values):
        array = np.array([float(v) for v in values])
        self.norm.text = f"norm {float(np.linalg.norm(array)):.4f}"
        self.norm.tooltip = " ".join(_fmt(v) for v in values)

    def _refresh(self, old, new):
        values = new.get("v") or []
        if (old.get("l") or []) != (new.get("l") or []) or len(old.get("v") or []) != len(values):
            return False
        if self.cells:
            for cell, value in zip(self.cells, values):
                cell.text = _fmt(value)
        else:
            self._set_norm(values)
        return True


class ButtonView(WidgetView):
    def _build(self, ui):
        ui.Button(self.name, height=24, clicked_fn=lambda: self.request())

    def _refresh(self, old, new):
        return True


class CheckboxView(WidgetView):
    def _build(self, ui):
        with ui.HStack(height=0, spacing=6):
            self.box = ui.CheckBox(width=20)
            self.set_model(self.box.model, bool(self.desc.get("v")))
            self.box.model.add_value_changed_fn(self._changed)
            ui.Label(self.name)

    def _changed(self, model):
        if not self.setting:
            # mc_rtc checkboxes toggle on a request without data
            self.request()

    def _refresh(self, old, new):
        self.set_model(self.box.model, bool(new.get("v")))
        return True


class SingleInputView(WidgetView):
    """String/Integer/Number input: read-only value, Edit -> text field, Done/Enter sends (as mc-rtc-magnum)."""

    def __init__(self, mirror, desc):
        super().__init__(mirror, desc)
        self.editing = False
        self.fields = []

    def busy(self):
        return self.editing

    def _parse(self, texts):
        """Request data from the edited field texts (ValueError if invalid)."""
        return texts[0]

    def _display(self, desc):
        """Read-only text of the value."""
        return _fmt(desc.get("v", ""))

    def _values(self, desc):
        return [desc.get("v", "")]

    def _build(self, ui):
        with ui.HStack(height=0, spacing=6):
            ui.Label(self.name, width=ui.Percent(35))
            if self.editing:
                ui.Button("Done", width=50, clicked_fn=self._done)
                self._build_fields(ui)
            else:
                ui.Button("Edit", width=50, clicked_fn=self._edit)
                self.text = ui.Label(self._display(self.desc))

    def _build_fields(self, ui):
        field = ui.StringField()
        field.model.set_value(_edit_text(self.desc.get("v", "")))
        field.model.add_end_edit_fn(lambda _: self._done())
        self.fields = [field]

    def _edit(self):
        self.editing = True
        self.frame.rebuild()

    def _done(self):
        if not self.editing:
            return
        self.editing = False
        try:
            value = self._parse([f.model.as_string for f in self.fields])
        except ValueError as e:
            self.mirror.state.logs.add(f"WARNING: {self.name}: invalid value ({e})")
            self.frame.rebuild()
            return
        if value != self.desc.get("v"):
            self.request(value)
            # show the requested value until mc_rtc reports its own
            self.desc = {**self.desc, "v": value}
        self.frame.rebuild()

    def _refresh(self, old, new):
        if self.editing:
            return True
        self.text.text = self._display(new)
        return True


class IntegerInputView(SingleInputView):
    def _parse(self, texts):
        return int(texts[0].strip())


class NumberInputView(SingleInputView):
    def _parse(self, texts):
        return float(texts[0].strip())


class ArrayInputView(SingleInputView):
    def _display(self, desc):
        return "  ".join(_fmt(v) for v in desc.get("v") or [])

    def _parse(self, texts):
        return [float(t.strip()) for t in texts]

    def _build(self, ui):
        labels, values = self.desc.get("l") or [], self.desc.get("v") or []
        with ui.HStack(height=0, spacing=6):
            ui.Label(self.name, width=ui.Percent(35))
            ui.Button("Done" if self.editing else "Edit", width=50,
                      clicked_fn=self._done if self.editing else self._edit)
        self.fields, self.cells = [], []
        with ui.HStack(height=0, spacing=4):
            for i, value in enumerate(values):
                with ui.VStack(height=0):
                    if labels:
                        ui.Label(labels[i] if i < len(labels) else "", style=GUI_DIM_STYLE)
                    if self.editing:
                        field = ui.StringField()
                        field.model.set_value(_edit_text(value))
                        self.fields.append(field)
                    else:
                        self.cells.append(ui.Label(_fmt(value)))

    def _refresh(self, old, new):
        if self.editing:
            return True
        values = new.get("v") or []
        if len(values) != len(self.cells) or (old.get("l") or []) != (new.get("l") or []):
            return False
        for cell, value in zip(self.cells, values):
            cell.text = _fmt(value)
        return True


class NumberSliderView(WidgetView):
    def __init__(self, mirror, desc):
        super().__init__(mirror, desc)
        self.dragging = False

    def busy(self):
        return self.dragging

    def _build(self, ui):
        with ui.HStack(height=0, spacing=6):
            ui.Label(self.name, width=ui.Percent(35))
            self.slider = ui.FloatSlider(min=float(self.desc.get("min", 0.0)), max=float(self.desc.get("max", 1.0)))
            self.set_model(self.slider.model, float(self.desc.get("v", 0.0)))
            self.slider.model.add_value_changed_fn(self._changed)
            self.slider.model.add_begin_edit_fn(lambda _: setattr(self, "dragging", True))
            self.slider.model.add_end_edit_fn(lambda _: setattr(self, "dragging", False))

    def _changed(self, model):
        if self.setting:
            return
        # sent while dragging (as mc-rtc-magnum), the slider keeps the user's value
        self.mirror.request(self.key, model.as_float)
        self.hold_until = time.perf_counter() + self.HOLD

    def _refresh(self, old, new):
        if (old.get("min"), old.get("max")) != (new.get("min"), new.get("max")):
            return False
        self.set_model(self.slider.model, float(new.get("v", 0.0)))
        return True


class ComboInputView(WidgetView):
    def _items(self, desc):
        """(choices, message): message replaces the combo when the choices are unavailable."""
        return [str(v) for v in desc.get("o") or []], None

    def _build(self, ui):
        self.items, message = self._items(self.desc)
        current = self.desc.get("v", "")
        # an empty first entry when the current value is not one of the choices
        self.offset = 0 if current in self.items else 1
        entries = [""] * self.offset + self.items
        index = self.items.index(current) + self.offset if current in self.items else 0
        with ui.HStack(height=0, spacing=6):
            ui.Label(self.name, width=ui.Percent(35))
            if message:
                ui.Label(message, style=GUI_DIM_STYLE)
                return
            self.combo = ui.ComboBox(index, *entries)
            self.combo.model.add_item_changed_fn(self._changed)

    def _changed(self, model, item):
        if self.setting:
            return
        i = model.get_item_value_model().as_int - self.offset
        if 0 <= i < len(self.items) and self.items[i] != self.desc.get("v"):
            self.request(self.items[i])

    def _refresh(self, old, new):
        items, _ = self._items(new)
        current = new.get("v", "")
        if items != self.items or (current in items) != (self.offset == 0) or not hasattr(self, "combo"):
            return False
        index = items.index(current) + self.offset if current in items else 0
        self.set_model(self.combo.model.get_item_value_model(), index)
        return True


class DataComboInputView(ComboInputView):
    def _items(self, desc):
        return _resolve_data(self.mirror.data, desc.get("r") or [])


class TableView(WidgetView):
    def _build(self, ui):
        header, rows = self.desc.get("h") or [], self.desc.get("rows") or []
        ui.Label(self.name, height=0)
        with ui.VStack(height=0, spacing=2):
            with ui.HStack(height=0, spacing=4):
                for title in header:
                    ui.Label(str(title), style=GUI_HEADER_STYLE)
            for row in rows:
                with ui.HStack(height=0, spacing=4):
                    for cell in row:
                        ui.Label(str(cell))


# ---- forms (mc_rtc Form and Schema elements)
#
# Field descriptions come from the plugin (Form) or from a JSON schema (Schema, _schema_fields). Each field keeps
# the user's value (value None = not set, the displayed temp value is then sent for required fields only if set).
# The owner (FormView / SchemaView) provides value_of() for "$name" data combo references and rebuilds the form.


class FormField:
    def __init__(self, owner, desc):
        self.owner = owner
        self.desc = desc
        self.name = desc.get("name", "")
        self.required = bool(desc.get("required", False))
        self.locked = False

    def title(self):
        """Displayed name (* marks required fields)."""
        return self.name + (" *" if self.required else "")

    def ready(self):
        """True if the field has a value that can be sent."""
        return True

    def collect(self):
        """Value of the field in the request data."""
        return None

    def set_value(self, value):
        """Set the field from request-like data (form defaults, generic array data...)."""
        pass

    def text_value(self):
        """Value as text, for the "$name" references of data combos."""
        return ""

    def children(self):
        """Nested fields (objects, arrays, one-of)."""
        return []

    def find(self, name):
        """Field named name in this subtree, or None."""
        if self.name == name:
            return self
        for child in self.children():
            found = child.find(name)
            if found is not None:
                return found
        return None

    def missing(self, prefix=""):
        """Names of the required fields without value."""
        return [] if self.ready() else [prefix + self.name]

    def any_locked(self):
        """True if the user edited this field or a nested one."""
        return self.locked or any(c.any_locked() for c in self.children())

    def unlock(self):
        """Forget the user edits (after sending): mc_rtc defaults apply again."""
        self.locked = False
        for child in self.children():
            child.unlock()

    def edited(self):
        """User edit: keep the value across description updates, let the owner refresh."""
        self.locked = True
        self.owner.changed()

    def build(self, ui):
        """Create the omni.ui widgets of the field."""
        pass


class UnsupportedField(FormField):
    def build(self, ui):
        ui.Label(f"{self.name} ({self.desc.get('kind')} not supported)", style=GUI_DIM_STYLE)


class ConstField(FormField):
    """Schema "const" property: hidden, always sent."""

    def collect(self):
        return self.desc.get("value")

    def text_value(self):
        return str(self.desc.get("value", ""))


class SimpleField(FormField):
    ZERO = {"checkbox": False, "integer": 0, "number": 0.0, "string": ""}

    def __init__(self, owner, desc):
        super().__init__(owner, desc)
        self.kind = desc.get("kind")
        default = desc.get("default")
        self.temp = default if default is not None else self.ZERO.get(self.kind)
        self.value = self.temp if desc.get("user_default", False) else None

    def ready(self):
        return self.value is not None and (self.kind != "string" or self.value != "")

    def collect(self):
        return self.value if self.ready() else self.temp

    def set_value(self, value):
        self.value = self.temp = value

    def text_value(self):
        return "" if self.value is None else str(self.value)

    def build(self, ui):
        with ui.HStack(height=0, spacing=6):
            ui.Label(self.title(), width=ui.Percent(35))
            if self.kind == "checkbox":
                box = ui.CheckBox(width=20)
                box.model.set_value(bool(self.temp))
                box.model.add_value_changed_fn(self._checked)
            else:
                field = ui.StringField()
                field.model.set_value("" if self.value is None and self.kind == "string" else _edit_text(self.temp))
                field.model.add_end_edit_fn(self._edited)

    def _checked(self, model):
        self.set_value(model.as_bool)
        self.edited()

    def _edited(self, model):
        text = model.as_string.strip()
        try:
            value = int(text) if self.kind == "integer" else float(text) if self.kind == "number" else model.as_string
        except ValueError:
            self.owner.error(f"{self.name}: invalid number {text!r}")
            return
        self.set_value(value)
        self.edited()


class ArrayField(FormField):
    def __init__(self, owner, desc):
        super().__init__(owner, desc)
        self.labels = desc.get("labels") or []
        self.fixed = bool(desc.get("fixed", True))
        self.temp = [float(v) for v in desc.get("default") or []]
        self.value = list(self.temp) if desc.get("user_default", False) else None

    def ready(self):
        return self.value is not None and len(self.value) > 0

    def collect(self):
        return self.value if self.ready() else self.temp

    def set_value(self, value):
        self.value = [float(v) for v in value]
        self.temp = list(self.value)

    def text_value(self):
        return " ".join(_fmt(v) for v in self.temp)

    def build(self, ui):
        ui.Label(self.title(), height=0)
        with ui.HStack(height=0, spacing=4):
            for i, value in enumerate(self.temp):
                with ui.VStack(height=0):
                    ui.Label(self.labels[i] if i < len(self.labels) else str(i), style=GUI_DIM_STYLE)
                    field = ui.StringField()
                    field.model.set_value(_edit_text(value))
                    field.model.add_end_edit_fn(functools.partial(self._edited, i))
                    if not self.fixed:
                        ui.Button("-", width=20, clicked_fn=functools.partial(self._remove, i))
            if not self.fixed:
                ui.Button("+", width=20, clicked_fn=self._add)

    def _edited(self, i, model):
        try:
            self.temp[i] = float(model.as_string.strip())
        except ValueError:
            self.owner.error(f"{self.name}[{i}]: invalid number")
            return
        self.value = list(self.temp)
        self.edited()

    def _remove(self, i):
        del self.temp[i]
        self.value = list(self.temp)
        self.locked = True
        self.owner.rebuild()

    def _add(self):
        self.temp.append(0.0)
        self.value = list(self.temp)
        self.locked = True
        self.owner.rebuild()


class ComboField(FormField):
    def __init__(self, owner, desc):
        super().__init__(owner, desc)
        self.values = [str(v) for v in desc.get("values") or []]
        self.send_index = bool(desc.get("send_index", False))
        self.value = None
        index = desc.get("default_index", -1)
        if len(self.values) == 1:
            self.value = self.values[0]
        if isinstance(index, int) and 0 <= index < len(self.values):
            self.value = self.values[index]

    def current_values(self):
        """(choices, message) of the combo (data combos resolve them from the GUI data)."""
        return self.values, None

    def ready(self):
        return self.value is not None

    def collect(self):
        values, _ = self.current_values()
        if self.send_index:
            return values.index(self.value) if self.value in values else 0
        return self.value

    def set_value(self, value):
        values, _ = self.current_values()
        if self.send_index and isinstance(value, int) and 0 <= value < len(values):
            self.value = values[value]
        elif str(value) in values or not values:
            self.value = str(value)

    def text_value(self):
        return self.value or ""

    def build(self, ui):
        values, message = self.current_values()
        if self.value is not None and values and self.value not in values:
            self.value = None
        with ui.HStack(height=0, spacing=6):
            ui.Label(self.title(), width=ui.Percent(35))
            if message:
                ui.Label(message, style=GUI_DIM_STYLE)
                return
            if len(values) == 1 and self.value is not None:
                ui.Label(self.value)
                return
            index = values.index(self.value) + 1 if self.value in values else 0
            combo = ui.ComboBox(index, "", *values)
            combo.model.add_item_changed_fn(functools.partial(self._changed, values))

    def _changed(self, values, model, item):
        i = model.get_item_value_model().as_int - 1
        self.value = values[i] if 0 <= i < len(values) else None
        self.edited()


class DataComboField(ComboField):
    def current_values(self):
        return _resolve_data(self.owner.mirror.data, self.desc.get("ref") or [], self.owner.value_of)


class PoseField(FormField):
    """point3d [x y z], rotation [qw qx qy qz], transform {translation, rotation} (numeric edition only)."""

    def __init__(self, owner, desc):
        super().__init__(owner, desc)
        self.kind = desc.get("kind")
        default = list(desc.get("default") or [])
        if self.kind == "point3d":
            self.t, self.q = (default + [0.0] * 3)[:3], None
        elif self.kind == "rotation":
            self.t, self.q = None, (default + [1.0, 0.0, 0.0, 0.0])[:4] if len(default) == 4 else [1.0, 0.0, 0.0, 0.0]
        else:
            self.t = (list(desc.get("default_translation") or []) + [0.0] * 3)[:3]
            self.q = default if len(default) == 4 else [1.0, 0.0, 0.0, 0.0]
        self.is_set = bool(desc.get("user_default", False))

    def ready(self):
        return self.is_set

    def collect(self):
        if self.kind == "point3d":
            return list(self.t)
        if self.kind == "rotation":
            return list(self.q)
        return {"translation": list(self.t), "rotation": list(self.q)}

    def set_value(self, value):
        if self.kind == "point3d":
            self.t = [float(v) for v in value][:3]
        elif self.kind == "rotation":
            self.q = [float(v) for v in value][:4]
        elif isinstance(value, dict):
            self.t = [float(v) for v in value.get("translation", self.t)][:3]
            rotation = value.get("rotation", self.q)
            if len(rotation) == 4:
                self.q = [float(v) for v in rotation]
        self.is_set = True

    def build(self, ui):
        ui.Label(self.title(), height=0)
        rows = []
        if self.t is not None:
            rows.append(("t", ["x", "y", "z"], self.t))
        if self.q is not None:
            rows.append(("q", ["qw", "qx", "qy", "qz"], self.q))
        for which, labels, values in rows:
            with ui.HStack(height=0, spacing=4):
                for i, label in enumerate(labels):
                    ui.Label(label, width=22, style=GUI_DIM_STYLE)
                    field = ui.StringField()
                    field.model.set_value(_edit_text(float(values[i])))
                    field.model.add_end_edit_fn(functools.partial(self._edited, which, i))

    def _edited(self, which, i, model):
        try:
            value = float(model.as_string.strip())
        except ValueError:
            self.owner.error(f"{self.name}: invalid number")
            return
        (self.t if which == "t" else self.q)[i] = value
        self.is_set = True
        self.edited()


class ObjectField(FormField):
    def __init__(self, owner, desc, root=False):
        super().__init__(owner, desc)
        self.root = root
        self.fields = [_make_field(owner, d) for d in desc.get("f") or []]

    def children(self):
        return self.fields

    def ready(self):
        return all(f.ready() for f in self.fields if f.required)

    def collect(self):
        return {f.name: f.collect() for f in self.fields if f.required or f.ready()}

    def set_value(self, value):
        if isinstance(value, dict):
            for f in self.fields:
                if f.name in value:
                    f.set_value(value[f.name])

    def missing(self, prefix=""):
        prefix = prefix if self.root else f"{prefix}{self.name}/"
        return [m for f in self.fields if f.required for m in f.missing(prefix)]

    def build(self, ui):
        if self.root:
            self._build_fields(ui)
            return
        frame = ui.CollapsableFrame(self.title(), collapsed=False, height=0)
        with frame:
            with ui.HStack(height=0):
                ui.Spacer(width=10)
                with ui.VStack(height=0, spacing=4):
                    self._build_fields(ui)

    def _build_fields(self, ui):
        required = [f for f in self.fields if f.required]
        optional = [f for f in self.fields if not f.required]
        with ui.VStack(height=0, spacing=4):
            for f in required:
                f.build(ui)
            if optional and required:
                with ui.CollapsableFrame("Optional fields", collapsed=True, height=0):
                    with ui.VStack(height=0, spacing=4):
                        for f in optional:
                            f.build(ui)
            else:
                for f in optional:
                    f.build(ui)


class GenericArrayField(FormField):
    """List of items described by one template field (+ / - buttons); sent as the list of the item values."""

    def __init__(self, owner, desc):
        super().__init__(owner, desc)
        self.template = (desc.get("f") or [{}])[0]
        self.minimum = int(desc.get("min") or 0)
        self.maximum = desc.get("max")
        self.items = []
        for value in desc.get("prefill") or []:
            self._add(value)
        if isinstance(desc.get("data"), list):
            self.set_value(desc["data"])

    def _add(self, value=NO_DATA):
        """Append an item made from the template, optionally with a value."""
        item = _make_field(self.owner, {**self.template, "required": True})
        if value is not NO_DATA and value is not None:
            item.set_value(value)
        self.items.append(item)

    def children(self):
        return self.items

    def ready(self):
        return all(i.ready() for i in self.items) and (self.required or len(self.items) > 0)

    def collect(self):
        return [i.collect() for i in self.items]

    def set_value(self, value):
        self.items = []
        for v in value or []:
            self._add(v)

    def missing(self, prefix=""):
        return [m for i, item in enumerate(self.items) for m in item.missing(f"{prefix}{self.name}[{i}]/")] or \
            ([] if self.ready() else [prefix + self.name])

    def build(self, ui):
        with ui.CollapsableFrame(self.title(), collapsed=False, height=0):
            with ui.VStack(height=0, spacing=4):
                for i, item in enumerate(self.items):
                    with ui.HStack(height=0, spacing=6):
                        ui.Label(f"[{i}]", width=30)
                        with ui.VStack(height=0):
                            item.build(ui)
                        if len(self.items) > self.minimum:
                            ui.Button("-", width=20, clicked_fn=functools.partial(self._remove, i))
                if self.maximum is None or len(self.items) < self.maximum:
                    ui.Button("+", width=20, clicked_fn=self._plus)

    def _remove(self, i):
        del self.items[i]
        self.locked = True
        self.owner.rebuild()

    def _plus(self):
        self._add()
        self.locked = True
        self.owner.rebuild()


class OneOfField(FormField):
    """Choice between field types; sent as [option index, option value]."""

    def __init__(self, owner, desc):
        super().__init__(owner, desc)
        self.options = desc.get("f") or []
        self.active = None
        self.index = None
        if "data_index" in desc:
            self._select(int(desc["data_index"]), desc.get("data_value"))

    def _select(self, index, value=NO_DATA):
        """Activate option index (with an optional value)."""
        if not 0 <= index < len(self.options):
            self.active = self.index = None
            return
        self.index = index
        self.active = _make_field(self.owner, {**self.options[index], "required": True})
        if value is not NO_DATA and value is not None:
            self.active.set_value(value)

    def children(self):
        return [self.active] if self.active is not None else []

    def ready(self):
        return self.active is not None and self.active.ready()

    def collect(self):
        return [self.index, self.active.collect()] if self.active is not None else None

    def set_value(self, value):
        if isinstance(value, list) and len(value) == 2:
            self._select(int(value[0]), value[1])

    def build(self, ui):
        names = [o.get("name", str(i)) for i, o in enumerate(self.options)]
        with ui.HStack(height=0, spacing=6):
            ui.Label(self.title(), width=ui.Percent(35))
            combo = ui.ComboBox(self.index + 1 if self.index is not None else 0, "", *names)
            combo.model.add_item_changed_fn(self._changed)
        if self.active is not None:
            with ui.HStack(height=0):
                ui.Spacer(width=10)
                with ui.VStack(height=0):
                    self.active.build(ui)

    def _changed(self, model, item):
        self._select(model.get_item_value_model().as_int - 1)
        self.locked = True
        self.owner.rebuild()


FORM_FIELDS = {"checkbox": SimpleField, "integer": SimpleField, "number": SimpleField, "string": SimpleField,
               "array": ArrayField, "combo": ComboField, "data_combo": DataComboField, "point3d": PoseField,
               "rotation": PoseField, "transform": PoseField, "object": ObjectField,
               "generic_array": GenericArrayField, "one_of": OneOfField, "const": ConstField}


def _make_field(owner, desc):
    """Form field of a field description (UnsupportedField for unknown kinds)."""
    return FORM_FIELDS.get(desc.get("kind"), UnsupportedField)(owner, desc)


def _transfer(old, new):
    """Keep the values the user edited when the form description changes."""
    previous = {f.name: f for f in old.children()}
    for field in new.children():
        before = previous.get(field.name)
        if before is None or type(before) is not type(field):
            continue
        if isinstance(field, ObjectField):
            _transfer(before, field)
        elif before.locked:
            field.set_value(before.collect())
            field.locked = True


def _form_structure(fields):
    """Form description without the default values (changes of the defaults alone do not rebuild an edited form)."""
    return [{k: (_form_structure(v) if k == "f" else v) for k, v in f.items()
             if not k.startswith("default") and not k.startswith("data")} for f in fields]


class FormOwnerMixin:
    """value_of / changed / rebuild / error for the fields of a FormView or SchemaView."""

    root = None
    error_label = None

    def value_of(self, name):
        """Text value of the field name, for "$name" data combo references."""
        field = self.root.find(name) if self.root is not None else None
        return field.text_value() if field is not None else ""

    def changed(self):
        # data combos may depend on the other fields ($name references)
        """A field changed: rebuild if some data combo depends on other fields."""
        if self.root is not None and _has_refs(self.root):
            self.frame.rebuild()

    def rebuild(self):
        """Rebuild the form widgets (array items added/removed, one-of option changed)."""
        self.frame.rebuild()

    def error(self, message):
        """Show a message under the form."""
        if self.error_label is not None:
            self.error_label.text = message

    def send(self):
        """Send the form if all required fields are set; returns True if sent."""
        missing = self.root.missing()
        if missing:
            self.error(f"Missing: {', '.join(missing)}")
            return False
        data = self.root.collect()
        self.root.unlock()
        self.request(data)
        return True


def _has_refs(field):
    """True if a data combo of the subtree references another field ("$name")."""
    if isinstance(field, DataComboField) and any(str(r).startswith("$") for r in field.desc.get("ref") or []):
        return True
    return any(_has_refs(c) for c in field.children())


class FormView(FormOwnerMixin, WidgetView):
    """mc_rtc Form: fields + a button named like the form that sends them."""

    def __init__(self, mirror, desc):
        super().__init__(mirror, desc)
        self.root_desc = None
        self.last_build = 0.0

    def _build(self, ui):
        fields = self.desc.get("f") or []
        if self.root is None or fields != self.root_desc:
            old = self.root
            self.root = ObjectField(self, {"name": "", "f": fields, "required": True}, root=True)
            self.root_desc = fields
            if old is not None:
                _transfer(old, self.root)
        self.last_build = time.perf_counter()
        with ui.VStack(height=0, spacing=4):
            self.root.build(ui)
            self.error_label = ui.Label("", style=GUI_ERROR_STYLE, word_wrap=True, height=0)
            ui.Button(self.name, height=26, clicked_fn=self.send)

    def _refresh(self, old, new):
        fields = new.get("f") or []
        if _form_structure(fields) != _form_structure(self.root_desc or []):
            return False
        # new defaults: shown when the user is not editing this form (at most once per second)
        if fields != self.root_desc and not self.root.any_locked() and time.perf_counter() - self.last_build > 1.0:
            return False
        return True


def _schema_default(item_type, i, count, maximum):
    """mc_rtc-imgui defaults of fixed-size schema arrays: identity for 3x3 (9) and 6x6 (36) matrices."""
    if item_type not in ("number", "integer"):
        return {"boolean": False, "string": ""}.get(item_type)
    span = {9: 3, 36: 6}.get(count) if count == maximum else None
    value = 1 if span and i % span == i // span else 0
    return float(value) if item_type == "number" else value


def _schema_field(name, prop, required):
    """JSON schema property -> form field description (same rules as mc_rtc-imgui ObjectForm)."""
    if not isinstance(prop, dict):
        return None
    base = {"name": name, "required": required}
    if "enum" in prop:
        return {**base, "kind": "combo", "values": [str(v) for v in prop["enum"]], "send_index": False}
    if "const" in prop:
        return {**base, "kind": "const", "value": prop["const"]}
    kind = prop.get("type", "")
    has_default = "default" in prop
    default = prop.get("default")
    if kind == "boolean":
        return {**base, "kind": "checkbox", "default": bool(default), "user_default": has_default}
    if kind == "integer":
        if name == "robotIndex":
            return {**base, "kind": "data_combo", "ref": ["robots"], "send_index": True}
        return {**base, "kind": "integer", "default": default if has_default else 0, "user_default": has_default}
    if kind == "number":
        return {**base, "kind": "number", "default": default if has_default else 0.0, "user_default": has_default}
    if kind == "string":
        refs = {"robot": ["robots"], "r1": ["robots"], "r2": ["robots"], "body": ["bodies", "$robot"],
                "surface": ["surfaces", "$robot"], "r1Surface": ["surfaces", "$r1"], "r2Surface": ["surfaces", "$r2"],
                "frame": ["frames", "$robot"]}
        if name in refs:
            return {**base, "kind": "data_combo", "ref": refs[name], "send_index": False}
        return {**base, "kind": "string", "default": default if has_default else "", "user_default": has_default}
    if kind == "array":
        items = prop.get("items") or {}
        if isinstance(items, list):
            # tuple validation (one schema per position): the first one describes the items
            items = items[0] if items else {}
        items = dict(items)
        if items.get("oneOf"):
            # as mc_rtc-imgui: only the first possible item type
            items = {**items, **items["oneOf"][0]}
            items.pop("oneOf", None)
        item_type = items.get("type")
        template = _schema_field("", items, True) if item_type not in (None, "array") else None
        if template is None:
            return None
        count, maximum = int(prop.get("minItems", 0)), prop.get("maxItems")
        return {**base, "kind": "generic_array", "f": [template], "min": count, "max": maximum,
                "prefill": [_schema_default(item_type, i, count, maximum) for i in range(count)]}
    if kind == "object":
        return {**base, "kind": "object", "f": _schema_fields(prop)}
    return None


def _schema_fields(schema):
    """Form fields of a JSON schema object (properties, required)."""
    required = set(schema.get("required") or [])
    fields = []
    for name in sorted(schema.get("properties") or {}):
        if name == "completion":
            continue
        field = _schema_field(name, schema["properties"][name], name in required)
        if field is not None:
            fields.append(field)
    # robot selectors first: the other combos depend on them
    fields.sort(key=lambda f: f["name"] not in ("robot", "robotIndex", "r1", "r2"))
    return fields


class SchemaView(FormOwnerMixin, WidgetView):
    """mc_rtc Schema: choose a schema of the directory, fill the generated form, send."""

    def __init__(self, mirror, desc):
        super().__init__(mirror, desc)
        self.selected = None
        self.schema_count = 0

    def _schemas(self, desc):
        return self.mirror.schemas.get(desc.get("schema"), {}) or {}

    def _build(self, ui):
        schemas = self._schemas(self.desc)
        self.schema_count = len(schemas)
        titles = sorted(schemas)
        with ui.VStack(height=0, spacing=4):
            with ui.HStack(height=0, spacing=6):
                ui.Label(self.name, width=ui.Percent(35))
                index = titles.index(self.selected) + 1 if self.selected in titles else 0
                combo = ui.ComboBox(index, "", *titles)
                combo.model.add_item_changed_fn(functools.partial(self._select, titles))
            if self.selected not in schemas:
                return
            if self.root is None:
                self.root = ObjectField(self, {"name": "", "f": _schema_fields(schemas[self.selected]),
                                               "required": True}, root=True)
            self.root.build(ui)
            self.error_label = ui.Label("", style=GUI_ERROR_STYLE, word_wrap=True, height=0)
            ui.Button(self.name, height=26, clicked_fn=self._send)

    def _select(self, titles, model, item):
        i = model.get_item_value_model().as_int - 1
        self.selected = titles[i] if 0 <= i < len(titles) else None
        self.root = None
        self.frame.rebuild()

    def _send(self):
        if self.send():
            # a new empty form, as mc-rtc-magnum
            self.root = None
            self.frame.rebuild()

    def _refresh(self, old, new):
        return old.get("schema") == new.get("schema") and len(self._schemas(new)) == self.schema_count


GUI_WIDGETS = {"Label": LabelView, "ArrayLabel": ArrayLabelView, "Button": ButtonView, "Checkbox": CheckboxView,
               "StringInput": SingleInputView, "IntegerInput": IntegerInputView, "NumberInput": NumberInputView,
               "NumberSlider": NumberSliderView, "ArrayInput": ArrayInputView, "ComboInput": ComboInputView,
               "DataComboInput": DataComboInputView, "Table": TableView, "Form": FormView, "Schema": SchemaView}


# ---- plots (mc_rtc gui plots: standard and XY), rendered with numpy into an image


class PlotState:
    """Accumulated data of one mc_rtc plot (series points, latest polygons, axes)."""
    MAX_POINTS = 4000

    def __init__(self, pid):
        self.id = pid
        self.title = ""
        self.axes = {}
        self.series = {}
        self.points = {}
        self.polygons = {}
        self.active = True
        self.dirty = True

    def update(self, plot):
        """Plot entry of a GUI payload: new points, series styles, polygons, axes."""
        title = plot.get("title", "")
        if title != self.title:
            self.series, self.points, self.polygons = {}, {}, {}
            self.title = title
        self.axes = plot.get("axes") or {}
        for s in plot.get("series") or []:
            self.series[int(s.get("did", 0))] = s
        values = plot.get("points") or []
        for i in range(0, len(values) - 2, 3):
            series = self.points.setdefault(int(values[i]), collections.deque(maxlen=self.MAX_POINTS))
            series.append((values[i + 1], values[i + 2]))
        if "polygons" in plot:
            self.polygons = {int(p.get("did", 0)): p for p in plot["polygons"] or []}
        self.dirty = True


def _axis_range(axis, arrays):
    """Axis bounds: configured min/max, else data extent with a 5 % margin."""
    axis = axis or {}
    data = np.concatenate([a[np.isfinite(a)] for a in arrays]) if arrays else np.zeros(0)
    lo, hi = axis.get("min"), axis.get("max")
    if lo is None:
        lo = float(data.min()) if data.size else 0.0
    if hi is None:
        hi = float(data.max()) if data.size else 1.0
    if hi - lo < 1e-9:
        lo, hi = lo - 0.5, hi + 0.5
    elif axis.get("min") is None or axis.get("max") is None:
        pad = 0.05 * (hi - lo)
        lo = lo - pad if axis.get("min") is None else lo
        hi = hi + pad if axis.get("max") is None else hi
    return lo, hi


def _rgba8(color):
    """mc_rtc color [r, g, b, a] (0-1) -> RGBA uint8."""
    return np.array([int(max(0.0, min(1.0, float(c))) * 255) for c in (list(color or []) + [1.0] * 4)[:4]],
                    dtype=np.uint8)


def _draw_polyline(img, px, py, color, closed=False, dash=None):
    """Draw a 2-pixel polyline on img (pixel coordinates, non-finite points skipped)."""
    ok = np.isfinite(px) & np.isfinite(py)
    px, py = px[ok], py[ok]
    if closed and len(px) > 2:
        px, py = np.append(px, px[0]), np.append(py, py[0])
    if len(px) == 0:
        return
    if len(px) == 1:
        _draw_dot(img, px[0], py[0], color, 2)
        return
    lengths = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(px), np.diff(py)))])
    t = np.arange(0.0, lengths[-1] + 0.5, 0.5)
    if dash:
        t = t[(t // dash) % 2 == 0]
    x = np.interp(t, lengths, px)
    y = np.interp(t, lengths, py)
    h, w = img.shape[:2]
    for dx, dy in ((0, 0), (1, 0), (0, 1)):
        xi = np.clip(np.round(x).astype(int) + dx, 0, w - 1)
        yi = np.clip(np.round(y).astype(int) + dy, 0, h - 1)
        img[yi, xi] = color


def _draw_dot(img, x, y, color, radius):
    """Draw a filled disc on img."""
    if not (np.isfinite(x) and np.isfinite(y)):
        return
    h, w = img.shape[:2]
    y0, y1 = max(0, int(y - radius)), min(h, int(y + radius) + 1)
    x0, x1 = max(0, int(x - radius)), min(w, int(x + radius) + 1)
    if y0 >= y1 or x0 >= x1:
        return
    gy, gx = np.mgrid[y0:y1, x0:x1]
    mask = (gx - x) ** 2 + (gy - y) ** 2 <= radius ** 2
    img[y0:y1, x0:x1][mask] = color


def _fill_polygon(img, px, py, color):
    """Fill a polygon on img (even-odd rule, alpha blended)."""
    ok = np.isfinite(px) & np.isfinite(py)
    px, py = px[ok], py[ok]
    if len(px) < 3 or color[3] == 0:
        return
    h, w = img.shape[:2]
    x0, x1 = max(0, int(px.min())), min(w - 1, int(px.max()) + 1)
    y0, y1 = max(0, int(py.min())), min(h - 1, int(py.max()) + 1)
    if x0 >= x1 or y0 >= y1:
        return
    gy, gx = np.mgrid[y0:y1 + 1, x0:x1 + 1] + 0.5
    inside = np.zeros(gx.shape, dtype=bool)
    j = len(px) - 1
    for i in range(len(px)):
        crosses = (py[i] > gy) != (py[j] > gy)
        with np.errstate(divide="ignore", invalid="ignore"):
            xc = (px[j] - px[i]) * (gy - py[i]) / (py[j] - py[i]) + px[i]
        inside ^= crosses & (gx < xc)
        j = i
    region = img[y0:y1 + 1, x0:x1 + 1]
    alpha = color[3] / 255.0
    region[inside, :3] = (region[inside, :3] * (1 - alpha) + color[:3] * alpha).astype(np.uint8)


def _render_plot(plot, width, height):
    """RGBA image of a plot and its axis ranges {x, y, y2}."""
    img = np.empty((height, width, 4), dtype=np.uint8)
    img[:] = (32, 32, 32, 255)
    arrays = {"x": [], 0: [], 1: []}
    for did, points in plot.points.items():
        values = np.array(points, dtype=np.float64).reshape(-1, 2)
        side = int(plot.series.get(did, {}).get("side", 0))
        arrays["x"].append(values[:, 0])
        arrays[side].append(values[:, 1])
    for entry in plot.polygons.values():
        for poly in entry.get("polygons") or []:
            values = np.array(poly.get("points") or [], dtype=np.float64).reshape(-1, 2)
            arrays["x"].append(values[:, 0])
            arrays[int(entry.get("side", 0))].append(values[:, 1])
    ranges = {"x": _axis_range(plot.axes.get("x"), arrays["x"]),
              "y": _axis_range(plot.axes.get("y"), arrays[0]),
              "y2": _axis_range(plot.axes.get("y2"), arrays[1])}

    def to_px(x):
        lo, hi = ranges["x"]
        return (np.asarray(x, dtype=np.float64) - lo) / (hi - lo) * (width - 1)

    def to_py(y, side):
        lo, hi = ranges["y2" if side == 1 else "y"]
        return (1.0 - (np.asarray(y, dtype=np.float64) - lo) / (hi - lo)) * (height - 1)

    grid = np.array([60, 60, 60, 255], dtype=np.uint8)
    for k in range(1, 4):
        img[int(k * (height - 1) / 4), :] = grid
        img[:, int(k * (width - 1) / 4)] = grid
    lo, hi = ranges["y"]
    if lo < 0 < hi:
        img[int(to_py(0.0, 0)), :] = (110, 110, 110, 255)
    for entry in plot.polygons.values():
        side = int(entry.get("side", 0))
        for poly in entry.get("polygons") or []:
            values = np.array(poly.get("points") or [], dtype=np.float64).reshape(-1, 2)
            px, py = to_px(values[:, 0]), to_py(values[:, 1], side)
            _fill_polygon(img, px, py, _rgba8(poly.get("fill")))
            _draw_polyline(img, px, py, _rgba8(poly.get("outline")), closed=bool(poly.get("closed", True)))
    for did, points in plot.points.items():
        series = plot.series.get(did, {})
        values = np.array(points, dtype=np.float64).reshape(-1, 2)
        side, style, color = int(series.get("side", 0)), int(series.get("style", 0)), _rgba8(series.get("color"))
        px, py = to_px(values[:, 0]), to_py(values[:, 1], side)
        if style == 3:
            # Point style: the last point only (as mc-rtc-magnum)
            _draw_dot(img, px[-1], py[-1], color, 4)
        else:
            _draw_polyline(img, px, py, color, dash={1: 3, 2: 8}.get(style))
    return img, ranges


class PlotsWindow:
    """The mc_rtc plots: one tab button per plot, ended plots are kept until closed (as mc-rtc-magnum)."""

    WIDTH, HEIGHT = 640, 320

    def __init__(self, mirror):
        import omni.ui as ui

        self.mirror = mirror
        self.selected = None
        self.signature = None
        self.labels = None
        self.provider = ui.ByteImageProvider()
        self.window = ui.Window("mc_rtc plots", width=self.WIDTH + 40, height=self.HEIGHT + 200)
        with self.window.frame:
            self.frame = ui.Frame(build_fn=self._build)

    def _build(self):
        import omni.ui as ui

        plots = self.mirror.plots
        self.labels = None
        with ui.VStack(spacing=4, height=0):
            with ui.VGrid(column_count=max(1, min(4, len(plots))), row_height=24, height=0):
                for pid, plot in plots.items():
                    title = plot.title + ("" if plot.active else " (ended)")
                    ui.Button(title, style=GUI_TAB_SELECTED_STYLE if pid == self.selected else GUI_TAB_STYLE,
                              clicked_fn=functools.partial(self._select, pid))
            plot = plots.get(self.selected)
            if plot is None:
                return
            with ui.HStack(height=0):
                y_label = ui.Label("")
                y2_label = ui.Label("", alignment=ui.Alignment.RIGHT)
            ui.ImageWithProvider(self.provider, height=ui.Pixel(self.HEIGHT),
                                 fill_policy=ui.IwpFillPolicy.IWP_STRETCH)
            x_label = ui.Label("", alignment=ui.Alignment.CENTER)
            self.labels = (x_label, y_label, y2_label)
            with ui.VGrid(column_count=3, row_height=20, height=0):
                for did, series in sorted(plot.series.items()):
                    side = " (right)" if int(series.get("side", 0)) == 1 else ""
                    ui.Label(f"\u25a0 {series.get('legend', did)}{side}", style={"color": _abgr(series.get("color"))})
                for entry in plot.polygons.values():
                    polygons = entry.get("polygons") or [{}]
                    ui.Label(f"\u25a1 {entry.get('legend', '')}", style={"color": _abgr(polygons[0].get("outline"))})
            if not plot.active:
                ui.Button("Close this plot", height=24, clicked_fn=functools.partial(self._close, plot.id))
        plot.dirty = True

    def _select(self, pid):
        self.selected = pid
        self.frame.rebuild()

    def _close(self, pid):
        self.mirror.plots.pop(pid, None)
        self.selected = None

    def refresh(self):
        """Main loop: rebuild on new plots/series, else redraw the selected plot when it changed."""
        plots = self.mirror.plots
        if self.selected not in plots:
            self.selected = next(iter(plots), None)
        signature = (self.selected, tuple((pid, p.title, p.active, tuple(sorted(p.series)), len(p.polygons))
                                          for pid, p in plots.items()))
        if signature != self.signature:
            self.signature = signature
            self.frame.rebuild()
            return
        plot = plots.get(self.selected)
        if plot is None or not plot.dirty or self.labels is None:
            return
        plot.dirty = False
        img, ranges = _render_plot(plot, self.WIDTH, self.HEIGHT)
        self.provider.set_data_array(np.ascontiguousarray(img), [self.WIDTH, self.HEIGHT])
        x_label, y_label, y2_label = self.labels
        axes = plot.axes
        x_label.text = f"{(axes.get('x') or {}).get('label', '')}  [{ranges['x'][0]:.4g} .. {ranges['x'][1]:.4g}]"
        y_label.text = f"{(axes.get('y') or {}).get('label', '')}  [{ranges['y'][0]:.4g} .. {ranges['y'][1]:.4g}]"
        if any(int(s.get("side", 0)) == 1 for s in plot.series.values()):
            y2_label.text = f"{(axes.get('y2') or {}).get('label', '')}  [{ranges['y2'][0]:.4g} .. {ranges['y2'][1]:.4g}]"

    def destroy(self):
        self.window.visible = False
        self.window.destroy()


# ------------------------------------------------------------------------------------------ bridge


def _recv_exact(sock, n):
    """Read exactly n bytes from the socket (ConnectionError when closed)."""
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("connection closed")
        buf += chunk
    return bytes(buf)


def recv_msg(sock):
    """Read one bridge frame: (JSON header, raw payload)."""
    header_len, payload_len = FRAME.unpack(_recv_exact(sock, FRAME.size))
    header = json.loads(_recv_exact(sock, header_len))
    return header, _recv_exact(sock, payload_len) if payload_len else b""


def send_msg(sock, header, payload=b""):
    """Write one bridge frame."""
    raw = json.dumps(header).encode()
    sock.sendall(FRAME.pack(len(raw), len(payload)) + raw + payload)


class BridgeServer(threading.Thread):
    """Lockstep TCP bridge, one client at a time. Isaac only steps when a 'step' message arrives."""

    def __init__(self, state, host, port):
        super().__init__(daemon=True)
        self.state = state
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host, port))
        self.sock.listen(1)

    def run(self):
        """Accept clients one at a time; on disconnect, reset the client state and the markers/GUI."""
        while True:
            conn, addr = self.sock.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.state.logs.add(f"Bridge client connected from {addr[0]}:{addr[1]}")
            self.state.bridge_client = f"{addr[0]}:{addr[1]}"
            try:
                self._serve(conn)
            except (ConnectionError, OSError):
                pass
            except Exception as e:
                self.state.logs.add(f"ERROR: bridge: {type(e).__name__}: {e}")
            finally:
                conn.close()
                self.state.bridge_client = None
                self.state.ticker = None
                self.state.client_commands = []
                try:
                    self.state.executor.call(self.state.markers.clear, timeout=10)
                    self.state.executor.call(lambda: self.state.apply_gui_mode(None), timeout=10)
                except Exception as e:
                    self.state.logs.add(f"WARNING: cannot clear the mc_rtc markers/GUI: {e}")
                self.state.logs.add("Bridge client disconnected")

    def _serve(self, conn):
        """Serve one client: each request runs on the main thread, its reply is sent back."""
        while True:
            header, payload = recv_msg(conn)
            t_recv = time.perf_counter()
            started = []

            def run():
                started.append(time.perf_counter())
                return self.state.handle_bridge(header, payload)

            try:
                reply, out = self.state.executor.call(run, timeout=120)
                timings = reply.setdefault("timings", {})
                # wait: queued behind the main loop (render, UI); server: receive -> reply
                timings["wait"] = 1e3 * (started[0] - t_recv)
                timings["server"] = 1e3 * (time.perf_counter() - t_recv)
            except Exception as e:
                reply, out = {"ok": False, "error": str(e)}, b""
            send_msg(conn, reply, out)


def make_handler(state):
    """HTTP request handler class bound to the server state."""
    class Handler(BaseHTTPRequestHandler):
        server_version = "mc_isaac_server/" + SERVER_VERSION

        def log_message(self, fmt, *args):
            pass

        def _send(self, code, payload):
            body = json.dumps(payload).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            url = urlparse(self.path)
            if url.path == "/status":
                self._send(200, state.status())
            elif url.path == "/logs":
                try:
                    since = int(parse_qs(url.query).get("since", ["0"])[0])
                except ValueError:
                    self._send(400, {"error": "'since' must be an integer"})
                    return
                entries, next_id = state.logs.since(since)
                self._send(200, {"next": next_id, "logs": [{"id": i, "time": t, "msg": m} for i, t, m in entries]})
            elif state.streamer is not None and url.path in ("/", "/index.html"):
                self._send_bytes(200, STREAM_PAGE.replace("@VERSION@", SERVER_VERSION).encode(), "text/html; charset=utf-8")
            elif state.streamer is not None and url.path == "/snapshot.jpg":
                _, jpeg = state.streamer.next_frame(-1, timeout=0.0)
                if jpeg is None:
                    self._send(503, {"error": "no frame yet (is a scene loaded?)"})
                else:
                    self._send_bytes(200, jpeg, "image/jpeg")
            elif state.streamer is not None and url.path == "/stream":
                self._stream()
            else:
                self._send(404, {"error": f"unknown endpoint {url.path}"})

        def _send_bytes(self, code, body, content_type):
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _stream(self):
            """MJPEG multipart stream until the client disconnects."""
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            seq = -1
            try:
                while not state.shutdown_requested.is_set():
                    seq, jpeg = state.streamer.next_frame(seq)
                    if jpeg is None:
                        continue
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                     + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_POST(self):
            url = urlparse(self.path)
            if url.path == "/shutdown":
                state.logs.add("Shutdown requested over HTTP")
                state.shutdown_requested.set()
                self._send(200, {"state": "stopping"})
            elif url.path == "/scene":
                spec = self._read_json()
                force = parse_qs(url.query).get("force", ["0"])[0] == "1"
                if spec is not None:
                    self._run_main(lambda: state.load_scene(spec, force), timeout=600)
            elif url.path == "/reset":
                self._run_main(state.reset_scene, timeout=60)
            elif url.path == "/clear":
                self._run_main(state.clear_scene, timeout=60)
            else:
                self._send(404, {"error": f"unknown endpoint {url.path}"})

        def do_HEAD(self):
            sha = self._upload_sha()
            try:
                found = sha is not None and state.cache.has(sha)
            except ValueError:
                found = False
            self.send_response(200 if found else 404)
            self.send_header("Content-Length", "0")
            self.end_headers()

        def do_PUT(self):
            sha = self._upload_sha()
            if sha is None:
                self._send(404, {"error": "use PUT /upload/<sha256>?filename=<name>"})
                return
            filename = parse_qs(urlparse(self.path).query).get("filename", [""])[0]
            try:
                length = int(self.headers.get("Content-Length", "-1"))
                if length < 0:
                    raise ValueError("Content-Length required")
                state.cache.store(sha, filename, self.rfile, length)
            except ValueError as e:
                self._send(400, {"error": str(e)})
                return
            except Exception as e:
                state.logs.add(f"ERROR: upload of {filename} failed: {type(e).__name__}: {e}")
                self._send(500, {"error": f"{type(e).__name__}: {e}"})
                return
            state.logs.add(f"Asset stored: {filename} ({sha[:12]}, {length / 1e6:.1f} MB)")
            self._send(200, {"stored": filename})

        def _upload_sha(self):
            """sha256 of an /upload/<sha256> path, else None."""
            parts = urlparse(self.path).path.strip("/").split("/")
            return parts[1] if len(parts) == 2 and parts[0] == "upload" else None

        def _read_json(self):
            """JSON body of the request (sends 400 and returns None if invalid)."""
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= MAX_JSON_BODY:
                    raise ValueError("missing or too large body")
                return json.loads(self.rfile.read(length))
            except ValueError as e:
                self._send(400, {"error": f"invalid JSON body: {e}"})
                return None

        def _run_main(self, fn, timeout):
            """Run fn on the main thread and send its JSON result (503 if not ready)."""
            if state.state != "ready":
                self._send(503, {"error": f"server not ready ({state.state})"})
                return
            try:
                self._send(200, state.executor.call(fn, timeout=timeout))
            except Exception as e:
                self._send(500, {"error": str(e)})

    return Handler


def get_isaac_version():
    """Isaac Sim version (isaacsim.core.version, else $ISAAC_PATH/VERSION)."""
    try:
        from isaacsim.core.version import get_version

        return str(get_version()[0])
    except Exception:
        pass
    isaac_path = os.environ.get("ISAAC_PATH", "")
    version_file = os.path.join(isaac_path, "VERSION")
    if isaac_path and os.path.isfile(version_file):
        with open(version_file) as f:
            return f.read().strip()
    return "unknown"


def has_app_window():
    """True/False if Kit has/has not a window, None if unknown."""
    try:
        import omni.appwindow

        return omni.appwindow.get_default_app_window() is not None
    except Exception:
        return None


def is_stage_loading():
    """True while USD stage assets are still loading."""
    try:
        import omni.usd

        _, loaded, total = omni.usd.get_context().get_stage_loading_status()
        return loaded < total
    except Exception:
        return False


def warm_up(app, state, timeout, stable_frames=30, max_frame_time=0.1):
    """Update the app until shaders/materials are loaded, i.e. frames become consistently fast.

    The server only reports "ready" afterwards, so the first controller does not wait for shader compilation.
    """
    state.state = "warming_up"
    state.logs.add("Warming up (loading shaders/materials)...")
    start = time.time()
    fast = 0
    while app.is_running() and not state.shutdown_requested.is_set():
        t0 = time.perf_counter()
        app.update()
        dt = time.perf_counter() - t0
        fast = fast + 1 if dt < max_frame_time and not is_stage_loading() else 0
        if fast >= stable_frames:
            state.logs.add(f"Warm-up done in {time.time() - start:.1f}s")
            return
        if time.time() - start > timeout:
            state.logs.add(f"WARNING: warm-up not stable after {timeout:.0f}s, continuing anyway")
            return


def parse_args():
    """Server command line (normally passed by the mc_isaac_server launcher)."""
    parser = argparse.ArgumentParser(description="mc_isaac Isaac Sim server")
    parser.add_argument("--host", default="127.0.0.1", help="HTTP bind address (default: localhost only)")
    parser.add_argument("--http-port", type=int, default=5050)
    parser.add_argument("--bridge-port", type=int, default=5055, help="Lockstep TCP bridge port")
    parser.add_argument("--cache-dir", default="/tmp/mc_isaac_cache", help="Uploaded assets cache")
    parser.add_argument("--headless", action="store_true", help="Run Isaac Sim without window")
    parser.add_argument("--stream", choices=["none", "webrtc", "mjpeg"], default="none",
                        help="webrtc: interactive Isaac UI for the Isaac Sim WebRTC Streaming Client (implies headless); "
                             "mjpeg: view-only browser stream of the scene camera at http://<host>:<http-port>/")
    parser.add_argument("--stream-fps", type=float, default=15.0, help="MJPEG stream frame rate")
    parser.add_argument("--stream-size", default="1280x720", help="MJPEG stream resolution WxH")
    parser.add_argument("--max-fps", type=float, default=60.0, help="Max render rate when idle")
    parser.add_argument("--stepping-fps", type=float, default=30.0, help="Max render rate while a client is stepping")
    parser.add_argument("--vsync", action="store_true", help="Present frames with vsync (may slow stepping)")
    parser.add_argument("--no-fabric", action="store_true", help="Sync viewport through USD write-back (slower)")
    parser.add_argument("--warmup-timeout", type=float, default=300.0, help="Max seconds of warm-up before 'ready'")
    return parser.parse_args()


def main():
    """Start the HTTP and bridge threads, then Isaac Sim, warm it up, and run the main loop: queued
    requests, panel actions, idle timeline watch, GUI refresh and throttled rendering.
    """
    args = parse_args()
    if args.stream == "webrtc":
        args.headless = True
    if args.headless and args.stream == "none":
        # nothing to look at: only keep Kit's loop alive
        args.max_fps = min(args.max_fps, 5.0)
        args.stepping_fps = min(args.stepping_fps, 2.0)
    elif args.stream == "mjpeg" and args.headless:
        args.max_fps = min(args.max_fps, args.stream_fps)
        args.stepping_fps = min(args.stepping_fps, args.stream_fps)
    state = ServerState(args)

    if args.host not in ("127.0.0.1", "localhost", "::1"):
        state.logs.add(f"WARNING: HTTP API exposed on {args.host}, anyone reaching this address can control the simulation")
    try:
        httpd = ThreadingHTTPServer((args.host, args.http_port), make_handler(state))
        bridge = BridgeServer(state, args.host, args.bridge_port)
    except OSError as e:
        print(f"[mc_isaac_server] Cannot bind ports on {args.host} ({e}). Is a server already running?", flush=True)
        return 1
    httpd.daemon_threads = True
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    bridge.start()
    state.logs.add(f"HTTP API on http://{args.host}:{args.http_port}, bridge on {args.host}:{args.bridge_port} "
                   f"(server {SERVER_VERSION})")

    state.logs.add(f"Starting Isaac Sim ({'headless' if args.headless else 'window'}, stream: {args.stream})...")
    # SimulationApp must be created before importing any other omni/isaacsim module
    from isaacsim import SimulationApp

    if args.stream == "webrtc":
        # the UI is rendered for the streaming client instead of a local window
        app = SimulationApp({"headless": True, "hide_ui": False, "width": 1280, "height": 720,
                             "window_width": 1920, "window_height": 1080})
        from isaacsim.core.utils.extensions import enable_extension

        app.set_setting("/app/window/drawMouse", True)
        enable_extension("omni.services.livestream.nvcf")
        state.logs.add("WebRTC livestream enabled: connect with the Isaac Sim WebRTC Streaming Client to this host")
    else:
        app = SimulationApp({"headless": args.headless})
    state.app = app
    if args.stream == "mjpeg":
        state.streamer = MjpegStreamer(args.stream_fps, args.stream_size, state.logs.add)
    state.isaac_version = get_isaac_version()
    if not args.headless:
        state.window = has_app_window()
        if state.window is False:
            state.logs.add("WARNING: no window could be created (X11 access? DISPLAY?), running without window")

    def request_shutdown(signum, _frame):
        state.logs.add(f"Signal {signum} received, shutting down")
        state.shutdown_requested.set()

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)

    state.fabric_update = configure_kit_for_stepping(state.logs.add, use_fabric=not args.no_fabric, vsync=args.vsync)
    if state.streamer is not None:
        # headless Kit renders nothing until a render product exists: compile the RTX shaders now (can take minutes
        # the first time in a new container), not at the first scene load
        from pxr import UsdGeom

        import omni.usd

        UsdGeom.Camera.Define(omni.usd.get_context().get_stage(), "/mc_isaac_warmup_camera")
        state.streamer.attach("/mc_isaac_warmup_camera")
    warm_up(app, state, args.warmup_timeout)
    if state.streamer is not None:
        state.streamer.detach()
    if state.window:
        try:
            state.panel = IsaacPanel(state)
        except Exception as e:
            state.logs.add(f"WARNING: cannot create the mc_isaac panel: {type(e).__name__}: {e}")
    state.state = "ready"
    state.logs.add(f"Isaac Sim {state.isaac_version} ready")
    if args.stream == "mjpeg":
        state.logs.add(f"Browser stream (view only): http://{args.host}:{args.http_port}/")

    state.last_render = time.perf_counter()
    last_error = None
    try:
        while app.is_running() and not state.shutdown_requested.is_set():
            try:
                # serve requests, and render at most max_fps (stepping_fps while a client is stepping)
                period = state.render_period()
                state.executor.run_pending(period - (time.perf_counter() - state.last_render))
                state.run_panel_actions()
                state.watch_idle_timeline()
                if time.perf_counter() - state.last_render >= state.render_period():
                    if state.panel is not None:
                        state.panel.update()
                    if state.gui_mirror is not None:
                        state.gui_mirror.refresh()
                    state.render()
            except Exception as e:
                # keep serving: app.close() would exit before Python prints the traceback
                error = f"{type(e).__name__}: {e}"
                if error != last_error:
                    state.logs.add(f"ERROR in the main loop: {error}\n{traceback.format_exc()}")
                last_error = error
                state.last_render = time.perf_counter()
    finally:
        state.state = "stopping"
        state.logs.add("Closing Isaac Sim")
        httpd.shutdown()
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
