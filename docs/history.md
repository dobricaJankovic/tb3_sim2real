# History

Append-only log of problems/fixes and concepts/ideas from work sessions,
kept for the user's own reference. New conversations: do NOT read this whole
file for context — only append a new dated entry at the end after finishing
meaningful work. (See `CLAUDE.md`.) Everything before this file's inception
is in `git log`.

---

## 2026-09-06 — gzclient X11 auth crash on `backend:=gazebo`

**Problem:** `ros2 launch tb3_bringup bringup.launch.py backend:=gazebo
nav:=false` spawned the robot fine (`robot_state_publisher`, `gzserver`,
`spawn_entity` all succeeded — `/cmd_vel`, `/odom` live) but `gzclient`
aborted (SIGABRT) with `Authorization required, but no authorization
protocol specified`.

**Root cause:** Both the `isaacsim` and `tb3_ros` containers run as root,
but the host X server's access control is per-UID (`xhost` shows
`SI:localuser:<user>`, not root). `docker-compose.yml` mounted
`/tmp/.X11-unix` and passed `DISPLAY`, but never gave the container an
Xauthority cookie, so root's X11 clients couldn't authenticate. `gzserver`
only warns and keeps going (it's headless-capable); `gzclient` actually
needs to open a window and aborts without one.

**Fix:** Added `docker/x11-auth.sh` — generates a docker-specific Xauthority
cookie from the host's current `$DISPLAY`, re-keyed under a wildcard family
so it authenticates regardless of the container's hostname. `docker-compose.yml`
now mounts that cookie to `/root/.Xauthority` and sets `XAUTHORITY` for both
services. Run the script once per X session before `docker compose up/run`
(re-run after an X session restart — the cookie rotates on login).

Verified by opening a bare `gzclient` in a fresh container and confirming
its window actually appeared on the host X server (`xwininfo -root -tree`
showed the full Gazebo widget tree, no auth error in the log).

**Also:** updated `docs/troubleshooting.md`'s existing X11 bullet, which
recommended `xhost +local:docker` — that permanently loosens host X access
control instead of scoping a single cookie file, and doesn't get mentioned
again here for that reason.

## 2026-09-06 — Isaac Sim GUI never opened: the container is not root

**Problem:** `docker compose up -d isaacsim` reported `healthy` and the ROS 2
bridge loaded, but no Isaac Sim window ever appeared on the host.

**Root cause:** the previous entry's X11 fix assumed *both* containers run as
root. That is false for `isaacsim`. NVIDIA's `tools/docker/Dockerfile` ends
with `USER isaac-sim` (uid 1234, home `/isaac-sim`), so mounting the Xauthority
cookie at `/root/.Xauthority` put it somewhere uid 1234 cannot even traverse:

    $ docker exec isaacsim ls -la /root/.Xauthority
    ls: cannot access '/root/.Xauthority': Permission denied

Kit does not treat that as fatal. It logs `GLFW initialization failed`,
`failed to open the default display`, `Failed to acquire IWindowing interface`
and `IAppWindow::startup failed`, then runs on windowless. The image's
healthcheck only greps the Kit log for `AppReady`, so `docker ps` still says
`healthy` — the failure is completely silent from the outside.

**Fix:** mount the cookie at `/tmp/.docker.xauth` for the `isaacsim` service
and point `XAUTHORITY` there (`/tmp` is chowned to `isaac-sim` by the image).
`tb3_ros` genuinely does run as root, so it keeps `/root/.Xauthority` — the two
services now mount the same cookie at deliberately different paths, and the
compose comments say why. Verified: uid 1234 can now stat and read all 134
bytes of the cookie, where before it got EACCES.

Also corrected the "both containers run as root" claim in `docs/setup.md`,
`docs/troubleshooting.md` and `docker/x11-auth.sh`'s header, and documented the
healthy-but-windowless symptom so the next person recognises it.

**Concept — silent GUI failure vs. loud one.** `gzclient` aborts with SIGABRT
when it can't authenticate; Kit shrugs and keeps going. Same root cause, but
only one of them tells you. Whenever a GUI container reports healthy and shows
nothing, read the app's own log before touching DISPLAY or xhost.

## 2026-09-06 — Imported TB3 asset had no geometry at all

**Problem:** `tb3_world.usd` opened cleanly in Isaac Sim and rendered nothing.

**Root cause:** the expanded URDF points at its meshes by
`package://turtlebot3_description/meshes/...`, and the isaacsim container has no
ROS installed and therefore no such package anywhere on disk. `import_tb3.py`
never passed `ros_package_paths`, so `urdf_usd_converter` fell back to resolving
each URL as a *bare relative path* against the URDF's own directory
(`/scenes/meshes/bases/burger_base.stl`), found nothing, and emitted the link
anyway — an empty `Xform` carrying the correct transform and material binding
but no geometry. No error, no warning that survived to the console.

What kept this hidden: the inline `<collision>` primitives (`<box>`,
`<cylinder>`) have no external file, so they imported perfectly and the prim
tree looked fully populated. But collision geometry imports with
`purpose = "guide"`, which is not drawn. So the stage had a plausible-looking
hierarchy, correct joints, correct physics — and nothing visible.

Evidence, before vs after:

| | before | after |
|---|---|---|
| `geometries.usd` in the asset package | absent | 2.0 MB |
| robot mesh prims | 0 | 4 |
| `burger_base` points | — | 48040 |
| robot subtree bounds | FLT_MAX sentinels (empty) | 138 x 178 x 191 mm |

138 x 178 x 192 mm is the published size of a real Burger, so the fixed asset is
dimensionally right, not merely non-empty.

**Fix:** `docker cp` the `turtlebot3_description` share dir out of the tb3_ros
image onto the shared `isaac/scenes/` mount (gitignored, 40 MB), and pass
`ros_package_paths=[{'name': 'turtlebot3_description', 'path':
'/scenes/turtlebot3_description'}]`. Resolution is `path / relative_path`, so
`path` must be the package directory itself — pointing at its parent fails
identically and just as quietly.

Added `isaac/scripts/verify_asset.py` as a gate: it composes a stage and exits
non-zero when the robot subtree has no renderable geometry. Confirmed it fails
on the old broken asset and passes on the new one, so it isn't vacuous.

**Concept — two traversal traps in one script.** Writing that checker surfaced
both. (1) The importer marks visual meshes `instanceable = true`, so a plain
`Stage.Traverse()` stops at the instance boundary and reports *zero* meshes on a
perfectly good asset; `Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies())`
is required. (2) Bounding the whole stage is not a real check, because the
ground plane alone makes it non-empty — the robot subtree has to be bounded on
its own. A validator that gets either wrong is worse than none, since it reports
green on exactly the bug it exists to catch.

**Concept — FLT_MAX as the empty-bounds tell.** USD writes an inverted-infinite
range (`3.4028235e38` min, `-3.4028235e38` max) for geometry it cannot measure.
Seeing that in `extentsHint` is the fastest way to spot a mesh-less asset without
opening the viewport.

## 2026-09-06 — Consolidated Isaac Sim launcher; cross-container DDS fixed

**Change:** `isaac/scripts/build_scene.py` is gone, replaced by
`isaac/scripts/tb3_sim.py`. The old script built an OmniGraph into a stage and
told you to press Play by hand. The new one *is* the simulator launcher —
`SimulationApp` boots Kit, it opens the stage, builds the graph, attaches the
lidar and calls `app_utils.play()` itself — which is what the Isaac Sim
standalone examples (`carter_stereo.py`, `rtx_lidar.py`) actually do. It is now
the compose default command, so `docker compose run --rm isaacsim` starts the
simulator the way `gazebo.launch.py` starts gzserver/gzclient.
`backend:=isaacsim` remains attach-only and unchanged.

Three graph bugs were found by diffing the old file against Isaac Sim's own
reference graph in `isaacsim.ros2.nodes/python/tests/test_differential_base.py`
(`add_differential_drive`), which is the closest thing in the tree to our
contract:

1. `OnPlaybackTick` is `omni.graph.action.OnPlaybackTick`, not
   `isaacsim.core.nodes.OnPlaybackTick`.
2. `ROS2SubscribeTwist` outputs `vectord[3]`; `DifferentialController` takes
   scalar `double`. They cannot be connected directly — two
   `omni.graph.nodes.BreakVector3` nodes are required (`x` for linear, `z` for
   angular).
3. Target prims must be `usdrt.Sdf.Path(...)`, not plain strings, and they
   should point at `base_footprint` (where the importer put
   `PhysicsArticulationRootAPI`) rather than the reference prim above it.

**Problem:** with all that correct, `ros2 topic list` from `tb3_ros` showed every
Isaac Sim topic, `ros2 topic info --verbose` reported a healthy RELIABLE
publisher with a real GID — and `ros2 topic echo` hung forever. Zero messages.

**Root cause:** the two containers run as different uids. isaacsim is uid 1234
(NVIDIA's Dockerfile ends `USER isaac-sim`), tb3_ros is root. Fast-DDS creates
its `/dev/shm` segments with mode **0700**, so neither participant can open the
other's. Discovery rides UDP multicast and works perfectly; the data path
negotiates shared memory and silently delivers nothing. `ipc: host` was set
correctly the entire time and `/dev/shm` genuinely was shared — which is exactly
why this survived earlier debugging, including the unresolved
`RobotDefinitionReader` note from the previous session.

**Fix:** `docker/fastdds_udp_only.xml`, wired into both services via
`FASTRTPS_DEFAULT_PROFILES_FILE`. Confirmed by pointing that variable at the XML
for a single failing `ros2 topic echo` — data appeared instantly. Measured
after: `/clock` 81 Hz, `/odom` 76 Hz, `/tf` 68 Hz, `/joint_states` 60 Hz
headless. `/cmd_vel` verified by driving the robot 0.276 m.

**Concept — "connected" is not "delivering".** Every diagnostic short of moving
an actual byte was green: topic present, publisher present, QoS compatible,
matching domain, shared `/dev/shm`. DDS splits discovery from data transport,
so a transport that cannot open its buffers looks identical to an idle
publisher. `ros2 topic echo` is the only one of those checks that proves
anything; treat the rest as necessary, never sufficient.

**Concept — a dropped Python reference silently kills a sensor.** Writing
`LidarSensor(lidar, annotators=[]).attach_writer(...)` attaches the writer and
then lets the object be garbage collected, taking the render product with it.
`/scan` was never advertised and nothing logged a complaint. Binding it to a
name that outlives the run loop fixed it.

**Concept — RTX sensors need a real render path.** Under `--headless` the lidar
writer attaches and `/scan` is advertised but not one message is produced, while
every other topic behaves normally. A headless smoke test therefore reports a
healthy bridge and a dead lidar as the same thing.

**Open — the lidar is the wrong sensor.** `Example_Rotary_2D`'s single emitter
has `elevationDeg = [-2.0]`, i.e. it points 2 degrees *down* and scans the
floor. On an empty ground plane it still returns 652 of 3600 rays as hits at
2.0-3.5 m (a partial arc, spread rather than a clean circle because the robot
rests slightly pitched) — a phantom ring Nav2 would treat as obstacles. It also
reports no-return as `-1.0` rather than Gazebo's `inf`. Re-rating the config to
5 Hz / 360 samples advertises the right geometry but makes publishing erratic
and trips `Multi-tick is enabled but motion BVH is not active`, so the real fix
is authoring an LDS scan pattern rather than bending this one.

## 2026-09-06 — `ipc: host` was load-bearing for nothing, and cost us startup

**Problem — Isaac Sim aborted before Kit started.** `docker compose run --rm
isaacsim` died on
`Failed to create shared memory named carb-RStringInternals-62`. carb names that
segment after its own PID, which under `python.sh` is deterministically 62. With
`ipc: host` the name lived in the host's `/dev/shm`, which is sticky and shared;
a root-owned leftover sat at exactly that name, and this container is uid 1234,
so it could neither reuse nor unlink it. The PID repeats, so a single stale file
poisons that slot forever — the failure presents as a broken image, not as
stale state, which is why it looked unrecoverable.

**Fix — drop `ipc: host` from the isaacsim service.** A private IPC namespace
makes the collision structurally impossible rather than merely cleaned up.
Verified: the sim reached `PLAYING`, and from `tb3_ros` across the container
boundary `/clock` 30 Hz, `/tf` 30 Hz, `/joint_states` 30 Hz, `/odom` 28 Hz. The
run left *zero* new segments in the host `/dev/shm`, which is the actual proof —
the container's segments now live and die in its own namespace.

**Concept — a comment can outlive the design it describes.** `docker-compose.yml`
insisted both services needed a shared `/dev/shm` "or discovery works and data
silently does not." That was true when written, but `docker/fastdds_udp_only.xml`
was added later and disables the SHM transport outright
(`useBuiltinTransports=false`, UDPv4 only) precisely because the uid split made
SHM unusable. From that moment the shared `/dev/shm` carried no data and only
supplied collisions. Two correct facts, recorded a week apart, silently
contradicting each other. Worth re-reading the *reasons* in config when a fix
lands, not just the settings.

**Concept — no `shm_size` override is needed.** The instinct on removing
`ipc: host` is to compensate with a large `shm_size`, since the private default
is 64 MB. NVIDIA's own `tools/docker/run_docker.sh` passes `--network=host`
with no `--ipc` and no `--shm-size`, so Kit is known to run on the default, and
it did.

**Concept — Isaac Sim bundles *both* Humble and Jazzy, so `ROS_DISTRO` selects,
it does not rescue.** `docker-compose.yml` justified `ROS_DISTRO=jazzy` with
"the bundled distro is Jazzy, not Humble." That is wrong:
`/isaac-sim/exts/isaacsim.ros2.core/` contains `humble/` *and* `jazzy/`, and
with `LD_LIBRARY_PATH` pointed at either one, every dependency of
`librmw_fastrtps_cpp.so` resolves — Humble would probably work too. The
extension defaults to `ros_distro = "system_default"`, i.e. it reads
`ROS_DISTRO` and picks a directory; unset, it resolves nothing and the bridge
dies. Jazzy is the right pick because the image is Ubuntu 24.04 (noble) and
Jazzy is noble's distro, but it is a choice, not a constraint. Confirmed in the
boot log: `Attempting to load internal rclpy for ROS Distro: jazzy`.

**Open — cross-distro means no type-hash safety net.** Jazzy publishes message
type hashes in discovery; Humble does not understand them. The standard message
types we use are unchanged between the two, so the wire bytes match — but if a
definition ever did diverge, the result would be silent garbage rather than a
mismatch error, in keeping with everything else in this stack.

**Open — observed topic rates are well below the previously recorded ones.**
`docs/status.md` records `/clock` 81 Hz, `/odom` 76 Hz, `/tf` 68 Hz,
`/joint_states` 60 Hz; this run measured roughly half that (30/28/30/30), and
`/scan` came in at 0.66 Hz against a configured 10 Hz. The machine was under
load from other containers, so this is not necessarily a regression and was not
investigated. Re-measure on an idle host before drawing any conclusion.

## 2026-09-06 (later) — one manifest, two simulators

**Goal.** Stop hardcoding environments. `gazebo.launch.py` pointed at
`turtlebot3_gazebo`'s installed `turtlebot3_world.world` and Isaac Sim had no
environment at all, so adding a second world meant editing launch files and the
two backends shared no definition that could keep them in agreement.

**Concept — no world ships in both formats, so share the geometry instead.**
The instinct is to hunt for an environment distributed as both SDF and USD.
There isn't one (checked; the AWS RoboMaker warehouse is Gazebo-only, NVIDIA's
warehouses are USD-only). The workable definition of "the same asset" is a
shared *mesh* plus a manifest, with each simulator's wrapper generated from it.
That also happens to be exactly what a scanned office will need, since a scan
is a mesh and both simulators consume meshes — so the pipeline generalises,
whereas an SDF parser would have been thrown away.

**Design.** `worlds/<name>/world.yaml` is the source of truth.
`scripts/build_world.py` generates the Gazebo `.world` (+ `model.config`, so
`model://` resolves off `/worlds` on `GAZEBO_MODEL_PATH`);
`isaac/scripts/build_world_usd.py` generates the Isaac `.usd`. Meshes under
`meshes/` are shared byte-for-byte. `tb3_bringup/worlds.py` resolves the
registry for the ROS side. The registry is a plain directory tree, not an ament
package, because the isaacsim container has no ROS and cannot call
`get_package_share_directory`; both containers mount it at `/worlds`.
`turtlebot3_world`'s 15 bodies were derived mechanically from upstream's
`model.sdf` rather than retyped.

**Concept — `world:=` is refused for `backend:=isaacsim` rather than ignored.**
Isaac Sim runs in its own container and bringup only attaches to it over DDS, so
accepting the argument would imply a control the ROS side does not have. The
error names `WORLD=` instead. Silently ignoring it would have been the kind of
thing that costs an hour later.

**Problem — the bounds check earned itself back on the first run.** The manifest
carries a `verify.bounds_*` block computed analytically from the meshes and
placements, and the generator fails if the assembled stage disagrees. It caught
two bugs immediately, both of which produce a stage that loads clean, renders
something plausible, and reports no error:

1. *Collada up-axis.* `wall.dae`/`hexagon.dae` declare `up_axis Y_UP` but lay
   their vertices out Z-up (hexagon in XY, extruded along Z). Gazebo ignores the
   label and draws them upright. The converter faithfully copies both vertices
   and label, so referencing the result into a Z-up stage made
   `add_reference_to_stage` insert a corrective 90-degree X rotation — tipping
   the Isaac arena on its side while Gazebo stayed correct. Fixed with
   `convert_stage_up_z`, which a probe showed moves no geometry at all and only
   rewrites metadata. Units were never the problem: the converter applies
   `<unit>` correctly (raw 450 -> 11.43 m).
2. *Poses silently dropped.* Referencing brings the converted asset's own
   `translate/orient/scale` onto the prim, and `XformCommonAPI` cannot author a
   `rotateXYZ` over an `orient` — it returns **False rather than raising**. Every
   mesh body sat at the origin at asset scale. The pose now goes on a wrapper
   Xform with the reference on a child, and the return values are checked.

**Concept — a cache keyed on mtime hides a flag change.** After fixing (1), the
rebuild reused the stale converted meshes and "failed" identically, because the
cache compares mtimes and the converter *flags* had changed, not the inputs.
Worse, the cache directory was created by the container as uid 1234 and could
not be cleared from the host, so the obvious fix also failed. The cache dir is
now chmod 777 by its creator. When a rebuild reproduces a bug you just fixed,
suspect the cache before the fix.

**Concept — confine the uid-1234 output, don't chmod the tree.** The isaacsim
container cannot write into a checkout owned by you, and git records no
directory modes, so this cannot be fixed by committing anything. Build products
go in `worlds/<name>/isaac/`, created world-writable by
`scripts/build_world_usd.sh`. Files inside end up owned by 1234 but remain
deletable, because deletion depends on the *parent* directory being writable by
you — which is why this beats chmod-ing the world directory.

**Concept — `SimulationApp.close()` hard-exits.** It swallowed both the
traceback and the exit status, so a failing generator looked exactly like a
successful one: silent, status 0. Any standalone Isaac script needs to catch,
print, flush and force the status itself before closing.

**Verified.** Same manifest, both backends. Gazebo: `get_model_list` shows
`turtlebot3_world`, `/scan` 324/360 finite, 0.511-3.357 m. Isaac Sim: stage at
`/World/env`, robot spawned at the manifest's `(-2.0, -0.5)`, `/scan` 3372/3600
positive, 0.526-3.497 m (it was 652/3600 phantom hits on the bare ground plane).
Closest return agrees to 1.5 cm across backends from the same spawn — the real
parity evidence. Generator self-checks 15 colliders for 15 bodies, all mesh
approximations `none`, 0 rigid bodies. SDF generation is idempotent.

**Open — the `/scan` gap between backends is the lidar, not the world.** Isaac
publishes 3600 rays to Gazebo's 360 and uses `-1.0` for no-return where Gazebo
uses `inf`. Unchanged by this work; authoring a real LDS scan pattern is still
the fix, and it is still what stands between here and Nav2.

## 2026-09-08 — The X cookie was a directory: same windowless Kit, new cause

**Problem — "isaacsim is up but no window appeared", again.** The container was
healthy and fully functional: stage loaded at `/World/env`, physics playing,
`/clock /odom /joint_states /scan` publishing, `/cmd_vel` subscribed. Only the
GUI was missing. The log had the familiar trio — `Authorization required, but
no authorization protocol specified`, `GLFW initialization failed`,
`IAppWindow::startup failed` — plus `Cannot setup ExternalDragDrop without a
default window` and `Hotkeys cannot be setup without a default window`.

**Fix — the cookie path was a root-owned *directory*, not a file.**
`/tmp/tb3_sim2real.docker.xauth` was `drw-r--r-- root:root`. `x11-auth.sh` had
not been run for this X session, so when `docker compose` went to bind-mount
that path it did what Docker always does with a missing bind-mount source: it
created a root-owned directory there. `XAUTHORITY` inside the container then
pointed at a directory, no cookie was ever presented, and Kit went windowless.
Timestamps confirmed the order — directory created 18:32:45Z, container started
18:33:20Z. Remedy is `docker stop`, `sudo rm -rf` the directory, re-run
`x11-auth.sh`, restart.

**Concept — the documented ordering was load-bearing and nothing enforced it.**
"Run this once per X session *before* `docker compose up/run`" was written in
three places and was still only a comment. Skipping it does not fail loudly; it
silently manufactures the exact condition that breaks the GUI, and it is
self-perpetuating, because re-running the script afterwards cannot overwrite a
root-owned directory — `touch` just fails. `x11-auth.sh` now refuses to run
when the path is a directory (or an unwritable file) and prints the four-command
recovery, so the trap explains itself instead of surfacing as a windowless run
half an hour later.

**Concept — re-keying gives the cookie a new inode.** `xauth nmerge` writes a
temp file and renames it into place. A container already running keeps the old
inode through its bind mount, so a re-key never reaches it. Restarting the
container is part of the fix, not cleanup; the success message says so now.

**Concept — Kit's healthcheck cannot see this class of failure.** It greps the
log for `AppReady`, which appears whether or not a window was created, and the
X auth failures are logged as *warnings*. So `docker ps` says healthy and the
ROS topics all work — every signal short of looking at your screen says fine.
Both known causes of a windowless Kit (wrong cookie path for uid 1234, and now
a directory in place of the cookie) present identically, which is why the log
grep for `GLFW initialization failed` is the diagnostic that matters.
