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

## 2026-09-08 (later) — Nav2 running: stock map + RViz view, and two bugs on the way

**Concept — the params were already correct; only the map and the view were
missing.** The instinct was to re-copy `nav2_params.yaml` from
`turtlebot3_navigation2`. Checking first showed `config/nav2_{gazebo,isaacsim,
real}.yaml` were already byte-identical to
`turtlebot3_navigation2/param/humble/burger.yaml` — and identical to each
other, the seeded state `status.md` describes. Worth noting *which* upstream
file that is: the package ships both `param/burger.yaml` and
`param/humble/burger.yaml`, its own launch picks the latter on Humble, and they
differ (12822 vs 9839 bytes). Copying the wrong one is a silent downgrade. So
this reduced to `maps/` (which did not exist) and `rviz/tb3.rviz` (a stub).

**Concept — which frame the stock map is in, decided from the map, not from
folklore.** `map.yaml` gives origin `[-10, -10]` at 0.05 m/px, which says
nothing about where the arena sits. Reading the `.pgm` and converting occupied
cells to world coordinates put them at x[-2.95, 2.65], y[-2.60, 2.55], centred
on `(-0.15, -0.03)`. The arena is centred on the world origin, so **the map
frame is the Gazebo world frame**, and the initial pose is the manifest's spawn
`(-2.0, -0.5)`, not `(0, 0)`. (Had the map been SLAM'd from the robot's start,
the arena would have sat at `(+2.0, +0.5)` instead.) `nav2_bringup`'s
`tb3_simulation_launch.py` defaults agree — `x_pose -2.00`, `y_pose -0.50`.
Confirmed live afterwards: `map->odom` settled at `(0.033, 0.030)`, i.e. the
two frames coincide.

**Concept — initial pose by topic, no RViz click needed.** Publishing
`/initialpose` (`geometry_msgs/PoseWithCovarianceStamped`, `frame_id: map`) is
exactly what the "2D Pose Estimate" button does, so the whole bringup is
scriptable. Use `--times 5 -w 1`: AMCL's subscription is volatile, so a single
`--once` can be published before discovery completes and land nowhere. The
alternative is AMCL's own `set_initial_pose: true` + `initial_pose.{x,y,z,yaw}`
parameters, which skips the topic entirely — not used here, to keep the seeded
params byte-identical to upstream. Goals go the same way, to `/goal_pose`
(`PoseStamped`); `bt_navigator` subscribes and forwards to its own action.

**Problem — `backend:=gazebo` alone could never have worked.** The documented
invocation died with `world '' has no generated .world`.
`IncludeLaunchDescription` does not isolate launch configurations: the parent's
`world`, declared with `default_value=''` to mean "let the backend choose",
leaked into `backends/gazebo.launch.py` and beat its
`default_value='turtlebot3_world'`, because `DeclareLaunchArgument` only fills
in a value that is not already set. The empty-means-default comment in
`bringup.launch.py` was describing an intention the code did not implement.
Fixed by wrapping each include in `GroupAction(scoped=True, forwarding=False)`,
which fixes the whole class rather than this one argument.

**Problem — the stock RViz config is newer than this Nav2.** It lists
`nav2_rviz_plugins/Selector` and `.../Docking` panels; Humble's
`nav2_rviz_plugins` declares only `Navigation 2`, so RViz threw two
`PluginlibFactory` errors at startup and rendered both panels as error text
down the right-hand side. Dropped from the config.

**Verified.** Clean launch, no errors: `ros2 launch tb3_bringup
bringup.launch.py backend:=gazebo nav:=true rviz:=true`, with no `world:=`.
Gazebo GUI at RTF 1.00 / 62.5 FPS, RViz with Global Status Ok and Fixed Frame
`map`, all lifecycle nodes `active [3]`. Published `/initialpose` at the spawn;
AMCL converged to `(-1.967, -0.469)`. Goal `(2.0, 0.0)` -> stopped at
`(2.003, -0.009)`, ~9 mm out. Goal `(1.5, 1.5)` -> `controller_server: Reached
the goal!`, `bt_navigator: Goal succeeded`, RViz panel `Feedback: reached`,
0 recoveries.

**Concept — screenshotting this host needs two homemade tools.** No ImageMagick
and no `xdotool`/`wmctrl`, and PIL ships no XWD plugin, so verifying "does the
GUI actually appear" needed a small XWD->PNG parser plus an EWMH
`_NET_ACTIVE_WINDOW` raiser (plain `XRaiseWindow` is overridden by mutter).
Without raising, `xwd -id` on an occluded window captures whatever is drawn on
that screen region — the first "RViz" capture was Gazebo's viewport. Both live
in the session scratchpad, not the repo; rebuild them if this comes up again.

## 2026-09-08 (later) — The TurtleBot3 would not sit still: a restitution-0.8 floor

**Problem — the robot bounced and rocked on its caster with nothing driving
it.** Reported from the GUI, reproduced headless on the bare ground plane with
no ROS graph, no lidar and no `/cmd_vel` at all: 3.58 deg peak-to-peak in
pitch, pitch rate to +/-0.7 rad/s, and 7.7 mm of drift in seven seconds. Not a
tuning problem — two defaults nobody chose, stacked:

1. `isaacsim.core.api.objects.GroundPlane` authors *its own* physics material
   at **restitution 0.8** when it is not handed one. It is right there in the
   source, in a branch commented "set default values if no physics material
   given" (0.5 / 0.5 / **0.8**).
2. The URDF importer binds **no** physics material to any collider, because the
   URDF has nothing to say about friction or restitution. So every surface on
   the robot came up on PhysX's fallback.

PhysX combines restitution as an *average* by default, so every contact under
the robot was a 0.4 — a bouncy-ball floor under a 0.94 kg robot.

**Concept — why it hits a burger so much harder than it should.** The caster is
not decoration and it never leaves the ground. The imported base has its centre
of mass 4.3 mm *behind* the wheel axle, while `caster_back_link`'s 30 x 9 x 20
mm skid clears the floor by 0.5 mm (skid centre 5 mm up, half-height 4.5 mm
after the URDF's -90 deg roll). The chassis therefore tips back onto that skid
and rests there permanently, exactly as the real robot does. A permanently
loaded elastic contact under a rear skid is a rocking chair. The measured rest
pitch after the fix, -0.299 deg, matches the geometry: atan(0.5 / 81) = 0.354
deg, less contact penetration.

**Fix.** `import_tb3.py` now authors three physics materials and binds them for
the *physics* purpose (`material:binding:physics` — a plain `Bind()` writes the
render binding, which PhysX never reads, and nothing reports it):

- `floor`, mu 1.0, handed to `GroundPlane` so the 0.8 is never authored. Bound
  on the `/World/GroundPlane` prim as well, because `GroundPlane` binds what it
  is given only to its mesh collider and leaves the `collisionPlane` beside it
  on the fallback.
- `wheel`, mu 1.0, on both wheel colliders. turtlebot3_gazebo's `model.sdf`
  gives the tyres `mu = mu2 = 100000` — "must not slip", with upstream's own
  comment that the number is not real data. PhysX takes a coefficient, so 1.0
  is the honest spelling of the same intent.
- `chassis`, mu 0.1 with `frictionCombineMode = min`, bound on the articulation
  root so everything `merge_fixed_joints` folded into it inherits: the caster
  skid, the body box, the lidar cylinder. A high-friction skid fights the
  wheels on every in-place turn. (Gazebo dodges the question by making
  `caster_back_joint` a *ball* joint, so its caster rolls where ours slides.)

Restitution is 0 on all three, with `restitutionCombineMode = min` rather than
the default average, so "does not bounce" holds against whatever the other
collider brings instead of being averaged back up by it — which is precisely
how the ground plane's 0.8 reached the wheels. That also covers the world's
walls and pillars, which carry no material of their own.

**Verified.** Same headless measurement on the regenerated stage: pitch
peak-to-peak **0.000 deg**, z **0.000 mm**, horizontal drift **0.00 mm** over
seven seconds. It settles at -0.299 deg nose-up on the skid and freezes. Nothing
else was needed — scene stabilization, contact/rest offsets and solver
iteration counts were all measured as candidates and none of them was the cause
(the articulation defaults are already 32 position / 1 velocity iterations).

**Problem — a crash in `tb3_sim.py` looked exactly like a clean exit.** Found
while chasing the above. `main()` ran under `try/finally: os._exit(0)`, and
`os._exit` ends the process before Python reports the exception: a missing
stage, a bad prim path or a failed precondition printed nothing and exited 0.
The traceback is now printed explicitly and the status is 1. This mattered
immediately, because the new `check_surfaces()` guard — which refuses to start
on a stage whose colliders have no surface properties, since a stale
`tb3_world.usd` otherwise reads as a physics-tuning problem — would itself have
failed silently.

**Problem — re-importing left the old asset behind.** The URDF importer does
not overwrite: handed an existing `turtlebot3_burger.usd/` it writes
`turtlebot3_burger_1/` *inside* it and returns that, leaving the previous copy.
Every re-import kept working, so nothing pointed at the growing pile, and only
the printed `Robot asset:` line said which copy the new `tb3_world.usd`
referenced. `import_robot()` now clears the directory first.

**Concept — two Isaac Sim containers left over from a `timeout`-killed run make
everything look like a hang.** `timeout N docker compose run ...` kills the
compose *client*; the container keeps running and keeps the GPU. Two of them
accumulated during the sweep above and a normal 90-second startup stretched
past six minutes, which read as a deadlock in the code being tested. Check
`docker ps` before believing a Kit hang.

**Verified through ROS as well.** `tb3_sim.py` in `turtlebot3_world`, no Nav2,
no teleop, nothing publishing `/cmd_vel`: `/odom` position moved 3.4e-11 m over
ten seconds, and `/odom` twist sat at ~1e-5 m/s and ~1e-3 rad/s — numerical
noise. `check_surfaces()` passed on the regenerated stage without comment.

## 2026-09-13 — A locally built Isaac Sim retired a design on a fact that was not true

**Problem — "Isaac Sim cannot run on Ubuntu 22.04" was measured on the wrong
binaries.** On 2026-09-06 the single-container design was built, tested and
rejected: `verify.sh` scored `isaacsim6-humble` 3/5 because nine of 1104 shared
objects under `/isaac-sim/exts` needed `GLIBC_2.38` against jammy's 2.35, three
of them the ROS 2 bridge. `docs/architecture.md` recorded it as a property of
Isaac Sim. It was a property of the *build host*: the image under test,
`isaac-sim-docker:latest`, came from a local build of `~/isaacsim-6.0` compiled
on this noble machine. NVIDIA's released tarball is compiled to a lower floor —
across all 3189 shared objects in `isaac-sim-standalone-6.0.0-linux-x86_64` the
ceiling is `GLIBC_2.35`, exactly jammy, with the three bridge libraries at 2.34.
That floor is what the Isaac Sim README means by "Ubuntu 22.04/24.04".

**Fix — swap only the binaries, and the 3/5 becomes a pass.** Mounting the
official tree over `isaacsim6-humble:latest` with `ISAACSIM_PATH` pointed at it,
same jammy, same ROS, same `ros-isolate`: `isaacsim.ros2.core`, `.nodes` and
`.bridge` all started, and `ros2 topic echo /clock` from the system Humble in
the *same container* returned sim time. Second run on a writable tree: no errors
at all.

**Concept — mount a vendor tree read-only and Kit degrades quietly.** The first
run logged `HydraEngine rtx failed creating scene renderer`, `Cannot find
Documents/Kit/shared`, and `Python node cache update ... Aborting Python node
registration` — all caused by `:ro` on `/isaac-sim`, all gone on a writable
mount. The clock graph still worked because its nodes are C++; a graph using
Python-backed OGN nodes would have failed instead, and nothing would have said
why. Whatever ships must keep `/isaac-sim` writable.

**Concept — Isaac Sim 6 probes for *your* rclpy before using its own.** The
startup line `Attempting to load system rclpy` is not a warning sign; it is the
`use_internal_libs:=false` / `ros_installation_path:=` feature looking for a
user ROS install. We cannot take that branch — Kit is Python 3.12, Humble's
`rclpy` is `cpython-310` — so it falls back to the internal Humble build, which
is the default and what we want. The corollary: if custom message types ever
need to exist inside Kit, `IsaacSim-ros_workspaces`'
`ubuntu_22_humble_python_312_minimal.dockerfile` is the mechanism, because it
rebuilds Humble against 3.12 so `ros_installation_path` can point at it. That is
what that repo is *for*; it does not build a ROS userland you can `ros2 launch`
from.

**Concept — `nvcr.io/nvidia/isaac-sim` is not anonymously pullable.** NGC issues
an anonymous token but returns 401 on the manifest: it needs `docker login
nvcr.io` plus a one-time licence acceptance in a browser for the same NGC org.
The `docker/isaacsim-ros2/` Dockerfiles now default `ISAACSIM_IMAGE` to that
release anyway, with a documented escape hatch for wrapping an on-disk tree,
because the alternative — depending on whatever is tagged `isaac-sim-docker:latest`
on one machine — is what caused the error at the top of this entry.

**Design note.** The `turtlebot3_isaacsim` proposal that follows from this:
<https://claude.ai/code/artifact/e83a0463-ff46-4f38-b227-c025cd4b5a7e>

## 2026-09-13 (later) — 5/5 on the real artifact

`nvcr.io/nvidia/isaac-sim:6.0.1` pulled (NGC login required; anonymous returns
401 on the manifest), bridge libraries confirmed `GLIBC_2.34`, then
`docker build -f docker/isaacsim-ros2/Dockerfile.humble -t isaacsim6-humble:ngc`
and `verify.sh` unmodified:

    PASS  ROS 2 environment, RViz, Nav2 and the shared tooling
    PASS  Gazebo, and the ros_gz bridge
    PASS  ros-isolate strips ROS 2 and keeps the DDS settings
    PASS  Kit starts and reports its version
    PASS  ROS 2 bridge publishes /clock to the system ROS 2 installation
    5 passed, 0 failed, 0 skipped

Isaac Sim 6.0 and ROS 2 Humble in one Ubuntu 22.04 container, every part a
vendor artifact. The September 3/5 was the local build and nothing else.

**Still cold-start bound.** Step 5 took most of its 180-second budget because
`--rm` discards `/root`, where Kit writes its caches. That is the argument for
the cache volumes, not a defect — but it means `verify.sh` is close to timing
out on a cold machine and the poll budget should probably grow before anyone
relies on it in CI.

## 2026-09-13 (later still) — the whole interface contract, one container

`tb3_sim.py` mounted unchanged into `isaacsim6-humble:ngc`, `--headless`, and
every topic in the contract came up:

    /scan           6.034 Hz      tf odom -> base_footprint  present
    /odom          34.120 Hz      /cmd_vel                   advertised
    /joint_states  38.479 Hz      [Error] lines in sim log:  none
    /clock         37.792 Hz

So it is not just the bridge: the RTX lidar renders headless with no display,
the differential drive publishes, and the raw `odom->base_footprint` transform
is the one the design expects. Nothing in the script needed changing to move
from the noble container to the jammy one.

**Two numbers worth remembering.** `/scan` was advertised 166 seconds after
start, cold — the third measurement in a row pointing at cache volumes. And
`/clock` advances at ~38 Hz against a 1/60 s physics step, so the sim runs at
roughly 0.6x real time on this box with the lidar on. That is a Nav2 tuning
input, not a fault, and `use_sim_time` already covers correctness.

## 2026-09-13 (later still) — the merge, and two traps in a persistent container

`docker-compose.yml` is one service now. `docker/ros/Dockerfile` is `FROM`
`isaacsim6-humble:ngc` and installs only what the base lacks — audited against
the built image, which turned out to be four things: `joint_state_publisher`,
`cartographer`, `cartographer_ros`, and the TurtleBot3 set. The old
rviz2-instead-of-desktop trick went with it; ~466 MB of tutorials mattered
against a 5 GB image and does not against a 38 GB one.

**Cache paths are not where NVIDIA's compose file says.** Theirs mounts four
`$HOME`-relative paths, correct for an image whose user's home *is* `/isaac-sim`
and wrong here, where `HOME=/root`. Measured by diffing the filesystem across a
cold run:

    493 MB  /isaac-sim/kit/cache       shader cache -- NOT $HOME-relative
    6.3 MB  /root/.nv/ComputeCache     CUDA
    1.4 MB  /var/tmp/OptixCache_root   OptiX, i.e. the RTX lidar

A first attempt at this scan excluded `/tmp` and reported 12 MB of `.pyc` and
nothing else, which looked like evidence that there was no cache worth keeping.
The exclusion was the bug.

**Effect: `/scan` 164-168 s cold, 14 s warm.** Twelve times, and it belongs to
the volumes rather than to `up -d` — `docker compose run --rm` gets the same
warm cache.

**Trap 1 — `docker compose exec` does not run the entrypoint.** Only `run`
does. An interactive shell survives because `/root/.bashrc` sources ROS 2, but
Ubuntu's stock root `.bashrc` returns early when non-interactive, so
`exec tb3_ros bash -c 'ros2 ...'` gets a container with no `ros2` on `PATH` and
fails silently. This cost a whole measurement: two runs reported `elapsed=300s`,
which was not a slow start but the poll loop timing out because every
`ros2 topic list` had failed into `/dev/null`. Scripted use goes through
`docker compose exec tb3_ros /entrypoint.sh <cmd>`. The old `run --rm` workflow
never had this problem, and losing it is a real cost of the persistent
container.

**Trap 2 — `sleep infinity` as PID 1 reaps nothing.** Killed simulators left a
trail of `python3 <defunct>`. `init: true` puts docker-init in front. Both
verified: `/proc/1/comm` is `docker-init`, and the entrypoint form works.

**Aside, for whoever writes the next test script.** `pkill -9 -f "isaac-sim"`
matches the shell running it, because the pattern is in its own command line.
It kills itself, the step produces no output, and the exec exits 137.

---

## 2026-09-13 — the merged container, checked against the list it shipped with

Commit `5735893` left five checks. Three of them had answers today, and all
three are `PASS`. What follows is only what was surprising or worth keeping.

**`backend:=isaacsim` attaches to a simulator in its own container.** The
worry was that the DDS path might behave differently once both endpoints sit
inside one container; it does not. `wait_for_sim` logged
`simulator connected` 8 ms after start. Headless rates read from the ROS side:
`/clock` 60 Hz, `/odom` 60 Hz, `/joint_states` 60 Hz, `/scan` 10 Hz.

**The GUI works, and the cookie has one path now.** `./docker/x11-auth.sh`
rewrites `/tmp/tb3_sim2real.docker.xauth` by rename, so the file has a new
inode and a container started earlier keeps the old one through its bind
mount — the script says so and it is real: `docker compose restart tb3_ros`
was needed before Kit could authenticate. After that, Kit opened a 1440x900
`Isaac Sim Python 6.0.1` window and RViz opened beside it, both from the same
container through the same `/root/.Xauthority`.

**Drawing the viewport halves the lidar rate.** `/scan` is 10 Hz headless and
5.3 Hz with the GUI up, same scene, same machine. Worth knowing before anyone
reads a rate off a GUI run and files a lidar bug.

**Gazebo Classic is unharmed by sharing the box with Isaac Sim.** gzserver and
gzclient start, the robot spawns at the manifest pose, `/cmd_vel` at 0.15 m/s
for ~5 s moved it 0.8 m. One GPU, one X display, two very different renderers,
no conflict — though nothing has yet asked both simulators to run *at the same
time*, and nothing should.

**AMCL's silence is AMCL, not the merge.** `map->odom` never appears until an
initial pose arrives, so `tf2_echo map base_footprint` fails with
`Invalid frame ID "map"` and the global costmap logs `Timed out waiting for
transform` forever. One `/initialpose` at the spawn pose fixed it and AMCL
localised to `[-1.95, -0.48]` against a spawn of `[-2.0, -0.5]`. A single
`ros2 topic pub -1` was NOT enough — the message lands before AMCL's
subscription is matched and is dropped. `-r 2` for a few seconds works.

**Trap 3 — a killed `docker compose exec` leaves its processes running.**
`timeout 40 docker compose exec -T ... ros2 launch ...` kills the *client*;
the launch keeps running inside the container. Two orphaned `ros2 launch`
trees were publishing `robot_state_publisher` into the domain before this was
noticed, which is exactly the kind of thing that makes a later measurement
lie. Without a TTY there is no signal forwarding, so scripted runs must clean
up inside the container: `docker compose exec -T tb3_ros pkill -INT -f
"ros2 launch"`.

---

## 2026-09-13 — shared-memory DDS back on, and `/dev/shm` was never private

**Change:** removed `FASTRTPS_DEFAULT_PROFILES_FILE` and the
`docker/fastdds_udp_only.xml` mount from `docker-compose.yml`, and deleted the
file. Fast-DDS is on its default transports again, shared memory included.

**Why it was safe:** UDP-only existed because two containers ran as different
uids (1234 and root) and could not open each other's `/dev/shm` segments —
discovery worked over UDP multicast and the data path silently did not. One
image means one uid, so the failure has no way to occur.

**Result:** nothing changed except that the segments exist.
`/dev/shm/fastrtps_<hash>` appears at 537 KB, and the rates are identical to
the UDP-only run: `/clock` 60 Hz, `/odom` 60 Hz, `/joint_states` 60 Hz, `/scan`
10 Hz, with `ros2 topic echo /scan` returning a real `ranges` array, AMCL
localising, and `/cmd_vel` moving the robot 0.35 m. At a 360-point scan there
was never any throughput at stake; the point was to delete a workaround whose
reason had expired, and to find out whether anything else had quietly come to
depend on it. Nothing had.

**The finding that matters more than the change.** `docker-compose.yml` claimed
that dropping `ipc: host` gave the container a private `/dev/shm` and made the
`carb-RStringInternals-<pid>` collision "structurally impossible". It does not,
because `- /dev:/dev` is a *recursive* bind mount of the host's `/dev`, and it
covers whatever `/dev/shm` Docker prepared. Measured both ways: 110 identical
entries and the same `df` on both sides, and a file touched at `/dev/shm/x`
inside the container appears at `/dev/shm/x` on the host, owned by root.

So every Kit run has been writing into the host's `/dev/shm` all along — today's
runs left 19 root-owned `carb-RStringInternals-*` files there. The reason this
is not fatal is the uid, not the namespace: as root, Kit creates those files
`0666` and can reuse or unlink its own leftovers. The two-container era was
fatal because uid 1234 met a root-owned leftover. Comments corrected in
`docker-compose.yml` and `docs/troubleshooting.md` rather than changing the
`/dev` mount, which the real robot's USB devices need.

**Also:** `docker compose up -d` recreating the container destroys
`/ws/install` — it is in the container's writable layer, not a volume — so the
first `ros2 launch tb3_bringup` after any recreate fails with
`Package 'tb3_bringup' not found`. `colcon build` is 1.5 s here, so this is a
papercut rather than a problem, but it is a live input to the open item 5:
`docker compose run --rm` would hit it on *every* run.

---

## 2026-09-13 — `up -d` + `exec` wins, and the docs catch up with the merge

**Decision:** the documented way in is `docker compose up -d` once, then
`docker compose exec tb3_ros ...` per terminal. `run --rm` was the incumbent and
had a real case — it runs the entrypoint, so no silent missing-`ros2` trap, and
it starts in 1.2 s off the same warm cache volumes.

**What decided it:** `/ws/install` lives in the container's writable layer, not
in a volume. `run --rm` builds a new container every time, so the workspace
overlay is simply absent — `ros2 launch tb3_bringup` fails `Package not found`
until you `colcon build` again, *per terminal*, because each terminal is a
different container. Measured today, twice, the second time by accident:
`docker compose up -d` recreating the container after the compose edit wiped
`/ws/install` and the next launch failed exactly that way. The other half of it
is that Isaac Sim is a long-lived process you attach terminals to; `exec` joins
the container running it, `run --rm` starts a stranger.

**The trap stays, with a fix on the shelf.** `exec` skips the entrypoint, and
`bash -c 'ros2 ...'` lands in a shell with no `ros2`. `ENV BASH_ENV=/etc/…` in
the image would fix it properly — bash reads `BASH_ENV` for non-interactive
shells — except `/isaac-sim/python.sh` is itself `#!/bin/bash`, so it would
re-source the system ROS *after* `ros-isolate` stripped it and break Kit in the
exact way `ros-isolate` exists to prevent. Safe only if `ros-isolate` unsets
`BASH_ENV` as part of its job, which is arguably where that belongs. Not done.

**`scripts/build_world_usd.sh` was broken by the merge and nobody had run it.**
It called `docker compose run --rm ... isaacsim` — a service that stopped
existing in `776bf55` — and reached Kit through `--entrypoint
/isaac-sim/python.sh`, bypassing `ros-isolate`, which in the merged image means
the system ROS 2 stays ahead of Kit's own paths. Now `run --rm tb3_ros
isaacsim-python`, and deliberately the one command that keeps `run --rm`: a
one-shot batch job with no workspace and no running simulator to attach to.
Re-ran it end to end — 32 s, `verify: OK — Isaac stage matches the Gazebo
world`. Its `chmod 777` is gone with the uid it was for; the generated `.usd` is
now root-owned in the checkout instead, which the header says out loud.

**Docs.** `README.md`, `docs/setup.md`, `docs/troubleshooting.md` and
`docker/x11-auth.sh` still described two containers, two cookie paths, an
`isaacsim` service, and an Isaac Sim image built from source on this host. All
four rewritten. The lesson worth keeping is the one the world-build script
taught: a deleted compose service does not announce itself, and every `docker
compose run --rm <service>` in a script or a doc is a reference that the merge
silently invalidated.

## 2026-09-13 (later) — `turtlebot3_isaacsim`, written

A standalone package, peer of `turtlebot3_gazebo`, not a `tb3_bringup` backend:
its own models, its own worlds, its own substitution for the real bringup. The
plan in `docs/status.md` ("Then the package") is now code. Not yet run.

**The prior art was the whole design.** `isaacsim_bringup`
(`IsaacSim-ros_workspaces/humble_ws/src/`) is NVIDIA's own "start Isaac Sim from
a launch file" and is exactly the `gzserver.launch.py` analog, so the package
wraps it rather than reinventing it. It is neither in our image nor mounted,
which makes vendoring it a prerequisite rather than a given.

**Four sharp edges, all found by reading `run_isaacsim.py` rather than its
docs.** They are now pinned down once in `launch/isaacsim.launch.py`:

1. On the `standalone:=` path, `headless`, `gui`, `custom_args` and
   `play_sim_on_start` are **all ignored** — the branch is a bare
   `Popen(f"{python.sh} {standalone}")`. So `headless:=` has to become the
   standalone script's own argv.
2. `standalone` is pasted into a `shell=True` string with no quoting. That is
   how arguments reach the script at all, and it is why the launch file refuses
   any argument containing a space instead of letting it split silently.
3. `use_internal_libs:=true` strips `/opt/ros/<distro>` and the other distro's
   name — but **not a colcon overlay**. Your own workspace survives on
   `PYTHONPATH`/`LD_LIBRARY_PATH` and is inherited by Kit's Python 3.12, which
   is precisely the pollution the stripping exists to prevent. `exclude_install_path`
   is the vendor's mechanism; it now defaults to `$COLCON_PREFIX_PATH`, verified
   resolving to `/tmp/ws/install` in a test build.
4. `install_path` must be passed whenever Isaac Sim is not at the version-derived
   default under `$HOME`.

**Two Gazebo launch files have no counterpart, and both absences are
structural.** `gzclient` — Gazebo splits server and GUI into two processes,
Kit is one process that either opens a window or does not. And
`spawn_turtlebot3` — checked directly rather than assumed: NVIDIA's service
surface (`isaac_ros2_messages`) is `GetPrims`, `Get`/`SetPrimAttribute` and
`IsaacPose`, i.e. inspection, attribute setting and pose teleport, but
**nothing that creates a prim**. There is no `libgazebo_ros_factory.so`
equivalent, so the robot is referenced into the stage by the simulator script
and `x_pose`/`y_pose` are forwarded to it. The argument names are kept
identical so muscle memory transfers.

**Concept — the interpreter split decides the file layout.** The standalone
script runs on Kit's Python 3.12 with ROS 2 stripped, so it cannot call
`get_package_share_directory`. Every path it needs must arrive as an absolute
argument from the launch file, which does have ament. NVIDIA hit the same wall
and solved it the same way for `open_isaacsim_stage.py`. This is the same
constraint that made `worlds.py` a plain directory tree, and it is worth
stating as a rule: **in this project, anything Kit needs to find, ROS has to
hand it.**

**The tilted-lidar problem has a real fix now.**
`app.sensors.nv.lidar.profileBaseFolder` is a settings *list* the renderer walks
to resolve a profile by name, so a package can append its own folder and ship
its own sensor model. `models/lidar_configs/turtlebot3_lds.json` is an authored
TB3 LDS — 360 samples/rev at 5 Hz, 0.12–3.5 m, and **0° elevation**, against
`Example_Rotary_2D`'s `elevationDeg = [-2.0]` that made it scan the floor. Matches
`turtlebot3_gazebo`'s `<ray>` block field for field. Unverified until run.

**`docs/status.md` contradicts itself and should be corrected.** One bullet
still says the RTX lidar produces no messages at all under `--headless`. The
2026-09-13 entries above record `/scan` at 6.03 Hz and 10 Hz headless, and say
plainly that it renders headless with no display. The bullet is stale; it would
otherwise steer the new package toward a guard it does not need.

**Verified, so far as it can be without a GPU run:** the package builds under
`colcon` and installs every directory where it should; `empty_world.launch.py`
loads through the full include chain with `isaacsim_bringup` vendored in and
reports its arguments; the `OpaqueFunction` assembles the expected `standalone:=`
string in both the empty-world and turtlebot3_world cases; and the standalone
script's real `parse_args()` accepts `--x-pose -2.0` (argparse's negative-number
matcher) and resolves its default asset path from `TURTLEBOT3_MODEL`.

## 2026-09-13 (last) — `turtlebot3_isaacsim` split out to its own repo

Now at `~/turtlebot3_ws/src/turtlebot3_isaacsim`, its own git repo, beside
`turtlebot3` and `turtlebot3_simulations` in the same colcon workspace — which
is how every other TurtleBot3 package already lives. It is developed
independently from here on, and `tb3_sim2real` will import it alongside
`turtlebot3_gazebo` rather than contain it.

**`turtlebot3_isaacsim/` in this repo is now a stale duplicate.** It is left in
place only until the new repo is confirmed, and should then be deleted outright
rather than kept in sync. Two copies of a simulator launcher is exactly the
drift this project spends its effort avoiding everywhere else.

**The comparison prose left the code.** The first draft explained itself against
`turtlebot3_gazebo` in every docstring, which made the package read as an essay
about Gazebo rather than as a peer of it. All of it now lives in one
`DESIGN.md` beside the README, and the code carries only short notes at the
traps. Calibrated rather than guessed: `flake8` over `turtlebot3_gazebo`'s own
`launch/` reports 39 findings across exactly three classes (`D100`, `D103`,
`I201` — ament's docstring and import-grouping opinions, which ROBOTIS does not
satisfy either). The new package reports the same three classes plus the import
ordering that booting `SimulationApp` before the `isaacsim.*` imports forces,
and nothing else.

**Lesson worth keeping.** "Document the reasoning" and "keep the code clean"
pulled in opposite directions here, and the resolution was not a compromise
between them: the reasoning was worth every word, it was just in the wrong file.
A package that has to be a drop-in peer of another should *read* like that peer,
because whoever reaches for it will have the peer open beside it.

## 2026-09-14 — CLAUDE.md orients, and the skill list gets trimmed

**A fresh session had no cheap way to learn what this repo is.** `CLAUDE.md`
opened directly into the working agreements — good rules, no subject. The only
way to find the subject was to read `README.md`, then `docs/architecture.md`,
then enough of the tree to tell `isaac/scripts/` from `turtlebot3_isaacsim/`.
That is a survey, and it happened at the start of every conversation.

`CLAUDE.md` now leads with three short sections: what the repo is (three
backends, one interface, one container), the layout as an annotated tree, and a
one-line index saying which doc answers which question. The agreements follow
unchanged. The index is the part that does the work — it tells a new session to
read *one* doc, and which one, instead of the set.

**Writing the layout caught a live trap.** The first draft described
`turtlebot3_isaacsim/` as a standalone package usable on its own, which is what
its README says and what it was for one day. `docs/history.md` records the
2026-09-13 split: the real package now lives at
`~/turtlebot3_ws/src/turtlebot3_isaacsim` and the in-repo copy is a stale
duplicate awaiting deletion. Exactly the fact a new session would have missed,
and it would have missed it by editing the wrong copy. The entry now says so in
its first two words.

**Skills.** `.claude/settings.json` gained `skillOverrides`. The bundled skills
with no bearing on a ROS 2 / Isaac Sim repo are `off` (design, dataviz, the
three artifact skills, claude-api, claude-in-chrome); the harness-config ones
are `user-invocable-only`, hidden from the model but still typable as slash
commands (schedule, loop, update-config, keybindings-help,
fewer-permission-prompts, init). `code-review`, `simplify`, `security-review`
and `run` stay fully on.

Connectors needed no change: there are no MCP servers configured at any scope.
The two that show up — Google Drive, Anthropic Economic Index — are account-level
claude.ai connectors, toggled in `/mcp` or on claude.ai, not from any file in
this repo.

**Lesson worth keeping.** Context discipline is not only about what a session
reads; it is about whether the first file it reads is *enough*. A CLAUDE.md that
carries rules but not the subject guarantees a survey before any work starts.

## 2026-09-14 (later) — the Isaac side becomes an import, and worlds get a source of truth that reaches the real room

Three things were tangled together and came apart in one pass: a stale copy of a
package, a `world:=` that only half the backends honoured, and no answer to
"how does a real room get into both simulators".

### The copy was four worlds and an Isaac Sim version behind

`turtlebot3_isaacsim/` in this tree was a snapshot taken when the package split
out on 2026-09-13. By the time it was removed the real repository had the
warehouse, a simple room and a kitchen, `scripts/build_map.py` (Isaac Sim's own
occupancy-map generator behind a CLI), a committed burger asset, and a container
on Isaac Sim **6.1.0** — against this copy's 6.0.1.

It is now a source dependency: `tb3_sim2real.repos` pins it and NVIDIA's
`isaacsim_bringup`, and `scripts/workspace.sh` imports both into a gitignored
`src/`. Two things about that script are not obvious:

- NVIDIA ship nine packages per distro in `humble_ws/` and `jazzy_ws/`, so
  `isaacsim_bringup` appears **twice** and colcon refuses the whole build on a
  duplicate package name before compiling anything. The script drops a
  `COLCON_IGNORE` in every package except `${ROS_DISTRO}_ws`'s copy.
- `vcs import <dir>` prefixes the manifest's keys with `<dir>`, so keys written
  as `src/foo` and imported into `src` land in `src/src/foo`. Keys are bare
  names.

`docker/isaacsim-ros2/` went the same way, and that was the sharper call: it was
byte-identical to the imported package's copy except for the Isaac Sim tag, and
ours was the stale side. The base image is now built from
`src/turtlebot3_isaacsim/docker/isaacsim-ros2/Dockerfile.humble` and `docker/`
here is one thin layer — TurtleBot3, Gazebo Classic's worlds, the real robot's
drivers, cartographer, the workspace.

**Pinned to `IsaacSim-6.0.1` on purpose while Isaac Sim is 6.1.0.** The 6.1.0
tag reimplements `run_isaacsim.launch.py` as `run_isaacsim.launch.xml` —
argument for argument the same file, different name and source type — and
`turtlebot3_isaacsim` includes it by the old name, so a 6.1.0 pin dies with a
bare `[Errno 2] No such file or directory`. Not a runtime mismatch: the package
passes `install_path=/isaac-sim`, which overrides the launcher's own `version`
default, so the 6.0.1 launcher starts the 6.1.0 install. Moving it forward is
two lines in that package (`AnyLaunchDescriptionSource` and the new filename)
and belongs there.

### `world:=` now means one thing

It used to be refused for `backend:=isaacsim`, and honestly so: that backend
only *attached* to a simulator someone had already started with the right stage,
so accepting a world would have been a lie. `backends/isaacsim.launch.py` now
includes the imported package's `isaacsim.launch.py`, which includes NVIDIA's
`run_isaacsim.launch.py`, so Kit comes up behind the same `ros2 launch` as
everything else and the ROS side genuinely chooses the environment.

`world:=` therefore names an environment on all three backends — gzserver loads
its `.world`, Kit opens its `.usd`, the real robot loads nothing — and all three
take the spawn pose and the Nav2 map from the same manifest. It also accepts a
**path**, so a user's own world directory needs no entry in this repository.

Fallout worth recording:

- Gazebo spawns through `spawn_entity.py` directly now. Upstream's
  `spawn_turtlebot3.launch.py` takes `x_pose` and `y_pose` only and pins z, so a
  manifest yaw was silently dropped on Gazebo while Isaac Sim honoured it — two
  simulators at different headings from one manifest.
- The three `nav2_<backend>.yaml` were byte-identical copies of upstream's
  `burger.yaml`. One `nav2_params.yaml` now, with `bringup.launch.py` preferring
  `nav2_<backend>.yaml` the day one exists. Three files that must be kept in
  step are a worse record of "they do not differ yet" than one file.
- `tb3_bringup/maps/` held the stock map, which belongs to the *environment*.
  It moved to `worlds/turtlebot3_world/map/` and the manifest points at it.

### Cloning a real room: the map is the reference

The research question was what a normal TurtleBot3 user actually has that
describes their environment metrically. The answer is the occupancy map they had
to record for Nav2 anyway: it is in the frame Nav2 works in, at the planner's
resolution, and it records exactly the geometry the robot's own lidar can see.

`scripts/clone_world.py` reads it, extrudes the occupied cells into one OBJ
(interior faces culled, so a wall costs its surface and not its volume), and
writes a world directory with the map copied in beside the mesh.
`scripts/build_world.sh` then produces both simulators' representations from
that manifest — no second authoring step, no second source of truth.

Decisions behind that, each with a reason that is not aesthetic:

- **Not USD or SDF as the single format.** Gazebo Classic has no USD code path,
  and `gz-usd` targets gz-sim, has no releases, needs OpenUSD 24.08 built from
  source, and has had no commit since 2024-10. Isaac Sim 6.x ships importers for
  URDF, MJCF, Onshape and CAD — none for SDF. URDF has no world concept and its
  Isaac importer builds an articulation with rigid bodies, the opposite of a
  static room.
- **OBJ, not Collada.** Gazebo Classic's mesh loader reads `dae/obj/stl`; Isaac
  Sim's converter reads `obj/fbx/gltf`. OBJ is the intersection, and it carries
  neither a unit scale nor an up-axis for the two to read differently — which is
  precisely the trap `turtlebot3_world`'s meshes already document.
- **One mesh, not a few hundred boxes.** Rectangle-decomposing a real room's
  grid gives hundreds of boxes, and two simulators each approximating hundreds
  of boxes is *more* surface to disagree on, not less.
- **Not a 3D scan.** A phone scan is a two-million-triangle non-manifold shell
  that neither ODE nor PhysX collides with usefully. Keep it as a visual body if
  you want it; let the extrusion carry collision.
- **Stdlib and PyYAML only.** Reading a PGM and walking a grid does not need
  OpenCV, and a dependency a student has to install is a step they can fail.

What the clone captures is a **floor plan**, not a model of the room: a table
top, an overhang or a step is not in the map and so not in the clone. Those are
extra bodies, measured by hand, in the same manifest.

### `scripts/check_worlds.py`, and what it actually catches

Three ways one world quietly becomes two, each verified against a deliberately
broken copy:

1. *Manifest edited, artifacts not regenerated.* Both generated files carry the
   manifest's sha256 — in the `.world`'s XML header and the USD's
   `customLayerData`. (The USD check greps the bytes rather than depending on a
   USD reader; the digest is stored literally in usdc as in usda.)
2. *A generated file edited by hand, header intact.* The `.world` is parsed back
   with ElementTree and compared body by body against the manifest.
3. *The model no longer describes the real room.* The manifest is rasterised at
   the burger's 0.182 m beam height and compared with the world's own map.

The third needed one correction to be useful. Cell-exact IoU scored a *correct*
clone at 0.42, because a wall one cell thick in the map is a surface with two
sides in the model and the two land in neighbouring cells. Comparing with one
cell of slack in both directions, and reporting "how much of the map is
modelled" and "how much of the model is not in the map" separately, scores the
same clone 100%/0% — and a map mirrored about its x axis, which is a real bug
that plans and drives without looking broken, at 22%/80%.

It deliberately does not parse USD: that needs Kit or `usd-core`, and a check
that needs installing is a check that gets skipped. The Isaac stage is verified
where it is written instead, by `build_world_usd.py`'s bounds and collider
assertions.

### Two bugs found by actually running it

**`turtlebot3_node` aborts without a `namespace` parameter.** It declares it
statically with no default, so `backend:=real` died with `Statically typed
parameter 'namespace' must be initialized` before touching the serial port.
Upstream's `robot.launch.py` passes `{'namespace': namespace}`; this backend had
lifted the node without it. It now reaches `Failed to open the port
(/dev/ttyACM0)`, which is the correct failure with no OpenCR attached.

**An `OnProcessExit` handler without `handle_once` brings Nav2 up twice.** Nav2
is now started when `wait_for_sim` exits rather than beside it, so it comes up
when `/clock` exists rather than two minutes before it on a cold shader cache.
Left registered, the handler ran `bringup_launch.py` again on a later matching
exit and loaded every composable node into a second `nav2_container`. The
symptom names neither cause: a wall of `Transition is not registered` from
`map_server` and `amcl`, then `Node '/local_costmap/local_costmap' has already
been added to an executor`, then both containers aborting. The tell was that
every "Creating" line in the log appeared exactly twice under a single launch
action index.

A third thing, not a bug but a trap: a leftover `gzserver` from an earlier test
makes the next `backend:=gazebo` die with exit code 255 and nothing else.
`pkill -INT -f "ros2 launch"` does not take gzserver with it.

### Measured

Both simulated backends, from the ROS side, `world:=turtlebot3_world`:
`/clock` 10.0 / 56.1 Hz, `/scan` 5.0 / 3.5 Hz, `/odom` 29.4 / 66.0 Hz,
`/joint_states` 29.4 / 56.5 Hz (gazebo / isaacsim). Isaac's `/scan` returns the
arena — real ranges from 0.60 m, not an empty plane. `nav:=true` on Gazebo
brings the whole stack up with zero errors against the map now served out of
`worlds/turtlebot3_world/map/`.

One difference worth not mistaking for a fault: the `odom` frame does not start
in the same place. Gazebo's diff-drive plugin puts `odom` at the world origin,
Isaac Sim's `IsaacComputeOdometry` puts it at the robot. Both are valid; AMCL
resolves it into `map->odom`; it only bites if raw `/odom` is compared between
backends without saying which frame is meant.

Net: 47 tracked files where there were 67, about 2,500 lines fewer.

## 2026-09-14 (last) — the meshes were never going to convert, and three bugs were hiding behind one cache

A postscript to the restructure, and the most useful thing found all day. It
started as a routine check: the registry now recommends OBJ for user worlds, and
OBJ had never actually been through the pipeline.

**The first OBJ world came out 100x too small,** and the manifest's `verify`
bounds caught it — `-0.0295` where `-2.9500` was expected, on every axis. Isaac
Sim's `AssetConverterContext` defaults to **centimetres**: it authors the
converted layer with `metersPerUnit = 0.01`, and USD scales a reference by the
ratio of the two layers' units, so a mesh whose vertices are in metres arrives
in a metres stage divided by a hundred. A format that declares its own unit —
Collada's `<unit meter="...">` — escapes this entirely, which is exactly why
every world here had hidden it. `use_meter_as_world_unit = True` is the fix.

**Turning that flag on is what revealed the real problem.** The mesh cache is
keyed on mtime, so a settings change has to invalidate it; adding that
invalidation made the converter run for the first time on Isaac Sim 6.1, and it
answered:

    Unsupported import format: .dae. Supported formats: .bvh, .fbx, .glb,
    .gltf, .lxo, .md5, .obj, .ply, .stl, .usd, .usda, .usdc, .usdz

**Isaac Sim 6.1 does not read Collada.** Every build of `turtlebot3_world`
since the upgrade had been reusing `hexagon.usd` and `wall.usd` converted under
6.0. The world could not have been rebuilt from a clean checkout, and nothing
would have said so — `verify` passed each time, on stale artifacts that happened
to be correct.

So the meshes became OBJ. `scripts/dae_to_obj.py` bakes the declared
`<unit meter="0.0254">` into the vertices, because OBJ has no unit field, and
deliberately does **not** apply `up_axis`: these files declare `Y_UP` while
laying their vertices out Z-up, and Gazebo ignores the label, so the vertices
are the parity truth. The `.dae` files are deleted rather than kept beside the
`.obj` — two files holding the same geometry is the thing the registry exists
to prevent, and upstream still has them.

Checked before trusting it: each file has exactly one `<geometry>` and one
visual-scene node with no transform, face indices are in range, and the scaled
bounds reproduce **every** number in the manifest's `verify` block
analytically — `wall` at 0.25 after a -90 degree yaw gives x = +/-3.29955
against the recorded `-3.2996`; `head` at 0.8 offset to x = 3.5 gives 4.6732 and
z = 1.532; `left_hand` at 0.55 gives y = 3.3985. All four exact.

Then measured, same manifest, same meshes, robot at the manifest's spawn:

| | gazebo | isaacsim |
|---|---|---|
| rays | 360 | 360 |
| with a return | 324 | 307 |
| closest | 0.514 m | 0.521 m |
| furthest | 3.352 m | 3.366 m |

**7 mm apart on the closest return**, and the pre-conversion Collada
measurement was 324/360 at 0.511-3.357 m, so the conversion moved nothing.
Note also that Isaac now publishes **360** rays, not the 3600 `docs/status.md`
recorded: the imported package ships a real LDS scan pattern. That entry was
describing the simulator this repository used to carry.

### Two more, found on the way

**`ignore_animation` and `ignore_cameras` had never done anything.**
`AssetConverterContext` is a plain Python object, so assigning an attribute it
does not have succeeds silently. The real names are `ignore_animations` and
`ignore_camera`. Settings now go through a dict, and the code refuses any name
the context does not already carry — which is the same lesson CLAUDE.md already
records about node attributes that do not exist, in a place nobody had looked.

**A missing cache digest is a mismatch, not a clean slate.** The first version
of the invalidation wrote the settings digest without clearing a cache that
predated it, so the run that fixed the 100x error reused the 100x meshes and
failed identically. It took two rebuilds to notice that the fix was correct and
the cache was lying.

### And the check got sharper

`check_worlds.py`'s footprint comparison can now rasterise a yawed mesh, which
is what `turtlebot3_world`'s wall is — so the flagship world finally gets a real
comparison rather than a "needs a mesh library" skip. First run scored it 64%
spurious, which was the check being wrong rather than the world: the five
decorative hexagons sit **outside** the arena, and upstream's map was recorded
from inside it.

The fix is to read the map's third state. Occupied and free are not
complements; what is left is UNKNOWN, where the robot never looked. A model wall
standing in free space contradicts the map; one standing in unknown space does
not. With that distinction `turtlebot3_world` reads **100% of the map modelled,
0% contradicted, 946 model cells in space the map never saw** — and a map
mirrored about its x axis still fails, at 22% / 47%.

**The lesson, and it is the one this repository keeps relearning.** Three
independent bugs — a unit default, two misspelled flags, a stale cache — all
sat behind one cache entry that was correct for the wrong reason. Nothing
errored, `verify` passed, and both simulators agreed. A check that only runs
against cached work is not a check. The cache now carries a digest of what
produced it.

### Correction, same day

The lifecycle-manager bringup failure written up above as Kit holding the
machine was not that. Leftover `ros2 launch` processes from earlier tests were
still on the DDS domain, so two lifecycle managers were fighting over the same
nodes. On a restarted container the run is clean: zero errors, both managers
answering `is_active: True`, `map->odom` at exactly `[-2.000, -0.500]` after one
`/initialpose`.

Worth keeping for the mechanism rather than the conclusion.
`pkill -INT -f "ros2 launch"` issued from inside a `docker compose exec -T`
whose client has already exited does not reach them, and a `pkill -f gzserver`
placed after it in a subshell that exits first never runs at all. The visible
symptoms are a doubled `ros2 node list`, a second gzserver dying with exit code
255 and no message, and lifecycle transitions that "fail" while the nodes
themselves come up fine. `docker compose restart tb3_ros` is the reliable
reset, and costs a 1.5 s rebuild of `/ws/install`.

**The measurement lesson is the same one as the mesh cache, one layer up: a
result taken on a dirty environment is not a result.** Two write-ups today
blamed the system under test for state the test itself left behind.

---

## 2026-09-15/16 — colour, props, and measuring the two simulators against each other

### isaacsim_bringup moved to 6.1.0

The pin sat at `IsaacSim-6.0.1` while `/isaac-sim` is 6.1.0, because the 6.1.0
tag migrated every repo-owned launch file from Python to XML and
`turtlebot3_isaacsim` included `run_isaacsim.launch.py` by name. Fixed where it
belonged — in that package, with `AnyLaunchDescriptionSource` and the new
filename — so the pin could follow the simulator. Its `UPSTREAM.md` had already
documented the rename; only the code lagged. **Committed there, not pushed.**

### Isaac rendered everything grey, and Gazebo was wrong too

The manifest's only statement about colour was `Gazebo/White` — an **Ogre
material script name**. It means something to Gazebo and nothing to anything
else, so the USD generator had `ignore_materials=False` set faithfully and
nothing to apply it to. Colour is now data: `worlds.PALETTE` resolves the stock
names to the RGB in Gazebo 11's own `gazebo.material`, and each generator writes
its own dialect.

Fixing it surfaced a second bug. Upstream's `model.sdf` paints the wall
`Gazebo/FlatBlack` and the five hexagons `Gazebo/Green`; the manifest had
dropped both when it was derived, so all 15 bodies defaulted to white.
**Gazebo had been rendering `turtlebot3_world` wrong as well.**

Lesson worth keeping: a check that proves two files came from one source says
nothing about whether either is *right*. Provenance, bounds, collider counts and
the footprint test all passed on a stage that rendered entirely grey.

### dae_to_obj silently produced broken geometry

It read `library_geometries` directly and only *warned* that a visual-scene
`<matrix>` was being ignored. On `gazebo_models`' `cafe_table.dae` that puts the
tabletop at z = 0.00-0.04 m — flat on the floor — while the overall height comes
out 0.737 m against a correct 0.775 m. **A 38 mm difference in the bounding
box**, which is to say the manifest's `verify` block would have passed it.

Now the visual scene is walked properly. Upstream's two meshes carry no node
transforms and re-convert byte-for-byte identical, which is what made the change
safe to make.

The general shape of this bug is worth remembering: *structurally wrong,
dimensionally plausible*. It is the failure mode a bounds check cannot catch,
and the reason a bounds check is not sufficient on its own.

### small_office

A room anyone can write (five boxes and a partition) furnished with things
nobody can write as boxes (seven mesh props). Props are AWS RoboMaker's
small-house meshes under **MIT-0** — chosen over `osrf/gazebo_models` because
that is CC-BY *and has no chair at all*. Its `bookshelf`, `cabinet` and `table`
turn out to be pure SDF box assemblies, which is a useful thing to know: if you
only need a shelf, transcribe the boxes and skip the mesh pipeline entirely.

`scripts/make_map.py` writes a Nav2 map from a manifest, for a world that was
designed rather than cloned. Deliberately the narrow case — on a world with a
real counterpart it would make the footprint check compare the model with a
picture of itself.

### One instrument, not one per backend

`ros2 run tb3_bringup drive_test` publishes an identical open-loop sequence and
reads `/odom`, so it runs unchanged on all three backends.

**It had a bug that would have produced a convincing wrong answer.** `ros2 run`
does not set `use_sim_time`, so phases were timed on the wall clock while the
robot moved in sim time. Gazebo at RTF 1.0 hides that completely; Isaac at 0.95
does not, and `expected_distance` is computed from the same wall seconds. The
first four runs were discarded. It now detects a `/clock` publisher and switches
itself, which also keeps it correct on the real robot, where wall time is right.

### What the measurements found

**Isaac Sim under-rotates by 30%** at `wz = 0.5` and **52%** at `wz = 0.2`,
while linear motion is within 5%. Not slip — slipping wheels read *above* target,
free-spinning; these read below. A sweep settles the mechanism: tracking goes
47.8 / 73.1 / 83.3 / 88.3 % as the rate rises, while the *absolute* error stays
near-constant at 0.25-0.43 rad/s. With `stiffness = 0`,
`D * (w_target - w_actual) = tau`, so a fixed friction torque gives exactly a
constant offset. A units error would hold the ratio fixed; a velocity cap would
make high speed worse. Both excluded. The gain is the one `turtlebot3_isaacsim`
already marks `# TODO unverified gain`.

**Nav2 hides it almost entirely.** Five goal runs all SUCCEEDED, 22.16-23.16 s,
paths within 2.4%. A pure 180-degree rotation is 4.46 s on Gazebo and 4.56 s on
Isaac — and Isaac's residual yaw error is the *smaller*, 0.110 against 0.261 rad.
The loop closes on yaw error, so a plant delivering 70% just gets commanded
longer. The defect relocates rather than disappears: it matters for open loop,
dead reckoning, and above all for tuning gains against Isaac and expecting them
to transfer.

**Contact is where the two are genuinely not interchangeable.** Driven into the
table and held there, Gazebo deflects off the leg and pivots 90 degrees around
it, still moving 0.085 m and 3.1 rad during a phase commanding zero, and
recovers only 29% of a commanded reverse. Isaac stops dead and square —
0.0001 rad of yaw, an exact standstill, a clean 91% reverse, no contact impulse.
Neither is wrong; a real burger does deflect, so Gazebo is the more lifelike and
Isaac the more repeatable.

Good news: **neither invents odometry.** Isaac's odom path equals its
displacement to 1.7 mm; Gazebo's excess is the pivot arc it really travelled.
Blocked wheels are resisted in both, so Nav2 is never fed unbounded phantom
distance.

**The backends disagree about where `/odom`'s origin is.** Gazebo's first sample
at the spawn reads the world pose `(-1.4999, -1.5000)`; Isaac's reads zero. The
real `turtlebot3_node` starts at zero wherever it is switched on, so **Isaac
matches the real robot and Gazebo does not.** It breaks nothing — `odom` need
only be continuous, and AMCL absorbs the offset — but raw `/odom` positions are
not comparable between backends. Cause is upstream: `turtlebot3_gazebo`'s
`model.sdf` sets no `<odometry_source>`. Recorded rather than patched, because
overriding it means carrying our own copy of upstream's model.

### Stale numbers rot quietly

`worlds/README.md` claimed a mirrored map scores 22% / 80%. Re-measured: 27% /
27% about x, 31% / 24% about y. The old spurious figure predated the change that
stopped counting model cells in *unknown* space — the very fix that lowered it —
and nothing re-checked it. Also worth writing down: both mirrors fail on
**coverage**, not contradiction, because this arena is nearly symmetric. A doc
implying otherwise sends the next person to tune the wrong threshold.
