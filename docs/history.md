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
