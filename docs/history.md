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

## 2026-09-06 — can Isaac Sim and ROS share one container?

**Problem — the stated reason for the two-container split was wrong.** The split
was justified by "Humble is Python 3.10, Isaac Sim 6.0 is Python 3.12". That is
not a blocker. Kit bundles its own interpreter (`/isaac-sim/kit/python`, 3.12.13)
and the image has no system Python at all, so the two never meet. ROS 2 Python
code does not run inside Kit — the OmniGraph ROS nodes are C++.

**Concept — Isaac Sim's env setup appends, so a sourced ROS wins.**
`setup_python_env.sh` does `export PYTHONPATH=$PYTHONPATH:...` and the same for
`LD_LIBRARY_PATH`. Anything a sourced ROS 2 installation put there stays *ahead*
of Kit's own entries, so Kit loads the wrong interpreter's modules and aborts.
`docker/isaacsim-ros2/ros-isolate` strips `/opt/ros` and every sourced overlay
prefix out of `PATH`/`PYTHONPATH`/`LD_LIBRARY_PATH`, unsets the ament discovery
variables, and leaves `ROS_DOMAIN_ID`/`RMW_IMPLEMENTATION` untouched so the
simulator stays on the same DDS domain. This is the same fix NVIDIA's
`isaacsim_bringup/run_isaacsim.py` applies in `update_env_vars()`; in a merged
container it becomes necessary rather than redundant.

**Fixed — one container works, on 24.04/Jazzy.** `docker/isaacsim-ros2/` builds
`isaacsim6-jazzy` from `ros:jazzy-ros-base` plus `COPY --from` of the Isaac Sim
tree. `verify.sh` passes 5/5: ROS tooling, Gazebo, env isolation, Kit starting,
and `/clock` published by the bridge and received by the system ROS 2
installation *in the same container*.

**Open — the 22.04/Humble image is blocked upstream by glibc.** Kit itself runs
fine on Jammy (the `kit` binary and `libcarb.so` need at most `GLIBC_2.34`;
Jammy has 2.35), and 1095 of 1104 Isaac Sim extension libraries are within that
ceiling. Nine require `GLIBC_2.38`, which only Noble provides — and three of
them are the bridge: `libisaacsim.ros2.core.humble.so`,
`libisaacsim.ros2.core.jazzy.so`, `libisaacsim.ros2.nodes.plugin.so`. Both
bundled distros fail identically, so this is a build-host artifact, not a
Humble-versus-Jazzy issue. The remaining path is building Isaac Sim from source
on Jammy — `setup.sh` says "Tested on: Ubuntu 22.04 / 24.04" — which would
recompile those nine against 2.35.

**Concept — check the libraries you actually need, not a representative one.**
The decision to try this rested on sampling `kit` and `libcarb.so` for their
glibc ceiling. Those are the portable Kit SDK; Isaac Sim's *own* extension
libraries are built against the build host and were the ones that mattered.
Scanning two binaries out of eleven hundred looked like evidence and was not.

**Concept — `ros2 topic echo --once` does not wait for a publisher.** It reports
`topic does not appear to be published yet` and exits, so using it to check a
simulator that takes ~25 s to boot tests the race, not the bridge. Poll
`ros2 topic list` for the topic first. This produced a false failure that looked
exactly like the real glibc failure on the other image.
