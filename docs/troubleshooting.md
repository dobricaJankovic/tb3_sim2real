# Things that fail silently

- **`ROS_DOMAIN_ID` mismatch.** The stock TurtleBot3 image hardcodes `30` in
  `.bashrc`; Isaac Sim defaults to `0`. Different domains = total silence, no
  error. Set it explicitly for both services (compose does).
- **Separate `/dev/shm`.** Discovery succeeds over UDP, then Fast-DDS negotiates
  shared memory for the data path and nothing arrives. `ipc: host` on both.
- **Fast-DDS shared memory across containers running as different uids.**
  This bit us for real, and `ipc: host` was set correctly the whole time —
  `/dev/shm` genuinely was shared. The problem is that isaacsim runs as uid 1234
  (NVIDIA's image ends with `USER isaac-sim`) while tb3_ros runs as root, and
  Fast-DDS creates its `/dev/shm` segments with mode **0700**, so neither
  participant can open the other's. The symptom is maximally deceptive:
  discovery rides UDP multicast, so `ros2 topic list` from `tb3_ros` shows
  every Isaac Sim topic and `ros2 topic info --verbose` reports a healthy
  `RELIABLE` publisher with a real GID — but `ros2 topic echo` hangs forever
  and `ros2 topic hz` reports nothing. Everything looks connected; zero bytes
  move. Fixed by forcing UDP-only on both services via
  `docker/fastdds_udp_only.xml` + `FASTRTPS_DEFAULT_PROFILES_FILE` (compose
  sets both). Quick confirmation that you are looking at this and not something
  else: re-run the failing `ros2 topic echo` with
  `FASTRTPS_DEFAULT_PROFILES_FILE` pointed at that XML — if data appears
  instantly, it was SHM.
- **Isaac Sim publishes nothing until you press Play.** OmniGraph nodes are
  inert when stopped, which looks exactly like a broken DDS setup.
  `wait_for_sim` exists to make this legible in the log.
- **Frame names.** `base_footprint`, `base_scan`, `odom` are free text in the
  OmniGraph nodes. One wrong character and everything still *runs* — Nav2's
  costmap just stays empty.
- **QoS on `/scan`.** Sensor data usually wants Best Effort. A Reliable
  subscriber against a Best Effort publisher shows up in `ros2 topic list` while
  delivering nothing.
- **X11 for two containers.** Both Isaac Sim and RViz need `DISPLAY` and
  `/tmp/.X11-unix`. Neither container runs as you, but host X access control is
  per-UID (`xhost` shows `SI:localuser:<you>`), so the socket mount alone isn't
  enough — `gzclient`/RViz/Kit abort with "Authorization required, but no
  authorization protocol specified". Run `docker/x11-auth.sh` once per X session
  before `docker compose up/run` — it writes a re-keyed Xauthority cookie
  (family `ffff`, so it matches any hostname) that compose mounts into both
  services. (`xhost +local:root` also works but permanently loosens host X
  access control instead of scoping a cookie.)
- **The two services mount that cookie at different paths.** `tb3_ros` runs as
  root and reads `/root/.Xauthority`. `isaacsim` does *not* run as root —
  NVIDIA's Dockerfile ends with `USER isaac-sim` (uid 1234, home `/isaac-sim`),
  and uid 1234 cannot traverse `/root`, so it reads `/tmp/.docker.xauth`
  instead (`/tmp` is chowned to `isaac-sim` by the image). Mounting the cookie
  under `/root` for `isaacsim` fails **silently**: Kit logs `GLFW initialization
  failed`, `failed to open the default display`, and `IAppWindow::startup
  failed`, then keeps running with no window — and `docker ps` still says
  `healthy`, because the image's healthcheck only greps the Kit log for
  `AppReady`. Symptom: "the container is up but no Isaac Sim window appeared."
  Check `/isaac-sim/.nvidia-omniverse/logs/Kit/*/*/kit_*.log` for the GLFW
  lines to confirm.
- **No GPU in `tb3_ros`.** Without a `deploy.resources.reservations.devices:
  [gpu]` block (compose has it now), `gzserver`/`gzclient` silently fall back
  to `nouveau` and die with `libGL error: failed to load driver: nouveau`.
- **`GAZEBO_MODEL_PATH` unset.** `ros-humble-turtlebot3-gazebo` ships no
  Gazebo-specific ament hook, so `model://turtlebot3_world` in the world file
  won't resolve unless `GAZEBO_MODEL_PATH` is set explicitly (compose does).
  This is the standard TurtleBot3 e-manual `.bashrc` step — needed here since
  the container has no `.bashrc`.
- **`gzserver` hangs on startup, `/spawn_entity` never appears.** Its
  `ModelDatabase` does a *synchronous* fetch from `models.gazebosim.org` to
  refresh local model-cache metadata, independent of `GAZEBO_MODEL_PATH`. In
  this network it hangs instead of failing fast, and `spawn_entity.py`'s 30s
  wait always loses the race. `GAZEBO_MODEL_DATABASE_URI=` (empty, set in
  compose) skips the fetch — local models resolve fine without it.
- **Stale zombie `gzserver`/`gzclient` hold port `11345` across manual
  restarts.** The container's PID 1 (a bare `tail -f /dev/null` when kept
  alive for repeated `docker exec`) doesn't reap orphaned children, so a
  process killed via a `timeout`-wrapped launch can leave a zombie holding the
  Gazebo master port. Next `gzserver` start fails: `Unable to start
  server[bind: Address already in use]`. Kill it by explicit PID
  (`kill -9 <pid>`) before retrying.
- **UNRESOLVED — real TB3 topics invisible from the dev machine.** `ros2
  topic list` on the ETF network host only ever shows local topics
  (`/parameter_events`, `/rosout`), never the real robot's, even though
  `ROS_DOMAIN_ID=30` matches and both `tb3_ros_dev` containers run
  `--network host`. `ssh` and `ping` to the robot work fine (`ttl=63`, one
  router hop via `10.118.5.1`) — it's on a different `/24` than this host
  (`10.118.5.245/24` vs `10.118.19.161`). ROS 2's default discovery
  (`rmw_fastrtps_cpp`, SPDP) announces over UDP multicast, which campus-network
  routers don't forward across subnets/VLANs. Unicast works, multicast
  discovery doesn't — that's the whole bug. Not a Docker issue (network mode
  is already `host`). Fix is one of: a Fast-DDS Discovery Server
  (`ROS_DISCOVERY_SERVER`, unicast, no new packages), or switch to
  `rmw_cyclonedds_cpp` with an explicit static-peer `cyclonedds.xml` pointing
  at both machines' IPs. Needs doing on both the dev host and the robot —
  picking it up later.
- **RESOLVED — Isaac Sim GUI shows a blank viewport.** `tb3_world.usd` and
  `turtlebot3_burger.usd` opened without error but rendered nothing. Two
  independent bugs were stacked here, which is why it looked so strange:
  the stage genuinely had no renderable geometry (see the `package://` entry
  below), *and* the GUI window was never actually opening (see the Xauthority
  path entry above — the "confirmed running" Kit process was running windowless).
  Both are fixed; the stage now bounds to 138 x 178 x 191 mm.
- **Imported robot renders nothing, stage loads clean.** The URDF's meshes are
  `package://turtlebot3_description/meshes/...`, but the isaacsim container has
  no ROS and so no such package on disk. `urdf_usd_converter` resolves an
  unmatched `package://` URL to a *bare relative path* against the URDF's own
  directory, finds nothing, and still emits the link — as an empty `Xform` with
  the correct transform and material binding but no geometry. Nothing errors.
  What makes it especially quiet is that the inline `<collision>` primitives
  (`<box>`, `<cylinder>`) have no external file and import fine, so the prim
  tree looks populated — but they carry `purpose = "guide"` and are not drawn.
  Tells: no `geometries.usd` in the asset package, and `extentsHint` full of
  `3.4028235e38` (FLT_MAX) sentinels, which is USD's empty-bounds marker.
  Fix: copy the package share dir onto the shared mount and point the importer
  at it with `ros_package_paths=[{'name': ..., 'path': ...}]`, where `path` is
  the package directory *itself* — resolution is `path / relative_path`, so
  naming the parent silently fails the same way. Verify with
  `isaac/scripts/verify_asset.py`, which bounds the robot subtree specifically;
  checking the whole stage is not enough, because a ground plane alone clears it.
- **Files written by the isaacsim container can't be deleted from the host.**
  It runs as uid 1234, so anything it writes into `isaac/scenes/` is owned by
  1234 and `rm` fails with EACCES on the subdirectories. Remove them with a
  throwaway root container:
  `docker run --rm --user root --entrypoint bash -v $PWD/isaac/scenes:/scenes
  isaac-sim-docker:latest -c 'rm -rf /scenes/<path>'`.
