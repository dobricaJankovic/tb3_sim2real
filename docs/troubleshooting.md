# Things that fail silently

- **`ROS_DOMAIN_ID` mismatch.** The stock TurtleBot3 image hardcodes `30` in
  `.bashrc`; Isaac Sim defaults to `0`. Different domains = total silence, no
  error. Set it explicitly for both services (compose does).
- **Separate `/dev/shm`.** Discovery succeeds over UDP, then Fast-DDS negotiates
  shared memory for the data path and nothing arrives. `ipc: host` on both.
  If it still misbehaves, force UDP-only via `FASTRTPS_DEFAULT_PROFILES_FILE`.
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
- **UNRESOLVED — Isaac Sim GUI shows a blank viewport.** `isaac/scenes/tb3_world.usd`
  and `turtlebot3_burger.usd` (see `isaac/scripts/import_tb3.py`) both open fine
  via File -> Open — no error, the stage loads — but nothing renders in the
  viewport. The main Kit process (`isaacsim` service, `runapp.sh`) is confirmed
  running and the host X server is confirmed reachable (`DISPLAY=:1`,
  `xdpyinfo` succeeds), so it's not a dead process or unreachable X11. A scan of
  recent container logs turned up nothing RTX/GPU-specific, only unrelated
  asset-browser `PermissionError`s (`/home/ubuntu/workspace_cache.json`,
  populating `/home/ubuntu`) — those are the file browser panel, not the
  viewport renderer, and don't obviously explain it. Not investigated further
  yet; picking it up later.
