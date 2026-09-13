# Things that fail silently

- **`ROS_DOMAIN_ID` mismatch.** The stock TurtleBot3 image hardcodes `30` in
  `.bashrc`; Isaac Sim defaults to `0`. Different domains = total silence, no
  error. Set it explicitly for both services (compose does).
- **Fast-DDS shared memory across containers running as different uids.**
  *Historical — it cannot happen in the single-container layout, and the
  UDP-only workaround was removed on 2026-09-13. Keep reading only if you have
  a second container on the domain.* Two containers, `ipc: host` set correctly
  on both, `/dev/shm` genuinely shared — and still nothing arrived, because
  isaacsim ran as uid 1234 (NVIDIA's image ends with `USER isaac-sim`) while
  tb3_ros runs as root, and Fast-DDS's `/dev/shm` segments are not readable
  across uids. The symptom is maximally deceptive: discovery rides UDP
  multicast, so `ros2 topic list` shows every Isaac Sim topic and
  `ros2 topic info --verbose` reports a healthy `RELIABLE` publisher with a
  real GID — but `ros2 topic echo` hangs forever and `ros2 topic hz` reports
  nothing. Everything looks connected; zero bytes move. The fix was
  `docker/fastdds_udp_only.xml` + `FASTRTPS_DEFAULT_PROFILES_FILE`; recover it
  from git history if a second uid ever comes back. Quick confirmation that you
  are looking at this and not something else: re-run the failing
  `ros2 topic echo` with `FASTRTPS_DEFAULT_PROFILES_FILE` pointed at that XML —
  if data appears instantly, it was SHM.
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
- **Starting a container before `x11-auth.sh` turns the cookie into a
  directory.** Same windowless symptom as above, different cause, and the one
  you actually hit in practice — the ordering in the compose header is
  load-bearing. Both services bind-mount the cookie path, and Docker's
  behaviour for a bind-mount source that does not exist is to create it as a
  **root-owned directory**. `XAUTHORITY` then points at a directory, no cookie
  is presented, and Kit runs windowless while reporting healthy. Re-running
  `x11-auth.sh` afterwards does not recover it — the script cannot overwrite a
  root-owned directory — so it now detects this and prints the fix instead of
  failing on `touch`:

      docker stop isaacsim
      sudo rm -rf /tmp/tb3_sim2real.docker.xauth
      ./docker/x11-auth.sh
      docker compose run --rm isaacsim

  The restart at the end is required, not tidiness: `xauth nmerge` writes a
  temp file and renames it into place, so re-keying gives the path a new inode
  and a container already running keeps the old one through its bind mount.
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
- **The robot bounces and rocks in place with nothing commanding it.** Nothing
  errors, every topic is healthy, and the robot slowly wanders off its spawn.
  Two defaults nobody chose:
  `isaacsim.core.api.objects.GroundPlane` gives itself a physics material at
  **restitution 0.8** when it is not handed one, and the URDF importer binds no
  physics material to any collider, so every surface on the robot falls back to
  PhysX's. Restitution combines as an *average* by default, so each contact
  under the robot came out at 0.4 — a bouncy-ball floor under 0.94 kg.
  It bites a TurtleBot3 especially hard because the burger's centre of mass sits
  4.3 mm *behind* the wheel axle while the caster skid (`caster_back_link`)
  clears the floor by 0.5 mm: the chassis permanently rests on that skid, so
  there is always a loaded elastic contact to pump. Measured on the bare ground
  plane with no `/cmd_vel` at all: 3.58 deg peak-to-peak in pitch, pitch rate to
  +/-0.7 rad/s, 7.7 mm of drift in 7 s. With materials bound
  (`import_tb3.py`'s `bind_robot_surfaces()`) the identical run reads 0.000 deg
  and 0.00 mm — it settles 0.3 deg nose-up on the skid and stops dead.
  `tb3_sim.py` now refuses to start on a stage that has lost them, because the
  symptom otherwise reads as a physics-tuning problem rather than a stale asset.
- **A crash in `tb3_sim.py` used to look like a clean exit.** `main()` ran under
  `try/finally: os._exit(0)`, and `os._exit` ends the process before Python gets
  to report the exception — so a missing stage, a bad prim path or a failed
  precondition printed *nothing* and exited 0. The traceback is now printed
  explicitly and the status is 1. Worth remembering for any Kit script: the
  `os._exit` that dodges the TaskGroup teardown race also eats your errors.
- **Re-running `import_tb3.py` used to leave the old asset behind.** The
  importer does not overwrite: handed an existing `turtlebot3_burger.usd/` it
  writes `turtlebot3_burger_1/` *inside* it, returns that path, and leaves the
  previous copy in place. Every re-import kept working, so nothing pointed at
  the growing pile, and only the printed `Robot asset:` line said which copy the
  new `tb3_world.usd` actually referenced. The script now clears the directory
  first, so an import is an import.
- **Files written by the isaacsim container can't be deleted from the host.**
  It runs as uid 1234, so anything it writes into `isaac/scenes/` is owned by
  1234 and `rm` fails with EACCES on the subdirectories. Remove them with a
  throwaway root container:
  `docker run --rm --user root --entrypoint bash -v $PWD/isaac/scenes:/scenes
  isaac-sim-docker:latest -c 'rm -rf /scenes/<path>'`.
- **Isaac Sim aborts on startup with a carb shared-memory assertion.** The
  message is
  `RStringInternals.inl:667 Assertion (false) failed: Failed to create shared
  memory named carb-RStringInternals-62`, followed by `terminate called without
  an active exception` and a core dump, before Kit produces any other output.
  carb names that POSIX segment after its own PID, and under `python.sh` the
  PID is deterministically 62 nearly every run. The isaacsim service used to
  set `ipc: host`, which put that name in the *host's* `/dev/shm` — sticky
  (`drwxrwxrwt`), shared with every other container, and with anything that
  ever ran Kit as root. This service runs as uid 1234 (`USER isaac-sim`), so a
  root-owned leftover at that exact name can be neither reused nor unlinked.
  Because the name is PID-derived and the PID repeats, one stale file poisons
  that slot *permanently*: every later run fails identically, which is what
  makes it look like a broken image rather than stale state.
  What actually fixed it is that the container runs as **root**: Kit creates
  those files `0666`, so a leftover from an earlier *root* run can be reused or
  unlinked, and only a leftover owned by somebody else is fatal. Dropping
  `ipc: host` was believed to be the structural fix and is not one — measured
  2026-09-13, the container's `/dev/shm` is still the host's, because
  `- /dev:/dev` in `docker-compose.yml` recursively bind-mounts the host `/dev`
  over whatever private `/dev/shm` Docker set up. A file created at
  `/dev/shm/x` inside the container is visible at `/dev/shm/x` on the host, and
  root-owned there. If you do hit a poisoned name, clear it as shown below.
  If you meet this on an older checkout, clear the stale segments with
  `sudo rm -f /dev/shm/carb-RStringInternals-* /dev/shm/sem.carb-RStringInternals-*`
  while no Kit process is running.
