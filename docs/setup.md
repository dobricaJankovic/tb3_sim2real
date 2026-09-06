# Getting started

```bash
./docker/x11-auth.sh                   # once per X session, before any GUI container
docker compose build tb3_ros
docker compose up -d isaacsim          # or skip, for gazebo/real
docker compose run --rm tb3_ros

# inside tb3_ros:
scripts/seed_nav2_params.sh            # populate the three placeholder configs
colcon build --symlink-install && source install/setup.bash
ros2 launch tb3_bringup bringup.launch.py backend:=gazebo
```

## Environments

Worlds come from the registry in `worlds/`, not from `turtlebot3_gazebo`'s
installed worlds. `worlds/<name>/world.yaml` is the source of truth and both
simulators' wrappers are generated from it — see `worlds/README.md`.

The Gazebo `.world` is committed, so `backend:=gazebo` needs no build step. The
Isaac Sim `.usd` is gitignored and must be built once per clone, per world:

```bash
scripts/build_world_usd.sh turtlebot3_world   # ~1 min, needs the isaacsim image
```

Skipping it does not fail quietly: `tb3_sim.py` checks for the generated stage
and names this command. Rebuild it after editing a manifest or its meshes; also
rerun `scripts/build_world.py <name>` for the Gazebo side, since editing the
manifest alone changes neither generated file.

## X11 for the GUI containers

Neither container runs as *you*, but the host X server's access control is
per-user, so a plain `DISPLAY` + `/tmp/.X11-unix` mount isn't enough —
`gzclient`/RViz/Kit abort with `Authorization required, but no authorization
protocol specified`. `docker/x11-auth.sh` writes a docker-specific Xauthority
cookie that compose mounts into both services. Run it once per X session
(log in / restart X) before bringing up either container — the cookie rotates
on login, so a stale one fails the same way.

The two services mount that cookie at **different paths**, and it matters:
`tb3_ros` runs as root and reads it from `/root/.Xauthority`, but the Isaac Sim
image ends with `USER isaac-sim`, so `isaacsim` runs as uid 1234 and cannot
traverse `/root` — it reads the cookie from `/tmp/.docker.xauth` instead.
Getting this wrong fails *silently*: Kit logs `GLFW initialization failed` and
`IAppWindow::startup failed`, then runs on with no window, while the container's
healthcheck still reports `healthy` because it only greps the log for
`AppReady`. If you think Isaac Sim is up but see no window, check the Kit log
under `/isaac-sim/.nvidia-omniverse/logs/Kit/` before anything else.

See `docs/troubleshooting.md` for the full explanation.

## Where the two images come from

Only `tb3_ros` is built by this repo. The `isaacsim` service has an `image:` key
and no `build:` key, so `docker compose build` skips it — it expects
`isaac-sim-docker:latest` to already exist on the machine.

That image is built out of the Isaac Sim source tree, not here:

```bash
cd ~/isaacsim-6.0/tools/docker
./prep_docker_build.sh     # build.sh -r, then rsync the runtime into _container_temp
./build_docker.sh          # docker buildx from that context -> isaac-sim-docker:latest
```

`build_docker.sh` already defaults to the `isaac-sim-docker:latest` tag our
compose file references, so no arguments are needed. Takes roughly an hour.
Only re-run it when Isaac Sim's own source changes.

To run Isaac Sim standalone, outside this project, that repo also ships
`./run_docker.sh`, which drops you into a bash shell in the container with
`runapp` (GUI) and `runheadless` (WebRTC) aliases on the PATH. We don't use it
here — `docker compose up -d isaacsim` does the equivalent with the DDS-critical
settings (`ipc: host`, `ROS_DOMAIN_ID`) already wired in.

The asymmetry is the point: `tb3_ros` is ~10 min and you will rebuild it
constantly; the Isaac Sim image is 33 GB and you will almost never touch it.
