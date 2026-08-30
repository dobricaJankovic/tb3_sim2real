# Getting started

```bash
docker compose build tb3_ros
docker compose up -d isaacsim          # or skip, for gazebo/real
docker compose run --rm tb3_ros

# inside tb3_ros:
scripts/seed_nav2_params.sh            # populate the three placeholder configs
colcon build --symlink-install && source install/setup.bash
ros2 launch tb3_bringup bringup.launch.py backend:=gazebo
```

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
