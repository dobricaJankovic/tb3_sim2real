# tb3_sim2real

Nav2 on a TurtleBot3, against **three interchangeable backends** — the real
robot, Gazebo Classic, and Isaac Sim — from one launch command.

```bash
ros2 launch tb3_bringup bringup.launch.py backend:=gazebo
ros2 launch tb3_bringup bringup.launch.py backend:=isaacsim
ros2 launch tb3_bringup bringup.launch.py backend:=real
```

`use_sim_time` is derived from `backend`. You should never pass it by hand again.

## Architecture in one paragraph

Two containers, one DDS domain. **`isaacsim`** (Ubuntu 24.04, Kit Python 3.12)
runs the simulator using the ROS 2 Humble libraries bundled inside
`isaacsim.ros2.core` — no ROS installed, no ROS packages, no launch files.
**`tb3_ros`** (Ubuntu 22.04, system Humble, Python 3.10) runs everything that is
ROS: Nav2, RViz, `robot_state_publisher`, Gazebo, the real-robot drivers, and
this package. They only ever meet on the DDS wire.

**The Python 3.10 / 3.12 split is not a problem.** It would only matter if ROS 2
Python code had to run *inside* Kit's interpreter. It doesn't — Isaac Sim's
OmniGraph ROS 2 nodes are C++ against bundled libs that include
`libfastrtps.so.2.6.10`, the same Fast-DDS version stock Humble ships. Same RTPS,
same Humble message definitions. A single container is also blocked outright:
`isaacsim-6.0/tools/docker/Dockerfile` is `FROM nvcr.io/nvidia/base/ubuntu:noble`,
and Humble has no binaries for noble.

Full write-up: <https://claude.ai/code/artifact/f7809bfe-4918-4977-938c-e008f28c46e3>

## Layout

```
docker/ros/            the tb3_ros image (Humble + Nav2 + Gazebo + TB3)
docker-compose.yml     both services, network_mode+ipc host, shared ROS_DOMAIN_ID
isaac/scenes/          TB3 USD stages (gitignored — decide on LFS later)
isaac/scripts/         build_scene.py — the OmniGraph ROS 2 graph, scripted
scripts/               one-off host/container helpers
tb3_bringup/
  launch/bringup.launch.py     ← single entry point, dispatches on backend
  launch/backends/             ← the only layer that varies
  launch/common/               ← state_publisher + nav2, identical everywhere
  config/nav2_<backend>.yaml   ← same structure, divergent tuning
```

## The interface contract

All three backends must present an identical surface to Nav2.

| | real | gazebo | isaacsim |
|---|---|---|---|
| `/scan` | ld08_driver | gazebo lidar plugin | `ROS2PublishLaserScan` |
| `/odom` | turtlebot3_node | diff_drive plugin | `ROS2PublishOdometry` |
| tf `odom→base_footprint` | turtlebot3_node | diff_drive plugin | `ROS2PublishRawTransformTree` |
| `/joint_states` | turtlebot3_node | joint_state plugin | `ROS2PublishJointState` |
| `/cmd_vel` | turtlebot3_node | diff_drive plugin | `ROS2SubscribeTwist` |
| `/clock` | — (wall time) | gazebo_ros_init | `ROS2PublishClock` |
| tf `base_footprint→*` | **`robot_state_publisher` + TB3 URDF — the same node and file in all three** |||

That last row is the design's load-bearing decision. The URDF is the single
source of truth for the kinematic tree, so each backend supplies only a *raw*
`odom→base_footprint` transform and the two sims cannot drift from the real
robot's geometry.

## Getting started

```bash
docker compose build tb3_ros
docker compose up -d isaacsim          # or skip, for gazebo/real
docker compose run --rm tb3_ros

# inside tb3_ros:
scripts/seed_nav2_params.sh            # populate the three placeholder configs
colcon build --symlink-install && source install/setup.bash
ros2 launch tb3_bringup bringup.launch.py backend:=gazebo
```

### Where the two images come from

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

## Build order

Each step leaves something verifiable. Don't jump to Nav2 — it hides frame and
QoS mismatches, which is where nearly all the failures are.

1. **Prove the DDS link.** Any ROS 2 sample scene in Isaac Sim, press Play, then
   `ros2 topic echo /clock --once` from `tb3_ros`. Failure here is
   `ROS_DOMAIN_ID` or `/dev/shm`, nothing exotic.
2. **Get a TurtleBot3 into Isaac Sim.** No TB3 asset ships with Isaac Sim —
   import the URDF, fix up the articulation and drive joints, save to
   `isaac/scenes/`. This is real work; budget for it.
3. **Build the OmniGraph graph** via `isaac/scripts/build_scene.py`, and pin the
   frame names. Verify with `ros2 topic hz` and `ros2 run tf2_tools view_frames`.
4. **Write the backends, Gazebo first** — it's the one you already know works.
   Once `backend:=gazebo` reproduces your current two-terminal workflow, the
   dispatch code is done and the other two are just their own launch file.
5. **Tune Nav2 per backend, last.**

## Things that fail silently

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
  `/tmp/.X11-unix`. `xhost +local:docker` on the host if you hit permission errors.

## Status

Skeleton. Nothing here has been run yet.

- `config/nav2_*.yaml` are placeholders — run `scripts/seed_nav2_params.sh`.
- `rviz/tb3.rviz` is a stub; save a real one out of RViz.
- `isaac/scripts/build_scene.py` has plausible but unverified node/attribute
  names, and no lidar yet.
- No TB3 USD exists.
