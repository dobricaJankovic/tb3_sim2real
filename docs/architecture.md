# Architecture

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
same Humble message definitions.

**A single container is blocked by glibc, not by Python.** Tested on 2026-09-06;
see `docker/isaacsim-ros2/`, which builds exactly that image. Kit runs on jammy —
the `kit` binary and `libcarb.so` need at most `GLIBC_2.34` and jammy has 2.35 —
and so do 1085 of Isaac Sim's 1094 extension libraries. Nine need `GLIBC_2.38`,
which only noble provides, and three of those nine are the bridge:
`libisaacsim.ros2.core.humble.so`, `libisaacsim.ros2.core.jazzy.so` and
`libisaacsim.ros2.nodes.plugin.so`. Both bundled distros fail the same way, so
this is a property of NVIDIA's build host rather than a Humble/Jazzy question.
The same image on noble/Jazzy passes end to end, so the arrangement is sound and
only the prebuilt binaries are in the way; building Isaac Sim from source on
jammy (`setup.sh`: "Tested on: Ubuntu 22.04 / 24.04") would close it.

Full write-up: <https://claude.ai/code/artifact/f7809bfe-4918-4977-938c-e008f28c46e3>

## Layout

```
docker/ros/            the tb3_ros image (Humble + Nav2 + Gazebo + TB3)
docker-compose.yml     both services, network_mode+ipc host, shared ROS_DOMAIN_ID
isaac/scenes/          TB3 USD stages (gitignored — decide on LFS later)
isaac/scripts/         tb3_sim.py — the simulator launcher: stage + graph + play()
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

## Build order

Each step leaves something verifiable. Don't jump to Nav2 — it hides frame and
QoS mismatches, which is where nearly all the failures are.

1. **Prove the DDS link.** Any ROS 2 sample scene in Isaac Sim, press Play, then
   `ros2 topic echo /clock --once` from `tb3_ros`. Failure here is
   `ROS_DOMAIN_ID` or `/dev/shm`, nothing exotic.
2. **Get a TurtleBot3 into Isaac Sim.** No TB3 asset ships with Isaac Sim —
   import the URDF, fix up the articulation and drive joints, save to
   `isaac/scenes/`. This is real work; budget for it. Gate it on
   `isaac/scripts/verify_asset.py`: a mesh-less import still loads without error.
3. **Build the OmniGraph graph** via `isaac/scripts/tb3_sim.py`, and pin the
   frame names. Verify with `ros2 topic hz` and `ros2 run tf2_tools view_frames`.
4. **Write the backends, Gazebo first** — it's the one you already know works.
   Once `backend:=gazebo` reproduces your current two-terminal workflow, the
   dispatch code is done and the other two are just their own launch file.
5. **Tune Nav2 per backend, last.**
