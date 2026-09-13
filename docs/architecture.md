# Architecture

One container. **`tb3_ros`** (Ubuntu 22.04, system Humble, Python 3.10) runs
everything that is ROS — Nav2, RViz, `robot_state_publisher`, Gazebo, the
real-robot drivers, this package — *and* Isaac Sim 6.0 at `/isaac-sim`, which
Kit runs on its own bundled Python 3.12. The two interpreters never meet;
`ros-isolate` keeps the system installation out of Kit's search paths, and the
simulator reaches the rest of the system over DDS exactly as it did when it was
a separate container.

**The Python 3.10 / 3.12 split is not a problem.** Not because ROS 2 Python
stays out of Kit — it does not. `isaacsim.ros2.core` loads an `rclpy` during
startup, and the logs say so:

    Attempting to load system rclpy
    Could not import system rclpy: No module named 'rclpy'
    Attempting to load internal rclpy for ROS Distro: humble
    rclpy loaded

Isaac Sim 6 added the option to use *your* ROS installation inside Kit
(`use_internal_libs:=false` plus `ros_installation_path:=`), so it probes for a
system `rclpy` first. We cannot take that branch and do not want to: Kit runs
Python 3.12 and Humble's `rclpy` is a `cpython-310` extension module. The
fallback to the internal Humble build — rebuilt by NVIDIA against 3.12 — is the
supported path, and `use_internal_libs:=true` is its default. The OmniGraph
nodes themselves are C++ against bundled libs that include
`libfastrtps.so.2.6.10`, the same Fast-DDS version stock Humble ships. Same
RTPS, same Humble message definitions.

If we ever need custom message types or our own Python nodes running *inside*
Kit, that is what `IsaacSim-ros_workspaces`' `ubuntu_22_humble_python_312`
dockerfile exists for: it rebuilds Humble against 3.12 so
`ros_installation_path` can point at it. Nothing in the interface contract
below needs it.

Full write-up: <https://claude.ai/code/artifact/f7809bfe-4918-4977-938c-e008f28c46e3>

## One container or two

One container is what the repo does today, since 2026-09-13. Two containers is
what it did before, and the reason recorded here for that was wrong; the way it
was wrong is the part worth keeping.

Tested on 2026-09-06: both combined images built, `verify.sh` scored
`isaacsim6-jazzy` 5/5 and `isaacsim6-humble` 3/5. Nine of 1104 shared objects
under `/isaac-sim/exts` required `GLIBC_2.38`, which only noble provides, and
three of those nine were the bridge — `libisaacsim.ros2.core.humble.so`,
`libisaacsim.ros2.core.jazzy.so`, `libisaacsim.ros2.nodes.plugin.so`. This
document concluded that on 22.04 Isaac Sim "cannot work with NVIDIA's
binaries."

Those were not NVIDIA's binaries. The image under test, `isaac-sim-docker:latest`,
was built locally from `~/isaacsim-6.0` on *this noble host*, against its glibc
2.39. The conclusion was about the build host, not about Isaac Sim.

Re-measured on 2026-09-13 against the shipped release: across all 3189 shared
objects in `isaac-sim-standalone-6.0.0-linux-x86_64` the ceiling is
`GLIBC_2.35` — exactly jammy — and the three bridge libraries are at 2.34. That
floor is deliberate; it is what Isaac Sim's README means by "Operating System:
Windows 11 or Linux (Ubuntu 22.04/24.04)".

Confirmed end to end the same day, changing only the binaries: the official tree
mounted over `isaacsim6-humble:latest` with `ISAACSIM_PATH` pointed at it,
`isaacsim.ros2.core` / `.nodes` / `.bridge` all started, and
`ros2 topic echo /clock` from the system Humble **in the same container**
returned sim time. A second run against a writable tree logged no errors at all;
the RTX and Python-node-registration complaints in the first were artifacts of
a read-only mount.

Then built for real and re-tested the same day. `Dockerfile.humble` with
`ISAACSIM_IMAGE=nvcr.io/nvidia/isaac-sim:6.0.1`, `verify.sh` unmodified:

    5 passed, 0 failed, 0 skipped

Same script and same image name that scored 3/5 on 2026-09-06. The NGC
artifact's bridge libraries were checked first and are `GLIBC_2.34`, matching
the tarball, so the release floor holds for the container as well as the zip.

So one Ubuntu 22.04 container — official ROS 2 Humble plus the official Isaac
Sim release — is supported ground, and every part stays a vendor artifact. That
was the standard the 2026-09-06 decision was made against, so the decision is
reopened. Design note:
<https://claude.ai/code/artifact/e83a0463-ff46-4f38-b227-c025cd4b5a7e>

What survives from the earlier write-up regardless:

- **`ros-isolate` is still needed.** Isaac Sim's `setup_python_env.sh` *appends*
  to `PYTHONPATH`/`LD_LIBRARY_PATH`, so a sourced ROS 2 stays ahead of Kit's own
  entries. This is not a workaround: NVIDIA's own `isaacsim_bringup` launcher
  strips the same three variables when `use_internal_libs` is true.
- **A single-command launch never required a single container.**
- **The real wins of merging come from one uid, not from one container.** The
  1234-vs-root split is what forced UDP-only Fast-DDS and two X cookie paths.
  `ipc: host` must stay off either way — that was carb's PID-named segment.

The standing lesson: a locally built vendor artifact is not the vendor's
artifact, and measuring the wrong one can retire a design on a fact that is not
true.

## Layout

```
docker/isaacsim-ros2/  Isaac Sim 6.0 + ROS 2 Humble, generic; verify.sh scores it
docker/ros/            + TurtleBot3, drivers, worlds, workspace -> tb3_ros
scripts/build_images.sh  builds the two in order (the second is FROM the first)
docker-compose.yml     one service, network_mode host, private IPC, cache volumes
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
