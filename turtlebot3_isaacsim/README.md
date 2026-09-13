# turtlebot3_isaacsim

Isaac Sim simulation package for the TurtleBot3 — the counterpart of
[`turtlebot3_gazebo`](https://github.com/ROBOTIS-GIT/turtlebot3_simulations),
built to the same shape so that everything above it runs unchanged.

```bash
ros2 launch turtlebot3_isaacsim turtlebot3_world.launch.py
```

```bash
# second terminal — identical to Gazebo, identical to the real robot
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
    use_sim_time:=true map:=$HOME/map.yaml
```

`use_sim_time:=true` remains the only thing a consumer package changes.

## What it provides

The interface the real robot presents, so `turtlebot3_navigation2`,
`turtlebot3_cartographer` and `turtlebot3_teleop` cannot tell the difference:

| topic | real robot | this package |
|---|---|---|
| `/clock` | — (wall time) | `ROS2PublishClock` |
| `/scan` | `ld08_driver` | RTX lidar → `RtxLidarROS2PublishLaserScan` |
| `/odom` | `turtlebot3_node` | `IsaacComputeOdometry` → `ROS2PublishOdometry` |
| tf `odom`→`base_footprint` | `turtlebot3_node` | `ROS2PublishRawTransformTree` |
| `/joint_states` | `turtlebot3_node` | `ROS2PublishJointState` |
| `/cmd_vel` | `turtlebot3_node` | `ROS2SubscribeTwist` → `DifferentialController` |
| tf `base_footprint`→… | `robot_state_publisher` + the TB3 URDF — the same node and the same file in all three cases |

Only a *raw* `odom`→`base_footprint` transform is published. Everything below
`base_footprint` is `robot_state_publisher`'s, which is what lets one URDF serve
the real robot, Gazebo and Isaac Sim without any of them drifting.

## Prerequisites

**1. NVIDIA's `isaacsim_bringup`.** This package depends on it the way
`turtlebot3_gazebo` depends on `gazebo_ros`: it is the vendor's own "start the
simulator from a launch file". Vendor it unmodified into your workspace:

```bash
cp -r /path/to/IsaacSim-ros_workspaces/humble_ws/src/isaacsim_bringup src/
colcon build --packages-select isaacsim_bringup
```

**2. The robot asset.** USD models are build artifacts, not committed files.
From a ROS 2 shell:

```bash
export TURTLEBOT3_MODEL=burger
# from a source checkout
./scripts/build_models.sh burger
# or, against the installed package
"$(ros2 pkg prefix --share turtlebot3_isaacsim)"/scripts/build_models.sh burger
```

See [`models/README.md`](models/README.md).

**3. A world**, for `turtlebot3_world.launch.py`. See
[`worlds/README.md`](worlds/README.md). `empty_world.launch.py` needs no world
and is the right thing to run first.

## Launch files

| file | role | Gazebo counterpart |
|---|---|---|
| `turtlebot3_world.launch.py` | robot in the TB3 world | same name |
| `empty_world.launch.py` | robot on a bare ground plane | same name |
| `isaacsim.launch.py` | starts the simulator | `gzserver.launch.py` |
| `robot_state_publisher.launch.py` | the URDF → tf | same name |

Two Gazebo launch files have no counterpart here, both for structural reasons
rather than by omission:

- **`gzclient.launch.py`** — Gazebo splits server and GUI into two processes.
  Kit is one process that either opens a window or does not, so the GUI is
  `headless:=` on `isaacsim.launch.py`.
- **`spawn_turtlebot3.launch.py`** — `gazebo_ros` ships
  `libgazebo_ros_factory.so`, serving a `spawn_entity` service, so a robot can
  be pushed into an already-running server. Isaac Sim ships no equivalent:
  NVIDIA's service surface (`isaac_ros2_messages`) is `GetPrims`,
  `Get`/`SetPrimAttribute` and `IsaacPose` — inspection, attribute setting and
  pose teleport, but nothing that *creates* a prim. The robot is therefore
  referenced into the stage by the simulator script at startup, and
  `x_pose`/`y_pose` are forwarded to it. The argument names are kept identical.

## Common arguments

```bash
ros2 launch turtlebot3_isaacsim turtlebot3_world.launch.py \
    x_pose:=-2.0 y_pose:=-0.5 headless:=true
```

`isaacsim.launch.py` additionally takes `world`, `robot`, `yaw`, `lidar`,
`lidar_config`, `physics_hz`, `namespace`, and the Isaac Sim placement
arguments `isaac_install_path`, `isaac_version`, `ros_distro`,
`use_internal_libs`, `exclude_install_path`, `dds_type`.

## Status

Structurally complete and readable end to end; **not yet run**. What is carried
over from a verified implementation, and what is new, is worth separating:

**Carried over from a working setup** — the OmniGraph wiring, the lidar writer
attachment, the physics-material story, the `set_pose` fallback, and the
`os._exit` teardown. These were measured live: `/clock` 60 Hz, `/odom` 60 Hz,
`/joint_states` 60 Hz, `/scan` 10 Hz, `/cmd_vel` driving the robot for real.

**New here, and unverified:**

- `models/lidar_configs/turtlebot3_lds.json` — an authored TB3 LDS profile,
  360 samples/rev at 5 Hz with **0° elevation**. This replaces Isaac Sim's stock
  `Example_Rotary_2D`, a 200 m survey lidar whose single emitter sits at
  `elevationDeg = [-2.0]` and therefore scans the floor. Matches
  `turtlebot3_gazebo`'s `<ray>` block (0.12–3.5 m, 1°/sample, 5 Hz).
- `rotationDirection` is `CW`, copied from the stock profile rather than
  guessed at. Whether that yields REP-103 ordering in the published `LaserScan`
  is **the first thing to check** against Gazebo — a mirrored scan is a
  correctness bug that still looks plausible.
- Gazebo's ray sweeps `0 → 6.28` rad; the writer here is configured
  `-180° → +180°`. Self-consistent, but the two backends do not label the same
  ray with the same index.
- No-return is reported as `-1.0` by the RTX writer where Gazebo publishes
  `inf`. Nav2's obstacle layer filters on exactly that field.
- `author_surfaces()` writes materials *into the asset* rather than into a
  composed world stage. The logic is verified; this placement of it is not.
- `find_articulation_root()` searches for the root instead of hard-coding
  `…/Geometry/base_footprint`. More robust, but a different code path.
- Only `burger` geometry has been checked against the URDF. `waffle` and
  `waffle_pi` wheel and scan offsets come from `turtlebot3_gazebo`'s SDF.
- No camera. `waffle_pi`'s `libgazebo_ros_camera` plugin has no counterpart here
  yet.
