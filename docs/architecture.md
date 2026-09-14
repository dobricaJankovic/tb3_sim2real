# Architecture

One container. **`tb3_ros`** (Ubuntu 22.04, system Humble, Python 3.10) runs
everything that is ROS — Nav2, RViz, `robot_state_publisher`, Gazebo, the
real-robot drivers, this package — *and* Isaac Sim 6.1 at `/isaac-sim`, which
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

## What is written here, and what is imported

The Isaac Sim backend is not in this repository. `turtlebot3_isaacsim` is a peer
of `turtlebot3_gazebo`, developed as a standalone package in its own repository,
and it owns the robot asset, the OmniGraph that publishes the interface
contract, the RTX lidar profile, the physics materials, the occupancy-map
builder — and the container image that all of that needs. It is imported by
`scripts/workspace.sh` from the pins in `tb3_sim2real.repos`, together with
NVIDIA's `isaacsim_bringup`, which it includes in turn.

A copy of it here would be a second source of truth for the Isaac Sim side,
which is the one thing this repository exists to prevent. There was one, and by
the time it was removed it was four worlds, a map builder and a whole Isaac Sim
version behind. The same argument applies to its Dockerfile: the Isaac Sim +
ROS 2 base image is built from *that* repository's definition, and `docker/`
here is one thin layer on top.

What is written here is the part that is genuinely about sim-to-real: the world
registry and its generators, the launch layer that makes one `world:=` mean the
same environment on three backends, and the drift check that proves it still
does.

## Layout

```
docker/Dockerfile      one thin layer: TurtleBot3, Gazebo worlds, drivers, workspace
docker-compose.yml     one service, network_mode host, cache volumes
tb3_sim2real.repos     the source dependencies; scripts/workspace.sh imports them
src/                   where they land (gitignored)
scripts/
  workspace.sh         vcs import + colcon ignores
  build_images.sh      base (from the imported package) then this one
  build_world.py       world.yaml -> <name>.world       (host, PyYAML only)
  build_world_usd.py   world.yaml -> isaac/<name>.usd   (Isaac Sim's interpreter)
  build_world.sh       both, in one command
  clone_world.py       a real room's Nav2 map -> a world directory
  check_worlds.py      proves the representations still match the manifest
worlds/<name>/         the registry: world.yaml, meshes/, map/, generated wrappers
tb3_bringup/
  launch/bringup.launch.py     <- single entry point, dispatches on backend
  launch/backends/             <- the only layer that varies
  launch/common/               <- state_publisher + nav2, identical everywhere
  config/nav2_params.yaml      <- shared; nav2_<backend>.yaml wins where it exists
  tb3_bringup/worlds.py        <- the registry, read by the launch AND the generators
```

`worlds.py` being read by both sides is deliberate and load-bearing: the
generators run under Kit's own interpreter with the system ROS 2 stripped off
the search path, so it is plain Python and PyYAML with no ROS import. Both sides
therefore resolve `world:=` through the same code and cannot disagree about what
a world is.

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
| the environment | already around it | `<world>` in the `.world` | `/World/env` from the `.usd` |
| where it starts | you put it there | `world.yaml`'s `spawn` ||
| the Nav2 map | `world.yaml`'s `map:` — the same file for all three |||
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
2. **Get a TurtleBot3 into Isaac Sim.** No TB3 asset ships with Isaac Sim, so
   the URDF has to be imported and its articulation and drive joints fixed up.
   This was real work and it is *done*, in `turtlebot3_isaacsim`, which commits
   the resulting asset so a clone launches without a GPU: `scripts/
   import_turtlebot3.py` there is how to regenerate it, not how to obtain it.
   This repository did it from first principles once and then deleted its copy.
3. **Build the OmniGraph graph.** Also done, and also there
   (`scripts/turtlebot3_isaacsim.py`). Verify with `ros2 topic hz` and
   `ros2 run tf2_tools view_frames` — the frame names are the contract.
4. **Write the backends, Gazebo first** — it's the one you already know works.
   Once `backend:=gazebo` reproduces your current two-terminal workflow, the
   dispatch code is done and the other two are just their own launch file.
5. **Tune Nav2 per backend, last.** There is one `config/nav2_params.yaml`
   until then; `bringup.launch.py` prefers `config/nav2_<backend>.yaml` the day
   one exists. Three identical copies of upstream's file are a worse record of
   "the backends do not differ yet" than one file is.

## The environment contract

The robot's half of sim-to-real is solved by one URDF. The environment's half is
solved the same way, one level up: `worlds/<name>/world.yaml` is the source of
truth and both simulators' representations are *generated* from it, so neither
is hand-authored and the two cannot drift. `worlds/README.md` is the full
account, including how a real room is cloned into it from the occupancy map you
already recorded for Nav2, and what `scripts/check_worlds.py` catches.

Steps 2-3 above were about the robot. The world registry is the same argument
applied to the room, and the two together are what make a Gazebo run, an Isaac
Sim run and a real run comparable measurements rather than three demos.
