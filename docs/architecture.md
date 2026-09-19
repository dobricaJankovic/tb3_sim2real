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
  config/nav2_params.yaml      <- shared, unconditionally; see step 5 below
  tb3_bringup/worlds.py        <- the registry, read by the launch AND the generators
```

`worlds.py` being read by both sides is deliberate and load-bearing: the
generators run under Kit's own interpreter with the system ROS 2 stripped off
the search path, so it is plain Python and PyYAML with no ROS import. Both sides
therefore resolve `world:=` through the same code and cannot disagree about what
a world is.

## The launch entry point

`bringup.launch.py` is two layers, the way ROS already splits them:

  robot layer        drivers, odometry, robot_state_publisher. Supplied by a
                     simulator, or by the hardware itself.
  workstation layer  map server, localization, Nav2, RViz. Identical
                     everywhere; it does not know which robot it is driving.

`backend:=` answers exactly one question — who provides the robot layer.
`gazebo` and `isaacsim` start it in this file; `real` starts nothing here,
because the robot is a second computer already running its own
`turtlebot3_bringup`. That is why the file contributes no local processes for
`backend:=real`: correct, not broken. See `docs/roadmap.md` for the tethered
alternative this deliberately is not.

`world:=` names the ENVIRONMENT, not a file, and means the same thing
everywhere: the backend picks up whichever representation of it applies —
gzserver loads its `.world`, Kit opens its `.usd`, and the real robot loads
nothing because the environment is already around it. All three take the
robot's start pose and the Nav2 map from the same manifest, which is what
makes a run on one backend comparable with a run on another.

Both `backend:=` and `world:=` are required, with no default. A default would
silently attribute every unqualified run to one world — including a
real-robot run in a room that is not that world, where the map is simply
wrong and Nav2 localises into fiction rather than failing.

`bringup.launch.py` includes `nav2_bringup`'s `slam_launch.py`,
`localization_launch.py` and `navigation_launch.py` directly, rather than
going through its own `bringup_launch.py`. That wrapper exists to make `slam`
choose between `slam_launch.py` and `localization_launch.py` with an
`IfCondition(['not ', slam])`; here the three workstation modes are a flat
dispatch already (see the module docstring), so the wrapper has nothing left
to do. One consequence: `localization_launch.py` and `navigation_launch.py`
default `use_composition` to `False`, where `bringup_launch.py` defaults it
`True` and creates the shared `nav2_container` itself — so Nav2's nodes run as
separate processes here. Slightly more overhead, and it retires the failure
mode in `docs/network.md` where the container starts and no composable node is
ever loaded into it.

## The interface contract

All three backends must present an identical surface to Nav2.

| | real | gazebo | isaacsim |
|---|---|---|---|
| `/scan` | `hls_lfcd_lds_driver` (LDS-01) | gazebo lidar plugin | `ROS2PublishLaserScan` |
| `/odom` | turtlebot3_node | diff_drive plugin | `wheel_odometry` node |
| tf `odom→base_footprint` | turtlebot3_node | diff_drive plugin | `wheel_odometry` node |
| `/ground_truth/odom` | **nothing — there is none** | P3D plugin | `IsaacComputeOdometry` |
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

**Measured against the hardware, 2026-09-19** (`docs/experiment-plan.md` B2, B3;
`measurements/2026-09-19_real_interface.json`). Two rows of that table were
claims until then:

- The `/scan` row said `ld08_driver`, which is the LDS-02 driver. It is wrong,
  and is corrected above. The running node is
  `hls_lfcd_lds_driver/hlds_laser_publisher` and `/scan` reports
  `range_max: 3.5` — an **LDS-01**, which is what `.env` says and what both
  simulators model. `ld08_driver` *is* checked out in the robot's own
  `turtlebot3_ws`, unbuilt into the running graph, which is presumably where
  the claim came from.
- The last row rests on the robot and the container independently having the
  same `turtlebot3_description`. They do, and by more than version number:
  `turtlebot3_burger.urdf` is **byte-identical**, md5
  `51b1f9517b2666efefee18310009703d`, between the robot's source checkout
  (2.3.7, built in `~/turtlebot3_ws`) and the container's apt package (2.3.6).
  `base_footprint → base_scan` measures `(-0.032, 0, 0.182)` on the robot,
  exactly the figure Isaac Sim's asset uses.

### `/odom` means the same thing on all three, and that took work

All three integrate it from the wheels, so all three **drift**. Isaac Sim did
not until 2026-09-18: `IsaacComputeOdometry` reads the chassis prim, so its
`/odom` was the robot's true pose. That is what NVIDIA's reference graph wires
up and there is no stock encoder-odometry node to wire instead, so the package
had inherited it.

Odometry that cannot drift is not a harmless luxury. Nav2's entire job above
`odom` is to correct that drift; a backend without it makes localisation
unrealistically easy on exactly one of the three, which is a sim-to-real gap
manufactured by the apparatus. It also concealed real wheel slip — `/odom` now
over-reports a pivot by **4.8%** and a straight line by 0.08%, which is what a
burger pivoting on two wheels and a plastic skid actually does.

The true pose is still available on `/ground_truth/odom`, on both simulators,
and **on neither the real robot nor any hardware** — which is the point: the
three-layer decomposition in `docs/experiment.md` (command → wheel → body →
odom) is a measurement only a simulator can provide, not a workaround for one.

Two asymmetries worth knowing before using it. Gazebo's `/odom` is integrated
from the wheels *by the same plugin that drives them*, so it agrees with a
forward integration of `/joint_states` by construction and slip is invisible in
it; Isaac's is computed by a separate node from the published joint states.
And the ground-truth origins differ — Gazebo's P3D is world-absolute, Isaac's
is relative to the spawn pose — so consumers use deltas, never absolute
positions.

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
5. **Do not tune Nav2 per backend.** There is exactly one
   `config/nav2_params.yaml`, used unconditionally — `bringup.launch.py` no
   longer auto-prefers a `config/nav2_<backend>.yaml` if one appears. Per-backend
   tuning would absorb the sim-to-real gap into the tuning and make it
   unmeasurable, which is the opposite of what this repository is for; if the
   backends are ever deliberately tuned differently, that is a measured
   decision made in code, not a file that happens to exist.

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
