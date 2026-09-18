# Status

What is verified, and what is not. Start here before claiming something works.

## Verified on 2026-09-15

### Open-loop kinematics, one instrument, both backends

`ros2 run tb3_bringup drive_test`, `world:=empty_stage`, identical `/cmd_vel`
sequence, two runs per backend. Phases are timed on the **simulator's** clock.

| | gazebo | isaacsim |
|---|---|---|
| real-time factor | 0.9997 / 0.9995 | 0.9592 / 0.9517 |
| `straight` 0.15 m/s x 5 s (want 0.750 m) | 0.7358 m, **-1.9%** | 0.7175 / 0.7113 m, **-4.7%** |
| `rotate` 0.5 rad/s x 5 s (want 2.500 rad) | 2.4777 rad, **-0.9%** | 1.7211 / 1.7631 rad, **-30.3%** |
| `arc` displacement (want ~0.50 m) | 0.4466 m | 0.4445 / 0.4459 m |
| `arc` yaw (want 1.500 rad) | 1.4634 rad | 1.0729 / 1.0736 rad |
| drift at rest, before any motion | 2e-05 m, 0 rad | 0 m, 0 rad |
| run-to-run spread | **0.00000 on every metric** | 0.0063 m, 0.042 rad |

- **Neither backend drifts at rest.** `settle_pre` is the only phase never
  preceded by motion, and both are at zero. The nonzero numbers in the other
  stop phases are deceleration coast, not drift: `peak_vx` at the entry to
  `stop_1` is still 0.150 / 0.148 m/s, and both settle below 4e-04 m/s.
- **Gazebo is bit-identical between runs**; Isaac Sim is not, which is expected
  of GPU physics and is why two runs of each were taken. The spread is small
  enough to make the headline differences conclusive — except the arc's
  displacement, where spread and difference are the same size (0.0014 m) and
  the comparison is **inconclusive**.
- **Velocity onset differs.** Gazebo ramps over ~0.10 s; Isaac steps from
  exactly 0 to ~0.147 m/s in one sample. Not a fault, but it means the two do
  not agree on what happens in the first tenth of a second of any command.
- Isaac veers on the straight (-0.0085 / -0.0158 rad of yaw) where Gazebo holds
  0.00000 exactly.

### The one real defect: Isaac Sim's wheels chatter

**Superseded 2026-09-17.** The measurements below are correct and reproduce;
the diagnosis that followed them was wrong. Full write-up:
[`docs/worknotes/2026-09-17-lane-a-physics.md`](worknotes/2026-09-17-lane-a-physics.md),
which supersedes
[`measurements/isaac_angular_deficit.md`](../measurements/isaac_angular_deficit.md).

The wheels reach only 73% of the commanded joint velocity at `wz = 0.5` and
**48% at `wz = 0.2`**. That was read as a fixed friction torque against a
finite-gain velocity drive. It is not: **the drive damping was swept 10,000x
(1e3 to 1e7) and the error did not move**, so the number
`turtlebot3_isaacsim` marks `# TODO unverified gain` is not wrong, it is
irrelevant.

What those means were hiding is an oscillation. Commanded a steady -1.2121
rad/s the left wheel ranges over **-2.91 to +1.04 rad/s** and reverses
direction; 73% is its average. Gazebo's standard deviation on the same phase is
0.0000. Lift the robot off the ground and Isaac reproduces every commanded rate
**exactly**, so the whole deficit is created by the contact solve — and the
source is the wheel's cylindrical collider, which neither PhysX nor MuJoCo
rolls exactly. A sphere of the same radius cuts the chatter 40x at the original
timestep.

**Shipped: PhysX at 240 Hz** (four sub-steps per rendered frame), measured
2026-09-18 in `measurements/2026-09-18_isaacsim_sweep_240hz.json`. Wheel
tracking against the analytic command, Isaac Sim, same instrument and sequence
throughout:

| command | 60 Hz | **240 Hz** | 480 Hz |
|---|---|---|---|
| `vx = 0.10` | 0.976 / 0.973 | **1.000 / 0.999** | 1.003 / 1.000 |
| `vx = 0.15` | 0.985 / 0.982 | **1.000 / 0.999** | 0.999 / 0.998 |
| `vx = 0.22` | 0.991 / 0.988 | **1.000 / 0.999** | 1.000 / 0.999 |
| `wz = 0.2` | 0.483 / 0.475 | **0.999 / 0.869** | 0.987 / 0.878 |
| `wz = 0.5` | 0.744 / 0.744 | **0.969 / 0.964** | 0.972 / 0.963 |
| `wz = 1.0` | 0.875 / 0.875 | **1.007 / 1.007** | 1.011 / 1.012 |
| `wz = 1.5` | 0.959 / 0.893 | **1.002 / 1.006** | 0.998 / 1.009 |

**240 and 480 Hz are equivalent within run-to-run scatter**, so the cheaper rate
is the default: RTF 0.90 against 0.68. Only the slowest pivot, `wz = 0.2`, still
misses on one wheel.

The residual has moved from the actuator into **slip**. The wheels now turn as
commanded and the robot slides:

| | wheels say | body did | slip |
|---|---|---|---|
| `wz = 0.5` pivot | 2.3485 rad | 2.1826 rad | **−7.1%** |
| `wz = 0.2` pivot | 0.8750 rad | 0.7979 rad | **−8.8%** |
| `vx = 0.15` straight | 0.7365 m | 0.7325 m | −0.5% |

That is not a defect to remove. A burger pivoting on two wheels and a plastic
skid does slide, and Gazebo cannot show it at all — its wheels carry
`mu = 100000`. **The wheel collider is deliberately left as a cylinder** for the
same reason: a sphere contacts at a point, and the expectation is that Isaac is
the backend that resembles the real robot here.

### Isaac Sim's `/odom` is now integrated from the wheels

Changed and verified 2026-09-18
(`measurements/2026-09-18_isaacsim_wheel_odom.json`). It used to be the chassis
prim — the true pose — because that is what `IsaacComputeOdometry` reads and
what NVIDIA's reference graph wires up. All three backends now integrate
`/odom` from the wheels, and all three drift.

Measured against `/ground_truth/odom` in the same run:

| | `/odom` (wheel) | ground truth | odometry error |
|---|---|---|---|
| `rotate` `wz = 0.5`, yaw | 2.2501 rad | 2.1464 rad | **+4.83%** |
| `arc`, yaw | 1.2592 rad | 1.1918 rad | **+5.66%** |
| `straight`, distance | 0.7367 m | 0.7361 m | +0.08% |

Odometry over-reports every turn and is nearly exact in a straight line, which
is wheel slip in a pivot — a real phenomenon, previously invisible because
`/odom` could not be wrong. Gazebo cannot reproduce it at all: its wheels carry
`mu = 100000` and its `/odom` comes from the same plugin that drives them.

`/ground_truth/odom` now exists on **both** simulators and on neither hardware.
Its origins differ — Gazebo's P3D is world-absolute, Isaac's is relative to the
spawn pose (verified: spawned at `(-2.0, -0.5)`, reads `(-0.0, -0.0)`) — so
consumers use deltas.

### Isaac Sim's lidar is not rotated — every beam is 0 or 1 beam off

Measured 2026-09-17 with `ros2 run tb3_bringup scan_test`, against ranges
ray-cast from `worlds/small_office/world.yaml` — no reference scan, no
hand-measured room.

| | best-fit angular offset | residual rms |
|---|---|---|
| gazebo | **+0.000°** | 0.00242 m |
| isaacsim | **+0.544°** | 0.00608 m, from 0.01509 |

That 0.544° was read as a half-beam rotation for a day. It is not one.
Diagnosed 2026-09-18 by probing the sensor directly:
`docs/worknotes/2026-09-18-lidar-half-beam.md`.

With the profile's noise off, the azimuth the sensor reports and the azimuth
the geometry implies agree to **0.0000°**. The rays are aimed correctly. They
also fire at exactly integer degrees — and `ROS2PublishLaserScan` bins a return
by `floor((azimuth - azimuthRange[0]) / horizontalResolution)` and then labels
each bin with its **lower edge**, putting its bin boundaries at exactly the
same integer degrees. **Every ray lands exactly on a boundary**, and the
profile's own `azimuthErrorStd` decides which side.

So over 4320 rays, **50.7% are off by ~0.0° and 49.3% by a whole ~1.0°, and
0.0% by anything in between.** Half a beam is the mean of a coin flip, the same
way "under-rotates by 30%" was the mean of a chattering wheel. `scan_test`
averages 20 scans per beam, which is what turned the coin flip into a clean
0.544°; the residual never closing to Gazebo's 0.00242 m is the tell that no
single offset could remove it.

Nothing has been changed. `startAzimuthOffsetDeg = 0.5` in the profile would
make the error a constant +0.4998° instead of a coin flip — 34x less per-beam
scatter, same bias — but the half beam itself is structural to the node, the
profile belongs to `turtlebot3_isaacsim`, and trading angular noise for angular
bias is a question about AMCL and slam_toolbox rather than about the sensor.
See `docs/roadmap.md` section 6.

### Materials, and a fidelity bug in Gazebo too

Isaac Sim rendered every stage grey because the manifest's only statement about
colour was `Gazebo/White` — an Ogre script name, meaningful to Gazebo alone.
Colour is now data in the manifest and both generators write their own dialect.

Fixing it surfaced a second bug: upstream's `model.sdf` paints the wall
`Gazebo/FlatBlack` and the five hexagons `Gazebo/Green`, and the manifest had
dropped both, so all 15 bodies defaulted to white. **Gazebo had been rendering
`turtlebot3_world` wrong as well**, not only Isaac.

Verified by rendering both backends headless (`scripts/snapshot.py`,
`scripts/snapshot_gazebo.py`) and by resolving `ComputeBoundMaterial` on every
Gprim: 12/12 on `small_office`, 15/15 on `turtlebot3_world`.

### New in the registry

- **`small_office`** — 6.0 x 5.0 m room, five boxes plus a partition, seven mesh
  props from six OBJs (AWS RoboMaker, MIT-0). Builds for both backends; USD
  bounds match the value computed analytically from the OBJ vertices and the
  manifest placements, to four decimals.
- **`isaacsim_bringup` moved to the `IsaacSim-6.1.0` tag**, matching the
  installed simulator. The blocker was fixed at source in `turtlebot3_isaacsim`
  (`AnyLaunchDescriptionSource` + `run_isaacsim.launch.xml`). **Not pushed.**
- **`scripts/dae_to_obj.py` now applies visual-scene node transforms.** It used
  to warn and skip them, which on `gazebo_models`' `cafe_table.dae` put the
  tabletop flat on the floor while the overall height was only 38 mm out — an
  error `verify` cannot catch. Upstream's two meshes re-convert byte-for-byte
  identical, so `turtlebot3_world` is unaffected.

## Verified on 2026-09-14, after the restructure

One `ros2 launch` per backend, the world chosen by name, both simulated
backends measured from the ROS side in the `tb3_ros` container.

| | gazebo | isaacsim |
|---|---|---|
| `/clock` | 10.0 Hz | 56.1 Hz |
| `/scan` | 5.0 Hz | 3.5 Hz |
| `/odom` | 29.4 Hz | 66.0 Hz |
| `/joint_states` | 29.4 Hz | 56.5 Hz |
| `/cmd_vel` | subscribed | subscribed |
| tf `odom->base_footprint` | yes | yes |

- **`backend:=gazebo world:=turtlebot3_world`** — every contract topic live,
  robot at the manifest's spawn: tf reads `[-2.000, -0.500, 0.009]`.
- **`backend:=isaacsim world:=turtlebot3_world`** — Kit started *by the launch
  file*, via `turtlebot3_isaacsim` -> `isaacsim_bringup`. It opened the
  registry's generated stage (`/worlds/turtlebot3_world/isaac/turtlebot3_world.usd`)
  at the manifest's spawn and played. `/scan` returns the arena: real ranges
  from 0.60 m, not an empty plane. Cold start to first `/odom` was about 160 s
  on a cold shader cache.
- **`backend:=gazebo world:=empty_stage`** — comes up, robot at the origin.
- **`backend:=isaacsim world:=empty_stage`** — `artifacts.isaacsim.mode: none`
  correctly results in **no** `--world` on the simulator's command line, so Kit
  authors its own ground plane. Verified on the running process's argv.
- **`scripts/build_world.sh turtlebot3_world`** — regenerated both
  representations, from OBJ meshes (see below). The USD generator's own checks
  pass: 15 collision prims for 15 manifest bodies, all 6 mesh colliders at
  approximation `none`, 0 rigid bodies, and bounds matching the value computed
  analytically from the meshes and placements to four decimals.
  `verify: OK — Isaac stage matches the Gazebo world`.

### Parity, measured from the same meshes

One manifest, one pair of OBJ files, the robot at the manifest's spawn:

| | gazebo | isaacsim |
|---|---|---|
| rays | 360 | 360 |
| with a return | 324 | 307 |
| closest | 0.514 m | 0.521 m |
| furthest | 3.352 m | 3.366 m |

**7 mm apart on the closest return.** The two backends are measuring the same
geometry. For comparison, the same measurement against the *Collada* meshes
before the conversion read 324/360 and 0.511-3.357 m on Gazebo — so the mesh
conversion moved nothing.

The 324-vs-307 gap is the no-return encoding, not the geometry: Isaac reports
`-1.0` where Gazebo reports `inf`. See "Known open".
- **`scripts/check_worlds.py`** — 2/2 worlds consistent. Negative-tested against
  deliberately broken copies: a manifest edited without regenerating, a
  generated `.world` edited by hand with its header intact, and a map mirrored
  about its x axis. All three fail, each naming what is wrong.
- **`scripts/clone_world.py`** — round-tripped: the stock `turtlebot3_world`
  map cloned into a world directory scores 100% of the map modelled and 0% of
  the model absent from the map at the burger's 0.182 m beam height.
- **All four `nav` / `slam` combinations on gazebo**, measured 2026-09-16 by
  `ros2 node list` 34 s after launch. Neither flag: the backend and
  `robot_state_publisher`, nothing else. `nav:=true`: `amcl`, `map_server` and
  the full navigation stack under two lifecycle managers. `slam:=true`:
  `slam_toolbox` and `map_saver` **and no `amcl` or `map_server`**.
  `slam:=true nav:=true`: `slam_toolbox` plus the whole navigation stack, still
  with no `amcl` or `map_server` — Nav2 planning over a map slam_toolbox is
  drawing. `nav:=true` on a world with no map raises and names the fix;
  `slam:=true` on the same world does not. `backend:=real` with neither flag
  starts RViz and nothing else.
- **`nav:=true` on gazebo** — the whole stack up with **zero errors**: `amcl`,
  `map_server`, `planner_server`, `controller_server`, `behavior_server`,
  `bt_navigator`. `/map` is 384x384 at 0.05 m, served out of
  `worlds/turtlebot3_world/map/` rather than the bringup package.
- **`nav:=true` on isaacsim** — same eight nodes, same map, **zero errors**,
  both lifecycle managers answering `is_active: True`, and publishing one
  `/initialpose` at the manifest's spawn produced `map->odom` at exactly
  `[-2.000, -0.500]`. Measured on a freshly restarted container; see the trap
  below for why that matters.
- **`backend:=real`** — a real TurtleBot3 running its own stock
  `turtlebot3_bringup` on the Wi-Fi subnet, reached from the container across a
  router (2026-09-16, `docs/network.md`). All four robot nodes discovered,
  `/scan` at 5 Hz, `/odom` at 20 Hz, `tf` `odom->base_footprint` resolving.
  Nav2 loads: `map_server`, `amcl` and `controller_server` active and the local
  costmap updating at 1.7 Hz from the real lidar. Not verified past that —
  `planner_server` upward waits on `map->odom`, and the only map to hand was
  `turtlebot3_world`'s, which is not the room the robot is in.
- **`colcon build`** — 3 packages: `isaacsim_bringup`, `turtlebot3_isaacsim`,
  `tb3_bringup`.

### One difference between the backends, not a fault

The `odom` frame does not start in the same place. Gazebo's diff-drive plugin
puts `odom` at the world origin, so tf reads the spawn pose immediately; Isaac
Sim's `wheel_odometry` node starts its integration where the robot is, so tf
reads zero at spawn — as `IsaacComputeOdometry` did before it, so this is
unchanged by the 2026-09-18 odometry work. Both are valid odometry; AMCL
resolves the difference into `map->odom`.
It matters only if you compare raw `/odom` between backends without saying which
frame you mean.

### One trap, and it was mine, not the repository's

An earlier attempt at the Isaac Nav2 run logged

    Failed to change state for node: map_server
    Failed to bring up all requested nodes. Aborting bringup.

and it was tempting to write that up as Kit holding the machine while Nav2
configures. It was not. Leftover `ros2 launch` processes from previous tests
were still on the DDS domain: `pkill -INT -f "ros2 launch"` from inside a
`docker compose exec -T` that has already exited does not reach them, and
`pkill -f gzserver` in a subshell that exited first never runs at all. Two
lifecycle managers then fight over the same nodes, `ros2 node list` prints
every name twice, and a second gzserver dies with exit code 255 and no message.

On a restarted container the same run is clean. **If a backend misbehaves,
check `pgrep -af "ros2 launch|gzserver|isaac-sim"` before believing anything
else** — `docker compose restart tb3_ros` is the reliable reset, and costs a
1.5 s rebuild of `/ws/install`.

## Known open

- **`/scan` encodes no-return differently.** Isaac reports `-1.0` where Gazebo
  reports `inf`, and `-1.0` is not a valid `LaserScan` range — so the two
  backends disagree on the one field Nav2's obstacle layer filters on. It has
  not bitten a Nav2 run yet, because the obstacle layer discards out-of-range
  values either way, but it is a difference Nav2 is entitled to trip over.
  It belongs in `turtlebot3_isaacsim`, which owns the lidar profile.

  The *geometry* half of this is fixed and this note used to be wrong about it:
  Isaac published 3600 rays against Gazebo's 360 while this repository carried
  its own hand-rolled simulator. The imported package ships a real LDS scan
  pattern (`models/lidar_configs/turtlebot3_lds.json`) and publishes 360.
- **`backend:=real` is verified only as far as Nav2's costmaps.** A goal has
  never been driven on hardware, and localisation on a real map is untested —
  no world in the registry corresponds to the room the robot is in, so an
  authored lab world is the next step (`docs/roadmap.md` 3a).
- **The real backend's URDF is the robot's, not this repository's.** Under
  `backend:=real` the robot's own `turtlebot3_description` publishes
  `/robot_description` and `/tf_static`, so the "one URDF above
  `base_footprint`" property holds between `gazebo` and `isaacsim` but not
  across to `real`. Deliberate, 2026-09-16; the alternative was to stop the
  robot publishing it and push this repository's URDF from the workstation.
- **Nav2 comes up on both simulated backends; a goal has not been driven since
  the restructure.** Localisation is verified (`map->odom` at the spawn pose on
  both) and every lifecycle node is active, but the last recorded drive-to-goal
  is from before this work (see `docs/history.md`). Nothing in the change
  touches the planner or controller — only where the map comes from and that
  Nav2 now starts on `wait_for_sim`'s exit — but it is worth a run.
- **Per-backend Nav2 tuning is a non-goal, not unstarted work.** There is one
  `config/nav2_params.yaml`, verbatim `turtlebot3_navigation2`'s
  `param/humble/burger.yaml`, used unconditionally. `bringup.launch.py` used to
  auto-prefer `config/nav2_<backend>.yaml` if one appeared; that mechanism was
  removed 2026-09-16 because it made the wrong thing (tuning away the
  sim-to-real gap it exists to measure) convenient rather than impossible. See
  `docs/architecture.md`.
- **`isaacsim_bringup` is pinned to `IsaacSim-6.0.1` while Isaac Sim is 6.1.0.**
  The 6.1.0 tag reimplements `run_isaacsim.launch.py` as
  `run_isaacsim.launch.xml` — argument for argument the same, but a different
  name and source type — and `turtlebot3_isaacsim` includes it by the old name.
  It is not a runtime mismatch: the package passes `install_path=/isaac-sim`,
  which overrides the launcher's own `version` default, so the 6.0.1 launcher
  starts the 6.1.0 install. Moving the pin forward is a two-line change in
  `turtlebot3_isaacsim` (`AnyLaunchDescriptionSource` and the new filename) and
  belongs in that repository. Reasoning is in `tb3_sim2real.repos`.
- **The world registry has two worlds.** `turtlebot3_world` (upstream's arena,
  generated from a manifest derived mechanically from `turtlebot3_gazebo`'s
  `model.sdf`) and `empty_stage`. `turtlebot3_isaacsim` carries three more of
  its own — the warehouse, a simple room and a kitchen — as stock Isaac Sim
  environments; adopting them here is an `artifacts: {mode: adopted}` manifest
  each, and needs a Gazebo representation before `world:=` would mean the same
  thing on both.

## What changed on 2026-09-14

The Isaac Sim side stopped being written here. `turtlebot3_isaacsim` is imported
from its own repository rather than copied into this tree, and with it went
`isaac/` — the hand-rolled simulator launcher, the URDF importer, the asset
verifier and the scenes — and `docker/isaacsim-ros2/`, which was a copy of that
package's base image definition, one Isaac Sim version behind.

What replaced the attach-only Isaac backend is the reason it is worth the churn:
`world:=` now means the same environment on all three backends, because the
launch file starts the simulator instead of waiting for someone else to have
started it with the right stage.

`docs/history.md` has the detail, dated.
