# Status

What is verified, and what is not. Start here before claiming something works.

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
  representations. The USD generator's own checks pass: 15 collision prims for
  15 manifest bodies, all 6 mesh colliders at approximation `none`, 0 rigid
  bodies, and bounds within 2 cm of the value computed analytically from the
  meshes and placements. `verify: OK — Isaac stage matches the Gazebo world`.
- **`scripts/check_worlds.py`** — 2/2 worlds consistent. Negative-tested against
  deliberately broken copies: a manifest edited without regenerating, a
  generated `.world` edited by hand with its header intact, and a map mirrored
  about its x axis. All three fail, each naming what is wrong.
- **`scripts/clone_world.py`** — round-tripped: the stock `turtlebot3_world`
  map cloned into a world directory scores 100% of the map modelled and 0% of
  the model absent from the map at the burger's 0.182 m beam height.
- **`colcon build`** — 3 packages: `isaacsim_bringup`, `turtlebot3_isaacsim`,
  `tb3_bringup`.

### One difference between the backends, not a fault

The `odom` frame does not start in the same place. Gazebo's diff-drive plugin
puts `odom` at the world origin, so tf reads the spawn pose immediately; Isaac
Sim's `IsaacComputeOdometry` puts `odom` at the robot, so tf reads zero at
spawn. Both are valid odometry; AMCL resolves the difference into `map->odom`.
It matters only if you compare raw `/odom` between backends without saying which
frame you mean.

## Known open

- **`/scan` does not match between backends.** Isaac publishes 3600 rays where
  Gazebo publishes 360, and reports no-return as `-1.0` where Gazebo uses `inf`
  — and `-1.0` is not a valid `LaserScan` range, so the two backends disagree on
  the one field Nav2's obstacle layer filters on. The fix is an LDS-shaped RTX
  scan pattern, and it belongs in `turtlebot3_isaacsim`, which owns the lidar
  profile. Overriding `scanRateBaseHz`/`patternFiringRateHz` on a stock survey
  config is *not* a workaround: it advertises the right geometry, publishes
  erratically, and trips `Multi-tick is enabled but motion BVH is not active`.
- **`backend:=real` is unverified.** No robot has been on the bench. The launch
  file mirrors `turtlebot3_bringup/launch/robot.launch.py` minus the state
  publisher, which is shared.
- **Nav2 has not been re-run since the restructure.** It came up and drove to a
  goal on both simulated backends before it (see `docs/history.md`), and the
  only thing that changed for it is where the map comes from and that it now
  starts on `wait_for_sim`'s exit rather than beside it. Worth re-running before
  it is quoted.
- **Per-backend Nav2 tuning is unstarted.** There is one
  `config/nav2_params.yaml`, verbatim `turtlebot3_navigation2`'s
  `param/humble/burger.yaml`. `bringup.launch.py` prefers
  `config/nav2_<backend>.yaml` the day one exists. The delta between them is the
  measurement; there is no delta yet, so there is one file.
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
