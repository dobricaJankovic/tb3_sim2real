# Isaac Sim under-rotates: measured, and why

Measured 2026-09-15, `world:=empty_stage`, burger, open loop, steady state
(the average of the second half of each phase, after any ramp).

## What it looks like from ROS

Commanding `angular.z = 0.5` for 5 s in `drive_test`:

| | gazebo | isaacsim |
|---|---|---|
| yaw achieved (commanded 2.500 rad) | 2.478 rad, −0.9% | **1.742 rad, −30.3%** |
| `/odom` twist `wz` (commanded 0.5) | 0.497 | **0.367** |

Linear motion is nearly unaffected: 0.736 m against 0.716 m over the same
5 s, 2.9% apart. A wheel-radius error would skew both equally, so it is not
that — and `turtlebot3_isaacsim` sets `separation: 0.160, radius: 0.033`, which
is the real burger.

## Where it goes

The wheels are not slipping. They are not turning fast enough:

| cmd `wz` | wheel target | achieved L / R | tracking | odom `wz` |
|---|---|---|---|---|
| 0.20 | 0.4848 | −0.2325 / +0.2307 | **47.8%** | 54.4% |
| 0.50 | 1.2121 | −0.8789 / +0.8922 | 73.1% | 77.5% |
| 1.00 | 2.4242 | −1.9458 / +2.0943 | 83.3% | 87.9% |
| 1.50 | 3.6364 | −3.2961 / +3.1259 | **88.3%** | 87.9% |

Slipping wheels would read at or above target, free-spinning. These read below
it, so something is resisting and the drive is not overcoming it.

## The cause

The tracking *percentage* rises with speed while the *absolute* error stays
roughly constant — 0.25, 0.33, 0.40, 0.43 rad/s. That is the signature of a
fixed opposing torque against a finite-gain velocity drive:

    stiffness = 0, so  D * (w_target - w_actual) = tau_friction
    =>  w_target - w_actual = tau / D        <- constant, independent of target

A units or kinematics error would hold the *ratio* fixed instead, and a joint
velocity cap would make high speeds worse, not better. Both are excluded.

So the drive gain is too low for the wheel friction. In
`turtlebot3_isaacsim/scripts/import_turtlebot3.py`:

```python
# TODO unverified gain. Without it the importer warns that the actuator
# is created with no gain parameters.
override_joint_damping=1.0e5,
override_joint_stiffness=0.0,
```

which reaches the asset as `drive:angular:physics:damping = 1745.3292`
(= 1.0e5 * pi/180, so USD authored it per degree). No `maxForce` is authored on
either wheel joint, and the URDF's wheel joints are `type="continuous"` with no
`<limit>` at all, so nothing else is capping the torque. The gain is simply
not large enough, and the comment already says it was never verified.

## Why it matters more than 30% sounds

The error is worst where it is smallest in absolute terms. At `wz = 0.2` the
robot delivers **48%** of the commanded rate. That is Nav2's regime — final
alignment to a goal pose, in-place turns, recovery spins — so the deficit is
largest exactly where the planner is most sensitive to it.

It also rotates asymmetrically (|L| != |R|, e.g. 0.879 vs 0.892), which is a
net translation during what should be a pure pivot, and it matches the spurious
linear velocity seen during the `rotate` phase: `peak_vx` 0.021 m/s against
Gazebo's 0.0017, about 13x.

## Where the fix belongs

`turtlebot3_isaacsim`, which owns the robot asset, the drive gains and the
physics materials — not this repository. Raising the drive gain is the obvious
first thing to try; the wheel and caster friction (both 1.0) is the other half
of the ratio.
