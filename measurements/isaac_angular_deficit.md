# Isaac Sim under-rotates: measured, and why

> **The measurements here are correct and reproduce. The cause named below is
> wrong, and was superseded on 2026-09-17 by
> [`docs/worknotes/2026-09-17-lane-a-physics.md`](../docs/worknotes/2026-09-17-lane-a-physics.md).**
> The drive damping was swept 10,000x with no measurable effect on the error,
> which kills the finite-gain reading in "The cause". The real mechanism is the
> contact solve — see the correction at the end. This file is kept because its
> data is good and because the inference it drew from that data was a
> reasonable one to draw; what it was missing was the standard deviation.

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

## The cause ~~as diagnosed on 2026-09-15~~ — SUPERSEDED

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

## The correction, 2026-09-17

Raising the drive gain was indeed the obvious first thing to try. It was tried,
across four decades, and it does nothing.

| runtime damping `D` | differential error at `wz = 0.5` |
|---|---|
| 1e3 (0.01x) | 0.288 rad/s |
| 1e5 (as built) | 0.328 rad/s |
| 1e7 (NVIDIA's documented value) | 0.332 rad/s |

For the finite-gain reading to hold, `err * D` would have to be constant. It
spans 76 to 3,606,069 N·m instead, because `err` simply does not depend on `D`.
A Dynamixel XL430 stalls at about 1.4 N·m, so the implied torque was never
physical — the arithmetic above did not close and that should have been the
clue.

**What the means were hiding is an oscillation.** Commanded a steady −1.2121
rad/s, the left wheel ranges over **−2.91 to +1.04 rad/s** and reverses
direction; the 73% recorded here is its average, not its value. Gazebo's
standard deviation on the same phase is 0.0000. This file measured only means,
which is why a chattering signal read as a steady shortfall — and why a
chatter-induced bias, roughly set by the oscillation amplitude and so nearly
independent of the commanded rate, looked exactly like `tau / D` in a table.

Lift the robot off the ground and every commanded rate is reproduced
**exactly**, so the deficit is created entirely by the contact solve. The
source is the wheels' cylindrical collider, which neither PhysX nor MuJoCo
rolls exactly; a sphere of the same radius cuts the chatter 40x at the original
timestep.

The asymmetry and the spurious `peak_vx` noted above are consequences of the
same oscillation rather than of a gain imbalance.

### Where the fix belongs

Still `turtlebot3_isaacsim`, which owns the asset and the physics. The shipped
change is the PhysX sub-step rate (60 → 480 Hz), which fixes translation to
within 1% and takes rotation to 88% of command. The wheel collider is the
remaining decision and is open.
