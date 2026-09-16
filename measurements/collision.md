# Driving into something, and what odometry does about it

Measured 2026-09-16. `world:=small_office`, `sequence:=collide`, open loop, one
run per backend. Spawn (-1.5, -1.5) facing +x; the dining table occupies
x 0.7..2.5, y -1.65..-0.95 and is an **exact triangle-mesh collider**, not a
convex hull. The `approach` phase commands 0.15 m/s for 25 s — 3.75 m if
nothing were in the way.

| | gazebo | isaacsim |
|---|---|---|
| `approach` displacement | 2.177 m | 2.317 m |
| `approach` odom path length | 2.744 m | 2.319 m |
| commanded | 3.765 m | 3.750 m |
| yaw change while driving straight | **+1.577 rad** | **+0.0001 rad** |
| motion during `rest` (commanded 0) | 0.085 m, +3.101 rad | **0.000 m, 0.000 rad** |
| `reverse` recovered (commanded 0.300 m) | 0.088 m (29%) | 0.272 m (91%) |
| `peak_vx` during approach | 0.187 m/s | 0.149 m/s |

## The two behave completely differently on contact

**Gazebo deflects and pivots.** The robot met the table's near leg off-centre,
and because the command kept coming for another ten seconds it *pivoted around
the leg*, ending 90 degrees off heading (final yaw -1.584). It was still moving
when the command went to zero: 0.085 m and three radians of yaw during a phase
that commands nothing at all. It then could not reverse out cleanly — 29% of
the commanded back-off — because it was still wedged. `peak_vx` reached
0.187 m/s against a commanded 0.150, which is the contact impulse.

**Isaac Sim stops dead and square.** Zero yaw change through the whole
approach, an exact standstill during `rest`, and a clean 91% reverse. No
impulse: `peak_vx` never exceeds the commanded 0.150.

Neither is obviously right. A real burger driven into a table leg does deflect,
so Gazebo's behaviour is the more lifelike of the two; Isaac's is the more
repeatable. What matters for this repository is that **a collision is the one
place where the two backends are not interchangeable**, so a controller tuned
against contact behaviour in one will not transfer to the other.

## The good news: neither invents odometry

The question that motivated this run was whether wheels spinning against an
obstacle integrate distance the robot never travelled — an unbounded error that
Nav2 cannot see. They do not:

- Isaac's odom path (2.3189 m) equals its displacement (2.3172 m) to 1.7 mm.
  When it stopped, odometry stopped.
- Gazebo's path (2.744 m) exceeds its displacement (2.177 m), but that is the
  pivot, not phantom distance: the robot really did travel that arc.

Both integrated *less* than the 3.75 m commanded, so in both simulators the
blocked wheels are resisted rather than free-spinning.

## A contract difference, found on the way

The two backends do not put `/odom`'s origin in the same place:

| backend | first `/odom` sample at the spawn |
|---|---|
| gazebo | **(-1.4999, -1.5000)** — the world pose |
| isaacsim | (-0.0000, +0.0000) — zero, relative to the start |

The real `turtlebot3_node` starts its odometry at zero wherever the robot is
switched on, so **Isaac Sim matches the real robot here and Gazebo does not.**

It breaks nothing: `odom` is only required to be a continuous frame, and AMCL's
`map -> odom` absorbs the offset, so Nav2 works either way. But it means raw
`/odom` positions are **not comparable between backends**, and anything that
reads `/odom` as a position rather than as a delta will disagree.

It does not affect the measurements in `drive_test`: every figure there is a
difference within a phase, and the frame origin cancels.

The cause is upstream's, not this repository's. `turtlebot3_gazebo`'s
`models/turtlebot3_burger/model.sdf` sets no `<odometry_source>`, so
`gazebo_ros_diff_drive` uses its default and seeds the pose from the model's
world pose. Overriding it would mean carrying our own copy of upstream's model,
which is the duplication this repository exists to avoid — so it is recorded
here rather than patched.
