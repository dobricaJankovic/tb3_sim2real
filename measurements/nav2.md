# Nav2, both backends, same map and same goal

Measured 2026-09-16. `world:=turtlebot3_world`, `nav:=true`, headless. AMCL
seeded at the manifest spawn (-2.0, -0.5), then one `NavigateToPose` goal at
(2.0, 0.5). Three Gazebo runs, two Isaac Sim runs, each a fresh container.

| | gazebo a | gazebo b | gazebo c | isaacsim a | isaacsim b |
|---|---|---|---|---|---|
| result | SUCCEEDED | SUCCEEDED | SUCCEEDED | SUCCEEDED | SUCCEEDED |
| time to goal | 22.46 s | 22.16 s | 22.26 s | 22.66 s | 23.16 s |
| odom path travelled | 4.641 m | 4.610 m | 4.624 m | 4.646 m | 4.723 m |
| final yaw error | 0.046 | 0.027 | 0.072 | 0.035 | 0.065 rad |
| AMCL-reported goal error | 0.288 | 0.432 | 0.222 | 0.502 | 0.437 m |

**Nav2 reaches the goal on both backends, in the same time, over the same
path.** Time to goal spans 22.16-23.16 s across all five runs — a 4.5% spread
that straddles the two backends rather than separating them. Path length spans
4.610-4.723 m, 2.4%. No recovery behaviour was triggered in any run.

## The interesting part: closing the loop hides the rotation defect

`measurements/isaac_angular_deficit.md` records that Isaac Sim delivers only 73%
of a commanded 0.5 rad/s and 48% of 0.2 rad/s, open loop. The straight-ish goal
above barely exercises rotation, so it cannot show whether that costs anything.
A goal that is **pure rotation** can: same position, yaw + pi.

| | gazebo | isaacsim |
|---|---|---|
| 180 degree turn in place | SUCCEEDED, **4.46 s** | SUCCEEDED, **4.56 s** |
| final yaw error | 0.261 rad | **0.110 rad** |

Two per cent apart, and Isaac's residual error is the *smaller* of the two.

That is the honest conclusion and it is not the one the open-loop numbers
suggest: **Nav2 is essentially immune to this defect.** The controller closes
the loop on yaw error, so when the plant delivers 70% of the commanded rate the
controller simply keeps commanding until the error is gone. The deficit costs
time proportional to the shortfall only when the controller is already
saturated, and at `max_vel_theta: 1.0` against a burger's 2.84 rad/s limit, it
is not.

Where the defect *does* matter is unchanged: open-loop control, dead reckoning,
anything that trusts `/cmd_vel` to be executed, and above all **tuning a
controller against Isaac Sim and expecting the gains to transfer** to a real
burger that actually delivers what it is asked.

## Caveat on the goal-error column

`xy_goal_tolerance` is 0.25 m, and several runs report a larger final error
while still returning SUCCEEDED. That is not Nav2 overshooting its tolerance:
the figure here is `/amcl_pose` sampled about two seconds *after* the action
returned, which is neither the tf the controller checked nor the instant it
checked it. It is measured identically on both backends, so it is usable as a
comparison and not as an absolute.
