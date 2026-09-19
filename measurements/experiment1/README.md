# Experiment 1 — open space, open loop

**These are the official runs.** Anything outside this directory is a probe, a
sweep or a diagnostic; anything inside it is a result that a table in
`docs/experiment.md` is built from.

`docs/experiment-plan.md` B4. One `/cmd_vel` sequence, no lidar, no Nav2, no
map — Nav2 would correct exactly the errors being measured.

## Naming

```
<date>_exp1_<backend>_<sequence>_r<n>.json
```

`backend` is `gazebo`, `isaacsim` or `real`. `r<n>` is the repeat index, from 1.
The `.samples.json` beside each file is the unaveraged trace and is gitignored —
regenerate it by re-running. **Never rename a file to fix a mistake**: re-run it,
or leave it and say so in `notes.md`.

## The matrix

| sequence | repeats | what it isolates |
|---|---|---|
| `sweep` | 3 | command → wheel. No hand measurement on any backend. |
| `line` | 3 | wheel-diameter average — a pure scale error |
| `spin_cw`, `spin_ccw` | 3 each | wheelbase / wheel-diameter ratio |
| `square_cw`, `square_ccw` | 5 each | UMBmark: both systematic error types, separated |

22 runs per backend. Both spin and both square directions are required, not
optional: a systematic error that adds one way subtracts the other, and one
direction alone gives a number that cannot be attributed.

## How to read a file

`net` is the whole run reduced to the three numbers a tape measure gives,
resolved into the **start frame**:

| field | meaning | run-sheet mark |
|---|---|---|
| `forward_m` | along the starting heading | `line` ①, `square` ① |
| `lateral_m` | to the side of it | `line` ②, `square` ② |
| `yaw_rad` | net heading change, **accumulated**, not wrapped | `spin` ① (against 4π), `square` ③ |

`net.source` says where it came from — `/ground_truth/odom` on the simulators,
`/odom (NOT truth)` on hardware, where `/odom` is the thing being measured
rather than the measurement. On a real run the person's figures are in `hand`,
and comparing `hand` against `net` **is** the wheel → body layer.

Start frame rather than world frame on purpose: the two simulators' ground-truth
origins differ (Gazebo world-absolute, Isaac spawn-relative), and on hardware
there is no world frame at all — there is a mark on the floor and a robot that
was placed on it.

## Recording a hand measurement

```bash
ros2 run tb3_bringup drive_test --ros-args \
    -p label:=real -p sequence:=square_cw \
    -p truth_xy_m:="[1.47, 0.03]" -p truth_yaw_deg:=2.0 -p truth_method:=tape \
    -p out:=measurements/experiment1/<date>_exp1_real_square_cw_r1.json
```

## Conditions that must be recorded with the runs

In `notes.md` in this directory, on the day:

- **the floor** — vinyl, tile, sealed concrete, carpet. Unrecoverable afterwards
  and it is what makes the friction number mean anything to anyone else.
- **`physics.floor.mu`** the runs were taken at. It became manifest data on
  2026-09-19 and every Gazebo measurement before that date was taken at
  upstream's disowned `100000` sentinel.
- **Isaac's physics rate.** Never compare across rates without checking.
- **battery voltage at the start and end** of a hardware session, and re-run one
  condition at the end. The OpenCR tracks a commanded wheel velocity less well
  as the pack drains; if the two differ, the session is not internally
  comparable.
- **the reference point** on the robot that the tape measures to, and that the
  manifest's `spawn:` refers to. It has to mean the same thing in the room and
  in both simulators.
