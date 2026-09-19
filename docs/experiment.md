# The experiment

**The ordered work list that produces these numbers is
[`docs/experiment-plan.md`](experiment-plan.md)** — what can be done without
the robot, what needs it, and the findings deliberately left alone. This file
is where its output lands.

This is a structure to drop results into, not a results document. It exists so
a night's or a session's numbers land in tables that already have the right
shape, rather than in a pile that has to be re-organized before it means
anything. Fill the empty tables; do not restate what `measurements/*.md`
already says in prose — link to it instead.

## The claim

Not "simulator A differs from simulator B." The claim is **where in the stack
the sim-to-real gap actually lives**, decomposed into four layers that are
handled, and measured, independently:

| layer | how it is handled | expected finding |
|---|---|---|
| geometry | identical **by construction** — one manifest (`worlds/<name>/world.yaml`) generates both the Gazebo `.world` and the Isaac `.usd`, meshes shared byte-for-byte; `scripts/check_worlds.py` proves it | zero by design — the methodological contribution, not a measurement |
| perception | same LDS-01 profile on both simulated backends; measured against geometry computed from the manifest, not against each other | small — predicted not to matter |
| actuation | `ros2 run tb3_bringup drive_test`, open loop, one instrument for all three backends | diverges — see `measurements/isaac_angular_deficit.md` |
| task | `nav_test`, `NavigateToPose` goals, same map, same goals, all three backends | the actuation gap propagates here, closed-loop |

Layers 1 and 2 exist to license the claim about layers 3 and 4: if geometry and
perception are provably identical, any difference in actuation or task
performance cannot be attributed to the environment being subtly different
between backends.

**The one rule that must not be broken:** each backend is verified
independently against the *analytic* commanded value (kinematics for
actuation, manifest geometry for perception) — never tuned to match another
backend. Two backends agreeing after independent verification is a result.
Fitting one to the other destroys the comparison.

## Metric definitions

**Actuation (`drive_test`)**

- *wheel tracking error* — `|omega_joint − omega_commanded| / omega_commanded`,
  from `/joint_states` against the analytic
  `omega = (vx ± wz*L/2) / r` (`L = 0.160 m`, `r = 0.033 m` for the burger).
  The ground-truth target, not agreement between backends.
- *straight-line / rotation error* — commanded distance or angle vs. what
  `/odom` integrated over the same open-loop command.
- *decomposition* (command → wheel → body → odom) — see
  `docs/worknotes/2026-09-16-overnight-brief.md` §2 A5. Isolates actuator
  fidelity, wheel slip / contact model, and odometry computation as three
  separate numbers instead of one blended one.

**Perception (`drive_test` parked, or a dedicated static scan capture)**

- *range error vs. analytic* — measured `/scan` range per beam vs. the
  distance to the nearest wall computed from `worlds/<name>/world.yaml`.
- *valid-return fraction*, *angular alignment*, *noise* (stddev of repeated
  returns at a fixed pose).

**Task (`nav_test`)**

- success / failure / timeout
- time to goal (simulator clock, not wall clock)
- path length vs. straight-line distance
- final position error, final heading error
- number of recovery behaviours triggered
- minimum distance to an obstacle during the run

## Run matrix

Backends: `gazebo`, `isaacsim` (`real` when hardware is available — see
`docs/status.md` for what is verified on it). Worlds with a saved map today:
`turtlebot3_world`, `small_office` (`scripts/save_map.py <world>` makes one for
any other).

### Actuation — experiment 1, open space open loop

`measurements/experiment1/`, and its README for how to read a file. Every number
below is `net` — the whole run reduced to the three scalars a tape measure
gives, resolved into the pose the run started from. On the simulators `net`
comes from `/ground_truth/odom`; on hardware it will come from a person, in
`hand`, and the pair is the wheel → body layer.

**gazebo, 2026-09-19**, `empty_stage`, `physics.floor.mu = 1.0`, RTF ≈ 1.0.
Mean ± standard deviation across repeats:

| sequence | n | forward m | lateral m | yaw ° | nominal yaw ° |
|---|---|---|---|---|---|
| `line` | 3 | **+3.0039** ± 0.0000 | +0.0017 ± 0.0029 | +0.04 ± 0.08 | 0 |
| `spin_cw` | 3 | −0.0016 ± 0.0001 | −0.0001 ± 0.0004 | **−721.87** ± 0.06 | −720 |
| `spin_ccw` | 3 | −0.0024 ± 0.0001 | +0.0009 ± 0.0006 | **+721.75** ± 0.07 | +720 |
| `square_cw` | 5 | **+0.0251** ± 0.0037 | **+0.0179** ± 0.0030 | −361.29 ± 0.16 | −360 |
| `square_ccw` | 5 | **+0.0948** ± 0.0087 | **−0.0935** ± 0.0107 | +365.23 ± 0.48 | +360 |

`sweep` is run 3× as well and is not summarised here: it ends at a pose no
single measurement describes, which is the whole reason the other four exist.
Its value is the per-phase command → wheel comparison inside each file.

**isaacsim: not yet run.** It deadlocks FastDDS participant creation — `Node()`
construction blocks forever while Isaac is playing, and disabling the
shared-memory transport releases it. The UDP-only workaround was deliberately
not applied, because recording half of one matrix on a different transport from
the other half, undeclared, is the kind of confound this document exists to
prevent. `docs/troubleshooting.md` has the failure class; the decision is
whether to run both backends on UDP and say so.

**real: not yet run.** Needs the marked floor, the taped square and the
reference point (`docs/experiment-plan.md` B4).

#### What the Gazebo numbers say on their own

- **`line` overshoots by +3.9 mm on 3 m, with zero scatter** — a +0.13% scale
  error, identical to five decimal places across three repeats.
- **Pure rotation is symmetric**: `spin_cw` and `spin_ccw` overshoot by 1.87°
  and 1.75° over two full turns, agreeing with each other to **0.05°**.
- **The square is not symmetric.** `square_ccw` overshoots heading by 5.2°
  against `square_cw`'s 1.3°, and returns about **4× further** from the start
  mark. Since pure rotation is even-handed, the asymmetry appears only when
  translation and rotation are combined, and it is stable across r1–r5 rather
  than drifting. **Unexplained.** UMBmark exists to separate direction-
  asymmetric error, so this has to be understood before the hardware squares —
  otherwise the real robot's asymmetry cannot be told from the apparatus's.

### Perception — static scan vs. analytic

| backend | world | date | range error | valid-return % | angular alignment | noise (stddev) | file |
|---|---|---|---|---|---|---|---|
| | | | | | | | |

### Task — `nav_test`

| backend | world | date | before/after gain fix | success rate | time to goal | path length ratio | final pos. error | final heading error | recoveries | file |
|---|---|---|---|---|---|---|---|---|---|---|
| | | | | | | | | | | |

## Repeatability

Gazebo is bit-identical run to run; Isaac Sim is not
(`docs/status.md`: spread 0.0063 m, 0.042 rad on one sequence). Before treating
any `nav_test` delta as real rather than noise, the number of repeats needed is
`n` such that the standard deviation of the metric in question, divided by
`sqrt(n)`, is well below the effect size being claimed — see
`docs/worknotes/2026-09-16-overnight-brief.md` §2 A6 for the derivation once
the repeat-run data exists.

| metric | stddev (n repeats) | source |
|---|---|---|
| gazebo `line` forward | **0.0000 m** (3) | `measurements/experiment1/` |
| gazebo `spin` yaw | 0.06° (3 each way) | " |
| gazebo `square` return offset | 3–11 mm (5 each way) | " |
| isaacsim | not yet measured | blocked, above |
| real | not yet measured | needs the room |

**Gazebo's scatter is far below the systematic offsets being measured**, so
3 repeats is already generous for it — a fourth would change no conclusion. That
is expected and it sizes nothing: Gazebo is the deterministic backend. The
number that decides how many repeats experiment 2 needs is Isaac's and the real
robot's, and neither exists yet.

## Where the numbers live

Raw output (`drive_test --ros-args -p out:=...`, rosbags) goes in
`measurements/`, one file per run, named for what varies
(`<sequence>_<backend>[_<label>].json`). A dated `.md` in the same directory
interprets a specific finding (`isaac_angular_deficit.md`, `nav2.md`,
`collision.md` are the existing examples). This file is the index those feed
into, not a replacement for them.
