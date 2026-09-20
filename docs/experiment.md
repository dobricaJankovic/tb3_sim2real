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

**isaacsim, 2026-09-20**, `empty_stage`, `physics_hz = 240.0`,
`turtlebot3_isaacsim` @ `4231fbf`, shared-memory DDS — the **same transport as
the Gazebo half**, so the confound that blocked this run on 2026-09-19 does not
exist. 22 runs, all `net.source = /ground_truth/odom`:

| sequence | n | forward m | lateral m | yaw ° | nominal yaw ° |
|---|---|---|---|---|---|
| `line` | 3 | **+2.9952** ± 0.0029 | −0.0264 ± 0.0522 | −0.95 ± 1.65 | 0 |
| `spin_cw` | 3 | −0.0004 ± 0.0022 | −0.0024 ± 0.0019 | **−645.70** ± 1.28 | −720 |
| `spin_ccw` | 3 | +0.0031 ± 0.0024 | −0.0004 ± 0.0043 | **+645.49** ± 0.25 | +720 |
| `square_cw` | 5 | **−0.5628** ± 0.0204 | **−0.9937** ± 0.0549 | −318.17 ± 1.29 | −360 |
| `square_ccw` | 5 | **−0.5551** ± 0.0235 | **+1.1292** ± 0.0395 | +313.88 ± 0.61 | +360 |

The 2026-09-19 diagnosis — "it deadlocks FastDDS participant creation and
disabling the shared-memory transport releases it" — **was wrong about the
cause**, and the wrong cause was about to buy an undeclared transport change.
It is stale `/dev/shm` segments holding named mutexes, the failure class found
on 2026-09-20 and written up in `scripts/dds_clean.sh`. Clearing them took
`Node()` construction from "blocks forever" to instant with Isaac playing and
SHM on. Free space was never the mechanism, which is why "`/dev/shm` is only 2%
used" looked like it exonerated the segments and did not.

**The `sweep` runs are commanded differently from Gazebo's in one phase.** Top
linear rate is 0.20 m/s here against the Gazebo files' 0.22 — the instrument
was edited on 2026-09-19 for the real robot's stall at its own rated maximum.
`line`, `spin_*` and `square_*` never use that phase and are directly
comparable.

#### What the Isaac numbers say, next to Gazebo's

- **Rotation comes up ~10% short, symmetrically.** −645.70° and +645.49°
  against ±720°, the two directions agreeing to 0.2°. Gazebo *overshoots* by
  0.26%. This is the largest sim-to-sim difference in experiment 1.
- **Translation is close**: +2.9952 m on 3.0, a −0.16% scale error against
  Gazebo's +0.13% — but the lateral scatter is ±52 mm and the yaw scatter
  ±1.65°, where Gazebo held ±3 mm and ±0.08°.
- **The squares do not close.** Isaac ends **1.14 m** (CW) and **1.26 m** (CCW)
  from the start mark; Gazebo ended 31 mm and 133 mm off. That is the rotation
  deficit compounding over four corners, not a separate effect.
- **Isaac has no counterpart to Gazebo's CW/CCW asymmetry.** Its two directions
  mirror each other (−0.563/−0.994 against −0.555/+1.129); Gazebo's differ 4×.
  Whatever produces Gazebo's asymmetry is Gazebo's, which is worth knowing
  before the hardware squares.

#### Where the deficit lives: Isaac loses it at the wheel, Gazebo to slip

`sweep` separates the two layers, because it reports commanded wheel rate
against `/joint_states` alongside ground-truth body motion. The two simulators
fail in opposite places, and at opposite ends of the rate range:

| commanded | Gazebo wheels | Gazebo body | Isaac wheels | Isaac body | real body |
|---|---|---|---|---|---|
| `wz = 0.2` | 100.0% | 96.9% | 69–83% | **77–82%** | 96.1% |
| `wz = 0.5` | 100.0% | 94.7% | 87–100% | 89–90% | 96.0% |
| `wz = 1.0` | 100.0% | 89.4% | 97–98% | 94.4% | 95.7% |
| `wz = 1.5` | 100.0% | **82.9%** | 97–99% | 95.7% | 95.5% |
| `v = 0.20` | 100.0% | 97.4%* | 99.8% | 98.0% | 95.4% |

\* Gazebo's linear row is its `lin_0.22` phase; see the rate note above.

- **Gazebo's wheels reach the commanded rate exactly, at every rate**, and it
  loses everything between wheel and ground — −3% at `wz = 0.2` growing to
  **−17% at `wz = 1.5`**. Friction-limited slip, which is what `mu = 1.0`
  predicts and what the 2026-09-19 friction work put there deliberately.
- **Isaac's wheels do not reach the commanded rate**, worst when turning
  slowly, and its body then follows them to within 1–3%. The deficit is in the
  contact solve at the wheel — the cylindrical-collider chatter CLAUDE.md
  documents, whose *mean* falls below the command — not slip.
- **It is not a velocity ceiling.** The wheel joints in
  `payloads/Physics/physics.usda` are velocity drives (`damping = 1745.3292`,
  `stiffness = 0`, `type = "force"`) with **no `maxJointVelocity`, no
  `maxForce` and no joint limits authored**. Tracking gets *better* with rate,
  the reverse of saturation, and the fastest wheel rate in the whole experiment
  — 6.06 rad/s during `lin_0.20` — tracks at 99.9%, while the `spin`/`square`
  turns ask for only 1.21 rad/s and come up ~10% short.
- **Neither simulator has the real robot's shape.** Hardware tracks **95–97%
  flat across every rate**; Isaac is 78–82% where the robot is 96%, Gazebo is
  83% where the robot is 96%. They fail at opposite ends and the robot fails at
  neither. **Caveat that limits how hard this can be pushed:** the real column
  is `/odom`, integrated from the encoders, so it cannot show slip at all — it
  is the command → wheel layer only, against the simulators' command → body.
  For the layer hardware *can* see, Isaac at `wz = 0.5` (~90%) is further from
  the robot (~96%) than Gazebo is (100% at the wheel). Settling which
  simulator is right needs `spin_cw`/`spin_ccw` on hardware with a tape — the
  runs deliberately skipped on 2026-09-19.

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
