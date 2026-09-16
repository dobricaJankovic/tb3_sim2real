# The experiment

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

### Actuation — `drive_test`

| backend | world | date | dt / solver iters | vx sweep | wz sweep | before/after gain fix | file |
|---|---|---|---|---|---|---|---|
| | | | | | | | |

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

| metric | Isaac stddev (n repeats) | derived n for `nav_test` | source |
|---|---|---|---|
| | | | |

## Where the numbers live

Raw output (`drive_test --ros-args -p out:=...`, rosbags) goes in
`measurements/`, one file per run, named for what varies
(`<sequence>_<backend>[_<label>].json`). A dated `.md` in the same directory
interprets a specific finding (`isaac_angular_deficit.md`, `nav2.md`,
`collision.md` are the existing examples). This file is the index those feed
into, not a replacement for them.
