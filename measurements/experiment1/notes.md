# Experiment 1 — run conditions

## Session

- **Date:** 2026-09-19
- **Backends attempted:** `gazebo`, `isaacsim`
- **Backends completed:** `gazebo` only **in this session**. `isaacsim`
  produced no runs here and was completed on 2026-09-20 — its own section is at
  the end of this file, and it also corrects the cause diagnosed below.
- **World:** `empty_stage`, `headless:=true`, `rviz:=false`, no Nav2, no SLAM.
- **Runner:** `scripts/run_experiment1.sh <backend> 2026-09-19`, executed inside
  the `tb3_ros` container. The simulator is launched once and all repeats run
  against it without a respawn.
- **Matrix and repeats:** `sweep` ×3, `line` ×3, `spin_cw` ×3, `spin_ccw` ×3,
  `square_cw` ×5, `square_ccw` ×5 — 22 runs per backend.
- **`net.source`:** `/ground_truth/odom` on all 22 Gazebo files. No file fell
  back to `/odom`.
- **Floor (physical):** not applicable — these are simulated runs. No hardware
  runs were taken in this session, so there is no room floor, no tape
  measurement and no battery voltage to record.

## Friction

`worlds/empty_stage/world.yaml` declares **no `physics:` block**, so
`surface()` in `tb3_bringup/tb3_bringup/worlds.py` falls back to the module
default:

- `physics.floor.mu` = **1.0** (`FLOOR_MU`)
- `physics.floor.mu2` = **1.0** (follows `mu` when not given)

These are the **first runs taken after friction became manifest data on
2026-09-19**. Every Gazebo experiment-1 measurement before this date was taken
at upstream's disowned `100000` sentinel and is not comparable with these.

## Isaac Sim

- **Physics rate observed at runtime: 240.0 Hz.** Taken from the argument list
  Kit logged at startup in `/tmp/exp1_isaacsim.log`:
  `... --physics-hz 240.0 --lidar-config turtlebot3_lds --headless`.
  That line is the only place the rate appears at runtime; the runner does not
  echo an effective rate or a sub-step count separately. It matches the known
  config default rather than contradicting it.
- **`turtlebot3_isaacsim` commit:** `4231fbf Publish /odom from the wheels; the
  chassis pose becomes ground truth`
- **Real-time factor: not observed.** No Isaac run completed, and the Isaac log
  reports no RTF line.
- Isaac reached `Stage loaded and simulation is playing. ROS_DOMAIN_ID=30` on
  both attempts, so the failure is not in loading the stage.

## Real-time factor

| backend | RTF | how obtained |
|---|---|---|
| gazebo | ≈ 1.0 | 1188.8 s of commanded sim time across the 22 runs against ~20.7 min wall from launch to the last result file (includes the 35 s settle and per-run node startup). Neither backend's tooling prints an RTF line. |
| isaacsim | not observed | no run completed |

## What went wrong

**1. The starting state was not clean (Gazebo, first attempt — discarded).**
A `drive_test` process from a previous session was still running when the first
Gazebo attempt started, together with an orphaned `robot_state_publisher`
(started 15:24). The runner's stray check only looks for
`ros2 launch|gzserver|isaac-sim`, so it did not catch either. The leftover
`drive_test` resumed as soon as the new Gazebo published `/clock` and wrote a
contaminated `2026-09-19_exp1_gazebo_sweep_r2.json`; in the same attempt
`spawn_entity` timed out after 30 s and no robot spawned. That attempt was
killed, the contaminated `sweep_r2.json` and its `.samples.json` were
**deleted**, all strays were killed by PID and the `ros2` daemon was stopped.

**2. Gazebo, second attempt — clean, complete, kept.**
`spawn_entity` reported `Successfully spawned entity [burger]` and all 22 runs
completed. **These are the files in this directory.** Nothing was re-run inside
this attempt and no file was renamed.

**3. Isaac Sim — blocked, zero runs, twice.**
On both attempts Isaac loaded the stage and began playing, but every `drive_test`
launched afterwards hung with an empty log and ~0 s of accumulated CPU. Isolated
with a standalone probe:

- `rclpy.init()` succeeds.
- **`Node(...)` construction then blocks indefinitely** while Isaac Sim is
  running. drive_test never reaches `adopt_sim_time()` or `wait_for_odom()`, so
  none of its 180 s internal timeouts ever arm and it hangs rather than failing.
- With every Isaac process killed, the same probe creates a node instantly.
- With the FastDDS **shared-memory transport disabled** (UDP-only participant
  profile via `FASTRTPS_DEFAULT_PROFILES_FILE`) the probe creates a node
  instantly *while Isaac is running*, and sees `/clock` (1 publisher) and
  `/ground_truth/odom` (1 publisher).

So the blocker is FastDDS's shared-memory transport, not stale segments: after
Isaac was killed, participant creation worked immediately with all 410
pre-existing `/dev/shm` entries still in place, and `/dev/shm` had free space
(186 MB of 16 GB) and free inodes throughout. Isaac loads its own bundled
middleware — `Attempting to load internal rclpy for ROS Distro: humble` /
`rclpy loaded` — alongside the system Humble FastDDS, and `docker-compose.yml`
bind-mounts the host `/dev` so `/dev/shm` inside the container is the host's.

No Isaac runs were taken. Switching the DDS transport would have meant recording
half the official matrix under different conditions from the Gazebo half, which
is a decision for the experimenter, not the runner.

## Anything that looks off

Reported without interpretation:

- `square_ccw` net yaw is **+365.2°**, about 5° past the commanded 360°, while
  `square_cw` is **−361.3°**, about 1° past. The CCW return offset
  (fwd +0.095 m, lat −0.094 m) is roughly four times the CW one
  (fwd +0.025 m, lat +0.018 m).
- `spin_cw` and `spin_ccw` both overshoot 720° by ~1.8°, symmetrically.
- `line` forward distance is **3.0039 m** on all three repeats, with a standard
  deviation of 0.0000 m — the three runs agree to the printed precision.
- `sweep` lateral has by far the largest scatter of any sequence
  (sd 0.197 m over 3 repeats), and its yaw scatter is 4.7°.

---

## Added after review, 2026-09-19

**The CW/CCW asymmetry in `square` is not in the turning.** `square_ccw`
overshoots its heading by 5.2° against `square_cw`'s 1.3°, and its return
offset is about 4× larger — but `spin_cw` and `spin_ccw` overshoot by 1.80° and
1.75°, **symmetric to 0.05°**. Pure rotation is even-handed; the asymmetry only
appears once translation and rotation are combined. Unexplained, and it is not
noise: r1–r5 of each direction are stable and show no trend, so it is not
accumulated state from running repeats without a respawn either.

Worth knowing what this is before the hardware square runs, because UMBmark's
whole purpose is to separate direction-asymmetric error, and a simulator whose
wheel parameters are symmetric should not be producing much of one.

**The run was not started clean and the runner did not catch it.** A
`drive_test` from an earlier session survived, resumed when this matrix's Gazebo
published `/clock`, and wrote a contaminated `sweep_r2.json` — deleted, and the
matrix re-run from the start. The stray-process guard only looked for
`ros2 launch|gzserver|isaac-sim`; it now also looks for `drive_test` and
`robot_state_publisher`, which are exactly what a killed launch orphans.

**`/dev/shm` was not the Isaac blocker.** Checked afterwards: 178 MB used of
16 GB, 2%, with free inodes. 412 segments are present and some date from
2026-09-17, so they are stale — but there is no pressure and killing Isaac
restored participant creation with all of them still in place.

---

## Real backend, 2026-09-19

**Decided: the real leg of experiment 1 stops here, deliberately incomplete.**
`sweep` ×3 and `line` ×3 are recorded, in `real/`. **`spin_cw`, `spin_ccw`,
`square_cw` and `square_ccw` were not run and are not planned** — the
experimenter chose to move on to experiment 2 (`docs/experiment-plan.md` B6
onward, Nav2 in the lab room) rather than complete the matrix. Not a blocker,
not forgotten: a decision. If the full 22-run matrix is ever wanted for `real`,
this is the gap.

World: `empty_stage`, no Nav2, no SLAM, `backend:=real` — which starts no local
process (`bringup.launch.py` contributes nothing for `real`; the Pi's own
`robot.launch.py` supplies `/odom`, `/joint_states`, `/cmd_vel`). Driven by
hand, one repeat at a time, with a person moving the robot back to a floor mark
between repeats — `run_experiment1.sh` explicitly refuses `real` for this
reason.

**Files landed in the wrong place at first, and were moved.** The very first
runs (`square_cw_r1`, `sweep_r1..r3`) were launched from `/ws` inside the
container without `cd /repo` first, and `/ws` is **not** a bind mount —
`docker-compose.yml` only mounts `./tb3_bringup`, the two `src/` deps, `.` at
`/repo`, and `worlds/`. Those files existed only in the container's writable
layer, invisible on the host, until they were `mv`'d into `/repo/measurements/`
by hand. `give_back()` also chowns to whoever owns the directory a file lands
in *at write time* — since it first wrote into a root-owned `/ws` tree, the
chown was a no-op, and the files arrived in the bind mount still `root:root`;
fixed with an explicit `chown` from inside the container after the move. Every
run from `line_r2` onward wrote directly into `measurements/experiment1/real/`
in the container and came out owned correctly. **All real-backend files now
live in `measurements/experiment1/real/` and `measurements/bags/real/`**, kept
separate from the Gazebo files in the parent directory for the same reason —
this directory has a lot of files in it and the two are easy to conflate.

**`sweep`'s top linear phase was changed from 0.22 m/s to 0.20 m/s, in the
shared instrument.** The first attempt at `sweep` (before this decision) at the
original 0.22 m/s — the burger's rated max, the same value already recorded
for `gazebo`/`isaacsim` — produced `lin_0.22   wheel L 0.201/6.667 R 0.000/6.667`:
the right wheel reported zero velocity while commanded, and the robot spun
~344° instead of driving ~1.1 m straight. The experimenter tested by hand
afterward and confirmed the real robot has a problem at that commanded speed.
`SEQUENCES['sweep']` in `tb3_bringup/tb3_bringup/drive_test.py` was edited to
`lin_0.20` (0.20 m/s) to avoid it — **this is a change to the one instrument
shared by all three backends**, so a `real` `sweep` run from now on is not
commanded identically to the `gazebo`/`isaacsim` `sweep` files already on disk
at the old 0.22 m/s top rate; note this when comparing that one phase across
backends. `sweep_r1..r3` at the new cap all tracked cleanly, no repeat of the
stall.

**Repeat 2 of `sweep` was contaminated by hand-lifting the robot mid-run**
(a large `dyaw` jump across `stop_r4`→`lin_0.10` while wheels read
zero/matching-commanded — first suspected as an `/odom` glitch, then confirmed
by the experimenter as the robot having been picked up). Re-run clean; the
contaminated file was overwritten, not kept.

**A likely units mismatch in `/joint_states` velocity on hardware, found but
not fixed.** Every real run's `wheel_track_l`/`wheel_track_r` reads ≈
`WHEEL_RADIUS` (0.033) regardless of commanded rate — e.g. `line_r1`:
`wheel_wl=0.14893` against `cmd_wl=4.54545` (a commanded 0.15 m/s straight),
ratio 0.0328. Read as **rad/s** (what `cmd_wl`/`cmd_wr` are, and what both
simulators publish) that looks like a 97% tracking failure; read as **m/s**
(what `wheel_wl` numerically equals almost exactly — 0.149 vs. commanded
0.15 m/s) it is a normal ~1% error. The second reading is almost certainly
right, which means **the real robot's `/joint_states` velocity field is
probably in m/s, not rad/s**, and every `wheel_track_*`/`wheel_err_*` number in
every real-backend file recorded today is computed against the wrong assumed
units and should not be read at face value. `net`/`hand` (position/heading, not
wheel rate) are unaffected. Not fixed this session — needs checking against the
real driver's source before `drive_test.py` is changed, per this repo's
search-before-building rule.

**`line_r3`'s first attempt stalled mid-run with garbage readings, most likely
the battery.** Robot stopped after ~1.0 m of a commanded ~3.0 m straight
(no obstruction), and `/joint_states` read `wheel L 535/608` and similar —
non-physical values, including during the following `settle_post` while
commanded to be stopped. Deleted rather than kept or patched. Re-run after
charging, at `/battery_state` = **12.04 V, 85.6%**, and it completed cleanly.
No voltage reading exists from the stalled attempt to compare against, which
is exactly the gap the README's "record battery voltage" condition exists to
close — start-of-session voltage was not read before driving began today, only
after the stall. Read it first, every session, from now on.

### `line` results, ×3, hand-measured

| repeat | forward_m | lateral_m | distance_m |
|---|---|---|---|
| r1 | 2.86 | −0.72 | 2.949 |
| r2 | 2.82 | −0.68 | 2.901 |
| r3 | 2.91 | −0.38 | 2.935 |

All three drift to the **same side** (left of the commanded heading) over a
commanded 3.0 m straight, but the size of the drift is not consistent
(−0.72, −0.68, −0.38) — a systematic direction with an inconsistent
magnitude, reported without interpretation.

**Floor:** not recorded this session — outstanding, add before experiment 2.
**Reference point on the robot:** not written down explicitly by name this
session, only used consistently by the same person — should be stated
precisely (e.g. "point on the floor under the wheel axle midpoint") before any
further hand-measured run, real or otherwise.

---

## Isaac Sim backend, 2026-09-20 — the matrix is complete

**All 22 runs recorded, every one `net.source = /ground_truth/odom`.** With
`gazebo` (2026-09-19) that is two of the three backends complete; `real` stays
deliberately partial by the decision recorded above.

### Conditions

- **World:** `empty_stage`, `headless:=true`, `rviz:=false`, no Nav2, no SLAM.
- **Physics rate: 240.0 Hz** — `isaacsim.launch.py`'s `physics_hz` default at
  this commit, not overridden on the command line. (The first attempt's Kit
  log echoed `--physics-hz 240.0` in its argument list; this run's log does
  not print the line, so the rate is established from the launch default
  rather than observed at runtime.)
- **`turtlebot3_isaacsim` commit:** `4231fbf`, the pin from A0.
- **DDS transport: shared memory**, unchanged, same as the Gazebo half. See
  below — no UDP-only profile was needed or applied.
- **`physics.floor.mu`:** `empty_stage` declares no `physics:` block, so
  `FLOOR_MU` = 1.0 as for Gazebo. Isaac reads friction from its own material
  definitions, not from this; recorded for symmetry with the Gazebo entry.
- **Floor, battery, reference point:** not applicable, simulated runs.
- **Real-time factor: not observed.** Neither backend's tooling prints one and
  no separate `/clock` measurement was taken this session.

### Two launches, not one, and which runs came from which

`sweep` ×3, `line` ×3, `spin_cw` ×3 and `spin_ccw` ×3 ran against one Isaac
launch; `square_cw` ×5 and `square_ccw` ×5 against a second, about four hours
later. The Gazebo matrix was one launch throughout.

This does not enter any result — every number is a delta over its own run,
resolved into that run's start pose — but it is a condition, so it is written
down. **The split falls between whole sequences, never inside one**: three
`square_cw` runs from the first launch were **deleted and re-run**, because a
sequence's repeats are what its standard deviation is computed over, and for
the UMBmark squares the CW/CCW comparison is the entire point. A launch
boundary inside one of those numbers would be a caveat no reader could undo;
re-running five squares cost eighteen minutes.

`run_experiment1.sh` now takes an optional third argument overriding the
matrix, which is what made the resume possible:

    ./scripts/run_experiment1.sh isaacsim 2026-09-20 "square_cw:5 square_ccw:5"

### What went wrong

**1. The 2026-09-19 diagnosis was wrong, and it was the expensive kind of
wrong.** Recorded above: "the blocker is FastDDS's shared-memory transport, not
stale segments", with `/dev/shm` at 2% used offered as the evidence. The real
cause is stale segments — a participant that dies without closing can leave a
*named mutex held*, and the next participant that hashes onto that port blocks
forever before rclcpp emits its first log line. Free space was never the
mechanism, so "2% used" exonerated nothing. `scripts/dds_clean.sh --kill`
cleared 42 objects and `drive_test` created its node instantly with Isaac
playing and SHM on.

The observation that killing Isaac released the block, which is what pointed at
the transport, fits the segment explanation too: fewer live participants means a
different port hash. **A workaround that works is not a diagnosis** — the
UDP-only profile did unblock it, and had it been applied, half of experiment 1
would have been recorded on a transport the other half never used.

**2. A bug in the shared instrument, which aborted the first attempt at
`sweep` r3.** `_net()` decided whether the run had ground truth by testing
`'gx' in self.samples[0]` — the *first* sample only. `/odom` and
`/ground_truth/odom` are separate subscriptions and `wait_for_odom()` waits for
the first of them, so when truth arrived a beat later the whole run silently
fell back to `/odom (NOT truth)` despite every phase carrying a `truth_d`. On
r3 exactly **20 samples of 11875** — about 0.4 s, inside `settle_pre`, robot
stationary — lacked it.

Fixed to select over the truth-bearing samples, which is how `_summarise()`
already decided the same question per phase. Recomputed from the discarded
run's own samples the fix gives `fwd −1.599 / lat +1.558 / yaw +855.3°`,
in line with r1 and r2, so no data was ever missing. **This would have hit
Gazebo identically**; it did not only because the P3D plugin happened to
publish before the first sample every time. The runner's
`assert net.source == '/ground_truth/odom'` is what caught it — it stopped the
matrix rather than writing 22 files whose provenance was wrong.

**3. The runner leaked Isaac Sim.** Its EXIT trap killed `ros2 launch`, but Kit
is not a child that dies with it, and a stage was left playing with nobody
driving it. The trap now kills `turtlebot3_isaacsim.py` as well, TERM then
KILL. Note it still leaves `robot_state_publisher` and the two bridge nodes
behind; those were killed by PID afterwards.

**4. `docker compose up -d` emptied the workspace mid-session.** `/ws/install`
is in the container's writable layer, not a volume (`docs/setup.md` says so),
so recreating the container left `/ws/src` and nothing else, and the resumed
runner died instantly on `source /ws/install/setup.bash: No such file or
directory`. `colcon build --symlink-install` restored it in 2.4 s. The same
recreation also hit the `/tmp/tb3_sim2real.docker.xauth` trap that
`docker/x11-auth.sh` documents: the host `/tmp` had been cleared, and Docker
creates a missing bind-mount source as a root-owned **directory**.

**5. `pkill -f <pattern>` kills the shell running it** when the pattern appears
in that shell's own command line — exit 143, silently, before the rest of the
line runs. It looked like Isaac had ignored the kill. `dds_clean.sh` documents
this exact trap and matches on executable paths for the reason; the ad-hoc
cleanup commands here did not.

### Anything that looks off

Reported without interpretation:

- **`line` lateral scatter is ±52 mm and yaw scatter ±1.65°** over three
  repeats, against Gazebo's ±3 mm and ±0.08°. Forward distance is tight
  (±2.9 mm).
- **`spin` is 10.3% short of the commanded rotation in both directions**, and
  the two directions agree with each other to 0.2°.
- **Neither square closes**: 1.14 m (CW) and 1.26 m (CCW) from the start mark.
- **`sweep` wheel tracking is worst at the lowest commanded rotation rate**
  (69–83% at `wz = 0.2`) and best at the highest (97–99% at `wz = 1.5`) — the
  opposite of what a velocity ceiling would produce. The wheel joints carry no
  `maxJointVelocity`, no `maxForce` and no limits in
  `payloads/Physics/physics.usda`. The full three-backend table is in
  `docs/experiment.md`.
