# The plan for the hardware experiments

Written 2026-09-18, out of a full audit of the repository against the two
experiments it is about to run. `docs/experiment.md` is the shape the *results*
go into; this is the ordered work that produces them, split by the one thing
that gates everything: **whether the real robot is in the room**.

1. **Open space, open loop.** No scans, no Nav2. One `/cmd_vel` sequence,
   recorded. Measures actuation and odometry.
2. **The lab room, closed loop.** Boxes and walls built physically, copied into
   both simulators from one manifest, then Nav2 goals on all three.

This file states *what to do and in what order*. Where a decision already has a
written rationale, it links rather than restating — `docs/roadmap.md` holds the
reasoning, `docs/status.md` holds what is verified. Findings deliberately **not**
being fixed are at the bottom, so they are not reopened.

---

# Section A — can be done now, with no robot

No dependencies between these; take them in any order except A0.

**Worked through on 2026-09-18. A0-A5 and A7 are done; A6 is surveyed and
waits only on a sudo password.** Each item below records what it found, and two
of them found the plan wrong: `RewrittenYaml` cannot create absent keys (B5),
and `/imu` comes from Gazebo rather than Isaac Sim (A2). Section B is untouched
and still needs the robot.

## A0. The `.repos` pin — **fixed 2026-09-18, listed so it is not undone**

`tb3_sim2real.repos` pinned `turtlebot3_isaacsim` to
`fix-map-mirroring-add-small-worlds`, a feature branch abandoned on 2026-09-15
and by then eight commits behind. That ref has **no `nodes/wheel_odometry.py`**
and defaults `physics_hz` to **60.0** — so `scripts/workspace.sh` on a fresh
machine silently restored both defects this repository had just measured away.
Now pinned to `humble`. If a run must be reproduced exactly, pin the commit
(`4231fbf`, the tip every 2026-09-18 measurement was taken against).

**Check before trusting any Isaac number:**

```bash
grep -A1 'turtlebot3_isaacsim:' -A3 tb3_sim2real.repos | grep version
git -C src/turtlebot3_isaacsim log -1 --oneline
test -f src/turtlebot3_isaacsim/nodes/wheel_odometry.py && echo "wheel odom present"
```

## A1. Verify Gazebo actually publishes `/ground_truth/odom` — **PASSED 2026-09-18**

It attaches. `ros2 topic hz` reads **49.986 Hz** against the plugin's
`update_rate: 50.0`, `frame_id: world`, `child_frame_id: base_footprint`, and a
full `drive_test` reports `ground_truth: /ground_truth/odom` with a non-null
`truth_d` on every phase. Recorded:
`measurements/2026-09-18_gazebo_ground_truth.json`.

The wheel → body layer on Gazebo exists and reads **+0.06% / −0.55% / +0.16%**
across pivot, arc and straight — no measurable slip, which is what `mu = 100000`
predicts and what Isaac's −7.1% pivot slip now has to be compared against.
Numbers and the wording that goes with them: `docs/status.md`.

**Both simulators do have true position and no new node is needed** — Gazebo
from a P3D plugin injected into a temp copy of the robot SDF, Isaac from the
`IsaacComputeOdometry` node that used to feed its `/odom`. Their origins differ
(Gazebo world-absolute, Isaac spawn-relative), so **always use deltas**. On
hardware the topic does not exist and cannot; that layer gets measured by hand
in the lab instead.

## A2. `drive_test` must record a rosbag — **done 2026-09-18**

`tb3_bringup/recording.py` holds the recorder, `drive_test` and `nav_test` both
take `bag_dir:=`, and the topic lists live there rather than in either
instrument. The open-loop list:

```
/odom  /cmd_vel  /joint_states  /tf  /tf_static  /imu  /clock
/ground_truth/odom  /battery_state
```

Not every backend publishes all of them, and that is deliberate: rosbag2
records nothing for a topic that never appears, so the absence is how the bag
says which backend wrote it. Measured 2026-09-18, correcting an assumption in
this plan: `/imu` is published by **Gazebo** and the real robot, and **not by
Isaac Sim** — the reverse of what was written here. `/clock` and
`/ground_truth/odom` are the two simulators'; `/battery_state` is the robot's.

**Root-owned files are handled rather than avoided**, and for more than bags.
`recording.give_back()` chowns a written path to whoever owns the directory it
landed in, so anything an instrument writes into the bind mount comes out owned
by the host user. Verified on a 3.9 MB bag of a full `default` sequence — eight
topics, 11622 messages, `etfrobot:etfrobot`.

This item said "bags"; the JSON summaries and the `.samples.json` traces beside
them are written by the same root process into the same mount and had the same
problem — every such file already in `measurements/` was `root:root`. All three
instruments now call `give_back()` on every file they write, and the ones
already on disk were chowned back. `measurements/bags/` and `*.beams.json` are
gitignored, like `*.samples.json`.

**On `/battery_state` (real robot only):** a *control*, not a measurement. The
OpenCR tracks a commanded wheel velocity less well as the pack drains, so a run
at the end of a session may not be comparable with one from the start — and
without the voltage in the bag there is no way to find that out afterwards. One
topic. Drop it if the analysis shows no effect.

## A3. RViz must use the simulator's clock — **done 2026-09-18**

`launch/common/rviz.launch.py` now runs `rviz2` as a plain `Node` with
`parameters=[{'use_sim_time': ...}]` instead of wrapping
`nav2_bringup/rviz_launch.py`, which declares no such argument and could not
receive it. Upstream's one other contribution — shutting the launch down when
the window closes — is kept.

Verified against a running Gazebo: `ros2 param get /rviz2 use_sim_time` →
**True**, and the log's tf lines now carry *simulated* stamps (`at time 76.601`,
not a wall epoch). The skew-lookalike reason string — `the timestamp on the
message is earlier than all the data in the transform cache` — is **gone, 0
occurrences**. What remains is `discarding message because the queue is full`,
a different and benign reason, which is exactly the distinction that had to
survive: B1's diagnosis reads the reason string, and it now has only one
candidate.

## A4. Re-measure Isaac Sim's `/scan` rate in simulated time — **done 2026-09-18, nothing to fix**

**It is 5.0 Hz.** The 3.5 Hz in `docs/status.md` was a wall-clock number at an
RTF below 1.0, exactly as this item suspected.

| | `/scan`, simulated time | `/scan`, wall clock | RTF |
|---|---|---|---|
| gazebo | **5.0000 Hz** | 4.9985 Hz | 0.9997 |
| isaacsim | **5.0000 Hz** | 4.1925 Hz | 0.8385 |

Both match the profile's `scanRateBaseHz: 5.0` and the real LDS-01, with zero
jitter. Isaac's RTF is confirmed independently from `/clock` (25.417 simulated
seconds in 29.971 wall, **0.848**), so the wall figure is the sim figure times
the RTF and nothing else.

**So there is no sampling difference, and the perception row stays
attributable.** AMCL updates per scan and slam_toolbox adds a node per scan;
both backends deliver the same scans per simulated second over the same
trajectory. Nothing goes to `turtlebot3_isaacsim`.

`scan_test` now reports `scan_rate_hz` and `wall_rate_hz` in every run, so the
confound cannot return — and on hardware that first field is the robot's own
clock, which is the same statement for it and pairs with B2's `ros2 topic hz`.
The rest of the 2026-09-14 rate table had the same confound and is re-measured
in `docs/status.md`.

## A5. Keep the pre-flight checks passing

Both cheap, both passing after everything in this section (3/3 worlds
consistent; **13** tests in 0.28 s — two of the new ones pin AMCL's start pose,
and one of those is what caught the `RewrittenYaml` error in B5). Run after
every change here.

```bash
docker compose exec tb3_ros bash -lc 'cd /repo && python3 scripts/check_worlds.py'
docker compose exec tb3_ros bash -lc 'source /ws/install/setup.bash && \
    cd /ws/src/tb3_bringup && python3 -m pytest test/ -q'
```

## A6. Chrony on the workstation — **surveyed 2026-09-18, needs a sudo password**

Everything that can be checked without root has been. Confirmed on the
workstation that day: chrony is **not installed** (`dpkg -l chrony` → `un`),
`systemd-timesyncd` is the active NTP service, the address is **10.118.5.241**
— the one the Pi's `server` line names — and **`ufw` is active**, which settles
the open question in `docs/roadmap.md` §2: the firewall rule is required, not
optional. Without it the Pi's NTP requests are dropped and chrony on the robot
sits at `?` with nothing on either machine saying why.

Three commands, all needing a password, so they are run by hand:

```bash
sudo apt install chrony
sudo ufw allow from 10.118.16.0/22 to any port 123 proto udp
printf 'allow 10.118.16.0/22\nlocal stratum 10\n' | sudo tee -a /etc/chrony/chrony.conf
sudo systemctl restart chrony
```

Why each line, and the Pi half: `docs/roadmap.md` §2.

## A7. The AMCL initial-pose fix — **done 2026-09-18**

Written and verified on `turtlebot3_world`, which needs neither robot nor room.
The floor marker and the `lab_room` manifest are what wait for the room. Full
account, including the `RewrittenYaml` claim that turned out to be false:
**B5**.

---

# Section B — needs the real robot

In order. Each step gates the ones after it.

## B1. Clock sync, before anything is believed — **blocking**

Measured 2026-09-16: the workstation is **157 ms ahead** of the Pi, neither
machine runs chrony, and they sync to different upstreams. 157 ms is most of an
LDS-01 scan period and the same order as Nav2's `transform_tolerance`; the
2026-09-16 test already logged the dropped-scan message filter line. The Pi has
**no RTC**, so it does not drift by a constant — it boots arbitrary and *steps*,
and a step mid-run invalidates tf caches on both machines.

Full config, both machines, and the reasoning for chrony over timesyncd:
**`docs/roadmap.md` §2.** The workstation half is A6; the Pi half needs a sudo
password, so it is a by-hand step.

Verify with the check that trusts neither daemon — `date +%s.%N` on both, as
close together as possible. **Target single- to low-double-digit ms.** Check it
**right after the Pi boots**, because that is when experiments start, and never
let a correction land during a run.

## B2. Four confirmations, one command each

| what | command | expected |
|---|---|---|
| **lidar model** | `ros2 topic echo /scan --field range_max --once` | **3.5** = LDS-01, matching both sims. 8.0 = LDS-02, whose range both simulators would then truncate silently. `.env` says LDS-01, `docs/architecture.md:199` says `ld08_driver` (LDS-02) — **one is wrong; fix the doc after measuring.** |
| **no-return encoding** | `ros2 topic echo /scan --field ranges --once` | Isaac `-1.0`, Gazebo `inf`. Record what the real driver does. **No change planned** — see below. |
| **scan rate** | `ros2 topic hz /scan` | ~5 Hz. Pairs with A4. |
| **odom rate** | `ros2 topic hz /odom` | ~20 Hz. |

## B3. The robot's geometry

The "one URDF for all three" claim rests on the robot and the container
independently having the same stock `turtlebot3_description`. Nothing checks
it, and a skew moves frame origins with no error.

```bash
dpkg -l ros-humble-turtlebot3-description | tail -1     # on the Pi and in the container
ros2 run tf2_ros tf2_echo base_footprint base_scan      # want (-0.032, 0, 0.182)
```

Stock hardware, so this should pass. If anything looks off, measure the lidar
height with a tape against 0.182 m — the composition of `base_joint` (0.010)
and `scan_joint` (0.172), which Isaac Sim's asset uses exactly.

## B4. Experiment 1 — open space, open loop

```bash
ros2 launch tb3_bringup bringup.launch.py backend:={gazebo|isaacsim|real} world:=empty_stage
ros2 run tb3_bringup drive_test --ros-args \
    -p label:=<backend> -p sequence:=sweep \
    -p out:=measurements/<date>_drive_<backend>.json \
    -p bag_dir:=<somewhere not root-owned>/<date>_drive_<backend>
```

- **`sequence:=sweep`**, not `default`: four rotation rates and three linear
  ones, each held to steady state, which is what separates a fixed torque from
  a gain error. Includes `wz = 0.2` — the rate where Isaac still misses on one
  wheel, and the rate Nav2's final alignment actually uses.
- **Repeats.** Gazebo is bit-identical run to run, Isaac is not, and the real
  robot will be noisiest. Take enough to state a standard deviation: that
  number fills the Repeatability table in `docs/experiment.md` and decides how
  many repeats experiment 2 needs.
- **Record the physics rate with every Isaac run** (CLAUDE.md: never compare
  across rates without checking).
- **Clear the DDS domain between runs.** `pgrep -af "ros2 launch|gzserver|isaac-sim"`
  before believing any misbehaviour; `docker compose restart tb3_ros` is the
  reliable reset. `docs/status.md` has the full trap.

Ground truth is `/ground_truth/odom` on both simulators and **by hand in the
lab** on the real robot — mark the start pose, drive, measure where it stopped.
That is the only way the wheel → body layer exists on hardware.

## B5. Pin the start pose — **blocking for experiment 2, every backend**

**Stated plainly so it is not forgotten: with no fix, every Nav2 run on every
backend needs a human to click "2D Pose Estimate" in RViz.** That click is a
few centimetres and a few degrees of *independent random error per run*,
landing directly in `goal_err_m`, `path_ratio` and `elapsed_s` — the exact
numbers being compared. It cannot be separated from the sim-to-real gap
afterwards.

**The code half is done, 2026-09-18** — `pin_initial_pose()` in
`bringup.launch.py`, on the `elif nav:` branch, on every backend. AMCL's own
parameters, not a topic publish: the publish races AMCL's activation, and that
race swallowed the pose in the 2026-09-16 test. Decision and alternatives:
`docs/roadmap.md` §1.

**The plan said to do this with `nav2_common`'s `RewrittenYaml`, and that was
wrong.** The audit claimed it creates keys absent from the source file via a
dotted absolute path. It does not, on Humble: `substitute_params` walks the
paths the file ALREADY HAS (`pathify`) and rewrites only those, so all four of
these keys — absent from upstream's `nav2_params.yaml` — were silently dropped.
The launch succeeded, a params file was written, and AMCL waited for a mouse
anyway, with no error anywhere. `test_nav_pins_amcl_to_the_manifest_spawn`
performs the rewrite and reads the result back, which is what caught it, and is
why it checks the written file rather than the request.

What it does instead is write a temp copy of the params file with the four keys
added — the same move, for the same reason, as `with_ground_truth()` in
`backends/gazebo.launch.py`, so `config/nav2_params.yaml` stays verbatim
upstream. Chaining is unaffected: `localization_launch.py` wraps whatever path
it is handed in its own `RewrittenYaml` for `use_sim_time`.

Verified on `turtlebot3_world`, which needs no robot and no room. What still
waits on the room is the `lab_room` manifest and the floor marker.

**The physical half:** a floor marker in the real room at the manifest's
`spawn:`. Without it the pinned pose is a lie and AMCL starts confidently
wrong, so the marker is part of the protocol, not a convenience.

## B6. Build the lab room, and put it in the registry

Physically: boxes and walls. Then **author it as a manifest** — not two
hand-made environments. `check_worlds.py` can prove the manifest path and can
prove nothing about two environments built separately, and that proof is the
methodological contribution of the whole repository.

`worlds/small_office/world.yaml` is the template and the precedent: a room of
primitives, dimensions straight off a tape measure. Boxes-and-walls is exactly
the case it was written for.

```bash
# author worlds/lab_room/world.yaml, then:
docker compose exec tb3_ros bash -lc 'cd /repo && scripts/build_world.sh lab_room'
docker compose exec tb3_ros bash -lc 'cd /repo && python3 scripts/check_worlds.py'
```

Set `spawn:` to the floor marker's coordinates (B5). Measure to the same
reference in the room and in the manifest — wall interior faces are the honest
one.

**Neither `scripts/clone_world.py` nor `scripts/make_map.py` is used here.**
Both are retired for this direction and both say so at the top now; the
reasoning is `docs/roadmap.md` §3a and the banner in `worlds/README.md`. Every
other chapter of `worlds/README.md` — meshes, colour, OBJ, drift checking —
applies unchanged to an authored world.

## B7. Map the real room, once, for all three backends

```bash
ros2 launch tb3_bringup bringup.launch.py backend:=real world:=lab_room slam:=true
# drive it around, then in a second terminal:
scripts/save_map.py lab_room
```

Writes into `worlds/lab_room/map/` and adds `map:` to the manifest. There is no
`map:=` by design: one map per world, the same file on all three backends,
which is what makes the task comparison comparable.

Then re-run `check_worlds.py`. Its footprint score is now a **real** check
rather than a tautology, and it is the acceptance number for *did I build the
room I described*. `small_office` scores 100% / 0%; treat a materially worse
score as a reason to fix the room or the manifest before measuring anything.

Before driving anything, park on the marker and run the perception instrument:

```bash
ros2 run tb3_bringup scan_test --ros-args -p label:=real -p world:=lab_room \
    -p out:=measurements/<date>_scan_real.json
```

`scan_test` compares the real `/scan` against ranges ray-cast from `world.yaml`
— the same file both simulators are generated from. On hardware that is a
direct measurement of how accurately the room was built, with the same
instrument that measures the simulators. A `leak` is always a defect.

## B8. Add the lab room's goals to `nav_test`

`GOALS` in `tb3_bringup/tb3_bringup/nav_test.py:56` is keyed by world name; a
`lab_room` world raises `SystemExit` until it has an entry. That is deliberate
— a goal list that moves with the code is not a fixed experiment.

Keep the shape: three goals, ending where it began so a run repeats without
respawning, at least one mostly **rotation**. Check each is reachable with
clearance for the burger's 0.1 m `robot_radius`.

## B9. Experiment 2 — Nav2 in the lab room, all three backends

```bash
ros2 launch tb3_bringup bringup.launch.py backend:={gazebo|isaacsim|real} \
    world:=lab_room nav:=true
ros2 run tb3_bringup nav_test --ros-args \
    -p label:=<backend> -p world:=lab_room \
    -p out:=measurements/<date>_nav_<backend>.json \
    -p bag_dir:=<somewhere not root-owned>/<date>_nav_<backend>
```

Same map, same goals, same `nav2_params.yaml` on all three. Repeats per B4.

## B10. Friction — an open decision, to be settled with real data

Floor friction is **not** part of the single source of truth, and the two
simulators are far apart:

| | floor | wheel |
|---|---|---|
| gazebo | `model://ground_plane`, mu 100 / mu2 50 | mu **100000** |
| isaacsim | authored `GroundPlane`, 1.0 / 1.0, restitution 0 | `turtlebot3_isaacsim`'s wheel material |
| real | the lab floor — **unknown until B4** | rubber tyre + plastic skid |

`world.yaml` has no `physics:` block and `scripts/build_world.py` writes no
`<surface><friction>`, so neither value is in the manifest `check_worlds.py`
proves is shared. Gazebo's floor is a stock include; Isaac's is authored in
`runtime/turtlebot3_isaacsim.py:377`.

This is the cause of "Gazebo cannot show slip", and the wording matters:
**mu = 100000 is not a physical value, it is a don't-slip sentinel.** The honest
sentence is "Gazebo's stock burger has slip disabled", not "Gazebo's contact
model predicts no slip".

**Decided 2026-09-18: revisit with the real numbers in hand.** If the real
robot's slip sits far from both simulators, tuning friction is on the table —
it makes no sense to report a large divergence as a finding when one arbitrary
sentinel parameter explains it.

The rule that still holds, from `docs/experiment.md`: **never tune one backend
to match another backend.** Setting friction from a value measured on hardware
is not that — it is parameterising a model from ground truth, which is what the
240 Hz physics rate already did. Tuning Gazebo until it agrees with Isaac would
be, and stays out.

Measure first (B4), then decide; and either way record both backends' friction
values in `docs/experiment.md`, so the result is reported with its parameters
instead of resting on an undeclared default.

## B11. Analysis

Fill the tables in `docs/experiment.md`. Interpret a specific finding in a
dated `.md` in `measurements/`, as `isaac_angular_deficit.md` and `nav2.md` do.
Append to `docs/history.md`.

One standing warning from this repository's own history: **three times now, a
mean has impersonated a constant here.** "Under-rotates by 30%" was the mean of
a chattering wheel; "rotated by half a beam" was the mean of a coin flip. Both
times the giveaway was a correction that did not close the residual. When an
instrument averages, keep a path back to the unaveraged samples — the other
reason A2 matters.

---

# Deliberately not being fixed

Decided 2026-09-18. Recorded so they are not reopened.

### The `/scan` angular convention differs between backends — **no action**

| | `angle_min` | `angle_increment` |
|---|---|---|
| gazebo | 0.0 | 0.017493 (= 6.28 / 359) |
| isaacsim | −3.14159 | 0.0174533 (= exactly 1°) |
| real LDS-01 | expected 0.0 | expected 0.0174533 |

Isaac starts its sweep at the rear where Gazebo starts at the front, and
Gazebo's angular scale is 0.23% wide because upstream's `model.sdf` writes
`<max_angle>6.28</max_angle>` instead of 6.283185 — accumulating to 0.81° by
the last beam.

**Not being fixed, and not going in the thesis.** Nav2, AMCL and slam_toolbox
all reconstruct the angle as `angle_min + i * angle_increment` and are
indifferent; the practical difference in how the two simulators read a scan is
small; and it is not a question the work is about.

**The one consequence kept:** any analysis script written against the bags must
reconstruct the angle and must **never** index `ranges[i]` to mean a direction.
`ranges[0:30]` is "in front" on Gazebo and "behind" on Isaac.

### `/scan` encodes no-return three ways — **confirm only, no change**

Isaac `-1.0`, Gazebo `inf`, the real driver recorded in B2. Nav2 survives all
three: AMCL maps `<= range_min` to max range, the costmap obstacle layer
range-filters.

### Gazebo's lidar sits 1 mm low — **closed**

SDF puts the sensor at `base_link + 0.171` → 0.181 m above `base_footprint`;
URDF, Isaac and the real robot say 0.182 m. Irrelevant for a horizontal beam
against vertical walls. Recorded only as the evidence that Gazebo's *sensor
origin* does not come from the URDF, even though `base_footprint → base_scan`
does.

### One `nav2_params.yaml` for all three backends — **kept**

Verbatim upstream, used unconditionally; the `config/nav2_<backend>.yaml`
auto-prefer mechanism was removed 2026-09-16 on purpose
(`docs/architecture.md`). **Not a closed question** — if a measurement shows a
specific parameter that changes a conclusion, that is worth discussing with
evidence. It is the reflex that is banned, not the idea.

---

# What the audit confirmed, beyond what `docs/status.md` already records

`docs/status.md` is the verified-state doc and is not repeated here. These are
the checks this audit added:

- **`use_sim_time` reaches every Nav2 node.** All three nav2 launch files
  (`localization_`, `navigation_`, `slam_launch.py`) put it in
  `param_substitutions`, and `RewrittenYaml` rewrites it at every depth — so
  the mixed `False`/`True`/`true` literals inside the stock params file are all
  overridden by the value `bringup.launch.py` derives from `backend:=`.
- ~~**`RewrittenYaml` can create keys that are absent from the source file**, via
  dotted absolute paths.~~ **Wrong, and found wrong on 2026-09-18 by building
  it.** On Humble it rewrites only paths the source file already has. B5 has
  the detail and what replaced it.
- **Geometry parity holds right now** — `check_worlds.py` 3/3 consistent.
- **The launch interface is pinned** — 11 tests, 0.15 s, no GPU.
- **The two simulators' lidar specs match** — 360 beams, 0.12-3.5 m, 0.01 m
  Gaussian noise, 5 Hz nominal. The angular *convention* differs (above); the
  specs do not.
- **`/odom` means the same thing on all three** and the real backend's URDF is
  the same stock file, run on the robot rather than in the container —
  `docs/architecture.md`'s contract table, subject to B2 and B3.
