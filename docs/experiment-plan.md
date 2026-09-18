# The plan for the hardware experiments

Written 2026-09-18, the day before the real robot is available, out of a
full audit of the repository against the two experiments it is about to run.
`docs/experiment.md` is the shape the *results* go into; this is the ordered
list of work that produces them, split by the one thing that gates everything:
**whether the real robot is in the room**.

Two experiments, three backends each:

1. **Open space, open loop.** No scans, no Nav2. One `/cmd_vel` sequence,
   recorded. Measures actuation and odometry.
2. **The lab room, closed loop.** Boxes and walls built physically, copied
   into both simulators from one manifest, then Nav2 goals on all three.
   Measures whether the actuation gap survives to the task.

The audit's findings are recorded in full at the bottom, including the ones
**deliberately not being fixed** — those are there so nobody reopens them.

---

# Section A — can be done now, with no robot

Everything here runs against the simulators alone. An agent can pick these up
in any order; they do not depend on each other.

## A1. Verify Gazebo actually publishes `/ground_truth/odom` — **do this first**

**This is the one unverified thing in the whole chain.** Isaac Sim's ground
truth is proven by a recorded run
(`measurements/2026-09-18_isaacsim_wheel_odom.json` carries `truth_d` rows).
**Gazebo's never has been.** Every `drive_test` file from the Gazebo backend
predates the P3D plugin and contains no ground-truth rows at all, so the plugin
in `launch/backends/gazebo.launch.py` has been written and never exercised end
to end.

That matters because the failure mode is silent, and that file's own comment
names it: a plugin loaded the wrong way "is accepted and silently never
attaches". If P3D is not attaching, Gazebo has no ground truth, and the
**wheel → body** layer — slip, the whole point of experiment 1 — cannot be
measured on Gazebo at all.

```bash
ros2 launch tb3_bringup bringup.launch.py backend:=gazebo world:=empty_stage headless:=true
ros2 topic hz /ground_truth/odom                 # want ~50 Hz
ros2 run tb3_bringup drive_test --ros-args -p label:=gt_check -p out:=/tmp/gt.json
python3 -c "import json;d=json.load(open('/tmp/gt.json'));print(d['ground_truth']);
print([p.get('truth_d') for p in d['sequence']])"
```

Pass = `ground_truth: /ground_truth/odom` and non-null `truth_d` values. If it
fails, fix the plugin before anything else in this document.

**To answer the question directly: yes, both simulators have true position, and
no new node needs writing.** Gazebo gets it from a `libgazebo_ros_p3d.so`
plugin injected into a temp copy of the robot SDF; Isaac Sim gets it from the
`IsaacComputeOdometry` node that used to feed its `/odom` before 2026-09-18.
Both publish on `/ground_truth/odom`. Their origins differ — Gazebo's is
world-absolute, Isaac's is relative to the spawn pose — so **always use deltas,
never absolute positions**. On the real robot the topic does not exist and
cannot; that layer is measured by hand in the lab instead.

## A2. `drive_test` must record a rosbag

Experiment 1 is "drive and record rosbags", and `drive_test` currently records
no bag — only `nav_test` does (`tb3_bringup/tb3_bringup/nav_test.py:299`). The
JSON summary is a derived product; the bag is the raw data, and a metric nobody
thought of tonight can only be recovered from a bag.

Lift `start_bag()` out of `nav_test.py` into a shared helper and give
`drive_test` the same `bag_dir` parameter, so both instruments record the same
way. Topics for the open-loop run:

```
/odom  /cmd_vel  /joint_states  /tf  /tf_static  /imu  /clock  /ground_truth/odom
```

`/clock` and `/ground_truth/odom` simply produce nothing on the real robot,
which is correct and is how the bag records which backend it came from.

**On `/battery_state` (real robot only):** it is not part of the measurement,
it is a *control*. The OpenCR tracks a commanded wheel velocity less well as
the pack drains, so a run taken at the end of a session is not necessarily
comparable with one from the start — and without the voltage in the bag there
is no way to find that out afterwards. It costs one topic. Add it to the list
for `backend:=real`; ignore it if the analysis shows no effect.

## A3. RViz must use the simulator's clock

`launch/common/rviz.launch.py` declares `use_sim_time` and never forwards it,
and `nav2_bringup/rviz_launch.py` has no such argument to receive it. So RViz
runs on wall time inside both simulators and emits tf message-filter warnings.

Cosmetic on its own — but the warning text is **identical** to the one real
clock skew produces between the Pi and the workstation (B1), so leaving it will
actively confuse diagnosis on hardware day.

Fix: run `rviz2` as a plain `Node` with `parameters=[{'use_sim_time': ...}]`,
or keep the nav2 wrapper and set the parameter on the node afterwards.

## A4. Re-measure Isaac Sim's `/scan` rate in simulated time

Recorded on 2026-09-14: Gazebo 5.0 Hz, Isaac Sim **3.5 Hz**. The lidar profile
(`models/lidar_configs/turtlebot3_lds.json`) says `scanRateBaseHz: 5.0`, and
the real LDS-01 spins at 5 Hz, so Isaac is the odd one out.

**Why it matters, since it was asked:** a scan rate is evidence per metre
travelled. AMCL updates on scans, not on time; slam_toolbox adds a node per
scan. If Isaac genuinely delivers 30% fewer scans over the same trajectory,
then any localisation or mapping difference between the backends is partly a
sampling-rate difference and not a sensor-model difference — and the perception
row of `docs/experiment.md` stops being attributable, which is the one thing
that table exists to guarantee.

Two things to separate:

- That number is from **before** the 240 Hz physics change and may have been
  measured in wall time, where an RTF of 0.9 alone explains part of it.
- Re-measure in **simulated** time (`ros2 topic hz` under `use_sim_time`, or
  count messages against `/clock`) at the current 240 Hz default.

If it is 5.0 Hz in sim time, close this and record the number. If it is not,
it belongs in `turtlebot3_isaacsim`, which owns the profile and the render
loop.

## A5. Re-run the pre-flight checks, and keep them passing

Both are cheap and both passed on 2026-09-18:

```bash
docker compose exec tb3_ros bash -lc 'cd /repo && python3 scripts/check_worlds.py'
docker compose exec tb3_ros bash -lc 'source /ws/install/setup.bash && \
    cd /ws/src/tb3_bringup && python3 -m pytest test/ -q'
```

`check_worlds.py`: 3/3 worlds consistent. Launch-mode tests: 11 passed in
0.15 s. Run both after every change in this section.

## A6. Chrony on the workstation

The workstation half of B1 needs no robot. `sudo apt install chrony`, then
append to the stock `/etc/chrony/chrony.conf`:

```
allow 10.118.16.0/22   # answer time requests from the robot's subnet
local stratum 10       # keep serving even when our own upstream is unreachable
```

Leave the pool lines alone — they are how the workstation gets its own time.
Also check whether `ufw` is active; if it is, NTP needs
`sudo ufw allow from 10.118.16.0/22 to any port 123 proto udp`.

## A7. The AMCL initial-pose fix — code now, pin later (optional here)

The *implementation* needs no robot and no room: it reads the world's own
`spawn:`, so it can be written and verified on `turtlebot3_world` today. The
parts that wait for the room are the floor marker and the `lab_room` manifest,
and those are in **B5**, which is where this is recorded as blocking.

Doing it here is optional and safe — it only pins AMCL to where the manifest
already says the robot is. The proposal is written out in full in B5.

---

# Section B — needs the real robot

In order. Each step gates the ones after it.

## B1. Clock sync, before anything else is believed — **blocking**

Measured 2026-09-16: the workstation is **157 ms ahead** of the Pi, and neither
machine runs chrony. Both use `systemd-timesyncd` against **different
upstreams** — the workstation `ntp.ubuntu.com`, the Pi a relay on its own Wi-Fi
subnet — so the two clocks are disciplined independently and never compared.

157 ms is most of an LDS-01 scan period (200 ms) and the same order as Nav2's
`transform_tolerance`. The 2026-09-16 test already logged the symptom:

```
Message Filter dropping message: frame 'base_scan' ... the timestamp on the
message is earlier than all the data in the transform cache
```

Worse, the Pi has **no RTC** — no `/dev/rtc*`, no `fake-hwclock` — so it does
not drift by a constant offset. It boots at an arbitrary time and then *steps*
when NTP corrects it, and a step mid-run invalidates tf caches on both machines.

- Workstation side: **A6**, done in advance.
- Pi side: `sudo apt install chrony` (needs a password, so by hand), then
  `server 10.118.5.241 iburst prefer` in `/etc/chrony/chrony.conf`. Keep
  Ubuntu's stock `makestep 1.0 3` — that is what lets a just-booted RTC-less Pi
  take its one big jump immediately instead of slewing into a run.
- Verify: `chronyc tracking` on the Pi names the workstation as `Reference ID`,
  `chronyc sources -v` marks that line `*`, and then the check that trusts
  neither daemon — `date +%s.%N` on both, as close together as possible.
  **Target single- to low-double-digit milliseconds**, against the present 157.

Two cautions. Check the offset **right after the Pi boots**, because that is
when experiments start — not after it has been up an hour. And never let a
correction land *during* a run; let the clock settle before launching.

## B2. Four confirmations, one command each

All cheap, all capable of silently invalidating a day of data.

| what | command | expected |
|---|---|---|
| **lidar model** | `ros2 topic echo /scan --field range_max --once` | **3.5** = LDS-01 (matches both sims). 8.0 = LDS-02, and both simulators' 3.5 m ceiling then truncates the real robot's perception without erroring. `.env` says LDS-01; `docs/architecture.md:199` says `ld08_driver`, which is LDS-02 — **one of them is wrong, fix the doc after measuring.** |
| **no-return encoding** | `ros2 topic echo /scan --field ranges --once` | Isaac reports `-1.0`, Gazebo `inf`. Record what the real driver reports. **No change planned** — see "not being fixed" below — this is for the record only. |
| **scan rate** | `ros2 topic hz /scan` | ~5 Hz. Pairs with A4. |
| **odom rate** | `ros2 topic hz /odom` | ~20 Hz (recorded 2026-09-16). |

## B3. The robot's geometry

The URDF claim — one kinematic tree for all three backends — rests on the robot
and the container independently having the same stock `turtlebot3_description`.
Nothing checks this, and a version skew moves frame origins with no error.

```bash
dpkg -l ros-humble-turtlebot3-description | tail -1     # on the Pi and in the container
ros2 run tf2_ros tf2_echo base_footprint base_scan      # want (-0.032, 0, 0.182)
```

The robot was bought as a stock TurtleBot3, so this is expected to pass. If
anything looks off, measure the lidar height physically with a tape and compare
against 0.182 m — that number is the composition of `base_joint` (0.010) and
`scan_joint` (0.172) from the URDF, and Isaac Sim's asset uses it exactly.

## B4. Experiment 1 — open space, open loop

No scans, no Nav2, no map. `world:=empty_stage` on every backend,
`nav:=false slam:=false`.

```bash
ros2 launch tb3_bringup bringup.launch.py backend:={gazebo|isaacsim|real} world:=empty_stage
ros2 run tb3_bringup drive_test --ros-args \
    -p label:=<backend> -p sequence:=sweep \
    -p out:=measurements/<date>_drive_<backend>.json \
    -p bag_dir:=measurements/bags/<date>_drive_<backend>
```

- **`sequence:=sweep`**, not `default`: it holds four rotation rates and three
  linear ones long enough to reach steady state, which is what separates a
  fixed torque from a gain error. It includes `wz = 0.2`, the rate where Isaac
  still misses on one wheel and the rate Nav2's final alignment actually uses.
- **Repeats.** Gazebo is bit-identical run to run; Isaac Sim is not (spread
  0.0063 m, 0.042 rad). The real robot will be the noisiest of the three, so
  take enough repeats to state a standard deviation — that number goes in the
  Repeatability table of `docs/experiment.md` and decides how many repeats
  experiment 2 needs.
- **Record the physics rate with every Isaac run.** Do not compare an Isaac
  measurement against one taken before 2026-09-17 without checking it.
- **Clear the DDS domain between runs.** `pgrep -af "ros2 launch|gzserver|isaac-sim"`
  before believing any misbehaviour; `docker compose restart tb3_ros` is the
  reliable reset.

Ground truth: `/ground_truth/odom` on both simulators, and **by hand in the
lab** on the real robot — mark the start pose, drive, measure where it stopped.
That is the only way the wheel → body layer exists on hardware.

## B5. Pin the start pose — **blocking for experiment 2, on every backend**

**This is the item most likely to be forgotten, so it is stated plainly: with
no fix, every single Nav2 run on every backend requires a human to click "2D
Pose Estimate" in RViz.** That click injects a few centimetres and a few
degrees of *independent random error into every run*, and that error lands
directly in `goal_err_m`, `path_ratio` and `elapsed_s` — the exact numbers
being compared between backends. It cannot be separated from the sim-to-real
gap afterwards.

Nothing in the repository sets it today: there is no `/initialpose` publish and
no `set_initial_pose`, so AMCL's default of `false` leaves it waiting for a
human.

Use AMCL's own parameter, not a topic publish — the publish races AMCL's
activation, and that race is what swallowed the pose in the 2026-09-16 hardware
test. Verified against the installed `nav2_common/launch/rewritten_yaml.py`:
it accepts dotted absolute paths *and* creates keys absent from the source file,
so `config/nav2_params.yaml` needs no edit and stays verbatim upstream.

```python
# bringup.launch.py, in the `elif nav:` branch
from nav2_common.launch import RewrittenYaml
x, y, z, yaw = world.spawn
pinned = RewrittenYaml(source_file=params, param_rewrites={
    'amcl.ros__parameters.set_initial_pose': 'true',
    'amcl.ros__parameters.initial_pose.x':   f'{x:g}',
    'amcl.ros__parameters.initial_pose.y':   f'{y:g}',
    'amcl.ros__parameters.initial_pose.yaw': f'{yaw:g}',
}, convert_types=True)
stack.append(inc(nav2('localization_launch.py'), map=map_yaml, params_file=pinned))
```

Chaining is safe: `localization_launch.py` wraps the result in its own
`RewrittenYaml` for `use_sim_time`.

**The physical half:** a floor marker in the real room at the manifest's
`spawn:` coordinates. Without it the pinned pose is a lie and AMCL starts
confidently wrong, so the marker is part of the experimental protocol, not a
convenience.

## B6. Build the lab room, and put it in the registry

Physically: boxes and walls. Then author it as a manifest — **not two
hand-made environments**. `scripts/check_worlds.py` can prove the manifest path
and cannot prove anything about two environments built separately, and that
proof is the methodological contribution of the whole repository.

`worlds/small_office/world.yaml` is the template and the precedent: a room made
of primitives, dimensions straight off a tape measure. Boxes-and-walls is
exactly the case it was written for.

```bash
# author worlds/lab_room/world.yaml, then:
docker compose exec tb3_ros bash -lc 'cd /repo && scripts/build_world.sh lab_room'
docker compose exec tb3_ros bash -lc 'cd /repo && python3 scripts/check_worlds.py'
```

Set `spawn:` to the floor marker's coordinates (B5). Measure to the same
reference in the real room and in the manifest — the walls' interior faces are
the honest thing to measure against.

**Do not use `scripts/make_map.py` here.** Its own docstring says why: for a
world with a real counterpart, generating the map from the model makes
`check_worlds.py`'s footprint test tautological — the model compared against a
picture of itself — and destroys the one check that can notice the model
drifting from the real room. `scripts/clone_world.py` is also out; `roadmap.md`
§3a records it as being removed rather than extended.

## B7. Map the real room, once, for all three backends

```bash
ros2 launch tb3_bringup bringup.launch.py backend:=real world:=lab_room slam:=true
# drive it around, then in a second terminal:
scripts/save_map.py lab_room
```

This writes into `worlds/lab_room/map/` and adds the `map:` key to the
manifest. There is no `map:=` argument by design: one map per world, the same
file on all three backends, which is what makes the task-layer comparison
comparable at all.

Then re-run `check_worlds.py`. Its footprint score — "% of the map modelled,
% of the model contradicting it" — is now a **real** check rather than a
tautology, and it is the acceptance number for *did I build the room I
described*. `small_office` scores 100% / 0%; treat a materially worse score as
a reason to fix the room or the manifest before measuring anything.

Before driving anything, park the robot on the marker and run the perception
instrument:

```bash
ros2 run tb3_bringup scan_test --ros-args -p label:=real -p world:=lab_room \
    -p out:=measurements/<date>_scan_real.json
```

`scan_test` compares the real `/scan` against ranges ray-cast from
`world.yaml` — the same file both simulators are generated from. On hardware
that is a direct measurement of how accurately the room was built, using the
same instrument that measures the simulators. A `leak` (a beam passing through
a wall the manifest says is there) is always a defect.

## B8. Add the lab room's goals to `nav_test`

`GOALS` in `tb3_bringup/tb3_bringup/nav_test.py:56` is a hardcoded dict keyed
by world name; a `lab_room` world raises `SystemExit` until it has an entry.
That is deliberate — a goal list that moves with the code is not a fixed
experiment.

Keep the existing shape: three goals, ending where it began so a run repeats
without respawning, at least one of them mostly **rotation** — the regime where
Isaac's residual is worst and where a closed loop is most likely to hide it.
Check each goal is physically reachable with clearance for the burger's 0.1 m
`robot_radius`.

## B9. Experiment 2 — Nav2 in the lab room, all three backends

```bash
ros2 launch tb3_bringup bringup.launch.py backend:={gazebo|isaacsim|real} \
    world:=lab_room nav:=true
ros2 run tb3_bringup nav_test --ros-args \
    -p label:=<backend> -p world:=lab_room \
    -p out:=measurements/<date>_nav_<backend>.json \
    -p bag_dir:=measurements/bags/<date>_nav_<backend>
```

Same map, same goals, same `nav2_params.yaml` on all three. Repeats per the
number derived in B4.

## B10. Friction — an open decision, to be settled with real data

The audit found that floor friction is **not** part of the single source of
truth, and the two simulators are very far apart:

| | floor | wheel |
|---|---|---|
| gazebo | `model://ground_plane`, mu 100 / mu2 50 | mu **100000** / mu2 100000 |
| isaacsim | authored `GroundPlane`, static/dynamic **1.0**, restitution 0 | `turtlebot3_isaacsim`'s wheel material |
| real | the lab floor — **unknown until B4** | rubber tyre + plastic skid |

`world.yaml` carries no `physics:` block and `scripts/build_world.py` writes no
`<surface><friction>`, so neither value is in the manifest that
`check_worlds.py` proves is shared. Gazebo's floor comes from a stock include;
Isaac's is authored in `runtime/turtlebot3_isaacsim.py:377`.

This is the cause of the headline result "Gazebo cannot show slip", and the
wording matters: **mu = 100000 is not a physical value, it is a don't-slip
sentinel.** So the honest sentence is "Gazebo's stock burger has slip
disabled", not "Gazebo's contact model predicts no slip".

**Decision, taken 2026-09-18: revisit this with the real numbers in hand.** If
the real robot's slip turns out to sit far from both simulators, tuning the
friction is on the table. The reasoning is that it does not make sense to
report a large divergence as a finding when one parameter — a parameter that is
currently an arbitrary sentinel rather than a measurement — would bring it
close.

The rule that still holds is the one in `docs/experiment.md`: **never tune one
backend to match another backend.** Setting friction from a value measured on
the real robot is not that — it is parameterising a model from ground truth,
which is the same thing the 240 Hz physics rate did. Tuning Gazebo until it
agrees with Isaac would be, and stays forbidden.

So: measure first (B4), then decide, and whatever is decided, record both
backends' friction values in `docs/experiment.md` so the result is reported
with its parameters instead of resting on an undeclared default.

## B11. Analysis

Fill the empty tables in `docs/experiment.md` — actuation, perception, task,
repeatability. Interpret a specific finding in a dated `.md` in
`measurements/`, the way `isaac_angular_deficit.md` and `nav2.md` do. Then
append to `docs/history.md`.

One standing warning from this repository's own history, worth re-reading
before writing any conclusion: **three times now, a mean has impersonated a
constant here.** "Under-rotates by 30%" was the mean of a chattering wheel;
"rotated by half a beam" was the mean of a coin flip. Both times the giveaway
was a correction that did not close the residual. When an instrument averages,
keep a path back to the unaveraged samples — which is the other reason A2
matters.

---

# Deliberately not being fixed

Decided 2026-09-18. Recorded so they are not reopened.

### The `/scan` angular convention differs between backends — **no action**

| | `angle_min` | `angle_increment` |
|---|---|---|
| gazebo | 0.0 | 0.017493 (= 6.28 / 359) |
| isaacsim | −3.14159 | 0.0174533 (= exactly 1°) |
| real LDS-01 | expected 0.0 | expected 0.0174533 |

Two separate differences: Isaac starts its sweep at the rear where Gazebo
starts at the front, and Gazebo's angular scale is 0.23% wide because upstream's
`model.sdf` writes `<max_angle>6.28</max_angle>` instead of 6.283185 —
accumulating to 0.81° by the last beam.

**Not being fixed, and not going in the thesis.** Nav2, AMCL and slam_toolbox
all reconstruct the angle as `angle_min + i * angle_increment` and are
indifferent to both; the practical difference in how the two simulators read a
scan is small; and this is not a question the work is about.

**The one thing that follows from it:** any analysis script written against the
bags must reconstruct the angle and must never index `ranges[i]` directly to
mean a direction. `ranges[0:30]` is "in front" on Gazebo and "behind" on Isaac.

### `/scan` encodes no-return three different ways — **confirm only, no change**

Isaac `-1.0`, Gazebo `inf`, the real driver to be recorded in B2. Nav2 survives
all three: AMCL maps anything `<= range_min` to max range, and the costmap
obstacle layer range-filters. No change planned on any backend.

### Gazebo's lidar sits 1 mm low — **closed**

SDF puts the sensor at `base_link + 0.171` → 0.181 m above `base_footprint`;
the URDF, Isaac Sim and the real robot all say 0.182 m. Physically irrelevant
for a horizontal beam against vertical walls. Recorded only because it is the
evidence that Gazebo's *sensor origin* does not come from the URDF, even though
`base_footprint → base_scan` does.

### One `nav2_params.yaml` for all three backends — **kept**

Verbatim `turtlebot3_navigation2`'s `param/humble/burger.yaml`, used
unconditionally; the `config/nav2_<backend>.yaml` auto-prefer mechanism was
removed on 2026-09-16 on purpose, because per-backend tuning absorbs the
sim-to-real gap into the tuning. Kept for now. **Not a closed question** — if
the measurements show a specific parameter that changes a conclusion, that is
worth discussing with evidence. It is the reflex that is banned, not the idea.

---

# What the audit confirmed is already sound

Verified 2026-09-18 by reading the code and re-running the checks, so no one
re-derives it:

- **Isaac Sim's `/odom` is integrated from `/joint_states`**, not the chassis
  prim (`src/turtlebot3_isaacsim/nodes/wheel_odometry.py`, position not
  velocity). All three backends drift. Odometry over-reports a `wz = 0.5` turn
  by +4.83% against +0.08% on a straight line.
- **`/ground_truth/odom` on both simulators, on neither hardware** — subject to
  A1 for Gazebo.
- **240 Hz PhysX on Isaac Sim**, one default, in the package that owns the
  physics. Wheel tracking within 1% linear and 96-100% angular except
  `wz = 0.2`.
- **One URDF above `base_footprint`** — the same stock `turtlebot3_description`
  on all three; on `real` the `robot_state_publisher` runs *on the robot*
  rather than in the container, which is the same file on a different host
  (subject to B3).
- **`use_sim_time` is derived from `backend:=` and never set by hand.**
  Confirmed that all three nav2 launch files put it in `param_substitutions`
  and that `RewrittenYaml` rewrites it at every depth of the params file.
- **One map per world**, shared by all three backends, with no `map:=`.
- **Geometry is identical by construction, and currently is:**
  `check_worlds.py` 3/3 consistent; `small_office` 100% of map modelled, 0%
  contradicted.
- **The lidar specs match between the two simulators** — 360 beams,
  0.12-3.5 m, 0.01 m Gaussian noise, 5 Hz nominal.
- **The launch interface is pinned** — 11 tests, 0.15 s, no GPU.
- **Cross-subnet DDS works**, unicast peers from `.env`.
