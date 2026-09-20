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

**Worked through on 2026-09-18. Section A is DONE, A0 through A7.** Each item
below records what it found, and two of them found the plan wrong:
`RewrittenYaml` cannot create absent keys (B5), and `/imu` comes from Gazebo
rather than Isaac Sim (A2).

**Section B is open from B6 onward.** *Session of 2026-09-19, robot on:
**B1, B2 and B3 are all done. B4's real leg is deliberately partial and closed:
`sweep` ×3 and `line` ×3 are recorded, `spin_cw`/`spin_ccw`/`square_cw`/
`square_ccw` were decided against in favour of moving to experiment 2** — see
`measurements/experiment1/notes.md`, "Real backend, 2026-09-19", for what ran,
what didn't, and why (including a units-mismatch finding in real
`/joint_states` velocity that's flagged but not fixed, and a `sweep` sequence
edit — 0.22→0.20 m/s top linear rate — that makes that one phase
non-comparable with the already-recorded `gazebo`/`isaacsim` files). The next
thing to do is **B6**, building the lab room and putting it in the registry.*

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

## A6. Chrony on the workstation — **done 2026-09-18**

Installed and configured by hand (it needs a password; nothing here runs it
unattended). Verified the same day, on the workstation, without root:

| | |
|---|---|
| `dpkg -l chrony` | `ii  4.5-1ubuntu4.2` |
| `systemctl is-active chrony` | `active` |
| `systemctl is-active systemd-timesyncd` | **`inactive`** — the handover happened |
| `chronyc tracking` | `time.cloudflare.com`, stratum 4, last offset **+52 µs** |
| `/etc/chrony/chrony.conf` | `allow 10.118.16.0/22` and `local stratum 10`, lines 63-64 |
| `ss -uln` | **`0.0.0.0:123`** open — it is serving, not just consuming |

Only one daemon may own the clock and only one now does. The workstation is a
stratum-4 server on 10.118.5.241 with the robot's subnet allowed, which is
everything B1 needs from this side.

**Two things still need root and neither can be settled from here:** whether the
`ufw allow ... port 123` rule actually landed (`sudo ufw status | grep 123`) and
whether the robot has polled (`sudo chronyc clients` — it answers `501 Not
authorised` otherwise). The second is only answerable once the Pi is up, so both
are folded into B1 rather than left as loose ends here.

## A7. The AMCL initial-pose fix — **done 2026-09-18**

Written and verified on `turtlebot3_world`, which needs neither robot nor room.
The floor marker and the `lab_room` manifest are what wait for the room. Full
account, including the `RewrittenYaml` claim that turned out to be false:
**B5**.

---

# Section B — needs the real robot

In order. Each step gates the ones after it.

## B1. Clock sync, before anything is believed — **DONE 2026-09-19, both halves**

Measured 2026-09-16: the workstation was **157 ms ahead** of the Pi, neither
machine ran chrony, and they synced to different upstreams. 157 ms is most of an
LDS-01 scan period and the same order as Nav2's `transform_tolerance`; the
2026-09-16 test already logged the dropped-scan message filter line. The Pi has
**no RTC**, so it does not drift by a constant — it boots arbitrary and *steps*,
and a step mid-run invalidates tf caches on both machines.

**The workstation is done (A6):** chrony 4.5 active, `systemd-timesyncd`
handed over, tracking an upstream at 52 µs, serving on UDP 123 with
`allow 10.118.16.0/22`. What remains is the Pi and the proof.

**State on 2026-09-19, robot up 22 minutes, re-measured:** the Pi still has **no
chrony** — `systemd-timesyncd` `active`, still pointed at `10.118.16.1`, still no
`/dev/rtc*`. Offset workstation − robot is **−28 ms** (ws *behind*), from five
`date +%s.%N` pairs over one multiplexed SSH connection, rtt ~46 ms so the
uncertainty is ±23 ms. That is not the +157 ms of 2026-09-16 and it is not
progress: the workstation moved to chrony/Cloudflare on 2026-09-18, so the pair
is simply floating somewhere else. **Two independent clocks, neither aware of
the other, offset of the same order as the measurement uncertainty** — which is
exactly the state B1 exists to end.

Full config and the reasoning for chrony over timesyncd: **`docs/roadmap.md`
§2.** The Pi half needs a sudo password there, so it is a by-hand step:

```bash
# on the Pi
sudo apt install chrony
echo 'server 10.118.5.241 iburst prefer' | sudo tee -a /etc/chrony/chrony.conf
sudo systemctl restart chrony
sudo chronyc makestep          # take the boot jump now, not mid-run
```

### What happened, 2026-09-19

The Pi half was run by hand. It worked first time, and **no firewall rule was
needed** — see below. `chronyc sources -v` on the Pi marks `10.118.5.241`
**`*`**, the stock Ubuntu pool sources all `-` (not combined), which is `prefer`
doing its job.

**chrony needs about ten minutes before its own numbers mean anything**, and
reading them too early is misleading rather than merely imprecise. One poll in:
`Skew 1000000 ppm`, `Root dispersion 52.5 s` — it had selected the workstation
but had no idea yet how well. At 42 minutes of uptime:

| | one poll in | settled |
|---|---|---|
| `System time` | 1.07 ms fast | **0.40 ms slow** |
| `Last offset` | +4.93 ms | **−0.47 ms** |
| `Skew` | 1000000 ppm | **49.3 ppm** |
| `Root dispersion` | 52.5 s | **3.9 ms** |

### The third check had to be replaced, because it cannot see the answer

The plan's third check — `date +%s.%N` on both machines — is a daemon-independent
idea and the right instinct, but **its resolution floor is ±rtt/2, about ±20 ms
here**. It read −28 ms before chrony and −20 ms after. That difference is not
progress; it is the same noise twice, and the method cannot distinguish a
perfectly synced pair from a 20 ms offset. It is what found the original 157 ms
because 157 ms is far outside its floor.

What replaces it uses ROS itself, and is both independent of chrony *and* a
direct measurement of the thing B1 exists to protect. `/scan`'s `header.stamp`
is written by the Pi; the receive time is read on the workstation. **A one-way
message delay cannot be negative**, so the *minimum* observed stamp→receive
bounds the clock offset from below with no assumption about the network.

90 s of `/scan`, 442 messages, from the container:

| min | p50 | p90 | p99 | max | negative | > 200 ms |
|---|---|---|---|---|---|---|
| **+3.0 ms** | +10.1 | +23.6 | +49.2 | +138.2 | **0** | **0** |

So `offset(ws − robot) ≥ −3.0 ms`, which agrees with chrony's own −0.5 ms and
settles B1 to within a few ms rather than within 20. **Nothing arrives stamped
in the workstation's future** — which is precisely the condition that produced
the 2026-09-16 message-filter drops, and with the workstation 28 ms *behind* the
robot that morning, every scan did.

The tail is now the interesting number instead. **p99 is 49 ms and the worst of
442 was 138 ms**, against Nav2's usual `transform_tolerance` of 0.2–0.3 s. Wi-Fi
jitter, not clock skew, is what will eat that margin on hardware — and it is
within it today. Re-measure if a hardware Nav2 run starts dropping scans;
`measurements/2026-09-19_clock_sync.json` is the baseline.

### `ufw` was never enforcing, on either machine

The plan said the port-123 rule was mandatory because "`ufw` is active on the
workstation (checked 2026-09-18)". **That check was wrong, and the way it was
wrong is worth keeping.** `systemctl is-active ufw` reports `active` and
`is-enabled` reports `enabled` on *both* machines — while `/etc/ufw/ufw.conf`
says `ENABLED=no` and `sudo ufw status` says `Status: inactive`. The systemd
unit is up; the firewall it manages is off. The unit being active says nothing
about whether any packet is filtered.

No rule was needed and none was added. If ufw is ever turned on, the rule from
`docs/roadmap.md` §2 becomes real again.

### Still true, and still the protocol

Check it **right after the Pi boots**, because that is when experiments start,
and never let a correction land during a run. The Pi has no RTC, so every boot
repeats the jump: `sudo chronyc makestep` takes it immediately, and then wait
out the ten minutes above before believing `chronyc tracking`.

## B2. Four confirmations — **done 2026-09-19, all four**

Measured against the real robot with its stock `robot.launch.py` running on the
Pi and the reads taken from the container, which also exercises the unicast-DDS
path of `docs/network.md`. Raw record:
`measurements/2026-09-19_real_interface.json`, 40 scans.

| what | expected | **measured** |
|---|---|---|
| **lidar model** | 3.5 = LDS-01 | **3.5**, from `hls_lfcd_lds_driver/hlds_laser_publisher`. `.env` was right and `docs/architecture.md` was wrong; the doc is fixed. |
| **no-return encoding** | Isaac `-1.0`, Gazebo `inf` | **`0.0`** — a *third* encoding, and neither simulator's. See below. |
| **scan rate** | ~5 Hz | **4.986 Hz** on the robot's own clock, jitter std **0.43 ms** |
| **odom rate** | ~20 Hz | **19.988 Hz**, jitter std 5.4 ms |

Three more facts the same read produced, none of which needed another run:

- **The real scan's angular convention is `angle_min = 0.0`,
  `angle_increment = 0.0174533` (exactly 1°), `angle_max = 6.26573` (359°).**
  So it starts at the front like Gazebo and steps by exactly one degree like
  Isaac Sim — Gazebo's increment is the 0.23%-wide 0.017493 from upstream's
  `6.28`. Both simulators still differ from the robot in one respect each, and
  the "not being fixed" decision below is unchanged: every consumer
  reconstructs the angle arithmetically.
- **0.8% of returns exceed `range_max`** — 113 beams of 14400, from 3.512 m out
  to **4.191 m**, with `range_max` advertised as 3.5. The driver does not clamp
  its own declared maximum. **Neither simulator can produce this**: both cut
  cleanly at 3.5. Nav2 is safe (the costmap obstacle layer range-filters and
  AMCL clamps to `range_max`), so this needs no fix — but an analysis script
  written against the bags must not assume `r <= range_max`.
- **Of 360 beams, 169 read 0.0 in every one of the 40 scans, 84 flicker, 107
  never do.** The flickering ones are the dropout the encoding below is
  actually about; the always-zero ones are just an open room past 3.5 m.

## B3. The robot's geometry — **done 2026-09-19, passed, and stronger than asked**

The check asked for matching `turtlebot3_description` versions. The versions do
**not** match — and it does not matter, because the file does:

| | source | version |
|---|---|---|
| robot | `~/turtlebot3_ws/src/turtlebot3` @ `90a68bd`, branch `humble`, built from source | 2.3.7 |
| container | `ros-humble-turtlebot3-description` apt | 2.3.6-1jammy.20260718.021508 |

`turtlebot3_burger.urdf` is **byte-identical across the two**, md5
`51b1f9517b2666efefee18310009703d`. That is the claim the interface contract
actually rests on, and it is now checked rather than assumed — a version
comparison alone would have raised a false alarm here.

`base_footprint → base_scan` on the robot: **`(-0.032, 0, 0.182)`**, identity
rotation — the wanted figure exactly, so no tape measure was needed and Isaac
Sim's asset agrees with the hardware.

## B4. Experiment 1 — open space, open loop — **gazebo + isaacsim done, real partial by decision**

*Status: `gazebo` complete 2026-09-19 and `isaacsim` complete 2026-09-20, 22
runs each in `measurements/experiment1/`. The isaacsim blocker was stale Fast
DDS segments, not the shared-memory transport, so both backends ran on the same
transport and no UDP decision was needed. `real` is deliberately partial.
Results in `docs/experiment.md`, conditions in that directory's `notes.md`.
The sequences below are superseded by the four in
`measurements/experiment1/README.md`: `sweep` alone ends at a pose no tape can
measure, which is why `line`, `spin_*` and `square_*` (UMBmark) were added.* — **real leg deliberately partial, closed 2026-09-19**

`sweep` ×3 and `line` ×3 done for `real`; `spin_cw`/`spin_ccw`/`square_cw`/
`square_ccw` decided against, not forgotten — the experimenter chose to move on
to experiment 2 rather than complete the matrix. Full account:
`measurements/experiment1/notes.md`.

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

## B10. Friction — half settled 2026-09-19, the value still needs the robot

Floor friction is **not** part of the single source of truth, and the two
simulators are far apart:

| | floor | wheel |
|---|---|---|
| gazebo, until 2026-09-19 | `model://ground_plane`, mu 100 / mu2 50 | mu **100000** |
| **gazebo, now** | **`physics.floor.mu` in world.yaml, 1.0** | **the same 1.0** |
| isaacsim | authored `GroundPlane`, 1.0 / 1.0, restitution 0 | 1.0, authored into the .usd |
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

### Done 2026-09-19: the sentinel is gone, and it explained nearly all of it

The structural half no longer waits on the robot and has been built. Friction is
manifest data now — `physics.floor.mu`, default 1.0, written into the generated
world and carried to the wheel — so it is inside what `check_worlds.py` proves
instead of being two undeclared defaults in two packages.

Measured, four conditions, one `sweep` each: Gazebo pivot slip goes from −0.20%
to **−4.66%** at `wz = 0.5` and to **−15.66%** at `wz = 1.5`, straight-line slip
unchanged. **At `wz = 0.5` that is 0.05 percentage points from Isaac Sim's
−4.61%.** Two physics engines that disagreed by an order of magnitude agree once
one number nobody chose is replaced by one that is declared. ODE's
pair-combination rule was measured on the way and it is the **minimum**. Full
table and the traps: `docs/worknotes/2026-09-19-gazebo-friction.md`.

**What still needs the robot, and it is now the only open part:** 1.0 is dry
rubber on vinyl from a handbook band of roughly 0.6–1.0, and it is the top of
that band — the end that flatters Gazebo. B4 measures the real robot's pivot
slip; fitting `physics.floor.mu` to reproduce it turns the assumption into a
calibration against this platform on this floor. Until that happens **no text
may call 1.0 measured**, and the band is not a specific paper read for this
repository — the citation has to be verified before it is used.

*Why fitting is legitimate and not the banned move: it parameterises a model
from ground truth, exactly as the 240 Hz physics rate did. Tuning Gazebo until
it agreed with Isaac would be the banned move, and the agreement above was an
outcome of setting a physical value, not a target.*

*A direct measurement would be better than a fit and was designed: drawbar pull
for the tyre, a dead-drag for the skid, a load split from a kitchen scale. It
needs a luggage/fishing scale, which is not available, so it is recorded here
as the stronger method if one ever appears — it would be an independent
cross-check on the fit rather than a replacement for it.*

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

One standing warning from this repository's own history: **four times now, a
mean has impersonated a constant here.** "Under-rotates by 30%" was the mean of
a chattering wheel; "rotated by half a beam" was the mean of a coin flip; and
"0.8% of returns exceed range_max" was 19.4% read in a corner. Twice the
giveaway was a correction that did not close the residual; the third time it was
measuring again somewhere else. When an instrument averages, keep a path back to
the unaveraged samples — the other reason A2 matters.

## B12. The real lidar's range exceeds both simulators' — decide before experiment 2

**Measured 2026-09-19**, stationary, 120 scans, robot in a cluttered room with
roughly 5 m of open space on one side.
`measurements/2026-09-19_real_max_range.json`.

| | advertised `range_max` | furthest return observed | returns past 3.5 m |
|---|---|---|---|
| gazebo | 3.5 | 3.5 (hard cut) | none, by construction |
| isaacsim | 3.5 | 3.5 (`farRangeM: 3.5`) | none, by construction |
| **real LDS-01** | **3.5** | **4.20 m** | **3518 of 18172 finite returns — 19.4%** |

The first read of this found 0.8% and called it a tail artefact. It is not:

- **23 beams returned a range beyond 3.5 m on all 120 scans.** The farthest
  reliable one sits at **4.087 m with a standard deviation of 0.036 m** — a
  stable surface, seen every single revolution, that **both simulators would
  report as no-return**.
- The two 20°-wide sectors with genuinely open space (110–129°, 250–269°)
  returned **nothing at all**, which is the control: the far returns are
  surfaces, not noise.

**What is NOT established:** that 4.2 m is the sensor's ceiling. Nothing came
back between 4.2 and 5 m — and nothing is known to be there. Separating "the
sensor stops" from "the room stops" needs a wall at a measured distance, which
is B6's room. The claim that survives without any geometry assumption is the one
that matters: **the sensor returns reliably past its advertised `range_max`, and
the datasheet figure both simulators use is wrong for this unit.**

### Where it lands, and where it does not

| consumer | limit | does the extra range reach it? |
|---|---|---|
| AMCL | `laser_max_range: 100.0` | **yes** — nothing is clamped before the likelihood field |
| costmaps | `obstacle_max_range: 2.5`, `raytrace_max_range: 3.0` | **no** — everything past 3.0 m is discarded anyway |

So this is a **localisation** difference, not a planning one, and it cuts the
right way to matter: the real robot gives AMCL roughly 40% more usable range
than either simulator does, in a room the same size. That is a candidate
explanation for any localisation divergence experiment 2 finds, and it must be
settled *before* that result is interpreted rather than after.

### The decision

Raising `farRangeM` / `<max>` to a measured value is **parameterising a model
from ground truth**, the same move as the 240 Hz physics rate, and is allowed by
`docs/experiment.md`'s rule. Tuning one simulator to match the *other* is not,
and is not what this would be. Two things to settle first:

1. **Which number.** 4.2 m is this room's furthest surface, not a measured
   sensor limit. B6 gives a wall at a known distance; measure the drop-off
   against it and use that.
2. **Whether the reflectance model can carry it.** Isaac's profile has
   `minReflectance: 0.1` and `minReflectanceRange: 3.5` — the range at which
   that minimum reflectance is still detected. Raising `farRangeM` alone leaves
   that pair inconsistent, and the Isaac lidar is `turtlebot3_isaacsim`'s, not
   this repository's. Gazebo has no reflectance model at all, so its `<max>` is
   a clean cut either way — which is itself a difference in *kind* between the
   two simulators, not just in the number.

Until then, **nothing is changed**, and one consequence is already live: an
analysis script written against the bags must not assume `r <= range_max`. A
histogram binned to `range_max` silently drops a fifth of the real returns.

---

# Deliberately not being fixed

Decided 2026-09-18. Recorded so they are not reopened.

### The `/scan` angular convention differs between backends — **no action**

| | `angle_min` | `angle_increment` |
|---|---|---|
| gazebo | 0.0 | 0.017493 (= 6.28 / 359) |
| isaacsim | −3.14159 | 0.0174533 (= exactly 1°) |
| real LDS-01 | **0.0** (measured 2026-09-19) | **0.0174533** (measured) |

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

### `/scan` encodes no-return three ways — **all three now measured, no change**

| | no return | measured |
|---|---|---|
| isaacsim | `-1.0` | |
| gazebo | `inf` | |
| real LDS-01 | **`0.0`** | 2026-09-19, B2 |

Literally three ways, one per backend, with no two agreeing. Nav2 survives all
of them: AMCL maps `<= range_min` to max range and `0.0 < 0.12`, the costmap
obstacle layer range-filters. **No change** — but an analysis script must handle
all three, and on the real robot a `0.0` carries no information about whether
the beam saw nothing or the driver dropped it. 84 of 360 beams flickered between
`0.0` and a finite range over 40 stationary scans; 169 were `0.0` throughout.

### The real lidar sees a metre further than either simulator — **open decision, moved out of this section**

First measured as a 0.8% tail and written off. **Re-measured against ~5 m of
open space the same day, it is not a tail**, and it is the largest
sim-to-real gap in the perception row so far. It now has its own item: **B12**.

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
