# Open design decisions, and what to do next

Decisions taken in discussion but not yet built, with the reasoning, so the
next session does not re-derive them. Written 2026-09-16, the day a real robot
first appeared. Status of what *is* built: `docs/status.md`.

## 1. The start pose has to be pinned, not clicked

A real robot's odometry starts at zero wherever it was switched on. It does not
know it is standing at the manifest's `spawn:`; it knows only that it has not
moved since boot. Something must tell AMCL where in the map that is.

Three ways, and the choice is a measurement decision rather than a convenience
one:

- **RViz "2D Pose Estimate"** — a human with a mouse, once per run. Injects a
  few cm and a few degrees of *independent random error into every run*, which
  then lands in path length and time-to-goal and cannot afterwards be separated
  from the sim-to-real gap being measured. Fine for a demo, wrong for a result.
- **A fixed, automatically published pose** — what the simulated backends
  already do, from the manifest. The real robot can do the same if it is
  physically started on a floor marker at those coordinates. Then all three
  backends begin a run from the same pose in the same map with no human in the
  loop, which is what makes their traces comparable.
- **Global localization** (particles spread over free space, drive until it
  converges). Keep as a robustness mode; do not make it the experimental
  default. It requires an uncontrolled amount of driving before a run can
  start, and `worlds/README.md` already records that this arena is *nearly
  symmetric* — the case where a uniform cloud settles confidently into the
  wrong hypothesis.

**Decided:** the manifest's `spawn:` supplies the initial pose on every backend,
including `real`. A world name stays sufficient to define a run.

*The original decision added an `initial_pose:=` override "the way `map:=`
overrides the world's map". Built 2026-09-18 without one, because `map:=` no
longer exists either — it was removed for being a way for a run to drift away
from the world it claims to be in, and an overridable start pose is the same
argument. Add it the day a run genuinely needs to start somewhere other than
the marker.*

**Build it with Nav2's own parameters**, not a topic publish: AMCL has
`set_initial_pose: true` with `initial_pose.{x,y,yaw}`. A `/initialpose` publish
races AMCL's activation — that race is exactly what swallowed the pose during
the 2026-09-16 hardware test. *Built 2026-09-18; how, and the `RewrittenYaml`
dead end, are in `docs/experiment-plan.md` B5.*

**Physical prerequisite, confirmed available:** the robot can be placed at an
exact known location in the real room. The manifest's spawn coordinates become
that spot. Without this the pinned pose is a lie and AMCL starts wrong, so the
marker is part of the experimental protocol, not a convenience.

## 2. Clock sync between robot and workstation — **done 2026-09-19**

Not `use_sim_time`, which asks *which clock a node reads* (`/clock` vs the OS).
This asks *whether two machines' OS clocks agree*. On hardware both sides run
`use_sim_time:=false`, so both read their own clock, and there are two of them.

Every `header.stamp` is written by the publisher using its own clock. Scans and
`tf odom->base_footprint` are stamped by the Pi; Nav2 does the lookups on the
workstation. A Pi clock two seconds behind makes every scan arrive already
stale: tf extrapolation errors, costmaps refusing to clear observations, AMCL
dropping scans.

The 2026-09-16 test logged exactly that line — `Message Filter dropping
message: frame 'base_scan' ... the timestamp on the message is earlier than all
the data in the transform cache`. Most likely startup transient in that case,
but it is also the textbook skew symptom and the two are indistinguishable by
eye.

**A Raspberry Pi has no battery-backed RTC.** It boots at whatever time it last
shut down and then *jumps* when NTP corrects it. So this is not a constant
offset that could be tolerated — it is large at boot and steps
discontinuously, and a step mid-run invalidates tf caches on both sides.

**Done:** chrony on both, the Pi syncing to the workstation so the two agree
even when the campus network does not — the workstation 2026-09-18, the Pi
2026-09-19. The reasoning below is kept because it is why the configuration
looks the way it does; what was measured afterwards is
`docs/experiment-plan.md` B1.

### Measured state, 2026-09-16

Neither machine has chrony. Both run `systemd-timesyncd`, and — this is the
part that matters — **they sync to different upstreams**: the workstation to
`ntp.ubuntu.com`, the Pi to `10.118.16.1`, a relay on its own Wi-Fi subnet.
Two independently-disciplined clocks, never compared with each other.

Near-simultaneous `date +%s.%N`, workstation minus robot: **+157 ms**, the
workstation ahead. The Pi had been up ten days, so that is the *steady-state*
offset, not the boot transient. 157 ms is most of an LDS-01 scan period
(200 ms at 5 Hz) and is the same order as Nav2's usual `transform_tolerance`
of 0.2-0.3 s — so it is already eating the margin the message filter needs,
which is consistent with the dropped-scan line the hardware test logged.

The Pi has **no `/dev/rtc*`** — confirmed, not assumed. It also has no
`fake-hwclock` (Ubuntu Server 22.04 does not ship it, unlike Raspberry Pi OS),
so there is not even a "last known good time" to boot into: the clock
free-runs from whatever the hardware gives it and then jumps. Every reboot
repeats this. It is steady-state behaviour for this hardware, not a one-time
calibration.

### The configuration

`sudo apt install chrony` on both. The package stops and disables
`systemd-timesyncd` itself — only one daemon may own the clock — and since
neither machine has chrony yet, there is no existing conflict to unpick first.
**`sudo` on the Pi requires a password**, so this is a step to run by hand, not
something a script will do unattended.

Workstation, appended to the stock `/etc/chrony/chrony.conf` (keep its pool
lines; they are how it gets its own time):

```
allow 10.118.16.0/22   # answer time requests from the robot's subnet
local stratum 10       # keep serving even when our own upstream is unreachable
```

`local stratum 10` is the fallback-authority idiom: chrony only falls back to
it when it has nothing better, so it never overrides real upstream sync. Without
it, a workstation that has lost the campus network refuses to serve the robot
at all — which is precisely the lab-with-no-internet case.

Pi, in its `/etc/chrony/chrony.conf`:

```
server 10.118.5.241 iburst prefer
```

`iburst` for a fast first sync, which matters with no RTC; `prefer` so the
selection algorithm tracks the workstation specifically rather than averaging
it with a pool source. Keep Ubuntu's stock `makestep 1.0 3` line: it lets
chrony *step* the clock for the first three updates if the offset exceeds a
second, instead of slewing. That is exactly right for a just-booted RTC-less
Pi — take the big jump once, immediately, before anything is launched.
(`initstepslew` is an `ntpd` directive and does not exist in chrony;
`makestep` is the whole story. `rtcsync` in the stock file is dead weight here
with no RTC, harmless either way.)

**`ufw` is NOT enforcing on either machine — corrected 2026-09-19.** This
section said it was active on the workstation and the rule was mandatory. The
check behind that was `systemctl is-active ufw`, which reports `active` (and
`is-enabled` reports `enabled`) on *both* machines — while `/etc/ufw/ufw.conf`
carries `ENABLED=no` and `sudo ufw status` answers `Status: inactive`. **The
unit is up; the firewall it manages is off.** The Pi synced first time with no
rule added.

Keep the rule written down, because it is right the day ufw is turned on, and a
`?` that never becomes `*` in `chronyc sources -v` is exactly what a silently
dropped NTP request looks like:

```
sudo ufw allow from 10.118.16.0/22 to any port 123 proto udp
```

### Done on the workstation, 2026-09-18

Installed and configured by hand. Verified without root: chrony **4.5** `active`,
`systemd-timesyncd` **`inactive`** (the package performed the handover, so only
one daemon owns the clock), `chronyc tracking` disciplined to
`time.cloudflare.com` at stratum 4 with a last offset of **+52 µs**, both
`allow 10.118.16.0/22` and `local stratum 10` present in
`/etc/chrony/chrony.conf`, and `ss -uln` showing **`0.0.0.0:123`** — it is
serving, not merely consuming.

The workstation is now a stratum-4 NTP server on 10.118.5.241 with the robot's
subnet allowed. **The Pi half and the verification are `docs/experiment-plan.md`
B1**, and both need the robot powered on.

### Checking it

What matters more than the config is the checking. On the Pi, `chronyc
tracking` should name the workstation as `Reference ID` and `chronyc sources -v`
should mark that line `*`, not `+` or `?`. `chronyc clients` on the workstation
should list the robot once it has polled. Then the check that depends on
neither daemon's self-report: `date +%s.%N` on both, as close together as you
can manage. **Aim for single- to low-double-digit milliseconds**, not the
present 157. `chronyc makestep` forces an immediate correction rather than
waiting for chrony to slew, which is what you want after a boot rather than
mid-run.

Two things to be careful about. The offset right after the Pi boots is the
interesting one, since that is when experiments start — check it then, not
after the machine has been up an hour. And a correction applied *during* a run
is worse than a constant offset, because it invalidates tf caches on both
sides; let the clock settle before launching anything.

*Amended 2026-09-19, after doing it.* The `date +%s.%N` comparison is kept above
because it is how the original 157 ms was found, but **it cannot confirm a
success** — its floor is ±rtt/2, about ±20 ms over this Wi-Fi, and it read −28 ms
before chrony and −20 ms after. Use the ROS measurement in
`docs/experiment-plan.md` B1 instead. And `chronyc tracking` is meaningless for
the first ten minutes: one poll in it reported a skew of 1000000 ppm and a root
dispersion of 52 seconds while already tracking the right server.

## 3. `slam:=true`, on every backend — **built, 2026-09-16**

Follows `nav2_bringup`'s own shape, where `slam` swaps `localization_launch.py`
for `slam_launch.py`. Here it swaps `map_server` + AMCL for slam_toolbox, and
relaxes the "this world declares no map" error — the run is about to make one.

**What was actually built is better than this, and the paragraph above
understates it.** SLAM and navigation are not alternatives. slam_toolbox
supplies `/map` and `map -> odom`; the navigation stack consumes them and does
not care which of slam_toolbox or AMCL produced them. So `slam` and `nav` are
*independent*, and `slam:=true nav:=true` is a real mode — Nav2 planning over a
map that is still being drawn:

|  | `nav:=false` | `nav:=true` |
|---|---|---|
| `slam:=false` | robot only | `map_server` + AMCL + Nav2 |
| `slam:=true`  | slam_toolbox + Nav2 | Nav2 while mapping |

Both default false, and the dispatch in `bringup.launch.py` is flat — no
combination is rejected and neither flag is phrased as the absence of the
other. That also means `nav2_bringup/bringup_launch.py` is no longer included:
its whole job was the `IfCondition(['not ', slam])` switch, which has nothing
left to do. `slam_launch.py`, `localization_launch.py` and
`navigation_launch.py` are included directly instead. A side effect worth
knowing: those three default `use_composition` to False where
`bringup_launch.py` defaulted it True, so Nav2's nodes are separate processes
now — which retires the `docs/network.md` failure where the container comes up
and no composable node is ever loaded into it.

`backend:=real` with neither flag no longer errors. It starts RViz, which is
the one useful thing the workstation can do with no stack selected, and is also
how you see in one second that a robot publishing `/scan` has no `odom` frame.

**Collapsed to three modes, 2026-09-16.** Two problems with the grid above.
First, two cells named a "teleop" that no launch file in this repository ever
started — `grep -rn teleop` finds prose, not a launch file. Second,
`slam:=true` with `nav:=false` was a real, distinct mode: SLAM with no
planner running against it, thinly useful and one more combination to reason
about. `bringup.launch.py` now folds it in: `slam:=true` runs Nav2
unconditionally, so it and `slam:=true nav:=true` are now the same mode rather
than two — `nav:=true` alongside `slam:=true` changes nothing, and is accepted
rather than rejected. Three modes result: bare robot (neither flag), `nav:=true`
(saved map), `slam:=true` (Nav2 while mapping). To drive without Nav2, run
`ros2 run turtlebot3_teleop teleop_keyboard` or `ros2 run tb3_bringup
drive_test` in a second terminal; that is named in `README.md` now instead of
implied by a mode that did not exist.

**SLAM is allowed on the simulated backends too**, against the instinct that it
only makes sense on hardware. Three maps of one room are then available:

1. the geometric map from the manifest (`make_map.py`) — ground truth
2. a SLAM map made in the simulator — what the sensor model and SLAM produce
   given perfect geometry
3. a SLAM map made in the real room

1 vs 2 isolates error introduced by SLAM and the sensor model. 2 vs 3 isolates
how far the simulated environment differs from the real one. Run SLAM only on
hardware and those two error sources stay fused, and SLAM noise gets attributed
to the sim-to-real gap.

**Decided: slam_toolbox**, not the Cartographer that the TurtleBot3 docs use.
It is what `nav2_bringup/slam_launch.py` itself runs, so `slam:=true` becomes
an include of upstream's own launch rather than a parallel arrangement.

*The second half of that reasoning was wrong and is corrected here, 2026-09-16.*
It said slam_toolbox is a lifecycle node that comes up under Nav2's lifecycle
manager. On Humble it is not. `async_slam_toolbox_node` is built on plain
`rclcpp::Node` — no `LifecycleNode`, no `use_lifecycle_manager` parameter
anywhere in the package — and `slam_launch.py` sets `lifecycle_nodes =
['map_saver']`, so the only thing its `lifecycle_manager_slam` governs is
`map_saver_server`. slam_toolbox self-activates on construction and manages
itself. This does not change the decision — being upstream's own choice is
reason enough — but nothing should be built expecting a lifecycle transition
that will never arrive.

Two more facts worth having before starting. `nav2_bringup/slam_launch.py`
includes slam_toolbox's **`online_sync_launch.py`**, the *sync* node, not the
`online_async_launch.py` that most tutorials reach for. And `bringup_launch.py`
already does the switch this wants, as `IfCondition(slam)` against
`IfCondition(PythonExpression(['not ', slam]))` — so the minimal change is to
declare `slam` in `common/nav2.launch.py` and forward it into the existing
include, letting upstream fork, rather than arranging anything here.

**Built, and stronger than decided: there is no `map:=` argument at all.** A
map passed on the command line is a map that drifts away from the world it
describes. `world:=` is required, and its map is `worlds/<name>/map/` or it has
not been made yet — which is what `slam:=true` is for.

**Decided: the map is saved into `worlds/<name>/map/<name>.yaml`, never to
`map_saver_cli`'s default.** A map that lands in the working directory is a map
that drifts away from the world it describes, which is the failure the registry
exists to prevent. `map_saver_cli` is manual by nature, so this wants a thin
wrapper that takes a world name and writes into its directory — closing the
loop from SLAM to manifest to all three backends.

Built as `scripts/save_map.py <world>`, run by hand in a second terminal while
SLAM is still up — deliberately not automatic on shutdown, where a
half-finished run would silently overwrite a good map. It refuses to replace an
existing map without `--force`, and it adds the `map:` key to the manifest,
because a map on disk the manifest does not declare is invisible to every
backend.

Two ways to write the file, both present on Humble:
`ros2 run nav2_map_server map_saver_cli -f worlds/<name>/map`, which subscribes
to `/map` once and works for any publisher; or `ros2 service call
/slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '...'}}"`, which
asks slam_toolbox directly and so cannot race the `/map` subscription. Prefer
`map_saver_cli`: it is the same tool a geometric map from `make_map.py` would
use, so there is one way to write a `map.yaml` rather than two. (Note
`/slam_toolbox/save_map` and friends are hardcoded absolute service names, not
derived from the node name — renaming the node would not move them.)

**Caveat on the three-map comparison above:** it assumes the manifest is
authored to match the real room, so that `make_map.py` yields ground truth. The
world-generation tooling is not settled (see below), and if a world is ever
derived *from* a SLAM map instead, its geometry inherits SLAM's error and
comparison 1-vs-2 stops meaning what it says here.

## 3a. World generation is unsettled, and the clone helpers are going

`scripts/clone_world.py` and the generator path around it are **not considered
good** and are expected to be removed rather than extended. The lab world will
be provided as an authored USD and SDF plus a manifest, not derived from a
scan.

Nothing should be built on top of the clone path in the meantime. The question
of which direction is the source of truth — a room measured and authored into a
manifest, versus a manifest generated from a scan of the room — is deliberately
left open and will be settled when the replacement is designed.

## 4. URDF ownership — settled, no work

The robot's description is stock `turtlebot3_description`, the same upstream
file this repository loads, so the split recorded in `docs/status.md` costs
nothing in practice.

It *could* be forced the other way — launch `turtlebot3_node` and the lidar
directly on the robot instead of `robot.launch.py`, leaving the container's
`robot_state_publisher` as the only one — but it should not be. Keeping it on
the robot means static transforms do not cross the network, and the robot stays
independently useful. Better arrangement, not a compromise.

## 5. Drop `robot:=local|remote`, keep the concept — **done, 2026-09-16**

Always remote for a robot that drives; `local` needs the OpenCR and lidar
tethered by USB to the workstation, which cannot happen while it is mobile.

Rather than a flag nobody sets, describe the system in the two layers ROS
already uses: a **robot layer** (drivers, odometry, state publisher) supplied
either by hardware or by a simulator, and a **workstation layer** (map server,
localization, Nav2, RViz) that is identical everywhere. `backend:=` then
answers one question — who provides the robot layer — and `real` contributing
no local processes reads as correct rather than broken.

Keep the tethered-USB path as a comment, not an argument. It comes back the day
an x86 SBC or a Jetson goes on the robot.

**Built.** The `robot` argument is gone; `backends/real.launch.py` is deleted
rather than left unreferenced, because an unreferenced backend file is exactly
the thing that goes stale.

The tethered alternative — OpenCR and lidar on the workstation's own USB, so
the robot layer runs there too — is deliberately not an argument, because a
robot that drives cannot be tethered. It comes back the day an x86 SBC or a
Jetson goes on the robot, and the way back is to include upstream's
`turtlebot3_bringup/launch/robot.launch.py` rather than to re-derive it. One
thing that costs an hour if it is re-derived: `turtlebot3_node` declares
`namespace` as a statically typed parameter with no default, so it must be
passed as a PARAMETER and not merely as a frame prefix, or the process aborts
before it opens the serial port with `Statically typed parameter 'namespace'
must be initialized`.

One consequence, and it is the reason section 3's error was worth writing now:
`backend:=real` contributes no local processes, so `backend:=real nav:=false`
starts *nothing*. It now raises instead of exiting silently. `ros2 launch`
ignores unknown arguments, so a stale `robot:=remote` in someone's shell history
is inert rather than an error.

## 6. Isaac Sim's lidar is not rotated — diagnosed 2026-09-18

Found 2026-09-17 by `ros2 run tb3_bringup scan_test`, which parks the robot at
the manifest spawn in `small_office` and compares every beam against a range
**ray-cast from `world.yaml`** — the same file both simulators were generated
from, so there is no reference scan and no hand-measured room.

Fitting the angular offset that minimises the wall residual over every wall
beam:

| | best-fit offset | residual rms |
|---|---|---|
| gazebo | **+0.0000 rad (+0.000°)** | 0.00242 m |
| isaacsim | **+0.0095 rad (+0.544°)** | 0.00608 m, down from 0.01509 |

Gazebo is aligned exactly. Isaac reads **0.544°, which is 0.54 of one 1.0°
beam**, and that was recorded as a hint at a beam-indexing convention.

**The hint was half right and the wrong shape.** Full diagnosis in
`docs/worknotes/2026-09-18-lidar-half-beam.md`; in three lines:

- With the profile's noise off, the azimuth the sensor reports and the azimuth
  the geometry implies agree to **0.0000°**. Nothing is rotated, and there is
  no mounting or tf error.
- It **is** an indexing convention: `ROS2PublishLaserScan` bins by
  `floor((azimuth - azimuthRange[0]) / horizontalResolution)` and labels each
  bin with its lower edge. But the rays fire at exactly integer degrees and the
  bin edges are at exactly integer degrees, so **every ray lands exactly on a
  boundary** and `azimuthErrorStd` picks the side, fresh each revolution.
- So it is **not a constant offset at all**: over 4320 rays, 50.7% are off by
  ~0.0° and 49.3% by a whole ~1.0°, and **0.0% by half a beam**. `scan_test`
  averaging 20 scans per beam is what made a coin flip look like a rotation —
  and why removing 0.544° left the residual 2.5x worse than Gazebo's instead of
  closing it.

### What is left, and it is a decision rather than an experiment

Nothing has been changed. The half beam itself is structural to the node — a
bin labelled with its lower edge is half an increment low for *any* ray
placement, and shifting `azimuthRange` moves labels and edges together.

What *can* go is the coin flip. `startAzimuthOffsetDeg = 0.5` plants each ray
in the middle of its own bin: measured, the error becomes a **constant
+0.4998°** with per-beam scatter down **34x** (0.4888° → 0.0143°), same bias,
no randomness. It should also take out a chunk of the excess `/scan` noise
(`noise_spread_mean` 0.05584 against Gazebo's 0.02991).

Two things to settle before doing it:

1. It belongs in `turtlebot3_isaacsim`, which owns the lidar profile. Putting
   the number on this side would be a second source of truth for the Isaac
   backend.
2. It trades zero-mean angular noise for a constant angular bias. Which of
   those AMCL and slam_toolbox would rather have is a question about the
   consumers, and is worth answering with the three-map comparison rather than
   assumed.

## 7. Friction is manifest data now — the Isaac half is not

Built 2026-09-19. `physics.floor.mu` in `world.yaml` (default
`worlds.FLOOR_MU = 1.0`) is written into the generated `.world` by
`build_world.py` — which authors the ground plane in full rather than
`include`ing Gazebo's, whose undeclared `mu 100 / mu2 50` was the other half of
the problem — and carried to the robot's wheels by `bringup.launch.py` through
the temp-copy of the robot SDF that `with_ground_truth()` already established.
`check_worlds.py` checks the floor half, a launch test checks the wheel half.
The measurement that motivated it, and the four-condition table:
`docs/worknotes/2026-09-19-gazebo-friction.md`.

Same argument as colour, one section up in `tb3_bringup/worlds.py`: `mu: 100000`
is an ODE number only Gazebo can read, exactly as `Gazebo/White` was an Ogre
token only Gazebo could read.

**What is left is a second source of truth, and this repository exists to
prevent those.** `turtlebot3_isaacsim` authors its floor at runtime
(`static_friction=1.0`, `runtime/turtlebot3_isaacsim.py`) and its wheel material
into the asset (`scripts/import_turtlebot3.py`). Both happen to equal the
manifest's 1.0 today, so nothing is inconsistent — and nothing checks it, and
`check_worlds.py` parses no USD, so nothing can from this side. The fix belongs
there: read `physics.floor.mu` from the manifest the launch is already handed.
Until it lands, a change to `FLOOR_MU` here silently moves Gazebo and not Isaac,
which is precisely the failure mode the registry was built to make impossible.

**Not in scope, recorded so it is not reopened.** `slip1`/`slip2` stay `0.0`:
they are ODE's force-dependent slip, a compliance proportional to applied
force, and `0.0` means ordinary Coulomb friction rather than "slipping
disabled". And `caster_back_joint` stays upstream's *ball* joint, so Gazebo's
caster rolls where the real robot and Isaac drag a skid — structural, upstream's
asset, and more useful as a documented limitation that would explain a residual
than as a patch.

## Next steps, in order

*Superseded for the hardware session by
[`docs/experiment-plan.md`](experiment-plan.md), which orders this list against
the two experiments and says which half needs the robot. The reasoning for each
decision stays here.*

1. ~~**chrony on both machines.**~~ **Done, both halves** — the workstation
   2026-09-18, the Pi 2026-09-19. The Pi tracks 10.118.5.241 at **−0.5 ms**, and
   the check that trusts neither daemon is now a ROS one:
   `/scan`'s stamp→receive across the two machines has a **minimum of +3.0 ms
   over 442 messages and no negative values**, which bounds the offset without
   assuming anything about the network. The `date +%s.%N` check this section
   originally specified cannot resolve better than ±rtt/2 ≈ ±20 ms and is
   retired. `docs/experiment-plan.md` B1.
2. **A systemd unit on the Pi** running `robot.launch.py` at boot. Deletes
   HDMI, password and SSH from the workflow: power on, wait, it publishes.
3. ~~**`set_initial_pose` from the manifest**, replacing the `/initialpose`
   publish, on all backends.~~ **Built 2026-09-18** as `pin_initial_pose()` in
   `bringup.launch.py`, and verified on `turtlebot3_world`. NOT with
   `RewrittenYaml`: on Humble it rewrites only paths the source file already
   has, so all four keys were silently dropped and AMCL went on waiting for a
   mouse with no error anywhere — see `docs/experiment-plan.md` B5. It writes a
   temp copy of the params file instead, so `config/nav2_params.yaml` stays
   verbatim upstream. What remains is the floor marker in the real room.
4. ~~**`slam:=true`** wrapping slam_toolbox, the map-saving wrapper that writes
   into the world directory.~~ **Done 2026-09-16**, with `slam` and `nav`
   independent rather than alternative — see section 3. Not yet driven: SLAM
   has been launched on gazebo and produces the right node set, but no map has
   been made end-to-end and `save_map.py` has only been exercised against its
   guards.
5. **Author the lab world** (USD + SDF + manifest, not cloned), and save its
   SLAM map into `worlds/<name>/map/<name>.yaml`.
6. **The first real drive-to-goal**, which is still unrecorded — then the
   three-map comparison from section 3.
7. ~~**Isaac's half-beam lidar rotation** — section 6 above. One experiment
   splits indexing convention from mounting error.~~ **Diagnosed 2026-09-18**:
   nothing is rotated, and the error is 0 or one whole beam per ray rather than
   half of one. What remains is the decision in section 6, not an experiment.

## Not urgent: making the robot's address stop being a variable

Reaching the robot at all is covered in `docs/network.md`. Today its address is
a DHCP lease written into `.env`, and when the lease changes, discovery breaks
until someone notices and edits one line. Two standard ways out, neither
started, recorded here because the terms are not obvious:

- **A DHCP reservation.** The network's DHCP server is told "this particular
  network card always gets this particular address." The robot still asks for
  an address the normal way and nothing on it changes — it just always gets the
  same answer. Needs whoever administers the lab network to add the robot's MAC
  address. This is the clean fix, and it is a request to make rather than code
  to write.
- **mDNS / Avahi.** Lets a machine be reached by name — `turtlebot.local` —
  with no server anywhere deciding addresses; the robot answers for its own
  name on the local network. Usually already installed on Ubuntu. Good for SSH
  immediately. One catch for this repo: Fast DDS initial peers want IP
  addresses, not names, so `TB3_DDS_PEERS=turtlebot.local` would need the
  entrypoint to resolve the name at container start. Small addition, worth it
  only once the name resolves reliably on this network.

Either removes "check the robot's IP" from the workflow. The reservation is
more robust; mDNS needs no one's permission.
