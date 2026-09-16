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
including `real`; an `initial_pose:=` launch argument overrides it, the way
`map:=` overrides the world's map. A world name stays sufficient to define a
run.

**Build it with Nav2's own parameters**, not a topic publish: AMCL has
`set_initial_pose: true` with `initial_pose.{x,y,yaw}`. The repo currently
publishes `/initialpose`, which races AMCL's activation — that race is exactly
what swallowed the pose during the 2026-09-16 hardware test.

**Physical prerequisite:** a floor marker in the real room at the manifest's
spawn coordinates. Without it the pinned pose is a lie and AMCL starts wrong.

## 2. Clock sync between robot and workstation

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

**To do:** chrony on both, the Pi syncing to the workstation so the two agree
even when the campus network does not. Verify with `chronyc tracking`, or
`date +%s.%N` on both. Do this before trusting any real-robot measurement.

## 3. `slam:=true`, on every backend

Follows `nav2_bringup`'s own shape, where `slam` swaps `localization_launch.py`
for `slam_launch.py`. Here it swaps `map_server` + AMCL for Cartographer, and
relaxes the "this world declares no map" error — the run is about to make one.

`nav:=false` therefore stays. It is not dead weight: mapping is precisely the
mode where `map_server` and AMCL must *not* run, and `drive_test` and teleop
want it too. The only genuinely empty combination is `backend:=real` with
neither nav nor slam, which is worth an explicit error rather than a silent
exit with no output.

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

**Cartographer**, as the TurtleBot3-documented choice. Consequence to remember:
it is not a lifecycle node, so it sits outside Nav2's lifecycle manager rather
than coming up with it.

**Open:** where the map lands. It has to end up at `worlds/<name>/map.yaml` for
the registry to own it, but `map_saver_cli` is a manual step by nature. A thin
wrapper script that saves into the world directory would close the loop from
SLAM to manifest to all three backends.

## 4. URDF ownership — settled, no work

The robot's description is stock `turtlebot3_description`, the same upstream
file this repository loads, so the split recorded in `docs/status.md` costs
nothing in practice.

It *could* be forced the other way — launch `turtlebot3_node` and the lidar
directly on the robot instead of `robot.launch.py`, leaving the container's
`robot_state_publisher` as the only one — but it should not be. Keeping it on
the robot means static transforms do not cross the network, and the robot stays
independently useful. Better arrangement, not a compromise.

## 5. Drop `robot:=local|remote`, keep the concept

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

## Next steps, in order

1. **chrony on both machines.** Cheap, and it removes a whole class of ghost
   failure. Prerequisite for trusting any hardware measurement.
2. **A systemd unit on the Pi** running `robot.launch.py` at boot. Deletes
   HDMI, password and SSH from the workflow: power on, wait, it publishes.
3. **`set_initial_pose` from the manifest**, replacing the `/initialpose`
   publish, on all backends. Plus the floor marker in the real room.
4. **`slam:=true`** wrapping Cartographer, and the error for the
   nav-and-slam-both-false combination.
5. **Author or clone the lab world**, and save its SLAM map into
   `worlds/<name>/map.yaml`.
6. **The first real drive-to-goal**, which is still unrecorded — then the
   three-map comparison from section 3.

Reaching the robot at all is covered in `docs/network.md`. Two further items
from that discussion, neither urgent: resolving hostnames in `TB3_DDS_PEERS`
so mDNS names work in place of a DHCP lease, and a DHCP reservation so the
address stops being a variable.
