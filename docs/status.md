# Status

What is verified, and what is not. Start here before claiming something works.

## Verified 2026-09-15 to 2026-09-19

**Each subsection below carries its own date, and they are not all the same
day.** This heading said "2026-09-15" until 2026-09-18 while findings from the
17th and the 18th were appended under it — which is how the two contradictions
that used to sit in "Known open" survived. When you add a finding here, date it
in its own heading.

### Open-loop kinematics, one instrument, both backends — 2026-09-15

`ros2 run tb3_bringup drive_test`, `world:=empty_stage`, identical `/cmd_vel`
sequence, two runs per backend. Phases are timed on the **simulator's** clock.

| | gazebo | isaacsim |
|---|---|---|
| real-time factor | 0.9997 / 0.9995 | 0.9592 / 0.9517 |
| `straight` 0.15 m/s x 5 s (want 0.750 m) | 0.7358 m, **-1.9%** | 0.7175 / 0.7113 m, **-4.7%** |
| `rotate` 0.5 rad/s x 5 s (want 2.500 rad) | 2.4777 rad, **-0.9%** | 1.7211 / 1.7631 rad, **-30.3%** |
| `arc` displacement (want ~0.50 m) | 0.4466 m | 0.4445 / 0.4459 m |
| `arc` yaw (want 1.500 rad) | 1.4634 rad | 1.0729 / 1.0736 rad |
| drift at rest, before any motion | 2e-05 m, 0 rad | 0 m, 0 rad |
| run-to-run spread | **0.00000 on every metric** | 0.0063 m, 0.042 rad |

- **Neither backend drifts at rest.** `settle_pre` is the only phase never
  preceded by motion, and both are at zero. The nonzero numbers in the other
  stop phases are deceleration coast, not drift: `peak_vx` at the entry to
  `stop_1` is still 0.150 / 0.148 m/s, and both settle below 4e-04 m/s.
- **Gazebo is bit-identical between runs**; Isaac Sim is not, which is expected
  of GPU physics and is why two runs of each were taken. The spread is small
  enough to make the headline differences conclusive — except the arc's
  displacement, where spread and difference are the same size (0.0014 m) and
  the comparison is **inconclusive**.
- **Velocity onset differs.** Gazebo ramps over ~0.10 s; Isaac steps from
  exactly 0 to ~0.147 m/s in one sample. Not a fault, but it means the two do
  not agree on what happens in the first tenth of a second of any command.
- Isaac veers on the straight (-0.0085 / -0.0158 rad of yaw) where Gazebo holds
  0.00000 exactly.

### The one real defect: Isaac Sim's wheels chatter — 2026-09-15, resolved 2026-09-18

**Superseded 2026-09-17.** The measurements below are correct and reproduce;
the diagnosis that followed them was wrong. Full write-up:
[`docs/worknotes/2026-09-17-lane-a-physics.md`](worknotes/2026-09-17-lane-a-physics.md),
which supersedes
[`measurements/isaac_angular_deficit.md`](../measurements/isaac_angular_deficit.md).

The wheels reach only 73% of the commanded joint velocity at `wz = 0.5` and
**48% at `wz = 0.2`**. That was read as a fixed friction torque against a
finite-gain velocity drive. It is not: **the drive damping was swept 10,000x
(1e3 to 1e7) and the error did not move**, so the number
`turtlebot3_isaacsim` marks `# TODO unverified gain` is not wrong, it is
irrelevant.

What those means were hiding is an oscillation. Commanded a steady -1.2121
rad/s the left wheel ranges over **-2.91 to +1.04 rad/s** and reverses
direction; 73% is its average. Gazebo's standard deviation on the same phase is
0.0000. Lift the robot off the ground and Isaac reproduces every commanded rate
**exactly**, so the whole deficit is created by the contact solve — and the
source is the wheel's cylindrical collider, which neither PhysX nor MuJoCo
rolls exactly. A sphere of the same radius cuts the chatter 40x at the original
timestep.

**Shipped: PhysX at 240 Hz** (four sub-steps per rendered frame), measured
2026-09-18 in `measurements/2026-09-18_isaacsim_sweep_240hz.json`. Wheel
tracking against the analytic command, Isaac Sim, same instrument and sequence
throughout:

| command | 60 Hz | **240 Hz** | 480 Hz |
|---|---|---|---|
| `vx = 0.10` | 0.976 / 0.973 | **1.000 / 0.999** | 1.003 / 1.000 |
| `vx = 0.15` | 0.985 / 0.982 | **1.000 / 0.999** | 0.999 / 0.998 |
| `vx = 0.22` | 0.991 / 0.988 | **1.000 / 0.999** | 1.000 / 0.999 |
| `wz = 0.2` | 0.483 / 0.475 | **0.999 / 0.869** | 0.987 / 0.878 |
| `wz = 0.5` | 0.744 / 0.744 | **0.969 / 0.964** | 0.972 / 0.963 |
| `wz = 1.0` | 0.875 / 0.875 | **1.007 / 1.007** | 1.011 / 1.012 |
| `wz = 1.5` | 0.959 / 0.893 | **1.002 / 1.006** | 0.998 / 1.009 |

**240 and 480 Hz are equivalent within run-to-run scatter**, so the cheaper rate
is the default: RTF 0.90 against 0.68. Only the slowest pivot, `wz = 0.2`, still
misses on one wheel.

The residual has moved from the actuator into **slip**. The wheels now turn as
commanded and the robot slides:

| | wheels say | body did | slip |
|---|---|---|---|
| `wz = 0.5` pivot | 2.3485 rad | 2.1826 rad | **−7.1%** |
| `wz = 0.2` pivot | 0.8750 rad | 0.7979 rad | **−8.8%** |
| `vx = 0.15` straight | 0.7365 m | 0.7325 m | −0.5% |

That is not a defect to remove. A burger pivoting on two wheels and a plastic
skid does slide, and Gazebo cannot show it at all — its wheels carry
`mu = 100000`. **The wheel collider is deliberately left as a cylinder** for the
same reason: a sphere contacts at a point, and the expectation is that Isaac is
the backend that resembles the real robot here.

### Isaac Sim's `/odom` is now integrated from the wheels — 2026-09-18

Changed and verified 2026-09-18
(`measurements/2026-09-18_isaacsim_wheel_odom.json`). It used to be the chassis
prim — the true pose — because that is what `IsaacComputeOdometry` reads and
what NVIDIA's reference graph wires up. All three backends now integrate
`/odom` from the wheels, and all three drift.

Measured against `/ground_truth/odom` in the same run:

| | `/odom` (wheel) | ground truth | odometry error |
|---|---|---|---|
| `rotate` `wz = 0.5`, yaw | 2.2501 rad | 2.1464 rad | **+4.83%** |
| `arc`, yaw | 1.2592 rad | 1.1918 rad | **+5.66%** |
| `straight`, distance | 0.7367 m | 0.7361 m | +0.08% |

Odometry over-reports every turn and is nearly exact in a straight line, which
is wheel slip in a pivot — a real phenomenon, previously invisible because
`/odom` could not be wrong. Gazebo cannot reproduce it at all: its wheels carry
`mu = 100000` and its `/odom` comes from the same plugin that drives them.

`/ground_truth/odom` now exists on **both** simulators and on neither hardware.
Its origins differ — Gazebo's P3D is world-absolute, Isaac's is relative to the
spawn pose (verified: spawned at `(-2.0, -0.5)`, reads `(-0.0, -0.0)`) — so
consumers use deltas.

### Isaac Sim's lidar is not rotated: every beam is 0 or 1 beam off — 2026-09-17, diagnosed 2026-09-18

Measured 2026-09-17 with `ros2 run tb3_bringup scan_test`, against ranges
ray-cast from `worlds/small_office/world.yaml` — no reference scan, no
hand-measured room.

| | best-fit angular offset | residual rms |
|---|---|---|
| gazebo | **+0.000°** | 0.00242 m |
| isaacsim | **+0.544°** | 0.00608 m, from 0.01509 |

That 0.544° was read as a half-beam rotation for a day. It is not one.
Diagnosed 2026-09-18 by probing the sensor directly:
`docs/worknotes/2026-09-18-lidar-half-beam.md`.

With the profile's noise off, the azimuth the sensor reports and the azimuth
the geometry implies agree to **0.0000°**. The rays are aimed correctly. They
also fire at exactly integer degrees — and `ROS2PublishLaserScan` bins a return
by `floor((azimuth - azimuthRange[0]) / horizontalResolution)` and then labels
each bin with its **lower edge**, putting its bin boundaries at exactly the
same integer degrees. **Every ray lands exactly on a boundary**, and the
profile's own `azimuthErrorStd` decides which side.

So over 4320 rays, **50.7% are off by ~0.0° and 49.3% by a whole ~1.0°, and
0.0% by anything in between.** Half a beam is the mean of a coin flip, the same
way "under-rotates by 30%" was the mean of a chattering wheel. `scan_test`
averages 20 scans per beam, which is what turned the coin flip into a clean
0.544°; the residual never closing to Gazebo's 0.00242 m is the tell that no
single offset could remove it.

Nothing has been changed. `startAzimuthOffsetDeg = 0.5` in the profile would
make the error a constant +0.4998° instead of a coin flip — 34x less per-beam
scatter, same bias — but the half beam itself is structural to the node, the
profile belongs to `turtlebot3_isaacsim`, and trading angular noise for angular
bias is a question about AMCL and slam_toolbox rather than about the sensor.
See `docs/roadmap.md` section 6.

### Gazebo's `/ground_truth/odom` is real — 2026-09-18

**The one unverified link in the measurement chain, now exercised.** The P3D
plugin in `launch/backends/gazebo.launch.py` had been written and never run,
and its failure mode is silent: a world plugin loaded through the system plugin
loader "is accepted and silently never attaches". It attaches.

`ros2 topic hz`: **49.986 Hz** against the plugin's `update_rate` of 50.0,
`frame_id: world`, `child_frame_id: base_footprint`. A full `drive_test`
(`measurements/2026-09-18_gazebo_ground_truth.json`) reports
`ground_truth: /ground_truth/odom` with a non-null `truth_d` on every phase.

The wheel → body layer on Gazebo, which this makes measurable for the first
time — wheels forward-integrated by exact kinematics against the true body
pose:

| | wheels say | body did | slip |
|---|---|---|---|
| `rotate` `wz = 0.5`, yaw | 2.4781 rad | 2.4797 rad | **+0.06%** |
| `arc`, path length | 0.4927 m | 0.4900 m | **−0.55%** |
| `straight`, distance | 0.7358 m | 0.7370 m | **+0.16%** |

**Gazebo has no measurable slip**, which is what `mu = 100000` on its wheels
predicts, and it is the number Isaac's **−7.1%** pivot slip now has something to
be compared against. Say it as "Gazebo's stock burger has slip disabled", not
as "Gazebo's contact model predicts no slip": 100000 is a sentinel, not a
physical value. `docs/experiment-plan.md` B10.

Note the sign: on Gazebo the body moves very slightly FURTHER than the wheels
in a straight line, where Isaac's body falls short. At 0.16% over 0.74 m that
is 1.2 mm and is sampling, not a phenomenon.

### Isaac Sim's `/scan` is 5.0 Hz, not 3.5 — 2026-09-18

The 3.5 Hz in the 2026-09-14 table below was **measured on the wall clock**.
Measured on the *simulator's*, by `scan_test`, `world:=small_office`,
100 scans each:

| | `/scan`, simulated time | `/scan`, wall clock | real-time factor |
|---|---|---|---|
| gazebo | **5.0000 Hz** | 4.9985 Hz | 0.9997 |
| isaacsim | **5.0000 Hz** | 4.1925 Hz | 0.8385 |

Both exactly the profile's `scanRateBaseHz: 5.0` and the real LDS-01's 5 Hz,
with **zero jitter** — `max(gap) - min(gap)` is 0.00000 s on both, so neither
is a right-on-average rate arriving in bursts. Isaac's implied RTF of 0.8385 is
confirmed independently from `/clock`: 25.417 simulated seconds in 29.971 wall
seconds, **RTF 0.848**.

**So there is no sampling difference to attribute anything to**, and nothing
here belongs in `turtlebot3_isaacsim`. AMCL updates per scan and slam_toolbox
adds a node per scan; both backends deliver the same number of scans per
simulated second over the same trajectory, so a localisation difference between
them is a sensor-model difference. That is what the perception row of
`docs/experiment.md` needed to be true.

Every rate in the 2026-09-14 table has the same wall-clock confound. Re-measured
in simulated time, on the same worlds:

| | gazebo | isaacsim |
|---|---|---|
| `/scan` | 5.000 | 5.000 |
| `/odom` | 29.412 | 60.000 |
| `/joint_states` | 29.412 | 60.000 |
| `/ground_truth/odom` | 50.000 | 60.000 |
| `/clock` | 10 (unchanged) | 60.000 |

Isaac publishes **everything** at exactly its 60 Hz render rate; Gazebo's rates
are per-plugin. `scan_test` now reports `scan_rate_hz` and `wall_rate_hz` side
by side in every run, so this confound cannot come back — and on hardware the
same field is the real robot's own clock, which pairs with B2.

### The two machines' clocks now agree — 2026-09-19

chrony on the Pi, tracking the workstation (which became a stratum-4 server on
2026-09-18). `docs/experiment-plan.md` B1,
`measurements/2026-09-19_clock_sync.json`.

| | 2026-09-16 | 2026-09-19 |
|---|---|---|
| robot syncs to | `10.118.16.1` via timesyncd | **10.118.5.241 via chrony**, `*` in `sources -v` |
| offset | +157 ms | **−0.5 ms** (chrony), **≥ −3.0 ms** (independent) |

**The independent check is a ROS measurement now, not `date` over ssh.** That
comparison has a resolution floor of ±rtt/2 — about ±20 ms here — so it can find
a 157 ms skew but cannot confirm a fix; it read −28 ms before chrony and −20 ms
after, which is the same noise twice. Instead: `/scan`'s `header.stamp` is
written by the Pi and its receive time read on the workstation, and a one-way
delay cannot be negative, so the minimum bounds the offset with no assumption
about the network. Over 90 s and **442 scans: min +3.0 ms, p50 +10.1, p99 +49.2,
max +138.2, and zero negative.** Nothing arrives stamped in the workstation's
future — the condition behind the 2026-09-16 message-filter drops.

**What this turns into, for hardware Nav2 runs:** the risk is no longer skew, it
is the Wi-Fi tail. p99 at 49 ms and a worst-of-442 at 138 ms sit inside Nav2's
usual `transform_tolerance` of 0.2–0.3 s, but not by much. That baseline is the
thing to re-measure if a real run starts dropping scans.

Two traps found on the way, both recorded in `docs/experiment-plan.md` B1:
**`chronyc tracking` is meaningless for its first ten minutes** (one poll in, it
reported a skew of 1000000 ppm while already tracking the right server), and
**`systemctl is-active ufw` says `active` on both machines while ufw itself is
`ENABLED=no` and enforcing nothing** — which is where the docs' "ufw is active,
the rule is mandatory" claim came from. No firewall rule was needed.

### The real robot's interface, measured for the first time — 2026-09-19

The robot was on, its stock `robot.launch.py` running on the Pi, and everything
below was read from the container — which also re-verified the unicast-DDS path
in `docs/network.md`: 13 topics from the robot, on `ROS_DOMAIN_ID=30`, across
two subnets with no multicast. `measurements/2026-09-19_real_interface.json`,
40 scans. `docs/experiment-plan.md` B2 and B3.

| | expected | measured |
|---|---|---|
| lidar | LDS-01, `range_max 3.5` | **3.5**, `hls_lfcd_lds_driver` |
| `/scan` rate | ~5 Hz | **4.986 Hz**, jitter std **0.43 ms** |
| `/odom` rate | ~20 Hz | **19.988 Hz**, jitter std 5.4 ms |
| `/imu`, `/joint_states` | — | 19.996 Hz, ~20 Hz |
| `base_footprint → base_scan` | `(-0.032, 0, 0.182)` | **exactly that**, identity rotation |

`docs/architecture.md` said the real `/scan` came from `ld08_driver`, which is
the LDS-02 driver. **It was wrong**, and is corrected. `ld08_driver` is checked
out in the robot's own `turtlebot3_ws` but is not in the running graph.

**The one-URDF claim is now measured rather than assumed, and it survives a
surprise.** The robot builds `turtlebot3_description` **2.3.7** from source; the
container has apt's **2.3.6**. The versions differ — and the file does not:
`turtlebot3_burger.urdf` is byte-identical, md5
`51b1f9517b2666efefee18310009703d`. Comparing versions, which is what the plan
asked for, would have raised a false alarm; comparing the file is the check that
means something.

**Three things the real lidar does that neither simulator does:**

1. **No-return is `0.0`** — a third encoding, against Isaac's `-1.0` and
   Gazebo's `inf`. Harmless to Nav2 (`0.0 < range_min`, so AMCL maps it to max
   range), and it makes "saw nothing" and "dropped the beam" the same value:
   over 40 stationary scans, 169 of 360 beams were `0.0` throughout and **84
   flickered**.
2. **It sees a metre further than either simulator.** First read as a 0.8% tail
   and written off; re-measured the same day against ~5 m of open space,
   **19.4% of all finite returns exceed the advertised 3.5 m**, out to
   **4.20 m**, and **23 beams returned past 3.5 m on all 120 scans** — the
   farthest stable one at **4.087 m, σ 0.036 m**. Both simulators report
   no-return there, by construction. It reaches AMCL (`laser_max_range: 100.0`)
   and not the costmaps (`obstacle_max_range: 2.5`), so it is a localisation
   difference. **Open decision, `docs/experiment-plan.md` B12** — settle it
   before experiment 2's localisation numbers are interpreted.
3. **`angle_min = 0.0`, `angle_increment = 0.0174533` (exactly 1°).** It starts
   at the front like Gazebo and steps by exactly a degree like Isaac; Gazebo's
   increment is 0.23% wide (upstream writes `<max_angle>6.28</max_angle>`). So
   **neither simulator matches the robot's scan convention exactly**, and each
   differs in a different field.

Items 1 and 3 are not being fixed — reasons in `docs/experiment-plan.md`,
"Deliberately not being fixed". Item 2 is an open decision, not a closed one.

**Clock sync (B1) is still not done and is unaffected by any of this.** The Pi
has no chrony, no RTC, and syncs to `10.118.16.1` while the workstation syncs to
Cloudflare. Workstation − robot measured **−28 ms ± 23 ms**. Nothing above
depends on it: the rates are per-publisher deltas and the geometry is static.
The first thing that *will* depend on it is any tf lookup across the two
machines, which is every Nav2 run.

### Materials, and a fidelity bug in Gazebo too — 2026-09-15

Isaac Sim rendered every stage grey because the manifest's only statement about
colour was `Gazebo/White` — an Ogre script name, meaningful to Gazebo alone.
Colour is now data in the manifest and both generators write their own dialect.

Fixing it surfaced a second bug: upstream's `model.sdf` paints the wall
`Gazebo/FlatBlack` and the five hexagons `Gazebo/Green`, and the manifest had
dropped both, so all 15 bodies defaulted to white. **Gazebo had been rendering
`turtlebot3_world` wrong as well**, not only Isaac.

Verified by rendering both backends headless (`scripts/snapshot.py`,
`scripts/snapshot_gazebo.py`) and by resolving `ComputeBoundMaterial` on every
Gprim: 12/12 on `small_office`, 15/15 on `turtlebot3_world`.

### New in the registry — 2026-09-15

- **`small_office`** — 6.0 x 5.0 m room, five boxes plus a partition, seven mesh
  props from six OBJs (AWS RoboMaker, MIT-0). Builds for both backends; USD
  bounds match the value computed analytically from the OBJ vertices and the
  manifest placements, to four decimals.
- **`isaacsim_bringup` moved to the `IsaacSim-6.1.0` tag**, matching the
  installed simulator. The blocker was fixed at source in `turtlebot3_isaacsim`
  (`AnyLaunchDescriptionSource` + `run_isaacsim.launch.xml`). Pushed; confirmed
  on `origin/humble` 2026-09-18.
- **`scripts/dae_to_obj.py` now applies visual-scene node transforms.** It used
  to warn and skip them, which on `gazebo_models`' `cafe_table.dae` put the
  tabletop flat on the floor while the overall height was only 38 mm out — an
  error `verify` cannot catch. Upstream's two meshes re-convert byte-for-byte
  identical, so `turtlebot3_world` is unaffected.

## Verified on 2026-09-14, after the restructure

One `ros2 launch` per backend, the world chosen by name, both simulated
backends measured from the ROS side in the `tb3_ros` container.

| | gazebo | isaacsim |
|---|---|---|
| `/clock` | 10.0 Hz | 56.1 Hz |
| `/scan` | 5.0 Hz | 3.5 Hz |
| `/odom` | 29.4 Hz | 66.0 Hz |
| `/joint_states` | 29.4 Hz | 56.5 Hz |
| `/cmd_vel` | subscribed | subscribed |
| tf `odom->base_footprint` | yes | yes |

**Every Isaac column in that table is a WALL-CLOCK rate at an RTF below 1.0,
and the sim-time rates are above (2026-09-18).** Read it as "the topics are
live", which is what it was taken to establish, and not as a rate comparison.

- **`backend:=gazebo world:=turtlebot3_world`** — every contract topic live,
  robot at the manifest's spawn: tf reads `[-2.000, -0.500, 0.009]`.
- **`backend:=isaacsim world:=turtlebot3_world`** — Kit started *by the launch
  file*, via `turtlebot3_isaacsim` -> `isaacsim_bringup`. It opened the
  registry's generated stage (`/worlds/turtlebot3_world/isaac/turtlebot3_world.usd`)
  at the manifest's spawn and played. `/scan` returns the arena: real ranges
  from 0.60 m, not an empty plane. Cold start to first `/odom` was about 160 s
  on a cold shader cache.
- **`backend:=gazebo world:=empty_stage`** — comes up, robot at the origin.
- **`backend:=isaacsim world:=empty_stage`** — `artifacts.isaacsim.mode: none`
  correctly results in **no** `--world` on the simulator's command line, so Kit
  authors its own ground plane. Verified on the running process's argv.
- **`scripts/build_world.sh turtlebot3_world`** — regenerated both
  representations, from OBJ meshes (see below). The USD generator's own checks
  pass: 15 collision prims for 15 manifest bodies, all 6 mesh colliders at
  approximation `none`, 0 rigid bodies, and bounds matching the value computed
  analytically from the meshes and placements to four decimals.
  `verify: OK — Isaac stage matches the Gazebo world`.

### Parity, measured from the same meshes

One manifest, one pair of OBJ files, the robot at the manifest's spawn:

| | gazebo | isaacsim |
|---|---|---|
| rays | 360 | 360 |
| with a return | 324 | 307 |
| closest | 0.514 m | 0.521 m |
| furthest | 3.352 m | 3.366 m |

**7 mm apart on the closest return.** The two backends are measuring the same
geometry. For comparison, the same measurement against the *Collada* meshes
before the conversion read 324/360 and 0.511-3.357 m on Gazebo — so the mesh
conversion moved nothing.

The 324-vs-307 gap is the no-return encoding, not the geometry: Isaac reports
`-1.0` where Gazebo reports `inf`. See "Known open".
- **`scripts/check_worlds.py`** — 2/2 worlds consistent. Negative-tested against
  deliberately broken copies: a manifest edited without regenerating, a
  generated `.world` edited by hand with its header intact, and a map mirrored
  about its x axis. All three fail, each naming what is wrong.
- **`scripts/clone_world.py`** — round-tripped: the stock `turtlebot3_world`
  map cloned into a world directory scores 100% of the map modelled and 0% of
  the model absent from the map at the burger's 0.182 m beam height. *The
  measurement stands; the path is **retired** as of 2026-09-16 and is not how
  the lab world gets built — `docs/roadmap.md` §3a, `docs/experiment-plan.md`
  B6. Read this row as "it worked", not as "use it".*
- **All four `nav` / `slam` combinations on gazebo**, measured 2026-09-16 by
  `ros2 node list` 34 s after launch. Neither flag: the backend and
  `robot_state_publisher`, nothing else. `nav:=true`: `amcl`, `map_server` and
  the full navigation stack under two lifecycle managers. `slam:=true`:
  `slam_toolbox` and `map_saver` **and no `amcl` or `map_server`**.
  `slam:=true nav:=true`: `slam_toolbox` plus the whole navigation stack, still
  with no `amcl` or `map_server` — Nav2 planning over a map slam_toolbox is
  drawing. `nav:=true` on a world with no map raises and names the fix;
  `slam:=true` on the same world does not. `backend:=real` with neither flag
  starts RViz and nothing else.
- **`nav:=true` on gazebo** — the whole stack up with **zero errors**: `amcl`,
  `map_server`, `planner_server`, `controller_server`, `behavior_server`,
  `bt_navigator`. `/map` is 384x384 at 0.05 m, served out of
  `worlds/turtlebot3_world/map/` rather than the bringup package.
- **`nav:=true` on isaacsim** — same eight nodes, same map, **zero errors**,
  both lifecycle managers answering `is_active: True`, and publishing one
  `/initialpose` at the manifest's spawn produced `map->odom` at exactly
  `[-2.000, -0.500]`. Measured on a freshly restarted container; see the trap
  below for why that matters.
- **`backend:=real`** — a real TurtleBot3 running its own stock
  `turtlebot3_bringup` on the Wi-Fi subnet, reached from the container across a
  router (2026-09-16, `docs/network.md`). All four robot nodes discovered,
  `/scan` at 5 Hz, `/odom` at 20 Hz, `tf` `odom->base_footprint` resolving.
  Nav2 loads: `map_server`, `amcl` and `controller_server` active and the local
  costmap updating at 1.7 Hz from the real lidar. Not verified past that —
  `planner_server` upward waits on `map->odom`, and the only map to hand was
  `turtlebot3_world`'s, which is not the room the robot is in.
- **`colcon build`** — 3 packages: `isaacsim_bringup`, `turtlebot3_isaacsim`,
  `tb3_bringup`.

### One difference between the backends, not a fault

The `odom` frame does not start in the same place. Gazebo's diff-drive plugin
puts `odom` at the world origin, so tf reads the spawn pose immediately; Isaac
Sim's `wheel_odometry` node starts its integration where the robot is, so tf
reads zero at spawn — as `IsaacComputeOdometry` did before it, so this is
unchanged by the 2026-09-18 odometry work. Both are valid odometry; AMCL
resolves the difference into `map->odom`.
It matters only if you compare raw `/odom` between backends without saying which
frame you mean.

### One trap, and it was mine, not the repository's

An earlier attempt at the Isaac Nav2 run logged

    Failed to change state for node: map_server
    Failed to bring up all requested nodes. Aborting bringup.

and it was tempting to write that up as Kit holding the machine while Nav2
configures. It was not. Leftover `ros2 launch` processes from previous tests
were still on the DDS domain: `pkill -INT -f "ros2 launch"` from inside a
`docker compose exec -T` that has already exited does not reach them, and
`pkill -f gzserver` in a subshell that exited first never runs at all. Two
lifecycle managers then fight over the same nodes, `ros2 node list` prints
every name twice, and a second gzserver dies with exit code 255 and no message.

On a restarted container the same run is clean. **If a backend misbehaves,
check `pgrep -af "ros2 launch|gzserver|isaac-sim"` before believing anything
else** — `docker compose restart tb3_ros` is the reliable reset, and costs a
1.5 s rebuild of `/ws/install`.

## Known open

- **`/scan` encodes no-return three different ways**, one per backend: Isaac
  `-1.0`, Gazebo `inf`, and — measured 2026-09-19 — the real LDS-01 **`0.0`**.
  `-1.0` is not a valid `LaserScan` range, so the backends disagree on the one
  field Nav2's obstacle layer filters on. It has not bitten a Nav2 run, because
  every consumer discards out-of-range values either way. Left alone
  deliberately now that all three are known (`docs/experiment-plan.md`); if
  Isaac's is ever changed it belongs in `turtlebot3_isaacsim`, which owns the
  lidar profile.

  The *geometry* half of this is fixed and this note used to be wrong about it:
  Isaac published 3600 rays against Gazebo's 360 while this repository carried
  its own hand-rolled simulator. The imported package ships a real LDS scan
  pattern (`models/lidar_configs/turtlebot3_lds.json`) and publishes 360.
- **`backend:=real` is verified only as far as Nav2's costmaps.** A goal has
  never been driven on hardware, and localisation on a real map is untested —
  no world in the registry corresponds to the room the robot is in, so an
  authored lab world is the next step (`docs/roadmap.md` 3a).
- **The real backend's URDF is the robot's, not this repository's.** Under
  `backend:=real` the robot's own `turtlebot3_description` publishes
  `/robot_description` and `/tf_static`, so the "one URDF above
  `base_footprint`" property holds between `gazebo` and `isaacsim` by
  construction and across to `real` only by coincidence. Deliberate,
  2026-09-16; the alternative was to stop the robot publishing it and push this
  repository's URDF from the workstation. **Measured 2026-09-19: the
  coincidence holds — the two files are byte-identical** (above). It is still
  two copies, so it can still diverge the day either side is updated; the md5
  is the check, not the version number.
- **Nav2 comes up on both simulated backends; a goal has not been driven since
  the restructure.** Localisation is verified (`map->odom` at the spawn pose on
  both) and every lifecycle node is active, but the last recorded drive-to-goal
  is from before this work (see `docs/history.md`). Nothing in the change
  touches the planner or controller — only where the map comes from and that
  Nav2 now starts on `wait_for_sim`'s exit — but it is worth a run.
- **Per-backend Nav2 tuning is a non-goal, not unstarted work.** There is one
  `config/nav2_params.yaml`, verbatim `turtlebot3_navigation2`'s
  `param/humble/burger.yaml`, used unconditionally. `bringup.launch.py` used to
  auto-prefer `config/nav2_<backend>.yaml` if one appeared; that mechanism was
  removed 2026-09-16 because it made the wrong thing (tuning away the
  sim-to-real gap it exists to measure) convenient rather than impossible. See
  `docs/architecture.md`.
- **The world registry has three worlds.** `turtlebot3_world` (upstream's
  arena, generated from a manifest derived mechanically from
  `turtlebot3_gazebo`'s `model.sdf`), `small_office` and `empty_stage`. None of
  them is the room the robot is actually in; authoring that one is
  `docs/experiment-plan.md` B6. `turtlebot3_isaacsim` carries three more of
  its own — the warehouse, a simple room and a kitchen — as stock Isaac Sim
  environments; adopting them here is an `artifacts: {mode: adopted}` manifest
  each, and needs a Gazebo representation before `world:=` would mean the same
  thing on both.

## What changed on 2026-09-14

The Isaac Sim side stopped being written here. `turtlebot3_isaacsim` is imported
from its own repository rather than copied into this tree, and with it went
`isaac/` — the hand-rolled simulator launcher, the URDF importer, the asset
verifier and the scenes — and `docker/isaacsim-ros2/`, which was a copy of that
package's base image definition, one Isaac Sim version behind.

What replaced the attach-only Isaac backend is the reason it is worth the churn:
`world:=` now means the same environment on all three backends, because the
launch file starts the simulator instead of waiting for someone else to have
started it with the right stage.

`docs/history.md` has the detail, dated.
