# Status

`backend:=gazebo nav:=true` verified working end-to-end, autonomous navigation
included: `gzserver`, `gzclient` and RViz all on screen over X11, robot spawn,
all interface-contract topics (`/scan`, `/odom`, `/tf`, `/joint_states`,
`/cmd_vel`, `/clock`) live, AMCL localised and Nav2 driving the robot to a
commanded goal. See `docs/history.md` for the run's numbers.

- `config/nav2_*.yaml` are seeded (stock `turtlebot3_navigation2` params,
  identical across backends) — the tuning divergence per backend is still
  unstarted. Verified byte-identical to
  `turtlebot3_navigation2/param/humble/burger.yaml`, i.e. the Humble-specific
  variant, not the newer one beside it.
- `maps/map.yaml` + `map.pgm` are the stock `turtlebot3_navigation2` map,
  copied in. Its frame coincides with the Gazebo world frame (occupied cells
  centre on the origin), so the initial pose to feed AMCL is the world
  manifest's spawn, `(-2.0, -0.5)` yaw 0 — the same numbers `nav2_bringup`
  uses as `x_pose`/`y_pose`. A SLAM-produced map of our own generated world is
  still worth having; this one is upstream's recording.
- `rviz/tb3.rviz` is the stock `turtlebot3_navigation2` view (Map, LaserScan,
  RobotModel, TF, both costmaps, `/plan`, particle cloud, Navigation 2 panel),
  minus two post-Humble panels this Nav2 cannot load.
- `backend:=real` is unverified.
- `isaac/scripts/tb3_sim.py` (replaces the old `build_scene.py`) is the Isaac Sim
  *simulator launcher*, not a scene-editing helper: it boots Kit, opens the
  stage, builds the ROS 2 OmniGraph, attaches the lidar and calls `play()`.
  Since the 2026-09-13 container merge there is no separate `isaacsim` service:
  run it inside `tb3_ros` with `isaacsim-python /scripts/tb3_sim.py`, which goes
  through `ros-isolate`. Verified live:
  `/clock` 81 Hz, `/odom` 76 Hz, `/tf` 68 Hz, `/joint_states` 60 Hz, all read
  from `tb3_ros` across the container boundary, with `/cmd_vel` subscribed.
- `/cmd_vel` drives the robot for real, not just in principle: commanding
  `linear.x=0.15` moved `/odom` by 0.276 m, which is what 0.15 m/s gives over
  the elapsed sim time at the observed ~0.6 real-time factor. That exercises the
  whole chain — SubscribeTwist -> BreakVector3 -> DifferentialController ->
  ArticulationController -> physics -> IsaacComputeOdometry -> `/odom`.
- **`/scan` publishes but the sensor model is wrong, and it must be replaced
  before Nav2.** Messages are well-formed (frame `base_scan`, `angle_min/max`
  +/-pi, `range_min/max` 0.12/3.5, sim-time stamps) and arrive steadily. Two
  problems, both from using the stock `Example_Rotary_2D` config:
  1. **It points 2 degrees down.** Its single emitter has
     `elevationDeg = [-2.0]`, so it scans the floor rather than the world. On a
     completely empty ground plane it still reports 652 of 3600 rays as hits at
     2.0-3.5 m — a partial arc where the tilted beam meets the ground, spread
     out rather than a clean circle because the robot rests slightly pitched.
     Nav2 would happily fill its costmap with that phantom ring.
  2. **No-return is `-1.0`, not `inf`.** 2948 of 3600 rays come back as `-1.0`,
     which is not a valid LaserScan range. Gazebo publishes `inf`, so the two
     backends disagree on the one field Nav2's obstacle layer filters on.
  Resolution is to author a real LDS scan-pattern config (0 degree elevation,
  360 samples/rev at 5 Hz) instead of re-rating a survey lidar. Note that
  overriding `scanRateBaseHz`/`patternFiringRateHz` on the stock config is *not*
  a workaround — it advertises the right geometry but makes publishing erratic
  and trips `Multi-tick is enabled but motion BVH is not active`.
- **RTX lidar needs a real render path.** With `--headless` the writer attaches
  cleanly and `/scan` is advertised, but not one message is ever produced. Every
  other topic is unaffected, so a headless smoke test will tell you the bridge
  is fine while `/scan` is silently dead.
- `isaac/scripts/import_tb3.py` imports the TB3 URDF and assembles
  `isaac/scenes/tb3_world.usd`. Three steps, all verified working:
  `docker exec <tb3_ros> ... xacro ... > isaac/scenes/turtlebot3_burger.urdf`,
  `docker cp <tb3_ros>:/opt/ros/humble/share/turtlebot3_description
  isaac/scenes/turtlebot3_description` (gitignored, 40 MB — the importer needs
  it to resolve the URDF's `package://` mesh URLs), then `docker compose run
  --rm --entrypoint /isaac-sim/python.sh isaacsim /scripts/import_tb3.py`.
  Produces `isaac/scenes/turtlebot3_burger.usd` (asset) and
  `isaac/scenes/tb3_world.usd` (ground plane + light + robot @ `/World/turtlebot3`).
  `build_scene.py`'s CHASSIS_PRIM/LIDAR_PRIM were corrected to match the
  actual import output (see comments there). Wheel joint drive gains
  (`override_joint_damping` in `import_tb3.py`) are an unverified placeholder.
  Re-running it is now idempotent — the importer does not overwrite, so the
  script clears the asset directory first.
- **Surface properties are authored here, not inherited.** Neither the URDF nor
  the importer supplies friction or restitution, and `GroundPlane` defaults its
  own material to restitution 0.8, so the robot used to rock on its caster and
  creep across the floor with nothing commanding it (3.58 deg peak-to-peak in
  pitch, 7.7 mm drift in 7 s). `bind_robot_surfaces()` binds a floor/wheel
  (mu 1.0) and chassis+caster (mu 0.1) material, restitution 0 throughout,
  combined with `min` rather than the default average. The same measurement now
  reads 0.000 deg and 0.00 mm. `tb3_sim.py`'s `check_surfaces()` refuses to
  start on a stage that has lost them, since the symptom otherwise looks like a
  physics-tuning problem rather than a stale asset.
- `isaac/scripts/verify_asset.py` (new) gates the above: it composes a stage and
  fails if the robot subtree has no renderable geometry. Current output for
  `tb3_world.usd` is 4 robot meshes (burger_base 48040 pts, lds 7231, each tire
  10812) bounding to 138 x 178 x 191 mm, which matches the physical burger.
  Run it after every re-import — the failure it catches is completely silent.
- Isaac Sim's ROS2 bridge needed explicit env (`ROS_DISTRO=jazzy`,
  `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `LD_LIBRARY_PATH=.../ros2.core/jazzy/lib`
  in `docker-compose.yml`) to come up at all — it bundles Jazzy, not Humble.
  Live cross-container DDS node discovery (`RobotDefinitionReader` querying
  `robot_state_publisher`) still failed even with a discovery warm-up delay;
  unresolved, not blocking since `import_tb3.py` uses the offline xacro path
  instead. Worth revisiting before `build_scene.py`'s OmniGraph nodes (which
  DO need the bridge, to publish /clock, /scan, /odom etc.) can be verified.

## World registry (`worlds/`)

One manifest per environment drives both simulators; the Gazebo `.world` and
the Isaac Sim `.usd` are generated from it and the meshes are shared
byte-for-byte. `bringup.launch.py world:=<name>` selects it for Gazebo,
`WORLD=<name>` for Isaac Sim. Adding an environment is a directory, not a code
change. See `worlds/README.md`.

`turtlebot3_world` verified working in **both** backends from the same manifest:

- **Gazebo** — `get_model_list` reports `ground_plane, turtlebot3_world,
  burger`; shared meshes resolve through `model://` off `/worlds`;
  `/scan` returns 324 of 360 rays finite, 0.511-3.357 m.
- **Isaac Sim** — stage referenced at `/World/env`, robot spawned at the
  manifest's `(-2.0, -0.5)`, `/scan` returns 3372 of 3600 rays positive,
  0.526-3.497 m. Previously, on the bare ground plane, only 652 of 3600 were
  hits and all of those were the phantom tilt ring.
- **Parity evidence**: closest return 0.511 m (Gazebo) vs 0.526 m (Isaac) from
  the same spawn pose — 1.5 cm apart, i.e. the two backends are measuring the
  same geometry. Generation is idempotent (regenerating matches the committed
  output byte-for-byte).
- The generator self-checks: 15 collision prims for 15 manifest bodies, all 6
  mesh colliders at approximation `none` (exact), 0 rigid bodies, and world
  bounds within 2 cm of the value computed analytically from the meshes and
  placements. Those checks caught two real bugs on first run, both of which
  produce a valid-looking stage and no error — see `docs/history.md`.

Still open, and unchanged by this work:

- The `/scan` differences between backends are the **known lidar problem**, not
  a world problem: Isaac publishes 3600 rays where Gazebo publishes 360, and
  reports no-return as `-1.0` (228 rays) where Gazebo uses `inf` (36 rays).
  Authoring a real LDS scan pattern is still the fix.
- `nav:=true` still requires a map that does not exist yet.

## One container (2026-09-13)

Isaac Sim and ROS 2 Humble now run in a single Ubuntu 22.04 image. The
September conclusion that this was impossible was measured on a locally built
Isaac Sim; NVIDIA's release is compiled to `GLIBC_2.34` and runs on jammy. See
`docs/architecture.md` and the `docs/history.md` entries for 2026-09-13.

Verified on branch `isaacsim-humble-single`:

- `docker/isaacsim-ros2/verify.sh isaacsim6-humble:ngc` — **5/5**, including
  Kit starting and the bridge publishing `/clock` to the system Humble in the
  same container.
- `tb3_sim.py` unmodified in the merged image, headless: `/scan` 6.03 Hz,
  `/odom` 34.1 Hz, `/joint_states` 38.5 Hz, `/clock` 37.8 Hz, tf
  `odom->base_footprint`, `/cmd_vel` subscribed, no errors.
- Cache volumes: `/scan` ready 164-168 s cold, **14 s warm**.

### Check these before writing `turtlebot3_isaacsim`

Ordered by how much they would change the package if they fail.

1. **`bringup.launch.py backend:=isaacsim` still works in one container.**
   Asserted in commit `776bf55`, not tested. Attaching over DDS should be
   unaffected by both endpoints sharing a container, but "should" is what
   produced the 3/5 conclusion in September. This is the property that makes
   the merge safely revertible, so it goes first.
2. **The Isaac Sim GUI has never been run in the merged image.** Every test so
   far was `--headless`. The whole point of the change is that you launch
   bringup and *watch* the simulator come up, so X11 into this image — one
   cookie at `/root/.Xauthority` now, not two — is load-bearing and unproven.
   Run `./docker/x11-auth.sh` first.
3. **`backend:=gazebo` in the merged image.** Gazebo Classic and Isaac Sim now
   share one container, one GPU and one X display; nothing has asked them to
   before. Gazebo was working end-to-end before the merge, so a regression here
   is a merge problem, not a Gazebo problem.
4. **Shared-memory DDS.** Drop `FASTRTPS_DEFAULT_PROFILES_FILE` and the
   `docker/fastdds_udp_only.xml` mount and confirm data still flows. The reason
   for UDP-only — Fast-DDS's 0700 segments across uid 1234 and root — is gone
   with the merge. Its own commit, so a failure is unambiguous.
5. **Decide `exec` vs `run --rm` as the documented path.** The warm cache lives
   in named volumes, so `docker compose run --rm tb3_ros` gets the same 14 s
   start *and* runs the entrypoint, avoiding the silent-failure trap where
   `exec` leaves no `ros2` on `PATH` in a non-interactive shell. `up -d` +
   `exec` only buys attached shells.

### Then the package

`turtlebot3_isaacsim`, mirroring `turtlebot3_gazebo` file for file, built on
NVIDIA's `isaacsim_bringup` (`IsaacSim-ros_workspaces/humble_ws/src/`), which is
already "a `ros2 launch` that starts Isaac Sim" and takes
`standalone:=<script.py>`. It reads the world registry rather than carrying its
own worlds, which is what finally makes `world:=` a real launch argument for the
Isaac backend.

Decided 2026-09-13: the package stands alone, and a convenience layer that
imports it alongside `turtlebot3_gazebo` comes later. Whether `tb3_bringup`
becomes that layer or is replaced by it is deliberately left open.

Design note: <https://claude.ai/code/artifact/e83a0463-ff46-4f38-b227-c025cd4b5a7e>
