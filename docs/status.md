# Status

`backend:=gazebo nav:=false` verified working end-to-end: `gzserver`,
`gzclient` (GUI over X11), robot spawn, and all interface-contract topics
(`/scan`, `/odom`, `/tf`, `/joint_states`, `/cmd_vel`, `/clock`) confirmed live.

- `config/nav2_*.yaml` are seeded (stock `turtlebot3_navigation2` params,
  identical across backends) — the tuning divergence per backend is still
  unstarted.
- `maps/map.yaml` does not exist yet — `nav:=true` will fail until a real map
  is produced (run SLAM, save it).
- `rviz/tb3.rviz` is still a stub; save a real one out of RViz.
- `backend:=real` is unverified.
- `isaac/scripts/tb3_sim.py` (replaces the old `build_scene.py`) is the Isaac Sim
  *simulator launcher*, not a scene-editing helper: it boots Kit, opens the
  stage, builds the ROS 2 OmniGraph, attaches the lidar and calls `play()`.
  It is the compose default, so `docker compose run --rm isaacsim` starts the
  simulator the way `gazebo.launch.py` starts gzserver/gzclient. Verified live:
  `/clock` 81 Hz, `/odom` 76 Hz, `/tf` 68 Hz, `/joint_states` 60 Hz, all read
  from `tb3_ros` across the container boundary, with `/cmd_vel` subscribed.
- **`/scan` is the one gap.** The topic is advertised and the scan geometry is
  right (1.000 deg/sample over 360 deg, 0.12-3.5 m, matching
  `turtlebot3_gazebo`'s `hls_lfcd_lds` exactly), but no messages are delivered.
  No errors; the RTX plugin logs `Multi-tick is enabled but motion BVH is not
  active. This is not supported.` Under investigation — everything else in the
  interface contract is live.
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
