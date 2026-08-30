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
- `backend:=isaacsim` and `backend:=real` are unverified.
- `isaac/scripts/build_scene.py` has plausible but unverified OmniGraph
  node/attribute names, and no lidar yet.
- `isaac/scripts/import_tb3.py` (new) imports the TB3 URDF and assembles
  `isaac/scenes/tb3_world.usd` — run once, verified working:
  `docker exec <tb3_ros> ... xacro ... > isaac/scenes/turtlebot3_burger.urdf`,
  then `docker exec isaacsim /isaac-sim/python.sh /scripts/import_tb3.py`.
  Produces `isaac/scenes/turtlebot3_burger.usd` (asset) and
  `isaac/scenes/tb3_world.usd` (ground plane + light + robot @ `/World/turtlebot3`).
  `build_scene.py`'s CHASSIS_PRIM/LIDAR_PRIM were corrected to match the
  actual import output (see comments there). Wheel joint drive gains
  (`override_joint_damping` in `import_tb3.py`) are an unverified placeholder.
- Isaac Sim's ROS2 bridge needed explicit env (`ROS_DISTRO=jazzy`,
  `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `LD_LIBRARY_PATH=.../ros2.core/jazzy/lib`
  in `docker-compose.yml`) to come up at all — it bundles Jazzy, not Humble.
  Live cross-container DDS node discovery (`RobotDefinitionReader` querying
  `robot_state_publisher`) still failed even with a discovery warm-up delay;
  unresolved, not blocking since `import_tb3.py` uses the offline xacro path
  instead. Worth revisiting before `build_scene.py`'s OmniGraph nodes (which
  DO need the bridge, to publish /clock, /scan, /odom etc.) can be verified.
