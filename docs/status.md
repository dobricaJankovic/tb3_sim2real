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
- `isaac/scripts/build_scene.py` has plausible but unverified node/attribute
  names, and no lidar yet.
- No TB3 USD exists.
