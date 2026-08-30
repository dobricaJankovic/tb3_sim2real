# tb3_sim2real

Nav2 on a TurtleBot3, against **three interchangeable backends** — the real
robot, Gazebo Classic, and Isaac Sim — from one launch command.

```bash
ros2 launch tb3_bringup bringup.launch.py backend:=gazebo
ros2 launch tb3_bringup bringup.launch.py backend:=isaacsim
ros2 launch tb3_bringup bringup.launch.py backend:=real
```

`use_sim_time` is derived from `backend`. You should never pass it by hand again.

## Docs

- [`docs/architecture.md`](docs/architecture.md) — the two-container split, repo
  layout, the Nav2 interface contract, and build order.
- [`docs/setup.md`](docs/setup.md) — getting started, where the two Docker
  images come from.
- [`docs/troubleshooting.md`](docs/troubleshooting.md) — failure modes that
  don't throw errors.
- [`docs/status.md`](docs/status.md) — what's verified working vs. still
  unstarted.
