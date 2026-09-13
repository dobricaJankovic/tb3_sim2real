# tb3_sim2real

Nav2 on a TurtleBot3, against **three interchangeable backends** — the real
robot, Gazebo Classic, and Isaac Sim — from one launch command.

```bash
ros2 launch tb3_bringup bringup.launch.py backend:=gazebo
ros2 launch tb3_bringup bringup.launch.py backend:=isaacsim
ros2 launch tb3_bringup bringup.launch.py backend:=real
```

`use_sim_time` is derived from `backend`. You should never pass it by hand again.

## Environments

Worlds live in [`worlds/`](worlds/README.md), one directory each. `world.yaml`
is the source of truth; the Gazebo `.world` and the Isaac Sim `.usd` are both
generated from it and the meshes are shared byte-for-byte, so the two backends
cannot drift. Adding an environment is a new directory, not a code change.

```bash
ros2 launch tb3_bringup bringup.launch.py backend:=gazebo world:=turtlebot3_world

# Isaac picks its world where the simulator starts, not on the ROS side:
docker compose exec tb3_ros isaacsim-python /scripts/tb3_sim.py --world turtlebot3_world
```

`bringup.launch.py backend:=isaacsim` only *attaches* to a running simulator, so
the world is chosen where Kit is started. Everything runs in one container —
Isaac Sim, Gazebo, Nav2 and the real-robot drivers — and you open a terminal
into it with `docker compose exec`; see [`docs/setup.md`](docs/setup.md).

## Docs

- [`docs/architecture.md`](docs/architecture.md) — why it is one container,
  repo layout, the Nav2 interface contract, and build order.
- [`docs/setup.md`](docs/setup.md) — getting started, the `up -d` + `exec`
  workflow, and where the two Docker images come from.
- [`docs/troubleshooting.md`](docs/troubleshooting.md) — failure modes that
  don't throw errors.
- [`worlds/README.md`](worlds/README.md) — the world registry: how one manifest
  feeds both simulators, and how to add your own environment.
- [`docs/status.md`](docs/status.md) — what's verified working vs. still
  unstarted.
