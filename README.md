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
WORLD=turtlebot3_world docker compose run --rm isaacsim   # Isaac picks it here
```

Isaac Sim runs in its own container, so its world is selected where the
simulator starts rather than by `bringup.launch.py`, which only attaches to it.

## Docs

- [`docs/architecture.md`](docs/architecture.md) — the two-container split, repo
  layout, the Nav2 interface contract, and build order.
- [`docs/setup.md`](docs/setup.md) — getting started, where the two Docker
  images come from.
- [`docs/troubleshooting.md`](docs/troubleshooting.md) — failure modes that
  don't throw errors.
- [`worlds/README.md`](worlds/README.md) — the world registry: how one manifest
  feeds both simulators, and how to add your own environment.
- [`docs/status.md`](docs/status.md) — what's verified working vs. still
  unstarted.
