# tb3_sim2real

Nav2 on a TurtleBot3, against **three interchangeable backends** — the real
robot, Gazebo Classic, and Isaac Sim — from one launch command.

```bash
ros2 launch tb3_bringup bringup.launch.py backend:=gazebo
ros2 launch tb3_bringup bringup.launch.py backend:=isaacsim
ros2 launch tb3_bringup bringup.launch.py backend:=real
```

`use_sim_time` is derived from `backend`. You should never pass it by hand
again. Everything above `base_footprint` is one URDF and one
`robot_state_publisher` in all three cases, so the three are indistinguishable
from the waist up and each backend supplies only a raw `odom -> base_footprint`
plus sensor topics.

## Environments

```bash
ros2 launch tb3_bringup bringup.launch.py backend:=isaacsim world:=turtlebot3_world
ros2 launch tb3_bringup bringup.launch.py backend:=gazebo   world:=~/worlds/my_office
```

`world:=` names an **environment**, not a file, and means the same thing on
every backend: gzserver loads its `.world`, Kit opens its `.usd`, and the real
robot loads nothing but takes the same spawn pose and the same Nav2 map. It
takes a registry name or a path, so your own environment needs no entry here.

Three come with the repository: `turtlebot3_world` (ROBOTIS' arena),
`empty_stage`, and `small_office` — a 6 x 5 m room written as six boxes and
furnished with meshes, because a chair back is curved and its convex hull is a
solid wedge.

Worlds live in [`worlds/`](worlds/README.md), one directory each.
`world.yaml` is the source of truth; both simulators' representations are
generated from it and the meshes are shared byte-for-byte, so the two backends
cannot drift. `scripts/check_worlds.py` proves it, and fails when they do.

### Cloning your real room

The methodology, in full, is in [`worlds/README.md`](worlds/README.md). In
short: the occupancy map you already recorded for Nav2 *is* the reference.

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/lab_room   # in the real room
scripts/clone_world.py --map ~/maps/lab_room.yaml --name lab_room
scripts/build_world.sh lab_room
ros2 launch tb3_bringup bringup.launch.py backend:=isaacsim world:=lab_room
```

## Layout

```
tb3_bringup/       the ROS 2 package: bringup.launch.py dispatches on `backend`
worlds/            the world registry; world.yaml is the source of truth
scripts/           the generators, the cloner, the drift check, the images
docker/            one thin layer over the Isaac Sim + ROS 2 base image
src/               source dependencies, imported by scripts/workspace.sh
measurements/      recorded backend comparisons (see below)
```

## Comparing the backends

The claim that three backends are interchangeable is only worth something if it
is measured, and it has to be measured with **one** instrument — a per-backend
script would be a per-backend result.

```bash
ros2 run tb3_bringup drive_test --ros-args -p label:=gazebo -p out:=/tmp/g.json
ros2 run tb3_bringup drive_test --ros-args -p sequence:=collide -p label:=isaacsim -p out:=/tmp/i.json
```

It publishes an identical open-loop `/cmd_vel` sequence and records `/odom`,
both of which every backend is contracted to provide — so it runs unchanged on
`real` as well. Open loop on purpose: Nav2 would correct exactly the errors
being measured. `sequence:=collide` drives into a wall and keeps driving, which
is how you find out whether odometry integrates distance the robot never
travelled.

The Isaac Sim backend is [`turtlebot3_isaacsim`][tb3i], a peer of
`turtlebot3_gazebo` developed in its own repository and **imported** here, never
copied — a second copy of it would be a second source of truth for the Isaac
side, which is the one thing this repository exists to prevent. It is pinned in
`tb3_sim2real.repos`, along with NVIDIA's `isaacsim_bringup`.

[tb3i]: https://github.com/dobricaJankovic/turtlebot3_isaacsim

Everything runs in one container — Isaac Sim 6.1, Gazebo Classic, Nav2 and the
real-robot drivers — and you open a terminal into it with `docker compose exec`.

```bash
scripts/workspace.sh        # fetch the source dependencies into src/
scripts/build_images.sh     # base image, then this one
./docker/x11-auth.sh        # once per X session, before any GUI
docker compose up -d
docker compose exec tb3_ros bash
```

## Docs

- [`worlds/README.md`](worlds/README.md) — the registry, how one manifest feeds
  both simulators, and **how to clone a real environment**.
- [`docs/architecture.md`](docs/architecture.md) — why it is one container, what
  is imported rather than written here, the Nav2 interface contract.
- [`docs/setup.md`](docs/setup.md) — getting started, `up -d` + `exec`, where
  the images come from.
- [`docs/troubleshooting.md`](docs/troubleshooting.md) — failure modes that
  don't throw errors.
- [`docs/status.md`](docs/status.md) — what's verified working vs. unstarted.
