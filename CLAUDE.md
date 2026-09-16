# tb3_sim2real

## What this is

Nav2 on a TurtleBot3 against **three interchangeable backends** — the real
robot, Gazebo Classic, Isaac Sim — from one launch command. The point of the
repo is that the three are indistinguishable from the waist up: everything
above `base_footprint` is one URDF and one `robot_state_publisher`, and the
only thing a consumer package ever changes is `use_sim_time`. The same argument
one level up is the world registry: one manifest per environment, both
simulators' representations generated from it.

```bash
ros2 launch tb3_bringup bringup.launch.py \
    backend:={gazebo|isaacsim|real} world:=<name|path> [nav:=true] [slam:=true]
```

`backend` and `world` are required. `nav` and `slam` are independent and both
default false: `slam` chooses where `map -> odom` comes from (slam_toolbox
building a map, or `map_server` + AMCL reading one), `nav` switches on the
navigation stack, and `slam:=true nav:=true` navigates while mapping. There is
no `map:=` — a map lives in `worlds/<name>/map/` and is written there by
`scripts/save_map.py <world>`.

Everything runs in **one container**, `tb3_ros` — Ubuntu 22.04, system ROS 2
Humble, plus Isaac Sim 6.1 at `/isaac-sim` on Kit's own Python 3.12. `docker
compose up -d` then `docker compose exec`.

## Layout

```
tb3_bringup/          the ROS 2 package: launch/bringup.launch.py dispatches on
                      `backend`; launch/backends/ is the only layer that varies;
                      launch/common/ (state_publisher, rviz) is identical
                      everywhere; the Nav2 and slam_toolbox halves are
                      nav2_bringup's own launch files, included directly;
                      tb3_bringup/worlds.py is the registry, read by
                      the launch files AND the generators
worlds/<name>/        world registry. world.yaml is the source of truth; the
                      Gazebo .world and the Isaac .usd are generated from it and
                      share meshes byte-for-byte. Adding a world is a new
                      directory, not a code change.
scripts/              workspace.sh (vcs import), build_images.sh,
                      build_world.py / build_world_usd.py / build_world.sh (the
                      generators), clone_world.py (a real room's Nav2 map -> a
                      world), make_map.py (the inverse, for a world that was
                      DESIGNED -- read its docstring before using it),
                      dae_to_obj.py (Isaac Sim reads no Collada),
                      save_map.py (a SLAM map -> worlds/<name>/, the only
                      supported destination),
                      check_worlds.py (the drift check), snapshot.py /
                      snapshot_gazebo.py (render either backend headless)
measurements/         recorded backend comparisons. `ros2 run tb3_bringup
                      drive_test` is the instrument -- ONE instrument for all
                      three backends, because a per-backend script would be a
                      per-backend result.
docker/               one thin layer over the base image; the base itself is
                      NOT here, see below
src/                  imported source dependencies (gitignored)
```

**The Isaac Sim side is imported, never copied.** `turtlebot3_isaacsim` lives in
its own repository and is a peer of `turtlebot3_gazebo`; it owns the robot
asset, the OmniGraph, the lidar profile, the physics materials, the
occupancy-map builder and the Isaac Sim + ROS 2 base image. `tb3_sim2real.repos`
pins it, `scripts/workspace.sh` fetches it into `src/`, and `backend:=isaacsim`
includes its `isaacsim.launch.py`. Edit it *there*. A copy here would be a
second source of truth for the Isaac backend, which is the thing this repo
exists to prevent — there was one, and it went four worlds and a whole Isaac Sim
version stale before it was removed on 2026-09-14.

`backend:=isaacsim` **starts** the simulator; `world:=` selects the environment
on every backend.

**Colour is manifest data, not a Gazebo script name.** A body's `material:` is
resolved through `worlds.PALETTE` and written by each generator in its own
dialect. `Gazebo/White` alone is an Ogre token that only Gazebo can read, and
while that was all the manifest carried, every Isaac Sim stage rendered grey.

**Isaac Sim currently under-rotates**, by 30% at `wz = 0.5` and 52% at
`wz = 0.2`. It is a wheel-drive gain in `turtlebot3_isaacsim`, diagnosed in
`measurements/isaac_angular_deficit.md`. Do not treat an Isaac angular result as
ground truth until that is fixed.

## Where to look

Read the one doc your task needs, not the set:

- `docs/architecture.md` — why one container, the Python 3.10/3.12 split, the
  Nav2 interface contract, build order.
- `docs/setup.md` — getting started, `up -d` + `exec`, where the images come from.
- `docs/status.md` — what is verified working vs. unstarted. **Start here** if
  you are about to claim something works.
- `worlds/README.md` — the registry AND the environment-cloning methodology:
  real room -> Nav2 map -> manifest -> both simulators, and what stops them
  drifting. This is the architectural core, not a reference appendix.
- `docs/network.md` — reaching the real robot from the workstation when the
  two are on different subnets, and the robot-layer / workstation-layer split
  that goes with it. Read it the first time `ros2 topic list` comes back empty.
- `docs/roadmap.md` — design decisions taken but not yet built, and the
  ordered next steps. Read before starting new work on the real robot.
- `docs/troubleshooting.md` — failure modes that don't throw errors.
- `docs/history.md` — append-only session log. See the working agreement below:
  do not read it for context.

# Working agreements for this repo

- **Search before building.** Before writing anything substantial — a script,
  an OmniGraph, a driver, a workflow — first go looking for what already
  exists: vendor sample assets, the upstream repo's own examples and *tests*,
  and prior art here. Ask the user as well; they often know a reference exists
  that no grep would surface. Trust upstream schema and test files (`.ogn`
  definitions, message definitions, `test_*.py`) over upstream prose docs,
  which go stale. This is not a style preference — it is written down because
  this repo's own Isaac Sim launcher was built from first principles while
  NVIDIA shipped a fully prewired TurtleBot3 with working ROS 2 graphs, and
  because the skill docs consulted on the way named node attributes that do not
  exist.
- **Get the bigger picture before executing.** Push back on a narrow
  instruction until the goal behind it is clear: what it is being built
  toward, what already exists, and how far the change should reach. State the
  plan and let it be corrected *before* starting rather than after. This saves
  time in both directions — it stops work aimed at the wrong target, and it
  forces the target to be made explicit. Asking costs a minute; a wrong
  assumption costs a session.
- **Commit autonomously.** After completing a fix or a meaningful chunk of
  work, create a git commit without waiting for explicit confirmation each
  time. Still use judgment: stop and ask before destructive git operations
  (force-push, `reset --hard`, amending already-pushed commits) or before
  pushing to a remote.
- **History log.** `docs/history.md` is an append-only log of
  problems/fixes and concepts/ideas from work sessions, for the user's own
  reference. When starting a new conversation, do NOT read the whole file —
  it's not needed for context. Only append a new dated entry at the end
  after finishing meaningful work (a bug fixed, a design decision made,
  an idea worth remembering).
