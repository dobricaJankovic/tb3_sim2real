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
                      world; RETIRED, see roadmap 3a -- author a manifest
                      instead), make_map.py (the inverse, for a world that was
                      DESIGNED -- read its docstring before using it),
                      dae_to_obj.py (Isaac Sim reads no Collada),
                      save_map.py (a SLAM map -> worlds/<name>/, the only
                      supported destination),
                      check_worlds.py (the drift check), snapshot.py /
                      snapshot_gazebo.py (render either backend headless),
                      dds_clean.sh (stale /dev/shm Fast DDS segments are what
                      make nodes start, log NOTHING and never join the graph --
                      run it when a launch hangs with no error)
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

**Isaac Sim's wheels chatter against the ground rather than tracking their
command.** What was recorded as "under-rotates by 30%" is the *mean of an
oscillating signal*: commanded a steady -1.2121 rad/s the wheel ranges over
-2.91 to +1.04 and reverses direction. It is not a drive gain — the damping was
swept 10,000x with no measurable effect — it is the contact solve, and the
source is the cylindrical wheel collider, which no solver here rolls exactly.
Running physics at **240 Hz** — four PhysX sub-steps per rendered frame, and
the `turtlebot3_isaacsim` default since 2026-09-18 — takes wheel tracking to
within 1% on every linear command and 96-100% on every angular one but the
slowest. Measured in `docs/worknotes/2026-09-17-lane-a-physics.md` and
`measurements/2026-09-18_isaacsim_sweep_240hz.json`; 240 and 480 Hz are
equivalent within run-to-run scatter, so the cheaper rate is the default.

**The wheel collider is deliberately NOT changed.** Spheres would remove the
chatter entirely, but a sphere contacts at a point and a cylinder is the more
faithful model of a tyre for a robot that pivots on two wheels and a skid — and
the expectation is that Isaac, not Gazebo, is the one that resembles the real
robot here.

What remains is **slip, not actuation**: the wheels turn as commanded and the
robot slides, −7% in a pivot at `wz = 0.5` against −0.5% driving straight. That
is a real phenomenon, it is what a burger on two wheels and a skid actually
does. **Do not compare an Isaac measurement with one taken before 2026-09-17
without checking the physics rate it ran at.**

**"And Gazebo cannot show it at all (`mu = 1e5`)" stood here until 2026-09-19,
and it was false.** Gazebo was *configured* not to, by a number
`turtlebot3_gazebo`'s own `model.sdf` labels `<!-- This friction pamareter
don't contain reliable data!! -->`. Friction is `physics.floor.mu` in the
manifest now (default 1.0, dry rubber on vinyl, an assumption until experiment
1 fits it), written by the generator and carried to the wheel, and Gazebo pivot
slip goes from −0.2% to **−15.7%** at `wz = 1.5`. At `wz = 0.5` it lands on
**−4.66%** against Isaac Sim's −4.61% — an order-of-magnitude disagreement
between two engines closing to 0.05 points when one undeclared default is
replaced by a declared one. Method, the four-condition table and what was
deliberately NOT changed (`slip1/slip2`, the ball-joint caster):
`docs/worknotes/2026-09-19-gazebo-friction.md`. **Every Gazebo measurement
taken before 2026-09-19 was taken at the sentinel** — same trap as the physics
rate, check which side of it a number comes from.

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
- `docs/experiment-plan.md` — **the ordered work list for the two hardware
  experiments**, split into what can be done without the robot and what needs
  it, plus the findings deliberately NOT being fixed. Start here if you are
  picking up a task toward the measurements.
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
