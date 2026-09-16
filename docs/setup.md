# Getting started

```bash
scripts/workspace.sh                   # source deps into src/ -- do this FIRST
./docker/x11-auth.sh                   # once per X session, before any GUI
scripts/build_images.sh                # base image, then tb3_ros (see below)
docker compose up -d                   # one container; it stays up
docker compose exec tb3_ros bash       # terminal 1

# inside tb3_ros:
colcon build --symlink-install && source install/setup.bash
ros2 launch tb3_bringup bringup.launch.py backend:=gazebo   world:=turtlebot3_world nav:=true
ros2 launch tb3_bringup bringup.launch.py backend:=isaacsim world:=turtlebot3_world nav:=true
```

`scripts/workspace.sh` first, and not only because nothing builds without it:
the `src/` entries in `docker-compose.yml` are bind mounts, and Docker creates
a missing host path as an empty root-owned directory rather than failing. Skip
it and the container comes up with an empty workspace for a reason nothing
reports. It needs an SSH key with access to the `turtlebot3_isaacsim`
repository; run it inside the container if your host has no `vcstool`.

**One command starts Isaac Sim.** `backend:=isaacsim` includes
`turtlebot3_isaacsim`, which includes NVIDIA's `run_isaacsim.launch.py`, so Kit
comes up behind the same `ros2 launch` as everything else — there is no second
terminal and no separate simulator to start first. Nav2, when `nav:=true`, is
held back by an event handler until `/clock` appears, which on a cold shader
cache is two to three minutes.

`headless:=true` runs it with no window. `isaacsim` (GUI) and
`isaacsim-python <script>` also exist in the container for one-off work; both go
through `ros-isolate`, the only correct way to start Kit from a shell that has
sourced ROS 2.

## `up -d` + `exec`, not `run --rm`

Decided 2026-09-13, after measuring both. `docker compose run --rm tb3_ros`
looks attractive — it runs the entrypoint, and the warm Kit cache lives in named
volumes, so it starts just as fast (1.2 s) — but it creates a **new container**
every time, and

- `/ws/install` is in the container's writable layer, not a volume. A fresh
  container has `/ws/src` and nothing else, so `ros2 launch tb3_bringup ...`
  fails with `Package 'tb3_bringup' not found` until you `colcon build` again —
  in every terminal, because every terminal is a different container.
- The simulator is a long-lived process you attach terminals *to*. `exec` joins
  the container running it; `run --rm` starts a stranger beside it.

So the container stays up and you `exec` into it. The same reasoning says
`docker compose up -d` is not free either: **recreating** the container (a
compose file edit, `up -d --force-recreate`) also destroys `/ws/install`, and
the first launch afterwards fails the same way. Rebuild is 1.5 s.

### The one trap

`exec` does **not** run `/entrypoint.sh`; only `run` does. An interactive shell
gets away with it because `/root/.bashrc` sources ROS 2, but Ubuntu's stock root
`.bashrc` returns early for non-interactive shells, so

```bash
docker compose exec tb3_ros bash -c 'ros2 topic list'    # bash: ros2: not found
```

lands in a container with no `ros2` on `PATH` — and the failure is quiet enough
to corrupt a measurement rather than stop it. Anything scripted goes through the
entrypoint explicitly:

```bash
docker compose exec tb3_ros /entrypoint.sh bash -c 'ros2 topic list'
```

Two more things that bite scripts: a killed `docker compose exec -T` kills the
*client*, leaving its `ros2 launch` running inside the container (clean up with
`docker compose exec -T tb3_ros pkill -INT -f "ros2 launch"`), and `ros2 topic
hz` on a simulated topic needs `--use-sim-time`.

## Environments

Worlds come from the registry in `worlds/`, not from `turtlebot3_gazebo`'s
installed worlds. `worlds/<name>/world.yaml` is the source of truth and both
simulators' representations are generated from it — see `worlds/README.md`,
which is also where the real-environment cloning workflow lives.

The Gazebo `.world` is committed, so `backend:=gazebo` needs no build step. The
Isaac Sim `.usd` is a build product and is gitignored, so it has to be built
once per clone, per world:

```bash
scripts/build_world.sh turtlebot3_world   # ~1 min; builds BOTH
```

One command builds both on purpose: regenerating one without the other is the
single way the two backends can come to describe different rooms. Skipping it
does not fail quietly either — the launch names this command rather than letting
gzserver come up with an empty scene.

```bash
scripts/check_worlds.py                   # stdlib only, under a second
```
A world that was designed rather than cloned from a real room has no recorded
map, so `nav:=true` has nothing to localise against; `scripts/make_map.py <name>`
writes one from the manifest. Read the warning at the top of that script first —
it is the narrow case, not the habit.

To see a stage rather than measure it:

```bash
docker compose run --rm tb3_ros isaacsim-python /repo/scripts/snapshot.py \
    --world small_office
```

which renders it headless to `worlds/<name>/isaac/snapshot.png`. That exists
because the last bug in this area was a stage that loaded, collided and measured
correctly and rendered entirely grey.


verifies that every world's generated artifacts still came from its manifest,
that nobody hand-edited one, and that the model still matches the map recorded
in the real room. Worth a pre-commit hook.

## X11 for the GUI

The container does not run as *you*, but the host X server's access control is
per-user, so a plain `DISPLAY` + `/tmp/.X11-unix` mount isn't enough —
`gzclient`/RViz/Kit abort with `Authorization required, but no authorization
protocol specified`. `docker/x11-auth.sh` writes a docker-specific Xauthority
cookie that compose mounts at `/root/.Xauthority`. Run it once per X session
(log in / restart X) before bringing the container up — the cookie rotates on
login, so a stale one fails the same way.

There is **one** cookie path now. There were two while Isaac Sim had its own
container, because NVIDIA's image ends with `USER isaac-sim` (uid 1234) and
could not traverse `/root`; one image, one uid, one path.

`x11-auth.sh` rewrites the cookie by rename, so the file has a new inode and a
container started earlier still holds the old one through its bind mount. After
re-keying, `docker compose restart tb3_ros` before expecting a window.

Getting X wrong fails *silently* on the Isaac side: Kit logs `GLFW
initialization failed` and `IAppWindow::startup failed`, then runs on with no
window while publishing ROS topics normally. If you think Isaac Sim is up but
see no window, check the Kit log under `/isaac-sim/kit/logs/` (a named volume;
it outlives the container) before anything else.

See `docs/troubleshooting.md` for the full explanation.

## Where the images come from

Two images, built in order by `scripts/build_images.sh`, because the second is
`FROM` the first:

```bash
scripts/build_images.sh                # both
scripts/build_images.sh --base-only    # just the base, e.g. before verify.sh
```

- `isaacsim61-humble:ngc` — built `FROM nvcr.io/nvidia/isaac-sim:6.1.0`. Isaac
  Sim 6.1 + ROS 2 Humble on Ubuntu 22.04, generic: no TurtleBot3, no X11, no
  workspace. **Its Dockerfile is not in this repository.** It belongs to
  `turtlebot3_isaacsim`, which is imported here rather than copied, and its
  container is part of what is imported — a second copy of it here was the
  stale one, still on 6.0.1, when this was written. Check it on its own with
  `src/turtlebot3_isaacsim/docker/isaacsim-ros2/verify.sh isaacsim61-humble:ngc`.
- `tb3_sim2real/tb3_ros:humble` — `docker/Dockerfile`, one thin layer on top:
  Gazebo Classic's TurtleBot3 worlds, the real robot's drivers, cartographer,
  and the workspace. This is the one you will rebuild.

Pulling the NGC image needs `docker login nvcr.io` **and** a one-time licence
acceptance in a browser for the same NGC org; the repository is not anonymously
pullable and will hand out a token and then 401 on the manifest.

Nothing here is built from the Isaac Sim source tree any more. A locally built
Isaac Sim links against the build host's glibc — on a noble host that means
`GLIBC_2.38`, which jammy cannot provide, and that single fact is what made this
project believe for a week that Isaac Sim and Humble could not share an image.
The released binaries are built to `GLIBC_2.34`. See `docs/architecture.md`.
