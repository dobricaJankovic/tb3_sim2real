# World registry

One environment definition, both simulators.

Each directory here is a world. `world.yaml` is the **source of truth**; the
Gazebo `.world` and the Isaac Sim `.usd` are both *generated* from it, and the
geometry under `meshes/` is shared byte-for-byte between them. Neither wrapper
is ever hand-authored, so the two backends cannot quietly drift apart.

```
worlds/<name>/
  world.yaml       source of truth      committed
  meshes/          shared geometry      committed
  <name>.world     generated (SDF)      committed  <- Gazebo
  model.config     generated            committed  <- makes model:// resolve
  isaac/           generated, gitignored  <- Isaac Sim
    <name>.usd     the stage tb3_sim.py references
    _converted/    mesh->USD cache
```

The `.world` is committed because it is reviewable text and it means the Gazebo
backend needs no build step. The `.usd` is not, because it is a build product
that can only be produced inside the isaacsim container.

Isaac's output is confined to `isaac/` because that container runs as uid 1234
while your checkout is yours, so its output directory has to be world-writable.
Keeping that to one subdirectory leaves `world.yaml` and `meshes/` at normal
permissions. `scripts/build_world_usd.sh` creates it: git records no directory
modes, so this cannot be fixed by committing anything.

## Using one

```bash
# Gazebo — selected by the ROS-side launch
ros2 launch tb3_bringup bringup.launch.py backend:=gazebo world:=turtlebot3_world

# Isaac Sim — selected where the simulator starts, not by bringup, because it
# runs in its own container and bringup only attaches to it over DDS
WORLD=turtlebot3_world docker compose run --rm isaacsim
ros2 launch tb3_bringup bringup.launch.py backend:=isaacsim
```

`world:=` with `backend:=isaacsim` is refused rather than ignored — accepting it
would imply the ROS side can change what the simulator loaded, and it cannot.

## Adding one

1. `mkdir worlds/my_office && cd worlds/my_office`
2. Drop the geometry in `meshes/` (`.dae`, `.obj`, `.stl` — whatever both
   assimp and Gazebo read; an office scan exported as OBJ is the expected case).
3. Write `world.yaml`:

   ```yaml
   name: my_office            # must equal the directory name
   description: >-
     Ground floor, scanned 2026-09.
   spawn:
     xyz: [0.0, 0.0, 0.01]
     yaw: 0.0
   bodies:
     - name: shell
       geometry: {type: mesh, uri: meshes/office.obj, scale: [1, 1, 1]}
       xyz: [0, 0, 0]
       rpy: [0, 0, 0]
   ```

   Besides `mesh`, the generators understand `cylinder` (`radius`, `length`),
   `box` (`size`) and `sphere` (`radius`).

4. Generate both wrappers:

   ```bash
   scripts/build_world.py my_office                      # -> my_office.world
   scripts/build_world_usd.sh my_office                  # -> isaac/my_office.usd
   ```

5. `world:=my_office` / `WORLD=my_office`. No launch file changes.

## Two things that bite

**Mesh collisions must stay exact.** Every body here is static, so triangle-mesh
collision is legal. A convex hull is not a safe default: the `turtlebot3_world`
wall is a thin hexagonal shell whose hull is a *solid* prism, which seals the
robot inside the arena at spawn. `build_world_usd.py` sets the approximation
explicitly for this reason.

**Collada unit and up-axis are not reliable.** Both meshes here declare
`<unit name="inch" meter="0.0254"/>` and `up_axis Y_UP`, but their vertices are
laid out Z-up (the hexagon lies in XY, extruded along Z). Gazebo renders them
upright, so a converter that *honours* `Y_UP` would tip the Isaac arena on its
side while Gazebo stayed correct — a divergence between backends that no error
message would report. `build_world_usd.py` verifies converted bounds against the
manifest's expectation instead of trusting the converter.
