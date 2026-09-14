# The world registry

One environment definition, three backends.

Each directory here is a world. `world.yaml` is the **source of truth**; the
Gazebo `.world` and the Isaac Sim `.usd` are *generated* from it, the geometry
under `meshes/` is shared byte-for-byte between them, and the Nav2 map under
`map/` is the same file for all three backends. Neither simulator's wrapper is
ever hand-authored, so the two cannot quietly drift apart.

```
worlds/<name>/
  world.yaml       source of truth        committed
  meshes/          shared geometry        committed
  map/             Nav2 map               committed  <- all three backends
  <name>.world     generated (SDF)        committed  <- Gazebo
  model.config     generated              committed  <- makes model:// resolve
  isaac/           generated, gitignored              <- Isaac Sim
    <name>.usd     the stage Kit opens
    _converted/    mesh->USD cache
```

The `.world` is committed because it is reviewable text and it means the Gazebo
backend needs no build step. The `.usd` is not, because it is a binary build
product that can only be produced inside the container.

Isaac's output is confined to `isaac/` because the container runs as root while
your checkout is yours. Keeping its writes to one subdirectory leaves
`world.yaml`, `meshes/` and `map/` at normal permissions.
`scripts/build_world.sh` creates it on the host first, for the same reason.

## What is here

| world | what it is | geometry |
|---|---|---|
| `turtlebot3_world` | ROBOTIS' standard arena, derived from upstream's `model.sdf` | 9 cylinders + 6 meshes |
| `empty_stage` | ground plane and a light, nothing else | none |
| `small_office` | a 6 x 5 m room with a partition and furniture | 5 boxes + 7 meshes |

`small_office` is the one to read if you are adding your own: the room is boxes
you could take off a tape measure, the furniture is meshes because it has to be,
and the colour is manifest data because both renderers need it.

## Using one

```bash
ros2 launch tb3_bringup bringup.launch.py backend:=gazebo   world:=turtlebot3_world
ros2 launch tb3_bringup bringup.launch.py backend:=isaacsim world:=turtlebot3_world
ros2 launch tb3_bringup bringup.launch.py backend:=real     world:=turtlebot3_world
```

`world:=` names the **environment**, not a file, and means the same thing on
every backend. gzserver loads its `.world`, Kit opens its `.usd`, and the real
robot loads nothing — the environment is already around it — but all three take
the spawn pose and the Nav2 map from the same manifest, which is what makes a
run on one comparable with a run on another.

It also takes a **path**, so an environment of your own needs no entry here:

```bash
ros2 launch tb3_bringup bringup.launch.py backend:=gazebo world:=~/worlds/my_office
```

A whole registry of your own works too — point `TB3_WORLDS` at it before
`docker compose up` and `world:=` resolves names against yours instead.

---

# Cloning a real environment

The hard part of sim-to-real is not the launch files. It is getting the room the
robot is actually in into two simulators, and keeping it there.

```
                       the real room
                             |
                 drive it, record a Nav2 map
                             |
                      world.yaml + meshes/          <- the ONLY thing you edit
                       /               \
             build_world.py       build_world_usd.py
                     /                   \
            <name>.world            isaac/<name>.usd
                (Gazebo)                (Isaac Sim)
```

## Why the map

A normal TurtleBot3 user already has exactly one artefact that describes their
real environment metrically: the occupancy map they had to record for Nav2. It
is in the frame Nav2 works in, it is at the resolution the planner uses, and it
is a record of precisely the geometry the robot's own lidar can see. It costs
nothing extra, because you need it anyway.

```bash
# in the real room
ros2 launch turtlebot3_cartographer cartographer.launch.py
ros2 run nav2_map_server map_saver_cli -f ~/maps/lab_room

# then, once
scripts/clone_world.py --map ~/maps/lab_room.yaml --name lab_room --spawn 0 0
scripts/build_world.sh lab_room
```

That writes `worlds/lab_room/` — the occupied cells extruded into one OBJ, a
manifest whose single body is that mesh, and the map copied in beside it — and
then generates both simulators' representations from that manifest. The room is
now runnable on all three backends, from one `world:=lab_room`.

**What this captures is a floor plan, not a model of the room.** A horizontal
lidar sees one plane. A table top, an overhang, a step down, a glass wall: none
of them are in the map, so none of them are in the clone. Measure those and add
them as extra bodies in `world.yaml` — the manifest shape is the same whether a
body came from the map or from a tape measure, and both simulators get it.

## Why not a 3D scan

Photogrammetry or a phone's LiDAR gives a two-million-triangle non-manifold
shell. Neither ODE nor PhysX will collide with that usefully, so it has to be
decimated and its normals fixed in Blender first — which is an afternoon, a
skill, and a second authoring step that then has nothing to keep it in agreement
with anything. If you have a scan and want it, add it as an extra *visual* body
and let the extruded geometry carry collision.

## Why one mesh and not boxes

Rectangle-decomposing an occupancy grid into `box` bodies is the obvious
alternative and it is worse: a real room comes out as several hundred boxes, and
two simulators each approximating several hundred boxes is more surface to
disagree on, not less. One mesh is loaded by both of them from the same file.

## Colour is manifest data

A body's `material:` is read by both generators and written in each one's own
dialect — SDF `<ambient>`/`<diffuse>`/`<specular>` for Gazebo, a bound
`UsdPreviewSurface` for Isaac Sim.

```yaml
material: Gazebo/Green                                  # a stock Gazebo name
material: {color: [0.45, 0.30, 0.17], roughness: 0.6}   # or the numbers
material: asset                                         # or the mesh's own .mtl
```

It did not always work this way, and the failure is worth recording. The
manifest used to carry only a Gazebo material *script name*, which the SDF
generator pasted into a `<script>` block. That is an Ogre token: it means
something to Gazebo and nothing to anything else, so **every Isaac Sim stage
rendered grey** while the Gazebo one looked correct — two representations that
disagreed about something no check was looking at.

So the names are now resolved to RGB in `worlds.PALETTE`, using the values in
Gazebo 11's own `media/materials/scripts/gazebo.material`, and both backends
read the result. A stock name still works, because upstream SDF is full of them;
it just means the same thing on both sides now.

`asset` — the default for a mesh — keeps whatever the geometry file brought, so
a prop exported with an `.mtl` is not clobbered by a manifest that says nothing
about it. An explicit `material:` always wins; on the USD side it binds
`strongerThanDescendants` for exactly that reason.

`build_world_usd.py` counts the prims that actually resolve to a bound surface
and reports it, because a stage that renders grey loads, collides and measures
perfectly and fails nothing else. On the Gazebo side a colour edited into the
`.world` by hand is caught by the provenance digest like any other edit —
`check_worlds.py`'s round-trip compares name, pose and geometry, not material.

## Props you cannot make from boxes

A room is boxes. A chair is not: it has a curved back, and its convex hull is a
solid wedge that fills the space a robot could drive under. Same for a round bin
rim, a tapered table leg, a shelf's open bays.

`worlds/small_office` is the worked example — walls and a partition written by
hand as six boxes, and six props as meshes:

```yaml
  - name: chair_north
    geometry: {type: mesh, uri: meshes/chair.obj, scale: [1.0, 1.0, 1.0]}
    xyz: [1.6, -0.55, 0.0]
    rpy: [0, 0, 3.14159]
    material: {color: [0.25, 0.28, 0.33], roughness: 0.7}
```

The props are the AWS RoboMaker small-house meshes, under **MIT-0** — no
attribution condition, so they can simply be committed. `osrf/gazebo_models` is
the better-known source and was rejected: it is CC-BY 3.0, and it has no chair
of any kind. Its `bookshelf`, `cabinet` and `table` are not meshes at all but
SDF box assemblies, which you can transcribe straight into `bodies:` if that is
all you need.

```bash
git clone --depth 1 -b ros2 \
  https://github.com/aws-robotics/aws-robomaker-small-house-world /tmp/aws
scripts/dae_to_obj.py /tmp/aws/models/aws_robomaker_residential_ChairD_01/meshes/aws_ChairD_01_visual.DAE \
  -o worlds/my_world/meshes/chair.obj
```

Take the `_visual.DAE`, never the `_collision.DAE`: the two carry different root
transforms, and at 360-760 triangles the visual mesh is cheap enough to serve as
exact collision geometry directly. Record what you took and under what licence —
`worlds/small_office/meshes/SOURCE.md` is the pattern.

## Why OBJ

Not a preference — a requirement on one side and the right answer on the other.

**Isaac Sim does not read Collada.** On 6.1.0 the asset converter answers

    Unsupported import format: .dae. Supported formats: .bvh, .fbx, .glb,
    .gltf, .lxo, .md5, .obj, .ply, .stl, .usd, .usda, .usdc, .usdz

so a world whose meshes are `.dae` builds for Gazebo and **cannot be built for
Isaac Sim at all**. Gazebo Classic's loader reads `dae`, `obj` and `stl`, so OBJ
and STL are the intersection.

Between those two, OBJ — and unlike Collada it carries neither a unit scale nor
an up-axis header for the two backends to interpret differently, which is
exactly the trap `turtlebot3_world`'s original meshes embodied. If what you have
is Collada, convert it once:

```bash
scripts/dae_to_obj.py worlds/my_world/meshes/room.dae
```

which bakes the declared `<unit>` into the vertices (OBJ has no unit field),
applies the visual scene's node transforms, and prints the resulting bounds so
you can check them against your manifest's `verify` block.

Applying the node transforms matters more than it sounds. The converter used to
read `library_geometries` directly and merely warn that a `<matrix>` was being
ignored, and on a real prop that produces geometry which is *structurally* wrong
and *dimensionally* plausible. `gazebo_models`' `cafe_table.dae` instances its
tabletop under a separate `+29 inch` Z translate: with the transform dropped,
the top lands at z = 0.00-0.04 m — lying on the floor — while the overall height
comes out 0.737 m against a correct 0.775 m. A 38 mm difference in the bounding
box, which is to say `verify` would have passed it.

Because the root transform is baked in, a converted prop takes
`scale: [1, 1, 1]`, sits with its base at z = 0, and is placed by its footprint
centre. Then point the manifest at the `.obj` and delete the `.dae`:
two files holding the same geometry is the thing this registry exists to
prevent.

## Why not USD or SDF as the single format

Both were considered and neither works today. Gazebo Classic has no USD code
path at all, and the one converter that exists — [`gz-usd`][gz-usd] — targets
gz-sim rather than Classic, has no releases, requires building OpenUSD 24.08
from source, and has not had a commit since October 2024. In the other
direction, Isaac Sim 6.x ships importers for URDF, MJCF, Onshape and CAD, and
none for SDF. URDF is not a substitute either: it has no concept of a world —
no lights, no ground plane, no `<static>` — and Isaac's URDF importer builds an
articulation with rigid bodies, which is the opposite of a static room.

So the canonical form is a small manifest that both can be *generated* from,
and the generators are two files of a few hundred lines each.

[gz-usd]: https://github.com/gazebosim/gz-usd

## Keeping them from drifting

```bash
scripts/check_worlds.py          # every world; exit status is the answer
```

Three ways a world can quietly stop being one world, and what catches each:

| what goes wrong | what catches it |
|---|---|
| manifest edited, artifacts not regenerated | the sha256 of `world.yaml` is stamped into the `.world` header and the USD's `customLayerData` |
| a generated file edited by hand | the `.world` is parsed back and compared body by body against the manifest |
| the model no longer matches the real room | the manifest is cut at the burger's 0.182 m beam height and compared against the world's own map |

The third is the one worth understanding. It answers a different question from
the first two: not "were these built from the same file" but "does this file
still describe the room the robot drove around". It reports how much of the map
is modelled and how much of the model is not in the map, with one cell of slack.
A correct clone scores 100% / 0%. A map mirrored about its x axis — a real bug,
and one that plans and drives without looking broken — scores 27% / 27%, and
mirrored about y, 31% / 24%. Both fail on coverage well before the contradiction
figure matters, which is the point: an arena as symmetric as this one cannot be
caught by contradictions alone.

It runs on stdlib and PyYAML in under a second, so put it in a pre-commit hook.
It deliberately does **not** parse the USD, which would need Kit or `usd-core`;
the Isaac stage is checked where it is written instead, by `build_world_usd.py`,
which asserts the assembled stage's bounds against the manifest's `verify` block
and refuses to write one whose colliders are wrong.

## Adding one by hand

For a measured room, or anything simpler than a scan:

1. `mkdir worlds/my_office`
2. Put any geometry in `meshes/` (`.obj` for preference — see above).
3. Write `world.yaml`:

   ```yaml
   name: my_office            # must equal the directory name
   description: >-
     Ground floor, measured 2026-09.
   spawn:
     xyz: [0.0, 0.0, 0.01]
     yaw: 0.0
   map: map/my_office.yaml    # optional; needed for nav:=true
   bodies:
     - name: wall_n
       geometry: {type: box, size: [5.0, 0.1, 1.0]}
       xyz: [0, 2.0, 0.5]
       rpy: [0, 0, 0]
     - name: shell
       geometry: {type: mesh, uri: meshes/office.obj, scale: [1, 1, 1]}
       xyz: [0, 0, 0]
       rpy: [0, 0, 0]
       material: asset            # keep the mesh's own .mtl
   ```

   Besides `mesh`, the generators understand `cylinder` (`radius`, `length`),
   `box` (`size`) and `sphere` (`radius`). Poses are SDF's: `xyz` in metres,
   `rpy` in radians, extrinsic XYZ.

4. `scripts/build_world.sh my_office`
5. `world:=my_office`. No launch file changes.

## Worlds that are already authored

Some environments should not be generated. `turtlebot3_world`'s geometry is
upstream's; Isaac Sim's `Simple_Warehouse` is NVIDIA's and is fetched from their
asset root rather than living on disk at all. Those declare where their
artifacts are instead of having them written:

```yaml
artifacts:
  gazebo:   {mode: adopted, path: warehouse.world}
  isaacsim: {mode: adopted, path: /Isaac/Environments/Simple_Warehouse/warehouse.usd,
             world_z: 0.0}
```

- `generated` (the default) — the generator writes the file from `bodies`.
- `adopted` — the file is authored elsewhere; the generator leaves it alone and
  `world:=` simply points at it. An Isaac path may be a local file, an Isaac
  asset-root path (`/Isaac/...`) or a URL.
- `none` — this backend needs no file. `empty_stage` uses it for Isaac Sim,
  which authors its own ground plane and light.

There is one mechanism, not two: `world:=`, the spawn pose and the map work
identically either way, and `check_worlds.py` still runs — it just checks the
footprint against the map rather than a provenance digest, since there is no
generator to claim provenance.

`artifacts` is the escape hatch, not the habit. A world with `mode: adopted` on
both backends is two hand-authored files that nothing can prove agree; use it
when the artifacts genuinely come from somewhere else, and generate otherwise.

## Two things that bite

**Mesh collisions must stay exact.** Every body here is static, so triangle-mesh
collision is legal. A convex hull is not a safe default: the `turtlebot3_world`
wall is a thin hexagonal shell whose hull is a *solid* prism, which seals the
robot inside the arena at spawn. `build_world_usd.py` sets the approximation
explicitly, and verifies it afterwards, for this reason.

**A mesh can arrive at the wrong scale, and nothing will say so.** This is what
the manifest's `verify` block is for, and it has now caught two of them:

- *Centimetres.* Isaac Sim's asset converter authors converted layers with
  `metersPerUnit = 0.01` by default, and USD scales a reference by the ratio of
  the two layers' units — so a mesh whose vertices are in metres arrives in the
  (metres) stage exactly **100x too small**. `build_world_usd.py` sets
  `use_meter_as_world_unit`. A format that declares its own unit escapes this;
  OBJ and STL, which declare nothing, do not. It went unnoticed for as long as
  every world here used Collada.
- *Collada's headers.* `turtlebot3_world`'s original meshes declared
  `<unit name="inch" meter="0.0254"/>` and `up_axis Y_UP` while laying their
  vertices out Z-up. Gazebo applies the unit and ignores the axis, so Z-up at
  0.0254 m is the parity truth; a converter that *honoured* `Y_UP` would tip the
  Isaac arena on its side while Gazebo stayed correct, and one that ignored
  `<unit>` would make it 39.4x too large. Both are now baked into the committed
  `.obj` and cannot recur.

`build_world_usd.py` verifies the assembled stage's bounds against `verify`
rather than trusting the converter, and it clears the mesh cache whenever the
converter settings change — the cache is keyed on mtime alone, so a stale entry
would otherwise survive the very fix that was meant to correct it.
