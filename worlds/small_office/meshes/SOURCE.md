# Where these came from

All six are the `_visual.DAE` meshes of the AWS RoboMaker small-house world,
converted once with `scripts/dae_to_obj.py`:

    https://github.com/aws-robotics/aws-robomaker-small-house-world  (branch ros2)
    models/aws_robomaker_residential_<Prop>/meshes/aws_<Prop>_visual.DAE

| file | upstream model | size (m) | tris |
|---|---|---|---|
| `table.obj` | `KitchenTable_01` | 0.70 x 1.80 x 0.80 | 756 |
| `chair.obj` | `ChairD_01` | 0.57 x 0.49 x 0.78 | 712 |
| `shelf.obj` | `ShoeRack_01` | 0.89 x 0.30 x 1.14 | 520 |
| `cabinet.obj` | `NightStand_01` | 0.73 x 0.47 x 0.68 | 616 |
| `coffee_table.obj` | `CoffeeTable_01` | 1.32 x 0.65 x 0.32 | 360 |
| `bin.obj` | `Trash_01` | 0.29 x 0.28 x 0.34 | 720 |

**Licence: MIT-0** (Amazon, 2019) — use without restriction and, unlike
CC-BY, with no attribution condition, so the files can simply be committed.
This is why they were chosen over `osrf/gazebo_models` (CC-BY 3.0), which also
has no chair of any kind.

Each converts with its base at z = 0 and centred in x/y, so a body places it by
its footprint centre and takes `scale: [1, 1, 1]` — the root node transform is
baked into the vertices by the converter.

**Use the `_visual.DAE`, never the `_collision.DAE`.** The two carry different
root transforms (ShoeRack's collision mesh adds a 90 degree rotation and a 60.3
unit translate), and the visual meshes are already cheap enough — 360 to 756
triangles — to use directly as exact collision geometry.

Materials are NOT carried: `dae_to_obj.py` writes positions and faces only.
Colour comes from the manifest instead, which is what makes it identical in
both simulators. See `worlds/README.md`.
