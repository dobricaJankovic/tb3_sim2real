#!/usr/bin/env python3
#
# Copyright 2026 dobricaJankovic
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Build this package's robot asset: turtlebot3_description's URDF -> USD.

Produces `models/turtlebot3_<model>/turtlebot3_<model>.usd`, which is this
package's counterpart of turtlebot3_gazebo's `models/turtlebot3_<model>/model.sdf`
-- a self-contained, referenceable robot with geometry, an articulation, and
surface properties. Run it once per model; the result is a build artifact, not
something to hand-edit.

Normally driven by the wrapper, which handles the xacro step for you:

    scripts/build_models.sh burger

Directly, if you prefer:

    isaacsim-python scripts/import_turtlebot3.py --model burger \\
        --urdf /tmp/turtlebot3_burger.urdf \\
        --description-share /opt/ros/$ROS_DISTRO/share/turtlebot3_description

**Why the URDF must be expanded first.** This runs on Kit's Python 3.12 with the
system ROS 2 stripped off PATH and PYTHONPATH, so `xacro` is not callable from
here and neither is the ROS package index. The importer's core does not parse
XACRO either. turtlebot3_description's URDF *is* a xacro template (it carries a
${namespace} argument), so it has to be expanded by a ROS shell beforehand.

**Why --description-share matters more than it looks.** The expanded URDF points
at its meshes by `package://turtlebot3_description/meshes/...` URL, and with no
ROS package index there is nothing to resolve that against. Skipping it does NOT
fail the import: the converter resolves each unmatched URL to a bare relative
path, finds nothing, and emits the link as an *empty* Xform -- correct
transform, correct material binding, no geometry. You get a stage that loads
clean and renders nothing at all. The collision primitives are inline URDF
<box>/<cylinder> and import either way, which is what makes it so quiet.
verify() at the bottom is the check for exactly that.
"""

import argparse
import os
import shutil
import sys
import traceback

SHARE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

WHEEL_LINKS = ['wheel_left_link', 'wheel_right_link']


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model', default=os.environ.get('TURTLEBOT3_MODEL', 'burger'),
                   choices=['burger', 'waffle', 'waffle_pi'])
    p.add_argument('--urdf', required=True,
                   help='xacro-expanded URDF (see module docstring)')
    p.add_argument('--description-share', required=True,
                   help="turtlebot3_description's share directory, for "
                        'resolving package:// mesh URLs')
    p.add_argument('--output', default='',
                   help='output .usd (default: models/turtlebot3_<model>/ in '
                        'this package)')
    return p.parse_args()


args = parse_args()
if not args.output:
    args.output = os.path.join(SHARE, 'models', f'turtlebot3_{args.model}',
                               f'turtlebot3_{args.model}.usd')

for path, what in ((args.urdf, 'URDF'), (args.description_share, 'description share')):
    if not os.path.exists(path):
        sys.exit(f'error: no {what} at {path}')

from isaacsim import SimulationApp                                   # noqa: E402

sim_app = SimulationApp({'headless': True})

from isaacsim.core.utils.extensions import enable_extension          # noqa: E402

enable_extension('isaacsim.asset.importer.urdf')
sim_app.update()

from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig  # noqa: E402
from pxr import PhysxSchema, Usd, UsdPhysics, UsdShade                     # noqa: E402


def import_robot():
    """Convert the URDF. Returns the path to the asset's .usda entry point."""
    # The importer does not overwrite: given an existing turtlebot3_burger.usd/
    # it writes turtlebot3_burger_1/ *inside* it and returns that, leaving the
    # old asset behind. Re-importing therefore keeps working while silently
    # accumulating copies, and only the returned path says which one is new.
    # Clear it so that an import is an import.
    if os.path.isdir(args.output):
        print(f'removing previous {args.output}', flush=True)
        shutil.rmtree(args.output)
    os.makedirs(os.path.dirname(args.output), exist_ok=True)

    config = URDFImporterConfig(
        urdf_path=args.urdf,
        usd_path=args.output,
        # Maps `package://turtlebot3_description/<rel>` -> `<path>/<rel>`, so
        # this must be the package's share directory itself, not its parent.
        ros_package_paths=[
            {'name': 'turtlebot3_description', 'path': args.description_share},
        ],
        merge_fixed_joints=True,
        fix_base=False,                # a mobile robot, not bolted to the world
        robot_type='Wheeled',
        joint_drive_type='force',
        # Matches how turtlebot3_isaacsim.py drives the wheels: the graph's
        # IsaacArticulationController writes velocityCommand.
        joint_target_type='velocity',
        # TODO unverified gain. Without it the importer logs "Stiffness and
        # damping not available ... actuator will be created without gain
        # parameters". This is a plausible starting point, not a tuned value.
        override_joint_damping=1.0e5,
        override_joint_stiffness=0.0,
    )
    importer = URDFImporter()
    importer.config = config
    return importer.import_urdf()


def find_articulation_root(stage):
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return prim
    raise RuntimeError('the import produced no articulation root, so nothing '
                       'could ever drive the wheels')


def physics_material(stage, path, *, friction, restitution=0.0,
                     friction_combine=None):
    """Author a UsdPhysics material at `path`.

    Static and dynamic friction are deliberately the same number: nothing in the
    URDF or in turtlebot3_gazebo's SDF distinguishes them, so inventing a
    difference here would be inventing data.

    restitutionCombineMode is 'min' rather than PhysX's default 'average',
    because "this surface does not bounce" should hold against whatever the
    other collider brings rather than be averaged back up by it.
    """
    material = UsdShade.Material.Define(stage, path)
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr().Set(float(friction))
    api.CreateDynamicFrictionAttr().Set(float(friction))
    api.CreateRestitutionAttr().Set(float(restitution))
    physx = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
    physx.CreateRestitutionCombineModeAttr().Set('min')
    if friction_combine is not None:
        physx.CreateFrictionCombineModeAttr().Set(friction_combine)
    return material


def bind(prim, material):
    """Bind for the *physics* purpose.

    A plain Bind() writes `material:binding`, which is the render purpose --
    PhysX never reads it, and the collider goes on using the default material
    with nothing anywhere reporting a problem. The purpose argument is what
    makes it `material:binding:physics`.

    weakerThanDescendants so a binding lower down wins: the root link's material
    is the fallback for every shape merged into it, and the wheels override it.
    """
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        material, UsdShade.Tokens.weakerThanDescendants, 'physics')


def author_surfaces(stage, root):
    """Give the robot's colliders real surface properties, inside the asset.

    The URDF says nothing about friction or restitution and the importer binds
    no physics material to any collider, so every surface comes up on PhysX's
    fallback. That is not cosmetic on a burger: its centre of mass sits 4.3 mm
    BEHIND the wheel axle while the caster skid (caster_back_link's
    30 x 9 x 20 mm box) clears the floor by only 0.5 mm, so the chassis
    permanently rests on that skid, exactly as the real robot does. A
    permanently loaded elastic contact under a rear skid is a rocking chair --
    the robot rocks and creeps with the wheels commanded to nothing at all, and
    nothing is logged.

    Authored into the asset rather than into a world stage so that the .usd this
    script produces is self-contained, the way model.sdf is for Gazebo.

    Values, and where they come from:

    wheel    turtlebot3_gazebo's model.sdf gives both tyres mu = mu2 = 100000 --
             "must not slip", with upstream's own comment that the number is not
             real data. PhysX takes a coefficient rather than ODE's mu, so 1.0
             is the honest spelling of the same intent.
    chassis  the caster is a smooth skid dragging on the floor, so a low
             coefficient, with frictionCombineMode 'min' so it stays slippery
             whatever floor it meets. A high-friction skid fights the wheels on
             every in-place turn. (Gazebo dodges this by making
             caster_back_joint a *ball* joint, so its caster rolls where ours
             slides.)
    """
    materials_root = root.GetPath().AppendChild('PhysicsMaterials')
    wheel = physics_material(stage, materials_root.AppendChild('wheel'),
                             friction=1.0)
    chassis = physics_material(stage, materials_root.AppendChild('chassis'),
                               friction=0.1, friction_combine='min')

    # Root link first (the fallback for everything merge_fixed_joints folded
    # into it: chassis box, lidar cylinder, caster skid), wheels second.
    bind(root, chassis)

    for link in WHEEL_LINKS:
        matches = [p for p in Usd.PrimRange(root) if p.GetName() == link]
        if not matches:
            raise RuntimeError(
                f'no {link} under {root.GetPath()} -- the importer named the '
                f'links differently, so the tyre material would land nowhere')
        colliders = [p for m in matches for p in Usd.PrimRange(m)
                     if p.HasAPI(UsdPhysics.CollisionAPI)]
        if not colliders:
            raise RuntimeError(f'{link} has no collider to bind a tyre '
                               f'material to')
        for collider in colliders:
            bind(collider, wheel)

    print('surfaces: wheels mu=1.0, chassis and caster skid mu=0.1, '
          'restitution 0 throughout', flush=True)


def verify(stage, root):
    """Fail on the silent failure: an import with no renderable geometry.

    See the module docstring -- a missing --description-share produces a stage
    that loads clean and renders nothing, and the only native clue is FLT_MAX
    sentinels in the asset's extentsHint.
    """
    from pxr import UsdGeom

    meshes = [p for p in Usd.PrimRange(root) if p.IsA(UsdGeom.Mesh)]
    if not meshes:
        raise RuntimeError(
            f'{args.output} has no mesh geometry at all.\n'
            f'  The package:// mesh URLs did not resolve. Check that\n'
            f'    {args.description_share}\n'
            f'  is turtlebot3_description\'s share directory itself (the one '
            f'containing meshes/), not its parent.')

    total = 0
    for mesh in meshes:
        points = UsdGeom.Mesh(mesh).GetPointsAttr().Get()
        total += len(points) if points else 0
    print(f'geometry: {len(meshes)} meshes, {total} points', flush=True)
    if total == 0:
        raise RuntimeError(f'{args.output}: {len(meshes)} meshes but no points')


def main():
    asset = import_robot()
    print(f'imported: {asset}', flush=True)

    stage = Usd.Stage.Open(asset)
    root = find_articulation_root(stage)
    print(f'articulation root: {root.GetPath()}', flush=True)

    verify(stage, root)
    author_surfaces(stage, root)
    stage.Save()

    print(f'\nwrote {args.output}', flush=True)
    print('Now run:  ros2 launch turtlebot3_isaacsim empty_world.launch.py',
          flush=True)


if __name__ == '__main__':
    status = 0
    try:
        main()
    except BaseException:
        traceback.print_exc()
        status = 1
    finally:
        sys.stdout.flush()
        sys.stderr.flush()
        # A graceful sim_app.close() races a background task-pool teardown
        # ("Destroying busy TaskGroup!") and aborts the process -- with the work
        # already on disk, so skip the graceful shutdown rather than chase it.
        os._exit(status)
