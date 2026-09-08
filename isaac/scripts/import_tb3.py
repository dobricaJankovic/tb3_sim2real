"""Import the TurtleBot3 URDF into Isaac Sim and assemble the base world stage.

Two-container workflow — this script is only the isaacsim-side half. The URDF
importer's core does not parse XACRO, and the isaacsim container has no
ROS/xacro installed, so expand it in tb3_ros first and drop the flat URDF on
the shared isaac/scenes/ bind mount:

    docker exec <tb3_ros container> bash -c \
      'source /opt/ros/humble/setup.bash; xacro \
       $(ros2 pkg prefix --share turtlebot3_description)/urdf/turtlebot3_${TURTLEBOT3_MODEL}.urdf \
       namespace:=' > isaac/scenes/turtlebot3_${TURTLEBOT3_MODEL}.urdf

The expanded URDF still points at its meshes by `package://` URL, and the
isaacsim container has no ROS and therefore no turtlebot3_description on disk
to resolve them against. Copy the package's share directory onto the same bind
mount (gitignored — 40 MB of STL):

    docker cp <tb3_ros container>:/opt/ros/humble/share/turtlebot3_description \
      isaac/scenes/turtlebot3_description

Skipping that step does NOT fail the import. The converter resolves each
unmatched `package://turtlebot3_description/meshes/...` to a bare relative
path, finds nothing, and emits the link as an *empty* Xform — correct
transform, correct material binding, no geometry. You get a stage that loads
clean and renders nothing at all, with the only visible clue being FLT_MAX
sentinels in the asset's `extentsHint`. See docs/troubleshooting.md.

Then, inside the isaacsim container:

    /isaac-sim/python.sh /scripts/import_tb3.py --model burger

Produces:
    /scenes/turtlebot3_<model>.usd/...  the robot as a standalone, reusable asset
    /scenes/tb3_world.usd               ground plane + light + robot reference,
                                         the stage tb3_sim.py loads by default

tb3_world.usd also carries the surface properties neither the URDF nor the
importer provides — without them the robot rocks on its caster and creeps
across the floor with no command given. See bind_robot_surfaces().

Verified live against isaacsim.asset.importer.urdf 1.4.3 on the burger model:
4 visual meshes bounding to 138 x 178 x 191 mm, matching the physical robot.
Re-check with verify_asset.py after any change here.
"""

import argparse
import os
import shutil

from isaacsim import SimulationApp

sim_app = SimulationApp({'headless': True})

from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import add_reference_to_stage, create_new_stage
from pxr import PhysxSchema, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

enable_extension('isaacsim.asset.importer.urdf')
sim_app.update()

from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig


def import_robot(model: str) -> str:
    """Returns the path to the imported asset's .usda entry point."""
    # The importer does not overwrite: given an existing turtlebot3_burger.usd/
    # it writes turtlebot3_burger_1/ *inside* it and returns that, leaving the
    # old asset behind. Re-importing therefore keeps working while silently
    # accumulating copies, and only the returned path says which one the new
    # tb3_world.usd actually references. Clear it so an import is an import.
    asset = f'/scenes/turtlebot3_{model}.usd'
    if os.path.isdir(asset):
        print(f'removing previous {asset}', flush=True)
        shutil.rmtree(asset)

    config = URDFImporterConfig(
        urdf_path=f'/scenes/turtlebot3_{model}.urdf',
        usd_path=asset,
        # Maps `package://turtlebot3_description/<rel>` -> `<path>/<rel>`, so
        # this must be the package's share directory itself, not its parent.
        # Without it every visual mesh imports as an empty Xform (see module
        # docstring); the collision primitives are inline URDF <box>/<cylinder>
        # and import either way, which is what makes the failure so quiet.
        ros_package_paths=[
            {'name': 'turtlebot3_description', 'path': '/scenes/turtlebot3_description'},
        ],
        merge_fixed_joints=True,
        fix_base=False,                # mobile robot, not bolted to the world
        robot_type='Wheeled',
        joint_drive_type='force',
        joint_target_type='velocity',  # matches tb3_sim.py's ArticulationController usage
        # TODO: unverified gain — importer logs "Stiffness and damping not
        # available ... actuator will be created without gain parameters"
        # without this. Value below is a plausible starting point, not tuned.
        override_joint_damping=1.0e5,
        override_joint_stiffness=0.0,
    )
    importer = URDFImporter()
    importer.config = config
    return importer.import_urdf()


# Where the physics materials below are authored. Nothing downstream hard-codes
# it — tb3_sim.py's check_surfaces() resolves each collider's bound material
# rather than looking under a path — so it is free to change.
MATERIALS_PRIM = '/World/PhysicsMaterials'

# The articulation root, and the wheel links whose colliders get the tyre
# material. Same prims tb3_sim.py drives; the caster, the chassis box and the
# lidar cylinder are not listed because merge_fixed_joints folds all three into
# the root link, where they inherit its material.
ROBOT_ROOT = '/World/turtlebot3/Geometry/base_footprint'
WHEEL_LINKS = ['wheel_left_link', 'wheel_right_link']


def physics_material(name: str, *, friction: float, restitution: float = 0.0,
                     friction_combine: str | None = None):
    """Author a physics material at MATERIALS_PRIM/<name>.

    Static and dynamic friction are deliberately the same number: nothing in
    the URDF or in turtlebot3_gazebo's SDF distinguishes them, so inventing a
    difference here would be inventing data.

    restitutionCombineMode is 'min' on every material rather than PhysX's
    default 'average', because "this surface does not bounce" should hold
    against whatever the other collider brings rather than be averaged back up
    by it — which is exactly how GroundPlane's 0.8 (see build_world) reached
    the wheels.
    """
    from isaacsim.core.api.materials.physics_material import PhysicsMaterial

    material = PhysicsMaterial(prim_path=f'{MATERIALS_PRIM}/{name}',
                               static_friction=float(friction),
                               dynamic_friction=float(friction),
                               restitution=float(restitution))
    physx = PhysxSchema.PhysxMaterialAPI.Apply(material.prim)
    physx.CreateRestitutionCombineModeAttr().Set('min')
    if friction_combine is not None:
        physx.CreateFrictionCombineModeAttr().Set(friction_combine)
    return material


def bind_physics_material(prim, material) -> None:
    """Bind for the *physics* purpose.

    A plain Bind() writes `material:binding`, which is the render purpose —
    PhysX never reads it, and the collider goes on using the default material
    with nothing anywhere reporting a problem. The purpose argument is what
    makes it `material:binding:physics`.

    weakerThanDescendants so a binding lower down wins: the root link's
    material is the fallback for every shape merged into it, and the wheels
    override it.
    """
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        material.material, UsdShade.Tokens.weakerThanDescendants, 'physics')


def bind_robot_surfaces(stage) -> None:
    """Give the robot's colliders real surface properties.

    The URDF says nothing about friction or restitution and the importer binds
    no physics material to any collider, so every surface on the robot comes up
    on PhysX's fallback material. The floor is worse than a fallback:
    GroundPlane authors its *own* material at restitution 0.8 when it is not
    handed one — the "set default values if no physics material given" branch
    in isaacsim.core.api.objects.ground_plane. Restitution combines as an
    average by default, so every wheel-on-floor and skid-on-floor contact came
    out at 0.4: a bouncy-ball floor under a 0.94 kg robot.

    That is not cosmetic on a burger. Its centre of mass sits 4.3 mm BEHIND the
    wheel axle while the caster skid (caster_back_link's 30 x 9 x 20 mm box)
    clears the floor by only 0.5 mm, so the chassis permanently rests on that
    skid, exactly as the real robot does. A permanently loaded elastic contact
    under a rear skid is a rocking chair, and the robot rocked and crept with
    the wheels commanded to nothing at all.

    Values, and where they come from:

    floor/wheel  turtlebot3_gazebo's models/turtlebot3_burger/model.sdf gives
                 both tyres mu = mu2 = 100000 — "must not slip", with
                 upstream's own comment that the number is not real data. PhysX
                 takes a coefficient rather than ODE's mu, so 1.0 is the honest
                 spelling of the same intent.
    chassis      the caster is a smooth skid dragging on the floor, so it gets
                 a low coefficient, with frictionCombineMode 'min' so it stays
                 slippery whatever floor it meets. A high-friction skid fights
                 the wheels on every in-place turn. (Gazebo dodges this by
                 making caster_back_joint a *ball* joint, so its caster rolls
                 where ours slides.)
    restitution  0 on every surface here. Neither a tyre, a plastic skid nor a
                 floor is elastic at these speeds.
    """
    wheel = physics_material('wheel', friction=1.0)
    chassis = physics_material('chassis', friction=0.1, friction_combine='min')

    # Root link first (the fallback for everything merged into it), wheels
    # second (they override it).
    bind_physics_material(stage.GetPrimAtPath(ROBOT_ROOT), chassis)
    for link in WHEEL_LINKS:
        prim = stage.GetPrimAtPath(f'{ROBOT_ROOT}/{link}')
        if not prim.IsValid():
            raise RuntimeError(f'no {link} under {ROBOT_ROOT} — the importer '
                               f'named the links differently, so the tyre '
                               f'material would land nowhere')
        colliders = [p for p in Usd.PrimRange(prim)
                     if p.HasAPI(UsdPhysics.CollisionAPI)]
        if not colliders:
            raise RuntimeError(f'{link} has no collider to bind a tyre '
                               f'material to')
        for collider in colliders:
            bind_physics_material(collider, wheel)

    print('surfaces: wheels mu=1.0, chassis and caster skid mu=0.1, '
          'restitution 0 throughout', flush=True)


def build_world(robot_usda: str) -> None:
    create_new_stage()

    import omni.usd
    stage = omni.usd.get_context().get_stage()

    UsdGeom.Xform.Define(stage, '/World')

    # Handed a floor material explicitly. Left to itself GroundPlane authors
    # one at restitution 0.8, which is half of why the robot bounced where it
    # stood — see bind_robot_surfaces() for the whole story.
    from isaacsim.core.api.objects import GroundPlane
    floor = physics_material('floor', friction=1.0)
    GroundPlane(prim_path='/World/GroundPlane', physics_material=floor)
    # GroundPlane binds what it is given to its mesh collider and leaves the
    # infinite `collisionPlane` beside it on PhysX's fallback material. Binding
    # the parent as well is what makes both resolve to the same floor.
    bind_physics_material(stage.GetPrimAtPath('/World/GroundPlane'), floor)

    light = UsdLux.DistantLight.Define(stage, '/World/DistantLight')
    light.CreateIntensityAttr(1000)

    # ROBOT_PRIM in tb3_sim.py. The imported asset's own root prim is
    # named after the URDF's <robot name="...">  (turtlebot3_<model>), not
    # this — referencing pins it at the path the rest of the pipeline expects.
    add_reference_to_stage(usd_path=robot_usda, prim_path='/World/turtlebot3')

    # After the reference: this binds materials onto the robot's colliders, and
    # they do not exist on this stage until the reference is composed.
    bind_robot_surfaces(stage)

    sim_app.update()
    stage.GetRootLayer().Export('/scenes/tb3_world.usd')


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', default='burger', choices=['burger', 'waffle', 'waffle_pi'])
    args = parser.parse_args()

    robot_usda = import_robot(args.model)
    print(f'Robot asset: {robot_usda}', flush=True)

    build_world(robot_usda)
    print('Saved /scenes/tb3_world.usd', flush=True)

    # sim_app.close() races a background task-pool teardown (Isaac Sim 6.0
    # known issue: "Destroying busy TaskGroup!") and aborts the process —
    # after everything above, the abort happens with the work already on
    # disk, so skip the graceful shutdown rather than chase the race.
    os._exit(0)


if __name__ == '__main__':
    main()
