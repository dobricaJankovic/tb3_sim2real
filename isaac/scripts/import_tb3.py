"""Import the TurtleBot3 URDF into Isaac Sim and assemble the base world stage.

Two-container workflow — this script is only the isaacsim-side half. The URDF
importer's core does not parse XACRO, and the isaacsim container has no
ROS/xacro installed, so expand it in tb3_ros first and drop the flat URDF on
the shared isaac/scenes/ bind mount:

    docker exec <tb3_ros container> bash -c \
      'source /opt/ros/humble/setup.bash; xacro \
       $(ros2 pkg prefix --share turtlebot3_description)/urdf/turtlebot3_${TURTLEBOT3_MODEL}.urdf \
       namespace:=' > isaac/scenes/turtlebot3_${TURTLEBOT3_MODEL}.urdf

Then, inside the isaacsim container:

    /isaac-sim/python.sh /scripts/import_tb3.py --model burger

Produces:
    /scenes/turtlebot3_<model>.usd/...  the robot as a standalone, reusable asset
    /scenes/tb3_world.usd               ground plane + light + robot reference,
                                         the stage build_scene.py expects at
                                         --stage

Verified once, live, against isaacsim.asset.importer.urdf 1.4.3 /
isaacsim.core.utils.stage 6.0 on the burger model — see the two mismatches
against build_scene.py's assumptions noted below, both already reflected
there.
"""

import argparse
import os

from isaacsim import SimulationApp

sim_app = SimulationApp({'headless': True})

from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import add_reference_to_stage, create_new_stage
from pxr import UsdGeom, UsdLux

enable_extension('isaacsim.asset.importer.urdf')
sim_app.update()

from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig


def import_robot(model: str) -> str:
    """Returns the path to the imported asset's .usda entry point."""
    config = URDFImporterConfig(
        urdf_path=f'/scenes/turtlebot3_{model}.urdf',
        usd_path=f'/scenes/turtlebot3_{model}.usd',
        merge_fixed_joints=True,
        fix_base=False,                # mobile robot, not bolted to the world
        robot_type='Wheeled',
        joint_drive_type='force',
        joint_target_type='velocity',  # matches build_scene.py's ArticulationController usage
        # TODO: unverified gain — importer logs "Stiffness and damping not
        # available ... actuator will be created without gain parameters"
        # without this. Value below is a plausible starting point, not tuned.
        override_joint_damping=1.0e5,
        override_joint_stiffness=0.0,
    )
    importer = URDFImporter()
    importer.config = config
    return importer.import_urdf()


def build_world(robot_usda: str) -> None:
    create_new_stage()

    import omni.usd
    stage = omni.usd.get_context().get_stage()

    UsdGeom.Xform.Define(stage, '/World')

    from isaacsim.core.api.objects import GroundPlane
    GroundPlane(prim_path='/World/GroundPlane')

    light = UsdLux.DistantLight.Define(stage, '/World/DistantLight')
    light.CreateIntensityAttr(1000)

    # ROBOT_PRIM in build_scene.py. The imported asset's own root prim is
    # named after the URDF's <robot name="...">  (turtlebot3_<model>), not
    # this — referencing pins it at the path the rest of the pipeline expects.
    add_reference_to_stage(usd_path=robot_usda, prim_path='/World/turtlebot3')

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
