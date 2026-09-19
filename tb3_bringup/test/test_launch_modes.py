"""The three modes, pinned. No simulator, no GPU, ~2 seconds.

    bare          robot + RViz, nothing else
    nav:=true     map_server + AMCL + Nav2 on the world's saved map
    slam:=true    slam_toolbox + Nav2, mapping while navigating

This is the interface the whole repository is about, and it is the thing an
edit to bringup.launch.py is most likely to break silently: every combination
still *launches*, it just launches the wrong set of nodes, which only shows up
minutes later as a missing transform. So the check is on which launch files get
included, not on whether the description builds.

It calls setup() against a hand-built LaunchContext rather than running
`ros2 launch`, which is what makes it cheap enough to run every time.

Two launch internals are reached into, because launch exposes no public way to
ask "what would this include?": the resolved path of an IncludeLaunchDescription
and the actions an event handler is holding. Both are name-mangled privates and
both are version-specific, so each is looked up through a list of candidate
names and the test SKIPS rather than fails when none matches — a launch upgrade
should not read as a broken interface.
"""

import os

import pytest
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.utilities import perform_substitutions

# Where the include's resolved path hides, newest spelling first.
_LOCATION_ATTRS = ('_LaunchDescriptionSource__location', 'launch_file_path')
# Where an event handler keeps the actions it will emit.
_HANDLER_ATTRS = ('_OnActionEventBase__actions_on_event',
                  '_OnProcessExit__actions_on_exit')


def _bringup():
    import importlib.util
    path = os.path.join(get_package_share_directory('tb3_bringup'),
                        'launch', 'bringup.launch.py')
    spec = importlib.util.spec_from_file_location('bringup_launch', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _first_attr(obj, names):
    for name in names:
        value = getattr(obj, name, None)
        if value:
            return value
    return None


def _included(**overrides):
    """Basenames of every launch file a given set of arguments would include."""
    mod = _bringup()
    context = LaunchContext()
    for action in mod.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument) and action.default_value:
            context.launch_configurations[action.name] = perform_substitutions(
                context, action.default_value)
    context.launch_configurations.update(overrides)

    names, pending = [], list(mod.setup(context))
    while pending:
        action = pending.pop(0)
        if isinstance(action, RegisterEventHandler):
            # On a simulated backend the whole stack is deferred until /clock
            # arrives, so it lives inside the handler rather than in the list.
            held = _first_attr(action.event_handler, _HANDLER_ATTRS)
            if held is None:
                pytest.skip('launch changed how event handlers hold actions')
            pending.extend(held)
            continue
        for sub in getattr(action, '_GroupAction__actions', None) or []:
            source = getattr(sub, 'launch_description_source', None)
            if source is None:
                continue
            location = _first_attr(source, _LOCATION_ATTRS)
            if location is None:
                pytest.skip('launch changed where an include keeps its path')
            if not isinstance(location, str):
                location = perform_substitutions(context, location)
            names.append(os.path.basename(location))
    return set(names)


ROBOT = {'state_publisher.launch.py', 'gazebo.launch.py', 'rviz.launch.py'}


@pytest.mark.parametrize('overrides,expected', [
    ({}, ROBOT),
    ({'nav': 'true'}, ROBOT | {'localization_launch.py', 'navigation_launch.py'}),
    ({'slam': 'true'}, ROBOT | {'slam_launch.py', 'navigation_launch.py'}),
    # slam:=true nav:=true is accepted and means slam:=true, rather than being
    # a rejected combination anyone has to remember.
    ({'slam': 'true', 'nav': 'true'}, ROBOT | {'slam_launch.py', 'navigation_launch.py'}),
])
def test_modes(overrides, expected):
    assert _included(backend='gazebo', world='turtlebot3_world', **overrides) == expected


@pytest.mark.parametrize('overrides,expected', [
    # backend:=real contributes no robot layer: the robot runs its own bringup.
    ({}, {'rviz.launch.py'}),
    ({'nav': 'true'}, {'rviz.launch.py', 'localization_launch.py',
                       'navigation_launch.py'}),
])
def test_real_starts_no_robot_layer(overrides, expected):
    assert _included(backend='real', world='turtlebot3_world', **overrides) == expected


def test_nav_without_a_map_names_the_fix():
    with pytest.raises(RuntimeError) as excinfo:
        _included(backend='gazebo', world='empty_stage', nav='true')
    message = str(excinfo.value)
    assert 'slam:=true' in message and 'save_map.py' in message


@pytest.mark.parametrize('backend,given,forwarded', [
    # Empty means "leave it to the backend", which owns the number.
    ('isaacsim', None, None),
    ('isaacsim', '240.0', '240.0'),
    # Gazebo declares no such argument; forwarding it would fail the launch.
    ('gazebo', '240.0', None),
])
def test_physics_hz_reaches_isaacsim_only(backend, given, forwarded):
    world = 'empty_stage' if backend == 'isaacsim' else 'turtlebot3_world'
    overrides = {'physics_hz': given} if given else {}
    mod, context = _bringup(), LaunchContext()
    for action in mod.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument) and action.default_value:
            context.launch_configurations[action.name] = perform_substitutions(
                context, action.default_value)
    context.launch_configurations.update(backend=backend, world=world, **overrides)
    for action in mod.setup(context):
        for sub in getattr(action, '_GroupAction__actions', None) or []:
            arguments = dict(getattr(sub, 'launch_arguments', []))
            if 'x_pose' in arguments:        # the backend include, whichever it is
                assert arguments.get('physics_hz') == forwarded
                return
    pytest.fail('no backend include found')


def test_backend_does_not_pin_its_own_physics_rate():
    """physics_hz reaches turtlebot3_isaacsim only when the operator set it.

    The rate belongs to that package. This file pinned its own 480 for a day
    after the package had moved to 240, which is the whole reason the default
    here is empty rather than a number.
    """
    import importlib.util
    path = os.path.join(get_package_share_directory('tb3_bringup'),
                        'launch', 'backends', 'isaacsim.launch.py')
    spec = importlib.util.spec_from_file_location('isaacsim_backend', path)
    backend = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(backend)

    pkg = get_package_share_directory('turtlebot3_isaacsim')

    def forwarded(**overrides):
        context = LaunchContext()
        for action in backend.generate_launch_description().entities:
            if isinstance(action, DeclareLaunchArgument):
                context.launch_configurations[action.name] = (
                    perform_substitutions(context, action.default_value)
                    if action.default_value else '')
        context.launch_configurations.update(overrides)
        group, = backend.include_simulator(context, pkg)
        # Scoping is the invariant, not a detail: without it every argument
        # declared in the backend file leaks into turtlebot3_isaacsim's own
        # launch file and OVERRIDES its defaults, which is how an unset
        # physics_hz once reached the simulator as `--physics-hz ''` and
        # killed it on startup.
        assert getattr(group, '_GroupAction__scoped') is True
        assert getattr(group, '_GroupAction__forwarding') is False
        include, = getattr(group, '_GroupAction__actions')
        return dict(include.launch_arguments)

    assert 'physics_hz' not in forwarded()
    assert forwarded(physics_hz='240.0')['physics_hz'] == '240.0'
    # The pass-through arguments must still all arrive.
    assert set(backend.PASS_THROUGH) <= set(forwarded())


def test_nav_pins_amcl_to_the_manifest_spawn():
    """AMCL starts where the manifest says, with nobody clicking in RViz.

    The failure this pins is silent and expensive: with set_initial_pose unset
    AMCL waits, every run needs a human to click "2D Pose Estimate", and the
    few centimetres that click is off land in exactly the numbers the
    experiments compare. docs/experiment-plan.md B5.

    Performed for real and read back rather than inspected: this failure mode
    is entirely silent. The first implementation used nav2_common's
    RewrittenYaml, on an audit's word that it creates keys absent from the
    source file. It does not -- it rewrites only paths the file already has --
    so the launch succeeded, a params file was written, and AMCL waited for a
    mouse anyway. Nothing reported anything. This test is what caught it.
    """
    import yaml

    from tb3_bringup import worlds

    mod = _bringup()
    world = worlds.World.load('turtlebot3_world')
    params = os.path.join(get_package_share_directory('tb3_bringup'),
                          'config', 'nav2_params.yaml')
    written = mod.pin_initial_pose(params, world)
    with open(written) as f:
        amcl = yaml.safe_load(f)['amcl']['ros__parameters']
    x, y, _z, yaw = world.spawn

    assert amcl['set_initial_pose'] is True
    assert amcl['initial_pose']['x'] == pytest.approx(x)
    assert amcl['initial_pose']['y'] == pytest.approx(y)
    assert amcl['initial_pose']['yaw'] == pytest.approx(yaw)
    # Everything else survives the rewrite: this is upstream's params file with
    # four keys added, not a params file this repository now maintains.
    assert amcl['robot_model_type'] == 'nav2_amcl::DifferentialMotionModel'


def test_the_params_file_itself_is_never_edited():
    """The rewrite exists so config/nav2_params.yaml stays verbatim upstream.

    One params file, unconditional, is what keeps the sim-to-real gap from
    being absorbed into per-backend tuning (docs/architecture.md). A pinned
    pose written INTO that file would be the first edit.
    """
    params = os.path.join(get_package_share_directory('tb3_bringup'),
                          'config', 'nav2_params.yaml')
    with open(params) as f:
        text = f.read()
    assert 'set_initial_pose' not in text and 'initial_pose:' not in text


def _gazebo_backend():
    import importlib.util
    path = os.path.join(get_package_share_directory('tb3_bringup'),
                        'launch', 'backends', 'gazebo.launch.py')
    spec = importlib.util.spec_from_file_location('gazebo_backend', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_gazebo_wheel_friction_follows_the_manifest():
    """The wheel gets the floor's coefficient, not upstream's sentinel.

    turtlebot3_gazebo ships the tyres at mu = 100000 with its own comment
    saying the number is not real data. That single value is the whole of why
    this backend shows no wheel slip while Isaac Sim and the real robot do, so
    a regression here would not raise -- it would quietly restore a backend
    that cannot slip, and the sim-to-real comparison would silently become a
    comparison with a sentinel.

    Read back off the written SDF rather than trusting the call, for the same
    reason test_nav_pins_amcl_to_the_manifest_spawn does.

    slip1/slip2 are asserted UNCHANGED. They are ODE's force-dependent slip,
    not an on/off switch for slipping, and turning them on would add a second
    free parameter that nothing measures. Exactly one thing moves.
    """
    import xml.etree.ElementTree as ET

    from tb3_bringup import worlds

    mod = _gazebo_backend()
    src = os.path.join(get_package_share_directory('turtlebot3_gazebo'),
                       'models', 'turtlebot3_burger', 'model.sdf')
    mu = worlds.World.load('turtlebot3_world').surface['mu']
    written = mod.with_surfaces(src, mu)

    root = ET.parse(written).getroot()
    wheels = [c for c in root.iter('collision')
              if c.get('name') in mod.WHEEL_COLLISIONS]
    assert len(wheels) == 2, 'both wheels must be rewritten, not one'
    for col in wheels:
        ode = col.find('surface/friction/ode')
        assert float(ode.findtext('mu')) == pytest.approx(mu)
        assert float(ode.findtext('mu2')) == pytest.approx(mu)
        assert float(ode.findtext('slip1')) == 0.0
        assert float(ode.findtext('slip2')) == 0.0

    # The other job of the same rewrite still happens.
    assert 'libgazebo_ros_p3d.so' in open(written).read()

    # And upstream's file is untouched: this repository edits a copy.
    assert '100000' in open(src).read()


def test_gazebo_floor_and_wheel_carry_the_same_number():
    """Both sides of the contact are equal, so the combine rule cannot matter.

    ODE's rule for combining a contact pair is believed to be the minimum and
    is not verified here; PhysX defaults to averaging. Setting both sides to
    the same coefficient makes minimum, average and geometric mean agree, so
    the two backends resolve to the same effective friction without this
    repository having to be right about either engine's convention.
    """
    import xml.etree.ElementTree as ET

    from tb3_bringup import worlds

    world = worlds.World.load('turtlebot3_world')
    floor = None
    for model in ET.parse(world.gazebo_world()).getroot().iter('model'):
        if model.get('name') == 'ground_plane':
            floor = model.find('.//collision/surface/friction/ode')
    assert floor is not None, 'the generated world declares no floor friction'
    assert float(floor.findtext('mu')) == pytest.approx(world.surface['mu'])
