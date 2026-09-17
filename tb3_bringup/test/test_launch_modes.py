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
