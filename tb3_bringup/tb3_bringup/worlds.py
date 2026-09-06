"""Resolve worlds out of the repo's world registry.

One environment, two simulators. `worlds/<name>/world.yaml` is the source of
truth; the Gazebo `.world` and the Isaac Sim `.usd` beside it are both generated
from it, and the meshes under `worlds/<name>/meshes/` are shared byte-for-byte.
Adding an environment is therefore a matter of adding a directory, never of
editing launch files — which is the reason this module exists at all.

The registry is a plain directory tree rather than an ament package because the
isaacsim container has no ROS and cannot call get_package_share_directory. Both
containers mount the same tree (see docker-compose.yml) and find it through
TB3_WORLDS_DIR, so the two sides genuinely read the same files.
"""

import os

import yaml

#: Where the registry is mounted inside both containers. Overridable so the
#: generators and tests can run against a checkout on the host.
ENV_VAR = 'TB3_WORLDS_DIR'
DEFAULT_ROOT = '/worlds'


def root():
    """Registry root. Falls back to the checkout when the mount is absent."""
    explicit = os.environ.get(ENV_VAR)
    if explicit:
        return explicit
    if os.path.isdir(DEFAULT_ROOT):
        return DEFAULT_ROOT
    # Running from a source checkout (host-side tooling, tests): walk up out of
    # tb3_bringup/tb3_bringup/ to the repo root.
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(os.path.dirname(os.path.dirname(here)), 'worlds')


def available(worlds_root=None):
    r = worlds_root or root()
    if not os.path.isdir(r):
        return []
    return sorted(n for n in os.listdir(r)
                  if os.path.isfile(os.path.join(r, n, 'world.yaml')))


def _fail(msg, worlds_root):
    raise RuntimeError(
        f'{msg}\n'
        f'  registry: {worlds_root}\n'
        f'  available: {", ".join(available(worlds_root)) or "(none)"}\n'
        f'  If that list is empty the registry is probably not mounted; '
        f'check the worlds volume in docker-compose.yml or set {ENV_VAR}.')


def manifest(name, worlds_root=None):
    """Parsed world.yaml for `name`."""
    r = worlds_root or root()
    path = os.path.join(r, name, 'world.yaml')
    if not os.path.isfile(path):
        _fail(f"unknown world '{name}': no {name}/world.yaml", r)
    with open(path) as f:
        return yaml.safe_load(f)


def world_file(name, worlds_root=None):
    """Generated Gazebo .world for `name`.

    Missing means build_world.py was never run for it (or the generated file was
    not committed). Say so explicitly: gzserver's own failure for a missing world
    is to come up with an empty scene and no error, which is unreadable.
    """
    r = worlds_root or root()
    path = os.path.join(r, name, f'{name}.world')
    if not os.path.isfile(path):
        _fail(f"world '{name}' has no generated {name}.world "
              f"(run: scripts/build_world.py {name})", r)
    return path


def usd_file(name, worlds_root=None):
    """Generated Isaac Sim .usd for `name`. Not checked for existence here.

    The USD is gitignored and built inside the isaacsim container, so a caller on
    the ROS side cannot meaningfully verify it.
    """
    r = worlds_root or root()
    return os.path.join(r, name, f'{name}.usd')


def spawn(name, worlds_root=None):
    """Robot start pose as (x, y, z, yaw).

    Shared by both backends so a Gazebo run and an Isaac Sim run of the same
    world start from the same place; otherwise Nav2 comparisons between them are
    not comparing anything.
    """
    m = manifest(name, worlds_root)
    s = m.get('spawn') or {}
    xyz = list(s.get('xyz', [0.0, 0.0, 0.0]))
    xyz += [0.0] * (3 - len(xyz))
    return xyz[0], xyz[1], xyz[2], float(s.get('yaw', 0.0))


def resolve(name_or_path, worlds_root=None):
    """Accept either a registry name or a direct path to a .world file.

    The path form is an escape hatch for a one-off file that is not worth adding
    to the registry; `world:=` is otherwise a name.
    """
    if name_or_path.endswith('.world') and os.path.isfile(name_or_path):
        return name_or_path
    return world_file(name_or_path, worlds_root)
