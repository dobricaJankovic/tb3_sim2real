"""The world registry: one environment, three backends.

A *world* is a directory holding `world.yaml` plus whatever that manifest
points at. The manifest is the source of truth; the Gazebo `.world` and the
Isaac Sim `.usd` are representations of it, and `tb3_bringup` never reads
anything else. That is what makes `world:=` mean the same thing whichever
backend is running:

    backend:=gazebo    -> load the world's .world in gzserver
    backend:=isaacsim  -> open the world's .usd in Kit
    backend:=real      -> nothing to load; the world contributes its map

`world:=` takes a registry name (`turtlebot3_world`) or a path to a world
directory (`/home/me/my_office`). The path form is what makes a user's own
environment a first-class citizen without adding it to this repository.

This module is plain Python with PyYAML and no ROS import on purpose: the
generators run under Isaac Sim's own interpreter, where the system ROS 2 is
deliberately stripped off the search path (ros-isolate) and
`get_package_share_directory` does not exist. Both sides find the registry the
same way and genuinely read the same files.
"""

import os

import yaml

#: Where the registry is mounted in the container. Overridable so the
#: generators, the tests and a user's own registry can live elsewhere.
ENV_VAR = 'TB3_WORLDS_DIR'
DEFAULT_ROOT = '/worlds'

MANIFEST = 'world.yaml'

#: How a backend gets its representation of a world.
#:   generated  a generator writes it from the manifest (the normal case)
#:   adopted    the artifact is authored elsewhere and only validated here
#:   none       this backend needs no file at all
MODES = ('generated', 'adopted', 'none')

BACKENDS = ('real', 'gazebo', 'isaacsim')


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
                  if os.path.isfile(os.path.join(r, n, MANIFEST)))


def locate(name_or_path, worlds_root=None):
    """Resolve `world:=` to a world directory.

    A path wins over a name, so a directory called `turtlebot3_world` sitting in
    your home directory is reachable as `world:=./turtlebot3_world` even though
    the registry has one by that name.
    """
    r = worlds_root or root()

    expanded = os.path.expanduser(name_or_path)
    if os.sep in expanded or expanded in ('.', '..'):
        d = os.path.abspath(expanded)
        if not os.path.isfile(os.path.join(d, MANIFEST)):
            raise RuntimeError(
                f'world: {d} is not a world directory (no {MANIFEST} in it).\n'
                f'  A world directory holds {MANIFEST} plus the artifacts it '
                f'names. See worlds/README.md.')
        return d

    d = os.path.join(r, name_or_path)
    if not os.path.isfile(os.path.join(d, MANIFEST)):
        raise RuntimeError(
            f"world: unknown world '{name_or_path}'\n"
            f'  registry: {r}\n'
            f'  available: {", ".join(available(r)) or "(none)"}\n'
            f'  world:= takes a registry name or a path to a world directory.\n'
            f'  If that list is empty the registry is probably not mounted; '
            f'check the worlds volume in docker-compose.yml or set {ENV_VAR}.')
    return d


class World:
    """A loaded world manifest, and where each backend's representation lives."""

    def __init__(self, directory, manifest):
        self.dir = directory
        self.manifest = manifest
        self.name = manifest['name']

    # -- loading ------------------------------------------------------------

    @classmethod
    def load(cls, name_or_path, worlds_root=None):
        d = locate(name_or_path, worlds_root)
        path = os.path.join(d, MANIFEST)
        with open(path) as f:
            m = yaml.safe_load(f) or {}

        name = m.get('name')
        if name != os.path.basename(d):
            raise RuntimeError(
                f'world: {path} declares name={name!r} but lives in '
                f'{os.path.basename(d)}/. They must match — the directory name '
                f'is what `world:=` and Gazebo\'s model:// URIs resolve on.')

        for backend, spec in (m.get('artifacts') or {}).items():
            if backend not in ('gazebo', 'isaacsim'):
                raise RuntimeError(
                    f'world: {path} has artifacts.{backend}; expected one of '
                    f'gazebo, isaacsim. (backend:=real loads no world file.)')
            mode = (spec or {}).get('mode', 'generated')
            if mode not in MODES:
                raise RuntimeError(
                    f'world: {path} has artifacts.{backend}.mode={mode!r}; '
                    f'expected one of {", ".join(MODES)}')
        return cls(d, m)

    # -- the manifest -------------------------------------------------------

    def artifact(self, backend):
        """The `artifacts.<backend>` block, with defaults filled in.

        Absent means generated, which is what a world written from the template
        in worlds/README.md gets: both backends generated from `bodies`.
        """
        spec = dict(((self.manifest.get('artifacts') or {}).get(backend)) or {})
        spec.setdefault('mode', 'generated')
        if spec['mode'] == 'generated':
            spec['path'] = {'gazebo': f'{self.name}.world',
                            'isaacsim': f'isaac/{self.name}.usd'}[backend]
        return spec

    @property
    def bodies(self):
        return self.manifest.get('bodies') or []

    @property
    def spawn(self):
        """Robot start pose as (x, y, z, yaw).

        Shared by every backend so that a Gazebo run, an Isaac Sim run and a
        real run of the same environment start from the same place; otherwise
        comparisons between them are not comparing anything.
        """
        s = self.manifest.get('spawn') or {}
        xyz = [float(v) for v in s.get('xyz', [0.0, 0.0, 0.0])]
        xyz += [0.0] * (3 - len(xyz))
        return xyz[0], xyz[1], xyz[2], float(s.get('yaw', 0.0))

    # -- per-backend representations ----------------------------------------

    def gazebo_world(self):
        """Path to the SDF world gzserver should load."""
        spec = self.artifact('gazebo')
        if spec['mode'] == 'none':
            raise RuntimeError(
                f"world: '{self.name}' declares artifacts.gazebo.mode=none, so "
                f'it cannot be run with backend:=gazebo.')
        path = os.path.join(self.dir, spec['path'])
        if not os.path.isfile(path):
            hint = (f'run: scripts/build_world.py {self.name}'
                    if spec['mode'] == 'generated' else
                    f'artifacts.gazebo.path points at {spec["path"]}, which is '
                    f'not there')
            raise RuntimeError(
                f"world: '{self.name}' has no {os.path.basename(path)} — {hint}.\n"
                f'  (gzserver\'s own failure for a missing world is to come up '
                f'with an empty scene and no error, which is unreadable.)')
        return path

    def isaac_usd(self):
        """What to pass turtlebot3_isaacsim as `world:=`.

        An empty string is meaningful, not an error: it means "no environment
        reference", and the simulator then authors its own ground plane and
        light. That is what `empty_stage` is.

        An adopted path may be an Isaac Sim asset-root path
        (/Isaac/Environments/...) or a URL, neither of which exists on this
        filesystem, so only local paths are checked.
        """
        spec = self.artifact('isaacsim')
        if spec['mode'] == 'none':
            return ''
        path = spec['path']
        if path.startswith(('/Isaac/', 'http://', 'https://', 'omniverse://')):
            return path
        path = os.path.join(self.dir, path)
        if spec['mode'] == 'generated' and not os.path.isfile(path):
            raise RuntimeError(
                f"world: '{self.name}' has no generated "
                f'{os.path.relpath(path, self.dir)} — run: '
                f'scripts/build_world.sh {self.name}\n'
                f'  (the USD is a build product and is not committed; it needs '
                f'the container.)')
        return path

    def isaac_world_z(self):
        """Metres to raise the Isaac stage by so its floor meets z = 0.

        Zero for anything authored floor-at-zero, which is every world this
        repository generates. Adopted stock environments are not obliged to
        agree: Simple_Room's floor is at -0.7696. It must match the value the
        occupancy-map builder is given, or the map is cut from the same room at
        a different height — which loads in Nav2 and localises the robot into a
        scene that is not there.
        """
        return float(self.artifact('isaacsim').get('world_z', 0.0))

    def map(self):
        """Path to the Nav2 map for this environment, or None.

        The map is the one artifact all three backends share, and for an
        environment cloned from a real room it is also the *input* the clone was
        made from — see worlds/README.md.
        """
        rel = self.manifest.get('map')
        if not rel:
            return None
        path = os.path.join(self.dir, rel)
        if not os.path.isfile(path):
            raise RuntimeError(
                f"world: '{self.name}' declares map: {rel}, which is not there "
                f'({path}).')
        return path
