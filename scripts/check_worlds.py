#!/usr/bin/env python3
"""Prove that a world's representations still agree with its manifest.

    scripts/check_worlds.py            # every world in the registry
    scripts/check_worlds.py lab_room
    scripts/check_worlds.py --quiet    # exit status only, for a hook or CI

Nothing in this repository can stop someone editing a generated file. What it
can do is make that impossible to miss, which is what this checks:

  provenance  every generated artifact carries the sha256 of the world.yaml it
              came from -- in the .world's XML header, in the USD's
              customLayerData. A manifest edited without a regenerate, or a
              generated file edited by hand, fails here.
  round trip  the generated .world is parsed back and compared body by body
              against the manifest. That catches the hand-edit that kept the
              header.
  footprint   where the geometry can be rasterised without a mesh library, the
              manifest is cut at lidar height and compared against the world's
              own Nav2 map. This is the leg nothing else covers: whether the
              model still describes the room the robot actually drove around.

The Isaac Sim stage is not parsed here -- reading USD needs Kit or usd-core,
and neither is a dependency worth adding to a check that should run in a
pre-commit hook. It is covered instead at the moment it is written:
build_world_usd.py asserts the assembled stage's bounds against the manifest's
`verify` block and refuses to write a stage whose colliders are wrong. What is
checked here is that the stage on disk was built from THIS manifest.

Stdlib and PyYAML only, deliberately: a check that needs installing is a check
that gets skipped.
"""

import argparse
import hashlib
import math
import os
import re
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'tb3_bringup'))

from tb3_bringup import worlds  # noqa: E402

#: Height of the burger's LDS above base_footprint. The footprint check cuts
#: the world at exactly the plane the map was recorded at.
LIDAR_Z = 0.182

#: A wall one cell thick in the map is a surface with two sides in the model,
#: and the two land in neighbouring cells. The comparison is therefore made
#: with one cell of slack in both directions: the question is whether the walls
#: are in the same PLACE, not whether a rasteriser rounded them the same way.
SLACK = 1

#: Below these, the manifest and the map are describing different rooms. Not
#: tight, deliberately -- a real map has furniture that moved, a doorway that
#: was open once, and a grey rim where the robot never looked.
COVERED_MIN = 0.85
SPURIOUS_MAX = 0.25


class Result:
    def __init__(self, name):
        self.name = name
        self.lines = []
        self.failed = False

    def ok(self, msg):
        self.lines.append(('ok', msg))

    def note(self, msg):
        self.lines.append(('--', msg))

    def fail(self, msg):
        self.lines.append(('FAIL', msg))
        self.failed = True


# --- provenance --------------------------------------------------------------

def digest(path):
    with open(path, 'rb') as f:
        return hashlib.sha256(f.read()).hexdigest()


def check_provenance(world, res):
    want = digest(os.path.join(world.dir, worlds.MANIFEST))

    spec = world.artifact('gazebo')
    if spec['mode'] == 'generated':
        path = os.path.join(world.dir, spec['path'])
        if not os.path.isfile(path):
            res.fail(f'{spec["path"]} is missing; run scripts/build_world.sh '
                     f'{world.name}')
        else:
            with open(path) as f:
                head = f.read(2048)
            m = re.search(r'world\.yaml sha256: ([0-9a-f]{64})', head)
            if not m:
                res.fail(f'{spec["path"]} carries no provenance line — it '
                         f'predates the stamp, or it was hand-written')
            elif m.group(1) != want:
                res.fail(f'{spec["path"]} was generated from a different '
                         f'world.yaml ({m.group(1)[:12]} != {want[:12]}); '
                         f'run scripts/build_world.sh {world.name}')
            else:
                res.ok(f'{spec["path"]} matches world.yaml')

    spec = world.artifact('isaacsim')
    if spec['mode'] == 'generated':
        path = os.path.join(world.dir, spec['path'])
        if not os.path.isfile(path):
            # Not a failure: the USD is a build product that is never committed,
            # so a fresh checkout has none and the Gazebo backend does not care.
            res.note(f'{spec["path"]} not built (it is gitignored; '
                     f'scripts/build_world.sh {world.name} makes it)')
        else:
            # The digest is stored as customLayerData. Rather than depend on a
            # USD reader, look for it in the bytes: crusd or usda, the string is
            # stored literally either way.
            with open(path, 'rb') as f:
                blob = f.read()
            if want.encode() in blob:
                res.ok(f'{spec["path"]} matches world.yaml')
            else:
                res.fail(f'{spec["path"]} was built from a different '
                         f'world.yaml; run scripts/build_world.sh {world.name}')


# --- round trip --------------------------------------------------------------

def sdf_bodies(path):
    """Body name -> (pose, geometry) as the generated .world actually holds it."""
    root = ET.parse(path).getroot()
    out = {}
    for col in root.iter('collision'):
        pose = [float(v) for v in (col.findtext('pose') or '0 0 0 0 0 0').split()]
        geom = col.find('geometry')
        kind = next(iter(geom), None)
        if kind is None:
            continue
        g = {'type': kind.tag}
        if kind.tag == 'cylinder':
            g['radius'] = float(kind.findtext('radius'))
            g['length'] = float(kind.findtext('length'))
        elif kind.tag == 'box':
            g['size'] = [float(v) for v in kind.findtext('size').split()]
        elif kind.tag == 'sphere':
            g['radius'] = float(kind.findtext('radius'))
        elif kind.tag == 'mesh':
            g['uri'] = kind.findtext('uri')
            g['scale'] = [float(v) for v in (kind.findtext('scale') or '1 1 1').split()]
        out[col.get('name')] = (pose, g)
    return out


def close(a, b, tol=1e-6):
    return all(abs(float(x) - float(y)) <= tol for x, y in zip(a, b))


def check_roundtrip(world, res):
    spec = world.artifact('gazebo')
    if spec['mode'] != 'generated':
        res.note(f'artifacts.gazebo.mode={spec["mode"]}; round trip skipped')
        return
    path = os.path.join(world.dir, spec['path'])
    if not os.path.isfile(path):
        return

    try:
        got = sdf_bodies(path)
    except ET.ParseError as e:
        res.fail(f'{spec["path"]} is not parseable XML: {e}')
        return

    problems = []
    for body in world.bodies:
        name = body['name']
        if name not in got:
            problems.append(f'{name}: in the manifest, not in the .world')
            continue
        pose, g = got.pop(name)
        want_pose = list(body.get('xyz', [0, 0, 0])) + list(body.get('rpy', [0, 0, 0]))
        if not close(pose, want_pose, 1e-4):
            problems.append(f'{name}: pose {pose} != manifest {want_pose}')
        wg = body['geometry']
        if g['type'] != wg['type']:
            problems.append(f'{name}: {g["type"]} != manifest {wg["type"]}')
        elif g['type'] == 'mesh':
            if not g['uri'].endswith(wg['uri']):
                problems.append(f'{name}: mesh {g["uri"]} != manifest {wg["uri"]}')
            if not close(g['scale'], wg.get('scale', [1, 1, 1]), 1e-4):
                problems.append(f'{name}: scale {g["scale"]} != manifest')
        elif g['type'] == 'box':
            if not close(g['size'], wg['size'], 1e-4):
                problems.append(f'{name}: size {g["size"]} != manifest {wg["size"]}')
        elif g['type'] == 'cylinder':
            if not close([g['radius'], g['length']], [wg['radius'], wg['length']], 1e-4):
                problems.append(f'{name}: cylinder != manifest')
        elif g['type'] == 'sphere':
            if not close([g['radius']], [wg['radius']], 1e-4):
                problems.append(f'{name}: radius != manifest')
    for name in got:
        problems.append(f'{name}: in the .world, not in the manifest')

    if problems:
        res.fail(f'{spec["path"]} does not round-trip:\n      '
                 + '\n      '.join(problems))
    else:
        res.ok(f'{spec["path"]} round-trips: {len(world.bodies)} bodies')


# --- footprint ---------------------------------------------------------------

def obj_triangles(path, scale):
    verts, tris = [], []
    sx, sy, sz = [float(s) for s in scale]
    with open(path) as f:
        for line in f:
            if line.startswith('v '):
                x, y, z = line.split()[1:4]
                verts.append((float(x) * sx, float(y) * sy, float(z) * sz))
            elif line.startswith('f '):
                idx = [int(tok.split('/')[0]) for tok in line.split()[1:]]
                idx = [i - 1 if i > 0 else len(verts) + i for i in idx]
                for k in range(1, len(idx) - 1):
                    tris.append((verts[idx[0]], verts[idx[k]], verts[idx[k + 1]]))
    return tris


def slice_segments(tris, z):
    """Intersect triangles with the plane z, as 2D segments."""
    segs = []
    for tri in tris:
        pts = []
        for a, b in ((0, 1), (1, 2), (2, 0)):
            p, q = tri[a], tri[b]
            if (p[2] - z) * (q[2] - z) > 0 or p[2] == q[2]:
                continue
            t = (z - p[2]) / (q[2] - p[2])
            pts.append((p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1])))
        if len(pts) >= 2:
            segs.append((pts[0], pts[1]))
    return segs


def rasterise(world, res):
    """Occupied-cell set of the manifest at LIDAR_Z, or None if not possible."""
    grid = set()
    step = None

    def stamp(x, y):
        grid.add((int(math.floor(x / step)), int(math.floor(y / step))))

    step = 0.05
    for body in world.bodies:
        g = body['geometry']
        x, y, z = [float(v) for v in body.get('xyz', [0, 0, 0])]
        rpy = [float(v) for v in body.get('rpy', [0, 0, 0])]
        if g['type'] in ('cylinder', 'sphere') and any(abs(a) > 1e-6 for a in rpy[:2]):
            return None                       # tilted: not a 2D footprint
        if g['type'] == 'cylinder':
            h = float(g['length'])
            if not (z - h / 2 <= LIDAR_Z <= z + h / 2):
                continue
            r = float(g['radius'])
            n = max(8, int(2 * math.pi * r / step))
            for i in range(n):
                a = 2 * math.pi * i / n
                stamp(x + r * math.cos(a), y + r * math.sin(a))
        elif g['type'] == 'sphere':
            r = float(g['radius'])
            d = abs(LIDAR_Z - z)
            if d >= r:
                continue
            rr = math.sqrt(r * r - d * d)
            n = max(8, int(2 * math.pi * rr / step))
            for i in range(n):
                a = 2 * math.pi * i / n
                stamp(x + rr * math.cos(a), y + rr * math.sin(a))
        elif g['type'] == 'box':
            sx, sy, sz = [float(v) for v in g['size']]
            if not (z - sz / 2 <= LIDAR_Z <= z + sz / 2):
                continue
            ca, sa = math.cos(rpy[2]), math.sin(rpy[2])
            n = max(4, int(max(sx, sy) / step) * 2)
            for i in range(n + 1):
                for u, v in ((-sx / 2 + sx * i / n, -sy / 2),
                             (-sx / 2 + sx * i / n, sy / 2),
                             (-sx / 2, -sy / 2 + sy * i / n),
                             (sx / 2, -sy / 2 + sy * i / n)):
                    stamp(x + u * ca - v * sa, y + u * sa + v * ca)
        elif g['type'] == 'mesh':
            uri = g['uri']
            if not uri.lower().endswith('.obj'):
                return None                   # would need a mesh library
            if any(abs(a) > 1e-6 for a in rpy):
                return None                   # rotated mesh: not handled here
            tris = obj_triangles(os.path.join(world.dir, uri),
                                 g.get('scale', [1, 1, 1]))
            for (ax, ay), (bx, by) in slice_segments(tris, LIDAR_Z - z):
                n = max(1, int(math.hypot(bx - ax, by - ay) / (step / 2)))
                for i in range(n + 1):
                    stamp(x + ax + (bx - ax) * i / n, y + ay + (by - ay) * i / n)
        else:
            return None
    return grid


def map_cells(map_yaml, step=0.05):
    import clone_world                       # same directory; reuse its reader
    grid, cols, rows, resolution, origin = clone_world.occupancy(map_yaml)
    cells = set()
    for r in range(rows):
        for c in range(cols):
            if grid[r][c]:
                x = origin[0] + (c + 0.5) * resolution
                y = origin[1] + (r + 0.5) * resolution
                cells.add((int(math.floor(x / step)), int(math.floor(y / step))))
    return cells


def near(a, b, slack=SLACK):
    """How many cells of `a` have a cell of `b` within `slack`."""
    offsets = [(dx, dy) for dx in range(-slack, slack + 1)
               for dy in range(-slack, slack + 1)]
    return sum(1 for (x, y) in a
               if any((x + dx, y + dy) in b for dx, dy in offsets))


def check_footprint(world, res):
    try:
        map_yaml = world.map()
    except RuntimeError as e:
        res.fail(str(e))
        return
    if not map_yaml:
        res.note('no map declared; footprint check skipped')
        return

    model = rasterise(world, res)
    if model is None:
        res.note('geometry needs a mesh library to rasterise; footprint check '
                 'skipped (the Isaac stage bounds check covers scale and axes)')
        return

    recorded = map_cells(map_yaml)
    if not recorded or not model:
        res.note('nothing to compare')
        return

    covered = near(recorded, model) / len(recorded)
    spurious = 1.0 - near(model, recorded) / len(model)
    msg = (f'footprint vs {os.path.relpath(map_yaml, world.dir)}: '
           f'{covered:.0%} of the map is modelled, {spurious:.0%} of the model '
           f'is not in the map ({len(model)} model / {len(recorded)} map cells '
           f'at {LIDAR_Z} m)')
    if covered < COVERED_MIN or spurious > SPURIOUS_MAX:
        res.fail(msg + '\n      The model and the map disagree about where the '
                       'walls are. A mirrored or offset map is the usual cause; '
                       "check the map yaml's origin.")
    else:
        res.ok(msg)


# --- driver ------------------------------------------------------------------

def check(name):
    res = Result(name)
    try:
        world = worlds.World.load(name)
    except RuntimeError as e:
        res.fail(str(e))
        return res

    for body in world.bodies:
        g = body['geometry']
        if g['type'] == 'mesh':
            p = os.path.join(world.dir, g['uri'])
            if not os.path.isfile(p):
                res.fail(f'{body["name"]}: mesh {g["uri"]} is missing')

    check_provenance(world, res)
    check_roundtrip(world, res)
    check_footprint(world, res)
    return res


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('world', nargs='*', help='worlds to check; default is all')
    ap.add_argument('--quiet', action='store_true', help='exit status only')
    args = ap.parse_args()

    names = args.world or worlds.available()
    if not names:
        print(f'check_worlds: no worlds in {worlds.root()}', file=sys.stderr)
        return 1

    failed = 0
    for name in names:
        res = check(name)
        failed += bool(res.failed)
        if args.quiet:
            if res.failed:
                print(f'{res.name}: FAILED')
                for tag, line in res.lines:
                    if tag == 'FAIL':
                        print(f'    {line}')
            continue
        print(f'{res.name}')
        for tag, line in res.lines:
            print(f'  {tag:>4}  {line}')
        print()

    print(f'check_worlds: {len(names) - failed}/{len(names)} worlds consistent')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
