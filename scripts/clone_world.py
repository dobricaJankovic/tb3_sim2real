#!/usr/bin/env python3
"""Turn a Nav2 map of a real room into a world both simulators can load.

    scripts/clone_world.py --map ~/maps/lab_room.yaml --name lab_room

The real environment is the reference, and the artefact that already describes
it is the one you had to make anyway: the occupancy map you drove around the
room to record with slam_toolbox or cartographer, and that Nav2 then localises
against. This reads it and writes a world directory:

    worlds/lab_room/
      world.yaml          the manifest, with ONE mesh body and the spawn pose
      meshes/lab_room.obj the walls, extruded from the occupied cells
      map/                the map it was built from, copied in

after which `scripts/build_world.sh lab_room` produces the Gazebo .world and
the Isaac Sim .usd from it, and `world:=lab_room` runs on all three backends
against the same room.

WHY THE MAP AND NOT A 3D SCAN. A phone scan or photogrammetry gives a
two-million-triangle non-manifold shell that neither ODE nor PhysX will collide
with usefully, and cleaning it up is a Blender afternoon. The occupancy map is
already metric, already aligned to the frame Nav2 works in, already a record of
exactly the geometry the robot's own sensor can see, and it costs nothing extra
because you need it regardless. What it cannot capture is anything the lidar
plane misses -- an overhang, a table top, a step -- so it is a floor plan, not a
model of the room. Add those as extra bodies in world.yaml by hand; the shape
of the manifest is the same either way.

WHY A MESH AND NOT BOXES. A traced outline extrudes to one OBJ that both
backends load byte-for-byte identical, which is the property the whole registry
is built on. Rectangle-decomposing the same grid into `box` bodies produces
hundreds of them for a real room, and two simulators each approximating the
same hundreds of boxes is more surface for them to disagree on, not less.

OBJ rather than Collada on purpose: Gazebo Classic reads dae/obj/stl and Isaac
Sim's converter reads obj/fbx/gltf, so OBJ is the intersection -- and unlike
Collada it carries no unit scale and no up-axis header for the two of them to
interpret differently. See worlds/README.md, "Two things that bite".

Pure stdlib plus PyYAML. Reading a PGM and walking a grid boundary does not
need OpenCV, and a dependency a student has to install is a step they can fail.
"""

import argparse
import os
import sys

import yaml

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'tb3_bringup'))

from tb3_bringup import worlds  # noqa: E402

TEMPLATE = """\
# {name} -- cloned from a real environment by scripts/clone_world.py.
#
# THIS FILE IS THE SOURCE OF TRUTH. The Gazebo .world and the Isaac Sim .usd are
# both generated from it:
#
#   scripts/build_world.sh {name}
#
# Edit this, then regenerate. Never hand-edit a generated file --
# scripts/check_worlds.py records this file's hash in the generated ones and
# fails when they stop matching.
#
# Cloned from: {map_src}
#   {cols} x {rows} cells at {resolution} m, origin {origin}
#   {n_tri} triangles, walls {height} m tall
#
# The mesh is the floor plan the robot's own lidar recorded, so it holds
# everything at beam height and nothing above or below it. Anything the beam
# misses -- a table top, an overhang, a step -- is invisible here and has to be
# added as a body of its own. Measure it and append:
#
#   - name: table
#     geometry: {{type: box, size: [1.2, 0.6, 0.75]}}
#     xyz: [1.0, 0.5, 0.375]
#     rpy: [0, 0, 0]

name: {name}
description: >-
  Cloned from an occupancy map of the real environment.

# Where the robot starts. Set this to where you actually put the robot in the
# real room: it is the pose all three backends spawn at, and the initial pose to
# hand AMCL.
spawn:
  xyz: [{spawn_x:g}, {spawn_y:g}, 0.01]
  yaw: 0.0

# The map this world was cloned FROM, and the one Nav2 uses on every backend.
# Copied in rather than referenced, so the world directory is self-contained and
# the map cannot be edited out from under the geometry that was traced from it.
map: map/{map_name}

# Checked against the assembled Isaac Sim stage, which is the one place a mesh
# can come out at the wrong scale or on its side without anything reporting it.
verify:
  bounds_min: [{xmin:.4f}, {ymin:.4f}, 0.0]
  bounds_max: [{xmax:.4f}, {ymax:.4f}, {height:.4f}]
  tolerance: 0.05

bodies:
  - name: shell
    geometry: {{type: mesh, uri: meshes/{name}.obj, scale: [1, 1, 1]}}
    xyz: [0, 0, 0]
    rpy: [0, 0, 0]
"""


def read_pgm(path):
    """Minimal binary/ASCII PGM reader. Returns (cols, rows, maxval, pixels)."""
    with open(path, 'rb') as f:
        data = f.read()

    def tokens():
        i = 0
        while i < len(data):
            c = data[i:i + 1]
            if c == b'#':
                while i < len(data) and data[i:i + 1] != b'\n':
                    i += 1
            elif c.isspace():
                i += 1
            else:
                j = i
                while j < len(data) and not data[j:j + 1].isspace():
                    j += 1
                yield data[i:j], j
                i = j

    it = tokens()
    magic, _ = next(it)
    cols, _ = next(it)
    rows, _ = next(it)
    maxval, end = next(it)
    cols, rows, maxval = int(cols), int(rows), int(maxval)

    if magic == b'P5':
        if maxval > 255:
            raise SystemExit('clone_world: 16-bit PGM not supported')
        pix = list(data[end + 1:end + 1 + cols * rows])
    elif magic == b'P2':
        pix = [int(t) for t, _ in it]
    else:
        raise SystemExit(f'clone_world: {path} is not a PGM (magic {magic!r})')

    if len(pix) < cols * rows:
        raise SystemExit(
            f'clone_world: {path} is truncated: {len(pix)} of {cols * rows} pixels')
    return cols, rows, maxval, pix[:cols * rows]


def classify(map_yaml):
    """Read a Nav2 map into occupied/free grids, plus its metadata.

    Returns (occupied, free, cols, rows, resolution, origin). Both grids are
    indexed `[r][c]` with r counting UP from the map origin, which is the
    map_server convention: the .pgm's first row is the TOP of the image, i.e.
    the highest y, so the rows are reversed here once and never again.

    Occupied and free are not complements. The third state is UNKNOWN -- where
    the robot never looked -- and keeping it distinct is what lets a caller
    tell "the model has a wall the map does not" (a contradiction, in free
    space) from "the model has a wall the map never saw" (not a contradiction,
    in unknown space).
    """
    with open(map_yaml) as f:
        meta = yaml.safe_load(f)

    img = os.path.join(os.path.dirname(os.path.abspath(map_yaml)), meta['image'])
    cols, rows, maxval, pix = read_pgm(img)

    resolution = float(meta['resolution'])
    origin = [float(v) for v in meta['origin']]
    negate = int(meta.get('negate', 0))
    occupied_thresh = float(meta.get('occupied_thresh', 0.65))

    free_thresh = float(meta.get('free_thresh', 0.196))

    # map_server's rule: p = (maxval - value) / maxval, inverted if negate.
    # p above occupied_thresh is an obstacle, p below free_thresh is free, and
    # what is left is unknown. Unknown is deliberately not a wall: the grey
    # border round a SLAM map is where the robot never looked, and extruding it
    # would box the room in with geometry that is not there.
    def p_of(v):
        return v / maxval if negate else (maxval - v) / maxval

    occ = [[False] * cols for _ in range(rows)]
    free = [[False] * cols for _ in range(rows)]
    for r in range(rows):
        row = (rows - 1 - r) * cols
        for c in range(cols):
            p = p_of(pix[row + c])
            if p > occupied_thresh:
                occ[r][c] = True
            elif p < free_thresh:
                free[r][c] = True
    return occ, free, cols, rows, resolution, origin


def occupancy(map_yaml):
    """Just the occupied grid. See classify()."""
    occ, _free, cols, rows, resolution, origin = classify(map_yaml)
    return occ, cols, rows, resolution, origin


def despeckle(grid, cols, rows, min_neighbours):
    """Drop isolated occupied cells.

    A SLAM map has salt: single cells lit by one bad return. Each one becomes a
    free-standing pillar in the mesh that the planner then refuses to path
    through. An occupied cell with fewer than `min_neighbours` occupied
    neighbours in its 8-neighbourhood is noise, not a wall.
    """
    if min_neighbours <= 0:
        return grid
    out = [row[:] for row in grid]
    for r in range(rows):
        for c in range(cols):
            if not grid[r][c]:
                continue
            n = sum(grid[r + dr][c + dc]
                    for dr in (-1, 0, 1) for dc in (-1, 0, 1)
                    if (dr or dc) and 0 <= r + dr < rows and 0 <= c + dc < cols)
            if n < min_neighbours:
                out[r][c] = False
    return out


def extrude(grid, cols, rows, resolution, origin, height):
    """Extrude occupied cells into a watertight OBJ, culling interior faces.

    Every occupied cell is a box, but a face is only emitted where the cell
    borders something that is not occupied. A solid block of wall therefore
    costs its surface rather than its volume, which is the difference between
    a few thousand triangles for a room and a few hundred thousand.

    Returns (obj_text, n_triangles, (xmin, ymin, xmax, ymax)).
    """
    ox, oy = origin[0], origin[1]
    verts = {}
    faces = []

    def v(x, y, z):
        key = (round(x, 6), round(y, 6), round(z, 6))
        if key not in verts:
            verts[key] = len(verts) + 1        # OBJ indices are 1-based
        return verts[key]

    def quad(a, b, c, d):
        faces.append((a, b, c))
        faces.append((a, c, d))

    def occ(r, c):
        return 0 <= r < rows and 0 <= c < cols and grid[r][c]

    for r in range(rows):
        for c in range(cols):
            if not grid[r][c]:
                continue
            x0, y0 = ox + c * resolution, oy + r * resolution
            x1, y1 = x0 + resolution, y0 + resolution
            # top; the bottom face is never emitted -- it is against the floor
            quad(v(x0, y0, height), v(x1, y0, height),
                 v(x1, y1, height), v(x0, y1, height))
            if not occ(r, c - 1):
                quad(v(x0, y0, 0), v(x0, y1, 0), v(x0, y1, height), v(x0, y0, height))
            if not occ(r, c + 1):
                quad(v(x1, y1, 0), v(x1, y0, 0), v(x1, y0, height), v(x1, y1, height))
            if not occ(r - 1, c):
                quad(v(x1, y0, 0), v(x0, y0, 0), v(x0, y0, height), v(x1, y0, height))
            if not occ(r + 1, c):
                quad(v(x0, y1, 0), v(x1, y1, 0), v(x1, y1, height), v(x0, y1, height))

    if not faces:
        raise SystemExit(
            'clone_world: the map has no occupied cells above the threshold.\n'
            '  Check occupied_thresh and negate in the map yaml, and that you '
            'passed the map and not an empty one.')

    order = sorted(verts.items(), key=lambda kv: kv[1])
    lines = [
        '# GENERATED by scripts/clone_world.py from an occupancy map.',
        '# Regenerate rather than edit; see the world.yaml beside it.',
    ]
    lines += [f'v {x:.4f} {y:.4f} {z:.4f}' for (x, y, z), _ in order]
    lines += [f'f {a} {b} {c}' for a, b, c in faces]
    lines.append('')

    xs = [x for (x, _, _), _ in order]
    ys = [y for (_, y, _), _ in order]
    return '\n'.join(lines), len(faces), (min(xs), min(ys), max(xs), max(ys))


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--map', required=True,
                    help='Nav2 map yaml recorded in the real environment')
    ap.add_argument('--name', required=True, help='world name (a directory name)')
    ap.add_argument('--into', default=None,
                    help='where to write the world directory; default is the '
                         'registry root')
    ap.add_argument('--height', type=float, default=1.0,
                    help='wall height in metres (default 1.0; only the lidar '
                         'plane at ~0.18 m is measured, the rest is for looks)')
    ap.add_argument('--despeckle', type=int, default=2, metavar='N',
                    help='drop occupied cells with fewer than N occupied '
                         'neighbours (default 2; 0 disables)')
    ap.add_argument('--spawn', nargs=2, type=float, default=(0.0, 0.0),
                    metavar=('X', 'Y'),
                    help='where the robot starts, in map coordinates')
    ap.add_argument('--force', action='store_true',
                    help='overwrite an existing world directory')
    args = ap.parse_args()

    grid, cols, rows, resolution, origin = occupancy(args.map)
    grid = despeckle(grid, cols, rows, args.despeckle)
    obj, n_tri, (xmin, ymin, xmax, ymax) = extrude(
        grid, cols, rows, resolution, origin, args.height)

    out = os.path.join(args.into or worlds.root(), args.name)
    if os.path.exists(out) and not args.force:
        raise SystemExit(f'clone_world: {out} exists; pass --force to overwrite')
    os.makedirs(os.path.join(out, 'meshes'), exist_ok=True)
    os.makedirs(os.path.join(out, 'map'), exist_ok=True)

    with open(os.path.join(out, 'meshes', f'{args.name}.obj'), 'w') as f:
        f.write(obj)

    # The map is copied, not referenced: a world directory that carries the map
    # it was traced from is one you can hand to someone else, and one where the
    # geometry and the map cannot be edited apart.
    map_name = os.path.basename(args.map)
    with open(args.map) as f:
        meta = yaml.safe_load(f)
    src_img = os.path.join(os.path.dirname(os.path.abspath(args.map)), meta['image'])
    with open(src_img, 'rb') as a, \
            open(os.path.join(out, 'map', os.path.basename(src_img)), 'wb') as b:
        b.write(a.read())
    with open(os.path.join(out, 'map', map_name), 'w') as f:
        yaml.safe_dump(meta, f, default_flow_style=None, sort_keys=False)

    with open(os.path.join(out, worlds.MANIFEST), 'w') as f:
        f.write(TEMPLATE.format(
            name=args.name, map_src=os.path.abspath(args.map), map_name=map_name,
            cols=cols, rows=rows, resolution=resolution,
            origin=[round(v, 4) for v in origin],
            n_tri=n_tri, height=args.height,
            spawn_x=args.spawn[0], spawn_y=args.spawn[1],
            xmin=xmin, ymin=ymin, xmax=xmax, ymax=ymax))

    occ_cells = sum(sum(row) for row in grid)
    print(f'{args.name}: {occ_cells} occupied cells -> {n_tri} triangles, '
          f'{xmax - xmin:.2f} x {ymax - ymin:.2f} m')
    print(f'wrote {out}')
    print(f'next: scripts/build_world.sh {args.name}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
