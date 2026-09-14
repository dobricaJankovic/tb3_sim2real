#!/usr/bin/env python3
"""Convert a Collada mesh to OBJ, baking its declared unit into the vertices.

    scripts/dae_to_obj.py worlds/my_world/meshes/room.dae

Isaac Sim's asset converter does **not** read Collada. On 6.1.0 it answers

    Unsupported import format: .dae. Supported formats: .bvh, .fbx, .glb,
    .gltf, .lxo, .md5, .obj, .ply, .stl, .usd, .usda, .usdc, .usdz

so a world whose meshes are `.dae` builds for Gazebo and cannot be built for
Isaac Sim at all. That is not a bug to work around: OBJ is the right shared
format anyway, because it carries no `<unit>` and no `up_axis` for the two
backends to interpret differently. See worlds/README.md, "Why OBJ".

WHAT THIS BAKES IN. Collada declares its unit (`<unit meter="0.0254"/>` for a
file authored in inches) and Gazebo applies it. OBJ has no such field, so the
factor is multiplied into the vertices here and the mesh means metres
afterwards. The manifest's `scale` is untouched and stays correct.

`up_axis` is deliberately NOT applied. turtlebot3_world's meshes declare
`Y_UP` while laying their vertices out Z-up, and Gazebo ignores the header —
so the vertices, not the label, are the parity truth. Copying them unrotated
is what keeps the two backends agreeing. Check the printed bounds against your
manifest's `verify` block before trusting the result; that is the only thing
that catches an axis or unit mistake, in either direction.

NODE TRANSFORMS ARE APPLIED. The visual scene is walked and each node's
`<matrix>`/`<translate>`/`<rotate>`/`<scale>` is composed down to the geometry
it instances. This used to be skipped with a warning, and the failure it caused
is the reason it no longer is: gazebo_models' `cafe_table.dae` instances its
tabletop under a separate `+29 inch` Z translate, so ignoring node transforms
produced a table with CORRECT OVERALL BOUNDS and the top lying on the floor —
geometry that is wrong in the one way a `verify` bounds check cannot catch.

Handles the common export: `<triangles>` or `<polylist>` over a POSITION
source, instanced from the visual scene directly or through `<library_nodes>`.
Skinning and morph targets are not interpreted; a file with no visual scene
falls back to emitting every geometry untransformed, with a note.
"""

import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET

NS = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}


def parse_geometry(mesh):
    """One <mesh> to (verts, faces) in its own coordinates."""
    verts, faces = [], []

    sources = {}
    for el in mesh.findall('c:source', NS):
        fa = el.find('c:float_array', NS)
        acc = el.find('c:technique_common/c:accessor', NS)
        sources[el.get('id')] = ([float(v) for v in fa.text.split()],
                                 int(acc.get('stride', '3')))

    vtag = mesh.find('c:vertices', NS)
    vid = vtag.find("c:input[@semantic='POSITION']", NS).get('source')[1:]
    pos, stride = sources[vid]
    for i in range(0, len(pos), stride):
        verts.append(tuple(pos[i:i + 3]))

    for prim in (mesh.findall('c:triangles', NS)
                 + mesh.findall('c:polylist', NS)):
        inputs = prim.findall('c:input', NS)
        nin = max(int(i.get('offset', '0')) for i in inputs) + 1
        voff = int(next(i for i in inputs
                        if i.get('semantic') == 'VERTEX').get('offset', '0'))
        idx = [int(v) for v in prim.find('c:p', NS).text.split()][voff::nin]
        if prim.tag.endswith('polylist'):
            counts = [int(v) for v in prim.find('c:vcount', NS).text.split()]
            k = 0
            for n in counts:
                poly = idx[k:k + n]
                k += n
                for j in range(1, n - 1):
                    faces.append((poly[0], poly[j], poly[j + 1]))
        else:
            for i in range(0, len(idx), 3):
                faces.append(tuple(idx[i:i + 3]))
    return verts, faces


# --- 4x4 row-major affine transforms, flat tuples, stdlib only ----------------

IDENTITY = (1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.)


def mat_mul(a, b):
    return tuple(sum(a[r * 4 + k] * b[k * 4 + c] for k in range(4))
                 for r in range(4) for c in range(4))


def mat_apply(m, p):
    x, y, z = p
    return (m[0] * x + m[1] * y + m[2] * z + m[3],
            m[4] * x + m[5] * y + m[6] * z + m[7],
            m[8] * x + m[9] * y + m[10] * z + m[11])


def node_matrix(node):
    """A node's own transform: its children composed in DOCUMENT ORDER.

    Collada applies a node's transform elements left to right, so a node with
    <translate> then <rotate> means translate * rotate — the rotate happens in
    the translated frame. Sorting them, or handling only the first, silently
    moves geometry.
    """
    m = IDENTITY
    for el in node:
        tag = el.tag.split('}')[-1]
        v = [float(x) for x in el.text.split()] if el.text and el.text.strip() else []
        if tag == 'matrix':
            m = mat_mul(m, tuple(v))
        elif tag == 'translate':
            m = mat_mul(m, (1., 0., 0., v[0], 0., 1., 0., v[1],
                            0., 0., 1., v[2], 0., 0., 0., 1.))
        elif tag == 'scale':
            m = mat_mul(m, (v[0], 0., 0., 0., 0., v[1], 0., 0.,
                            0., 0., v[2], 0., 0., 0., 0., 1.))
        elif tag == 'rotate':
            x, y, z, deg = v
            n = math.sqrt(x * x + y * y + z * z) or 1.0
            x, y, z = x / n, y / n, z / n
            c, si = math.cos(math.radians(deg)), math.sin(math.radians(deg))
            t = 1.0 - c
            m = mat_mul(m, (t * x * x + c, t * x * y - si * z, t * x * z + si * y, 0.,
                            t * x * y + si * z, t * y * y + c, t * y * z - si * x, 0.,
                            t * x * z - si * y, t * y * z + si * x, t * z * z + c, 0.,
                            0., 0., 0., 1.))
        elif tag == 'lookat':
            print('  note: <lookat> node transform is not applied.',
                  file=sys.stderr)
    return m


def read_dae(src):
    root = ET.parse(src).getroot()
    unit = root.find('.//c:asset/c:unit', NS)
    meter = float(unit.get('meter')) if unit is not None else 1.0
    up = root.findtext('.//c:asset/c:up_axis', default='Y_UP', namespaces=NS)

    geometries = {g.get('id'): parse_geometry(g.find('c:mesh', NS))
                  for g in root.findall('.//c:library_geometries/c:geometry', NS)}
    library_nodes = {n.get('id'): n
                     for n in root.findall('.//c:library_nodes//c:node', NS)}

    verts, faces = [], []

    def emit(geom_id, m):
        gv, gf = geometries.get(geom_id, ([], []))
        base = len(verts)
        verts.extend(mat_apply(m, v) for v in gv)
        faces.extend((base + a, base + b, base + c) for a, b, c in gf)

    def walk(node, parent, seen):
        m = mat_mul(parent, node_matrix(node))
        for el in node:
            tag = el.tag.split('}')[-1]
            if tag == 'instance_geometry':
                emit(el.get('url', '')[1:], m)
            elif tag == 'instance_node':
                ref = el.get('url', '')[1:]
                # A cycle here would recurse forever; Collada does not forbid
                # one and a broken exporter can write it.
                if ref in library_nodes and ref not in seen:
                    walk(library_nodes[ref], m, seen | {ref})
            elif tag == 'node':
                walk(el, m, seen)

    for scene in root.findall('.//c:library_visual_scenes/c:visual_scene', NS):
        for node in scene.findall('c:node', NS):
            walk(node, IDENTITY, frozenset())

    if not faces:
        # No visual scene, or one that instances nothing: fall back to the raw
        # geometry library so a bare mesh export still converts.
        print('  note: no geometry instanced from a visual scene; emitting '
              'library_geometries untransformed.', file=sys.stderr)
        for gid in geometries:
            emit(gid, IDENTITY)

    if not verts or not faces:
        raise SystemExit(f'dae_to_obj: no triangles found in {src}')
    return verts, faces, meter, up


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('dae')
    ap.add_argument('-o', '--output', help='default: the input with .obj')
    args = ap.parse_args()

    verts, faces, meter, up = read_dae(args.dae)
    dst = args.output or os.path.splitext(args.dae)[0] + '.obj'

    lo = [min(v[i] for v in verts) * meter for i in range(3)]
    hi = [max(v[i] for v in verts) * meter for i in range(3)]
    print(f'{args.dae}: unit meter={meter:g}, up_axis={up} (not applied), '
          f'{len(verts)} vertices, {len(faces)} triangles')
    print(f'  bounds min {[round(v, 4) for v in lo]} '
          f'max {[round(v, 4) for v in hi]}')

    with open(dst, 'w') as f:
        f.write(f'# GENERATED by scripts/dae_to_obj.py from '
                f'{os.path.basename(args.dae)}.\n')
        f.write(f'# <unit meter="{meter:g}"> is baked into the vertices; OBJ has '
                f'no unit field.\n')
        for v in verts:
            f.write('v %.6f %.6f %.6f\n' % tuple(c * meter for c in v))
        for a, b, c in faces:
            f.write('f %d %d %d\n' % (a + 1, b + 1, c + 1))
    print(f'wrote {dst}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
