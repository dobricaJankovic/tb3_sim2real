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

Handles the common export: `<triangles>` or `<polylist>` over a POSITION
source. Anything more exotic (multiple UV sets, skinning, node transforms in
the visual scene) is not interpreted, and the bounds print is how you find out.
"""

import argparse
import os
import sys
import xml.etree.ElementTree as ET

NS = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}


def read_dae(src):
    root = ET.parse(src).getroot()
    unit = root.find('.//c:asset/c:unit', NS)
    meter = float(unit.get('meter')) if unit is not None else 1.0
    up = root.findtext('.//c:asset/c:up_axis', default='Y_UP', namespaces=NS)

    nodes = root.findall('.//c:library_visual_scenes//c:node', NS)
    for n in nodes:
        for tag in ('matrix', 'rotate', 'translate', 'scale'):
            if n.find(f'c:{tag}', NS) is not None:
                print(f'  note: visual scene node carries a <{tag}>, which is '
                      f'NOT applied. Check the bounds below.', file=sys.stderr)

    verts, faces = [], []
    for g in root.findall('.//c:library_geometries/c:geometry', NS):
        mesh = g.find('c:mesh', NS)
        base = len(verts)

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
                        faces.append((base + poly[0], base + poly[j],
                                      base + poly[j + 1]))
            else:
                for i in range(0, len(idx), 3):
                    a, b, c = idx[i:i + 3]
                    faces.append((base + a, base + b, base + c))

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
