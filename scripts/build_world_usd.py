"""Generate an Isaac Sim .usd for a world from the SAME manifest Gazebo uses.

    scripts/build_world_usd.sh turtlebot3_world      # from the host

Reads /worlds/<name>/world.yaml and writes /worlds/<name>/isaac/<name>.usd. Use
the wrapper rather than calling this directly: it creates the output directory
on the host, so it stays yours rather than root's. The Gazebo counterpart is
scripts/build_world.py, which reads the same manifest and the same meshes; that
shared source is the only reason the two backends stay in agreement. See
worlds/README.md.

Isaac Sim ships no SDF importer (only URDF, MJCF and heightmap — checked in
/isaac-sim/exts), which is why this authors USD directly rather than trying to
consume the generated .world.

Meshes go through omni.kit.asset_converter, which is bundled in the image with
libassimp behind it. Converted results are cached in <world>/isaac/_converted/ and
reused, since conversion is the slow part and the meshes rarely change.

Two things this script is careful about, both of which fail silently otherwise:

1. COLLIDERS ARE EXACT, NEVER CONVEX. Every body is static, so triangle-mesh
   collision is legal. The turtlebot3_world wall is a thin hexagonal shell whose
   convex hull is a solid prism — hulling it seals the robot inside the arena at
   spawn, and nothing reports a problem.

2. CONVERTED BOUNDS ARE VERIFIED, NOT TRUSTED. Collada carries a unit scale and
   an up-axis, and these meshes declare `inch` and `Y_UP` while actually being
   laid out Z-up. Gazebo draws them upright. A converter honouring Y_UP would
   tip the Isaac arena on its side while Gazebo stayed correct — a divergence
   between backends with no error anywhere. The manifest's `verify.bounds_*` is
   checked against the assembled stage, and a mismatch is fatal.
"""

import argparse
import asyncio
import os
import sys
import traceback

import yaml

from isaacsim import SimulationApp

# Headless is correct here: this produces an asset and renders nothing. (Note
# the contrast with tb3_sim.py, where --headless silently kills the RTX lidar.)
sim_app = SimulationApp({'headless': True})

import omni.kit.asset_converter  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage, create_new_stage  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdPhysics  # noqa: E402

WORLDS_ROOT = os.environ.get('TB3_WORLDS_DIR', '/worlds')
ROOT_PRIM = '/World'


def load_manifest(name):
    path = os.path.join(WORLDS_ROOT, name, 'world.yaml')
    if not os.path.isfile(path):
        avail = [n for n in sorted(os.listdir(WORLDS_ROOT))
                 if os.path.isfile(os.path.join(WORLDS_ROOT, n, 'world.yaml'))] \
            if os.path.isdir(WORLDS_ROOT) else []
        raise SystemExit(
            f'build_world_usd: no manifest at {path}\n'
            f'  registry: {WORLDS_ROOT} (available: {", ".join(avail) or "(none)"})\n'
            f'  If that is empty, the worlds volume is not mounted.')
    with open(path) as f:
        m = yaml.safe_load(f)
    if m.get('name') != name:
        raise SystemExit(
            f'build_world_usd: {path} declares name={m.get("name")!r} but lives '
            f'in worlds/{name}/. They must match.')
    return m


async def _convert(src, dst):
    ctx = omni.kit.asset_converter.AssetConverterContext()
    # Keep materials: the arena should look like the Gazebo one, and the RTX
    # lidar traces render geometry, so visuals are not merely cosmetic here.
    ctx.ignore_materials = False
    ctx.ignore_animation = True
    ctx.ignore_cameras = True
    # Label the converted stage Z-up. Measured behaviour (isaac/scripts probe,
    # asset_converter 6.0.1): this flag moves NO geometry — bounds are identical
    # with and without it — it only sets the stage's upAxis metadata.
    #
    # That is exactly what is needed here. These Collada files declare
    # `up_axis Y_UP` but lay their vertices out Z-up, and the converter faithfully
    # copies both the vertices and the (wrong) label. A stage labelled Y-up then
    # makes add_reference_to_stage insert a corrective 90-degree X rotation when
    # referencing into our Z-up world, tipping the arena on its side while Gazebo
    # — which ignores the label — stays upright. Correcting the label removes the
    # correction.
    ctx.convert_stage_up_z = True
    task = omni.kit.asset_converter.get_instance().create_converter_task(
        src, dst, lambda *_: None, ctx)
    ok = await task.wait_until_finished()
    if not ok:
        raise SystemExit(
            f'build_world_usd: converting {src} failed: '
            f'{task.get_status()} {task.get_error_message()}')


def out_dir(world_dir):
    """Where this script's build products go: <world>/isaac/.

    A dedicated gitignored subdirectory, so everything this container writes as
    root is confined to one place and the manifest and meshes keep normal
    permissions. scripts/build_world_usd.sh creates it host-side, which is why
    this only checks.
    """
    d = os.path.join(world_dir, 'isaac')
    if not os.path.isdir(d):
        raise SystemExit(
            f'build_world_usd: {d} does not exist.\n'
            f'  Run the wrapper instead, which creates it on the host first so '
            f'it does not end up owned by root:\n'
            f'    scripts/build_world_usd.sh {os.path.basename(world_dir)}')
    return d


def convert_mesh(world_dir, rel_uri):
    """Convert one mesh to USD, cached. Returns the converted .usd path."""
    src = os.path.join(world_dir, rel_uri)
    if not os.path.isfile(src):
        raise SystemExit(f'build_world_usd: manifest references missing mesh {src}')
    cache = os.path.join(out_dir(world_dir), '_converted')
    os.makedirs(cache, exist_ok=True)
    # This container is root and the checkout is yours, so a default-mode cache
    # directory is one you cannot clear. That matters more than it sounds: the
    # cache is keyed on mtime only, so a stale entry is reused silently after a
    # converter FLAG changes, and the rebuild you just ran was not the rebuild
    # you thought. Keep it clearable from the host.
    try:
        os.chmod(cache, 0o777)
    except OSError:
        pass
    dst = os.path.join(cache, os.path.splitext(os.path.basename(rel_uri))[0] + '.usd')

    if os.path.isfile(dst) and os.path.getmtime(dst) >= os.path.getmtime(src):
        print(f'  cached  {rel_uri} -> {os.path.relpath(dst, world_dir)}')
        return dst
    print(f'  convert {rel_uri} -> {os.path.relpath(dst, world_dir)}')
    asyncio.get_event_loop().run_until_complete(_convert(src, dst))
    return dst


def place(stage, prim_path, xyz, rpy, scale=None):
    """Apply an SDF-style pose to a prim.

    SDF rpy is extrinsic XYZ (R = Rz*Ry*Rx), which is exactly USD's default
    XformCommonAPI rotation order, so the triple carries across unchanged apart
    from radians -> degrees.
    """
    xf = UsdGeom.XformCommonAPI(stage.GetPrimAtPath(prim_path))
    deg = [float(a) * 180.0 / 3.14159265358979 for a in rpy]

    # Every one of these returns False rather than raising when it cannot author
    # the op — which is what happens on a prim that already carries an `orient`
    # from a reference. Unchecked, the body just quietly sits at the origin at
    # the wrong scale, and the only symptom is a world that looks subtly wrong.
    ok = {
        'translate': xf.SetTranslate(Gf.Vec3d(*[float(v) for v in xyz])),
        'rotate': xf.SetRotate(Gf.Vec3f(*deg),
                               UsdGeom.XformCommonAPI.RotationOrderXYZ),
    }
    if scale is not None:
        ok['scale'] = xf.SetScale(Gf.Vec3f(*[float(s) for s in scale]))

    failed = [k for k, v in ok.items() if not v]
    if failed:
        raise SystemExit(
            f'build_world_usd: could not author {", ".join(failed)} on '
            f'{prim_path} (XformCommonAPI refused). This happens when the prim '
            f'already has xformOps from a reference — author the pose on a '
            f'wrapper Xform and put the reference on a child.')


def add_collider(prim, exact):
    """Static collider. Never a rigid body — every manifest body is static.

    `exact` selects triangle-mesh collision, which is legal precisely because
    nothing here moves. The alternative, convexHull, is what seals the robot
    inside a hollow wall.
    """
    UsdPhysics.CollisionAPI.Apply(prim)
    if exact:
        mesh_api = UsdPhysics.MeshCollisionAPI.Apply(prim)
        mesh_api.CreateApproximationAttr().Set('none')


def build(name):
    world_dir = os.path.join(WORLDS_ROOT, name)
    manifest = load_manifest(name)

    print(f'building {name} from {world_dir}/world.yaml')
    converted = {}
    for body in manifest['bodies']:
        g = body['geometry']
        if g['type'] == 'mesh' and g['uri'] not in converted:
            converted[g['uri']] = convert_mesh(world_dir, g['uri'])

    create_new_stage()
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(stage, ROOT_PRIM)
    stage.SetDefaultPrim(stage.GetPrimAtPath(ROOT_PRIM))

    for body in manifest['bodies']:
        g = body['geometry']
        path = f'{ROOT_PRIM}/{body["name"]}'
        xyz, rpy = body.get('xyz', [0, 0, 0]), body.get('rpy', [0, 0, 0])

        if g['type'] == 'mesh':
            # The pose goes on a wrapper Xform, with the reference on a CHILD.
            # Referencing brings the converted asset's own xformOpOrder
            # (translate/orient/scale, plus the metersPerUnit compensation Isaac
            # adds) onto the prim, and XformCommonAPI cannot author over an
            # `orient` op — it fails and returns false rather than raising, so
            # the pose is silently dropped and the body sits at the origin at
            # asset scale. A clean wrapper has no ops to collide with.
            UsdGeom.Xform.Define(stage, path)
            add_reference_to_stage(usd_path=converted[g['uri']],
                                   prim_path=f'{path}/geom')
            place(stage, path, xyz, rpy, g.get('scale', [1.0, 1.0, 1.0]))
            # Exact collision on every mesh under the reference; the referenced
            # payload may hold several.
            for prim in Usd.PrimRange(stage.GetPrimAtPath(path)):
                if prim.IsA(UsdGeom.Mesh):
                    add_collider(prim, exact=True)
        else:
            if g['type'] == 'cylinder':
                # SDF cylinders are Z-aligned, and so is UsdGeom.Cylinder's
                # default axis, so no extra rotation is needed.
                prim = UsdGeom.Cylinder.Define(stage, path)
                prim.CreateRadiusAttr(float(g['radius']))
                prim.CreateHeightAttr(float(g['length']))
                prim.CreateAxisAttr(UsdGeom.Tokens.z)
                r, h = float(g['radius']), float(g['length'])
                prim.CreateExtentAttr([Gf.Vec3f(-r, -r, -h / 2), Gf.Vec3f(r, r, h / 2)])
            elif g['type'] == 'box':
                sx, sy, sz = [float(v) for v in g['size']]
                prim = UsdGeom.Cube.Define(stage, path)
                prim.CreateSizeAttr(1.0)
                prim.CreateExtentAttr([Gf.Vec3f(-.5, -.5, -.5), Gf.Vec3f(.5, .5, .5)])
                place(stage, path, xyz, rpy, [sx, sy, sz])
                add_collider(stage.GetPrimAtPath(path), exact=False)
                continue
            elif g['type'] == 'sphere':
                prim = UsdGeom.Sphere.Define(stage, path)
                prim.CreateRadiusAttr(float(g['radius']))
            else:
                raise SystemExit(f'build_world_usd: unsupported geometry {g["type"]!r}')
            place(stage, path, xyz, rpy)
            # Analytic primitives get analytic collision, not a mesh
            # approximation — cheaper and exact.
            add_collider(stage.GetPrimAtPath(path), exact=False)

    out = os.path.join(out_dir(world_dir), f'{name}.usd')
    stage.Export(out)
    print(f'wrote {out}')
    verify(stage, manifest, name)
    return out


def verify(stage, manifest, name):
    """Compare the assembled stage's bounds against the manifest's expectation.

    This is the check that catches a Collada unit or up-axis misread, which
    would otherwise leave Isaac Sim and Gazebo silently disagreeing about the
    same world. See the module docstring.
    """
    # Colliders first. These are cheap invariants of the manifest itself, and
    # each one is a silent failure if violated: no collider means the robot
    # drives through a wall the lidar can still see (the RTX lidar traces render
    # geometry, physics uses colliders, and the two are independent); a convex
    # approximation on the hollow wall seals the robot in; a rigid body on a
    # static prop makes the arena fall over on play().
    n_col = n_mesh = n_rigid = 0
    approximations = set()
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            n_rigid += 1
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        n_col += 1
        if prim.HasAPI(UsdPhysics.MeshCollisionAPI):
            n_mesh += 1
            attr = UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr()
            approximations.add(attr.Get() if attr else None)

    print(f'verify: {n_col} collision prims ({n_mesh} mesh, '
          f'approximations={sorted(str(a) for a in approximations) or "n/a"}), '
          f'{n_rigid} rigid bodies')

    problems = []
    if n_col < len(manifest['bodies']):
        problems.append(f'{n_col} collision prims for {len(manifest["bodies"])} '
                        f'manifest bodies — some body has no collider, so the '
                        f'robot will pass through geometry the lidar still sees')
    if n_rigid:
        problems.append(f'{n_rigid} rigid bodies, expected 0 — every manifest '
                        f'body is static; a rigid body makes the arena collapse '
                        f'on play()')
    bad_approx = approximations - {'none'}
    if bad_approx:
        problems.append(f'mesh collision approximations {sorted(bad_approx)} — '
                        f'must be "none" (exact). A convex hull of a hollow wall '
                        f'is a solid prism and seals the robot inside')
    if problems:
        raise SystemExit('build_world_usd: collider checks failed:\n  '
                         + '\n  '.join(problems))

    expect = manifest.get('verify')
    if not expect:
        print('verify: manifest declares no expected bounds; skipping '
              '(recommended for any world using Collada meshes)')
        return

    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                             [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    box = cache.ComputeWorldBound(stage.GetPrimAtPath(ROOT_PRIM)).ComputeAlignedRange()
    lo, hi = box.GetMin(), box.GetMax()
    tol = float(expect.get('tolerance', 0.02))
    want_lo = [float(v) for v in expect['bounds_min']]
    want_hi = [float(v) for v in expect['bounds_max']]

    print(f'verify: bounds min {[round(v, 4) for v in lo]} max {[round(v, 4) for v in hi]}')
    print(f'        expect min {want_lo} max {want_hi} (tol {tol})')

    bad = [f'{ax}{kind}: got {got:.4f} want {want:.4f}'
           for ax, got, want, kind in
           [('xyz'[i], lo[i], want_lo[i], 'min') for i in range(3)] +
           [('xyz'[i], hi[i], want_hi[i], 'max') for i in range(3)]
           if abs(got - want) > tol]
    if bad:
        raise SystemExit(
            'build_world_usd: generated bounds disagree with the manifest:\n  '
            + '\n  '.join(bad)
            + f'\n\nThe usual cause is the Collada unit/up-axis trap: a scale of '
              f'~39.4x means <unit> was ignored, and swapped Y/Z extents mean the '
              f'converter honoured up_axis="Y_UP" when Gazebo does not. '
              f'{name}.usd was written but does NOT match the Gazebo world.')
    print('verify: OK — Isaac stage matches the Gazebo world')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--world', default=os.environ.get('WORLD', 'turtlebot3_world'),
                    help='world name (directory under the registry); '
                         'defaults to $WORLD')
    args = ap.parse_args()

    # SimulationApp.close() hard-exits the process, which swallows both an
    # in-flight traceback and the exit code — a failure here otherwise looks
    # exactly like a success (silent, status 0). So catch everything, print it,
    # flush, and force the status ourselves after the app is down.
    status = 0
    try:
        build(args.world)
    except BaseException:
        traceback.print_exc()
        status = 1
    sys.stdout.flush()
    sys.stderr.flush()
    print(f'build_world_usd: {"OK" if status == 0 else "FAILED"}', flush=True)
    sim_app.close()
    os._exit(status)


if __name__ == '__main__':
    sys.exit(main())
