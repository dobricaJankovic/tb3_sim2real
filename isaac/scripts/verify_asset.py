"""Fail loudly on a USD stage that composes cleanly but has no visible geometry.

The URDF importer resolves an unmatched `package://` mesh URL to a bare relative
path, finds nothing, and still emits the link — as an empty Xform with the right
transform and material binding. The stage then opens without a single error and
renders nothing. This checks the one thing that separates the two cases: whether
any renderable geometry has a non-empty bound.

    /isaac-sim/python.sh /scripts/verify_asset.py [stage.usd] [robot-prim]

Defaults to /scenes/tb3_world.usd and /World/turtlebot3. Exits non-zero when
the robot subtree is empty, so it can gate a re-import.
"""

import sys

from isaacsim import SimulationApp

sim_app = SimulationApp({'headless': True})

from pxr import Usd, UsdGeom


def main() -> int:
    stage_path = sys.argv[1] if len(sys.argv) > 1 else '/scenes/tb3_world.usd'
    subtree = sys.argv[2] if len(sys.argv) > 2 else '/World/turtlebot3'
    stage = Usd.Stage.Open(stage_path)
    if stage is None:
        print(f'FAIL: could not open {stage_path}')
        return 1

    # TraverseInstanceProxies is required, not optional: the importer marks
    # every visual mesh `instanceable = true`, and a plain Traverse() stops at
    # the instance boundary and reports zero robot meshes on a perfectly good
    # asset.
    meshes = []
    for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
        if prim.IsA(UsdGeom.Mesh):
            points = UsdGeom.Mesh(prim).GetPointsAttr().Get()
            meshes.append((prim.GetPath(), len(points) if points else 0))

    # 'guide' is excluded on purpose: imported collision primitives carry that
    # purpose, and they are exactly what makes a mesh-less robot look non-empty.
    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]
    )

    print(f'stage:  {stage_path}')
    print(f'meshes: {len(meshes)}')
    for path, count in sorted(meshes):
        print(f'  {count:8d} pts  {path}')

    # Checking the whole stage is not enough — a ground plane alone clears it
    # while the robot is invisible, which is exactly the bug this exists to
    # catch. Bound the robot subtree on its own.
    prim = stage.GetPrimAtPath(subtree)
    if not prim.IsValid():
        print(f'FAIL: {subtree} does not exist on this stage')
        return 1
    bounds = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    print(f'{subtree} bounds: {bounds.GetMin()} -> {bounds.GetMax()}')

    empty = [p for p, n in meshes if n == 0]
    if bounds.IsEmpty() or not meshes or empty:
        print(f'FAIL: {subtree} has no renderable geometry '
              f'({len(empty)} mesh prims with zero points)')
        return 1

    print('OK: renderable geometry present')
    return 0


if __name__ == '__main__':
    code = main()
    # Same TaskGroup teardown race as import_tb3.py — skip the graceful close.
    sys.stdout.flush()
    import os
    os._exit(code)
