#!/usr/bin/env python3
"""Render a world's Isaac Sim stage to a PNG, headless.

    scripts/snapshot.sh small_office            # both backends
    docker compose run --rm tb3_ros isaacsim-python /repo/scripts/snapshot.py \
        --world small_office --out /worlds/small_office/isaac/snapshot.png

`check_worlds.py` proves the two representations came from one manifest, and
`build_world_usd.py` proves the stage has the right bounds, colliders and bound
materials. None of that proves it LOOKS right, and the most recent bug here was
exactly that: an Isaac stage that loaded, collided and measured correctly and
rendered entirely grey. So there is a way to look at it that does not need
someone sitting in front of a window.

The stage carries no light of its own — `turtlebot3_isaacsim` adds one at
runtime, and the generated USD is an environment, not a scene — so this adds a
distant light and a dome when the stage has none, for the snapshot only.
"""

import argparse
import os
import sys

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _c in (os.path.join(_REPO, 'tb3_bringup'), '/repo/tb3_bringup', '/ws/src/tb3_bringup'):
    if os.path.isdir(_c):
        sys.path.insert(0, _c)
        break

ap = argparse.ArgumentParser()
ap.add_argument('--world', required=True)
ap.add_argument('--out', default='')
ap.add_argument('--width', type=int, default=1280)
ap.add_argument('--height', type=int, default=720)
ap.add_argument('--frames', type=int, default=120,
                help='render iterations before capture; RTX needs to converge')
args = ap.parse_args()

from isaacsim import SimulationApp                      # noqa: E402

app = SimulationApp({'headless': True, 'width': args.width, 'height': args.height})

from pxr import Gf, Usd, UsdGeom, UsdLux                # noqa: E402
import omni.usd                                         # noqa: E402
from isaacsim.core.utils.stage import open_stage        # noqa: E402
from isaacsim.core.utils.viewports import set_camera_view  # noqa: E402
from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file  # noqa: E402

from tb3_bringup import worlds                          # noqa: E402


def main():
    world = worlds.World.load(args.world)
    usd = world.isaac_usd()
    if not usd:
        raise SystemExit(f'snapshot: {world.name} has no Isaac Sim stage')
    if not os.path.exists(usd):
        raise SystemExit(f'snapshot: {usd} not built; run scripts/build_world.sh')

    open_stage(usd)
    stage = omni.usd.get_context().get_stage()

    if not any(p.IsA(UsdLux.BoundableLightBase) or p.IsA(UsdLux.NonboundableLightBase)
               for p in stage.Traverse()):
        UsdLux.DistantLight.Define(stage, '/World/SnapshotKey') \
            .CreateIntensityAttr(3000)
        UsdLux.DomeLight.Define(stage, '/World/SnapshotFill') \
            .CreateIntensityAttr(700)

    # Frame the stage from its own bounding box, so this works on any world
    # without a per-world camera pose to keep in step with the geometry.
    bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                             [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    rng = bbox.ComputeWorldBound(stage.GetPrimAtPath('/World')).ComputeAlignedRange()
    lo, hi = rng.GetMin(), rng.GetMax()
    centre = [(lo[i] + hi[i]) / 2 for i in range(3)]
    span = max(hi[0] - lo[0], hi[1] - lo[1], 1.0)
    set_camera_view(eye=[centre[0] + span * 0.85,
                         centre[1] - span * 0.85,
                         centre[2] + span * 0.75],
                    target=centre)

    vp = get_active_viewport()
    for _ in range(args.frames):
        app.update()

    out = args.out or os.path.join(world.dir, 'isaac', 'snapshot.png')
    os.makedirs(os.path.dirname(out), exist_ok=True)
    capture_viewport_to_file(vp, out)
    # The capture is asynchronous; without pumping the app it is written after
    # shutdown, which is to say never.
    for _ in range(60):
        app.update()
    print(f'snapshot: wrote {out} ({args.width}x{args.height})')


try:
    main()
finally:
    app.close()
