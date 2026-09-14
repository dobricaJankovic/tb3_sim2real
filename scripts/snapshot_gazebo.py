#!/usr/bin/env python3
"""Render a world's Gazebo stage to a PNG, without a window or a screenshot tool.

    scripts/snapshot_gazebo.py small_office

The counterpart of scripts/snapshot.py, which does the same for Isaac Sim. Both
exist for the same reason: the checks prove the two representations came from
one manifest and have the right bounds, colliders and materials, and none of
that proves either one LOOKS right. When the two are meant to be the same room,
being able to put the two pictures side by side is the check.

No X screenshot utility is involved. Gazebo Classic can write frames from a
camera sensor itself, so this injects a temporary camera into a COPY of the
generated .world, runs gzserver for a few seconds, and takes the last frame.
The generated file is never touched.
"""

import argparse
import glob
import math
import os
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), 'tb3_bringup'))

from tb3_bringup import worlds                          # noqa: E402

CAMERA = """
<model name="tb3_snapshot_camera">
  <static>true</static>
  <pose>{x:.4f} {y:.4f} {z:.4f} 0 {pitch:.4f} {yaw:.4f}</pose>
  <link name="link">
    <sensor name="camera" type="camera">
      <camera>
        <horizontal_fov>1.0472</horizontal_fov>
        <image><width>{w}</width><height>{h}</height></image>
        <clip><near>0.1</near><far>200</far></clip>
        <save enabled="true"><path>{out}</path></save>
      </camera>
      <always_on>1</always_on>
      <update_rate>2</update_rate>
    </sensor>
  </link>
</model>
"""


def bounds(world):
    """Axis-aligned extent of the generated .world, from its own geometry."""
    lo = [1e9] * 3
    hi = [-1e9] * 3
    tree = ET.parse(os.path.join(world.dir, world.artifact('gazebo')['path']))
    for vis in tree.getroot().iter('visual'):
        pose = (vis.findtext('pose') or '0 0 0 0 0 0').split()
        x, y, z = [float(v) for v in pose[:3]]
        for i, v in enumerate((x, y, z)):
            lo[i] = min(lo[i], v)
            hi[i] = max(hi[i], v)
    if lo[0] > hi[0]:
        return [-5, -5, 0], [5, 5, 1]
    return lo, hi


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('world')
    ap.add_argument('--out', default='')
    ap.add_argument('--width', type=int, default=1280)
    ap.add_argument('--height', type=int, default=720)
    ap.add_argument('--seconds', type=float, default=10.0)
    a = ap.parse_args()

    world = worlds.World.load(a.world)
    src = os.path.join(world.dir, world.artifact('gazebo')['path'])
    if not os.path.exists(src):
        raise SystemExit(f'snapshot_gazebo: {src} not built')

    lo, hi = bounds(world)
    centre = [(lo[i] + hi[i]) / 2 for i in range(3)]
    span = max(hi[0] - lo[0], hi[1] - lo[1], 2.0) + 2.0
    eye = (centre[0] + span * 0.85, centre[1] - span * 0.85, centre[2] + span * 0.75)
    dx, dy = centre[0] - eye[0], centre[1] - eye[1]
    dz = centre[2] - eye[2]
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(-dz, math.hypot(dx, dy))

    tmp = tempfile.mkdtemp(prefix='tb3snap-')
    shots = os.path.join(tmp, 'frames')
    os.makedirs(shots, exist_ok=True)
    text = open(src).read()
    cam = CAMERA.format(x=eye[0], y=eye[1], z=eye[2], pitch=pitch, yaw=yaw,
                        w=a.width, h=a.height, out=shots)
    staged = os.path.join(tmp, 'snapshot.world')
    with open(staged, 'w') as f:
        f.write(text.replace('</world>', cam + '</world>'))

    proc = subprocess.Popen(['gzserver', '--verbose', staged],
                            stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    try:
        proc.wait(timeout=a.seconds)
    except subprocess.TimeoutExpired:
        proc.terminate()
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()

    frames = sorted(glob.glob(os.path.join(shots, '*.jpg'))
                    + glob.glob(os.path.join(shots, '*.png')))
    if not frames:
        err = proc.stderr.read().decode()[-800:] if proc.stderr else ''
        shutil.rmtree(tmp, ignore_errors=True)
        raise SystemExit('snapshot_gazebo: the camera sensor wrote no frames. '
                         'gzserver needs a GL context, so DISPLAY must be set '
                         'and reachable.\n' + err)

    out = a.out or os.path.join(world.dir, 'gazebo_snapshot' +
                                os.path.splitext(frames[-1])[1])
    shutil.copy(frames[-1], out)
    shutil.rmtree(tmp, ignore_errors=True)
    print(f'snapshot_gazebo: wrote {out} (from {len(frames)} frames)')


if __name__ == '__main__':
    main()
