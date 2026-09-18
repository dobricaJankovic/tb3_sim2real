#!/usr/bin/env python3
#
# Copyright 2026 dobricaJankovic
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Where do the RTX lidar's rays point, and which output bin do they land in?

The instrument behind `docs/worknotes/2026-09-18-lidar-half-beam.md` and
`measurements/2026-09-18_isaacsim_scan_binning.json`. It answers two questions
that `ros2 run tb3_bringup scan_test` cannot, because scan_test only ever sees
the published `/scan` and the thing in question is what happens *before* it:

  1. Is the azimuth the sensor reports the direction the ray actually went?
     `--mode noiseless` zeroes rangeAccuracyM and azimuthErrorStd, so in a box
     room of known size the range alone fixes the true azimuth and the two can
     be compared with nothing inferred.
  2. Where do the rays sit relative to the LaserScan publisher's bin edges?
     `--mode shipped` runs the profile exactly as the package ships it and
     dumps a GenericModelOutput per revolution, deduped by the lidar's own
     timestamp. `--mode half-bin` repeats it with startAzimuthOffsetDeg = 0.5,
     which is the one knob that moves the rays off the bin boundaries.

Run it inside the container, on Kit's Python:

    docker compose exec tb3_ros /entrypoint.sh \
        isaacsim-python measurements/2026-09-18_isaacsim_scan_binning.probe.py \
            --mode shipped --out /tmp/shipped.json

No robot and no world: a box room and the lidar alone, because a chair or a
wheel would only add cases to classify.
"""

import argparse
import json

parser = argparse.ArgumentParser()
parser.add_argument('--profile',
                    default='/ws/src/turtlebot3_isaacsim/models/lidar_configs/turtlebot3_lds.json')
parser.add_argument('--mode', default='shipped',
                    choices=('shipped', 'noiseless', 'half-bin'))
parser.add_argument('--out', default='/tmp/scan_binning.json')
parser.add_argument('--revolutions', type=int, default=12)
parser.add_argument('--frames', type=int, default=1200)
args = parser.parse_args()

from isaacsim import SimulationApp                                   # noqa: E402
simulation_app = SimulationApp({'headless': True})

import numpy as np                                                   # noqa: E402
import omni                                                          # noqa: E402
from pxr import Gf, UsdGeom, UsdLux                                   # noqa: E402
import isaacsim.core.experimental.utils.app as app_utils             # noqa: E402
import isaacsim.core.utils.prims as prim_utils                       # noqa: E402
from isaacsim.core.utils.stage import create_new_stage               # noqa: E402
from isaacsim.sensors.experimental.rtx import (                      # noqa: E402
    Lidar, LidarSensor, parse_generic_model_output_data)

# The same translation runtime/turtlebot3_isaacsim.py performs, kept here
# rather than imported because that module pulls in the whole simulator.
PROFILE_PREFIX = 'omni:sensor:Core:'
PROFILE_RENAMES = {'reportRateBaseHz': 'patternFiringRateHz',
                   'minReflectanceRange': 'minReflectionRangeM',
                   'wavelengthNm': 'waveLengthNm'}
PROFILE_TOKENS = ('scanType', 'intensityProcessing', 'rotationDirection',
                  'rayType', 'intensityMappingType')
PROFILE_STRUCTURAL = ('emitterStateCount', 'emitterStates')
PROFILE_UNSUPPORTED = ('avgPowerW',)

# Inner faces 1 m from the sensor on all four sides. Small enough that the
# corners (1.414 m) are well inside the profile's 3.5 m range, which keeps
# every one of the 360 rays returning something.
HALF_WIDTH = 1.0

with open(args.profile) as f:
    profile = json.load(f)['profile']

# Range noise goes in every mode: it is 0.01 m against an effect worth
# ~0.001 m at normal incidence, and it is not what is being measured.
profile['rangeAccuracyM'] = 0.0
profile['rangeResolutionM'] = 0.0001
if args.mode == 'noiseless':
    profile['azimuthErrorStd'] = 0.0
    profile['elevationErrorStd'] = 0.0
elif args.mode == 'half-bin':
    profile['startAzimuthOffsetDeg'] = 0.5

attributes = {}
for key, value in profile.items():
    if key in PROFILE_STRUCTURAL or key in PROFILE_UNSUPPORTED:
        continue
    if key in PROFILE_TOKENS:
        value = value.upper()
    attributes[PROFILE_PREFIX + PROFILE_RENAMES.get(key, key)] = value
for key, value in profile['emitterStates'][0].items():
    attributes['{}emitterState:s001:{}'.format(PROFILE_PREFIX, key)] = value

create_new_stage()
stage = omni.usd.get_context().get_stage()
UsdGeom.Xform.Define(stage, '/World')
UsdLux.DistantLight.Define(stage, '/World/DistantLight').CreateIntensityAttr(1000)
for name, translate, scale in (
        ('wall_px', (HALF_WIDTH + 0.5, 0.0, 0.0), (1.0, 8.0, 2.0)),
        ('wall_nx', (-HALF_WIDTH - 0.5, 0.0, 0.0), (1.0, 8.0, 2.0)),
        ('wall_py', (0.0, HALF_WIDTH + 0.5, 0.0), (8.0, 1.0, 2.0)),
        ('wall_ny', (0.0, -HALF_WIDTH - 0.5, 0.0), (8.0, 1.0, 2.0))):
    cube = UsdGeom.Cube.Define(stage, '/World/' + name)
    cube.CreateSizeAttr(1.0)
    cube.AddTranslateOp().Set(Gf.Vec3d(*translate))
    cube.AddScaleOp().Set(Gf.Vec3f(*scale))

lidar = Lidar.create(path='/World/lidar', attributes=attributes,
                     accumulate_outputs=True,
                     tick_rate=float(profile['scanRateBaseHz']),
                     translations=[[0.0, 0.0, 0.5]])
prim = prim_utils.get_prim_at_path(lidar.paths[0])
unknown = sorted(a for a in attributes if not prim.HasAttribute(a))
if unknown:
    raise RuntimeError('the OmniLidar schema has no {}'.format(', '.join(unknown)))
scan_hz = float(prim.GetAttribute(PROFILE_PREFIX + 'scanRateBaseHz').Get())
firing_hz = int(prim.GetAttribute(PROFILE_PREFIX + 'patternFiringRateHz').Get())
h_res = 360.0 * scan_hz / firing_hz
print('probe: mode={} horizontalResolution={:.6f} deg'.format(args.mode, h_res),
      flush=True)

sensor = LidarSensor(lidar, annotators=['generic-model-output'])
app_utils.play()

# Deduped by the lidar's own scan-start timestamp: reading the annotator every
# rendered frame latches the SAME revolution several times over -- 60 Hz of
# frames against 5 Hz of revolutions -- and counting those as separate scans
# would make a per-revolution effect look constant.
snapshots, seen = [], set()
for _ in range(args.frames):
    simulation_app.update()
    try:
        data, _meta = sensor.get_data('generic-model-output')
    except Exception:                                                # noqa: BLE001
        continue
    if data is None:
        continue
    try:
        gmo = parse_generic_model_output_data(data)
    except Exception:                                                # noqa: BLE001
        continue
    n = int(gmo.numElements)
    ts = int(gmo.timestampNs)
    if n == 0 or ts in seen:
        continue
    seen.add(ts)
    snapshots.append({'n': n, 'timestampNs': ts,
                      'azimuth': np.array(gmo.x[:n], dtype=np.float64).tolist(),
                      'range': np.array(gmo.z[:n], dtype=np.float64).tolist()})
    if len(snapshots) >= args.revolutions:
        break

with open(args.out, 'w') as f:
    json.dump({'mode': args.mode, 'h_res_deg': h_res,
               'half_width_m': HALF_WIDTH, 'snapshots': snapshots}, f)
print('probe: wrote {} ({} revolutions)'.format(args.out, len(snapshots)), flush=True)
simulation_app.close()
