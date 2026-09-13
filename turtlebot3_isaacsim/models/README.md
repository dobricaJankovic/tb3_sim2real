# models

This package's robot assets — the counterpart of `turtlebot3_gazebo/models/`.

```
models/
├── lidar_configs/
│   └── turtlebot3_lds.json          RTX lidar profile (committed)
└── turtlebot3_<model>/
    └── turtlebot3_<model>.usd       robot asset (generated, gitignored)
```

## The robot asset

`turtlebot3_<model>.usd` is what `model.sdf` is for Gazebo: a self-contained,
referenceable robot carrying geometry, an articulation, and surface properties.
It is **generated**, not committed — USD trees are large and binary-ish, and the
URDF in `turtlebot3_description` is the actual source of truth.

```bash
./scripts/build_models.sh burger
```

### Why there is no `urdf/` directory here

`turtlebot3_gazebo` carries a second copy of the URDF with its mesh URLs
repointed at `turtlebot3_gazebo/models/turtlebot3_common/meshes/`. That copy
exists purely so Gazebo's `model://` resolution can find the meshes at *runtime*.

Isaac Sim resolves meshes once, at URDF-import time, and bakes the geometry into
the USD. After the import the asset needs nothing from
`turtlebot3_description` at all. So a second copy would buy nothing and could
only drift — this package depends on `turtlebot3_description` directly and
imports from it.

## The lidar profile

`lidar_configs/turtlebot3_lds.json` models the TB3's LDS, matching
`turtlebot3_gazebo`'s `<ray>` block:

| | Gazebo `<ray>` | `turtlebot3_lds.json` |
|---|---|---|
| samples/rev | 360 | 1800 Hz ÷ 5 Hz = 360 |
| resolution | 1.0° | 1.0° |
| rate | 5 Hz | `scanRateBaseHz: 5.0` |
| range | 0.12–3.5 m | `nearRangeM`/`farRangeM` |
| range resolution | 0.015 m | `rangeResolutionM` |
| noise σ | 0.01 m | `rangeAccuracyM` |
| elevation | horizontal | `elevationDeg: [0.0]` |

It is registered by appending this directory to
`app.sensors.nv.lidar.profileBaseFolder`, a settings list the renderer walks to
resolve a profile by name. That is what lets the package ship its own sensor
model rather than borrow a vendor one.

**Why not Isaac Sim's stock `Example_Rotary_2D`.** It is a 200 m survey lidar,
and its single emitter sits at `elevationDeg = [-2.0]` — it scans the floor. On
a bare ground plane that still returns hits, spread into a partial arc where the
tilted beam meets the ground, which Nav2 will happily treat as an obstacle ring.
Re-rating it by overriding `scanRateBaseHz`/`patternFiringRateHz` is **not** a
workaround: the scan pattern baked into that config still assumes its own
30 Hz / 32000 Hz, the plugin warns `Multi-tick is enabled but motion BVH is not
active`, and `/scan` publishes in bursts instead of steadily. Authoring a
self-consistent profile is the fix.

**Unverified.** `rotationDirection` is `CW`, carried over from the stock profile
rather than guessed. Whether that produces REP-103 ordering in the published
`LaserScan` is the first thing to check against Gazebo: a mirrored scan looks
entirely plausible and is a correctness bug.
