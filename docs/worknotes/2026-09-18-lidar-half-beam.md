# Isaac Sim's lidar is not rotated

`docs/roadmap.md` §6 has carried "Isaac Sim's lidar is rotated by half a beam —
measured, not diagnosed" since 2026-09-17, with 0.544° on record and an
explicit note that half a bin *hints* at a beam-indexing convention but is not
a diagnosis. This is the diagnosis. It is not the one the hint pointed at.

**Headline, in three lines:**

1. **The rays point exactly where they should.** With the profile's own noise
   switched off, the azimuth the sensor reports and the azimuth the geometry
   implies agree to **0.0000°, sd 0.0000** (§2). There is no mounting error, no
   tf error, and nothing is rotated.
2. **No beam is ever half a beam off.** Over 4320 rays, **50.7% are off by
   ~0.0° and 49.3% by ~1.0°, and 0.0% by anything in between** (§4). Half a
   beam is the *mean of a coin flip*, exactly as "30% under-rotation" turned
   out to be the mean of a chattering wheel.
3. **The coin flip is that every ray lands exactly on a bin boundary.** The
   rays fire at exactly integer degrees; the publisher's bin edges are at
   exactly integer degrees; the profile's `azimuthErrorStd` of 0.015° decides
   which side (§3).

Nothing was changed. §6 says what the options are and why none of them is free.

---

## 0. Provenance

| | |
|---|---|
| `tb3_sim2real` | `1a5779b` |
| `turtlebot3_isaacsim` | `4231fbf` |
| Isaac Sim | `6.1.0-rc.26+release.49347.2d230af4.gl` |
| lidar profile | `models/lidar_configs/turtlebot3_lds.json`, unmodified except where §2/§4 say |
| writer | `RtxLidarROS2PublishLaserScan`, `horizontalFov=360`, `horizontalResolution=1.0`, `azimuthRange=[-180, 180]`, `rotationRate=5` |
| instrument | `measurements/2026-09-18_isaacsim_scan_binning.probe.py` |
| results | `measurements/2026-09-18_isaacsim_scan_binning.json` |
| re-analysed | `measurements/2026-09-17_scan_{gazebo,isaacsim}.beams.json` |

The probe runs the lidar alone in a box room — no robot, no world. A chair or a
wheel would only have added cases to classify.

## 1. What the publisher actually promises

From Isaac Sim's own C++, not its prose docs:
`exts/isaacsim.ros2.nodes/isaacsim/ros2/nodes/ogn/nodes/OgnROS2PublishLaserScan.cpp`,
`publishFromGMO()` → `getLaserScanOutputIndex()`. The header is not shipped, but
upstream restates the arithmetic in its own test,
`isaacsim/ros2/nodes/tests/test_rtx_sensor.py`, under a comment saying it
"reproduce[s] publishFromGMO's float32 binning":

```python
diff_f32    = azimuth_f32 - az_start_f32
bin_indices = (diff_f32 / h_res_f32).astype(np.int64)     # truncation
```

and `angle_min` on the message is `azimuthRange[0]` verbatim. So:

> **Bin `i` spans the half-open azimuth sector
> `[angle_min + i*inc, angle_min + (i+1)*inc)`, and is labelled with that
> sector's LOWER EDGE.**

A consumer doing what `sensor_msgs/LaserScan` says to do — beam direction is
`angle_min + i * angle_increment` — therefore reads the *edge* of the sector,
while the ray that filled it was somewhere inside. The reported angle is
**never too high and almost always too low**, by somewhere between 0 and one
whole increment. Where in that interval it lands is decided entirely by where
the rays sit, which is §3.

This is also where `/scan`'s `-1.0` for no-return comes from, already on record
in `docs/status.md`: `std::fill(m_linearDepthBuffer.begin(), ..., -1.0f)`.

Gazebo has no such step. It enumerates ray *directions* as sample positions, so
`angle_min + i * angle_increment` **is** the ray. That is the whole reason it
measures 0.000° and Isaac does not — not a better sensor model, a different
kind of number.

## 2. The rays point exactly where they should

`--mode noiseless` zeroes `rangeAccuracyM` and `azimuthErrorStd`. In a box room
whose inner faces are a known 1.0 m from the sensor, the range alone fixes the
true azimuth: `range(a) = 1.0 / cos(a - 90°k)`. Comparing that against the
azimuth the sensor reports, over 264 beams with real angular leverage, in each
of three revolutions:

| | reported − geometric |
|---|---|
| mean | **+0.0000°** |
| sd | **0.0000°** |
| median | **+0.0000°** |

The first six azimuths come back as `0.0, -1.0, -2.0, -3.0, -4.0, -5.0` and the
first six ranges as `1.0, 1.000152, 1.00061, 1.001372, 1.002442, 1.00382` —
which is `1/cos(k°)` to six figures.

Two things fall out of that table, and the second one is the whole answer:

- **The azimuth the sensor reports IS the direction the ray went.** Not a
  sector start, not a bin label. So the lidar is aimed correctly and `/scan`'s
  problem is entirely downstream of the sensor.
- **The 360 rays fire at exactly integer degrees**, because
  `horizontalResolution = 360 * scanRateBaseHz / patternFiringRateHz =
  360 * 5 / 1800 = 1.0°` divides 360 evenly and `startAzimuthOffsetDeg` is 0.

## 3. …which is exactly where the bin edges are

`azimuthRange[0] = -180.0` and `inc = 1.0`, so §1's bin boundaries sit at
`-180, -179, …` — **at exactly the integer degrees §2 just found the rays at.**
Every single ray is fired precisely onto a boundary between two output bins.

What then decides the bin is the profile's own `azimuthErrorStd = 0.015`, which
jitters each revolution by a few thousandths of a degree either way. Run as
shipped, the fractional part of the azimuths wanders between −0.025 and +0.037
and is **redrawn every revolution** (deduping the annotator by the lidar's own
`timestampNs` matters here: read per rendered frame, 60 Hz of frames latch the
same 5 Hz revolution a dozen times and the jitter looks constant when it is
not).

A ray a hair *above* the boundary keeps its bin and reports correctly. A ray a
hair *below* it falls into the bin beneath, which is labelled one whole degree
lower. There is no third case.

## 4. The measurement

`--mode shipped`, 12 revolutions, 4320 rays. "Correction" is what a consumer
must ADD to the angle the message labels a bin with, to get the direction the
ray in it actually went.

| | as shipped | with `startAzimuthOffsetDeg = 0.5` |
|---|---|---|
| mean | **+0.4928°** | **+0.4998°** |
| sd | **0.4888°** | **0.0143°** |
| min → max | +0.0004 → +0.9996 | +0.4575 → +0.5437 |
| within 0.1° of **0.0** | **50.7%** | 0.0% |
| within 0.1° of **0.5** | **0.0%** | **100.0%** |
| within 0.1° of **1.0** | **49.3%** | 0.0% |

The left column is the finding. Not one ray in 4320 is half a beam off; they
are split almost exactly evenly between not off at all and off by a whole beam.
Per revolution the mean swings over `0.585, 0.580, 0.259, 0.499, 0.254, 0.501,
0.577, 0.663, 0.499, 0.503, 0.416, 0.578` — because the jitter within one
revolution is a slow drift rather than per-ray noise, so contiguous *arcs* of
the scan go the same way together and the split is 150/210 or 270/90 before it
is 180/180.

The right column is §6.

## 5. Why the recorded scans said 0.544°, and the two things that corroborate it

`ros2 run tb3_bringup scan_test` averages **20 scans per beam** before it
compares anything. A bin that holds the correct ray on half the revolutions and
its neighbour on the other half averages to a range halfway between the two —
i.e. to an effective angle half a beam off, every time, on every beam. The
bimodal truth is averaged into a unimodal artefact before the fit ever runs.

Re-solving the 2026-09-17 recordings for the offset each beam needs *on its
own* — against ranges ray-cast from `worlds/small_office/world.yaml`,
restricted to wall beams with real angular leverage — gives exactly that:

| | n | mean | sd | median |
|---|---|---|---|---|
| gazebo | 149 | −0.024° | 0.142° | −0.010° |
| isaacsim | 136 | **+0.481°** | 0.262° | **+0.490°** |

Both unimodal, Isaac's in one clean peak at half a beam, matching the 0.544°
already on record. Which is precisely what §4 predicts an average of 20 scans
to look like, and is *not* evidence of a rotation.

Two independent corroborations, both from data that already existed:

- **Scan-to-scan spread.** For beams with angular leverage, the recorded
  max-minus-min over the 20 scans, divided by how much range one whole beam of
  angle buys at that beam: **gazebo 0.67, isaacsim 1.78.** Gazebo's spread is
  its own range noise. Isaac's beams genuinely swing by more than a full beam
  between revolutions — they are changing bins.
- **The residual that never closed.** Removing 0.544° took Isaac's wall
  residual from 0.01509 m to 0.00608 m, still 2.5x worse than Gazebo's
  0.00242 m. A rigid rotation would have closed it. A coin flip cannot be
  removed by any single offset, which is why it did not.

## 6. What could be done, and why none of it is free

Nothing was changed. The options, in full, because the shape of this is not
obvious:

**The half beam itself is structural to the node and cannot be removed from
here.** §1's bin is labelled with its lower edge while the ray that filled it is
somewhere inside, so the expected error is half an increment for *any* ray
placement. Shifting `azimuthRange` shifts the labels and the edges together and
buys nothing: rays mid-bin still report the edge.

**The coin flip can be removed, and that is a real gain.**
`startAzimuthOffsetDeg = 0.5` moves the rays half a bin off the boundaries and
plants each one firmly in the middle of its own. §4's right-hand column is that
run: the error stops being 0-or-1 and becomes a *constant* +0.4998°, with the
per-beam scatter down **34x**, from 0.4888° to 0.0143°. Same bias, no
randomness — strictly better, since the bias was already 0.5 on average. It
would also take out a chunk of the excess `/scan` noise (`noise_spread_mean`
0.05584 against Gazebo's 0.02991).

Two reasons it was not just applied:

1. It belongs in `turtlebot3_isaacsim`, which owns the lidar profile, not here.
   A copy of the number on this side would be a second source of truth for the
   Isaac backend — the exact thing this repo exists to prevent.
2. It trades zero-mean angular *noise* for a constant angular *bias*, and which
   of those AMCL and slam_toolbox would rather have is a question about the
   consumers, not about the sensor. Worth deciding deliberately.

**What is NOT worth doing:** correcting `angle_min` downstream by half a beam.
It would make the mean right and leave every individual beam still 0 or 1 off,
which is the error that actually costs anything.
