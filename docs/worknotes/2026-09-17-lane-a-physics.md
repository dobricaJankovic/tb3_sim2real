# Lane A — the actuation layer, measured

Overnight 2026-09-16/17, against the brief in
`docs/worknotes/2026-09-16-overnight-brief.md` §2. Written for someone reading
it cold.

**Headline:** the leading hypothesis (H1, "the drive is damping-limited, raise
the gain") is **dead**, and it was killed by a measurement rather than by
argument. What replaces it is sharper and was not on the brief's list: the
deficit is **differential-mode only**. Forward motion tracks its commanded
wheel velocity to 96–98%; yaw tracks to 67–71%. A wheel in the arc phase
*exceeds* its commanded velocity, which no resisting torque can cause.

---

## 0. Provenance — what these numbers were taken against

Pinned, because lane B was restructuring `src/turtlebot3_isaacsim` during the
same night and a measurement against a moving build is not a measurement.

| | |
|---|---|
| `tb3_sim2real` | `6d44cc3` at session start |
| `turtlebot3_isaacsim` | `83352fb` → `d47ab57` during the night (see below) |
| burger asset | `models/turtlebot3_burger/**` SHA-256 of the file list: `98bd618…` — **unchanged all night** |
| physics | `--physics-hz 60.0`, i.e. `dt = 1/60 s = 16.67 ms`, `device='cpu'` |
| world | `empty_stage` (ground plane, no environment) |
| Gazebo | `max_wheel_torque 20 N·m`, `max_wheel_acceleration 1.0 rad/s²`, diff-drive update rate 30 Hz |

Lane B's commits `32f687b` (moved `scripts/turtlebot3_isaacsim.py` →
`runtime/`) and `d47ab57` (`SCAN_OFFSET`) landed mid-session. Neither touches
`models/`, and the `SCAN_OFFSET` fix changed **waffle only** — burger's
`(-0.032, 0, 0.182)` is unchanged and correct (`0.010` base_joint + `0.172`
scan_joint). **The burger drive measurements below are all against one asset.**

`scripts/check_worlds.py`: **3/3 worlds consistent**, run before any work
(re-run at the end, see §9).

## 1. A0 — baseline, both backends, before anything was touched

`drive_test`, `sequence:=default`, `world:=empty_stage`, headless, open loop.
Files: `measurements/2026-09-16_pre_{gazebo,isaacsim}_default.json`.

The instrument was extended first (§2), so the baseline records the wheel joints
as well as `/odom`. The commands and their timing are unchanged, so it remains
comparable with the 2026-09-15 runs already in `measurements/`.

| phase | | gazebo | isaacsim |
|---|---|---|---|
| `straight` 0.15 m/s × 5 s | distance (want 0.750) | **0.7511** | **0.7127** (−4.9%) |
| `rotate` 0.5 rad/s × 5 s | yaw (want 2.500) | **2.4776** (−0.9%) | **1.7164** (−31.3%) |
| `arc` 0.1 / 0.3 × 5 s | yaw | 1.4634 | 1.0777 |

Reproduces `measurements/isaac_angular_deficit.md` (−30.3%) and
`docs/status.md`. Nothing has drifted; the deficit is real and current.

## 2. A1 — where the deficit is, measured rather than assumed

### A1.3 first: does the REFERENCE backend track its own command?

Nobody had checked. It does, and it is not close:

| gazebo phase | commanded ω (L/R, rad/s) | measured ω (L/R) | tracking |
|---|---|---|---|
| `straight` | 4.5455 / 4.5455 | 4.5455 / 4.5455 | **1.0000 / 1.0000** |
| `rotate` | −1.2121 / 1.2121 | −1.2119 / 1.2117 | 0.9998 / 0.9997 |
| `arc` | 2.3030 / 3.7576 | 2.3031 / 3.7574 | 1.0000 / 1.0000 |

**Gazebo tracks the analytic command to within 0.03%.** So "Isaac is the one
that is wrong" is defensible: it is measured against the command, not against
Gazebo, and the reference independently satisfies the same contract.

Gazebo's remaining −0.9% yaw is therefore **not** drive error. 2.500 − 2.4776 =
0.022 rad, which at 0.5 rad/s is 44 ms of lost time — the acceleration ramp at
the start of the phase (`max_wheel_acceleration 1.0 rad/s²` in the SDF), not a
steady-state error. Its steady state is exact.

### A1.2 — Isaac Sim, and the structure nobody had looked for

The deficit is at the joint. But it is **not** a uniform shortfall, and this is
the finding that redirects the whole diagnosis. Splitting the two wheels into
common mode (ω_L + ω_R)/2, which drives the robot forward, and differential
mode (ω_R − ω_L), which yaws it:

| phase | common cmd → got | ratio | differential cmd → got | ratio |
|---|---|---|---|---|
| `straight` | 4.545 → 4.445 | **0.978** | 0.000 → 0.005 | — |
| `rotate` | 0.000 → 0.014 | — | 2.424 → 1.614 | **0.666** |
| `arc` | 3.030 → 2.918 | **0.963** | 1.455 → 1.031 | **0.709** |

**Forward motion tracks to 96–98%. Yaw tracks to 67–71%.**

And in `arc` the left wheel is commanded 2.3030 and delivers **2.4021** — it
runs 4.3% *faster* than asked. A resisting torque against a velocity servo can
only ever make a wheel slow. A wheel that overshoots its own target is being
**back-driven**: the drive is not holding it. That single number is
incompatible with the "fixed friction torque against a finite-gain drive"
reading in `measurements/isaac_angular_deficit.md`, which was inferred from
pure-rotation phases only — where common mode is zero and the two modes cannot
be told apart.

### A1.1 — which Physics variant is live (F3)

Settled, at the stage and at runtime. `measurements/isaac_drive_probe.json`:

```
variant_selections   {"/World/turtlebot3:Physics": "physx"}
mjc_actuators        []                       <- none on the composed stage
authored damping     1745.3292  (per deg/s, as the converter wrote it)
runtime damping      100000.0   (per rad/s, SI)
runtime max effort   3.4028e+38 (FLT_MAX)
```

Three consequences, all of which remove hypotheses rather than add them:

1. **F3 is false as a live defect.** The root layer selects `Physics = "physx"`
   and `asset_layer()` references that layer, so the `mujoco` variant — the one
   that deletes `PhysicsDriveAPI:angular` and substitutes `MjcActuator` prims
   carrying the degree-converted gain — is **never composed**. No `MjcActuator`
   exists on the stage. The unit bug in NVIDIA's converter is real and is still
   sitting in `payloads/Physics/mujoco.usda`, but it is **latent**: it would
   only bite if something selected that variant. It is not tonight's cause.
2. **The degree → radian conversion is CORRECT on the live path.** Authored
   `1745.3292` per deg/s arrives at the solver as exactly `100000.0` per rad/s,
   which is the `1.0e5` `import_turtlebot3.py` asked for. There is no missing
   factor of 57.3 in the physx variant.
3. **H2 (force saturation) is dead on inspection.** `max_force` is unauthored
   in USD and the runtime max effort is FLT_MAX. Nothing caps the torque.

## 3. H1 is dead — the falsification test, with both measurements (F4)

The brief's criterion: *`err * D` must be constant; measure at `D` and `100*D`;
if the error does not fall by ~100x the damping-limited hypothesis is dead.*

Swept **10,000x**, two decades either side of the authored value, commanding the
articulation directly. `measurements/isaac_drive_probe.json`:

| runtime `D` (N·m per rad/s) | differential err (rad/s) at target 0.485 / 1.212 / 2.424 / 3.636 | common err (rad/s) at 1.818 / 4.545 / 9.091 / 13.64 |
|---|---|---|
| **1 000** (0.01×) | 0.2201 / 0.2878 / 0.2748 / 0.3057 | 0.0763 / 0.0873 / 0.0744 / 0.0737 |
| **100 000** (as built) | 0.2321 / 0.3277 / 0.3242 / 0.2618 | 0.0744 / 0.0856 / 0.0790 / 0.0775 |
| **10 000 000** (the tutorial's value) | 0.2203 / 0.3321 / 0.3606 / 0.2948 | 0.0751 / 0.0846 / 0.0763 / 0.0744 |

**The error does not fall. It does not move at all.** Across a 10,000x change
in the gain every number is the same to within its own run-to-run scatter, and
at the highest damping the differential error is marginally *worse*.

`err * D`, which is the implied resisting torque and must be constant if the
drive is damping-limited, instead spans **76 N·m to 3 606 069 N·m** — it tracks
`D` exactly, because `err` is independent of `D`. For scale, a Dynamixel XL430
stalls at about 1.4 N·m.

> **H1 — "the drive is damping-limited" — is falsified.**
> `D = 1e3 → 1e7`, error `0.288 → 0.332 rad/s` at the `wz = 0.5` operating
> point. Required for H1 to survive: `0.288 → 0.0000288`.

This matters beyond the hypothesis. **F2's recommendation — adopt NVIDIA's
documented `1.0e7` — would have changed nothing measurable**, and the reason
the tutorial's number looked promising is that the symptom it is supposed to
cure was never actually a gain problem. Had the night been spent raising the
gain until the symptom moved, it would never have moved, or would have moved by
noise and been reported as a fix.

Also dead, as by-products of the same run:

- **H0 (the OmniGraph emits a wrong command)** — excluded. This probe sets
  joint velocity targets on the articulation directly: no `/cmd_vel`, no
  `ROS2SubscribeTwist`, no `DifferentialController`, no
  `IsaacArticulationController`. The deficit is unchanged, so nothing upstream
  of the joint is responsible.
- **H4 (odometry bug)** — excluded twice over. The deficit is visible in
  `/joint_states` and in the articulation's own joint velocities, neither of
  which is odometry. Separately, Isaac's `/odom` is `IsaacComputeOdometry`
  reading the chassis prim, i.e. the true body pose, so it cannot drift from
  the body at all (§5).
- **H2 (torque saturation)** — dead, FLT_MAX.

