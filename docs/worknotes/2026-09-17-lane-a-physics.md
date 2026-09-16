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

## 4. What it actually is: the wheel is not slow, it is **chattering**

The mean was hiding the signal. Same runs, same phases, but the standard
deviation of the joint velocity over the steady-state window instead of only
its mean (`measurements/2026-09-16_pre_*_default.samples.json`):

| phase | wheel | commanded | **gazebo** mean / sd | **isaacsim** mean / sd | isaac sd as % of command |
|---|---|---|---|---|---|
| `straight` | L | 4.5455 | 4.5455 / **0.0000** | 4.4432 / 0.0698 | 1.5% |
| `straight` | R | 4.5455 | 4.5455 / **0.0000** | 4.4477 / 0.0450 | 1.0% |
| `rotate` | L | −1.2121 | −1.2118 / **0.0000** | −0.7925 / **0.4463** | **36.8%** |
| `rotate` | R | 1.2121 | 1.2117 / **0.0000** | 0.8214 / **0.3726** | **30.7%** |
| `arc` | L | 2.3030 | 2.3031 / **0.0000** | 2.4021 / 0.3062 | 13.3% |
| `arc` | R | 3.7576 | 3.7574 / **0.0000** | 3.4334 / 0.1676 | 4.5% |

During `rotate`, commanded a steady −1.2121 rad/s, the left wheel ranges from
**−2.9066 to +1.0422 rad/s**. It overshoots the target by 2.4x and it reverses
direction. The mean of that, −0.79, is what every previous measurement recorded
as "Isaac Sim delivers 65% of the commanded rate".

**It does not deliver 65% of the rate. It is unstable, and 65% is the average
of the instability.** Gazebo's standard deviation is 0.0000 to four decimals in
every phase — its wheels are not merely accurate, they are noise-free.

This is consistent with everything else and explains what the gain sweep could
not:

- **Why the gain does not matter.** The oscillation amplitude is set by the
  contact/solver interaction, not by the servo gain, so `D` can move four
  decades without changing the mean. At the highest damping the differential
  error is marginally *worse*, which is what a stiffer drive at a fixed
  timestep should do to a chattering contact.
- **Why a wheel exceeds its command.** It is oscillating, not tracking.
- **Why the error looked like a constant absolute offset.** A chatter-induced
  bias is roughly set by the oscillation amplitude and so barely depends on the
  commanded rate — which reads exactly like `tau / D` in a table of means, and
  is why the original diagnosis in `measurements/isaac_angular_deficit.md` was
  a reasonable inference from the data it had.
- **Why differential mode is far worse than common mode.** Yaw is the soft
  degree of freedom: it is resisted by the caster skid scrubbing sideways,
  which is a sliding box contact and the classic source of stick-slip chatter.

### A related asymmetry the world registry does not cover

The two backends' robots do not have the same physics materials, and nothing
checks this. `turtlebot3_gazebo`'s `model.sdf` gives each wheel
**`mu = 100000`**; `turtlebot3_isaacsim`'s asset gives the wheel material
**`staticFriction = dynamicFriction = 1.0`**, with the chassis and caster at
0.1 and `frictionCombineMode = "min"`.

`scripts/check_worlds.py` proves the *world* is identical across backends by
construction. **The robot is not part of the manifest**, so its materials,
masses and drive parameters are authored independently in two packages, and a
divergence there is exactly as damaging to a comparison as a divergence in the
world would be — with nothing to catch it. This is the same class of defect as
F9.1 (a map carries no provenance) and is recorded here rather than fixed: it
is a schema question, not a tonight question.

### And a fidelity point that outlives this bug

Gazebo's `sd = 0.0000` is not Gazebo being better. `gazebo_ros_diff_drive`
drives the wheels through an ODE joint motor with `fmax = max_wheel_torque`,
i.e. an **ideal velocity source** with a force cap; the wheel takes the
commanded velocity exactly, and with `mu = 100000` it cannot slip either. So:

| | actuator | wheel slip |
|---|---|---|
| gazebo | ideal velocity source, exact | effectively impossible (`mu = 1e5`) |
| isaacsim | dynamic, torque-driven, currently unstable | possible, friction 1.0 |
| real burger | encoder PID, tracks well, finite | real |

**Neither simulator models the actuator faithfully, and they fail in opposite
directions** — one cannot be wrong, the other cannot be right. That is a
sharper statement of where the sim-to-real gap lives than "Isaac under-rotates"
and it does not depend on fixing anything.

## 5. A2 — the discriminating tests, and the mechanism

`measurements/isaac_drive_probe2.json`. Each row rebuilds the stage, changes
exactly one thing, and commands the wheels directly. Errors in rad/s;
`common` drives the robot forward, `diff` pivots it.

| what was changed | common 1.818 | common 4.545 | diff 1.212 | diff 2.424 |
|---|---|---|---|---|
| **nothing** (60 Hz, as shipped) | 0.0833 | 0.0951 | 0.3482 | 0.3591 |
| **no gravity, no ground plane** | **0.0000** | **0.0000** | **0.0000** | **0.0000** |
| physics 120 Hz | 0.0291 | 0.0345 | 0.1989 | 0.2166 |
| physics 240 Hz | 0.0028 | 0.0002 | 0.0935 | 0.0687 |
| solver iterations 64 | 0.0284 | 0.0339 | 0.2965 | 0.0554 |
| solver iterations 255 | 0.0019 | 0.0013 | 0.1670 | 0.2458 |
| caster friction 0.1 → 0.0 | 0.0069 | 0.0219 | 0.3325 | 0.2685 |

### The decisive row

**Lift the robot off the ground and the error is exactly zero — in every mode,
at every rate, to four decimal places.** Tracking ratio 1.0000.

The drive is not weak, not saturated, not mistuned and not miscommanded. Given
nothing to push against it reproduces its commanded velocity perfectly. **The
entire deficit is created by the contact solve**, and it therefore could never
have been fixed by a gain — which is why the 10,000x sweep in §3 did nothing.

This is also the test that no ROS-level measurement can perform, and it took
about forty seconds.

### What it is

Both remaining knobs point the same way. The error falls monotonically with the
physics timestep — 0.0833 → 0.0291 → 0.0028 in common mode across 60/120/240 Hz
— and it falls with solver iterations at fixed dt. That is the signature of an
**unconverged contact solve**: at `dt = 1/60` the solver does not reach a
consistent state between the wheel drive constraints and the ground contacts,
and the residual shows up as the velocity chatter in §4. **H5 is supported; H1,
H2, H0 and H4 are dead, and H3 is confirmed in the specific sense that contact
is *necessary* — but the fault is in how the contact is solved, not in the
friction values.**

**The caster is not the cause.** Taking its friction to zero leaves the
differential error essentially unchanged (0.3482 → 0.3325), which rules out the
stick-slip reading that §4's reasoning suggested and that the asymmetry between
the modes would otherwise support. It does help common mode (0.0833 → 0.0069),
so the caster is a real drag on straight-line motion — just not the instability.

Why yaw is the worse mode is therefore **not** settled. It is the softer degree
of freedom and it is the one the caster contact couples into, but the caster
test says the coupling is not through friction. Recorded as open in §9.

## 6. A5 — the error decomposed into three layers

Ground truth for the body pose is free in a simulator and impossible on
hardware, so this is a use of the apparatus rather than a workaround for it.
Getting it required one change per backend, and the two changes are not
symmetric — which is itself the first result:

| backend | what `/odom` is | ground truth from |
|---|---|---|
| gazebo | **wheel-integrated.** `/odom` and a forward integration of `/joint_states` agree to five decimals *by construction*, so slip is invisible in both | `/ground_truth/odom`, a P3D plugin added by `backends/gazebo.launch.py` |
| isaacsim | **the true body pose.** `IsaacComputeOdometry` reads the chassis prim | `/odom` itself |
| real | wheel-integrated, with real slip | **nothing** — the two lower layers are not separable at all |

So the two simulators' `/odom` topics do not mean the same thing. Comparing
them to each other compares an encoder model with a motion-capture system.

### Gazebo, `sequence:=sweep` (`measurements/2026-09-16_pre_gazebo_sweep.json`)

| phase | wheel-integrated | true body | `/odom` | wheel → body (slip) | body → odom |
|---|---|---|---|---|---|
| `lin_0.10` | 0.49307 m | 0.49377 m | 0.49307 m | +0.14% | −0.14% |
| `lin_0.15` | 0.73576 m | 0.73771 m | 0.73576 m | +0.26% | −0.26% |
| `lin_0.22` | 1.07147 m | 1.07565 m | 1.07895 m | +0.39% | +0.31% |
| `rot_0.2` | 0.99226 rad | 0.99564 rad | 0.99224 rad | +0.34% | −0.34% |
| `rot_0.5` | 2.47810 rad | 2.47215 rad | 2.47765 rad | −0.24% | +0.22% |

**Every layer of the Gazebo chain agrees to better than 0.4%.** Command → wheel
is exact (§2), wheel → body slips by a few tenths of a percent, and body → odom
recovers it. There is essentially no actuation error, no slip and no odometry
error to find.

That is worth stating plainly rather than treating as a null: **the reference
backend does not model the phenomena the sim-to-real gap is made of.** With
`mu = 100000` on the wheels the tyres cannot slip, and with an ODE joint motor
the actuator cannot lag. A controller tuned against Gazebo has never met either.

## 7. Looking for an operating point that meets A4

`measurements/isaac_drive_probe3.json`, `isaac_drive_probe4.json`. Error as a
percentage of the commanded wheel velocity, over the A4 matrix; `sd` is the
standard deviation of the joint velocity within the steady-state window, i.e.
the chatter itself.

| setting | rtf* | diff % at wz = 0.2 / 0.5 / 1.0 | common % at vx = 0.10 / 0.15 / 0.22 | typical sd (diff) |
|---|---|---|---|---|
| **60 Hz, as shipped** | 0.65 | **47.2 / 25.5 / 15.8** | 2.85 / 2.07 / 1.40 | ~0.45 |
| 240 Hz | 0.15 | 25.3 / 8.70 / 3.39 | 0.26 / 0.06 / 0.11 | 0.12–0.26 |
| 240 Hz, no sleep/stabilisation | 0.14 | 28.6 / 9.21 / 2.58 | 0.26 / 0.28 / 0.18 | 0.06–0.27 |
| 240 Hz + 64 solver iterations | 0.14 | 49.9 / −2.03 / −0.10 | 0.19 / 0.20 / −0.09 | **0.47–0.91** |
| **480 Hz** | 0.07 | **3.39 / 3.84 / −0.48** | **0.00 / 0.05 / 0.05** | **0.09–0.17** |
| 480 Hz + 64 iterations | 0.07 | 13.1 / −2.98 / −2.03 | −0.18 / −0.03 / 0.20 | 0.21–0.64 |
| 960 Hz + 64 iterations | 0.03 | −1.22 / 0.10 / −1.42 | −0.32 / −0.03 / 0.08 | 0.31–0.98 |

\* as this probe accounts it; the real figure is measured through the launcher
in §8, because `set_dt` sets PhysX sub-steps per frame rather than the frame
rate itself.

Three things fall out, and two of them are traps:

1. **Disabling sleep and stabilisation does nothing** (25.3 → 28.6 at the worst
   point, inside the scatter). The low-rate regime is not being clamped by a
   velocity threshold. That hypothesis is dead.
2. **More solver iterations is a trap.** It reduces the *mean* error — at 960 Hz
   with 64 iterations every number is inside 1.5% — while **tripling the
   chatter**: `sd` goes from 0.17 to 0.47–0.98 rad/s. The plant gets noisier and
   its average gets better, and if only the mean is reported that reads as a
   fix. It is not one: a controller has to track the actual signal, not its
   average. **Do not raise the iteration count.**
3. **The residual differential error is scatter, not bias.** At one fixed
   setting the wz = 0.2 error came out 25.3, 28.6, 49.9, 3.4 and 13.1% in
   different runs, and several rows are *negative* — the wheel overshoots. A
   single-shot measurement of the pivot at 0.2 rad/s does not mean anything,
   which is A6's question arriving early and uninvited.

**The operating point is 480 Hz with the solver left alone.** Common mode is
inside 0.06% at every speed, differential inside 3.9% and falling to 0.5% at
1.0 rad/s, with the lowest chatter of any setting tried.

