# Lane A — the actuation layer, measured

Overnight 2026-09-16/17, against the brief in
`docs/worknotes/2026-09-16-overnight-brief.md` §2. Written for someone reading
it cold.

**Headline, in three lines:**

1. **H1 is dead.** "The drive is damping-limited, raise the gain" was killed by
   measurement, not argument: the damping moved 10,000x and the error did not
   move at all. NVIDIA's documented `1.0e7` would have changed nothing.
2. **The wheel is not slow, it is unstable.** Commanded a steady −1.2121 rad/s
   it ranges over −2.91 to +1.04; the "30% under-rotation" on record since
   2026-09-15 is the *mean of a chattering signal*. Gazebo's sd is 0.0000.
3. **The cause is the wheel's collision shape** (§13). The wheels are cylinders,
   which no solver here rolls exactly. Replacing them with spheres cuts the
   chatter 40x at the original timestep. Running physics at 480 Hz — the fix
   applied tonight — damps the same symptom and fixes translation completely;
   the two together meet the whole acceptance matrix inside 0.54%.

Tonight's shipped change is the 480 Hz sub-step rate. The collider is reported
with numbers and deliberately not changed; §13 says why.

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

## 8. A3 — the fix, and what it is not

**It is not a change to the asset, and not a gain.** The brief's A3 assumed the
fix would be a drive parameter written by `scripts/import_turtlebot3.py`. The
measurements say otherwise, so the fix is where the mechanism is: the physics
sub-step rate.

`turtlebot3_isaacsim`, commit `c4a224e`:

```
launch/isaacsim.launch.py     physics_hz    default 60.0 -> 480.0
runtime/turtlebot3_isaacsim.py --physics-hz default 60.0 -> 480.0
```

Both, so the launcher and a direct invocation agree. The reasoning sits on the
launch argument, where someone changing the number will read it.

`import_turtlebot3.py` and `models/` are **untouched** — no re-bake, no asset
diff, and the `1.0e5` damping with its "TODO unverified gain" comment is left
exactly as it was. It is no longer known to be wrong; it is known not to
matter, which is a different and better-supported statement. The comment should
eventually say so.

### What it costs

Measured through the launcher, not inferred: **RTF 0.68** at 480 Hz — sim
13.63 s in 20.00 s of wall clock.

And `/clock` still publishes at **41 Hz**. That confirms the mechanism is what
the code comment claims: `SimulationManager.setup_simulation(dt=)` resolves to
`PhysxScene.set_steps_per_second`, i.e. PhysX sub-steps *within* a frame. The
OmniGraph, and therefore every ROS topic and the whole interface contract,
still ticks at the render rate. **Eight times the physics for about a third of
the wall clock, and no change to the published interface.**

## 9. A4 — acceptance. Partly met, and the part that is not is named

`measurements/2026-09-17_post_isaacsim_sweep.json` against
`measurements/2026-09-16_pre_isaacsim_sweep.json`. Same instrument, same
sequence, same world, same commit of everything except `physics_hz`.

### Layer 1: wheel tracking against the analytic command (the A4 target)

| command | 60 Hz L / R | 480 Hz L / R | within 1%? |
|---|---|---|---|
| `vx = 0.10` | 0.9762 / 0.9731 | **1.0028 / 0.9995** | **yes** |
| `vx = 0.15` | 0.9846 / 0.9819 | **0.9992 / 0.9984** | **yes** |
| `vx = 0.22` | 0.9906 / 0.9876 | **1.0002 / 0.9990** | **yes** |
| `wz = 0.2` | 0.4830 / 0.4746 | 0.9867 / 0.8778 | no — R is 12.2% short |
| `wz = 0.5` | 0.7437 / 0.7438 | 0.9723 / 0.9634 | no — 2.8 / 3.7% |
| `wz = 1.0` | 0.8745 / 0.8745 | 1.0110 / 1.0122 | no — 1.1 / 1.2% |
| `wz = 1.5` | 0.9589 / 0.8925 | 0.9977 / 1.0085 | borderline — 0.2 / 0.9% |

**Translation meets the 1% criterion at every speed. Rotation does not**,
though it improves from 48% to 88% at the worst point.

### Chatter, which is the thing actually being fixed

| phase | 60 Hz sd | 480 Hz sd | | 60 Hz range | 480 Hz range |
|---|---|---|---|---|---|
| `rotate` wz 0.5 | 0.4463 | **0.1164** | −3.8x | −2.91 … +1.04 | −1.89 … −0.51 |
| `straight` vx 0.15 | 0.0698 | **0.0133** | −5.2x | 4.13 … 4.59 | 4.51 … 4.59 |

The wheel no longer reverses direction under a steady command.

### The `drive_test` acceptance numbers

| | commanded | accept | 60 Hz | 480 Hz | |
|---|---|---|---|---|---|
| rotate | 2.500 rad | ≥ 2.45 | 1.804 | **2.195** | **FAILS** (87.8%) |
| straight | 0.750 m | ≥ 0.735 | 0.7161 | **0.7352** | **passes**, by 0.0002 m |

### F5 — did rotation and translation improve together?

**Yes.** Rotation 72.2% → 87.8%, translation 95.5% → 98.0%, in the same run
from the same single change. F5 was the check on whether the mechanism was the
right one, and it passes.

### The `wz` sweep is not flat

80.0 / 87.8 / 98.8 / 94.5% at wz = 0.2 / 0.5 / 1.0 / 1.5. **The low-rate regime
is still the worst**, and that is Nav2's final-alignment regime. A4 asked for a
flat sweep and this is not one.

### Where the remaining error went — it changed layer

| | 60 Hz | 480 Hz |
|---|---|---|
| command → wheel (actuator) | the dominant error: 25–52% | 0.2–3.7%, and ~12% at `wz = 0.2` |
| wheel → body (slip) | −0.4 to −3.9% | **−2.5 to −8.5%** |

At 60 Hz the wheels never reached their command and the robot barely slipped. At
480 Hz the wheels track and **the robot genuinely slides** in a pivot. The
residual has moved from the actuator into the contact, which is a more honest
place for it — a real burger pivoting on two wheels and a plastic skid does
slip — but it is now the thing standing between this and the 1% target, and it
has not been characterised.

**Verdict: the translation half of A4 is met and the rotation half is not.**
Reported as such rather than as a fix.

## 10. A6 — repeatability, and how many repeats the navigation experiment needs

Ten complete runs of `sequence:=default` on Isaac Sim at 480 Hz, each a fresh
simulator (`measurements/2026-09-17_rep_isaacsim_r1..r10.json`). A fresh boot
each time on purpose: that is what varies between two `nav_test` runs, so
anything held constant inside one process would understate the spread.

| metric | mean | sd | cv | range |
|---|---|---|---|---|
| `straight` distance | 0.73207 m | 0.00487 | **0.66%** | 0.0170 |
| `rotate` yaw | 2.24431 rad | 0.03560 | **1.59%** | 0.0960 |
| `arc` yaw | 1.28306 rad | 0.02067 | 1.61% | 0.0718 |
| `arc` distance | 0.45112 m | 0.00961 | 2.13% | 0.0335 |
| `straight` wheel tracking | 1.00069 | 0.00059 | **0.06%** | 0.0020 |
| `rotate` wheel tracking | 0.96942 | 0.01701 | 1.76% | 0.0525 |

`docs/status.md` records the pre-fix spread as 0.0063 m and 0.042 rad; at
480 Hz it is 0.0049 m and 0.0356 rad. **Slightly more repeatable than before,
not less** — worth saying, because raising a solver rate could easily have gone
the other way, and because the 64-iteration variants in §7 show what it looks
like when it does.

Note the split: the *actuator* is now almost perfectly repeatable in a straight
line (cv 0.06%), while everything involving rotation sits near 1.6–2.1%. The
run-to-run variability lives in the same place the residual error does — the
contact, in a pivot.

### Repeats needed, for a two-sided test at 95% with 80% power

`n = 2 (2.8 s / d)²` per arm:

| to resolve a difference of | in `rotate` yaw | in `straight` distance |
|---|---|---|
| 5% | 2 | 1 |
| 2% | **10** | 2 |
| 1% | **40** | 7 |

**So `nav_test` needs about 10 runs per cell to call a 2% difference, and 40 for
1%.** The existing `measurements/nav2.md` compares three Gazebo runs against two
Isaac runs and reports a 4.5% spread in time-to-goal; at n = 3 that design can
only resolve differences of roughly 5% and above, so its conclusion ("Nav2
reaches the goal on both backends, in the same time") is supported for large
effects and cannot exclude a small one.

**Caveat.** These are `drive_test`'s open-loop numbers. `nav_test` closes the
loop, and a controller both suppresses plant noise and adds its own, so the
variance of a navigation metric has to be measured rather than inherited from
here. The right reading of this table is "expect to need ~10, measure it".

## 11. A7 — the perception layer, against the manifest's own geometry

`ros2 run tb3_bringup scan_test`, new tonight. The robot is parked at the
manifest spawn in `small_office` and never commanded, so nothing here is
confounded by §9's actuation residual. The expected range down every beam is
**ray-cast from `worlds/small_office/world.yaml`** — the same file both
simulators were generated from — so there is no reference scan and no
hand-measured room.

The room is five boxes and the props are seven meshes. Only the boxes are
ray-cast, which is the method rather than a limitation: a prop can only ever
*occlude* a wall, so the box-only prediction is an upper bound and each beam
sorts itself without anyone modelling a chair.

| | **gazebo** |
|---|---|
| beams crossing the scan plane | 5 boxes |
| valid return fraction | 0.8056 |
| **wall range error vs analytic** | **mean +0.00052 m, sd 0.00236 m, max 0.00607 m** (n = 234) |
| prop returns (shorter than the wall behind) | 56 |
| **leaks** (a beam passing through a wall) | **0** |

**Gazebo's lidar reproduces the manifest to half a millimetre of mean error and
2.4 mm of spread**, against an LDS-01 spec of ±15 mm. Zero leaks, so the
generated SDF geometry is where the manifest says it is, to the sensor's
ability to tell.

The 19% of beams with no return are the far corners of a 6 × 5 m room seen from
(−1.5, −1.5) with a 3.5 m maximum range — geometry, not a fault, and measured
identically on both backends.

**This is the near-null the four-layer table predicted for perception**, and it
is what licenses the claim that the gap lives in actuation: the same apparatus
that finds a 30% actuation error finds a 0.5 mm perception error.

### Isaac Sim, same pose, same analytic reference

| | gazebo | **isaacsim** |
|---|---|---|
| valid return fraction | 0.8056 | 0.8250 |
| wall range error, mean | +0.00052 m | +0.00085 m |
| wall range error, sd | **0.00236 m** | **0.01509 m** |
| max abs error | 0.00607 m | 0.04691 m |
| prop returns | 56 | 68 |
| leaks | **0** | **9** |

Two differences, and the second one is a real defect.

**Isaac's lidar is 6.4x noisier — and that is correct.** An sd of 15.1 mm is
the LDS-01's own ±15 mm specification; `turtlebot3_isaacsim` models the sensor's
noise and the Gazebo plugin, as configured, essentially does not. Neither has a
range *bias*: both means are under a millimetre.

**Isaac's scan is rotated by half a degree.** The 9 leaks are not holes in a
wall: they are one contiguous run of beams, indices 154–162, all against
`wall_south` at grazing incidence, each reading 5–7 cm long. At grazing
incidence `dr/dθ` is about 4.9 m/rad, so a fraction of a degree of misalignment
becomes centimetres of range error there and is invisible everywhere else.

Fitting the angular offset that minimises the wall residual, over every wall
beam:

| | best-fit offset | residual rms |
|---|---|---|
| gazebo | **+0.0000 rad (+0.000°)** | 0.00242 m |
| isaacsim | **+0.0095 rad (+0.544°)** | 0.00608 m, down from 0.01509 |

Gazebo's scan is aligned exactly. Isaac's is off by **0.544°, which is 0.54 of
one 1.0° beam** — close enough to half a bin to suggest a beam-indexing
convention difference (range reported at the edge of an azimuth sector rather
than its centre) rather than a physical mounting error. Removing it cuts the
residual by 2.5x and accounts for every leak.

**This is exactly the class of error the repository exists to catch** — F6
warned about "a silent few-centimetre lidar offset", and this is its angular
twin. It is small, it is systematic, it biases every scan match slightly, and
nothing else would have found it: it is only visible against an analytic ground
truth, which only exists because the world is generated from a manifest.

Cause not established — half a beam is a strong hint, not a diagnosis. Next
test in §13.

## 12. A8 — `nav_test`, built and run

New tonight: `ros2 run tb3_bringup nav_test`. Sends `NavigateToPose` goals from
a per-world fixed list and records, per goal: status, time on the simulator's
clock, path length against straight-line distance, final position and heading
error from `map -> base_footprint`, recovery-behaviour count, and closest
approach to an obstacle — with a rosbag alongside so a metric nobody thought of
tonight can be derived without re-running the matrix. One instrument, all
backends, and one `nav2_params.yaml`, per F7.

`world:=turtlebot3_world`, `nav:=true`, AMCL seeded at the manifest spawn.

| goal | | gazebo | isaacsim @ 480 Hz |
|---|---|---|---|
| `far_corner` | time / pos err / yaw err | 22.10 s / 0.2269 m / 0.0372 rad | 22.45 s / 0.2455 m / 0.0514 rad |
| `spin_in_place` (yaw + π) | | 8.60 s / 0.0565 m / 0.1823 rad | 11.25 s / 0.0476 m / 0.2380 rad |
| `back_home` | | 27.90 s / 0.0270 m / 0.2012 rad | 29.82 s / 0.0146 m / 0.2326 rad |
| recoveries | | 0 / 0 / 0 | 0 / 0 / 0 |
| closest approach | | 0.345 / 0.397 / 0.356 m | 0.400 / 0.452 / 0.400 m |

Both backends complete every goal with no recovery behaviour. Times are within
1.6% on the translation goals and 31% apart on the pure rotation
(8.60 s vs 11.25 s) — the one goal that exercises the axis with the remaining
deficit, and the direction is what §9 predicts.

**One run each, so per §10 this resolves differences of roughly 5% and up. The
31% rotation gap is comfortably outside that; the 1.6% translation difference
is not, and must not be read as a difference at all.**

### What is missing from A8, and why

The before/after navigation comparison on Isaac Sim — the same matrix at 60 Hz —
**was not obtained.** Two attempts, both lost to a container-level failure
rather than anything about the physics:

1. First attempt: `map_server` started but never advertised
   `map_server/get_state`, so `lifecycle_manager_localization` waited forever
   and the run never began. Nav2's own processes were alive; discovery was not.
2. Second attempt, after killing every stale process: Isaac Sim reached
   `isaacsim.ros2.bridge` startup and stopped advancing, and `ros2 topic list`
   returned nothing at all.

The container had by then been through roughly twenty simulator launches.
`docker compose restart tb3_ros` cleared it, and the very next Isaac run
(`scan_test`, §11) came up in 16 s. **The signature is accumulated DDS or
shared-memory state, not the 60 Hz setting**, and it is worth knowing about
because it presents as a hung simulator.

Rerunning it needs the `physics_hz` plumbing in §13 and about ten minutes.

## 13. The cause: it is the wheel collider

The last test of the night, and it changes the recommendation.

The wheels are `UsdGeom.Cylinder` prims. **Neither PhysX nor MuJoCo has a true
rolling cylinder** — both approximate one, and a faceted wheel bumps as it
rolls. A sphere of the same radius is the control: it touches at one exact
point, every solver handles it analytically, and it rolls identically about the
wheel axis. Swapped in memory, never on disk
(`measurements/isaac_wheel_collider.json`):

| | diff % at wz = 0.2 / 0.5 / 1.0 | common % at vx = 0.10 / 0.15 / 0.22 | chatter sd (diff) |
|---|---|---|---|
| 60 Hz, cylinder (as shipped) | 46.3 / 24.9 / 12.6 | 2.50 / 1.61 / 1.18 | 0.163 / 0.165 / 0.759 |
| **60 Hz, sphere** | **13.9 / 6.5 / 3.9** | 2.79 / 1.86 / 1.28 | **0.0041 / 0.0163 / 0.0156** |
| 480 Hz, cylinder (tonight's fix) | 4.9 / 3.9 / −0.9 | −0.04 / −0.04 / −0.04 | 0.135 / 0.249 / 0.171 |
| **480 Hz, sphere** | **0.54 / 0.23 / 0.13** | **0.02 / 0.12 / −0.01** | **0.0032 / 0.0038 / 0.0034** |

**Changing only the collider shape, at the original 60 Hz, cuts the chatter by
a factor of 40** (sd 0.163 → 0.0041) and the differential error by 3.3x. No
timestep change, no gain change.

And the two effects are separable, which is the useful part:

- **The cylinder causes the chatter.** A sphere removes it at any rate.
- **The timestep causes the common-mode error.** A sphere does not help it at
  all (2.50 → 2.79%); 480 Hz takes it to 0.04%.
- **Together they meet A4 in full** — every cell of the acceptance matrix
  inside 0.54%, both modes, with sd 0.003.

So tonight's fix was damping a symptom. It is a real improvement and the
mechanism was correctly identified as the contact solve, but **the source is
the wheel's collision shape**, and that is an asset question.

### Not the analytic-cylinder setting

PhysX can represent cones and cylinders analytically instead of as convex
hulls. Those settings do not exist in this build: `/physics/collisionCone...`
and `/physics/collisionCylinderCustomGeometry` both read `None`, and forcing
them produced results **bit-identical** to the baseline
(`measurements/isaac_cylinder_custom.json`) — a clean negative control, and
consistent with Isaac Sim 6.1 running Newton rather than classic PhysX.

### Why the asset was NOT changed tonight

A sphere is not a tyre. It touches at a point, so it does not resist sliding
along the wheel axis or contribute to tipping resistance the way a 0.018 m-wide
cylinder does. On flat ground driving forward that is invisible; for a robot
that pivots on two wheels and a skid it may not be, and `small_office` has
props to bump into. Swapping the collider is a change to how the robot contacts
the world in **every** measurement this repository will ever take, and it
should be chosen deliberately with lateral behaviour checked — not at the end
of an unattended night on the strength of six numbers.

The timestep fix is in and validated. The collider finding is reported with its
measurements so the decision can be made in daylight.

## 14. What is known, what is not, and what to do next

### Settled

| | |
|---|---|
| **H1** drive is damping-limited | **DEAD.** `D` 1e3 → 1e7 (10,000x), error 0.288 → 0.332 rad/s. `err*D` spans 76 to 3.6e6 N·m. |
| **H2** torque saturation | **DEAD.** Runtime max effort is FLT_MAX; no `maxForce` authored. |
| **H0** the OmniGraph emits a wrong command | **DEAD.** Deficit reproduces with ROS, the DifferentialController and the ArticulationController all bypassed. |
| **H4** odometry bug | **DEAD.** Visible in the articulation's own joint velocities; and Isaac's `/odom` is the chassis prim, so it cannot drift from the body. |
| **H6** sleep / stabilisation threshold | **DEAD.** Disabling both changes nothing (25.3 → 28.6%, inside the scatter). |
| **F3** the mujoco variant is live | **FALSE.** Stage composes `Physics = "physx"`; no `MjcActuator` prims exist. The degree-converted gain in `mujoco.usda` is latent, and the physx path's deg→rad conversion is correct (1745.3292 → exactly 1e5). |
| **H3 / H5** contact, and how it is solved | **SUPPORTED.** Off the ground the error is exactly 0.0000; on the ground it falls monotonically with sub-step rate. |

### Not settled

1. ~~Why the differential mode is so much worse~~ — **answered in §13: the
   cylinder wheel collider.** What remains open is the *decision*: sphere
   colliders, some Newton-specific collider option not yet investigated, or
   keeping the finer timestep. Needs a lateral-contact check either way.
2. **Why the residual scatters so much at `wz = 0.2`** — 25.3, 28.6, 49.9, 3.4
   and 13.1% at one fixed setting. **Next test:** hold each rate for 30 s rather
   than 2.5 s and repeat 5x; decide whether the mean is converging slowly or the
   process is genuinely non-stationary.
3. **The 0.544° lidar rotation** (§11). **Next test:** command a known yaw, or
   simply park at several yaws, and check whether the offset is constant in the
   sensor frame (an indexing convention) or varies with heading (a mounting or
   tf error). Then read `horizontalResolution` and the azimuth-to-index mapping
   in `turtlebot3_isaacsim`'s lidar profile against `models/lidar_configs/turtlebot3_lds.json`.
4. **Whether 480 Hz is the right price.** RTF 0.68 is comfortable, but 240 Hz
   (RTF would be higher) already fixes translation completely and gets rotation
   to 8.7%. If rotation is fixed properly by (1), the rate could come back down.

### Recommended next steps, in order

1. **Decide the wheel collider (§13).** Sphere colliders meet the full
   acceptance matrix; the open question is what they cost laterally. Test: push
   the robot sideways and into a prop with each shape and compare. If a sphere
   holds up, the change belongs in `import_turtlebot3.py` and the physics rate
   can probably come back down from 480 Hz, which buys back real time.
2. **Finish plumbing `physics_hz`.** Done tonight at the backend:
   `backends/isaacsim.launch.py` now declares and forwards it (default 480.0).
   It is still not settable from `bringup.launch.py`, whose `backend_args` is a
   fixed dict — **one line there**, deliberately left alone because lane C was
   rewriting that file.
3. **Rerun A8's before/after** once (2) exists, with 10 runs per cell per §10.
4. **Fix `measurements/isaac_angular_deficit.md`.** Its conclusion ("the drive
   gain is too low for the wheel friction") is now known to be wrong, and it is
   the document a reader would find first. It should keep its measurements —
   they are correct and they reproduce — and say what they actually meant.
   Likewise `docs/status.md`'s "Isaac Sim currently under-rotates … It is a
   wheel-drive gain" and the same claim in `CLAUDE.md`.
5. **Consider a robot-level equivalent of `check_worlds.py`** (§4). The world is
   identical across backends by construction and the *robot* is not: wheel
   friction is 1e5 in Gazebo and 1.0 in Isaac Sim, and nothing compares them.

### Things a reader should not conclude from this report

- **Not** that Isaac Sim is fixed. Translation meets the 1% target; rotation
  does not, and the low-rate pivot — Nav2's final-alignment regime — is still
  the worst case at 80% of commanded yaw.
- **Not** that Gazebo is the accurate one. It is *exact*, which is different:
  an ideal velocity source with `mu = 1e5` wheels cannot lag and cannot slip.
  Neither simulator's actuator resembles a real one; they fail in opposite
  directions.
- **Not** that the gain in `import_turtlebot3.py` is correct. It is *unverified
  and irrelevant* — four decades of it change nothing measurable.

## 15. Gates and housekeeping

- `scripts/check_worlds.py` **before**: 3/3 worlds consistent.
- `scripts/check_worlds.py` **after**: 3/3 worlds consistent. No world artifact
  was regenerated and nothing under `worlds/`, `measurements/` or `maps/` was
  deleted.
- No `git push`, no force operations, no amends. All commits local, on
  `overnight/2026-09-16`, plus two in `src/turtlebot3_isaacsim` (its own repo,
  also local).
- `src/turtlebot3_isaacsim` was modified by lane B during the session. The
  burger asset's SHA-256 was unchanged throughout, so every drive measurement
  here is against one asset. Lane B's `SCAN_OFFSET` fix touched waffle only.
- One `drive_test` run was discarded: a simulator survived teardown and two
  robots published `/odom` on two clocks. The harness now refuses to start
  beside a live publisher, and `drive_test`'s samples carry a
  backwards-time check that catches it after the fact.

### For the morning, to run by hand

Nothing needs a remote. The only commands worth running are the ones in §13.
