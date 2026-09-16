# Overnight brief — 2026-09-16

Written for an agent working unattended. **Everything you need is in this file**
— paths, line numbers, measured values, acceptance numbers. Do not spend the
night re-deriving context from the repository.

There is no real robot available. You have Isaac Sim and Gazebo Classic in the
`tb3_ros` container, which is already up.

---

## 0. Context, in six lines

`tb3_sim2real` runs Nav2 on a TurtleBot3 against three interchangeable backends
— real hardware, Gazebo Classic, Isaac Sim — from one launch command, with the
environment generated for all three from one manifest so that geometry is
identical by construction and the backend is the only variable between two runs.
The Isaac side lives in `src/turtlebot3_isaacsim`, a separate repository
imported by `scripts/workspace.sh`; it is a peer of `turtlebot3_gazebo` and owns
the robot asset, the OmniGraph, the lidar profile and the physics materials.
Read `CLAUDE.md` at the repo root if you need more. Do **not** read
`docs/history.md`.

### The thesis this work serves

Not "I compared two simulators". The claim is **where in the stack the
sim-to-real gap actually lives**, in four layers:

| layer | how it is handled | expected finding |
|---|---|---|
| geometry | identical **by construction** — one manifest to SDF + USD, meshes shared byte-for-byte, `scripts/check_worlds.py` proves it | zero by design; this is the methodological contribution |
| perception | same LDS-01 profile both sides; measurable against manifest geometry | small — predicted not to matter |
| **actuation** | `drive_test`, open loop, one instrument for all backends | **diverges — already measured** |
| task | Nav2 navigation runs, same map, same goals | the actuation gap propagates here |

Layers 1 and 2 exist to license the claim about layers 3 and 4. Tonight is
mostly layer 3, plus the instrument for layer 4.

### The one methodological rule

**Do not tune Isaac Sim to match Gazebo.** Each backend is verified
independently against the *analytic* commanded joint velocity. If they then
agree, that is a result. Fit one to the other and the comparison the thesis is
made of is destroyed. This rule is not negotiable and it is the easiest one to
break by accident.

---

## 1. What I found, and why it matters

### F1 — Ground truth for the wheel gain is the command, not the robot

This was the open question ("tune it to what?"). The answer is that no robot is
needed.

The differential-drive kinematics are exact and unit-checked. For the burger,
wheel separation `L = 0.160 m`, wheel radius `r = 0.033 m`
(`scripts/turtlebot3_isaacsim.py:45`):

```
omega_left  = (vx - wz*L/2) / r
omega_right = (vx + wz*L/2) / r
```

The OmniGraph computes those numbers; the actuator's only job is to make the
joint spin at them. **A joint that does not reach its own commanded velocity is
broken, not "different from reality."** Gazebo tracks to −0.9%; the real
OpenCR runs an encoder PID and tracks too. "Track the command" is the shared
contract of all three backends.

So the gain is tuned against an analytic target with no hardware present, and
the acceptance test is a number (§2.4), not a comparison.

What *does* need the real robot is a different question and is **not** tuning:
the transient (Gazebo ramps over ~0.10 s, Isaac steps in one sample) and the
slip. Those are fidelity comparisons for later. Keep them out of tonight.

### F2 — NVIDIA's own tutorial specifies 100x the damping currently used

`src/turtlebot3_isaacsim/UPSTREAM.md:464`, the repo's own research note:

> The TurtleBot3 tutorial also says to run the Gain Tuner and set Damping/Kd to
> `10000000.0` on `wheel_left_joint` and `wheel_right_joint` — velocity drives
> need zero stiffness and non-zero damping or the wheels will not track commands.

`src/turtlebot3_isaacsim/scripts/import_turtlebot3.py:95-98`:

```python
# TODO unverified gain. Without it the importer warns that the actuator
# is created with no gain parameters.
override_joint_damping=1.0e5,
override_joint_stiffness=0.0,
```

`1.0e5` against a documented `1.0e7`. The comment says outright that the value
was never verified. **Preferring the vendor's documented value over a
hand-picked one is the house rule here**, so `1.0e7` is the first thing to try
— but see F4 before concluding anything from it.

### F3 — A probable unit bug in NVIDIA's URDF converter, carried into the asset

This is the more interesting finding and it may be the actual root cause.

The committed asset, at
`src/turtlebot3_isaacsim/models/turtlebot3_burger/turtlebot3_burger.usd/turtlebot3_burger/`:

```
payloads/Physics/physics.usda:95,113   drive:angular:physics:damping = 1745.3292
payloads/Physics/mujoco.usda:36,48     mjc:gainPrm[0]  =  1745.3292236328125
payloads/Physics/mujoco.usda:32,44     mjc:biasPrm[2]  = -1745.3292236328125
```

`1.0e5 * pi/180 = 1745.3292519943293`. Exact match (float32 rounding aside).

That conversion is **correct for USD**, whose angular drive damping is expressed
per degree/s. But **MuJoCo works in radians**, and the converter
(`URDF USD Converter v0.1.3`) wrote the degree-converted number into the MuJoCo
actuator as well. Worse, `mujoco.usda:55,61` *deletes* `PhysicsDriveAPI:angular`
from both wheel joints and replaces them with `MjcActuator` prims:

```usda
over "wheel_left_joint" (
    delete apiSchemas = ["PhysicsDriveAPI:angular", "PhysicsJointStateAPI:angular"]
)
```

Isaac Sim 6.1 runs the Newton backend, which is MuJoCo-backed. **If the mujoco
variant is the one active at runtime, the PhysX damping is not the gain that
runs at all, and the effective damping is 1745 rather than the intended 1e5 — a
factor of 57.3 (= 180/pi) short, from a bug in someone else's converter.**

The actuator is a plain velocity servo: `gainType="fixed"` with
`gainPrm[0]=k` gives force `= k*ctrl`, and `biasType="affine"` with
`biasPrm[2]=-k` gives `-k*velocity`, so `tau = k*(u - v)`. `forceRange` is
±inf, so it is not force-limited in the MuJoCo path.

**First thing to establish tonight: which Physics variant is actually selected
at runtime.** Everything else depends on it.

### F4 — The arithmetic does not close, so do not declare victory on the gain

Steady-state error for a velocity servo is `err = tau_resist / D`.

From `docs/status.md` and `measurements/isaac_angular_deficit.md`, the measured
deficits, converted to wheel angular velocity (my derivation, corroborating the
"constant absolute error" claim in status.md):

| command | commanded wheel omega | measured deficit | wheel omega deficit |
|---|---|---|---|
| `vx = 0.15 m/s` | 4.5455 rad/s | −4.7% | 0.214 rad/s |
| `wz = 0.2 rad/s` | 0.4848 rad/s | −52% | 0.252 rad/s |
| `wz = 0.5 rad/s` | 1.2121 rad/s | −30.3% | 0.367 rad/s |

Roughly constant in absolute terms — the signature of a fixed resisting torque
against a finite-gain drive.

But the implied torque is not physical. At `D = 1745`, `tau = 1745 * 0.3 ~ 520
N*m`. At `D = 1e5`, `~30 kN*m`. A Dynamixel XL430's stall torque is about
1.4 N*m. **So even if raising the damping removes the symptom, the mechanism is
not understood.**

Therefore the single most important instruction in this brief:

> **`err * D` must be constant.** Measure the steady-state error at the current
> `D` and at `100*D`. If the error does not fall by ~100x, the damping-limited
> hypothesis is **dead**. Record the numbers, say so in the report, and move
> down the hypothesis list. **Do not keep cranking the gain until the symptom
> goes away.**

A symptom removed by an unexplained mechanism is worthless in a thesis, and
worse than worthless if it hides the real cause.

### F5 — A prediction that checks the whole diagnosis

A constant wheel-rate deficit of ~0.21–0.37 rad/s also accounts for the −4.7%
straight-line error (want 0.750 m, got 0.7175 m). So:

> If the fix is right, **rotation and translation improve together.** If
> rotation improves and translation does not, the mechanism is wrong.

Test it. It is free — `drive_test` already measures both.

### F6 — The asset is missing four URDF frames; I believe the doc is stale, not the code

`scripts/import_turtlebot3.py:89` sets `merge_fixed_joints=True`. Frame counts
in the committed asset:

```
base_footprint: present     base_link:        ABSENT
wheel_left_link: present    base_scan:        ABSENT
wheel_right_link: present   imu_link:         ABSENT
                            caster_back_link: ABSENT
```

`UPSTREAM.md` explicitly warns against exactly this: *"Leave `merge_fixed_joints`
at `False` … Enabling it would destroy `base_footprint`, `base_scan` and
`imu_link` — frames nav2 needs."* `DESIGN.md` separately claims
`base_footprint` survives as a real prim.

**I do not think this is a bug.** `import_turtlebot3.py:155` reads "Root
first, as the fallback for everything merge_fixed_joints folded into" — the
code knows it merges and handles the consequence deliberately. The
architecture moved to
`robot_state_publisher` supplying everything above `base_footprint` from
`turtlebot3_description`, so the Isaac asset only needs to supply
`odom -> base_footprint` plus sensor topics. The doc predates that.

**Do not change `merge_fixed_joints`.** Instead: verify where the lidar prim is
attached and whether its offset matches the URDF's `base_scan` origin — a silent
few-centimetre lidar offset is precisely the class of error this repository
exists to prevent — then reconcile `UPSTREAM.md` and `DESIGN.md` to whatever is
true.

### F7 — `params_file()` is a hazard to the experiment, not just dead code

`tb3_bringup/launch/bringup.launch.py:74-84` selects `config/nav2_<backend>.yaml`
if it exists, else the shared `nav2_params.yaml`. No per-backend file exists.

It is not merely unused. **If Nav2 is ever tuned per backend, the sim-to-real
gap has been absorbed into the tuning and can no longer be measured.** One
params file for all three backends is a methodological requirement of the
thesis, and the code should make the wrong thing impossible rather than
convenient. Delete the function; pass `config/nav2_params.yaml` directly.

### F8 — The mode grid documents a feature that does not exist

`README.md:21-22`, `docs/roadmap.md:168-169` and
`tb3_bringup/launch/bringup.launch.py:35-36` all describe two modes as
"slam_toolbox + teleop" and "robot only; teleop". **No launch file in the
repository starts teleop** — `grep -rn teleop` finds four prose files and zero
launch files. The mode as advertised provides slam_toolbox and nothing to drive
with. Fixed by the lane C interface change below, plus a doc line naming the
second terminal.

### F9 — Map handling: three concrete defects

1. **A map carries no provenance, and it is the one artifact all three backends
   share.** Run `backend:=gazebo ... slam:=true`, drive, `save_map.py --force`,
   and the *real robot* now localises against a map recorded in Gazebo. Nothing
   warns. For a repository whose thesis is that the three must not drift, this
   is the drift vector sitting in the middle of the design.
2. **Two worlds, two naming conventions.** `turtlebot3_world` uses
   `map/map.yaml`, `small_office` uses `map/small_office.yaml`, and
   `scripts/save_map.py` defaults `--name` to `map`. So `save_map.py
   small_office` writes a second map beside the first, correctly declines to
   rewrite the manifest, prints a NOTE — and then **its closing message still
   tells you to run `nav:=true`, which will load the old map.** Small real bug.
3. **`bringup.launch.py:45` says the map lives at `worlds/<name>/map.yaml`.**
   Everything else in the repo (README, CLAUDE.md, `save_map.py`, both
   manifests) says `worlds/<name>/map/<stem>.yaml`, declared by the manifest's
   `map:` key. One stale line.

Fix 2 and 3 tonight (lane C). **Leave 1 alone** — it is a manifest schema
change and it is not on tonight's critical path. It is recorded here so it is
not lost.

---

## 2. Lane A — the physics. Serial, owns the GPU and the container.

This is the thesis content and the critical path. One GPU, one container, so
**only this lane may run a simulator.** Work in order; do not skip A0.

### A0 — Baseline before touching anything

Re-run the existing `drive_test` matrix and the angular sweep from
`measurements/isaac_angular_deficit.md` on the **current** build, both backends,
and write them to `measurements/` as the pre-change baseline with today's date.
Ten minutes. Without it, the morning's comparison is against numbers taken on a
different build and any difference is unattributable.

Record the physics dt and solver iteration counts in the same file. They must be
pinned and known or none of tonight's numbers are reproducible.

### A1 — Establish where the deficit actually is

Everything downstream depends on this and it has never been confirmed on the
current build.

1. Determine which Physics variant (`none` / `physics` / `physx` / `mujoco`) is
   selected at runtime, and therefore which gain is live (see F3).
2. Measure the wheel joint velocity from `/joint_states` against the analytic
   commanded omega from F1. This is the residual to drive to zero.
3. **Do the same for Gazebo.** Nobody has ever checked whether the reference
   backend tracks its own commanded joint velocity either. Its −0.9% rotation
   and −1.9% translation could be slip rather than drive error. If the reference
   has an unmeasured error, the claim "Isaac is the one that is wrong" is not
   defensible.

### A2 — Hypotheses, in order, each with a discriminating test

| # | hypothesis | discriminating signature / test |
|---|---|---|
| H1 | drive is damping-limited | `err * D` constant. Measure at `D` and `100*D`. **Falsified if error does not fall ~100x** (F4). |
| H2 | force / torque saturation | error grows with load rather than staying constant; check the PhysX drive `maxForce` at runtime (`mjc:forceRange` is ±inf, so not in the MuJoCo path) |
| H3 | deficit is at the wheel-ground contact, not the joint | joint tracks command but body motion does not — settled by A1 and A5 |
| H4 | odometry bug | `/odom` yaw rate vs the articulation root's ground-truth angular velocity from the stage. Only Isaac can run this test and it is nearly free. |
| H5 | solver iteration count / timestep | error falls with more position iterations at fixed `D` |

H1 is first because of F2 and F3, not because it is most likely. Respect the
falsification criterion.

### A3 — The fix

Through `scripts/import_turtlebot3.py`, or an explicit post-import patch step
inside it. **Never by hand-editing the generated USD** — a hand-edited asset is
not reproducible and the next re-import silently reverts it.

If F3 is confirmed (MuJoCo actuator running with the degree-converted gain), the
correct fix is a post-import step that writes the radian-correct value into
`mjc:gainPrm[0]` / `mjc:biasPrm[2]`, with a comment naming the converter version
and the `180/pi` factor. That is a workaround for an upstream bug and should be
labelled as one, so it can be removed when the converter is fixed.

After re-baking, **diff the asset** and accept only the intended attribute
change. The importer writes into a `_01` directory rather than overwriting
(`UPSTREAM.md`), and converter version drift can change unrelated things.

### A4 — Acceptance

Do not report success without these numbers:

- steady state `|omega_joint − omega_commanded| / omega_commanded < 1%`, at
  `wz ∈ {0.2, 0.5, 1.0}` and `vx ∈ {0.1, 0.15, 0.22}` — the analytic target
  from F1, **not** agreement with Gazebo
- `drive_test` rotate: 2.500 rad commanded, accept ≥ 2.45 (within 2%)
- `drive_test` straight: 0.750 m commanded, accept ≥ 0.735 (within 2%)
- **rotation and translation improved together** (F5)
- no oscillation or instability at the final damping value, at the pinned dt
- the full `wz` sweep is flat — the deficit does not reappear at low rates,
  which was the worst regime (52% at `wz = 0.2`) and is exactly Nav2's regime
  for final alignment

### A5 — Decompose the error into three layers

The strongest new result available tonight, and a thesis figure on its own.
Ground truth body pose is free in both simulators — Gazebo's
`/gazebo/model_states`, Isaac's articulation-root prim pose — and impossible on
hardware, so this is a use of the apparatus rather than a workaround for it.

| stage | measured as | isolates |
|---|---|---|
| command → wheel | commanded omega vs `/joint_states` | actuator / drive fidelity |
| wheel → body | `/joint_states` integrated vs ground-truth body pose | wheel slip, contact model |
| body → odom | ground-truth body pose vs `/odom` | odometry computation |

It also settles H3 and H4 as a by-product, so it pays for itself twice. Run it
on both simulators, before and after the fix.

### A6 — Repeatability, and how many repeats the navigation experiment needs

Gazebo is bit-identical run to run; Isaac is not (spread 0.0063 m, 0.042 rad per
`docs/status.md`). Run ~10 repeats of one `drive_test` sequence on Isaac, report
the standard deviation per metric, and **derive the number of repeats
`nav_test` needs** for its differences to be distinguishable from noise. This is
the first question an examiner asks about an experimental chapter.

### A7 — The perception layer, against an analytic ground truth

Park the robot at a fixed pose in `small_office` on both backends — static, so
no dynamics confound — record `/scan`, and compare measured range against the
distance to each wall **computed from `worlds/small_office/world.yaml`**. This
is possible only because the world is generated from one source of truth; it is
the registry paying off.

Report: range error vs analytic, valid-return fraction, angular alignment,
noise. `DESIGN.md` has pieces of this for Isaac alone; as a *paired* comparison
it becomes the perception row of the four-layer table. Expect a near-null
result — which is what licenses the claim that the gap lives in actuation.

### A8 — Build `nav_test` and run it

Same shape and philosophy as `drive_test`: **one instrument, all backends.**
Sends `NavigateToPose` goals from a fixed list and records per goal:

- success / failure / timeout
- time to goal (simulator clock)
- path length vs straight-line distance
- **final position error and final heading error** — where an angular deficit
  shows up
- number of recovery behaviours triggered
- minimum distance to an obstacle

Record a rosbag alongside each run. Cheap, and it means a forgotten metric can
be re-derived without re-running the matrix.

Then run it on Gazebo and Isaac on `turtlebot3_world` and `small_office`, which
both already have maps, **before and after** the gain fix. That gives
before/after navigation numbers on two backends by morning, which is a thesis
figure.

If time runs out, building `nav_test` correctly beats running it hastily.

---

## 3. Lane B — `src/turtlebot3_isaacsim` as a package other people can use

Text work, parallel-safe. **Must not touch `scripts/import_turtlebot3.py` or
`models/` — lane A owns those tonight.**

- **Branches.** The default branch is currently
  `fix-map-mirroring-add-small-worlds`, which is not a default branch name for a
  public package. Create `humble` (default, tested) and `jazzy` (holds
  `docker/isaacsim-ros2/Dockerfile.jazzy`, carrying an explicit "not tested"
  note; only Humble is tested). **Local branch work only** — write the exact
  push/rename commands into the report for the user to run in the morning.
- **Asset smoke test**, headless: articulation root present, expected frames
  present, drive parameters as intended, lidar profile
  (`models/lidar_configs/turtlebot3_lds.json`) resolves, lidar prim offset
  matches the URDF. This also settles F6 permanently instead of by argument.
- **`scripts/` is doing two jobs.** It holds user-facing scripts
  (`build_images.sh`, `build_models.sh`, `build_map.py`) alongside what looks
  like runtime code (`turtlebot3_isaacsim.py` is the simulator entry point the
  launch files invoke; `assets.py` looks like a library). Runtime code does not
  belong in `scripts/`. Propose a layout, and **state the plan in the report
  before executing it** if it changes any path a launch file references.
- **Six near-duplicate launch files** (`empty_world`, `kitchen`, `simple_room`,
  `turtlebot3_world`, `warehouse`, plus `isaacsim` and
  `robot_state_publisher`). Collapse toward one parameterised launch plus
  examples — **without breaking `launch/isaacsim.launch.py`, which
  `tb3_sim2real`'s `backends/isaacsim.launch.py` includes.** That include is a
  hard interface; verify it still works before committing.
- **Docs.** `README.md` stays. `DESIGN.md` is the "why" and is worth keeping.
  `UPSTREAM.md` is verified-against-source research and is unusually valuable —
  keep it, or move it to `tb3_sim2real/docs/` if it reads as private notes.
  Anything that is session log rather than documentation moves to
  `tb3_sim2real/docs/`, which is the repository the user can actually reach.
- Remove dead code and commentary that restates the line below it. Do not strip
  comments that document a non-obvious failure — those are the valuable ones.

## 4. Lane C — `tb3_sim2real` tidy and the experiment document

Text work, parallel-safe. **Must not touch `src/` or
`tb3_bringup/tb3_bringup/nav_test.py` (lane A).**

**The launch interface collapses to three modes.** This is decided; implement it
as specified, do not redesign it.

```
(neither)    robot + RViz. Drive it, run drive_test, attach your own stack.
nav:=true    map_server + AMCL + Nav2, on the world's saved map.
             No map -> error naming the fix: run slam:=true, then
             scripts/save_map.py <world>
slam:=true   slam_toolbox + Nav2. Navigate while building the map.
```

One sentence documents it: **`nav:=true` navigates on a saved map; `slam:=true`
navigates while making one; neither gives you a bare robot.** `slam:=true
nav:=true` is accepted and means the same as `slam:=true` — do not reject it,
that is a combination that would have to be documented.

The dispatch in `tb3_bringup/launch/bringup.launch.py` becomes:

```python
if slam:         stack.append(inc(nav2('slam_launch.py'), params_file=params))
elif nav:        stack.append(inc(nav2('localization_launch.py'), map=world.map(), params_file=params))
if nav or slam:  stack.append(inc(nav2('navigation_launch.py'), params_file=params))
```

Also in lane C:

- **Delete `params_file()`** (F7) and pass `config/nav2_params.yaml` directly.
- **Fix F8** — the mode grid in `README.md`, `docs/roadmap.md` and the launch
  docstring must stop claiming teleop is launched. Name the second terminal
  instead: `ros2 run turtlebot3_teleop teleop_keyboard`, or
  `ros2 run tb3_bringup drive_test`.
- **Fix F9.2 and F9.3** — one map naming convention (default `--name` to the
  world name), make `save_map.py`'s closing message conditional so it cannot
  tell you to run a mode that will load a different map, and correct
  `bringup.launch.py:45`.
- **Compact `bringup.launch.py`.** It is 278 lines: 110 code, 132 prose, 36
  blank. The logic is invisible. Move, do not delete: the two-layer and
  `backend:=`/`world:=` essays and the `nav2()` "why not bringup_launch.py"
  note go to `docs/architecture.md`; the `backend:=real` tethered-alternative
  paragraph and the `turtlebot3_node` `namespace` gotcha (which document code
  that does not exist in this repository) go to `docs/roadmap.md`. Target ~190
  lines with no behaviour change. **Keep** the `inc()` scoping comment, the
  `handle_once` comment, the slam_toolbox-is-not-a-lifecycle-node note and
  `wait_for_sim`'s rationale — each documents a failure that costs hours.
- **Audit `tb3_bringup/config/nav2_params.yaml` against the burger's real
  limits** (0.22 m/s, 2.84 rad/s, footprint, inflation) before lane A's
  navigation runs. If the controller's maximum velocities exceed what the robot
  can do, Isaac's actuation deficit is partly masked by controller saturation
  and the experiment measures the wrong thing. **Report findings; do not retune
  per backend** (F7).
- **Draft `docs/experiment.md`**: the four-layer framing from §0, metric
  definitions, the run matrix, and empty result tables ready to fill. The
  morning's numbers should drop into a structure rather than a pile.

---

## 5. Spawning subagents

**Priority if the night runs short: A > C > B.** A is thesis content, C unblocks
the experiment, B is polish.

**Parallelism is limited by hardware, not by the work.** One GPU, one container:
only lane A may run a simulator. Lanes B and C are text-only and may run
alongside it. The directory exclusions in §3 and §4 are what keep them from
colliding — respect them and no coordination is needed.

Model selection:

| lane | model | why |
|---|---|---|
| A — physics | **highest available** (Opus) | Novel diagnosis under a falsification criterion, with a real chance the leading hypothesis is wrong. This is the thesis; do not economise here. |
| C — interface + docs | **Sonnet** | A well-specified refactor: the target interface, the dispatch, the deletions and the move targets are all written out above. |
| B — package cleanup | **Sonnet** | Judgment about what counts as clutter and what is load-bearing needs reasoning, but the target state is described. |
| mechanical passes | **Haiku** | Only for genuinely mechanical work — a whitespace pass, a rename applied across known files. Not for deciding what to delete. |

**Give each subagent its context in the prompt.** Everything it needs is in this
file: paths, line numbers, measured values, target numbers. Paste the relevant
section rather than telling it to go and read the repository — a subagent that
re-derives context burns its budget on search and arrives at the work with a
worse picture than this file already contains. State the acceptance criteria
numerically in the prompt, or it will declare success on prose.

## 6. Reporting

Every lane appends a dated report to `docs/worknotes/`, with numbers.

**Negative results are required, not optional.** A falsified hypothesis with a
measurement attached is thesis material; "I tried some things" is not. If H1 is
dead, the report says so, with the two error values and the two damping values
that killed it. If the night ends mid-diagnosis, the report says exactly what is
known, what is not, and what the next test is.

Write for someone reading it cold in the morning who was not here.

## 7. Hard rules

- **No `git push`.** No force operations, no `reset --hard`, no amending pushed
  commits. Local commits only. Any command touching a remote goes into the
  report for the user to run.
- **Do not delete anything** under `worlds/`, `measurements/` or `maps/`.
- **Do not regenerate world artifacts** — it can invalidate committed files that
  other measurements depend on. Run `scripts/check_worlds.py` as a gate before
  and after the night's work and report the result.
- **Do not tune Isaac to match Gazebo** (§0). Verify each against the analytic
  command independently.
- **Do not tune Nav2 per backend** (F7).
- **Do not change `merge_fixed_joints`** (F6) — verify and reconcile the docs.
- **Do not hand-edit the generated USD** (A3).
- Commits in this repository carry **no** `Co-Authored-By` or `Claude-Session`
  trailers.
