# Gazebo's missing wheel slip was one number, and upstream disowned it

2026-09-19. `measurements/2026-09-19_friction_gazebo_{upstream,wheel_only,floor_only,both}.json`.

## What was believed

`docs/experiment.md` and `CLAUDE.md` both said Gazebo cannot show wheel slip,
with `mu = 1e5` named as the reason and the careful wording that **"mu = 100000
is not a physical value, it is a don't-slip sentinel"**. True, and never tested
— nobody had run Gazebo with a different number, so "cannot show slip" and
"was configured not to" were indistinguishable.

## Where the number comes from

`turtlebot3_gazebo/models/turtlebot3_burger/model.sdf`, both wheel collisions,
with ROBOTIS's own comment above it, verbatim and including the typo:

```xml
<!-- This friction pamareter don't contain reliable data!! -->
<mu>100000.0</mu>
<mu2>100000.0</mu2>
```

So it is not a modelling decision anyone made and can defend. It is a
placeholder that has been quietly setting one third of this repository's
sim-to-real comparison.

It is **not** this repository's generator: `scripts/build_world.py` wrote no
friction at all and `include`d Gazebo's stock `model://ground_plane`, whose own
surface is `mu 100 / mu2 50` — also ~100x a real floor, also undeclared, and
invisible in both the manifest and the generated world.

## The experiment

Four conditions, one `sweep` each, same instrument, same day, `empty_stage`
geometry. Run against `backends/gazebo.launch.py` **directly** rather than
through `bringup.launch.py`, because bringup derives the wheel's mu from the
manifest's floor — correct for a run, and useless for the one experiment that
has to vary them independently.

Slip is ground truth against wheel-integrated motion: `(truth - wheel) / wheel`.

### Pivot slip, %

| floor | wheel | wz=0.2 | wz=0.5 | wz=1.0 | wz=1.5 |
|---|---|---|---|---|---|
| 100/50 | **100000** (upstream) | −0.30 | −0.20 | −0.14 | −0.14 |
| 100/50 | 1.0 | −2.05 | −4.13 | −9.02 | −15.14 |
| 1.0 | 100000 | −2.51 | −4.60 | −9.49 | −15.60 |
| **1.0** | **1.0** (now) | **−2.58** | **−4.66** | **−9.55** | **−15.66** |

### Straight-line slip, %

| floor | wheel | 0.10 m/s | 0.15 | 0.22 |
|---|---|---|---|---|
| 100/50 | 100000 | +0.18 | +0.31 | −0.27 |
| 1.0 | 1.0 | −0.13 | +0.24 | −0.33 |

Unchanged, within noise, in every condition — slip here is a **pivot**
phenomenon, which is what a robot that turns by scrubbing two wheels against a
dragging skid should do. Driving straight, the wheels roll.

## Three findings

**1. "Gazebo cannot show slip" is false.** It shows −15.7% at `wz = 1.5`. It
was configured not to, by a number its own author labelled unreliable.

**2. ODE combines a contact pair by the MINIMUM — now measured, not assumed.**
Lowering *either* side alone reproduces essentially the whole effect
(−15.14 and −15.60 against −15.66 for both), while the pair that keeps one high
value (100 against 100000) shows none. Only `min` explains all four rows.

This was worth measuring because it was guessed wrong first. An earlier attempt
at this table ran through `bringup.launch.py`, which overrode the command-line
`wheel_mu` with the manifest's floor value, so "upstream" silently ran at 100
and "wheel only" was a duplicate of it. The two conditions that remained
suggested the opposite rule. The tell was two rows agreeing to the digit;
`grep -o "<mu>" /tmp/tb3_robot_*.sdf` on what was actually spawned settled it.

**3. The two simulators agree once the sentinel is gone.** At `wz = 0.5`, the
only rate where a directly comparable Isaac measurement with ground truth
exists (`2026-09-18_isaacsim_wheel_odom.json`, `default` sequence, same
commanded rate and duration):

| | pivot slip at wz = 0.5 |
|---|---|
| Isaac Sim | −4.61% |
| Gazebo, upstream mu | −0.20% |
| **Gazebo, mu = 1.0** | **−4.66%** |

An order-of-magnitude disagreement between two physics engines closes to
**0.05 percentage points** when one undeclared default is replaced by a
declared one. That is the strongest evidence this repository has for the claim
that its sim-to-real differences are dominated by parameters nobody chose
rather than by engine behaviour.

Stated carefully: one rate, one Isaac run, different sequence name (same
command). It is a strong hint, not a closed result. The matched comparison is
experiment 1.

## What was changed, and what deliberately was not

Changed: the floor's coefficient is now `physics.floor.mu` in `world.yaml`
(default `worlds.FLOOR_MU = 1.0`), `build_world.py` writes the ground plane out
in full instead of including Gazebo's, and `bringup.launch.py` passes the same
number to the wheel through `backends/gazebo.launch.py`'s existing temp-copy of
the robot SDF. `check_worlds.py` checks the floor half and
`test_gazebo_wheel_friction_follows_the_manifest` checks the wheel half.

Both sides of the pair carry the same number. Given finding 2 that is not
strictly necessary — `min` means the lower one wins — but it keeps the manifest's
declaration meaningful under any engine's rule, and PhysX averages rather than
minimises, so Isaac needs it.

**Not changed, `slip1`/`slip2`.** They stay `0.0`. These are ODE's
force-dependent slip, a compliance proportional to applied force — `0.0` means
ordinary Coulomb friction, **not** "slipping disabled". Coulomb slip was always
available; with `mu = 100000` the limit was simply never reached. Turning FDS on
would add a second free parameter with nothing to set it from, and the value of
the table above is that exactly one thing moved.

**Not changed, `caster_back_joint`.** Upstream makes it a *ball* joint, so
Gazebo's caster **rolls** where the real robot and Isaac Sim drag a skid. That
is structural rather than a coefficient, it is upstream's asset, and it stays —
recorded as the limitation that would explain a residual disagreement rather
than patched over.

## 1.0 is an assumption, not a measurement

Dry rubber on vinyl. Handbook values for rubber on dry solids sit near 1.0;
flooring slip-resistance testing (ASTM D2047, DIN 51131) with standardised
rubber sliders reads nearer 0.5–0.7. The honest band is about 0.6–1.0 and this
is the **top** of it — the end that flatters Gazebo, since a lower coefficient
slips more.

It is 1.0 rather than mid-band deliberately: `turtlebot3_isaacsim` already
authors exactly 1.0 for its wheel and floor, so adopting it moves **one**
backend and leaves every Isaac measurement taken before today still comparable.

`docs/experiment-plan.md` B10 replaces it with a value fitted to the real
robot's measured pivot slip once experiment 1 is in. Until then **no text should
call it measured**, and the handbook band above is not a specific paper read for
this repository — verify the source before citing it.

## Isaac Sim's floor is still a second source of truth

`turtlebot3_isaacsim` authors its floor at runtime (`static_friction=1.0`) and
its wheel into the asset. Both happen to equal the manifest's value today, so
nothing is inconsistent — but nothing *checks* it either, and `check_worlds.py`
parses no USD. Reading `physics.floor.mu` from the manifest is a task in that
repository. See `docs/roadmap.md`.
