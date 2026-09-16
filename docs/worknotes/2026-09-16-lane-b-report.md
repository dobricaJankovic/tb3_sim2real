# Lane B report — 2026-09-16

Executed against `docs/worknotes/2026-09-16-overnight-brief.md` §3, in
`src/turtlebot3_isaacsim` (its own git repository, remote
`git@github.com-dobrica-personal:dobrica-jankovic/turtlebot3_isaacsim.git`).
Did not touch `scripts/import_turtlebot3.py` or `models/` (lane A's tonight).
Text/CPU-only throughout — no simulator started, per the GPU-exclusivity rule
in §5/§7. Five commits, all local, on a new `humble` branch (plus one on a new
`jazzy` branch); nothing pushed.

```
d47ab57  Add scripts/smoke_test.py; fixes a real SCAN_OFFSET bug it found
32f687b  Move launch-invoked code out of scripts/ into runtime/
83352fb  Collapse the five world launch files onto one parameterised world.launch.py
1a8bf06  Reconcile UPSTREAM.md and DESIGN.md with merge_fixed_joints=True
3359c71  Split the humble/jazzy Dockerfiles onto their own branches   (base of jazzy)
269701f  jazzy: hold the untested Ubuntu 24.04 Dockerfile, marked as such   (jazzy branch)
```

`git diff --stat fix-map-mirroring-add-small-worlds humble`: 17 files changed,
504 insertions, 338 deletions. `scripts/import_turtlebot3.py` and `models/` do
not appear in that diff — confirmed untouched.

## Branches (local only — commands for the morning)

Created `humble` from the tip of `fix-map-mirroring-add-small-worlds` (tonight's
work is on it) and `jazzy` from `humble`'s first commit. `Dockerfile.jazzy` used
to sit alongside `Dockerfile.humble` in one directory on one branch, wired into
nothing (`scripts/build_images.sh` only ever builds `Dockerfile.humble`) and
untested. It now lives only on `jazzy`, with an explicit not-tested banner
added to its header. `humble` is meant to become the new default branch;
`fix-map-mirroring-add-small-worlds` was left exactly as it was — not deleted,
not rebased onto.

To publish this in the morning:

```bash
cd src/turtlebot3_isaacsim
git push origin humble jazzy

# make humble the default on GitHub (either works):
gh repo edit dobricaJankovic/turtlebot3_isaacsim --default-branch humble
#  — or via the web UI: Settings > Branches > Default branch —

# only after confirming humble looks right as the default and nothing external
# still points at the old name:
git push origin --delete fix-map-mirroring-add-small-worlds
```

I did not run any of these — they touch the remote, which §7 reserves for you.

## F6 — the lidar offset, and the `merge_fixed_joints` doc contradiction

**The offset is correct on burger, and was wrong (never-shipped) on
waffle/waffle_pi.** Checked by hand against
`turtlebot3_ws/src/turtlebot3/turtlebot3_description/urdf/turtlebot3_*.urdf`:
`base_footprint -> base_scan` is `base_joint` (`(0, 0, 0.010)` on all three
models) composed with `scan_joint`. Burger's `scan_joint` is
`(-0.032, 0, 0.172)`, giving `(-0.032, 0, 0.182)` — exactly
`SCAN_OFFSET['burger']` in `runtime/turtlebot3_isaacsim.py`, already right.
Waffle/waffle_pi's `scan_joint` is `(-0.064, 0, 0.122)`, giving
`(-0.064, 0, 0.132)` — but the code had `(-0.064, 0.0, 0.122)`, `scan_joint`'s
z alone, missing the `+0.010` from `base_joint`. Fixed in `d47ab57`. Neither
model is built (`README.md`: "only `burger` has been checked against the
URDF"), so nothing shipped with the wrong number, but it would have on the
first `build_models.sh waffle`.

**The articulation-root prim is `base_footprint`, confirmed directly in the
committed asset** (`grep -n ArticulationRootAPI`-equivalent: the physics
material bindings in `turtlebot3_burger.usda` are all under
`Geometry/base_footprint`, and it's the only named link besides the two
wheels). `merge_fixed_joints=True` cannot remove it — it's the URDF's root
link, never a fixed joint's *child* — so `odom -> base_footprint`, the only
frame this package owns, is unaffected regardless of the flag. What the flag
does fold away are `base_link`, `base_scan`, `imu_link`, `caster_back_link` as
*separate named prims*; their geometry merges into `base_footprint`'s mesh.

`UPSTREAM.md` said the opposite — "Leave `merge_fixed_joints` at `False`...
Enabling it would destroy `base_footprint`, `base_scan` and `imu_link`" —
directly contradicting `scripts/import_turtlebot3.py:89`, which sets it to
`True` with no explanation beyond a TODO about the damping. It isn't a bug:
`robot_state_publisher` (`launch/robot_state_publisher.launch.py`) supplies
every frame from `base_footprint` down on all three backends by reading
`turtlebot3_description`'s URDF directly, never the USD stage, so this package
never needed those prims to exist as named USD entities. `import_turtlebot3.py`
already says this in its own comment ("Root first, as the fallback for
everything merge_fixed_joints folded into it") — the docs just hadn't been
told. Rewrote the "URDF importer" section and the `odom` child-frame item in
`UPSTREAM.md` §8 to say so, with the derivation; added a "Where the lidar sits"
paragraph to `DESIGN.md` recording the same offset check so nobody has to
redo it by hand next time (`1a8bf06`).

## Asset smoke test (`scripts/smoke_test.py`)

Headless, `isaacsim-python scripts/smoke_test.py [--model ...]`. Checks, in
order: articulation root present and singular; `base_footprint`,
`wheel_left_link`, `wheel_right_link` present; `base_link`/`base_scan`/
`imu_link`/`caster_back_link` confirmed *absent* as separate prims (the
intended shape under `merge_fixed_joints=True`, not a defect — see F6 above);
both wheel joints' drive damping/stiffness and the asset's selected Physics
variant, **reported, not asserted** (tonight's gain question is F3/F4/A1/A2,
lane A's); the lidar profile JSON's shape against what
`profile_attributes()` actually reads; and `SCAN_OFFSET` against the URDF.

That last check reads `SCAN_OFFSET` out of `runtime/turtlebot3_isaacsim.py`
with `ast.literal_eval` rather than importing the module (which starts a
`SimulationApp` at import time) or retyping the numbers a second time (which
would drift exactly the way the thing it's checking already had).

**What ran tonight, and what didn't.** The two checks that need only the
filesystem — `SCAN_OFFSET` vs. the URDF, and the lidar profile's shape — were
run directly with plain `python3` (no Isaac Sim needed for either) and pass:

```
PASS  SCAN_OFFSET['burger'] == base_joint + scan_joint from the URDF: got (-0.032, 0.0, 0.182), expected (-0.032, 0.0, 0.182)
PASS  SCAN_OFFSET['waffle'] == base_joint + scan_joint from the URDF: got (-0.064, 0.0, 0.132), expected (-0.064, 0.0, 0.132)
PASS  SCAN_OFFSET['waffle_pi'] == base_joint + scan_joint from the URDF: got (-0.064, 0.0, 0.132), expected (-0.064, 0.0, 0.132)
PASS  models/lidar_configs/turtlebot3_lds.json has every top-level field profile_attributes() reads: missing none
PASS  models/lidar_configs/turtlebot3_lds.json has exactly one emitter state (profile_attributes() only authors s001): found 1
PASS  models/lidar_configs/turtlebot3_lds.json emitter state s001 has every field the schema needs: missing none
```

The three checks that need the stage open (articulation root, link set, drive
parameters) have **not** been run — they need `SimulationApp`, i.e. Isaac Sim,
which lane A owns tonight. Run them in the morning:
`isaacsim-python scripts/smoke_test.py`, then `--model waffle` and
`--model waffle_pi` once those are built.

## `scripts/` -> `runtime/`

Plan, as required before touching a path a launch file references: move
`turtlebot3_isaacsim.py` (the simulator entry point `isaacsim.launch.py`
invokes) and `assets.py` (the library it imports) into a new `runtime/`
directory; leave `build_images.sh`, `build_models.sh`, `build_map.py` in
`scripts/`; leave `import_turtlebot3.py` in `scripts/` untouched (lane A's, and
it's invoked by a shell script by hand, not by a launch file, so it fits
`scripts/`'s definition anyway). Update the one path this changes —
`launch/isaacsim.launch.py`'s `os.path.join(pkg_isaacsim, 'scripts',
'turtlebot3_isaacsim.py')` -> `'runtime'` — and `CMakeLists.txt`'s
`install(DIRECTORY ...)` line, plus every doc reference (`README.md`,
`DESIGN.md` x3, `UPSTREAM.md`, `CLAUDE.md`, `scripts/build_map.py`'s
docstring).

Executed in `32f687b`. Verified afterward: `get_package_share_directory` +
the new relative path resolves to a real file (checked with
`ament_index_python` inside the `tb3_ros` container, install tree mirrored by
hand with symlinks since this wasn't rebuilt); every launch file in the
package still dry-runs cleanly with `ros2 launch turtlebot3_isaacsim <file>
--show-args` — including `isaacsim.launch.py` itself, same arguments and
defaults as before; and `ros2 launch tb3_bringup bringup.launch.py
backend:=isaacsim world:=turtlebot3_world --show-args`, from the *consuming*
repo, also still resolves. No simulator was started for any of this — `--show-
args` only walks the static launch-description tree, it doesn't execute the
`OpaqueFunction` that builds the `standalone:=` command string, so it can't
catch a bad path inside that function; the `get_package_share_directory` check
above is what actually confirmed the new file is reachable at that path.

None of this was rebuilt through `colcon` — I mirrored the `runtime/` move and
the new `world.launch.py` into `/ws/install/.../share/turtlebot3_isaacsim/`
by hand with symlinks, just for the checks above, and left the stale
pre-move copies of `turtlebot3_isaacsim.py`/`assets.py` sitting under the old
`.../share/turtlebot3_isaacsim/scripts/` (harmless — nothing references that
path anymore — but a plain `colcon build --symlink-install` in the morning
will clear them out properly and is worth doing before trusting the install
tree for anything else).

## Six launch files -> one parameterised + five examples

New `launch/world.launch.py` holds the `isaacsim.launch.py` +
`robot_state_publisher.launch.py` pair that all five per-world files were
rebuilding by hand. `empty_world.launch.py`, `turtlebot3_world.launch.py`,
`warehouse.launch.py`, `simple_room.launch.py` and `kitchen.launch.py` are now
thin wrappers: each declares only its own world's defaults — with every
measurement comment (spawn-pose rationale, `world_z` derivations, mirroring
caveats) preserved verbatim — and includes `world.launch.py` with them.
`launch/isaacsim.launch.py` itself is untouched.

Verified the same way as the `runtime/` move: `--show-args` on all five
wrappers plus `world.launch.py` and `isaacsim.launch.py`, confirming identical
argument sets and defaults to before the refactor (e.g. `warehouse`'s
`x_pose` still defaults to `-7.0`, `simple_room`'s `world_z` to `0.7696`,
`kitchen`'s pose to the origin) — full output is in the commit message of
`83352fb`. Net line count is roughly flat (726 vs. 724 before) because of
added docstrings, but the actual duplication — the `isaacsim_cmd` /
`robot_state_publisher_cmd` construction, previously repeated five times — now
exists exactly once; `grep -l isaacsim_cmd launch/*.py` returns only
`world.launch.py`.

## Docs

`README.md`, `DESIGN.md`, `UPSTREAM.md` all stay, updated in place rather than
moved. Looked specifically for session-log content to move to
`tb3_sim2real/docs/` per the brief — `DESIGN.md`'s `## Status` section and
`UPSTREAM.md` throughout read as dated, sourced reference documentation (what's
verified, what a doc claim says and where), not a diary; there's no
first-person narration of a working session to extract. Judgment call: left
both in place. If you disagree on a specific section, it's a `git mv` plus a
link fix, not a rewrite.

## Dead code / redundant comments

Grepped `runtime/`, `scripts/*.py` (excluding `import_turtlebot3.py`),
`scripts/*.sh` and `launch/*.py` for `TODO`/`FIXME`/`XXX`, commented-out code,
and comments that just restate the following line. Found nothing to remove —
every comment in this codebase documents a non-obvious failure or a
measurement, which is exactly what the brief says to keep. Read `assets.py` in
full and skimmed `turtlebot3_isaacsim.py`; both are tight.

## Gate

`scripts/check_worlds.py` (from `tb3_sim2real`, stdlib+PyYAML, no GPU) at the
end of this session: `check_worlds: 3/3 worlds consistent` — `empty_stage`,
`small_office`, `turtlebot3_world` all pass provenance, round-trip and
footprint checks. `worlds/`, `measurements/` and `maps/` were never touched by
this lane.

## What's left, if anyone picks this up

- Run `isaacsim-python scripts/smoke_test.py` for real (needs the GPU lane A
  had tonight) and fold the drive-parameter values it reports into whatever
  lane A concludes about F3/F4.
- Build `waffle`/`waffle_pi` with the corrected `SCAN_OFFSET` and smoke-test
  them before trusting either.
- Push `humble`/`jazzy` and flip the GitHub default branch (commands above).
- If `jazzy` is meant to be kept current, it needs periodic merges from
  `humble` — right now it only has the branch-split commit, nothing from
  tonight's other four.
