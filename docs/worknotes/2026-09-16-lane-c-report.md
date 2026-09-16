# Lane C report — 2026-09-16

Executed against `docs/worknotes/2026-09-16-overnight-brief.md` §4. Text-only,
did not touch `src/` or `tb3_bringup/tb3_bringup/nav_test.py`. Ran alongside
Lane A, which was actively using the GPU and the container throughout (its
uncommitted changes to `tb3_bringup/launch/backends/gazebo.launch.py`,
`tb3_bringup/package.xml`, `tb3_bringup/tb3_bringup/drive_test.py` and a new
`measurements/2026-09-16_pre_gazebo_default.json` were visible in the working
tree the whole time and were deliberately left alone — not staged, not
committed, not read for content). Three commits, on `overnight/2026-09-16`,
none pushed:

- `a07ace6` — the three-mode interface, F7, and the bringup.launch.py compaction
- `18721d3` — F9.2 (save_map.py)
- `7fd5e8f` — docs/experiment.md

## The interface change

`tb3_bringup/launch/bringup.launch.py` now implements the three modes from
§4 exactly: bare robot, `nav:=true` (map_server + AMCL + Nav2 on the saved
map), `slam:=true` (slam_toolbox + Nav2 together). The dispatch:

```python
stack = []
if slam:
    stack.append(inc(nav2('slam_launch.py'), params_file=params))
elif nav:
    map_yaml = world.map()
    if not map_yaml:
        raise RuntimeError(...)          # unchanged error, names the fix
    stack.append(inc(nav2('localization_launch.py'), map=map_yaml, params_file=params))
if nav or slam:
    stack.append(inc(nav2('navigation_launch.py'), params_file=params))
```

**The one real behaviour change:** `slam:=true` alone now also starts
`navigation_launch.py`. Previously that combination started only
slam_toolbox with no planner running against it — the mode the brief says not
to reintroduce. `slam:=true nav:=true` is accepted (not rejected) and is
identical to `slam:=true` alone, since `nav or slam` is already true either
way.

**Verified in-container** (`docker exec tb3_ros`, using `backend:=real` so no
simulator was started, respecting the GPU exclusivity rule):

- `backend:=real world:=empty_stage nav:=true` (empty_stage has no map) still
  raises the same `nav:=true needs a map...` error, evaluated before any node
  starts.
- `backend:=real world:=empty_stage slam:=true rviz:=false` brought up
  `sync_slam_toolbox_node` *and* the full Nav2 stack
  (`controller_server`, `planner_server`, `bt_navigator`, `behavior_server`,
  `smoother_server`, `waypoint_follower`, `velocity_smoother`, two
  `lifecycle_manager`s, `map_saver_server`) together — the new behaviour,
  confirmed.
- `ros2 launch ... --show-args` renders the new argument descriptions
  correctly, confirming the package's symlink install (`/ws/src/tb3_bringup`
  → `/repo/tb3_bringup`) picks up source edits with no rebuild.
- No leftover processes after either dry run; Lane A's own
  `backend:=gazebo world:=empty_stage` run (pid 5907/6553/6562, gzserver) was
  running throughout and was not touched or interfered with.

## F7 — `params_file()` deleted

`config/nav2_params.yaml` is now passed unconditionally; the function that
silently preferred `config/nav2_<backend>.yaml` if one existed is gone. Also
updated the three places that described that mechanism as available or
upcoming, since leaving them would have been the same stale-doc problem as
F8: `docs/architecture.md` (`Layout` block and build-order step 5),
`docs/status.md` (the "per-backend Nav2 tuning is unstarted" bullet, which
described a mechanism that no longer exists), and the header comment in
`tb3_bringup/config/nav2_params.yaml` itself.

## F8 — mode grid / teleop claim

Fixed in `README.md`, `docs/roadmap.md` (§3, with a dated correction appended
in the file's own established style rather than rewritten in place — the
"built 2026-09-16" grid stays as a historical record, followed by a
"Collapsed to three modes, 2026-09-16" paragraph), and the launch docstring.
All three now name the second terminal: `ros2 run turtlebot3_teleop
teleop_keyboard` or `ros2 run tb3_bringup drive_test`.

`docs/roadmap.md` §5 (the tethered-alternative paragraph and the
`turtlebot3_node` `namespace` gotcha) previously pointed at "the comment in
`bringup.launch.py`" for this content; since that comment moved out of the
launch file as part of the compaction, §5 now contains the content itself
instead of a pointer into a file that no longer has it.

## F9.2 and F9.3

**F9.2** (`scripts/save_map.py`): `--name` now defaults to the world's name
instead of the literal string `"map"`, so every world gets the same naming
convention going forward. Existing committed maps
(`turtlebot3_world/map/map.yaml`, `small_office/map/small_office.yaml`) were
**not** renamed or touched — the brief's hard rule against deleting under
`worlds/` and the instruction to leave F9.1 (map provenance) alone both argue
for touching only future saves, not rewriting committed history. The closing
message is now conditional: it only tells you to run `nav:=true` when the
manifest actually points at the map that was just saved; otherwise it names
the map that will really load and how to fix it (edit the manifest by hand,
or `--force`).

**F9.3** (`bringup.launch.py:45`): fixed as part of the rewrite — the module
now says `worlds/<name>/map/` throughout, matching README, CLAUDE.md,
`save_map.py` and both manifests. Also found and fixed two more instances of
the same stale flat-path form in `docs/roadmap.md` (§3's "Decided:" paragraph
and the "Next steps" list) while in the area — same bug, same fix, cheap to
catch since it was the exact pattern F9.3 named.

**F9.1 was not touched**, as instructed — it is a manifest schema change and
stays recorded only in the overnight brief.

## `bringup.launch.py` compaction

278 → 222 lines (target was ~190; the four required comments plus a docstring
that fully documents the new three-mode interface account for the gap — I
chose completeness of the interface documentation over hitting the line
target exactly). Nothing prose-only was deleted; each moved paragraph has a
named destination:

- the two-layer split, the `backend:=`/`world:=` reasoning, and the `nav2()`
  "why not `bringup_launch.py`" note → new `## The launch entry point`
  section in `docs/architecture.md`
- the `backend:=real` tethered-alternative paragraph and the
  `turtlebot3_node` `namespace` gotcha → `docs/roadmap.md` §5 (see above)

Kept in place, verbatim, as instructed: the `inc()` scoping comment, the
`handle_once` comment, the slam_toolbox-is-not-a-lifecycle-node note, and
`wait_for_sim`'s rationale.

## `nav2_params.yaml` audit against the burger's real limits (report only, not retuned)

Checked `controller_server`'s `FollowPath` (DWB) block and both costmaps
against the burger's documented limits (0.22 m/s linear, 2.84 rad/s angular)
and the file's own claim to be verbatim
`ros-humble-turtlebot3-navigation2`'s `param/humble/burger.yaml`.

- **Linear velocity: at the limit, not over it.** `max_vel_x` / `max_speed_xy`
  = 0.22 m/s, exactly the burger's real maximum. No masking risk on this axis.
- **Angular velocity: well under the limit.** `max_vel_theta` = 1.0 rad/s
  against a real 2.84 rad/s ceiling (35%). The controller cannot ask for more
  than the robot can deliver, so there is no risk of Nav2 commanding an
  unreachable `wz` and the resulting hardware saturation being misread as an
  Isaac actuation problem — the specific failure mode the brief names.
  **However**, `measurements/nav2.md` (Lane A, same night) already shows the
  opposite risk in practice: because the controller is nowhere near saturated
  and Nav2 closes the loop on yaw error, a pure-180°-turn goal reached final
  yaw error 0.110 rad on Isaac vs. 0.261 rad on Gazebo — Isaac's *smaller* of
  the two — even though `drive_test` (open loop) shows Isaac under-rotating
  by 30-52% at the same commanded rates. **Closed-loop `nav_test` numbers can
  show little or no angular deficit even though it is real and already
  measured open-loop; that is an expected consequence of these velocity
  limits, not a contradiction to chase down.** Worth stating explicitly in
  whatever compares `drive_test` and `nav_test` results, so a reviewer does
  not read "Nav2 hides it" as "the deficit isn't real."
- **Footprint.** `robot_radius: 0.1` m (both costmaps) is a circular
  approximation of the burger's rectangular footprint (~138 × 178 mm nominal,
  circumscribing radius ≈ 0.113 m). The circle fully covers both face-normal
  directions (0.1 m > each half-dimension) but under-covers the diagonal
  corners by about 1.3 cm. This is upstream's own stock value, not something
  introduced by this repo, and is a standard trade-off for a circular
  footprint model — flagged for awareness, not a defect to fix here.
- **Inflation.** `local_costmap.inflation_radius` = 1.0 m vs.
  `global_costmap.inflation_radius` = 0.55 m — asymmetric, and both are
  upstream's stock values. The local costmap's rolling window is 3×3 m; a
  1.0 m inflation radius can occupy most of that window near any wall or
  piece of furniture. `small_office` is roughly 6×5 m, noticeably smaller
  than the arena these defaults were presumably tuned against
  (`turtlebot3_world`). **Worth watching specifically on `small_office`
  `nav_test` runs**: a shrunk feasible corridor there is a room-size effect,
  not a backend effect, and could inflate the recovery-behaviour count or
  path-length ratio in a way that has nothing to do with sim-to-real. Not
  retuned, per F7 and the explicit instruction not to touch this file's
  values.

**Bottom line: nothing in `nav2_params.yaml` exceeds the burger's real
velocity envelope, so Lane A's navigation runs are not at risk of hardware
saturation masking the actuation deficit.** The two things worth carrying
into Lane A's `nav_test` analysis are the closed-loop-hides-open-loop-deficit
effect above (already visible in `measurements/nav2.md`) and the
`small_office`-specific inflation-radius caveat.

## `docs/experiment.md`

Drafted per §4: the four-layer framing from the brief's §0, metric
definitions for each layer (actuation/perception/task), an empty run matrix,
and an empty repeatability table for A6's repeat-count derivation. Links to
`measurements/*.md` rather than duplicating their content, so it stays a
structure rather than a second copy of the results.

## Gate

`scripts/check_worlds.py`: **3/3 worlds consistent**, run before Lane C
started and again after — unaffected, since no world artifact was touched
(the `save_map.py` change only affects the default filename for *future*
saves).

## Not done / explicitly out of scope for Lane C

- F6 (asset frame reconciliation) and F9.1 (map provenance) — Lane A /
  explicitly deferred, per the brief.
- Lane B (`src/turtlebot3_isaacsim` package cleanup) — not started; a
  separate lane, and `src/` is off-limits here regardless.
- No `git push`. Everything above is local commits on `overnight/2026-09-16`
  only.
