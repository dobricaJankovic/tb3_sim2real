# Working agreements for this repo

- **Search before building.** Before writing anything substantial — a script,
  an OmniGraph, a driver, a workflow — first go looking for what already
  exists: vendor sample assets, the upstream repo's own examples and *tests*,
  and prior art here. Ask the user as well; they often know a reference exists
  that no grep would surface. Trust upstream schema and test files (`.ogn`
  definitions, message definitions, `test_*.py`) over upstream prose docs,
  which go stale. This is not a style preference — it is written down because
  `isaac/scripts/tb3_sim.py` was built from first principles while NVIDIA
  shipped a fully prewired TurtleBot3 with working ROS 2 graphs, and because
  the skill docs consulted on the way named node attributes that do not exist.
- **Get the bigger picture before executing.** Push back on a narrow
  instruction until the goal behind it is clear: what it is being built
  toward, what already exists, and how far the change should reach. State the
  plan and let it be corrected *before* starting rather than after. This saves
  time in both directions — it stops work aimed at the wrong target, and it
  forces the target to be made explicit. Asking costs a minute; a wrong
  assumption costs a session.
- **Commit autonomously.** After completing a fix or a meaningful chunk of
  work, create a git commit without waiting for explicit confirmation each
  time. Still use judgment: stop and ask before destructive git operations
  (force-push, `reset --hard`, amending already-pushed commits) or before
  pushing to a remote.
- **History log.** `docs/history.md` is an append-only log of
  problems/fixes and concepts/ideas from work sessions, for the user's own
  reference. When starting a new conversation, do NOT read the whole file —
  it's not needed for context. Only append a new dated entry at the end
  after finishing meaningful work (a bug fixed, a design decision made,
  an idea worth remembering).
