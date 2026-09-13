# worlds

Environments — the counterpart of `turtlebot3_gazebo/worlds/*.world`.

```
worlds/
└── turtlebot3_world.usd    generated, gitignored
```

**None ship with this package yet.** `empty_world.launch.py` needs no world and
is the right thing to run first; `turtlebot3_world.launch.py` will fail with a
message naming the missing file until one is built.

## What a world USD has to be

The simulator script composes the scene as gzserver composes a `.world` with a
spawned model, so a world here is a *pure environment*:

- **no robot** — it is referenced separately, at `/World/turtlebot3`
- **no ground plane and no light** — the script authors both, because the floor
  needs a physics material it controls (left to itself Isaac Sim's `GroundPlane`
  authors restitution 0.8, which makes a TurtleBot3 rock on its caster skid and
  creep with nothing commanding it)
- **Z-up, metres, origin matching the Gazebo world's origin**, so that a map
  recorded in one backend is valid in the other
- colliders on everything the lidar should see

It is referenced at `/World/env`, so nothing in it may assume a prim path
outside its own subtree.

## Building one

Not yet automated here. The environment geometry lives in
`turtlebot3_gazebo/models/turtlebot3_world/`; converting an SDF world to USD
means walking its `<include>`/`<model>` placements and referencing the same
meshes at the same poses. Keeping the meshes byte-identical between the two
backends is what makes a Gazebo run and an Isaac Sim run comparable at all —
parity is the point, and a separately modelled world quietly destroys it.
