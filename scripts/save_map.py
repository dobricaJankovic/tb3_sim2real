#!/usr/bin/env python3
"""Write the SLAM map being built right now into its world's directory.

    scripts/save_map.py lab_room

Run it in a second terminal while `slam:=true` is still up and you are happy
with the map. It is deliberately NOT automatic on shutdown: a half-finished or
badly-closed run would then silently overwrite a good map, and a map is
expensive to make.

The point is the destination, not the saving. `map_saver_cli` writes to the
working directory by default, and a map that lands in the working directory is
a map that drifts away from the world it describes — which is the failure the
registry exists to prevent. So this takes a WORLD NAME, never a path, writes
into `worlds/<name>/map/`, and adds the `map:` key to the manifest if it is
not there yet. After this, `nav:=true world:=<name>` needs no further argument
on any of the three backends.

`map_saver_cli` rather than `/slam_toolbox/save_map`, because it subscribes to
`/map` and so works for any publisher — the same tool a geometric map from
`make_map.py` would use. One way to write a map.yaml, not two.
"""

import argparse
import os
import subprocess
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'tb3_bringup'))

from tb3_bringup import worlds  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('world', help='registry name, or a path to a world directory')
    ap.add_argument('--name', default='map',
                    help='basename for map.yaml/map.pgm (default: map)')
    ap.add_argument('--topic', default='/map', help='occupancy grid topic')
    ap.add_argument('--force', action='store_true',
                    help='overwrite an existing map for this world')
    args = ap.parse_args()

    world = worlds.World.load(args.world)
    map_dir = os.path.join(world.dir, 'map')
    stem = os.path.join(map_dir, args.name)
    rel = os.path.join('map', args.name + '.yaml')

    # A world's map is the artifact every backend shares and a cloned world's
    # map is the input it was made from, so replacing one silently is not a
    # thing this script should do on its own.
    if os.path.isfile(stem + '.yaml') and not args.force:
        sys.exit(
            f"'{world.name}' already has {rel}.\n"
            f'  Pass --force to replace it, or --name <other> to keep both.\n'
            f'  If this world was cloned from a real room, its existing map is '
            f'the recording the model was built from: see worlds/README.md '
            f'before overwriting it.')

    made_dir = not os.path.isdir(map_dir)
    os.makedirs(map_dir, exist_ok=True)
    cmd = ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
           '-f', stem, '-t', args.topic]
    print(' '.join(cmd))
    if subprocess.call(cmd) != 0:
        if made_dir and not os.listdir(map_dir):
            os.rmdir(map_dir)  # a failed save leaves no trace in the registry
        sys.exit(
            f'\nmap_saver_cli failed. It waits for one message on {args.topic} '
            f'and gives up if none arrives.\n'
            f'  Is slam:=true still running? `ros2 topic hz {args.topic}`')

    # The manifest is the source of truth; a map on disk that it does not
    # declare is invisible to every backend.
    manifest = os.path.join(world.dir, 'world.yaml')
    with open(manifest) as f:
        text = f.read()
    if world.manifest.get('map') != rel:
        if 'map:' in text and world.manifest.get('map'):
            print(f'\nNOTE: {manifest} declares map: '
                  f'{world.manifest["map"]}, not {rel}. Left alone — edit it by '
                  f'hand if the new map is the one you want.')
        else:
            with open(manifest, 'a') as f:
                f.write(f'\n# The Nav2 map of this environment, recorded by '
                        f'slam_toolbox via scripts/save_map.py.\nmap: {rel}\n')
            print(f'\n{manifest}: added map: {rel}')

    print(f'\n{stem}.yaml\n{stem}.pgm\n\n'
          f'    ros2 launch tb3_bringup bringup.launch.py '
          f'backend:=<backend> world:={world.name} nav:=true')


if __name__ == '__main__':
    main()
