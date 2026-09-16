# Talking to the real robot over the network

The robot is a second computer. When it is on a different subnet from the
workstation — which it is here, and which is the normal case for a robot on
Wi-Fi and a workstation on Ethernet — ROS 2 will not find it, and will not say
so. This is what that looks like and how it is set up.

## The failure

```
ros2 topic list       # /parameter_events and /rosout, nothing else
```

No error, no warning, on either machine. `ROS_DOMAIN_ID` is right, both are on
Humble, ping works, SSH works.

DDS discovers participants by announcing them to the multicast address
`239.255.0.1`. **No router forwards multicast between subnets** unless someone
has deliberately configured it, and campus networks do not. SSH and ping are
unicast, so they cross and prove nothing about discovery. The two ROS graphs
simply never hear of each other.

Measured here on 2026-09-16:

| | address | subnet |
|---|---|---|
| workstation | `10.118.5.241` | `10.118.5.0/24`, wired |
| TurtleBot3 | `10.118.19.161` | `10.118.16.0/22`, Wi-Fi |

## Diagnosing it

In order — each step rules out one thing:

```bash
ping <robot>                      # routing. Passing proves nothing about DDS.
ssh <robot> 'echo $ROS_DOMAIN_ID' # must match this machine's
ip -brief addr                    # compare the two subnets. Different -> this doc.
```

Then the test that actually matters, because the fix below only works if UDP
crosses the router:

```bash
ssh <robot> 'nc -u -l 7400 > /tmp/rx &'      # listener on the robot
echo hello | nc -u -w1 <robot> 7400          # from the workstation
ssh <robot> 'cat /tmp/rx'                    # arrived?
```

Do it in both directions. If UDP does not cross, nothing here helps and the
answer is a Fast DDS Discovery Server or a VPN.

## The fix: name the robot as a unicast peer

Multicast is replaced by telling Fast DDS the robot's address directly, so
discovery packets are sent to it as unicast. In the container this is one
variable, in `.env` (gitignored, because a DHCP lease is site data and not
repository data):

```bash
TB3_DDS_PEERS=10.118.19.161
```

`docker/entrypoint.sh` turns that into a Fast DDS profile at
`/tmp/fastdds_peers.xml` on container start, and `docker-compose.yml` points
`FASTRTPS_DEFAULT_PROFILES_FILE` at it. Unset `TB3_DDS_PEERS` writes a profile
with no peer list at all, which leaves Fast DDS stock — a simulator-only run is
unaffected.

Changing `.env` needs `docker compose up -d` to recreate the container, and a
recreate empties `/ws/install`, so `colcon build --symlink-install` after.

### The trap, and it cost an hour

**Naming any initial peer replaces Fast DDS's default locator list, and the
defaults are where multicast and localhost live.** A profile that lists only
the robot reaches the robot and stops the machine finding *its own processes*.

The symptom is not an error. Nav2's `component_container_isolated` starts, no
composable node is ever loaded into it, and the launch sits there indefinitely.
`ros2 node list` returns empty while `ros2 topic list` still shows the robot.

So the generated list always re-adds both:

- `239.255.0.1` — the default ROS 2 discovery multicast address, restoring
  same-subnet discovery
- `127.0.0.1` — restoring same-host discovery

## Running it

The robot runs its own stock bringup, unchanged:

```bash
ssh etfrobot@10.118.19.161
ros2 launch turtlebot3_bringup robot.launch.py
```

That publishes `/scan`, `/odom`, the URDF and tf — the whole **robot layer**.
The workstation therefore runs only the **workstation layer**, which is what
`backend:=real` means and all it means:

```bash
docker compose exec tb3_ros bash
ros2 launch tb3_bringup bringup.launch.py backend:=real world:=<name>
```

There is no second argument to remember. `backend:=real` starts no local
processes at all: no `common/state_publisher.launch.py`, no drivers. If it did,
a second `robot_state_publisher` would fight the robot's over `/tf_static` and
`/robot_description`, and the drivers would go looking for USB devices attached
to the other machine.

> This used to be `robot:=remote`, with a `robot:=local` that tethered the
> OpenCR and the lidar to the workstation over USB. The flag is gone as of
> 2026-09-16 — a robot that drives cannot be tethered, so `remote` was the only
> value anyone ever passed. `ros2 launch` ignores unknown arguments silently, so
> an old `robot:=remote` in a note or a shell history still works; it simply has
> no effect. The tethered path is recorded as a comment in
> `bringup.launch.py`, for the day an x86 SBC or a Jetson goes on the robot.

The cost of this split is that the real backend's kinematic tree comes from the
robot's own `turtlebot3_description`, not from this repository's URDF — so the
"one URDF above `base_footprint`" property holds across `gazebo` and
`isaacsim` but not across `real`. The alternative was to stop the robot
publishing it; that decision is recorded in `docs/history.md`.

## Verifying

```bash
docker compose exec tb3_ros /entrypoint.sh bash -c 'ros2 node list'
```

Four nodes: `turtlebot3_node`, `robot_state_publisher`, `hlds_laser_publisher`,
`diff_drive_controller`. If topics appear but nodes do not, discovery is
half-working — suspect the locator trap above, or a distro mismatch (see below).

```bash
ros2 topic hz /scan          # 5 Hz on an LDS-01
ros2 run tf2_ros tf2_echo odom base_footprint
```

`ros2 daemon stop` after any change to these variables. The daemon caches the
graph and will keep reporting the old one.

## Two things that will bite later

**Both addresses are DHCP leases.** When the robot stops being visible, check
its address before anything else. One line in `.env`, then `up -d`.

**The workstation's own ROS 2 is Jazzy; the robot is Humble.** Cross-distro DDS
is not supported. It half-works — `topic list`, `topic echo` and `service list`
do, `node list` silently returns nothing — which is worse than failing. Use the
container for anything that matters. The host-side equivalent of this setup
lives in `~/.ros/fastdds_unicast.xml` and `~/.bashrc` if it is ever wanted.
