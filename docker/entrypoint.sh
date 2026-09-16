#!/usr/bin/env bash
set -e

source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f /ws/install/setup.bash ]; then
    source /ws/install/setup.bash
fi

# Cross-subnet discovery, written here rather than mounted because the
# addresses are DHCP leases -- site data, not repository data.
#
# DDS announces participants over multicast, and no router forwards multicast
# between subnets. A robot on another subnet is therefore invisible however
# correct ROS_DOMAIN_ID is, with no error on either side. Every address in
# TB3_DDS_PEERS (comma- or space-separated) becomes an explicit unicast
# discovery target instead.
#
# The <initialPeersList> element is written ONLY when there is something to put
# in it: an empty list is not the same as an absent one -- it would switch off
# the multicast that every same-subnet setup discovers by. Unset TB3_DDS_PEERS
# thus leaves Fast DDS entirely stock. Why any of this: docs/network.md
profiles=${FASTRTPS_DEFAULT_PROFILES_FILE:-/tmp/fastdds_peers.xml}
{
    echo '<?xml version="1.0" encoding="UTF-8" ?>'
    echo '<dds xmlns="http://www.eprosima.com">'
    echo '  <profiles>'
    echo '    <participant profile_name="tb3_peers" is_default_profile="true">'
    echo '      <rtps>'
    echo '        <builtin>'
    if [ -n "${TB3_DDS_PEERS:-}" ]; then
        echo '          <initialPeersList>'
        # Naming any initial peer REPLACES Fast DDS's default locator list, and
        # the defaults are where multicast and localhost live. Both go back in
        # explicitly or this machine stops finding its own processes: the
        # symptom is Nav2's component container starting and no composable node
        # ever loading into it, no error anywhere, the launch just sitting
        # there. 239.255.0.1 is the default ROS 2 discovery multicast address;
        # it restores same-subnet discovery, 127.0.0.1 restores same-host.
        # Measured 2026-09-16, docs/network.md.
        for peer in 239.255.0.1 127.0.0.1 $(echo "${TB3_DDS_PEERS}" | tr ',' ' '); do
            echo '            <locator><udpv4>'
            echo "              <address>${peer}</address>"
            echo '            </udpv4></locator>'
        done
        echo '          </initialPeersList>'
    fi
    echo '        </builtin>'
    echo '      </rtps>'
    echo '    </participant>'
    echo '  </profiles>'
    echo '</dds>'
} > "${profiles}"

# Loud about the settings that cause silent failures: a domain mismatch and,
# now, a peer list that is empty when it should not be.
echo "ROS_DISTRO=${ROS_DISTRO}  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}  TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL:-unset}"
echo "DDS peers=${TB3_DDS_PEERS:-<none: multicast only>}  profiles=${profiles}"

exec "$@"
