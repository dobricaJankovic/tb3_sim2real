# The tb3_sim2real image: TurtleBot3 on top of Isaac Sim + ROS 2 Humble.
#
# Isaac Sim IS in here now, and that is the change of 2026-09-13. This used to
# say it "must not be" and that Isaac Sim "lives in its own noble-based image",
# because a locally built Isaac Sim needs GLIBC_2.38 and jammy has 2.35. NVIDIA's
# *released* build is compiled to 2.34 and runs here; see docs/architecture.md.
#
# Two Dockerfiles, deliberately:
#
#   docker/isaacsim-ros2/Dockerfile.humble  Isaac Sim + ROS 2 Humble, and
#       nothing else. No TurtleBot3, no X11, no workspace overlay. It stands on
#       its own and is not specific to this repo.
#   this file                               everything that IS specific to us:
#       the TurtleBot3 description, the real-robot drivers, the sim worlds, and
#       the workspace the bringup package is built in.
#
# The base is built first. scripts/build_images.sh does both in order.

ARG BASE_IMAGE=isaacsim6-humble:ngc
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive
ENV COLCON_WS=/ws
# ROS_DISTRO is inherited and deliberately not re-set: declaring it again would
# silently disagree with the base if its tag ever changes.

# --- what the base does NOT already provide ----------------------------------
# The base installs ros-${ROS_DISTRO}-desktop, navigation2, nav2-bringup,
# gazebo-ros-pkgs, gazebo-ros2-control, xacro, tf2-tools,
# pointcloud-to-laserscan, colcon and vcstool. Audited against the built image
# rather than assumed, so this list is only the remainder.
#
# The old rviz2-instead-of-desktop trick is gone with them. It saved ~466 MB by
# skipping the tutorials and demos, which mattered against a 5 GB image and does
# not against a 38 GB one where Isaac Sim is 25 GB of it. demo-nodes-cpp comes
# with desktop anyway, and talker/listener stays the cheapest DDS check there is.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-cartographer \
    ros-${ROS_DISTRO}-cartographer-ros \
    python3-argcomplete \
    build-essential git nano \
 && rm -rf /var/lib/apt/lists/*

# --- TurtleBot3: description, real-robot drivers, sim worlds ------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-turtlebot3-msgs \
    ros-${ROS_DISTRO}-turtlebot3 \
    ros-${ROS_DISTRO}-turtlebot3-description \
    ros-${ROS_DISTRO}-turtlebot3-bringup \
    ros-${ROS_DISTRO}-turtlebot3-navigation2 \
    ros-${ROS_DISTRO}-turtlebot3-simulations \
    ros-${ROS_DISTRO}-turtlebot3-gazebo \
    ros-${ROS_DISTRO}-dynamixel-sdk \
    ros-${ROS_DISTRO}-hls-lfcd-lds-driver \
    ros-${ROS_DISTRO}-ld08-driver \
    ros-${ROS_DISTRO}-coin-d4-driver \
 && rm -rf /var/lib/apt/lists/*

# TODO: if we end up needing the local turtlebot3_ws sources instead of the
# apt packages (patched drivers, coin_d4_driver, etc), clone or bind-mount them
# into ${COLCON_WS}/src and drop the apt lines above.

WORKDIR ${COLCON_WS}
RUN mkdir -p ${COLCON_WS}/src

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# `isaacsim` and `isaacsim-python` come from the base and are the ONLY correct
# way to start Kit: they go through ros-isolate, which takes /opt/ros and every
# sourced overlay back off PYTHONPATH/LD_LIBRARY_PATH/PATH. Calling
# /isaac-sim/python.sh directly from a ROS shell loads the wrong interpreter's
# modules, because setup_python_env.sh APPENDS to PYTHONPATH and the sourced
# ROS 2 stays ahead of Kit's own entries.
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "[ -f ${COLCON_WS}/install/setup.bash ] && source ${COLCON_WS}/install/setup.bash" >> /root/.bashrc && \
    echo "alias cb='colcon build --symlink-install'" >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
