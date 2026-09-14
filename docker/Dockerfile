# The tb3_sim2real image: TurtleBot3 on top of Isaac Sim + ROS 2 Humble.
#
# One thin layer, deliberately. The base -- NVIDIA's official
# nvcr.io/nvidia/isaac-sim:6.1.0 with ROS 2 Humble added, plus the `isaacsim`
# and `isaacsim-python` wrappers that run ros-isolate -- is not built here. It
# is built from src/turtlebot3_isaacsim/docker/isaacsim-ros2/Dockerfile.humble,
# the same definition that package uses to run standalone, because this
# repository imports that package rather than copying it and the same has to be
# true of the environment it needs. scripts/build_images.sh builds both in order.
#
# What is left here is exactly what is specific to sim-to-real and absent from a
# pure Isaac Sim image: Gazebo Classic's TurtleBot3 worlds, the real robot's
# drivers, cartographer, and the workspace tb3_bringup is built in.

ARG BASE_IMAGE=isaacsim61-humble:ngc
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
# vcstool comes from the base and is what scripts/workspace.sh needs, so the
# source dependencies can be imported from inside the container on a host that
# has no ROS 2 installed at all.
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
