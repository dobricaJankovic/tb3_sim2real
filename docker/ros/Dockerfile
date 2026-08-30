# The "everything that is ROS" image. Ubuntu 22.04 / Humble / python 3.10.
#
# Isaac Sim is NOT in here and must not be. It lives in its own noble-based
# image and we reach it over DDS.
#
# Seeded from turtlebot3_ws/src/turtlebot3/docker/humble/Dockerfile, plus
# Gazebo Classic so this one image can be the gazebo backend too.

FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV COLCON_WS=/ws

# --- Nav2 + visualisation + the shared bits -----------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-pointcloud-to-laserscan \
    ros-${ROS_DISTRO}-cartographer \
    ros-${ROS_DISTRO}-cartographer-ros \
    python3-colcon-common-extensions \
    python3-argcomplete \
    build-essential git nano \
 && rm -rf /var/lib/apt/lists/*

# --- Gazebo Classic backend ---------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-gazebo-ros2-control \
 && rm -rf /var/lib/apt/lists/*

# --- TurtleBot3: description, real-robot drivers, sim worlds ------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-turtlebot3-msgs \
    ros-${ROS_DISTRO}-turtlebot3 \
    ros-${ROS_DISTRO}-turtlebot3-simulations \
    ros-${ROS_DISTRO}-dynamixel-sdk \
    ros-${ROS_DISTRO}-hls-lfcd-lds-driver \
    ros-${ROS_DISTRO}-ld08-driver \
 && rm -rf /var/lib/apt/lists/*

# TODO: if we end up needing the local turtlebot3_ws sources instead of the
# apt packages (patched drivers, coin_d4_driver, etc), clone or bind-mount them
# into ${COLCON_WS}/src and drop the apt lines above.

WORKDIR ${COLCON_WS}
RUN mkdir -p ${COLCON_WS}/src

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "[ -f ${COLCON_WS}/install/setup.bash ] && source ${COLCON_WS}/install/setup.bash" >> /root/.bashrc && \
    echo "alias cb='colcon build --symlink-install'" >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
