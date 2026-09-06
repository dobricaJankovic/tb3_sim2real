# The "everything that is ROS" image. Ubuntu 22.04 / Humble / python 3.10.
#
# Isaac Sim is NOT in here and must not be. It lives in its own noble-based
# image and we reach it over DDS.
#
# Seeded from turtlebot3_ws/src/turtlebot3/docker/humble/Dockerfile, plus
# Gazebo Classic so this one image can be the gazebo backend too.

FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV COLCON_WS=/ws
# ROS_DISTRO is inherited from the base image and is deliberately not re-set
# here: declaring it again would silently disagree with the base if the FROM
# tag above ever changes.

# --- Nav2 + visualisation + the shared bits -----------------------------------
# rviz2 rather than the ros-humble-desktop metapackage. Desktop pulls 91 extra
# packages (~577 MB against ~111 MB) and nearly all of it is tutorials and demos
# — turtlesim, the rclcpp/rclpy minimal examples, pendulum, the dummy robot.
# nav2-bringup depends on rviz-common but NOT on rviz2, so RViz has to be asked
# for by name or rviz/tb3.rviz has nothing to open it.
#
# demo-nodes-cpp is kept on purpose despite being a demo: talker/listener is the
# cheapest cross-container DDS check there is, and the failures on this project
# are usually silent ones where that is exactly what you want to reach for.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-demo-nodes-cpp \
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

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "[ -f ${COLCON_WS}/install/setup.bash ] && source ${COLCON_WS}/install/setup.bash" >> /root/.bashrc && \
    echo "alias cb='colcon build --symlink-install'" >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
