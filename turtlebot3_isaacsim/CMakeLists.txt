################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(turtlebot3_isaacsim)

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)

################################################################################
# Install
################################################################################
# No targets to build: unlike turtlebot3_gazebo there are no C++ Gazebo plugins
# here. Isaac Sim's equivalent of a model plugin is an OmniGraph node, authored
# from Python inside Kit -- see scripts/turtlebot3_isaacsim.py.
install(DIRECTORY launch models worlds
  DESTINATION share/${PROJECT_NAME}/
)

# USE_SOURCE_PERMISSIONS, because install(DIRECTORY) otherwise applies its
# default 644 and strips the execute bit off scripts/build_models.sh -- which
# fails at the point someone runs it out of the install tree rather than the
# source checkout, i.e. the documented way.
install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}/
  USE_SOURCE_PERMISSIONS
)

################################################################################
# Macro for ament package
################################################################################
ament_package()
