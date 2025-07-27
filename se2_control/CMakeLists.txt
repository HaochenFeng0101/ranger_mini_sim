cmake_minimum_required(VERSION 3.0.2)
project(se2_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  tf
  nodelet
  cmake_utils
  four_wheel_steering_msgs
  std_msgs
  sensor_msgs
  message_generation
)

find_package(Eigen3 REQUIRED)

add_message_files(
  FILES
  SE2Command.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    nav_msgs
    geometry_msgs
    tf
    nodelet
    cmake_utils
    four_wheel_steering_msgs
    std_msgs
    sensor_msgs
    message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

# Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)

# Install scripts
install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
  USE_SOURCE_PERMISSIONS
)

# Install nodelet plugin
install(FILES nodelet_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
) 