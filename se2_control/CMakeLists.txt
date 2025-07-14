cmake_minimum_required(VERSION 2.8.3)
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

# Generate messages
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

# SE2Control library
add_library(se2_control_lib
  src/SE2Control.cpp
)

target_link_libraries(se2_control_lib
  ${catkin_LIBRARIES}
)

# SE2 Control Nodelet
add_library(se2_control_nodelet
  src/se2_control_nodelet.cpp
)

target_link_libraries(se2_control_nodelet
  se2_control_lib
  ${catkin_LIBRARIES}
)

# Control example
add_executable(se2_control_example
  src/control_example.cpp
)

target_link_libraries(se2_control_example
  se2_control_lib
  ${catkin_LIBRARIES}
)

# SE2 Simulator
add_executable(se2_simulator
  src/se2_simulator.cpp
)

target_link_libraries(se2_simulator
  ${catkin_LIBRARIES}
)

# Install targets
install(TARGETS se2_control_lib se2_control_nodelet se2_control_example se2_simulator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Also install nodelet library to package-local lib dir for nodelet compatibility
install(TARGETS se2_control_nodelet
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
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

# Install nodelet plugin
install(FILES nodelet_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
) 