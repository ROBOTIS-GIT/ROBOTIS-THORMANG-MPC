################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(thormang3_action_module)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  robotis_controller_msgs
  robotis_framework_common
  thormang3_action_module_msgs
)

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES thormang3_action_module
  CATKIN_DEPENDS roscpp robotis_framework_common
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(thormang3_action_module
  src/action_module.cpp
)

add_dependencies(thormang3_action_module ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

################################################################################
# Install
################################################################################
install(TARGETS thormang3_action_module
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY data/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
################################################################################
# Test
################################################################################
