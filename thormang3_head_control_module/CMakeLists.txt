################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(thormang3_head_control_module)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  robotis_math
  robotis_controller_msgs
  robotis_framework_common
  roscpp
  std_msgs
  cmake_modules
)

find_package(Eigen REQUIRED)

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
  LIBRARIES thormang3_head_control_module
  CATKIN_DEPENDS roscpp robotis_framework_common
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(thormang3_head_control_module
  src/head_control_module.cpp
)

add_dependencies(thormang3_head_control_module ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(thormang3_head_control_module
  ${catkin_LIBRARIES}
)

################################################################################
# Install
################################################################################
install(TARGETS thormang3_head_control_module
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

################################################################################
# Test
################################################################################
