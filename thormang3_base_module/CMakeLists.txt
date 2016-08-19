################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(thormang3_base_module)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  robotis_math
  robotis_controller_msgs
  robotis_framework_common
  roscpp
  std_msgs
  geometry_msgs
  cmake_modules
  thormang3_kinematics_dynamics
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
  LIBRARIES thormang3_base_module
  CATKIN_DEPENDS roscpp std_msgs robotis_framework_common
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(thormang3_base_module
  src/base_module.cpp
  src/base_module_state.cpp
)

add_dependencies(thormang3_base_module ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(thormang3_base_module
  yaml-cpp
  ${catkin_LIBRARIES}
)

################################################################################
# Install
################################################################################
install(TARGETS thormang3_base_module
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
