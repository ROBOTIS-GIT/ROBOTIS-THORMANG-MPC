################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(thormang3_feet_ft_module)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  robotis_math
  robotis_framework_common
  thormang3_kinematics_dynamics
  thormang3_feet_ft_module_msgs
  ati_ft_sensor
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
  LIBRARIES thormang3_feet_ft_module
  CATKIN_DEPENDS roscpp geometry_msgs robotis_framework_common
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(thormang3_feet_ft_module
  src/feet_force_torque_sensor_module.cpp
)

add_dependencies(thormang3_feet_ft_module ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(thormang3_feet_ft_module
  yaml-cpp
  ${catkin_LIBRARIES}
)

################################################################################
# Install
################################################################################
install(TARGETS thormang3_feet_ft_module
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
