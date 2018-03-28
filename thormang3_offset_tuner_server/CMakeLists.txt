################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(thormang3_offset_tuner_server)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  cmake_modules
  dynamixel_sdk
  robotis_device
  robotis_math
  robotis_framework_common
  robotis_controller
  robotis_controller_msgs
  thormang3_kinematics_dynamics
  thormang3_base_module
  thormang3_offset_tuner_msgs
)

find_package(Eigen REQUIRED)

set(ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:${CATKIN_DEVEL_PREFIX}/lib/pkgconfig")
find_package(PkgConfig)
pkg_check_modules(yaml_cpp yaml-cpp REQUIRED)

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
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${yaml_cpp_INCLUDE_DIRS}
)

add_executable(thormang3_offset_tuner_server_node 
   src/thormang3_offset_tuner_server.cpp
   src/main.cpp
)

add_dependencies(thormang3_offset_tuner_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(thormang3_offset_tuner_server_node ${yaml_cpp_LIBRARIES} ${catkin_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS thormang3_offset_tuner_server_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

################################################################################
# Test
################################################################################
