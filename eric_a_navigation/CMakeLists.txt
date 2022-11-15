################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.0.2)
project(eric_a_navigation)
set(CMAKE_CXX_FLAGS "-std=c++17 -O3 ${CMAKE_CXX_FLAGS}")
################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
    rospy
    actionlib
    actionlib_msgs
    std_msgs
    message_generation
    roscpp
    geometry_msgs
    tf2
    tf2_ros
    move_base
)

################################################################################
# Setup for python modules and scripts
################################################################################

catkin_python_setup()

################################################################################
# Declare ROS messages, services and actions
################################################################################

add_service_files(
  FILES
  ClassNumber.srv
)


generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)
################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES eric_a_teleop
 CATKIN_DEPENDS rospy std_msgs 
#  DEPENDS system_lib
)

################################################################################
# Build
################################################################################
include_directories(include ${catkin_INCLUDE_DIRS} )
################################################################################
# Install
################################################################################
catkin_install_python(PROGRAMS
  # nodes/init_pose.py
  # nodes/eric_a_costmap_clear.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch maps param rviz bag
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_library(Mcl_emcl2 src/Mcl.cpp)
#add_library(ExpResetMcl_emcl2 src/ExpResetMcl.cpp)
add_library(ExpResetMcl2_emcl2 src/ExpResetMcl2.cpp)
add_library(Particle_emcl2 src/Particle.cpp)
add_library(OdomModel_emcl2 src/OdomModel.cpp)
add_library(Pose_emcl2 src/Pose.cpp)
add_library(Scan_emcl2 src/Scan.cpp)
add_library(LikelihoodFieldMap_emcl2 src/LikelihoodFieldMap.cpp)

add_executable(emcl2_node src/emcl2_node.cpp)

add_dependencies(emcl2_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(emcl2_node Mcl_emcl2 ExpResetMcl2_emcl2 Particle_emcl2 Pose_emcl2 Scan_emcl2 OdomModel_emcl2 LikelihoodFieldMap_emcl2 ${catkin_LIBRARIES})
################################################################################
# Test
################################################################################
