cmake_minimum_required(VERSION 3.0.2)
project(eric_a_imu)

###################################
# Packages
###################################
find_package(catkin REQUIRED
  roscpp
  std_msgs
  std_srvs
  sensor_msgs
  tf
  geometry_msgs
  message_generation
)

## add Service....
add_service_files(
  FILES
  all_data_reset.srv
  euler_angle_init.srv
  euler_angle_reset.srv
  pose_velocity_reset.srv
  reboot_sensor.srv
)

generate_messages(   
  DEPENDENCIES  
  std_msgs
  std_srvs
  geometry_msgs
) 

###################################
## catkin specific configuration ##
###################################
catkin_package(
  LIBRARIES iahrs_driver
  CATKIN_DEPENDS roscpp std_msgs std_srvs sensor_msgs tf
)

###########
## Build ##
###########
include_directories(
 ${catkin_INCLUDE_DIRS}
)

#add_compile_options(-std=c++11)
add_executable(eric_a_imu src/iahrs_driver.cpp)
add_dependencies(eric_a_imu ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(eric_a_imu ${catkin_LIBRARIES})
