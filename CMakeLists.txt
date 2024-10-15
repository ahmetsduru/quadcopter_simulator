cmake_minimum_required(VERSION 3.0.2)
project(quadcopter_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  mavros_msgs
  roscpp
  std_msgs
  sensor_msgs
  message_generation
  genmsg
  tf2
  tf2_geometry_msgs
  geometry_msgs
  message_filters
)

generate_messages(
  DEPENDENCIES
  mavros_msgs
  std_msgs
  sensor_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES quadcopter_control
 CATKIN_DEPENDS mavros_msgs roscpp std_msgs sensor_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(real_time_control src/real_time_control.cpp)
target_link_libraries(real_time_control ${catkin_LIBRARIES})
add_dependencies(real_time_control quadcopter_control_generate_messages_cpp)

add_executable(trajectory_generator src/trajectory_generator.cpp)
target_link_libraries(trajectory_generator ${catkin_LIBRARIES})
add_dependencies(trajectory_generator quadcopter_control_generate_messages_cpp)

add_executable(mid_level_cont src/mid_level_cont.cpp)
target_link_libraries(mid_level_cont ${catkin_LIBRARIES})
add_dependencies(mid_level_cont quadcopter_control_generate_messages_cpp)

add_executable(low_level_cont src/low_level_cont.cpp)
target_link_libraries(low_level_cont ${catkin_LIBRARIES})
add_dependencies(low_level_cont quadcopter_control_generate_messages_cpp)

add_executable(eom src/eom.cpp)
target_link_libraries(eom ${catkin_LIBRARIES})
add_dependencies(eom quadcopter_control_generate_messages_cpp)

add_executable(circle_traj src/circle_traj.cpp)
target_link_libraries(circle_traj ${catkin_LIBRARIES})
add_dependencies(circle_traj quadcopter_control_generate_messages_cpp)

add_executable(circle_heli_traj src/circle_heli_traj.cpp)
target_link_libraries(circle_heli_traj ${catkin_LIBRARIES})
add_dependencies(circle_heli_traj quadcopter_control_generate_messages_cpp)

add_executable(rviz_data src/rviz_data.cpp)
target_link_libraries(rviz_data ${catkin_LIBRARIES})
add_dependencies(rviz_data quadcopter_control_generate_messages_cpp)

add_executable(data_logger src/data_logger.cpp)
target_link_libraries(data_logger ${catkin_LIBRARIES})
add_dependencies(data_logger quadcopter_control_generate_messages_cpp)