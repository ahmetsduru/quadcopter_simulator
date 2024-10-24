cmake_minimum_required(VERSION 3.0.2)
project(quadcopter_control)

## Compile as C++17, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
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

add_service_files(
  FILES
  WaypointService.srv
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

## BaseClass, MidLevelController, and LowLevelController files
add_library(base_class src/base_class.cpp)
add_dependencies(base_class ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(mid_level_cont src/mid_level_cont.cpp)
target_link_libraries(mid_level_cont base_class ${catkin_LIBRARIES})
add_dependencies(mid_level_cont ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(low_level_cont src/low_level_cont.cpp)
target_link_libraries(low_level_cont base_class ${catkin_LIBRARIES})
add_dependencies(low_level_cont ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Other executable files
add_executable(real_time_control src/real_time_control.cpp)
target_link_libraries(real_time_control ${catkin_LIBRARIES})
add_dependencies(real_time_control quadcopter_control_generate_messages_cpp)

add_executable(trajectory_generator src/trajectory_generator.cpp)
target_link_libraries(trajectory_generator ${catkin_LIBRARIES})
add_dependencies(trajectory_generator quadcopter_control_generate_messages_cpp)

add_executable(eom src/eom.cpp)
target_link_libraries(eom ${catkin_LIBRARIES})
add_dependencies(eom quadcopter_control_generate_messages_cpp)

add_executable(rviz_data src/rviz_data.cpp)
target_link_libraries(rviz_data ${catkin_LIBRARIES})
add_dependencies(rviz_data quadcopter_control_generate_messages_cpp)

## Add data_logger executable (updated part)
add_executable(data_logger_node src/data_logger_node.cpp src/data_logger.cpp)
target_link_libraries(data_logger_node ${catkin_LIBRARIES})
add_dependencies(data_logger_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Add waypoint_server executable
add_executable(waypoint_server src/waypoint_server.cpp)
target_link_libraries(waypoint_server ${catkin_LIBRARIES})
add_dependencies(waypoint_server quadcopter_control_generate_messages_cpp)
