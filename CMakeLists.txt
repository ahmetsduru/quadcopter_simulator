cmake_minimum_required(VERSION 3.0.2)
project(quadcopter_control)

## Compile as C++17, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Set the build type. Options are:
## Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
## Debug          : w/ debug symbols, w/o optimization
## Release        : w/o debug symbols, w/ optimization
## RelWithDebInfo : w/ debug symbols, w/ optimization
## MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the build type" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Coverage" "Debug" "Release" "RelWithDebInfo" "MinSizeRel")
endif()

## General optimizations (balanced for performance and stability)
if(CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
  add_compile_options("-O2" "-march=native")
endif()

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
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Apply specific optimizations for performance-critical targets
set(CRITICAL_OPT_FLAGS "-O3 -march=native -funroll-loops")

add_executable(mid_level_cont_node src/mid_level_cont_node.cpp src/mid_level_cont.cpp)
target_link_libraries(mid_level_cont_node ${catkin_LIBRARIES})
add_dependencies(mid_level_cont_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
set_target_properties(mid_level_cont_node PROPERTIES COMPILE_FLAGS "${CRITICAL_OPT_FLAGS}")

add_executable(low_level_cont_node src/low_level_cont_node.cpp src/low_level_cont.cpp)
target_link_libraries(low_level_cont_node ${catkin_LIBRARIES})
add_dependencies(low_level_cont_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
set_target_properties(low_level_cont_node PROPERTIES COMPILE_FLAGS "${CRITICAL_OPT_FLAGS}")

add_executable(real_time_control src/real_time_control.cpp)
target_link_libraries(real_time_control ${catkin_LIBRARIES})
add_dependencies(real_time_control quadcopter_control_generate_messages_cpp)
set_target_properties(real_time_control PROPERTIES COMPILE_FLAGS "${CRITICAL_OPT_FLAGS}")

## No optimizations for RViz-related nodes
set(RVIZ_OPT_FLAGS "-O0 -g")

add_executable(rviz_data_handler_node src/rviz_data_handler_node.cpp src/rviz_data_handler.cpp)
target_link_libraries(rviz_data_handler_node ${catkin_LIBRARIES})
add_dependencies(rviz_data_handler_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
set_target_properties(rviz_data_handler_node PROPERTIES COMPILE_FLAGS "${RVIZ_OPT_FLAGS}")

## Other executables
add_executable(trajectory_generator_node src/trajectory_generator_node.cpp src/trajectory_generator.cpp)
target_link_libraries(trajectory_generator_node ${catkin_LIBRARIES})
add_dependencies(trajectory_generator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(eom src/eom.cpp)
target_link_libraries(eom ${catkin_LIBRARIES})
add_dependencies(eom quadcopter_control_generate_messages_cpp)

add_executable(data_logger_node src/data_logger_node.cpp src/data_logger.cpp)
target_link_libraries(data_logger_node ${catkin_LIBRARIES})
add_dependencies(data_logger_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(waypoint_server_node src/waypoint_server_node.cpp src/waypoint_server.cpp)
target_link_libraries(waypoint_server_node ${catkin_LIBRARIES})
add_dependencies(waypoint_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############

install(TARGETS
  mid_level_cont_node
  low_level_cont_node
  real_time_control
  trajectory_generator_node
  eom
  rviz_data_handler_node
  data_logger_node
  waypoint_server_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
