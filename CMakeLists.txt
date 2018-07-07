cmake_minimum_required(VERSION 2.8.3)
project(serial_logger)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

## catkin specific configuration ##
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
)

## Build ##

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node 
  src/serial_logger_node.cpp
  src/main.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)


## Testing ##

if (CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()
endif()
