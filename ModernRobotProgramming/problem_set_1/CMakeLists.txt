cmake_minimum_required(VERSION 2.8.3)
project(problem_set_1)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(minimal_simulator1 src/minimal_simulator.cpp)
add_executable(minimal_controller1 src/minimal_controller.cpp)
add_executable(sin_commander1 src/sin_commander.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(minimal_simulator1  ${catkin_LIBRARIES} )
target_link_libraries(minimal_controller1  ${catkin_LIBRARIES} )
target_link_libraries(sin_commander1  ${catkin_LIBRARIES} )
