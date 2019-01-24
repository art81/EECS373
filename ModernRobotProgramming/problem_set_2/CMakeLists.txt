cmake_minimum_required(VERSION 2.8.3)
project(problem_set_2)

find_package(catkin_simple REQUIRED)

catkin_simple()

# Executables:
cs_add_executable(minimal_controller2 src/minimal_controller.cpp)
cs_add_executable(minimal_simulator2 src/minimal_simulator.cpp)
cs_add_executable(sin_commander_service src/sin_commander_service.cpp)
cs_add_executable(sin_commander_client src/sin_commander_client.cpp)

cs_install()
cs_export()

