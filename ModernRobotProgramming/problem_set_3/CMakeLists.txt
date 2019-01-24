cmake_minimum_required(VERSION 2.8.3)
project(problem_set_3)

find_package(catkin_simple REQUIRED)

find_package(Boost REQUIRED COMPONENTS system thread)
catkin_simple()

# Libraries: uncomment the following and edit arguments to create a new library
cs_add_library(sinCommanderActionServerLib src/sin_commander_action_server_lib.cpp)   

# Executables: uncomment the following and edit arguments to compile new nodes
cs_add_executable(sinCommanderActionClient src/sin_commander_action_client.cpp)
cs_add_executable(sinCommanderActionServer src/sin_commander_action_server.cpp)

target_link_libraries(sinCommanderActionServer sinCommanderActionServerLib)

cs_install()
cs_export()
    
