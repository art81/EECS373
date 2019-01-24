cmake_minimum_required(VERSION 2.8.3)
project(my_stdr_control)

find_package(catkin_simple REQUIRED)

catkin_simple() 

# Executables: uncomment the following and edit arguments to compile new nodes
# may add more of these lines for more nodes from the same package
cs_add_executable(my_stdr_open_loop_commander src/my_stdr_open_loop_commander.cpp)

cs_install()
cs_export()
    
