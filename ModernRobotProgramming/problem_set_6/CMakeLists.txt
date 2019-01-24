cmake_minimum_required(VERSION 2.8.3)
project(problem_set_6)

find_package(catkin_simple REQUIRED)

catkin_simple()  

# Executables: uncomment the following and edit arguments to compile new nodes
# may add more of these lines for more nodes from the same package
cs_add_executable(ps6 src/ps6.cpp)

cs_install()
cs_export()
    
