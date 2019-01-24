cmake_minimum_required(VERSION 2.8.3)
project(shipment_filler)

find_package(catkin_simple REQUIRED)

catkin_simple()

if(MSVC)  
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Zi")  
else()  
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")  
endif()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# use carefully;  can interfere with point-cloud library
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x") 

# Executables: uncomment the following and edit arguments to compile new nodes
cs_add_executable(fill_order src/fill_order.cpp)
#cs_add_executable(unload_box src/unload_box.cpp)

cs_install()
cs_export()
    
