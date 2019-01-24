cmake_minimum_required(VERSION 2.8.3)
project(problem_set_4)

find_package(catkin_simple REQUIRED)

catkin_simple()

cs_add_executable(traj_sender src/irb120_traj_sender)

cs_install()
cs_export()
