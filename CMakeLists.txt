cmake_minimum_required(VERSION 2.8.11)
project(matlink_standalone)
set(CMAKE_BUILD_TYPE Debug)

add_executable(matlink_test matlink_test.cpp mavlinkClass.cpp)
