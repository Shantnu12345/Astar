cmake_minimum_required(VERSION 3.7)

add_definitions(-std=c++17)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(Planner)

include_directories(./src)

add_executable(Planner src/main.cpp src/Planner.cpp src/helper.cpp)
