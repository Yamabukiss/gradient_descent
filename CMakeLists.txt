cmake_minimum_required(VERSION 3.25)
project(optimize_project)

set(CMAKE_CXX_STANDARD 17)

include_directories(../eigen/eigen-3.4.0)

add_executable(optimize_project main.cpp searcher.cpp solver.cpp)
