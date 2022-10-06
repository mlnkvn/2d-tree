cmake_minimum_required(VERSION 3.20)
project(2d_tree)

set(CMAKE_CXX_STANDARD 20)

include_directories(include)

add_executable(2d_tree
        include/primitives.h
        src/2dtree.cpp
        src/main.cpp)
