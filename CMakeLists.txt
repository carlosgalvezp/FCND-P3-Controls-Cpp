cmake_minimum_required(VERSION 3.5)
project(CPPSim)

# CXX Flags
set(CMAKE_CXX_FLAGS "--std=c++11 -O3 -Wno-unused-parameter -Wno-unused-local-typedefs")

# Include directories
include_directories(src lib)

# Subdirectories
add_subdirectory(src)
