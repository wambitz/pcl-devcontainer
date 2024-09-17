# Global CMakeLists.txt
cmake_minimum_required(VERSION 3.10)

# Project Name
project(PCL)

# Set C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable all warnings
set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

# Add compiler flags for debugging and optimizations
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0")

# Include directories for PCL and Boost globally
include_directories(${PCL_INCLUDE_DIRS})

# Link directories for PCL globally
link_directories(${PCL_LIBRARY_DIRS})

# Add definitions from PCL globally
add_definitions(${PCL_DEFINITIONS})

# Add subdirectories for each project
add_subdirectory(00_HelloWorld)
