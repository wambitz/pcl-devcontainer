# 00_HelloWorld/CMakeLists.txt
cmake_minimum_required(VERSION 3.10)

# Find only the necessary PCL components for this subproject
find_package(PCL 1.13 REQUIRED COMPONENTS common io visualization)

# Define the executable for this subproject
add_executable(hello_pcl hello.cpp)

# Link the necessary libraries
target_link_libraries(hello_pcl ${PCL_LIBRARIES})