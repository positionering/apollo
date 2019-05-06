# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
cmake_minimum_required(VERSION 3.1.0)

project(pose-grej)

# Find librealsense2 installed package
find_package(realsense2 REQUIRED)

# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
SET(CMAKE_CXX_FLAGS  "-Wno-narrowing")

# Add the application sources to the target
add_executable(pose realsense-pose.cpp)

# Link librealsense2 to the target
target_link_libraries(pose ${realsense2_LIBRARY})
