cmake_minimum_required(VERSION 3.10)
project(flightctl_cpp CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

add_executable(flightctl_cpp
    src/main.cpp
    src/dynamics.cpp
    src/params.cpp
    src/lqr_controller.cpp
    src/nn_controller.cpp
)
target_include_directories(flightctl_cpp PRIVATE include)
