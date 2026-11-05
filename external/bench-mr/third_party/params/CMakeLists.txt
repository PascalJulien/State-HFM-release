cmake_minimum_required(VERSION 2.8.11)
project(params)

set(CMAKE_CXX_STANDARD 11)

# Enable warnings and error on warning
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror")

# Disable globally unnecessary warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-parameter")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-function")

# Set compile flags for versions
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

find_package(nlohmann_json 3.2.0)

add_executable(params_demo include/params.hpp demo.cpp)
target_link_libraries(params_demo)

add_library(params INTERFACE)
target_include_directories(params INTERFACE include/)
