# Project
cmake_minimum_required(VERSION 3.1)
project(LIDIUM LANGUAGES CXX)
cmake_policy(SET CMP0072 NEW)

# General policy
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/build)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Child CMakeLists
add_subdirectory(extern)
add_subdirectory(src)




 
