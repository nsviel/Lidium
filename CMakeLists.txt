#Project
cmake_minimum_required(VERSION 3.1)
project(DUNE LANGUAGES CXX)

#General policy
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/build)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
cmake_policy(SET CMP0072 NEW)

#Library dependencies
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(GLEW REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PCL 1.2 REQUIRED)
find_package(OpenMP REQUIRED)

#Header libraries
include_directories(${CMAKE_SOURCE_DIR}/../extern)

#Source files
file(GLOB_RECURSE ALL_SOURCES src/* extern/*)

#Packing
set(ALL_LIBS
  ${OPENGL_LIBRARY}
  ${GLFW_LIBRARY}
  ${GLEW_LIBRARY}
  ${GLM_LIBRARY}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  OpenMP::OpenMP_CXX
  Eigen3::Eigen
  glfw
  armadillo
  -lstdc++fs
  -lblas
  -llapack
  -fopenmp)

#Executable
add_executable(executable
  ${ALL_SOURCES})

#Library linking
target_link_libraries(executable PRIVATE ${OpenMP_CXX_FLAGS}
  ${ALL_LIBS})

#Options compilation
target_compile_options(executable PRIVATE ${OpenMP_CXX_FLAGS})


 
