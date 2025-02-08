cmake_minimum_required(VERSION 3.10)
project(IMU)

cmake_policy(SET CMP0167 NEW)
add_compile_options(-Wno-deprecated-declarations)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})

find_package(NumCpp REQUIRED)
find_package(Python3 COMPONENTS Development REQUIRED)

include_directories(${NumCpp_INCLUDE_DIRS})
include_directories(${Python3_INCLUDE_DIRS})
include_directories("/home/d3shin/miniconda3/envs/ece276a/lib/python3.13/site-packages/numpy/_core/include")
include_directories(${CMAKE_SOURCE_DIR}/utils)
include_directories(${CMAKE_SOURCE_DIR}/utils/csv2/single_include)
include_directories(${CMAKE_SOURCE_DIR}/utils/eigen)
include_directories(${CMAKE_SOURCE_DIR}/utils/matplotlib-cpp)
include_directories(core)

add_subdirectory(utils/pybind11)
add_executable(IMU
  # entry
  main.cpp
  
  # dataset and utilities
  core/ViconDataset.cpp
  core/IMUDataset.cpp
 
  # core functionality
  core/DataProcessor.cpp
  core/Bridge.cpp
)

target_include_directories(IMU PRIVATE
  ${CMAKE_SOURCE_DIR}/utils/csv2/single_include  # Ensure utils/csv2/single_include is included
  ${NumCpp_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
  "/home/d3shin/miniconda3/envs/ece276a/lib/python3.13/site-packages/numpy/_core/include"
  ${CMAKE_SOURCE_DIR}/core/eigen
  ${CMAKE_SOURCE_DIR}/core/matplotlib-cpp
  core
)

target_link_libraries(IMU PRIVATE NumCpp::NumCpp)
target_link_libraries(IMU PRIVATE pybind11::embed)
target_link_libraries(IMU PRIVATE Python3::Python)
