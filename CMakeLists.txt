cmake_minimum_required(VERSION 3.12)
project(MultiLane)

set(CMAKE_CXX_STANDARD 11)


find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#find_package(Eigen3 REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIR})
#message(STATUS "eigen3 directory is: " ${EIGEN3_INCLUDE_DIR} )


aux_source_directory(./ SRI_DIR)
add_executable(MultiLane ${SRI_DIR})

target_link_libraries(MultiLane ${OpenCV_LIBS})