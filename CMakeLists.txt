cmake_minimum_required(VERSION 3.12)
project(MultiLane)

set(CMAKE_CXX_STANDARD 11)


find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})


aux_source_directory(./ SRI_DIR)
add_executable(MultiLane ${SRI_DIR})

target_link_libraries(MultiLane ${OpenCV_LIBS})