cmake_minimum_required(VERSION 2.8)
project(megamap)

find_package( OpenCV )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# Use C++11
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# ROOT_PATH definition
add_definitions (-DROOT_PATH="${CMAKE_SOURCE_DIR}")


find_package(OpenCV REQUIRED)
include_directories(src)

add_executable(voronoi src/voronoi.cpp)
target_link_libraries(voronoi ${OpenCV_LIBS})
