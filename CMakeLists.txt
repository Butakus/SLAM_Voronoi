cmake_minimum_required(VERSION 2.8)
project(megamap)


# Use C++11
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# ROOT_PATH definition
add_definitions (-DROOT_PATH="${CMAKE_SOURCE_DIR}")
# Put binaries in bin directory
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin/")

# OpenCV
find_package(OpenCV REQUIRED)
include_directories (${OpenCV_INCLUDE_DIRS})


add_library (Nodelib include/astar/Node.cpp)
add_library (Edgelib include/astar/Edge.cpp)
add_library (AStarlib include/astar/AStar.cpp)
add_library (ACOlib include/aco/ACOPlanner.cpp)

include_directories (include)

add_executable(voronoi src/voronoi.cpp)
target_link_libraries(voronoi ${OpenCV_LIBS} Nodelib Edgelib AStarlib ACOlib)
