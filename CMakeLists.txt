cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(semantic_kitti_tools)

find_package(PCL 1.2 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} "${PROJECT_SOURCE_DIR}/include" "/usr/local/include")
link_directories(${PCL_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS} ${OpenCV_DEFINITIONS})

add_executable(point_cloud_mapping src/point_cloud_mapping.cpp)
target_link_libraries(point_cloud_mapping ${PCL_LIBRARIES})

add_executable(depth_map_generator src/depth_map_generator.cpp)
target_link_libraries(depth_map_generator ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# set(CMAKE_BUILD_TYPE Debug)
