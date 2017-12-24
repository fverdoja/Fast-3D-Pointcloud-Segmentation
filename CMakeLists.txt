cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(SupervoxelSegmentation)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (supervoxel_segmentation supervoxel_segmentation.cpp Clustering.h Clustering.cpp ColorUtilities.h ColorUtilities.cpp ClusteringState.h ClusteringState.cpp Testing.h Testing.cpp)
target_link_libraries (supervoxel_segmentation ${PCL_LIBRARIES} ${OpenCV_LIBS}) 
