cmake_minimum_required(VERSION 2.8.3)
project(supervoxel_clustering)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Option to compile with or withour ROS compatibility (through Catkin)
## To use this package without ROS support call cmake with the option: -DUSE_CATKIN=OFF
## Default is ON (i.e., ROS support enabled)
option(USE_CATKIN "Use Catkin to enable ROS support" ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
if(USE_CATKIN)
  find_package(catkin REQUIRED COMPONENTS
    roscpp
  )
endif(USE_CATKIN)

## System dependencies are found with CMake's conventions
find_package(PCL 1.8 REQUIRED)
find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
if(USE_CATKIN)
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES clustering color_utilities clustering_state testing
    CATKIN_DEPENDS roscpp
    DEPENDS PCL OpenCV
  )
endif(USE_CATKIN)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Declare a C++ library
add_library(clustering src/clustering.cpp)
add_library(color_utilities src/color_utilities.cpp)
add_library(clustering_state src/clustering_state.cpp)
add_library(testing src/testing.cpp)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(supervoxel_clustering src/supervoxel_clustering.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(supervoxel_clustering
  clustering
  color_utilities
  clustering_state
  testing
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

