# CMakeList.txt : CMake project for EdgeDetection_Bazazian, include source and define
# project specific logic here.
#

# Following line is required to perform compatibility checks
cmake_minimum_required(VERSION 3.5)
project(edgedetection_bazazian)

find_package(PCL 1.2 REQUIRED)
message("PCL_LIBRARIES - ${PCL_LIBRARIES}")

# All header directories not under your control go here
# include_directories(SYSTEM)
# Headers for this project go here
# include_directories(include)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${CMAKE_SOURCE_DIR}/inc)
link_directories(${CMAKE_SOURCE_DIR}/lib)

# Add source to this project's executable.
add_executable (edge "main.cpp" "main.h")

# Add the required libraries
target_link_libraries(edge ${PCL_LIBRARIES})
target_link_libraries(edge E57Format-d)
target_link_libraries(edge xerces-c_3D)

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET edge PROPERTY CXX_STANDARD 20)
endif()