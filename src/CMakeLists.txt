# CMakeList.txt : CMake project for EdgeDetection_Bazazian, include source and define
# project specific logic here.
#

# Following line is required to perform compatibility checks
# Make sure that the version is at least 3.5, as required by PCL
cmake_minimum_required(VERSION 3.5)
project(pcl_cmake_minimum)

# Use the following to link against all the PCL libraries
# Remove REQUIRED if PCL is not compulsory
# Note: might be heavy, not the best practice
# find_package(PCL REQUIRED)
# Prefer to use the correct component libraries instead
find_package(PCL COMPONENTS common REQUIRED)

# All header directories not under your control go here
# ${PCL_INCLUDE_DIRS} not required to be manually added since PCL 1.9
# include_directories(SYSTEM)
# Headers for this project go here
# include_directories(include)

# Add source to this project's executable.
add_executable (edge "main.cpp" "main.h")

# Add the required libraries
target_link_libraries(edge ${PCL_LIBRARIES})
# Add the dependencies, if any
# add_dependencies(edge some_other_target)

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET edge PROPERTY CXX_STANDARD 20)
endif()

# TODO: Add tests and install targets if needed.
