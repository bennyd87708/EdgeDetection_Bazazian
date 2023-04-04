# CMakeList.txt : Top-level CMake project file, do global configuration
# and include sub-projects here.
#
cmake_minimum_required (VERSION 3.8)

project ("EdgeDetection_Bazazian")
set(Boost_NO_WARN_NEW_VERSIONS 1)

# Include sub-projects.
add_subdirectory ("src")
