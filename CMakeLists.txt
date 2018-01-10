cmake_minimum_required(VERSION 2.8)
project(3dmvlab_lib)

set( CMAKE_CXX_COMPILER "g++" )
set (CMAKE_BUILD_TYPE "Release")
set (CMAKE_CXX_FLAGS "-std=c++11 -O3")

# list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

# Eigen
include_directories( "/usr/include/eigen3/" )

# FLANN
find_package(PkgConfig REQUIRED)
pkg_check_modules(FLANN REQUIRED flann)
# link_directories(${FLANN_LIBRARY_DIRS})
include_directories(${FLANN_INCLUDE_DIRS})

set( THIRD_PARTY_LIBS
  ${FLANN_LIBRARY_DIRS}
)

############### source and test ######################
include_directories( ${PROJECT_SOURCE_DIR}/include )
add_subdirectory( src )
add_subdirectory( test )
