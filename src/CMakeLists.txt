set( LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib )
add_subdirectory( ${CMAKE_CURRENT_SOURCE_DIR}/io )
add_subdirectory( ${CMAKE_CURRENT_SOURCE_DIR}/kdtree )
add_subdirectory( ${CMAKE_CURRENT_SOURCE_DIR}/filter )
add_subdirectory( ${CMAKE_CURRENT_SOURCE_DIR}/registration )

add_library( common STATIC
  point_cloud.cpp
)
