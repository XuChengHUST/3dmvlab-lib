add_library( filter STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/remove_outlier.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/voxel_filter.cpp
  ${PROJECT_SOURCE_DIR}/src/cloud_math.cpp
)
target_link_libraries( filter
  kdtree
)
