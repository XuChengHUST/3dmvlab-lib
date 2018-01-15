set( EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin )

add_executable( testPointCloud testPointCloud.cpp )
target_link_libraries( testPointCloud
  io
)

add_executable( testKdtree testKdtree.cpp )
target_link_libraries( testKdtree
  kdtree io
)

add_executable( testOutlier testRemove.cpp )
target_link_libraries( testOutlier
  io filter
)

add_executable( testVoxelFilter testVoxelFilter.cpp )
target_link_libraries( testVoxelFilter
  io filter
)
