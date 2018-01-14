set( EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin )

add_executable( testPointCloud testPointCloud.cpp )
target_link_libraries( testPointCloud
  io
)

add_executable( testKdtree testKdtree.cpp )
target_link_libraries( testKdtree
  kdtree io
)


add_executable( testRemove testRemove.cpp )
target_link_libraries( testRemove
  kdtree io filter math
  ${THIRD_PARTY_LIBS}
)

add_executable( testCluster testCluster.cpp )
target_link_libraries(testCluster
  kdtree io cluster
)

add_executable( testmath testmath.cpp )
target_link_libraries(testmath
  io math
)

add_executable( testFiting testFiting.cpp )
target_link_libraries(testFiting
  io fit math
)

add_executable( testVoxelfilter testVoxelfilter.cpp )
target_link_libraries(testVoxelfilter
  io voxelfilter
)
