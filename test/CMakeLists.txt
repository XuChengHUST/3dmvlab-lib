add_executable( testPointCloud testPointCloud.cpp )
target_link_libraries( testPointCloud
  io
)

add_executable( testKdtree testKdtree.cpp )
target_link_libraries( testKdtree
  ${THIRD_PARTY_LIBS}
  kdtree io
)


add_executable( testRemove testRemove.cpp )
target_link_libraries( testRemove
  kdtree io filter
  ${THIRD_PARTY_LIBS}
)

add_executable( testCluster testCluster.cpp )
target_link_libraries(testCluster
  kdtree io cluster
)
