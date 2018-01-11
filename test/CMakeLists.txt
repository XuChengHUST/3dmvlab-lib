add_executable( testPointCloud testPointCloud.cpp )
target_link_libraries( testPointCloud
  io
)

add_executable( testKdtree testKdtree.cpp )
target_link_libraries( testKdtree
  ${THIRD_PARTY_LIBS}
  kdtree io
)
