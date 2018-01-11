add_executable( testPointCloud testPointCloud.cpp )
target_link_libraries( testPointCloud
  io
)

add_executable( testKdtree testKdtree.cpp )
target_link_libraries( testKdtree
  ${THIRD_PARTY_LIBS}
  kdtree io
)


add_executable( testremove testremove.cpp )
target_link_libraries( testremove
  kdtree io filter
  ${THIRD_PARTY_LIBS}
)
