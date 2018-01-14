add_library( kdtree STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/kdtree_flann.cpp
)
target_link_libraries( kdtree
  ${FLANN_LIBS}
)
