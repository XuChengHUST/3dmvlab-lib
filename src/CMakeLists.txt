set( LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib )

add_library( common STATIC
  point_cloud.cpp
)

add_library( io STATIC
  ac_type.cpp
  io.cpp
  stream.cpp
)

add_library( kdtree STATIC
  kdtree_flann.cpp
)
target_link_libraries( kdtree
  ${FLANN_LIBS}
)

add_library( filter STATIC
  remove_outliers.cpp
  cloud_math.cpp
)
target_link_libraries( filter
  kdtree
)


add_library( registration STATIC
  svdRT.cpp
)
