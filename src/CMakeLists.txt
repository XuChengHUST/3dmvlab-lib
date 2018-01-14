add_library( io STATIC
  ac_type.cpp
  io.cpp
  stream.cpp
)

add_library( kdtree STATIC
  kdtree_flann.cpp
)
target_link_libraries( kdtree
  ${THIRD_PARTY_LIBS}
)


add_library( filter STATIC
    remove_outliers.cpp
    kdtree_flann.cpp
)
# target_link_libraries( filter
#   ${THIRD_PARTY_LIBS}
# )

add_library( cluster STATIC
  kdtree_flann.cpp
  cluster.cpp
)

add_library( math STATIC
  cloud_math.cpp
)
