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
