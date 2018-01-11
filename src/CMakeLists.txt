<<<<<<< HEAD
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
=======

>>>>>>> 756abd7752b21a06312a4b716e561f463f747b43
