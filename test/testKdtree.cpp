#include "io/io.h"
#include "point_cloud.h"
#include "kdtree/kdtree_flann.h"

#include <iostream>
#include <vector>

int main(int argc, char** argv) {
  if(argc != 2) {
    std::cout << "usage: kdtree file." << '\n';
    return 1;
  }
  pc::PointCloud pts;
  pc::readACToPointCloud(argv[1], pts);
  std::cout << "pts size:" << pts.size() << '\n';
  pc::KdTreeFLANN kdtree;
  kdtree.setInputCloud(pts);
  std::vector<int> k_indices;
  std::vector<float> k_sqr_distances;
  kdtree.nearestKSearch(pts.at(10), 100, k_indices, k_sqr_distances);
  int index = 0;
  std::vector<float>::iterator dist_iter = k_sqr_distances.begin();
  std::vector<int>::iterator idx_iter = k_indices.begin();
  for(;dist_iter != k_sqr_distances.end(); ++dist_iter, ++idx_iter, ++index) {
    std::cout << "the " << index << " th index is: " << *idx_iter <<  " sqr_dist is: " << *dist_iter << '\n';
  }
}
