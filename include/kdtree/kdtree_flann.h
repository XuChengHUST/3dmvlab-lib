#ifndef KDTREE_KDTREE_FLANN_H
#define KDTREE_KDTREE_FLANN_H

#include <flann/flann.h>
#include "point_cloud.h"

namespace pc {
  class KdTreeFLANN {
  public:
    void setInputCloud(const pc::PointCloud& cloud);

    int nearestKSearch(const PointNormal& pt, int k,
                       std::vector<int> &k_indices,
                       std::vector<float> &k_sqr_distances) const;
  private:
    flann::Index<flann::L2_Simple<float>> flann_index_;

    std::shared_ptr<std::vector<float>> cloud_;

    flann::SearchParams param_k_;
  };
}
