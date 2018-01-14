#ifndef KDTREE_FLANN_H
#define KDTREE_FLANN_H

#include <flann/flann.h>
#include "point_cloud.h"

namespace pc {
  class KdTreeFLANN {
  public:
    typedef ::flann::L2_Simple<float> Dist;
    typedef ::flann::Index<Dist> FLANNIndex;

    KdTreeFLANN();

    void setInputCloud(const pc::PointCloud& cloud);

    void convertCloudToArray(const pc::PointCloud& cloud);

    void nearestKSearch(const PointNormal& pt, int k,
                       std::vector<int> &k_indices,
                       std::vector<float> &k_sqr_distances) const;
  private:
    std::shared_ptr<FLANNIndex> flann_index_;

    std::vector<float> array_;

    int dim_;

    float epsilon_;

    bool sorted_;

    ::flann::SearchParams param_k_;

    ::flann::SearchParams param_radius_;
  };
}

#endif
