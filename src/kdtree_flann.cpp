#include "kdtree/kdtree_flann.h"

namespace pc {
  KdTreeFLANN::KdTreeFLANN() :
    flann_index_(), array_(), dim_(3), epsilon_(0.0f), sorted_(true),
    param_k_(::flann::SearchParams(-1, epsilon_)),
    param_radius_ (::flann::SearchParams(-1, epsilon_, sorted_)) {
    }

  void KdTreeFLANN::setInputCloud(const pc::PointCloud& cloud) {
    convertCloudToArray(cloud);
    flann_index_.reset(new FLANNIndex(::flann::Matrix<float>(&array_[0], cloud.size(), dim_),
                                      ::flann::KDTreeSingleIndexParams (15)));
    flann_index_->buildIndex();
  }

  int KdTreeFLANN::nearestKSearch(const PointNormal& pt, int k,
                                  std::vector<int> &k_indices,
                                  std::vector<float> &k_sqr_distances) const {
    k_indices.resize(k);
    k_sqr_distances.resize(k);

    std::vector<float> query{pt.x, pt.y, pt.z};

    ::flann::Matrix<int> k_indices_mat (&k_indices[0], 1, k);
    ::flann::Matrix<float> k_distances_mat (&k_sqr_distances[0], 1, k);

    flann_index_->knnSearch (::flann::Matrix<float>(&query[0], 1, dim_),
                             k_indices_mat, k_distances_mat,
                             k, param_k_);
  }

  void KdTreeFLANN::convertCloudToArray(const PointCloud& cloud) {
    if(cloud.empty())
      return;
    array_.reserve(dim_ * static_cast<int>(cloud.size()));
    for(pc::PointCloud::const_iterator iter = cloud.begin(); iter != cloud.end(); ++iter) {
      array_.push_back(iter->x);
      array_.push_back(iter->y);
      array_.push_back(iter->z);
    }
  }
}
