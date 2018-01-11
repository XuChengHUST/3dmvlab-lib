#include "kdtree_flann.h"

namespace pc
{
  void KdTreeFLANN::setInputCloud(const pc::PointCloud& cloud)
  {
    cloud_->reserve(3*cloud.size());
    for(pc::PointCloud::iterator iter = cloud.begin(); iter != cloud.end(); iter++)
    {
      cloud_.push_back(iter->x);
      cloud_.push_back(iter->y);
      cloud_.push_back(iter->z);
    }
    flann_index_.reset(
      new flann::Index<flann::L2_Simple<float>>(
        flann::Matrix<float>(cloud_->get(), cloud.size(), 3),
        flann::KDTreeSingleIndexParams (15)));
    flann_index_.buildIndex();
  }

  int KdTreeFLANN::nearestKSearch(const PointNormal& pt, int k,
                                  std::vector<int> &k_indices,
                                  std::vector<float> &k_sqr_distances) const {
    flann_index_->knnSearch();

  }
}
