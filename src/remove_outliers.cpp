#include "filter/remove_outliers.h"

namespace pc {
  void remove_outliers(pc::PointCloud& cloud, std::vector<int>& indices,
                       int k = 10, float factor = 1.0f) {
    indices.clear();
    pc::KdTreeFLANN kdtree;
    kdtree.setInputCloud(cloud);

    //计算每个点到最近K个点的平均距离
    std::vector<float> meandists_point_knn;
    for(size_t i = 0; i != cloud.size(); ++i ) {
      std::vector<int> k_indices;
      std::vector<float> k_sqr_distances;
      kdtree.nearestKSearch(cloud.at(i), k, k_indices, k_sqr_distances);

      int knn_count = 0;
      float sumdist_point_knn = 0.0f;
      std::vector<float>::iterator dist_iter = k_sqr_distances.begin();
      for(std::vector<float>::iterator dist_iter = k_sqr_distances.begin();
          dist_iter != k_sqr_distances.end();
          ++dist_iter, ++knn_count)
        sumdist_point_knn += sqrt( *dist_iter );
      meandists_point_knn.push_back(sumdist_point_knn/knn_count);
    }

    float meandist_cloud_knn = mean(meandists_point_knn);
    float stdevdist_cloud_knn = standard_deviation(meandists_point_knn);
    float min_dist_threshold = meandist_cloud_knn - factor * stdevdist_cloud_knn;
    float max_dist_threshold = meandist_cloud_knn + factor * stdevdist_cloud_knn;

    for(size_t i = 0; i != meandists_point_knn.size(); ++i) {
      if(meandists_point_knn.at(k) >= min_dist_threshold &&
         meandists_point_knn.at(k) <= max_dist_threshold)
        indices.push_back(i);
    }
  }
}
