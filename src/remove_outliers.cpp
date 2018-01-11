#include "remove_outliers.h"

void remove_outliers(pc::PointCloud& cloud, std::vector<int>& out, float a, int k)
{
  out.clear();
  pc::KdTreeFLANN kdtree;
  kdtree.setInputCloud(cloud);
  std::vector<float> d_set;     //全部点k领域平均距离

  for(size_t i = 0; i != cloud.size(); ++i )
  {
    std::vector<int> k_indices;
    std::vector<float> k_sqr_distances;
    kdtree.nearestKSearch(cloud.at(i), k, k_indices, k_sqr_distances);

    int index = 0;
    float distance=0;
    std::vector<float>::iterator dist_iter = k_sqr_distances.begin();
    std::vector<int>::iterator idx_iter = k_indices.begin();
    for(; dist_iter != k_sqr_distances.end(); ++dist_iter, ++idx_iter, ++index)
    {
      distance += sqrt( *dist_iter );
    }
    distance = distance/index;
    d_set.push_back(distance);
  }

  float sum_d = 0;
  float sum_d_squ = 0;
  int N = d_set.size();
  for(auto it = d_set.begin(); it != d_set.end(); ++it)
  {
    sum_d += (*it);
    sum_d_squ += pow(*it, 2);
  }
  float mean = sum_d/N;                       //平均值
  float sd = sqrt(sum_d_squ/N - pow(mean,2)); //标准差

  float min = mean - a*sd;
  float max = mean + a*sd;
  for(size_t k = 0;k != N; ++k)
  {
    if(d_set.at(k) > min && d_set.at(k) < max)
      out.push_back(k);
  }
  return;
}
