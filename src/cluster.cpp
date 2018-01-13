#include "cluster.h"
#include "kdtree/kdtree_flann.h"
#include <list>
#include <vector>
#include <queue>
#include <ctime>
#include <algorithm>

namespace pc
{
  void cluster(pc::PointCloud& cloud, std::vector<std::vector<int> >& indices, int k_close, float distance_threshold)
  {
    clock_t time_start = clock();
    indices.clear();
    std::vector<int> checked_queue;
    std::vector<bool> processed(cloud.size(),false);  //点是否处理

    pc::KdTreeFLANN kdtree;
    kdtree.setInputCloud(cloud);
    std::vector<int> k_indices;
    std::vector<float> k_sqr_distances;

    std::cout << "the number of point in cloud =  "<< cloud.size() <<std::endl;
    for(size_t i = 0; i != cloud.size(); ++i )
    {
      if(processed[i])
        continue;
      checked_queue.push_back(i);   //第ｉ个点
      processed[i] = true;
      int checked_queue_idx = 0;
      while(checked_queue_idx < checked_queue.size())
      {
        kdtree.nearestKSearch(cloud.at(checked_queue.at(checked_queue_idx) ), k_close, k_indices, k_sqr_distances);
        std::vector<float>::iterator dist_iter = k_sqr_distances.begin();
        std::vector<int>::iterator idx_iter = k_indices.begin();
        for(; dist_iter != k_sqr_distances.end(); ++dist_iter, ++idx_iter)
        {
          if(processed[*idx_iter])
            continue;
          if( sqrt( *dist_iter ) < distance_threshold )
          {
            checked_queue.push_back( *idx_iter );
            processed[*idx_iter] = true;
          }
        }
        checked_queue_idx++;
      }
      indices.push_back(checked_queue);
      checked_queue.clear();
      k_indices.clear();
      k_sqr_distances.clear();
    }
    std::cout << "time in cluster = " << 1000* (clock() - time_start)/(double)CLOCKS_PER_SEC << "ms" <<std::endl;
    return;
  }
}
