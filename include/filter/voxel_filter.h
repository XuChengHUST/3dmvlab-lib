#ifndef VOXEL_FILTER_H
#define VOXEL_FILTER_H

#include "math/cloud_math.h"
#include "point_cloud.h"
#include "point_type.h"
#include "common_include.h"
#include <cfloat>

namespace pc{

  struct cloud_point_index_idx{
    unsigned int idx;
    unsigned int cloud_point_index;

    cloud_point_index_idx (unsigned int idx_, unsigned int cloud_point_index_) : idx (idx_), cloud_point_index (cloud_point_index_) {}
    bool operator < (const cloud_point_index_idx &p) const { return (idx < p.idx); }
  };

  //栅格质心标准化
  void voxelfilter(PointCloud& cloud, PointCloud& centroid_set, float voxel_size, bool if_normal);
  //栅格中任取一点
  void voxelfilter(PointCloud& cloud, std::vector<int>& indices, float voxel_size);

}

#endif
