#ifndef REMOVE_OUTLIERS_H
#define REMOVE_OUTLIERS_H

#include "point_cloud.h"
#include "common_include.h"
#include "math/cloud_math.h"
#include "kdtree/kdtree_flann.h"

namespace pc {
  void remove_outliers(pc::PointCloud& cloud, std::vector<int>& indices,
                       int k, float factor);
}

#endif
