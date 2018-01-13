#ifndef REMOVE_OUTLIERS_H
#define REMOVE_OUTLIERS_H

#include "point_cloud.h"
#include "common_include.h"
#include "kdtree/kdtree_flann.h"

namespace pc {
  void remove_outliers(pc::PointCloud& cloud, std::vector<int>& out,
                       int k = 10, float factor = 1.0f);
}

#endif
