#ifndef CLOUD_MATH_H
#define CLOUD_MATH_H

#include <vector>
#include <numeric>
#include <cmath>
#include "point_type.h"
#include "point_cloud.h"

namespace pc {
  PointNormal mean(const PointCloud& pts);

  float mean(const std::vector<float>& v);

  float standard_deviation(const std::vector<float>& v);
}

#endif
