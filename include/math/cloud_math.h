#ifndef CLOUD_MATH_H
#define CLOUD_MATH_H

#include <vector>
#include <numeric>
#include <cmath>
#include "point_type.h"
#include "point_cloud.h"

namespace pc {

  // template <typename T>
  // float mean(const std::vector<T>& v_set);

  float mean(const std::vector<float>& v);
  float standard_deviation(const std::vector<float>& v);
  PointNormal mean(const PointCloud& pts);

  // template <typename T>
  // float standard_deviation(const std::vector<T>& v_set);
  //
  // float mean(const std::vector<float>& v)
  // {
  //   float sum = std::accumulate(v.begin(), v.end(), 0.0f);
  //   return (sum/static_cast<int>(v.size()));
  // }
  // float standard_deviation(const std::vector<float>& v)
  // {
  //   float sqr_sum = std::inner_product(v.begin(), v.end(), v.begin(), 0.0f);
  //   float mean_value = mean(v);
  //   return std::sqrt(sqr_sum/static_cast<int>(v.size()) - mean_value * mean_value);
  // }
}

#endif
