#include "math/cloud_math.h"

namespace pc{
  // template <typename T>
  // float mean(const std::vector<T>& v_set)
  // {
  //   float sum = std::accumulate(v_set.begin(), v_set.end(), 0.0f);
  //   return (sum/static_cast<int>(v_set.size()));
  // }
  //
  // PointNormal mean(const PointCloud& pts)
  // {
  //   PointNormal point;
  //   for(size_t i = 0; i != pts.size(); ++i )
  //   {
  //     point.x += pts.at(i).x;
  //     point.y += pts.at(i).y;
  //     point.z += pts.at(i).z;
  //   }
  //   point.x /= pts.size();
  //   point.y /= pts.size();
  //   point.z /= pts.size();
  //   return point;
  // }
  //
  // template <typename T>
  // float standard_deviation(const std::vector<T>& v_set)
  // {
  //   float sqr_sum = std::inner_product(v_set.begin(), v_set.end(), v_set.begin(), 0.0f);
  //   float mean_value = mean(v_set);
  //   return std::sqrt(sqr_sum/static_cast<int>(v_set.size()) - mean_value * mean_value);
  // }

  float mean(const std::vector<float>& v)
  {
    float sum = std::accumulate(v.begin(), v.end(), 0.0f);
    return (sum/static_cast<int>(v.size()));
  }
  float standard_deviation(const std::vector<float>& v)
  {
    float sqr_sum = std::inner_product(v.begin(), v.end(), v.begin(), 0.0f);
    float mean_value = mean(v);
    return std::sqrt(sqr_sum/static_cast<int>(v.size()) - mean_value * mean_value);
  }

}
