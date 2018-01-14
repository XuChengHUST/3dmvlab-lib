#ifndef CLOUD_MATH_H
#define CLOUD_MATH_H

#include <vector>
#include <numeric>
#include <cmath>

namespace pc {
  float mean(const std::vector<float>& v);

  float standard_deviation(const std::vector<float>& v);
}

#endif
