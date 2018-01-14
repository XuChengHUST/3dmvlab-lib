#ifndef FITING_H
#define FITING_H

#include "math/cloud_math.h"
#include "point_cloud.h"
#include "point_type.h"
#include "common_include.h"

namespace pc{

  //最小二乘拟合平面cloud,  输出parameters（nx,ny,nz,d）
  void least_squars_plane(const pc::PointCloud& cloud, std::vector<float>& parameters);
  //ransac拟合平面
  void ransac_plane(const pc::PointCloud& cloud, std::vector<float>& parameters, int num_iter, float distance_threshold);
}


#endif
