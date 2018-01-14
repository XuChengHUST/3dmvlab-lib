#ifndef SVDRT_H
#define SVDRT_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>

#include "point_cloud.h"

namespace pc {
  Eigen::Matrix4f svdRT(const Eigen::MatrixX3f& P, const Eigen::MatrixX3f& Q);

  Eigen::Matrix4f svdRT(const PointCloud& P, const PointCloud& Q);
}

#endif
