#include "point_cloud.h"

namespace pc {
  bool mapCloudToMatrix(const PointCloud& pts, Eigen::MatrixX3f& matrix) {
    if(!pts.empty()) {
      std::cout << "mapCloudToMatrix: point cloud is empty." << std::endl;
      return false;
    }
    matrix.resize(pts.size(), 3);
    for(int i = 0; i < matrix.rows(); ++i)
      matrix.block<1,3>(i,0) << pts.at(i).x, pts.at(i).y, pts.at(i).z;
  }
}
