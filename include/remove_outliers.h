#include "point_cloud.h"
#include "common_include.h"
#include "kdtree/kdtree_flann.h"

//输出去孤点后的点云序号序号
namespace pc {
  void remove_outliers(pc::PointCloud& cloud, std::vector<int>& out,
                       int k = 10, float factor = 1.0f);
}
