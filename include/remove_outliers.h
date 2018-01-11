#include "point_cloud.h"
#include "common_include.h"
#include "kdtree/kdtree_flann.h"

//输出去孤点后的点云序号序号
void remove_outliers(pc::PointCloud& cloud, std::vector<int>& out, float a, int k);
