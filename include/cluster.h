#ifndef CLUSTER_H
#define CLUSTER_H

#include "kdtree/kdtree_flann.h"
#include <list>
#include <vector>
#include <queue>
#include <algorithm>
<<<<<<< HEAD
#include <vector>
=======
#include <string>
>>>>>>> 739014921de787cca733afe567fab21d9a910460

namespace pc{
  void cluster(pc::PointCloud& cloud, std::vector<std::vector<int> >& cluster_indices, int k_close, float distance_threshold);
}

#endif
