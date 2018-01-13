<<<<<<< HEAD
#ifndef REMOVEOUTLIERS_H
#define REMOVEOUTLIERS_H
=======
#ifndef REMOVE_OUTLIERS_H
#define REMOVE_OUTLIERS_H

>>>>>>> 43fc53aa90eeca6bd5fd6fbbb6f45596c5ea765f
#include "point_cloud.h"
#include "common_include.h"
#include "kdtree/kdtree_flann.h"

namespace pc {
  void remove_outliers(pc::PointCloud& cloud, std::vector<int>& out,
                       int k , float factor );
}

#endif
