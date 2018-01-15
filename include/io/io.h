#ifndef IO_H
#define IO_H

#include "common_include.h"
#include "io/ac_type.h"
#include "point_cloud.h"
#include <fstream>

namespace pc {
  void readACToPointCloud(const std::string& filename, pc::PointCloud& pc);
  void readASCToPointCloud(const std::string& filename, pc::PointCloud& pc);
  // void writePCFileAscii(const std::string& filename, std::vector<int>& point);
}

#endif
