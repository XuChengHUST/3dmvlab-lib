#include "math/cloud_math.h"
#include "io/io.h"
#include "point_type.h"
#include "point_cloud.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <string>

int main(int argc ,char** argv)
{
  if(argc != 1)
  {
    std::cout << " Usage: testmath " << std::endl;
  }
  pc::PointCloud pts;
  pc::ReadASC_xyz(argv[1], pts);
  pc::pointNormal point = mean_point(pts);
  std::cout << point.x << " "
            << point.y << " "
            << poiny.z << '\n';
  return 0;
}
