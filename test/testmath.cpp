#include "math/cloud_math.h"
#include "io/io.h"
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
  pc::PointNormal point = mean(pts);
  std::cout << point.x << " "
            << point.y << " "
            << point.z << '\n';
  return 0;
}
