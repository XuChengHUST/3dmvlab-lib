#include "io/io.h"
#include "remove_outliers.h"

#include <iostream>
#include <fstream>
#include <vector>

int main(int argc, char** argv) {
  if(argc != 5) {
    std::cout << "usage:remove_outliers" << '\n';
    return 1;
  }
  pc::PointCloud pts;
  // pc::readACToPointCloud(argv[1], pts);
  // void readACToPointCloud(const std::string& filename, pc::PointCloud& pc);
  // void ReadASC_xyz(const std::string& filename, pc::PointCloud& pc);
  pc::ReadASC_xyz(argv[1],pts);
  std::vector<int> rest_point;
  float a = atof(argv[2]);
  int k = atoi(argv[3]);
  pc::remove_outliers(pts,rest_point,a,k);
  std::ofstream ofs(argv[4]);
  for(size_t i = 0; i != rest_point.size(); ++i ) {
    ofs << pts.at(rest_point[i]).x << " "
        << pts.at(rest_point[i]).y << " "
        << pts.at(rest_point[i]).z << '\n';
  }
  ofs.close();
  return 0;
}
