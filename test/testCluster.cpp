#include "io/io.h"
#include "cluster.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <string>

int main(int argc, char** argv) {
  if(argc != 4) {
    std::cout << "usage:remove_outliers" << '\n';
    return 1;
  }
  pc::PointCloud pts;
  pc::ReadASC_xyz(argv[1], pts);
  std::vector<std::vector<int> > all_point;
  std::vector<int> rest_point;
  int k_close = atof(argv[2]);
  float distance_threshold = atof(argv[3]);
  pc::cluster(pts, all_point, k_close, distance_threshold);

  // std::ofstream ofs(argv[4]);
  // std::cout << "all_point.size()=  " << all_point.size() <<std::endl;
  for(size_t i = 0; i != all_point.size(); ++i )
  {
    std::string name("../data/cluster/part");
    std::string num(std::to_string(i));
    std::string format(".asc");
    std::string filename = name + num + format;
    std::ofstream ofs(filename);
    for(size_t j = 0; j != all_point[i].size(); ++j)
    {
      ofs << pts.at(all_point[i][j]).x << " "
          << pts.at(all_point[i][j]).y << " "
          << pts.at(all_point[i][j]).z << '\n';
    }
    ofs.close();
  }


  return 0;
}
