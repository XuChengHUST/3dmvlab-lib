#include "io/io.h"

namespace pc {
  void readACToPointCloud(const std::string& filename, pc::PointCloud& pc) {
    ACFileType ac(filename);
    if(ac.numVerts_ == -1) {
      std::cerr << filename << " file open failed." << std::endl;
      return;
    }
    pc.reserve(ac.numVerts_);
    for(int i = 0; i < ac.numVerts_; ++i) {
      pc.push_back(pc::PointNormal(ac.pVertex_[3*i],
                                   ac.pVertex_[3*i+1],
                                   ac.pVertex_[3*i+2]));
    }
  }
  // void writePCFileAscii(const std::string& filename, pc::PointCloud& pc)
  // {
  //   std::ofstream ofs;
  //   ofs.open(filename);
  //   if(!ofs.is_open())
  //   {
  //     std::cout << filename << " open failed." << '\n';
  //     return;
  //   }
  //   else
  //   {
  //     for(int i = 0; i != pts.size(); ++i)
  //       ofs << pc.at(i).x << " "
  //           << pc.at(i).y << " "
  //           << pc.at(i).z << '\n';
  //   }
  //   ofs.close();
  // }
}
