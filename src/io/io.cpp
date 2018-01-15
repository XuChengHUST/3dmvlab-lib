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
  void readASCToPointCloud(const std::string& filename, pc::PointCloud& pc) {
    std::ifstream inFile;
    std::ifstream fin;
    int txt_size = 0;
    double x, y, z;
    inFile.open(filename.c_str());
    while(!inFile.eof())
    {
      inFile >> x >> y >> z;
      txt_size++;
    }
    inFile.close();

    fin.open(filename.c_str());
    for(size_t i=0;i != txt_size;++i)
    {
      fin >> x >> y >> z;
      pc.push_back(pc::PointNormal(x,y,z) );
    }
    fin.close();
  }
}
