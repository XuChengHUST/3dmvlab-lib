#include "io/io.h"
#include "filter/voxel_filter.h"

int main(int argc, char** argv)
{
  if(argc != 5)
  {
    std::cout << "usage: testVoxelfilter" << '\n';
    return 1;
  }
  pc::PointCloud pts;
  pc::readASCToPointCloud(argv[1], pts);
  pc::PointCloud ptsout;
  std::vector<int> rest_point;
  float voxel_size = atof(argv[2]);
  bool if_normal = false;
  voxelfilter(pts, ptsout, voxel_size,if_normal);
  voxelfilter(pts, rest_point, voxel_size);
  std::ofstream ofs_1(argv[3]);
  for(int i = 0; i != rest_point.size(); ++i)
  {
    ofs_1 << pts[rest_point.at(i)].x << " "
        << pts[rest_point.at(i)].y << " "
        << pts[rest_point.at(i)].z << '\n';
  }
  ofs_1.close();
  std::ofstream ofs_2(argv[4]);
  for(int i = 0; i != rest_point.size(); ++i)
  {
    ofs_2 << ptsout.at(i).x << " "
        << ptsout.at(i).y << " "
        << ptsout.at(i).z << '\n';
  }
  ofs_2.close();
  return 0;
}
