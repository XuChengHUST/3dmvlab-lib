#include "fiting.h"
#include "io/io.h"
int main(int argc, char** argv)
{
  if(argc != 4 )
  {
    std::cout << "Usage: testfiting" << std::endl;
    return 1;
  }

  pc::PointCloud pts;
  pc::ReadASC_xyz(argv[1], pts);
  int num_iter = atoi(argv[2]);
  float distance_threshold = atof(argv[3]);
  std::vector<float> parameters_1;
  std::vector<float> parameters_2;
  least_squars_plane(pts,parameters_1);
  ransac_plane(pts, parameters_2, num_iter, distance_threshold);
  return 0;
}
