#include "point_cloud.h"
#include "io/io.h"

int main(int argc, char** argv)
{
  // Eigen::MatrixX3f matrix;
  // matrix.resize(10,3);
  // pc::PointCloud pc(matrix);
  // std::cout << "pts size:" << pc.size() << '\n'
  //           << pc;

  pc::PointCloud pts;
  // pc::readACToPointCloud(argv[1], pts);
  pc::ReadASC_xyz(argv[1], pts);
  std::cout << "pts size:" << pts.size() << '\n'
            << pts;
}
