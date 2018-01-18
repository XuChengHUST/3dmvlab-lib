#include "point_cloud.h"

int main(int argc, char** argv) {
  Eigen::MatrixX3f matrix;
  matrix.resize(10,3);
  pc::PointCloud pc(matrix);
  std::cout << "pts size:" << pc.size() << '\n'
            << pc;
}
