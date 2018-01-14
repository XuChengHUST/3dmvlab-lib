#include "svdRT.h"

namespace pc {
  Eigen::Matrix4f svdRT(const Eigen::MatrixX3f& P, const Eigen::MatrixX3f& Q) {
    if(P.rows() < 3 || Q.rows() < 3) {
      std::cout << "svdRT error: point number is smaller than 3." << std::endl;
      return Eigen::Matrix4f::Identity();
    }
    Eigen::Vector3f centra_P = P.colwise().mean();
    Eigen::MatrixX3f centralized_P = P - centra_P.replicate(P.rows(), 1);

    Eigen::Vector3f centra_Q = Q.colwise().mean();
    Eigen::MatrixX3f centralized_Q = Q - centra_Q.replicate(Q.rows(), 1);

    Eigen::Matrix3f W = Centralized_Q.transpose() * Centralized_P;
    Eigen::FullPivLU<Eigen::Matrix3f> lu_decomp(W);
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(W, Eigen::ComputeThinU | Eigen::ComputeThinV);

    Eigen::Matrix3f S = Eigen::Matrix3f::Identity();
    S(2,2) = svd.matrixU().determinant() * svd.matrixV().determinant();
    Eigen::Matrix3f R = svd.matrixU() * S * svd.matrixV().transpose();
    Eigen::Vector3d t = Centra_Q - R * Centra_P;

    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block<3,3>(0,0) = R;
    T.block<3,1>(0,3) = t;

    return T;
  }

  Eigen::Matrix4f svdRT(const PointCloud& P, const PointCloud& Q) {
    Eigen::MatrixX3f matrix_P, matrix_Q;
    mapCloudToMatrix(P, matrix_P);
    mapCloudToMatrix(Q, matrix_Q);
    return svdRT(matrix_P, matrix_Q);
  }

}
