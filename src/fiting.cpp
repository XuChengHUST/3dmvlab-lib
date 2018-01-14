#include "fiting.h"


void fitting_plane_least(pc::PointCloud& cloud,std::vector<float>& parameters)
{
  clock_t time_start = clock();

  parameters.clear();
  Eigen::Matrix<float,3,1> p_;
  float p_x = 0;float p_y = 0;float p_z = 0;
  for(size_t i=0;i!=point_set.size();++i)
  {
    p_x += point_set.at(i).x;
    p_y += point_set.at(i).y;
    p_z += point_set.at(i).z;
  }
  p_x = p_x/point_set.size();p_y = p_y/point_set.size();p_z = p_z/point_set.size();
  p_<<p_x,p_y,p_z;

  Eigen::Matrix<float,3,1> p;   //p每个点
  Eigen::Matrix<float,1,3> p_trans;
  Eigen::Matrix<float,3,3> M;
  M << 0,0,0;
  for(size_t j = 0;j != point_set.size(); ++j)
  {

    p<<point_set.at(j).x, point_set.at(j).y, point_set.at(j).z;
    p-=p_;
    p_trans=p.transpose();
    M+=(p*p_trans);
  }

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float, 3, 3> > eigen_solver ( M );
  Eigen::Matrix<float, 3, 1> coe_set;
  coe_set<<eigen_solver.eigenvectors().col(0)[0],eigen_solver.eigenvectors().col(0)[1],eigen_solver.eigenvectors().col(0)[2];

  float d;
  Eigen::Matrix<float,3,1> p_cen_trans;
  d=p_cen_trans.transpose()*coe_set;
  parameters.push_back(coe_set[0]);parameters.push_back(coe_set[1]);parameters.push_back(coe_set[2]);parameters.push_back(d);
  std::cout <<"time use in fitting_plane_least is " <<1000*  (clock() - time_start)/(double)CLOCKS_PER_SEC <<"ms" << std::endl;
  std::cout<<coe_set[0]<<"  "<<coe_set[1]<<"  "<<coe_set[2]<<"  "<<d<<std::endl;
}
//ransac拟合平面
void fitting_plane_ransac(const std::vector<Point>& point_set, int num_iter, float threshold, std::vector<float>& parameters)
{

}
