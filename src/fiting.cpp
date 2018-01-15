#include "fiting.h"


namespace pc{

  void least_squars_plane(const PointCloud& cloud, std::vector<float>& parameters)
  {
    clock_t time_start = clock();
    parameters.clear();
    //方程形式 ax + by + cz = d d用coeff_d表示

    PointNormal centroid = mean(cloud); //质心

    Eigen::Matrix<float,3,1> centroid_matrix;
    centroid_matrix << centroid.x, centroid.y, centroid.z;

    Eigen::Matrix<float,3,1> each_point;
    Eigen::Matrix<float,1,3> each_point_transpose;
    Eigen::Matrix<float,3,3> to_solve_matrix;
    to_solve_matrix << 0, 0, 0;
    for(size_t i = 0; i != cloud.size(); ++i)
    {
      each_point << cloud.at(i).x, cloud.at(i).y, cloud.at(i).z;
      each_point -= centroid_matrix;
      each_point_transpose = each_point.transpose();
      to_solve_matrix += (each_point * each_point_transpose);
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float, 3, 3> > eigen_solver ( to_solve_matrix );
    Eigen::Matrix<float, 3, 1> coefficient_set;
    coefficient_set << eigen_solver.eigenvectors().col(0)[0],
                       eigen_solver.eigenvectors().col(0)[1],
                       eigen_solver.eigenvectors().col(0)[2];

    float coeff_d;
    Eigen::Matrix<float,3,1> p_centroid_transpose;
    coeff_d = centroid_matrix.transpose() * coefficient_set;
    parameters.push_back(coefficient_set[0]);
    parameters.push_back(coefficient_set[1]);
    parameters.push_back(coefficient_set[2]);
    parameters.push_back(coeff_d);

    std::cout <<"time use in least_squars_plane is " <<1000*  (clock() - time_start)/(double)CLOCKS_PER_SEC <<"ms" << std::endl;
    std::cout << coefficient_set[0] << "  "
              << coefficient_set[1] << "  "
              << coefficient_set[2] << "  "
              << coeff_d << '\n';
    std::cout << std::endl;
    return;
  }

  void ransac_plane(const PointCloud& cloud, std::vector<float>& parameters, int num_iter, float distance_threshold)
  {
    clock_t time_stt = clock();
    parameters.clear();
    //要取得[a,b]的随机整数，使用(rand() % (b-a+1))+ a;
    std::vector<PointCloud> all_inliers_set;
    std::vector<int> num_inliers_set;
    for(size_t i = 0; i != num_iter; ++i)
    {
      //三点用 最小二乘法拟合 或 直接计算
      // int order_a = rand() % cloud.size();
      // int order_b = rand() % cloud.size();
      // int order_c = rand() % cloud.size();
      // PointCloud pts;
      // pts.push_back(cloud.at(order_a));
      // pts.push_back(cloud.at(order_b));
      // pts.push_back(cloud.at(order_c));
      // std::vector<float> random;
      // least_squars_plane(pts,random);
      // Eigen::Matrix<float,3,1> plane_normal;
      // plane_normal << random[0], random[1], random[2];
      // float coeff_d = random[3];

      int order_a = rand() % cloud.size();
      int order_b = rand() % cloud.size();
      int order_c = rand() % cloud.size();
      Eigen::Matrix<float,3,1>  point_a, point_b, point_c;
      point_a << cloud.at(order_a).x, cloud.at(order_a).y, cloud.at(order_a).z;
      point_b << cloud.at(order_b).x, cloud.at(order_b).y, cloud.at(order_b).z;
      point_c << cloud.at(order_c).x, cloud.at(order_c).y, cloud.at(order_c).z;
      Eigen::Matrix<float,3,1> plane_normal;
      plane_normal = (point_a - point_b).cross(point_a - point_c);    //叉乘
      float coeff_d = plane_normal[0] * point_a[0] + plane_normal[1] * point_a[1] + plane_normal[2] * point_a[2];

      //点到平面的距离
      //ax + by +ca = d
      //方法一. 距离公式 distance = ( a * (x - x_0) + b * (y - y_0) + b * (z - z_0) - d ) /  sqrt (a^2 +  b^2 + c^2)
      //方法二. Hesse normal form   r.dot(n) - d 其中 r = (x,y,z) n = (a,b,c)
      PointCloud each_inliers_set; //每次迭代的内点
      for(size_t j = 0; j != cloud.size(); ++j)
      {
        Eigen::Matrix<float, 3, 1> checked_point;
        checked_point << cloud.at(j).x, cloud.at(j).x, cloud.at(j).z;
        float distance = plane_normal.dot(checked_point) - coeff_d;
        if(distance <= distance_threshold)
          each_inliers_set.push_back(cloud.at(j));
      }
      all_inliers_set.push_back(each_inliers_set);

      int each_num_inliers = each_inliers_set.size();  //每次迭代的内点个数
      num_inliers_set.push_back(each_num_inliers);
      each_inliers_set.clear();
    }

    std::vector<int>::iterator result;
    result = std::max_element(num_inliers_set.begin(), num_inliers_set.end() );
    int order = std::distance(num_inliers_set.begin(), result);
    PointCloud  p_set = all_inliers_set.at(order);

    std::cout << "最大内点数" << p_set.size() << std::endl;
    std::cout << "time use in ransac compute is " << 1000*  (clock() - time_stt)/(double)CLOCKS_PER_SEC << "ms" << std::endl;
    least_squars_plane(p_set, parameters);
    return;
  }
}
