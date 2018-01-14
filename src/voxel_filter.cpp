#include "filter/voxel_filter.h"

namespace pc{

  void voxelfilter(PointCloud& cloud, PointCloud& centroid_set, float voxel_size, bool if_normal)
  {
    // indices.clear();
    std::cout<<"采样前的点 =  "<< cloud.size() <<std::endl;
    clock_t time_start = clock();
    //获取最大/小点
    Eigen::Array4f min_p, max_p;
    min_p.setConstant (FLT_MAX);
    max_p.setConstant (-FLT_MAX);
    Eigen::Array4f pt = Eigen::Array4f::Zero ();
    for (size_t cp = 0; cp < cloud.size(); ++cp)
    {
      // memcpy (&pt[0], &point_set.at(cp).x, sizeof (float));
      // memcpy (&pt[1], &point_set.at(cp).y, sizeof (float));
      // memcpy (&pt[2], &point_set.at(cp).z, sizeof (float));
      pt[0] = cloud.at(cp).x;
      pt[1] = cloud.at(cp).y;
      pt[2] = cloud.at(cp).z;
      min_p = (min_p.min) (pt);
      max_p = (max_p.max) (pt);
    }

    //设置栅格的大小
    Eigen::Array4f inverse_leaf_size_;
    Eigen::Array4f leaf_size_;
    leaf_size_[0] = voxel_size;
    leaf_size_[1] = voxel_size;
    leaf_size_[2] = voxel_size;
    if (leaf_size_[3] == 0)
      leaf_size_[3] = 1;
    inverse_leaf_size_ = Eigen::Array4f::Ones () / leaf_size_.array ();

    //检查大小
    int64_t dx = static_cast<int64_t>((max_p[0] - min_p[0]) * inverse_leaf_size_[0])+1;
    int64_t dy = static_cast<int64_t>((max_p[1] - min_p[1]) * inverse_leaf_size_[1])+1;
    int64_t dz = static_cast<int64_t>((max_p[2] - min_p[2]) * inverse_leaf_size_[2])+1;

    if( (dx*dy*dz) > static_cast<int64_t>(std::numeric_limits<int32_t>::max()) )
    {
      std::cout<<"Leaf size is too small for the input dataset. Integer indices would overflow"<<std::endl;
      return;
    }

    //计算最小和最大边界框值
    Eigen::Vector4i min_b_, max_b_, div_b_, divb_mul_;
    min_b_[0] = static_cast<int> (floor (min_p[0] * inverse_leaf_size_[0]));
    max_b_[0] = static_cast<int> (floor (max_p[0] * inverse_leaf_size_[0]));
    min_b_[1] = static_cast<int> (floor (min_p[1] * inverse_leaf_size_[1]));
    max_b_[1] = static_cast<int> (floor (max_p[1] * inverse_leaf_size_[1]));
    min_b_[2] = static_cast<int> (floor (min_p[2] * inverse_leaf_size_[2]));
    max_b_[2] = static_cast<int> (floor (max_p[2] * inverse_leaf_size_[2]));

    //计算沿所有轴所需的分割数
    div_b_ = max_b_ - min_b_ + Eigen::Vector4i::Ones ();
    div_b_[3] = 0;

    std::vector<cloud_point_index_idx> index_vector;
    index_vector.reserve (cloud.size());
    divb_mul_ = Eigen::Vector4i (1, div_b_[0], div_b_[0] * div_b_[1], 0);
    pt  = Eigen::Vector4f::Zero ();
    int centroid_size = 3;

    //第一遍：遍历所有点，并将其插入右边的叶子
    for (size_t cp = 0; cp < cloud.size(); ++cp)
    {
      memcpy (&pt[0], &cloud.at(cp).x, sizeof (float));
      memcpy (&pt[1], &cloud.at(cp).y, sizeof (float));
      memcpy (&pt[2], &cloud.at(cp).z, sizeof (float));
      int ijk0 = static_cast<int> (floor (pt[0] * inverse_leaf_size_[0]) - min_b_[0]);
      int ijk1 = static_cast<int> (floor (pt[1] * inverse_leaf_size_[1]) - min_b_[1]);
      int ijk2 = static_cast<int> (floor (pt[2] * inverse_leaf_size_[2]) - min_b_[2]);
      //计算质心叶索引
      int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];
      index_vector.push_back (cloud_point_index_idx (idx, static_cast<unsigned int> (cp)));
    }
    std::cout <<"time use in compute idx is        " <<1000*  (clock() - time_start)/(double)CLOCKS_PER_SEC <<"ms" << std::endl;

    //第二遍：使用表示目标单元格的值作为索引对index_vector向量排序,实际上属于同一输出单元格的所有点将彼此相邻
    clock_t time_sort = clock();
    std::sort (index_vector.begin (), index_vector.end (), std::less<cloud_point_index_idx> ());
    std::cout <<"time use in sort is               " <<1000*  (clock() - time_sort)/(double)CLOCKS_PER_SEC <<"ms" << std::endl;

    //第三遍：计算质心，将它们插入到最终位置
    clock_t time_com_centroid = clock();
    Eigen::VectorXf centroid_have_normal = Eigen::VectorXf::Zero (3);
    Eigen::VectorXf centroid_nor = Eigen::VectorXf::Zero (3);
    if (!if_normal)
    {
       for (size_t cp = 0; cp < index_vector.size ();)
       {
         centroid_have_normal[0] = cloud.at(index_vector[cp].cloud_point_index).x;
         centroid_have_normal[1] = cloud.at(index_vector[cp].cloud_point_index).y;
         centroid_have_normal[2] = cloud.at(index_vector[cp].cloud_point_index).z;

         size_t i = cp + 1;
         while (i < index_vector.size () && index_vector[i].idx == index_vector[cp].idx)
         {
           centroid_have_normal[0] += cloud.at(index_vector[i].cloud_point_index).x;
           centroid_have_normal[1] += cloud.at(index_vector[i].cloud_point_index).y;
           centroid_have_normal[2] += cloud.at(index_vector[i].cloud_point_index).z;
           ++i;
         }
         //使质心标准化,并存入vector中
         PointNormal centroid;
         centroid_nor.normalize();
         centroid_have_normal /= static_cast<float> (i - cp);
         centroid.x = centroid_have_normal[0];
         centroid.y = centroid_have_normal[1];
         centroid.z = centroid_have_normal[2];
         centroid_set.push_back(centroid);
         cp = i;
       }
    }
    else //downsample_xyz with normal
    {
      for (size_t cp = 0; cp < index_vector.size ();)
      {
        centroid_have_normal[0] = cloud.at(index_vector[cp].cloud_point_index).x;
        centroid_have_normal[1] = cloud.at(index_vector[cp].cloud_point_index).y;
        centroid_have_normal[2] = cloud.at(index_vector[cp].cloud_point_index).z;
        centroid_nor[0] = cloud.at(index_vector[cp].cloud_point_index).nx;
        centroid_nor[1] = cloud.at(index_vector[cp].cloud_point_index).ny;
        centroid_nor[2] = cloud.at(index_vector[cp].cloud_point_index).nz;

        size_t i = cp + 1;
        while (i < index_vector.size () && index_vector[i].idx == index_vector[cp].idx)
        {
          centroid_have_normal[0] += cloud.at(index_vector[i].cloud_point_index).x;
          centroid_have_normal[1] += cloud.at(index_vector[i].cloud_point_index).y;
          centroid_have_normal[2] += cloud.at(index_vector[i].cloud_point_index).z;
          centroid_nor[0] += cloud.at(index_vector[i].cloud_point_index).nx;
          centroid_nor[1] += cloud.at(index_vector[i].cloud_point_index).ny;
          centroid_nor[2] += cloud.at(index_vector[i].cloud_point_index).nz;
          ++i;
        }
        //使质心标准化,并存入vector中
        PointNormal centroid;
        centroid_nor.normalize();
        centroid_have_normal /= static_cast<float> (i - cp);
        centroid.x = centroid_have_normal[0];
        centroid.y = centroid_have_normal[1];
        centroid.z = centroid_have_normal[2];
        centroid.nx = centroid_nor[0];
        centroid.ny = centroid_nor[1];
        centroid.nz = centroid_nor[2];
        centroid_set.push_back(centroid);
        cp = i;
      }
    }
    std::cout << "time use in centroid_normalize is " <<1000*  (clock() - time_com_centroid)/(double)CLOCKS_PER_SEC <<"ms" << std::endl;
    std::cout << "time use in all                is " <<1000*  (clock() - time_start)/(double)CLOCKS_PER_SEC <<"ms" << std::endl;
    std::cout << "采样后的点 =  "<< centroid_set.size() << std::endl;
    return;
  }

  void voxelfilter(PointCloud& cloud, std::vector<int>& indices, float voxel_size)
  {
    indices.clear();
    std::cout<<"采样前的点 =  "<< cloud.size() <<std::endl;
    clock_t time_start = clock();
    //获取最大/小点
    Eigen::Array4f min_p, max_p;
    min_p.setConstant (FLT_MAX);
    max_p.setConstant (-FLT_MAX);
    Eigen::Array4f pt = Eigen::Array4f::Zero ();
    for (size_t cp = 0; cp < cloud.size(); ++cp)
    {
      // memcpy (&pt[0], &point_set.at(cp).x, sizeof (float));
      // memcpy (&pt[1], &point_set.at(cp).y, sizeof (float));
      // memcpy (&pt[2], &point_set.at(cp).z, sizeof (float));
      pt[0] = cloud.at(cp).x;
      pt[1] = cloud.at(cp).y;
      pt[2] = cloud.at(cp).z;
      min_p = (min_p.min) (pt);
      max_p = (max_p.max) (pt);
    }

    //设置栅格的大小
    Eigen::Array4f inverse_leaf_size_;
    Eigen::Array4f leaf_size_;
    leaf_size_[0] = voxel_size;
    leaf_size_[1] = voxel_size;
    leaf_size_[2] = voxel_size;
    if (leaf_size_[3] == 0)
      leaf_size_[3] = 1;
    inverse_leaf_size_ = Eigen::Array4f::Ones () / leaf_size_.array ();

    //检查大小
    int64_t dx = static_cast<int64_t>((max_p[0] - min_p[0]) * inverse_leaf_size_[0])+1;
    int64_t dy = static_cast<int64_t>((max_p[1] - min_p[1]) * inverse_leaf_size_[1])+1;
    int64_t dz = static_cast<int64_t>((max_p[2] - min_p[2]) * inverse_leaf_size_[2])+1;

    if( (dx*dy*dz) > static_cast<int64_t>(std::numeric_limits<int32_t>::max()) )
    {
      std::cout<<"Leaf size is too small for the input dataset. Integer indices would overflow"<<std::endl;
      return;
    }

    //计算最小和最大边界框值
    Eigen::Vector4i min_b_, max_b_, div_b_, divb_mul_;
    min_b_[0] = static_cast<int> (floor (min_p[0] * inverse_leaf_size_[0]));
    max_b_[0] = static_cast<int> (floor (max_p[0] * inverse_leaf_size_[0]));
    min_b_[1] = static_cast<int> (floor (min_p[1] * inverse_leaf_size_[1]));
    max_b_[1] = static_cast<int> (floor (max_p[1] * inverse_leaf_size_[1]));
    min_b_[2] = static_cast<int> (floor (min_p[2] * inverse_leaf_size_[2]));
    max_b_[2] = static_cast<int> (floor (max_p[2] * inverse_leaf_size_[2]));

    //计算沿所有轴所需的分割数
    div_b_ = max_b_ - min_b_ + Eigen::Vector4i::Ones ();
    div_b_[3] = 0;

    std::vector<cloud_point_index_idx> index_vector;
    index_vector.reserve (cloud.size());
    divb_mul_ = Eigen::Vector4i (1, div_b_[0], div_b_[0] * div_b_[1], 0);
    pt  = Eigen::Vector4f::Zero ();
    int centroid_size = 3;

    //第一遍：遍历所有点，并将其插入右边的叶子
    for (size_t cp = 0; cp < cloud.size(); ++cp)
    {
      memcpy (&pt[0], &cloud.at(cp).x, sizeof (float));
      memcpy (&pt[1], &cloud.at(cp).y, sizeof (float));
      memcpy (&pt[2], &cloud.at(cp).z, sizeof (float));
      int ijk0 = static_cast<int> (floor (pt[0] * inverse_leaf_size_[0]) - min_b_[0]);
      int ijk1 = static_cast<int> (floor (pt[1] * inverse_leaf_size_[1]) - min_b_[1]);
      int ijk2 = static_cast<int> (floor (pt[2] * inverse_leaf_size_[2]) - min_b_[2]);
      //计算质心叶索引
      int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];
      index_vector.push_back (cloud_point_index_idx (idx, static_cast<unsigned int> (cp)));
    }
    std::cout <<"time use in compute idx is        " <<1000*  (clock() - time_start)/(double)CLOCKS_PER_SEC <<"ms" << std::endl;

    //第二遍：使用表示目标单元格的值作为索引对index_vector向量排序,实际上属于同一输出单元格的所有点将彼此相邻
    clock_t time_sort = clock();
    std::sort (index_vector.begin (), index_vector.end (), std::less<cloud_point_index_idx> ());
    std::cout <<"time use in sort is               " <<1000*  (clock() - time_sort)/(double)CLOCKS_PER_SEC <<"ms" << std::endl;


    //第三遍：计算质心，将它们插入到最终位置
    clock_t time_com_centroid = clock();
    Eigen::VectorXf centroid_have_normal = Eigen::VectorXf::Zero (3);
    Eigen::VectorXf centroid_nor = Eigen::VectorXf::Zero (3);
    for (size_t cp = 0; cp < index_vector.size ();)
    {
      size_t i = cp + 1;
      while (i < index_vector.size () && index_vector[i].idx == index_vector[cp].idx)
      {
        indices.push_back(index_vector[i].cloud_point_index);
        ++i;
      }
      cp = i;
    }
    std::cout << "time use in centroid_normalize is " <<1000*  (clock() - time_com_centroid)/(double)CLOCKS_PER_SEC <<"ms" << std::endl;
    std::cout << "time use in all                is " <<1000*  (clock() - time_start)/(double)CLOCKS_PER_SEC <<"ms" << std::endl;
    std::cout << "采样后的点 =  "<< indices.size() << std::endl;
    return;
  }
}
