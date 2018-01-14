#ifndef FITING_H
#define FITING_H


//最小二乘拟合平面,输入point_set,  输出parameters（nx,ny,nz,d）
void fitting_plane_least(const std::vector<Point>& point_set,std::vector<float>& parameters);
//ransac拟合平面
void fitting_plane_ransac(const std::vector<Point>& point_set, int num_iter, float threshold, std::vector<float>& parameters);

#endif
