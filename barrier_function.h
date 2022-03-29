#pragma once
#include"matrix.h"
#include<vector>
using std::vector;
extern Matrix m;
class Barrier_Function {
 public:
  //返回单个障碍物对于某一轨迹的barrier function总和(如果需要考虑静态障碍物，这里还需要做修正)
  double get_obs_bar_value(vector<vector<double>> location, vector<double> center, vector<vector<double>> A,double t);
  //用于判断对于单个障碍物是否符合限制条件
  bool check_obs_constraint(vector<vector<double>> location, vector<double> center, vector<vector<double>> A);
  //判断对于多个障碍物是否符合限制条件
  bool check_all_obs_constraint(vector<vector<double>> location, vector<vector<double>> obs_center,vector<vector<vector<double>>> obs_mat);
  //返回单个障碍物对于整个轨迹的导函数，一阶偏导数即(N+1)*4*1,0,1...N个状态
  //如果考虑动态障碍物，此处还需要修改
  vector<vector<vector<double>>> get_obs_bar_first_derivatives(vector<vector<double>> location, vector<double> center, vector<vector<double>> A,double t);
  //二阶偏导数即(N+1)*4*4,0.1...N个状态
  vector<vector<vector<double>>> get_obs_bar_second_derivatives(vector<vector<double>> location, vector<double> center, vector<vector<double>> A,double t);
};