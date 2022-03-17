#pragma once
#include<vector>
using std::vector;
//定义state为1*4行向量，共有0,1...,N个状态，control为1*2行向量，共有0,1...,N-1个状态
class Model {
  public:
   double t_r;//采样时间
   void set_pra(double t);
   //从状态量中分别获取速度和theta
   //get_loc 用于constraints.h
   vector<vector<double>> get_loc(vector<vector<double>> X);
   vector<double> get_vel(vector<vector<double>> X);
   vector<double> get_theta(vector<vector<double>> X);
   //从控制量中获取车辆各个时刻的加速度
   vector<double> get_acc(vector<vector<double>> U);
   //给定t时刻下的状态和控制，返回下一时刻的状态
   vector<double> forward_simulate(vector<double> state,vector<double> control);
   //给定初始状态和控制集，返回生成的所有状态
   vector<vector<double>> get_nomial_trajectory(vector<double> X_0,vector<vector<double>> U);
//输入速度、theta、加速度值，返回df_dx
   vector<vector<vector<double>>> get_A_matrix (vector<double> velocity_vals, vector<double> theta, vector<double> acceleration_vals);
//输入theta值，返回df_du
   vector<vector<vector<double>>> get_B_matrix (vector<double> theta);
};