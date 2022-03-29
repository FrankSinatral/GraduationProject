#pragma once
#include"matrix.h"
#include"barrier_function.h"
#include"vehicle_model.h"
#include<vector>
using std::vector;
//处理代价函数的模块，包括计算代价函数值以及偏导数
extern Matrix m;
extern Model model;
extern Barrier_Function bar;
class L_Functions{
    public:
     double w_acc,w_steer,w_vel,w_ref;//各项权重
     double a_high,a_low,delta_bar;//限制条件
     double v_ref;//参考速度
     //double t;//barrier function中的参数t，需要选取一个初值
    //参数初始化
     void set_par(double w_a,double w_s,double w_v,double w_r,double a_h,double a_l,double del_bar,double v_r);
    //vector<vector<vector<double>>> l_ux(N,vector<vector<double>>(2,vector<double>(4,0))); //事实上，l_ux即为2*4的零矩阵
    //首先求cost function对于control的一二阶偏导数
    //l_u是一个2*1向量，0,1...N - 1个状态 
     vector<vector<vector<double>>> get_control_first_derivatives(vector<vector<double>> control,double t);
     //l_uu是一个2*2矩阵，0,1...,N - 1个状态
     vector<vector<vector<double>>> get_control_second_derivatives(vector<vector<double>> control,double t);
    //下面求cost function对于state的一二阶偏导数
    //l_x是一个4*1向量，0,1...N个状态
     vector<vector<vector<double>>> get_state_first_derivatives(vector<vector<double>> state,vector<vector<double>> center,vector<vector<vector<double>>> A,double t);
    //l_xx是一个4*4向量，0,1...N个状态 
     vector<vector<vector<double>>> get_state_second_derivatives(vector<vector<double>> state,vector<vector<double>> center,vector<vector<vector<double>>> A,double t);
    
    
    //计算代价函数
    //加速度
     double cost_acc(vector<vector<double>> control);
     //可以视为转动速度
     double cost_steer(vector<vector<double>> control);
     //参考速度
     double cost_veltracking(vector<vector<double>> state);
     //参考轨迹
     double cost_reftracking(vector<vector<double>> state);
     //将加速度限制转化为barrier function
     double cost_acc_bf(vector<vector<double>> control,double t);
     //将扭转角度限制转化为barrier function
     double cost_delta_bf(vector<vector<double>> control,double t);
     //计算obscalcle限制并转化为barrier function,利用Barrier_Function类进行计算，直接加入到代价函数总和中
     //计算代价函数总和
     double cost_all(vector<vector<double>> state, vector<vector<double>> control,vector<vector<double>> center,vector<vector<vector<double>>> A,double t);
     //检验控制集是否满足限制条件
     bool check_control(vector<vector<double>> U);

};