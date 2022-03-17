#include"cost_function1.h"
#include<math.h>
void L_Functions::set_par(double w_a,double w_s,double w_v,double w_r,double v_r) {
 w_acc = w_a;
 w_steer = w_s;
 w_vel = w_v;
 w_ref = w_r;
 v_ref = v_r;
 }

     
//vector<vector<vector<double>>> l_ux(N,vector<vector<double>>(2,vector<double>(4,0))); //事实上，l_ux即为2*4的零矩阵
//首先求cost function对于control的一二阶偏导数


vector<vector<vector<double>>> L_Functions::get_control_first_derivatives(vector<vector<double>> control) {
 int n = control.size();
 vector<vector<vector<double>>> l_u(n,vector<vector<double>>(2,vector<double>(1,0)));
 for (int i = 0; i < n; ++i) {
  l_u[i] = {{2*w_acc*control[i][0]},{2*w_steer*control[i][1]}};
 }
 return l_u;
}

vector<vector<vector<double>>> L_Functions::get_control_second_derivatives(vector<vector<double>> control) {
 int n = control.size();
 vector<vector<vector<double>>> l_uu(n,vector<vector<double>>(2,vector<double>(2,0)));
 for (int i = 0; i < n; ++i) {
 l_uu[i] = {{2*w_acc,0},{0,2*w_steer}};
 }
 return l_uu;
}



vector<vector<vector<double>>> L_Functions::get_state_first_derivatives(vector<vector<double>> state) {
 int n = state.size();//Horizon N + 1
 vector<vector<vector<double>>> l_x(n,vector<vector<double>>(4,vector<double>(1,0)));
 vector<vector<double>> loc = model.get_loc(state);
 for (int i = 0; i < n; ++i) {
  l_x[i] = {{0},{2*w_ref*state[i][1]},{2*w_vel*(state[i][2] - v_ref)},{0}};
 }
 return l_x;
}


 vector<vector<vector<double>>> L_Functions::get_state_second_derivatives(vector<vector<double>> state) {
 int n = state.size();
 vector<vector<vector<double>>> l_xx(n,vector<vector<double>>(4,vector<double>(4,0)));
 vector<vector<double>> loc = model.get_loc(state);
 for (int i = 0; i < n; ++i) {
  l_xx[i] = {{0,0,0,0},{0,2*w_ref,0,0},{0,0,2*w_vel,0},{0,0,0,0}};
 }
 return l_xx;
 }   


//计算代价函数
//加速度
double L_Functions::cost_acc(vector<vector<double>> control) {
 double sum = 0;
 for (int i = 0; i < control.size(); ++i) {
  sum += w_acc*control[i][0]*control[i][0];
 }
 return sum;
}
//可以视为转动速度
double L_Functions::cost_steer(vector<vector<double>> control) {
 double sum = 0;
 for (int i = 0; i < control.size(); ++i) {
  sum += w_steer*control[i][1]*control[i][1];
 }
 return sum;
}
//参考速度
double L_Functions::cost_veltracking(vector<vector<double>> state) {
 double sum = 0;
 for (int i = 0; i < state.size(); ++i) {
  sum += w_vel*(state[i][2] - v_ref)*(state[i][2] - v_ref);
 }
 return sum;
}
//参考轨迹
double L_Functions::cost_reftracking(vector<vector<double>> state) {
 double sum = 0;
 for (int i = 0; i < state.size(); ++i) {
  sum += w_ref*state[i][1]*state[i][1];//此处对参考轨迹进行了简化
 }
 return sum;
}
//计算obscalcle限制并转化为barrier function,利用Barrier_Function类进行计算，直接加入到代价函数总和中
//计算代价函数总和
double L_Functions::cost_all(vector<vector<double>> state, vector<vector<double>> control) {
  double ans = 0;
  double sum_1 = cost_acc(control), sum_2 = cost_steer(control);
  double sum_3 = cost_veltracking(state), sum_4 = cost_reftracking(state);
  ans = sum_1 + sum_2 + sum_3 + sum_4;
  return ans;
}