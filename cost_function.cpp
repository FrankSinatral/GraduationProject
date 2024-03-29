#include"cost_function.h"
#include<math.h>
void L_Functions::set_par(double w_a,double w_s,double w_v,double w_r,double a_h,double a_l,double del_bar,double v_r) {
 w_acc = w_a;
 w_steer = w_s;
 w_vel = w_v;
 w_ref = w_r;
 a_high = a_h;
 a_low = a_l;
 delta_bar = del_bar;
 v_ref = v_r;
 }

     
//vector<vector<vector<double>>> l_ux(N,vector<vector<double>>(2,vector<double>(4,0))); //事实上，l_ux即为2*4的零矩阵
//首先求cost function对于control的一二阶偏导数
//l_u是一个2*1向量，0,1...N - 1个状态 
vector<vector<vector<double>>> L_Functions::get_control_first_derivatives(vector<vector<double>> control,double t) {
 int n = control.size();
 vector<vector<vector<double>>> l_u(n,vector<vector<double>>(2,vector<double>(1,0)));
 for (int i = 0; i < n; ++i) {
  l_u[i] = {{2*w_acc*control[i][0] + 1/(t*(a_high - control[i][0]))-1/(t*(control[i][0] - a_low))},
            {2*w_steer*control[i][1] + 1/(t*(delta_bar - control[i][1])) - 1/(t*(delta_bar + control[i][1]))}};
 }
 return l_u;
}

//l_uu是一个2*2矩阵，0,1...,N - 1个状态
vector<vector<vector<double>>> L_Functions::get_control_second_derivatives(vector<vector<double>> control,double t) {
 int n = control.size();
 vector<vector<vector<double>>> l_uu(n,vector<vector<double>>(2,vector<double>(2,0)));
 for (int i = 0; i < n; ++i) {
 l_uu[i] = {{2*w_acc + 1/(t*(a_high - control[i][0])*(a_high - control[i][0])) + 1/(t*(control[i][0] - a_low)*(control[i][0] - a_low)),0},
            {0,2*w_steer + 1/(t*(delta_bar - control[i][1])*(delta_bar - control[i][1])) + 1/(t*(delta_bar + control[i][1])*(delta_bar + control[i][1]))}};
 }
 return l_uu;
}

//下面求cost function对于state的一二阶偏导数
//l_x是一个4*1向量，0,1...N个状态
vector<vector<vector<double>>> L_Functions::get_state_first_derivatives(vector<vector<double>> state,vector<vector<double>> center,vector<vector<vector<double>>> A,double t) {
 int n = state.size();//Horizon N + 1
 int l = center.size();//障碍物个数
 vector<vector<vector<double>>> l_x(n,vector<vector<double>>(4,vector<double>(1,0)));
 vector<vector<double>> loc = model.get_loc(state);
 vector<vector<vector<double>>> obs_bar_first_derivatives(n,vector<vector<double>>(4,vector<double>(1,0)));
 for (int j = 0; j < l; ++j) {
  obs_bar_first_derivatives = m.matrixAdd_2(obs_bar_first_derivatives,bar.get_obs_bar_first_derivatives(loc,center[j],A[j],t));
 }
 for (int i = 0; i < n; ++i) {
  l_x[i] = {{0},{2*w_ref*state[i][1]},{2*w_vel*(state[i][2] - v_ref)},{0}};
  l_x[i] = m.matrixAdd(l_x[i],obs_bar_first_derivatives[i]);//加上障碍物求导项
 }
 return l_x;
}
//无限制情形

//l_xx是一个4*4向量，0,1...N个状态 
vector<vector<vector<double>>> L_Functions::get_state_second_derivatives(vector<vector<double>> state,vector<vector<double>> center,vector<vector<vector<double>>> A,double t) {
 int n = state.size();
 int l = center.size();
 vector<vector<vector<double>>> l_xx(n,vector<vector<double>>(4,vector<double>(4,0)));
 vector<vector<double>> loc = model.get_loc(state);
 vector<vector<vector<double>>> obs_bar_second_derivatives(n,vector<vector<double>>(4,vector<double>(4,0)));
 for (int j = 0; j < l; ++j) {
  obs_bar_second_derivatives = m.matrixAdd_2(obs_bar_second_derivatives,bar.get_obs_bar_second_derivatives(loc,center[j],A[j],t));
 }
 for (int i = 0; i < n; ++i) {
  l_xx[i] = {{0,0,0,0},{0,2*w_ref,0,0},{0,0,2*w_vel,0},{0,0,0,0}};
  l_xx[i] = m.matrixAdd(l_xx[i],obs_bar_second_derivatives[i]);
 }
 return l_xx;
 }

/*vector<vector<vector<double>>> L_Functions::get_control_first_derivatives(vector<vector<double>> control) {
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
 */    


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
//将加速度限制转化为barrier function
double L_Functions::cost_acc_bf(vector<vector<double>> control,double t) {
 double sum = 0;
 for (int i = 0; i < control.size(); ++i) {
  sum -= (1/t)*log((a_high - control[i][0])*(control[i][0] - a_low));
 }
 return sum;
}
//将扭转角度限制转化为barrier function
double L_Functions::cost_delta_bf(vector<vector<double>> control,double t) {
 double sum = 0;
 for (int i = 0; i < control.size(); ++i) {
  sum -= (1/t)*log((delta_bar - control[i][1])*(control[i][1] + delta_bar));
 }
 return sum;
}
//计算obscalcle限制并转化为barrier function,利用Barrier_Function类进行计算，直接加入到代价函数总和中
//计算代价函数总和
double L_Functions::cost_all(vector<vector<double>> state, vector<vector<double>> control,vector<vector<double>> center,vector<vector<vector<double>>> A,double t) {
 double ans = 0;
 int l = center.size();//障碍物个数
 vector<vector<double>> loc = model.get_loc(state);
 double sum_1 = cost_acc(control), sum_2 = cost_steer(control);
 double sum_3 = cost_veltracking(state), sum_4 = cost_reftracking(state);
 double sum_5 = cost_acc_bf(control,t), sum_6 = cost_delta_bf(control,t);
 double sum_7 = 0;
 for (int j = 0; j < l; ++j) {
  sum_7 += bar.get_obs_bar_value(loc,center[j],A[j],t);
 }
 ans = sum_1 + sum_2 + sum_3 + sum_4 + sum_5 + sum_6 + sum_7;
 return ans;
}
//检验控制集是否满足限制条件
bool L_Functions::check_control(vector<vector<double>> U) {
 int m = U.size();
 for (int i = 0; i < m; ++i) {
  if(U[i][0] >= a_high || U[i][0] <= a_low) {
   return false;
  }
 if(U[i][1] <= -delta_bar || U[i][1] >= delta_bar) {
   return false;
  }
 } 
 return true;   
}
