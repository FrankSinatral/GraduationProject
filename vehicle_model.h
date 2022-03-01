#include<vector>
#include<math.h>
using namespace std;
//定义state为1*4行向量，共有0,1...,N个状态，control为1*2行向量，共有0,1...,N-1个状态
class Model {
   public:
     double t_r;//采样时间
     int N;//horizon
     //写一个给定状态获取速度文件等一系列的函数
     //从状态量中分别获取速度和theta
     vector<double> get_vel(vector<vector<double>> X) {
       vector<double> ans(N + 1,0);
       for (int i = 0; i < N + 1;++i) {
         ans[i] = X[i][2];
       }
      return ans;
     }
     vector<double> get_theta(vector<vector<double>> X) {
       vector<double> ans(N + 1,0);
       for (int i = 0; i < N + 1;++i) {
         ans[i] = X[i][3];
       }
      return ans;
     }
     //从控制量中获取车辆各个时刻的加速度
     vector<double> get_acc(vector<vector<double>> U) {
       vector<double> ans(N,0);
       for (int i = 0; i < N;++i) {
         ans[i] = U[i][0];
       }
      return ans;
     }
     //给定t时刻下的状态和控制，返回下一时刻的状态
     vector<double> forward_simulate(vector<double> state,vector<double> control){
       vector<double> next_state(4);
       next_state[0] = state[0] + cos(state[3])*(state[2]*t_r + (control[0]*t_r*t_r)/2);
       next_state[1] = state[1] + sin(state[3])*(state[2]*t_r + (control[0]*t_r*t_r)/2);
       next_state[2] = state[2] + control[0]*t_r;
       next_state[3] = state[3] + control[1]*t_r;
       return next_state;
     }
     //输入theta值，返回df_du
     vector<vector<vector<double>>> get_B_matrix (vector<double> theta) {
       vector<vector<vector<double>>> B(N,vector<vector<double>>(4,vector<double>(2,0)));
       for(int i = 0; i < N; ++i) {
           B[i] = {{t_r*t_r*cos(theta[i])/2, 0},{t_r*t_r*sin(theta[i])/2, 0},{t_r, 0},{0, t_r}};
       }
       return B;
       //B是一个N*4*2的矩阵，0,1..N - 1时刻，表示df_du
     }
     //输入速度、theta、加速度值，返回df_dx
     vector<vector<vector<double>>> get_A_matrix (vector<double> velocity_vals, vector<double> theta, vector<double> acceleration_vals) {
       vector<vector<vector<double>>> A(N, vector<vector<double>>(4,vector<double>(4,0)));
       for(int i = 0; i < N; ++i) {
           A[i] = {{1,0,cos(theta[i])*t_r,-(velocity_vals[i]*t_r + (acceleration_vals[i]*t_r*t_r)/2)*sin(theta[i])},{0,1,sin(theta[i])*t_r,(velocity_vals[i]*t_r + (acceleration_vals[i]*t_r*t_r)/2)*cos(theta[i])},
           {0,0,1,0},{0,0,0,1}};
       }
       return A;
       //A是一个N*4*4的矩阵，0,1..N - 1时刻，表示df_dx
     }
};