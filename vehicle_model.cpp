#include<vector>
#include<math.h>
using namespace std;
//定义state为1*4行向量，共有0,1...,N个状态，control为1*2行向量，共有0,1...,N-1个状态
class Model {
   double t;//采样时间
   int N;//horizon
   vector<double> forward_simulate(vector<double> state,vector<double> control){
       vector<double> next_state(4);
       next_state[0] = state[0] + cos(state[3])*(state[2]*t + (control[0]*t*t)/2);
       next_state[1] = state[1] + sin(state[3])*(state[2]*t + (control[0]*t*t)/2);
       next_state[2] = state[2] + control[0]*t;
       next_state[3] = state[3] + control[1]*t;
       return next_state;

   }
   void get_B_matrix (vector<double> theta) {
       vector<vector<vector<double>>> B(N,vector<vector<double>>(4,vector<double>(2,0)));
       for(int i = 0; i < N; ++i) {
           B[i] = {{t*t*cos(theta[i])/2, 0},{t*t*sin(theta[i])/2, 0},{t, 0},{0, t}};
       }
       //B是一个4*2*N的矩阵，表示df_du
   }
   void get_A_matrix (vector<double> velocity_vals, vector<double> theta, vector<double> acceleration_vals) {
       vector<vector<vector<double>>> A(N, vector<vector<double>>(4,vector<double>(4,0)));
       for(int i = 0; i < N; ++i) {
           A[i] = {{1,0,cos(theta[i])*t,-(velocity_vals[i]*t + (acceleration_vals[i]*t*t)/2)*sin(theta[i])},{0,1,sin(theta[i])*t,(velocity_vals[i]*t + (acceleration_vals[i]*t*t)/2)*cos(theta[i])},
           {0,0,1,0},{0,0,0,1}};
       }
       //A是一个4*4*N的矩阵，表示df_dx
   }
};