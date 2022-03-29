#include<vector>
#include<fstream>
using std::vector;
#include"vehicle_model.h"
#include"iLQR.h"
#include"cost_function.h"
#include"constraints.h"
#include"barrier_function.h"
extern Model model;
extern Barrier_Function bar;

extern Obstacle obs_1;
extern Obstacle obs_2;
extern Obstacle obs_3;
extern iLQR ilqr;
extern L_Functions l;
int main() {
 vector<double> X_0 = {0,0,8,0};//设置初始状态
 l.set_par(1,1,1,0.5,0.6,-0.6,0.4,8);//初始化权重函数和控制集限制条件
 //初始化控制集
 vector<double> U_0 = {0,0.2};
 vector<double> U_1 = {0,-0.2};
 vector<double> U_2 = {0,0};
 vector<vector<double>> U;
 for (int i = 0; i < 10; ++i) {
  U.push_back(U_0);
 }
 for (int i = 0; i < 10; ++i) {
 U.push_back(U_1);
 }
 for (int i = 0; i < 10; ++i) {
  U.push_back(U_2);
 }
 for (int i = 0; i < 10; ++i) {
  U.push_back(U_1);
 }
 for (int i = 0; i < 10; ++i) {
  U.push_back(U_0);
 }
 model.set_pra(0.2);//设置采样时间为0.2s
 //初始化障碍物参数，赋值障碍物矩阵函数和矩阵中心
 obs_1.set_obstacle_pra({20,0},3,2,0.6,0);//s_safe的合理值为0.6
 obs_2.set_obstacle_pra({40,0},3,2,0.6,0);
 obs_3.set_obstacle_pra({60,0},3,2,0.6,0);
 vector<vector<double>> obs_center;
 obs_center.push_back(obs_1.center);
 obs_center.push_back(obs_2.center);
 obs_center.push_back(obs_3.center);
 vector<vector<double>> ans_1 = obs_1.get_obs_matrix(obs_1.theta,obs_1.l,obs_1.w,obs_1.s_safe);
 vector<vector<double>> ans_2 = obs_2.get_obs_matrix(obs_2.theta,obs_2.l,obs_2.w,obs_2.s_safe);
 vector<vector<double>> ans_3 = obs_3.get_obs_matrix(obs_3.theta,obs_3.l,obs_3.w,obs_3.s_safe);
 vector<vector<vector<double>>> obs_mat;
 obs_mat.push_back(ans_1);
 obs_mat.push_back(ans_2);
 obs_mat.push_back(ans_3);
 //设置cilqr中的其他参数
 double t_0 = 0.5;
 double u = 2;
 double eps_1 = 0.1;
 double eps_2 = 0.1;
 
 vector<vector<vector<double>>> result = ilqr.algorithm_cilqr(X_0,U,obs_center,obs_mat,t_0,u,eps_1,eps_2);
 vector<vector<double>> X_desired = result[0];
 vector<vector<double>> U_desired = result[1];
 std::ofstream fout("test8_3.csv");
 for (int i = 0; i < X_desired.size(); ++i) {
   for (int j = 0; j < 4; ++j) {
     if (j == 3) {
       fout << X_desired[i][j] << std::endl;
     } else {
       fout << X_desired[i][j] << ",";
     }
   }
 }
 fout << std::endl;
 for (int i = 0; i < U_desired.size(); ++i) {
   for (int j = 0; j < 2; ++j) {
     if (j == 1) {
       fout << U_desired[i][j] << std::endl;
     } else {
       fout << U_desired[i][j] << ",";
     }
   }
 }
 fout.close();
 return 0;
}