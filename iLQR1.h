#pragma once
#include"vehicle_model.h"
#include"matrix.h"
#include"cost_function1.h"
#include<vector>
using std::vector;
extern L_Functions l;
extern Model model;
extern Matrix m;
class iLQR {
  public:
   //vector<vector<double>> get_nomial_trajectory(vector<double> X_0,vector<vector<double>> U);
   //输入原有的状态集和控制集，以及计算的k,K,更新新的状态集和控制集，返回值中第一维为新的状态集，第二维为新的控制集
   //add backtracking line search parameter
   vector<vector<vector<double>>> forward_pass(vector<vector<double>> X,vector<vector<double>> U,vector<vector<vector<double>>> k,vector<vector<vector<double>>> K,double alpha);
   vector<vector<double>> get_X_new(vector<vector<vector<double>>> A);
   vector<vector<double>> get_U_new(vector<vector<vector<double>>> A);
   //backward_pass过程,返回值的第一维为k，第二维为K
   
   vector<vector<vector<vector<double>>>> backward_pass(vector<vector<double>> X,vector<vector<double>> U);
   vector<vector<vector<double>>> get_k(vector<vector<vector<vector<double>>>> A);
   vector<vector<vector<double>>> get_K(vector<vector<vector<vector<double>>>> A);  
   

   vector<vector<vector<double>>> algorithm_ilqr(vector<double> X_0,vector<vector<double>> U,double eps);

  
};