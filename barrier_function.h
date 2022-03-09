#include<vector>
#include"matrix.h"
#include<math.h>
using std::vector;
class Barrier_Function {
 public:
  class Matrix m;
  //返回单个障碍物对于某一轨迹的barrier function总和(注意这一点非常重要)
  double get_obs_bar_value(vector<vector<double>> location, vector<double> center, vector<vector<double>> A,double t) {
    double sum = 0;
    int n = location.size();
    for (int i = 0; i < n; ++i) {
     vector<double> terms = m.vectorSubtract(location[i],center);
     double f = 1 - (A[0][0]*terms[0]*terms[0] + (A[1][0] + A[0][1])*terms[0]*terms[1] + A[1][1]*terms[1]*terms[1]);
     sum += (-1/t)*log(f);
    }
    return sum;
  }
  //用于判断对于单个障碍物是否符合限制条件
  bool check_obs_constraint(vector<vector<double>> location, vector<double> center, vector<vector<double>> A,double t) {
    int n = location.size();
    for (int i = 0; i < n; ++i) {
     vector<double> terms = m.vectorSubtract(location[i],center);
     double f = 1 - (A[0][0]*terms[0]*terms[0] + (A[1][0] + A[0][1])*terms[0]*terms[1] + A[1][1]*terms[1]*terms[1]);
     if (f >= 0) {
       return false; 
     }
    }
    return true;
  }
  //判断对于多个障碍物是否符合限制条件
  bool check_all_obs_constraint(vector<vector<double>> location, vector<vector<double>> obs_center,vector<vector<vector<double>>> obs_mat,double t) {
    int m = obs_center.size();//获取障碍物个数
    for (int i = 0; i < m; ++i) {
      bool flag = check_obs_constraint(location,obs_center[i],obs_mat[i],t);
      if (!flag) {
        return false;
      }
    }
    return true;
  }
  //返回单个障碍物对于整个轨迹的导函数，一阶偏导数即(N+1)*4*1,0,1...N个状态
  //二阶偏导数即(N+1)*4*4,0.1...N个状态
  vector<vector<vector<double>>> get_obs_bar_first_derivatives(vector<vector<double>> location, vector<double> center, vector<vector<double>> A,double t) {
    int n = location.size();//Horizon N + 1
    vector<vector<vector<double>>> ans(n,vector<vector<double>>(4,vector<double>(1,0)));
    for (int i = 0; i < n; ++i) {
     vector<double> terms = m.vectorSubtract(location[i],center);
     double f = 1 - (A[0][0]*terms[0]*terms[0] + (A[1][0] + A[0][1])*terms[0]*terms[1] + A[1][1]*terms[1]*terms[1]);
     double f_x = -(2*A[0][0]*terms[0] + (A[1][0] + A[0][1])*terms[1]);
     double f_y = -(2*A[1][1]*terms[1] + (A[1][0] + A[0][1])*terms[0]);
     //double f_xx = -2*A[0][0];
     //double f_xy = -(A[1][0] + A[0][1]);
     //double f_yy = -2*A[1][1];
     ans[i][0][0] = (-1/t)*(f_x/f);
     ans[i][1][0] = (-1/t)*(f_y/f); 
    }
    return ans;
    
  }
  vector<vector<vector<double>>> get_obs_bar_second_derivatives(vector<vector<double>> location, vector<double> center, vector<vector<double>> A,double t) {
    int n = location.size(); //Horizon N + 1
    vector<vector<vector<double>>> ans(n,vector<vector<double>>(4,vector<double>(4,0)));
    for (int i = 0; i < n; ++i) {
     vector<double> terms = m.vectorSubtract(location[i],center);
     double f = 1 - (A[0][0]*terms[0]*terms[0] + (A[1][0] + A[0][1])*terms[0]*terms[1] + A[1][1]*terms[1]*terms[1]);
     double f_x = -(2*A[0][0]*terms[0] + (A[1][0] + A[0][1])*terms[1]);
     double f_y = -(2*A[1][1]*terms[1] + (A[1][0] + A[0][1])*terms[0]);
     double f_xx = -2*A[0][0];
     double f_xy = -(A[1][0] + A[0][1]);
     double f_yy = -2*A[1][1];
     ans[i][0][0] = (-1/t)*((f_xx*f - f_x*f_x)/(f*f));
     ans[i][1][0] = ans[i][0][1]= (-1/t)*((f_xy*f - f_x*f_y)/(f*f));
     ans[i][1][1] = (-1/t)*((f_yy*f - f_y*f_y)/(f*f));  
    }
    
    return ans;
  }
};