#include<vector>
#include"matrix.h"
#include<math.h>
using std::vector;
class Barrier_Function {
 public:
  class Matrix m;
  //返回一个障碍物对于某一轨迹的barrier function总和
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
  vector<vector<vector<double>>> get_obs_bar_first_derivatives(vector<vector<double>> location, vector<double> center, vector<vector<double>> A,double t) {
    //class Matrix m;
    int n = location.size();
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
    //class Matrix m;
    int n = location.size();
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