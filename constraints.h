#include<vector>
#include<math.h>
#include"matrix.h"
using std::vector;
class Obstacle{
 public:
  vector<double> center;
  double l,w,t_safe,s_safe,v_obstacle;
  double theta;//障碍物的偏转角
 public:
  //设置障碍物的各项参数
  void set_obstacle_pra(vector<double> cen,double l_0,double w_0,double t_sa,double s_sa,double v_obs) {
    center = cen;
    l = l_0;
    w = w_0;
    t_safe = t_sa;
    s_safe = s_sa;
    v_obstacle = v_obs;
  }
 public:
  vector<vector<double>> get_obs_matrix(double theta,double l,double w,double v,double t_safe,double s_safe) {
    vector<vector<double>> rot_matrix = {{cos(theta),sin(theta)},{-sin(theta),cos(theta)}};
    double long_axis = l + v*t_safe + s_safe;
    double short_axis = w + s_safe;
    vector<vector<double>> diag_matrix = {{1/(long_axis*long_axis),0},{0,1/(short_axis*short_axis)}};
    class Matrix m;
    vector<vector<double>> obs_matrix = m.congruentTrans(diag_matrix,rot_matrix);
    return obs_matrix;
  }
  /*vector<vector<double>> get_rot_matrix(double theta) {
    vector<vector<double>> ans = {{cos(theta),sin(theta)},{-sin(theta),cos(theta)}};
    return ans;
  }
  vector<double> get_axis(double l,double w,double v,double t_safe,double s_safe) {
    double long_axis = l + v*t_safe + s_safe;
    double short_axis = w + s_safe;
    vector<double> ans;
    ans.push_back(long_axis);
    ans.push_back(short_axis);
    return ans;
  }
  vector<vector<double>> get_diag(double a, double b) {
    vector<vector<double>> ans = {{1/(a*a),0},{0,1/(b*b)}};
    return ans;
  }*/
  //return obstacle function
  /*double get_obstacle_constraint(vector<double> location,vector<double> center,double theta,double l,double w,double t_safe,double s_safe,double v) {
    class Matrix m;
    vector<double> axis = get_axis(l,w,v,t_safe,s_safe);
    double a = axis[0],b = axis[1];
    vector<vector<double>> A = get_diag(a,b);
    vector<vector<double>> P = get_rot_matrix(theta);
    vector<vector<double>> rot_matrix = m.congruentTrans(A,P);
    vector<vector<double>> t = m.matrixMultiply(m.matrixMultiply(m.vecToMat(m.vectorSubtract(location,center)),rot_matrix),m.matrixTranspose(m.vecToMat(m.vectorSubtract(location,center))));
    double ans = 1 - t[0][0];
    return ans;
    
  }*/

};
class Constraints{
 public:
  //检验控制集是否满足限制条件
  bool check_control(vector<vector<double>> U,vector<double> a_contraint,double delta_bar) {
    int m = U.size();
    for (int i = 0; i < m; ++i) {
      if(U[i][0] > a_contraint[0] || U[i][0] < a_contraint[1]) {
          return false;
      }
      if(U[i][1] < -delta_bar || U[i][1] > delta_bar) {
          return false;
      }
    } 
    return true;   
  }
  bool check_obstacle(vector<vector<double>> X) {
    
  }
  bool check_all(vector<vector<double>> X,vector<vector<double>> U,vector<double> a_constraint,double delta_bar) {
    return check_control(U,a_constraint,delta_bar) && check_obstacle(X);
  }
};