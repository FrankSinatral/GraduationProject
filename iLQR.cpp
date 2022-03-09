#include<vector>
#include"vehicle_model.h"
#include"matrix.h"
#include"cost_function.h"
#include"constraints.h"
using std::vector;
class iLQR {
  public:
    class Model model;
    class Matrix m;
    class L_Functions l;
   //int N;//Horizon
   //输入初始状态和控制集，返回nomial_trajectory(1*4共0,1...,N个状态)
   vector<vector<double>> get_nomial_trajectory(vector<double> X_0,vector<vector<double>> U) {
       int n = U.size();//Horizon N
       vector<vector<double>> X(n + 1,vector<double>(4,0));
       X[0] = X_0;
       for (int i = 0; i < n;++i) {
           X[i + 1] = model.forward_simulate(X[i],U[i]); //调用Model类里面的forward_simulate函数
       }
       return X;
   }
   //输入原有的状态集和控制集，以及计算的k,K,更新新的状态集和控制集，返回值中第一维为新的状态集，第二维为新的控制集
   //add backtracking line search parameter
   vector<vector<vector<double>>> forward_pass(vector<vector<double>> X,vector<vector<double>> U,vector<vector<vector<double>>> k,vector<vector<vector<double>>> K,double alpha) {
       vector<vector<vector<double>>> ans;
       int n = X.size();
       vector<vector<double>> X_new(n,vector<double>(4,0));
       X_new[0] = X[0];
       vector<vector<double>> U_new(n - 1, vector<double>(2,0));
       for (int i = 0; i < n - 1; ++i) {
           //此处写完backward_pass之后还有待调试
           U_new[i] = m.vectorAdd(m.vectorAdd(U[i],m.scaleVec(m.columnToRow(k[i]),alpha)),m.matToVec(m.matrixTranspose(m.matrixMultiply(K[i],m.matrixTranspose(m.vecToMat(m.vectorSubtract(X_new[i],X[i])))))));
           X_new[i + 1] = model.forward_simulate(X_new[i],U_new[i]);
       }
       ans.push_back(X_new);
       ans.push_back(U_new);
       return ans;
   }
   vector<vector<double>> get_X_new(vector<vector<vector<double>>> A) {
       return A[0];
   }
   vector<vector<double>> get_U_new(vector<vector<vector<double>>> A) {
       return A[1];
   }
   //backward_pass过程,返回值的第一维为k，第二维为K
   
   vector<vector<vector<vector<double>>>> backward_pass(vector<vector<double>> X,vector<vector<double>> U,vector<vector<double>> obs_center,vector<vector<vector<double>>> obs_mat,double t) {
       int n = U.size(); //Horizon N
       //class Obstacle obs;
       //vector<vector<double>> A = obs.get_obs_matrix(obs.theta,obs.l,obs.w,obs.v_obstacle,obs.t_safe,obs.s_safe);
       vector<vector<vector<vector<double>>>> ans;
       //此处限制条件应该写在另外一个文件中
       double a_high = 0,a_low = 0,delta_bar = 0;
       //调用constraint类直接得到所有的状态偏导
       vector<vector<vector<double>>> l_x = l.get_state_first_derivatives(X,obs_center,obs_mat,t);
       vector<vector<vector<double>>> l_xx = l.get_state_second_derivatives(X,obs_center,obs_mat,t);
       vector<vector<vector<double>>> l_u = l.get_control_first_derivatives(U,t);
       vector<vector<vector<double>>> l_uu = l.get_control_second_derivatives(U,t);
       vector<vector<double>> l_ux(2,vector<double>(4,0));

       //获取model的各个状态量,调用model类的函数
       vector<double> vel = model.get_vel(X);
       vector<double> theta = model.get_theta(X);
       vector<double> acc = model.get_acc(U);
       //调用model类得到dynamic equation的偏导数信息
       vector<vector<vector<double>>> df_dx = model.get_A_matrix(vel,theta,acc);
       vector<vector<vector<double>>> df_du = model.get_B_matrix(theta);


       //返回最后一步的Value function的偏导数
       vector<vector<double>> V_x= l_x[n];
       vector<vector<double>> V_xx= l_xx[n];
       //为feeforward和feeback term预分配内存
       vector<vector<vector<double>>> k(n,vector<vector<double>>(2,vector<double>(1,0)));
       vector<vector<vector<double>>> K(n,vector<vector<double>>(2,vector<double>(4,0)));
       //为Q分配内存
       vector<vector<double>> Q_x(4,vector<double>(1,0));
       vector<vector<double>> Q_u(2,vector<double>(1,0));
       vector<vector<double>> Q_xx(4,vector<double>(4,0));
       vector<vector<double>> Q_uu(2,vector<double>(2,0));
       vector<vector<double>> Q_ux(2,vector<double>(4,0));
       vector<vector<double>> Q_uu_inv(2,vector<double>(2,0));
       //从N - 1一直到0计算状态
       for (int i = n - 1; i >= 0; --i) {
           Q_x = m.matrixAdd(l_x[i],m.matrixMultiply(m.matrixTranspose(df_dx[i]),V_x));
           Q_u = m.matrixAdd(l_u[i],m.matrixMultiply(m.matrixTranspose(df_du[i]),V_x));
           Q_xx = m.matrixAdd(l_xx[i],m.matrixMultiply(m.matrixMultiply(m.matrixTranspose(df_dx[i]),V_xx),df_dx[i]));
           Q_ux = m.matrixAdd(l_ux,m.matrixMultiply(m.matrixMultiply(m.matrixTranspose(df_du[i]),V_xx),df_dx[i]));
           Q_uu = m.matrixAdd(l_uu[i],m.matrixMultiply(m.matrixMultiply(m.matrixTranspose(df_du[i]),V_xx),df_du[i]));
           Q_uu_inv = m.matrixInv(Q_uu);
           k[i] = m.changeSign(m.matrixMultiply(Q_uu_inv,Q_u));
           K[i] = m.changeSign(m.matrixMultiply(Q_uu_inv,Q_ux));

           V_x = m.matrixSubtract(Q_x,m.matrixMultiply(m.matrixMultiply(m.matrixTranspose(K[i]),Q_uu),k[i]));
           V_xx = m.matrixSubtract(Q_xx,m.matrixMultiply(m.matrixMultiply(m.matrixTranspose(K[i]),Q_uu),K[i]));
       }
       ans.push_back(k);
       ans.push_back(K);
       return ans;

   }
   vector<vector<vector<double>>> get_k(vector<vector<vector<vector<double>>>> A) {
       return A[0];
   }
   vector<vector<vector<double>>> get_K(vector<vector<vector<vector<double>>>> A) {
       return A[1];
   }
  
};
class CiLQR {
 public:
  class iLQR ilqr;
  class Matrix m;
  class Model model;
  class L_Functions l;
  class Barrier_Function bar;
  vector<vector<vector<double>>> algorithm_cilqr(vector<double> X_0, vector<vector<double>> U,vector<vector<double>> obs_center,vector<vector<vector<double>>> obs_mat,double t_0,double u,double eps) {
    vector<vector<vector<double>>> ans;
    int obs_number = obs_center.size();//获取障碍物个数
    double t = t_0;
    int n = U.size();//Horizon N
    vector<vector<double>> X = ilqr.get_nomial_trajectory(X_0,U);
    vector<vector<double>> X_outer = X;
    vector<vector<double>> U_outer = U;
    vector<vector<double>> X_inner = X;
    vector<vector<double>> U_inner = U;
    vector<vector<double>> X_inner_new(n + 1,vector<double>(4,0));
    vector<vector<double>> U_inner_new(n,vector<double>(2,0));
    vector<vector<double>> X_outer_new(n + 1,vector<double>(4,0));
    vector<vector<double>> U_outer_new(n,vector<double>(2,0));
    while(1) {
     while(1) {
     vector<vector<vector<double>>> k = ilqr.get_k(ilqr.backward_pass(X,U,obs_center,obs_mat,t));
     vector<vector<vector<double>>> K = ilqr.get_K(ilqr.backward_pass(X,U,obs_center,obs_mat,t));
     //line search
     while(1) {
      double alpha = 1;
      X_inner_new = ilqr.get_X_new(ilqr.forward_pass(X,U,k,K,alpha));
      U_inner_new = ilqr.get_U_new(ilqr.forward_pass(X,U,k,K,alpha));
      vector<vector<double>> loc_new = model.get_loc(X_inner_new);
      if (l.check_control(U_inner_new) && bar.check_all_obs_constraint(loc_new,obs_center,obs_mat,t) && (l.cost_all(X_inner_new,U_inner_new,obs_center,obs_mat,t) <= l.cost_all(X_inner,U_inner,obs_center,obs_mat,t))) {
       break;
      }
      alpha /= 2;   
     }
     if (abs(l.cost_all(X_inner_new,U_inner_new,obs_center,obs_mat,t) - l.cost_all(X_inner,U_inner,obs_center,obs_mat,t)) < eps) {
       break;  
     } else {
        X_inner = X_inner_new;
        U_inner = U_inner_new;
     }
    }
    U_outer_new = U_inner_new;
    X_outer_new = X_inner_new;
    if (abs(l.cost_all(X_outer_new,U_outer_new,obs_center,obs_mat,t) - l.cost_all(X_outer,U_outer,obs_center,obs_mat,t)) < eps) {
      ans.push_back(X_outer_new);
      ans.push_back(U_outer_new);
      break;
    } else {
       t = u*t;
    }

    }
    return ans; 
  }
};

