#include<vector>
using std::vector;
class Constraints{
    double w_acc,w_steer,w_vel,w_ref;//各项权重
    int N;//Horizon
    double t;//采样时间
    //vector<vector<vector<double>>> l_ux(N,vector<vector<double>>(2,vector<double>(4,0))); //事实上，l_ux即为2*4的零矩阵
    //首先求cost function对于control的一二阶偏导数
    vector<vector<vector<double>>> get_control_first_derivatives(vector<vector<double>> control,double a_high, double a_low, double delta_bar) {
        vector<vector<vector<double>>> l_u(N,vector<vector<double>>(2,vector<double>(1,0)));
        for (int i = 0; i < N; ++i) {
            l_u[i] = {{2*w_acc*control[i][0] + 1/(t*(a_high - control[i][0]))-1/(t*(control[i][0] - a_low))},
            {2*w_steer*control[i][1] + 1/(t*(delta_bar - control[i][1])) - 1/(t*(delta_bar + control[i][1]))}};
        }
        return l_u;
    }
    vector<vector<vector<double>>> get_control_second_derivatives(vector<vector<double>> control,double a_high, double a_low, double delta_bar) {
        vector<vector<vector<double>>> l_uu(N,vector<vector<double>>(2,vector<double>(2,0)));
        for (int i = 0; i < N; ++i) {
            l_uu[i] = {{2*w_acc + 1/(t*(a_high - control[i][0])*(a_high - control[i][0])) + 1/(t*(control[i][0] - a_low)*(control[i][0] - a_low)),0},
            {0,2*w_steer + 1/(t*(delta_bar - control[i][1])*(delta_bar - control[i][1])) + 1/(t*(delta_bar + control[i][1])*(delta_bar + control[i][1]))}};
        }
        return l_uu;
    }
    //下面求cost function对于state的一二阶偏导数
    //l_x是一个4*1向量，0,1...N个状态
    vector<vector<vector<double>>> get_state_first_derivatives(vector<vector<double>> state) {
        vector<vector<vector<double>>> l_x(N + 1,vector<vector<double>>(4,vector<double>(1,0)));
        for (int i = 0; i < N + 1; ++i) {
            l_x[i] = {{0},{2*w_ref*state[i][1]},{2*w_vel*state[i][2]},{0}};//此处少了涉及一个位置信息相关的函数求导
        }
        return l_x;
    }
    //l_xx是一个4*4向量，0,1...N个状态 
    vector<vector<vector<double>>> get_state_second_derivatives() {
        vector<vector<vector<double>>> l_xx(N + 1,vector<vector<double>>(4,vector<double>(4,0)));
        for (int i = 0; i < N + 1; ++i) {
            l_xx[i] = {{0,0,0,0},{0,2*w_ref,0,0},{0,0,2*w_vel,0},{0,0,0,0}};//此处少了涉及一个位置信息相关的函数求导
        }
        return l_xx;
    }
    
};