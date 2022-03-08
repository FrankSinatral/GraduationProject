

#include<vector>
using std::vector;
double cost_acceleration(double w_acc,vector<double> control) {
    return w_acc*control[0]*control[0];
}
double cost_steeringangle(double w_steer,vector<double> control) {
    return w_steer*control[1]*control[1];
}
double cost_velocitytracking(double w_vel,vector<double> state,double v_ref) {
    return w_vel*(state[2]-v_ref)*(state[2] - v_ref);
}
double cost_refercencetracking(vector<double> state) {
    return state[1]*state[1];
}