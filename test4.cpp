#include<vector>
#include<fstream>
using std::vector;
#include"vehicle_model.h"
#include"iLQR1.h"
#include"cost_function1.h"
#include"matrix.h"
extern Model model;
extern iLQR ilqr;
extern L_Functions l;
int main() {
 vector<double> X_0 = {0,0,4,0};
 l.set_par(1,1,10,1,10);
 model.set_pra(0.2);
 vector<double> U_0 = {1,0.1};
 vector<double> U_1 = {0,-0.1};
 vector<double> U_2 = {0,0};
 vector<vector<double>> U;
 double eps = 0.1;
 for (int i = 0; i < 10; ++i) {
  U.push_back(U_0);
 }
 for (int i = 0; i < 10; ++i) {
 U.push_back(U_1);
 }
 for (int i = 0; i < 20; ++i) {
  U.push_back(U_2);
 }
 vector<vector<vector<double>>> result = ilqr.algorithm_ilqr(X_0,U,eps);
 vector<vector<double>> X_desired = result[0];
 vector<vector<double>> U_desired = result[1];
 std::ofstream fout("test4.csv");
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