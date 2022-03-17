#pragma once
#include<vector>
using std::vector;
class Matrix
{
  public:
    vector<vector<double>> matrixAdd(vector<vector<double>> A, vector<vector<double>> B);
    vector<vector<double>> matrixSubtract(vector<vector<double>> A, vector<vector<double>> B);
    vector<vector<double>> matrixMultiply(vector<vector<double>> A, vector<vector<double>> B);
    vector<vector<double>> matrixTranspose(vector<vector<double>> A);
    double matrixDeterminant(vector<vector<double>> A, int n);
    vector<vector<double>> matrixInv(vector<vector<double>> A);
    vector<double> vectorAdd(vector<double> A,vector<double> B);
    vector<double> vectorSubtract(vector<double> A,vector<double> B);
    vector<vector<double>> vecToMat(vector<double> A);//{1,2,3} -> {{1,2,3}}
    vector<double> matToVec(vector<vector<double>> A);//{{1,2,3}} -> {1,2,3}
    vector<vector<double>> changeSign(vector<vector<double>> A);
    vector<double> columnToRow(vector<vector<double>> A);
    vector<double> scaleVec(vector<double> A,double alpha);
    vector<vector<double>> congruentTrans(vector<vector<double>> A,vector<vector<double>> P);
    vector<vector<vector<double>>> matrixAdd_2(vector<vector<vector<double>>> A,vector<vector<vector<double>>> B);
};


