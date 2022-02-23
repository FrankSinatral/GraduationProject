#include<iostream>
#include<vector>
#include<math.h>
using std::vector;
class Matrix
{
  public:
     vector<vector<int>> matrixAdd(vector<vector<int>> A, vector<vector<int>> B) {
       int m = A.size(), n = A[0].size();
       vector<vector<int>> C(m,vector<int>(n,0));
       for (int i =0; i < m; ++i) {
         for (int j = 0; j < n; ++j) {
            C[i][j] = A[i][j] + B[i][j];
        }
       }
        return C;
    }
     vector<vector<int>> matrixMultiply(vector<vector<int>> A, vector<vector<int>> B) {
         int m = A.size(), n = A[0].size(), k = B[0].size();
         vector<vector<int>> C(m,vector<int>(k,0));
         for (int i = 0; i < m; ++i) {
             for (int j = 0; j < k; ++j) {
                 for (int l = 0; l < n; ++l) {
                     C[i][j] += A[i][l]*B[l][j];
                 }
             }
         }
         return C;
     }
     vector<vector<int>> matrixTranspose(vector<vector<int>> A) {
         int m = A.size(), n = A[0].size();
         vector<vector<int>> B(n,vector<int>(m,0));
         for (int i = 0; i < m; ++i) {
             for (int j = 0; j < n; ++j) {
                 B[j][i] = A[i][j];
             }
         }
         return B;
     }
};
int main()
{
    vector<vector<int>> a;
    a = {{1,2}};
    vector<vector<int>> b;
    b = {{3,4}};
    vector<vector<int>> c;
    c = {{5},{6}};
    Matrix m;
    vector<vector<int>> d = m.matrixMultiply(m.matrixMultiply(m.matrixTranspose(a),b),c);
    std::cout  << d[0][0] <<" "<< d[1][0] << std::endl;
    return 0;
}

