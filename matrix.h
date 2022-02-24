#include<iostream>
#include<vector>

using namespace std;
class Matrix
{
  public:
    vector<vector<double>> matrixAdd(vector<vector<double>> A, vector<vector<double>> B);
    vector<vector<double>> matrixMultiply(vector<vector<double>> A, vector<vector<double>> B);
    vector<vector<double>> matrixTranspose(vector<vector<double>> A);
    double matrixDeterminant(vector<vector<double>> A, int n);
    vector<vector<double>> matrixInv(vector<vector<double>> A);
};
vector<vector<double>> Matrix::matrixAdd(vector<vector<double>> A, vector<vector<double>> B) {
  int m = A.size(), n = A[0].size();
  vector<vector<double>> C(m,vector<double>(n,0));
  for (int i =0; i < m; ++i) {
    for (int j = 0; j < n; ++j) {
      C[i][j] = A[i][j] + B[i][j];
    }
  }
  return C;
}
vector<vector<double>> Matrix::matrixMultiply(vector<vector<double>> A, vector<vector<double>> B) {
  int m = A.size(), n = A[0].size(), k = B[0].size();
  vector<vector<double>> C(m,vector<double>(k,0));
  for (int i = 0; i < m; ++i) {
    for (int j = 0; j < k; ++j) {
      for (int l = 0; l < n; ++l) {
        C[i][j] += A[i][l]*B[l][j];
      }
    }
  }
  return C;
}
vector<vector<double>> Matrix::matrixTranspose(vector<vector<double>> A) {
  int m = A.size(), n = A[0].size();
    vector<vector<double>> B(n,vector<double>(m,0));
      for (int i = 0; i < m; ++i) {
        for (int j = 0; j < n; ++j) {
          B[j][i] = A[i][j];
        }
      }
  return B;
}
double Matrix::matrixDeterminant(vector<vector<double>> A, int n) {
  if (n == 1) {
    return A[0][0];
    }
  double ans = 0;
  vector<vector<double>> temp(n - 1,vector<double>(n - 1,0));
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n - 1; ++j) {
      for (int k = 0; k < n - 1;++k) {
        temp[j][k] = A[j + 1][(k>=i)? k + 1:k];
      }
    }
    double t = matrixDeterminant(temp,n - 1);
    if (i%2 == 0) {
      ans += A[0][i]*t;
    } else {
      ans -= A[0][i]*t;
    }
  }
  return ans;
}
vector<vector<double>> Matrix::matrixInv(vector<vector<double>> A) {
  int n = A.size();
  double Det = matrixDeterminant(A,n);
  double tempDet = 0;
  if (Det == 0) {
    std::cout << "There is no such inverse matrix" << std::endl;
  }
  vector<vector<double>> ans(n,vector<double>(n,0));
  vector<vector<double>> temp(n - 1,vector<double>(n - 1,0));
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      for (int s = 0; s < n -1;++s) {
        for (int t = 0; t < n - 1;++t) {
          temp[s][t] = A[(s>=j)?s+1:s][(t>=i)?t+1:t];
        }
      }
      tempDet = matrixDeterminant(temp,n - 1);
      if ((i - j)%2 ==0){
       ans[i][j] = tempDet/Det;
      } else {
        ans[i][j] = -tempDet/Det;
      }
    }
  }
  return ans;
}
