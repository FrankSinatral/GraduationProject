#include"matrix.h"
#include<iostream>
vector<vector<vector<double>>> Matrix::matrixAdd_2(vector<vector<vector<double>>> A, vector<vector<vector<double>>> B) {
  int m = A.size();
  int n = A[0].size();
  int t = A[0][0].size();
  vector<vector<vector<double>>> ans(m,vector<vector<double>>(n,vector<double>(t,0)));
  for (int i = 0; i < m; ++i) {
    for (int j = 0; j < n; ++j) {
      for (int s = 0; s < t; ++s) {
        ans[i][j][s] = A[i][j][s] + B[i][j][s];
      }
    }
  }
  return ans;
}
//实现矩阵的合同变换
vector<vector<double>> Matrix::congruentTrans(vector<vector<double>> A,vector<vector<double>> P) {
  vector<vector<double>> ans = matrixMultiply(matrixMultiply(matrixTranspose(P),A),P);
  return ans;
}
//将一个行向量统一乘以一个标量alpha
vector<double> Matrix::scaleVec(vector<double> A,double alpha) {
  int m = A.size();
  vector<double> ans(m,0);
  for (int i = 0; i < m;++i) {
    ans[i] = alpha*A[i];
  }
  return ans;
}
//将一个列向量（矩阵）转化为行向量
vector<double> Matrix::columnToRow(vector<vector<double>> A) {
  int m = A.size();
  vector<double> ans(m,0);
  for (int i = 0; i < m; ++i) {
    ans[i] = A[i][0]; 
  }
  return ans;
}
vector<vector<double>> Matrix::matrixSubtract(vector<vector<double>> A, vector<vector<double>> B) {
  int m = A.size(), n = A[0].size();
  vector<vector<double>> C(m,vector<double>(n,0));
  for (int i =0; i < m; ++i) {
    for (int j = 0; j < n; ++j) {
      C[i][j] = A[i][j] - B[i][j];
    }
  }
  return C;
}
vector<vector<double>> Matrix::changeSign(vector<vector<double>> A) {
  int m = A.size(), n = A[0].size();
  vector<vector<double>> ans(m,vector<double>(n,0));
  for (int i = 0; i < m; ++i) {
    for (int j = 0; j < n; ++j) {
      ans[i][j] = -A[i][j];
    }
  }
  return ans;
}
vector<double> Matrix::matToVec(vector<vector<double>> A) {
  int m = A[0].size();
  vector<double> ans(m,0);
  for (int i = 0; i < m; ++i) {
    ans[i] = A[0][i];
  }
  return ans;
}
//{1,2,3} -> {{1,2,3}}
vector<vector<double>> Matrix::vecToMat(vector<double> A) {
  int m = A.size();
  vector<vector<double>> ans(1,vector<double>(m,0));
  for (int i = 0; i < m; ++i) {
    ans[0][i] = A[i];
  }
  return ans;
}
vector<double> Matrix::vectorAdd(vector<double> A,vector<double> B) {
  int m = A.size();
  vector<double> C(m,0);
  for (int i = 0; i < m; ++i) {
    C[i] = A[i] + B[i];
  }
  return C;
}
vector<double> Matrix::vectorSubtract(vector<double> A,vector<double> B) {
  int m = A.size();
  vector<double> C(m,0);
  for (int i = 0; i < m; ++i) {
    C[i] = A[i] - B[i];
  }
  return C;
}
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

