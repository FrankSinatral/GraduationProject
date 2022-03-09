#include<iostream>
#include<vector>
#include "matrix.h"

#include<math.h>
 
int main()
{
    int sum = 0;
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 3; ++j) {
        if (i + j >= 4) {
          break;
        }
        sum +=(i + j);
      }
    }
    std::cout << sum << std::endl;
    return 0;
}