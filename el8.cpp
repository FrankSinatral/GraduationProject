#include"constraints.h"
#include"matrix.h"
#include"barrier_function.h"
#include"vehicle_model.h"
#include"iLQR.h"
#include"cost_function.h"
Model model;
Obstacle obs_1;
Obstacle obs_2;
Obstacle obs_3;
Matrix m;
Barrier_Function bar;
iLQR ilqr;
L_Functions l;