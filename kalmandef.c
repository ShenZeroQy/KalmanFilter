//#include "kalmandef.h"
//#include"matrix.h"
////noise
//const float _Kalman_Q_data[__Kalman_X_demin*__Kalman_X_demin] =
//{ 0.005,0,0,0,0,0.01,0,0,0,0,0.005,0,0,0,0,0.01 };
//const float _Kalman_R_data[__Kalman_X_demin] =
//{ 10,0,0,10 };
//const float _Kalman_W_data[__Kalman_X_demin*__Kalman_X_demin] =
//{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
//
////initial state
//const float _Kalman_X0_data[__Kalman_X_demin] =
//{ -100,2,200,20 };
//const float _Kalman_P0_data[__Kalman_X_demin*__Kalman_X_demin] =
//{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
//
////system module
//const float _LKF_Module0_A_data[__Kalman_X_demin*__Kalman_X_demin] =
//{ 1,1,0,0,0,1,0,0,0,0,1,1,0,0,0,1 };
//const float _LKF_Module0_H_data[__Kalman_Y_demin*__Kalman_X_demin] =
//{ 1,0,0,0, 0,0,1,0 };
//
//
//Matrixd Kalman_X0;
//Matrixd Kalman_P0;
//
//
//
//
//Matrixd Kalman_Q;
//Matrixd Kalman_R;
//Matrixd Kalman_W;
//
////Kalman_X0.col = 1;
////Kalman_X0.row = __Kalman_X_demin;
////Kalman_X0.trans_flag = 0;
////Kalman_X0.data = _Kalman_X0_data;
////
////Kalman_P0.col = Kalman_P0.row = __Kalman_X_demin;
////Kalman_P0.trans_flag = 0;
////Kalman_P0.data = _Kalman_P0_data;
////
////Kalman_Q.col = Kalman_Q.row = __Kalman_X_demin;
////Kalman_Q.trans_flag = 0;
////Kalman_Q.data = _Kalman_Q_data;
////Kalman_R.col = Kalman_R.row = __Kalman_Y_demin;
////Kalman_R.trans_flag = 0;
////Kalman_R.data = _Kalman_R_data;
////Kalman_W.col = Kalman_W.row = __Kalman_X_demin;
////Kalman_W.trans_flag = 0;
////Kalman_W.data = _Kalman_W_data;
