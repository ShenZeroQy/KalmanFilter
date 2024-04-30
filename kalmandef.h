#pragma once
#include "matrix.h"
#include"KalmanFuncList.h"
//所有矩阵数据按行写
//该file只能被 include 一次
/***************************************************/
//额外的实验参数
#define Module34_A_impause 0.08
#define Module_T 0.01
#define Module34_D_impause 0.12
#define Obs_Nos_P 15
/**用户修改以下参数************/
//noise

const float _Kalman_Q_data[__Kalman_X_demin*__Kalman_X_demin] =
{ 0.5,0,0,0,0.9,0,0,0,0.8 };
const float _Kalman_R_data[__Kalman_Y_demin] =
{ Obs_Nos_P+10 };
const float _Kalman_Rr_data[__Kalman_Y_demin] =
{ Obs_Nos_P };
const float _Kalman_W_data[__Kalman_X_demin*__Kalman_X_demin] =
{ 1,0,0,0,1,0,0,0,1 };

//initial state
const float _Kalman_X0_data[__Kalman_X_demin] =
{ -400,-2,5 };
const float _Kalman_P0_data[__Kalman_X_demin*__Kalman_X_demin] =
{ 1,0,0,0,1,0,0,0,1 };

//system module
#if _LKF_MODULE_NUM_>0
const float _LKF_Module0_A_data[__Kalman_X_demin*__Kalman_X_demin] =
//{ 1 ,Module_T,Module_T*Module_T / 2,0,1,Module_T,0,0,1 };
{ 1 ,Module_T,0,0,1,0,0,0,1 };
const float _LKF_Module0_H_data[__Kalman_Y_demin*__Kalman_X_demin] =
{ 1,0,0 };
#endif
#if _LKF_MODULE_NUM_>1
const float _LKF_Module1_A_data[__Kalman_X_demin*__Kalman_X_demin] =
{ 1 ,Module_T,Module_T*Module_T / 2,0,1,Module_T,0,0,1 };
const float _LKF_Module1_H_data[__Kalman_Y_demin*__Kalman_X_demin] =
{ 1,0,0 };
#endif

#if _LKF_MODULE_NUM_>2
const float _LKF_Module2_A_data[__Kalman_X_demin*__Kalman_X_demin] =
{ 1 ,Module_T,Module_T*Module_T / 2*(1 + Module34_A_impause),
0,1,Module_T*(1+Module34_A_impause),
0,0,1+Module34_A_impause };
//{ 1, Module_T, (500*Module_T - 1 + exp(-500 * Module_T)) / (500*500),
//0, 1, (1 - exp(-500 * Module_T)) / 500,
//0, 0, exp(-500 * Module_T) };
const float _LKF_Module2_H_data[__Kalman_Y_demin*__Kalman_X_demin] =
{ 1,0,0 };
#endif
#if _LKF_MODULE_NUM_>3
const float _LKF_Module3_A_data[__Kalman_X_demin*__Kalman_X_demin] =
{ 1 ,Module_T,Module_T*Module_T / 2 * (1 - Module34_D_impause),
0,1,Module_T*(1 - Module34_D_impause),
0,0,1 - Module34_D_impause };
const float _LKF_Module3_H_data[__Kalman_Y_demin*__Kalman_X_demin] =
{ 1,0,0 };
#endif
//#if _LKF_MODULE_NUM_>2
//const float _LKF_Module2_A_data[__Kalman_X_demin*__Kalman_X_demin] =
//{ 1,Module_T,Module_T*Module_T / 2,0,1,Module_T,0,0,1, };
//const float _LKF_Module2_H_data[__Kalman_Y_demin*__Kalman_X_demin] =
//{ 1,0,0 };
//#endif

//mats declearation
Matrixd Kalman_X0;
Matrixd Kalman_P0;

Matrixd Kalman_Q;
Matrixd Kalman_R;
Matrixd Kalman_W;


/***************************************/
//用户将以下代码写入main
/*
Kalman_X0.col = 1;
Kalman_X0.row = __Kalman_X_demin;
Kalman_X0.trans_flag = 0;
Kalman_X0.data = _Kalman_X0_data;

Kalman_P0.col = Kalman_P0.row = __Kalman_X_demin;
Kalman_P0.trans_flag = 0;
Kalman_P0.data = _Kalman_P0_data;

Kalman_Q.col = Kalman_Q.row = __Kalman_X_demin;
Kalman_Q.trans_flag = 0;
Kalman_Q.data = _Kalman_Q_data;
Kalman_R.col = Kalman_R.row = __Kalman_Y_demin;
Kalman_R.trans_flag = 0;
Kalman_R.data = _Kalman_R_data;
Kalman_W.col = Kalman_W.row = __Kalman_X_demin;
Kalman_W.trans_flag = 0;
Kalman_W.data = _Kalman_W_data;
*/

