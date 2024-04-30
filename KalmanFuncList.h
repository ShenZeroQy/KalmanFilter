#pragma once
#include"matrix.h"
/************************************************************/
//kf宏,用户根据需要修改


#define __Kalman_X_demin 3
#define __Kalman_Y_demin 1
#define __Kalman_N 3710

//预测器
#define KALMAN_PREDICT
#define KALMAN_PRE_STEP 4

//不带有控制输入的卡尔曼滤波
#define KALMAN_NO_CONTRAL

#define _LKF_MODULE_NUM_ 4 
//最大支持4个LKF MODULE

#define _EKF_MODULE_NUM_ 0  
//最大支持4个LKF MODULE

///************************************************************//
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//！！！用户根据模型去修改kalmanFuncList.c中函数！！！
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//！用户根据模型去修改kalmandef.h中参数！！！
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//！！！用户根据模型去修改kalmanFuncList.c中函数！！！
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//！用户根据模型去修改kalmandef.h中参数！！！
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//*************************************************************//


//不变的头
typedef void(*fn_p)(Matrixd* mat, Matrixd* fn_mat);

#if _LKF_MODULE_NUM_>0
void LKF_A_module0(Matrixd* X, Matrixd* A_X);
void LKF_H_module0(Matrixd* X, Matrixd* H_X);
#endif
#if _LKF_MODULE_NUM_>1
void LKF_A_module1(Matrixd* X, Matrixd* A_X);
void LKF_H_module1(Matrixd* X, Matrixd* H_X);
#endif
#if _LKF_MODULE_NUM_>2
void LKF_A_module2(Matrixd* X, Matrixd* A_X);
void LKF_H_module2(Matrixd* X, Matrixd* H_X);
#endif
#if _LKF_MODULE_NUM_>3
void LKF_A_module3(Matrixd* X, Matrixd* A_X);
void LKF_H_module3(Matrixd* X, Matrixd* H_X);
#endif

/*********************************/
#if _EKF_MODULE_NUM_>0
void EKF_A_module0(Matrixd* X, Matrixd* A_X);
void EKF_H_module0(Matrixd* X, Matrixd* H_X);
void EKF_Jac_A_module0(Matrixd* X, Matrixd* Jac_A_X);
void EKF_Jac_H_module0(Matrixd* X, Matrixd* Tac_H_X);
#endif
#if _EKF_MODULE_NUM_>1
void EKF_A_module1(Matrixd* X, Matrixd* A_X);
void EKF_H_module1(Matrixd* X, Matrixd* H_X);
void EKF_Jac_A_module1(Matrixd* X, Matrixd* Jac_A_X);
void EKF_Jac_H_module1(Matrixd* X, Matrixd* Tac_H_X);
#endif
#if _EKF_MODULE_NUM_>2
void EKF_A_module2(Matrixd* X, Matrixd* A_X);
void EKF_H_module2(Matrixd* X, Matrixd* H_X);
void EKF_Jac_A_module2(Matrixd* X, Matrixd* Jac_A_X);
void EKF_Jac_H_module2(Matrixd* X, Matrixd* Tac_H_X);
#endif
#if _EKF_MODULE_NUM_>3
void EKF_A_module3(Matrixd* X, Matrixd* A_X);
void EKF_H_module3(Matrixd* X, Matrixd* H_X);
void EKF_Jac_A_module3(Matrixd* X, Matrixd* Jac_A_X);
void EKF_Jac_H_module3(Matrixd* X, Matrixd* Tac_H_X);
#endif

//////////////////////////////////////////////////////////////////////



/////////////////////////////////////////////////////////////////////