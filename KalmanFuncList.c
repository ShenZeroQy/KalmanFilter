#include "KalmanFuncList.h"

//for vxworks


#if _LKF_MODULE_NUM_>0
extern const float _LKF_Module0_A_data[];
extern const float _LKF_Module0_H_data[];
#endif
#if _LKF_MODULE_NUM_>1
extern const float _LKF_Module1_A_data[];
extern const float _LKF_Module1_H_data[];
#endif
#if _LKF_MODULE_NUM_>2
extern const float _LKF_Module2_A_data[];
extern const float _LKF_Module2_H_data[];
#endif
#if _LKF_MODULE_NUM_>3
extern const float _LKF_Module3_A_data[];
extern const float _LKF_Module3_H_data[];
#endif
//用户修改函数体
///////////////////////////////////////////////////
//KF
#if _LKF_MODULE_NUM_>0
void LKF_A_module0(Matrixd* X, Matrixd* A_X)
{
	static Matrixd A;
	//static const float A_data[] =
	//{ 1,Module_T,Module_T*Module_T / 2,0,1,Module_T,0,0,1, };
	A.row = __Kalman_X_demin;
	A.col = __Kalman_X_demin;
	//A.data = A_data;
	A.data = _LKF_Module0_A_data;
	//Matrix_show(&A);
	//*(X->data + 1) -= 0.01;
	Matrix_mul_Matrtix(&A, X, A_X);
	return;
}
void LKF_H_module0(Matrixd* X, Matrixd* H_X)
{
	static Matrixd H;
	//static const float H_data[] =
	//{ 1,0,0 };
	H.row = __Kalman_Y_demin;
	H.col = __Kalman_X_demin;
	H.data = _LKF_Module0_H_data;
	//Matrix_show(&H);
	Matrix_mul_Matrtix(&H, X, H_X);
	
	return;
}
#endif
#if _LKF_MODULE_NUM_>1
void LKF_A_module1(Matrixd* X, Matrixd* A_X)
{
	static Matrixd A;
	A.row = __Kalman_X_demin;
	A.col = __Kalman_X_demin;
	A.data = _LKF_Module1_A_data;
	//*(X->data + 1) += 0.01;
	//Matrix_show(&A);
	Matrix_mul_Matrtix(&A, X, A_X);
	return;
}
void LKF_H_module1(Matrixd* X, Matrixd* H_X)
{
	static Matrixd H;
	H.row = __Kalman_Y_demin;
	H.col = __Kalman_X_demin;
	H.data = _LKF_Module1_H_data;
	//Matrix_show(&H);
	Matrix_mul_Matrtix(&H, X, H_X);
	return;
}
#endif
#if _LKF_MODULE_NUM_>2
void LKF_A_module2(Matrixd* X, Matrixd* A_X)
{
	static Matrixd A;
	A.row = __Kalman_X_demin;
	A.col = __Kalman_X_demin;
	A.data = _LKF_Module2_A_data;
	//Matrix_show(&A);
	//*(X->data + 2) -= 0.01;
	Matrix_mul_Matrtix(&A, X, A_X);
	return;
}
void LKF_H_module2(Matrixd* X, Matrixd* H_X)
{
	static Matrixd H;

	H.row = __Kalman_Y_demin;
	H.col = __Kalman_X_demin;
	H.data = _LKF_Module2_H_data;
	//Matrix_show(&H);
	Matrix_mul_Matrtix(&H, X, H_X);
	return;
}
#endif
#if _LKF_MODULE_NUM_>3
void LKF_A_module3(Matrixd* X, Matrixd* A_X)
{
	static Matrixd A;
	A.row = __Kalman_X_demin;
	A.col = __Kalman_X_demin;
	A.data = _LKF_Module3_A_data;
	//Matrix_show(&A);
	//*(X->data + 2) -= 0.01;
	Matrix_mul_Matrtix(&A, X, A_X);
	return;
}
void LKF_H_module3(Matrixd* X, Matrixd* H_X)
{
	static Matrixd H;

	H.row = __Kalman_Y_demin;
	H.col = __Kalman_X_demin;
	H.data = _LKF_Module3_H_data;
	//Matrix_show(&H);
	Matrix_mul_Matrtix(&H, X, H_X);
	return;
}
#endif
/*****************************************************/
//EKF
#if _EKF_MODULE_NUM_>0
void EKF_A_module0(Matrixd* X, Matrixd* A_X)
{

}
void EKF_H_module0(Matrixd* X, Matrixd* H_X)
{

}
void EKF_Jac_A_module0(Matrixd* X, Matrixd* Jac_A_X)
{

}
void EKF_Jac_H_module0(Matrixd* X, Matrixd* Tac_H_X)
{

}
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

////////////////////////////////////
//EKF


/////////////////////////////////
//UKF