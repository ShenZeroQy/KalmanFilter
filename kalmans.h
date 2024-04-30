#pragma once
/*
 * kalman.h
 *
 * FileName : 	kalman.h
 * Author   : 	QYShen
 * Version  : 	v0.0
 * Date     : 	2021-03-29
 * Note	    : 	
 * 
 * Copyright (C) Murphys@IOE
 */
#include "KalmanFuncList.h"

#define _IMM_VS 
//利用闲置空间
#ifndef	_KALMAN_H_
#define	_KALMAN_H_
#endif
//
// x = Ax + Bd + w  meaning the state vector x evolves during one time
// 
// z = Hx + v       meaning the observation vector z is a linear function
// 
// where w ~ N(0,Q) meaning w is gaussian noise with covariance Q
//       v ~ N(0,R) meaning v is gaussian noise with covariance R
//
// Do not define any system input (control) functions:
// 	
// 	u = 0;
#ifdef _KALMAN_H_
#ifdef KALMAN_NO_CONTRAL
typedef void(*fliter_p)(void* self,Matrixd* Xpre, Matrixd* Ppre, Matrixd*Obs);
#else
typedef void(*fliter_p)(KF* self,Matrixd* Xpre, Matrixd* Ppre, Matrixd*Obs, Matrixd* Con);
#endif

//typedef struct
//{
//	#ifdef KALMAN_NO_CONTRAL
//	const char No_Control_input_flag=1;
//	
//
//	#else//预留待扩
//	 char No_Control_input_flag;
//	 Matrixd* Control ;//as input
//	 int Con_Dimen; 
//	 fn_p B;
//	#endif
//	int Sys_Dimen;
//	int Obs_Dimen;
//	
//	fn_p A;	
//	fn_p H;
//	Matrixd* Sin;// as input 
//	Matrixd* Sout;// as output
//	Matrixd* Q;
//	Matrixd* R;
//	Matrixd* W;
//	
//	fliter_p filter;
//
//}KF;
typedef struct
{
	int Sys_Dimen;
	int Obs_Dimen;
	int len;
	int index;
	fn_p A;
	fn_p H;

	//only for EKF//*
	fn_p Jacbi_A;
	fn_p Jacbi_H;
	///////////////*

	//only for IMM//*
	float IMM_det_S;
	Matrixd* IMM_inv_S;
	///////////////*

	Matrixd* Sin;// as input ,read only;
	Matrixd* Sout;// as output ,read only;
	Matrixd* Q;
	Matrixd* R;
	Matrixd* W;

	//以下设为protect
	//////////
	Matrixd* X;
	Matrixd* P;
	Matrixd* Xfpre;
	Matrixd* Pfpre;
	Matrixd* K;
	Matrixd* det;
	Matrixd* tem;
	Matrixd* old;
	Matrixd* I;
	//////////
}KF;

#endif

/*********************/
//func declearation
void KF_init_LKF_init_module(KF* lkf, int m_n);
void KF_init_set_noise_prameter_QRW(KF* kf, Matrixd* Q_addr, Matrixd* R_addr, Matrixd* W_addr);
#ifdef KALMAN_NO_CONTRAL

void KF_init_LKF_init_space(KF* lkf, int X_dim, int Y_dim, int N);
void LKF_fliter(KF* self, Matrixd* Xpre, Matrixd* Ppre, Matrixd*Obs);
#else
void KF_init_LKF_init_space(KF* lkf, int X_dim, int Y_dim, int N, Matrixd* drive);
void KF_init_set_contral_prameter_QRW(KF* kf, int D_dim, Matrixd* B_addr);

void LKF_fliter(Matrixd* Xpre, Matrixd* Ppre, Matrixd*Obs, Matrixd* Con);

#endif




