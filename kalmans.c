/*
 * �������˲��ļ�
 * ʵ���˲��㷨
 *
 * FileName : 	kalman.c
 * Author   : 	QYShen
 * Version  : 	v1.3
 * Date     : 	15/04/2021
 * Note	    : 	ĿǰLKF���ã��û��Լ��޸�Kalmandef.h��KalmanFuncList.c
 * 
 * Copyright (C) Murphys@IOE
 */
//#include"pch.h"
#include "kalmans.h"

 // IMM�㷨�������ñ����洢����VS V->det,S->Xfpre
void KF_init_LKF_init_module(KF* lkf,int modlue_num)//��ʼ������ָ��
{
	
	lkf->Jacbi_A = NULL;
	lkf->Jacbi_H = NULL;
	switch (modlue_num)
	{
#if _LKF_MODULE_NUM_>0
	case 0:
		lkf->A = &LKF_A_module0;
		lkf->H = &LKF_H_module0;
		break;
#endif
#if _LKF_MODULE_NUM_>1
	case 1:
		lkf->A = &LKF_A_module1;
		lkf->H = &LKF_H_module1;
		break;
#endif
#if _LKF_MODULE_NUM_>2
	case 2:
		lkf->A = &LKF_A_module2;
		lkf->H = &LKF_H_module2;
		break;
#endif
#if _LKF_MODULE_NUM_>3

	case 3:
		lkf->A = &LKF_A_module3;
		lkf->H = &LKF_H_module3;
		break;
#endif
	default:

		break;
	}
 
}
#ifdef KALMAN_NO_CONTRAL
void KF_init_LKF_init_space(KF* lkf,int X_dim,int Y_dim,int N)//��ʼ�ڴ�
{
	//lkf = (KF*)lkf;
	
	lkf->Sys_Dimen = X_dim;
	lkf->Obs_Dimen = Y_dim;
	lkf->len = N;
	lkf->index = 0;
	
#ifdef _IMM_VS
	lkf->IMM_inv_S = (Matrixd*)malloc(sizeof(Matrixd));
	Matrix_init(lkf->IMM_inv_S,1,1,'d');// storage S to Xfpre����-������������IMM 
#endif

	//��ʡ�ڴ棬������Sin��Sout
	lkf->Sin = (Matrixd*)malloc(sizeof(Matrixd));
	//Matrix_init(lkf->Sin, Y_dim, N,'d');
	Matrix_init(lkf->Sin, Y_dim,1 , 'd');
	lkf->Sout = (Matrixd*)malloc(sizeof(Matrixd));
	//Matrix_init(lkf->Sout, X_dim, N, 'd');
	Matrix_init(lkf->Sout, X_dim, 1, 'd');
	
	//lkf->filter = &LKF_fliter;
	//�˺���Ϊ�����˽�к��������ڳ���������⣬Ϊ�˽���ϣ�����selfָ�룬
	//û��ʹ��������selfָ�룬����Ӻ������ٹ�Ϊ���Ԫ�أ�������Ϊ�����ĺ�����
	//���ڸ߰汾C�����������Բ���ô�ֽ⣬��Ȼ���ຯ��˽�л���Ϊ��֧�ֹŶ����������ҽ�����롣

	//�˲�ʱ�õõ��ı���
	lkf->X= (Matrixd*)malloc(sizeof(Matrixd));
	Matrix_init(lkf->X, X_dim, 1, 'd');
	lkf->P = (Matrixd*)malloc(sizeof(Matrixd));
	Matrix_init(lkf->P, X_dim, X_dim, 'd');
	lkf->Xfpre = (Matrixd*)malloc(sizeof(Matrixd));
	Matrix_init(lkf->Xfpre, X_dim, 1, 'd');
	lkf->Pfpre = (Matrixd*)malloc(sizeof(Matrixd));
	Matrix_init(lkf->Pfpre, X_dim, X_dim, 'd');
	lkf->K = (Matrixd*)malloc(sizeof(Matrixd));
	Matrix_init(lkf->K, X_dim, X_dim, 'd');
	lkf->det = (Matrixd*)malloc(sizeof(Matrixd));
	Matrix_init(lkf->det, Y_dim, 1, 'd');
	lkf->tem = (Matrixd*)malloc(sizeof(Matrixd));
	Matrix_init(lkf->tem, 1, 1, 'd');
	//lkf->tem = (Matrixd*)malloc(sizeof(Matrixd));
	//Matrix_init(lkf->tem, X_dim, X_dim, 'd');
	lkf->old = (Matrixd*)malloc(sizeof(Matrixd));
	Matrix_init(lkf->old, X_dim, X_dim, 'd');
	lkf->I = (Matrixd*)malloc(sizeof(Matrixd));
	Matrix_init(lkf->I, X_dim, X_dim, 'i');
	//��Щ������Ϊprotect
	return;

}
void KF_release(KF* kf)//��������
{
	Matrix_release(kf->Sin);
	Matrix_release(kf->Sout);
	free(kf->Sin);
	free(kf->Sout);
	//ÿһ����Ҫ�黹.......

	//Matrix_release(kf->A);

	return;
}
void LKF_fliter(KF* Self, Matrixd* Xpre, Matrixd* Ppre, Matrixd*Obs)
{
	Cal_state S = 0;
	static int cnt;
	//printf("processing:%d\n",cnt++);
	//Matrix_show(Xpre);

	Self->A(Xpre, Self->Xfpre);
	//Matrix_show(Self->Xfpre);
	//ɧ����
	Self->A(Ppre, Self->Pfpre);
	Matrix_transpose(Self->Pfpre);
	Self->A(Self->Pfpre, Self->tem);
	Matrix_transpose(Self->tem);
	Matrix_copy(Self->tem, Self->Pfpre);
	//ɧ����
	if ((S = Matrix_mul_Matrtix(Self->W, Self->Q, Self->tem))!=1)
		printf("KF_err,Mat_mulW_Q;");
	
	Matrix_transpose(Self->W);//���Σ��
	Matrix_mul_Matrtix(Self->tem, Self->W, Self->old);
	Matrix_transpose(Self->W);//Σ�����
	if ((S =Matrix_add(Self->Pfpre, Self->old) ) != 1)
		printf("KF_err,Mat_add Pfpre+old;");
	//�ȴ󺣸�����������
	Self->H(Self->Pfpre, Self->tem);
	Matrix_transpose(Self->tem);
	Self->H(Self->tem, Self->old);
	Matrix_transpose(Self->old);
	if ((S = Matrix_add(Self->old, Self->R)) != 1)
		printf("KF_err,Mat_add H*Pfpre*H^T+R;");
	//��������
	if ((S =MatrixS_inverse(Self->old, Self->tem,&(Self->IMM_det_S)) ) != 1)
		printf("KF_err,Mat_inv ;");
#ifdef _IMM_VS
	Matrix_copy(Self->tem, Self->IMM_inv_S);//storage inv_S����-������������IMM
#endif
	//printf("computed inverse done\n");
	//Matrix_show(Self->tem);
	//Matrix_show(Self->Pfpre);
	Self->H(Self->Pfpre, Self->old);
	Matrix_transpose(Self->tem);
	if ((S = Matrix_mul_Matrtix(Self->tem, Self->old, Self->K)) != 1)
		printf("KF_err,Mat_mul K;");
	Matrix_transpose(Self->K);

	//Matrix_show(Self->K);
	
	//�������
	Self->H(Self->Xfpre, Self->tem);
	Matrix_mul_num(Self->tem, -1);
	Matrix_copy(Obs, Self->det);
	if ((S = Matrix_add(Self->det, Self->tem) ) != 1)//det as V of _______________________IMM
		printf("KF_err,Matadd det;");
	
	//��з�
	Matrix_mul_Matrtix(Self->K, Self->det, Self->tem);
	Matrix_add(Self->tem, Self->Xfpre);
	Matrix_copy(Self->tem, Self->X);
	
	//Matrix_Assign_col(Self->sout,index,X)
	Self->index += 1;
	//ת�ñ�copy��ö�
	//��������
	Self->H(Self->I, Self->tem);
	//�������ͷ
	if ((S = Matrix_mul_Matrtix(Self->K, Self->tem, Self->old)) != 1)
		printf("KF_err,Mat_mul K_tem;");
	Matrix_mul_num(Self->old, -1);
	Matrix_add(Self->old, Self->I);
	if ((S =Matrix_mul_Matrtix(Self->old, Self->Pfpre, Self->P) ) != 1)
		printf("KF_err,Mat_mul P;");
	//��
	return;
}




	
#else
void KF_init_LKF_init_space(KF* lkf, int X_dim, int Y_dim, int N, Matrixd* drive)
{
	return;
}
void KF_init_set_contral_prameter_QRW(KF* kf, int D_dim, Matrixd* B_addr)
{
	return;
}
void KF_release(KF* kf)
{
	return;
}
void LKF_fliter(KF* self,Matrixd* Xpre, Matrixd* Ppre, Matrixd*Obs, Matrixd* Con)
{
	return;
}

#endif

void KF_init_set_noise_prameter_QRW(KF* kf, Matrixd* Q_addr, Matrixd* R_addr, Matrixd* W_addr)
{
	kf->Q = Q_addr;
	kf->R = R_addr;
	kf->W = W_addr;
	return;
}