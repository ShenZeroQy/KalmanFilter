#pragma once
#include "kalmans.h"
#include<stdio.h>
#include<math.h>
#define IMM_Module_Num (_LKF_MODULE_NUM_+_EKF_MODULE_NUM_)
typedef void(*Fn_Gama_p)(Matrixd* V, Matrixd* inv_S, float det_S, float* ret);

typedef void(*KF_Procces_p)(void* KF, Matrixd* Xpre, Matrixd* Ppre, Matrixd*Obs);

void Imm_gama(Matrixd* V, Matrixd* inv_S,float det_S ,float* ret);
typedef struct
{
	Matrixd* C;//
			   //
	Matrixd* U;//
			   //
	Matrixd* TU;//
				  //
	Matrixd* TP;//
	Fn_Gama_p Gama; //
	float Gama_ret[IMM_Module_Num];//
	KF* filter;

	KF_Procces_p lkf_p;
	KF_Procces_p ekf_p;

	Matrixd* SP;
	Matrixd* SX;


	int Module_num;
	
	Matrixd* tem;
	Matrixd* old;
}IMM;

void IMM_init_set_parm(IMM* imm, KF* K_base, Matrixd* TP_addr, int Mn);
void IMM_init_IMM_init_space(IMM* imm, int KF_X_dim);
void IMM_release(IMM* imm);
void IMM_process(IMM* imm, Matrixd* XSpre, Matrixd* PSpre, Matrixd*Obs);

