#pragma once
#include "matrix.h"
#include"imm.h"
float TP_data[IMM_Module_Num*IMM_Module_Num] = 
{ 0.8,0.02,0.08,0.1,
	0.1,0.6,0.2,0.1,
	0.25,0.15,0.5,0.1,
	0.07,0.01,0.02,0.9 };
	/*{0.9,0.05,0.015,0.035,
	0.05,0.9,0.01,0.04,
	0.05,0.049,0.9,0.001,
	0.01,0.07,0.02,0.9};*/
float U0_data[IMM_Module_Num] = 
	{ 0.1,0.3,0.3,0.3 };

//const float TP_data[IMM_Module_Num*IMM_Module_Num] = 
//	{ 0.2,0.5,0.15,
//	0.5,0.3,0.1,
//	0.05,0.05,0.6,
//	};
//float U0_data[IMM_Module_Num] = 
//	{ 0.1,0.3,0.3 };
//const float TP_data[IMM_Module_Num*IMM_Module_Num] =
//{ 0.9,0.1,
//0.05,0.95};
//float U0_data[IMM_Module_Num] =
//{ 0.1,0.9};

Matrixd TP;
Matrixd U0;

//用户在主函数写以下内容
/*TP.row = IMM_Module_Num;
TP.col = IMM_Module_Num;
TP.trans_flag = 0;
TP.data = TP_data;
U0.row = 1;
U0.col = IMM_Module_Num;
U0.trans_flag = 0;
U0.data = U0_data;*/