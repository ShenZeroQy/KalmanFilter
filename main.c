#include "general.h"
#include"immdef.h"
#include"kalmandef.h"
#include"imm.h"
#include"guass.h"



int main(char argc, char* argv)
{

	//variables
	float ftem;
	int i,j;
	int k;
	//kf
	KF kf[_LKF_MODULE_NUM_];
	FILE* FPin=NULL;
	FILE* FPobs = NULL;
	FILE*FPKFres = NULL;
	FILE*IMM_rec1 = NULL;
	FILE*IMM_rec2 = NULL;
	FILE*IMM_ana = NULL;
	FILE*IMM_rec3 = NULL;
	FILE*IMM_rec4 = NULL;
	char Filename[] = { 'd','a','t','a', '/','k', 'f','i','.','t','x','t','\0' };
	char n = '1';

	static Matrixd H;
	static Matrixd A;
	Matrixd system[__Kalman_N ];
	Matrixd observe[__Kalman_N];
	Matrixd tem;
	
	float segmaQ[__Kalman_X_demin] = { (0.0005),(0.0001),(0.0005), };
	float segmaR[__Kalman_Y_demin] = { (Obs_Nos_P)};
	
	
	//imm
	IMM imm;
	//mats
	Matrixd Gama_outer;
	Matrixd mat2;
	Matrixd mat3;
	Matrixd* matp;
	
	
	//variables end
	static int		uselessflag = 0;
	static int		mat_flag=0;
	static int		kf_flag=0;
	static int		imm_flag=0;
		if(uselessflag == 0){
			printf(" Qyshen start to work\n");
			uselessflag = 1;
		}
	//printf(" Qyshen start to work\n");
	
	if(kf_flag==0)
	{
		Matrix_init(&Gama_outer, 1, IMM_Module_Num, 'd');
		Matrix_init(&mat2, IMM_Module_Num*IMM_Module_Num, 1, '0');
		kf_flag = 1;
		
		
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
		Kalman_R.data = _Kalman_Rr_data;
		Kalman_W.col = Kalman_W.row = __Kalman_X_demin;
		Kalman_W.trans_flag = 0;
		Kalman_W.data = _Kalman_W_data;
		printf("show:Q R W X_0 P_0\n");
		Matrix_show(&Kalman_Q);
		Matrix_show(&Kalman_R);
		Matrix_show(&Kalman_W);
		Matrix_show(&Kalman_X0);
		Matrix_show(&Kalman_P0);
		for (i = 0; i < _LKF_MODULE_NUM_; i++)
		{
			KF_init_LKF_init_module(kf + i, i);
			KF_init_LKF_init_space(kf + i, __Kalman_X_demin, __Kalman_Y_demin, __Kalman_N);

			KF_init_set_noise_prameter_QRW(kf + i, &Kalman_Q, &Kalman_R, &Kalman_W);


			Matrix_copy(&Kalman_X0, (kf + i)->X);
			Matrix_copy(&Kalman_P0, (kf + i)->P);
		}
		A.row = 3;
		A.col = 3;
		A.data = _LKF_Module0_A_data;
		A.trans_flag = 0;
		H.row = 1;
		H.col = 3;
		H.trans_flag = 0;
		H.data = _LKF_Module0_H_data;

		Matrix_init(&tem, 1, 1, 'd');
		for (i = 0; i < __Kalman_N; i++)
		{
			Matrix_init(system + i, __Kalman_X_demin, 1, '0');
			Matrix_init(observe + i, __Kalman_Y_demin, 1, '0');
		}
		Matrix_copy(&Kalman_X0, system);//initial value
		Matrix_mul_Matrtix(&H, &Kalman_X0, observe);
		//load system data
		//FILE pointer
		fopen_s(&FPin, "data/exp_trace.txt", "r");
		fopen_s(&FPobs, "data/observe.txt", "w");
		for (i = 0; i < __Kalman_N; i++)
		{
			//Matrix_Pointer_Assign_Notrans_Element(system + i, 0, 0) = 0;
			//fscanf_s(FPin, (system + i)->data);
			Matrix_load(system + i, FPin, __Kalman_X_demin);
			//Matrix_show(system + i);
		}
		for (i = 0; i < __Kalman_N; i++)
		{
			/*Matrix_mul_Matrtix(&A, system + i - 1, &tem);
			for (j = 0; j < __Kalman_X_demin; j++)
				*(tem.data + j) += Guass_noise(*(segmaQ + j));
			Matrix_copy(&tem, system + i);*/
			Matrix_mul_Matrtix(&H, system + i , &tem);
			for (j = 0; j < __Kalman_Y_demin; j++)
				*(tem.data + j) += Guass_noise(*(segmaR + j));
			Matrix_copy(&tem, observe + i);
			//printf("i=%d\n", i);
			//Matrix_show(observe + i);
			Matrix_save(observe + i, FPobs,'d');
		}
		fclose(FPobs);
		fclose(FPin);
		
		
		for (k='0',i=0; i<2; k++,i++)//2个滤波器作为对照
		{
			Filename[7] = k;
			//printf(Filename);
			fopen_s(&FPKFres, Filename, "w");
			for (j = 0; j < __Kalman_N; j++)
			{
				LKF_fliter(kf+i, (kf+i)->X, (kf+i)->P, observe + j);
				Matrix_save((kf+i)->X, FPKFres, 'd');
			}	
			fclose(FPKFres);
		}
		Kalman_R.data = _Kalman_R_data;
		for (i = 0; i < _LKF_MODULE_NUM_; i++)
		{
			//reinit kf
			KF_init_set_noise_prameter_QRW(kf + i, &Kalman_Q, &Kalman_R, &Kalman_W);
			Matrix_copy(&Kalman_X0, (kf + i)->X);
			Matrix_copy(&Kalman_P0, (kf + i)->P);
		}
		//IMM init
		TP.row = IMM_Module_Num;
		TP.col = IMM_Module_Num;
		TP.trans_flag = 0;
		TP.data = TP_data;
		U0.row = 1;
		U0.col = IMM_Module_Num;
		U0.trans_flag = 0;
		U0.data = U0_data;
		Matrix_show(&TP);
		

		//reopen
		fopen_s(&FPKFres, "data/imm.txt", "w");
		fopen_s(&FPin, "data/imm_gama.txt", "w");
		fopen_s(&FPobs, "data/imm_u.txt", "w");
		fopen_s(&IMM_rec1, "data/imm_c.txt", "w");
		fopen_s(&IMM_rec2, "data/imm_tu.txt", "w");
		fopen_s(&IMM_rec3, "data/imm_tp.txt", "w");
		//fopen_s(&IMM_rec4, "data/imm_ana.txt", "w");
		fopen_s(&IMM_ana, "data/imm_ana.txt", "w");
		IMM_init_set_parm(&imm, kf, &TP, IMM_Module_Num);
		IMM_init_IMM_init_space(&imm, __Kalman_X_demin);
		imm.U = &U0;
		for (j = 0; j < __Kalman_N; j++)
		{
			IMM_process(&imm, imm.SX, imm.SP, observe + j);
			if (!(j%100))
				i = i;
			for (i = 0; i < IMM_Module_Num; i++)
			{
				*(Gama_outer.data + i) = *(imm.Gama_ret + i);
			}
			Matrix_save(imm.SX, FPKFres, 'd');
			Matrix_save(&Gama_outer, FPin, 'd');
			Matrix_save(imm.U, FPobs, 'd');
			Matrix_save(imm.C, IMM_rec1, 'd');
			Matrix_save(imm.TU, IMM_rec2, 'd');
			Matrix_save(imm.TP, IMM_rec3, 'd');
			Matrix_save(&mat2, IMM_ana, 'd');
		}
		fclose(FPKFres);
		fclose(FPobs);
		fclose(FPin);
		fclose(IMM_rec1); 
		fclose(IMM_rec2);
		fclose(IMM_rec3);
		fclose(IMM_ana);
		//fcloseall();


	}
	



/*------------------------------------------------------*/
// 电视信号处理	
	end:
	return 0;

}
