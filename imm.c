#include"imm.h"


#define IMM_PI 3.141592653589
#define isinf(x) ((x)>1e14?1:0)
//
void Imm_gama(Matrixd* V, Matrixd* inv_S,float det_S, float* ret)
{
	//do not reserve V,inv_S
	int i, j;
	Matrixd tem;
	
	Matrix_init(&tem, 1, 1, 'd');
	if (det_S <= 0)
	{		
		*ret = 0;
	}
	else
	{
		det_S = sqrt(det_S);
		*ret = 1 / pow(2 * IMM_PI, __Kalman_X_demin / 2) / det_S;
		Matrix_mul_Matrtix(inv_S, V, &tem);
		Matrix_transpose(V);
		Matrix_mul_Matrtix(V, &tem,inv_S);		
		if (inv_S->col == 1 && inv_S->row == 1)
		{
			det_S = *(inv_S->data);
			*ret *= exp(det_S);
		}
		else
		{
			
			*ret = 0;
		}
	}
	Matrix_release(&tem);
	return;

}


void IMM_init_set_parm(IMM* imm, KF* K_base, Matrixd* TP_addr, int Mn)
{
	imm->Gama = &Imm_gama;
	imm->lkf_p = &LKF_fliter;
	imm->ekf_p = NULL;
	imm->Module_num = Mn;
	imm->filter = K_base;
	imm->TP = TP_addr;
	return;
}
void IMM_init_IMM_init_space(IMM* imm, int KF_X_dim)
{

	imm->C = (Matrixd*)malloc(sizeof(Matrixd));
	Matrix_init(imm->C, 1, IMM_Module_Num, 'd');
	
	imm->TU = (Matrixd*)malloc(sizeof(Matrixd));
	Matrix_init(imm->TU, IMM_Module_Num, IMM_Module_Num, 'd');

	imm->SX = (Matrixd*)malloc(sizeof(Matrixd));
	Matrix_init(imm->SX, KF_X_dim, 1, 'd');
	imm->SP = (Matrixd*)malloc(sizeof(Matrixd));
	Matrix_init(imm->SP, KF_X_dim, KF_X_dim, 'd');
	imm->tem = (Matrixd*)malloc(sizeof(Matrixd));
	Matrix_init(imm->tem, KF_X_dim, KF_X_dim, 'd');
	imm->old = (Matrixd*)malloc(sizeof(Matrixd));
	Matrix_init(imm->old, KF_X_dim, KF_X_dim, 'd');
	return;

}
void IMM_release(IMM* imm)
{
	

	return;
}

void IMM_process(IMM* imm, Matrixd* XSpre, Matrixd* PSpre, Matrixd*Obs)
{
	//static float TP_lamda[IMM_Module_Num];
	float adjTp_fac=0.2;
	int gama_inf = 0;
	int gama_max_index = 0;
	float gama_punishment_fac = 4;
	int i, j;
	int k;
	float tem_sum;
	float j_inf;
	float temu, ga_uc_f_flag;//deal inf val in U
	Matrixd mat_adder;
	KF* kf_iter;
	Matrix_init(&mat_adder, 1, 1, 'd');
	
	for (j = 0; j < imm->Module_num; j++)
	{
		tem_sum = 0;
		for (i = 0; i < imm->Module_num; i++)
		{
			tem_sum += Matrix_Pointer_Seek_Element((imm->U), 0, i)*Matrix_Pointer_Seek_Element((imm->TP), i, j);
		}
		if (tem_sum > 1||tem_sum<0)//概率不能大于1
		{
			printf("critial err:Cj>1\n");
			return;
		}
		Matrix_Pointer_Assign_Notrans_Element((imm->C), 0, j) = tem_sum;
		
	}

	for (j = 0; j < imm->Module_num; j++)
	{
		for (i = 0; i < imm->Module_num; i++)
		{
			//for (k = 0; k < imm->Module_num; k++)
			Matrix_Pointer_Assign_Notrans_Element((imm->TU), i, j) = Matrix_Pointer_Seek_Element((imm->U), 0, i)*Matrix_Pointer_Seek_Element((imm->TP), i, j) / Matrix_Pointer_Assign_Notrans_Element((imm->C), 0, j);
		}
	}//uij done
	//Matrix_show(imm->TU);
	//Xfpre
	//Pfpre
	for (j = 0; j < imm->Module_num; j++)
	{
		kf_iter = imm->filter + j;
		Matrix_reshape(&mat_adder, kf_iter->Sys_Dimen, 1);
		for (k = 0; k < mat_adder.col*mat_adder.row; k++)
			*(mat_adder.data + k) = 0;//clr mat_adder
		for (i = 0; i < imm->Module_num; i++)
		{
			kf_iter = imm->filter + i;
			Matrix_copy(kf_iter->X, imm->tem);
			Matrix_mul_num(imm->tem, Matrix_Pointer_Seek_Element((imm->TU), i, j));
			Matrix_add(&mat_adder, imm->tem);
		}
		kf_iter = imm->filter + j;//back
		Matrix_copy(&mat_adder, kf_iter->Xfpre);
		//Matrix_show(kf_iter->Xfpre);
	}

	for (j = 0; j < imm->Module_num; j++)
	{
		kf_iter = imm->filter + j;
		Matrix_reshape(&mat_adder, kf_iter->Sys_Dimen, kf_iter->Sys_Dimen);
		for (k = 0; k < mat_adder.col*mat_adder.row; k++)
			*(mat_adder.data + k) = 0;//clr mat_adder
		for (i = 0; i < imm->Module_num; i++)
		{
			kf_iter = imm->filter + i;
			Matrix_copy(kf_iter->P, imm->tem);
			Matrix_mul_num(imm->tem, Matrix_Pointer_Seek_Element((imm->TU), i, j));
			Matrix_add(&mat_adder, imm->tem);//
			Matrix_copy(kf_iter->X, imm->tem);
			Matrix_mul_num(imm->tem, -1);
			Matrix_add(imm->tem, kf_iter->Xfpre);
			Matrix_copy(imm->tem, imm->old);
			Matrix_transpose(imm->old);
			Matrix_mul_Matrtix(imm->tem, imm->old, kf_iter->tem);
			Matrix_add(&mat_adder, kf_iter->tem);
		}
		kf_iter = imm->filter + j;
		Matrix_copy(&mat_adder, kf_iter->Pfpre);
		//Matrix_show(kf_iter->Pfpre);
	}
	for (j = 0; j < imm->Module_num; j++)
	{
		kf_iter = imm->filter + j;
		
		////update TP_lamda
		//tem_sum= 0;
		//for (k = 0; k < kf_iter->Sys_Dimen; k++)
		//	tem_sum += ((kf_iter->X->data)[k] - (kf_iter->Xfpre->data)[k])*
		//				((kf_iter->X->data)[k] - (kf_iter->Xfpre->data)[k]);
		//TP_lamda[j] = sqrt(tem_sum);
		////update TP_lamda done
		Matrix_copy(kf_iter->Xfpre, kf_iter->X);
		Matrix_copy(kf_iter->Pfpre, kf_iter->P);
		
	}//mixed X P 

	//Step2: KF process
	for (j = 0; j < _LKF_MODULE_NUM_; j++)
	{
		kf_iter = imm->filter + j;
		imm->lkf_p(kf_iter, kf_iter->X, kf_iter->P, Obs);
	}
	for (j = _LKF_MODULE_NUM_; j < imm->Module_num; j++)
	{
		kf_iter = imm->filter + j;
		imm->ekf_p(kf_iter, kf_iter->X, kf_iter->P, Obs);
	}//KF done

	//Step3: update module possibility
	for (j = 0; j < imm->Module_num; j++)
	{
		kf_iter = imm->filter + j;
		//
		/*printf("j=%dV:\n",j);
		Matrix_show(kf_iter->det);
		printf("j=%dS:\n", j);
		Matrix_show(kf_iter->Xfpre);*/
		imm->Gama(kf_iter->det, kf_iter->IMM_inv_S,kf_iter->IMM_det_S, imm->Gama_ret + j);
	}//gama(k) done

	//limit gama
	ga_uc_f_flag = 0;
	for (j = 0; j < imm->Module_num; j++)
	{
		if (*(imm->Gama_ret + j) > 1e10&&*(imm->Gama_ret + j)>ga_uc_f_flag)
		{
			ga_uc_f_flag=*(imm->Gama_ret + j) ;
			if (isinf(ga_uc_f_flag))
				ga_uc_f_flag = 1e12;
		}
	}
	if (ga_uc_f_flag > 1e10)
	{
		for (j = 0; j < imm->Module_num; j++)

			*(imm->Gama_ret + j) /= ga_uc_f_flag/100;
	}
	//judge inf
	for (j = 0; j < imm->Module_num; j++)
	{
		ga_uc_f_flag = *(imm->Gama_ret + j);
		if (isinf(ga_uc_f_flag))
		{
			*(imm->Gama_ret + j) = 1e10;
		}
			
	}
	tem_sum = 0;
	for (j = 0; j < imm->Module_num; j++)
	{
		//Bug:无限似然
		temu = *((imm->Gama_ret) + j)*Matrix_Pointer_Seek_Element((imm->C), 0, j);
		if (temu > 1e15)
			temu = 1e15;
		Matrix_Pointer_Assign_Notrans_Element((imm->U), 0, j) = temu;
		tem_sum += temu;
		//Matrix_Pointer_Assign_Notrans_Element((imm->U), 0, j)= *((imm->Gama_ret) + j)*Matrix_Pointer_Seek_Element((imm->C), 0, j);
		//tem_sum += Matrix_Pointer_Assign_Notrans_Element((imm->U), 0, j);
	}
	
	for (j = 0; j < imm->Module_num; j++)
	{
		Matrix_Pointer_Assign_Notrans_Element((imm->U), 0, j) /= tem_sum;
	}//Uj done

	//Step4 :output intersection
	for (k = 0; k < imm->SX->col*imm->SX->row; k++)
		*(imm->SX->data + k) = 0;//clr SX
	for (j = 0; j < imm->Module_num; j++)
	{
		kf_iter = imm->filter + j;
		Matrix_copy(kf_iter->X, imm->tem);
		Matrix_mul_num(imm->tem, Matrix_Pointer_Seek_Element((imm->U), 0, j));
		Matrix_add(imm->SX, imm->tem);
	}//SX done
	for (k = 0; k < imm->SP->col*imm->SP->row; k++)
		*(imm->SP->data + k) = 0;//clr SP
	for (j = 0; j < imm->Module_num; j++)
	{
		kf_iter = imm->filter + j;
		Matrix_copy(kf_iter->P, imm->tem);
		Matrix_mul_num(imm->tem, Matrix_Pointer_Seek_Element((imm->U), 0, j));
		Matrix_add(imm->SP, imm->tem);//
		Matrix_copy(kf_iter->X, imm->tem);
		Matrix_mul_num(imm->tem, -1);
		Matrix_add(imm->tem, imm->SX);
		Matrix_copy(imm->tem, imm->old);
		Matrix_transpose(imm->old);
		Matrix_mul_Matrtix(imm->tem, imm->old, kf_iter->tem);
		Matrix_add(imm->SP, kf_iter->tem);//		
	}//SP done
	//imm done

	//}//refresh TP  TP-->>TU
	Matrix_copy(imm->TP, imm->tem);
	Matrix_mul_num(imm->TU, -1);// nolonger save TU
	Matrix_add(imm->tem, imm->TU);//tem=TP-TU
	//for (j = 0; j < imm->Module_num; j++)
	//{
	//	for (i = 0; i < imm->Module_num; i++)
	//	{ 
	//		Matrix_Pointer_Assign_Notrans_Element(imm->TP,i,j)*=
	//			exp(adjTp_fac*Matrix_Pointer_Assign_Notrans_Element(imm->tem, i, j));			
	//	}
	//}

	//Matrix_show(imm->TP);
	for (i = 0; i < imm->Module_num; i++)
	{
		tem_sum = 0;
		for (j = 0; j < imm->Module_num; j++)
		{
			//自适应
			Matrix_Pointer_Assign_Notrans_Element(imm->TP, i, j) *=
				exp(adjTp_fac*Matrix_Pointer_Assign_Notrans_Element(imm->tem, i, j));
		}
	}
	//Matrix_show(imm->TP);
	//似然值惩罚
	
	for (k = 0; k < imm->Module_num; k++)
	{			
		if (*(imm->Gama_ret + k) > 1e5)
		{				
			gama_inf = 1; break;
		}
		else			
		{
			if (k != 0)
				if (*(imm->Gama_ret + k) > *(imm->Gama_ret + gama_max_index))
					gama_max_index = k;
		}
	}
	Matrix_reshape(imm->old, imm->Module_num, 1);
	for (k = 0; k < imm->Module_num; k++)
		Matrix_Pointer_Assign_Notrans_Element(imm->old, k, 0) =
			fabs(*(imm->Gama_ret + k) - *(imm->Gama_ret + gama_max_index));
	
	if (gama_inf == 0)
	{
		for (j = 0; j < imm->Module_num; j++)
		{
			//按列惩罚
			for (i = 0; i < imm->Module_num; i++)
			{
				if (j != gama_max_index && i != j);
				Matrix_Pointer_Assign_Notrans_Element(imm->TP, i, j) /= temu =
					gama_punishment_fac * (1 + log(Matrix_Pointer_Assign_Notrans_Element(imm->old, j, 0) + 1));
			}
		}
	}
	//Matrix_show(imm->TP);
	//Matrix_show(imm->old);
	for (i = 0; i < imm->Module_num; i++)
	
	{
		//按行求和
		tem_sum = 0;
		for (j = 0; j < imm->Module_num; j++)
		{
			//protect
			if (Matrix_Pointer_Assign_Notrans_Element(imm->TP, i, j) < 0.05)
				Matrix_Pointer_Assign_Notrans_Element(imm->TP, i, j) = 0.05;
			else if (Matrix_Pointer_Assign_Notrans_Element(imm->TP, i, j) >= 0.95)
				Matrix_Pointer_Assign_Notrans_Element(imm->TP, i, j) = 0.95;
			tem_sum += Matrix_Pointer_Assign_Notrans_Element(imm->TP, i, j);
		}
		for (j = 0; j < imm->Module_num; j++)
		{
			Matrix_Pointer_Assign_Notrans_Element(imm->TP, i, j) /= tem_sum;//renormalize
		}
	}

	//refresh TP done
	//BUG:TP->1

	//for (j = 0; j < imm->Module_num; j++)//save TP
	//{
	//	for (i = 0; i < imm->Module_num; i++)
	//	{

	//		if (Matrix_Pointer_Assign_Notrans_Element(imm->TP, i, j) < 1e-5)
	//			Matrix_Pointer_Assign_Notrans_Element(imm->TP, i, j) = 1e-5;
	//		else if ( Matrix_Pointer_Assign_Notrans_Element(imm->TP, i, j) >=0.999)
	//			Matrix_Pointer_Assign_Notrans_Element(imm->TP, i, j) = 0.999;
	//	}
	//}
	
	//Matrix_show(imm->TP);

	/*Matrix_release(&mat_adder);
	printf("%dres:", kf_iter->index);
	Matrix_show(imm->SX);
	Matrix_show(imm->SP)*/;
	return;


}

void IMM_Gama_module(IMM*imm, float*Gama_rec,float*det_Gama)
{
	return;
}
void IMM_predict(IMM*imm, Matrixd* Xpre_out, float*Gama_rec,float*det_Gama,int step)//不允许操作IMM的Gama_ret
{
	int i, j;
	int k;
	float sum;
	KF* kf_iter;
	Matrix_reshape(imm->tem, 1, imm->Module_num);//storage normallized


}
