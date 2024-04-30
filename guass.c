#include"guass.h"
//#include"pch.h"
float Guass_noise(float segma)
{
	//see：https://baike.baidu.com/item/box-muller/4023794?fr=aladdin
	//int i, j;
	float u1, u2;
	float R, sita;
	float z1, z0;
	u1 = (float)rand()/ 35767.0;
	u2 = (float)rand() / 35767.0;
	R = sqrt((-2) * log(u1));
	sita = 2 * 3.14159265358979 * u2;
	z0 = R * cos(sita);
	z1 = R * sin(sita);
	//*segma = 0;
	//float seq[10000];
	/*for (int i = 0; i < 10000; i++)
	{
		u1 = (float)rand() / 35767.0;
		u2 = (float)rand() / 35767.0;
		R = sqrt(-2 * log(u1));
		sita = 2 * 3.14159265358979 * u2;
		z0 = R * cos(sita);
		z1 = R * sin(sita);
		seq[i] = z0;
		*aveg += z0/10000;
	}
	for (int i = 0; i < 10000; i++)
	{
		*segma += (seq[i] - *aveg)*(seq[i] - *aveg);
	}
	*segma /= 9999;*/
	return (segma*(z0+z1)/2);
}

Cal_state Matrix_Guass_noise(Matrixd*mat,Matrixd* segma)
{
	Cal_state res;
	int i=mat->row;
	int j=mat->col;
	if(mat->row*mat->col>segma->col*segma->row)
	{
		//造成segma的越界（只读）访问
		res=-1;

	}
	else
	{
		res=1;
	}
	Matrix_clr_trans_flag(mat);
	for(i--;i>=0;i--)
		for(j--;j>=0;j--)
		{
			Matrix_Pointer_Assign_Notrans_Element(mat,i,j)=Matrix_Pointer_Assign_Notrans_Element(mat,i,j)+Matrix_Pointer_Seek_Element(segma,i,j);
		}
	return res;
}