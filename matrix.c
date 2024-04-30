#include"matrix.h"
//#include"pch.h"
//#4月14新修版
#define _Vxworks_Show




Cal_state Matrix_init(Matrixd* mat, int row, int col, char f)
{
	int i, j;
	//mat->com = NULL;
	mat->trans_flag = 0;
	mat->col = col;
	mat->row = row;
	mat->data = (float*)malloc(col*row * sizeof(float));
	
	switch (f)
	{
	case '0'://全0
		for (i = 0; i < row; i++)
			for (j = 0; j < col; j++)
				*(mat->data + i * col + j) = 0;
		break;
	case '1'://全1
		for (i = 0; i < row; i++)
			for (j = 0; j < col; j++)
				*(mat->data + i * col + j) = 1;
		break;
	case'i'://单位
		for (i = 0; i < row; i++)
			for (j = 0; j < col; j++)
			{
				if (i == j)
					*(mat->data + i * col + j) = 1;
				else
					*(mat->data + i * col + j) = 0;
			}
		break;
	case't'://test
		for (i = 0; i < row; i++)
			for (j = 0; j < col; j++)
			{

				*(mat->data + i * col + j) = i * col + j;

			}
		break;
	default:
		break;
	}

	return 1;

}
Cal_state Matrix_transpose(Matrixd* mat)
{
	int tem = mat->col;
	mat->col = mat->row;
	mat->row = tem;
	mat->trans_flag = !(mat->trans_flag);
	return 1;
}
Cal_state Matrix_add(Matrixd* mat, Matrixd* adder)//结果存在mat原位
{
	int i, j;
	//只允许adder比mat大
	if (mat->col < adder->col || mat->row < adder->row)
		return -1;
	else
	{
		Matrix_clr_trans_flag(mat);
		for (i = 0; i < mat->row; i++)
			for (j = 0; j < mat->col; j++)
				Matrix_Pointer_Assign_Notrans_Element(mat, i, j) = Matrix_Pointer_Assign_Notrans_Element(mat, i, j) + Matrix_Pointer_Seek_Element(adder, i, j);
		return 1;
	}

}
Cal_state Matrix_mul_num(Matrixd* mat, float num)
{
	int i, j;
	for (i = 0; i < mat->row; i++)
		for (j = 0; j < mat->col; j++)
			*(mat->data + i * mat->col + j) *= num;
	return 1;
}
Cal_state Matrix_mul_Matrtix(Matrixd* mat_left, Matrixd* mat_right, Matrixd* mat_result)//f="l/row"
{
	int i, j, k;
	float sum ;
	if (mat_left->col != mat_right->row)
		return -1;
	else
	{
		Matrix_reshape(mat_result, mat_left->row, mat_right->col);
		for (i = 0; i < mat_result->row; i++)
			for (j = 0; j < mat_result->col; j++)
			{
				sum=0;
				for (k = 0; k < mat_left->col; k++)
					sum += Matrix_Pointer_Seek_Element(mat_left, i, k)*Matrix_Pointer_Seek_Element(mat_right, k, j);
				Matrix_Pointer_Assign_Notrans_Element(mat_result, i, j) = sum;
			}
		return 1;
	}
}
Cal_state Matrix_copy(Matrixd* mat, Matrixd* copy)
{
	int i, j;
	if (copy->data == NULL)
		return 0;
	//经过复制，转置标志位置0，存储空间线性重排。
	copy->col = mat->col;
	copy->row = mat->row;
	copy->trans_flag = 0;
	copy->data = (float*)realloc(copy->data, mat->col*mat->row * sizeof(float));
	for (i = 0; i < mat->row; i++)
		for (j = 0; j < mat->col; j++)
			*(copy->data + i * copy->col + j) = Matrix_Pointer_Seek_Element(mat, i, j);
	return 1;
}
Cal_state Matrix_reshape(Matrixd* mat, int row, int col)//只有不重新分配地址的时候，才保留原数据
{
	Cal_state s = 0;
	if (row <= 0 || col <= 0)
		return -1;
	
	if (mat->col*mat->row < row * col)
	{
		mat->data = (float*)realloc(mat->data, row*col * sizeof(float));//enlarge memory
		if (mat->data == NULL)
		{
			printf("critial err from matrix_reshape:realloc");
			s = -1;
		}
		s = 2;
	}
	else
	{
		Matrix_clr_trans_flag(mat);//relinearlize memory
		s = 1;
	}
	mat->row = row;
	mat->col = col;
	return s;

}
Cal_state Matrix_save(Matrixd* mat, FILE* fp, char Mode)
{
	int i, j;
	float* num;
	char* flag;
	j = sizeof(float) / sizeof(char);
	if (fp == NULL)
		return 0;
	else

	{
		//if(Mode=='a')
		Matrix_clr_trans_flag(mat);
		//Matrix_reshape(mat, mat->row*mat->col, 1);
		fprintf(fp, "\0");
		for (i = 0; i < mat->row*mat->col; i++)
		{
			num = mat->data + i;
			fprintf(fp, " %f\t", *num);
		}
		//fputc("\n", fp);
		fprintf(fp, "\n");
		return 1;
	}
}
Cal_state Matrix_load(Matrixd* mat, FILE* fp, int size)
{
	int cnt=0;
	cnt = 0;
	if(size > mat->col*mat->row)
		Matrix_reshape(mat,1,size);
	if (fp == NULL)
		return 0;
	for (cnt = 0; cnt < size; cnt++)
	{
		if (fscanf_s(fp, "%f\t", (mat->data) + cnt) == EOF)
		{
			printf("critial err from matrix_load");
			return -1;
		}
	}
	
	return 1;
}
void Matrix_release(Matrixd* mat)
{
	free(mat->data);
	//free(mat);
	return;
}
void Matrix_clr_trans_flag(Matrixd* mat)
{
	//为了提高效率,本代码执行(伪转置)逻辑,对于非线性存储的元素,可能不便运算,如reshape,可用此函数完成真正的转置,再次将存储空间线性化
	int i, j;
	float* tem=NULL;
	if (mat->trans_flag)
	{	
		tem = (float*)malloc(mat->col*mat->row * sizeof(float));
		for (i = 0; i < mat->row; i++)
			for (j = 0; j < mat->col; j++)
				*(tem + i * mat->col + j) = Matrix_Pointer_Seek_Element(mat, i, j);
		mat->trans_flag = 0;
		for (i = 0; i < mat->row; i++)
			for (j = 0; j < mat->col; j++)
				*(mat->data + i * mat->col + j) = *(tem + i * mat->col + j);//正序存储
		free(tem);//为了保险起见,保留了mat->data的指向.降低了效率.

	}
	return;
}
Cal_state MatrixS_inverse(Matrixd* mat, Matrixd* mat_inv,float* det)
{
	//matlab 是用奇异值分解来计算，包括伪逆；
	//eigen 处理的简单粗暴，它直接认为矩阵非奇异。
	Matrixd matl, matu;
	
	Cal_state s = 0;
	int i;
	Matrixd matl_inv, matu_inv;
	
	*det = 1;
	if (mat->col != mat->row)
		//不是方阵
		return 0;
	
	Matrix_init(&matl, mat->row, mat->col, '0');
	Matrix_init(&matu, mat->row, mat->col, '0');
	Matrix_reshape(mat_inv, mat->row, mat->col);
	MatrixS_LUc(mat, &matl, &matu);
	//Matrix_show(&matl);
	//Matrix_show(&matu);
	//Matrix_mul_Matrtix(&matl, &matu, mat_inv);
	//Matrix_show(mat_inv);	
	for (i = 0; i < mat->row; i++)
		(*det) *= Matrix_Assign_Notrans_Element(matl, i, i)*Matrix_Assign_Notrans_Element(matu, i, i);
	if (*det == 0)
		s = -1;
	else
	{	
		Matrix_init(&matl_inv, mat->row, mat->col, 'd');
		Matrix_init(&matu_inv, mat->row, mat->col, 'd');
		Matrix_transpose(&matl);
		MatrixS_Up_Triangle_inverse(&matu, &matu_inv);
		MatrixS_Up_Triangle_inverse(&matl, &matl_inv);
		//Matrix_show(&matl_inv);
		//Matrix_show(&matu_inv);
		Matrix_transpose(&matl_inv);
		Matrix_mul_Matrtix(&matu_inv, &matl_inv, mat_inv);
		Matrix_release(&matl_inv);
		Matrix_release(&matu_inv);
		s = 1;
	}
	Matrix_release(&matl);
	Matrix_release(&matu);
	return s;

}
// float MatrixS_det(Matrixd* mat)//使用克莱恩分解法计算
// {	//优化了三阶行列式作为递归出口
// 	//10阶以内暂时是没有问题的，阶次过高会导致内存开销扩大
// 	/*
// 	内存开销随阶次的函数关系k（n）可以如下计算
// 	set k（3）=9；
// 	set k（2）=4；
// 	k（n）=k（n-1)*(n-2)+k(n-2);
// 	k(n)<n!;
// 	*/
// 	//计算复杂度接近n！，不推荐用此函数判断矩阵是否满秩。
// 	float result;
// 	//矩阵的转置不改变行列式的值
// 	if (mat->col != mat->row)//不是方阵
// 		return -1;
// 	else
// 	{
// 		int d = mat->col;
// 		if (d == 1)
// 		{
// 			return *(mat->data);
// 		}
// 		else
// 		{
// 			if (d == 2)
// 			{
// 				return *(mat->data)**(mat->data + 3) - *(mat->data + 1)**(mat->data + 2);
// 			}
// 			else
// 			{
// 				if (d == 3)
// 				{
// 					float zhu = *(mat->data)**(mat->data + 4)**(mat->data + 8) +
// 						*(mat->data + 1)**(mat->data + 5)**(mat->data + 6) +
// 						*(mat->data + 2)**(mat->data + 3)**(mat->data + 7);
// 					float fu = *(mat->data)**(mat->data + 5)**(mat->data + 7) +
// 						*(mat->data + 1)**(mat->data + 3)**(mat->data + 8) +
// 						*(mat->data + 2)**(mat->data + 4)**(mat->data + 6);
// 					return zhu - fu;
// 				}
// 				else
// 				{
// 					result = 0;//求和变量清零
// 					Matrixd* iter;
// 					Matrixd* Rsv;
// 					Rsv = (Matrixd*)malloc(d * sizeof(Matrixd));//为伴随阵开辟空间
// 					int i, j, k;
// 					for (i = 0; i < d; i++)
// 					{
// 						Matrix_init(Rsv + i, d - 1, d - 1, 't');//初始化所有伴随阵
// 						//Matrix_show(Rsv + i);
// 					}
// 					for (k = 0; k < d; k++)//遍历d个伴随阵
// 					{
// 						iter = Rsv + k;//指向第k个伴随阵
// 						int jump_line = k;
// 						for (i = 0; i < d - 1; i++)
// 						{
// 							int line = -1;
// 							for (j = 0; j < d - 1; j++)//遍历所有元素
// 							{
// 								line++;
// 								if (line == jump_line)
// 									line += 1;//跳过一列
// 								*(iter->data + i * iter->col + j) = *(mat->data + i * mat->col + line);
// 								//*(((Rsv + k)->data) + i * ((Rsv + k)->col) + j) = 1;//debug 用
// 							}
// 							//这样做完以后，要留最后一行作为伴随阵的系数
// 						}
// 						//Matrix_show(Rsv + k);//for debug
// 					}
// 					for (k = 0; k < d; k++)//计算d个余子式,取最后一行分解。
// 					{
// 						result += (*(mat->data + d * d - d + k))*MatrixS_det(Rsv + k)*((k % 2) ? -1 : 1);//d=mat->col
// 						//Matrix_release(Rsv + k);//参与计算后清理内存
// 						//printf_s("bolck=%x\n", (Rsv + k)->data);
// 					}
// 					for (k = 0; k < d; k++)//计算d个余子式,取最后一行分解。
// 					{
// 						//Matrix_release(Rsv + k);//参与计算后清理内存
// 						free((Rsv + k)->data);
// 						//这里不可以调用Matrix_release()函数来清内存，因为传参以后，Rsv被编译器优化掉了，这就导致寻址失败。
// 						//建议只在main函数中调用Matrix_release()函数，封装函数直接free对应指针。
// 					}
// 					//printf_s("result=%f\n,order=%d", result, d);
// 					free(Rsv);
// 					return result;
// 				}
// 			}
// 		}
// 	}
// }
void Matrix_show(Matrixd* mat)
{
#ifdef _C6747_H

#endif
#ifdef _Vxworks_Show
	int i, j;
	float num;
	for (i = 0; i < mat->row; i++)
	{
		for (j = 0; j < mat->col; j++)
		{
			num = Matrix_Pointer_Seek_Element(mat, i, j);
			printf("%4.4f ", num);

		}
		printf("\n");

	}

#endif // !_Vxworks_Show

#ifdef _VS_win
	int i, j;
	float num;
	for (i = 0; i < mat->row; i++)
	{
		for (j = 0; j < mat->col; j++)
		{
			num = Matrix_Pointer_Seek_Element(mat, i, j);
			printf("%4.4f ", num);

		}
		printf("\n");

	}
#endif
	return;
}
// Cal_state Matrix_QRc(Matrixd* mat, Matrixd* Q, Matrixd* R)
// {
// 	//本函数来源网络，暂不能保证效率和稳定性。
// 	//see:https://blog.csdn.net/baidu_41647951/article/details/84401504
// 	//https ://blog.csdn.net/baidu_41647951/article/details/84401504
// 	if (mat->col > mat->row)
// 		return 0;//保证列数小于行数
// 	else
// 	{
// 		Matrix_reshape(Q, mat->row, mat->row);
// 		Matrix_copy(mat, R);
// 		int m = mat->row;
// 		int n = mat->col;
// 		int i, j, k, nn, jj;
// 		nn = n;
// 		if (m == n)
// 		{
// 			nn = m - 1;
// 		}
// 		float u, alpha, w, t;
// 		for (k = 0; k <= nn - 1; k++)//在大循环k：0~m当中，进行H矩阵的求解，左乘Q，以及左乘A
// 		{
// 			u = 0.0;
// 			for (i = k; i <= m - 1; i++)
// 			{
// 				w = fabs(Matrix_Pointer_Assign_Notrans_Element(R, i, k));
// 				if (w > u)
// 					u = w;
// 			}
// 			alpha = 0.0;
// 			for (i = k; i <= m - 1; i++)
// 			{
// 				t = Matrix_Pointer_Assign_Notrans_Element(R, i, k) / u;
// 				alpha = alpha + t * t;
// 			}
// 			if (Matrix_Pointer_Assign_Notrans_Element(R, k, k) > 0.0)
// 				u = -u;
// 			alpha = u * sqrt(alpha);
// 			if (fabs(alpha) + 1.0 == 1.0)
// 			{
// 				return -1;//分解失败
// 				//有全零列导致分解失败
// 			}
// 			u = sqrt(2.0*alpha*(alpha - Matrix_Pointer_Assign_Notrans_Element(R, k, k)));
// 			if ((u + 1.0) != 1.0)
// 			{
// 				Matrix_Pointer_Assign_Notrans_Element(R, k, k) = (Matrix_Pointer_Assign_Notrans_Element(R, k, k) - alpha) / u;
// 				for (i = k + 1; i <= m - 1; i++)
// 					Matrix_Pointer_Assign_Notrans_Element(R, i, k) /= u;
// 				//以上就是H矩阵的求得，实际上程序并没有设置任何数据结构来存储H矩
// 				//阵，而是直接将u向量的元素赋值给原A矩阵的原列向量相应的位置，这样做
// 				//这样做是为了计算左乘矩阵Q和A
// 				for (j = 0; j <= m - 1; j++)
// 				{
// 					t = 0.0;
// 					for (jj = k; jj <= m - 1; jj++)
// 						t = t + Matrix_Pointer_Assign_Notrans_Element(R, jj, k) * Matrix_Pointer_Assign_Notrans_Element(Q, jj, j);
// 					for (i = k; i <= m - 1; i++)
// 						Matrix_Pointer_Assign_Notrans_Element(Q, i, j) -= 2.0*t*Matrix_Pointer_Assign_Notrans_Element(R, i, k);
// 				}
// 				//左乘矩阵Q，循环结束后得到一个矩阵，再将这个矩阵转置一下就得到QR分解中的Q矩阵
// 				//也就是正交矩阵
// 				for (j = k + 1; j <= n - 1; j++)
// 				{
// 					t = 0.0;
// 					for (jj = k; jj <= m - 1; jj++)
// 						t = t + Matrix_Pointer_Assign_Notrans_Element(R, jj, k) * Matrix_Pointer_Assign_Notrans_Element(R, jj, j);
// 					for (i = k; i <= m - 1; i++)
// 						Matrix_Pointer_Assign_Notrans_Element(R, i, j) -= 2.0*t*Matrix_Pointer_Assign_Notrans_Element(R, i, k);
// 				}
// 				//H矩阵左乘A矩阵，循环完成之后，其上三角部分的数据就是上三角矩阵R
// 				Matrix_Pointer_Assign_Notrans_Element(R, k, k) = alpha;
// 				for (i = k + 1; i <= m - 1; i++)
// 					Matrix_Pointer_Assign_Notrans_Element(R, i, k) = 0.0;
// 			}
// 		}
// 		Matrix_transpose(Q);
// 		Matrix_clr_trans_flag(Q);
// 		//QR分解完成
// 		//Matrix_show(Q);
// 		//Matrix_show(R);
// 		return 1;
// 	}
// }
Cal_state MatrixS_LUc(Matrixd* mat, Matrixd* L, Matrixd* U)
{
	//L 下三角 U 上三角 mat=L*U
	//使用直接三角分解法，算法复杂度n^3
	//https://blog.csdn.net/weixin_44116061/article/details/105628206
	//https://baike.baidu.com/item/lu%E5%88%86%E8%A7%A3/764245?fr=aladdin
	
	int N;
	int i, j;
	int r, k;

	float sumLrkUki_r_i;
	float sumLikUkr_r_i;
	float re;
	if (mat->col != mat->row)
		//不是方阵
		return 0;
	else
	{
		N = mat->col;
		//Matrix_clr_trans_flag(mat);
		Matrix_reshape(L, N, N);
		Matrix_reshape(U, N, N);
		
		for (i = 0; i < N - 1; i++)
		{
			for (j = i + 1; j < N; j++)//高效率
			{
				Matrix_Pointer_Assign_Notrans_Element(L, i, j) = 0;
				Matrix_Pointer_Assign_Notrans_Element(U, j, i) = 0;//LU初始为0
			}
		}
		//for (int i = 0; i < N ; i++)//低效率
		//{
		//	for (int j = 0; j < N; j++)
		//	{
		//		Matrix_Pointer_Assign_Notrans_Element(L, j, i) = 0;
		//		Matrix_Pointer_Assign_Notrans_Element(U, i, j) = 0;//LU初始为0
		//	}
		//}
		for (i = 0; i < N; i++)
		{
			Matrix_Pointer_Assign_Notrans_Element(U, 0, i) = Matrix_Pointer_Seek_Element(mat, 0, i);
			Matrix_Pointer_Assign_Notrans_Element(L, i, 0) = Matrix_Pointer_Seek_Element(mat, i, 0) / Matrix_Pointer_Seek_Element(U, 0, 0);
		}
		
		for (r = 1; r < N; r++)
		{
			for (i = r; i < N; i++)
			{
				re = 0.0;
				for (k = 0; k < r; k++)
				{
					re += Matrix_Pointer_Assign_Notrans_Element(L, r, k)*Matrix_Pointer_Assign_Notrans_Element(U, k, i);
				}
				sumLrkUki_r_i = re;
				Matrix_Pointer_Assign_Notrans_Element(U, r, i) = Matrix_Pointer_Seek_Element(mat, r, i) - sumLrkUki_r_i;
				if (i == r)
					Matrix_Pointer_Assign_Notrans_Element(L, r, r) = 1;
				else
					if (r == N)
						Matrix_Pointer_Assign_Notrans_Element(L, N, N) = 1;
					else
					{
						re = 0.0;
						for (k = 0; k < r; k++)
						{
							re += Matrix_Pointer_Assign_Notrans_Element(L, i, k)*Matrix_Pointer_Assign_Notrans_Element(U, k, r);
						}
						sumLikUkr_r_i = re;
						Matrix_Pointer_Assign_Notrans_Element(L, i, r) = (Matrix_Pointer_Seek_Element(mat, i, r) - sumLikUkr_r_i) / Matrix_Pointer_Assign_Notrans_Element(U, r, r);

					}
			}
		}
		return 1;

	}
}
void MatrixS_Up_Triangle_inverse(Matrixd* mat, Matrixd* inv)
{
	//要求mat是上三角方阵，且主对角元均不为0,
	//允许mat->trans_flag=1
	//对上三角矩阵求其inverse。
	//封装为MatrixS_inverse的子函数
	//算法复杂度n^2
	int N;
	Vector One_col;
	Vector solu_col;
	
	int i, j;
	int ii;
	float back;
	if (mat->col != mat->row)
		//不是方阵
		return;
	else
		N = mat->col;
	Matrix_reshape(inv, N, N);
	
	One_col.len = N;
	One_col.data = (float*)malloc(N * sizeof(float));
	
	solu_col.len = N;
	solu_col.data = (float*)malloc(N * sizeof(float));
	
	for (j = N - 1; j >= 0; j--)//every col
	{
		for (i = 0; i < N; i++)
			if (i == j)
				*(One_col.data + i) = 1;
			else
			{
				*(One_col.data + i) = 0;
				if (i > j)
					Matrix_Pointer_Assign_Notrans_Element(inv, i, j) = 0;
			}
		//one col done
		for (i = j; i >= 0; i--)//倒序往上迭代，视mat的大小为j*j
		{
			
			back = *(solu_col.data + i) = *(One_col.data + i) / Matrix_Pointer_Seek_Element(mat, i, i);
			//////////////////////////////////////////////////////////////////////
			
			for (ii = i - 1; ii >= 0; ii--)//call back without change mat
			{
				back = *(solu_col.data + i)*Matrix_Pointer_Seek_Element(mat, ii, i);
				*(One_col.data + ii) -= back;
			}
		}
		for (i = 0; i <= j; i++)
		{
			//copy solution vector to inv
			Matrix_Pointer_Assign_Notrans_Element(inv, i, j) = *(solu_col.data + i);
		}

		//one col done
	}
	// all col done 
	free(solu_col.data);
	free(One_col.data);
	//printf("suucceed computing tri inverse");
	return;




}
Cal_state MatrixS_cholesky_c(Matrixd* mat, Matrixd* L)
{
	//精度误差逐层累积，矩阵的左上角部分认为误差为0；矩阵右下角累计误差可以达到最大舍入误差的100倍，甚至更高。
	//定义最大舍入误差为精确到小数点后的位数。
	int i, j, k, p;
	float sum;
	if (mat->col != mat->row)
		//不是方阵
		return 0;
	else
	{
		Matrix_reshape(L, mat->row, mat->col);
		
		for (k = 0; k < mat->col; k++)
		{
			for (i = 0; i < k; i++)
				Matrix_Pointer_Assign_Notrans_Element(L, i, k) = 0;//构造三角阵
			
			sum = 0;
			for (p = 0; p < k; p++)
				sum += Matrix_Pointer_Assign_Notrans_Element(L, k, p)*Matrix_Pointer_Assign_Notrans_Element(L, k, p);
			sum = Matrix_Pointer_Assign_Notrans_Element(mat, k, k) - sum;
			if (sum < 0)//理论上来说，sum一定为正，但浮点计算有舍入误差，应警惕对负值开平方
				sum = 0.000001;
			Matrix_Pointer_Assign_Notrans_Element(L, k, k) = sqrt(sum);
			//L(k,k) done


			for (i = k + 1; i < mat->row; i++)
			{
				sum = 0;
				for (j = 0; j < k; j++)
				{
					sum += Matrix_Pointer_Assign_Notrans_Element(L, i, j)*Matrix_Pointer_Assign_Notrans_Element(L, k, j);
				}
				sum = Matrix_Pointer_Assign_Notrans_Element(mat, i, k) - sum;
				if (Matrix_Pointer_Assign_Notrans_Element(L, k, k) == 0)//警惕0作为分母
					Matrix_Pointer_Assign_Notrans_Element(L, k, k) = 0.0001;
				Matrix_Pointer_Assign_Notrans_Element(L, i, k) = sum / Matrix_Pointer_Assign_Notrans_Element(L, k, k);
			}
			//L(i,k) done

		}


		return 1;
	}


}

//待后续加入
// void Matrix_assign_by_Vec(Matrixd* mat, Vector* row_base, int row)
// {
// 	//一行一行处理
// 	Matrix_reshape(mat, row, row_base->len);
// 	int i, j;
// 	for (i = 0; i < row; i++)
// 	{
// 		for (j = 0; j < row_base->len; j++)
// 		{
// 			Matrix_Pointer_Assign_Notrans_Element(mat, i, j) = *((row_base + i)->data + j);
// 		}
// 	}
// }
// void Matrix_Splic(Matrixd* mat, Matrixd* under_mat)
// {
// 	//|mat|=  |mat      |
// 	//|   |   |under_mat|
// 	//要求under_mat的列数大于等于mat
// 	if (under_mat->col < mat->col)
// 		return;
// 	else
// 	{
// 		Matrixd copy;
// 		Matrix_init(&copy, mat->row + under_mat->row, mat->col, 'd');
// 		//Matrix_clr_trans_flag(mat);
// 		//Matrix_copy(mat, &copy);
// 		//Matrix
// 		int i, j;
// 		for (i = 0; i < mat->row; i++)
// 		{
// 			for (j = 0; j < mat->col; j++)
// 				Matrix_Assign_Notrans_Element(copy, i, j) = Matrix_Pointer_Seek_Element(mat, i, j);
// 		}
// 		for (i = mat->row; i < mat->row + under_mat->row; i++)
// 		{
// 			int ind = i - mat->row;
// 			for (j = 0; j < mat->col; j++)
// 				Matrix_Assign_Notrans_Element(copy, i, j) = Matrix_Pointer_Seek_Element(under_mat, ind, j);
// 		}
// 		Matrix_copy(&copy, mat);
// 		Matrix_release(&copy);
// 		return;
// 	}
// }
// void Matrix_get_col(Matrixd* mat, Vector* col_vec, int col)
// {
	//	//默认col_vec 已经分配了存储
	//	col_vec->len = mat->row;
	//	col_vec->data = (float*)realloc(mat->row, sizeof(float));
	//	int i;
	//	for (i = 0; i < mat->row; i++)
	//	{
	//		*(col_vec->data + i) = Matrix_Pointer_Seek_Element(mat, i, col);
	//	}
	//	return;
//}


