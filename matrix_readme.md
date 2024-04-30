欢迎使用矩阵函数的帮助文档！！
作者：申庆瑜
版权(C)：Murphy工作室所有，禁止外传。
Bug反馈：shen99855@outlook.com
版本：v1.02
	日期：2021 04
	数据类型：
			
		Matrixd：结构体，可变大小矩阵
			成员：
				row（行数）
				col（列数）
				data*（数据指针）
				trans_flag(转置标志)
			使用方法：
				重要准则：定义后，必须调用Matrix_init（）来初始化，使用完成后，必须调用Matrix_release()释放空间。
				高级:精通指针和内存管理机制的用户可以忽略上条准则。
				元素查找：
					对于Matrixd，可使用 Matrix_Seek_Element(name,i,j) 这个宏查找到name[i,j]
					对于Matrixd*  可使用Matrix_Pointer_Seek_Element(name,i,j)  这个宏查找到name[i,j]
				元素赋值：
					注意，只能对trans_flag=0的矩阵的元素进行赋值。
					如果不知道trans_flag是否为0，可以调用一次Matrix_clr_trans_flag（），这个函数可以将对应矩阵的trans_flag置0。
					保证矩阵不被转置的前提条件下：
						对于Matrixd，可使用  Matrix_Assign_Notrans_Element(name,i,j) 这个宏查定位name[i,j]
						对于Matrixd*  可使用 Matrix_Pointer_Assign_Notrans_Element(name,i,j)  这个宏查定位name[i,j]
					示例：
						Matrixd* mat ,copy;
						...
						Matrix_Pointer_Assign_Notrans_Element(copy,i,j) = Matrix_Pointer_Seek_Element(mat, i, j);
		Vector：结构体，可变长度向量
			成员：
				len（长度）
				data*（数据指针）
			因为向量相比较矩阵而言，简单得多，
			所以没有封装像Matrix_init()等那样的基础函数，
			用户可以自己管理data*的指向，但应注意内存泄漏问题。
		Cal_state:枚举，用以指示计算状态
			参见其定义，及各个函数的返回值定义。
	函数表：
		总则：
			Matrix：函数中出现此词汇，意为操作矩阵
			MatrixS：函数中出现此词汇，意为操作方阵
		基础类函数：
			基础类函数实现了矩阵的内存分配，分割和构造等，是所有计算类函数实现的基石。
			Cal_state Matrix_init( Matrixd* mat,int row, int col, int f)
				功能描述：初始化mat为row行col列，并分配内存。
				参数：
					mat：操作对象矩阵的地址
					row：行数
					col：列数
					f：标志位，可以有以下几种选项：
						'0':全0阵
						'1':全1阵
						'i'：单位阵
						't'：从0开始元素逐个递增1
						'd'：只分配内存，不对元素进行初始赋值，是最快的操作

				返回值：
					success：成功
				其他说明：
					参见Matrixd使用的重要准则。
			Cal_state Matrix_transpose(Matrixd* mat)
				功能描述：矩阵的转置。
				参数：
					mat：操作对象矩阵的地址
				返回值：
					success：成功
				其他说明：
					只改变内存读取顺序，不改变内存实际排列顺序。
			void Matrix_clr_trans_flag(Matrixd* mat)	
				功能描述：清零矩阵的trans_flag。
				参数：
					mat：操作对象矩阵的地址
				返回值：
					
				其他说明：
					参见Matrixd使用方法的元素赋值一节。
			Cal_state Matrix_reshape(Matrixd* mat, int row, int col)
				功能描述：矩阵大小重新分配为row行col列。
				参数：
					mat：操作对象矩阵的地址
					row：行数
					col：列数
				返回值：
					error：row和col非法
					success:成功，且原数据保留
					optial：内存被扩大，原数据丢失,但是存储空间大小成功改变。
				其他说明：
					返回success时，矩阵的trans_flag=0。
			Cal_state Matrix_copy(Matrixd* mat, Matrixd* copy)
				功能描述：矩阵复制，copy=mat，并保留mat。
				参数：
					mat：操作对象矩阵的地址
					copy：镜像矩阵地址
				返回值：
					do_noing：使用了未初始化的copy
					success:成功，且copy的transflag=0；					
				其他说明：
					返回success时，copy的存储空间线性排列。
			void Matrix_show(Matrixd* mat)
				功能描述：打印矩阵，根据不同平台可有多种GUI显示。
				参数：
					mat：操作对象矩阵的地址
				返回值：
					空					
				其他说明：
					该函数对调试非常有帮助，用户可以自己写。
					对于嵌入式系统，建议留空。
			void Matrix_release(Matrixd* mat)
				功能描述：释放矩阵内存。
				参数：
					mat：操作对象矩阵的地址
				返回值：
					空					
				其他说明：
					参见Matrixd使用的重要准则。
		计算类函数：
			计算类函数用以实现以矩阵为操作对象的计算方法，算法等。
			Cal_state Matrix_add( Matrixd* mat, Matrixd* adder)
				功能描述：矩阵加，mat=mat+adder
				参数：
					mat：操作对象矩阵的地址
					adder：被加数
				返回值：
					error:adder不符合规范，继续操作将造成越界访问。
					success:成功
				其他说明：
					结果存入mat原位。
					要求adder的行或列数均大于等于mat，最好是等于（matlab亦如此要求）。
			Cal_state Matrix_mul_num(Matrixd* mat, float num)
				功能描述：矩阵数乘，mat=mat*num
				参数：
					mat：操作对象矩阵的地址
					num：乘的系数
				返回值：
					success:成功
				其他说明：
					结果存入mat原位。
			Cal_state Matrix_mul_Matrtix(Matrixd* mat_left, Matrixd* mat_right,Matrixd* mat_result)	
				功能描述：矩阵乘矩阵，mat_result=mat_left*mat_right
				参数：
					mat_left：左矩阵的地址
					mat_right：右矩阵的地址
					mat_result：结果
				返回值：
					success:成功
					error：left->rol!=right->row
				其他说明：
					result矩阵大小自适应。
			Cal_state Matrix_QRc(Matrixd* mat, Matrixd* Q, Matrixd* R);
				功能描述：矩阵QR分解，mat=Q*R，不知道的请百度
				参数：
					mat：操作对象矩阵的地址
					Q:输出参数
					R:输出参数
					
				返回值：
					success:成功
					error：有全零列导致分解失败
					do_nothing:不满足mat的列数小于等于行数
				其他说明：
					要求mat的列数小于等于行数
					没有做算法优化。
			Cal_state MatrixS_LUc(Matrixd* mat, Matrixd* L, Matrixd* U);
				功能描述：矩阵LU分解，mat=L*U，不知道的请百度
				参数：
					mat：操作对象矩阵的地址
					L:输出参数
					U:输出参数
					
				返回值：
					success:成功					
					do_nothing:非方阵
				其他说明：
					算法复杂度1/3 *N^3
			Cal_state MatrixS_cholesky_c(Matrixd* mat, Matrixd* L);		
				功能描述：正定实对称矩阵桥莱克斯及分解，mat=L*trans(L)，不知道的请百度
				参数：
					mat：操作对象矩阵的地址
					L:输出参数					
				返回值：
					success:成功					
					do_nothing:非方阵
				其他说明：
					精度误差逐层累积，矩阵的左上角部分认为误差为0；矩阵右下角累计误差可以达到最大舍入误差的100倍，甚至更高。
			float MatrixS_det(Matrixd* mat)
				功能描述：矩阵行列式
				参数：
					mat：操作对象矩阵的地址，要求是方阵					
				返回值：
					矩阵行列式					
					do_nothing:非方阵
				其他说明：
					方阵才有行列式！！！
					//不能告诉外人的善意的提醒：使用LUc计算行列式。
			Cal_state MatrixS_inverse(Matrixd* mat, Matrixd* mat_inv)
				功能描述：矩阵求逆，mat_inv=inv(mat)
				参数：
					mat：操作对象矩阵的地址
					mat_inv：mat的逆
				返回值：
					do_nothing:不是方阵
					error:mat非满秩
					success:成功
				其他说明：
					方阵才有逆矩阵。
					做了算法和存储优化，可以放心大胆的用。