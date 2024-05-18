#include "kalman_filter.h"


Matrix_DataTypDef* Matrix_Init(Matrix_DataTypDef* matrix, float* matrixdata, int i_length, int j_length)
{
	if (matrixdata == NULL)
	{
		return NULL;
	}

	matrix->data = matrixdata;
	matrix->i_length = i_length;
	matrix->j_length = j_length;

	return matrix;
}

void Matrix_Multiply(Matrix_DataTypDef* matrix_left, Matrix_DataTypDef* matrix_right, Matrix_DataTypDef* matrix_rev)
{
	if (matrix_left == NULL || matrix_right == NULL || matrix_rev == NULL)
	{
		return;
	}
	/*维度检测*/
	if (matrix_left->j_length == matrix_right->i_length)
	{
		/*返回矩阵维度检测*/
		if (matrix_rev->i_length == matrix_left->i_length && matrix_right->j_length)
		{
			/*初始化返回矩阵*/
			for (int i = 0; i < matrix_rev->i_length; i++)
			{
				for (int j = 0; j < matrix_rev->j_length; j++)
				{
					//matrix_rev->data[i][j] = 0;
					*(matrix_rev->data + i * matrix_rev->j_length + j) = 0;
				}
			}
			/*矩阵计算*/
			for (int r_j = 0; r_j < matrix_right->j_length; )//先遍历右矩阵
			{
				for (int r_i = 0; r_i < matrix_right->i_length; )
				{
					for (int l_j = 0; l_j < matrix_left->j_length; l_j++)
					{
						for (int l_i = 0; l_i < matrix_left->i_length; l_i++)
						{
							//matrix_left->data[r_i][r_j] += matrix_left->data[l_i][l_j] * matrix_right->data[r_i][r_j];
							*(matrix_rev->data + l_i * matrix_rev->j_length + r_j) += *(matrix_left->data + l_i * matrix_left->j_length + l_j) * *(matrix_right->data + r_i * matrix_right->j_length + r_j);
							//Matrix_Printf(matrix_rev);
							//printf("-------------------------------------\r\n");
						}
						r_i++;
					}
					r_j++;
				}
			}

		}
		else
		{
			return;
		}
	}
	else
	{
		return;
	}
}

void KalManFilter_Init(KalmanFilter_DataHandleTypeDef* kalman_filter)
{
	kalman_filter->AMatrix.data=NULL;
	kalman_filter->AMatrix.i_length=0;
	kalman_filter->AMatrix.j_length=0;
	
	kalman_filter->BMatrix.data=NULL;
	kalman_filter->BMatrix.i_length=0;
	kalman_filter->BMatrix.j_length=0;
	
	kalman_filter->HMatrix.data=NULL;
	kalman_filter->HMatrix.i_length=0;
	kalman_filter->HMatrix.j_length=0;
	
	kalman_filter->QMatrix.data=NULL;
	kalman_filter->QMatrix.i_length=0;
	kalman_filter->QMatrix.j_length=0;
	
	kalman_filter->RMatrix.data=NULL;
	kalman_filter->RMatrix.i_length=0;
	kalman_filter->RMatrix.j_length=0;
	
	kalman_filter->Priori_PMatrix.data=NULL;
	kalman_filter->Priori_PMatrix.i_length=0;
	kalman_filter->Priori_PMatrix.j_length=0;
	
	kalman_filter->LastPriori_PMatrix.data=NULL;
	kalman_filter->LastPriori_PMatrix.i_length=0;
	kalman_filter->LastPriori_PMatrix.j_length=0;
	
	kalman_filter->Prior_DataMatrix.data=NULL;
	kalman_filter->Prior_DataMatrix.i_length=0;
	kalman_filter->Prior_DataMatrix.j_length=0;
	
	kalman_filter->Posteriori_DataMatrix.data=NULL;
	kalman_filter->Posteriori_DataMatrix.i_length=0;
	kalman_filter->Posteriori_DataMatrix.j_length=0;
	
	kalman_filter->GainMatrix.data=NULL;
	kalman_filter->GainMatrix.i_length=0;
	kalman_filter->GainMatrix.j_length=0;
	
	kalman_filter->UMatrix.data=NULL;
	kalman_filter->UMatrix.i_length=0;
	kalman_filter->UMatrix.j_length=0;
	
	kalman_filter->ZMatrix.data=NULL;
	kalman_filter->ZMatrix.i_length=0;
	kalman_filter->ZMatrix.j_length=0;
}

void KalmanFilter_Process(KalmanFilter_DataHandleTypeDef* kalman_filter,const float* matrix_u)
{
	//获取当前t时刻的输入量
	for(int i=0;i<kalman_filter->UMatrix.i_length;i++)
	{
		for(int j=0;j<kalman_filter->UMatrix.j_length;j++)
		{
			*(kalman_filter->UMatrix.data+i*kalman_filter->UMatrix.j_length+kalman_filter->UMatrix.j_length)=*(matrix_u+i*kalman_filter->UMatrix.j_length+kalman_filter->UMatrix.j_length);

		}
	}
	//状态先验估计 X~(t+1)=AX^(t)+BU(t) 
	//Matrix_Multiply(&kalman_filter->AMatrix,&kalman_filter->Posteriori_DataMatrix,);
	//更新方差矩阵Q Q(t+1)=(average-x~(t+1))^2/N
	
	//状态观测估计 Z(t+1)=HX^(t)
	
	//更新方差矩阵R R(t+1)=(average-z(t+1))^2/N
	
	//先验误差协方差 P~(t+1)=AP^(t)AT + Q
	 
    //更新kalman Gain Gain(t+1)=P~(t+1)HT/HP~(t+1)HT + R
	
	//后验估计 X^(t+1)=X~(t+1) + Gain(t+1)( Z(t+1) - HX~(t+1))
	
	//更新误差协方差 P^(t+1)=(I - Gain(t+1)H)P~(t+1)
	
}