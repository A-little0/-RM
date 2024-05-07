#ifndef __KALMAN_FILTER_H
#define __KALMAN_FILTER_H


#include "main.h"

typedef struct{
		float* data;
		uint8_t i_length;
		uint8_t j_length;
}Matrix_DataTypDef;

typedef struct{
	/*系统属性*/
	Matrix_DataTypDef AMatrix;//状态转移矩阵
	
	Matrix_DataTypDef BMatrix;//控制矩阵
	
	Matrix_DataTypDef HMatrix;//空间转移系数矩阵
	/*中间量*/
	Matrix_DataTypDef Priori_PMatrix;//先验协方差矩阵
	Matrix_DataTypDef LastPriori_PMatrix;//上一次协方差矩阵
	
	Matrix_DataTypDef QMatrix;//过程方差矩阵
	
	Matrix_DataTypDef RMatrix;//测量方差矩阵
	
	Matrix_DataTypDef GainMatrix;//kalman增益
	/*输入量*/
	Matrix_DataTypDef UMatrix;//输入矩阵
	/*状态 输出量*/
    Matrix_DataTypDef Prior_DataMatrix;//先验估计值
	Matrix_DataTypDef Posteriori_DataMatrix;//后验估计值
	
	Matrix_DataTypDef ZMatrix;//观测值
	
	
}KalmanFilter_DataHandleTypeDef;

#endif