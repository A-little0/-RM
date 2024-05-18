#ifndef __KALMAN_FILTER_H
#define __KALMAN_FILTER_H


#include "main.h"

typedef struct{
		float* data;
		uint8_t i_length;
		uint8_t j_length;
}Matrix_DataTypDef;

typedef struct{
	/*ϵͳ����*/
	Matrix_DataTypDef AMatrix;//״̬ת�ƾ���
	
	Matrix_DataTypDef BMatrix;//���ƾ���
	
	Matrix_DataTypDef HMatrix;//�ռ�ת��ϵ������
	/*�м���*/
	Matrix_DataTypDef Priori_PMatrix;//����Э�������
	Matrix_DataTypDef LastPriori_PMatrix;//��һ��Э�������
	
	Matrix_DataTypDef QMatrix;//���̷������
	
	Matrix_DataTypDef RMatrix;//�����������
	
	Matrix_DataTypDef GainMatrix;//kalman����
	/*������*/
	Matrix_DataTypDef UMatrix;//�������
	/*״̬ �����*/
    Matrix_DataTypDef Prior_DataMatrix;//�������ֵ
	Matrix_DataTypDef Posteriori_DataMatrix;//�������ֵ
	
	Matrix_DataTypDef ZMatrix;//�۲�ֵ
	
	
}KalmanFilter_DataHandleTypeDef;

#endif