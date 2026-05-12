/* ****************************
 * Project description:
 *
 * 批量梯度下降(BGD)拟合算法头文件
 * 方程：z = A + Bx + Cy + Dx^2 + Exy + Fy^2 + Gx^3 + Hx^2y + Ixy^2 + Jy^3
 *
 * Author: 创新基地 -> 2019 Mao
 *
 * Creation Date: 2021/10/21 - 1
 * ****************************/

#ifndef USER_BGD_H
#define USER_BGD_H

/* ***************************** Include & Define Part     	*****************************/
#include "User_header.h"

#define VarNum 10				// 变量数
#define LearnCount 35		// 学习点数
#define LearnRate 0.01	// 学习率
#define LearnTime 50000	// 最大迭代次数

/* ***************************** Function Declaration Part  *****************************/

uint16_t Grad_Descent( float coorX[LearnCount] , float coorY[LearnCount] , float data[LearnCount] , float result[VarNum] );

uint16_t Check_BGD( float var_group[] , float result[] );

float GetSum( float data[] , unsigned int length );


#endif
