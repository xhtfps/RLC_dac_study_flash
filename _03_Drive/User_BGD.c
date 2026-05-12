/* ****************************
 * Project description:
 *
 * 批量梯度下降(BGD)拟合算法
 * 方程：z = A + Bx + Cy + Dx^2 + Exy + Fy^2 + Gx^3 + Hx^2y + Ixy^2 + Jy^3
 *
 * Author: 创新基地 -> 2019 Mao
 *
 * Creation Date: 2021/10/21 - 1
 * ****************************/
 
 /* ***************************** Include & Define Part     	*****************************/
#include "User_BGD.h"

/* ***************************** Initialization Part        *****************************
 * 初始化函数区
 * */

// 空

/* ***************************** Custom Function Part       *****************************
 * 自定义函数区
 */

// 批量梯度下降拟合 参数：1>横坐标参数数组；2>纵坐标参数数组；3>测试数据数组 3>拟合结果参数数组； 返回：迭代次数
uint16_t Grad_Descent( float coorX[LearnCount] , float coorY[LearnCount] , float data[LearnCount] , float result[VarNum] )
{
	uint16_t count0 , count1;					// 两循环计数器
	float var_group[VarNum] = {0};		// 参数存放数组			
	float step[VarNum];								// 变量步进数组
	float dJ[VarNum][LearnCount];			// 变量偏导数组
	float temp , step_sum;						// 临时变量，步进和

	// 给变量数组赋初值
	//for( count1 = 0 ; count1 < VarNum ; count1 ++ )	
	//	var_group[count1] = 0;

	// 迭代循环
	for( count0 = 0 ; count0 < LearnTime ; count0 ++ )	
	{
		// 求和得偏导值前先循环求出数组数值
		for( count1 = 0 ; count1 < LearnCount ; count1 ++ )	
		{
			temp = var_group[0] + var_group[1]*coorX[count1] + var_group[2]*coorY[count1]	// A + Bx + Cy
				+ var_group[3]*coorX[count1]*coorX[count1]									// Dx^2
				+ var_group[4]*coorX[count1]*coorY[count1]									// Exy
				+ var_group[5]*coorY[count1]*coorY[count1]									// Fy^2
				+ var_group[6]*coorX[count1]*coorX[count1]*coorX[count1]		// Gx^3
				+ var_group[7]*coorX[count1]*coorX[count1]*coorY[count1]		// Hx^2y
				+ var_group[8]*coorX[count1]*coorY[count1]*coorY[count1]		// Ixy^2
				+ var_group[9]*coorY[count1]*coorY[count1]*coorY[count1]		// Jy^3
				- data[count1];																							// Zi

			dJ[0][count1] = temp;
			dJ[1][count1] = temp * coorX[count1];
			dJ[2][count1] = temp * coorY[count1];
			dJ[3][count1] = temp * coorX[count1]*coorX[count1];
			dJ[4][count1] = temp * coorX[count1]*coorY[count1];
			dJ[5][count1] = temp * coorY[count1]*coorY[count1];
			dJ[6][count1] = temp * coorX[count1]*coorX[count1]*coorX[count1];
			dJ[7][count1] = temp * coorX[count1]*coorX[count1]*coorY[count1];
			dJ[8][count1] = temp * coorX[count1]*coorY[count1]*coorY[count1];
			dJ[9][count1] = temp * coorY[count1]*coorY[count1]*coorY[count1];
			
		}

		// 算出各变量步长并步进
		for( count1 = 0 ; count1 < VarNum ; count1 ++ )	
		{
			step[count1] = LearnRate/LearnCount * GetSum( dJ[count1] , LearnCount );
			var_group[count1] = var_group[count1] - step[count1];
		}

		// 前3个数据的步长和约等于0则迭代结束(前3个数据的步长较大)
		step_sum = GetSum( step , 3 );	
		if( step_sum < (float)0.0000001 && step_sum > (float)-0.0000001 )
			break;
	}
	
	// 输出到返回数组
	for( count1 = 0 ; count1 < VarNum ; count1 ++ )	
		result[count1] = var_group[count1];
	
	// 返回迭代次数
	return count0;	
}

// 检验梯度下降算法 参数：1>实际参数数组；2>拟合结果数组；
uint16_t Check_BGD( float var_group[] , float result[] )
{
	uint8_t count;					// 计数器
	uint16_t learn_time;		// 迭代次数
	float data[LearnCount];	// 采集数据
	float coorX[LearnCount] = {-6 , -4 , -2 , 0 , 2 , 4 , 6 			// 横坐标数组
														,-6 , -4 , -2 , 0 , 2 , 4 , 6 
														,-6 , -4 , -2 , 0 , 2 , 4 , 6 
														,-6 , -4 , -2 , 0 , 2 , 4 , 6
														,-6 , -4 , -2 , 0 , 2 , 4 , 6};
	float coorY[LearnCount] = {4 , 4 , 4 , 4 , 4 , 4 , 4					// 纵坐标数组
														,2 , 2 , 2 , 2 , 2 , 2 , 2
														,0 , 0 , 0 , 0 , 0 , 0 , 0
														,-2 , -2 , -2 , -2 , -2 , -2 , -2
														,-4 , -4 , -4 , -4 , -4 , -4 , -4};
	
	// 采集数据赋值(模拟实际采集)
	for( count = 0 ; count < LearnCount ; count ++ )							
	{
		coorX[count] = coorX[count] / 6;
		coorY[count] = coorY[count] / 4;
		data[count] = var_group[0] + var_group[1]*coorX[count] + var_group[2]*coorY[count]
								+ var_group[3] * coorX[count]*coorX[count]
								+ var_group[4] * coorX[count]*coorY[count]
								+ var_group[5] * coorY[count]*coorY[count]
								+ var_group[6] * coorX[count]*coorX[count]*coorX[count]
								+ var_group[7] * coorX[count]*coorX[count]*coorY[count]
								+ var_group[8] * coorX[count]*coorY[count]*coorY[count]
								+ var_group[9] * coorY[count]*coorY[count]*coorY[count];
	}
		
	// 进行梯度拟合
	learn_time = Grad_Descent( coorX , coorY , data , result );		
	
	// 返回迭代次数
	return learn_time;
	
}

// 求和函数 参数：1>求和数组；2>数组长度； 返回：求和结果
float GetSum( float data[] , unsigned int length )
{
	unsigned int count;	// 计数器
	float sum = 0;			// 参数和

	// 循环求和
	for( count = 0 ; count < length ; count ++ )
		sum = sum + data[count];

	// 返回和值
	return sum;
}





