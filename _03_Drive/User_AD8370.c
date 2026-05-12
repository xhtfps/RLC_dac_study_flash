/* ****************************
 * Project description:
 *
 * AD8370 initialization file
 *
 * Author: 创新基地 -> 2019 Mao
 *
 * Creation Date: 2021/10/27 - 1
 * ****************************/
 
/* ***************************** Include & Define Part     	*****************************
 * 头文件声明及宏定义区
 * */
#include "User_AD8370.h"

#define Ver 0.055744
#define Pre 7.079458
#define MaxT 50.1187

/* ***************************** Variable definition Part   *****************************
 * 全局变量定义区
 * */

/* ***************************** Initialization Part        *****************************
 * 初始化函数区
 * */
 
// AD8370初始化
void AD8370_Init()
{
	// 初始化GPIO口
	AD8370_GPIOInit();
	
	AD8370_SetTimes( 1 , 2 );
	
}

// 初始化AD8370的GPIO空
void AD8370_GPIOInit()
{
	GPIO_InitTypeDef GPIO_InitStruct;

	SPI_GPIO_Init( 1 );

	RCC_AHB1PeriphClockCmd( LTCH_CLK , ENABLE );

	GPIO_InitStruct.GPIO_Pin = LTCH1_Pin | LTCH2_Pin;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Fast_Speed;
	GPIO_Init( LTCH_Port , &GPIO_InitStruct );
}

/* ***************************** Custom Function Part       *****************************
 * 自定义函数区
 */

// 设置增益 参数：1>放大倍数；2>发送模式(0为控制第一级，1为第二级，2为两级均控制)
void AD8370_SetTimes( float times , uint8_t mode )
{
	uint32_t data = 0;
	
	if( times > (float)MaxT || times < 0 )
		times = 1;
	
	if( times > (float)Pre )
	{
		data = times / ( Pre * Ver );
		data |= 0x80;
	}
	else 
	{
		data = times / (float)Ver;
		data &= 0x7f;
	}
	
	if( mode == 0 )
	{
		GPIO_ResetBits( LTCH_Port , LTCH1_Pin );
		User_SPI_SendData( data , 8 , 1 );
		GPIO_SetBits( LTCH_Port , LTCH1_Pin );
	}
	else if( mode == 1 )
	{
		GPIO_ResetBits( LTCH_Port , LTCH2_Pin );
		User_SPI_SendData( data , 8 , 1 );
		GPIO_SetBits( LTCH_Port , LTCH2_Pin );
	}
	else
	{
		GPIO_ResetBits( LTCH_Port , LTCH1_Pin );
		GPIO_ResetBits( LTCH_Port , LTCH2_Pin );
		User_SPI_SendData( data , 8 , 1 );
		GPIO_SetBits( LTCH_Port , LTCH1_Pin );
		GPIO_SetBits( LTCH_Port , LTCH2_Pin );
	}
	
}


