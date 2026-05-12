/* ****************************
 * Project description:
 *
 * IIC initialization file
 *
 * Author: 创新基地 -> 2019 Mao
 *
 * Creation Date: 2021/10/28 - 1
 * ****************************/
 
 /* ***************************** Include & Define Part     	*****************************/
#include "User_IIC.h"

/* ***************************** Initialization Part        *****************************
 * 初始化函数区
 * */

// IIC初始化
void IIC_Init()
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_AHB1PeriphClockCmd( IIC_GPIO_CLK , ENABLE );
	
	//IO口模拟的IIC通信配置
	GPIO_InitStruct.GPIO_Pin = SCL_Pin | SDA_Pin;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Fast_Speed;
	GPIO_Init( IIC_Port , &GPIO_InitStruct );
	
//	GPIO_InitStruct.GPIO_Pin = SDA_Pin;
//	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
//	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
//	GPIO_Init( IIC_Port , &GPIO_InitStruct );
	
	GPIO_ResetBits( IIC_Port , SCL_Pin );
	GPIO_ResetBits( IIC_Port , SDA_Pin );
	
}

/* ***************************** Custom Function Part       *****************************
 * 自定义函数区
 */

// 发送起始信号
void IIC_Start()
{
	// 数据线拉高，为输出下降沿做准备
	GPIO_SetBits( IIC_Port , SDA_Pin );
	
	// 时钟线拉高
	GPIO_SetBits( IIC_Port , SCL_Pin );
	delay_us(5); 	// 协议要求数据线高电平至少持续4.7us
	
	// 数据线拉低，触发下降沿，即发出起始信号
	GPIO_ResetBits( IIC_Port , SDA_Pin );
	delay_us(5);	// 这里要求大于4us
	
	// 时钟线拉低，允许数据线改变，为后续发送数据做准备
	GPIO_ResetBits( IIC_Port , SCL_Pin );
}

// 发送终止信号
void IIC_Stop()
{
	// 时钟线拉低,允许数据线改变
	GPIO_ResetBits( IIC_Port , SCL_Pin );
	delay_us(1);
	// 数据线拉低，为输出上升沿做准备
	GPIO_ResetBits( IIC_Port , SDA_Pin );
	delay_us(4);
	
	// 时钟线拉高
	GPIO_SetBits( IIC_Port , SCL_Pin );
	delay_us(5);	// 协议要求数据线低电平至少持续4us
	
	// 数据线拉高，触发上升沿，即发出终止信号
	GPIO_SetBits( IIC_Port , SDA_Pin );
	delay_us(5);	// 这里要求大于4.7us
	
	// 时钟线拉高，不允许数据线改变
	//GPIO_SetBits( IIC_Port , SCL_Pin );
}

// 发送数据 参数：1>数据；
uint8_t IIC_SendData( uint8_t data )
{
	uint8_t count; // 计数器
	
	for( count = 8 ; count > 0 ; count -- )
	{
		// 时钟线拉低，允许数据线改变
		GPIO_ResetBits( IIC_Port , SCL_Pin );
		delay_us(1);
		
		// 根据数据改变数据线
		if( (data >> (count-1) ) & 0x01 )
			GPIO_SetBits( IIC_Port , SDA_Pin );
		else
			GPIO_ResetBits( IIC_Port , SDA_Pin );
		delay_us(4);
		
		// 时钟线拉高，将数据发送
		GPIO_SetBits( IIC_Port , SCL_Pin );
		delay_us(5); // 要求大于4us
	}
	
	// 时钟线拉低，允许从机改变数据线应答
	GPIO_ResetBits( IIC_Port , SCL_Pin );
	delay_us(5);
	
	// 时钟线拉高，以读取应答位
	GPIO_SetBits( IIC_Port , SCL_Pin );
	delay_us(5);
	
	// 返回应答位
	if( GPIO_ReadInputDataBit( IIC_Port , SDA_Pin ) )
		return 1;
	else 
		return 0;
	
}

uint8_t IIC_RecData()
{
	uint8_t rec_data;
	
	
	return rec_data;
}







