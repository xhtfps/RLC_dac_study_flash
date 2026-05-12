/* ****************************
 * Project description:
 *
 * SPI initialization file
 *
 * Author: 创新基地 -> 2019 Mao
 *
 * Creation Date: 2021/10/17 - 1
 * ****************************/
 
 /* ***************************** Include & Define Part     	*****************************/
#include "User_SPI.h"

/* ***************************** Initialization Part        *****************************
 * 初始化函数区
 * */

// SPI外设初始化(芯片自带SPI)
void User_SPI_Init()
{
	SPI_InitTypeDef SPI_InitStruct;
	
	// 初始化GPIO口
	SPI_GPIO_Init( 0 );
	
	// 使能SPI1时钟
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_SPI1 , ENABLE );
	
	// 配置SPI1
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;	// 波特率四分配
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;												// 第一个边沿捕获
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;  												// 时钟极性为低电平
	SPI_InitStruct.SPI_CRCPolynomial = 7; 													// CRC校验(未启用)
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_16b;									// 16位数据
	SPI_InitStruct.SPI_Direction = SPI_Direction_1Line_Tx;					// 方向为单工发送数据
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;									// 高位在前传输
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;											// 主机模式
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;													// 软件管理NSS位
	SPI_Init( SPI1 , &SPI_InitStruct );
	
	// 使能SPI1
	//SPI_Cmd( SPI1 , ENABLE );
}

// SPI的GPIO口初始化 参数：1>工作模式(0为自带SPI，1为GPIO口模拟SPI)
void SPI_GPIO_Init( uint8_t mode )
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_AHB1PeriphClockCmd( SPI_GPIO_CLK , ENABLE );
	
	if( mode == 0 )	// STM32自带的SPI外设配置
	{
		GPIO_PinAFConfig( SPI_Port , SCK_Source | MOSI_Source | MISO_Source | NSS_Source , GPIO_AF_SPI1 );
		
		GPIO_InitStruct.GPIO_Pin = SCK_Pin | MOSI_Pin | NSS_Pin;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
		GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
		GPIO_Init( SPI_Port , &GPIO_InitStruct );
		
		GPIO_InitStruct.GPIO_Pin = MISO_Pin;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
		GPIO_Init( SPI_Port , &GPIO_InitStruct );
	}
	else if( mode == 1 ) // 利用IO口模拟的SPI通信配置
	{
		GPIO_InitStruct.GPIO_Pin = SCK_Pin | MOSI_Pin | NSS_Pin;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_InitStruct.GPIO_Speed = GPIO_Fast_Speed;
		GPIO_Init( SPI_Port , &GPIO_InitStruct );
		
		GPIO_InitStruct.GPIO_Pin = MISO_Pin;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
		GPIO_Init( SPI_Port , &GPIO_InitStruct );
	}
	
}

/* ***************************** Custom Function Part       *****************************
 * 自定义函数区
 */

// 发送数据(仅用于GPIO口模拟SPI通信) 参数：1>数据；2>数据长度；3>通信相位(0为下降沿装载数据，1为上升沿)
void User_SPI_SendData( uint32_t data , uint8_t length , uint8_t pha )
{
	int8_t count;
	
	GPIO_ResetBits( SPI_Port , NSS_Pin );
	delay_us(1);
	
	if( pha == 0 )
		for( count = length-1 ; count >= 0 ; count -- )
		{
			GPIO_SetBits( SPI_Port , SCK_Pin );
			
			if( (data >> count ) & 0x00000001 )
				GPIO_SetBits( SPI_Port , MOSI_Pin );
			else
				GPIO_ResetBits( SPI_Port , MOSI_Pin );
			delay_us(1);
			
			GPIO_ResetBits( SPI_Port , SCK_Pin );
			delay_us(1);
		
		}
	else
		for( count = length-1 ; count >= 0 ; count -- )
		{
			GPIO_ResetBits( SPI_Port , SCK_Pin );
			
			if( (data >> count ) & 0x00000001 )
				GPIO_SetBits( SPI_Port , MOSI_Pin );
			else
				GPIO_ResetBits( SPI_Port , MOSI_Pin );
			delay_us(1);
			
			GPIO_SetBits( SPI_Port , SCK_Pin );
			delay_us(1);
		
		}
	
	GPIO_SetBits( SPI_Port , NSS_Pin );
	
}







