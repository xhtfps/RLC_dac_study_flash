/* ****************************
 * Project description:
 *
 * DAC8526 initialization file
 *
 * Author: 创新基地 -> 2019 Mao
 *
 * Creation Date: 2021/10/27 - 1
 * ****************************/
 
/* ***************************** Include & Define Part     	*****************************
 * 头文件声明及宏定义区
 * */
#include "User_DAC8562.h"

/* ***************************** Variable definition Part   *****************************
 * 变量定义区
 * */
 
float DAC8562_Vref = 5;

uint32_t ACData[ACDataLength] = {0};
uint8_t ACMode = 0;
float ACVolRate = 1;

/* ***************************** Initialization Part        *****************************
 * 初始化函数区
 * */
 
// DAC8562初始化
void DAC8562_Init( uint8_t mode )
{
	// 初始化GPIO口
	DAC8562_GPIOInit();
	
	// 初始化定时器(用于输出交流)
	//DAC8562_TIMInit();
	
	// 复位所有寄存器
	User_SPI_SendData( 0x280001 , 24 , 0 );
	
	if( mode == 0 )
	{
		// 使能DAC-A
		User_SPI_SendData( 0x200001 , 24 , 0 );
		
		// 将DAC-A的LDAC引脚配置为芯片自动更新
		User_SPI_SendData( 0x300001 , 24 , 0 );
		
		// 将DAC-A的LDAC引脚配置为硬件IO口更新
		//DAC8562_SendData( 0x300000 );
	}
	else if( mode == 1 )
	{
		// 使能DAC-B
		User_SPI_SendData( 0x200002 , 24 , 0 );	
		
		// 将DAC-B的LDAC引脚配置为芯片自动更新
		User_SPI_SendData( 0x300002 , 24 , 0 );
	}
	
	// 启用内部参考，且增益设为2
	User_SPI_SendData( 0x380001 , 24 , 0 );
	
}

// 初始化DAC8562的GPIO空
void DAC8562_GPIOInit()
{
	GPIO_InitTypeDef GPIO_InitStruct;

	SPI_GPIO_Init( 1 );

	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOF , ENABLE );

	GPIO_InitStruct.GPIO_Pin = LDAC_Pin | CLR_Pin;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Fast_Speed;
	GPIO_Init( GPIOF , &GPIO_InitStruct );
}

/* ***************************** Custom Function Part       *****************************
 * 自定义函数区
 */

// 输出直流 参数：1>电压
void DAC8562_OutDC( float vol )
{
	uint32_t data = 0;
	
	if( vol > DAC8562_Vref || vol < 0 )
		vol = 0;
	
	data = vol / DAC8562_Vref * 0xffff;
	
	User_SPI_SendData( data | 0x180000 , 24 , 0 );
	
}

// 输出交流 参数：1>峰峰值；2>频率
void DAC8562_OutAC( float vpp , uint32_t fre )
{
	
	ACVolRate = vpp / DAC8562_Vref;
	
	//Set_ACFre();
	
	//TIM_Cmd( DAC8562_TIM , DISABLE );
	
}

// 生成波形数据 参数：1>波形序号( 0:正弦波；1:方波；2:三角波 )
void Set_ACData( uint8_t sign )
{
	uint8_t count;
	float vol;
	
	if( sign == 0 )
	{
		for( count = 0 ; count < ACDataLength ; count ++ )
		{
			vol = sin( (float)count/ACDataLength * 2*PI  );
			ACData[count] = vol / DAC8562_Vref *0xffff;
		}
	}
	else if( sign == 1 )
	{
		for( count = 0 ; count < ACDataLength / 2 ; count ++ )
			ACData[count] = 0xffff;
		for( count = ACDataLength / 2 ; count < ACDataLength ; count ++ )
			ACData[count] = 0;
	}
	else if( sign == 2 )
	{
		ACData[0] = 0;
		for( count = 1 ; count < ACDataLength /2 ; count ++ )
		{
			vol = (float)count/ACDataLength *2 *DAC8562_Vref;
			ACData[count] = vol / DAC8562_Vref *0xffff;
			ACData[ACDataLength - count] = ACData[count];
		}
		ACData[ACDataLength /2] = 0xffff;
	}
}

