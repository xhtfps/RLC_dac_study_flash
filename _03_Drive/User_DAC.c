/* ****************************
 * Project description:
 *
 * DAC initialization file
 *
 * Author: 创新基地 -> 2019 Mao
 *
 * Creation Date: 2021/11/03 - 1
 * ****************************/
 
/* ***************************** Include & Define Part     	*****************************
 * 头文件声明及宏定义区
 * */
#include "User_DAC.h"
 
#define BaseVol 3.299

/* ***************************** Variable definition Part   *****************************
 * 变量定义区
 * */
 
 uint16_t WaveData[DACDataLength] = {0};
 uint16_t WaveCount = 0;

/* ***************************** Initialization Part        *****************************
 * 初始化函数区
 * */

// 初始化DAC
void User_DAC_Init()
{
	// 初始化DAC输出GPIO口
	User_DAC_GPIO_Init( 1 );
	
	// 初始化定时器TIM6
	User_DAC_TIM_Init();
	
	// 定时器TIM6的中断初始化
	//User_DAC_TIM_NVIC_Init();
	
	// DAC数据传输DMA初始化
	User_DAC_DMA_Init();
	
	// 配置DAC
	User_DAC_Configure();
	
	// 通道初始化为1.65V
	DAC_SetChannel1Data( DAC_Align_12b_R , 0x07ff ); 
}

// 配置DAC
void User_DAC_Configure()
{
	DAC_InitTypeDef DAC_InitStruct;
	
	// 重置DAC
	DAC_DeInit();
	
	// 使能DAC时钟
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_DAC , ENABLE );
	
	// 使用默认值配置DAC
	DAC_StructInit( &DAC_InitStruct );	
	DAC_Init( DAC_Channel_1 , &DAC_InitStruct );
	
	// 自定义配置DAC
//	DAC_InitStruct.DAC_Trigger = DAC_Trigger_T6_TRGO;											// 触发源 -> TIM6
//	DAC_InitStruct.DAC_WaveGeneration = DAC_WaveGeneration_None;					// 波形产生 -> 无
//	DAC_InitStruct.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;// 三角波幅值 -> 一位控制
//	DAC_InitStruct.DAC_OutputBuffer = DAC_OutputBuffer_Enable;  					// 输出缓冲区 -> 使能
//	DAC_Init( DAC_Channel_1 , &DAC_InitStruct );
	
	// 允许DAC1使用DMA传输
	DAC_DMACmd( DAC_Channel_1 , ENABLE );
	
	// 使能DAC通道1
	DAC_Cmd( DAC_Channel_1 , ENABLE );
}

// 初始化DAC输出GPIO口
void User_DAC_GPIO_Init( uint8_t ch_num )
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA , ENABLE );
	
	if( ch_num == 1 )
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
	else
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	
	GPIO_Init( GPIOA , &GPIO_InitStruct );
}

// DAC数据传输DMA初始化
void User_DAC_DMA_Init()
{
	DMA_InitTypeDef DMA_InitStruct;
	
	// 使能DMA1时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	
	// 复位DMA1数据流5
	DMA_DeInit( DMA1_Stream1 );																							// 初始化数据流
	
	DMA_InitStruct.DMA_Channel = DMA_Channel_7;															// DMA通道 -> 通道7
	DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)&WaveData;								// 目标地址 -> 波形数据
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&DAC ->DHR12R1 ;			// 源地址 -> DAC1通道12位右对齐数据保持寄存器
	DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;										// 传输方向 -> 内存至外设
	DMA_InitStruct.DMA_BufferSize = DACDataLength;													// 数据大小 -> DAC数据长度
  DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;						// 外设地址递增	-> 否
  DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;										// 内存地址递增 -> 是
  DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;// 源数据大小 -> 半字
  DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;				// 目标数据大小 -> 半字
  DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;														// 工作模式 -> 循环模式
  DMA_InitStruct.DMA_Priority = DMA_Priority_High;												// DMA优先级 -> 高优先级
  DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;         						// FIFO模式 -> 失能
  DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;							// FIFO阈值 -> 满阈值
  DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;								// 内存突发
  DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;				// 外设突发
  DMA_Init( DMA1_Stream1 , &DMA_InitStruct );
	
	// 清除DMA1中断标志
	//DMA_ClearFlag( DMA1_Stream5 , DMA_FLAG_TCIF0 );  
	// 使能传输完成时触发中断
	//DMA_ITConfig( DMA1_Stream5 , DMA_IT_TC , ENABLE );  
	
	// 使能DMA1
	DMA_Cmd(DMA1_Stream1, ENABLE);
}

// DAC输出触发器TIM6初始化
void User_DAC_TIM_Init()
{
	TIM_TimeBaseInitTypeDef TIM_BaseInitStruct;
	
	// 使能TIM6时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
	
	// 配置TIM6
	TIM_BaseInitStruct.TIM_Prescaler = (1000 - 1);						// 预分频系数
	TIM_BaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;	// 计数方式 -> 向上计数
	TIM_BaseInitStruct.TIM_Period = (21 - 1);									// 重装载值
	TIM_BaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;			// 时钟分频
	TIM_BaseInitStruct.TIM_RepetitionCounter = 0;							// 重复计数器(TIM1和TIM8)
	TIM_TimeBaseInit( TIM6 , &TIM_BaseInitStruct );
	
	// 选择触发源 -> 重装载值更新触发
	TIM_SelectOutputTrigger( TIM6 , TIM_TRGOSource_Update );
	
	// 数据更新失能 -> 失能(即使能数据更新)
	TIM_UpdateDisableConfig( TIM6 , DISABLE ); 
	
	TIM_DMACmd( TIM6 , TIM_DMA_Update , ENABLE );
	
	// 允许TIM6触发更新中断
	//TIM_ITConfig( TIM6 , TIM_IT_Update , ENABLE );
	
	// 失能TIM6
	TIM_Cmd( TIM6 , DISABLE );
	
	// 波形输出频率预设为1kHz
	Set_TriggerFre( DACDataLength * 1000 );
}	

// 输出定时器TIM3的中断初始化
void User_DAC_TIM_NVIC_Init()
{
	NVIC_InitTypeDef NVIC_InitStruct;
	
	// Configure TIM6 -> NVIC
	NVIC_InitStruct.NVIC_IRQChannel = TIM6_DAC_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_2 );
	NVIC_Init( &NVIC_InitStruct );
}

/* ***************************** Custom Function Part       *****************************
 * 自定义函数区
 */

// 生成波形数据 参数：1>波形序号( 0:正弦波；1:方波；2:三角波 )
void Set_WaveData( uint8_t sign )
{
	uint8_t count;
	float vol;
	
	if( sign == 0 )
	{
		// 生成正弦波数据
		for( count = 0 ; count < ACDataLength ; count ++ )
		{
			vol = sin( (float)count/ACDataLength * 2*PI  ) + 1;
			WaveData[count] = vol / BaseVol *0x0fff;
		}
	}
	else if( sign == 1 )
	{
		// 生成方波数据
		for( count = 0 ; count < DACDataLength / 2 ; count ++ )
			WaveData[count] = 0x0fff;
		for( count = DACDataLength / 2 ; count < DACDataLength ; count ++ )
			WaveData[count] = 0;
	}
	else if( sign == 2 )
	{
		// 生成三角波数据
		WaveData[0] = 0;
		for( count = 1 ; count < DACDataLength /2 ; count ++ )
		{
			vol = (float)count/DACDataLength *2 *BaseVol;
			WaveData[count] = vol / BaseVol *0x0fff;
			WaveData[DACDataLength - count] = WaveData[count];
		}
		WaveData[DACDataLength /2] = 0x0fff;
	}
}

// 设置TIM6触发率 参数：1>预设触发率 返回：实际触发率
float Set_TriggerFre( float tf )
{
	uint32_t psc = 1;
	uint32_t arr = 1;
	
	TIM_Cmd(TIM6 , DISABLE);						// 关闭定时器
	
	do																	// 循环查找满足条件的PSC和ARR值
	{
		arr = 84000000 / (tf * psc);
		psc ++;
	}
	while( arr > 65535 );
	psc --;
	
	tf = (float)84000000 / (psc * arr); // 由PSC和ARR算出实际采样率
	
	TIM6 -> ARR = arr - 1;							// 改变定时器重装载值改变采样率
	TIM6 -> PSC = psc - 1;							// 改变定时器分频系数改变采样率
	
	return tf;
}

 
/* ***************************** IRQHandler Part    	     	*****************************
 * 中断执行函数区
 */

//void TIM6_DAC_IRQHandler()
//{
//	// 清除中断标志位
//	TIM_ClearITPendingBit(TIM6,TIM_IT_Update);
//	
//	DAC_SetDualChannelData( DAC_Align_12b_R , WaveData[WaveCount] , WaveData[WaveCount] ); 
//	
//	WaveCount ++;
//	WaveCount %= DACDataLength; 
//}
 
 
/* ***************************** 						END 	   	     	*****************************/






