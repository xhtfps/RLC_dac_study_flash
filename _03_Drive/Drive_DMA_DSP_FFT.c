/*
*********************************************************************************************************
*                                              _04_OS
* File			 : Drive_DMA.HD.c  双ADC同步采样
* By  			 : 
* platform   : STM32F407ZG
*	Data   		 : 2022/7/9
* function 	 : 双ADC同步采样配置程序
*********************************************************************************************************
*/
#include "Drive_DMA_DSP_FFT.h"  
#include "arm_math.h"//DSP库
/*********************************************************************************************************/
/*变量声明*/

//#define  ADC_PRECISION 			ADC_Resolution_12b 
//u16  ADC1_DMA2_Buff[ADC1_DMA2_LENTH*2] = {0};		 
//u16  ADC2_DMA2_Buff[ADC2_DMA2_LENTH] = {0};		
//u16  ADC3_DMA2_Buff[ADC3_DMA2_LENTH] = {0};	
//u16  DAC1_DMA1_Buff[DAC1_DMA1_LENTH] = {0};	
u32 ADCData[ADCDataLength*2] = {0};//adc输入数组
float ADCDataOut[ADCDataLength] = {0};//
float ADCDataInput[2*ADCDataLength] = {0};
float fft_inputbuf1[ADCDataLength*2];	//FFT输入数组
float fft_outputbuf1[ADCDataLength];	//FFT输出数组
float fft_inputbuf2[ADCDataLength*2];	//FFT输入数组
float fft_outputbuf2[ADCDataLength];	//FFT输出数组
arm_cfft_radix4_instance_f32 scfft;//调用DSP指令进行FFT
u8 FFTNum;
u8 computeflag;
u32 BaseIndexADC1;
float BaseValueADC1;	//FFT输入数组 实部+虚部
u32 BaseIndexADC2;
float BaseValueADC2;	//FFT输入数组 实部+虚部
float pha1,pha2;
float ADC1VOL;
float ADC2VOL;
float pha;
float ADCfre;
float ADCfre1;
extern float Sample_rate;//采样率
u32 PhaseIndex;
/*********************************************************************************************************/

//功能函数区
void User_GPIO_doubleInit()	
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	// Enable GPIO clock
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA , ENABLE );
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOC , ENABLE );

	// Configure ADC -> GPIO
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	
	GPIO_Init( GPIOA , &GPIO_InitStruct );//初始化ADGPIO口
}

// ADC初始化
void User_ADC_double_Init()	
{
	// Creat ADC InitStruct and ComInitStruct
	ADC_InitTypeDef ADC_InitStruct;
	ADC_CommonInitTypeDef ADC_ComInitStruct;
	
	// Enable the ADC clock
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_ADC1 , ENABLE );
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_ADC2 , ENABLE );
	
	// Configure ADC Common
	ADC_ComInitStruct.ADC_Mode = ADC_DualMode_RegSimult;										// 工作模式(双通道规则同步)
	ADC_ComInitStruct.ADC_Prescaler = ADC_Prescaler_Div4;										// 时钟分频系数
	ADC_ComInitStruct.ADC_DMAAccessMode = ADC_DMAAccessMode_2;							// DMA传输模式
	ADC_ComInitStruct.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;	// 采样间隔延迟
	
	ADC_CommonInit( &ADC_ComInitStruct );
	
	// Configure ADC
	ADC_InitStruct.ADC_Resolution            = ADC_Resolution_12b;       				// 精度设置(12位)
	ADC_InitStruct.ADC_ScanConvMode          = ENABLE;													// 是否扫描
	ADC_InitStruct.ADC_ContinuousConvMode    = DISABLE;													// 是否连续
	ADC_InitStruct.ADC_ExternalTrigConvEdge  = ADC_ExternalTrigConvEdge_Rising; // 触发配置
	ADC_InitStruct.ADC_ExternalTrigConv      = ADC_ExternalTrigConv_T3_TRGO;		// 触发源
	ADC_InitStruct.ADC_DataAlign             = ADC_DataAlign_Right;							// 数据右对齐
	ADC_InitStruct.ADC_NbrOfConversion       = 1;    
	
	ADC_Init( ADC1 , &ADC_InitStruct );	
	ADC_Init( ADC2 , &ADC_InitStruct );
	
	// Configure ADC Channel
	ADC_RegularChannelConfig( ADC1 , ADC_Channel_1 , 1 , ADC_SampleTime_15Cycles );
	ADC_RegularChannelConfig( ADC2 , ADC_Channel_2 , 1 , ADC_SampleTime_15Cycles );
	
	ADC_DMARequestAfterLastTransferCmd( ADC1 , ENABLE );
	ADC_DMARequestAfterLastTransferCmd( ADC2 , ENABLE );
	
	// Enable ADC -> DMA
	ADC_DMACmd( ADC1 , ENABLE );
	ADC_DMACmd( ADC2 , ENABLE );
	
	// Enable ADC
	ADC_Cmd( ADC1 , ENABLE );
	ADC_Cmd( ADC2 , ENABLE );
}
//双通道同步采样
// DMA初始化
void User_DMA_doubleInit()	
{
	DMA_InitTypeDef DMA_InitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	/**************************ADC DMA NVIC配置**********************************/  
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	DMA_DeInit( DMA2_Stream0 );
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);	// 初始化数据流
	
	DMA_InitStruct.DMA_Channel = DMA_Channel_0;														// DMA通道
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&ADC -> CDR;				// 源地址(外设地址)
	DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)&ADCData;							// 目标地址
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;									// 传输方向(外设->内存)
	DMA_InitStruct.DMA_BufferSize = ADCDataLength;												// 数据大小
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;					// 源地址是否递增
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;									// 目标地址是否递增
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;	// 源数据大小(字节)
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;					// 目标数据大小(字节)
	DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;													// 工作模式
	DMA_InitStruct.DMA_Priority = DMA_Priority_High;
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init( DMA2_Stream0 , &DMA_InitStruct );
	DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);												//传输完成中断   
	DMA_Cmd(DMA2_Stream0, ENABLE); 
}

// 定时器初始化
void User_TIM_doubleInit(u16 arr,u16 psc)	
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
  TIM_Cmd(TIM3, DISABLE);
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); //初始化定时器
  
  TIM_TimeBaseStructure.TIM_Period = arr-1;         //计数值  42MHz*2/1000/168=500Hz  
  TIM_TimeBaseStructure.TIM_Prescaler = psc-1;     //预分频器1000分频
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;     //时钟输入1分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  
  TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);  //选择TIM3的UPDATA事件更新为触发源
	
  TIM_ARRPreloadConfig(TIM3, ENABLE); //允许TIM3定时重载
}
void DMA2_Stream0_IRQHandler(void) 
{
    if(DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0))
    {
         computeflag=1;//完成标志位
        ADC_DMACmd(ADC1,DISABLE);
        ADC_DMACmd(ADC2,DISABLE);
		
        TIM_Cmd(TIM3, DISABLE);	
        DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
        DMA_ClearFlag(DMA2_Stream0,DMA_IT_TCIF0);
    }
//		LED1=!LED1;//指示中断正常运行
		DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);


} 

void FFT_Handle()
{
    u32 i;
    u16 OutDelayCount = 0;
	u16 temp1 = 0, temp2 = 0;
	OutDelayCount = 0;
    // 同时使能两个ADC的DMA！
    ADC_DMACmd(ADC1, ENABLE);
    ADC_DMACmd(ADC2, ENABLE);

    TIM_Cmd(TIM3, ENABLE);
    while(computeflag != 1)
    {
        OutDelayCount++;
        if(OutDelayCount > 300)
            break;
        OSTimeDly(10);
    }
    TIM_Cmd(TIM3, DISABLE);
    computeflag = 0;

    // FFT数据准备
    for(i=0; i<ADCDataLength; i++)
    {
        fft_inputbuf1[2*i]   = (double)(ADCData[i] & 0xffff) * 3.3 / 4096;
        fft_inputbuf1[2*i+1] = 0;
        fft_inputbuf2[2*i]   = (double)(ADCData[i] >> 16) * 3.3 / 4096;
        fft_inputbuf2[2*i+1] = 0;
    }

    // ADC1处理（只清除直流分量，不清除基频！）
    arm_cfft_radix4_init_f32(&scfft, ADCDataLength, 0, 1);
    arm_cfft_radix4_f32(&scfft, fft_inputbuf1);
    arm_cmplx_mag_f32(fft_inputbuf1, fft_outputbuf1, ADCDataLength);
    fft_outputbuf1[0] = 0;  // ← 只清除直流分量（索引0）！
    arm_max_f32(fft_outputbuf1, ADCDataLength/2, &BaseValueADC1, &BaseIndexADC1);
//    pha1 = atan2(fft_inputbuf1[2*BaseIndexADC1+1], fft_inputbuf1[2*BaseIndexADC1]) * 180 / PI;

    // ADC2处理（只清除直流分量，不清除基频！）
    arm_cfft_radix4_init_f32(&scfft, ADCDataLength, 0, 1);
    arm_cfft_radix4_f32(&scfft, fft_inputbuf2);
    arm_cmplx_mag_f32(fft_inputbuf2, fft_outputbuf2, ADCDataLength);
    fft_outputbuf2[0] = 0;  // ← 只清除直流分量（索引0）！
    arm_max_f32(fft_outputbuf2, ADCDataLength/2, &BaseValueADC2, &BaseIndexADC2);
//  pha2 = atan2(fft_inputbuf2[2*BaseIndexADC2+1], fft_inputbuf2[2*BaseIndexADC2]) * 180 / PI;
	
	if (BaseValueADC1 >= BaseValueADC2)
	{
	  PhaseIndex = BaseIndexADC1;
	}
	else
	{
	  PhaseIndex = BaseIndexADC2;
	}

	pha1 = atan2(fft_inputbuf1[2 * PhaseIndex + 1], fft_inputbuf1[2 * PhaseIndex]) * 180 / PI;
	pha2 = atan2(fft_inputbuf2[2 * PhaseIndex + 1], fft_inputbuf2[2 * PhaseIndex]) * 180 / PI;
  
    pha = pha1 - pha2;
    ADCfre = BaseIndexADC1 * Sample_rate / ADCDataLength;
    ADCfre1 = BaseIndexADC2 * Sample_rate / ADCDataLength;

    ADC1VOL = 4 * BaseValueADC1 / ADCDataLength * 1.0295f;
    ADC2VOL = 4 * BaseValueADC2 / ADCDataLength * 1.0295f;
}
/*********************************************************2022.7.09*************（HD）*********************************************/



