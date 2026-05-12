/*
*********************************************************************************************************
*                                              _04_OS
* File			 : Drive_DMA.c
* By  			 : 
* platform   : STM32F407ZG
*	Data   		 : 2018/7/16
* function 	 : DMA配置程序
*********************************************************************************************************
*/
#include "Drive_DMA.h"   
#define  ADC_PRECISION 			ADC_Resolution_12b 

/* 私有宏定义 ----------------------------------------------------------------*/ 

/* 私有（静态）函数声明 ------------------------------------------------------*/
 
 extern float Sample_rate;//采样率
//float FREQ_MAX_POINT;//找到最大的频点
//float FREQ;
/* 全局变量定义 --------------------------------------------------------------*/ 
u16  ADC1_DMA2_Buff[ADC1_DMA2_LENTH*2] = {0};		 
u16  ADC2_DMA2_Buff[ADC2_DMA2_LENTH] = {0};		
u16  ADC3_DMA2_Buff[ADC3_DMA2_LENTH] = {0};	
u16  DAC1_DMA1_Buff[DAC1_DMA1_LENTH] = {0};	
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
/* 全局函数编写 --------------------------------------------------------------*/
/**----------------------------------------------------------------------------
* @FunctionName  : ADC1_DMA2_ReLoad()    	 
* @Description   : None 	
* @Data          : 2016/5/24	
* @Explain       : Speed: 0 ~ 7 共8个档位，数值越小采集速度越快	
------------------------------------------------------------------------------*/ 	
void ADC1_DMA2_Reload()
{ 
	DMA_InitTypeDef 	  DMA_InitStructure;  
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef       ADC_InitStructure;
	GPIO_InitTypeDef  	  GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
	
	/**************************ADC DMA NVIC配置**********************************/  
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	
	/* DMA配置 ------------------------------------*/ 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);//DMA2时钟使能 
	DMA_DeInit(DMA2_Stream0);
	while (DMA_GetCmdStatus(DMA2_Stream0) != DISABLE){}//等待DMA可配置 
		 
	/* 配置 DMA Stream */
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;  //通道选择
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->DR;//DMA外设地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)ADC1_DMA2_Buff;//DMA 存储器0地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//外设到存储器模式
	DMA_InitStructure.DMA_BufferSize = ADC1_DMA2_LENTH;//数据传输量 
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//外设数据长度:16位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//存储器数据长度:16位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;// 使用循环模式 DMA_Mode_Normal DMA_Mode_Circular
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//中等优先级
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
	DMA_Init(DMA2_Stream0,&DMA_InitStructure);//初始化DMA Stream
	  
	DMA_SetCurrDataCounter(DMA2_Stream0,ADC1_DMA2_LENTH);//数据传输量   
	DMA_Cmd(DMA2_Stream0, ENABLE);                      //开启DMA传输 	
	DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);
	
	/* GPIO配置 -----------------------------------*/  
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);//使能GPIOA时钟  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;//PA1 通道1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//模拟输入
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//不带上下拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化 
	
  ADC_DeInit();
	
	/* ADC配置 ------------------------------------*/ 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE); //使能ADC1时钟 
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	  //ADC1复位
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);  //复位结束	 

	
	ADC_DeInit();
 
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;                      //ADC独立模式
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;                   //选择时钟频率 2分频
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;       //禁止 DMA 直接访问模式
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;  //采样时间间隔
  ADC_CommonInit(&ADC_CommonInitStructure);
 
  /**********************ADC Init 结构体参数初始化******************************/  
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;                        //分辨率12Bit
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;                                 //开启扫描模式
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;                            //连续转换
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising; //外部上升沿触发
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;        //外部触发源 TIM3_TRGO
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;                        //数据右对齐
  ADC_InitStructure.ADC_NbrOfConversion = 1;                                    //转换通道1个
  ADC_Init(ADC1, &ADC_InitStructure);

//	ADC_ExternalTrigConvCmd(ADC1,ENABLE);
	
	ADC_RegularChannelConfig(ADC1,ADC_Channel_1,1,ADC_SampleTime_3Cycles);  
	ADC_DMARequestAfterLastTransferCmd(ADC1,ENABLE); 
	ADC_DMACmd(ADC1,ENABLE);	
	ADC_Cmd(ADC1,ENABLE);//开启AD转换器	
//  ADC_SoftwareStartConv(ADC1); //软件启动采集   
	
}	

void DMA2_Stream0_IRQHandler(void) 
{
if(DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0))
  {
		computeflag=1;
		DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
  } 

}
void  FFT_Handle()
{		
		u32 i,temp;
		TIM_Cmd(TIM3,ENABLE);
	  while(computeflag!=1);
		TIM_Cmd(TIM3,DISABLE);
		computeflag=0;
	//fft处理中
	  	for(i=0; i<ADCDataLength; i++)
	{
			fft_inputbuf1[2*i]=(double)(ADC1_DMA2_Buff[i])*3.3/4096;//ADC1采样数组输入FFT输入数组中
			fft_inputbuf1[i*2+1] = 0;//虚部全部为0
		
			fft_inputbuf2[2*i]=(double)(ADC1_DMA2_Buff[i]>>16)*3.3/4096;//同理
			fft_inputbuf2[i*2+1] = 0;//虚部全部为0
		
	}
//ADC1的相位与幅度值
	arm_cfft_radix4_init_f32(&scfft,ADCDataLength,0,1);//初始化scfft结构体，设定FFT相关参数
	arm_cfft_radix4_f32(&scfft,fft_inputbuf1);	//FFT计算（基4）
	arm_cmplx_mag_f32(fft_inputbuf1,fft_outputbuf1,ADCDataLength);	//把运算结果复数求模得幅值
	arm_max_f32(fft_outputbuf1, ADCDataLength/2, &BaseValueADC1, &BaseIndexADC1); 	//求最大值(由于傅里叶频谱的对称性采用一半) 直流分量幅度
	fft_outputbuf1[BaseIndexADC1]=0;
	arm_max_f32(fft_outputbuf1, ADCDataLength/2, &BaseValueADC1, &BaseIndexADC1);
	pha1 = atan2(fft_inputbuf1[2*BaseIndexADC1+1],fft_inputbuf1[2*BaseIndexADC1]) * 180 / PI;
	
	

//ADC2的相位与幅度值
	arm_cfft_radix4_init_f32(&scfft,ADCDataLength,0,1);//初始化scfft结构体，设定FFT相关参数
	arm_cfft_radix4_f32(&scfft,fft_inputbuf2);	//FFT计算（基4）
	arm_cmplx_mag_f32(fft_inputbuf2,fft_outputbuf2,ADCDataLength);	//把运算结果复数求模得幅值
	arm_max_f32(fft_outputbuf2, ADCDataLength/2, &BaseValueADC2, &BaseIndexADC2); 	//求最大值(由于傅里叶频谱的对称性采用一半) 直流分量幅度
	fft_outputbuf2[BaseIndexADC2]=0;
	arm_max_f32(fft_outputbuf2, ADCDataLength/2, &BaseValueADC2, &BaseIndexADC2);
	
	pha2 = atan2(fft_inputbuf2[2*BaseIndexADC2+1],fft_inputbuf2[2*BaseIndexADC2]) * 180 / PI;
	
	pha=pha1-pha2;//相位提取双通道采样时候才有意义,单路信号没有意义
	
	
	ADCfre=BaseIndexADC1*Sample_rate/ADCDataLength;//频率提取
	ADCfre1=BaseIndexADC2*Sample_rate/ADCDataLength;
	
	ADC1VOL=4*BaseValueADC1/ADCDataLength;//转换成峰峰值,这是转换规则.详情见文档.
	
	ADC2VOL=4*BaseValueADC2/ADCDataLength;//同理
}
void User_TIM_Init(u16 arr,u16 psc)	
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
void User_GPIO_Init()	
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
void User_ADC12_Init()	
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
void User_DMA12_Init()	
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
	DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)&ADC1_DMA2_Buff;							// 目标地址
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