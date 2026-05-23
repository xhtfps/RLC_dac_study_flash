#ifndef __DRIVE_DMA_DSP_FFTH
#define __DRIVE_DMA_DSP_FFTH

/* 头文件包含 ----------------------------------------------------------------*/  
#include "User_header.h"

/* 全局宏定义 ----------------------------------------------------------------*/ 
//#define 	ADC1_DMA2_LENTH 		4000
//#define 	DAC1_DMA1_LENTH			2000 

#define    ADCDataLength    	1024//基四的采样点数
//#define    ADCDsamplingfre    256000//采样频率
//#define    ADC_DR  ((uint32_t)0x40012308)
//#define 	ADC1_DMA2_LENTH 		9//2048//1
//#define 	ADC2_DMA2_LENTH 		8//2048//1
//#define 	ADC3_DMA2_LENTH 		8
//#define 	DAC1_DMA1_LENTH			1


/* 结构体声明 ----------------------------------------------------------------*/ 
																	
/* 全局变量声明 --------------------------------------------------------------*/ 

//extern u16 ADC1_DMA2_Buff[ADC1_DMA2_LENTH];
//extern u16 ADC2_DMA2_Buff[ADC2_DMA2_LENTH];
//extern u16 ADC3_DMA2_Buff[ADC3_DMA2_LENTH];
//extern u16 DAC1_DMA1_Buff[DAC1_DMA1_LENTH];

extern float ADCfre;
extern float ADCfre1;
extern float ADC1VOL;
extern float ADC2VOL;
extern float pha;
extern float FFT_TargetFre;
/* 全局函数声明 --------------------------------------------------------------*/    

void User_ADC_double_Init();

void User_DMA_doubleInit();

void User_TIM_doubleInit(u16 arr,u16 psc);

void User_GPIO_doubleInit();

void  FFT_Handle();

void FFT_SetTargetFre(float targetFre);

//void ADC1_DMA2_Reload(void); 
//void ADC3_DMA2_Reload(u8 Speed);
//void ADC3_DMA2_Init(void);	

//void DAC1_DMA1_Init(void);
//void DAC1_DMA1_Reload(u32 speed);
//float Phase_ana(u8 flag);
#endif



