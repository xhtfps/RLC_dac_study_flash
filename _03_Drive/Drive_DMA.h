#ifndef __DRIVE_DMA_H
#define __DRIVE_DMA_H 

/* 头文件包含 ----------------------------------------------------------------*/  
#include "User_header.h"

/* 全局宏定义 ----------------------------------------------------------------*/ 
//#define 	ADC1_DMA2_LENTH 		4000
//#define 	DAC1_DMA1_LENTH			2000 
#define 	ADC1_DMA2_LENTH 		1024//2048//1
#define 	ADC2_DMA2_LENTH 		8//2048//1
#define 	ADC3_DMA2_LENTH 		8
#define 	DAC1_DMA1_LENTH			1
#define    ADCDataLength    	1024//基四的采样点数
#define    ADCDsamplingfre    51200//采样频率

/* 结构体声明 ----------------------------------------------------------------*/ 
																	
/* 全局变量声明 --------------------------------------------------------------*/ 

extern u16 ADC1_DMA2_Buff[ADC1_DMA2_LENTH*2];
extern u16 ADC2_DMA2_Buff[ADC2_DMA2_LENTH];
extern u16 ADC3_DMA2_Buff[ADC3_DMA2_LENTH];
extern u16 DAC1_DMA1_Buff[DAC1_DMA1_LENTH];
extern float ADCfre1;
extern float ADCfre;
extern float ADC1VOL;

/* 全局函数声明 --------------------------------------------------------------*/    
void ADC1_DMA2_Reload(void); 
//void ADC3_DMA2_Reload(u8 Speed);
//void ADC3_DMA2_Init(void);	

//void DAC1_DMA1_Init(void);
//void DAC1_DMA1_Reload(u32 speed);
//void User_ADC12_Init(void);
//void User_GPIO_Init(void);
//void  FFT_Handle(void);
//void User_TIM_Init(u16 arr,u16 psc);
//void User_DMA12_Init(void);
#endif



