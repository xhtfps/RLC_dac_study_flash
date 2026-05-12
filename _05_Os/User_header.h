/*
*********************************************************************************************************
                                               _04_OS
    File			 : User_header.c
    By  			 : Muhe
    platform   : STM32F407ZG
	Data   		 : 2018/7/16
*********************************************************************************************************
*/
#ifndef __USER_HEADER_H
#define __USER_HEADER_H

#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t

/* OS 文件*/
#include "Os_cpu.h"
#include "Os_UI.h"
#include "Os_malloc.h"

/* 系统文件*/
#include "stdio.h"
#include <stdlib.h>
#include "arm_math.h"
#include "stm32f4xx.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_can.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_syscfg.h"
#include "sys.h"
#include "usart.h"
#include "delay.h"

/*应用文件*/

#include "App_Touch.h"
#include "App_LED.h"

#include "User.h"

/*驱动文件*/

#include "TFT_LCD.h"
#include "W25Q64.h"
#include "fontupd.h"

#include "Drive_GPIO.h"
#include "Drive_PS2.h"
#include "Drive_Communication.h"
#include "Drive_FFT.h"
#include "Drive_SI5351a.h" 
#include "Drive_ADF4351.h"
#include "Drive_AD8370.h"
#include "Drive_DMA.h"
#include "Drive_Show.h"
#include "Drive_ADS1256.h"
#include "Drive_DMA_DSP_FFT.h"  
#include "Drive_DAC.h"
#include "Drive_Flash.h"


#include "User_ADC.h"
#include "User_DAC.h"

#include "User_SPI.h"
#include "User_IIC.h"
#include "User_BGD.h"
#include "User_DAC8562.h"
#include "User_ADC.h"
//#include "User_AD8370.h"
#include "User_PGA2310.h"

#endif


