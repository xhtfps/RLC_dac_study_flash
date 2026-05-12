/* ****************************
 * Project description:
 *
 * DAC initialization head file
 *
 * Author: 创新基地 -> 2019 Mao
 *
 * Creation Date: 2021/11/03 - 1
 * ****************************/

#ifndef USER_DAC_H
#define USER_DAC_H

/* ***************************** Include & Define Part     	*****************************
 * 头文件声明及宏定义区
 * */
#include "User_header.h"

#define DACDataLength 32

/* ***************************** Function Declaration Part  *****************************/

void User_DAC_Init( void );

void User_DAC_Configure(void);
void User_DAC_GPIO_Init( uint8_t ch_num );
void User_DAC_DMA_Init( void );
void User_DAC_TIM_Init( void );
void User_DAC_TIM_NVIC_Init( void );

void Set_WaveData( uint8_t sign );
float Set_TriggerFre( float tf );

#endif
