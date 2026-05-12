/* ****************************
 * Project description:
 *
 * AD8370 initialization head file
 *
 * Author: 创新基地 -> 2019 Mao
 *
 * Creation Date: 2021/10/27 - 1
 * ****************************/

#ifndef USER_AD8370_H
#define USER_AD8370_H

/* ***************************** Include & Define Part     	*****************************/
#include "User_header.h"

#define LTCH_CLK RCC_AHB1Periph_GPIOF
#define LTCH_Port GPIOF

#define LTCH1_Pin GPIO_Pin_1
#define LTCH2_Pin GPIO_Pin_3

/* ***************************** Function Declaration Part  *****************************/

void AD8370_Init( void );
void AD8370_GPIOInit( void );

void AD8370_SetTimes( float times , uint8_t mode );




#endif
