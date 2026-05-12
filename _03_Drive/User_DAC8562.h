/* ****************************
 * Project description:
 *
 * DAC8526 initialization head file
 *
 * Author: 创新基地 -> 2019 Mao
 *
 * Creation Date: 2021/10/27 - 1
 * ****************************/

#ifndef USER_DAC8562_H
#define USER_DAC8562_H

/* ***************************** Include & Define Part     	*****************************/
#include "User_header.h"

#define LDAC_Pin GPIO_Pin_1
#define CLR_Pin GPIO_Pin_3

#define ACDataLength 32

/* ***************************** Function Declaration Part  *****************************/

void DAC8562_Init( uint8_t mode );
void DAC8562_GPIOInit( void );

void DAC8562_OutDC( float vol );




#endif
