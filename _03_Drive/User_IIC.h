/* ****************************
 * Project description:
 *
 * IIC initialization head file
 *
 * Author: 创新基地 -> 2019 Mao
 *
 * Creation Date: 2021/10/28 - 1
 * ****************************/

#ifndef USER_IIC_H
#define USER_IIC_H

/* ***************************** Include & Define Part     	*****************************/
#include "User_header.h"

#define IIC_GPIO_CLK RCC_AHB1Periph_GPIOE
#define IIC_Port GPIOE

#define SCL_Pin GPIO_Pin_0
#define SDA_Pin GPIO_Pin_2

/* ***************************** Function Declaration Part  *****************************/

// IIC驱动相关
void IIC_Init(void);

void IIC_Start(void);
void IIC_Stop(void);
uint8_t IIC_SendData( uint8_t data );
uint8_t IIC_RecData(void);



#endif
