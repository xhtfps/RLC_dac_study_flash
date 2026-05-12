/* ****************************
 * Project description:
 *
 * SPI initialization head file
 *
 * Author: 创新基地 -> 2019 Mao
 *
 * Creation Date: 2021/10/17 - 1
 * ****************************/

#ifndef USER_SPI_H
#define USER_SPI_H

/* ***************************** Include & Define Part     	*****************************/
#include "User_header.h"

#define SPI_GPIO_CLK RCC_AHB1Periph_GPIOE
#define SPI_Port GPIOE

#define NSS_Source GPIO_PinSource0
#define SCK_Source GPIO_PinSource2
#define MISO_Source GPIO_PinSource6
#define MOSI_Source GPIO_PinSource4

#define NSS_Pin GPIO_Pin_0
#define SCK_Pin GPIO_Pin_2
#define MISO_Pin GPIO_Pin_6
#define MOSI_Pin GPIO_Pin_4

/* ***************************** Function Declaration Part  *****************************/

// SPI驱动相关
void User_SPI_Init(void);
void SPI_GPIO_Init( uint8_t mode );
void User_SPI_SendData( uint32_t data , uint8_t length , uint8_t pha );




#endif
