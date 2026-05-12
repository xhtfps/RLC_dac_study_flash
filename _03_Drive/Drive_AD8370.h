#ifndef _Drive_AD8370_H_
#define _Drive_AD8370_H_

#include "User_header.h"

#define AD8370_1_CS               GPIO_Pin_13
#define AD8370_1_SCLK             GPIO_Pin_11
#define AD8370_1_SDIO             GPIO_Pin_9

#define AD8370_1_CS_0()           GPIOG->BSRRH = AD8370_1_CS
#define AD8370_1_CS_1()           GPIOG->BSRRL = AD8370_1_CS
 
#define AD8370_1_SCLK_0()         GPIOG->BSRRH = AD8370_1_SCLK
#define AD8370_1_SCLK_1()         GPIOG->BSRRL = AD8370_1_SCLK
 
#define AD8370_1_SDIO_0()         GPIOG->BSRRH = AD8370_1_SDIO
#define AD8370_1_SDIO_1()         GPIOG->BSRRL = AD8370_1_SDIO

#define AD8370_2_CS               GPIO_Pin_7
#define AD8370_2_SCLK             GPIO_Pin_5
#define AD8370_2_SDIO             GPIO_Pin_0

#define AD8370_2_CS_0()           GPIOB->BSRRH = AD8370_2_CS
#define AD8370_2_CS_1()           GPIOB->BSRRL = AD8370_2_CS
 
#define AD8370_2_SCLK_0()         GPIOB->BSRRH = AD8370_2_SCLK
#define AD8370_2_SCLK_1()         GPIOB->BSRRL = AD8370_2_SCLK
 
#define AD8370_2_SDIO_0()         GPIOB->BSRRH = AD8370_2_SDIO
#define AD8370_2_SDIO_1()         GPIOB->BSRRL = AD8370_2_SDIO

void AD8370_Init(void);
void AD8370_SetGain(float Gain,u8 num);

#endif
