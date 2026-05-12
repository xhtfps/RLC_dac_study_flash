#ifndef _Drive_SI5351A_h
#define _Drive_SI5351A_h

#include "User_header.h"

typedef unsigned  short int   uint;
typedef enum {FALSE = 0, TRUE = !FALSE} bool;

//IIC◊‹œﬂ“˝Ω≈≈‰÷√
#define SCL_H         GPIOB_OUT(7) = 1
#define SCL_L         GPIOB_OUT(7) = 0

#define SDA_H         GPIOB_OUT(5) = 1
#define SDA_L         GPIOB_OUT(5) = 0

#define SCL_read      GPIOB_IN(7)
#define SDA_read      GPIOB_IN(5)


#define SI_CLK0_CONTROL	16			// Register definitions
#define SI_CLK1_CONTROL	17
#define SI_CLK2_CONTROL	18
#define SI_SYNTH_PLL_A	26
#define SI_SYNTH_PLL_B	34
#define SI_SYNTH_MS_0		42
#define SI_SYNTH_MS_1		50
#define SI_SYNTH_MS_2		58
#define SI_PLL_RESET		177

#define SI_R_DIV_1		0x00			// R-division ratio definitions
#define SI_R_DIV_2		0b00010000
#define SI_R_DIV_4		0b00100000
#define SI_R_DIV_8		0b00110000
#define SI_R_DIV_16		0b01000000
#define SI_R_DIV_32		0b01010000
#define SI_R_DIV_64		0b01100000
#define SI_R_DIV_128		0b01110000

#define SI_CLK_SRC_PLL_A	0x00
#define SI_CLK_SRC_PLL_B	0b00100000
#define XTAL_FREQ	25000000			// Crystal frequency



void I2C_GPIO_Config(void);
void I2C_SendByte(u8 dat);
uint8_t i2cSendRegister(uint8_t reg, uint8_t data);
void IIC_SI5351A_GPIO_Init(void);


void setupPLL(uint8_t pll, uint8_t mult, uint32_t num, uint32_t denom);
void setupMultisynth(uint8_t synth,uint32_t divider,uint8_t rDiv);

void si5351aSetFrequency(uint32_t frequency , u8 Chanal );






#endif

