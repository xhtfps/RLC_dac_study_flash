#include "Drive_AD8370.h"

/*
 * Return:      void
 * Parameters:  void
 * Description: AD8370初始化
 */
void AD8370_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG | RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin = AD8370_1_SDIO|AD8370_1_SCLK|AD8370_1_CS;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOG,&GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = AD8370_2_SDIO|AD8370_2_SCLK|AD8370_2_CS;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOB,&GPIO_InitStructure);
  
    AD8370_1_CS_1();
    AD8370_1_SCLK_1();
    AD8370_1_SDIO_1();
	  AD8370_2_CS_1();
    AD8370_2_SCLK_1();
    AD8370_2_SDIO_1();
}

/*
 * Return:      void
 * Parameters:  Data: 要写入的字节
 * Description: 向AD8370写入一个字节
 */
void AD8370_1_WriteByte(uint8_t Data)
{
    uint8_t mask;
    
    AD8370_1_CS_0();
	  delay_us(5);
    
    for(mask=0x80; mask; mask>>=1)
    {
        AD8370_1_SCLK_0();
			  delay_us(5);
        
        if(Data & mask)
            AD8370_1_SDIO_1();
        else
            AD8370_1_SDIO_0();

				delay_us(1);
        AD8370_1_SCLK_1();
				delay_us(8);
    }
    AD8370_1_SDIO_1();
    
    AD8370_1_CS_1();
}

/*
 * Return:      void
 * Parameters:  Data: 要写入的字节
 * Description: 向AD8370写入一个字节
 */
void AD8370_2_WriteByte(uint8_t Data)
{
    uint8_t mask;
    
    AD8370_2_CS_0();
	  delay_us(5);
    
    for(mask=0x80; mask; mask>>=1)
    {
        AD8370_2_SCLK_0();
			  delay_us(5);			 
        
        if(Data & mask)
            AD8370_2_SDIO_1();
        else
            AD8370_2_SDIO_0();

				delay_us(1);
        AD8370_2_SCLK_1();
				delay_us(8);
    }
    AD8370_2_SDIO_1();
    
    AD8370_2_CS_1();
}

/*
 * Return:      void
 * Parameters:  Gain: 要设置的增益值（0.0 --- 50.118899）
 * Description: 设置AD8370的增益
 */
void AD8370_SetGain(float Gain,u8 num)
{
    uint8_t data;
    
    if(Gain < 7.079488f)
    {
        data = Gain / 0.055744f;
        data &= 0x7F;
    }
    else
    {
        data = Gain / (0.055744f * 7.079458f);
        data |= 0x80;
    }
		
		if(num == 1)
       AD8370_1_WriteByte(data);
		else if(num == 2)
       AD8370_2_WriteByte(data);
}
