#include "Drive_ADS1256.h"


//***************************
//		Pin assign	   	
//		STM32						ADS1256

//		GPIOF_Pin_0 		---> SCLK
//		GPIOF_Pin_2			<--- DIN
//		GPIOF_Pin_4  		---> DOUT
//		GPIOF_Pin_6 		<--- DRDY
//		GPIOF_Pin_8 		---> CS
//		GPIOF_Pin_10  	<--- PDWN

//***************************	

/*端口定义*/ 
#define SCLK		 	 GPIO_Pin_2
#define DIN	 			 GPIO_Pin_6
#define DOUT			 GPIO_Pin_4
#define DRDY			 GPIO_Pin_8
#define CS				 GPIO_Pin_10
#define PDWN		   GPIO_Pin_12

#define ADS1256_CS_0()   		GPIO_ResetBits(GPIOF,CS)
#define ADS1256_CS_1()   		GPIO_SetBits(GPIOF,CS)

#define ADS1256_SCLK_0()  	GPIO_ResetBits(GPIOF,SCLK)
#define ADS1256_SCLK_1()  	GPIO_SetBits(GPIOF,SCLK)

#define ADS1256_DIN_0()   	GPIO_ResetBits(GPIOF,DOUT) //注意这里，单片机的DOUT连接ADS1256的DIN
#define ADS1256_DIN_1()   	GPIO_SetBits(GPIOF,DOUT)

#define ADS1256_DOUT			 (GPIOF->IDR & DIN)          //
#define ADS1256_DRDY			 (GPIOF->IDR & DRDY)
#define ADS1256_PDWN		   (GPIOF->IDR & PDWN)



/*
*********************************************************************************************************
*	函 数 名: Init_ADS1256_GPIO
*	功能说明: 初始化ADS1256 GPIO
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void Init_ADS1256_GPIO(void)
{

    GPIO_InitTypeDef  GPIO_InitStructure;

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);

		GPIO_InitStructure.GPIO_Pin = SCLK | DOUT | CS ;
		GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_DOWN; 
		GPIO_Init(GPIOF,&GPIO_InitStructure);
	
	  ADS1256_CS_1();
	
	  GPIO_InitStructure.GPIO_Pin = DIN | DRDY | PDWN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
		GPIO_Init(GPIOF,&GPIO_InitStructure);
}


/*
*********************************************************************************************************
*	函 数 名: SPI_WriteByte
*	功能说明: 向SPI总线发送8个bit数据。 不带CS控制。
*	形    参: _data : 数据
*	返 回 值: 无
*********************************************************************************************************
*/
void SPI_WriteByte(u8 TxData)
{
	u8 i;
	
	/* 连续发送多个字节时，需要延迟一下 */
	delay_us(10);
	
	for(i = 0; i < 8; i++)
	{
		ADS1256_SCLK_1();
		
		if (TxData & 0x80)
			ADS1256_DIN_1();  
		else
			ADS1256_DIN_0();
		
		TxData <<= 1;
		
		delay_us(5);
		ADS1256_SCLK_0();	
		delay_us(5);
	}
	
} 

/*
*********************************************************************************************************
*	函 数 名: SPI_ReadByte
*	功能说明: 从SPI总线接收8个bit数据。 不带CS控制。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
u8 SPI_ReadByte(void)
{
	u8 i;
	u8 read = 0;
	
	for (i = 0; i < 8; i++)
	{
		ADS1256_SCLK_1();
		read = read<<1;
		delay_us(5);
		ADS1256_SCLK_0();
		
		if(ADS1256_DOUT)
				read ++;
		
		delay_us(5);
	}
	
	return read;
}


/*
*********************************************************************************************************
*	函 数 名: ADS1256WREG
*	功能说明: ADS1256 写数据
*	形    参: regaddr:寄存器地址
						databyte:待写入数据
*	返 回 值: 无
*********************************************************************************************************
*/
void ADS1256WREG(u8 regaddr,u8 databyte)
{
	
	ADS1256_CS_0();
	while(ADS1256_DRDY);//当ADS1256_DRDY为低时才能写寄存器
	
	//向寄存器写入数据地址
	SPI_WriteByte(ADS1256_CMD_WREG | (regaddr & 0x0F));
	
	//写入数据的个数n-1
	SPI_WriteByte(0x00);
	
	delay_us(10);
	
	//向regaddr地址指向的寄存器写入数据databyte
	SPI_WriteByte(databyte);
	
	ADS1256_CS_1();
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256RREG
*	功能说明: ADS1256 读数据
*	形    参: regaddr:寄存器地址
*	返 回 值: 无
*********************************************************************************************************
*/
u8 ADS1256RREG(u8 regaddr)
{
	u8 databyte;
	
	ADS1256_CS_0();
	while(ADS1256_DRDY);//当ADS1256_DRDY为低时才能写寄存器
	
	//向寄存器写入数据地址
	SPI_WriteByte(ADS1256_CMD_RREG | (regaddr & 0x0F));
	
	//写入数据的个数n-1
	SPI_WriteByte(0x00);
	
	delay_us(10);
	
	//向regaddr地址指向的寄存器写入数据databyte
	databyte=SPI_ReadByte();
	
	ADS1256_CS_1();

  return databyte;
}


/*
*********************************************************************************************************
*	函 数 名: ADS1256_Init
*	功能说明: 初始化ADS1256
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void ADS1256_Init(void)
{
	Init_ADS1256_GPIO(); 																							  //ADS1256_GPIO初始化
	delay_ms(10);
	
	ADS1256_CS_0();
	delay_us(100);
	while(ADS1256_DRDY);
	SPI_WriteByte(ADS1256_CMD_REST);																		//复位
	delay_ms(10);
	
	while(ADS1256_DRDY);
	SPI_WriteByte(ADS1256_CMD_SELFCAL);																	 //自动校准
	
	while(ADS1256_DRDY);                             		 								//当ADS1256_DRDY为低时才能写寄存器
	SPI_WriteByte(ADS1256_CMD_SYNC);                 										//同步
	SPI_WriteByte(ADS1256_CMD_WAKEUP);              										//同步唤醒
	
	while(ADS1256_DRDY);
	ADS1256WREG(ADS1256_STATUS,0x04);               										// 高位在前、不使用缓冲
	ADS1256WREG(ADS1256_MUX,ADS1256_MUXP_AIN0 | ADS1256_MUXN_AINCOM);		//设置通道
	ADS1256WREG(ADS1256_ADCON,ADS1256_GAIN_1);      										// 放大倍数1
	ADS1256WREG(ADS1256_DRATE,ADS1256_DRATE_30000SPS); 									// 数据1000sps,不同速率注意采样时间间隔
	ADS1256WREG(ADS1256_IO,0x00);  																			//不使用数据IO口
	
	while(ADS1256_DRDY);
	SPI_WriteByte(ADS1256_CMD_SELFCAL);																	 //自动校准
	ADS1256_CS_1();

}             



/*
*********************************************************************************************************
*	函 数 名: ADS1256ReadData
*	功能说明: 读取24位AD值
*	形    参: channel:通道选择
*	返 回 值: AD值
*********************************************************************************************************
*/
u32 ADS1256ReadData(u8 channel)  
{
	u32 sum=0;

	while(ADS1256_DRDY);								//当ADS1256_DRDY为低时才能写寄存器
	ADS1256WREG(ADS1256_MUX,channel);		//设置通道
	
  ADS1256_CS_0();
	delay_us(200);
	
	SPI_WriteByte(ADS1256_CMD_SYNC);		//同步
	SPI_WriteByte(ADS1256_CMD_WAKEUP);	//同步唤醒               
	SPI_WriteByte(ADS1256_CMD_RDATA);   //读取数据

	sum = (SPI_ReadByte() << 16);
	sum |= (SPI_ReadByte() << 8);
	sum |= SPI_ReadByte();

	ADS1256_CS_1();
	
	while(ADS1256_DRDY);
	
	return sum;
}

/*
*********************************************************************************************************
*	函 数 名: Get_Val
*	功能说明: 将读取的24位AD值转化成电压值
*	形    参: addata:AD值
*	返 回 值: 电压值
*********************************************************************************************************
*/
float Get_Val(u32 addata)
{
	 /*		PGA由ADS1256初始化设置，VREF为电路的基准源，根据实际情况修改*/
	 u8 PGA=1;
	 float VREF=2.4918f;
	
	 float val;
	
	 /*  负电压的转化  */
	 if(addata > 0x800000)
	 {
			addata=0xFFFFFF-addata;
		  val= -2*VREF/(PGA*8388607)*addata;
	 }
   else
			val=2*VREF/(PGA*8388607)*addata;

	 return val;
}



/*
*********************************************************************************************************
*	函 数 名: Middle_Filter
*	功能说明: 滑动平均滤波
*	形    参: channel:通道选择
						times:数据个数
*	返 回 值: 滤波后的AD值
*********************************************************************************************************
*/
u32 Moving_Average_Filter(u8 channel,u16 times) 
{

	 u16 i,counts=0;
	 u32 sum=0,data[100];
	 u32 value;
	 u32 max,min;


	 //读取times个数据
	 while(counts < times)
	 {
			data[counts++]=ADS1256ReadData(channel);
	 }

	 max=min=data[0];
	 
	 for(i=0;i<times;i++)
	 {
		  if(data[i] > max)
				max=data[i];
			
			if(data[i] < min)
				min=data[i];
			
			sum+=data[i];
	 }

	 value=(sum - max - min)/(times - 2);
	 
	 return value;
}




float ADS1256_ReadVol( uint8_t ch_1 , uint8_t ch_2  ) //可进行差分读取
{
	float adc_data;
	float vol;
	float base_vol = 4.9896;;
	
//	if( ch_1 < 8 )
//		ch_1 = ch_1 * 16;
//	else 
//		ch_1 = 0;
	
//	if( mode == 0 )
//		ch_2 = ADS1256_MUXN_AINCOM;
	
	adc_data = ADS1256ReadData( ch_1 | ch_2 );
	
	adc_data = adc_data > 0x7fffff ? adc_data - 0xffffff : adc_data;
	
	vol = (float)adc_data / 0x7fffff * base_vol;
	
	return vol;
}

//快速排序算法  ( 调用示例:quick_sort(buff,0,num-1) )
void quick_sort(float num[], int l, int r)  
{  
    int i, j;  
	float x;
    if (l < r)  
    {  
        i = l;  
        j = r;  
        x = num[i];  
        while (i < j)  
        {  
            while(i < j && num[j] > x)   
                j--; /* 从右向左找第一个小于x的数 */  
            if(i < j)   
                num[i++] = num[j];  
      
  
            while(i < j && num[i] < x)   
                i++; /* 从左向右找第一个大于x的数 */  
            if(i < j)   
                num[j--] = num[i];  
  
        }  
        num[i] = x;  
        quick_sort(num, l, i-1); /* 递归调用 */  
        quick_sort(num, i+1, r);  
    }  
} 
//中值滤波
float ADS1256_ave_fiter(u8 channel,u8 times)
{
	u8 i,l=0,h=0;
	float dat[11],avedat,sum;
	for(i=0;i<times;i++)
	{
		dat[i]=ADS1256_ReadVol(channel,ADS1256_MUXN_AINCOM);
		sum+=dat[i];
//		delay_ms(5);
	}
//	l=0;
//	h=0;
//	for(i=0;i<times/2;i++)
//	{
//		if(dat[i]<dat[l])
//		l=i;
//		if(dat[i]>dat[h])
//		h=i;
//	}
//	avedat=(sum-dat[l]-dat[h])/(times-2);
	quick_sort(dat,0,times-1);
//	avedat=sum/times;
	return dat[times/2];
}

