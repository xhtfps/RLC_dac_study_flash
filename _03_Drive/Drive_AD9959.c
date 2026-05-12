/************************************************************
                    AD9959 驱动程序
					AD9959--单片机 
硬件连接:  CS			——PA6;     PB5
          SCLK 		——PB1;   	 PB4
          UPDATE	——PB0;	   PB6
          SP0    	——PA7;   	 PA7
          SP1			——PA2;     PA2
          SP2			——PB10; 	 PB10
          SP3			——PC0;     PB7
          SDIO0		——PA5;     PB3
          SDIO1		——PA4;     PA15
          SDIO2		——PA3;     PA12
          SDIO3		——PA8;     PA11
		  AD9959_PWR(PDC)——PA9;     PB12
          RST			——PA10;    PB13  
          GND			--GND(0V)  

**************************************************************/

#include "Drive_AD9959.H"
//#include "task_manage.h"

u8 CSR_DATA0[1] = {0x10};     // 开 CH0
u8 CSR_DATA1[1] = {0x20};      // 开 CH1
u8 CSR_DATA2[1] = {0x40};      // 开 CH2
u8 CSR_DATA3[1] = {0x80};      // 开 CH3		
u8 CSR_DATAall[1] = {0xF0}; 		//															
 
u8 FR2_DATA[2] = {0x00,0x00};//default Value = 0x0000
u8 CFR_DATA[3] = {0x00,0x03,0x02};//default Value = 0x000302	   
																	
u8 CPOW0_DATA0[2] = {0x00,0x00};//default Value = 0x0000   @ = POW/2^14*360
u8 CPOW0_DATA1[2] = {0x00,0x00};																	
u8 CPOW0_DATA2[2] = {0x00,0x00};
u8 CPOW0_DATA3[2] = {0x00,0x00};

u8 LSRR_DATA[2] = {0x00,0x00};//default Value = 0x----
																	
u8 RDW_DATA[4] = {0x00,0x00,0x00,0x00};//default Value = 0x--------
																	
u8 FDW_DATA[4] = {0x00,0x00,0x00,0x00};//default Value = 0x--------


//AD9959初始化
void Init_AD9959(void)  
{ 
        u8 CFTW0_DATA[4] ={0x00,0x00,0x00,0x00};    //中间变量
      u32 Temp;  
        
    u32 A_temp;//=0x23ff;
    u8 ACR_DATA[3] = {0x00,0x00,0x00};//default Value = 0x--0000 Rest = 18.91/Iout 

    u16 P_temp=0;
    

    GPIO_InitTypeDef  GPIO_InitStructure;       
     u8 FR1_DATA[3] = {0xD0,0x00,0x00};//20倍频 Charge pump control = 75uA FR1<23> -- VCO gain control =0时 system clock below 160 MHz;
            
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOF | RCC_AHB1Periph_GPIOB, ENABLE);
            // 初始化GPIOB的管脚（PB5, PB4, PB6, PB10, PB7, PB3, PB12, PB13）
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;      // 输出模式
            GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     // 推挽输出
            GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;   // 无上下拉
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  // 50MHz速度

            // PB5(CS), PB4(SCLK), PB6(UPDATE), PB10(SP2), PB7(SP3), PB3(SDIO0), PB12(PDC), PB13(RST)
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6;
            GPIO_Init(GPIOE, &GPIO_InitStructure);

            // 初始化GPIOA的管脚（PA15, PA12, PA11）
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
            GPIO_Init(GPIOF, &GPIO_InitStructure);

            // 初始化GPIOC的管脚（PA7）
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
            GPIO_Init(GPIOB, &GPIO_InitStructure);

        
    Intserve();  //IO口初始化
	delay_ms(50);
  IntReset();  //AD9959复位  
    delay_ms(50);
    //写功能寄存器1  三个字节        写入0xD0,0x00,0x00  更新IO寄存器
  WriteData_AD9959(FR1_ADD,3,FR1_DATA,1);                    //设置为高频率时钟并且锁相环设置为20

   
//写入初始频率
//    Write_frequence(0,SinFre[0]); 
//    Write_frequence(1,SinFre[1]);
//    Write_frequence(2,SinFre[2]);
//    Write_frequence(3,SinFre[3]);    

//    Write_Phase(3, SinPhr[3]);
//    Write_Phase(0, SinPhr[0]);
//    Write_Phase(1, SinPhr[1]);
//    Write_Phase(2, SinPhr[2]);    

        
    Write_frequence(0,50000);
    Write_frequence(1,50000);
    Write_frequence(2,50000);
    Write_frequence(3,50000);

        Temp=(u32)1000000*8.589934592;               //数字频率转化到32位寄存器数据对应  初始化输出1M的频率
      CFTW0_DATA[3]=(u8)Temp;                            //最低8位
      CFTW0_DATA[2]=(u8)(Temp>>8);
      CFTW0_DATA[1]=(u8)(Temp>>16);
      CFTW0_DATA[0]=(u8)(Temp>>24);                //最高8位
        
        //1023为最大输出电压，及寄存器后十位为1 对应输出电压576mv需要根据板子来校准！！！！！！！！！！！！！
        A_temp=(u32)((300.0/576.0f)*1023);//输出300mv
        A_temp|=0x1000;
        ACR_DATA[2] = (u8)A_temp;      //低位数据
        ACR_DATA[1] = (u8)((A_temp>>8)|0x10); //高位数据0......
//            
        WriteData_AD9959(CSR_ADD,1,CSR_DATA0,1);
    WriteData_AD9959(CFTW0_ADD,4,CFTW0_DATA,1);
    WriteData_AD9959(ACR_ADD,3,ACR_DATA,1);
    WriteData_AD9959(CPOW0_ADD,2,CPOW0_DATA0,1);

    // 通道2
    WriteData_AD9959(CSR_ADD,1,CSR_DATA2,1);
    WriteData_AD9959(CFTW0_ADD,4,CFTW0_DATA,1);
    WriteData_AD9959(ACR_ADD,3,ACR_DATA,1);
    WriteData_AD9959(CPOW0_ADD,2,CPOW0_DATA2,1);

        
        //初始化输出同向
        Write_Phase(0,0);
        Write_Phase(1,30);
        Write_Phase(2,60);
        Write_Phase(3,90);
		
        Write_Amplitude(0,100);     //400
        Write_Amplitude(1,150);     //600
        Write_Amplitude(2,200);     //800
        Write_Amplitude(3,576);     //2304

		Write_frequence(0,50000);    
		Write_frequence(1,60000);
		Write_frequence(2,70000);
		Write_frequence(3,1);

}

//延时
void delay1 (u32 length)
{
	length = length*12;
   while(length--);
}
//IO口初始化
void Intserve(void)		   
{   
		AD9959_PWR=0;
    CS = 1;							//文档中为CS非
    SCLK = 0;
    UPDATE = 0;
    PS0 = 0;
    PS1 = 0;
    PS2 = 0;
    PS3 = 0;
    SDIO0 = 0;
    SDIO1 = 0;
    SDIO2 = 0;
    SDIO3 = 0;
}
//AD9959复位
void IntReset(void)	  
{
  Reset = 0;
	delay1(1);
	Reset = 1;			//高电平复位引脚
	delay1(30);
	Reset = 0;
}
 //AD9959更新数据
void IO_Update(void)  
{
	UPDATE = 0;
	delay1(2);
	UPDATE = 1;
	delay1(4);
	UPDATE = 0;
}
/*--------------------------------------------
函数功能：控制器通过SPI向AD9959写数据
RegisterAddress: 寄存器地址
NumberofRegisters: 所含字节数
*RegisterData: 数据起始地址
temp: 是否更新IO寄存器
----------------------------------------------*/
void WriteData_AD9959(u8 RegisterAddress, u8 NumberofRegisters, u8 *RegisterData,u8 temp)
{
	u8	ControlValue = 0;
	u8	ValueToWrite = 0;
	u8	RegisterIndex = 0;
	u8	i = 0;

	ControlValue = RegisterAddress;
//写入地址
	SCLK = 0;
	CS = 0;	 
	for(i=0; i<8; i++)
	{
		SCLK = 0;
		if(0x80 == (ControlValue & 0x80))
		SDIO0= 1;	  
		else
		SDIO0= 0;	  
		SCLK = 1;
		ControlValue <<= 1;
	}
	SCLK = 0;
//写入数据
	for (RegisterIndex=0; RegisterIndex<NumberofRegisters; RegisterIndex++)
	{
		ValueToWrite = RegisterData[RegisterIndex];
		for (i=0; i<8; i++)
		{
			SCLK = 0;
			if(0x80 == (ValueToWrite & 0x80))
			SDIO0= 1;	  
			else
			SDIO0= 0;	  
			SCLK = 1;
			ValueToWrite <<= 1;
		}
		SCLK = 0;		
	}	
	if(temp==1)
		IO_Update();	
  CS = 1;
} 

u8 CFTW0_DATA[4] ={0x00,0x00,0x00,0x00};	//中间变量
/*---------------------------------------
函数功能：设置通道输出频率
Channel:  输出通道
Freq:     输出频率
---------------------------------------*/
void Write_frequence(u8 Channel,u32 Freq)
{	 
		
	  u32 Temp;     

		if(Freq>500000000)
			Freq = 500000000;
		//将输入频率因子分为四个字节 8.58993459=(2^32)/500000000
	  Temp=(u32)Freq*8.589934592;	   			//数字频率转化到32位寄存器数据对应 
	  CFTW0_DATA[3]=(u8)Temp;							//最低8位
	  CFTW0_DATA[2]=(u8)(Temp>>8);
	  CFTW0_DATA[1]=(u8)(Temp>>16);
	  CFTW0_DATA[0]=(u8)(Temp>>24);				//最高8位
		
	  if(Channel==0)	  
	  {
			//通道0使能 但是频率控制字由传参fre控制
			WriteData_AD9959(CSR_ADD,1,CSR_DATA0,1);		//控制寄存器写入CH0通道
      WriteData_AD9959(CFTW0_ADD,4,CFTW0_DATA,1);	//CTW0 address 0x04.输出CH0设定频率
		}
	  else if(Channel==1)	
	  {//通道一
			WriteData_AD9959(CSR_ADD,1,CSR_DATA1,1);//控制寄存器写入CH1通道
      WriteData_AD9959(CFTW0_ADD,4,CFTW0_DATA,1);//CTW0 address 0x04.输出CH1设定频率	
	  }
	  else if(Channel==2)	
	  {//通道2
			WriteData_AD9959(CSR_ADD,1,CSR_DATA2,1);//控制寄存器写入CH2通道
      WriteData_AD9959(CFTW0_ADD,4,CFTW0_DATA,1);//CTW0 address 0x04.输出CH2设定频率	
	  }
	  else if(Channel==3)	
	  {//通道3
			WriteData_AD9959(CSR_ADD,1,CSR_DATA3,1);//控制寄存器写入CH3通道
      WriteData_AD9959(CFTW0_ADD,4,CFTW0_DATA,1);//CTW0 address 0x04.输出CH3设定频率	
	  }						

	
} 
void Write_frequence_no_update(u8 Channel,u32 Freq)
{	 
//		u8 CFTW0_DATA[4] ={0x00,0x00,0x00,0x00};	//中间变量
	  u32 Temp;     

		if(Freq>500000000)
			Freq = 500000000;
		//将输入频率因子分为四个字节 8.58993459=(2^32)/500000000
	  Temp=(u32)Freq*8.589934592;	   			//数字频率转化到32位寄存器数据对应 
	  CFTW0_DATA[3]=(u8)Temp;							//最低8位
	  CFTW0_DATA[2]=(u8)(Temp>>8);
	  CFTW0_DATA[1]=(u8)(Temp>>16);
	  CFTW0_DATA[0]=(u8)(Temp>>24);				//最高8位
		
	  if(Channel==0)	  
	  {
			//通道0使能 但是频率控制字由传参fre控制
			WriteData_AD9959(CSR_ADD,1,CSR_DATA0,0);		//控制寄存器写入CH0通道
      WriteData_AD9959(CFTW0_ADD,4,CFTW0_DATA,0);	//CTW0 address 0x04.输出CH0设定频率
		}
	  else if(Channel==1)	
	  {//通道一
			WriteData_AD9959(CSR_ADD,1,CSR_DATA1,0);//控制寄存器写入CH1通道
      WriteData_AD9959(CFTW0_ADD,4,CFTW0_DATA,0);//CTW0 address 0x04.输出CH1设定频率	
	  }
	  else if(Channel==2)	
	  {//通道2
			WriteData_AD9959(CSR_ADD,1,CSR_DATA2,0);//控制寄存器写入CH2通道
      WriteData_AD9959(CFTW0_ADD,4,CFTW0_DATA,0);//CTW0 address 0x04.输出CH2设定频率	
	  }
	  else if(Channel==3)	
	  {//通道3
			WriteData_AD9959(CSR_ADD,1,CSR_DATA3,0);//控制寄存器写入CH3通道
      WriteData_AD9959(CFTW0_ADD,4,CFTW0_DATA,0);//CTW0 address 0x04.输出CH3设定频率	
	  }	
	  else if(Channel==4)	
	  {//全部通道
			WriteData_AD9959(CSR_ADD,1,CSR_DATAall,0);//控制寄存器写入CH3通道
      WriteData_AD9959(CFTW0_ADD,4,CFTW0_DATA,0);//CTW0 address 0x04.输出CH3设定频率	
	  }						

	
} 

/*---------------------------------------
函数功能：设置通道输出幅度
Channel:  输出通道
Ampli:    输出幅度
---------------------------------------*/
u8 ACR_DATA[3] = {0x00,0x00,0x00};//default Value = 0x--0000 Rest = 18.91/Iout 

void Write_Amplitude(u8 Channel, u16 Ampli)
{ 
	u32 A_temp;//=0x23ff;

	
	if(Ampli>576)
			Ampli = 576;
	A_temp=(u32)((Ampli/576.0f)*1023);//输出300mv		寄存器后十位都为1时幅值最大，需要校准每块板子！！！！！
	A_temp|=0x1000;
	ACR_DATA[2] = (u8)A_temp;  	//低位数据
	ACR_DATA[1] = (u8)((A_temp>>8)|0x10); //高位数据
	
  if(Channel==0)
  {
		WriteData_AD9959(CSR_ADD,1,CSR_DATA0,1); 
    WriteData_AD9959(ACR_ADD,3,ACR_DATA,1); 
	}
  else if(Channel==1)
  {
		WriteData_AD9959(CSR_ADD,1,CSR_DATA1,1); 
    WriteData_AD9959(ACR_ADD,3,ACR_DATA,1);
	}
  else if(Channel==2)
  {
	  WriteData_AD9959(CSR_ADD,1,CSR_DATA2,1); 
    WriteData_AD9959(ACR_ADD,3,ACR_DATA,1); 
	}
  else if(Channel==3)
  {
		WriteData_AD9959(CSR_ADD,1,CSR_DATA3,1); 
    WriteData_AD9959(ACR_ADD,3,ACR_DATA,1); 
	}
}

void Write_Amplitude_no_update(u8 Channel, u16 Ampli)
{ 
	u32 A_temp;//=0x23ff;
	u8 ACR_DATA[3] = {0x00,0x00,0x00};//default Value = 0x--0000 Rest = 18.91/Iout 
	
	if(Ampli>576)
			Ampli = 576;
	A_temp=(u32)((Ampli/576.0f)*1023);//输出300mv		寄存器后十位都为1时幅值最大，需要校准每块板子！！！！！
	A_temp|=0x1000;
	ACR_DATA[2] = (u8)A_temp;  	//低位数据
	ACR_DATA[1] = (u8)((A_temp>>8)|0x10); //高位数据
	
  if(Channel==0)
  {
		WriteData_AD9959(CSR_ADD,1,CSR_DATA0,0); 
    WriteData_AD9959(ACR_ADD,3,ACR_DATA,0); 
	}
  else if(Channel==1)
  {
		WriteData_AD9959(CSR_ADD,1,CSR_DATA1,0); 
    WriteData_AD9959(ACR_ADD,3,ACR_DATA,0);
	}
  else if(Channel==2)
  {
	  WriteData_AD9959(CSR_ADD,1,CSR_DATA2,0); 
    WriteData_AD9959(ACR_ADD,3,ACR_DATA,0); 
	}
  else if(Channel==3)
  {
		WriteData_AD9959(CSR_ADD,1,CSR_DATA3,0); 
    WriteData_AD9959(ACR_ADD,3,ACR_DATA,0); 
	}
}
/*---------------------------------------
函数功能：设置通道输出相位
Channel:  输出通道
Phase:    输出相位,范围：0~16383(对应角度：0°~360°)
---------------------------------------*/
void Write_Phase(u8 Channel,u16 Phase)
{
	u16 P_temp=0;
	
	if(Phase > 360)
				Phase = 360;
	P_temp = (u16)(Phase *45.511111 );
//	P_temp &= 0x3FFF;
	
	//加入转化的逻辑
	CPOW0_DATA0[1]=(u8)P_temp;
	CPOW0_DATA0[0]=(u8)(P_temp>>8);
	
	
	if(Channel==0)
  {//使能通道1		写入相位控制字
		WriteData_AD9959(CSR_ADD,1,CSR_DATA0,1); 
    WriteData_AD9959(CPOW0_ADD,2,CPOW0_DATA0,1);
  }
  else if(Channel==1)
  {
		WriteData_AD9959(CSR_ADD,1,CSR_DATA1,1); 
    WriteData_AD9959(CPOW0_ADD,2,CPOW0_DATA0,1);
  }
  else if(Channel==2)
  {
		WriteData_AD9959(CSR_ADD,1,CSR_DATA2,1); 
    WriteData_AD9959(CPOW0_ADD,2,CPOW0_DATA0,1);
  }
  else if(Channel==3)
  {
		WriteData_AD9959(CSR_ADD,1,CSR_DATA3,1); 
    WriteData_AD9959(CPOW0_ADD,2,CPOW0_DATA0,1);
  }
	else if(Channel==4)	//用于所有通道先同步相位
  {
		WriteData_AD9959(CSR_ADD,1,CSR_DATAall,1); 
    WriteData_AD9959(CPOW0_ADD,2,CPOW0_DATA0,1);
  }
}	 

//写入阶段_无更新
void Write_Phase_no_update(u8 Channel,u16 Phase)
{
	u16 P_temp=0;
	
	if(Phase > 360)
				Phase = 360;
	P_temp = (u16)(Phase *45.511111 );
//	P_temp &= 0x3FFF;
	
	//加入转化的逻辑
	CPOW0_DATA0[1]=(u8)P_temp;
	CPOW0_DATA0[0]=(u8)(P_temp>>8);
	
	
	if(Channel==0)
  {//使能通道1		写入相位控制字
		WriteData_AD9959(CSR_ADD,1,CSR_DATA0,0); 
    WriteData_AD9959(CPOW0_ADD,2,CPOW0_DATA0,0);
  }
  else if(Channel==1)
  {
		WriteData_AD9959(CSR_ADD,1,CSR_DATA1,0); 
    WriteData_AD9959(CPOW0_ADD,2,CPOW0_DATA0,0);
  }
  else if(Channel==2)
  {
		WriteData_AD9959(CSR_ADD,1,CSR_DATA2,0); 
    WriteData_AD9959(CPOW0_ADD,2,CPOW0_DATA0,0);
  }
  else if(Channel==3)
  {
		WriteData_AD9959(CSR_ADD,1,CSR_DATA3,0); 
    WriteData_AD9959(CPOW0_ADD,2,CPOW0_DATA0,0);
  }
	else if(Channel==4)	//用于所有通道先同步相位
  {
		WriteData_AD9959(CSR_ADD,1,CSR_DATAall,0); 
    WriteData_AD9959(CPOW0_ADD,2,CPOW0_DATA0,0);
  }
}
