#include "Drive_Communication.h"

/*通信数据结构体*/
communicationDataStruct communicationData = {{0},{0}};

#if DDS_TYPE == 1
//dds结构体变量初始化 
DDSDataStruct dds[2]; 
#else
DDSDataStruct dds;
#endif

#define Usart6_RX 		GPIO_Pin_7
#define Usart6_TX 		GPIO_Pin_6

u8  USART6_RX_BUF[USART6_REC_LEN];	//接收缓冲,最大USART6_REC_LEN个字节.
u16 USART6_RX_STA = 0;       //接收状态标记	

void Usart6_Init_DDS(u32 bound) {
	GPIO_InitTypeDef 	GPIO_InitStructure;
	USART_InitTypeDef 	USART_InitStructure;
	NVIC_InitTypeDef 	NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //使能GPIOC时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);//使能USART6时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6); //GPIOC6复用为USART6
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6); //GPIOA7复用为USART6
	
	//USART1端口配置
	GPIO_InitStructure.GPIO_Pin = Usart6_RX | Usart6_TX; //GPIOC6与GPIOC7
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure); //初始化PC6，PC7

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART6, &USART_InitStructure); //初始化串口1
	
	USART_Cmd(USART6, ENABLE);  //使能串口1 
		
#if EN_USART6_RX	
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);//开启相关中断
	//Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
#endif
}

void USART6_IRQHandler (void) {
	static u16 i=0;
	static u8 rx_flag =0;
	u8 Res;
	if (USART_GetITStatus(USART6, USART_IT_RXNE) != RESET) {
		Res =USART_ReceiveData(USART6);//(USART1->DR);	//读取接收到的数据
		
		if(i == 0 && Res == 0xAA)
			rx_flag =1;
		if(rx_flag == 1)
			communicationData.rxBuffer[i++] = Res; 
		if (i == RX_LENGTH) {
			i=0;
			rx_flag =0;
			communicationData.flag = True;  
		}		  		 
	}
}

/**
* @brief  获取CRC校验码
* @param  none
* @retval CRC校验码
*/
static u16 crc_16(u8 *Datas, u16 Length) 
{
	u8 j;
	u16 temp = 0xFFFF, i;

	for(i=0; i<Length; ++i)
	{
		temp ^= Datas[i];

		for(j=0; j<8; ++j)
		{
			if(temp & 1)
			{
				temp >>= 1;
				temp ^= 0xA001;
			}
			else
			{
				temp >>= 1;
			}
		}
	}

	return temp;
}

/**
 * @brief  USART1发送数据
 * @param  *sendData : 待发送的数据
           length    : 数据长度
 * @retval 无
 */
static void Usart6_Send_Data(u8 *sendData,u16 length)
{
	u16 i;
	for(i = 0; i < length; i++)
	{
		USART_SendData(USART6,sendData[i]);
		while(USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET);
	}
}

/**
* @brief  发送波形数据
* @param  data :DDS结构体变量
* @retval none
*/
#if DDS_TYPE == 1
void SendData(DDSDataStruct data,u8 ch)     
{
	u16 crcValue;
	u32 range=(u32)(data.range*1000);
	u32 phase=(u32)(data.phase*100);
	
	communicationData.txBuffer[0]  = 0xAA;							//帧头
	communicationData.txBuffer[1]  = 0xA5;
	
	communicationData.txBuffer[2]  = (u8) data.mode >>8;			//模式
	communicationData.txBuffer[3]  = (u8)(data.mode &0xff);  
	
	communicationData.txBuffer[4]  = ch;							//通道
	communicationData.txBuffer[5]  = data.output;					//输出开关
	
	communicationData.txBuffer[6]  = (u8)(data.fre >>24);			//频率
	communicationData.txBuffer[7]  = (u8)((data.fre>>16) & 0xff);
	communicationData.txBuffer[8]  = (u8)((data.fre>>8) & 0xff);
	communicationData.txBuffer[9]  = (u8)(data.fre & 0xff);
	
	communicationData.txBuffer[10] = (u8)(range>>24);				//幅度
	communicationData.txBuffer[11] = (u8)((range>>16) & 0xff);
	communicationData.txBuffer[12] = (u8)((range>>8) & 0xff);
	communicationData.txBuffer[13] = (u8)(range & 0xff);  
	
	communicationData.txBuffer[14] = (u8)(phase>>24);				//相位
	communicationData.txBuffer[15] = (u8)((phase>>16) & 0xff);
	communicationData.txBuffer[16] = (u8)((phase>>8) & 0xff);
	communicationData.txBuffer[17] = (u8)(phase & 0xff);  
	
	communicationData.txBuffer[18] = (u8)(data.step>>24);			//步进
	communicationData.txBuffer[19] = (u8)((data.step>>16) & 0xff);
	communicationData.txBuffer[20] = (u8)((data.step>>8) & 0xff);
	communicationData.txBuffer[21] = (u8)(data.step & 0xff);  
	
	communicationData.txBuffer[22] = (u8)(data.step_time>>24);		//步进时间
	communicationData.txBuffer[23] = (u8)((data.step_time>>16) & 0xff);
	communicationData.txBuffer[24] = (u8)((data.step_time>>8) & 0xff);
	communicationData.txBuffer[25] = (u8)(data.step_time & 0xff);  
	
	communicationData.txBuffer[26] = (u8)(data.fre_start>>24);		//起始频率
	communicationData.txBuffer[27] = (u8)((data.fre_start>>16) & 0xff);
	communicationData.txBuffer[28] = (u8)((data.fre_start>>8) & 0xff);
	communicationData.txBuffer[29] = (u8)(data.fre_start & 0xff);  
	
	communicationData.txBuffer[30] = (u8)(data.fre_stop>>24);		//终止频率
	communicationData.txBuffer[31] = (u8)((data.fre_stop>>16) & 0xff);
	communicationData.txBuffer[32] = (u8)((data.fre_stop>>8) & 0xff);
	communicationData.txBuffer[33] = (u8)(data.fre_stop & 0xff);  
	
	/*生成CRC校验码*/
	crcValue = crc_16(&communicationData.txBuffer[0],TX_LENGTH - 3);

	communicationData.txBuffer[TX_LENGTH-3] = crcValue>>8;
	communicationData.txBuffer[TX_LENGTH-2] = crcValue & 0xff;
	
	communicationData.txBuffer[TX_LENGTH-1] = 0x55;					//帧尾
	
	Usart6_Send_Data(communicationData.txBuffer,TX_LENGTH);
}

void DDSInit(void) 
{
	Usart6_Init_DDS(115200);
	// 幅度
	/*0.2->80mv, 0.2*1.02->100mv*/
	dds[0].range=dds[1].range=0.2f * 1.04f;
	// 频率
	dds[0].fre=dds[1].fre=10000;
	// 相位
	dds[0].phase=dds[1].phase=0;
	// 扫频步进
	dds[0].step=dds[1].step=1000;
	// 扫频时间
	dds[0].step_time=dds[1].step_time=1000;
	// 扫频起始
	dds[0].fre_start=dds[1].fre_start=1000;
	// 扫频终止 
	dds[0].fre_stop=dds[1].fre_stop=2000000;
	// 模式选择
	dds[0].mode=dds[1].mode=NORMAL;
	// 输出
	dds[0].output=dds[1].output=1;
	SendData(dds[0], 0);
    delay_ms(100);
	SendData(dds[1], 1);
}
#else 
void SendData(DDSDataStruct data,u8 ch)
{
	u16 crcValue;
	u32 range=(u32)(data.range*1000);
	
	communicationData.txBuffer[0]  = 0xAA;												//帧头
	communicationData.txBuffer[1]  = 0xA5;

	communicationData.txBuffer[2]  = (u8) data.mode >>8;     		 	//模式
	communicationData.txBuffer[3]  = (u8)(data.mode &0xff);  

	communicationData.txBuffer[4]  = (u8)(data.fre>>24);       		//频率
	communicationData.txBuffer[5]  = (u8)((data.fre>>16) & 0xff);
	communicationData.txBuffer[6]  = (u8)((data.fre>>8) & 0xff);
	communicationData.txBuffer[7]  = (u8)(data.fre & 0xff);

	communicationData.txBuffer[8]  = (u8)(range>>24);  						//幅度
	communicationData.txBuffer[9]  = (u8)((range>>16) & 0xff);
	communicationData.txBuffer[10] = (u8)((range>>8) & 0xff);
	communicationData.txBuffer[11] = (u8)(range & 0xff);  

	communicationData.txBuffer[12] = data.output;  								//输出开关

	communicationData.txBuffer[13] = (u8)(data.step>>24); 	 	 						//步进
	communicationData.txBuffer[14] = (u8)((data.step>>16) & 0xff);
	communicationData.txBuffer[15] = (u8)((data.step>>8) & 0xff);
	communicationData.txBuffer[16] = (u8)(data.step & 0xff);  
	
	communicationData.txBuffer[17] = (u8)(data.step_time>>24);						//步进时间
	communicationData.txBuffer[18] = (u8)((data.step_time>>16) & 0xff);
	communicationData.txBuffer[19] = (u8)((data.step_time>>8) & 0xff);
	communicationData.txBuffer[20] = (u8)(data.step_time & 0xff);  

	communicationData.txBuffer[21] = (u8)(data.fre_start>>24); 						//起始频率
	communicationData.txBuffer[22] = (u8)((data.fre_start>>16) & 0xff);
	communicationData.txBuffer[23] = (u8)((data.fre_start>>8) & 0xff);
	communicationData.txBuffer[24] = (u8)(data.fre_start & 0xff);

	communicationData.txBuffer[25] = (u8)(data.fre_stop>>24);  						//终止频率
	communicationData.txBuffer[26] = (u8)((data.fre_stop>>16) & 0xff);
	communicationData.txBuffer[27] = (u8)((data.fre_stop>>8) & 0xff);
	communicationData.txBuffer[28] = (u8)(data.fre_stop & 0xff);

	/*生成CRC校验码*/
	crcValue = crc_16(&communicationData.txBuffer[0],TX_LENGTH - 3);

	communicationData.txBuffer[TX_LENGTH-3] = crcValue>>8;
	communicationData.txBuffer[TX_LENGTH-2] = crcValue & 0xff;
	
	communicationData.txBuffer[TX_LENGTH-1] = 0x55;							//帧尾
	
	Usart6_Send_Data(communicationData.txBuffer,TX_LENGTH);
}

void DDSInit(void) {
	Usart6_Init_DDS(115200);
	/*	输出幅度 2v	*/
	dds.range=0.52f;						//峰峰值=幅度*2
	/*	输出频率	100000Hz	*/
	dds.fre=20700000;
	/*	扫频步进	1000Hz	*/
	dds.step=1000;
	/*	扫频时间	1000us	*/
	dds.step_time=1000;
	/*	扫频起始频率1000Hz		*/
	dds.fre_start=1000;
	/*	扫频终止200000	Hz	*/
	dds.fre_stop=200000;
	/*	默认为普通输出模式		*/
	dds.mode=NORMAL;
	/*	默认不打开输出		*/
	dds.output=1; 
	SendData(dds,0);
}
#endif


