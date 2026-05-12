#ifndef DRIVE_COMMUNICATION_H
#define DRIVE_COMMUNICATION_H

#include "User_header.h"

#define USART6_REC_LEN  			400  	//定义最大接收字节数 200
#define EN_USART6_RX 			1		//使能（1）/禁止（0）串口1接收

extern u8  USART6_RX_BUF[USART6_REC_LEN];
extern u16 USART6_RX_STA;

void Usart6_Init_DDS(u32 bound);


#define DDS_TYPE 1 //1:9958	0:9851

#ifndef True
#define True 1
#endif

#ifndef False
#define False  0
#endif

/* 通信协议指令 */
#define	NORMAL		0x0001 //普通模式
#define	SWEEP		0x0002 //扫频模式
#define	MODULATION	0x0004 //调制模式
#define	INPHASE		0x0008 //同相输出模式

#define    RX_LENGTH      48
#define    TX_LENGTH      48

typedef struct communicationDataStruct
{
    u8 rxBuffer[RX_LENGTH];
    u8 txBuffer[TX_LENGTH];
    u8 flag;					  //接收置位
}communicationDataStruct;



/** 
  * @brief  Modulation Type Configuration
  */
typedef enum
{
    AD9958_DISABLE = 0,
    AD9958_AMPLITUDE,
    AD9958_FREQUENCY,
    AD9958_PHASE,
}AFP_TypeDef;

/** 
  * @brief  Modulation Level Selection
  */
typedef enum
{
    TWO_LEVEL_MODULATION = 0x00,
    FOUR_LEVEL_MODULATION,
    EIGHT_LEVEL_MODULATION,
    SIXTEEN_LEVEL_MODULATION,
}ModulationLevel_TypeDef;

#if DDS_TYPE==1
typedef struct DDSDataStruct
{
	u8  mode;      //输出模式
	u8  output;    //输出开关

	float range;   //幅度
	float phase;   //相位 	
	u32 fre;       //普通模式的频率

	u32 fre_start; //扫频模式的起始频率
	u32 fre_stop;	 //扫频模式的截止频率
	u32 step;      //步进
	u32 step_time; //步进时间(us)

	u8 modulation_mode;       /* 调制或扫频模式 */
	u8 modulation_levels;			/* 数字调制阶数 */
	float ask[16]; 						/* 数字调制模式下ASK的系数值 */
	float fsk[16];  					/* 数字调制模式下FSK的频率值 */
	float psk[16];  					/* 数字调制模式下PSK的相位值 */
	
}DDSDataStruct; 
extern DDSDataStruct dds[2];
#else 
typedef struct DDSDataStruct
{
	u8  mode;      	//输出模式
	u8  output;    	//输出开关
	
	float range;   	//幅度 
    u32 fre;       	//普通模式的频率
	
	u32 step;      	//步进
	u32 step_time; 	//步进时间(us)
    u32 fre_start; 	//扫频模式的起始频率
    u32 fre_stop;	//扫频模式的截止频率
	
}DDSDataStruct; 
extern DDSDataStruct dds;
#endif

void SendData(DDSDataStruct data,u8 ch);
void DDSInit(void);


extern communicationDataStruct communicationData; 

#endif



/******************************************(C)  COPYRIGHT 2017 吴远泸 *********************************************************/











