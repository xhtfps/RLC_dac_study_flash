/*
*********************************************************************************************************
                                               _04_OS
    File			 : Drive_Timer.c
    By  			 : Muhe
    platform   : STM32F407ZG
	Data   		 : 2018/7/16
    function 	 : 定时器配置程序
*********************************************************************************************************
*/
#include "Drive_Timer.h"



/** ----------------------------------------------------------------------------
    @FunctionName  : TIM1_Init()
    @Description   : TIM_Period和TIM_Period根据需要改动
    @Data          : 2018/7/16
    @Explain       :
    ------------------------------------------------------------------------------*/

void TIM1_Init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);  	//TIM1时钟使能
	TIM_TimeBaseStructure.TIM_Period =5000-1;//重装载值
	TIM_TimeBaseStructure.TIM_Prescaler =8400-1;//预分频
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//不分频
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上累计模式，即从0开始加到溢出值
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);//配置
	TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);//允许定时器 更新中断
	TIM_ARRPreloadConfig(TIM1,ENABLE);
	TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	NVIC_InitStructure.NVIC_IRQChannel= TIM1_UP_TIM10_IRQn; //定时器 x 中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 2 ; //抢占优先级 1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority= 0; //响应优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);// 初始化 NVIC
	TIM_Cmd(TIM1, ENABLE);  //使能TIM1
}


/** ----------------------------------------------------------------------------
    @FunctionName  : TIM2_Init()
    @Description   : None
    @Data          : 2016/3/19
    @Explain       : 用于测频（0HZ--8MHZ）
    @PS						 : 如果运行在系统上，尽量只测5M以下，频率太高会影响任务运行
    ------------------------------------------------------------------------------*/
void TIM2_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM_ICStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE); ///使能 TIMx时钟
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_TIM2); //GPIOB10复用为定时器2
//配置时钟及初始化预装值
	/*---------------------------------------------------------------------------------------------*/
	TIM_TimeBaseStructure.TIM_Period = 0xffffffff;//重装载值 32位
	TIM_TimeBaseStructure.TIM_Prescaler =8-1;//预分频
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上累计模式，即从0开始加到溢出值
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);//配置						    		//@
	/*---------------------------------------------------------------------------------------------*/
	TIM_ICStructure.TIM_Channel=TIM_Channel_3;	//通道2
	TIM_ICStructure.TIM_ICPolarity=TIM_ICPolarity_Rising;//捕获极性
	TIM_ICStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;//捕获选择
	TIM_ICStructure.TIM_ICFilter=0;														//捕获滤波
	TIM_ICStructure.TIM_ICPrescaler=TIM_ICPSC_DIV8;					//捕获分频
	TIM_ICInit(TIM2, &TIM_ICStructure);
	TIM_ITConfig(TIM2,TIM_IT_CC3,ENABLE);//允许定时器 更新中断
//配置中断层
	/*---------------------------------------------------------------------------------------------*/
	NVIC_InitStructure.NVIC_IRQChannel= TIM2_IRQn; //定时器 x 中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 2 ; //抢占优先级 1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority= 0; //响应优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);// 初始化 NVIC
	TIM_Cmd(TIM2,ENABLE); //使能定时器 x
}


/** ----------------------------------------------------------------------------
    @FunctionName  : TIM3_Init()
    @Description   : TIM_Period和TIM_Period根据需要改动
    @Data          : 2018/7/16
    @Explain       : ADC采样率调节
    ------------------------------------------------------------------------------*/
void TIM3_Init(u32 pre,u32 period)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	TIM_Cmd(TIM3, DISABLE);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = pre-1;
	TIM_TimeBaseStructure.TIM_Period = period-1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up ;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_ARRPreloadConfig(TIM3, ENABLE);
	TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);
	TIM_Cmd(TIM3, ENABLE);
}



/** ----------------------------------------------------------------------------
    @FunctionName  : TIM4_Init()
    @Description   : TIM_Period和TIM_Period根据需要改动
    @Data          : 2018/7/16
    @Explain       : 通信发送速率控制（8K）
    ------------------------------------------------------------------------------*/
void TIM4_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
//配置时钟及初始化预装值
	/*---------------------------------------------------------------------------------------------*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE); ///使能 TIMx时钟				   //@
	TIM_TimeBaseStructure.TIM_Period =2-1;//重装载值
	TIM_TimeBaseStructure.TIM_Prescaler =2625-1;//预分频
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上累计模式，即从0开始加到溢出值
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);//配置						    		//@
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);//允许定时器 更新中断							//@
	/*---------------------------------------------------------------------------------------------*/
//配置中断层
	/*---------------------------------------------------------------------------------------------*/
	NVIC_InitStructure.NVIC_IRQChannel= TIM4_IRQn; //定时器 x 中断							//@
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 1 ; //抢占优先级 4
	NVIC_InitStructure.NVIC_IRQChannelSubPriority= 0 ; //响应优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);// 初始化 NVIC
	TIM_Cmd(TIM4,ENABLE); //使能定时器 x				4								//@
	/*---------------------------------------------------------------------------------------------*/
}

/** ----------------------------------------------------------------------------
    @FunctionName  : TIM5_Init()
    @Description   : TIM_Period和TIM_Period根据需要改动
    @Data          : 2018/7/16
    @Explain       : 通信接收速率控制
    ------------------------------------------------------------------------------*/
void TIM5_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM_ICStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE); ///使能 TIMx时钟
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_TIM5); //GPIOB10复用为定时器2
//配置时钟及初始化预装值
	/*---------------------------------------------------------------------------------------------*/
	TIM_TimeBaseStructure.TIM_Period = 1000000-1;//重装载值 32位
	TIM_TimeBaseStructure.TIM_Prescaler =84-1;//预分频
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上累计模式，即从0开始加到溢出值
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);//配置						    		//@
	/*---------------------------------------------------------------------------------------------*/
	TIM_ICStructure.TIM_Channel=TIM_Channel_3;	//通道2
	TIM_ICStructure.TIM_ICPolarity=TIM_ICPolarity_Rising;//捕获极性
	TIM_ICStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;//捕获选择
	TIM_ICStructure.TIM_ICFilter=0;														//捕获滤波
	TIM_ICStructure.TIM_ICPrescaler=TIM_ICPSC_DIV8;					//捕获分频
	TIM_ICInit(TIM5, &TIM_ICStructure);
	TIM_ITConfig(TIM5,TIM_IT_CC3|TIM_IT_Update,ENABLE);//允许定时器 更新中断
//配置中断层
	/*---------------------------------------------------------------------------------------------*/
	NVIC_InitStructure.NVIC_IRQChannel= TIM5_IRQn; //定时器 x 中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 2 ; //抢占优先级 1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority= 0; //响应优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);// 初始化 NVIC
	TIM_Cmd(TIM5,ENABLE); //使能定时器 x
}

/** ----------------------------------------------------------------------------
    @FunctionName  : TIM6_Init()
    @Description   : TIM_Period和TIM_Period根据需要改动
    @Data          : 2018/7/16
    @Explain       :
    ------------------------------------------------------------------------------*/
void TIM6_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
//配置时钟及初始化预装值
	/*---------------------------------------------------------------------------------------------*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE); ///使能 TIMx时钟				   //@
	TIM_TimeBaseStructure.TIM_Period =1000000-1;//重装载值
	TIM_TimeBaseStructure.TIM_Prescaler =84-1;//预分频
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上累计模式，即从0开始加到溢出值
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);//配置						    		//@
	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);//允许定时器 更新中断							//@
	/*---------------------------------------------------------------------------------------------*/
//配置中断层
	/*---------------------------------------------------------------------------------------------*/
	NVIC_InitStructure.NVIC_IRQChannel= TIM6_DAC_IRQn; //定时器 x 中断							//@
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 1 ; //抢占优先级 4
	NVIC_InitStructure.NVIC_IRQChannelSubPriority= 0 ; //响应优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);// 初始化 NVIC
	TIM_Cmd(TIM6,ENABLE); //使能定时器 x				5								//@
	/*---------------------------------------------------------------------------------------------*/
}

/** ----------------------------------------------------------------------------
    @FunctionName  : TIM7_Init()
    @Description   : TIM_Period和TIM_Period根据需要改动
    @Data          : 2018/7/16
    @Explain       :
    ------------------------------------------------------------------------------*/
void TIM7_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
//配置时钟及初始化预装值
	/*---------------------------------------------------------------------------------------------*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE); ///使能 TIMx时钟				   //@
	TIM_TimeBaseStructure.TIM_Period =5000-1;//重装载值
	TIM_TimeBaseStructure.TIM_Prescaler =8400-1;//预分频
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上累计模式，即从0开始加到溢出值
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);//配置						    		//@
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE);//允许定时器 更新中断							//@
	/*---------------------------------------------------------------------------------------------*/
//配置中断层
	/*---------------------------------------------------------------------------------------------*/
	NVIC_InitStructure.NVIC_IRQChannel= TIM7_IRQn; //定时器 x 中断							//@
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 1 ; //抢占优先级 4
	NVIC_InitStructure.NVIC_IRQChannelSubPriority= 0 ; //响应优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);// 初始化 NVIC
	TIM_Cmd(TIM7,ENABLE); //使能定时器 x				5								//@
	/*---------------------------------------------------------------------------------------------*/
}

/** ----------------------------------------------------------------------------
    @FunctionName  : TIM8_Init()
    @Description   : TIM_Period和TIM_Period根据需要改动
    @Data          : 2018/7/16
    @Explain       : 超声波测距
    ------------------------------------------------------------------------------*/
void TIM8_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_1 ;      //PC34
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB,GPIO_Pin_1);
//	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11;    //芯片沟?
//	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
////	GPIO_InitStructure.GPIO_Pin=GPIO_PuPd_UP;
//	GPIO_Init(GPIOD,&GPIO_InitStructure);
//	GPIO_ResetBits(GPIOD,GPIO_Pin_11);
//配置时钟及初始化预装值
	/*---------------------------------------------------------------------------------------------*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE); ///使能 TIMx时钟				   //@
	TIM_TimeBaseStructure.TIM_Period =200-1;//重装载值
	TIM_TimeBaseStructure.TIM_Prescaler =168-1;//预分频
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上累计模式，即从0开始加到溢出值
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);//配置						    		//@
	TIM_ITConfig(TIM8,TIM_IT_Update,ENABLE);//允许定时器 更新中断							//@
	/*---------------------------------------------------------------------------------------------*/
//配置中断层
	/*---------------------------------------------------------------------------------------------*/
	NVIC_InitStructure.NVIC_IRQChannel= TIM8_UP_TIM13_IRQn; //定时器 x 中断							//@
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 3 ; //抢占优先级 3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority= 1 ; //响应优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);// 初始化 NVIC
	TIM_Cmd(TIM8,DISABLE); //使能定时器 x				4								//@
}


/** ----------------------------------------------------------------------------
    @FunctionName  : TIM9_Init()
    @Description   : TIM_Period和TIM_Period根据需要改动
    @Data          : 2018/7/16
    @Explain       :
    ------------------------------------------------------------------------------*/
void TIM9_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
//配置时钟及初始化预装值
	/*---------------------------------------------------------------------------------------------*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,ENABLE); ///使能 TIMx时钟				   //@
	TIM_TimeBaseStructure.TIM_Period =5000-1;//重装载值
	TIM_TimeBaseStructure.TIM_Prescaler =8400-1;//预分频
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上累计模式，即从0开始加到溢出值
	TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure);//配置						    		//@
	TIM_ITConfig(TIM9,TIM_IT_Update,ENABLE);//允许定时器 更新中断							//@
	/*---------------------------------------------------------------------------------------------*/
//配置中断层
	/*---------------------------------------------------------------------------------------------*/
	NVIC_InitStructure.NVIC_IRQChannel= TIM1_BRK_TIM9_IRQn; //定时器 x 中断							//@
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 1 ; //抢占优先级 3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority= 1 ; //响应优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);// 初始化 NVIC
	TIM_Cmd(TIM9,ENABLE); //使能定时器 x				4								//@
	/*---------------------------------------------------------------------------------------------*/
}

/** ----------------------------------------------------------------------------
    @FunctionName  : TIM10_Init()
    @Description   : TIM_Period和TIM_Period根据需要改动
    @Data          : 2018/7/16
    @Explain       :
    ------------------------------------------------------------------------------*/
void TIM10_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
//配置时钟及初始化预装值
	/*---------------------------------------------------------------------------------------------*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10,ENABLE); ///使能 TIMx时钟				   //@
	TIM_TimeBaseStructure.TIM_Period =5000-1;//重装载值
	TIM_TimeBaseStructure.TIM_Prescaler =8400-1;//预分频
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上累计模式，即从0开始加到溢出值
	TIM_TimeBaseInit(TIM10, &TIM_TimeBaseStructure);//配置						    		//@
	TIM_ITConfig(TIM10,TIM_IT_Update,ENABLE);//允许定时器 更新中断							//@
	/*---------------------------------------------------------------------------------------------*/
//配置中断层
	/*---------------------------------------------------------------------------------------------*/
	NVIC_InitStructure.NVIC_IRQChannel= TIM1_UP_TIM10_IRQn; //定时器 x 中断				//@
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 1 ; //抢占优先级 3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority= 0 ; //响应优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);// 初始化 NVIC
	TIM_Cmd(TIM10,ENABLE); //使能定时器 x				4								//@
	/*---------------------------------------------------------------------------------------------*/
}

/** ----------------------------------------------------------------------------
    @FunctionName  : TIM11_Init()
    @Description   : TIM_Period和TIM_Period根据需要改动
    @Data          : 2018/7/16
    @Explain       :
    ------------------------------------------------------------------------------*/
void TIM11_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
//配置时钟及初始化预装值
	/*---------------------------------------------------------------------------------------------*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11,ENABLE); ///使能 TIMx时钟				   //@
	TIM_TimeBaseStructure.TIM_Period =5000-1;//重装载值
	TIM_TimeBaseStructure.TIM_Prescaler =8400-1;//预分频
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上累计模式，即从0开始加到溢出值
	TIM_TimeBaseInit(TIM11, &TIM_TimeBaseStructure);//配置						    		//@
	TIM_ITConfig(TIM11,TIM_IT_Update,ENABLE);//允许定时器 更新中断							//@
	/*---------------------------------------------------------------------------------------------*/
//配置中断层
	/*---------------------------------------------------------------------------------------------*/
	NVIC_InitStructure.NVIC_IRQChannel= TIM1_TRG_COM_TIM11_IRQn; //定时器 x 中断			//@
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 5 ; //抢占优先级 3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority= 0 ; //响应优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);// 初始化 NVIC_
	TIM_Cmd(TIM11,ENABLE); //使能定时器 x				4								//@
	/*---------------------------------------------------------------------------------------------*/
}

/** ----------------------------------------------------------------------------
    @FunctionName  : TIM12_Init()
    @Description   : TIM_Period和TIM_Period根据需要改动
    @Data          : 2018/7/16
    @Explain       :
    ------------------------------------------------------------------------------*/
void TIM12_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
//配置时钟及初始化预装值
	/*---------------------------------------------------------------------------------------------*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12,ENABLE); ///使能 TIMx时钟				   //@
	TIM_TimeBaseStructure.TIM_Period =5000-1;//重装载值
	TIM_TimeBaseStructure.TIM_Prescaler =8400-1;//预分频
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上累计模式，即从0开始加到溢出值
	TIM_TimeBaseInit(TIM12, &TIM_TimeBaseStructure);//配置						    		//@
	TIM_ITConfig(TIM12,TIM_IT_Update,ENABLE);//允许定时器 更新中断							//@
	/*---------------------------------------------------------------------------------------------*/
//配置中断层
	/*---------------------------------------------------------------------------------------------*/
	NVIC_InitStructure.NVIC_IRQChannel= TIM8_BRK_TIM12_IRQn; //定时器 x 中断			//@
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 2; //抢占优先级 4
	NVIC_InitStructure.NVIC_IRQChannelSubPriority= 0 ; //响应优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);// 初始化 NVIC_
	TIM_Cmd(TIM12,ENABLE); //使能定时器 x				4								//@
	/*---------------------------------------------------------------------------------------------*/
}

/** ----------------------------------------------------------------------------
    @FunctionName  : TIM13_Init()
    @Description   : TIM_Period和TIM_Period根据需要改动
    @Data          : 2018/7/16
    @Explain       :
    ------------------------------------------------------------------------------*/
void TIM13_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
//配置时钟及初始化预装值
	/*---------------------------------------------------------------------------------------------*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13,ENABLE); ///使能 TIMx时钟				   //@
	TIM_TimeBaseStructure.TIM_Period =5000-1;//重装载值
	TIM_TimeBaseStructure.TIM_Prescaler =8400-1;//预分频
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上累计模式，即从0开始加到溢出值
	TIM_TimeBaseInit(TIM13, &TIM_TimeBaseStructure);//配置						    		//@
	TIM_ITConfig(TIM13,TIM_IT_Update,ENABLE);//允许定时器 更新中断							//@
	/*---------------------------------------------------------------------------------------------*/
//配置中断层
	/*---------------------------------------------------------------------------------------------*/
	NVIC_InitStructure.NVIC_IRQChannel= TIM8_UP_TIM13_IRQn; //定时器 x 中断			//@
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 5 ; //抢占优先级 3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority= 0 ; //响应优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);// 初始化 NVIC_
	TIM_Cmd(TIM13,ENABLE); //使能定时器 x				4								//@
	/*---------------------------------------------------------------------------------------------*/
}


/** ----------------------------------------------------------------------------
    @FunctionName  : TIM14_Init()
    @Description   : TIM_Period和TIM_Period根据需要改动
    @Data          : 2018/7/16
    @Explain       :
    ------------------------------------------------------------------------------*/
void TIM14_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
//配置时钟及初始化预装值
	/*---------------------------------------------------------------------------------------------*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14,ENABLE); ///使能 TIMx时钟				   //@
	TIM_TimeBaseStructure.TIM_Period =5000-1;//重装载值
	TIM_TimeBaseStructure.TIM_Prescaler =8400-1;//预分频
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上累计模式，即从0开始加到溢出值
	TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStructure);//配置						    		//@
	TIM_ITConfig(TIM14,TIM_IT_Update,ENABLE);//允许定时器 更新中断							//@
	/*---------------------------------------------------------------------------------------------*/
//配置中断层
	/*---------------------------------------------------------------------------------------------*/
	NVIC_InitStructure.NVIC_IRQChannel= TIM8_TRG_COM_TIM14_IRQn; //定时器 x 中断			//@
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 5 ; //抢占优先级 3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority= 0 ; //响应优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);// 初始化 NVIC_
	TIM_Cmd(TIM14,ENABLE); //使能定时器 x				4								//@
	/*---------------------------------------------------------------------------------------------*/
}








