/*
*********************************************************************************************************
                                               _04_OS
    File			 : Drive_Interruput.c
    By  			 : Muhe
    platform   : STM32F407ZG
	Data   		 : 2018/7/16
    function 	 : 中断服务函数配置程序
*********************************************************************************************************
*/
#include "Drive_Interrupt.h"


/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有（静态）函数声明 -------------------------------------------------------*/

/* 全局变量定义 --------------------------------------------------------------*/
double g_Freq=0;
extern  u16 Cnt_Times;
extern  u32 rising,rising_last,falling;
extern u8 Display_Flag;
/* 全局函数编写 --------------------------------------------------------------*/


/** ----------------------------------------------------------------------------
    @FunctionName  : TIM1_UP_TIM10_IRQHandler()
    @Description   : 自己根据需要配置中断逻辑
    @Data          : 2018/7/16
    @Explain       :
    ------------------------------------------------------------------------------*/
void TIM1_UP_TIM10_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM1,TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM1,TIM_IT_Update); //清除中断标志位
	}
}

/** ----------------------------------------------------------------------------
    @FunctionName  : TIM2_IRQHandler()
    @Description   : 测频
    @Data          : 2018/7/16
    @Explain       :
    ------------------------------------------------------------------------------*/
//=========================================================================
void TIM2_IRQHandler(void)
{
    static u16 Cnt_Interrupt;
//	static u32 NowCapture=0,LastCapture=0;
//	static u32 Capture=0;
//	if(TIM_GetITStatus(TIM2,TIM_IT_CC3)!=RESET)
//	{
//		TIM_ClearITPendingBit(TIM2,TIM_IT_CC3);
//		NowCapture=TIM2->CCR3;
//		if(NowCapture>LastCapture)
//			Capture=NowCapture-LastCapture;
//		else
//			Capture=(0xffffffff-NowCapture)+LastCapture;
//		g_Freq=SystemCoreClock*10/2/Capture;
//		LastCapture=NowCapture;
//	}

    if(TIM2->SR&TIM_FLAG_CC3&&(Display_Flag==0))//TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)//捕获1发生捕获事件
    {
        if(Cnt_Interrupt>=Cnt_Times)
        {
            rising=TIM2->CCR3-rising_last;
            rising_last=TIM2->CCR3;
            Cnt_Interrupt=0;
        }
        Cnt_Interrupt++;
//         return;
    }
//     if(TIM2->SR&TIM_FLAG_CC2)//TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET)
//     {
//         falling=TIM2->CCR2-rising_last;
//         return;
//     }
     TIM2->SR=0;

}


/** ----------------------------------------------------------------------------
    @FunctionName  : TIM3_IRQHandler()
    @Description   : 自己根据需要配置中断逻辑
    @Data          : 2018/7/16
    @Explain       :
    ------------------------------------------------------------------------------*/
void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM3,TIM_IT_Update); //清除中断标志位
	}
}

/** ----------------------------------------------------------------------------
    @FunctionName  : TIM4_IRQHandler()
    @Description   : 自己根据需要配置中断逻辑
    @Data          : 2018/7/16
    @Explain       : 通信编码
    ------------------------------------------------------------------------------*/
void TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4,TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM4,TIM_IT_Update); //清除中断标志位
	}
}

/** ----------------------------------------------------------------------------
    @FunctionName  : TIM5_IRQHandler()
    @Description   : 自己根据需要配置中断逻辑
    @Data          : 2018/7/16
    @Explain       : 通信接收
    ------------------------------------------------------------------------------*/
void TIM5_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM5,TIM_IT_CC3) != RESET)
	{
		TIM_ClearITPendingBit(TIM5,TIM_IT_CC3|TIM_IT_Update); //清除中断标志位
	}
	else if(TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM5, TIM_IT_CC3|TIM_IT_Update); //清除中断标志位
	}
}

/** ----------------------------------------------------------------------------
    @FunctionName  : TIM6_DAC_IRQHandler()
    @Description   : 自己根据需要配置中断逻辑
    @Data          : 2018/7/16
    @Explain       :
    ------------------------------------------------------------------------------*/
void TIM6_DAC_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM6,TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM6,TIM_IT_Update); //清除中断标志位
	}
}

/** ----------------------------------------------------------------------------
    @FunctionName  : TIM7_IRQHandler()
    @Description   : 自己根据需要配置中断逻辑
    @Data          : 2018/7/16
    @Explain       :
    ------------------------------------------------------------------------------*/
void TIM7_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM7,TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM7,TIM_IT_Update); //清除中断标志位
	}
}

/** ----------------------------------------------------------------------------
    @FunctionName  : TIM8_UP_TIM13_IRQHandler()
    @Description   : 自己根据需要配置中断逻辑
    @Data          : 2018/7/16
    @Explain       : 超声波测距
    ------------------------------------------------------------------------------*/
void TIM8_UP_TIM13_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM8,TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM8,TIM_IT_Update); //清除中断标志位
	}
}

/** ----------------------------------------------------------------------------
    @FunctionName  : TIM1_BRK_TIM9_IRQHandler()
    @Description   : 自己根据需要配置中断逻辑
    @Data          : 2018/7/16
    @Explain       :
    ------------------------------------------------------------------------------*/
void TIM1_BRK_TIM9_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM9,TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM9,TIM_IT_Update); //清除中断标志位
	}
}


/** ----------------------------------------------------------------------------
    @FunctionName  : TIM1_TRG_COM_TIM11_IRQHandler()
    @Description   : 自己根据需要配置中断逻辑
    @Data          : 2018/7/16
    @Explain       :
    ------------------------------------------------------------------------------*/
void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM11,TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM11,TIM_IT_Update); //清除中断标志位
	}
}

/** ----------------------------------------------------------------------------
    @FunctionName  : TIM8_BRK_TIM12_IRQHandler()
    @Description   : 自己根据需要配置中断逻辑
    @Data          : 2018/7/16
    @Explain       :
    ------------------------------------------------------------------------------*/
void TIM8_BRK_TIM12_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM12,TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM12,TIM_IT_Update); //清除中断标志位
	}
}

/** ----------------------------------------------------------------------------
    @FunctionName  : TIM8_TRG_COM_TIM14_IRQHandler()
    @Description   : 自己根据需要配置中断逻辑
    @Data          : 2018/7/16
    @Explain       :
    ------------------------------------------------------------------------------*/
void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM14,TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM14,TIM_IT_Update); //清除中断标志位
	}
}

/** ----------------------------------------------------------------------------
    @FunctionName  : EXTI0_IRQHandler()
    @Description   : 外部中断0服务程序
    @Data          : 2018/7/16
    @Explain       : 超声波测距
    ------------------------------------------------------------------------------*/
void EXTI0_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line0)!=RESET)
	{
	}
}

/** ----------------------------------------------------------------------------
    @FunctionName  : EXTI1_IRQHandler()
    @Description   : 外部中断1服务程序
    @Data          : 2018/7/16
    @Explain       :
    ------------------------------------------------------------------------------*/
void EXTI1_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line1)!=RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line1); //清除LINE1上的中断标志位
	}
}

/** ----------------------------------------------------------------------------
    @FunctionName  : EXTI2_IRQHandler()
    @Description   : 外部中断2服务程序
    @Data          : 2018/7/16
    @Explain       :
    ------------------------------------------------------------------------------*/
void EXTI2_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line2)!=RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line2); //清除LINE2上的中断标志位
	}
}

/** ----------------------------------------------------------------------------
    @FunctionName  : EXTI3_IRQHandler()
    @Description   : 外部中断3服务程序
    @Data          : 2018/7/16
    @Explain       : 自己根据需要配置中断逻辑
    ------------------------------------------------------------------------------*/
void EXTI3_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line3)!=RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line3); //清除LINE3上的中断标志位
	}
}

/** ----------------------------------------------------------------------------
    @FunctionName  : EXTI4_IRQHandler()
    @Description   : 外部中断4服务程序
    @Data          : 2018/7/16
    @Explain       : 自己根据需要配置中断逻辑
    ------------------------------------------------------------------------------*/
void EXTI4_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line4)!=RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line4); //清除LINE3上的中断标志位
	}
}

/** ----------------------------------------------------------------------------
    @FunctionName  : EXTI9_5_IRQHandler()
    @Description   : 外部中断5-9服务程序，自己根据需要配置中断线
    @Data          : 2018/7/16
    @Explain       : 自己根据需要配置中断逻辑
    ------------------------------------------------------------------------------*/
void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line5)!=RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line5); //清除LINE5上的中断标志位
	}
}

/** ----------------------------------------------------------------------------
    @FunctionName  : EXTI15_10_IRQHandler()
    @Description   : 外部中断10-15服务程序，自己根据需要配置中断线
    @Data          : 2018/7/16
    @Explain       : 自己根据需要配置中断逻辑
    ------------------------------------------------------------------------------*/
void EXTI15_10_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line10)!=RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line10); //清除LINE10上的中断标志位
	}
}















