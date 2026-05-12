/*
*********************************************************************************************************
                                               _04_OS
    File			 : Drive_TouchKey.c
    By  			 : Muhe
    platform   : STM32F407ZG
	Data   		 : 2018/7/16
    function 	 : 触屏键盘驱动程序
*********************************************************************************************************
*/

#include "Drive_TouchKey.h"

u16 start_x=530; //起始横坐标
u16 start_y=30;  //起始纵坐标
u16 shift_x=60;  //横坐标偏移量
u16 shift_y=60;  //纵坐标偏移量

/*
*************************************************************************

	功能：	画键盘
	参数：

**************************************************************************
*/
void TouchKey_Draw(void)
{
	OS_Rect_Draw(start_x,start_y,start_x+shift_x*4,start_y+shift_y*5,1,Black);//画外轮廓线
	OS_Line_Draw(start_x,start_y+shift_y*1,start_x+shift_x*4,start_y+shift_y*1,Black);//画横线
	OS_Line_Draw(start_x,start_y+shift_y*2,start_x+shift_x*4,start_y+shift_y*2,Black);
	OS_Line_Draw(start_x,start_y+shift_y*3,start_x+shift_x*4,start_y+shift_y*3,Black);
	OS_Line_Draw(start_x,start_y+shift_y*4,start_x+shift_x*3,start_y+shift_y*4,Black);
	OS_Line_Draw(start_x+shift_x*1,start_y+shift_y*1,start_x+shift_x*1,start_y+shift_y*5,Black);//画竖线
	OS_Line_Draw(start_x+shift_x*2,start_y+shift_y*1,start_x+shift_x*2,start_y+shift_y*5,Black);
	OS_Line_Draw(start_x+shift_x*3,start_y+shift_y*1,start_x+shift_x*3,start_y+shift_y*4,Black);
	OS_String_Show(start_x+20,start_y+shift_y*1+15,32,0,"1   2   3  <--");//显示键
	OS_String_Show(start_x+20,start_y+shift_y*2+15,32,0,"4   5   6  C");
	OS_String_Show(start_x+20,start_y+shift_y*3+15,32,0,"7   8   9");
	OS_String_Show(start_x+20,start_y+shift_y*4+15,32,0,"0   .   Enter");
}


/*
*************************************************************************

	功能：	清空显示区
	参数：

**************************************************************************
*/
void Clear_Show(void)
{
	OS_Rect_Draw(start_x+1,start_y+1,start_x+shift_x*4-1,start_y+shift_y*1-1,0,White);
}

/*
*************************************************************************

	功能：	扫描按键
	参数：

**************************************************************************
*/
float TouchKey_Scan(void)
{
	static u8 End_flag=0,count=0,databuf[40],press_flag=0;
	if(Touch_Judge(start_x+shift_x*0,start_y+shift_y*1,start_x+shift_x*1,start_y+shift_y*2) == TOUCH_VALID_FULL)
	{
		databuf[count]='1';
		press_flag=1;
	}
	if(Touch_Judge(start_x+shift_x*1,start_y+shift_y*1,start_x+shift_x*2,start_y+shift_y*2) == TOUCH_VALID_FULL)
	{
		databuf[count]='2';
		press_flag=1;
	}
	if(Touch_Judge(start_x+shift_x*2,start_y+shift_y*1,start_x+shift_x*3,start_y+shift_y*2) == TOUCH_VALID_FULL)
	{
		databuf[count]='3';
		press_flag=1;
	}
	if(Touch_Judge(start_x+shift_x*3,start_y+shift_y*1,start_x+shift_x*4,start_y+shift_y*2) == TOUCH_VALID_FULL)
	{
		count--;
		Clear_Show();
	}
	if(Touch_Judge(start_x+shift_x*0,start_y+shift_y*2,start_x+shift_x*1,start_y+shift_y*3) == TOUCH_VALID_FULL)
	{
		databuf[count]='4';
		press_flag=1;
	}
	if(Touch_Judge(start_x+shift_x*1,start_y+shift_y*2,start_x+shift_x*2,start_y+shift_y*3) == TOUCH_VALID_FULL)
	{
		databuf[count]='5';
		press_flag=1;
	}
	if(Touch_Judge(start_x+shift_x*2,start_y+shift_y*2,start_x+shift_x*3,start_y+shift_y*3) == TOUCH_VALID_FULL)
	{
		databuf[count]='6';
		press_flag=1;
	}
	if(Touch_Judge(start_x+shift_x*3,start_y+shift_y*2,start_x+shift_x*4,start_y+shift_y*3) == TOUCH_VALID_FULL)
	{
		count=0;
		Clear_Show();
	}
	if(Touch_Judge(start_x+shift_x*0,start_y+shift_y*3,start_x+shift_x*1,start_y+shift_y*4) == TOUCH_VALID_FULL)
	{
		databuf[count]='7';
		press_flag=1;
	}
	if(Touch_Judge(start_x+shift_x*1,start_y+shift_y*3,start_x+shift_x*2,start_y+shift_y*4) == TOUCH_VALID_FULL)
	{
		databuf[count]='8';
		press_flag=1;
	}
	if(Touch_Judge(start_x+shift_x*2,start_y+shift_y*3,start_x+shift_x*3,start_y+shift_y*4) == TOUCH_VALID_FULL)
	{
		databuf[count]='9';
		press_flag=1;
	}
	if(Touch_Judge(start_x+shift_x*3,start_y+shift_y*3,start_x+shift_x*4,start_y+shift_y*4) == TOUCH_VALID_FULL)
		End_flag=1;
	if(Touch_Judge(start_x+shift_x*0,start_y+shift_y*4,start_x+shift_x*1,start_y+shift_y*5) == TOUCH_VALID_FULL)
	{
		databuf[count]='0';
		press_flag=1;
	}
	if(Touch_Judge(start_x+shift_x*1,start_y+shift_y*4,start_x+shift_x*2,start_y+shift_y*5) == TOUCH_VALID_FULL)
	{
		databuf[count]='.';
		press_flag=1;
	}
	if(Touch_Judge(start_x+shift_x*2,start_y+shift_y*4,start_x+shift_x*4,start_y+shift_y*5) == TOUCH_VALID_FULL)
		End_flag=1;
	if(press_flag==1)
	{
		if(count == 0)
			Clear_Show();
		count++;
		press_flag=0;
	}
	databuf[count]='\0';
	if(count!=0)
		OS_String_Show(start_x+10,start_y+15,32,0,(char *)databuf); //显示输入值
	if(End_flag==1 && count!=0)
	{
		count=0;
		End_flag=0;
		return atof((char *)databuf);
	}
	else
		return 0;
}

void Interface(float data1,float data2,u8 step,u8 mode)
{
	if(mode == 0)
	{
		LCD_Clear(White);
		OS_String_Show(30,30,32,0,"AD8370_Gain_db");
		OS_Rect_Draw(280,25,390,70,1,Black);
		OS_Num_Show(290,30,32,1,data1,"%.0f");
		OS_String_Show(30,80,32,0,"           Av:");
		OS_Num_Show(290,80,32,1,data2,"%.3f");
		OS_String_Show(30,165,32,0,"Step:   1     4");
		OS_Rect_Draw(130,150,200,210,1,Black);
		OS_Rect_Draw(230,150,300,210,1,Black);
		if(step == 1)
			OS_Rect_Draw(130,150,200,210,1,Red);
		else if(step == 4)
			OS_Rect_Draw(230,150,300,210,1,Red);
	}
	else
	{
		LCD_Clear(White);
		OS_String_Show(640,400,32,0,"校准");
		OS_Rect_Draw(625,380,720,450,1,Black);
		OS_String_Show(30,30,32,0,"AD8370_1 Gain");
		OS_Rect_Draw(250,25,360,70,1,Black);
		OS_Num_Show(260,30,32,1,data1,"%.3f");
		OS_String_Show(30,90,32,0,"AD8370_2 Gain");
		OS_Rect_Draw(250,85,360,130,1,Black);
		OS_Num_Show(260,90,32,1,data2,"%.3f");
		OS_String_Show(30,165,32,0,"Step:   1     2     5     10");
		OS_Rect_Draw(130,150,200,210,1,Black);
		OS_Rect_Draw(230,150,300,210,1,Black);
		OS_Rect_Draw(330,150,400,210,1,Black);
		OS_Rect_Draw(430,150,500,210,1,Black);
		switch(step)
		{
			case 1 :
				OS_Rect_Draw(130,150,200,210,1,Red);
				break;
			case 2 :
				OS_Rect_Draw(230,150,300,210,1,Red);
				break;
			case 5 :
				OS_Rect_Draw(330,150,400,210,1,Red);
				break;
			case 10:
				OS_Rect_Draw(430,150,500,210,1,Red);
				break;
			default:
				break;
		}
	}
}




