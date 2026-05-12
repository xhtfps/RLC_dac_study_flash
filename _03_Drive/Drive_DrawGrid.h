#ifndef DRIVE_DRAWGRID_H
#define DRIVE_DRAWGRID_H

#include "User_header.h"

#define GridDataLenth 40

typedef struct GridStruct
{
	uint16_t Xstart;							//起始横坐标
	uint16_t Ystart;							//起始纵坐标
	uint16_t Xend;								//终止横坐标
	uint16_t Yend;								//终止纵坐标坐标

	uint16_t GridColor;						//网格颜色
	uint16_t CursorColor;					//光标颜色

	uint16_t GridMode;						//有无网格	

	uint16_t Xnumber;							//横线数量(包括外轮廓)
	uint16_t Ynumber;							//竖线数量(包括外轮廓)

	float Xmax;										//横坐标最大值
	float Xmin;										//横坐标最小值
	float Ymax;										//纵坐标最大值
	float Ymin;										//纵坐标最小值
	
	uint8_t CursorMode;						//有无光标
	float CursorX;     						//横光标坐标
	float CursorY;    						//纵光标坐标
	float LastCursorX; 						//上次横光标坐标
	float LastCursorY; 						//上次纵光标坐标

}GridStruct;

















#endif








