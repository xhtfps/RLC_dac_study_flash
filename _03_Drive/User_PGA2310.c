/* ****************************
 * Project description:
 *
 * PGA2310 initialization file
 *
 * Author: 创新基地 -> 2019 Mao
 *
 * Creation Date: 2021/11/02 - 1
 * ****************************/
 
/* ***************************** Include & Define Part     	*****************************
 * 头文件声明及宏定义区
 * */
#include "User_PGA2310.h"

/* ***************************** Variable definition Part   *****************************
 * 全局变量定义区
 * */

/* ***************************** Initialization Part        *****************************
 * 初始化函数区
 * */

void PGA2310_Init()
{
	// 初始化SPI通信
	SPI_GPIO_Init( 1 );
	
	PGA2310_SetAv( 0 , 0 );
}

/* ***************************** Custom Function Part       *****************************
 * 自定义函数区
 */

void PGA2310_SetAv( float av_l , float av_r )
{
	uint8_t data_l = 0 , data_r = 0;
	uint16_t data =  0;
	
	if( av_l > 31.5 || av_l < -95.5 )
		av_l = 0;
	if( av_r > 31.5 || av_r < -95.5 )
		av_r = 0;
	
	// Av = 31.5 - 0.5 * ( 255 - data )
	// -> data = 2 * Av + 192
	data_l = 2 * av_l + 192;
	data_r = 2 * av_r + 192;
	
	data = (data_r << 8) | data_l;
	
	User_SPI_SendData( data , 16 , 1 );
	
}

