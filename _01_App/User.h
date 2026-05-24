/* ****************************
 * Project description:
 *
 * A Project empty template head file
 *
 * Author: 创新基地 -> 2019 Mao
 *
 * Creation Date: 2021/09/05
 *
 * Update date: 2021/11/03
 * ****************************/
#ifndef USER_H
#define USER_H

/* ***************************** Include & Define Part     	*****************************
 * 头文件声明及宏定义区
 * */
#include "User_header.h"

#define TitleLength 7
#define TitleStr "网络阻抗测试仪"
#define ModelVerStr "MD"
#define UserVerStr " User:XHT"

#define MenuChoiceNum 5
#define Menu1Choice1 "RLC测量"
#define Menu1Choice2 "网络测量"
#define Menu1Choice3 "信号源调节"
#define Menu1Choice4 "学习模式"
#define Menu1Choice5 "恢复出厂"

/* ***************************** Function Declaration Part  *****************************
 * 函数声明区
 * */
void User_main(void);

void Init_All(void);

void Disp_Main(void);
void Show_Val( uint8_t location , float value , char *str );
void Change_Menu( uint8_t menu_sign );

void MenuHaddler_1(void);
void MenuHaddler_2(void);
void MenuHaddler_3(void);
void MenuHaddler_4(void);
void MenuHaddler_5(void);

float PS2_ReadNum( float num );



extern float Sample_rate;//采样率
/* ***************************** Variable definition Part   *****************************/


#endif



