#ifndef __DRIVE_TOUCHKEY_H
#define __DRIVE_TOUCHKEY_H

#include "User_header.h"

void TouchKey_Draw(void);
void Clear_Show(void);
float TouchKey_Scan(void);
void Interface(float data1,float data2,u8 step,u8 mode) ;

#endif

