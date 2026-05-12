/* ****************************
 * Project description:
 *
 * PGA2310 initialization head file
 *
 * Author: 创新基地 -> 2019 Mao
 *
 * Creation Date: 2021/11/02 - 1
 * ****************************/

#ifndef USER_PGA2310_H
#define USER_PGA2310_H

/* ***************************** Include & Define Part     	*****************************/
#include "User_header.h"


/* ***************************** Function Declaration Part  *****************************/

void PGA2310_Init( void );

void PGA2310_SetAv( float av_l , float av_r );



#endif
