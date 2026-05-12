#ifndef DRIVE_ADF4351_H_
#define DRIVE_ADF4351_H_

#include "User_header.h"

#define ADF4351_CLK 					PGout(7)
#define ADF4351_OUTPUT_DATA 			PGout(5)  
#define ADF4351_LE 						PGout(3)
#define ADF4351_CE 						PGout(1)




void ADF4351Init(void);
void ReadToADF4351(u8 count, u8 *buf);
void WriteToADF4351(u8 count, u8 *buf);
void WriteOneRegToADF4351(u32 Regster);
void ADF4351_Init_some(void);
void ADF4351WriteFreq(float Fre);		//	(xx.x) M Hz

#endif

