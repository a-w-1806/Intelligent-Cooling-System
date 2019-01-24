#ifndef DS18B20H
#define DS18B20H

#include <REG52.H>
#include "HD7279A.h"

#define U8 unsigned char
#define U16 unsigned short

sbit DS18B20_DQ = P1^3;

void DS18B20_Init();
bit DS18B20_Reset();
void DS18B20_WriteData(U8 wData);
U8 DS18B20_ReadData();
#endif