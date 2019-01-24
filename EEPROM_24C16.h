#ifndef EEPROM24C16H
#define EEPROM24C16H

#include <REG52.H>	/* Special function register declarations */             
#include "Somenop.h"

sbit ECLK  = P1^1;
sbit EDTA  =  P1^0;

void estart();
void estop();
bit ack();
void ewrite_byte(unsigned char dat);
unsigned char eread_byte();
void ewrite_add(unsigned char add,unsigned char dat);
unsigned char eread_add(unsigned char add);

#endif