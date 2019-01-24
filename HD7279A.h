#ifndef HD7279AH
#define HD7279AH

#define CMD_RESET 0xA4

#include "Somenop.h"

void send_byte(unsigned char cmd);
unsigned char receive_byte(void);
void Init_7279(void);
void write_7279(unsigned char cmd, unsigned char dat);
unsigned char ReadKey(void);

#endif