#ifndef HD7279AH
#define HD7279AH

#define CMD_RESET 0xA4
#define Somenop();       _nop_();_nop_();_nop_();_nop_();_nop_();
#define Somenop10();	 Somenop();Somenop();
#define Somenop25();	 Somenop();Somenop();Somenop();Somenop();Somenop();
#define Somenop50();	 Somenop25();Somenop25();

void send_byte(unsigned char cmd);
unsigned char receive_byte(void);
void Init_7279(void);
void write_7279(unsigned char cmd, unsigned char dat);
unsigned char ReadKey(void);

#endif