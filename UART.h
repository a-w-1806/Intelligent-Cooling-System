#ifndef UARTH
#define UARTH

#define U8 unsigned char
#define FOSC    11059200        //振荡频率
#define BAUD    9600            //波特率
#define TC_VAL  (256-FOSC/16/12/BAUD)

#include <REG52.H>	/* Special function register declarations */    

void UART_ISR(void);
void InitUART(void);
void SendOneByte(U8 c);

#endif // UARTH
