#include "UART.h"

void UART_ISR(void) interrupt 4 {
    U8 RX_Data;
    //只响应"接收"中断，"发送"中断来了就直接抹掉
    if (RI) {
        RI = 0;	//串口中断标志不能自己清除，需要手动清除
        RX_Data = SBUF;
        SendOneByte(RX_Data);
    } else
        TI = 0;		//串口发中断是发送完缓冲区数据之后产生
}
 
/****************串口初始化函数*************/
void InitUART(void) {
    TMOD = 0x20;    //定时器1，模式2工作模式	   
    SCON = 0x50;    //串口工作模式1，允许REN   /* SCON: 模式 1,  8-bit UART, 使能接收         */
    TH1 = TC_VAL;
    TL1 = TH1;
    PCON = 0x80; 	//发送速率加倍
    ES = 1;
    EA = 1;
    TR1 = 1;
}
/**************串口发送字符函数*************/
void SendOneByte(U8 c) {
    ES = 0;			//禁止中断，让串口安心工作啊
    SBUF = c;
    while(!TI);		//等待发送完毕
    TI = 0;			//清TI中断
    ES = 1;			//打开中断
}
