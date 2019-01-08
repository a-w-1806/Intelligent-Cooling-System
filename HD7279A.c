#include <REG52.H>
#include <intrins.h>

#include "HD7279A.h"

sbit CS  = P1^4;
sbit CLK  = P1^5;
sbit DATA = P1^7;

/*  Send data to HD7279A according to the protocol */
void send_byte(unsigned char cmd) {
    unsigned char i;
    Somenop50(); Somenop50(); //时序初始延时
    for(i = 0; i < 8; i++) {       
        /* Write the most significant bit (MSB) of cmd. */
        /* If MSB is 1, then write 1, else 0 */
        if (cmd & 0x80)
            DATA = 1; 
        else
            DATA = 0;
        // CLK形成下降沿,先1后0，注意延时时间要满足时序要求
        CLK = 1; Somenop10(); Somenop10();
        CLK = 0; Somenop10(); Somenop10();
        /* Get next bit ready */
        cmd = cmd << 1;       
    }
    DATA = 0;  //依照时序要求全部写完后Data清零 
}		  

unsigned char receive_byte(void) {
    unsigned char i, in_byte;
    DATA = 1;  
    Somenop50(); Somenop50(); 
    for (i = 0; i < 8; i++) {
        CLK = 1; Somenop10(); Somenop10();
        //左移一位，空出最后一位存放新进来的位DATA 
        in_byte = in_byte * 2;  
        //若新进来的位DATA为1，则in_byte的末位置1，否则不需要变
        if (DATA)  
            in_byte=in_byte|0x01;
        CLK = 0; Somenop10(); Somenop10();
    }
    DATA = 0;
    return in_byte; //返回接收值
}

void Init_7279(void) {
     CS = 0;  			        // 片选使能置0
     Somenop50(); Somenop50();    // 延时
     send_byte(CMD_RESET);          // 7279复位命令
                                  // CMD_RESET=A4H 
     Somenop50(); Somenop50();  // 再延时
     CS = 1;   	   // 片选使能置1，完成初始化
}

void write_7279(unsigned char cmd, unsigned char dat) {
     CS = 0;
     Somenop50(); Somenop();
     send_byte(cmd);           // 写指令
     Somenop25(); Somenop();
     send_byte(dat);        // 写数据
     Somenop(); Somenop();
     CS = 1;
}
	
/* Return the code of the current pressed key */
unsigned char ReadKey(void) {
   unsigned char readkey;
   CS = 0;
   Somenop50(); Somenop50();
   send_byte(0x15);		  //读键值命令15H
   Somenop25(); Somenop25();
   readkey = receive_byte(); 	  //接收键值(按时序接收)
   CS = 1;
   return readkey;		  //返回键值
}
