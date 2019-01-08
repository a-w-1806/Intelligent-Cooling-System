/*
@author
Yuchen Wang

@Processor
STC89C52: 8-bit Microcontroller
	ROM: 8K bytes flash
	RAM: 256 bytes, so we need two hex numbers for addressing.

@Chip
HD7279A: used to connect to LED and keyboard.

@Temperature_Sensor
DS18B20

@Memory
24C16: A 16K E2PROM (Electrically Erasable Programmable Read-Only Memory)
	It can store data even without electriciy, which is different from RAM.

@Other
Cooling Fan
Power
*/

#include <REG52.H>                
#include "main.h"

#include <stdio.h>                
#include <math.h>
#include <intrins.h>
#include <string.h>

unsigned char LEDValue[50] = {0xFC,0x44,0x79,0x5D,0xC5,0x9D,0xBD,0x54,0xFD,0xDD,0xF5,0xAD,0xB8,0x6D,0xB9,0xB1, //0-F
  							0xFE,0x46,0x7B,0x5F,0xC7,0x9F,0xBF,0x56,0xFF,0xDF,0xF7,0xAF,0xBA,0x6F,0xBB,0xB3, //0.-F.
	                        0x00,0xA9,0xF1,0x21,0x2C,0x25,0x2D,0x01}; // 32null，33t，34P,35r,36u,37n,38o,39-
  
unsigned char DispBuff[8] = {32,32,32,32,32,32,32,32};
unsigned char code KeyTabel[4] = {0x3B, 0x3A, 0x39, 0x38}; // down up back enter

unsigned char run_options[10];
unsigned char run_options_start_address = 0;

unsigned char temp_threshold[2];
unsigned char temp_threshold_start_address = 10;

unsigned char temperature[2];

unsigned char motor_thre;
unsigned char motor_now = 0;

unsigned char timerH = 0xFF;
unsigned char timerL = 0x00;

unsigned char counter = 0;

unsigned char temp[5]; // bai shi ge shifen baifen

unsigned char PWM = 50;

unsigned char TEMP_CONTROL_MODE = FALSE;

unsigned char KeyFlag = 0;

unsigned char PIDPwd[4] = {0,0,0,0};
unsigned char PIDGoalTempAddress = 12;

struct _pid {
	unsigned char goal_temp[3];
	unsigned char current_temp[3];
	unsigned char Kp, Ki, Kd;
	float current_PWM;
	float err, err_last;
	float integral;
} pid;

unsigned char check_LED (char c) {
	if (c >= '0' && c <= '9') return (c - '0');
	else if (c >= 'A' && c <= 'F') return (c - 'A' + 10);
	switch (c) {
		case ' ': return 32;
		case 't': return 33;
		case 'P': return 34;
		case 'r': return 35;
		case 'u': return 36;
		case 'n': return 37;
		case 'o': return 38;
		case '-': return 39;
		case 'b': return 11;
		case 'd': return 13;
	}
}

void show_main_menu(void) {
	char* menu_buffer[] = {"tP- ", "run-", "Con-", "PA- ", "P1d-"};
	unsigned char current_menu = 0;
	while(1) {
		display(DispBuff); //显示（按显缓单元的内容显示）
		Key();
		if (KeyValue != 0xff) {   
		// 有键按下，按照键号执行菜单显示或进入子菜单
			if (KeyNum == DOWN) {
				current_menu = change_menu_ptr(current_menu, FALSE, 5);
				display_string_in_row(menu_buffer[current_menu], TRUE);
			} else if (KeyNum == UP) {
				current_menu = change_menu_ptr(current_menu, TRUE, 5);
				display_string_in_row(menu_buffer[current_menu], TRUE);
			} else if (KeyNum == ENTER) {
				switch(current_menu) {
					case 0: show_temperature(FALSE); break;
					case 1: show_motor_test(); break;
					case 2:	con_with_temp(FALSE); 
							display_string_in_row("Con-", TRUE); 
							Motor = 0; 
							break;
					case 3: show_PA_menu(); break;
					case 4: con_with_temp(TRUE); 
							display_string_in_row("Con-", TRUE); 
							Motor = 0;
							break;
				}
				display_string_in_row("    ", FALSE);
			}
		}
  }
}

///////////////////////////////////////////////////////////////////////////////

void DS18B20_Init()
{
	DS1820_Reset();
	DS1820_WriteData(0xCC); // 跳过ROM
	DS1820_WriteData(0x4E); // 写暂存器
	DS1820_WriteData(0x20); // 往暂存器的第三字节中写上限值
	DS1820_WriteData(0x00); // 往暂存器的第四字节中写下限值
	DS1820_WriteData(0x7F); // 将配置寄存器配置为12 位精度
	DS1820_Reset();
}


/**********************************************************
*DS1820 复位及存在检测(通过存在脉冲可以判断DS1820 是否损坏)
*函数名称:DS1820_Reset()
*说明:函数返回一个位标量(0 或1)flag=0 存在,反之flag=1 不存在
**********************************************************/
bit DS1820_Reset()
{
	U8 i;
	bit flag;
	DS1820_DQ = 0; //拉低总线
	for (i=240;i>0;i--); //延时480 微秒,产生复位脉冲
	DS1820_DQ = 1; //释放总线
	for (i=40;i>0;i--); //延时80 微秒对总线采样
	flag = DS1820_DQ; //对数据脚采样
	for (i=200;i>0;i--); //延时400 微秒等待总线恢复
	return (flag); //根据flag 的值可知DS1820 是否存在或损坏 ，可加声音告警提示DS1820 故障
}

/**********************************************************
*写数据到DS1820
*函数名称:DS1820_WriteData()
**********************************************************/
void DS1820_WriteData(U8 wData)
{
    U8 i,j;
    for (i=8;i>0;i--)
    {
    DS1820_DQ = 0; //拉低总线,产生写信号
    for (j=2;j>0;j--); //延时4us
    DS1820_DQ = wData&0x01; //发送1 位
    for (j=30;j>0;j--); //延时60us,写时序至少要60us
    DS1820_DQ = 1; //释放总线,等待总线恢复
    wData>>=1; //准备下一位数据的传送
    }
}

/**********************************************************
*从DS1820 中读出数据
*函数名称:DS1820_ReadData()
**********************************************************/
U8 DS1820_ReadData()
{
    U8 i,j,TmepData;
    for (i=8;i>0;i--)
    {
    TmepData>>=1;
    DS1820_DQ = 0; //拉低总线,产生读信号
    for (j=2;j>0;j--); //延时4us
    DS1820_DQ = 1; //释放总线,准备读数据
    for (j=4;j>0;j--); //延时8 微秒读数据
    if (DS1820_DQ == 1)
    { TmepData |= 0x80;}
    for (j=30;j>0;j--); //延时60us
    DS1820_DQ = 1; //拉高总线,准备下一位数据的读取.
    }
    return (TmepData);//返回读到的数据
}

void Init_7279(void)
{
     CS = 0;  			        // 片选使能置0
     Somenop50();Somenop50();    // 延时
     send_byte(CMD_RESET);          // 7279复位命令
                                  // CMD_RESET=A4H 
     Somenop50();Somenop50();  // 再延时
     CS = 1;   	   // 片选使能置1，完成初始化
}

void display(unsigned char buff[])
{
  unsigned char i;
  for( i=0 ; i<8 ; i++ )
     write_7279(0x90+i,LEDValue[buff[i]]);	 
}





 unsigned char ReadKey(void)
{
   unsigned char readkey;
   CS = 0;
   Somenop50();Somenop50();
   send_byte(0x15);		  //读键值命令15H
   Somenop25();Somenop25();
   readkey = receive_byte(); 	  //接收键值(按时序接收)
   CS = 1;
   return(readkey);		  //返回键值
}




 void Key(void)
{
   unsigned char temp,i;
   temp = ReadKey();  //读键值
   if (temp==0xff)
   {
	KeyNum = 0xff;	
	KeyValue = 0xff;
	KeyFlag = 0;
   }
   else
   {
      if(KeyValue!=0xff && KeyFlag < 250){
		KeyNum=0xff;
		KeyFlag++;
	  } 
      else{
		KeyValue = temp;  
		for(i=0;i<=3;i++)
			if (KeyValue == KeyTabel[i])
			{ KeyNum = i; break; }	
      }

   }	
}


void write_7279(unsigned char cmd, unsigned char dat) 
{
     CS = 0;
     Somenop50();Somenop();
     send_byte(cmd);           // 写指令
     Somenop25();Somenop();
     send_byte(dat);        // 写数据
     Somenop();Somenop();
     CS = 1;
}
	
	
void send_byte(unsigned char cmd)
{
      unsigned char i;
       Somenop50();Somenop50(); //时序初始延时
       for(i = 0;i < 8;i++)
       {       //写cmd的最高位，  
                if(cmd&0x80) // 最高位为1则写"1"
                        DATA = 1; 
                else   // 最高位为0写"0"
                        DATA = 0;
                // CLK形成下降沿,先1后0，注意延时时间要满足时序要求
                CLK = 1; Somenop10();Somenop10();
                CLK = 0; Somenop10();Somenop10();
                //命令行左移，为写下一个位做准备
                cmd = cmd << 1;       
        }
        DATA = 0;  //依照时序要求全部写完后Data清零 
}		  

unsigned char receive_byte(void)
{
      unsigned char i, in_byte;
      DATA=1;  
      Somenop50();Somenop50(); 
      for (i=0;i<8;i++)
      {
              CLK=1;Somenop10();Somenop10();
              //左移一位，空出最后一位存放新进来的位DATA 
              in_byte=in_byte*2;  
               //若新进来的位DATA为1，则in_byte的末位置1，否则不需要变
              if (DATA)  
                       in_byte=in_byte|0x01;
              CLK=0; Somenop10();Somenop10();
       }
       DATA=0;
       return (in_byte); //返回接收值
}

//////////////////////////////////////////////////////////////////////////

void display_string_in_row (char* string, unsigned char upper){
	unsigned char start_index;
	unsigned char i;
	if (upper == TRUE) start_index = 4;
	else start_index = 0;
	
	for (i = 0; i < 4; i++) {
		DispBuff[start_index+i] = check_LED(string[i]);
	}
	display(DispBuff);
}

void display_int_in_row(unsigned char i, unsigned char upper) {
	unsigned char quotient, rem;
	unsigned char current_index;
	if (upper == TRUE)	current_index = 7;
	else current_index = 3;
	display_string_in_row("    ", upper);
	if (i == 0) {
		DispBuff[current_index] = 0;
		display(DispBuff);
		return;
	}
	while (TRUE) {
		rem = i % 10;
		quotient = i / 10;
		if (rem == 0 && quotient == 0) break;
		DispBuff[current_index] = rem;	//print out remainder from right to left
		current_index--;
		i /= 10;
	}
	display(DispBuff);
}

unsigned char change_menu_ptr(unsigned char current, unsigned char inc, unsigned char volume){
	if (inc == TRUE) {
		if (current == volume - 1) return 0;
		else return ++current;
	}
	else {
		if (current == 0) return (volume - 1);
		else return --current;
	}
}

void show_PA_menu(void) {
	char* menu_buffer[] = {" run", " Con", " P1d"};
	unsigned char current_menu = 0;
	display_string_in_row(" run ", FALSE);
	while(1) {
		display(DispBuff); //显示（按显缓单元的内容显示）
		Key();
		if(KeyValue != 0xff) { 
			if (KeyNum == DOWN) {
				current_menu = change_menu_ptr(current_menu, FALSE, 3);
				display_string_in_row(menu_buffer[current_menu], FALSE);
			}
			else if (KeyNum == UP) {
				current_menu = change_menu_ptr(current_menu, TRUE, 3);
				display_string_in_row(menu_buffer[current_menu], FALSE);
			}
			else if (KeyNum == ENTER) {
				switch(current_menu) {
					case 0: show_current_run();
							display_string_in_row("PA- ", TRUE);
							display_string_in_row(" run", FALSE);
							break;
					case 1: show_current_temp_threshold(); 
							display_string_in_row("PA- ", TRUE);
							display_string_in_row(" Con", FALSE);
							break;
					case 2: show_current_PID_goal_temp();
							display_string_in_row("PA- ", TRUE);
							display_string_in_row(" P1d", FALSE);
							break;
				}
			}
			else if (KeyNum == BACK) {
				return;
			}
		}
	}
}

void show_current_run(void) {
	unsigned char current_menu = 0;

	read_run_options_from_C16();
	display_string_in_row("P- 0", TRUE);
	display_int_in_row(run_options[0], FALSE);

	while (1) {
		// display(DispBuff); //显示（按显缓单元的内容显示）
		Key();
		if (KeyValue != 0xff)
		{ 
			if (KeyNum == DOWN) {
				current_menu = change_menu_ptr(current_menu, FALSE, 10);
				DispBuff[7] = current_menu;
				display_int_in_row(run_options[current_menu], FALSE);
				// display_string_in_row(menu_buffer[current_menu], FALSE);
			}
			else if (KeyNum == UP) {
				current_menu = change_menu_ptr(current_menu, TRUE, 10);
				DispBuff[7] = current_menu;
				display_int_in_row(run_options[current_menu], FALSE);
				// display_string_in_row(menu_buffer[current_menu], FALSE);
			}
			else if (KeyNum == ENTER) {
				DispBuff[4] = 10;
				// run_options[current_menu] = change_num_conti(run_options[current_menu]);
				change_num_conti(&run_options[current_menu]);
				write_run_options_to_C16();
				wait_until_release();
				DispBuff[4] = 34; // "P"
				display(DispBuff);
			}
			if (KeyNum == BACK){
				write_run_options_to_C16();
				return;
			}
		}
	}
}

void show_current_temp_threshold(void) {
	unsigned char current_menu = 0;
	char* menu_buffer[2] = {"PA-b", "PA-F"};

	read_temp_threshold_from_C16();
	display_string_in_row(menu_buffer[current_menu], TRUE);
	display_int_in_row(temp_threshold[current_menu], FALSE);

	while (1) {
		Key();
		if (KeyValue != 0xff) { 
			if (KeyNum == DOWN) {
				current_menu = change_menu_ptr(current_menu, FALSE, 2);
				display_string_in_row(menu_buffer[current_menu], TRUE);
				display_int_in_row(temp_threshold[current_menu], FALSE);
			}
			else if (KeyNum == UP) {
				current_menu = change_menu_ptr(current_menu, TRUE, 2);
				display_string_in_row(menu_buffer[current_menu], TRUE);
				display_int_in_row(temp_threshold[current_menu], FALSE);
			}
			
			else if (KeyNum == ENTER) {
				DispBuff[4] = 10;
				DispBuff[5] = 39;	// "-"
				DispBuff[6] = 32;	// " "
				// temp_threshold[current_menu] = change_num_conti(temp_threshold[current_menu]);
				change_num_conti(&temp_threshold[current_menu]);
				write_temp_threshold_to_C16();
				DispBuff[4] = 34; // "P"
				DispBuff[5] = 10;
				DispBuff[6] = 39;
				display(DispBuff);
			}
			if (KeyNum == BACK){
				write_temp_threshold_to_C16();
				return;
			}
		}
	}
}


void change_num_conti(unsigned char *num) {
	unsigned char original = *num;

	display(DispBuff);
	wait_until_release();
	while (TRUE) {
		Key();
		switch (KeyNum) {
			case UP: (*num) = change_menu_ptr(*num, TRUE, 100); 
					display_int_in_row(*num, FALSE);
					break;
			case DOWN: (*num) = change_menu_ptr(*num, FALSE, 100); 
					display_int_in_row(*num, FALSE);
					break;
			case BACK: (*num) = original;
					display_int_in_row(*num, FALSE);
					return;
			case ENTER: return;
			default: break;
		}
	}
}

void wait_until_release(void) {
	while (KeyNum != 0xff || KeyValue != 0xff) {
		Key();
	}
	return;
}



void show_temperature(unsigned char upper) {
	unsigned char i, count;
	count = 0;
	InitUART();
	while(1) {
		Key();
		if (KeyNum == BACK) return;
		DS1820_Reset();
		DS1820_WriteData(0xcc);
		DS1820_WriteData(0x44);

		DS1820_Reset();
		DS1820_WriteData(0xcc);
		DS1820_WriteData(0xbe);

		for (i = 0; i < 2; i++) {
			temperature[i] = DS1820_ReadData();
		}
		DS1820_Reset();

		display_temperature(upper);
		count++;
		if (count == 100) {
			send_temp_to_computer();
			count = 0;
		}
		Somenop50();
		if(TEMP_CONTROL_MODE) break;
	}
}

void display_temperature(unsigned char upper) {
	unsigned char start_address = 0;
	U8 temp_data,temp_data_2 ;
	U16 TempDec	;

	if (upper) start_address = 4;
	temp_data = temperature[1];
	temp_data &= 0xf0; //取高4 位
	if (temp_data == 0xf0) { //判断是正温度还是负温度读数
		DispBuff[start_address + 0] = 39;//负温度读数求补,取反加1,判断低8 位是否有进位
		if (temperature[0] == 0){
	 //有进位,高8 位取反加1
		temperature[0] = ~temperature[0] + 1;
		temperature[1] = ~temperature[1] + 1;
		} else {
		 //没进位,高8 位不加1
		temperature[0] = ~temperature[0] + 1;
		temperature[1] = ~temperature[1];
		}
	}

	temp_data = (temperature[1] & 0x07) << 4;  //取高字节低4位(温度读数高4位)，注意此时是12位精度
	temp_data_2 = temperature[0] >> 4; //取低字节高4位(温度读数低4位)，注意此时是12位精度
	temp_data = temp_data | temp_data_2; //组合成完整数据
	temp[0] = temp_data / 100;  //取百位转换为ASCII码
	temp[1] = (temp_data % 100) / 10; //取十位转换为ASCII码
	temp[2] = (temp_data % 100) % 10; //取个位转 换为ASCII码
	temperature[0] &= 0x0f;  //取小数位转换为ASCII码
	TempDec = temperature[0] * 625;  //625=0.0625* 10000,  表示小数部分，扩大1万倍，方便显示
	temp[3] = TempDec / 1000;  //取小数十分位转换为ASCII码
	temp[4] = (TempDec % 1000) / 100;  //取小数百分位转换为ASCII码
	if(DispBuff[start_address + 0] == 39){
		if(temp[0] != 0) {
			DispBuff[start_address+1] = temp[0];
			DispBuff[start_address+2] = temp[1];
			DispBuff[start_address+3] = temp[2];
		} else {
			DispBuff[start_address+1] = temp[1];
			DispBuff[start_address+2] = temp[2] + 16;
			DispBuff[start_address+3] = temp[3];
		}
	} else {
		if(temp[0] != 0) {
			DispBuff[start_address+0] = temp[0];
			DispBuff[start_address+1] = temp[1];
			DispBuff[start_address+2] = temp[2] + 16;
			DispBuff[start_address+3] = temp[3];
		} else {
			DispBuff[start_address+0] = temp[1];
			DispBuff[start_address+1] = temp[2] + 16;
			DispBuff[start_address+2] = temp[3];
			DispBuff[start_address+3] = temp[4];
		}
	}
	display(DispBuff);
}

void show_motor_test(void) {
	unsigned char current_menu = 0;
	TEMP_CONTROL_MODE = FALSE;
	read_run_options_from_C16();
	display_string_in_row("r- 0", TRUE);
	display_int_in_row(run_options[0], FALSE);

	while (1) {
		// display(DispBuff); //显示（按显缓单元的内容显示）
		Key();
		if (KeyValue != 0xff) { 
			if (KeyNum == DOWN) {
				current_menu = change_menu_ptr(current_menu, FALSE, 10);
				DispBuff[7] = current_menu;
				display_int_in_row(run_options[current_menu], FALSE);
			} else if (KeyNum == UP) {
				current_menu = change_menu_ptr(current_menu, TRUE, 10);
				DispBuff[7] = current_menu;
				display_int_in_row(run_options[current_menu], FALSE);
			} else if (KeyNum == ENTER) {
				display_string_in_row("run-", TRUE);
				PWM = run_options[current_menu];
				run_motor_with_PWM();
				DispBuff[4] = 35;	// r
				DispBuff[5] = 39;	// -
				DispBuff[6] = 32;	// space
				DispBuff[7] = current_menu;
				display_int_in_row(run_options[current_menu], FALSE);
				wait_until_release();
			} else if (KeyNum == BACK) {
				DispBuff[4] = 35;	// r
				DispBuff[5] = 36;	// -
				DispBuff[6] = 37;	// space
				DispBuff[7] = 39;
				return;
			}
		}
	}
}


void run_motor_with_PWM() {
	unsigned char i;

	motor_thre = PWM;

	while(1) {
		if (TEMP_CONTROL_MODE) {
			for (i = 0; i < 10; i++) {
				TR0 = 1;
				while (TR0 == 1);
			}
			break;
		} else {
			TR0 = 1;
			Key();
			if (KeyNum == BACK) {
				TR0 = 0;
				Motor = 0;
				return;
			}
		}
	}

}

void timer0(void) interrupt 1 using 3 {
	TH0 = timerH;
	TL0 = timerL;

	if (motor_now < motor_thre) {
		Motor = 1;
		motor_now++;
		P2 = 0x00;
	}
	else if (motor_now < 100) {
		Motor = 0;
		motor_now++;
		P2 = 0x02;
	}
	else {
		motor_now = 0;
		if (TEMP_CONTROL_MODE)	TR0 = 0;
	}
	
}

void con_with_temp(unsigned char usingPID) {
	unsigned int i = 0;
	TEMP_CONTROL_MODE = TRUE;
	PIDInit();
	while(1) {
		Key();
		if (KeyNum == BACK){
			TEMP_CONTROL_MODE = FALSE;
			return;
		}
		show_temperature(TRUE);
		if (i % 6 == 0)	display_int_in_row(PWM, FALSE);
		Key();
		if (KeyNum == BACK) {
			TEMP_CONTROL_MODE = FALSE;
			return;
		}
		if (usingPID)	calc_PWM_PID();
		else	calc_current_PWM();
		Key();
		if (KeyNum == BACK){
			TEMP_CONTROL_MODE = FALSE;
			return;
		}
		run_motor_with_PWM();
		Key();
		if (KeyNum == BACK){
			TEMP_CONTROL_MODE = FALSE;
			return;
		}
		i++;

	}
}

void calc_current_PWM(){
	// Now we have temp[]. We have to change PWM.
	unsigned char current_temp = 10 * temp[1] + temp[2];
	unsigned int nominator, denom;

	read_temp_threshold_from_C16();
	nominator = (temp[1] * 1000 + temp[2] * 100 + temp[3] * 10 + temp[4]) - temp_threshold[0]* 100;
	denom = 2 * (temp_threshold[1] - temp_threshold[0]);
	if (current_temp >= temp_threshold[1]) {
		PWM = 100; return;
	}
	else if (current_temp < temp_threshold[0]) {
		PWM = 0; 
		return;
	}
	PWM = nominator / denom + 50;
}

void estart() //起始信号 当时钟线为1，数据线有个下降沿
{   
    ECLK=1;
    EDTA=1;
    Somenop();
    EDTA=0; 
    ECLK=0;
    Somenop();
}

void estop()//终止信号 当时钟线为1，数据线有个上升沿
{
    EDTA=0;
    ECLK=1;
    Somenop();
    EDTA=1;
    ECLK=0;
    Somenop();
}

bit ack() //应答信号由从机发出信号为sda由1变为0
{
    
    ECLK=1;
    EDTA=1;
    if(EDTA==1){
        ECLK=0;
        return 1;
    }else{
        ECLK=0;
        return 0;
    }
    
}

void init_24c16()//24c16初始化
{
    EDTA=1;
    Somenop();
    ECLK=1;
    Somenop();
}

void ewrite_byte(unsigned char dat) //字节写（写数据或地址）数据线sda不变，scl有个上升沿，写入数据
{
    unsigned char i;
    for(i=0;i<8;i++)
    {
        ECLK=0;
        Somenop();
        EDTA=dat&0x80;
        Somenop();
        ECLK=1;
        Somenop();
        dat <<= 1;
    }
    ECLK=0;
    Somenop();
}

unsigned char eread_byte() //字节读 scl有下降沿读出
{
    unsigned char i,k;
    for(i=0;i<8;i++)
    {
    ECLK=1;
    Somenop();
    k=(k<<1)|EDTA;
    ECLK=0;
    Somenop();
    }
    return k;
}

void ewrite_add(unsigned char add,unsigned char dat)
{
    do{
    estart();
    ewrite_byte(0xa0);
    }
    while(ack());
    
    ewrite_byte(add);
    ack();
    do{
    ewrite_byte(dat);
    }
    while(ack());
    estop();
}

unsigned char eread_add(unsigned char add)
{
    unsigned char dat;

    do{
    estart();
    ewrite_byte(0xa0);
    }
    while(ack());
    
    ewrite_byte(add);
    ack();

    
    do{
    estart();
    ewrite_byte(0xa1);
    }
    while(ack());

    dat=eread_byte();
    estop();
    return dat;
}

void read_run_options_from_C16() {
	unsigned char i;
	for(i = 0; i < 10; i++) {
		run_options[i] = eread_add(run_options_start_address+i);
	}
}

void write_run_options_to_C16() {
	unsigned char i;
	for(i = 0; i < 10; i++) {
		ewrite_add(run_options_start_address+i, run_options[i]);
	}
}

void read_temp_threshold_from_C16() {
	unsigned char i;
	for(i = 0; i < 2; i++) {
		temp_threshold[i] = eread_add(temp_threshold_start_address+i);
	}
}

void write_temp_threshold_to_C16() {
	unsigned char i;
	for(i = 0; i < 2; i++) {
		ewrite_add(temp_threshold_start_address+i, temp_threshold[i]);
	}
}

void show_current_PID_goal_temp() {
	unsigned char i;
	unsigned char flag = check_pwd();
	if (flag == FALSE)	return;
	display_string_in_row("A- P", TRUE);
	read_PID_goal_temp_from_C16();
	DispBuff[0] = 32;
	DispBuff[1] = pid.goal_temp[0];
	DispBuff[2] = pid.goal_temp[1];
	DispBuff[3] = pid.goal_temp[2];
	display(DispBuff);

	flag = FALSE;
	for(i = 1; i < 4; i++){
		while(TRUE) {
			Key();
			switch (KeyNum) {
				case BACK: return;
				case ENTER: flag = TRUE; 
							pid.goal_temp[i-1] = DispBuff[i];
							break;
				case UP:
					if (i != 2) {
						DispBuff[i] = change_menu_ptr(DispBuff[i], TRUE, 10); 
					}
					else {
						DispBuff[i] = change_menu_ptr(DispBuff[i]-16, TRUE, 10);
						DispBuff[i] += 16;
					}
					display(DispBuff);
					break;
				case DOWN:
					if (i != 2){
						DispBuff[i] = change_menu_ptr(DispBuff[i], FALSE, 10); 
					} else {
						DispBuff[i] = change_menu_ptr(DispBuff[i]-16, FALSE, 10);
						DispBuff[i] += 16;
					}
					display(DispBuff);
					break;
			}
			if (flag) {
				flag = FALSE;
				break;
			}
		}
		write_PID_goal_temp_to_C16();
	}
}

void read_PID_goal_temp_from_C16() {
	unsigned char i;
	for(i = 0; i < 3; i++) {
		pid.goal_temp[i] = eread_add(PIDGoalTempAddress+i);
		if (pid.goal_temp[i] >= 10 && i != 1)	pid.goal_temp[i] = 0;
	}
}

void write_PID_goal_temp_to_C16() {
	unsigned char i;
	for(i = 0; i < 3; i++) {
		ewrite_add(PIDGoalTempAddress+i, pid.goal_temp[i]);
	}
}

unsigned char check_pwd() {
	unsigned char i;
	unsigned char flag = FALSE;
	display_string_in_row("PA55", TRUE);
	display_string_in_row("0000", FALSE);
	
	for(i = 0; i < 4; i++) {
		while(TRUE) {
			Key();
			switch (KeyNum) {
				case BACK: return FALSE;
				case UP: DispBuff[i] = change_menu_ptr(DispBuff[i], TRUE, 10);
						 	display(DispBuff); 
						 	break;
				case DOWN: DispBuff[i] = change_menu_ptr(DispBuff[i], FALSE, 10); 
							display(DispBuff); 
							break;
				case ENTER: flag=TRUE; 
							break;
			}
			if (flag) {
				flag = FALSE;
				break;
			}
		}
	}
	for(i = 0; i < 4; i++) {
		if (DispBuff[i] != PIDPwd[i])	return FALSE;
	}
	return TRUE;
}

void PIDInit() {
	pid.err_last = 0;
	pid.integral = 0;
}

void calc_PWM_PID() {
	unsigned char i;
	float current_tempFloat, goalTempFloat;
	read_PID_goal_temp_from_C16();
	// PIDInit();
	goalTempFloat = pid.goal_temp[0] * 10 + (pid.goal_temp[1]-16) + pid.goal_temp[2] * 0.1;
	pid.Kp = 50;
	pid.Ki = 0;	// 下次先试试看Ki=0会发生什么 也许能验证对错
	pid.Kd = 20;

	for (i = 0; i < 3; i++) {
		pid.current_temp[i] = temp[i+1];
	}
	current_tempFloat = pid.current_temp[0] * 10 + pid.current_temp[1] + pid.current_temp[2] * 0.1;

	pid.err = current_tempFloat - goalTempFloat;

	if (pid.err < 0) {
		PWM = 0;
		return;
	}
	pid.integral += pid.err;
	pid.current_PWM = pid.Kp * pid.err + pid.Ki * pid.integral + pid.Kd * (pid.err - pid.err_last) + 50;
	pid.err_last = pid.err;
	if (pid.current_PWM >= 100)	pid.current_PWM = 100;
	else if (pid.current_PWM <= 0) pid.current_PWM = 50;

	PWM = pid.current_PWM;
}

void UART_ISR(void) interrupt 4
{
    U8 RX_Data;
    //只响应"接收"中断，"发送"中断来了就直接抹掉
    if(RI)
   {
     RI = 0;	//串口中断标志不能自己清除，需要手动清除
     RX_Data=SBUF;
     SendOneByte(RX_Data);
   }
   else
     TI = 0;		//串口发中断是发送完缓冲区数据之后产生
}
 
/****************串口初始化函数*************/
void InitUART(void)
{
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
void SendOneByte(U8 c)
{
    ES = 0;			//禁止中断，让串口安心工作啊
    SBUF = c;
    while(!TI);		//等待发送完毕
    TI = 0;			//清TI中断
    ES = 1;			//打开中断
}

void send_temp_to_computer(){
	SendOneByte(temp[1] * 10 + temp[2]);
	SendOneByte(temp[3] * 10 + temp[4]);
}

void main (void) {
  	Init_7279();	// 初始化堆栈    			// 初始化7279
  	DS18B20_Init();
	Motor = 0;
  	display_string_in_row("tP- ", TRUE);
	TMOD = 0x01;
	TH0 = timerH;
	TL0 = timerL;
	EA = 1;
	ET0 = 1;
 	show_main_menu();
}