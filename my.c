#include <REG52.H>                
#include "my.h"

#include <stdio.h>                
#include <math.h>
#include <intrins.h>
#include <string.h>

unsigned char LEDValue[50] = {0xFC,0x44,0x79,0x5D,0xC5,0x9D,0xBD,0x54,0xFD,0xDD,0xF5,0xAD,0xB8,0x6D,0xB9,0xB1, //0-F
  							0xFE,0x46,0x7B,0x5F,0xC7,0x9F,0xBF,0x56,0xFF,0xDF,0xF7,0xAF,0xBA,0x6F,0xBB,0xB3, //0.-F.
	                        0x00,0xA9,0xF1,0x21,0x2C,0x25,0x2D,0x01}; // 32null，33t，34P,35r,36u,37n,38o,39-
  
unsigned char DispBuff[8] = {32,32,32,32,32,32,32,32};
unsigned char code KeyTabel[4] = {0x3B, 0x3A, 0x39, 0x38}; // down up back enter

unsigned char runOptions[10];
unsigned char runOptionsStartAddress = 0;

unsigned char tempThreshold[2];
unsigned char tempThresholdStartAddress = 10;

unsigned char temperature[2];

unsigned char MotorThre;
unsigned char MotorNow = 0;

unsigned char timerH = 0xFF;
unsigned char timerL = 0x00;

unsigned char counter = 0;

unsigned char temp[5]; // bai shi ge shifen baifen

unsigned char PWM = 50;

unsigned char TEMPCONTROLMODE = FALSE;

unsigned char KeyFlag = 0;

unsigned char PIDPwd[4] = {0,0,0,0};
// unsigned char PIDParam[3];
unsigned char PIDGoalTempAddress = 12;

struct _pid{
	unsigned char goalTemp[3];
	unsigned char currentTemp[3];
	unsigned char Kp, Ki, Kd;
	float currentPWM;
	float err, errLast;
	float integral;
}pid;

unsigned char checkLED (char c){
	if (c >= '0' && c <= '9') return (c-'0');
	else if (c >= 'A' && c <= 'F') return (c-'A'+10);
	switch (c){
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

void showMainMenu(void){
	char* menuBuffer[] = {"tP- ", "run-", "Con-", "PA- ", "P1d-"};
	unsigned char currentMenu = 0;
	while(1){
		display(DispBuff); //显示（按显缓单元的内容显示）
		Key();
		if(KeyValue!=0xff)
		{   
		// 有键按下，按照键号执行菜单显示或进入子菜单
			if (KeyNum==DOWN){
				currentMenu = changeMenuPtr(currentMenu, FALSE, 5);
				displayStringInRow(menuBuffer[currentMenu], TRUE);
			}
			else if (KeyNum==UP){
				currentMenu = changeMenuPtr(currentMenu, TRUE, 5);
				displayStringInRow(menuBuffer[currentMenu], TRUE);
			}
			else if (KeyNum==ENTER){
				switch(currentMenu){
					case 0: showTemperature(FALSE);break;
					case 1: showMotorTest();break;
					case 2:	conWithTemp(FALSE);displayStringInRow("Con-", TRUE);Motor=0;break;
					case 3: showPAMenu();break;
					case 4: conWithTemp(TRUE);displayStringInRow("Con-", TRUE);Motor=0;break;
				}
				displayStringInRow("    ", FALSE);
			}
		}
  }
}

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

void displayStringInRow (char* string, unsigned char upper){
	unsigned char startIndex;
	unsigned char i;
	if (upper == TRUE) startIndex = 4;
	else startIndex = 0;
	
	for (i = 0; i < 4; i++){
		DispBuff[startIndex+i] = checkLED(string[i]);
	}
	display(DispBuff);
}

void displayIntInRow(unsigned char i, unsigned char upper){
	unsigned char shang, yu;
	unsigned char currentIndex;
	if (upper == TRUE)	currentIndex = 7;
	else currentIndex = 3;
	displayStringInRow("    ", upper);
	if (i==0){
		DispBuff[currentIndex] = 0;
		display(DispBuff);
		return;
	}
	while (TRUE){
		yu = i % 10;
		shang = i / 10;
		if (yu == 0 && shang == 0) break;
		DispBuff[currentIndex] = yu;	//print out yu from right to left
		currentIndex--;
		i /= 10;
	}
	display(DispBuff);
}

unsigned char changeMenuPtr(unsigned char current, unsigned char inc, unsigned char volume){
	if (inc == TRUE){
		if (current == volume - 1) return 0;
		else return ++current;
	}
	else{
		if (current == 0) return (volume - 1);
		else return --current;
	}
}

void showPAMenu(void){
	char* menuBuffer[] = {" run", " Con", " P1d"};
	unsigned char currentMenu = 0;
	displayStringInRow(" run ", FALSE);
	while(1){
		display(DispBuff); //显示（按显缓单元的内容显示）
		Key();
		if(KeyValue!=0xff)
		{ 
			if (KeyNum==DOWN){
				currentMenu = changeMenuPtr(currentMenu, FALSE, 3);
				displayStringInRow(menuBuffer[currentMenu], FALSE);
			}
			else if (KeyNum==UP){
				currentMenu = changeMenuPtr(currentMenu, TRUE, 3);
				displayStringInRow(menuBuffer[currentMenu], FALSE);
			}
			else if (KeyNum==ENTER){
				switch(currentMenu){
					case 0: showCurrentRun();displayStringInRow("PA- ", TRUE);displayStringInRow(" run", FALSE); break;
					case 1: showCurrentTmpThreshould(); displayStringInRow("PA- ", TRUE);displayStringInRow(" Con", FALSE);break;
					case 2: showCurrentPIDGoalTemp(); displayStringInRow("PA- ", TRUE);displayStringInRow(" P1d", FALSE);break;
				}
			}
			else if (KeyNum==BACK){
				return;
			}
		}
	}
}

void showCurrentRun(void){
	unsigned char currentMenu = 0;

	readRunOptionsFromC16();
	displayStringInRow("P- 0", TRUE);
	displayIntInRow(runOptions[0], FALSE);

	while (1){
		// display(DispBuff); //显示（按显缓单元的内容显示）
		Key();
		if(KeyValue!=0xff)
		{ 
			if (KeyNum==DOWN){
				currentMenu = changeMenuPtr(currentMenu, FALSE, 10);
				DispBuff[7] = currentMenu;
				displayIntInRow(runOptions[currentMenu], FALSE);
				// displayStringInRow(menuBuffer[currentMenu], FALSE);
			}
			else if (KeyNum==UP){
				currentMenu = changeMenuPtr(currentMenu, TRUE, 10);
				DispBuff[7] = currentMenu;
				displayIntInRow(runOptions[currentMenu], FALSE);
				// displayStringInRow(menuBuffer[currentMenu], FALSE);
			}
			else if (KeyNum==ENTER){
				DispBuff[4] = 10;
				// runOptions[currentMenu] = changeNumConti(runOptions[currentMenu]);
				changeNumConti(&runOptions[currentMenu]);
				writeRunOptionsToC16();
				waitUntilRelease();
				DispBuff[4] = 34; // "P"
				display(DispBuff);
			}
			if (KeyNum==BACK){
				writeRunOptionsToC16();
				return;
			}
		}
	}
}

void showCurrentTmpThreshould(void){
	unsigned char currentMenu = 0;
	char* menuBuffer[2] = {"PA-b", "PA-F"};

	readTempThresholdFromC16();
	displayStringInRow(menuBuffer[currentMenu], TRUE);
	displayIntInRow(tempThreshold[currentMenu], FALSE);

	while (1){
		Key();
		if(KeyValue!=0xff)
		{ 
			if (KeyNum==DOWN){
				currentMenu = changeMenuPtr(currentMenu, FALSE, 2);
				displayStringInRow(menuBuffer[currentMenu], TRUE);
				displayIntInRow(tempThreshold[currentMenu], FALSE);
			}
			else if (KeyNum==UP){
				currentMenu = changeMenuPtr(currentMenu, TRUE, 2);
				displayStringInRow(menuBuffer[currentMenu], TRUE);
				displayIntInRow(tempThreshold[currentMenu], FALSE);
			}
			
			else if (KeyNum==ENTER){
				DispBuff[4] = 10;
				DispBuff[5] = 39;	// "-"
				DispBuff[6] = 32;	// " "
				// tempThreshold[currentMenu] = changeNumConti(tempThreshold[currentMenu]);
				changeNumConti(&tempThreshold[currentMenu]);
				writeTempThresholdToC16();
				DispBuff[4] = 34; // "P"
				DispBuff[5] = 10;
				DispBuff[6] = 39;
				display(DispBuff);
			}
			if (KeyNum==BACK){
				writeTempThresholdToC16();
				return;
			}
		}
	}
}


void changeNumConti(unsigned char *num){
	unsigned char original = *num;

	display(DispBuff);
	waitUntilRelease();
	while (TRUE){
		Key();
		switch (KeyNum){
			case UP: (*num) = changeMenuPtr(*num, TRUE, 100); displayIntInRow(*num, FALSE);break;
			case DOWN: (*num) = changeMenuPtr(*num, FALSE, 100); displayIntInRow(*num, FALSE);break;
			case BACK: (*num) = original;displayIntInRow(*num, FALSE);return;
			case ENTER: return;
			default: break;
		}
	}
}

void waitUntilRelease(void){
	while (KeyNum!=0xff || KeyValue!=0xff){	Key();}
	return;
}



void showTemperature(unsigned char upper){
	unsigned char i;
	while(1){
		Key();
		if (KeyNum == BACK) return;
		DS1820_Reset();
		DS1820_WriteData(0xcc);
		DS1820_WriteData(0x44);

		DS1820_Reset();
		DS1820_WriteData(0xcc);
		DS1820_WriteData(0xbe);

		for (i=0;i<2;i++){
			temperature[i] = DS1820_ReadData();
		}
		DS1820_Reset();

		displayTemperature(upper);
		Somenop50();
		if(TEMPCONTROLMODE) break;
	}
}

void displayTemperature(unsigned char upper){
	unsigned char startAddress = 0;
	U8 temp_data,temp_data_2 ;
	U16 TempDec	;

	if (upper) startAddress = 4;
	temp_data = temperature[1];
	temp_data &= 0xf0; //取高4 位
	if (temp_data==0xf0){ //判断是正温度还是负温度读数
		DispBuff[startAddress+0]=39;//负温度读数求补,取反加1,判断低8 位是否有进位
		if (temperature[0]==0){
	 //有进位,高8 位取反加1
		temperature[0]=~temperature[0]+1;
		temperature[1]=~temperature[1]+1;
		}
		else{
		 //没进位,高8 位不加1
		temperature[0]=~temperature[0]+1;
		temperature[1]=~temperature[1];
		}
	}

	temp_data = (temperature[1]&0x07)<<4;  //取高字节低4位(温度读数高4位)，注意此时是12位精度
	temp_data_2 = temperature[0]>>4; //取低字节高4位(温度读数低4位)，注意此时是12位精度
	temp_data = temp_data | temp_data_2; //组合成完整数据
	temp[0] = temp_data/100;  //取百位转换为ASCII码
	temp[1] = (temp_data%100)/10; //取十位转换为ASCII码
	temp[2] = (temp_data%100)%10; //取个位转 换为ASCII码
	temperature[0]&=0x0f;  //取小数位转换为ASCII码
	TempDec = temperature[0]*625;  //625=0.0625* 10000,  表示小数部分，扩大1万倍，方便显示
	temp[3] = TempDec/1000;  //取小数十分位转换为ASCII码
	temp[4] = (TempDec%1000)/100;  //取小数百分位转换为ASCII码
	if(DispBuff[startAddress+0]==39){
		if(temp[0]!=0){
			DispBuff[startAddress+1]=temp[0];
			DispBuff[startAddress+2]=temp[1];
			DispBuff[startAddress+3]=temp[2];
		}
		else {
			DispBuff[startAddress+1]=temp[1];
			DispBuff[startAddress+2]=temp[2]+16;
			DispBuff[startAddress+3]=temp[3];
		}
	}
	else{
		if(temp[0]!=0){
			DispBuff[startAddress+0]=temp[0];
			DispBuff[startAddress+1]=temp[1];
			DispBuff[startAddress+2]=temp[2]+16;
			DispBuff[startAddress+3]=temp[3];
		}
		else {
			DispBuff[startAddress+0]=temp[1];
			DispBuff[startAddress+1]=temp[2]+16;
			DispBuff[startAddress+2]=temp[3];
			DispBuff[startAddress+3]=temp[4];
		}
	}
	display(DispBuff);
}

void showMotorTest(void){
	unsigned char currentMenu = 0;
	TEMPCONTROLMODE = FALSE;
	readRunOptionsFromC16();
	displayStringInRow("r- 0", TRUE);
	displayIntInRow(runOptions[0], FALSE);

	while (1){
		// display(DispBuff); //显示（按显缓单元的内容显示）
		Key();
		if(KeyValue!=0xff)
		{ 
			if (KeyNum==DOWN){
				currentMenu = changeMenuPtr(currentMenu, FALSE, 10);
				DispBuff[7] = currentMenu;
				displayIntInRow(runOptions[currentMenu], FALSE);
				// displayStringInRow(menuBuffer[currentMenu], FALSE);
			}
			else if (KeyNum==UP){
				currentMenu = changeMenuPtr(currentMenu, TRUE, 10);
				DispBuff[7] = currentMenu;
				displayIntInRow(runOptions[currentMenu], FALSE);
				// displayStringInRow(menuBuffer[currentMenu], FALSE);
			}
			else if (KeyNum==ENTER){
				displayStringInRow("run-", TRUE);
				PWM = runOptions[currentMenu];
				runMotorWithPWM();
				DispBuff[4] = 35;	// r
				DispBuff[5] = 39;	// -
				DispBuff[6] = 32;	// space
				DispBuff[7] = currentMenu;
				displayIntInRow(runOptions[currentMenu], FALSE);
				waitUntilRelease();
			}
			else if (KeyNum==BACK){
				DispBuff[4] = 35;	// r
				DispBuff[5] = 36;	// -
				DispBuff[6] = 37;	// space
				DispBuff[7] = 39;
				return;
			}
		}
	}
}


void runMotorWithPWM(){
	unsigned char i;

	MotorThre = PWM;

	// displayIntInRow(PWM, FALSE);
	while(1){
		if (TEMPCONTROLMODE) {
			for (i=0; i<10; i++){
				TR0 = 1;
				while (TR0 == 1);
			}
			break;
		}
		else{
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

void timer0(void) interrupt 1 using 3{
	TH0 = timerH;
	TL0 = timerL;

	if (MotorNow < MotorThre){
		Motor = 1;
		MotorNow++;
		P2 = 0x00;
	}
	else if (MotorNow < 100) {
		Motor = 0;
		MotorNow++;
		P2 = 0x02;
	}
	else {
		MotorNow = 0;
		if (TEMPCONTROLMODE)	TR0 = 0;
	}
	
}

	unsigned int i = 0;
void conWithTemp(unsigned char usingPID){
	TEMPCONTROLMODE = TRUE;
	PIDInit();
	while(1){
		Key();
		if (KeyNum == BACK){
			TEMPCONTROLMODE = FALSE;
			return;
		}
		showTemperature(TRUE);
		if (i%6==0)	displayIntInRow(PWM, FALSE);
		Key();
		if (KeyNum == BACK){
			TEMPCONTROLMODE = FALSE;
			return;
		}
		if (usingPID)	calcPWMPID();
		else	calcCurrentPWM();
		Key();
		if (KeyNum == BACK){
			TEMPCONTROLMODE = FALSE;
			return;
		}
		runMotorWithPWM();
		Key();
		if (KeyNum == BACK){
			TEMPCONTROLMODE = FALSE;
			return;
		}
		i++;

	}
}

void calcCurrentPWM(){
	// Now we have temp[]. We have to change PWM.
	unsigned char currentTemp = 10*temp[1]+temp[2];
	unsigned int nominator, denom;

	readTempThresholdFromC16();
	nominator = (temp[1]*1000+temp[2]*100+temp[3]*10+temp[4]) - tempThreshold[0]*100;
	denom = 2 * (tempThreshold[1] - tempThreshold[0]);
	if (currentTemp >= tempThreshold[1]){PWM = 100; return;}
	else if (currentTemp < tempThreshold[0]) {PWM = 0; return;}

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

void readRunOptionsFromC16(){
	unsigned char i;
	for(i=0; i<10; i++){
		runOptions[i] = eread_add(runOptionsStartAddress+i);
	}
}

void writeRunOptionsToC16(){
	unsigned char i;
	for(i=0; i<10; i++){
		ewrite_add(runOptionsStartAddress+i, runOptions[i]);
	}
}

void readTempThresholdFromC16(){
	unsigned char i;
	for(i=0; i<2; i++){
		tempThreshold[i] = eread_add(tempThresholdStartAddress+i);
	}
}

void writeTempThresholdToC16(){
	unsigned char i;
	for(i=0; i<2; i++){
		ewrite_add(tempThresholdStartAddress+i, tempThreshold[i]);
	}
}

void showCurrentPIDGoalTemp(){
	unsigned char i;
	unsigned char flag = checkPwd();
	if (flag == FALSE)	return;
	displayStringInRow("A- P", TRUE);
	readPIDGoalTempFromC16();
	DispBuff[0] = 32;
	DispBuff[1] = pid.goalTemp[0];
	DispBuff[2] = pid.goalTemp[1];
	DispBuff[3] = pid.goalTemp[2];
	display(DispBuff);

	flag = FALSE;
	for(i = 1; i < 4; i++){
		while(TRUE){
			Key();
			switch (KeyNum){
				case BACK: return;
				case ENTER: flag=TRUE; pid.goalTemp[i-1] = DispBuff[i];break;
				// case UP: DispBuff[i]=changeMenuPtr(DispBuff[i], TRUE, 10); display(DispBuff);break;
				case UP:
					if (i!=2){
						DispBuff[i]=changeMenuPtr(DispBuff[i], TRUE, 10); 
					}
					else{
						DispBuff[i]=changeMenuPtr(DispBuff[i]-16, TRUE, 10);
						DispBuff[i] += 16;
					}
					display(DispBuff);
					break;
				case DOWN:
					if (i!=2){
						DispBuff[i]=changeMenuPtr(DispBuff[i], FALSE, 10); 
					}
					else{
						DispBuff[i]=changeMenuPtr(DispBuff[i]-16, FALSE, 10);
						DispBuff[i] += 16;
					}
					display(DispBuff);
					break;
			}
			// if (i==2)	DispBuff[i] += 16;
			if (flag){
				flag = FALSE;
				break;
			}
		}
		writePIDGoalTempToC16();
	}
}

void readPIDGoalTempFromC16(){
	unsigned char i;
	for(i=0; i<3; i++){
		pid.goalTemp[i] = eread_add(PIDGoalTempAddress+i);
		if (pid.goalTemp[i] >= 10 && i != 1)	pid.goalTemp[i] = 0;
	}
}

void writePIDGoalTempToC16(){
	unsigned char i;
	for(i=0; i<3; i++){
		ewrite_add(PIDGoalTempAddress+i, pid.goalTemp[i]);
	}
}

unsigned char checkPwd(){
	unsigned char i;
	unsigned char flag = FALSE;
	displayStringInRow("PA55", TRUE);
	displayStringInRow("0000", FALSE);
	
	for(i = 0; i < 4; i++){
		while(TRUE){
			Key();
			switch (KeyNum){
				case BACK: return FALSE;
				case UP: DispBuff[i]=changeMenuPtr(DispBuff[i], TRUE, 10); display(DispBuff); break;
				case DOWN: DispBuff[i]=changeMenuPtr(DispBuff[i], FALSE, 10); display(DispBuff); break;
				case ENTER: flag=TRUE; break;
			}
			if (flag){
				flag = FALSE;
				break;
			}
		}
	}
	for(i = 0; i < 4; i++){
		if (DispBuff[i] != PIDPwd[i])	return FALSE;
	}
	return TRUE;
}

void PIDInit(){
	pid.errLast = 0;
	pid.integral = 0;
}

void calcPWMPID(){
	unsigned char i;
	float currentTempFloat, goalTempFloat;
	readPIDGoalTempFromC16();
	// PIDInit();
	goalTempFloat = pid.goalTemp[0] * 10 + (pid.goalTemp[1]-16) + pid.goalTemp[2] * 0.1;
	pid.Kp = 50;
	pid.Ki = 0;	// 下次先试试看Ki=0会发生什么 也许能验证对错
	pid.Kd = 20;

	for (i=0; i<3; i++){
		pid.currentTemp[i] = temp[i+1];
	}
	currentTempFloat = pid.currentTemp[0] * 10 + pid.currentTemp[1] + pid.currentTemp[2] * 0.1;

	pid.err = currentTempFloat - goalTempFloat;
	// DispBuff[1] = (int)pid.err / 10;
	// DispBuff[2] = (int)pid.err % 10 + 16;
	// DispBuff[3] = (pid.err - DispBuff[1]*10 - DispBuff[2] - 16) * 10;
	// DispBuff[1] = pid.goalTemp[0];
	// DispBuff[2] = pid.goalTemp[1]-16;
	// DispBuff[3] = pid.goalTemp[2];
	// display(DispBuff);
	// return;
	// 现在0 和 100 在切换

	if (pid.err < 0){
		PWM = 0;
		return;
	}
	pid.integral += pid.err;
	pid.currentPWM = pid.Kp * pid.err + pid.Ki * pid.integral + pid.Kd * (pid.err - pid.errLast);
	pid.errLast = pid.err;
	if (pid.currentPWM >= 100)	pid.currentPWM = 100;

	PWM = pid.currentPWM;
}

void main (void){
  	Init_7279();	// 初始化堆栈    			// 初始化7279
  	DS18B20_Init();
	Motor = 0;
  	displayStringInRow("tP- ", TRUE);
	TMOD = 0x01;
	TH0 = timerH;
	TL0 = timerL;
	EA = 1;
	ET0 = 1;
 	showMainMenu();
}