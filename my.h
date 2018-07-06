#ifndef MYH
#define MYH

#define Somenop();       _nop_();_nop_();_nop_();_nop_();_nop_();
#define Somenop10();	 Somenop();Somenop();
#define Somenop25();	 Somenop();Somenop();Somenop();Somenop();Somenop();
#define Somenop50();	 Somenop25();Somenop25();
#define CMD_RESET 0xA4
#define PIDA 12 
#define PIDB 10
#define DOWN 0x00
#define UP 0x01
#define BACK 0x02
#define ENTER 0x03
#define UPPER_LINE 0x00
#define LOWER_LINE 0x01
#define DOUBLE_LINE 0x02
#define TRUE 0x01
#define FALSE 0x00
#define U8 unsigned char
#define U16 unsigned short


sbit CS  = P1^4;
sbit CLK  = P1^5;
sbit DATA = P1^7;
sbit DS1820_DQ = P1^3;
// extern DS1820_DQ
sbit Motor  = P1^2;
sbit ECLK  = P1^1;
sbit EDTA  =  P1^0;
// extern sbit CS;
// extern sbit CLK;
// extern sbit DATA;
// extern sbit DS1820_DQ;
// extern sbit Motor;
// extern sbit ECLK;
// extern sbit EDTA;


// KeyValue is the address, and KeyNum is the index
unsigned char KeyNum, KeyValue;
unsigned char TempCount ,RunCount ,MotorCount ,MotorNow, PACount;
unsigned char rNum, bFNwum, NowEPVal, LastVal;
int  NowEPVal1, LastVal1;
unsigned int  KeyCount1, KeyCount2;
unsigned char PAStep;
void Init_7279(void) ;
void send_byte (unsigned char) ;
void delay (unsigned char);
void write_7279 (unsigned char,unsigned char); 
void display (unsigned char buff[]);
void Key(void);
unsigned char receive_byte (void);
unsigned char ReadKey (void);
void displayStringInRow (char* string, unsigned char upper);
unsigned char changeMenuPtr(unsigned char current, unsigned char inc, unsigned char volume);

void showPAMenu(void);
void showCurrentRun(void);
void showCurrentTmpThreshould(void);
// unsigned char changeNumConti(unsigned char i);
void changeNumConti(unsigned char *num);
void waitUntilRelease(void);

void showTemperature(unsigned char);
void displayTemperature(unsigned char);

// sbit DS1820_DQ = P1^4;
void DS18B20_Init();
bit DS1820_Reset();
void DS1820_WriteData(U8 wData);
U8 DS1820_ReadData();

void showMotorTest(void);
void runMotorWithPWM();

void conWithTemp(void);
void calcCurrentPWM();

void readRunOptionsFromC16();
void writeRunOptionsToC16();
void readTempThresholdFromC16();
void writeTempThresholdToC16();

void showCurrentPIDGoalTemp();
unsigned char checkPwd();

void readPIDGoalTempFromC16();
void writePIDGoalTempToC16();

#endif