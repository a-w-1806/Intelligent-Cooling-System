#include "EEPROM_24C16.h"

//起始信号 当时钟线为1，数据线有个下降沿
void estart() {   
    ECLK = 1;
    EDTA = 1;
    Somenop();
    EDTA = 0; 
    ECLK = 0;
    Somenop();
}

//终止信号 当时钟线为1，数据线有个上升沿
void estop() {
    EDTA = 0;
    ECLK = 1;
    Somenop();
    EDTA = 1;
    ECLK = 0;
    Somenop();
}

//应答信号由从机发出信号为sda由1变为0
bit ack() {
    
    ECLK = 1;
    EDTA = 1;
    if (EDTA == 1) {
        ECLK = 0;
        return 1;
    } else {
        ECLK = 0;
        return 0;
    }
    
}

//字节写（写数据或地址）数据线sda不变，scl有个上升沿，写入数据
void ewrite_byte(unsigned char dat) {
    unsigned char i;
    for(i = 0; i < 8; i++)
    {
        ECLK = 0;
        Somenop();
        EDTA = dat & 0x80;
        Somenop();
        ECLK = 1;
        Somenop();
        dat <<= 1;
    }
    ECLK = 0;
    Somenop();
}

//字节读 scl有下降沿读出
unsigned char eread_byte() {
    unsigned char i, k;
    for(i = 0; i < 8; i++) {
        ECLK = 1;
        Somenop();
        k = (k << 1) | EDTA;
        ECLK = 0;
        Somenop();
    }
    return k;
}

void ewrite_add(unsigned char add,unsigned char dat) {
    do {
    estart();
    ewrite_byte(0xa0);
    }
    while(ack());
    
    ewrite_byte(add);
    ack();
    do {
    ewrite_byte(dat);
    }
    while(ack());
    estop();
}

unsigned char eread_add(unsigned char add) {
    unsigned char dat;

    do {
    estart();
    ewrite_byte(0xa0);
    }
    while(ack());
    
    ewrite_byte(add);
    ack();

    do {
    estart();
    ewrite_byte(0xa1);
    }
    while(ack());

    dat = eread_byte();
    estop();
    return dat;
}