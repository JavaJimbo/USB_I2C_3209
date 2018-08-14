/*********************************************************************
 *
 *                  General Delay rouines
 *
 *********************************************************************
 * FileName:        DelayPIC32.c
 * Dependencies:    Compiler.h
 * Processor:       PIC32
 * Compiler:        Microchip C32 v1.05 or higher
 *
 * JBS:				8-04-2012 Tweaked multiplier for better accuracy with 8 Mhz crystal
 ********************************************************************/
#include "Delay.h"
//#include "HardwareProfileDispatcher.h"
//#include "Compiler.h"
//#define __DELAY_C

void DelayMs(long ms){
unsigned char i;

    while(ms--){
        i=4;
        while(i--){
            Delay10us(4);  // was 25
        }
    }
}

void Delay10us(long dwCount){
unsigned short i;

	while(dwCount--)	
		for (i=0; i<87; i++);  // This was 80 == dwCount*((DWORD)(0.00001/(1.0/GetInstructionClock())/10));
}

void DelayUs(unsigned short Count){
unsigned short i;

	while(Count--)	
		for (i=0; i<1000; i++);
}