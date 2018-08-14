/* 
 * File:   Definitions.h
 * Author: Jim
 *
 * Created on October 21, 2017, 3:15 PM
 */

#ifndef DEFINITIONS_H
#define	DEFINITIONS_H
// #define BUFFERSIZE 256 // was 128

#define CONFIG_ADDRESS 0x03C0
#define NVRAM_SIZE 1024
#define MAX_RX_BUFFERSIZE (NVRAM_SIZE * 4)
#define MAX_TX_BUFFERSIZE MAX_RX_BUFFERSIZE
#define MAXBUFFER MAX_TX_BUFFERSIZE
#define CHARGEPUMP_ADDRESS_LOW 1022
#define CHARGEPUMP_ADDRESS_HIGH 1023

#define USB_BUFFERSIZE 64

#define true TRUE
#define false FALSE

//#ifndef FALSE
//#define FALSE 0
//#define TRUE !FALSE
//#endif

#endif	/* DEFINITIONS_H */

