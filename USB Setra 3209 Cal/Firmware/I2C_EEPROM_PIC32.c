/* I2C_EEPROM_PIC32.c - I2C routines for reading/writing 24LC256 EEprom
 * FOR PIC32MX220F032D - USES I2C1 or I2C2
 * 
 * TO SELECT I2C PORT, UNCOMMENT EITHER "USE_I2C1" or "USE_I2C2"
 * in header file I2C_EEPROM_PIC32.h
 * 
 * Adapted from:
 * http://hades.mech.northwestern.edu/index.php/PIC32MX:_I2C_EEPROM
 * and also from:
 * https://gobotronics.wordpress.com/2010/12/09/i2c-eeprom-pic32/
 * 

 * Compiled with XC32 V1.30
 * 
 * Created 1-2-1017
 */

#include "I2C_EEPROM_PIC32.h"
#include <plib.h>

#define TEST_OUT PORTCbits.RC0
#define RD 1
#define WR 0

void getWriteAck(unsigned char SlaveAddress);

/*******************************************************************
 *	Name:	EepromWriteBlock()
 *	
 *	Inputs:
 *	device - I2C address of the chip
 *	startAddress - 16-bit memory address to write the data to
 *	*ptrData - pointer to data string to be stored
 *	numBytes - numBytes - Number of bytes to be written
 *	
 *  Returns: 1 if successful, 0 if EEPROM does not ACKnowledge a command.
 * 
 *	A block of data with length numBytes is sent to the EEPROM.  
 *  Data is written one byte at a time
 *	beginning at startAddress using function MasterWriteI2C().
 *	As each byte is written, an ACKnowledge bit is read from EEPROM
 *	
 *******************************************************************/
unsigned char EepromWriteBlock(unsigned char device, unsigned short startAddress, unsigned char *ptrData, unsigned short numBytes) 
{
    unsigned char addressHigh, addressLow;
    unsigned short i;

    addressHigh = (unsigned char) ((startAddress & 0xFF00) >> 8);
    addressLow = (unsigned char) (startAddress & 0x00FF);

    StartI2C(); // Send the Start Bit
    IdleI2C(); // Wait to complete    

    MasterWriteI2C(device | WR); // Send EEPROM Device ID and WRITE Command    
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) return (0); // Get ACK from EEPROM

    MasterWriteI2C(addressHigh); // Send EEPROM high address byte
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) return (0); // Get ACK from EEPROM    

    MasterWriteI2C(addressLow); // Send EEPROM low address byte
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) return (0); // Get ACK from EEPROM    


    // Now send block of data
    for (i = 0; i < numBytes; i++) {
        MasterWriteI2C(ptrData[i]);
        IdleI2C(); //Wait to complete
        if (I2CSTATbits.ACKSTAT) return (0); // Get ACK from EEPROM        
    }
    StopI2C(); // Send the Stop condition
    IdleI2C(); // Wait to complete

    // Wait for EEprom to complete write process
    getWriteAck(device);
    return (1);
}

/*******************************************************************
 *	Name:	getWriteAck.c
 * 
 *	Inputs:
 *	unsigned char device - I2C address of the chip
 *	
 *	Description:
 *	This function polls the I2C EEPROM until an 
 *	acknowledgment is received, 
 *  indicating write process is done.
 *	
 *******************************************************************/
void getWriteAck(unsigned char device) {
    while (1) {
        StartI2C(); //  Send the Start Bit
        IdleI2C(); //  Wait to complete

        MasterWriteI2C(device | WR); // Send ID and WRITE command
        IdleI2C(); // Wait to complete

        if (I2CSTATbits.ACKSTAT == 0) { // Wait for EEPROM to acknowledge
            StopI2C(); //Send the Stop condition
            IdleI2C(); //Wait to complete
            break;
        }

        StopI2C(); //Send the Stop condition
        IdleI2C(); //Wait to complete 
    }
}

/* Reads a block of bytes from EEPROM beginning at startAddress
 *	Inputs:
 *	device - I2C address of the chip
 *	startAddress - 16-bit memory address to write the data to 	
 *	numBytes - numBytes - Number of bytes to be written
 * 
 *  Output: *ptrData - pointer to data read from EEPROM 
 *  Returns: 1 if successful, 0 if EEPROM does not ACKnowledge a command.
 */
unsigned char EepromReadBlock(unsigned char device, unsigned short startAddress, unsigned char *ptrData, unsigned char numBytes) 
{
    unsigned char addressHigh, addressLow;
    unsigned char i;

    addressHigh = (unsigned char) ((startAddress & 0xFF00) >> 8);
    addressLow = (unsigned char) (startAddress & 0x00FF);

    StartI2C(); // Send the Start Bit
    IdleI2C(); // Wait to complete    

    MasterWriteI2C(device | WR); // Send EEPROM Device ID and WRITE Command    
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) return (0); // Get ACK from EEPROM

    MasterWriteI2C(addressHigh); // Send EEPROM high address byte
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) return (0); // Get ACK from EEPROM    

    MasterWriteI2C(addressLow); // Send EEPROM low address byte
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) return (0); // Get ACK from EEPROM    

    RestartI2C(); // Now send START sequence again:
    IdleI2C(); // Wait to complete

    MasterWriteI2C(device | RD); // Now send ID with READ Command    
    IdleI2C(); // Wait to complete    
    if (I2CSTATbits.ACKSTAT) return (0); // Get ACK from EEPROM

    // Now receive block of data:
    for (i = 0; i < numBytes; i++) {
        I2CCONbits.RCEN = 1;
        while (!DataRdyI2C()); // Wait for incoming data byte
        ptrData[i] = I2CRCV; // Read data byte from buffer

        if (i < numBytes - 1) { // Master sends ACKS to EEPROM after each read,
            I2CCONbits.ACKDT = 0;
            I2CCONbits.ACKEN = 1;
        } else { // except for last read, when NACK is sent.            
            I2CCONbits.ACKDT = 1;
            I2CCONbits.ACKEN = 1;
        }
        while (I2CCONbits.ACKEN == 1); // Wait till ACK/NACK sequence is over 
    }
    StopI2C();
    IdleI2C();
    return (1); // Return 1 to indicate successful read operation
}

/*******************************************************************
 *	Name:	EepromWriteByte()
 *	
 *	Inputs:
 *	device - I2C address of the chip
 *	startAddress - 16-bit memory address to write the data to
 *	*ptrData - pointer to data string to be stored
 *	numBytes - numBytes - Number of bytes to be written
 *	
 *  Returns: 1 if successful, 0 if EEPROM does not ACKnowledge a command.
 * 
 *	A block of data with length numBytes is sent to the EEPROM.  
 *  Data is written one byte at a time
 *	beginning at startAddress using function MasterWriteI2C().
 *	As each byte is written, an ACKnowledge bit is read from EEPROM
 *	
 *******************************************************************/
unsigned char EepromWriteByte (unsigned char device, unsigned short address, unsigned char data) {
    unsigned char addressHigh, addressLow;

    addressHigh = (unsigned char) ((address & 0xFF00) >> 8);
    addressLow = (unsigned char) (address & 0x00FF);

    StartI2C(); // Send the Start Bit
    IdleI2C(); // Wait to complete    

    MasterWriteI2C(device | WR); // Send EEPROM Device ID and WRITE Command    
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) return (0); // Get ACK from EEPROM

    MasterWriteI2C(addressHigh); // Send EEPROM high address byte
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) return (0); // Get ACK from EEPROM    

    MasterWriteI2C(addressLow); // Send EEPROM low address byte
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) return (0); // Get ACK from EEPROM    

    MasterWriteI2C(data);
    IdleI2C(); //Wait to complete
    if (I2CSTATbits.ACKSTAT) return (0); // Get ACK from EEPROM        

    StopI2C(); // Send the Stop condition
    IdleI2C(); // Wait to complete

    // Wait for EEprom to complete write process
    getWriteAck(device);
    return (1);
}

/*  Reads a single byte from EEPROM at address
 *	Inputs:
 *	device - I2C address of the chip
 *	address - 16-bit memory address to write the data to 	
 * 
 *  Output: *ptrData - pointer to data read from EEPROM 
 *  Returns: 1 if successful, 0 if EEPROM does not ACKnowledge a command.
 */
unsigned char EepromReadByte (unsigned char device, unsigned short address, unsigned char *ptrData) {
    unsigned char addressHigh, addressLow;
    unsigned char i;

    addressHigh = (unsigned char) ((address & 0xFF00) >> 8);
    addressLow = (unsigned char) (address & 0x00FF);

    StartI2C(); // Send the Start Bit
    IdleI2C(); // Wait to complete    

    MasterWriteI2C(device | WR); // Send EEPROM Device ID and WRITE Command    
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) return (0); // Get ACK from EEPROM

    MasterWriteI2C(addressHigh); // Send EEPROM high address byte
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) return (0); // Get ACK from EEPROM    

    MasterWriteI2C(addressLow); // Send EEPROM low address byte
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) return (0); // Get ACK from EEPROM    

    RestartI2C(); // Now send START sequence again:
    IdleI2C(); // Wait to complete

    MasterWriteI2C(device | RD); // Now send ID with READ Command    
    IdleI2C(); // Wait to complete    
    if (I2CSTATbits.ACKSTAT) return (0); // Get ACK from EEPROM

    // Now receive data byte:
    I2CCONbits.RCEN = 1;
    while (!DataRdyI2C()); // Wait for incoming data byte
    *ptrData = I2CRCV; // Read data byte from buffer

    I2CCONbits.ACKDT = 1; // Send NACK to EEPROM
    I2CCONbits.ACKEN = 1;
    while (I2CCONbits.ACKEN == 1); // Wait till ACK/NACK sequence is over 

    StopI2C();
    IdleI2C();
    return (1); // Return 1 to indicate successful read operation
}
