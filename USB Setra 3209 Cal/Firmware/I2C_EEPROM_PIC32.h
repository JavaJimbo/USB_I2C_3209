/* I2C_EEPROM_PIC32.h - I2C routines for reading/writing 24LC256 EEprom
 * FOR PIC32MX220F032D - USES I2C1 or I2C2 
 * TO SELECT I2C PORT, UNCOMMENT EITHER "USE_I2C1" or "USE_I2C2" BELOW
 * 
 * Created 1-2-1017
 */

#ifndef I2C_EEPROM_PIC32_H
#define I2C_EEPROM_PIC32_H

// UNCOMMENT ONLY ONE:
// #define USE_I2C1     // SELECTS I2C PORT #1
#define USE_I2C2        // SELECTS I2C PORT #2


#define EEPROM_ADDRESS 0xA0

#ifdef USE_I2C1
#define StartI2C() StartI2C1() 
#define StopI2C() StopI2C1()
#define IdleI2C() IdleI2C1()
#define MasterWriteI2C MasterWriteI2C1
#define MasterReadI2C() MasterReadI2C1()
#define MasterputsI2C MasterputsI2C1
#define MastergetsI2C MastergetsI2C1
#define I2CSTATbits I2C1STATbits
#define I2CCONbits I2C1CONbits
#define RestartI2C() RestartI2C1()
#define OpenI2C OpenI2C1
#define CloseI2C() CloseI2C1()
#define AckI2C() AckI2C1()
#define NotAckI2C() NotAckI2C1()
#define DataRdyI2C() DataRdyI2C1()
#define I2CRCV I2C1RCV
#endif

#ifdef USE_I2C2
#define StartI2C() StartI2C2() 
#define StopI2C() StopI2C2()
#define IdleI2C() IdleI2C2()
#define MasterWriteI2C MasterWriteI2C2
#define MasterReadI2C() MasterReadI2C2()
#define MasterputsI2C MasterputsI2C2
#define MastergetsI2C MastergetsI2C2
#define I2CSTATbits I2C2STATbits
#define I2CCONbits I2C2CONbits
#define RestartI2C() RestartI2C2()
#define OpenI2C OpenI2C2
#define CloseI2C() CloseI2C2()
#define AckI2C() AckI2C2()
#define NotAckI2C() NotAckI2C2()
#define DataRdyI2C() DataRdyI2C2()
#define I2CRCV I2C2RCV
#endif

unsigned char EepromWriteBlock (unsigned char device, unsigned short startAddress, unsigned char *ptrData, unsigned short numBytes);
unsigned char EepromReadBlock (unsigned char device, unsigned short startAddress, unsigned char *ptrData, unsigned char numBytes);
unsigned char EepromWriteByte (unsigned char device, unsigned short address, unsigned char data);
unsigned char EepromReadByte (unsigned char device, unsigned short address, unsigned char *ptrData);

#endif 

