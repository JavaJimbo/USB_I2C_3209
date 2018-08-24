/********************************************************************************************************************
 * FileName: main.c Adapted from Microchip CDC serial emulator
 * Compiled for PIC32MX795 XC32 compiler version 1.30
 * 
 * Jim Sedgwick 7-29-18   
 * Separated UART and USB buffers.
 * 7-30-18: 
 * 7-31-18:
 * 8-1-18: 
 * 8-8-18: Put back CRC check, added "BOOT UP" to $HELLO command, removed sprintf() from $TEST
 *          Added I2C timeouts to prevent PIC from getting stuck waiting for ACKs or data.
 * 8-20-18: Couldn't find any issues.
 * 8-24-18: Clamp I2C to ground when not communicating. Implement power supply sequencing, add CloseI2C() command.
 *********************************************************************************************************************/
#include "./USB/usb.h"
#include "./USB/usb_function_cdc.h"
#include "HardwareProfile.h"
#include "./uart2.h"

#define POWERUP_DELAY 50



// #define USE_USB

/** CONFIGURATION **************************************************/
#pragma config UPLLEN   = ON        // USB PLL Enabled
#pragma config FPLLMUL  = MUL_15        // PLL Multiplier
#pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider
#pragma config FPLLODIV = DIV_1         // PLL Output Divider
#pragma config FPBDIV   = DIV_1         // Peripheral Clock divisor
#pragma config FWDTEN   = OFF           // Watchdog Timer
#pragma config WDTPS    = PS1           // Watchdog Timer Postscale
#pragma config FCKSM    = CSDCMD        // Clock Switching & Fail Safe Clock Monitor
#pragma config OSCIOFNC = OFF           // CLKO Enable
#pragma config POSCMOD  = HS            // Primary Oscillator
#pragma config IESO     = OFF           // Internal/External Switch-over
#pragma config FSOSCEN  = OFF           // Secondary Oscillator Enable (KLO was off)
#pragma config FNOSC    = PRIPLL        // Oscillator Selection
#pragma config CP       = OFF           // Code Protect
#pragma config BWP      = OFF           // Boot Flash Write Protect
#pragma config PWP      = OFF           // Program Flash Write Protect
#pragma config ICESEL   = ICS_PGx2      // ICE/ICD Comm Channel Select

/** I N C L U D E S **********************************************************/

#include <XC.h>

#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "usb_config.h"
#include "USB/usb_device.h"
#include "USB/usb.h"
#include "HardwareProfile.h"
#include "Definitions.h"
#include "Delay.h"
#include "I2C_EEPROM_PIC32.h"

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>

#define I2C_TIMEOUT 1000
// #define TEST_OUT LATBbits.LATB3
#define DIAGNOSTICS

#define CDC_COMMAND 0b10001100
#define RDC_COMMAND 0b10001110
#define DSP_COMMAND 0b10001101

#define CR 13
#define LF 10
#define BACKSPACE 8

#define RELAY_K3_ON() PORTSetBits(IOPORT_B, BIT_13)
#define RELAY_K2_ON() PORTSetBits(IOPORT_B, BIT_14)
#define RELAY_K1_ON() PORTSetBits(IOPORT_B, BIT_15)

#define RELAY_K3_OFF() PORTClearBits(IOPORT_B, BIT_13)
#define RELAY_K2_OFF() PORTClearBits(IOPORT_B, BIT_14)
#define RELAY_K1_OFF() PORTClearBits(IOPORT_B, BIT_15)


#define RD 1
#define WR 0
#define WRITE_MEM 0xA0
#define READ_MEM 0x20
#define READ_RESULT 0x40
#define PCAP_DEVICE 0x50
#define POWER_ON_RESET 0x88
#define INITIALIZE 0x8A
#define WRITE_CONFIGURATION 0xA3
#define READ_CONFIGURATION 0x23

#define ACTIVATE_STORE 0x2D
#define STORE_NVRAM 0x96
#define ACTIVATE_RECALL 0x59
#define RECALL_NVRAM 0x99
#define ACTIVATE_ERASE 0xB8
#define ERASE_NVRAM 0x9C

#define MEM_CTRL_REGISTER 0x36      // = 54 decimal
// #define CHARGE_PUMP_REGISTER 0x3E   // = 62 decimnal

#define EEDEVICE 0xA0

// UART FOR PC SERIAL PORT
#define HOSTuart UART2
#define HOSTbits U2STAbits
#define HOST_VECTOR _UART_2_VECTOR

#define CR 13
#define LF 10
#define BACKSPACE 8

#define false FALSE
#define true TRUE
/** V A R I A B L E S ********************************************************/
//char USB_Out_Buffer[CDC_DATA_OUT_EP_SIZE] = "\rTesting USB IO";
char USBReceivedData[CDC_DATA_IN_EP_SIZE];
unsigned char bootFlag = true;

// unsigned char   NextUSBOut;
//char RS232_In_Data;
// int NumUSBbytesReceived = 0;  // Number of characters in the buffer
// unsigned char   RS232cp;       // current position within the buffer
// unsigned char   USBReceivedData_Rdy = false;
// USB_HANDLE      lastTransmission;

unsigned char HOSTRxBuffer[MAX_RX_BUFFERSIZE];
unsigned char HOSTRxBufferFull = false;
unsigned char HOSTTxBuffer[MAX_TX_BUFFERSIZE];
unsigned char HOSTTxBufferFull = false;

unsigned char USBTxPacket[USB_BUFFERSIZE];
unsigned char USBRxBuffer[USB_BUFFERSIZE];
unsigned char USBRxBufferFull = false;
unsigned char USBTxBuffer[MAX_TX_BUFFERSIZE];
unsigned char USBTxBufferFull = false;

int USBTxIndex = 0;
unsigned char   processInputString(unsigned char *ptrBuffer);
unsigned char   executeCommand(unsigned char *ptrCommand, short *ptrValue);
unsigned char   setPWM(unsigned char *ptrPWMstring);
unsigned char   setOutput(unsigned char *ptrPin, unsigned char *ptrPinState);
unsigned char   diagnosticsPrintf(unsigned char *ptrString);
unsigned char   sendToUART(unsigned char UartID, unsigned char *ptrUARTpacket);
unsigned char   sendToUSB(unsigned char *ptrUSBpacket);
extern BOOL     CRCcheck(char *ptrPacket);
extern UINT16   CRCcalculate(char *ptrPacket, BOOL addCRCtoPacket);
unsigned char   OpenPcapI2C();
unsigned char   ClosePcapI2C();
unsigned char   PCAPPowerOnReset();
unsigned char   InitializePCAP();
unsigned char   PcapReadNVRAM(unsigned short startAddress, unsigned short numBytes);
unsigned char   PcapWriteNVRAM(unsigned short startAddress, unsigned short numBytes, unsigned char *ptrData);
unsigned char   RecallPcapNVRAM();
unsigned char   StorePcapNVRAM();
unsigned char   ErasePcapNVRAM();
unsigned char   SendOpcode(unsigned char opcode);
unsigned char   TestRead(unsigned char *ptrReply);
unsigned char   replyToHost(unsigned char *ptrMessage);
unsigned char   PcapReadResultRegisters(unsigned char registerAddress, unsigned short numBytes, unsigned char *ptrData);
unsigned char   storeVersionString();

unsigned short NVRAMsize = 0;

unsigned char NVRAMdata[NVRAM_SIZE];
unsigned char CopyNVRAMdata[NVRAM_SIZE];

#define CHARGE_PUMP_LOW 1022
#define CHARGE_PUMP_HIGH 1023

void InitializeNVRAM(void);
#define MAXSTRING 128
unsigned char strPCapFirmwareVersion[MAXSTRING] = "";
unsigned char strDataBufferCopy[MAXSTRING];
//BOOL stringPrinted;


/** P R I V A T E  P R O T O T Y P E S ***************************************/
void InitializeSystem(void);
void ProcessIO(void);
void USBDeviceTasks(void);
void YourHighPriorityISRCode();
void YourLowPriorityISRCode();
void USBCBSendResume(void);
void BlinkUSBStatus(void);
void UserInit(void);
void InitializeUSART(void);
void putcUSART(char c);
unsigned char getcUSART ();
static unsigned char dataReady = false; 

unsigned char OpenPcapI2C()
{
    DelayMs(POWERUP_DELAY);
    OpenI2C(I2C_EN, 299);    
    return true;
}

unsigned char ClosePcapI2C()
{
    CloseI2C();
    mPORTASetPinsDigitalOut(BIT_2 | BIT_3);
    mPORTAClearBits(BIT_2 | BIT_3);    
    DelayMs(POWERUP_DELAY);
    return true;
}


void main(void)
{        
int i;
int LEDcounter;
    
    LATAbits.LATA2 = 0;
    LATAbits.LATA3 = 0;
    LATBbits.LATB13 = 0;
    LATBbits.LATB14 = 0;
    LATBbits.LATB15 = 0; 
    
    RELAY_K3_OFF();
    RELAY_K2_OFF();
    RELAY_K1_OFF();    

    InitializeSystem();   
    
    ClosePcapI2C();
    // ShutDownI2C();
    
    // printf("\rStarting Setra 3209 Cal program");
    
    LEDcounter = 1000;   
    
    mLED_1_Off();
    mLED_2_Off();
    mLED_3_Off();
    mLED_4_Off();
    
    // Make sure CHARGE PUMP bytes are initialized to 0x00:
    for (i = 0; i < NVRAM_SIZE; i++)  NVRAMdata[i] = CopyNVRAMdata[i] = 0x00;
        
    // OpenPcapI2C();

#ifdef USE_USB    
    // printf("\rStarting USB version");
    while(1) 
    {
        ClrWdt(); // Reset watchdog timer        
        
        if (USBRxBufferFull) 
        {
            USBRxBufferFull = false;   
            // printf("\rBUFFER FULL");
            if (!CRCcheck(USBRxBuffer)) 
               replyToHost("!CRC ERROR");                        
            else if (!processInputString(USBRxBuffer))
            processInputString(USBRxBuffer);
            USBRxBuffer[0] = '\0';
        }
        
        #if defined(USB_POLLING)
		// Check bus status and service USB interrupts.
        USBDeviceTasks();
        #endif
        ProcessIO();        
    }
#else
    printf("\rTesting power supply sequencing");
    while(1) 
    {
        ClrWdt(); // Reset watchdog timer
        
        if (HOSTRxBufferFull) 
        {
            HOSTRxBufferFull = false;               
            //if (!CRCcheck(HOSTRxBuffer)) 
            //   replyToHost("!CRC ERROR");                        
            //else if (!processInputString(HOSTRxBuffer))
            //    replyToHost("!COMMAND STRING ERROR");
            if (!processInputString(HOSTRxBuffer))
                replyToHost("!COMMAND STRING ERROR");  // $$$$
        }
        DelayMs(1);
        if (LEDcounter) 
            LEDcounter--;
        else
        {            
            LEDcounter = 1000;
            mLED_1_Toggle();
        }
    }

#endif    
}
 
void InitializeUSART(void)
{
    UART2Init();
}//end InitializeUSART

#define mDataRdyUSART() UART2IsPressed()
#define mTxRdyUSART()   U2STAbits.TRMT

void putcUSART(char c)  
{
    UART2PutChar(c);
}

unsigned char getcUSART ()
{
	char  c;
    c = UART2GetChar();
	return c;
}

void BlinkUSBStatus(void)
{
    static WORD led_count=0;
    
    if(led_count == 0)led_count = 10000U;
    led_count--;

    #define mLED_Both_Off()         {mLED_1_Off();mLED_2_Off();}
    #define mLED_Both_On()          {mLED_1_On();mLED_2_On();}
    #define mLED_Only_1_On()        {mLED_1_On();mLED_2_Off();}
    #define mLED_Only_2_On()        {mLED_1_Off();mLED_2_On();}

    if(USBSuspendControl == 1)
    {
        if(led_count==0)
        {
            mLED_1_Toggle();
            if(mGetLED_1())
            {
                mLED_2_On();
            }
            else
            {
                mLED_2_Off();
            }
        }//end if
    }
    else
    {
        if(USBDeviceState == DETACHED_STATE)
        {
            mLED_Both_Off();
        }
        else if(USBDeviceState == ATTACHED_STATE)
        {
            mLED_Both_On();
        }
        else if(USBDeviceState == POWERED_STATE)
        {
            mLED_Only_1_On();
        }
        else if(USBDeviceState == DEFAULT_STATE)
        {
            mLED_Only_2_On();
        }
        else if(USBDeviceState == ADDRESS_STATE)
        {
            if(led_count == 0)
            {
                mLED_1_Toggle();
                mLED_2_Off();
            }//end if
        }
        else if(USBDeviceState == CONFIGURED_STATE)
        {
            if(led_count==0)
            {
                mLED_1_Toggle();
                if(mGetLED_1())
                {
                    mLED_2_Off();
                }
                else
                {
                    mLED_2_On();
                }
            }//end if
        }//end if(...)
    }//end if(UCONbits.SUSPND...)

}//end BlinkUSBStatus

void USBCBSuspend(void)
{
}
void USBCBWakeFromSuspend(void)
{
}
void USBCB_SOF_Handler(void)
{
}
void USBCBErrorHandler(void)
{
}
void USBCBCheckOtherReq(void)
{
    USBCheckCDCRequest();
}//end

void USBCBStdSetDscHandler(void)
{
}//end
void USBCBInitEP(void)
{
    CDCInitEP();
}

void USBCBSendResume(void)
{
    static WORD delay_count;
    
    if(USBGetRemoteWakeupStatus() == TRUE) 
    {
        if(USBIsBusSuspended() == TRUE)
        {
            USBMaskInterrupts();
            USBCBWakeFromSuspend();
            USBSuspendControl = 0; 
            USBBusIsSuspended = FALSE;  //So we don't execute this code again, 
            delay_count = 3600U;        
            do
            {
                delay_count--;
            }while(delay_count);
            
            //Now drive the resume K-state signalling onto the USB bus.
            USBResumeControl = 1;       // Start RESUME signaling
            delay_count = 1800U;        // Set RESUME line for 1-13 ms
            do
            {
                delay_count--;
            } while(delay_count);
            USBResumeControl = 0;       //Finished driving resume signalling

            USBUnmaskInterrupts();
        }
    }
}


#if defined(ENABLE_EP0_DATA_RECEIVED_CALLBACK)
void USBCBEP0DataReceived(void)
{
}
#endif

BOOL USER_USB_CALLBACK_EVENT_HANDLER(int event, void *pdata, WORD size)
{
    switch(event)
    {
        case EVENT_TRANSFER:
            //Add application specific callback task or callback function here if desired.
            break;
        case EVENT_SOF:
            USBCB_SOF_Handler();
            break;
        case EVENT_SUSPEND:
            USBCBSuspend();
            break;
        case EVENT_RESUME:
            USBCBWakeFromSuspend();
            break;
        case EVENT_CONFIGURED: 
            USBCBInitEP();
            break;
        case EVENT_SET_DESCRIPTOR:
            USBCBStdSetDscHandler();
            break;
        case EVENT_EP0_REQUEST:
            USBCBCheckOtherReq();
            break;
        case EVENT_BUS_ERROR:
            USBCBErrorHandler();
            break;
        case EVENT_TRANSFER_TERMINATED:
            break;
        default:
            break;
    }      
    return TRUE; 
}


void InitializeSystem(void) 
{    
   
    
    SYSTEMConfigPerformance(60000000);
    #if defined(USE_USB_BUS_SENSE_IO)
    tris_usb_bus_sense = INPUT_PIN; // See HardwareProfile.h
    #endif
    
    #if defined(USE_SELF_POWER_SENSE_IO)
    tris_self_power = INPUT_PIN;	// See HardwareProfile.h
    #endif        
    
    // UserInit();
    // Turn off JTAG so we get the pins back
    mJTAGPortEnable(false);
    
    // Set up HOST UART
    
    UARTConfigure(HOSTuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(HOSTuart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(HOSTuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
#define SYS_FREQ 60000000
    UARTSetDataRate(HOSTuart, SYS_FREQ, 115200);
    UARTEnable(HOSTuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure HOST UART Interrupts
    INTEnable(INT_SOURCE_UART_TX(HOSTuart), INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(HOSTuart), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(HOSTuart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(HOSTuart), INT_SUB_PRIORITY_LEVEL_0);
    
    mPORTASetPinsDigitalOut(BIT_2 | BIT_3);
    mPORTAClearBits(BIT_2 | BIT_3);

    PORTClearBits(IOPORT_B, BIT_3 | BIT_12 | BIT_13 | BIT_14 | BIT_15);
    mPORTBSetPinsDigitalOut(BIT_3 | BIT_12 | BIT_13 | BIT_14 | BIT_15);

    PORTSetPinsDigitalIn(IOPORT_B, BIT_1);
    PORTSetPinsDigitalOut(IOPORT_B, BIT_0);
    PORTSetBits(IOPORT_E, BIT_0);
    PORTSetPinsDigitalIn(IOPORT_D, BIT_15);  // Initialize FAULT signal input   
    
    // Turn on the interrupts
    INTEnableSystemMultiVectoredInt();

	// lastTransmission = 0;

    USBDeviceInit();	// Initializes USB module SFRs and firmware
    					// variables to known states.        
    
	mInitAllLEDs();    
}//end UserInit

unsigned char PCAPPowerOnReset()
{
    StartI2C(); // Send the Start Bit
    IdleI2C(); // Wait to complete    

    MasterWriteI2C(PCAP_DEVICE | WR); // Send EEPROM Device ID and WRITE Command    
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) return false; // printf("\rNO ACK #1"); // Get ACK from EEPROM    
        
    MasterWriteI2C(POWER_ON_RESET); // Send opcode and two high address bits
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) return false; // intf("\rNO ACK #2"); // Get ACK from EEPROM       
    
    StopI2C(); // Send the Stop condition
    IdleI2C(); // Wait to complete    
    return true;
}

unsigned char InitializePCAP()
{    
    StartI2C(); // Send the Start Bit
    IdleI2C(); // Wait to complete    

    MasterWriteI2C(PCAP_DEVICE | WR); // Send EEPROM Device ID and WRITE Command    
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) 
    {
        //printf("\rNO ACK #1"); // Get ACK from EEPROM
        return false;
    }
    
    MasterWriteI2C(INITIALIZE); // Send opcode and two high address bits
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT)
    {
        //printf("\rNO ACK #2"); // Get ACK from EEPROM
        return false;
    }
    
    StopI2C(); // Send the Stop condition
    IdleI2C(); // Wait to complete
    return true;
}



unsigned char ErasePcapNVRAM()
{
unsigned char command[1] =  {ACTIVATE_ERASE};

    if (!PcapWriteNVRAM(MEM_CTRL_REGISTER + CONFIG_ADDRESS, 1, command))
        return false;
    if (!SendOpcode(ERASE_NVRAM))
        return false;
    return true;
}

unsigned char RecallPcapNVRAM()
{   
unsigned char command[1] =  {ACTIVATE_RECALL};
 
    if (!PcapWriteNVRAM(MEM_CTRL_REGISTER + CONFIG_ADDRESS, 1, command))
        return false;
    if (!SendOpcode(RECALL_NVRAM))
        return false;
    return true;
}

unsigned char executeCommand(unsigned char *ptrCommand, short *ptrData)
{
    char strHexValue[16];
    char strReply[MAX_TX_BUFFERSIZE] = "$OK";    
    unsigned short NVRAMstartAddress, address;
    unsigned short numDataBytes = 0;
    unsigned char registerAddress = 0;
    unsigned short i;    
    #define MAX_RESULT_REGISTERS 35
    unsigned char arrRegister[MAX_RESULT_REGISTERS];
    unsigned char ReplyByte;
    
    union {
        unsigned char byte[2];
        unsigned int  integer;	
    } convert;
    #define HighByte byte[1]
    #define LowByte byte[0]    

    if (!strcmp(ptrCommand, "HELLO"))
    {
        if (bootFlag) 
        {
            strcpy(strReply, "$SYSTEM BOOTUP");
            bootFlag = false;
        }
        else strcpy(strReply, "$OK HELLO_WORLD");
    }
    else if (!strcmp(ptrCommand, "OPEN"))
    {
        OpenPcapI2C();
    }    
    else if (!strcmp(ptrCommand, "CLOSE"))
    {
        ClosePcapI2C();
    }      
    else if (!strcmp(ptrCommand, "REBOOT"))
    {
        while(1);
    }    
    else if (!strcmp(ptrCommand, "WRITE_VERSION"))
    {       
        if (storeVersionString()) strcpy(strReply, "$OK ");
        else strcpy(strReply, "$ERROR Bad version string");
    }    
    else if (!strcmp(ptrCommand, "READ_VERSION"))
    {
        strcpy(strReply, "$OK ");
        strcat(strReply, strPCapFirmwareVersion);
    }        
    else if (!strcmp(ptrCommand, "POR"))
    {
        if (!PCAPPowerOnReset()) strcpy(strReply, "$I2C_ERROR");
    }
    else if (!strcmp(ptrCommand, "INIT"))
    {
        if (!InitializePCAP()) strcpy(strReply, "$I2C_ERROR");
    }
    else if (!strcmp(ptrCommand, "CDC"))  
    {
        if (!SendOpcode(CDC_COMMAND)) strcpy(strReply, "$I2C_ERROR");
    }
    else if (!strcmp(ptrCommand, "RDC"))
    {
        if (!SendOpcode(RDC_COMMAND)) strcpy(strReply, "$I2C_ERROR");
    }    
    else if (!strcmp(ptrCommand, "DSP"))
    {
        if (!SendOpcode(DSP_COMMAND)) strcpy(strReply, "$I2C_ERROR");
    }
    else if (!strcmp(ptrCommand, "STORE"))
    {
        if (!StorePcapNVRAM()) strcpy(strReply, "$I2C_ERROR");        
    }
    else if (!strcmp(ptrCommand, "RECALL")) 
    {
        if (!RecallPcapNVRAM()) strcpy(strReply, "$I2C_ERROR");
    }
    else if (!strcmp(ptrCommand, "ERASE"))
    {
        if (!ErasePcapNVRAM()) strcpy(strReply, "$I2C_ERROR");        
    }    
    else if (!strcmp(ptrCommand, "TEST"))
    {
        if (!TestRead(&ReplyByte)) strcpy(strReply, "$I2C_ERROR");
        else if (ReplyByte != 0x11) strcpy(strReply, "$TEST_READ_ERROR");
    }
    else if (!strcmp(ptrCommand, "PRINT"))
    {
        for (i = 0; i < NVRAM_SIZE; i++)
        {        
            if ((i % 16) == 0) 
            {
                DelayMs(2);
                printf("\r#%04X: %02X, ", i, NVRAMdata[i]);
            }
            else printf("%02X, ", NVRAMdata[i]);
        }
    }    
    // else if (!strcmp(ptrCommand, "CLEAR"))
    //    ClearNVRAMdata();
    else if (!strcmp(ptrCommand, "UPLOAD"))
    {        
        convert.byte[0] = (unsigned char) ptrData[1];
        convert.byte[1] = (unsigned char) ptrData[0];
        NVRAMstartAddress = convert.integer;   
        
        convert.byte[0] = (unsigned char) ptrData[3];
        convert.byte[1] = (unsigned char) ptrData[2];
        numDataBytes = convert.integer;       
        
        strReply[0] = '\0';        
        if ((NVRAMstartAddress + numDataBytes) > NVRAM_SIZE)
            strcpy(strReply, "$ADDRESS_OUT_OF_RANGE");
        else
        {
            strcpy(strReply, "$OK> ");
            for (i = NVRAMstartAddress; i < (NVRAMstartAddress + numDataBytes); i++)  
            {
                sprintf(strHexValue, " %02X", NVRAMdata[i]);
                strcat(strReply, strHexValue);
            }
        }
    }        
    else if (!strcmp(ptrCommand, "BOGUS_MAP"))
    {       
        unsigned char testData = 0;
        for (i = 0; i < 960; i++)
        {
            NVRAMdata[i] = testData++;
            if (testData > 254) testData = 0x0;
        }
    }
    else if (!strcmp(ptrCommand, "DOWNLOAD"))
    {                
        convert.byte[0] = (unsigned char) ptrData[1];
        convert.byte[1] = (unsigned char) ptrData[0];
        NVRAMstartAddress = convert.integer;   
        
        convert.byte[0] = (unsigned char) ptrData[3];
        convert.byte[1] = (unsigned char) ptrData[2];
        numDataBytes = convert.integer;               
        
        if ((NVRAMstartAddress + numDataBytes) > NVRAM_SIZE)
            strcpy(strReply, "$ADDRESS_OUT_OF_RANGE");
        else
        {            
            for (i = 0; i < numDataBytes; i++)
            {
                address = i + NVRAMstartAddress;
                {
                    // Copy downloaded data to any address in NVRAM
                    // EXCEPT for the two CHARGE PUMP bytes,
                    // which should never be overwritten by external data;
                    if (address != CHARGE_PUMP_LOW && address != CHARGE_PUMP_HIGH)
                        NVRAMdata[address] = (unsigned char) ptrData[i+4]; 
                }
            }
        }
    }
    else if (!strcmp(ptrCommand, "READ_MAP"))
    {                
        if (!PcapReadNVRAM(0x0000, NVRAM_SIZE))
            strcpy(strReply, "$I2C_ERROR");
    }       
    else if (!strcmp(ptrCommand, "WRITE_MAP"))
    {       
        for (i = 0; i < NVRAM_SIZE; i++)
            CopyNVRAMdata[i] = NVRAMdata[i];
        
        if (!PcapWriteNVRAM(0x0000, NVRAM_SIZE, NVRAMdata))
            strcpy(strReply, "$I2C_ERROR");        
    } 
    else if (!strcmp(ptrCommand, "CHECK_MAP"))
    {        
        for (i = 0; i < NVRAM_SIZE; i++)
        {
          if (NVRAMdata[i] != CopyNVRAMdata[i]) break;
        }
        if (i == NVRAM_SIZE) strcpy(strReply, "$OK NVRAM WRITE SUCCESS");
        else strcpy(strReply, "$ERROR NVRAM INCORRECT");
    }
    else if (!strcmp(ptrCommand, "READ"))
    {        
        convert.byte[0] = (unsigned char) ptrData[1];
        convert.byte[1] = (unsigned char) ptrData[0];
        NVRAMstartAddress = convert.integer;   
        
        convert.byte[0] = (unsigned char) ptrData[3];
        convert.byte[1] = (unsigned char) ptrData[2];
        numDataBytes = convert.integer;   
        
        if ((NVRAMstartAddress + numDataBytes) > NVRAM_SIZE)
            strcpy(strReply, "$ADDRESS_OUT_OF_RANGE");
        else if (PcapReadNVRAM(NVRAMstartAddress, numDataBytes))
        {            
            strcpy(strReply, "$OK ");
            for (i = NVRAMstartAddress; i < (NVRAMstartAddress + numDataBytes); i++)
            {
                sprintf(strHexValue, "%02X ", NVRAMdata[i]);
                strcat(strReply, strHexValue);
            }                        
        }
        else strcpy(strReply, "$I2C_ERROR");
    }    
    else if (!strcmp(ptrCommand, "STARTUP"))  // Resets PCAP, initializes, transfers NVRAM to RAM, and stores charge pump bytes.
    {                
        if (!PCAPPowerOnReset()) strcpy(strReply, "$I2C_ERROR");
        else if (!InitializePCAP()) strcpy(strReply, "$I2C_ERROR");
        else if (!RecallPcapNVRAM()) strcpy(strReply, "$I2C_ERROR");
        else if (!PcapReadNVRAM(CHARGE_PUMP_LOW, 2)) strcpy(strReply, "$I2C_ERROR");
        else if (!TestRead(&ReplyByte)) strcpy(strReply, "$I2C_ERROR");
        else if (ReplyByte != 0x11) sprintf(strReply, "$TEST_READ_ERROR");        
        else sprintf(strReply, "$OK %02X %02X", NVRAMdata[CHARGE_PUMP_LOW], NVRAMdata[CHARGE_PUMP_HIGH]);        
    }        
    else if (!strcmp(ptrCommand, "READ_RESULT"))
    {        
        registerAddress = (unsigned char) ptrData[0];
        numDataBytes = (unsigned char) ptrData[1];        
        
        for (i = 0; i < MAX_RESULT_REGISTERS; i++) arrRegister[i] = 0;            
        
        if ((registerAddress + numDataBytes) > MAX_RESULT_REGISTERS)        
            strcpy(strReply, "$REGISTER_OUT_OF_RANGE");                    
        else if (PcapReadResultRegisters(registerAddress, numDataBytes, arrRegister))
        {     
            strcpy(strReply, "$OK ");
            for (i = 0; i < numDataBytes; i++)
            {
                sprintf(strHexValue, " %02X", arrRegister[i]);
                strcat(strReply, strHexValue);
            }                 
        }
        else  strcpy(strReply, "$PCAP ERROR");        
    }       
    
    else if (!strcmp(ptrCommand, "WRITE"))
    {          
        convert.byte[0] = (unsigned char) ptrData[1];
        convert.byte[1] = (unsigned char) ptrData[0];
        NVRAMstartAddress = convert.integer;   
        
        convert.byte[0] = (unsigned char) ptrData[3];
        convert.byte[1] = (unsigned char) ptrData[2];
        numDataBytes = convert.integer;   
        
        if ((NVRAMstartAddress + numDataBytes) > NVRAM_SIZE)
        {
            strcpy(strReply, "$ERROR: ADRESS OUT OF BOUNDS");   
            return false;
        }        
        //if (!PcapWriteNVRAM(NVRAMstartAddress, numDataBytes, &ptrData[4]))
        //    strcpy(strReply, "!I2C_ERROR");
        for (i = NVRAMstartAddress; i < numDataBytes; i++)
        {
            // Copy data to any address in NVRAM
            // EXCEPT for the two CHARGE PUMP bytes,
            // which should never be overwritten by external data;
            if (i != CHARGE_PUMP_LOW && i != CHARGE_PUMP_HIGH) 
                NVRAMdata[i] = (unsigned char) ptrData[i+4];
        }        
    }   
    else if (!strcmp(ptrCommand, "RESET_RELAYS"))
    {
        ClosePcapI2C();        
        RELAY_K1_OFF();
        RELAY_K2_OFF();
        RELAY_K3_OFF();
    }
    else if (!strcmp(ptrCommand, "SET_5V_VOUT"))
    {
        RELAY_K1_OFF();
        RELAY_K2_ON();
        RELAY_K3_OFF();
        OpenPcapI2C();
    }
    else if (!strcmp(ptrCommand, "SET_24V_VOUT"))
    {
        RELAY_K1_ON();
        RELAY_K2_ON();
        RELAY_K3_OFF();
        OpenPcapI2C();
    }
    else if (!strcmp(ptrCommand, "SET_MA_OUT"))
    {
        RELAY_K1_ON();
        RELAY_K2_ON();
        RELAY_K3_OFF();
        OpenPcapI2C();
    }
    else if (!strcmp(ptrCommand, "SET_THERMISTOR"))
    {
        ClosePcapI2C();
        RELAY_K1_OFF();
        RELAY_K2_OFF();
        RELAY_K3_ON();        
    }    
    else strcpy(strReply, "$ERROR UNRECOGNIZED COMMAND");
    // strcat(strReply, "<");
    replyToHost(strReply);
    return true;
}

#define TEST_READ_COMMAND 0x7E
unsigned char TestRead(unsigned char *ptrReply)
{       
short timeout;
    I2CRCV = 0x00;
    StartI2C(); // Send the Start Bit
    IdleI2C(); // Wait to complete    
    
    MasterWriteI2C(PCAP_DEVICE | WR); // Send EEPROM Device ID and WRITE Command    
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) 
        return false;
        
    MasterWriteI2C(TEST_READ_COMMAND); // Send opcode
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) 
        return false;
    
    RestartI2C(); // Now send START sequence again:
    IdleI2C(); // Wait to complete

    MasterWriteI2C(PCAP_DEVICE | RD); // Send EEPROM Device ID and READ Command     
    IdleI2C(); // Wait to complete    
    if (I2CSTATbits.ACKSTAT) 
        return false;  
    
    // Now receive data byte:
    I2CCONbits.RCEN = 1;
    // while (!DataRdyI2C()); // Wait for incoming data byte
    timeout = I2C_TIMEOUT;
    while (!DataRdyI2C())
    {
        DelayMs(1);
        if (timeout) timeout--;
        else     
        {
            StopI2C();
            IdleI2C();        
            return false;
        }
    }
        
    *ptrReply = I2CRCV; // Read data byte from buffer    
    

    I2CCONbits.ACKDT = 1; // Send NACK to EEPROM
    I2CCONbits.ACKEN = 1;
    // while (I2CCONbits.ACKEN == 1); // Wait till ACK/NACK sequence is over 
    timeout = I2C_TIMEOUT;
    while (I2CCONbits.ACKEN == 1) // Wait till ACK/NACK sequence is over 
    {
        if (timeout) timeout--;
        else
        {                
            StopI2C();
            IdleI2C();        
            return false;                
        }
        DelayMs(1);
    } 
    StopI2C();
    IdleI2C();        
    return true;
}

unsigned char SendOpcode(unsigned char opcode)
{       
    StartI2C(); // Send the Start Bit
    IdleI2C(); // Wait to complete    
    
    MasterWriteI2C(PCAP_DEVICE | WR); // Send EEPROM Device ID and WRITE Command    
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) 
    {
        //printf("\rNO ACK #1"); // Get ACK from EEPROM
        return false;
    }        
        
    MasterWriteI2C(opcode); // Send opcode
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) 
    {
        //printf("\rNO ACK #2"); // Get ACK from EEPROM
        return false;
    }                

    StopI2C();
    IdleI2C();        
    return true;
}


unsigned char   PcapWriteNVRAM(unsigned short startAddress, unsigned short numBytes, unsigned char *ptrData)
{    
    union {
        unsigned char byte[2];
        unsigned int  integer;	
    } convert;
    #define HighByte byte[1]
    #define LowByte byte[0]    
    unsigned short i;
        
    // WRITE DATA
    StartI2C(); // Send the Start Bit
    IdleI2C(); // Wait to complete    

    MasterWriteI2C(PCAP_DEVICE | WR); // Send EEPROM Device ID and WRITE Command    
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) 
    {
        //printf("\rNO ACK #1"); // Get ACK from EEPROM
        return false;
    }
    
    convert.integer = startAddress;
    MasterWriteI2C(WRITE_MEM | (convert.HighByte & 0x03)); // Send opcode ORed with two high address bits    
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) 
    {
        //printf("\rNO ACK #2"); // Get ACK from EEPROM    
        return false;
    }

    MasterWriteI2C(convert.LowByte); // Send EEPROM low address byte    
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT)
    {
        //printf("\rNO ACK #3"); // Get ACK from EEPROM    
        return false;
    }

    // Now send test data
    for (i = 0; i < numBytes; i++) 
    {
        MasterWriteI2C(ptrData[i]);        
        IdleI2C(); //Wait to complete
        if (I2CSTATbits.ACKSTAT)
        {
            //printf("\rNO ACK #4"); // Get ACK from EEPROM 
            return false;
        }        
    }    
    
    StopI2C(); // Send the Stop condition
    IdleI2C(); // Wait to complete
    
    // printf ("DONE.");

    return true;
}

unsigned char   PcapReadNVRAM(unsigned short startAddress, unsigned short numBytes)
{
    unsigned short i, timeout;
        
    union {
        unsigned char byte[2];
        unsigned int  integer;	
    } convert;
    #define HighByte byte[1]
    #define LowByte byte[0]       
    
    if ((startAddress + numBytes) > NVRAM_SIZE) return false;
    
    // printf("\rReading %d bytes @ %d", numBytes, startAddress);
        
    StartI2C(); // Send the Start Bit
    IdleI2C(); // Wait to complete    
    
    MasterWriteI2C(PCAP_DEVICE | WR); // Send EEPROM Device ID and WRITE Command    
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) return false;
    
    convert.integer = startAddress;
    MasterWriteI2C(READ_MEM | (convert.HighByte & 0x03)); // Send opcode ORed with two high address bits
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) return false;
    
    MasterWriteI2C(convert.LowByte); // Send EEPROM low address byte
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) return false;

    RestartI2C(); // Now send START sequence again:
    IdleI2C(); // Wait to complete

    MasterWriteI2C(PCAP_DEVICE | RD); // Send EEPROM Device ID and READ Command     
    IdleI2C(); // Wait to complete    
    if (I2CSTATbits.ACKSTAT) return false;
    
    DelayMs(4);

    for (i = 0; i < numBytes; i++) 
    {
        I2CCONbits.RCEN = 1;
        // while (!DataRdyI2C()); // Wait for incoming data byte
        timeout = I2C_TIMEOUT;
        while (!DataRdyI2C())
        {
            DelayMs(1);
            if (timeout) timeout--;
            else
            {                
                StopI2C();
                IdleI2C();        
                return false;                
            }
        }
        
        NVRAMdata[i+startAddress] = I2CRCV; // Read data byte into NVRAM beginning at start address

        if (i < numBytes - 1) {     // Master sends ACKS to EEPROM after each read
            I2CCONbits.ACKDT = 0;
            I2CCONbits.ACKEN = 1;
        } else {                    // But for last read, NACK is sent.            
            I2CCONbits.ACKDT = 1;
            I2CCONbits.ACKEN = 1;
        }
        // while (I2CCONbits.ACKEN == 1); // Wait till ACK/NACK sequence is over 
        timeout = I2C_TIMEOUT;
        while (I2CCONbits.ACKEN == 1) // Wait till ACK/NACK sequence is over 
        {
            if (timeout) timeout--;
            else
            {                
                StopI2C();
                IdleI2C();        
                return false;                
            }
            DelayMs(1);
        }
    }    
    StopI2C();
    IdleI2C();            
    return true;
}

unsigned char   PcapReadResultRegisters(unsigned char registerAddress, unsigned short numBytes, unsigned char *ptrData)
{
    unsigned short i, timeout; 
    
    // printf("\rReading %d bytes @ %d", numBytes, startAddress);
        
    StartI2C(); // Send the Start Bit
    IdleI2C(); // Wait to complete    
    
    MasterWriteI2C(PCAP_DEVICE | WR); // Send EEPROM Device ID and WRITE Command    
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) return false;
        
    //Delay10us(100);
    
    MasterWriteI2C(READ_RESULT | (registerAddress & 0x3F)); // Send opcode ORed with six address bits
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) return false;    
    
    //Delay10us(100);

    RestartI2C(); // Now send START sequence again:
    IdleI2C(); // Wait to complete

    MasterWriteI2C(PCAP_DEVICE | RD); // Send EEPROM Device ID and READ Command     
    IdleI2C(); // Wait to complete    
    if (I2CSTATbits.ACKSTAT) return false;
    
    // DelayMs(4); 

    for (i = 0; i < numBytes; i++) 
    {
        I2CCONbits.RCEN = 1;
        // while (!DataRdyI2C()); // Wait for incoming data byte
        timeout = I2C_TIMEOUT;
        while (!DataRdyI2C()) // Wait for incoming data byte        
        {
            DelayMs(1);
            if (timeout) timeout--;
            else 
            {
                StopI2C();
                IdleI2C();     
                return false;
            }
        }
        
        ptrData[i] = I2CRCV; // Read data byte from buffer

        if (i < numBytes - 1) {     // Master sends ACKS to EEPROM after each read
            I2CCONbits.ACKDT = 0;
            I2CCONbits.ACKEN = 1;
        } else {                    // But for last read, NACK is sent.            
            I2CCONbits.ACKDT = 1;
            I2CCONbits.ACKEN = 1;
        }
    
        // while (I2CCONbits.ACKEN == 1); // Wait till ACK/NACK sequence is over 
        timeout = I2C_TIMEOUT;
        while (I2CCONbits.ACKEN == 1) // Wait till ACK/NACK sequence is over 
        {
            DelayMs(1);
            if (timeout) timeout--;
            else 
            {
                StopI2C();
                IdleI2C();      
                return false;                
            }
        }
    }        
    StopI2C();
    IdleI2C();      
    return true;
}

unsigned char StorePcapNVRAM()
{   
unsigned char command[1] = {ACTIVATE_STORE};

    if (!PcapWriteNVRAM(MEM_CTRL_REGISTER + CONFIG_ADDRESS, 1, command))
        return false;
    if (!SendOpcode(STORE_NVRAM))
        return false;
    return true;
}

unsigned char processInputString(unsigned char *ptrBuffer) 
{        
    unsigned char ptrCommand[64], *ptrToken;
    unsigned char delimiters[] = "$>[\r ";
    short dataIndex = 0, intValue = 0;
    short arrByteData[NVRAM_SIZE];      
    unsigned char dataFlag = true;
    
    if (strchr(ptrBuffer, '@')!= NULL)    
    {
        if (strlen(ptrBuffer) < MAXSTRING) strcpy(strDataBufferCopy, ptrBuffer);
        else strDataBufferCopy[0] = '\0';       
        dataFlag = false;
    }    

    ptrToken = strtok(ptrBuffer, delimiters);
    
    if (ptrToken==NULL) return (false);
    if (strlen(ptrToken) > 64) return false;        
    strcpy (ptrCommand, ptrToken);     
    if (dataFlag)
    {
        dataIndex = 0;
        do
        {
            ptrToken = strtok(NULL, delimiters);
            if (ptrToken)
            {         
                intValue = (int) strtol(ptrToken, NULL, 16);            
                if (dataIndex < NVRAM_SIZE)      
                {
                    arrByteData[dataIndex] = (short) intValue;
                    dataIndex++;
                }
                else return false;
            }        
        } while (ptrToken != NULL);
    }
    if (executeCommand(ptrCommand, arrByteData)) return (true);
    else return (false);
}

unsigned char storeVersionString()
{
int i, j, length;
char ch = 0;

    length = strlen(strDataBufferCopy);
    
    if (length == 0 || length > MAXSTRING) return false;    
    
    for (i = 0; i < length; i++)
    {
        ch = strDataBufferCopy[i];
        if (ch == '$') break;
    }

    if (ch != '$') return false;

    for (i = i; i < length; i++)
    {
        ch = strDataBufferCopy[i];
        if (ch == ' ') break;
    }

    if (ch != ' ') return false;

    j = 0;
    for (i = i; i < length; i++)
    {
        ch = strDataBufferCopy[i];
        if (ch == '[') break;
        if (j >= MAXSTRING-2) return false;
        strPCapFirmwareVersion[j++] = ch;        
    }
    strPCapFirmwareVersion[j] = '\0';
    if (ch != '[') return false;
    else return true;
}



// HOST UART interrupt handler it is set at priority level 2
void __ISR(HOST_VECTOR, ipl2) IntHostUartHandler(void) 
{
    unsigned char ch;
    static unsigned short HOSTRxIndex = 0;

    if (HOSTbits.OERR || HOSTbits.FERR) {
        if (UARTReceivedDataIsAvailable(HOSTuart))
            ch = UARTGetDataByte(HOSTuart);
        HOSTbits.OERR = 0;
        HOSTRxIndex = 0;
    } else if (INTGetFlag(INT_SOURCE_UART_RX(HOSTuart))) {
        INTClearFlag(INT_SOURCE_UART_RX(HOSTuart));
        if (UARTReceivedDataIsAvailable(HOSTuart)) {
            ch = toupper(UARTGetDataByte(HOSTuart));
            // ch = UARTGetDataByte(HOSTuart);
            if (ch == '$') HOSTRxIndex = 0;
            if (ch == LF || ch == 0);
            else if (ch == BACKSPACE) {
                while (!UARTTransmitterIsReady(HOSTuart));
                UARTSendDataByte(HOSTuart, ' ');
                while (!UARTTransmitterIsReady(HOSTuart));
                UARTSendDataByte(HOSTuart, BACKSPACE);
                if (HOSTRxIndex > 0) HOSTRxIndex--;
            } else if (ch == CR) {
                if (HOSTRxIndex < (MAX_RX_BUFFERSIZE - 1)) {
                    HOSTRxBuffer[HOSTRxIndex] = CR;
                    HOSTRxBuffer[HOSTRxIndex + 1] = '\0'; 
                    HOSTRxBufferFull = true;
                }
                HOSTRxIndex = 0;
            }                
            else if (HOSTRxIndex < MAX_RX_BUFFERSIZE)
                HOSTRxBuffer[HOSTRxIndex++] = ch;
        }
    }
    if (INTGetFlag(INT_SOURCE_UART_TX(HOSTuart))) {
        INTClearFlag(INT_SOURCE_UART_TX(HOSTuart));
    }
}


unsigned char checkForCR(unsigned char *ptrBuffer)
{
    int i;
    unsigned char ch;
    
    for (i = 0; i < USB_BUFFERSIZE; i++)
    {        
        ch = ptrBuffer[i];
        if (ch == '\0') break;
        else if (ch == '\r') return true;
    }
    return false;
}

void ProcessIO(void)
{   
    static int bytesReceived = 0;               
    static char USBIncomingPacket[USB_BUFFERSIZE] = "";
    int i = 0, numBytesToSend = 0;
    char ch;
    
    //Blink the LEDs according to the USB device status
    BlinkUSBStatus();
    // User Application USB tasks
    if((USBDeviceState < CONFIGURED_STATE)||(USBSuspendControl==1)) return;

	if (!dataReady) // only check for new USB buffer if the old RS232 buffer is
	{				// empty.  This will cause additional USB packets to be NAK'd
		bytesReceived = getsUSBUSART(USBIncomingPacket, USB_BUFFERSIZE); //until the buffer is free.        
		if (bytesReceived > 0)
		{	
            USBIncomingPacket[bytesReceived] = '\0';
            if (checkForCR(USBIncomingPacket))            
                USBRxBufferFull = true;                                        
            strcat(USBRxBuffer, USBIncomingPacket);		
            // printf("\rRX: %s", USBRxBuffer);
		}
	}

    // Check if any bytes are waiting in the queue to send to the USB host.
    // If any bytes are waiting, and the endpoint is available, prepare to
    // send the USB packet to the host.
    if (USBUSARTIsTxTrfReady() && dataReady)
	{
        // If there is USB data to send, copy it to the outgoing data buffer here:
        for (i = 0; i < USB_BUFFERSIZE; i++)
        {        
            if (USBTxIndex >= MAX_TX_BUFFERSIZE) break;
            ch = USBTxBuffer[USBTxIndex++];
            USBTxPacket[i] = ch;
            if (ch == '\r') 
            {
                i++;
                USBTxIndex = 0;  
                dataReady = false;
                break;
            }
            if (ch == '\0') break;
        }            
        numBytesToSend = i;
        USBTxBuffer[i] = '\0';
        // If all USB data has been sent, clear flag and data index here:
        if (USBTxIndex >= strlen(USBTxBuffer))
        {
            USBTxIndex = 0;
            dataReady = false;
        }
        // Now send outgoing USB data:
        if (numBytesToSend) putUSBUSART(USBTxPacket, numBytesToSend);  
        // printf("\rTX: %s, %d bytes", USBTxPacket, numBytesToSend);
	}
    
    CDCTxService();
}//end ProcessIO

#ifdef USE_USB
unsigned char replyToHost(unsigned char *ptrMessage){
int length;    
    if (ptrMessage == NULL) return (FALSE);
    if (strlen(ptrMessage) > MAX_TX_BUFFERSIZE) return (FALSE);
    strcpy(USBTxBuffer, ptrMessage);
    length = strlen(USBTxBuffer);
    USBTxIndex = 0;
    CRCcalculate(USBTxBuffer, true);
    // printf("\rMessage: %s, length: %d", USBTxBuffer, USBTxIndex);
    dataReady = true;
    return(TRUE);
}
#else
unsigned char replyToHost(unsigned char *ptrMessage){
    if (ptrMessage == NULL) return (FALSE);
    if (strlen(ptrMessage) > MAX_TX_BUFFERSIZE) return (FALSE);
    strcpy(HOSTTxBuffer, ptrMessage);
    CRCcalculate(HOSTTxBuffer, true);
    sendToUART(HOSTuart, HOSTTxBuffer);    
    return(TRUE);
}
#endif

unsigned char sendToUART(unsigned char UartID, unsigned char *ptrUARTpacket)
{
    short i;
    unsigned char ch;

    if (strlen(ptrUARTpacket) < MAX_TX_BUFFERSIZE) {
        i = 0;
        do {
            ch = ptrUARTpacket[i++];
            if (!ch) break;
            while (!UARTTransmitterIsReady(UartID));
            UARTSendDataByte(UartID, ch);
        } while (i < MAX_TX_BUFFERSIZE);
        return (true);
    }
    else return (false);
}




/** EOF main.c *************************************************/

