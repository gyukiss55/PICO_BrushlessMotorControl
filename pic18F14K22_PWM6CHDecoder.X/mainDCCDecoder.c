/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

#if defined(__XC)
    #include <xc.h>        /* XC8 General Include File */
#elif defined(HI_TECH_C)
    #include <htc.h>       /* HiTech General Include File */
#elif defined(__18CXX)
    #include <p18cxxx.h>   /* C18 General Include File */
#endif

#if defined(__XC) || defined(HI_TECH_C)

#include <stdint.h>        /* For uint8_t definition */
#include <stdbool.h>       /* For true/false definition */

#endif

#include "system.h"        /* System funct/params, like osc/peripheral config */
#include "user.h"          /* User funct/params, such as InitApp */
#include "usart.h" 
#include "interrupts.h" 

extern uint8_t intLowCnt; 
extern uint8_t intHighCnt; 

/******************************************************************************/
/* User Global Variable Declaration                                           */
/******************************************************************************/

/* i.e. uint8_t <variable_name>; */

/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/

//_INT2Interrupt
//void __attribute__((interrupt,auto_psv)) _INT2Interrupt(void)
//{
    
//}

//_T2Interrupt
//void __attribute__((interrupt,auto_psv)) _T2Interrupt(void)
//{
    
//}

// DELAY_XX 2 == 5 usec // 400 = 1 msec
#define DELAY_1mSec 400
#define DELAY_10uSec 4

#define EEPROM_MANUFACTIDL 0
#define EEPROM_MANUFACTIDH 1
#define EEPROM_MANUFACTVERSION 2    
#define EEPROM_LOCALADDRESS 3
#define EEPROM_ACCSPEEDSTEP 4
#define EEPROM_ACCSPEEDTIME 5
#define EEPROM_DECSPEEDSTEP 6
#define EEPROM_DECSPEEDTIME 7

void DelayMS (uint16_t ms)
{
    for (uint16_t j = 0; j <ms; ++j)
    {
        for (uint16_t i = 0; i <DELAY_1mSec; ++i)
           asm ("NOP");
    }
}

void Delay10US (uint16_t us)
{
    for (uint16_t j = 0; j < us; ++j)
    {
        for (uint8_t i = 0; i <DELAY_10uSec; ++i)
           asm ("NOP");
    }
}

unsigned char readEEPROM(unsigned char address)
{
    EEADR = address; //Address to be read
    EECON1bits.EEPGD = 0; //Selecting EEPROM Data Memory
    EECON1bits.RD = 1; //Initialise read cycle
    return EEDATA; //Returning data
}

void writeEEPROM(unsigned char address, unsigned char data)
{
    unsigned char INTCON_SAVE; //To save INTCON register value
    EEADR = address; //Address to write
    EEDATA = data; //Data to write
    EECON1bits.EEPGD = 0; //Selecting EEPROM Data Memory
    EECON1bits.WREN = 1; //Enable writing of EEPROM
    INTCON_SAVE=INTCON; //Backup INCON interupt register
    INTCON=0; //Diables the interrupt
    EECON2=0x55; //Required sequence for write to internal EEPROM
    EECON2=0xAA; //Required sequence for write to internal EEPROM
    EECON1bits.WR = 1; //Initialise write cycle
    INTCON = INTCON_SAVE; //Enables Interrupt
    EECON1bits.WREN = 0; //To disable write
    while(PIR2bits.EEIF == 0); //Checking for complition of write operation
    PIR2bits.EEIF = 0; //Clearing EEIF bit
}

void CV_Modify (uint8_t cvNum, uint8_t cvVal)
{
    writeEEPROM (cvNum, cvVal);
}



static uint8_t dccPacket[8];
static uint8_t dccPacketCnt = 0;

static uint8_t dccManufactIDLow = 1955 & 0xFF;
static uint8_t dccManufactIDHigh = 1955 >> 8;
static uint8_t dccManufactVersion = 3;

static uint8_t dccLocalAddr = 3;
static uint8_t dccAccSpeedStep = 1;
static uint8_t dccAccSpeedTime = 10;
static uint8_t dccDecSpeedStep = 1;
static uint8_t dccDecSpeedTime = 10;

void main(void)
{
    /* Configure the oscillator for the device */
    ConfigureOscillator();
    
    /* Initialize I/O and Peripherals for application */
    InitApp();
    
    usartInit ();
            
    uint8_t t0prev = 0;
    uint16_t timer3Cnt = 0;
    uint16_t timer3Prev = 0;
    
    uint8_t pa02 = 0;
    uint8_t pa02Prev = 0;
    while(1)
    {
        uint16_t timer3 = (uint16_t)GetTimerCounterLow () + ((uint16_t)GetTimerCounterHigh () << 8);
        if (timer3Prev > timer3)
            ++timer3Cnt;
        if (timer3Prev != timer3) {
            uint8_t printFlag = 0;
            dccPacketCnt = GetLastDCCPacket (dccPacket);
            if (isUSARTEmpty ()) {
                PrintDCCStatus ();
                printFlag = 1;
            }
            
            uint8_t crc = 0;
            uint8_t dir = 0;
            for (uint8_t i = 1; i < dccPacket[0]; ++i)
                crc ^= dccPacket[i];

            if (crc == dccPacket[dccPacket[0]]) {
                if ((dccPacket[0] == 3) && ((dccPacket[1] == dccLocalAddr) || (dccPacket[1] == 0))) {
                    if ((dccPacket[2] & 0xC0) == 0x40) { // speed control
                        uint8_t s = (dccPacket[2] & 0xF) << 4;
                        if ((dccPacket[2] & 0xF) == 1)
                            s = 1;
                        if (dccPacket[2] & 0x20)
                            dir = 1;
                        if (printFlag) {
                            printUSARTstr ("DirSpeed: ");
                            printUSARTuint8 (dir);
                            printUSART (' ');
                            printUSARTuint8 (s);
                            printUSARTstr ("\r\n");
                        }
                        SetDirSpeed (dir, s);
                    }
                }
                if ((dccPacket[0] == 4) && ((dccPacket[1] == dccLocalAddr) || (dccPacket[1] == 0))) {
                    if (dccPacket[2] == 0x3F) { // speed control
                        uint8_t s = dccPacket[3] & 0x7F;
                        if (dccPacket[3] & 0x80)
                            dir = 1;
                        if (s > 1)
                            s = s << 1;
                        if (printFlag) {
                            printUSARTstr ("DirSpeed: ");
                            printUSARTuint8 (dir);
                            printUSART (' ');
                            printUSARTuint8 (s);
                            printUSARTstr ("\r\n");
                        }
                        SetDirSpeed (dir, s);
                    }
                }
                
                if ((dccPacket[0] == 5) && ((dccPacket[1] == dccLocalAddr) || (dccPacket[1] == 0))) {
                    if (dccPacket[2] == 0xEC) { // CV modify
                        uint8_t cvNum = dccPacket[3] + 1;
                        uint8_t cvVal = dccPacket[4];
                        CV_Modify (cvNum, cvVal);
                    }
                }
            }

            timer3Prev = timer3;
            usartLoop (timer3);
        }

        uint8_t t0 = GetTimerCounterLow ();
        if (t0 != t0prev) {
            //LATCbits.LATC4 = !LATCbits.LATC4;
            t0prev = t0;
        }
        
        pa02 = PORTAbits.RA2;
        if (pa02Prev != pa02) {
            pa02Prev = pa02;
            LATCbits.LATC4 = !LATCbits.LATC4;
        }
        if (isUSARTEmpty ()) {
            PrintDCCStatus ();
        }
    }

}

