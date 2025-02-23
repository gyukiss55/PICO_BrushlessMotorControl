/******************************************************************************/
/*Files to Include                                                            */
/******************************************************************************/

#if defined(__XC)
    #include <xc.h>         /* XC8 General Include File */
#elif defined(HI_TECH_C)
    #include <htc.h>        /* HiTech General Include File */
#elif defined(__18CXX)
    #include <p18cxxx.h>    /* C18 General Include File */
#endif

#if defined(__XC) || defined(HI_TECH_C)

#include <stdint.h>         /* For uint8_t definition */
#include <stdbool.h>        /* For true/false definition */

#endif

#include "usart.h"

#define DCC_LOGICAL_DEV         100
#define DCC_LOGICAL_HIGH_MIN ((2 * 58 * 16) - DCC_LOGICAL_DEV)
#define DCC_LOGICAL_HIGH_MAX ((2 * 58 * 16) + DCC_LOGICAL_DEV)

#define DCC_LOGICAL_LOW_MIN ((2 * 100 * 16) - DCC_LOGICAL_DEV)
#define DCC_LOGICAL_LOW_MAX ((2 * 100 * 16) + DCC_LOGICAL_DEV)

#define DCC_LOGICAL_MID_MIN (((100 + 58) * 16) - DCC_LOGICAL_DEV)
#define DCC_LOGICAL_MID_MAX (((100 + 58) * 16) + DCC_LOGICAL_DEV)

#include "user.h"

/******************************************************************************/
/* Interrupt Routines                                                         */
/******************************************************************************/

/* High-priority service */

#define CCT1BUFFERLENGTH 32

uint8_t intLowCnt = 0; 
uint8_t intHighCnt = 0; 

static uint8_t pwmValue = 0; 
static uint8_t pwmStatus = 0; 

static uint8_t timer0CounterLow = 0; 
static uint8_t timer0CounterHigh = 0; 
static uint16_t cct1Value = 0;
static uint16_t cct1ValuePrev = 0;
static uint16_t cct1diffValue;
static uint16_t cct1diffCounter = 0;

static uint8_t dccLevel = 0;
static uint8_t dccMidLevelCounter = 0;
static uint8_t dccStatus = 0;
// 0 - pre preamble
// 1 - preamble DCC_LOGICAL_HIGH dccStatusPhase 12
// 2 - start 8 bit DCC packet data DCC_LOGICAL_LOW dccStatusPhase 1
// 3 - 8 bit DCC packet data dccStatusPhase 8
// 4 - 8 bit DCC packet data DCC_LOGICAL_HIGH dccStatusPhase 1
static uint8_t dccStatusPhase = 0;
static uint8_t dccPacketData[8];
static uint8_t dccPacketDataIndex = 0;
static uint8_t dccPacketDataReady[8];
static uint8_t dccPacketDataReadyCnt = 0;



#if defined(__XC) || defined(HI_TECH_C)
void __interrupt(high_priority) high_isr(void)
#elif defined (__18CXX)
#pragma code high_isr=0x08
#pragma interrupt high_isr
void high_isr(void)
#else
#error "Invalid compiler selection for implemented ISR routines"
#endif

{
    ++intHighCnt;
    LATCbits.LATC4 = !LATCbits.LATC4;
    if(INTCONbits.TMR0IF) {
        //LATCbits.LATC4 = !LATCbits.LATC4;  
        TMR0=0xffff - 16000 + 1; // 16 MHz 16 * 1000 = 1 msec

        ++timer0CounterLow;
        if (timer0CounterLow == 0)
            ++timer0CounterHigh;
          
        INTCONbits.TMR0IF = 0;
    }
    
    if(PIR2bits.TMR3IF) {
        TMR3=0xffff - 1600 + 1; // 16 MHz 16 * 100 = 100 usec
        --pwmStatus;
        if (pwmStatus == 0)
            LATAbits.LATA4 = 0;
        else if (pwmStatus == pwmValue)
            LATAbits.LATA4 = 1;
        PIR2bits.TMR3IF = 0;
    }
    
    if (INTCON3bits.INT1IF) {
        LATAbits.LATA4 = ~LATAbits.LATA4;
        INTCON3bits.INT1IF = 0;
    }

    if (PIR1bits.CCP1IF) {
        
        cct1Value = CCPR1;
        if (cct1Value > cct1ValuePrev)
            cct1diffValue = cct1Value - cct1ValuePrev;
        else
            cct1diffValue = cct1Value + (0xFFFF - cct1ValuePrev) + 1;
        cct1ValuePrev = cct1Value;
        ++cct1diffCounter;
        if (cct1diffValue >= DCC_LOGICAL_HIGH_MIN && cct1diffValue <= DCC_LOGICAL_HIGH_MAX)
            dccLevel = 1;
        else if (cct1diffValue >= DCC_LOGICAL_LOW_MIN && cct1diffValue <= DCC_LOGICAL_LOW_MAX)
            dccLevel = 0;
        else if (cct1diffValue >= DCC_LOGICAL_MID_MIN && cct1diffValue <= DCC_LOGICAL_MID_MAX) {
            dccLevel = 2;
            ++dccMidLevelCounter;
            if (dccMidLevelCounter > 20) {
                dccMidLevelCounter = 0;
                CCP1CONbits.CCP1M0 = ~CCP1CONbits.CCP1M0;
            }
            dccStatus = 0;
        } else {
            dccLevel = 3;
            dccStatus = 0;
        }
        switch (dccStatus) {
// 0 - pre preamble
// 1 - preamble DCC_LOGICAL_HIGH dccStatusPhase 12
// 2 - start 8 bit DCC packet data DCC_LOGICAL_LOW dccStatusPhase 1
// 3 - 8 bit DCC packet data dccStatusPhase 8
// 4 - 8 bit DCC packet data DCC_LOGICAL_HIGH dccStatusPhase 1
            case 0:
                if (dccLevel == 1) {
                    dccStatus = 1;
                    dccStatusPhase = 1;
                } else
                    dccStatus = 0;
                break;
                
            case 1:
                if (dccLevel == 1) {
                    if (dccStatusPhase < 12)
                        ++dccStatusPhase;
                } else if (dccLevel == 0) {
                    if (dccStatusPhase < 12)
                        dccStatus = 0;
                    else {
                        dccStatus = 3;
                        dccStatusPhase = 0;
                        dccPacketDataIndex = 0;
                    }
                }
                    
                break;
                
            case 2:
                if (dccLevel == 0) {
                    if (dccPacketDataIndex <= 6) {
                        dccStatus = 3;
                        dccStatusPhase = 0;
                    } else
                        dccStatus = 0;
                    
                } else if (dccLevel == 1) {
                    if (dccPacketDataIndex >=3) {
                        dccPacketDataReady[0] = dccPacketDataIndex;
                        for (uint8_t i = 0; i < dccPacketDataIndex; ++i)
                            dccPacketDataReady[i+1] = dccPacketData[i];
                        ++dccPacketDataReadyCnt;
                    }
                    dccStatus = 0;
                } 
                break;
                
            case 3:
                if (dccStatusPhase == 0) 
                    dccPacketData[dccPacketDataIndex] = 0;
                else
                    dccPacketData[dccPacketDataIndex] = dccPacketData[dccPacketDataIndex] << 1;
                if (dccLevel == 1) {
                    dccPacketData[dccPacketDataIndex] |= 1;
                }
                ++dccStatusPhase;
                if (dccStatusPhase == 8) {
                    dccStatus = 2;
                    ++dccPacketDataIndex;
                    dccStatusPhase = 0;
                }
                break;
                
            default:
                dccStatus = 0;
                break;
        }

        
        LATCbits.LATC4 = !LATCbits.LATC4;
        PIR1bits.CCP1IF = 0;
    }

}


uint8_t GetTimerCounterLow ()
{
    return timer0CounterLow;
}

uint8_t GetTimerCounterHigh ()
{
    return timer0CounterHigh;
}

uint8_t PrintDCCStatus ()
{
   if (dccPacketDataIndex > 0) {
        printUSARTstr ("DCC packet:");
        if (dccPacketDataReady[0] > 0 && dccPacketDataReady[0] < 7){
            for (uint8_t i = 0; i <= dccPacketDataReady[0]; ++i) {
                printUSART(' ');
                printUSARTuint8 (dccPacketDataReady[i]);
            }
            printUSARTstr ("\r\n");
        }
    }
    return dccPacketDataReadyCnt;
  
}

uint8_t GetLastDCCPacket (uint8_t* data)
{
    if (dccPacketDataIndex > 0) {
         if (dccPacketDataReady[0] > 0 && dccPacketDataReady[0] < 7){
            for (uint8_t i =0; i <= dccPacketDataReady[0] + 1; ++i) {
                data[i] = dccPacketDataReady[i];
            }
        }
    }
    return dccPacketDataReadyCnt;
}

uint8_t SetDirSpeed (uint8_t d, uint8_t s)
{
    if (s == 1)
        pwmValue = 0;
    else
        pwmValue = s;
    if (d)
        LATAbits.LATA5 = 1;
    else
        LATAbits.LATA5 = 0;
    return pwmValue;    
}
