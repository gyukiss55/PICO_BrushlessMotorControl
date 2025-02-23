/******************************************************************************/
/* Files to Include                                                           */
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

#define USARTBUFFERLENGTH  250

static uint8_t transmitBuffer[USARTBUFFERLENGTH];
static uint8_t transmitIndexBegin = 0;
static uint8_t transmitIndexEnd = 0;

void usartInit ()
{

}
    
void usartLoop (uint16_t timer3)
{
    if (transmitIndexBegin != transmitIndexEnd) {
        if (TXSTAbits.TRMT) {
            TXREG = transmitBuffer[transmitIndexBegin];
            ++transmitIndexBegin;
            if (transmitIndexBegin >= USARTBUFFERLENGTH)
                transmitIndexBegin = 0;
        }
    }
}

void printUSART (uint8_t ch) 
{
    transmitBuffer[transmitIndexEnd] = ch;
    ++transmitIndexEnd;
    if (transmitIndexEnd >= USARTBUFFERLENGTH)
        transmitIndexEnd = 0;       
}

uint8_t isUSARTEmpty () 
{
    return (transmitIndexEnd == transmitIndexBegin);
}


void printUSARTstr (char * ptr)
{
    while (*ptr) {
        printUSART (*ptr);
        ++ptr;
    }
}


void printUSARThexa (uint8_t v)
{
    v = v & 0xf;
    if (v < 10)
        printUSART (v + '0');
    else
        printUSART (v + 'A' - 10);
}


void printUSARTuint8 (uint8_t v)
{
    printUSARThexa (v >> 4);
    printUSARThexa (v);
}


void printUSARTuint16 (uint16_t v)
{
    printUSARTuint8 (v >> 8);
    printUSARTuint8 (v);
}
