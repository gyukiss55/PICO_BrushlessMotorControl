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

#include "user.h"

/******************************************************************************/
/* User Functions                                                             */
/******************************************************************************/

/* <Initialize variables in user.h and insert code for user algorithms.> */

#define F_CPU 16000000/4



static int baudrate2SPBRG (int baud_rate) {
    (int) (((float)(F_CPU)/(float)baud_rate)-1);
}

void InitApp(void)
{
    /* TODO Initialize User Ports/Peripherals/Project here */

    ADCON0bits.ADON = 0;
    
    LATA = 0;
    PORTA = 0;
    TRISA = 0xff;
    LATB = 0;
    PORTB = 0;
    TRISB = 0xff;
    LATC = 0;
    PORTC = 0;
    TRISC = 0xff;
    
    //TRISAbits.RA0 = 1; // External interrupt 0 (19)
    TRISAbits.RA1 = 1; // External interrupt 1 (18)
    TRISAbits.RA2 = 1; // External interrupt 2 (17)
    TRISAbits.RA4 = 0; // sw PWM (3)
    TRISAbits.RA5 = 0; // direction (2)
    TRISBbits.RB5 = 1; // UART RX (12)
    TRISBbits.RB7 = 0; // UART TX (10)
    //TRISCbits.RC3 = 0; // PWM out (7)
    
    TRISCbits.RC5 = 1; // CCP1/P1A input pin (5)
    
    //PORTBbits.RB5 = 1;
    //LATBbits.LATB5 = 1;
    
    //PORTBbits.RB7 = 0;
    //LATBbits.LATB7 = 0;
    
    TRISCbits.RC4 = 0; // debug pin (6)
    
    // Timer0 real time clock
    
    T0CONbits.TMR0ON=0;
    T0CONbits.T08BIT=0;
    T0CONbits.T0CS=0;
    T0CONbits.T0SE=0;
    T0CONbits.PSA=1;
    T0CONbits.T0PS=0;
    //TMR1=0xffff - 16000 + 1; // 16 MHz 16 * 1000 = 1 msec
    TMR0=0xffff - 16000 + 1; // 16 MHz 16 * 1000 = 1 msec

    INTCON2bits.TMR0IP = 1;
    INTCONbits.TMR0IF = 0;
    INTCONbits.TMR0IE = 1;
    T0CONbits.TMR0ON=1;    
    
    TMR3ON = 0;
    TMR3IP = 1;
    TMR3IF = 0;
    TMR3IE = 1;
    TMR3=0xffff - 160 + 1; // 16 MHz 16 * 100 = 100 usec
    TMR3ON = 1;
    
// PWM    
    /*
    
    PR2 = 200;
    TMR2 = 200;
    T2CONbits.T2CKPS = 0b11; // PWM
    
    
    T1CONbits.T1CKPS = 0b00;
    TMR1H = (16000 >> 8) & 0x00ff;
    TMR1L = 16000 & 0x00ff;
    PIR1bits.TMR1IF = 0;
    //PIE1bits.TMR1IE = 1;
    T1CONbits.TMR1ON = 1;
    
    T2CONbits.TMR2ON = 1;
    
    CCP1CONbits.CCP1M = 0b1100;
    CCP1CONbits.DC1B = 0b11;
    
    T3CONbits.TMR3ON = 1;

     */
    
// CCT1
    
    PIE1bits.CCP1IE = 0;
    IPR1bits.CCP1IP = 1; // 1 = High priority
    PIR1bits.CCP1IF = 0; //A TMR1 register capture occurred (must be cleared by software)
    
    CCP1CONbits.CCP1M = 0b0100;	// 0100 = Capture mode, every falling edge
                        // 0101 = Capture mode, every rising edge
    
    T3CONbits.T3CCP1 = 0;   // 1 = Timer3 is the clock source for compare/capture of ECCP1
					// 0 = Timer1 is the clock source for compare/capture of ECCP1
    T1CONbits.TMR1ON = 1;
    PIE1bits.CCP1IE = 1; // 1 = Enables the CCP1 interrupt 
    
// External interrupt 2 (RA2 17 pin))

   /**
    //INTCONbits.INT0P = 1; // high priority
    INTCONbits.INT0IF = 0; // interrupt flag
    INTCON2bits.INTEDG0 = 1;
    INTCONbits.INT0IE = 1; // interrupt enable
   */ 
    INTCON3bits.INT1P = 1; // high priority
    INTCON3bits.INT1IF = 0; // interrupt flag
    INTCON2bits.INTEDG1 = 1; //1 = Interrupt on rising edge
    INTCON3bits.INT1IE = 1; // interrupt enable
   /**
    INTCON3bits.INT2P = 1; // high priority
    INTCON3bits.INT2IF = 0; // interrupt flag
    INTCON2bits.INTEDG2 = 1;
    INTCON3bits.INT2IE = 1; // interrupt enable
    */
// UART    
    BAUDCONbits.BRG16 = 1; 
    //SPBRG=((16 MHz)/(64 �9600))-1 

    BRGH = 1;
    BRG16 = 1;
    SPBRGH = (((415 * 4) - 1) >> 8) & 0xff;
    SPBRG = (415 * 4) - 1;//baudrate2SPBRG (9600) ;
    TX9 = 0;
    SYNC = 0;
    SPEN = 1;
    RX9 = 0;
    CREN = 1;
    TXSTAbits.TXEN = 1;
    
    
    //PWM1CONbits.PRSEN = 0;
    //PWM1CONbits.PDC = 0;
    /* Setup analog functionality and port direction */

    /* Initialize peripherals */

    /* Configure the IPEN bit (1=on) in RCON to turn on/off int priorities */

    /* Enable interrupts */
    //INTCON2bits.INTEDG2 = 0; // External Interrupt 2 on falling edge
    
   /*
    *  INTCON3bits.INT2IP = 1;
    INTCON3bits.INT2IF = 0;
    INTCON3bits.INT2IE = 1; // Enables the INT2 external interrupt
    */
    
    RCONbits.IPEN = 1;
    INTCONbits.GIEH = 1; // Enables all unmasked interrupts
    INTCONbits.GIEL = 1; // Enables all high priority interrupts
}


char USART_ReceiveChar()
{
    while(RCIF==0);      /*wait for receive interrupt flag*/
    if(RCSTAbits.OERR)
    {           
        CREN = 0;
        asm("NOP");
        CREN=1;
    }
    return(RCREG);       /*received in RCREG register and return to main program */
}
