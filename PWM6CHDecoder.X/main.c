#include <xc.h>

// Configuration bits
#pragma config FCMEN = OFF
#pragma config IESO = OFF
#pragma config MCLRE = ON
#pragma config WDTEN = OFF

// Define configuration bits for internal oscillator at maximum frequency
// #pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator; CLKOUT function disabled)
// #pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)

// Define constants for port configurations
// #define _XTAL_FREQ 16000000     // Assuming max internal clock is set to 16 MHz

#define _XTAL_FREQ 8000000  // Set the oscillator frequency to 8 MHz

const unsigned char data_table[8] = {
    0x00, 
    0b010010, // AH CL
    0b011000, // BH CL
    0b001001, // BH AL
    0b100001, // CH AL
    0b100100, // CH BL
    0b000110, // AH BL
	0x00 
};

unsigned char cnt;

// Function prototypes
void init_ports(void);
void update_outputs(unsigned char input_combination);

void main(void) {
    init_ports();  // Initialize I/O ports
    
    while (1) {
        unsigned char input_combination = PORTA;  // Read RA0, RA1, RA2, and RA4 as inputs
        update_outputs(input_combination);  // Update outputs based on input combination
    }
}

void init_ports(void) {
    TRISA = 0b11111111;  // Set RA0, RA1, RA2, and RA4 as inputs (1), others as outputs (0)
    TRISC = 0x00;        // Set all PORTC pins as outputs (6 digital outputs)
    
    //ANSELA = 0x00;       // Set all pins on PORTA to digital mode
    //ANSELC = 0x00;       // Set all pins on PORTC to digital mode
}

void update_outputs(unsigned char input_combination) {
	unsigned char addr = input_combination & 0b111;
    unsigned char value = (cnt << 6) & 0b11000000;
	if (input_combination & 0b10000)
		LATC = data_table[addr] || value;
	else
		LATC = value;
}

#if defined _use_update_outputs_v0_
void update_outputs_v0(unsigned char input_combination) {
    switch (input_combination) {
        // case 0b00000: LATC = 0b000000; break; // All outputs off
        // case 0b00001: LATC = 0b000000; break; // All outputs off
        // case 0b00010: LATC = 0b000000; break; // All outputs off
        // case 0b00011: LATC = 0b000000; break; // All outputs off
        // case 0b00100: LATC = 0b000000; break; // All outputs off
        // case 0b00101: LATC = 0b000000; break; // All outputs off
        // case 0b00110: LATC = 0b000000; break; // All outputs off
        // case 0b00111: LATC = 0b000000; break; // All outputs off
        // case 0b10000: LATC = 0b000000; break; // All outputs off
        case 0b10001: LATC = 0b010010; break; // AH CL
        case 0b10010: LATC = 0b011000; break; // BH CL
        case 0b10011: LATC = 0b001001; break; // BH AL
        case 0b10100: LATC = 0b100001; break; // CH AL
        case 0b10101: LATC = 0b100100; break; // CH BL
        case 0b10110: LATC = 0b000110; break; // AH BL
        // case 0b10111: LATC = 0b000000; break; // All outputs off
        default: LATC = 0b000000; break; // All outputs off
    }
}
#endif


