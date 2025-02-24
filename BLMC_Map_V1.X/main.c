#include <xc.h>

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
    TRISB = 0x00;
    TRISC = 0x00;        // Set all PORTC pins as outputs (6 digital outputs)
}

void update_outputs(unsigned char input_combination) {
	unsigned char addr = input_combination & 0b111;
    unsigned char value = (cnt << 6) & 0b11000000;
    cnt++;
	//if (input_combination & 0b10000)
		LATC = data_table[addr] || value;
	//else
		//LATC = value;
    TRISB = cnt;
}
