/*
 * Freescale Cup linescan camera code
 *
 *	This method of capturing data from the line
 *	scan cameras uses a flex timer module, periodic
 *	interrupt timer, an ADC, and some GPIOs.
 *	CLK and SI are driven with GPIO because the FTM2
 *	module used doesn't have any output pins on the
 * 	development board. The PIT timer is used to 
 *  control the integration period. When it overflows
 * 	it enables interrupts from the FTM2 module and then
 *	the FTM2 and ADC are active for 128 clock cycles to
 *	generate the camera signals and read the camera 
 *  output.
 *
 *	PTB8		- camera CLK
 *	PTB23 		- camera SI
 *      ADC0_DP1 	- camera AOut
 *
 * Author:  Alex Avery
 * Modified: Abhimanyu Gutpa
 * Created:  11/20/15
 * Modified:  5/5/17
 */

#include "MK64F12.h"

#include "CameraControl.h"
#include "MotorControl.h"
#include "uart.h"
#include "stdio.h"

void LED_Init(void);
void Button_Init(void);

int i = 0;
int runstate = 0;
int configstate = 0;
void LED_Init(void){
	// Enable clocks on Ports B and E for LED timing
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTE_MASK;
	
	// Configure the Signal Multiplexer for GPIO
  PORTB_PCR22 = PORT_PCR_MUX(1);
	PORTB_PCR21 = PORT_PCR_MUX(1);
	PORTE_PCR26 = PORT_PCR_MUX(1);
	
	// Switch the GPIO pins to output mode
	GPIOB_PDDR |= (1<<21) | (1<<22);
	GPIOE_PDDR |= (1<<26);
	
	// Turn off the LEDs
  GPIOB_PDOR |= (1<<21) | (1<<22);
	GPIOE_PDOR |= (1<<26);
	
	// Turn on the Red LED
	GPIOB_PCOR |= (1<<22);
}

void Button_Init(void){
	// Enable clock for Port C PTC6 button      
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTA_MASK;
	
	// Configure the Mux for the buttons
	PORTC_PCR6 = PORT_PCR_MUX(1); // THIS MAY NEED TO CHANGE
  PORTA_PCR4 = PORT_PCR_MUX(1);

	// Set the SW2 as an input
	GPIOC_PDDR &= ~(1<<6);
	// Set the SW3 as an input
  GPIOA_PDDR &= ~(1<<4);
	
	PORTA_PCR4 |= PORT_PCR_IRQC(9);
	PORTC_PCR6 |= PORT_PCR_IRQC(11);
	
	// Enable Port C interrupts
	NVIC_EnableIRQ(PORTA_IRQn);
  NVIC_EnableIRQ(PORTC_IRQn);
}

void PORTA_IRQHandler(void){ //For switch 3
	// Clear the interrupt
	PORTA_PCR4 |= PORT_PCR_ISF_MASK;
	
	// If we are in the config state
	if(runstate == 1) {
		// Turn Off All LEDS
		GPIOB_PDOR |= (1<<21) | (1<<22);
		GPIOE_PDOR |= (1<<26);
		// Turn on Blue LED
		GPIOB_PCOR |= (1<<21);
		// Turn on Additional LEDS based on configstate
		if(configstate == 0) {
			// Set speed to 75
			setMaxMotorSpeed(70);
			// SET CONFIGURATIONS TO STATE 1
			configstate = 1;
		} else if (configstate == 1) {
			// Set speed to 50
			setMaxMotorSpeed(80);
			// Turn on Red LED
			GPIOB_PCOR |= (1<<22);
			// SET CONFIGURATIONS TO STATE 2
			configstate = 2;
		} else {
			// Set speed to 100
			setMaxMotorSpeed(90);
			// Turn on Green LED
			GPIOE_PCOR |= (1<<26);
			// SET CONFIGURATIONS TO STATE 0
			configstate = 0;
		}
	}
	return;
}

void PORTC_IRQHandler(void){ //For switch 2
	// Clear the Interrupt Flag
	PORTC_PCR6 |= PORT_PCR_ISF_MASK;
	// If Switch 2 was pressed
	if((GPIOC_PDIR&(1<<6))==0) {
		// Turn Off the LEDs
		GPIOB_PDOR |= (1<<21) | (1<<22);
		GPIOE_PDOR |= (1<<26);
		
		// If we are in runstate 0
		if(runstate == 0) {
			// Turn on the Blue LED
			GPIOB_PCOR |= (1<<21);
			// Move to the Configuration State
			runstate = 1;
		} else if (runstate == 1){
			// Turn on the Green LED
			GPIOE_PCOR |= (1<<26);
			// Move to the Go! State
			runstate = 2;
		} else {
			// The Car has already escaped our hands
		}
	}
	return;
}

int main(void){
	// Initialize variables
	float offset;
	float turn;
	// Initialize the LEDs
	LED_Init();
	// Initialize the Buttons
	Button_Init();
	// Initialize the Motors
	Init_Motors();
	// While we aren't going
	while(runstate != 2){}
	// Initialize the Camera
	Init_Camera();
	// Do Forever
	for(;;) {
	} //for
} //main