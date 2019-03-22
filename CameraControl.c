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
 * Modified: Abhimanyu Gupta
 * Created:  11/20/15
 * Modified:  5/5/17
 */

#include "MK64F12.h"
#include "CameraControl.h"
#include "uart.h"
#include "stdio.h"
#include "motorControl.h"

// Default System clock value
// period = 1/20485760  = 4.8814395e-8
#define DEFAULT_SYSTEM_CLOCK 20485760u 
// Integration time (seconds)
// Determines how high the camera values are
// Don't exceed 100ms or the caps will saturate
// Must be above 1.25 ms based on camera clk 
//	(camera clk is the mod value set in FTM2)
#define INTEGRATION_TIME .0075f

int motorFrequency = 10000;
int time = 0;

// Pixel counter for camera logic
// Starts at -2 so that the SI pulse occurs
//		ADC reads start
int pixcnt = -2;
// clkval toggles with each FTM interrupt
int clkval = 0;
// line stores the current array of camera data
uint16_t line[128];
uint16_t smooth[128];
int diff[128];
uint16_t thresh[128];
// Distances to the nearest dips
float offset = 0;

// These variables are for streaming the camera
//	 data over UART
int debugcamdata = 0;
int capcnt = 0;
char str[100];

// PID Control Variables
float servoError = 0.0f;
float oldServoError = 0.0f;
float servoDesired = 7.5f;
float servoActual = 7.5f;
float leftMotorError = 0.0f;
float oldLeftMotorError = 0.0f;
float leftMotorDesired = 50.0f;
float leftMotorActual = 50.0f;
float rightMotorError = 0.0f;
float oldRightMotorError = 0.0f;
float rightMotorDesired = 50.0f;
float rightMotorActual = 50.0f;
float kp = 0.55; float ki = 0.1; float kd = 0.25;
int maxspeed = 75;

unsigned int turning = 0;
unsigned int braking = 0;
unsigned int braketime = 0;

// ADC0VAL holds the current ADC value
uint16_t ADC0VAL;

void Init_Camera(void)
{
	init_GPIO(); // For CLK and SI output on GPIO
	init_FTM2(); // To generate CLK, SI, and trigger ADC
	init_ADC0(); // For Camera Output
	init_PIT();	 // To trigger camera read based on integration time
} //main

/* Smooth the line read using a moving window average of size 3*/
void Smooth_Line(void) {
	int i;
	// First pixel is the same
	smooth[0] = line[0];
	// Last Pixel is the same
	smooth[127] = line[127];
	// For all pixels inbetween
	for(i = 1; i < 127; i++) {
		// Take the moving average
		if (i == 1 || i == 126) {
			smooth[i] = (((line[i-1])*0.3)+((line[i])*0.4)+((line[i+1])*0.3));
		} else {
			smooth[i] = (((line[i-2])*0.1)+((line[i-1])*0.225)+((line[i])*0.35)+((line[i+1])*0.225)+((line[i+2])*0.1));
		}
	}
}

void Diff_Line(void) {
	int i;
	int minloc = 0;
	int maxloc = 0;
	int min = 0;
	int max = 0;
	int topspeed = 20;
	int wheeloff = 0;
  float turn;
	int zero = 0;
  int a_value = 0;
	
	
	diff[0] = 0;
	for(i = 1; i < 127; i++) {
		diff[i] = smooth[i-1] - smooth[i+1];
		if(diff[i] < min) {
			min = diff[i];
			minloc = i;
		}
		if(diff[i] > max){
			max = diff[i];
			maxloc = i;
		}	
	}
	diff[127] = 0;
	offset = (((maxloc + minloc)/2)-64.0f)/32.0f;

	turn = (2.5*offset);
	servoDesired = 7.5+turn;
	
	topspeed = maxspeed*(1-offset*offset);
  wheeloff = offset*(maxspeed - topspeed);
	rightMotorDesired = topspeed+wheeloff; leftMotorDesired = topspeed-wheeloff;

}


/* ADC0 Conversion Complete ISR  */
void ADC0_IRQHandler(void) {
	// Clear FTM2 External Trigger Flag
	int temp = FTM2_EXTTRIG;
	FTM2_EXTTRIG &= ~FTM_EXTTRIG_TRIGF_MASK;
	// Reading ADC0_RA clears the conversion complete flag
	ADC0VAL = ADC0_RA;	
}

/* 
* FTM2 handles the camera driving logic
*	This ISR gets called once every integration period
*		by the periodic interrupt timer 0 (PIT0)
*	When it is triggered it gives the SI pulse,
*		toggles clk for 128 cycles, and stores the line
*		data from the ADC into the line variable
*/
void FTM2_IRQHandler(void){ //For FTM timer
	// Clear interrupt
  FTM2_SC |= FTM_SC_TOF_MASK;
	
	// Toggle clk
	GPIOB_PTOR |= (1<<9);
	
	// Line capture logic
	if ((pixcnt >= 2) && (pixcnt < 256)) {
		if (!clkval) {	// check for falling edge
			// ADC read (note that integer division is 
			//  occurring here for indexing the array)
			line[pixcnt/2] = ADC0VAL;
		}
		pixcnt += 1;
	} else if (pixcnt < 2) {
		if (pixcnt == -1) {
			GPIOB_PSOR |= (1 << 23); // SI = 1
		} else if (pixcnt == 1) {
			GPIOB_PCOR |= (1 << 23); // SI = 0
			// ADC read
			line[0] = ADC0VAL;
		} 
		pixcnt += 1;
	} else {
		GPIOB_PCOR |= (1 << 9); // CLK = 0
		clkval = 0; // make sure clock variable = 0
		pixcnt = -2; // reset counter
		// Disable FTM2 interrupts (until PIT0 overflows
		//   again and triggers another line capture)
		FTM2_SC &= ~(FTM_SC_TOIE_MASK);
		// Smooth the Result
		Smooth_Line();
		// Differentiate the Smooth Result
		Diff_Line();
		// Thresh the Differentiated Result
		//Thresh_Line();
	}
	
	// Perform PID
	// Set the old errors
	//oldServoError = servoError;
	oldLeftMotorError = leftMotorError;
	oldRightMotorError = rightMotorError;
	// Calculate the new errors
	//servoError = servoDesired - servoActual;
	leftMotorError = leftMotorDesired - leftMotorActual;
	rightMotorError = rightMotorDesired - rightMotorActual;
	// Calculate new servoActual
	//servoActual = servoActual + kp*(servoError-oldServoError) + ki*servoError + kd*(servoError-oldServoError);
	// Calculate the new leftMotorActual
	leftMotorActual = leftMotorActual + kp*(leftMotorError-oldLeftMotorError) + ki*leftMotorError + kd*(leftMotorError-oldLeftMotorError);
	// Calculate the new rightMotorActual
	rightMotorActual = rightMotorActual + kp*(rightMotorError-oldRightMotorError) + ki*rightMotorError + kd*(rightMotorError-oldRightMotorError);
	// Set the Values
	//SetServoDutyCycle(servoActual,50);
  if((offset*offset) < 0.07) {
    SetRightMotorDutyCycle(rightMotorActual,motorFrequency,1);
    SetLeftMotorDutyCycle(leftMotorActual,motorFrequency,1);
  } else if ((offset*offset) < 0.1 && (offset*offset) > 0.07 ) {
    //if(braketime > 0) {
      SetRightMotorDutyCycle(leftMotorActual,motorFrequency,0);
      SetLeftMotorDutyCycle(rightMotorActual,motorFrequency,0);
		
      //braketime--;
		SetRightMotorDutyCycle(rightMotorActual,motorFrequency,1);
    SetLeftMotorDutyCycle(leftMotorActual,motorFrequency,1);
    }

		else if ((offset*offset) < 0.25 && (offset*offset) > 0.1 ) {
			SetRightMotorDutyCycle(rightMotorActual,motorFrequency,0);
      SetLeftMotorDutyCycle(leftMotorActual,motorFrequency,0);

			SetRightMotorDutyCycle(rightMotorActual,motorFrequency,1);
			SetLeftMotorDutyCycle(leftMotorActual,motorFrequency,1);
		}	
		else {
			SetRightMotorDutyCycle(rightMotorActual,motorFrequency,1);
			SetLeftMotorDutyCycle(leftMotorActual,motorFrequency,1);
    }
		return;
	}
}

/* PIT0 determines the integration period
*		When it overflows, it triggers the clock logic from
*		FTM2. Note the requirement to set the MOD register
* 	to reset the FTM counter because the FTM counter is 
*		always counting, I am just enabling/disabling FTM2 
*		interrupts to control when the line capture occurs
*/
void PIT0_IRQHandler(void){
	if (debugcamdata) {
		// Increment capture counter so that we can only 
		//	send line data once every ~2 seconds
		capcnt += 1;
	}
	// Clear interrupt
	PIT_TFLG0 |= PIT_TFLG_TIF_MASK;
	
	// Setting mod resets the FTM counter
	FTM2_MOD = (DEFAULT_SYSTEM_CLOCK/1000000);
	
	// Enable FTM2 interrupts (camera)
	FTM2_SC |= FTM_SC_TOIE_MASK;

	return;
}


/* Initialization of FTM2 for camera */
void init_FTM2(){
	// Enable clock
	SIM_SCGC6 |= SIM_SCGC6_FTM2_MASK;
	
	// Disable Write Protection
	FTM2_MODE |= FTM_MODE_WPDIS_MASK;
	
	// Set output to '1' on init
	FTM2_OUTINIT |= FTM_OUTINIT_CH0OI_MASK;
	
	// Initialize the CNT to 0 before writing to MOD
	FTM2_CNT = 0;
	
	// Set the Counter Initial Value to 0
	FTM2_CNTIN = 0;
	
	// Set the period (~10us)
	FTM2_MOD |= FTM_MOD_MOD(DEFAULT_SYSTEM_CLOCK/1000);
	
	// 50% duty
	FTM2_C0V |= FTM_CnV_VAL(DEFAULT_SYSTEM_CLOCK/2000);
	
	// Set edge-aligned mode
	FTM2_C0SC |= FTM_CnSC_MSB_MASK;
	
	// Enable High-true pulses
	// ELSB = 1, ELSA = 0
	FTM2_C0SC |= FTM_CnSC_ELSB_MASK & ~FTM_CnSC_ELSA_MASK;
	
	// Enable hardware trigger from FTM2
	FTM2_EXTTRIG |= FTM_EXTTRIG_INITTRIGEN_MASK | FTM_EXTTRIG_CH0TRIG_MASK;
	
	// Don't enable interrupts yet (disable)
	FTM2_SC &= ~FTM_SC_TOIE_MASK;
	
	// No prescalar, system clock
	FTM2_SC |= FTM_SC_PS(0) | FTM_SC_CLKS(1);
	
	// Set up interrupt
	NVIC_EnableIRQ(FTM2_IRQn);
	
	return;
}

/* Initialization of PIT timer to control 
* 		integration period
*/
void init_PIT(void){
	// Setup periodic interrupt timer (PIT)
	// Enable clock for timers
	SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;
	PIT_MCR &= ~PIT_MCR_MDIS_MASK;
	
	// Enable timers to continue in debug mode
	PIT_MCR &= ~PIT_MCR_FRZ_MASK; // In case you need to debug
	
	// PIT clock frequency is the system clock
	// Load the value that the timer will count down from
	PIT_LDVAL0 = DEFAULT_SYSTEM_CLOCK*INTEGRATION_TIME;
	
	// Enable timer interrupts
	PIT_TCTRL0 |= PIT_TCTRL_TIE_MASK;
	
	// Enable the timer
	PIT_TCTRL0 |= PIT_TCTRL_TEN_MASK;

	// Clear interrupt flag
	PIT_TFLG0 |= PIT_TFLG_TIF_MASK; 
	
	// Enable PIT interrupt in the interrupt controller
	NVIC_EnableIRQ(PIT0_IRQn); //not sure 
	
	return;
}

/* Set up pins for GPIO
* 	PTB9 		- camera clk
*		PTB23		- camera SI
*		PTB22		- red LED
*/
void init_GPIO(void){
	// Enable LED and GPIO so we can see results
	// initialize clock for all parts
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	// Configure Signal Multiplexer for GPIO
	PORTB_PCR9 = PORT_PCR_MUX(1);  // Camera Clock
	PORTB_PCR23 = PORT_PCR_MUX(1); // Camera SI
	PORTB_PCR22 = PORT_PCR_MUX(1); // Red LED
	// Switch Pins to Ouput mode
	GPIOB_PDDR |= (1<<9) | (1<<22) | (1<<23);
	// Turn Off Everything
  GPIOB_PDOR |= (1<<9) | (1<<22) | (1<<23);
	return;
}

/* Set up ADC for capturing camera data */
void init_ADC0(void) {
	unsigned int calib;
  // Turn on ADC0
	SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK;
	ADC0_SC1A &= ~ADC_SC1_ADCH_MASK;
	ADC0_SC1A |= ADC_SC1_ADCH(0);
	ADC0_SC1A |= ADC_SC1_AIEN_MASK;
	
	// Single ended 16 bit conversion, no clock divider
	ADC0_CFG1 |= ADC_CFG1_ADIV(0) | ADC_CFG1_MODE(3);
    
  // Do ADC Calibration for Singled Ended ADC. Do not touch.
  ADC0_SC3 = ADC_SC3_CAL_MASK;
  while ( (ADC0_SC3 & ADC_SC3_CAL_MASK) != 0 );
  calib = ADC0_CLP0; calib += ADC0_CLP1; calib += ADC0_CLP2;
  calib += ADC0_CLP3; calib += ADC0_CLP4; calib += ADC0_CLPS;
  calib = calib >> 1; calib |= 0x8000;
  ADC0_PG = calib;
    
  // Select hardware trigger.
  ADC0_SC2 |= ADC_SC2_ADTRG_MASK;
    
  // Set to single ended mode	
	ADC0_SC1A &= ~ADC_SC1_DIFF_MASK;
	
	// Set up FTM2 trigger on ADC0
	SIM_SOPT7 |= SIM_SOPT7_ADC0TRGSEL(10); // FTM2 select
	SIM_SOPT7 |= SIM_SOPT7_ADC0ALTTRGEN_MASK; // Alternative trigger en.
	SIM_SOPT7 &= ~SIM_SOPT7_ADC0PRETRGSEL_MASK; // Pretrigger A
	
	// Enable NVIC interrupt
  NVIC_EnableIRQ(ADC0_IRQn);
}

void setMaxMotorSpeed(int speed) {
	maxspeed = speed;
}