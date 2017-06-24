/*
 * main.h
 *
 *  Created on: Apr 14, 2014
 *      Author: michael
 */

#ifndef MAIN_H_
#define MAIN_H_

//--------------------------------------------------
// Global Defines
//--------------------------------------------------
// Soft I2C
#define PIN_SCL					PC3
#define PIN_SDA					PC2
// ADC, voltage measurement
#define VCAPDIVH				560.0		//[kOhm]
#define VCAPDIVL				160.0		//[kOhm]
#define	VCAP_CONV_FACT_MV		(uint16_t)( 10000.0 / 1023 * 1.1 * ( VCAPDIVL + VCAPDIVH ) / VCAPDIVL )	//to get [10*mV] from the ADC value
#define N_WAKEUPS_PER_ADC_MEAS	30			//Only measure voltage every 30 * 8 = 4 min if sleeping
#define ADC_MUX_VCAP			0x05
#define ADC_MUX_SOLAR			0x04
#define ADC_MUX_TEMP			0x08
#define ADC_MUX_VREF			0x0E
#define ADC_MUX_GND				0x0F
// Sleeping behaviour
#define SLEEP_THRESHOLD			2000		//[mV]
#define BLINKY_THRESHOLD		2500		//[mV]
#define FLAG_isRunning      	0			//PWM output is running
#define FLAG_wakeWDT 			1			//WDT timeout occurred (8 seconds passed)
#define FLAG_putNewFrame		2			//Tell main to render a new PWM frame

// Pseudo functions
#define SBI(reg, bit) ( reg |=  ( 1 << bit ) )
#define CBI(reg, bit) ( reg &= ~( 1 << bit ) )
#define IBI(reg, bit) ( (reg&(1<<bit)) != 0 )
#define SLEEP()				{ asm volatile("sleep"); }
#define SLEEP_SET_ADC()     { SMCR=0b00000011; }	//For ADC measurements
#define SLEEP_SET_IDLE()    { SMCR=0b00000001; }	//When T0 still needs to run
#define SLEEP_SET_STANDBY() { SMCR=0b00001101; }	//When only the crystal needs to run
#define SLEEP_SET_PDOWN() 	{ SMCR=0b00000101; }	//When everything except the WDT is off

//--------------------------------------------------
// Global Variables
//--------------------------------------------------
extern volatile uint8_t flags;

//--------------------------------------------------
// Functions
//--------------------------------------------------
void init(void);
void onWDT();

uint16_t readVCAP();
uint16_t readVSolar();
uint16_t readADC( uint8_t mux, uint8_t n );




#endif /* MAIN_H_ */
