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
// GPIO
#define PIN_BTN1 		PD0
#define PIN_UART_TX		PD1
#define PIN_BTN2 		PD2
#define PIN_DISP_RST 	PD3
#define PIN_DISP_DC 	PD4	// Data / Command
#define PIN_DISP_CS 	PD5
#define PIN_PUMP_DAT 	PD6
#define PIN_PUMP_CLK 	PD7
#define PIN_SS_REL 		PB0
#define PIN_1WIRE_DAT 	PB1
#define PIN_NRF_CSN		PB2
#define PIN_SPI_MOSI	PB3
#define PIN_SPI_MISO	PB4
#define PIN_SPI_SCK		PB5
#define PIN_NRF_CE 		PC0
#define PIN_I2C_SDA 	PC4
#define PIN_I2C_SCL 	PC5

// Constants
#define BAUD 			38400	// Serial baudrate for user interface
#define TPIC_DELAY_US 	1
#define ADC_AVG_FACT 	6		//Sample 2^N times and average result
#define ADC_MUX_PH		6
#define ADC_MUX_TEMP	8
#define T_OFF			60*60*4	//Aquarium pump Off for 4 h
#define T_ON			60*5	//Aquarium pump ON for 5 min

// Flags
#define FLAG_PREQ_ON		0		//Requested to switch Pump ON
#define FLAG_PREQ_OFF		1		//Requested to switch Pump Off
#define FLAG_REPORT_STATUS 	2		//Report status over nRF24

// Pseudo functions
#define SBI(reg, bit) 		( reg |=  ( 1 << bit ) )
#define CBI(reg, bit) 		( reg &= ~( 1 << bit ) )
#define IBI(reg, bit) 		( (reg&(1<<bit)) != 0 )
#define PUMP_ON()			{ SBI( PORTB, PIN_SS_REL ); }
#define PUMP_OFF()			{ CBI( PORTB, PIN_SS_REL ); }
#define IS_PUMP()			  IBI( PORTB, PIN_SS_REL )
#define SLEEP()				{ asm volatile("sleep"); }
#define SLEEP_SET_ADC()     { SMCR=0b00000011; }	//For ADC measurements
#define SLEEP_SET_IDLE()    { SMCR=0b00000001; }	//When T0 still needs to run
#define SLEEP_SET_STANDBY() { SMCR=0b00001101; }	//When only the crystal needs to run
#define SLEEP_SET_PDOWN() 	{ SMCR=0b00000101; }	//When everything except the WDT is off

//--------------------------------------------------
// Timer0 for timebase tick (stolen from Arduino)
//--------------------------------------------------
#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( ((a) * 1000L) / (F_CPU / 1000L) )
#define T0_PRESCALE 1024
#define T0_RELOAD    244/2
#define MICROSECONDS_PER_TIMER0_OVERFLOW ( clockCyclesToMicroseconds(T0_PRESCALE*(T0_RELOAD+1L)) )
// the whole number of milliseconds per timer0 overflow
#define MILLIS_INC       (MICROSECONDS_PER_TIMER0_OVERFLOW / 1000)
// the fractional number of microseconds per timer0 overflow.
#define MILLIS_INC_FRACT (MICROSECONDS_PER_TIMER0_OVERFLOW % 1000)
extern volatile uint32_t g_Millis;	//Milisecond counter (dont touch!)
uint32_t millis();					//Return number of milliseconds since startup

//--------------------------------------------------
// Global Variables
//--------------------------------------------------
extern volatile uint8_t flags;

//--------------------------------------------------
// Functions
//--------------------------------------------------
void init(void);


#endif /* MAIN_H_ */
