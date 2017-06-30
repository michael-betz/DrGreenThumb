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
#define TPIC_DELAY_US 	1
#define ADC_AVG_FACT 	7		//Sample 2^N times and average result
#define T_OFF			60*60*4	//Aquarium pump Off for 4 h
#define T_ON			60*5	//Aquarium pump ON for 5 min

// Flags
#define FLAG_TICK		1		//250 ms tick
#define FLAG_PREQ_ON	2		//Requested to switch Pump ON
#define FLAG_PREQ_OFF	3		//Requested to switch Pump Off

// Pseudo functions
#define SBI(reg, bit) ( reg |=  ( 1 << bit ) )
#define CBI(reg, bit) ( reg &= ~( 1 << bit ) )
#define IBI(reg, bit) ( (reg&(1<<bit)) != 0 )
#define PUMP_ON()			{ SBI( PORTB, PIN_SS_REL ); }
#define PUMP_OFF()			{ CBI( PORTB, PIN_SS_REL ); }
#define IS_PUMP()			IBI( PINB, PIN_SS_REL )
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


#endif /* MAIN_H_ */
