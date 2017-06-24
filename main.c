/*
 * main.c
 *
 *  Created on: June 23, 2017
 *      Author: michael
 *
 *  Control an aquarium pump, 5 peristaltic pumps and a water valve.
 *  Measure temperature, water level and ph
 * 
 */
#include <inttypes.h>
#include <avr/io.h>
#include <util/delay.h>
#include "main.h"
#include "rprintf.h"

#define PUMP_DAT PD6
#define PUMP_CLK PD7

void init(void) {
//--------------------------------------------------
// Serial debugging at 38400 baud/s
//--------------------------------------------------
	//Set UART TX pin and TPIC6B595 pins as output
	DDRD = (1<<PD1) | (1<<PUMP_DAT) | (1<<PUMP_CLK);
	odDebugInit();
	//Setup ADC for PH level measurement
	ADCSRA = (1<<ADEN) | 6;		// Enable ADC, fADC=fsys/64
	ADMUX = (1<<REFS0) | 6;	// AVcc reference, mux=ADC6
}

#define TPIC_DELAY_US 1

void setTPIC( uint8_t dat ){
	// Set the 8 open drain outputs on the TPIC6B595 to `dat`
	// Note: drain6 and drain7 are physically connected
	// so they will be OR-ed here for higher current capability
	if ( dat&0x80 || dat&0x40 ) {
		dat |= 0xC0;
	}
	// clock in 8 bits into the shift register
	for (uint8_t x=0; x<=7; x++){
		if( dat & 0x80 ){
			SBI( PORTD, PUMP_DAT );
		} else {
			CBI( PORTD, PUMP_DAT );
		}
		_delay_us( TPIC_DELAY_US );
		SBI( PORTD, PUMP_CLK );
		_delay_us( TPIC_DELAY_US );
		CBI( PORTD, PUMP_CLK );
		dat <<= 1;
	}
	// latch them to the outputs
	CBI( PORTD, PUMP_DAT );
	_delay_us( TPIC_DELAY_US );
	SBI( PORTD, PUMP_DAT );
	_delay_us( TPIC_DELAY_US );
	CBI( PORTD, PUMP_DAT );
}

uint16_t getAdc( ){
	uint16_t result=0;
	for ( uint8_t x=0; x<64; x++ ){
		SBI( ADCSRA, ADSC );
		uint8_t x=0;
		while( IBI(ADCSRA, ADSC) );
		result += ADC;
	}
	return result;
}

int main(){
	init();				//Now WDT is running and will generate an interrupt on overflow
	rprintf("\n\n---------------------------------------\n");
	rprintf(" Hello world, this is DrGreenThumb ! \n");
	rprintf("---------------------------------------\nGit: ");
	rprintf( GIT_VERSION );
	rprintf( "\n" );
	uint8_t temp=1;
	while(1){
		temp ^= temp << 1;
		setTPIC( temp );
		rprintf("getAdc: %u\n", getAdc() );
		_delay_ms( 100 );
	}
	return 0;
}