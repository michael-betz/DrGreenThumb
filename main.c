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
#include <avr/interrupt.h>
#include "main.h"
#include "rprintf.h"
#include "myNRF24.h"
#include "mySPI_avr.h"

//--------------------------------------------------
// Globals
//--------------------------------------------------
volatile uint8_t flags=0;

void init(void) {
	odDebugInit();
	CBI( UCSR0B, RXEN0 );					// Disable UART RX
	//--------------------------------------------------
	// GPIOs
	//--------------------------------------------------
	// Set as outputs
	DDRD =  (1<<PIN_UART_TX) | (1<<PIN_PUMP_DAT) | (1<<PIN_PUMP_CLK) | (1<<PIN_DISP_RST)
		  | (1<<PIN_DISP_DC) | (1<<PIN_DISP_CS)  | (1<<PIN_PUMP_DAT) | (1<<PIN_PUMP_CLK);
	DDRB =  (1<<PIN_SS_REL)  | (1<<PIN_NRF_CSN)  | (1<<PIN_SPI_MOSI) | (1<<PIN_SPI_SCK);
	DDRC =  (1<<PIN_NRF_CE)  | (1<<PIN_I2C_SDA)  | (1<<PIN_I2C_SDA);
	// Enable weak pullups on button
	PORTD = (1<<PIN_BTN1) | (1<<PIN_BTN2);
	//--------------------------------------------------
	// Timer0 for for a 250 ms tick
	//--------------------------------------------------
	TCCR0A = (1<<WGM01) | (1<<WGM00);		 //Fast PWM mode, no output
	TCCR0B = (1<<WGM02) | (1<<CS02) | (0<<CS01) | (1<<CS00);//TOP=OCR0A, 1024 prescaler
	OCR0A = 244;							 //Overvlow every 15.616 ms
	TIMSK0 = (1<<TOIE0);
	//--------------------------------------------------
	// ADC for PH level measurement
	//--------------------------------------------------
	ADCSRA = (1<<ADEN)  | 6;	// Enable ADC, fADC=fsys/64
	ADMUX  = (1<<REFS0) | 6;	// AVcc reference, mux=ADC6
}

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
			SBI( PORTD, PIN_PUMP_DAT );
		} else {
			CBI( PORTD, PIN_PUMP_DAT );
		}
		_delay_us( TPIC_DELAY_US );
		SBI( PORTD, PIN_PUMP_CLK );
		_delay_us( TPIC_DELAY_US );
		CBI( PORTD, PIN_PUMP_CLK );
		dat <<= 1;
	}
	// latch them on the outputs
	CBI( PORTD, PIN_PUMP_DAT );
	_delay_us( TPIC_DELAY_US );
	SBI( PORTD, PIN_PUMP_DAT );
	_delay_us( TPIC_DELAY_US );
	CBI( PORTD, PIN_PUMP_DAT );
}

uint16_t getAdc( ){
	uint32_t result=0;
	for ( uint16_t x=0; x<(1<<ADC_AVG_FACT); x++ ){
		SBI( ADCSRA, ADSC );
		while( IBI(ADCSRA, ADSC) );
		result += ADC;
	}
	return result>>ADC_AVG_FACT;
}

void handle1HzTick(){
	// rprintf(".");

}

void hexdump( uint8_t *buffer, uint16_t len ){
	rprintf("0x00:  ");
	for( uint16_t x=0; x<len; x++){
		rprintf( "%02x ", buffer[x] );
		if( ((x+1)%8) == 0 ){
			rprintf( "\n" );
			rprintf( "0x%02x:  ", x+1);
		}
	}
	rprintf("\n\n");
}

void handleReceivedData(){
	uint8_t nRec, recBuffer[64];
	nRec = nRfGet_RX_Payload_Width();
	if( nRec<=0 || nRec>=64 ){
		return;
	}
	nRfRead_payload( recBuffer, nRec );
	hexdump( recBuffer, nRec );
	if( recBuffer[0] == 1 ){
		PUMP_ON();
		rprintf("Pump ON\n" );
	} else if( recBuffer[0] == 0 ){
		PUMP_OFF();
		rprintf("Pump Off\n" );
	}
}

void handle4HzTick(){
	//Skip one tick every 6944 ticks to get 250 ms more precisely
	static uint16_t nTicks=0;
	static uint8_t t=0;
	if( nTicks++ >= 6944 ){
		nTicks = 0;
		return;
	}
	if( nRfIsRxDataReady() ){
		handleReceivedData();
	}
	if( IS_PUMP() ){
		if ( !IBI(PIND,PIN_BTN2) ) {
			PUMP_OFF();
			rprintf("Pump Off\n" );
		}	
	} else {
		if( !IBI(PIND,PIN_BTN1) ){
			PUMP_ON();
			rprintf("Pump ON\n" );
		}
	}		
	if( t++ >= 3 ){
		t = 0;
		handle1HzTick();
	}
}

int main(){
	init();
	rprintf("\n\n---------------------------------------\n");
	rprintf(" Hello world, this is DrGreenThumb ! \n");
	rprintf("---------------------------------------\nGit: ");
	rprintf( GIT_VERSION );
	rprintf( "\n" );
	nRfInitRX();
	nRfHexdump();
	sei();

	// Main loop
	while(1){
		if( IBI(flags, FLAG_TICK) ){
			handle4HzTick();
			CBI(flags, FLAG_TICK);
		}
		if( IBI(flags, FLAG_RX) ){
			handleReceivedData();
			CBI(flags, FLAG_RX);
		}
	}
	return 0;
}

volatile uint8_t intTickCounter = 0;

ISR( TIMER0_OVF_vect ){				//Called every 15.616 ms
	intTickCounter++;
	if ( intTickCounter >= 16 ){
		intTickCounter = 0;
		SBI( flags, FLAG_TICK );	//Called every 249.9 ms
	}
}
