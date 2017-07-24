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
#include "millis.h"
#include "debugPrint.h"
#include "myNRF24.h"
#include "mySPI_avr.h"
#include "nRF24L01.h"
#include "i2cmaster.h"
#include "ds18b20.h"
#include "VL53L0X.h"

//--------------------------------------------------
// Globals
//--------------------------------------------------
volatile uint8_t flags=0;
uint8_t rxAddrs[5] = { 0xE2, 0xE7, 0xE7, 0xE7, 0xE7 };


void init(void) {
	debugInit();
	CBI( UCSR0B, RXEN0 );					// Disable UART RX
	//--------------------------------------------------
	// GPIOs
	//--------------------------------------------------
	// Set as outputs
	DDRD =  (1<<PIN_UART_TX) | (1<<PIN_PUMP_DAT) | (1<<PIN_PUMP_CLK) | (1<<PIN_DISP_RST)
		  | (1<<PIN_DISP_DC) | (1<<PIN_DISP_CS)  | (1<<PIN_PUMP_DAT) | (1<<PIN_PUMP_CLK);
	DDRB =  (1<<PIN_SS_REL)  | (1<<PIN_NRF_CSN)  | (1<<PIN_SPI_MOSI) | (1<<PIN_SPI_SCK);
	DDRC =  (1<<PIN_NRF_CE);
	// Enable weak pullups on button and I2C lines
	PORTC = (1<<PIN_I2C_SCL) | (1<<PIN_I2C_SDA);
	PORTD = (1<<PIN_BTN1)    | (1<<PIN_BTN2);
	//--------------------------------------------------
	// ADC for PH level measurement
	//--------------------------------------------------
	ADCSRA = (1<<ADEN)  | 6;	// Enable ADC, fADC=fsys/64
	initMillis();
	i2c_init();
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

uint16_t getAdc(){
	uint32_t result=0;
	for ( uint16_t x=0; x<(1<<ADC_AVG_FACT); x++ ){
		SBI( ADCSRA, ADSC );
		while( IBI(ADCSRA, ADSC) );
		result += ADC;
	}
	return result;//>>ADC_AVG_FACT;
}

uint16_t getPh(){
	ADMUX  = (1<<REFS0) | ADC_MUX_PH;	// AVcc reference, mux=ADC6
	return getAdc();
}

uint16_t getTemp(){
	ADMUX  = (1<<REFS1) | (1<<REFS0) | ADC_MUX_TEMP;	// AVcc reference, mux=ADC6
	return getAdc();
}

typedef struct {
	uint16_t seq;
	uint16_t rawPhValue;
	uint16_t rawTempValue;
	uint8_t pumpRunning;
	int16_t rawWaterTempValue;
	uint16_t waterLevel;
} drGtData;

void reportStatusNRF(){
	static uint16_t seq=0;
	// uint8_t txAddrs[5] = { 0xE5, 0xE7, 0xE7, 0xE7, 0xE9 };
	drGtData dat;
	//Read water temperature (over 1-wire without ROM matching)
	ds18b20read( &PORTB, &DDRB, &PINB, (1<<PIN_1WIRE_DAT), 0, &dat.rawWaterTempValue );
	// Get water level from TOF distance sensor over I2C
	dat.waterLevel = readRangeContinuousMillimeters( 0 );
	// Get PH-level from ADC
	dat.rawPhValue = getPh();
	// Get AVR temp. from ADC
	dat.rawTempValue = getTemp();
	// Get pump status from GPIO
	dat.pumpRunning = IS_PUMP();
	dat.seq = seq;
	debug_str("TX: ");
	hexDump( (uint8_t*)&dat, sizeof(dat) );
	debug_str(" --> ");
	nRfSendBytes( (uint8_t*)&dat, sizeof(dat), rxAddrs, 0 );
	debug_str("\n");
	seq++;
}

void everyMinute(){
	SBI( flags, FLAG_REPORT_STATUS );
}

void handle1HzTick(){
	static uint16_t remainingPumpTime=0;
	static uint8_t minuteTick=0;
	if( minuteTick++ >= 59 ){
		minuteTick = 0;
		everyMinute();
	}
	if( remainingPumpTime<=0 ){
		if( IS_PUMP() ){
			SBI(flags,FLAG_PREQ_OFF);
		} else {
			SBI(flags,FLAG_PREQ_ON);
		}
	} else {
		remainingPumpTime--;
	}

	if( IBI( flags, FLAG_REPORT_STATUS ) ){		// Report status over nRF
		reportStatusNRF();
		CBI(flags,FLAG_REPORT_STATUS);
	}
	//Start water temperature conversion (over 1-wire without ROM matching)
	ds18b20convert( &PORTB, &DDRB, &PINB, (1<<PIN_1WIRE_DAT), 0 );

	if( IBI(flags,FLAG_PREQ_OFF) ){
		remainingPumpTime = T_OFF;
		PUMP_OFF();
		debug_str("Pump Off for ");
		debug_dec(T_OFF);
		debug_str(" s\n");
		SBI( flags, FLAG_REPORT_STATUS );	// Report pump status over nRF in 1 s
		CBI( flags, FLAG_PREQ_OFF );
	}
	if( IBI(flags,FLAG_PREQ_ON) ){
		remainingPumpTime = T_ON;
		PUMP_ON();
		debug_str("Pump On for ");
		debug_dec(T_ON);
		debug_str(" s\n");
		SBI( flags, FLAG_REPORT_STATUS );	// Report pump status over nRF in 1s
		CBI( flags, FLAG_PREQ_ON );
	}
}

void handleReceivedData(){
   uint8_t nRec, recBuffer[64];
   while( !nRfIsRXempty() ){
		nRec = nRfGet_RX_Payload_Width();		
		if( nRec<=0 || nRec>=64 ){
			nRfWrite_register( STATUS, (1<<RX_DR) );//Clear Data ready flag	
			return;
		}
		nRfRead_payload( recBuffer, nRec );
		debug_str("RX: ");
		hexDump( recBuffer, nRec );
		debug_str("\n");
		if( recBuffer[0] == 1 ){
			SBI( flags, FLAG_PREQ_ON );
		} else if( recBuffer[0] == 0 ){
			SBI( flags, FLAG_PREQ_OFF );
		}
    }
    nRfWrite_register( STATUS, (1<<RX_DR) );		//Clear Data ready flag	
}

void handle4HzTick(){
	static uint8_t t=0;
	static uint32_t lastCallMs = 0;
	uint32_t curMs = millis();
	if( curMs-lastCallMs < 250 ){
		return;
	}
	lastCallMs = curMs;
	if( nRfIsDataReceived() ){
		handleReceivedData();
	}
	if( !IBI(PIND,PIN_BTN1) ){
		SBI( flags, FLAG_PREQ_ON );
	}
	if( !IBI(PIND,PIN_BTN2) ){
		SBI( flags, FLAG_PREQ_OFF );
	}
	if( t++ >= 3 ){
		t = 0;
		handle1HzTick();
	}
}

void myInitNrf(){
    nRfInitRX();
    nRfSetupRXPipe( 0, rxAddrs );
    nRfFlush_tx();
    nRfFlush_rx();
    nRfWrite_register( STATUS, (1<<RX_DR) );		        //Clear Data ready flag
    nRfHexdump();
}

int main(){
	init();
	setTPIC( 0 );
	debug_str("\n\n---------------------------------------\n");
	debug_str(" Hello world, this is DrGreenThumb ! \n");
	debug_str("---------------------------------------\nGit: ");
	debug_str( GIT_VERSION );
	debug_putc( '\n' );

	myInitNrf();

	// Set 1 wire temp sensor to 12 bit precission
	ds18b20wsp( &PORTB, &DDRB, &PINB, (1<<PIN_1WIRE_DAT), 0, 0x00, 0xFF, DS18B20_RES12 );
	ds18b20convert( &PORTB, &DDRB, &PINB, (1<<PIN_1WIRE_DAT), 0 );

	// Initialize TOF distance sensor for water level measurement
	initVL53L0X(1);
	setMeasurementTimingBudget( 3000 * 1000UL );	// integrate over 3000 ms per measurement
	startContinuous( 0 );

	sei();
	// Main loop
	while(1){
		handle4HzTick();
	}
	return 0;
}



