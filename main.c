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
volatile uint8_t g_flags=0;
uint8_t rxAddrs[5] = { 0xE2, 0xE7, 0xE7, 0xE7, 0xE7 };
int16_t g_currentWaterLevel=0;
uint8_t g_dosingCounter = 0;
uint8_t g_recipePulseLengths[ DOSING_CHANNELS ];
uint8_t g_recipeCurrentId = 0;

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
	uint8_t flags;
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
	// dat.waterLevel = readRangeContinuousMillimeters( 0 );
	dat.waterLevel = (uint16_t)g_currentWaterLevel;
	// Get PH-level from ADC
	dat.rawPhValue = getPh();
	// Get AVR temp. from ADC
	dat.rawTempValue = getTemp();
	// Get pump status from GPIO
	dat.flags = (IS_PUMP()<<FLAG_IS_PUMP) | g_flags;
	dat.seq = seq;
	debug_str("TX: ");
	hexDump( (uint8_t*)&dat, sizeof(dat) );
	debug_str(" --> ");
	nRfSendBytes( (uint8_t*)&dat, sizeof(dat), rxAddrs, 0 );
	debug_str("\n");
	seq++;
}

void everyMinute(){
	SBI( g_flags, FLAG_REPORT_STATUS );
}

/**
*  @details    Implement a first order IIR filter to approximate a K sample 
*              moving average.  This function implements the equation:
*                  y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
*  @param      sample - the 16-bit value of the current sample.
*/
#define WATER_LEVEL_FILTER_NAVG 	5  	// This is roughly = log2( 1 / alpha )
#define WATER_LEVEL_FILTER_N_FRACT  5	// How many fractional bits (to get correct result, float-divide g_currentWaterLevel by 2^N)
void waterLevelFilter( int16_t sample ){
    static int32_t filter = 0;
    filter += ( ((int32_t)sample<<16) - filter ) >> WATER_LEVEL_FILTER_NAVG;
    ///< Round by adding .5 and truncating.
    g_currentWaterLevel = (int16_t)((filter+(0x8000>>WATER_LEVEL_FILTER_N_FRACT)) >> (16-WATER_LEVEL_FILTER_N_FRACT) );
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
			SBI(g_flags,FLAG_PREQ_OFF);
		} else {
			SBI(g_flags,FLAG_PREQ_ON);
		}
	} else {
		remainingPumpTime--;
	}
	
	// Update water level moving average every second
	waterLevelFilter( (int16_t)readRangeContinuousMillimeters(0) );
	
	// Report status over nRF if requested
	if( IBI( g_flags, FLAG_REPORT_STATUS ) ){
		reportStatusNRF();
		CBI(g_flags,FLAG_REPORT_STATUS);
	}
	
	//Start water temperature conversion (over 1-wire without ROM matching)
	ds18b20convert( &PORTB, &DDRB, &PINB, (1<<PIN_1WIRE_DAT), 0 );

	if( IBI(g_flags,FLAG_PREQ_OFF) ){
		remainingPumpTime = T_OFF;
		PUMP_OFF();
		debug_str("Pump Off for ");
		debug_dec(T_OFF);
		debug_str(" s\n");
		SBI( g_flags, FLAG_REPORT_STATUS );	// Report pump status over nRF in 1 s
		CBI( g_flags, FLAG_PREQ_OFF );
	}
	if( IBI(g_flags,FLAG_PREQ_ON) ){
		remainingPumpTime = T_ON;
		PUMP_ON();
		debug_str("Pump On for ");
		debug_dec(T_ON);
		debug_str(" s\n");
		SBI( g_flags, FLAG_REPORT_STATUS );	// Report pump status over nRF in 1s
		CBI( g_flags, FLAG_PREQ_ON );
	}
}

// Enable a TPIC output for `pulseLength` in [1/4 s]
void dosingPulse( uint8_t outputId, uint8_t pulseLength ){
	setTPIC( 0 );
	CBI( g_flags, FLAG_IS_DOSING );
	if ( outputId > 7 ){
		return;
	}
	if ( pulseLength <= 0 ){
		return;
	}
	g_dosingCounter = pulseLength;
	setTPIC( 1<<outputId );
	SBI( g_flags, FLAG_IS_DOSING );
	debug_str("Dosing for ");
	debug_dec_fix( pulseLength, 2 );
	debug_str(" seconds on output ");
	debug_dec( outputId );
	debug_putc('\n');
	// SBI( g_flags, FLAG_REPORT_STATUS );			// Report status over nRF in 1s
}

// Starts the recipe state machine which will go through a list of 7 dosing pulse length values
void startDosingRecipe( uint8_t *dosingPulseLengths ){
	//dosingPulseLengths = 7 * uint8_t [1/4s]
	dosingPulse( 0, 0 ); 	//Disable any dosing in progress
	// Copy recipe to global array
	for (uint8_t i = 0; i<DOSING_CHANNELS; i++){
		g_recipePulseLengths[i] = dosingPulseLengths[i];
	}
	// Initialize and start recipe state machine
	g_recipeCurrentId = 0;
	SBI( g_flags, FLAG_IS_RECIPE );
}

void handleDosingStateMachine(){
	// must be called periodically (every 0.25s)
	// If no recipe is in progress, return
	if( !IBI( g_flags, FLAG_IS_RECIPE ) ){
		return;
	}	
	// If a dosing pulse is in progress, return
	if( IBI( g_flags, FLAG_IS_DOSING ) ){
		return;
	}
	if( g_recipeCurrentId < DOSING_CHANNELS ){
		// We should start a new dosing pulse
		dosingPulse( g_recipeCurrentId, g_recipePulseLengths[ g_recipeCurrentId ] );
		g_recipeCurrentId++;
	} else {
		// We are done with the recipe
		CBI( g_flags, FLAG_IS_RECIPE );
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
		switch (recBuffer[0]){
			case 0:
				// Pump Off command
				SBI( g_flags, FLAG_PREQ_OFF );
			break;
			case 1:
				// Pump On command
				SBI( g_flags, FLAG_PREQ_ON );
			break;
			case 2:
				// Dosing Pump command <pumpId> <pulseLength[s/4]>
				if (nRec == 3){
					// Cancel any dosing recipe in progress
					CBI( g_flags, FLAG_IS_RECIPE );
					dosingPulse( recBuffer[1], recBuffer[2] );
				}
			break;
			case 3:
				// Dosing recipe command 7 x <pulseLength[s/4]>
				if (nRec == DOSING_CHANNELS+1){
					startDosingRecipe( &recBuffer[1] );
				}
			break;
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
		SBI( g_flags, FLAG_PREQ_ON );
	}
	if( !IBI(PIND,PIN_BTN2) ){
		SBI( g_flags, FLAG_PREQ_OFF );
	}
	if( IBI( g_flags, FLAG_IS_DOSING ) ){
		SBI( g_flags, FLAG_REPORT_STATUS );				// Report status over nRF every 1s
		if ( g_dosingCounter > 0 ) {
			g_dosingCounter--;
			// debug_str(".");
		} else {
			setTPIC( 0 );
			CBI( g_flags, FLAG_IS_DOSING );
			debug_str("Dosing finished\n");
		}
	}
	handleDosingStateMachine();
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
	setMeasurementTimingBudget( 900 * 1000UL );	// integrate over 900 ms per measurement
	startContinuous( 0 );

	sei();
	// Main loop
	while(1){
		handle4HzTick();
	}
	return 0;
}



