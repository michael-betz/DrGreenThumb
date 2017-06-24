/*
 * myNRF24.c
 *
 *  Created on: May 8, 2014
 *      Author: michael
 */

#include <inttypes.h>
#include <stdio.h>
#ifdef __AVR_ATmega328P__   //AVR version
	#define	AVR_VERSION
    #include "mySPI_avr.h"
	#include <util/delay.h>
	#include "rprintf.h"
#else                       //Raspi version
	#define	RASPI_VERSION
    #include "mySPI_raspi.h"
    #include <bcm2835.h>    //Raspi GPIO library
#endif
#include "nRF24L01.h"
#include "myNRF24.h"

uint8_t cacheCONFIG;								//We cache the config register as global variable, so it does not need to be read over SPI each time it is changed

void nRfInit(){	//Init with default config
	const uint8_t rxTxAdr[5] = { 0xE0, 0xE7, 0xE7, 0xE7, 0xE7 };
    initSPI();
	NRF_CE_OFF();
	NRF_CHIP_DESELECT();
	_delay_ms( 5 );
	cacheCONFIG = 0b00001100;						//3xIRQs on, 16bit CRC, PDown, PTX mode
	nRfWrite_register( CONFIG, 		cacheCONFIG );
	nRfWrite_register( EN_AA,  		0b00111111 );	//Enable auto ACK on pipe0 - pipe5
	nRfWrite_register( EN_RXADDR,	0b00000001 );	//Only enable data pipe ERX_P0 for the start
	nRfWrite_register( SETUP_AW,    0b00000011);	//5 byte address width
	nRfWrite_register( SETUP_RETR,	0x3F );			//Automatic retransmit up to 15 times with delay of 1000 us
	nRfWrite_register( RF_CH,		64 );			//Set RF channel to 64
	nRfWrite_register( RF_SETUP,	0b00001111 );	//2 Mbps data rate, 0dBm power, LNA_HCURR = 1
	nRfWrite_registers(RX_ADDR_P0, 	rxTxAdr, 5 );	//Set RX pipe address
	nRfWrite_registers(TX_ADDR, 	rxTxAdr, 5 );	//Set transmit pipe address
	//nRfWrite_register( RX_PW_P0,	6 );			//6 byte static RX payload length (only needed for RX when dyn payload is off)
	nRfWrite_register( DYNPD,		0b00111111 );	//Enable dynamic payload length on all pipes
	nRfWrite_register( FEATURE, 	0b00000111 );	//Enable features: Dynamic payload length, Ack packets with payload, Disabled Ack for some packets
}

void nRfInitTX(){	//Init for sleeping and sending on demand
	nRfInit();
}

void nRfInitRX(){	//Init for continuously receiving data
    nRfInit();
	NRF_RX_MODE();
	NRF_PWR_UP();
	_delay_ms( 3 );									//Wait for Powerup
	NRF_CE_ON();
}

// The RX module monitors the air for packets which match its address
// This is done for 6 data pipes in parallel, which have individual settings
// pipeNumber: ERX_P0 - ERX_P5
void nRfSetupRXPipe( uint8_t pipeNumber, uint8_t *rxAddr ){
	if( pipeNumber <= 5){
//		Enable the pipe
//		----------------
		uint8_t temp = nRfRead_register( EN_RXADDR );	//We have to do a read modify write
		SBI( temp, pipeNumber);
		nRfWrite_register( EN_RXADDR, temp );			//Enable the data pipe
//		Set the pipe address
//		--------------------
		if( pipeNumber <= 1 ){							//First 2 pipes got a 5 byte address
			nRfWrite_registers( RX_ADDR_P0+pipeNumber, rxAddr, 5 );
		} else {										//The other 4 pipes got a 1 byte address
			nRfWrite_registers( RX_ADDR_P0+pipeNumber, rxAddr, 1 );
		}
	}
}

// Transfers data from bytesToSend to the TX FIFO
// len = 1 ... 32
// if noAck == 1 then the sender will not wait for an ack packet, even if ack is enabled
void nRfSendBytes( uint8_t *bytesToSend, uint8_t len, uint8_t noAck ){
	NRF_PWR_UP()
	nRfWrite_payload( bytesToSend, len, noAck );
	_delay_ms( 1 );									//Wait for Powerup
	NRF_CE_ON();
	_delay_us( 20 );
	NRF_CE_OFF();
	//The nRF will be powered down by the ISR
}

void nRfHandleISR(){
	uint8_t status, obsTX, dataPipe;
	status = nRfRead_registers(OBSERVE_TX, &obsTX, 1);	//Did the nRF24 module really triggered an interrupt
	if( IBI(status, TX_DS) ){	//TX finished interrupt
		if( (obsTX & 0x0F) > 0 )
			rprintf("ISR_TX_DS: retries = %d\n", obsTX & 0x0F );
//		Data Sent TX FIFO interrupt. Asserted when
//		packet transmitted on TX. If AUTO_ACK is acti-
//		vated, this bit is set high only when ACK is
//		received.
//		Write 1 to clear bit.
	}
	if( IBI(status, RX_DR) ){	//RX data received
		rprintf("ISR_RX_DR\n");
		dataPipe = ( status & 0b00001110 ) >> 1;
//		The RX_DR IRQ is asserted by a new packet arrival event. The procedure for handling this interrupt should
//		be: 1) read payload through SPI, 2) clear RX_DR IRQ, 3) read FIFO_STATUS to check if there are more
//		payloads available in RX FIFO, 4) if there are more data in RX FIFO, repeat from step 1)
//		Also triggered when an ACK with payload is received
	}
	if( IBI(status, MAX_RT) ){	//Maximum number of TX retries reached
		rprintf("ISR_MAX_RT: lost %d packets since reset\n", ( obsTX & 0xF0 ) >> 4 );
//		Maximum number of TX retransmits interrupt
//		Write 1 to clear bit. If MAX_RT is asserted it must
//		be cleared to enable further communication.
	}
	nRfWrite_register( STATUS, (1<<TX_DS)|(1<<RX_DR)|(1<<MAX_RT) );	//Clear all interrupt flags
	NRF_PWR_DOWN();
}

uint8_t nRfIsDataReady(){
//	return( nRfGet_status()&(1<<RX_DR) );                      //Check interrupt flag
	return( !(nRfRead_register(FIFO_STATUS)&(1<<RX_EMPTY)) );  //Check FIFO status
}


//------------------------------------------------
// Debug functions
//------------------------------------------------
void nRfHexdump( void ){
	uint8_t x;
	rprintf( "All nRF24 registers:\n" );
	rprintf("0x00:  ");
	for( x=0; x<=0x1D; x++){
		rprintf( "%02x ", nRfRead_register(x) );
		if( ((x+1)%8) == 0 ){
			rprintf( "\n" );
			rprintf( "0x%02x:  ", x+1);
		}
	}
	rprintf("\n\n");
}
