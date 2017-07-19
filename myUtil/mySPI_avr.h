/*
 * mySPI_avr.h
 *
 *  Created on: May 8, 2014
 *      Author: michael
 */

#ifndef MYSPI_AVR_H_
#define MYSPI_AVR_H_
#include <avr/io.h>
#include "../main.h"

//----------------------------------------
// Defines for the nRF24 module
//----------------------------------------
#define	NRF_CHIP_SELECT()	CBI( PORTB, PIN_NRF_CSN );
#define	NRF_CHIP_DESELECT() SBI( PORTB, PIN_NRF_CSN );
#define	NRF_CE_ON()			SBI( PORTC, PIN_NRF_CE );
#define	NRF_CE_OFF()		CBI( PORTC, PIN_NRF_CE );

void initSPI( void );
uint8_t nRfRead_registers(uint8_t reg, uint8_t *buf, uint8_t len);
uint8_t nRfRead_register( uint8_t reg );
uint8_t nRfWrite_registers( uint8_t reg, const uint8_t* buf, uint8_t len );
uint8_t nRfWrite_register(uint8_t reg, uint8_t value);
uint8_t nRfWrite_payload( const void* buf, uint8_t len, uint8_t command );
uint8_t nRfRead_payload( void* buf, uint8_t len );
uint8_t nRfFlush_rx(void);
uint8_t nRfFlush_tx(void);
uint8_t nRfGet_status(void);
uint8_t nRfGet_RX_Payload_Width(void);

#endif /* MYSPI_AVR_H_ */
