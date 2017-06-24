/*************************************************************************
* Title:    I2C master library using hardware TWI interface
* Author:   Peter Fleury <pfleury@gmx.ch>  http://jump.to/fleury
* File:     $Id: twimaster.c,v 1.3 2005/07/02 11:14:21 Peter Exp $
* Software: AVR-GCC 3.4.3 / avr-libc 1.2.3
* Target:   any AVR device with hardware TWI 
* Usage:    API compatible with I2C Software Library i2cmaster.h
**************************************************************************/
#include <stdint.h>
#include <avr/delay.h>
#include "../main.h"
#include "rprintf.h"
#include "i2cmaster.h"

/*************************************************************************
 Initialization of the I2C bus interface. Need to be called only once
*************************************************************************/
void i2c_init(void){
	//--------------------------------------------------
	// SOft I2C
	//--------------------------------------------------
	SET_SCL_TRI();
	SET_SDA_TRI();
	CBI( MCUCR, PUD );						// Enable Pull-ups globally
}/* i2c_init */


/*************************************************************************
  Send one byte to I2C device
  8 bit + ack read.
  Input:   byte to be transfered
  Return:  Returns ack
   	   	   0 write successful
           1 write failed
*************************************************************************/
uint8_t i2c_write( uint8_t data ){
	uint8_t i;
	rprintf("W");
	for( i=0; i<=7; i++ ){
		SET_SCL_LOW();
		I2C_DELAY();
		if( data & 0x80 ){
			SET_SDA_TRI();
			rprintf("1");
		} else {
			SET_SDA_LOW();
			rprintf("0");
		}
		I2C_DELAY();
		SET_SCL_TRI();
		I2C_DELAY();
		data <<= 1;
	}
	SET_SCL_LOW();
	I2C_DELAY();
	SET_SDA_TRI();
	I2C_DELAY();
	SET_SCL_TRI();							// Get the ACK
	I2C_DELAY();
	i = GET_SDA();
	i ? rprintf("A1") : rprintf("A0");
	I2C_DELAY();
	return i;
}

/*************************************************************************	
  Issues a start condition and sends address and transfer direction.
  return 0 = device accessible, 1= failed to access device
*************************************************************************/
uint8_t i2c_start( uint8_t address ){
    uint8_t ack;
	// send START condition
    rprintf("S");
    SET_SDA_LOW();
    I2C_DELAY();
    SET_SCL_LOW();
    I2C_DELAY();

	// send address and get the ack bit (should be low)
    ack = i2c_write( address );

    return ack;
}/* i2c_start */


/*************************************************************************
 Issues a start condition and sends address and transfer direction.
 If device is busy, use ack polling to wait until device is ready
 Input:   address and transfer direction of I2C device
*************************************************************************/
uint8_t i2c_start_wait( uint8_t address){
	uint16_t i=0, temp;
	temp = i2c_start( address );
	while( GET_SCL() == 0 ){	// As long as the slave holds the SCL line low (busy signal)
		I2C_DELAY();			// Wait
		i++;
	}
	rprintf("C%d", i);
	return temp;
}/* i2c_start_wait */


/*************************************************************************
 Terminates the data transfer and releases the I2C bus
*************************************************************************/
void i2c_stop(void){
	// send STOP condition
	rprintf("X");
    SET_SCL_LOW();
    I2C_DELAY();
    SET_SDA_LOW();
    I2C_DELAY();
    SET_SCL_TRI();
    I2C_DELAY();
    SET_SDA_TRI();
    I2C_DELAY();
}/* i2c_stop */


/*************************************************************************
 Read one byte from the I2C device,
 if ack == 1: request more data from device
 Return:  byte read from I2C device, or -1 on error
*************************************************************************/
uint8_t i2c_read( uint8_t ack ){
	uint8_t i, readData=0, temp;
	rprintf("R");
	for( i=0; i<=7; i++ ){
		SET_SCL_LOW();
		I2C_DELAY();
		SET_SDA_TRI();
		I2C_DELAY();
		SET_SCL_TRI();
		I2C_DELAY();
		temp = GET_SDA();
		rprintf("%d",temp);
		readData <<= 1;
		readData |= temp;
		I2C_DELAY();
	}
	SET_SCL_LOW();
	if( ack ){
		SET_SDA_TRI();
	} else {
		SET_SDA_LOW();		// Send the ACK (more data requested)
	}
	rprintf("A%d",ack);
	I2C_DELAY();
	SET_SCL_TRI();
	I2C_DELAY();
	return readData;
}/* i2c_readAck */
