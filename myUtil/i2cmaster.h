#ifndef _I2CMASTER_H
#define _I2CMASTER_H   1
/************************************************************************* 
* Title:    C include file for the SOFT I2C master interface
**************************************************************************/

#include <avr/io.h>
#include "../main.h"

/* I2C clock in Hz */
#define SCL_CLOCK  	400000L
#define I2C_DELAY() { _delay_us( 1000000L / SCL_CLOCK ); }

#define SET_SCL_LOW()	{ CBI(PORTC,PIN_SCL); SBI(DDRC,PIN_SCL);}// __asm__("nop"); }
#define SET_SDA_LOW()	{ CBI(PORTC,PIN_SDA); SBI(DDRC,PIN_SDA);}// __asm__("nop"); }
#define SET_SCL_TRI()	{ SBI(PORTC,PIN_SCL); CBI(DDRC,PIN_SCL);}// __asm__("nop"); }
#define SET_SDA_TRI()	{ SBI(PORTC,PIN_SDA); CBI(DDRC,PIN_SDA);}// __asm__("nop"); }
#define GET_SCL()		  IBI(PINC, PIN_SCL)
#define GET_SDA()		  IBI(PINC, PIN_SDA)

/** defines the data direction (reading from I2C device) in i2c_start(),i2c_rep_start() */
#define I2C_READ    1

/** defines the data direction (writing to I2C device) in i2c_start(),i2c_rep_start() */
#define I2C_WRITE   0


/**
 @brief initialize the I2C master interace. Need to be called only once 
 @param  void
 @return none
 */
extern void i2c_init(void);


/** 
 @brief Terminates the data transfer and releases the I2C bus 
 @param void
 @return none
 */
extern void i2c_stop(void);


/** 
 @brief Issues a start condition and sends address and transfer direction 
  
 @param    addr address and transfer direction of I2C device
 @retval   0   device accessible 
 @retval   1   failed to access device 
 */
extern uint8_t i2c_start(uint8_t addr);


/**
 @brief Issues a start condition and sends address and transfer direction 
   
 If device is busy, use ack polling to wait until device ready 
 @param    addr address and transfer direction of I2C device
 @return   ack
 */
extern uint8_t i2c_start_wait(uint8_t addr);

 
/**
 @brief Send one byte to I2C device
 @param    data  byte to be transfered
 @retval   0 write successful
 @retval   1 write failed
 */
extern uint8_t i2c_write(uint8_t data);


/**
 @brief    read one byte from the I2C device, if ACK = 1: request more data from device
 @return   byte read from I2C device
 */
extern uint8_t i2c_read( uint8_t ack );


/**@}*/
#endif
