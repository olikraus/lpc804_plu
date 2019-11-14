/*
  util.h
  
  LPC804 Project 

  Copyright (c) 2019, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification, 
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list 
    of conditions and the following disclaimer.
    
  * Redistributions in binary form must reproduce the above copyright notice, this 
    list of conditions and the following disclaimer in the documentation and/or other 
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  
  
*/

#ifndef UTIL_H
#define UTIL_H

#include <stdint.h>

#define I2C_OK 0
#define I2C_TIMEOUT_PRE_ADR 1

/* pending flag is active too long */
/* root cause: */
/* 1) SWM mapping was done after I2C subsystem enable (it must be done before) */
/* 2) SCL line is permanently pulled down */
#define I2C_TIMEOUT_POST_ADR 2

#define I2C_TIMEOUT_TX 3

/* there was an attempt to access the I2C bus, but the I2C subsystem is not ready */
#define I2C_NOT_IDLE 4 
/* transmission not available after sending the address with write request */
/* root cause: 1) wrong address, client not available */
/* 2) SDA line is mapped to an incorrect port (permanently pull up or pull down) */
/* 3) SCL line is mapped to an incorrect port (permanently pull up) */
#define I2C_NO_TX_POST_ADR 5

/* transmission not available after sending data (maybe NACK by client) */
#define I2C_NO_TX_POST_TX 6

/*
  Description:
    Setup I2C0 subsystem as I2C master
  Precondition:
    Signals must be assigned to the target GPIO before calling this function via SWM.
  Parameter:
    clkdiv: See table:

	main clk		clkdiv	Nominal I2C Bus Clock
	15 Mhz		37		99 KHz
	15 Mhz		9		375 KHz  
	12 Mhz		29		100 KHz
	12 Mhz		7		375 KHz
    Return:
      -
*/
void i2c_init(uint8_t clkdiv);

/*
  Parameter:
    adr: I2C address shifted to the left by 1, lowest bit must be 0
    buf: data, which should be sent to the client
    len: number of bytes to sent
  Return:
    Error code (see above) or I2C_OK if everything was sent.
*/
int i2c_write(uint8_t adr, uint8_t *buf, uint32_t len);


/* 
  Description:
    Replacement for ConfigSWM(uint32_t func, uint32_t port_pin)
    This function will connect an internal signal to the specified GPIO port
  
  Parameter:
    fn: A function number, e.g. T0_MAT0, see swm.h
    port: A port number for the GPIO port (0..30)

*/
void map_function_to_port(uint32_t fn, uint32_t port);


#endif