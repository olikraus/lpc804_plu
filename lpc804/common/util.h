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

#include "LPC8xx.h"
#include <stdint.h>


/*
  ring buffer
*/
struct _rb_struct
{
  uint8_t *ptr;  
  uint16_t start;
  uint16_t end;
  uint16_t len;
};
typedef struct _rb_struct rb_t;

/*
  usart 
*/
struct usart_struct
{
  LPC_USART_TypeDef *usart;
  rb_t rb;
};
typedef struct usart_struct usart_t;



/*
  Usually you would do a 
    LPC_IOCON->PIO0_7 
  to access the IOCON register for PIO0_7
  In cases, where you have only the number 7 this function will return the 
  corresponding register:
    *get_iocon_by_port(7)
  is the same as the above statement.
*/
__IO uint32_t *get_iocon_by_port(uint8_t port) __attribute__((noinline));


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
void i2c_init(uint8_t clkdiv, uint8_t scl, uint8_t sda);

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
  Prototype:
    void rb_init(rb_t *rb, uint8_t *buf, uint16_t len)

  Description:
    Prepares a ring (first in first out) buffer. 

  Parameter:
    rb: 	Address of a (uninitialized) buffer data structure
    buf:	Memory location large enough for "len" bytes
    len:	Size of the ring buffer. 

  Example:

    rb_t usart_rx_ring_buffer;
    uint8_t usart_rx_buf[32];
    ...
    rb_init(&usart_rx_ring_buffer, usart_rx_buf, 32);

*/
void rb_init(rb_t *rb, uint8_t *buf, uint16_t len);

/* 
  Prototype:
    int rb_add(rb_t *rb, uint8_t data)

  Description:
    Add a byte to the ring buffer.

  Precondition:
    rb_init() must be called on the "rb" argument.

  Parameter:
    rb: 	Address of a (initialized) buffer data structure
    data:	8 bit value, which should be stored in the ring buffer

  Return:
    0:	Data not stored, ring buffer is full
    1:	all ok
    
*/
int rb_add(rb_t *rb, uint8_t data);

/* 
  Prototype:
    int rb_get(rb_t *rb)

  Description:
    Get data out of the ring buffer and remove this data item from the ring buffer.

  Precondition:
    rb_init() must be called on the "rb" argument.

  Parameter:
    rb: 	Address of a (initialized) buffer data structure

  Return:
    -1 if there is no data available, otherwise the data which was added before.
    
*/
int rb_get(rb_t *rb);


void usart0_init(usart_t *usart, uint32_t brgval, uint8_t tx, uint8_t rx, uint8_t *rx_buf, uint16_t rx_len);
void usart_write_byte(usart_t *usart, uint8_t data);
void usart_write_string(usart_t *usart, const char *s);
int usart_read_byte(usart_t *usart);		// returns -1 if there is no byte available

/* 
  Description:
    Convert a 16 bit value to a decimal number.
    
  Parameter:
    v: Unsigned 16 bit value, which should be converter.
    
  Return:
    A pointer to a static buffer.

  Example:
    usart_write_string(&usart, u16toa(1234));
    usart_write_string(&usart, "\r\n");

*/
const char *u16toa(uint16_t v);


/* 
  Description:
    Replacement for ConfigSWM(uint32_t func, uint32_t port_pin)
    This function will connect an internal signal to the specified GPIO port
  
  Parameter:
    fn: A function number, e.g. T0_MAT0, see swm.h
    port: A port number for the GPIO port (0..30)

*/
void map_function_to_port(uint32_t fn, uint32_t port);

/* 

  Prototype:
    int16_t get_adc_by_port(uint8_t port)

  Description:
    Return the ADC number for the given GPIO port, e.g. return 5 (ADC_5) for GPIO 0_8.

  Args:
    port:	The port number (e.g. 8 for GPIO Port 0_8)

  Return:
    -1	if there is no ADC available for this port or the ADC number    

*/
int16_t get_adc_by_port(uint8_t port);

/* 

  Prototype:
    void adc_init(void)

  Description:
    Powerup ADC and enable clock for the ADC.
    Finally put the ADC into low power mode.

*/
void adc_init(void);

/* 

  Prototype:
    uint16_t adc_read(uint8_t port)

  Precondition: SWM and GPIO clock must be enabled, adc_init() must be called

  Descriptiion:
    Read a 12 bit value from the specified GPIO port. Not all ports can be used as ADC input.
    For any illegal port, this function will return 0x0ffff.
    This function will also change the port to input, disable and pullup/pullodowns and 
    enable the ADC on this port.

  Note: If the pullup/pulldown is changed, then the result might be incorrect.
    It may take several seconds until a stable voltage is reached after modifying the
    LPC804 output resistors. Consider to disable the output resistors as early as possible
    after reset:   *get_iocon_by_port(port) &= IOCON_MODE_MASK;

  Args:
    port:	The port number (e.g. 8 for GPIO Port 0_8)

  Return: 
    0xffff, if the port doesn't support ADC
    0xfffe, if for a timeout on caused by the ADC (e.g. caused by a missing call to adc_init())
    otherwise: 12 bit ADC result at the specified port (0..4095) 

*/
uint16_t adc_read(uint8_t port);


#endif