/*
  util.c
  
  The delay function delay_micro_seconds() will use the global variable
  SystemCoreClock. A call to SystemCoreClockUpdate() is required before
  using delay_micro_seconds().
  
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

#include <stddef.h>
#include "LPC8xx.h"
#include "syscon.h"
#include "iocon.h"
#include "gpio.h"
#include "swm.h"
#include "i2c.h"
#include "uart.h"
#include <adc.h>
#include "util.h"

/*====================================================*/

/*
  Usually we would do a 
    LPC_IOCON->PIO0_7 
  to access the IOCON register for PIO0_7
  In cases, where you have only the number 7 this function will return the 
  corresponding register:
    *get_iocon_by_port(7)
  is the same as the above statement.
*/

const uint8_t pio_to_iocon_offset[] = 
{
/* 00 */     0x44,
/* 01 */     0x2C,
/* 02 */     0x18,
/* 03 */     0x14,
/* 04 */     0x10,
/* 05 */     0x0C,
/* -- */	0x00,
/* 07 */     0x3C,
/* 08 */     0x38,
/* 09 */     0x34,
/* 10 */    0x20,
/* 11 */    0x1C,
/* 12 */    0x08,
/* 13 */    0x04,
/* 14 */    0x48,
/* 15 */    0x28,
/* 16 */    0x24,
/* 17 */    0x00,
/* 18 */    0x74,
/* 19 */    0x60,
/* 20 */    0x58,
/* 21 */    0x30,
/* 22 */    0x70,
/* 23 */    0x6C,
/* 24 */    0x68,
/* 25 */    0x64,
/* 26 */    0x54,
/* 27 */    0x50,
/* 28 */    0x4C,
/* 29 */    0x40,
/* 30 */    0x5C
};


__IO uint32_t *get_iocon_by_port(uint8_t port)
{
  return (__IO uint32_t *)  (((uint8_t *)LPC_IOCON_BASE)+pio_to_iocon_offset[port]);
}

/*====================================================*/

/*
  Function:
    void i2c_init(uint8_t clkdiv, uint8_t scl, uint8_t sda)

  Description:
    setup I2C0 as master. This function will enable all required clocks
  
  Parameter:
    clkdiv: See below
    scl: PIO port number for the clock line
    sda: PIO port number for the data line
  
  
  Nominal SCL rate = I2C function clock rate / 
  ( (CLKDIV + 1) * (MSTSCLHIGH + 2) + (CLKDIV + 1) * (MSTSCLLOW + 2) )
  Nominal SCL rate = I2C function clock rate / 
  ( (CLKDIV + 1) * (MSTSCLHIGH + MSTSCLLOW + 4) )
  Nominal SCL rate = I2C function clock rate / ( 4*CLKDIV + 4 )
  Nominal SCL rate * ( 4*CLKDIV + 4 ) = I2C function clock rate 
  Nominal SCL rate *  4*CLKDIV + Nominal SCL rate * 4  = I2C function clock rate 
  Nominal SCL rate *  4*CLKDIV = I2C function clock rate - Nominal SCL rate * 4
  CLKDIV = I2C function clock rate/(Nominal SCL rate *  4) - 1

  input clk	clkdiv	MSTSCLHIGH		MSTSCLLOW		Nominal Rate
  15			37		0				0				99 KHz
  15			9		0				0				375 KHz  
  12			29		0				0				100 KHz
  12			7		0				0				375 KHz
  
  15			29		0				1				100 KHz
  15			7		0				1				375 KHz
  12			5		0				1				400 KHz


  input clk	clkdiv	Nominal I2C Bus Clock
  15 Mhz		37		99 KHz
  15 Mhz		9		375 KHz  
  12 Mhz		29		100 KHz
  12 Mhz		7		375 KHz
  
  This setup does not use the fractional generator: 
  USART and I2C can be used in parallel

*/
void i2c_init(uint8_t clkdiv, uint8_t scl, uint8_t sda)
{
  Enable_Periph_Clock(CLK_IOCON);
  Enable_Periph_Clock(CLK_SWM);
  
  Enable_Periph_Clock(CLK_I2C0);
  Do_Periph_Reset(RESET_I2C0);
  
  *get_iocon_by_port(sda) |= 1<<IOCON_OD;
  *get_iocon_by_port(scl) |= 1<<IOCON_OD;
  map_function_to_port(I2C0_SDA, sda);
  map_function_to_port(I2C0_SCL, scl);


  /*
    15 MHz, I2C Busclock 100MHz
      CLKDIV = 37 
  */
  LPC_SYSCON->I2C0CLKSEL = 1;	/* main clock */
  
  LPC_I2C0->CLKDIV = clkdiv;
  LPC_I2C0->MSTTIME = 0;
    
  LPC_I2C0->CFG = CFG_MSTENA;

}

/*
  return:
    0: time out
    1: all ok
*/
int i2c_wait(void)
{
  uint32_t cnt = 0;
  while(!(LPC_I2C0->STAT & STAT_MSTPEND) )
  {
    cnt++;
    if ( cnt > (1<<17) )
      return 0;
  }
  return 1;
}


int i2c_write( uint8_t adr, uint8_t *buf, uint32_t  len )
{	
  uint32_t i;

  if ( i2c_wait() == 0 )
    return I2C_TIMEOUT_PRE_ADR;
  if ((LPC_I2C0->STAT & MASTER_STATE_MASK) != I2C_STAT_MSTST_IDLE)
    return I2C_NOT_IDLE;
  
  LPC_I2C0->MSTDAT = adr | 0;    					// Address with 0 for RWn bit (WRITE)
  LPC_I2C0->MSTCTL = CTL_MSTSTART;										// Start the transaction by setting the MSTSTART bit to 1 in the Master control register.
  if ( i2c_wait() == 0 )   // Wait for the address to be ACK'd
    return I2C_TIMEOUT_POST_ADR;
  if ((LPC_I2C0->STAT & MASTER_STATE_MASK) != I2C_STAT_MSTST_TX)
    return I2C_NO_TX_POST_ADR;
  
  for ( i = 0; i < len; i++ ) 
  {
    LPC_I2C0->MSTDAT = buf[i];               // Send the data to the slave
    LPC_I2C0->MSTCTL = CTL_MSTCONTINUE;                // Continue the transaction
    if ( i2c_wait() == 0 )  // Wait for the address to be ACK'd
      return I2C_TIMEOUT_TX;
    if ((LPC_I2C0->STAT & MASTER_STATE_MASK) != I2C_STAT_MSTST_TX)
      return I2C_NO_TX_POST_TX;
  }
  
  LPC_I2C0->MSTCTL = CTL_MSTSTOP;                    // Send a stop to end the transaction
  return I2C_OK;
}

/*====================================================*/
/* ring buffer implementation */


/* ring buffer */

/* 
  Prototype:
    void rb_clear(rb_t *rb)

  Description:
    Clear the data in the ring buffer.

  Precondition:
    rb_init() must be called on the "rb" argument.

  Parameter:
    rb: 	Address of a (initialized) buffer data structure

*/

void rb_clear(rb_t *rb)
{
  rb->start = 0;
  rb->end = 0;
}

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
void rb_init(rb_t *rb, uint8_t *buf, uint16_t len)
{
  rb->ptr = buf;
  rb->len = len;
  rb_clear(rb);
}

uint16_t rb_next_val(rb_t *rb, uint16_t val)
{
  val++;
  if ( val >= rb->len )
    val = 0;
  return val;
}

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
int rb_add(rb_t *rb, uint8_t data)
{
  uint32_t primask;
  uint16_t end;
  
  primask = __get_PRIMASK();	/* get the interrupt status */
  __disable_irq();			/* disable IRQs, this will modify PRIMASK */
  
  end = rb_next_val(rb, rb->end);
  if ( end == rb->start )
    return __set_PRIMASK(primask), 0; /* restore interrupt state & return error */
  rb->ptr[rb->end] = data;
  rb->end = end;
  __set_PRIMASK(primask);	/* restore interrupt state */
  return 1;
}

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
int rb_get(rb_t *rb)
{
  uint32_t primask;
  int data;
  if ( rb->end == rb->start )
    return -1;
  
  primask = __get_PRIMASK();	/* get the interrupt status */
  __disable_irq();			/* disable IRQs, this will modify PRIMASK */
  
  data = rb->ptr[rb->start];
  rb->start = rb_next_val(rb, rb->start);
  __set_PRIMASK(primask);	/* restore interrupt state */

  return data;  
}




/*====================================================*/



/*

  BRGVAL=5 		--> 115274.51935
  BRGVAL=11		--> 57637.25967
  BRGVAL=23		--> 28818.62983
  BRGVAL=47		--> 14409.31491
  BRGVAL=71		--> 9606.20994
  BRGVAL=575	--> 1200.77

  fclk = 15000000/(1+91/256) = 11066353.85773
  baud rate = 11066353.85773 / (16 * (BRGVAL+1)).

  8-N-1 configuration: Eight (8) data bits, no (N) parity bit, and one (1) stop bit

*/

void usart_init(LPC_USART_TypeDef * usart, uint32_t brgval)
{
  usart->BRG = brgval;
  usart->OSR = 15;
  
  usart->CTL = 0;
  usart->INTENSET = RXRDY;		/* enable RXRDY interrupt */
  
  usart->CFG =  UART_EN | DATA_LENG_8 | PARITY_NONE | STOP_BIT_1;
}

usart_t *usart0_struct_ptr;	// only for the IRQ handler

void usart0_init(usart_t *usart, uint32_t brgval, uint8_t tx, uint8_t rx, uint8_t *rx_buf, uint16_t rx_len)
{
  usart0_struct_ptr = usart;
  usart->usart = LPC_USART0;
  rb_init(&(usart->rb), rx_buf, rx_len);
  
  //Enable_Periph_Clock(CLK_IOCON);
  Enable_Periph_Clock(CLK_SWM);
  
  LPC_SYSCON->FRG0DIV = 255;
  LPC_SYSCON->FRG0MULT = 91;
  LPC_SYSCON->FRG0CLKSEL = FRGCLKSEL_MAIN_CLK;

  Enable_Periph_Clock(CLK_UART0);
  Do_Periph_Reset(RESET_UART0);  
 
  //*get_iocon_by_port(tx) = ..;
  //*get_iocon_by_port(rxl) = ..;
  map_function_to_port(U0_TXD, tx);
  map_function_to_port(U0_RXD, rx);

  
  LPC_SYSCON->UART0CLKSEL = FCLKSEL_FRG0CLK;
  //LPC_SYSCON->UART1CLKSEL = FCLKSEL_FRG0CLK;
  
  usart_init( LPC_USART0, brgval);
  
  NVIC_EnableIRQ(UART0_IRQn);
}

void __attribute__ ((interrupt)) UART0_Handler(void)
{
  if ( (usart0_struct_ptr->usart->STAT & RXRDY) != 0 )
  {
    rb_add(&(usart0_struct_ptr->rb), usart0_struct_ptr->usart->RXDAT);
  }
}


void usart_write_byte(usart_t *usart, uint8_t data)
{
  while( (usart->usart->STAT & TXRDY) == 0 )
    ;
  usart->usart->TXDAT = data;
}

void usart_write_bits(usart_t *usart, uint32_t bits, int cnt)
{
  uint32_t mask = 1<<(cnt-1);
  while( cnt > 0 )
  {
    if ( bits & mask )
    {
      usart_write_byte(usart, '1');
    }
    else
    {
      usart_write_byte(usart, '0');
    }
    mask >>= 1;
    cnt--;
  }
}

void usart_write_string(usart_t *usart, const char *s)
{
  if ( s == NULL )
    return;
  while( *s != '\0' )
    usart_write_byte(usart, *s++);
}

void usart_write_u16(usart_t *usart, uint16_t v)
{
  usart_write_string(usart, u16toa(v));
}

int usart_read_byte(usart_t *usart)
{
  return rb_get(&(usart0_struct_ptr->rb));
}


/*====================================================*/

static const char *u16toap(char * dest, uint16_t v)
{
  uint8_t pos;
  uint8_t d;
  uint16_t c;
  c = 10000;
  for( pos = 0; pos < 5; pos++ )
  {
      d = '0';
      while( v >= c )
      {
	v -= c;
	d++;
      }
      dest[pos] = d;
      c /= 10;
  }  
  dest[5] = '\0';
  return dest;
}

/* convert unsigned 16 bit value to decimal number */
const char *u16toa(uint16_t v)
{
  static char buf[6];
  const char *s = u16toap(buf, v);
  while( *s == '0' )
    s++;
  if ( *s == '\0' )
    s--;
  return s;
}


/*====================================================*/


/* 
  Description:
    Replacement for ConfigSWM(uint32_t func, uint32_t port_pin)
    This function will connect an internal signal to the specified GPIO port

  Precondition:
    Enable_Periph_Clock(CLK_SWM);

  Parameter:
    fn: A function number, e.g. T0_MAT0, see swm.h
    port: A port number for the GPIO port (0..30)

*/
void map_function_to_port(uint32_t fn, uint32_t port)
{
  /* first reset the pin assignment to 0xff (this is also the reset value */
  LPC_SWM->PINASSIGN[fn/4] |= ((0xffUL)<<(8*(fn%4)));
  /* then write the destination pin to it */
  LPC_SWM->PINASSIGN[fn/4] &= ~((port^255UL)<<(8*(fn%4)));
}

/*====================================================*/

static const uint8_t port_to_adc[17] = { 
  255, 
  0,		/* port 0_1 --> ADC_0 */
  255,
  255,
  11,		/* port 0_4 --> ADC_11 */
  255,
  255,
  1,		/* port 0_7 --> ADC_1 */
  5,		/* port 0_8 --> ADC_5 */
  4,		/* port 0_9 --> ADC_4 */
  7,		/* port 0_10 --> ADC_7 */
  6,		/* port 0_11 --> ADC_6 */
  255,
  10,		/* port 0_13 --> ADC_10 */
  2,		/* port 0_14 --> ADC_2 */
  8,		/* port 0_15 --> ADC_8 */
  3		/* port 0_16 --> ADC_3 */
};

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
int16_t get_adc_by_port(uint8_t port)
{
  uint8_t adc;
  if ( port >= 17 )
    return -1;
  adc = port_to_adc[port];
  if ( adc == 255 )
    return -1;
  return adc;
}


/* 

  Prototype:
    void adc_init(void)

  Description:
    Powerup ADC and enable clock for the ADC.
    Finally put the ADC into low power mode.

*/
void adc_init(void)
{

  LPC_SYSCON->PDRUNCFG &= ~ADC_PD;	/* power up ADC */
  LPC_SYSCON->ADCCLKSEL = 0;			/* FRO */
  LPC_SYSCON->ADCCLKDIV = 1;			/* devide by 1 */
  Enable_Periph_Clock(CLK_ADC);			/* enable clock for ADC */
  __NOP();								/* probably not required */
  LPC_ADC->CTRL = (1<<ADC_LPWRMODE) ;	
    /* clkdiv=0, enable low power mode */
}

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
uint16_t adc_read(uint8_t port)
{
  uint32_t result;
  uint32_t cnt = 0;
  int16_t adc = get_adc_by_port(port);
  if ( adc < 0 )
    return 0xffff;	/* invalid pin */
  
  GPIOSetDir( PORT0, port, INPUT);
  *get_iocon_by_port(port) &= IOCON_MODE_MASK;	/* clear any pullup/pulldown */
  /* if any pullup/pulldown had been active, then it make take several seconds until a reliable voltage is available! */
  
  LPC_SWM->PINENABLE0 &= ~(1<<(adc+10));	/* ADC_0 is at bit 10 */

  
  LPC_ADC->SEQA_CTRL = 1<<adc 				/* select the adc, clear all other bits */
    | (1<<ADC_TRIGPOL)						/* set the trigger polarity to low-high transition */
    | (1<<ADC_SEQ_ENA);						/* enable sequence a processing */
      
  LPC_ADC->SEQA_CTRL |= (1<<ADC_START);		/* start the conversion */
  
  /* wait for the result */
  for(;;)
  {
    result = LPC_ADC->DAT[adc];				/* read data reg, to clear the DATAVALID flag */
    if ( (result & 0x80000000) != 0 )
      break;
    cnt++;									/* max 4 with 15 MHz */
    if ( cnt >= 80 )							/* this is too much, return with timeout */
      return 0x0fffe;
  }
  
  LPC_ADC->CTRL = (1<<ADC_LPWRMODE) ;	
  
  return (result>>4) & 0x0fff;
}

/*=======================================================================*/
