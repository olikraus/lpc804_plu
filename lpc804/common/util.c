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

#include "LPC8xx.h"
#include "syscon.h"
#include "i2c.h"
#include "util.h"

/*
  Usually you would do a 
    LPC_IOCON->PIO0_7 
  to access the IOCON register for PIO0_7
  In cases, where you have only the number 7 this function will return the 
  corresponding register:
    *get_iocon_by_port(7)
  is the same as the above statement.
*/
__IO uint32_t *get_iocon_by_port(uint8_t port)
{
  return (__IO uint32_t *)LPC_IOCON_BASE;
}

/*
  
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

*/
void i2c_init(uint8_t clkdiv)
{
  
  Enable_Periph_Clock(CLK_I2C0);
  Do_Periph_Reset(RESET_I2C0);
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



/* 
  Description:
    Replacement for ConfigSWM(uint32_t func, uint32_t port_pin)
    This function will connect an internal signal to the specified GPIO port
  
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
