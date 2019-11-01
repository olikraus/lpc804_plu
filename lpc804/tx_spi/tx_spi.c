
#include <LPC8xx.h>
#include <syscon.h>
#include <gpio.h>
#include <swm.h>
#include <spi.h>
#include <delay.h>


/*=======================================================================*/
/* Configuration */
#define SYS_TICK_PERIOD_IN_MS 100


/*=======================================================================*/
/* system procedures and sys tick master task */



volatile uint32_t sys_tick_irq_cnt=0;


void __attribute__ ((interrupt)) SysTick_Handler(void)
{  
  sys_tick_irq_cnt++;
}

/*=======================================================================*/
/* 
  replacement for ConfigSWM(uint32_t func, uint32_t port_pin) 
  
  Args:
    fn: A function number, e.g. T0_MAT0, see swm.h
    port: A port number for the GPIO port (0..30)

*/
void mapFunctionToPort(uint32_t fn, uint32_t port)
{
  /* first reset the pin assignment to 0xff (this is also the reset value */
  LPC_SWM->PINASSIGN[fn/4] |= ((0xffUL)<<(8*(fn%4)));
  /* then write the destination pin to it */
  LPC_SWM->PINASSIGN[fn/4] &= ~((port^255UL)<<(8*(fn%4)));
}
/*=======================================================================*/
/*
  setup the hardware and start interrupts.
  called by "Reset_Handler"
*/
int __attribute__ ((noinline)) main(void)
{

  /* call to the lpc lib setup procedure. This will set the IRC as clk src and main clk to 24 MHz */
  SystemInit(); 

  /* if the clock or PLL has been changed, also update the global variable SystemCoreClock */
  SystemCoreClockUpdate();

  /* set systick and start systick interrupt */
  SysTick_Config(main_clk/1000UL*(unsigned long)SYS_TICK_PERIOD_IN_MS);
  
  /* */
  GPIOInit();
  
  /* enable clock for several subsystems */

  Enable_Periph_Clock(CLK_IOCON);
  Enable_Periph_Clock(CLK_SWM);
  Enable_Periph_Clock(CLK_SPI0);
  
  GPIOSetDir( PORT0, 15, OUTPUT);
  GPIOSetDir( PORT0, 9, OUTPUT);

  GPIOSetDir( PORT0, 2, INPUT);

  mapFunctionToPort(SPI0_SCK, 15);
  mapFunctionToPort(SPI0_MOSI, 9);
  
  
  
  
  LPC_SYSCON->FRG0DIV = 0;
  LPC_SYSCON->FRG0MULT = 255;		/* FRG0_SRC_CLK will be deviced by 150 */
  LPC_SYSCON->FRG0CLKSEL = FRGCLKSEL_MAIN_CLK;
  LPC_SYSCON->SPI0CLKSEL = FCLKSEL_FRG0CLK;	/* FCLKSEL_FRG0CLK or FCLKSEL_MAIN_CLK */
  LPC_SPI0->DIV = 0xffff;		/* divide by max */
  
  /* bit clock will around 0.9 Bit/second --> ok to observe the LEDs */
  
  LPC_SPI0->TXCTL = 
    SPI_CTL_RXIGNORE | 		/* do not read data from MISO */
    SPI_CTL_LEN(8) | 			/* send 8 bits */
    SPI_TXDATCTL_SSELN(3); 	/* do not use any slave select */

  LPC_SPI0->CFG = SPI_CFG_ENABLE | SPI_CFG_MASTER;
  /* CPHA and CPOL are not set: Data is valid with rising edge of the clock signal */  
  //LPC_SPI0->CFG = SPI_CFG_ENABLE | SPI_CFG_MASTER|SPI_CFG_CPHA|SPI_CFG_CPOL;

  for(;;)
  {
    if ( GPIOGetPinValue(PORT0, 2) == 0 )
    {
      while( (LPC_SPI0->STAT & SPI_STAT_TXRDY) == 0 )
	;
      LPC_SPI0->TXDAT = 0x99;	/* test pattern: 10011001 */
      //LPC_SPI0->TXDAT = 0xf0;	/* test pattern: 1111000 */
    }
  }
}
