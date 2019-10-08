/*

  plu_lut.c

  Test PLU LUT
  Use buttons at PIO0_2 and PIO0_10 to implement a AND function.
  Both buttons must be pressed to enlight LED at PIO0_15,
  
*/


#include <LPC8xx.h>
#include <iocon.h>
#include <syscon.h>
#include <gpio.h>
#include <swm.h>
#include <ctimer.h>
#include <delay.h>

/*=======================================================================*/
/* externals */
void plu(void);

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
int __attribute__ ((noinline)) main(void)
{
  /* call to the lpc lib setup procedure. This will set the IRC as clk src and main clk to 24 MHz */
  SystemInit(); 

  /* if the clock or PLL has been changed, also update the global variable SystemCoreClock */
  SystemCoreClockUpdate();

  /* set systick and start systick interrupt */
  SysTick_Config(main_clk/1000UL*(unsigned long)SYS_TICK_PERIOD_IN_MS);

  
  plu();		/* plu() will enable GPIO0 clock */
}