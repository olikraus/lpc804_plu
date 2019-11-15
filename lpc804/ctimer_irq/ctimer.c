/*

  ctimer.c
  
  Use ctimer to do a LED blink
  
  Assumes LED at PIO0_15
  LED will flash for 100ms and stays dark for 900ms.
  
    - CTIMER is configures as PWM
    - CTIMER output is routed via PIN0_30 to PIN_15 (just to test LVLSHFT0)
    
*/


#include "LPC8xx.h"
#include "iocon.h"
#include "syscon.h"
#include "gpio.h"
#include "swm.h"
#include "ctimer.h"
#include "delay.h"
#include "util.h"


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
/* replace the CTIMER0_Handler handler, see startup.c for all the handler names */
void __attribute__ ((interrupt)) CTIMER0_Handler(void)
{  
  LPC_GPIO_PORT->NOT0 = 1<<9;		/* toggle port 9 */
  LPC_CTIMER0->IR |= 1<<MR3INT;	/* clear interrupt request*/
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
  
  /* init GPIOs */
  GPIOInit();
  
  /* enable clock for several subsystems */
  Enable_Periph_Clock(CLK_IOCON);
  Enable_Periph_Clock(CLK_SWM);
  Enable_Periph_Clock(CLK_CTIMER0);
  
  GPIOSetDir( PORT0, 15, OUTPUT);
  GPIOSetDir( PORT0, 9, OUTPUT);
  
  LPC_CTIMER0->PR = 0;	/* no prescale */
  LPC_CTIMER0->MR[3] = 15000000;  /* PWM cycle length: one second with 15MHz AHB clock */
  LPC_CTIMER0->MCR |= 1<<MR3R;		/* Use MR3 value to reset the counter: MR3 is the upper value and sets the PWM cycle */
  LPC_CTIMER0->MCR |= 1<<MR3I;		/* Generate interrupt for any match between counter and MR[3] */
  LPC_CTIMER0->MR[0] = 14000000;  /* PWM duty cycle in MR0 */
  LPC_CTIMER0->PWMC |= 1<<PWMEN0;  /* PWM mode for MR0 */
  
  /* connect subsystems to the GPIOs */
  /* Just for testing: The signal is routed via PIN0_30 */
  map_function_to_port(T0_MAT0, 30);
  map_function_to_port(LVLSHFT_IN0, 30);  
  map_function_to_port(LVLSHFT_OUT0, 15);
  
  /* configure interrupt */
  NVIC_EnableIRQ(CTIMER0_IRQn);

  /* enable the timer */
  LPC_CTIMER0->TCR |= 1<<CEN;
}
