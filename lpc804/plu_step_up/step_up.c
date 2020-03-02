/*

  step_up.c

  
*/


#include "LPC8xx.h"
#include "iocon.h"
#include "syscon.h"
#include "gpio.h"
#include "swm.h"
#include "acomp.h"
#include "delay.h"
#include "util.h"

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
int __attribute__ ((noinline)) main(void)
{
  /* call to the lpc lib setup procedure. This will set the IRC as clk src and main clk to 24 MHz */
  SystemInit(); 

  /* if the clock or PLL has been changed, also update the global variable SystemCoreClock */
  SystemCoreClockUpdate();

  /* set systick and start systick interrupt */
  SysTick_Config(main_clk/1000UL*(unsigned long)SYS_TICK_PERIOD_IN_MS);

  
  plu();		/* plu() will enable GPIO0 clock */
  
  Enable_Periph_Clock(CLK_IOCON);
  Enable_Periph_Clock(CLK_ACMP);
  
  *get_iocon_by_port(1) &= IOCON_MODE_MASK;	/* clear pullup on the comparator */
  

  LPC_SYSCON->PDRUNCFG &= ~ACMP_PD;	/* power up analog comparator */

  LPC_SWM->PINENABLE0 &= ~ACMP_I2;		/* enable ACMP input at PIN0_1 */
  
  LPC_CMP->LAD = 1 | (2<<1);				/* enable ladder, 2*3.3V/31 = 0.213V */

  /* compare the ladder output against voltage at PIO0_1 */
  LPC_CMP->CTRL =
    ( 1 << COMPSA ) |					/* sync with bus clock */
    (ACOMP_IN2 << COMP_VP_SEL) |			/* PIO0_1 */
    (V_LADDER_OUT << COMP_VM_SEL);		/* 0.213V */
  
}
