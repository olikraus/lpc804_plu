
#include "LPC8xx.h"
#include "acomp.h"
#include "swm.h"
#include "syscon.h"
#include "gpio.h"
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
/*
  setup the hardware and start interrupts.
  called by "Reset_Handler"
*/
int __attribute__ ((noinline)) main(void)
{
  int i;
  int v;
  /* call to the lpc lib setup procedure. This will set the IRC as clk src and main clk to 24 MHz */
  SystemInit(); 

  /* if the clock or PLL has been changed, also update the global variable SystemCoreClock */
  SystemCoreClockUpdate();

  /* set systick and start systick interrupt */
  SysTick_Config(main_clk/1000UL*(unsigned long)SYS_TICK_PERIOD_IN_MS);
  
  GPIOInit();
  
  /* enable clock for several subsystems */

  Enable_Periph_Clock(CLK_IOCON);
  Enable_Periph_Clock(CLK_SWM);
  Enable_Periph_Clock(CLK_ACMP);
  

  LPC_SYSCON->PDRUNCFG &= ~ACMP_PD;	/* power up analog comparator */

  LPC_CMP->LAD = 1;				/* enable ladder */


  /* compare the ladder output against the 0.9V bandgap */
  LPC_CMP->CTRL =
    ( 1 << COMPSA ) |					/* sync with bus clock */
    (V_LADDER_OUT << COMP_VP_SEL) |
    (V_BANDGAP << COMP_VM_SEL);
    
  /* connect comparator output to PIO0_15 */
  map_function_to_port(ACOMP, 15);		/* ACOMP / COMP0_OUT */


  GPIOSetDir( PORT0, 15, OUTPUT);
  GPIOSetDir( PORT0, 9, OUTPUT);

  /* loop over all ladder values */
  v = 0;
  for(;;)
  {
  
    LPC_CMP->LAD = (v << 1) | 1; /* enable ladder + assign ladder value */
    
    for( i = 0; i < 15	; i++ )
    {
      GPIOSetBitValue(PORT0, 9, 1);
      delay_micro_seconds(100*v);
      GPIOSetBitValue(PORT0, 9, 0);
      delay_micro_seconds(100*(31-v));
    }
    
    v += 1;
    v &= 31;
    
  }
}
