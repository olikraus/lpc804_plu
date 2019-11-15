/*

  plu_sw_input.c


  This is an example to analyse the SWM.
  
  PIO0_2:
    Used as input for LVLSHFT 0
    This will enforce PIO_2 as input!!
    In IOCON for PIO0_2, the inverter is enabled for 100ms and disabled for 900ms
    
  PIO0_10
    Used as output (destination) for LVLSHFT 0
    Used as input for LUT 0
    This will enforce PIO_10 as input!!
    
  PIO0_15
    Configured as output (LED)
    Connected to output of LUT 0
    
  PIO0_9
    Configured as output (LED)
    Reference blink, level will be set by software
    Set for 100ms and off for 900ms (same as PIO0_2 inverter)
    
  LUT0:
    Configured as pass through, will just pass input 0 as output.
  
  Observation:
    LED at PIO0_9 will flash for 100ms (and stay dark for 900ms)
    Without pressing any button:
      LED at PIO0_15 will be on for 900ms and dark for 100ms
    Pressing PIO_10 button:
      LED at PIO0_15 will be permanently off
    Pressing PIO_2 button:
      LED at PIO0_15 will be on for 100ms and dark for 900ms: PIO_2 inverts 
      default behavior
      
  Results
    1) If at least one input function is connected to a GPIO pin, then the
         GPIO is configured as input. This seems to be similar to the sentence
	 in the user manual: 
	 "When you assign any function to a pin through the switch matrix, 
	 the GPIO output becomes disabled."
	 However the correct sentence in the user manual probably should be:
	 "When you assign any *input* function to a pin through the switch matrix, 
	 the GPIO output becomes disabled."
    2) According to the datasheet "It is not allowed to connect more than one 
        output or bidirectional function to a pin." However if a input function is
	connected to the pin, then the pin can get input from the level at the
	outside world. In such a case, the outside level wins and overrides the
	internal output function: This can be seen when PIO0_10 button is pressed.
	This will override the output of the LVLSHFT0 output.
    3) It is impossible to manually write a value to an input function via the GPIO
        port value. For a such a value, the GPIO must be configured as output, but
	because the GPIO is forced to be input whenever an input function is assigned,
	such writing of a value will not work.
    4) As a result of 3), the only way to change the value of an input pin is the
	inverter function of IOCON:
	a) configure pullup or pulldown and b) change the value via inverter function
    5) From my work on this code, it looks like, that the LVLSHFT function from
	PIO0_10 to PIO0_2 doesn't work. However the opposite direction works
	without problems.
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
  Enable_Periph_Clock(CLK_PLU);

  /* Configure input buttons, use pullup resistors */
  
  LPC_IOCON->PIO0_2 &= IOCON_MODE_MASK;
  LPC_IOCON->PIO0_2 |= MODE_PULLUP;

  LPC_IOCON->PIO0_10 &= IOCON_MODE_MASK;
  LPC_IOCON->PIO0_10 |= MODE_PULLUP;

  /* configuration of PIO0_2 and 10 as output is ignored, they are both forced to be inputs */
  //GPIOSetDir( PORT0, 2, OUTPUT);
  //GPIOSetDir( PORT0, 10, OUTPUT);
  
  /* Configure output LEDs */
  GPIOSetDir( PORT0, 9, OUTPUT);
  GPIOSetDir( PORT0, 15, OUTPUT);

  /* Assign GPIO inputs to PLU input and output lines */
  
  LPC_SWM->PINASSIGN_4PIN = 0;
  LPC_SWM->PINASSIGN_4PIN |= PLU_INPUT0_NONE;
  LPC_SWM->PINASSIGN_4PIN |= PLU_INPUT1_NONE;
  LPC_SWM->PINASSIGN_4PIN |= PLU_INPUT2_PIO0_10;  
  LPC_SWM->PINASSIGN_4PIN |= PLU_INPUT3_NONE;
  LPC_SWM->PINASSIGN_4PIN |= PLU_INPUT4_NONE;
  LPC_SWM->PINASSIGN_4PIN |= PLU_INPUT5_NONE;
  
  LPC_SWM->PINASSIGN_4PIN |= PLU_OUTPUT0_NONE;
  LPC_SWM->PINASSIGN_4PIN |= PLU_OUTPUT1_PIO0_15;
  LPC_SWM->PINASSIGN_4PIN |= PLU_OUTPUT2_NONE;
  LPC_SWM->PINASSIGN_4PIN |= PLU_OUTPUT3_NONE;
  LPC_SWM->PINASSIGN_4PIN |= PLU_OUTPUT4_NONE;
  LPC_SWM->PINASSIGN_4PIN |= PLU_OUTPUT5_NONE;
  LPC_SWM->PINASSIGN_4PIN |= PLU_OUTPUT6_NONE;
  LPC_SWM->PINASSIGN_4PIN |= PLU_OUTPUT7_NONE;
  
  
  /* read from PIO0_2 */
  map_function_to_port(LVLSHFT_IN0, 2);
  /* write to PIO0_10 */
  map_function_to_port(LVLSHFT_OUT0, 10);

  /* Assign LUT0 output to PLU output 1: Let OUTPUT1 read from LUT 0 */

  LPC_PLU0->OUTPUT_MUX[1] = 0;
  
  /* Configure LUT0 inputs */
  
  LPC_PLU0->LUT_MUX[0].INP0 = 2;		/* PLU INPUT2 */
  LPC_PLU0->LUT_MUX[0].INP1 = 63;		
  LPC_PLU0->LUT_MUX[0].INP2 = 63;
  LPC_PLU0->LUT_MUX[0].INP3 = 63;
  LPC_PLU0->LUT_MUX[0].INP4 = 63;
  
  /* Assign LUT0 truth table value */
  
  LPC_PLU0->LUT_TRUTH[0] = 2;
  
  for(;;)
  {
    LPC_GPIO_PORT->SET0 = 1<<9;
    LPC_IOCON->PIO0_2 |= 1<<6;
    delay_micro_seconds(100000);
    LPC_GPIO_PORT->CLR0 = 1<<9;
    LPC_IOCON->PIO0_2 &= ~(1<<6);
    delay_micro_seconds(900000);
  }
  
}