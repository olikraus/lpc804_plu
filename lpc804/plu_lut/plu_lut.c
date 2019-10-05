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
  
  /* init GPIOs */
  GPIOInit();
  
  /* enable clock for several subsystems */
  Enable_Periph_Clock(CLK_IOCON);
  Enable_Periph_Clock(CLK_SWM);
  Enable_Periph_Clock(CLK_PLU);

  /* Configure input buttons, use pullup resistors */
  
  GPIOSetDir( PORT0, 2, INPUT);
  LPC_IOCON->PIO0_2 &= IOCON_MODE_MASK;
  LPC_IOCON->PIO0_2 |= MODE_PULLUP;
  GPIOSetDir( PORT0, 10, INPUT);
  LPC_IOCON->PIO0_10 &= IOCON_MODE_MASK;
  LPC_IOCON->PIO0_10 |= MODE_PULLUP;

  /* Configure output LED */

  GPIOSetDir( PORT0, 15, OUTPUT);

  /* Assign GPIO inputs to PLU input and output lines */
  
  /* 1. write 0 to make a defailt assignment */
  LPC_SWM->PINASSIGN_4PIN = 0;
  /* 2. Assign PIO0_2 to PLU_INPUT2 */
  LPC_SWM->PINASSIGN_4PIN |= PLU_INPUT2_PIO0_2;
  /* 3. PIO0_10 can not be assigned any more, instead assign unused PIO0_22 */
  GPIOSetDir( PORT0, 22, OUTPUT);
  LPC_SWM->PINASSIGN_4PIN |= PLU_INPUT5_PIO0_22;
  /* 4. Unused PLU inputs must be cleared */  
  LPC_SWM->PINASSIGN_4PIN |= PLU_INPUT0_NONE;
  LPC_SWM->PINASSIGN_4PIN |= PLU_INPUT1_NONE;
  LPC_SWM->PINASSIGN_4PIN |= PLU_INPUT3_NONE;
  LPC_SWM->PINASSIGN_4PIN |= PLU_INPUT4_NONE;
  /* 5. Connect PLU output 1 to PIO0_15 */
  LPC_SWM->PINASSIGN_4PIN |= PLU_OUTPUT1_PIO0_15;
  /* 6. Make all other PLU outputs invald */
  LPC_SWM->PINASSIGN_4PIN |= PLU_OUTPUT0_NONE;
  LPC_SWM->PINASSIGN_4PIN |= PLU_OUTPUT2_NONE;
  LPC_SWM->PINASSIGN_4PIN |= PLU_OUTPUT3_NONE;
  LPC_SWM->PINASSIGN_4PIN |= PLU_OUTPUT4_NONE;
  LPC_SWM->PINASSIGN_4PIN |= PLU_OUTPUT5_NONE;
  LPC_SWM->PINASSIGN_4PIN |= PLU_OUTPUT6_NONE;
  LPC_SWM->PINASSIGN_4PIN |= PLU_OUTPUT7_NONE;
  
  /* Route PIO_10 to PIO22 */
  
  /* read from PIO0_10 */
  mapFunctionToPort(LVLSHFT_IN0, 10);  
  /* write to PIO0_22 */
  mapFunctionToPort(LVLSHFT_OUT0, 22);

  /* Assign LUT0 output to PLU output 1 */ 

  LPC_PLU0->OUTPUT_MUX[1] = 0;
  
  /* Configure LUT0 inputs */
  
  LPC_PLU0->LUT_MUX[0].INP0 = 2;		/* PLU input 2 */
  LPC_PLU0->LUT_MUX[0].INP1 = 5;		/* PLU input 5 */
  LPC_PLU0->LUT_MUX[0].INP2 = 63;
  LPC_PLU0->LUT_MUX[0].INP3 = 63;
  LPC_PLU0->LUT_MUX[0].INP4 = 63;
  
  /* Assign LUT0 truth table value */
  
  LPC_PLU0->LUT_TRUTH[0] = 8;
  
}