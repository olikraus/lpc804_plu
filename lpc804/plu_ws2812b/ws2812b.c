/*

  ws2812b.c

  
  go <= PIO0_18;
  b0 <= PIO0_19;
  b1 <= PIO0_20;
  b2 <= PIO0_21;
  b3 <= PIO0_22;

  PIO0_16 <= data;
  PIO0_17 <= active;

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
  b0 <= PIO0_19;
  b1 <= PIO0_20;
  b2 <= PIO0_21;
  b3 <= PIO0_22;

*/
void output_4bits(int x)
{
  if ( x&1)
    LPC_IOCON->PIO0_19 &= ~(1<<6);
  else
    LPC_IOCON->PIO0_19 |= (1<<6);
 
/*  
  x >>= 1;
  
  if ( x&1)
    LPC_IOCON->PIO0_20 &= ~(1<<6);
  else
    LPC_IOCON->PIO0_20 |= (1<<6);
  
  x >>= 1;
  
  if ( x&1)
    LPC_IOCON->PIO0_21 &= ~(1<<6);
  else
    LPC_IOCON->PIO0_21 |= (1<<6);
  
  x >>= 1;
  if ( x&1)
    
    LPC_IOCON->PIO0_22 &= ~(1<<6);
  else
    LPC_IOCON->PIO0_22 |= (1<<6);
    */
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

  
  GPIOInit();
  Enable_Periph_Clock(CLK_IOCON);
  Enable_Periph_Clock(CLK_SWM);
  
  
  plu();		/* plu() will enable GPIO0 clock */


  /*
    go <= PIO0_18;

    PIO0_16 <= data;
    PIO0_17 <= active;
  */
  GPIOSetDir( PORT0, 15, OUTPUT);

  /* clear the go flag */
  LPC_IOCON->PIO0_18 |= (1<<6);

  for(;;)
  {
    LPC_IOCON->PIO0_18 &= ~(1<<6);
    delay_system_ticks(200*2);
    LPC_IOCON->PIO0_18 |= (1<<6);
    delay_system_ticks(200*2);
    
    /*
    while ( LPC_GPIO_PORT->B0[17] != 0 )
      ;
    output_4bits(0x5);
    //LPC_GPIO_PORT->B0[18] = 1;
    LPC_IOCON->PIO0_18 &= ~(1<<6);
    while ( LPC_GPIO_PORT->B0[17] == 0 )
      ;
    //LPC_GPIO_PORT->B0[18] = 0;
    LPC_IOCON->PIO0_18 |= (1<<6);
    */
    if ( sys_tick_irq_cnt & 1 )
    {
      LPC_GPIO_PORT->B0[15] = 0;
    }
    else
    {
      LPC_GPIO_PORT->B0[15] = 1;
    }
  }
}
