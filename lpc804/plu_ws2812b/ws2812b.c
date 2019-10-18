/*

  ws2812b.c

  
  go <= PIO0_17;
  b0 <= PIO0_19;
  b1 <= PIO0_20;
  b2 <= PIO0_21;
  b3 <= PIO0_22;

  PIO0_16 <= data;
  PIO0_23 <= active;

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
  LPC_GPIO_PORT->B0[19] = x&1;
  x >>= 1;
  LPC_GPIO_PORT->B0[20] = x&1;
  x >>= 1;
  LPC_GPIO_PORT->B0[21] = x&1;
  x >>= 1;
  LPC_GPIO_PORT->B0[22] = x&1;  
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
    go <= PIO0_17;

    PIO0_16 <= data;
    PIO0_23 <= active;
  */
  GPIOSetDir( PORT0, 15, OUTPUT);


  LPC_IOCON->PIO0_17 &= IOCON_MODE_MASK;
  
  GPIOSetDir( PORT0, 17, OUTPUT);
  GPIOSetDir( PORT0, 19, OUTPUT);
  GPIOSetDir( PORT0, 20, OUTPUT);
  GPIOSetDir( PORT0, 21, OUTPUT);
  GPIOSetDir( PORT0, 22, OUTPUT);

    for(;;)
    {
      LPC_GPIO_PORT->B0[17] = 0;
      LPC_GPIO_PORT->B0[17] = 1;
    }
    
  for(;;)
  {
    while ( LPC_GPIO_PORT->B0[23] != 0 )
      ;
    output_4bits(0xf);
    LPC_GPIO_PORT->B0[17] = 1;
    while ( LPC_GPIO_PORT->B0[23] == 0 )
      ;
    LPC_GPIO_PORT->B0[17] = 0;
    
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
