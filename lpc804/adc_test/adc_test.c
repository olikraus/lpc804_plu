/*

  adc_test.c

  minicom -D /dev/ttyUSB0
  
  Output the ADC values for GPIO ports 0_7, 0_8 and 0_15

*/

#include <stddef.h>
#include <LPC8xx.h>
#include <syscon.h>
#include <gpio.h>
#include <iocon.h>
#include <swm.h>
#include <uart.h>
#include <delay.h>
#include <util.h>


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
  uint16_t cnt = 0;

  usart_t usart;
  uint8_t usart_rx_buf[32];
  

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
  
  /* 
    disable pullups for those pins, which are used as ADC 
    This is not required, because adc_read() will do this also, but
    results will be better if the pullups are disabled as early as possible.
  */
  *get_iocon_by_port(8) &= IOCON_MODE_MASK;
  *get_iocon_by_port(15) &= IOCON_MODE_MASK;
  *get_iocon_by_port(7) &= IOCON_MODE_MASK;

  /* init usart to output ADC results to a connected terminal */
  usart0_init(&usart, 5, /* tx */ 4, /* rx */ 0, usart_rx_buf, 32);  

  /* assume a LED at port 0_2 and let the LED flash during the adc measurements */
  LPC_SWM->PINENABLE0 |= SWDIO;	// disable SWDIO @PIN0_2 so that we can use the LED
  GPIOSetDir( PORT0, 2, OUTPUT);

  /* init adc */
  adc_init();

  for(;;)
  {
    GPIOSetBitValue(PORT0, 2, 1);
    delay_micro_seconds(20000);
    GPIOSetBitValue(PORT0, 2, 0);
    delay_micro_seconds(500000);
    usart_write_string(&usart, "cnt: ");
    usart_write_string(&usart, u16toa(cnt));
    usart_write_string(&usart, "\r\n");
    
    usart_write_string(&usart, "port 0_8 (in): ");
    usart_write_string(&usart, u16toa(adc_read(8)));
    usart_write_string(&usart, "\r\n");

    usart_write_string(&usart, "port 0_15 (out1): ");
    usart_write_string(&usart, u16toa(adc_read(15)));
    usart_write_string(&usart, "\r\n");

    usart_write_string(&usart, "port 0_7 (out2): ");
    usart_write_string(&usart, u16toa(adc_read(7)));
    usart_write_string(&usart, "\r\n");
    
    cnt++;
  }
}

