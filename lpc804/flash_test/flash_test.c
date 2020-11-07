/*

  uart_test.c

  This example will sent "Hello World\n" via USART0 interface.
  
  Configuration is 115200 8-N-1. Only "\n" is sent. Receiving terminal should add \r
  (e.g. use add CR function in "minicom")
  
  Unix: "tail -f /dev/ttyUSB0" should also work.

*/

#include <stddef.h>
#include <LPC8xx.h>
#include <syscon.h>
#include <gpio.h>
#include <swm.h>
#include <uart.h>
#include <delay.h>
#include <util.h>
#include <flash.h>


/*=======================================================================*/
/* Configuration */
#define SYS_TICK_PERIOD_IN_MS 100


/*=======================================================================*/
/* system procedures and sys tick master task */



volatile uint32_t sys_tick_irq_cnt=0;


void __attribute__ ((interrupt)) SysTick_Handler(void)
{  
  sys_tick_irq_cnt++;
  GPIOSetBitValue(PORT0, 9, (sys_tick_irq_cnt & 1) == 0?0:1);
}



/*=======================================================================*/
/*
  setup the hardware and start interrupts.
  called by "Reset_Handler"
*/
int __attribute__ ((noinline)) main(void)
{
  uint16_t data = 1234;
  uint16_t status;

  usart_t usart;
  uint8_t usart_rx_buf[32];

  __attribute__((aligned(8))) uint8_t flash_buf[64];
  

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

  usart0_init(&usart, 5, /* tx */ 4, /* rx */ 0, usart_rx_buf, 32);  

  GPIOSetDir( PORT0, 15, OUTPUT);


  delay_micro_seconds(1000000);
  
  usart_write_string(&usart, "Read Data = ");    
  usart_write_u16(&usart, *(uint16_t *)0x7f00);
  usart_write_string(&usart, "\n");


  for(;;)
  {
    delay_micro_seconds(4000000);
    
    GPIOSetBitValue(PORT0, 15, 1);
    delay_micro_seconds(1000000);
    GPIOSetBitValue(PORT0, 15, 0);
    delay_micro_seconds(1000000);
    usart_write_string(&usart, "Flash Test Data=");    
    usart_write_u16(&usart, data);
    usart_write_string(&usart, "\n");
    
    usart_write_string(&usart, "Erase Page: ");    
    status = flash_erase_page(0x07f00);
    usart_write_u16(&usart, status);
    usart_write_string(&usart, "\n");
    
    *(uint16_t *)flash_buf = data;
    usart_write_string(&usart, "Write Page: ");    
    status = flash_write_page(0x07f00, flash_buf);
    usart_write_u16(&usart, status);
    usart_write_string(&usart, "\n");

    if ( *(uint16_t *)0x7f00 == data )
      usart_write_string(&usart, "Compare Ok\n");
    else
      usart_write_string(&usart, "Compare Failed\n");
      
    data += 7;
    
    *(uint16_t *)flash_buf = data;
    usart_write_string(&usart, "Erase and Write Page: ");    
    status = flash_page(0x07f00, flash_buf);
    usart_write_u16(&usart, status);
    usart_write_string(&usart, "\n");

    if ( *(uint16_t *)0x7f00 == data )
      usart_write_string(&usart, "Compare Ok\n");
    else
      usart_write_string(&usart, "Compare Failed\n");
    
  }
}

