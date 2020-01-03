/*

  uart_monitor.c

  
  Configuration is 115200 8-N-1. 
  "\r\n" will be sent. 
  
  commands:
  lb <adr>		list bytes
  lw <adr>		list words
  lq <adr>		list 32bit words

*/

#include <stddef.h>
#include <LPC8xx.h>
#include <syscon.h>
#include <gpio.h>
#include <swm.h>
#include <uart.h>
#include <delay.h>
#include <util.h>
#include "ep.h"


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

usart_t usart;
uint8_t usart_rx_buf[32];
ep_t ep;

#define EP_LINE_MAX 128
char ep_line_buf[EP_LINE_MAX];
int ep_line_pos = 0;



void ep_usart_write_byte(ep_t *ep, int c)
{
  usart_write_byte(&usart, c);
}

void ep_line_add_char(int c)
{
  /*
  if ( ep_line_pos == 0 )
    if ( c == '\r' || c == '\n' )
      return;
  */
  if ( c == '\n' || c == '\r' )
  {
    ep_parse_cmd(&ep, ep_line_buf);
    ep_line_pos = 0;
    ep_line_buf[ep_line_pos] = '\0'; 

    
    return;
  }
  
  if ( ep_line_pos >= EP_LINE_MAX-1 )
    return;

  ep_line_buf[ep_line_pos++] = c;
  ep_line_buf[ep_line_pos] = '\0';
}

int __attribute__ ((noinline)) main(void)
{
  int data;

  

  /* call to the lpc lib setup procedure. This will set the IRC as clk src and main clk to 24 MHz */
  SystemInit(); 

  /* if the clock or PLL has been changed, also update the global variable SystemCoreClock */
  SystemCoreClockUpdate();

  /* set systick and start systick interrupt */
  SysTick_Config(main_clk/1000UL*(unsigned long)SYS_TICK_PERIOD_IN_MS);

  ep_init(&ep, ep_usart_write_byte);
  
  GPIOInit();
  
  /* enable clock for several subsystems */

  Enable_Periph_Clock(CLK_IOCON);
  Enable_Periph_Clock(CLK_SWM);

  usart0_init(&usart, 5, /* tx */ 4, /* rx */ 0, usart_rx_buf, 32);  

  GPIOSetDir( PORT0, 15, OUTPUT);

  *(uint32_t *)0xE0002000 = 0x0ffffffff;
  *(uint32_t *)0xE000EDFC |= 1<<24;	// enable DWT at 0x0e0001000
  

  for(;;)
  {
    GPIOSetBitValue(PORT0, 15, 1);
    delay_micro_seconds(100000);
    GPIOSetBitValue(PORT0, 15, 0);
    delay_micro_seconds(100000);    
    for(;;)
    {
      data = usart_read_byte(&usart);
      if ( data < 0 )
	break;
      
      if ( data == '\r' )
	usart_write_byte(&usart, '\n');
      if ( data == '\r' || data >= 32 )
      {
	usart_write_byte(&usart, data);
	ep_line_add_char(data);
      }
      else
      {
	usart_write_byte(&usart, '{');
	ep_out_num(&ep, data, 16, 2);
	usart_write_byte(&usart, '}');
      }
	
    }
  }
}

