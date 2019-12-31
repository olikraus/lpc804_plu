/*

  uart_test.c

  This example will sent "Hello World\n" via USART0 interface.
  
  Configuration is 115200 8-N-1. Only "\n" is sent. Receiving terminal should add \r
  (e.g. use add CR function in "minicom")

*/

#include <stddef.h>
#include <LPC8xx.h>
#include <syscon.h>
#include <gpio.h>
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
  GPIOSetBitValue(PORT0, 9, (sys_tick_irq_cnt & 1) == 0?0:1);
}

/*=======================================================================*/
/*

  BRGVAL=5 		--> 115274.51935
  BRGVAL=11		--> 57637.25967
  BRGVAL=23		--> 28818.62983
  BRGVAL=47		--> 14409.31491
  BRGVAL=71		--> 9606.20994
  BRGVAL=575	--> 1200.77

  fclk = 15000000/(1+91/256) = 11066353.85773
  baud rate = 11066353.85773 / (16 * (BRGVAL+1)).

  8-N-1 configuration: Eight (8) data bits, no (N) parity bit, and one (1) stop bit

*/

void usart_init(LPC_USART_TypeDef * usart, uint32_t brgval)
{
  usart->BRG = brgval;
  usart->OSR = 15;
  
  usart->CTL = 0;
  usart->CFG =  UART_EN | DATA_LENG_8 | PARITY_NONE | STOP_BIT_1;
}

LPC_USART_TypeDef   *usart0_init(uint32_t brgval, uint8_t tx, uint8_t rx)
{
  //Enable_Periph_Clock(CLK_IOCON);
  Enable_Periph_Clock(CLK_SWM);
  
  LPC_SYSCON->FRG0DIV = 255;
  LPC_SYSCON->FRG0MULT = 91;
  LPC_SYSCON->FRG0CLKSEL = FRGCLKSEL_MAIN_CLK;

  Enable_Periph_Clock(CLK_UART0);
  Do_Periph_Reset(RESET_UART0);  
 
  //*get_iocon_by_port(tx) = ..;
  //*get_iocon_by_port(rxl) = ..;
  map_function_to_port(U0_TXD, tx);
  map_function_to_port(U0_RXD, rx);

  
  LPC_SYSCON->UART0CLKSEL = FCLKSEL_FRG0CLK;
  //LPC_SYSCON->UART1CLKSEL = FCLKSEL_FRG0CLK;
  
  usart_init( LPC_USART0, brgval);

  return LPC_USART0;
}

void usart_write_byte(LPC_USART_TypeDef * usart, uint8_t data)
{
  while( (usart->STAT & TXRDY) == 0 )
    ;
  usart->TXDAT = data;
}

void usart_write_string(LPC_USART_TypeDef * usart, char *s)
{
  if ( s == NULL )
    return;
  while( *s != '\0' )
    usart_write_byte(usart, *s++);
}

/*=======================================================================*/
/*
  setup the hardware and start interrupts.
  called by "Reset_Handler"
*/
int __attribute__ ((noinline)) main(void)
{
  LPC_USART_TypeDef *usart;

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

  usart = usart0_init(5, /* tx */ 4, /* rx */ 0);  

  GPIOSetDir( PORT0, 15, OUTPUT);

  for(;;)
  {
    GPIOSetBitValue(PORT0, 15, 1);
    delay_micro_seconds(1000000);
    GPIOSetBitValue(PORT0, 15, 0);
    delay_micro_seconds(1000000);
    usart_write_string(usart, "Hello World\n");
  }
}

