/*

  uart_test.c

  This example will sent "Hello World\n" via USART0 interface.
  
  Configuration is 115200 8-N-1. Only "\n" is sent. Receiving terminal should add \r
  (e.g. use add CR function in "minicom")
  
  Received chars are echoed back enclosed in "<?>" once per second.
  
  Unix: "tail -f /dev/ttyUSB0" should also work.

*/

#include <stddef.h>
#include <LPC8xx.h>
#include <syscon.h>
#include <iocon.h>
#include <gpio.h>
#include <swm.h>
#include <uart.h>
#include <delay.h>
#include <util.h>


/*=======================================================================*/
/* Configuration */
#define SYS_TICK_PERIOD_IN_MS 100



/*=======================================================================*/
/* 1000/SYS_TICK_PERIOD_IN_MS = 1 second, zero is ok in both cases */
#define BP_CNT_LONG_PRESS_THRESHOLD (500/SYS_TICK_PERIOD_IN_MS)
#define BP_CNT_DEBOUNCE_DELAY (50/SYS_TICK_PERIOD_IN_MS)


struct _bp_struct
{
  uint8_t pin;
  uint8_t state;
  uint8_t cnt;
  volatile uint8_t event;	/* user must confirm this by writing BP_EVENT_NOTHING into this var */
};
typedef struct _bp_struct bp_t;

#define BP_STATE_WAIT 0
#define BP_STATE_DETECTED 1
#define BP_STATE_LONG_DETECTED 2
#define BP_STATE_OFF_DEBOUNCE 3


#define BP_EVENT_NOTHING 0
#define BP_EVENT_SHORT_PRESS 1
#define BP_EVENT_LONG_PRESS 2

void bp_init(bp_t *bp, uint8_t pin)
{
  volatile uint32_t *iocon = get_iocon_by_port(pin);
  
  GPIOSetDir( PORT0, pin, INPUT);
  *iocon &= IOCON_MODE_MASK;
  *iocon |= MODE_PULLUP;
  
  //get_iocon_by_port(pin) = MODE_PULLUP | 
  bp->pin = pin;
  bp->state = BP_STATE_WAIT;
  bp->cnt = 0;
  bp->event = BP_EVENT_NOTHING;
}

void bp_execute(bp_t *bp)
{
  int value = GPIOGetPinValue( 0, bp->pin ); // 0 or !0
  switch( bp->state )
  {
    case BP_STATE_WAIT:
      if ( bp->event == BP_EVENT_NOTHING )
      {
	if ( value == 0 )
	{
	  bp->cnt = BP_CNT_LONG_PRESS_THRESHOLD;
	  bp->state = BP_STATE_DETECTED;
	}
      }
      break;
    case BP_STATE_DETECTED:
      if ( value != 0 )
      {
	bp->state = BP_STATE_OFF_DEBOUNCE;
	bp->cnt = BP_CNT_DEBOUNCE_DELAY;
	bp->event = BP_EVENT_SHORT_PRESS;
      }
      else if ( bp->cnt == 0 )
      {
	bp->state = BP_STATE_LONG_DETECTED;	
      }
      else
      {
	bp->cnt--;
      }
      break;
    case BP_STATE_LONG_DETECTED:
      if ( value != 0 )
      {
	bp->state = BP_STATE_OFF_DEBOUNCE;
	bp->cnt = BP_CNT_DEBOUNCE_DELAY;
	bp->event = BP_EVENT_LONG_PRESS;
      }
      break;
    case BP_STATE_OFF_DEBOUNCE:
      if ( bp->cnt == 0 )
      {
	bp->state = BP_STATE_WAIT;
      }
      else
      {
	bp->cnt--;
      }
      break;
    default:
      bp->state = BP_STATE_WAIT;
      break;
  }
}

/* return current event and remove the ecent from bp */
int bp_get_event(bp_t *bp)
{
  int e = bp->event;
  bp->event = BP_EVENT_NOTHING;
  return e;
}


/*=======================================================================*/
/* system procedures and sys tick master task */



volatile uint32_t sys_tick_irq_cnt=0;

bp_t p2, p10;


void __attribute__ ((interrupt)) SysTick_Handler(void)
{  
  sys_tick_irq_cnt++;
  GPIOSetBitValue(PORT0, 9, (sys_tick_irq_cnt & 1) == 0?0:1);

  bp_execute(&p2);
  bp_execute(&p10);
}




/*=======================================================================*/
/*
  setup the hardware and start interrupts.
  called by "Reset_Handler"
*/
int __attribute__ ((noinline)) main(void)
{
  uint32_t cnt;

  usart_t usart;
  uint8_t usart_rx_buf[32];
  

  /* call to the lpc lib setup procedure. This will set the IRC as clk src and main clk to 24 MHz */
  SystemInit(); 

  /* if the clock or PLL has been changed, also update the global variable SystemCoreClock */
  SystemCoreClockUpdate();

    /* */
  GPIOInit();
  
  /* enable clock for several subsystems */

  Enable_Periph_Clock(CLK_IOCON);
  Enable_Periph_Clock(CLK_SWM);

  bp_init(&p2, 2);
  bp_init(&p10, 10);


  /* set systick and start systick interrupt */
  SysTick_Config(main_clk/1000UL*(unsigned long)SYS_TICK_PERIOD_IN_MS);
  

  usart0_init(&usart, 5, /* tx */ 4, /* rx */ 0, usart_rx_buf, 32);  

  GPIOSetDir( PORT0, 15, OUTPUT);

  delay_micro_seconds(1000000);
  usart_write_string(&usart, "Button Detect\n");
  for(;;)
  {
    cnt++;
    if ( (cnt&0x3ffff) == 0 )
    {
      /*
      usart_write_string(&usart, "p2state:");
      usart_write_u16(&usart, p2.state);
      usart_write_string(&usart, "\n");
      */
    }
    
    if ( p2.event > 0 )
    {
      usart_write_string(&usart, "p2:");
      usart_write_u16(&usart, bp_get_event(&p2));
      usart_write_string(&usart, "\n");
    }
    if ( p10.event > 0 )
    {
      usart_write_string(&usart, "p10:");
      usart_write_u16(&usart, bp_get_event(&p10));
      usart_write_string(&usart, "\n");
    }
    
    /*
    GPIOSetBitValue(PORT0, 15, 1);
    delay_micro_seconds(1000000);
    GPIOSetBitValue(PORT0, 15, 0);
    delay_micro_seconds(1000000);
    */
  }
}

