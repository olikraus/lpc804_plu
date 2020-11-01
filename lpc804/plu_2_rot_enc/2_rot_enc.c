/*

  rot_enc_usart.c

  Configuration is 115200 8-N-1. Only "\n" is sent. Receiving terminal should add \r
  (e.g. use add CR function in "minicom")
  
  Received chars are echoed back enclosed in "<?>" once per second.
  
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


/*=======================================================================*/
/* Configuration */
#define SYS_TICK_PERIOD_IN_MS 100


#define ROT_ENC_0_DIR_PIN 25
#define ROT_ENC_0_CNT_PIN 26
/* value of the direction bit for clock wise rotation */
#define ROT_ENC_0_CW_DIR 0

#define ROT_ENC_1_DIR_PIN 23
#define ROT_ENC_1_CNT_PIN 24
/* value of the direction bit for clock wise rotation */
#define ROT_ENC_1_CW_DIR 0


/*=======================================================================*/
/* extern */

extern void plu(void);

/*=======================================================================*/
/* system procedures and sys tick master task */



volatile uint32_t sys_tick_irq_cnt=0;
volatile uint8_t rot_enc_0_value = 0;
volatile uint8_t rot_enc_1_value = 0;

void __attribute__ ((interrupt)) SysTick_Handler(void)
{  
  sys_tick_irq_cnt++;
  //GPIOSetBitValue(PORT0, 9, (sys_tick_irq_cnt & 1) == 0?0:1);
}

void __attribute__ ((interrupt)) PININT0_Handler(void)
{  
  LPC_PIN_INT->IST = 1;		// clear interrupt
  if ( LPC_GPIO_PORT->B0[ROT_ENC_0_DIR_PIN] == ROT_ENC_0_CW_DIR )
  {
    if ( rot_enc_0_value < 255 )
    {
      rot_enc_0_value++;
    }
  }
  else
  {
    if ( rot_enc_0_value > 0 )
    {
      rot_enc_0_value--;
    }
  }    
}


void __attribute__ ((interrupt)) PININT1_Handler(void)
{  
  LPC_PIN_INT->IST = 2;		// clear interrupt
  if ( LPC_GPIO_PORT->B0[ROT_ENC_1_DIR_PIN] == ROT_ENC_1_CW_DIR )
  {
    if ( rot_enc_1_value < 255 )
    {
      rot_enc_1_value++;
    }
  }
  else
  {
    if ( rot_enc_1_value > 0 )
    {
      rot_enc_1_value--;
    }
  }    
}



/*=======================================================================*/
static const unsigned char u8toa_tab[3]  = { 100, 10, 1 } ;
const char *u8toap(char * dest, uint8_t v)
{
  uint8_t pos;
  uint8_t d;
  uint8_t c;
  for( pos = 0; pos < 3; pos++ )
  {
      d = '0';
      c = *(u8toa_tab+pos);
      while( v >= c )
      {
	v -= c;
	d++;
      }
      dest[pos] = d;
  }  
  dest[3] = '\0';
  return dest;
}

/* v = value, d = number of digits */
const char *u8toa(uint8_t v, uint8_t d)
{
  static char buf[4];
  d = 3-d;
  return u8toap(buf, v) + d;
}


      


/*=======================================================================*/
/*
  setup the hardware and start interrupts.
  called by "Reset_Handler"
*/
int __attribute__ ((noinline)) main(void)
{
  int dir0, dir1, tick;

  usart_t usart;
  uint8_t usart_rx_buf[32];
  

  /* call to the lpc lib setup procedure. This will set the IRC as clk src and main clk to 24 MHz */
  SystemInit(); 

  /* if the clock or PLL has been changed, also update the global variable SystemCoreClock */
  SystemCoreClockUpdate();

  /* set systick and start systick interrupt */
  SysTick_Config(main_clk/1000UL*(unsigned long)SYS_TICK_PERIOD_IN_MS);
  
  plu();
  
  GPIOInit();  
  Enable_Periph_Clock(CLK_IOCON);
  Enable_Periph_Clock(CLK_SWM);
  Enable_Periph_Clock(CLK_GPIO_INT);
  Do_Periph_Reset(CLK_GPIO_INT);
  
  LPC_SYSCON->PINTSEL0 = ROT_ENC_0_CNT_PIN;
  LPC_SYSCON->PINTSEL1 = ROT_ENC_1_CNT_PIN;
  LPC_PIN_INT->ISEL = 0;		// edge sensitive
  LPC_PIN_INT->SIENR = 3;		// enable rising edge interrupt 
  //LPC_PIN_INT->SIENF = 1;		// enable falling edge interrupt 
  
  NVIC_SetPriority(PININT0_IRQn, (1<<__NVIC_PRIO_BITS) - 1);  /* set Priority */
  NVIC_EnableIRQ(PININT0_IRQn);
  
  NVIC_SetPriority(PININT1_IRQn, (1<<__NVIC_PRIO_BITS) - 1);  /* set Priority */
  NVIC_EnableIRQ(PININT1_IRQn);
  

  usart0_init(&usart, 5, /* tx */ 4, /* rx */ 0, usart_rx_buf, 32);  

  //GPIOSetDir( PORT0, 15, OUTPUT);
  //GPIOSetDir( PORT0, 23, INPUT);

  for(;;)
  {
    //GPIOSetBitValue(PORT0, 15, 1);
    //delay_micro_seconds(1000000);
    //GPIOSetBitValue(PORT0, 15, 0);
    delay_micro_seconds(1000000/2);

    dir0 = LPC_GPIO_PORT->B0[ROT_ENC_0_DIR_PIN];
    dir1 = LPC_GPIO_PORT->B0[ROT_ENC_1_DIR_PIN];
    usart_write_string(&usart, "dir0=");    
    usart_write_byte(&usart, '0'+dir0);
    usart_write_string(&usart, " dir1=");    
    usart_write_byte(&usart, '0'+dir1);
    
    usart_write_string(&usart, " rot_enc_0_value=");    
    usart_write_string(&usart, u8toa(rot_enc_0_value,3));    

    usart_write_string(&usart, " rot_enc_1_value=");    
    usart_write_string(&usart, u8toa(rot_enc_1_value,3));    

    usart_write_string(&usart, " rise=");    
    usart_write_string(&usart, u8toa(LPC_PIN_INT->RISE,3));    
    
    usart_write_string(&usart, " ist=");    
    usart_write_string(&usart, u8toa(LPC_PIN_INT->IST,3));    
    
    usart_write_string(&usart, " ");    
    usart_write_bits(&usart, LPC_PLU0->OUTPUTS, 8);
    
    
    usart_write_string(&usart, "\n"); 
    
    
  }
}

