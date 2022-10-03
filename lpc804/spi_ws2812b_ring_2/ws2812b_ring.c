/*

  ws2812b_ring.c
  
  SPI controlled WS2812B with data conversion inside SPI ISR
  
  SPI clock input: 15MHz
  SPI Clock Div = 6 --> 2.5MHz SPI Clock --> 400ns
  
  Dataoutput is PIO0_15 (=MOSI)

  Log output via USART:  Unix: "tail -f /dev/ttyUSB0" should work.


*/


#include "LPC8xx.h"
#include "iocon.h"
#include "syscon.h"
#include "gpio.h"
#include "swm.h"
#include "spi.h"
#include "delay.h"
#include "util.h"

/*=======================================================================*/
/* Configuration */
#define SYS_TICK_PERIOD_IN_MS 100


/*=======================================================================*/
extern void plu(void);



/*=======================================================================*/
/* system procedures and sys tick master task */



volatile uint32_t sys_tick_irq_cnt=0;


void __attribute__ ((interrupt)) SysTick_Handler(void)
{  
  sys_tick_irq_cnt++;
}

/*=======================================================================*/

usart_t usart;
uint8_t usart_rx_buf[8];

/*=======================================================================*/
/* 
  SPI procedures for WS2812B LEDs 
  1) Call "void ws2812_spi_init(void)" 
  2) Call "ws2812_spi_out(int gpio, uint8_t *data, int cnt)" to output RGB Data
  currently the procedure uses around 50-60 system ticks
*/

struct _ws2812_spi
{
  volatile uint8_t *spi_data;
  volatile uint32_t byte_cnt;		// remaining bytes
  volatile uint32_t b;		// the current byte
  volatile uint32_t bit_cnt;
  volatile uint32_t post_data_wait;
  volatile uint32_t isr_ticks;
  volatile uint32_t max_isr_ticks;
  volatile int in_progress;
};
typedef struct _ws2812_spi ws2812_spi_t;

ws2812_spi_t ws2812_spi;

/*
  This SPI handler will do an online conversion of the bit values to the WS2812B format 
  4 bits of data are converted into two bytes (16 bit) of data
  This procedure is time critical: 6 Clock-Cycle per SPI bit 6*16 = 96 Clock Cycle
  ==> Upper Limit for this procedure are 96 clock cycles
*/
void __attribute__ ((interrupt)) SPI0_Handler(void)
{
  //uint32_t start = SysTick->VAL;
  //uint32_t end;
  if ( ws2812_spi.byte_cnt > 0 || ws2812_spi.bit_cnt > 0 )
  {
    uint32_t d = 0x4444;
    register uint32_t b;
    
    if ( ws2812_spi.bit_cnt == 0 )
    {
      ws2812_spi.b = *ws2812_spi.spi_data++;
      ws2812_spi.byte_cnt--;
      ws2812_spi.bit_cnt = 2;
    }
    
    b = ws2812_spi.b;
    if ( b & 128 )
      d |= 0x2000;
    if ( b & 64 )
      d |= 0x0200;
    if ( b & 32 )
      d |= 0x0020;
    if ( b & 16 )
      d |= 0x0002;
    b <<= 4;
    ws2812_spi.b = b;
    
    LPC_SPI0->TXDAT = d;
    ws2812_spi.bit_cnt--;
  }
  else
  {
    if ( ws2812_spi.post_data_wait > 0 )
    {
      /* wait for 50us, this are 125 SPI clocks (each is 0.4us) --> send 128 bits, 8x 16 Bit words */
      LPC_SPI0->TXDAT = 0;
      ws2812_spi.post_data_wait--;
    }
    else
    {
      
      /* ensure, that the SCK goes to low after the byte transfer: */
      /* Set the EOT flag at the end of the transfer */
      LPC_SPI0->TXCTL |= SPI_CTL_EOT;
      
      /* disable interrupt, needs to be re-enabled whenever new data should be transmitted */
      LPC_SPI0->INTENCLR = SPI_STAT_TXRDY; 
      ws2812_spi.in_progress = 0;
    }
  } 
  /*
  end = SysTick->VAL;
  if ( start < end )
    start += SysTick->LOAD;
  start -= end;			// calculate the duration in ticks
  if ( ws2812_spi.max_isr_ticks < start )
    ws2812_spi.max_isr_ticks = start;	// calculate maximum
  ws2812_spi.isr_ticks = start;
  */
}

void ws2812_spi_init(void)
{
  Enable_Periph_Clock(CLK_SPI0);
  
  
  LPC_SYSCON->SPI0CLKSEL = FCLKSEL_MAIN_CLK;
  
  LPC_SPI0->DIV = 6;
  LPC_SPI0->CFG = SPI_CFG_ENABLE | SPI_CFG_MASTER;
  
  NVIC_SetPriority(SPI0_IRQn, 2);  /* 0: highes priority, 3: lowest priority */
  NVIC_EnableIRQ(SPI0_IRQn);
  
}

void ws2812_spi_out(int gpio, uint8_t *data, int cnt)
{
  
  /* wait until data is transmitted */
  while( ws2812_spi.in_progress != 0 )
    ;

  GPIOSetDir( PORT0, gpio, OUTPUT);	// output for MOSI
  //map_function_to_port(SPI0_SCK, 9);
  map_function_to_port(SPI0_MOSI, gpio);

  
  LPC_SPI0->TXCTL =  
      SPI_CTL_RXIGNORE | 		/* do not read data from MISO */
      SPI_CTL_LEN(16) | 			/* send 16 bits */
      SPI_TXDATCTL_SSELN(3); 	/* do not use any slave select */
  
  ws2812_spi.spi_data = data;
  ws2812_spi.byte_cnt = cnt;
  ws2812_spi.bit_cnt = 0;
  ws2812_spi.post_data_wait = 9;		/* 8 would be sufficient, use 9 to be on the safe side */
  ws2812_spi.in_progress  = 1;
  /* load first byte, so that the TXRDY interrupt will be generated */
  /* this is just a zero byte and is ignored by the WS2812B */
  LPC_SPI0->TXDAT = 0;	
  LPC_SPI0->INTENSET |= SPI_STAT_TXRDY;	/* enable TX ready interrupt */

}

/*=======================================================================*/

/* https://stackoverflow.com/questions/3018313/algorithm-to-convert-rgb-to-hsv-and-hsv-to-rgb-in-range-0-255-for-both */


void hsv_to_rgb(uint8_t h, uint8_t s, uint8_t v, uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint8_t region, remainder, p, q, t;

    if (s == 0)
    {
        *r = v;
        *g = v;
        *b = v;
        return ;
    }

    region = h / 43;
    remainder = (h - (region * 43)) * 6; 

    p = (v * (uint16_t)(255 - s)) >> 8;
    q = (v * (uint16_t)(255 - ((s * remainder) >> 8))) >> 8;
    t = (v * (uint16_t)(255 - ((s * (uint16_t)(255 - remainder)) >> 8))) >> 8;

    switch (region)
    {
        case 0:
            *r = v; *g = t; *b = p;
            break;
        case 1:
            *r = q; *g = v; *b = p;
            break;
        case 2:
            *r = p; *g = v; *b = t;
            break;
        case 3:
            *r = p; *g = q; *b = v;
            break;
        case 4:
            *r = t; *g = p; *b = v;
            break;
        default:
            *r = v; *g = p; *b = q;
            break;
    }
}

/*=======================================================================*/

/* color component slide */
struct _ccs_t
{
  int16_t current;	/* contains the current color component */
  int16_t start;
  int16_t steps;
  
  int16_t dir;		/* 1 if start < end or -1 if start > end */
  int16_t num;
  int16_t quot;
  
  int16_t den;
  int16_t rem;  
  int16_t frac;
};
typedef struct _ccs_t ccs_t;


/*
  Setup change from "start" to "end" with a specified amount of "steps".
  After calling this procedure, ccs->current will contain the "start" value.
*/
void ccs_init(ccs_t *ccs, int16_t start, int16_t end, int16_t steps)
{
  ccs->start = start;
  ccs->num = end-start;
  ccs->den = steps-1;
  ccs->steps = steps;
  ccs->dir = 1;
  
  ccs->quot = ccs->num / ccs->den;
  if ( ccs->num < 0 )
  {
    ccs->num = -ccs->num;
    ccs->dir = -1;
  }
  ccs->rem = ccs->num % ccs->den;
  
  ccs->frac = ccs->den/2;
  ccs->current = start;
}

/*
  Make one step towards the "end" value. 
  ccs->current will contain the updated value.
*/
int16_t ccs_step(ccs_t *ccs)
{
  if ( ccs->steps == 0 ) 
    return 0;
  
  ccs->current += ccs->quot;
  ccs->frac += ccs->rem;
  if ( ccs->frac >= ccs->den )
  {
    ccs->current += ccs->dir;
    ccs->frac -= ccs->den;
  }
  ccs->steps--;
  return 1;
}

/*
  f(x) = (e-s)/(n-1) * x + s
  current = (num / den)  * (pos / den)
  
  Seek to the specified "pos"ition.
  "pos" must be between 0 and "end"-1
*/
void ccs_seek(ccs_t *ccs, int16_t pos)
{
  int16_t p;
  ccs->current = ccs->quot;
  ccs->current *= pos;
  p = ccs->rem * pos  + ccs->den/2;
  if ( ccs->dir >= 0 )
    ccs->current += p / ccs->den;
  else
    ccs->current -= p / ccs->den;
  ccs->frac = p % ccs->den;
  ccs->current += ccs->start;
}


#define LED_CNT 12

/*=======================================================================*/
int __attribute__ ((noinline)) main(void)
{
  static uint8_t a[LED_CNT*3];
  uint16_t min_brightness = 5;
  uint16_t max_brightness = 30;
  int h = 0;
  int i;
  ccs_t ccs_v;
  int is_v_up = 1;
  /* call to the lpc lib setup procedure. This will set the IRC as clk src and main clk to 15 MHz */
  SystemInit(); 

  /* if the clock or PLL has been changed, also update the global variable SystemCoreClock */
  SystemCoreClockUpdate();

  GPIOInit();
  Enable_Periph_Clock(CLK_IOCON);
  Enable_Periph_Clock(CLK_SWM);
  
  
  /* set systick and start systick interrupt */
  SysTick_Config(main_clk/1000UL*(unsigned long)SYS_TICK_PERIOD_IN_MS);

  usart0_init(&usart, 5, /* tx */ 4, /* rx */ 0, usart_rx_buf, 32);  
    
  ws2812_spi_init();	

  delay_micro_seconds(50);
  ccs_init(&ccs_v, min_brightness, max_brightness, 37);
  is_v_up = 1;
  for(;;)
  {
    h+=2;	/* will turn arount at 255 */
    for( i = 0; i < LED_CNT; i++ )
    {
      hsv_to_rgb(h+i*(256/LED_CNT), 255, ccs_v.current, a+i*3+0, a+i*3+1, a+i*3+2);
    }
    //hsv_to_rgb(h, 255, ccs_v.current, a+0, a+1, a+2);
    //hsv_to_rgb(h+20, 255, ccs_v.current, a+3, a+4, a+5);
    if ( ccs_step(&ccs_v) == 0 )
    {
      if ( is_v_up )
      {
	ccs_init(&ccs_v, max_brightness,min_brightness, 37);
	is_v_up = 0;
      }
      else
      {
	ccs_init(&ccs_v, min_brightness, max_brightness, 37);
	is_v_up = 1;
      }
    }

    ws2812_spi_out(15, a, LED_CNT*3);
    
    
    /* end user delay */
    delay_micro_seconds(20000L);
    
    if ( (h & 0x3f) == 0 )
    {
      usart_write_string(&usart, "spi isr ticks=");    
      usart_write_string(&usart, u16toa(ws2812_spi.isr_ticks));    
      usart_write_string(&usart, " spi max isr ticks=");    
      usart_write_string(&usart, u16toa(ws2812_spi.max_isr_ticks));    
      usart_write_string(&usart, "\n");    
    }
  }
  
}
