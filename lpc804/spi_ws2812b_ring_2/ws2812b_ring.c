/*

  ws2812b_ring.c
  
  SPI controlled WS2812B
  
  SPI clock input: 15MHz
  SPI Clock Div = 6 --> 2.5MHz SPI Clock --> 400ns
  
  Dataoutput is PIO0_15 (=MOSI)
  
  
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

struct _ws2812_spi
{
  volatile uint8_t *spi_data;
  volatile uint32_t cnt;		// remaining bytes
  volatile uint32_t b;		// the current byte
  volatile uint32_t bitcnt;		
};
typedef struct _ws2812_spi ws2812_spi_t;

ws2812_spi_t ws2812_spi;

/* this SPI handler will do an online conversion of the bit values to the WS2812B format */
/* 4 bits of date are converted into two bytes (16 bit) of data */ 
void __attribute__ ((interrupt)) SPI0_Handler(void)
{
  if ( ws2812_spi.cnt > 0 || ws2812_spi.bitcnt > 0 )
  {
    uint32_t d = 0x4444;
    register uint32_t b;
    
    if ( ws2812_spi.bitcnt == 0 )
    {
      ws2812_spi.b = *ws2812_spi.spi_data++;
      ws2812_spi.cnt--;
      ws2812_spi.bitcnt = 2;
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
    ws2812_spi.bitcnt--;
  }
  else
  {
    /* ensure, that the SCK goes to low after the byte transfer: */
    /* Set the EOT flag at the end of the transfer */
   LPC_SPI0->TXCTL |= SPI_CTL_EOT;
    
    /* disable interrupt, needs to be re-enabled whenever new data should be transmitted */
   LPC_SPI0->INTENCLR = SPI_STAT_TXRDY; 
  }  
}

void spi_init(void)
{
  Enable_Periph_Clock(CLK_SPI0);

  
  //map_function_to_port(SPI0_SCK, 9);
  GPIOSetDir( PORT0, 15, OUTPUT);	// output for MOSI
  map_function_to_port(SPI0_MOSI, 15);
  
  LPC_SYSCON->SPI0CLKSEL = FCLKSEL_MAIN_CLK;
  
  LPC_SPI0->DIV = 6;
  LPC_SPI0->CFG = SPI_CFG_ENABLE | SPI_CFG_MASTER;
  
  NVIC_SetPriority(SPI0_IRQn, 2);  /* 0: highes priority, 3: lowest priority */
  NVIC_EnableIRQ(SPI0_IRQn);
  
}

void spi_out(uint8_t *data, int cnt)
{
  
  /* wait until data is transmitted */
  while( ws2812_spi.cnt != 0 )
    ;
  
  LPC_SPI0->TXCTL =  
      SPI_CTL_RXIGNORE | 		/* do not read data from MISO */
      SPI_CTL_LEN(16) | 			/* send 16 bits */
      SPI_TXDATCTL_SSELN(3); 	/* do not use any slave select */
  
  ws2812_spi.spi_data = data;
  ws2812_spi.cnt = cnt;
  ws2812_spi.bitcnt = 0;
  /* load first byte, so that the TXRDY interrupt will be generated */
  /* this is just a zero byte and is ignored by the WS2812B */
  LPC_SPI0->TXDAT = 0;	
  LPC_SPI0->INTENSET |= SPI_STAT_TXRDY;	/* enable TX ready interrupt */

}

/*=======================================================================*/
/*
  convert 0 to 1000 and 1 to 1100
  "out" array must be 4x bigger than "in" array.
*/
void convert_to_ws2812b(int cnt, uint8_t *in, uint8_t *out)
{
  int i, j;
  uint8_t b;
  for ( i = 0; i < cnt; i++ )
  {
    b = in[i];
    j = 4;
    do
    {
      *out = 0x44;
      if ( b & 128 )
	*out |= 0x20;
      b <<= 1;
      if ( b & 128 )
	*out |= 0x02;
      b <<= 1;
      out++;
      j--;
    } while( j > 0 );
  }
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
  static uint8_t aa[LED_CNT*3*4];
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

    
  spi_init();	

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

    convert_to_ws2812b(LED_CNT*3, a, aa);
    //spi_out(aa, LED_CNT*3*4);
    spi_out(a, LED_CNT*3);
    //last_byte = 0;
    //spi_out(&last_byte, 1);
    
    /* LPC804 might have up to two bytes in the queue, wait for their transmission */    
    delay_micro_seconds(24+24);
    /* write the new values into the LEDs */
    delay_micro_seconds(50);
    
    /* end user delay */
    delay_micro_seconds(20000L);
  }
  
}
