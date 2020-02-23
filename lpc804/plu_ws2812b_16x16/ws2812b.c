/*

  ws2812b.c

  PLU will be configured as SPI client. 
  SPI clock should be 800kHz: SPI clock div for the main clk can be 18 (833kHz) or 19 (789kHz)
  
  PLU configuration:
    sck: PIO0_22;
    mosi: PIO0_21;
  
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

void spi_init(void)
{
  Enable_Periph_Clock(CLK_SPI0);
  
  map_function_to_port(SPI0_SCK, 9);
  map_function_to_port(SPI0_MOSI, 11);
  
  LPC_SYSCON->SPI0CLKSEL = FCLKSEL_MAIN_CLK;	
  LPC_SPI0->DIV = 14;	/* 13 still seems to work */
  LPC_SPI0->CFG = SPI_CFG_ENABLE | SPI_CFG_MASTER;
}

void spi_out(uint8_t *data, int cnt)
{
  int i;
  LPC_SPI0->TXCTL = 
      SPI_CTL_RXIGNORE | 		/* do not read data from MISO */
      SPI_CTL_LEN(8) | 			/* send 8 bits */
      SPI_TXDATCTL_SSELN(3); 	/* do not use any slave select */
  for( i = 0; i < cnt; i ++ )
  {
    /* wait until the tx register can accept further data */
    while( (LPC_SPI0->STAT & SPI_STAT_TXRDY) == 0 )
      ;

    /* ensure, that the SCK goes to low after the byte transfer: */
    /* Set the EOT flag at the end of the transfer */
    if ( i+1 == cnt )
    {
      LPC_SPI0->TXCTL |= SPI_CTL_EOT;
    }
    
    /* transfer one byte via SPI */
    LPC_SPI0->TXDAT = data[i];
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

/*=======================================================================*/

#define LED_WIDTH 16
#define LED_HEIGHT 16

struct _led_matrix
{
  /* line */
  uint8_t l_x, l_y, l_w;
  uint8_t l_rgba[2][4];
  ccs_t l_ccs[4];
  
  /* box */
  uint8_t b_x,  b_y, b_w, b_h;
  uint8_t b_rgba[4][4];
  ccs_t b_ccs0[4];
  ccs_t b_ccs1[4];
  

  /* pixel */
  uint8_t p_x;
  uint8_t p_y;
  uint8_t p_rgba[4];
  
  uint8_t *p;
};
typedef struct _led_matrix led_matrix_t;


uint8_t led_matrix_buffer[LED_WIDTH*LED_HEIGHT*3];
led_matrix_t lm;


/*
  return values:
    0 == 0 A
    255 = 0.02 A (value for one LED )
    255*256*3 = 195840 == 15.36A

  If a power source provides 3A, then the max current is
    255 * 3A / 0.02 A = 38250
*/
uint32_t calc_current(led_matrix_t *lm)
{
  uint32_t cnt = LED_WIDTH*LED_HEIGHT*3;
  uint32_t current = 0;
  lm->p = led_matrix_buffer;
  while( cnt > 0 )
  {
    current += *lm->p;
    cnt--;
    lm->p++;
  }
  return current;
}

void limit_current(led_matrix_t *lm, uint32_t max)
{
  uint32_t current = calc_current(lm);
  if ( current > max )
  {
    uint32_t cnt = LED_WIDTH*LED_HEIGHT*3;
    lm->p = led_matrix_buffer;
    
    current >>= 8;
    max >>= 8;
    while( cnt > 0 )
    {
      *lm->p = ((uint32_t)*lm->p * max) / current;
      
      cnt--;
      lm->p++;
    }
    
  }
}

/* 
  in: x, y 
  out: p
*/
void calc_p(led_matrix_t *lm) __attribute__((noinline));
void calc_p(led_matrix_t *lm)
{
  uint16_t o;
  uint8_t x;
  
  /* matrix has a snake kind of pattern */
  x = lm->p_x;
  if ( (lm->p_y & 1)==0 )
  {
    x = LED_WIDTH-1-lm->p_x;
  }
  
  o = lm->p_y;
  o *=LED_WIDTH*3;
  o+=x*3;
  lm->p = led_matrix_buffer;
  lm->p += o;
}

/* https://en.wikipedia.org/wiki/Linear_interpolation */
uint8_t lerp(uint8_t v0, uint8_t v1, uint8_t t)
{
  int16_t d;
  d = v1;
  d -= v0;
  d *= t;
  d >>= 8;
  d += v0;
  return (uint8_t)d;
}

/*
  in: p_rgba, p
  p will point to the next pixel
*/
void draw_pixel(led_matrix_t *lm)
{
  uint8_t a = lm->p_rgba[3];
  *lm->p = lerp(*lm->p, lm->p_rgba[0], a);
  lm->p++;
  *lm->p = lerp(*lm->p, lm->p_rgba[1], a);
  lm->p++;
  *lm->p = lerp(*lm->p, lm->p_rgba[2], a);
  lm->p++;
}

/*
  in: p_rgba, p, l_w
*/
void draw_line(led_matrix_t *lm)
{
  uint8_t w = lm->l_w;
  while( w > 0 )
  {
    draw_pixel(lm);
    w--;
  }
}

/*
  in: p_rgba
*/
void draw_all(led_matrix_t *lm)
{
  uint8_t h = LED_HEIGHT;
  lm->p_x = 0;
  lm->p_y = 0;
  lm->l_w = LED_WIDTH;
  calc_p(lm);
  while( h > 0 )
  {
    draw_line(lm);
    h--;
  }
}

/*
  in: l_x, l_y, l_w, lm->l_rgba[0], lm->l_rgba[1]
*/
void draw_color_line(led_matrix_t *lm)
{
  uint8_t i, j;
  for( i = 0; i < 4; i++ )
  {
    ccs_init( lm->l_ccs+i, lm->l_rgba[0][i], lm->l_rgba[1][i], lm->l_w );
  }
  lm->p_x = lm->l_x;
  lm->p_y = lm->l_y;
  for( j = 0; j < lm->l_w; j++ )
  {
    for( i = 0; i < 4; i++ )
    {
      lm->p_rgba[i] = lm->l_ccs[i].current;
      ccs_step(lm->l_ccs+i);      
    }
    calc_p(lm);
    draw_pixel(lm);
    lm->p_x++;
  }
}

void draw_color_box(led_matrix_t *lm)
{
  uint8_t i, j;
  for( i = 0; i < 4; i++ )
  {
    ccs_init( lm->b_ccs0+i, lm->b_rgba[0][i], lm->b_rgba[2][i], lm->b_h );
    ccs_init( lm->b_ccs1+i, lm->b_rgba[1][i], lm->b_rgba[3][i], lm->b_h );
  }
  lm->l_y = lm->b_y;
  lm->l_x = lm->b_x;
  lm->l_w = lm->b_w;
  for( j = 0; j < lm->b_h; j++ )
  {
    for( i = 0; i < 4; i++ )
    {
      lm->l_rgba[0][i] = lm->b_ccs0[i].current;
      lm->l_rgba[1][i] = lm->b_ccs1[i].current;
      ccs_step(lm->b_ccs0+i);
      ccs_step(lm->b_ccs1+i);
    }    
    draw_color_line(lm);
    lm->l_y += 1;
    
  }
}


/*=======================================================================*/
int __attribute__ ((noinline)) main(void)
{
  
  int h = 0;
  ccs_t ccs_v;
  /* call to the lpc lib setup procedure. This will set the IRC as clk src and main clk to 24 MHz */
  SystemInit(); 

  /* if the clock or PLL has been changed, also update the global variable SystemCoreClock */
  SystemCoreClockUpdate();

  /* set systick and start systick interrupt */
  SysTick_Config(main_clk/1000UL*(unsigned long)SYS_TICK_PERIOD_IN_MS);


    
  plu();		/* plu() will enable GPIO0 & SWM clock */
  spi_init();	

  delay_micro_seconds(50);
  ccs_init(&ccs_v, 40, 100, 27);
  for(;;)
  {
    h+=2;	
    /*
    hsv_to_rgb(h, 255, 128, a+0, a+1, a+2);
    hsv_to_rgb(h+20, 255, ccs_v.current, a+3, a+4, a+5);
    if ( ccs_step(&ccs_v) == 0 )
    {
      if ( is_v_up )
      {
	ccs_init(&ccs_v, 100,40, 17);
	is_v_up = 0;
      }
      else
      {
	ccs_init(&ccs_v, 40, 100, 17);
	is_v_up = 1;
      }
    }
    */
    hsv_to_rgb(h, 255, 32, lm.p_rgba+0, lm.p_rgba+1, lm.p_rgba+2);
    lm.p_rgba[3] = 255;

    /*
    lm.p_x = 0;
    lm.p_y = 0;    
    calc_p(&lm); draw_pixel(&lm); 
  
    lm.p_x = 1;
    lm.p_y = 1;    
    calc_p(&lm); draw_pixel(&lm);

    lm.p_x = 2;
    lm.p_y = 2;    
    calc_p(&lm); draw_pixel(&lm);

    lm.p_x = 3;
    lm.p_y = 3;    
    calc_p(&lm); draw_pixel(&lm);

    lm.l_x = 0;
    lm.l_y = 5;
    lm.l_w = 16;

    hsv_to_rgb(h, 255, 32, &(lm.l_rgba[0][0]), &(lm.l_rgba[0][1]), &(lm.l_rgba[0][2]));
    lm.l_rgba[0][3] = 255;
    hsv_to_rgb(h+64, 255, 32, &(lm.l_rgba[1][0]), &(lm.l_rgba[1][1]), &(lm.l_rgba[1][2]));
    lm.l_rgba[1][3] = 255;
    draw_color_line(&lm);
    */


    lm.b_x = 0;
    lm.b_y = 0;
    lm.b_w = 16;
    lm.b_h = 16;

    hsv_to_rgb(h, 255, 32, &(lm.b_rgba[0][0]), &(lm.b_rgba[0][1]), &(lm.b_rgba[0][2]));
    hsv_to_rgb(h+64, 255, 32, &(lm.b_rgba[1][0]), &(lm.b_rgba[1][1]), &(lm.b_rgba[1][2]));
    hsv_to_rgb(255-h, 255, 32, &(lm.b_rgba[2][0]), &(lm.b_rgba[2][1]), &(lm.b_rgba[2][2]));
    hsv_to_rgb(255-h+64, 255, 32, &(lm.b_rgba[3][0]), &(lm.b_rgba[3][1]), &(lm.b_rgba[3][2]));
    lm.b_rgba[0][3] = 255;
    lm.b_rgba[1][3] = 255;
    lm.b_rgba[2][3] = 255;
    lm.b_rgba[3][3] = 255;
    draw_color_box(&lm);

    //draw_all(&lm);
    //limit_current(lm, 38250);
    limit_current(&lm, 30000);
    spi_out(led_matrix_buffer, 3*LED_WIDTH*LED_HEIGHT);
    
    /* LPC804 has might have up to two bytes in the queue, wait for their transmission */    
    delay_micro_seconds(24+24);
    /* write the new values into the LEDs */
    delay_micro_seconds(50);
    
    /* end user delay */
    delay_micro_seconds(20000L);
  }
}
