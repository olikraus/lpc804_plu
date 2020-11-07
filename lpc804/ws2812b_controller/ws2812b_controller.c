/*

  ws2812b_controller.c

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
#include <spi.h>
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

/* number of LEDs in Ring 0 */
#define LED_R0_CNT 12
/* special LED in Ring 0 */
#define LED_R0_TOP 12

/*=======================================================================*/
/* extern */

extern void plu(void);

/*=======================================================================*/
/* system procedures and sys tick master task */



volatile uint32_t sys_tick_irq_cnt=0;

void __attribute__ ((interrupt)) SysTick_Handler(void)
{  
  sys_tick_irq_cnt++;
  //GPIOSetBitValue(PORT0, 9, (sys_tick_irq_cnt & 1) == 0?0:1);
}

/*=======================================================================*/

volatile uint8_t rot_enc_0_value = 0;
volatile uint8_t rot_enc_1_value = 0;

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
  4 bits of date are converted into two bytes (16 bit) of data
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

    v = ((uint16_t)v*(uint16_t)v/256);		// add a nonlinear curve to v

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

uint8_t led_ring_0[LED_R0_CNT*3];

void led_clear(int slot)
{
  int i;
  for( i = 0; i < LED_R0_CNT*3; i++ )
    led_ring_0[i] = 0;
}

void led_set_rgb(int slot, unsigned pos, uint8_t r, uint8_t g, uint8_t b)
{
  pos %= LED_R0_CNT;
  led_ring_0[pos*3] = r;
  led_ring_0[pos*3+1] = g;
  led_ring_0[pos*3+2] = b;
}

void led_add_rgb(int slot, unsigned pos, uint8_t r, uint8_t g, uint8_t b)
{
  pos %= LED_R0_CNT;
  led_ring_0[pos*3] += r;
  led_ring_0[pos*3+1] += g;
  led_ring_0[pos*3+2] += b;
}

void led_out(int slot)
{
  ws2812_spi_out(15, led_ring_0, LED_R0_CNT*3);
}


/*
  draw h selector into given slot
  pos: wrap around
*/
void led_draw_h_selector(int slot, unsigned pos, unsigned max, uint8_t s, uint8_t v)
{
  uint8_t h = pos * 256 / max;
  uint8_t r, g, b;
  unsigned i, j;
  for( i = 0; i < LED_R0_CNT; i++ )
  {
    if ( i == 0 )
      hsv_to_rgb(h + (i * 256)/LED_R0_CNT, s, v, &r, &g, &b);
    else
      hsv_to_rgb(h + (i * 256)/LED_R0_CNT, s, v/2, &r, &g, &b);
    j = LED_R0_CNT - 1 + LED_R0_TOP - i ;
    j %= LED_R0_CNT;
    led_set_rgb(slot, j, r, g, b);
  }
  
  led_set_rgb(0, LED_R0_TOP-1, 0, 0, 0);
  led_set_rgb(0, LED_R0_TOP+1, 0, 0, 0);
}


void led_draw_v_selector(int slot, unsigned pos, unsigned max, uint8_t h, uint8_t s)
{
  uint8_t v = pos * 256 / max;
  uint8_t r, g, b;
  unsigned i, j;
  for( i = 0; i < LED_R0_CNT; i++ )
  {
    hsv_to_rgb(h, s, v + (i * 256)/LED_R0_CNT, &r, &g, &b);
    j = LED_R0_CNT - 1 + LED_R0_TOP - i ;
    j %= LED_R0_CNT;
    led_set_rgb(slot, j, r, g, b);
  }
  
  led_set_rgb(0, LED_R0_TOP-1, 0, 0, 0);
  led_set_rgb(0, LED_R0_TOP+1, 0, 0, 0);
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
  int dir0, dir1;
  int i;

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
  

  ws2812_spi_init();

  usart0_init(&usart, 5, /* tx */ 4, /* rx */ 0, usart_rx_buf, 32);  


  for(;;)
  {
    //GPIOSetBitValue(PORT0, 15, 1);
    //delay_micro_seconds(1000000);
    //GPIOSetBitValue(PORT0, 15, 0);
    
    for( i = 0; i < 10; i++ )
    {
      led_clear(0);
      led_set_rgb(0, rot_enc_0_value, 40,0,0);
      led_add_rgb(0, rot_enc_1_value, 0,40,0);
      
      rot_enc_1_value &= 0x03f;
      led_draw_h_selector(0, rot_enc_1_value, 0x040, 255, 50);
      led_draw_v_selector(0, rot_enc_1_value, 0x040, 0, 0);
      led_out(0);
      
      
      delay_micro_seconds(1000000/20);
    }
    
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

