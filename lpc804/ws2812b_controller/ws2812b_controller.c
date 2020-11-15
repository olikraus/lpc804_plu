/*

  ws2812b_controller.c

  Configuration is 115200 8-N-1. Only "\n" is sent. Receiving terminal should add \r
  (e.g. use add CR function in "minicom")
  
  Received chars are echoed back enclosed in "<?>" once per second.
  
  Unix: "tail -f /dev/ttyUSB0" should also work.

  PIO0_..
  10			data out led ring for rotaray encoder 0
  15			data out led ring for rotaray encoder 1

*/

#include <stddef.h>
#include <LPC8xx.h>
#include <core_cmInstr.h>
#include <syscon.h>
#include <iocon.h>
#include <gpio.h>
#include <swm.h>
#include <uart.h>
#include <spi.h>
#include <delay.h>
#include <util.h>


/*=======================================================================*/
/* Configuration */
#define SYS_TICK_PERIOD_IN_MS 20

#define LED_CNT 60
#define LED_LIGHT_WS2812B_OUT_PIN 14


#define ROT_ENC_0_DIR_PIN 25
#define ROT_ENC_0_CNT_PIN 26
/* value of the direction bit for clock wise rotation */
#define ROT_ENC_0_CW_DIR 0
#define ROT_ENC_0_BUTTON 11
#define ROT_ENC_0_WS2812B_OUT_PIN 10

#define ROT_ENC_1_DIR_PIN 23
#define ROT_ENC_1_CNT_PIN 24
/* value of the direction bit for clock wise rotation */
#define ROT_ENC_1_CW_DIR 0
#define ROT_ENC_1_BUTTON 1
#define ROT_ENC_1_WS2812B_OUT_PIN 15

#define MODE_BUTTON_PIN 19




/* number of LEDs in Ring 0 */
#define LED_R0_CNT 12
/* special LED in Ring 0 */
#define LED_R0_TOP 1

/* 1000/SYS_TICK_PERIOD_IN_MS = 1 second, zero is ok in both cases */
/* delay until which a long press is detected 500/SYS_TICK_PERIOD_IN_MS = half second */
#define BP_CNT_LONG_PRESS_THRESHOLD (500/SYS_TICK_PERIOD_IN_MS)
/* debounce delay after button release, can be zero (which are actually SYS_TICK_PERIOD_IN_MS milliseconds */
#define BP_CNT_DEBOUNCE_DELAY (50/SYS_TICK_PERIOD_IN_MS)

/* Nmber of lights sources */
#define LIGHT_SOURCES_CNT 3

/*=======================================================================*/
/* definitions */
struct _bp_struct
{
  uint8_t pin;
  uint8_t state;
  uint8_t cnt;
  volatile uint8_t event;	/* user must confirm this by writing BP_EVENT_NOTHING into this var */
};
typedef struct _bp_struct bp_t;

/*=======================================================================*/
/* extern */

extern void plu(void);

/*=======================================================================*/
/* global variables */
bp_t bp_middle_button;
bp_t bp_rot_enc[2];


/*=======================================================================*/

struct rot_enc_struct
{
    volatile uint8_t value;
    uint8_t min;
    uint8_t max;
    uint8_t is_wrap_around;
};
typedef struct rot_enc_struct rot_enc_t;

rot_enc_t rotary_encoder[2];


void rot_enc_inc(rot_enc_t *re)
{
  if ( re->value < re->max )
  {
    re->value++;
  }
  else
  {
    if ( re->is_wrap_around )
    {
      re->value = re->min;
    }
  }
}

void rot_enc_dec(rot_enc_t *re)
{
  if ( re->value > re->min )
  {
    re->value--;
  }
  else
  {
    if ( re->is_wrap_around )
    {
      re->value = re->max;
    }
  }
}

void rot_enc_setup(rot_enc_t *re, uint8_t value, uint8_t min, uint8_t max, uint8_t is_wrap_around)
{
  if ( value < min )
    value = min;
  if ( value > max )
    value = max;
  re->value = value;
  re->min = min;
  re->max = max;
  re->is_wrap_around = is_wrap_around;
}

void __attribute__ ((interrupt)) PININT0_Handler(void)
{  
  LPC_PIN_INT->IST = 1;		// clear interrupt
  if ( LPC_GPIO_PORT->B0[ROT_ENC_0_DIR_PIN] == ROT_ENC_0_CW_DIR )
  {
    rot_enc_inc(rotary_encoder+0);
  }
  else
  {
    rot_enc_dec(rotary_encoder+0);
  }    
}


void __attribute__ ((interrupt)) PININT1_Handler(void)
{  
  LPC_PIN_INT->IST = 2;		// clear interrupt
  if ( LPC_GPIO_PORT->B0[ROT_ENC_1_DIR_PIN] == ROT_ENC_1_CW_DIR )
  {
    rot_enc_inc(rotary_encoder+1);
  }
  else
  {
    rot_enc_dec(rotary_encoder+1);
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

struct kelvin_rgb_struct
{
  /*uint16_t kelvin;*/
  uint8_t r, g, b;
};
typedef struct kelvin_rgb_struct kelvin_rgb_t;

/* table taken from https://andi-siess.de/rgb-to-color-temperature/ */
/* see also http://www.vendian.org/mncharity/dir3/blackbody/ */
static const kelvin_rgb_t kelvin_to_rgb_map[] = 
{
    {/*1000,*/ 255, 56, 0},
    {/*1100,*/ 255, 71, 0},
    {/*1200,*/ 255, 83, 0},
    {/*1300,*/ 255, 93, 0},
    {/*1400,*/ 255, 101, 0},
    {/*1500,*/ 255, 109, 0},
    {/*1600,*/ 255, 115, 0},
    {/*1700,*/ 255, 121, 0},
    {/*1800,*/ 255, 126, 0},
    {/*1900,*/ 255, 131, 0},
    {/*2000,*/ 255, 138, 18},
    {/*2100,*/ 255, 142, 33},
    {/*2200,*/ 255, 147, 44},
    {/*2300,*/ 255, 152, 54},
    {/*2400,*/ 255, 157, 63},
    {/*2500,*/ 255, 161, 72},
    {/*2600,*/ 255, 165, 79},
    {/*2700,*/ 255, 169, 87},
    {/*2800,*/ 255, 173, 94},
    {/*2900,*/ 255, 177, 101},
    {/*3000,*/ 255, 180, 107},
    {/*3100,*/ 255, 184, 114},
    {/*3200,*/ 255, 187, 120},
    {/*3300,*/ 255, 190, 126},
    {/*3400,*/ 255, 193, 132},
    {/*3500,*/ 255, 196, 137},
    {/*3600,*/ 255, 199, 143},
    {/*3700,*/ 255, 201, 148},
    {/*3800,*/ 255, 204, 153},
    {/*3900,*/ 255, 206, 159},
    {/*4000,*/ 255, 209, 163},
    {/*4100,*/ 255, 211, 168},
    {/*4200,*/ 255, 213, 173},
    {/*4300,*/ 255, 215, 177},
    {/*4400,*/ 255, 217, 182},
    {/*4500,*/ 255, 219, 186},
    {/*4600,*/ 255, 221, 190},
    {/*4700,*/ 255, 223, 194},
    {/*4800,*/ 255, 225, 198},
    {/*4900,*/ 255, 227, 202},
    {/*5000,*/ 255, 228, 206},
    {/*5100,*/ 255, 230, 210},
    {/*5200,*/ 255, 232, 213},
    {/*5300,*/ 255, 233, 217},
    {/*5400,*/ 255, 235, 220},
    {/*5500,*/ 255, 236, 224},
    {/*5600,*/ 255, 238, 227},
    {/*5700,*/ 255, 239, 230},
    {/*5800,*/ 255, 240, 233},
    {/*5900,*/ 255, 242, 236},
    {/*6000,*/ 255, 243, 239},
    {/*6100,*/ 255, 244, 242},
    {/*6200,*/ 255, 245, 245},
    {/*6300,*/ 255, 246, 247},
    {/*6400,*/ 255, 248, 251},
    {/*6500,*/ 255, 249, 253},
    {/*6600,*/ 254, 249, 255},
    {/*6700,*/ 252, 247, 255},
    {/*6800,*/ 249, 246, 255},
    {/*6900,*/ 247, 245, 255},
    {/*7000,*/ 245, 243, 255},
#ifdef OBSOLETE
    {/*7100,*/ 243, 242, 255},
    {/*7200,*/ 240, 241, 255},
    {/*7300,*/ 239, 240, 255},
    {/*7400,*/ 237, 239, 255},
    {/*7500,*/ 235, 238, 255},
    {/*7600,*/ 233, 237, 255},
    {/*7700,*/ 231, 236, 255},
    {/*7800,*/ 230, 235, 255},
    {/*7900,*/ 228, 234, 255},
    {/*8000,*/ 227, 233, 255},
    {/*8100,*/ 225, 232, 255},
    {/*8200,*/ 224, 231, 255},
    {/*8300,*/ 222, 230, 255},
    {/*8400,*/ 221, 230, 255},
    {/*8500,*/ 220, 229, 255},
    {/*8600,*/ 218, 229, 255},
    {/*8700,*/ 217, 227, 255},
    {/*8800,*/ 216, 227, 255},
    {/*8900,*/ 215, 226, 255},
    {/*9000,*/ 214, 225, 255},
    {/*9100,*/ 212, 225, 255},
    {/*9200,*/ 211, 224, 255},
    {/*9300,*/ 210, 223, 255},
    {/*9400,*/ 209, 223, 255},
    {/*9500,*/ 208, 222, 255},
    {/*9600,*/ 207, 221, 255},
    {/*9700,*/ 207, 221, 255},
    {/*9800,*/ 206, 220, 255},
    {/*9900,*/ 205, 220, 255},
    {/*10000,*/ 207, 218, 255},
    {/*10100,*/ 207, 218, 255},
    {/*10200,*/ 206, 217, 255},
    {/*10300,*/ 205, 217, 255},
    {/*10400,*/ 204, 216, 255},
    {/*10500,*/ 204, 216, 255},
    {/*10600,*/ 203, 215, 255},
    {/*10700,*/ 202, 215, 255},
    {/*10800,*/ 202, 214, 255},
    {/*10900,*/ 201, 214, 255},
    {/*11000,*/ 200, 213, 255},
    {/*11100,*/ 200, 213, 255},
    {/*11200,*/ 199, 212, 255},
    {/*11300,*/ 198, 212, 255},
    {/*11400,*/ 198, 212, 255},
    {/*11500,*/ 197, 211, 255},
    {/*11600,*/ 197, 211, 255},
    {/*11700,*/ 197, 210, 255},
    {/*11800,*/ 196, 210, 255},
    {/*11900,*/ 195, 210, 255},
    {/*12000,*/ 195, 209, 255}  
#endif
};

void kelvin_to_rgb(uint8_t kelvin, uint8_t v, uint8_t *r, uint8_t *g, uint8_t *b)
{
  unsigned cnt = sizeof(kelvin_to_rgb_map)/sizeof(kelvin_rgb_t);
  unsigned pos = ((unsigned)kelvin * (unsigned)cnt);
  unsigned frac = pos;
  int t1, t2;
  pos >>=8;
  frac &= 255;
  
  /* check for more than 12000 Kelvin */
  
  if ( pos >= cnt-1 )
  {
    *r = ((unsigned)kelvin_to_rgb_map[cnt-1].r*(unsigned)v)>>8;
    *g = ((unsigned)kelvin_to_rgb_map[cnt-1].g*(unsigned)v)>>8;
    *b = ((unsigned)kelvin_to_rgb_map[cnt-1].b*(unsigned)v)>>8;
    return;
  }
  
  /* interpolate between two entries of the Kelvin table */
  
  t1 = kelvin_to_rgb_map[pos].r;
  t2 = kelvin_to_rgb_map[pos+1].r;
  t1 = t1 + (((t2-t1)*(int)frac)>>8);
  *r = (t1*v)>>8;
  
  t1 = kelvin_to_rgb_map[pos].g;
  t2 = kelvin_to_rgb_map[pos+1].g;
  t1 = t1 + (((t2-t1)*(int)frac)>>8);
  *g = (t1*v)>>8;

  t1 = kelvin_to_rgb_map[pos].b;
  t2 = kelvin_to_rgb_map[pos+1].b;
  t1 = t1 + (((t2-t1)*(int)frac)>>8);
  *b = (t1*v)>>8;
  
}

/*=======================================================================*/

uint8_t led_rot_enc_ring[2][LED_R0_CNT*3];
uint8_t led_light_ring[LED_CNT*3];

void led_clear(int slot)
{
  int i;
  for( i = 0; i < LED_R0_CNT*3; i++ )
    led_rot_enc_ring[slot][i] = 0;
}

/* WS2812B has GRB sequence instead of RGB */

void led_set_rgb(int slot, unsigned pos, uint8_t r, uint8_t g, uint8_t b)
{
  pos %= LED_R0_CNT;
  led_rot_enc_ring[slot][pos*3] = g;
  led_rot_enc_ring[slot][pos*3+1] = r;
  led_rot_enc_ring[slot][pos*3+2] = b;
}


void led_out(int slot)
{
  if ( slot == 0 )
  {
    ws2812_spi_out(ROT_ENC_0_WS2812B_OUT_PIN, led_rot_enc_ring[0], LED_R0_CNT*3);	// Output via PIO0_10
  }
  else
  {
    ws2812_spi_out(ROT_ENC_1_WS2812B_OUT_PIN, led_rot_enc_ring[1], LED_R0_CNT*3);	// Output via PIO0_15
  }
}




void led_draw_all(int slot, uint8_t r, uint8_t g, uint8_t b)
{
  int pos;
  for( pos = 0; pos < LED_R0_CNT; pos++ )
  {
    led_rot_enc_ring[slot][pos*3] = g;
    led_rot_enc_ring[slot][pos*3+1] = r;
    led_rot_enc_ring[slot][pos*3+2] = b;
  }
}


/*
  draw h selector into given slot
  pos: wrap around
  max: the highest value of pos
*/
void led_draw_h_selector(int slot, unsigned pos, unsigned max, uint8_t s, uint8_t v)
{
  uint8_t h = (unsigned)pos * (unsigned)256 / ((unsigned)max+(unsigned)1);
  uint8_t r, g, b;
  unsigned i, j;
  for( i = 0; i < LED_R0_CNT; i++ )
  {
    if ( i == 0 )
      hsv_to_rgb(h + (i * 256)/LED_R0_CNT, s, v, &r, &g, &b);
    else
      hsv_to_rgb(h + (i * 256)/LED_R0_CNT, s, v/2, &r, &g, &b);
    j = LED_R0_CNT - i - 1;
    j += LED_R0_TOP;
    j %= LED_R0_CNT;
    led_set_rgb(slot, j, r, g, b);
  }
  
  led_set_rgb(slot, (LED_R0_TOP)%LED_R0_CNT, 0, 0, 0);
  led_set_rgb(slot, (LED_R0_TOP+LED_R0_CNT-2)%LED_R0_CNT, 0, 0, 0);
}


/*
  draw h selector into given slot
  pos: no wrap around, values for pos are 0..max
  max: the highest value of pos
*/
void led_draw_v_selector(int slot, unsigned pos, unsigned max, uint8_t h, uint8_t s)
{
  uint8_t v = (unsigned)pos * (unsigned)255 / ((unsigned)max);
  uint8_t vv;
  uint8_t r, g, b;
  unsigned i, j;
  for( i = 0; i < LED_R0_CNT; i++ )
  {
    vv = (unsigned)v + (i * (unsigned)255)/(unsigned)LED_R0_CNT;
    vv = ((uint16_t)vv*(uint16_t)vv)/256;		// add a nonlinear curve to v

    hsv_to_rgb(h, s, vv, &r, &g, &b);
    
    j = LED_R0_CNT - i - 1;
    j += LED_R0_TOP;
    j %= LED_R0_CNT;
    led_set_rgb(slot, j, r, g, b);
  }
  
  //led_set_rgb(slot, (LED_R0_TOP)%LED_R0_CNT, 0, 0, 0);
  //led_set_rgb(slot, (LED_R0_TOP+LED_R0_CNT-2)%LED_R0_CNT, 0, 0, 0);
}



/*
  draw s selector into given slot
  pos: no wrap around, values for pos are 0..max
  max: the highest value of pos
*/
void led_draw_s_selector(int slot, unsigned pos, unsigned max, uint8_t h, uint8_t v)
{
  uint8_t s = (unsigned)pos * (unsigned)255 / ((unsigned)max);
  uint8_t r, g, b;
  unsigned i, j;
  for( i = 0; i < LED_R0_CNT; i++ )
  {
    hsv_to_rgb(h, (unsigned)s + (i * (unsigned)255)/(unsigned)LED_R0_CNT, v, &r, &g, &b);
    
    j = LED_R0_CNT - i - 1;
    //j = i;
    j += LED_R0_TOP;
    j %= LED_R0_CNT;
    led_set_rgb(slot, j, r, g, b);
  }
  
  //led_set_rgb(slot, (LED_R0_TOP)%LED_R0_CNT, 0, 0, 0);
  //led_set_rgb(slot, (LED_R0_TOP+LED_R0_CNT-2)%LED_R0_CNT, 0, 0, 0);
}


/*
  draw kelvin selector into given slot
  pos: no wrap around, values for pos are 0..max
  max: the highest value of pos
*/
void led_draw_kelvin_selector(int slot, unsigned pos, unsigned max, uint8_t v)
{
  uint8_t kelvin = (unsigned)pos * (unsigned)255 / ((unsigned)max);
  uint8_t r, g, b;
  unsigned i, j;
  for( i = 0; i < LED_R0_CNT; i++ )
  {
    kelvin_to_rgb((unsigned)kelvin + (i * (unsigned)255)/(unsigned)LED_R0_CNT, v, &r, &g, &b);
    
    //hsv_to_rgb(h, (unsigned)s + (i * (unsigned)255)/(unsigned)LED_R0_CNT, v, &r, &g, &b);
    
    j = LED_R0_CNT - i - 1;
    //j = i;
    j += LED_R0_TOP;
    j %= LED_R0_CNT;
    led_set_rgb(slot, j, r, g, b);
  }
  
  //led_set_rgb(slot, (LED_R0_TOP)%LED_R0_CNT, 0, 0, 0);
  //led_set_rgb(slot, (LED_R0_TOP+LED_R0_CNT-2)%LED_R0_CNT, 0, 0, 0);
}

/*
  x: position on the ring: 0..255
  w: width of the trapezoid, 0..255
  r: length of left and right ramp, 0..255 ( but usually <30 )
*/
uint8_t trapezoid_fn(uint8_t x, uint8_t w, uint8_t r)
{
  uint8_t tu;
  uint8_t tl = 0;
  uint8_t yu;
  uint8_t yl;
  if ( w == 0 )
    return 0;
  if ( w == 255 )
    return 255;
  tu = w+1;
  
  tu/=2;
  if ( x <= tu )
    return 255;
  tl -= tu;
  if ( x >= tl )
    return 255;

  if ( x > tu+r )
    yu = 0;
  else
    yu = (((unsigned)tu+r-x)*(unsigned)256)/(unsigned)r;
  
  if ( x < tl-r )
    yl = 0;
  else
    yl = (((unsigned)x-tl+r)*(unsigned)256)/(unsigned)r;
  
  if ( yu > yl )
    return yu;
  return yl;
}

void led_draw_position_selector(int slot, unsigned pos, unsigned max, uint8_t w, uint8_t v)
{
  uint8_t p = (unsigned)pos * (unsigned)256 / ((unsigned)(max+1));
  uint8_t t, t2;
  //uint8_t r, g, b;
  unsigned i, j;
  for( i = 0; i < LED_R0_CNT; i++ )
  {
    //hsv_to_rgb(h, (unsigned)s + (i * (unsigned)255)/(unsigned)LED_R0_CNT, v, &r, &g, &b);
    
    t = trapezoid_fn( p + (i * (unsigned)256)/(unsigned)LED_R0_CNT, w==0?1:w, 255/LED_R0_CNT);
    
    t = ((unsigned)t*(unsigned)v)/(unsigned)256;

    t2 = trapezoid_fn( p + (i * (unsigned)256)/(unsigned)LED_R0_CNT, 3, 255/LED_R0_CNT);

    t2 = ((unsigned)t2*(unsigned)v)/(unsigned)256;
    
    j = LED_R0_CNT - i - 1;
    j += LED_R0_TOP;
    j %= LED_R0_CNT;
    if ( w == 0 )
      led_set_rgb(slot, j, 0, 0, t2);
    else
      led_set_rgb(slot, j, t-t2, t-t2, t);
  }
  
  //led_set_rgb(slot, 0, 40, 0, 0);
  //led_set_rgb(slot, 1, 0, 40, 0);
  //led_set_rgb(slot, 2, 0, 0, 40);
  //led_set_rgb(slot, 3, 0, 0, 40);
  //led_set_rgb(slot, (LED_R0_TOP)%LED_R0_CNT0, 40, 0, 0);
  //led_set_rgb(slot, (LED_R0_TOP+LED_R0_CNT-2)%LED_R0_CNT, 0, 0, 0);
}


void led_draw_width_selector(int slot, unsigned pos, unsigned max, uint8_t p, uint8_t v)
{
  uint8_t w = (unsigned)pos * (unsigned)255 / ((unsigned)(max));
  uint8_t t, t2;
  //uint8_t r, g, b;
  unsigned i, j;
  
  if ( w == 0 )
    v /= 2;
  for( i = 0; i < LED_R0_CNT; i++ )
  {
    //hsv_to_rgb(h, (unsigned)s + (i * (unsigned)255)/(unsigned)LED_R0_CNT, v, &r, &g, &b);
    
    t = trapezoid_fn( p + (i * (unsigned)256)/(unsigned)LED_R0_CNT, w==0?1:w, 255/LED_R0_CNT);
    
    t = ((unsigned)t*(unsigned)v)/(unsigned)256;

    t2 = trapezoid_fn( p + (i * (unsigned)256)/(unsigned)LED_R0_CNT, 1, 255/LED_R0_CNT);

    t2 = ((unsigned)t2*(unsigned)v)/(unsigned)256;
    
    j = LED_R0_CNT - i - 1;
    j += LED_R0_TOP;
    j %= LED_R0_CNT;
    led_set_rgb(slot, j, t2, t2, t);
  }
  
  //led_set_rgb(0, 0, 40, 0, 0);
  //led_set_rgb(0, 1, 0, 40, 0);
  //led_set_rgb(0, 2, 0, 0, 40);
  //led_set_rgb(0, 3, 0, 0, 40);
  //led_set_rgb(0, (LED_R0_TOP)%LED_R0_CNT0, 40, 0, 0);
  //led_set_rgb(0, (LED_R0_TOP+LED_R0_CNT-2)%LED_R0_CNT, 0, 0, 0);
}


void led_clear_light(void)
{
  int i;
  for( i = 0; i < LED_CNT*3; i++ )
    led_light_ring[i] = 0;
}

void led_add_light_rgb(unsigned pos, uint8_t r, uint8_t g, uint8_t b) 
{
  unsigned tmp;
  
  pos %= LED_CNT;
  
  tmp = (unsigned)led_light_ring[pos*3] + (unsigned)g;
  led_light_ring[pos*3] = tmp > 255 ? 255 : tmp;

  tmp = (unsigned)led_light_ring[pos*3+1] + (unsigned)r;
  led_light_ring[pos*3+1] = tmp > 255 ? 255 : tmp;

  tmp = (unsigned)led_light_ring[pos*3+2] + (unsigned)b;
  led_light_ring[pos*3+2] = tmp > 255 ? 255 : tmp;
  
}


void led_add_light_source(uint8_t w, uint8_t p, uint8_t r, uint8_t g, uint8_t b)
{
  unsigned i, j;
  uint8_t t;
  for( i = 0; i < LED_CNT; i++ )
  {
    t = trapezoid_fn( p + (i * (unsigned)256)/(unsigned)LED_CNT, w, 255/LED_CNT);    
    j = LED_CNT - i - 1;
    //j += 0;
    j %= LED_CNT;
    led_add_light_rgb(j, (r*(unsigned)t)>>8, (g*(unsigned)t)>>8, (b*(unsigned)t)>>8);  
  }
}



/*=======================================================================*/
/* button procedures */

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
/* rotary encoder / led state machine */

/* light sources specification */
#define ARGS_PER_LIGHT 8
/*
  Args 0..5 are also stored in the current editor (rot_enc_user_interface)
  Arg 0: Hue
  Arg 1: Saturation
  Arg 2: Value (Brightness)
  Arg 3: Position
  Arg 4: Width
  Arg 5: Color Temperature (0=1000 Kelvin, max=12000 Kelvin)
  Starting from here, args are not in current editor (rot_enc_user_interface)
  Arg 6: Mode (0:HSV or 1:Kelvin + V)
  Arg 7: Not required, but used as check-sum during flash storage
*/

uint8_t light_sources[LIGHT_SOURCES_CNT][ARGS_PER_LIGHT];

/* global variable, which defines the light source, which is currently under user control */
/* values are 0..LIGHT_SOURCES_CNT-1 */
uint8_t current_light_editor = 0;

typedef struct reui_struct reui_t;
typedef struct rel_struct rel_t;

void rel_draw_h_selector(rel_t *rel);
void rel_draw_s_selector(rel_t *rel);
void rel_draw_v_selector(rel_t *rel);
void rel_draw_position_selector(rel_t *rel);
void rel_draw_width_selector(rel_t *rel);
void rel_draw_kelvin_selector(rel_t *rel);

/* rotary encoder user interface definitions */
struct reui_struct
{
  void (*draw_ui)(rel_t *);
  uint8_t is_wrap_around;
  uint8_t max;
  
  uint8_t value;
 };


/* the number of different values, which can be changed by the rotary encoder */
#define REL_STATES 5

struct rel_struct
{
  int slot;		// led slot for this rotary encoder 0 or 1
  int state;
  
  uint8_t *ui_list;
  
  //uint8_t value[REL_STATES];		// the current value of the rotary encoder for this state
  //uint8_t max[REL_STATES];		// a constant, but kept for easier calculation
};

reui_t rot_enc_user_interface[ARGS_PER_LIGHT] = 
{
  { rel_draw_h_selector, /* wrap=*/ 1,  /*max=*/ LED_R0_CNT*7, /* value=*/ 0},
  { rel_draw_s_selector, /* wrap=*/ 0,  /*max=*/ LED_R0_CNT*2, /* value=*/ 0},
  { rel_draw_v_selector, /* wrap=*/ 0,  /*max=*/ LED_R0_CNT*2, /* value=*/ 0},
  { rel_draw_position_selector, /* wrap=*/ 1,  /*max=*/ LED_R0_CNT*5, /* value=*/ 0},
  { rel_draw_width_selector, /* wrap=*/ 0,  /*max=*/ LED_R0_CNT*5, /* value=*/ 0},
  { rel_draw_kelvin_selector, /* wrap=*/ 0,  /*max=*/ LED_R0_CNT*7, /* value=*/ 0}
};

void init_light_sources(void)
{
  int i;
  for( i = 0; i < LIGHT_SOURCES_CNT; i++ )
  {
    light_sources[i][0] = 0;
    light_sources[i][1] = rot_enc_user_interface[1].max/2;	/* saturation */
    light_sources[i][2] = 0;
    light_sources[i][3] = 0;	/* Position */
    light_sources[i][4] = rot_enc_user_interface[4].max/8;	/* initial width */
    light_sources[i][5] = 0;	/* Kelvin */
    light_sources[i][6] = 0;	/* Mode */
    light_sources[i][7] = 0;	/* Not used */
  }
}


#define UI_UNIFIED_VALUE(x) \
  ((uint8_t)(((unsigned)rot_enc_user_interface[x].value*(unsigned)255)/(unsigned)rot_enc_user_interface[x].max))

#define UI_H UI_UNIFIED_VALUE(0)
#define UI_S UI_UNIFIED_VALUE(1)
#define UI_V UI_UNIFIED_VALUE(2)
#define UI_P UI_UNIFIED_VALUE(3)
#define UI_W UI_UNIFIED_VALUE(4)

void rel_draw_h_selector(rel_t *rel)
{
  reui_t *ui = rot_enc_user_interface + rel->ui_list[rel->state];
  led_draw_h_selector(rel->slot, ui->value, ui->max, 255, 40);
  led_out(rel->slot);
}

void rel_draw_s_selector(rel_t *rel)
{
  reui_t *ui = rot_enc_user_interface + rel->ui_list[rel->state];
  led_draw_s_selector(rel->slot, ui->value, ui->max, UI_H, 40);
  led_out(rel->slot);
}

void rel_draw_v_selector(rel_t *rel)
{
  reui_t *ui = rot_enc_user_interface + rel->ui_list[rel->state];
  led_draw_v_selector(rel->slot, ui->value, ui->max, 0, 0);
  led_out(rel->slot);
}

void rel_draw_position_selector(rel_t *rel)
{
  reui_t *ui = rot_enc_user_interface + rel->ui_list[rel->state];
  led_draw_position_selector(rel->slot, ui->value, ui->max, UI_W, 40);
  led_out(rel->slot);
}

void rel_draw_width_selector(rel_t *rel)
{
  reui_t *ui = rot_enc_user_interface + rel->ui_list[rel->state];
  led_draw_width_selector(rel->slot, ui->value, ui->max, UI_P, 40);
  led_out(rel->slot);
}

void rel_draw_kelvin_selector(rel_t *rel)
{
  reui_t *ui = rot_enc_user_interface + rel->ui_list[rel->state];
  led_draw_kelvin_selector(rel->slot, ui->value, ui->max, 16);
  led_out(rel->slot);
}




rel_t rot_enc_led[2];

/*
  slot: 0 or 1
  ui: 0 (HSV), 1 (Kelvin) or 2 (position & width)
*/
static uint8_t rot_color[] = { 0, 1, 2, 255};
static uint8_t rot_kelvin[] = { 5, 2, 255};
static uint8_t rot_geometry[] = { 3, 4, 255};

void rel_init(rel_t *rel, int slot, int ui)
{  
  rel->slot = slot;
  rel->state = 0; 	// not sufficient, call rel_set_state after rel_init() 
  if ( ui == 0 )
  {
    rel->ui_list = rot_color;
  }
  else if ( ui == 1 )
    rel->ui_list = rot_kelvin;
  else
    rel->ui_list = rot_geometry;
}


void rel_show_led(rel_t *rel)
{
  reui_t *ui = rot_enc_user_interface + rel->ui_list[rel->state];
  ui->draw_ui(rel);
}

void rel_set_state(rel_t *rel, int state)
{
  reui_t *ui;
  rel->state = state;
  ui = rot_enc_user_interface + rel->ui_list[rel->state];
  rot_enc_setup(rotary_encoder+rel->slot, ui->value, 0, ui->max, ui->is_wrap_around);
  rel_show_led(rel);
}

/* update target light source */
void led_update_light(void)
{
  int i;
  uint8_t r, g, b;
  
  led_clear_light();
  for( i = 0; i < LIGHT_SOURCES_CNT; i++ )
  {
    if ( light_sources[i][6] == 0 )
    {  
      /* mode 0: HSV */
      hsv_to_rgb(
	/* h */ ((unsigned)light_sources[i][0]*255)/(unsigned)rot_enc_user_interface[0].max, 
	/* s */ ((unsigned)light_sources[i][1]*255)/(unsigned)rot_enc_user_interface[1].max, 
	/* v */ ((unsigned)light_sources[i][2]*255)/(unsigned)rot_enc_user_interface[2].max, 
	&r, &g, &b);
    }
    else
    {
      /* mode 1: Kelvin */
	kelvin_to_rgb(
	/* kelvin */ ((unsigned)light_sources[i][5]*255)/(unsigned)rot_enc_user_interface[5].max, 
	/* v */ ((unsigned)light_sources[i][2]*255)/(unsigned)rot_enc_user_interface[2].max, 
	  &r, &g, &b);
    }
    
    led_add_light_source(
      /* w */ ((unsigned)light_sources[i][4]*255)/(unsigned)rot_enc_user_interface[4].max, 
      /* p */ ((unsigned)light_sources[i][3]*255)/(unsigned)rot_enc_user_interface[3].max, 
      r, g, b);
    }  
  
  ws2812_spi_out(LED_LIGHT_WS2812B_OUT_PIN, led_light_ring, LED_CNT*3);	
}


/* copy values from light editor to light specification table */
void store_editor_to_light_source(void)
{
  int i;
  //for ( i = 0; i < ARGS_PER_LIGHT; i++ )
  
  /* copy only the HSV, Kelvin, Position and Width out of all arguments */
  
  for ( i = 0; i < 6; i++ )
  {
    light_sources[current_light_editor][i] = rot_enc_user_interface[i].value;
  }
}

void copy_light_source_to_editor(void)
{
  int i;
  for ( i = 0; i < 6; i++ )
  {
    rot_enc_user_interface[i].value = light_sources[current_light_editor][i];
  }
}


/* copy the value from the rotary encoder detection to the rel object and show the result on the LED ring */
void rel_read_and_update_led(rel_t *rel)
{
  int button = bp_get_event(bp_rot_enc+rel->slot);
  
  if ( button == BP_EVENT_SHORT_PRESS )
  {
    int state = rel->state;
    state++;
    if ( rel->ui_list[state] > 30 )	// actually this should be == 255 
      state = 0;
    rel_set_state(rel, state);  // this will assign the new state and also set the led ring for the rotary encoder
  }
  else if ( button == BP_EVENT_LONG_PRESS )
  {
    if ( rel->ui_list == rot_color )
    {
      /* indicate purple to the user for Kelvin change */
      led_draw_all(0, 80, 0, 80);
      led_out(rel->slot);
      delay_micro_seconds(200000);
      
      /* update light editor user inter fache */
      light_sources[current_light_editor][6] = 1; /* Kelvin */
      rel_init(rel, rel->slot, 1);		/* change to kelvin */
      rel_set_state(rel, 0);  // this will assign the new state and also set the led ring for the rotary encoder
            
      /* update light itself */
      led_update_light();
    }
    else if ( rel->ui_list == rot_kelvin )
    {
      /* indicate green to the user for HSV change */
      led_draw_all(0, 0, 127, 0);
      led_out(rel->slot);
      delay_micro_seconds(200000);
      
      /* update light editor user inter fache */
      light_sources[current_light_editor][6] = 0; /* HSV */
      rel_init(rel, rel->slot, 0);		/* change to hsv color */
      rel_set_state(rel, 0);  // this will assign the new state and also set the led ring for the rotary encoder
      
      /* update light itself */
      led_update_light();
    }
    else
    {
      light_sources[current_light_editor][6] = 0; /* doesn't matter, this is geometry */
      /* do nothing, this is the geometry slot */
    }
  }
  else
  {
    uint8_t v = rotary_encoder[rel->slot].value;
    reui_t *ui = rot_enc_user_interface + rel->ui_list[rel->state];
    if ( ui->value != v )
    {
      /* update LED ring only if required */
      ui->value = v;
      /* update light editor */
      rel_show_led(rel);
      /* copy current editor value to the light source array */
      store_editor_to_light_source();
      /* update the light based on the light source array */
      led_update_light();
    }
  }
}


/*
  switch editor to a different light source
*/
void change_light(void)
{
  store_editor_to_light_source();
  current_light_editor++;
  if ( current_light_editor >= LIGHT_SOURCES_CNT )
    current_light_editor = 0;

  led_draw_all(0, 40, 40, 40);
  led_out(0);
  led_draw_all(1, 40, 40, 40);
  led_out(1);

  delay_micro_seconds(10000);
  
  copy_light_source_to_editor();

  rel_init(rot_enc_led+0, 0, light_sources[current_light_editor][6]);	/* restore HSV/Kelvin mode */
  rel_set_state(rot_enc_led+0, 0);
  rel_set_state(rot_enc_led+1, 0);
  
}

void handle_mode_button(void)
{
  int event = bp_get_event(&bp_middle_button);
  if ( event == BP_EVENT_SHORT_PRESS )
  {
    change_light();
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
/* system procedures and sys tick master task */



volatile uint32_t sys_tick_irq_cnt=0;

void __attribute__ ((interrupt)) SysTick_Handler(void)
{  
  sys_tick_irq_cnt++;
  //GPIOSetBitValue(PORT0, 9, (sys_tick_irq_cnt & 1) == 0?0:1);
  
  bp_execute(&bp_middle_button);
  bp_execute(bp_rot_enc+0);
  bp_execute(bp_rot_enc+1);

  
}


/*=======================================================================*/
/*
  setup the hardware and start interrupts.
  called by "Reset_Handler"
*/
int __attribute__ ((noinline)) main(void)
{
  int i;

  usart_t usart;
  uint8_t usart_rx_buf[32];
  

  /* call to the lpc lib setup procedure. This will set the IRC as clk src and main clk to 24 MHz */
  SystemInit(); 

  /* if the clock or PLL has been changed, also update the global variable SystemCoreClock */
  SystemCoreClockUpdate();

  /* hardware subsystem setup */
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

  bp_init(&bp_middle_button, MODE_BUTTON_PIN);
  bp_init(bp_rot_enc+0, ROT_ENC_0_BUTTON);
  bp_init(bp_rot_enc+1, ROT_ENC_1_BUTTON);

  rot_enc_setup(rotary_encoder+0, 0, 0, 255, 0);
  rot_enc_setup(rotary_encoder+1, 0, 0, 255, 0);
  
  rel_init(rot_enc_led+0, 0, 0);
  rel_init(rot_enc_led+1, 1, 2);

  init_light_sources();
  copy_light_source_to_editor();

  rel_set_state(rot_enc_led+0, 0);
  rel_set_state(rot_enc_led+1, 0);


  /* set systick and start systick interrupt (after all subsystems are ready */
  SysTick_Config(main_clk/1000UL*(unsigned long)SYS_TICK_PERIOD_IN_MS);
  

  led_clear_light();
  ws2812_spi_out(LED_LIGHT_WS2812B_OUT_PIN, led_light_ring, LED_CNT*3);	


  for(;;)
  {
    //GPIOSetBitValue(PORT0, 15, 1);
    //delay_micro_seconds(1000000);
    //GPIOSetBitValue(PORT0, 15, 0);
    
    for( i = 0; i < 10; i++ )
    {
      /*
      led_clear(0);
      //led_set_rgb(0, rotary_encoder[0].value, 40,0,0);
      //led_add_rgb(0, rotary_encoder[1].value, 0,40,0);
      
      rotary_encoder[1].value &= 0x03f;
      led_draw_h_selector(0, rotary_encoder[1].value, 0x040, 255, 50);
      //led_draw_v_selector(0, rotary_encoder[1].value, 0x040, 0, 0);
      led_out(0);
      */
      
      rel_read_and_update_led(rot_enc_led+0);
      rel_read_and_update_led(rot_enc_led+1);
      handle_mode_button();
      
      delay_micro_seconds(1000000/20);
    }
    

    
    
    usart_write_string(&usart, " edit light source=");    
    usart_write_u16(&usart, current_light_editor);

    if ( light_sources[current_light_editor][6] == 0 )
    {
      usart_write_string(&usart, " HSV Mode");    
    }
    else
    {
      usart_write_string(&usart, " Kelvin Mode");    
    }
    
    usart_write_string(&usart, " re0 value=");    
    usart_write_string(&usart, u8toa(rotary_encoder[0].value ,3));    

    usart_write_string(&usart, "/");    
    usart_write_string(&usart, u8toa(rotary_encoder[0].max ,3));    

    usart_write_string(&usart, " re1 value=");    
    usart_write_string(&usart, u8toa(rotary_encoder[1].value ,3));    

    usart_write_string(&usart, "/");    
    usart_write_string(&usart, u8toa(rotary_encoder[1].max ,3));    

    /*
    usart_write_string(&usart, " USAT(270, 9)=");    
    usart_write_string(&usart, u8toa(__USAT(270, 9) ,3));    
    usart_write_string(&usart, " USAT(270, 8)=");    
    usart_write_string(&usart, u8toa(__USAT(270, 8) ,3));    
    */

    usart_write_string(&usart, "\n"); 
    

    //led_update_light();
    //led_add_light_source(4, 0, 255, 255, 255);

  }
  kelvin_to_rgb(1,2,NULL, NULL, NULL);
}

