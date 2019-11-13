
#include <LPC8xx.h>
#include <syscon.h>
#include <iocon.h>
#include <gpio.h>
#include <swm.h>
#include <i2c.h>
#include <delay.h>
#include "util.h"	/* i2c procedures */
#include "u8g2.h"


/*=======================================================================*/
/* u8x8cb.c */
uint8_t u8x8_gpio_and_delay_lpc804_sw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
uint8_t u8x8_gpio_and_delay_lpc804_only_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
uint8_t u8x8_byte_lpc804_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);


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



void i2c_out_blink_code(int err)
{
  int i;
  for(;;)
  {
    for( i = 0; i < err; i++ )
    {
      GPIOSetBitValue(PORT0, 15, 1);
      delay_micro_seconds(20000);
      GPIOSetBitValue(PORT0, 15, 0);
      delay_micro_seconds(400000);
    }
    delay_micro_seconds(1000000);
  }
}

/*=======================================================================*/

u8g2_t u8g2;
static uint16_t offset = 0;

void draw(u8g2_t *u8g2)
{
    u8g2_SetFontMode(u8g2, 1);	// Transparent
    u8g2_SetFontDirection(u8g2, 0);
    u8g2_SetFont(u8g2, u8g2_font_inb24_mf);
    u8g2_DrawStr(u8g2, 0+offset, 30, "U");
    
    u8g2_SetFontDirection(u8g2, 1);
    u8g2_SetFont(u8g2, u8g2_font_inb30_mn);
    u8g2_DrawStr(u8g2, 21+offset,8,"8");
        
    u8g2_SetFontDirection(u8g2, 0);
    u8g2_SetFont(u8g2, u8g2_font_inb24_mf);
    u8g2_DrawStr(u8g2, 51+offset,30,"g");
    u8g2_DrawStr(u8g2, 67+offset,30,"\xb2");
    
    u8g2_DrawHLine(u8g2, 2, 35, 47);
    u8g2_DrawHLine(u8g2, 3, 36, 47);
    u8g2_DrawVLine(u8g2, 45, 32, 12);
    u8g2_DrawVLine(u8g2, 46, 33, 12);
  
    u8g2_SetFont(u8g2, u8g2_font_4x6_tr);
    u8g2_DrawStr(u8g2, 1,54,"github.com/olikraus/u8g2");
}

/*=======================================================================*/
/*
  setup the hardware and start interrupts.
  called by "Reset_Handler"
*/
int __attribute__ ((noinline)) main(void)
{
  uint8_t cnt = 0;
  int err;

  /* call to the lpc lib setup procedure. This will set the IRC as clk src and main clk to 24 MHz */
  SystemInit(); 

  /* if the clock or PLL has been changed, also update the global variable SystemCoreClock */
  SystemCoreClockUpdate();

  /* set systick and start systick interrupt */
  SysTick_Config(main_clk/1000UL*(unsigned long)SYS_TICK_PERIOD_IN_MS);
  
  /* setup GPIO subsystem */
  GPIOInit();
  
  /* enable clock for several subsystems */
  Enable_Periph_Clock(CLK_IOCON);
  Enable_Periph_Clock(CLK_SWM);

  /* systick alive blink */
  GPIOSetDir( PORT0, 15, OUTPUT);
  LPC_IOCON->PIO0_9 =  IOCON_RESERVED789 | MODE_PULLUP;
  
  LPC_IOCON->PIO0_14 |= 1<<IOCON_OD;
  LPC_IOCON->PIO0_7 |= 1<<IOCON_OD;
  map_function_to_port(I2C0_SDA, 14);
  map_function_to_port(I2C0_SCL, 7);

  i2c_init(9);

  {
    uint8_t buf[1] = { 0x0af };
    err = i2c_write(0x078, buf, 1);
  }
  if ( err > 0 )
    i2c_out_blink_code(err);
  
  //GPIOSetDir( PORT0, 14, OUTPUT);
  //GPIOSetDir( PORT0, 7, OUTPUT);


  /* setup display */
  /*
  u8g2_Setup_ssd1306_i2c_128x64_noname_1(&u8g2, U8G2_R0, u8x8_byte_sw_i2c, u8x8_gpio_and_delay_lpc804_sw_i2c);
  */
  u8g2_Setup_ssd1306_i2c_128x64_noname_1(&u8g2, U8G2_R0, u8x8_byte_lpc804_i2c, u8x8_gpio_and_delay_lpc804_only_delay);
  u8g2_InitDisplay(&u8g2);
  u8g2_SetPowerSave(&u8g2, 0);

  for(;;)
  {
    u8g2_FirstPage(&u8g2);
    do
    {
      draw(&u8g2);
    } while( u8g2_NextPage(&u8g2) );
    
    //GPIOSetBitValue(PORT0, 15, (cnt & 1) == 0?0:1);
    cnt++;
    offset++;
    offset &=7;
    //delay_micro_seconds(1000000);
  }
}

