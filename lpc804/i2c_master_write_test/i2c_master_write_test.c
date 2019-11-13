
#include <LPC8xx.h>
#include <syscon.h>
#include <iocon.h>
#include <gpio.h>
#include <swm.h>
#include <i2c.h>
#include <delay.h>
#include "util.h"	/* i2c procedures */



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
/*
  setup the hardware and start interrupts.
  called by "Reset_Handler"
*/
int __attribute__ ((noinline)) main(void)
{
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
  
#define TEST3

#ifdef TEST0
  /* all ok, no error no blink code, assumes SSD1306 OLED on the bus */
  LPC_IOCON->PIO0_14 |= 1<<IOCON_OD;
  LPC_IOCON->PIO0_7 |= 1<<IOCON_OD;
  map_function_to_port(I2C0_SDA, 14);
  map_function_to_port(I2C0_SCL, 7);
  i2c_init(37);	/* 100KHz @15MHz */
  {
    uint8_t buf[1] = { 0x0af };
    err = i2c_write(0x078, buf, 1);
  }
#endif 

#ifdef TEST1
  /* wrong address --> I2C_NO_TX_POST_ADR 5 */
  LPC_IOCON->PIO0_14 |= 1<<IOCON_OD;
  LPC_IOCON->PIO0_7 |= 1<<IOCON_OD;
  map_function_to_port(I2C0_SDA, 14);
  map_function_to_port(I2C0_SCL, 7);
  i2c_init(37);	/* 100KHz @15MHz */
  {
    uint8_t buf[1] = { 0x0af };
    err = i2c_write(0x011, buf, 1);
  }
#endif 

#ifdef TEST2
  /* SCL pulled down --> I2C_TIMEOUT_POST_ADR 2 */
  LPC_IOCON->PIO0_9 =  IOCON_RESERVED789 | MODE_PULLDOWN;  
  LPC_IOCON->PIO0_14 |= 1<<IOCON_OD;
  LPC_IOCON->PIO0_7 |= 1<<IOCON_OD;
  map_function_to_port(I2C0_SDA, 14);
  map_function_to_port(I2C0_SCL, 9);
  i2c_init(37);	/* 100KHz @15MHz */
  {
    uint8_t buf[1] = { 0x0af };
    err = i2c_write(0x011, buf, 1);
  }
#endif 

#ifdef TEST3
  /* SDA pulled down --> I2C_NO_TX_POST_ADR 5 */
  LPC_IOCON->PIO0_9 =  IOCON_RESERVED789 | MODE_PULLDOWN;  
  LPC_IOCON->PIO0_14 |= 1<<IOCON_OD;
  LPC_IOCON->PIO0_7 |= 1<<IOCON_OD;
  map_function_to_port(I2C0_SDA, 9);
  map_function_to_port(I2C0_SCL, 7);
  i2c_init(37);	/* 100KHz @15MHz */
  {
    uint8_t buf[1] = { 0x0af };
    err = i2c_write(0x011, buf, 1);
  }
#endif 
  
  i2c_out_blink_code(err);
  
}

