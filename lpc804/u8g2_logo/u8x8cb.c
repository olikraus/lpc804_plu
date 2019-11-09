/*
  u8x8cb.c
*/

#include "LPC8xx.h"
#include "gpio.h"
#include "delay.h"
#include "u8x8.h"

/* although we use SW I2C, use the HW pins for later upgrade */
#define I2C_CLOCK_PIN 7
#define I2C_DATA_PIN  14

uint8_t u8x8_gpio_and_delay_lpc804_sw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  switch(msg)
  {
    case U8X8_MSG_GPIO_AND_DELAY_INIT:
      /* only support for software I2C*/
    
      /* I2C: assume external pullup */
      GPIOSetDir( PORT0, I2C_CLOCK_PIN, OUTPUT);
      GPIOSetDir( PORT0, I2C_DATA_PIN, OUTPUT);
        
      break;
    case U8X8_MSG_DELAY_NANO:
      /* not required for SW I2C */
      break;
    
    case U8X8_MSG_DELAY_10MICRO:
      delay_micro_seconds(arg_int*10UL);
      break;
    
    case U8X8_MSG_DELAY_100NANO:
      delay_micro_seconds(arg_int/10UL);
      break;
   
    case U8X8_MSG_DELAY_MILLI:
      delay_micro_seconds(arg_int*1000UL);
      break;
    case U8X8_MSG_DELAY_I2C:
      /* arg_int is 1 or 4: 100KHz (5us) or 400KHz (1.25us) */
      delay_micro_seconds(arg_int<=2?5:1);
      break;
    
    case U8X8_MSG_GPIO_I2C_CLOCK:
      if ( arg_int == 0 )
      {
	GPIOSetDir( PORT0, I2C_CLOCK_PIN, OUTPUT);	
	GPIOSetBitValue(PORT0, I2C_CLOCK_PIN, 0);
      }
      else
      {
	GPIOSetDir( PORT0, I2C_CLOCK_PIN, INPUT);	
      }
      break;
    case U8X8_MSG_GPIO_I2C_DATA:
      if ( arg_int == 0 )
      {
	GPIOSetDir( PORT0, I2C_DATA_PIN, OUTPUT);	
	GPIOSetBitValue(PORT0, I2C_DATA_PIN, 0);
      }
      else
      {
	GPIOSetDir( PORT0, I2C_DATA_PIN, INPUT);	
      }
      break;
/*
    case U8X8_MSG_GPIO_MENU_SELECT:
      u8x8_SetGPIOResult(u8x8, GPIOGetPinValue(PORT0, KEY_SELECT_PIN));
      break;
    case U8X8_MSG_GPIO_MENU_NEXT:
      u8x8_SetGPIOResult(u8x8, GPIOGetPinValue(PORT0, KEY_NEXT_PIN));
      break;
    case U8X8_MSG_GPIO_MENU_PREV:
      u8x8_SetGPIOResult(u8x8, GPIOGetPinValue(PORT0, KEY_PREV_PIN));
      break;
    
    case U8X8_MSG_GPIO_MENU_HOME:
      u8x8_SetGPIOResult(u8x8, GPIOGetPinValue(PORT0, KEY_HOME_PIN));
      break;
*/
    default:
      u8x8_SetGPIOResult(u8x8, 1);
      break;
  }
  return 1;
}
