/*
  u8x8cb.c
  
  
  
  u8g2_Setup_ssd1306_i2c_128x64_noname_1(&u8g2, U8G2_R0, u8x8_byte_sw_i2c, u8x8_gpio_and_delay_lpc804_sw_i2c);
  u8g2_Setup_ssd1306_i2c_128x64_noname_1(&u8g2, U8G2_R0, u8x8_byte_lpc804_i2c, u8x8_gpio_and_delay_lpc804_only_delay);
    
*/

#include "LPC8xx.h"
#include "gpio.h"
#include "delay.h"
#include "i2c.h"
#include "u8x8.h"


int i2c_write( uint8_t adr, uint8_t *buf, uint8_t len );


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


uint8_t u8x8_gpio_and_delay_lpc804_only_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  switch(msg)
  {
    case U8X8_MSG_GPIO_AND_DELAY_INIT:
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
  }
  return 1;    
}




/*
  template from https://github.com/olikraus/u8g2/wiki/Porting-to-new-MCU-platform

  assumes that the I2C0 outputs are mapped to the pins:
    mapFunctionToPort(I2C0_SDA, 14);
    mapFunctionToPort(I2C0_SCL, 7);


*/
uint8_t u8x8_byte_lpc804_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  /* u8g2/u8x8 will never send more than 32 bytes between START_TRANSFER and END_TRANSFER */
  /* add extra byte for the i2c address */
  static uint8_t buffer[34];		
  static uint8_t buf_idx;
  uint8_t *data;
 
  switch(msg)
  {
    case U8X8_MSG_BYTE_SEND:
      data = (uint8_t *)arg_ptr;      
      while( arg_int > 0 )
      {
	buffer[buf_idx++] = *data;
	data++;
	arg_int--;
      }      
      break;
    case U8X8_MSG_BYTE_INIT:
      /* add your custom code to init i2c subsystem */
      break;
    case U8X8_MSG_BYTE_SET_DC:
      /* ignored for i2c */
      break;
    case U8X8_MSG_BYTE_START_TRANSFER:
      buf_idx = 0;
      break;
    case U8X8_MSG_BYTE_END_TRANSFER:
      //buffer[0] = u8x8_GetI2CAddress(u8x8);
      //I2CmasterWrite( buffer, buf_idx-1);
      i2c_write(u8x8_GetI2CAddress(u8x8), buffer, buf_idx);
      //i2c_write(0x078, buffer, buf_idx);
      break;
    default:
      return 0;
  }
  return 1;
}
