
#include <LPC8xx.h>
#include <syscon.h>
#include <iocon.h>
#include <gpio.h>
#include <swm.h>
#include <i2c.h>
#include <delay.h>
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
/* 
  replacement for ConfigSWM(uint32_t func, uint32_t port_pin) 
  
  Args:
    fn: A function number, e.g. T0_MAT0, see swm.h
    port: A port number for the GPIO port (0..30)

*/
void mapFunctionToPort(uint32_t fn, uint32_t port)
{
  /* first reset the pin assignment to 0xff (this is also the reset value */
  LPC_SWM->PINASSIGN[fn/4] |= ((0xffUL)<<(8*(fn%4)));
  /* then write the destination pin to it */
  LPC_SWM->PINASSIGN[fn/4] &= ~((port^255UL)<<(8*(fn%4)));
}
/*=======================================================================*/


void i2c_init(void)
{
  
  //for(;;)
  {
    Enable_Periph_Clock(CLK_I2C0);
    Do_Periph_Reset(RESET_I2C0);
    /*
      15 MHz, I2C Busclock 100MHz
	CLKDIV = 74  
    */
    LPC_SYSCON->I2C0CLKSEL = 1;	/* main clock */
    
    LPC_I2C0->CLKDIV = 74;
    LPC_I2C0->MSTTIME = 0;
    
    
    LPC_I2C0->CFG = CFG_MSTENA;

    /*
    {
      uint8_t buf[1] = { 0xaf };
      i2c_write( 0x78, buf, 1);
    }
    */
    
  }  
  
}

/*
  return:
    0: time out
    1: all ok
*/
int i2c_wait(void)
{
  uint32_t cnt = 0;
  int i = 0;
  while(!(LPC_I2C0->STAT & STAT_MSTPEND) )
  {
    cnt++;
    if ( cnt > (1<<17) )
      return 0;
  }
  return 1;
}

#define I2C_OK 0
#define I2C_TIMEOUT_PRE_ADR 1

/* pending flag is active too long */
/* root cause: */
/* 1) SWM mapping was done after I2C subsystem enable (it must be done before) */
/* 2) SCL line is permanently pulled down */
#define I2C_TIMEOUT_POST_ADR 2

#define I2C_TIMEOUT_TX 3

/* there was an attempt to access the I2C bus, but the I2C subsystem is not ready */
#define I2C_NOT_IDLE 4 
/* transmission not available after sending the address with write request */
/* root cause: 1) wrong address, client not available */
/* 2) SDA line is mapped to an incorrect port (permanently pull up or pull down) */
/* 3) SCL line is mapped to an incorrect port (permanently pull up) */
#define I2C_NO_TX_POST_ADR 5

/* transmission not available after sending data (maybe NACK by client) */
#define I2C_NO_TX_POST_TX 6

int i2c_write( uint8_t adr, uint8_t *buf, uint8_t len )
{	
  uint32_t i;

  // Wait for MSTPENDING bit set in STAT register
  if ( i2c_wait() == 0 )
    return I2C_TIMEOUT_PRE_ADR;
  if ((LPC_I2C0->STAT & MASTER_STATE_MASK) != I2C_STAT_MSTST_IDLE)
    return I2C_NOT_IDLE;
  
  LPC_I2C0->MSTDAT = adr | 0;    					// Address with 0 for RWn bit (WRITE)
  LPC_I2C0->MSTCTL = CTL_MSTSTART;										// Start the transaction by setting the MSTSTART bit to 1 in the Master control register.
  if ( i2c_wait() == 0 )   // Wait for the address to be ACK'd
    return I2C_TIMEOUT_POST_ADR;
  if ((LPC_I2C0->STAT & MASTER_STATE_MASK) != I2C_STAT_MSTST_TX)
    return I2C_NO_TX_POST_ADR;
  
  for ( i = 0; i < len; i++ ) 
  {
    LPC_I2C0->MSTDAT = buf[i];               // Send the data to the slave
    LPC_I2C0->MSTCTL = CTL_MSTCONTINUE;                // Continue the transaction
    if ( i2c_wait() == 0 )  // Wait for the address to be ACK'd
      return I2C_TIMEOUT_TX;
    if ((LPC_I2C0->STAT & MASTER_STATE_MASK) != I2C_STAT_MSTST_TX)
      return I2C_NO_TX_POST_TX;
  }
  
  LPC_I2C0->MSTCTL = CTL_MSTSTOP;                    // Send a stop to end the transaction
  return I2C_OK;
}

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
  mapFunctionToPort(I2C0_SDA, 14);
  mapFunctionToPort(I2C0_SCL, 7);

  i2c_init();

  //LPC_IOCON->PIO0_14 |= 1<<IOCON_OD;
  //LPC_IOCON->PIO0_7 |= 1<<IOCON_OD;
  
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

