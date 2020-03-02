/*

  sol_bat.c

  minicom -D /dev/ttyUSB0
  
  Output the ADC values for GPIO ports 0_7, 0_8 and 0_15

  0_8: Input voltage (solar panal)
  0_15: Voltage for the comparator/Middle voltage before 10Ohm resistor
  0_7: Output voltage after 10Ohm resistor (battery)

  0_1: Comparator input

*/


#include "LPC8xx.h"
#include "iocon.h"
#include "syscon.h"
#include "gpio.h"
#include "swm.h"
#include "acomp.h"
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

usart_t usart;
uint8_t usart_rx_buf[32];

uint16_t in_adc_value;
uint16_t fb_adc_value;
uint16_t out_adc_value;

uint16_t in_millivolt;
uint16_t fb_millivolt;
uint16_t out_millivolt;


/*
  Uadc = 47 / (220 + 47 ) * Uq
  Uadc =  3300 * Vadc / 4095 = 660 * Vadc / 819


  47 / (220 + 47) * Uq = 600 * Vadc / 819
  47 / 267 * Uq = 600 * Vadc / 819
  Uq = 600*267*Vadc / (819 *47)

  Uq = 4.1618 * Vadc 
  Uq = 8523 * Vadc >> 11;
*/
uint16_t get_millivolt_by_adc(uint16_t adc_value)
{
  uint32_t u;
  u = adc_value;
  u *= 8523;
  u >>= 11;
  return u;
}

void calc(void)
{
  in_adc_value = adc_read(8);
  fb_adc_value = adc_read(15);
  out_adc_value = adc_read(7);
  
  in_millivolt = get_millivolt_by_adc(in_adc_value);
  fb_millivolt = get_millivolt_by_adc(fb_adc_value);
  out_millivolt = get_millivolt_by_adc(out_adc_value);

  usart_write_string(&usart, "Input: ");
  usart_write_string(&usart, u16toa(in_millivolt));
  usart_write_string(&usart, "mV ");

  usart_write_string(&usart, "FB: ");
  usart_write_string(&usart, u16toa(fb_millivolt));
  usart_write_string(&usart, "mV ");

  usart_write_string(&usart, "Out: ");
  usart_write_string(&usart, u16toa(out_millivolt));
  usart_write_string(&usart, "mV ");
  
}

void out_adc(const char *s, uint8_t adc_port)
{
  uint16_t adc = adc_read(adc_port);
  uint16_t mv = get_millivolt_by_adc(adc);
  
  usart_write_string(&usart, s);
  usart_write_string(&usart, ": ");
  usart_write_string(&usart, u16toa(mv));
  usart_write_string(&usart, "mV (");
  usart_write_string(&usart, u16toa(adc));
  usart_write_string(&usart, ") ");
  
}



int __attribute__ ((noinline)) main(void)
{
  /* call to the lpc lib setup procedure. This will set the IRC as clk src and main clk to 24 MHz */
  SystemInit(); 

  /* if the clock or PLL has been changed, also update the global variable SystemCoreClock */
  SystemCoreClockUpdate();

  /* set systick and start systick interrupt */
  SysTick_Config(main_clk/1000UL*(unsigned long)SYS_TICK_PERIOD_IN_MS);

  
  plu();		/* plu() will enable GPIO0 clock */
  

  Enable_Periph_Clock(CLK_IOCON);
  Enable_Periph_Clock(CLK_SWM);
  
  Enable_Periph_Clock(CLK_ACMP);
  
  /* assume a LED at port 0_2 and let the LED flash during the adc measurements */
  LPC_SWM->PINENABLE0 |= SWDIO;	// disable SWDIO @PIN0_2 so that we can use the LED
  GPIOSetDir( PORT0, 2, OUTPUT);

  *get_iocon_by_port(1) &= IOCON_MODE_MASK;	/* clear pullup on the comparator */
  *get_iocon_by_port(15) &= IOCON_MODE_MASK;	/* clear pullup on comparator ADC (also done by adc_read) */


  LPC_SYSCON->PDRUNCFG &= ~ACMP_PD;	/* power up analog comparator */

  LPC_SWM->PINENABLE0 &= ~ACMP_I2;		/* enable ACMP input at PIN0_1 */
  
  LPC_CMP->LAD = 1 | (2<<1);				/* enable ladder, 26*3.3V/31 = 0.213V */

  /* compare the ladder output against voltage at PIO0_1 */
  LPC_CMP->CTRL =
    ( 1 << COMPSA ) |					/* sync with bus clock */
    (ACOMP_IN2 << COMP_VP_SEL) |			/* PIO0_1 */
    (V_LADDER_OUT << COMP_VM_SEL);		/* 0.213V */

  /* init usart to output ADC results to a connected terminal */
  usart0_init(&usart, 5, /* tx */ 4, /* rx */ 0, usart_rx_buf, 32);  
  
  /* init adc */
  adc_init();

  for(;;)
  {
    //GPIOSetBitValue(PORT0, 2, 1);
    delay_micro_seconds(5000);
    GPIOSetBitValue(PORT0, 2, 0);
    delay_micro_seconds(500000);
    
    calc();
    
    usart_write_string(&usart, "\r\n");
  }
}
