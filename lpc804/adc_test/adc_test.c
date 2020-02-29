/*

  adc_test.c

  minicom -D /dev/ttyUSB0

*/

#include <stddef.h>
#include <LPC8xx.h>
#include <syscon.h>
#include <gpio.h>
#include <iocon.h>
#include <swm.h>
#include <uart.h>
#include <adc.h>
#include <delay.h>
#include <util.h>


/*=======================================================================*/
/* Configuration */
#define SYS_TICK_PERIOD_IN_MS 100


/*=======================================================================*/
/* system procedures and sys tick master task */



volatile uint32_t sys_tick_irq_cnt=0;


void __attribute__ ((interrupt)) SysTick_Handler(void)
{  
  sys_tick_irq_cnt++;
  GPIOSetBitValue(PORT0, 9, (sys_tick_irq_cnt & 1) == 0?0:1);
}


/*=======================================================================*/

static const uint8_t port_to_adc[17] = { 
  255, 
  0,		/* port 0_1 --> ADC_0 */
  255,
  255,
  11,		/* port 0_4 --> ADC_11 */
  255,
  255,
  1,		/* port 0_7 --> ADC_1 */
  5,		/* port 0_8 --> ADC_5 */
  4,		/* port 0_9 --> ADC_4 */
  7,		/* port 0_10 --> ADC_7 */
  6,		/* port 0_11 --> ADC_6 */
  255,
  10,		/* port 0_13 --> ADC_10 */
  2,		/* port 0_14 --> ADC_2 */
  8,		/* port 0_15 --> ADC_8 */
  3		/* port 0_16 --> ADC_3 */
};

int16_t get_adc_by_port(uint8_t port)
{
  uint8_t adc;
  if ( port >= 17 )
    return -1;
  adc = port_to_adc[port];
  if ( adc == 255 )
    return -1;
  return adc;
}
    

void adc_init(void)
{

  LPC_SYSCON->PDRUNCFG &= ~ADC_PD;	/* power up ADC */
  LPC_SYSCON->ADCCLKSEL = 0;			/* FRO */
  LPC_SYSCON->ADCCLKDIV = 1;			/* devide by 1 */
  Enable_Periph_Clock(CLK_ADC);			/* enable clock for ADC */
  __NOP();								/* probably not required */
  LPC_ADC->CTRL = (1<<ADC_LPWRMODE) ;	
    /* clkdiv=0, enable low power mode */
  
  
}

/* 
  precondition: SWM and GPIO clock must be enabled 
  return: 12 bit ADC result at the specified port
*/
uint16_t adc_read(uint8_t port)
{
  uint32_t result;
  int16_t adc = get_adc_by_port(port);
  if ( adc < 0 )
    return 0xffff;	/* invalid pin */
  
  LPC_SWM->PINENABLE0 &= ~(1<<(adc+10));	/* ADC_0 is at bit 10 */

  *get_iocon_by_port(port) &= IOCON_MODE_MASK;	/* clear any pullup/pulldown */
  
  LPC_ADC->SEQA_CTRL = 1<<adc;			/* select the adc, clear all other bits */
  LPC_ADC->SEQA_CTRL |= (1<<ADC_TRIGPOL);	/* set the trigger polarity to low-high transition */
  LPC_ADC->SEQA_CTRL |= (1<<ADC_SEQ_ENA);
  
  //result = LPC_ADC->DAT[adc];				/* read data reg, to clear the DATAVALID flag */
    
  LPC_ADC->SEQA_CTRL |= (1<<ADC_START);		/* start the conversion */
  
  /* wait for the result */
  for(;;)
  {
    result = LPC_ADC->DAT[adc];				/* read data reg, to clear the DATAVALID flag */
    if ( (result & 0x80000000) != 0 )
      break;
    
    /*
    result = LPC_ADC->SEQA_GDAT;				
    if ( (result & 0x80000000) != 0 )
      break;
    */
  }
  
  return (result>>4) & 0x0fff;
}

/*=======================================================================*/
/*
  setup the hardware and start interrupts.
  called by "Reset_Handler"
*/
int __attribute__ ((noinline)) main(void)
{
  uint16_t cnt = 0;

  usart_t usart;
  uint8_t usart_rx_buf[32];
  

  /* call to the lpc lib setup procedure. This will set the IRC as clk src and main clk to 24 MHz */
  SystemInit(); 

  /* if the clock or PLL has been changed, also update the global variable SystemCoreClock */
  SystemCoreClockUpdate();

  /* set systick and start systick interrupt */
  SysTick_Config(main_clk/1000UL*(unsigned long)SYS_TICK_PERIOD_IN_MS);
  
  /* */
  GPIOInit();
  
  /* enable clock for several subsystems */

  Enable_Periph_Clock(CLK_IOCON);
  Enable_Periph_Clock(CLK_SWM);

  usart0_init(&usart, 5, /* tx */ 4, /* rx */ 0, usart_rx_buf, 32);  

  LPC_SWM->PINENABLE0 |= SWDIO;	// disable SWDIO @PIN0_2 so that we can use the LED
  GPIOSetDir( PORT0, 2, OUTPUT);
  
  adc_init();

  for(;;)
  {
    GPIOSetBitValue(PORT0, 2, 1);
    delay_micro_seconds(1000000);
    GPIOSetBitValue(PORT0, 2, 0);
    delay_micro_seconds(1000000);
    usart_write_string(&usart, "cnt: ");
    usart_write_string(&usart, u16toa(cnt));
    usart_write_string(&usart, "\r\n");
    
    usart_write_string(&usart, "port 0_8: ");
    usart_write_string(&usart, u16toa(adc_read(8)));
    usart_write_string(&usart, "\r\n");

    usart_write_string(&usart, "port 0_15: ");
    usart_write_string(&usart, u16toa(adc_read(15)));
    usart_write_string(&usart, "\r\n");
    
    cnt++;
  }
}

