/*

  ws2812b.c

  PLU will be configured as SPI client. 
  SPI clock should 800kHz: SPI clock div for the main clk can be 18 (833kHz) or 19 (789kHz)
  
  PLU configuration:
    sck: PIO0_22;
    mosi: PIO0_21;
  
*/


#include <LPC8xx.h>
#include <iocon.h>
#include <syscon.h>
#include <gpio.h>
#include <swm.h>
#include <spi.h>
#include <delay.h>

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

void spi_init(void)
{
  Enable_Periph_Clock(CLK_SPI0);
  
  mapFunctionToPort(SPI0_SCK, 9);
  mapFunctionToPort(SPI0_MOSI, 11);
  
  LPC_SYSCON->SPI0CLKSEL = FCLKSEL_MAIN_CLK;	
  LPC_SPI0->DIV = 24;
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
int __attribute__ ((noinline)) main(void)
{
  uint8_t a[] = { 0, 0x3c, 0,  0, 0, 0x3c};
  int i;
  /* call to the lpc lib setup procedure. This will set the IRC as clk src and main clk to 24 MHz */
  SystemInit(); 

  /* if the clock or PLL has been changed, also update the global variable SystemCoreClock */
  SystemCoreClockUpdate();

  /* set systick and start systick interrupt */
  SysTick_Config(main_clk/1000UL*(unsigned long)SYS_TICK_PERIOD_IN_MS);


    
  plu();		/* plu() will enable GPIO0 & SWM clock */
  spi_init();	

  delay_micro_seconds(50);
  for(;;)
  {
    a[0]++;
    a[0] &= 255;
    spi_out(a, 6);
    
    /* LPC804 has might have up to two bytes in the queue, wait for their transmission */    
    delay_micro_seconds(24+24);
    /* write the new values into the LEDs */
    delay_micro_seconds(50);
    
    /* end user delay */
    delay_micro_seconds(10000L);
  }
  
}
