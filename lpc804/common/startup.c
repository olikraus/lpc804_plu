
/*

  startup.c

  LPC804

  Copyright (c) 2016, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification, 
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list 
    of conditions and the following disclaimer.
    
  * Redistributions in binary form must reproduce the above copyright notice, this 
    list of conditions and the following disclaimer in the documentation and/or other 
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  


  Requires code from the LPC804 example zip files

*/
#include <LPC8xx.h>


/*=======================================================================*/
/* external functions */
void __attribute__ ((interrupt)) SysTick_Handler(void);
//void __attribute__ ((interrupt, used)) UART_IRQ(void);
int __attribute__ ((noinline)) main(void);



/*=======================================================================*/
/* 
  Reserve some space for the stack. This is used to check if global variables + stack exceed RAM size.
  If -Wl,--gc-sections is used, then also define -Wl,--undefined=arm_stack_area to keep the variable in RAM.
  The name of the variable (here: arm_stack_area) does not matter.

  Heap (=dynamic memory allocation) is not supported
*/
#ifndef __STACK_SIZE
#define __STACK_SIZE 0x100
#endif
unsigned char arm_stack_area[__STACK_SIZE] __attribute__ ((section(".stack"))) __attribute__ ((aligned(8)));


/*=======================================================================*/
/* isr system procedures */

/* make the top of the stack known to the c compiler, value will be calculated by the linker script */
void __StackTop(void);

void __attribute__ ((interrupt)) __attribute__ ((noreturn)) Reset_Handler(void)
{
  register unsigned long *ptr;
  register unsigned long *start;
  register unsigned long *end;

  /*
    The following two operations are not required in normal mode reset mode,
    however it is usefull if the reset handler is started via ISP "Go" command
  */
  __set_MSP((uint32_t)__StackTop);
  //Chip_SYSCTL_Map(2);
  LPC_SYSCON->SYSMEMREMAP = (uint32_t) 2;	// user flash mode: use user isr vector table
  
  /*     
  Loop to copy data from read only memory to RAM. The ranges
  of copy from/to are specified by following symbols evaluated in 
  linker script.
  __etext: End of code section, i.e., begin of data sections to copy from.
  __data_start__/__data_end__: RAM address range that data should be
  copied to. Both must be aligned to 4 bytes boundary.  
  */
  extern unsigned long __data_start__[];
  extern unsigned long __data_end__[];
  extern unsigned long __etext[];
  ptr = __etext;
  start = __data_start__;
  end = __data_end__;
  while( start < end )
  {
    *start = *ptr;
    start++;
    ptr++;
  }
  
  /*
  Loop to zero out BSS section, which uses following symbols
  in linker script:
  __bss_start__: start of BSS section. Must align to 4
  __bss_end__: end of BSS section. Must align to 4
  */
  extern unsigned long __bss_start__[];
  extern unsigned long __bss_end__[];
  ptr = __bss_start__;
  end = __bss_end__;
  while( ptr < end )
  {
    *ptr = 0;
    ptr++;
  }

  /* Call main procedure */  
  main();
  
  /* finished, do nothing. */
  for(;;)
    ;
}

/* the default handler is only referenced via alias attribute */
void __attribute__ ((interrupt)) Default_Handler(void)
{
}

/* "NMI_Handler" is used in the ld script to calculate the checksum */
/* This is a weak function and can be overwritten by the user */
void __attribute__ ((interrupt, weak)) NMI_Handler(void)
{
}

/* "HardFault_Handler" is used in the ld script to calculate the checksum */
/* This is a weak function and can be overwritten by the user */
void __attribute__ ((interrupt, weak)) HardFault_Handler(void)
{
}

void __attribute__ ((interrupt, weak, alias("Default_Handler"))) SysTick_Handler(void);
void __attribute__ ((interrupt, weak, alias("Default_Handler"))) SVC_Handler(void);
void __attribute__ ((interrupt, weak, alias("Default_Handler"))) PendSV_Handler(void);

void __attribute__ ((interrupt, weak, alias("Default_Handler"))) SPI0_Handler(void);
void __attribute__ ((interrupt, weak, alias("Default_Handler"))) DAC0_Handler(void);
void __attribute__ ((interrupt, weak, alias("Default_Handler"))) UART0_Handler(void);
void __attribute__ ((interrupt, weak, alias("Default_Handler"))) UART1_Handler(void);
void __attribute__ ((interrupt, weak, alias("Default_Handler"))) I2C1_Handler(void);
void __attribute__ ((interrupt, weak, alias("Default_Handler"))) I2C0_Handler(void);

void __attribute__ ((interrupt, weak, alias("Default_Handler"))) MRT_Handler(void);
void __attribute__ ((interrupt, weak, alias("Default_Handler"))) CMP_Handler(void);
void __attribute__ ((interrupt, weak, alias("Default_Handler"))) WDT_Handler(void);
void __attribute__ ((interrupt, weak, alias("Default_Handler"))) BOD_Handler(void);

void __attribute__ ((interrupt, weak, alias("Default_Handler"))) WKT_Handler(void);
void __attribute__ ((interrupt, weak, alias("Default_Handler"))) ADC_SEQA_Handler(void);
void __attribute__ ((interrupt, weak, alias("Default_Handler"))) ADC_SEQB_Handler(void);
void __attribute__ ((interrupt, weak, alias("Default_Handler"))) ADC_THCMP_Handler(void);
void __attribute__ ((interrupt, weak, alias("Default_Handler"))) ADC_OVR_Handler(void);

void __attribute__ ((interrupt, weak, alias("Default_Handler"))) CTIMER0_Handler(void);

void __attribute__ ((interrupt, weak, alias("Default_Handler"))) PININT0_Handler(void);
void __attribute__ ((interrupt, weak, alias("Default_Handler"))) PININT1_Handler(void);
void __attribute__ ((interrupt, weak, alias("Default_Handler"))) PININT2_Handler(void);
void __attribute__ ((interrupt, weak, alias("Default_Handler"))) PININT3_Handler(void);
void __attribute__ ((interrupt, weak, alias("Default_Handler"))) PININT4_Handler(void);
void __attribute__ ((interrupt, weak, alias("Default_Handler"))) PININT5_Handler(void);
void __attribute__ ((interrupt, weak, alias("Default_Handler"))) PININT6_Handler(void);
void __attribute__ ((interrupt, weak, alias("Default_Handler"))) PININT7_Handler(void);

/* make the checksum known to the c compiler, value will be calculated by the linker script */
void LPC_checksum(void);

/*=======================================================================*/
/* isr vector */
/* based on LPC8xx.h */
/* see table 38 of the LPC804 user manual (page 39) */ 

typedef void (*isr_handler_t)(void);
isr_handler_t __isr_vector[48] __attribute__ ((section(".isr_vector"))) __attribute__ ((aligned(4)))= 
{
  __StackTop,			/* 0x00: Top of Stack, calculated by the linker script */
  Reset_Handler,		/* -15, 0x04: Reset Handler, DO NOT CHANGE THE ISR NAME (used for LPC_checksum calculation) */
  NMI_Handler,			/* .14, 0x08: NMI Handler, DO NOT CHANGE THE ISR NAME (used for LPC_checksum calculation) */
  HardFault_Handler,         /* -13, 0x0c: Hard Fault Handler, DO NOT CHANGE THE ISR NAME (used for LPC_checksum calculation) */
  0,					/* 0x10: Reserved, must be 0 */
  0,					/* 0x14: Reserved, must be 0 */
  0,					/* 0x18: Reserved, must be 0 */
  LPC_checksum,		/* 0x1c: Checksum, calculated by the linker script or the flash utility */
  0,                                /* Reserved */
  0,                                /* Reserved */
  0,                                /* Reserved */
  SVC_Handler,               /* -5, SVCall Handler */
  0,                                /* -4, Reserved */
  0,                                /* -3, Reserved */
  PendSV_Handler,         /* -2, PendSV Handler */
  SysTick_Handler,         /* -1, SysTick Handler */            


  SPI0_Handler,  		/* 0 SPI0_IRQn, SPI0 controller */
  0, 					/* 1 Not used/Reserved */
  DAC0_Handler,		/* 2 DAC0_IRQn, DAC0 Interrupt */
  UART0_Handler,		/* 3 UART0_IRQn, USART0 */
  UART1_Handler,		/* 4 UART1_IRQn, USART1 */
  0, 					/* 5 Not used */
  0,					/* 6 Not used */
  I2C1_Handler,		/* 7 I2C1 controller */
  I2C0_Handler,		/* 8 I2C0 controller */
  0,					/* 9 Not used/Reserved */
  MRT_Handler,			/* 10 MRT_IRQnMulti-Rate Timer */
  CMP_Handler,			/* 11 CMP_IRQn, Analog Comparator /CapTouch*/
  WDT_Handler,		/* 12 WDT_IRQn WDT */
  BOD_Handler,			/* 13 BOD_IRQn BOD Brown Out Detect */
  0,					/* 14 FLASH_IRQn (/Reserved according to the user manual) */
  WKT_Handler,			/* 15 WKT_IRQn Self wake-up timer */
  ADC_SEQA_Handler,	/* 16 ADC_SEQA_IRQn */
  ADC_SEQB_Handler,	/* 17 ADC_SEQB_IRQn */
  ADC_THCMP_Handler,	/* 18 ADC_THCMP_IRQn ADC threshold compare */
  ADC_OVR_Handler,		/* 19 ADC_OVR_IRQn ADC overrun  */
  0,					/* 20 Not used/Reserved */
  0,					/* 21 Not used/Reserved */
  0,					/* 22 Not used/Reserved */
  CTIMER0_Handler,		/* 23 CTIMER0_IRQn, CT32B0_IRQ */
  PININT0_Handler,		/* 24 PIO INT0, PININT0 */
  PININT1_Handler,		/* 25 PIO INT1, PININT1 */
  PININT2_Handler,		/* 26 PIO INT2, PININT2 */
  PININT3_Handler,		/* 27 PIO INT3, PININT3 */
  PININT4_Handler,		/* 28 PIO INT4, PININT4 */
  PININT5_Handler,		/* 29 PIO INT5, PININT5 */
  PININT6_Handler,		/* 30 PIO INT6, PININT6 */
  PININT7_Handler		/* 31 PIO INT7, PININT7 */
};

