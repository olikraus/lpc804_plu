/* LPC804 PLU setup file, generated by 'pluc' */
#include <stdint.h>

void plu(void)
{
	*(uint32_t *)0x40048084UL |= (1UL<<5);   /* Enable PLU Clock */
	*(uint32_t *)0x4004808CUL &= ~(1UL<<5);   /* Reset PLU */
	*(uint32_t *)0x4004808CUL |= (1UL<<5);   /* Clear Reset PLU */
	*(uint32_t *)0x40048080UL |= (1UL<<6);   /* Enable GPIO0 Clock */
	*(uint32_t *)0x40048088UL &= ~(1UL<<6);   /* Reset GPIO0 */
	*(uint32_t *)0x40048088UL |= (1UL<<6);   /* Clear Reset GPIO0 */
	*(uint32_t *)0x40048080UL |= (1UL<<7);   /* Enable SWM Clock */
	*(uint32_t *)0x40048088UL &= ~(1UL<<7);   /* Reset SWM */
	*(uint32_t *)0x40048088UL |= (1UL<<7);   /* Clear Reset SWM */
	*(uint32_t *)0x40048080UL |= (1UL<<18);   /* Enable IOCON Clock */
	*(uint32_t *)0x40048088UL &= ~(1UL<<18);   /* Reset IOCON */
	*(uint32_t *)0x40048088UL |= (1UL<<18);   /* Clear Reset IOCON */
	*(uint32_t *)0xA0002380UL |= (1UL<<8);    /* PIO0_8 GPIO DIRSETP: Setup as output */
	*(uint32_t *)0x40044038UL &= ~(3UL<<3);    /* PIO0_8 IOCON: Clear mode, deactivate any pull-up/pulldown */
	*(uint32_t *)0x40044034UL &= ~(3UL<<3);    /* PIO0_9 IOCON: Clear mode */
	*(uint32_t *)0x40044034UL |= (2UL<<3);    /* PIO0_9 IOCON: Enable pull-up */
	*(uint32_t *)0xA0002400UL |= (1UL<<9);    /* PIO0_9 GPIO DIRCLRP: Setup as input */
	*(uint32_t *)0x4004401cUL &= ~(3UL<<3);    /* PIO0_11 IOCON: Clear mode */
	*(uint32_t *)0x4004401cUL |= (2UL<<3);    /* PIO0_11 IOCON: Enable pull-up */
	*(uint32_t *)0xA0002400UL |= (1UL<<11);    /* PIO0_11 GPIO DIRCLRP: Setup as input */
	*(uint32_t *)0x4004405cUL &= ~(3UL<<3);    /* PIO0_30 IOCON: Clear mode */
	*(uint32_t *)0x4004405cUL |= (2UL<<3);    /* PIO0_30 IOCON: Enable pull-up */
	*(uint32_t *)0xA0002400UL |= (1UL<<30);    /* PIO0_30 GPIO DIRCLRP: Setup as input */
	/* LUT 0 Config Value */
	/* PIO0_9 FF0 FF1 FF2 FF3|FF0 */
	/* -001-|1 */
	/* -00-1|1 */
	/* -0100|1 */
	/* 100--|1 */
	*(uint32_t *)0x40028800 = 0x03030332UL; /*PLU*/
	/* LUT 1 Config Value */
	/* FF0 LUT5 LUT6|FF1 */
	/* 11-|1 */
	/* 0-1|1 */
	*(uint32_t *)0x40028804 = 0x000000d8UL; /*PLU*/
	/* LUT 2 Config Value */
	/* FF1 LUT7 LUT8|FF2 */
	/* 11-|1 */
	/* 0-1|1 */
	*(uint32_t *)0x40028808 = 0x000000d8UL; /*PLU*/
	/* LUT 3 Config Value */
	/* FF0 FF1 FF2 FF3|FF3 */
	/* -0-1|1 */
	/* 111-|1 */
	*(uint32_t *)0x4002880c = 0x0000b380UL; /*PLU*/
	/* LUT 4 Config Value */
	/* FF0 FF1 FF2 FF3|PIO0_8 */
	/* -01-|1 */
	/* -0-1|1 */
	/* 111-|1 */
	/* 10--|1 */
	/* 110-|1 */
	/* 0100|1 */
	*(uint32_t *)0x40028810 = 0x0000bbbeUL; /*PLU*/
	/* LUT 5 Config Value */
	/* FF1|LUT5 */
	/* 0|1 */
	*(uint32_t *)0x40028814 = 0x00000001UL; /*PLU*/
	/* LUT 6 Config Value */
	/* PIO0_9 PIO0_11 FF1 FF2 FF3|LUT6 */
	/* --100|1 */
	/* 10000|1 */
	*(uint32_t *)0x40028818 = 0x000000f2UL; /*PLU*/
	/* LUT 7 Config Value */
	/* FF0 FF2|LUT7 */
	/* 10|1 */
	*(uint32_t *)0x4002881c = 0x00000002UL; /*PLU*/
	/* LUT 8 Config Value */
	/* PIO0_9 PIO0_11 FF0 FF2 FF3|LUT8 */
	/* ---1-|1 */
	/* 10000|1 */
	*(uint32_t *)0x40028820 = 0xff00ff02UL; /*PLU*/
	/* PIO0_9 --> PLUINPUT1 */
	*(uint32_t *)0x4000c180 &= ~0x0000000cUL; *(uint32_t *)0x4000c180 |= 0x00000004UL; /*SWM*/
	/* PIO0_11 --> PLUINPUT3 */
	*(uint32_t *)0x4000c180 &= ~0x000000c0UL; *(uint32_t *)0x4000c180 |= 0x00000040UL; /*SWM*/
	/* PLUOUT1 --> PIO0_8 */
	*(uint32_t *)0x4000c180 &= ~0x0000c000UL; *(uint32_t *)0x4000c180 |= 0x00000000UL; /*SWM*/
	/* PLUINPUT1 --> LUT0_INP0 */
	*(uint32_t *)0x40028000 = 0x00000001UL; /*PLU*/
	/* FF0 --> LUT0_INP1 */
	*(uint32_t *)0x40028004 = 0x00000020UL; /*PLU*/
	/* FF1 --> LUT0_INP2 */
	*(uint32_t *)0x40028008 = 0x00000021UL; /*PLU*/
	/* FF2 --> LUT0_INP3 */
	*(uint32_t *)0x4002800c = 0x00000022UL; /*PLU*/
	/* FF3 --> LUT0_INP4 */
	*(uint32_t *)0x40028010 = 0x00000023UL; /*PLU*/
	/* FF0 --> LUT1_INP0 */
	*(uint32_t *)0x40028020 = 0x00000020UL; /*PLU*/
	/* LUT5 --> LUT1_INP1 */
	*(uint32_t *)0x40028024 = 0x0000000bUL; /*PLU*/
	/* LUT6 --> LUT1_INP2 */
	*(uint32_t *)0x40028028 = 0x0000000cUL; /*PLU*/
	/* FF1 --> LUT2_INP0 */
	*(uint32_t *)0x40028040 = 0x00000021UL; /*PLU*/
	/* LUT7 --> LUT2_INP1 */
	*(uint32_t *)0x40028044 = 0x0000000dUL; /*PLU*/
	/* LUT8 --> LUT2_INP2 */
	*(uint32_t *)0x40028048 = 0x0000000eUL; /*PLU*/
	/* FF0 --> LUT3_INP0 */
	*(uint32_t *)0x40028060 = 0x00000020UL; /*PLU*/
	/* FF1 --> LUT3_INP1 */
	*(uint32_t *)0x40028064 = 0x00000021UL; /*PLU*/
	/* FF2 --> LUT3_INP2 */
	*(uint32_t *)0x40028068 = 0x00000022UL; /*PLU*/
	/* FF3 --> LUT3_INP3 */
	*(uint32_t *)0x4002806c = 0x00000023UL; /*PLU*/
	/* FF0 --> LUT4_INP0 */
	*(uint32_t *)0x40028080 = 0x00000020UL; /*PLU*/
	/* FF1 --> LUT4_INP1 */
	*(uint32_t *)0x40028084 = 0x00000021UL; /*PLU*/
	/* FF2 --> LUT4_INP2 */
	*(uint32_t *)0x40028088 = 0x00000022UL; /*PLU*/
	/* FF3 --> LUT4_INP3 */
	*(uint32_t *)0x4002808c = 0x00000023UL; /*PLU*/
	/* LUT4 --> PLUOUT1 */
	*(uint32_t *)0x40028c04 = 0x00000004UL; /*PLU*/
	/* FF1 --> LUT5_INP0 */
	*(uint32_t *)0x400280a0 = 0x00000021UL; /*PLU*/
	/* PLUINPUT1 --> LUT6_INP0 */
	*(uint32_t *)0x400280c0 = 0x00000001UL; /*PLU*/
	/* PLUINPUT3 --> LUT6_INP1 */
	*(uint32_t *)0x400280c4 = 0x00000003UL; /*PLU*/
	/* FF1 --> LUT6_INP2 */
	*(uint32_t *)0x400280c8 = 0x00000021UL; /*PLU*/
	/* FF2 --> LUT6_INP3 */
	*(uint32_t *)0x400280cc = 0x00000022UL; /*PLU*/
	/* FF3 --> LUT6_INP4 */
	*(uint32_t *)0x400280d0 = 0x00000023UL; /*PLU*/
	/* FF0 --> LUT7_INP0 */
	*(uint32_t *)0x400280e0 = 0x00000020UL; /*PLU*/
	/* FF2 --> LUT7_INP1 */
	*(uint32_t *)0x400280e4 = 0x00000022UL; /*PLU*/
	/* PLUINPUT1 --> LUT8_INP0 */
	*(uint32_t *)0x40028100 = 0x00000001UL; /*PLU*/
	/* PLUINPUT3 --> LUT8_INP1 */
	*(uint32_t *)0x40028104 = 0x00000003UL; /*PLU*/
	/* FF0 --> LUT8_INP2 */
	*(uint32_t *)0x40028108 = 0x00000020UL; /*PLU*/
	/* FF2 --> LUT8_INP3 */
	*(uint32_t *)0x4002810c = 0x00000022UL; /*PLU*/
	/* FF3 --> LUT8_INP4 */
	*(uint32_t *)0x40028110 = 0x00000023UL; /*PLU*/
	/* LUT0 --> FF0 */
	/* LUT1 --> FF1 */
	/* LUT2 --> FF2 */
	/* LUT3 --> FF3 */
	/* PIO0_30 --> PLU_CLKIN */
	*(uint32_t *)0x4000c01c &= ~0xff000000UL; *(uint32_t *)0x4000c01c |= 0x1e000000UL; /*SWM*/
	/* CLKOUT --> PIO0_30 */
	*(uint32_t *)0x4000c014 &= ~0xff000000UL; *(uint32_t *)0x4000c014 |= 0x1e000000UL; /*SWM*/
	/* CLKOUT clock source select register: Select main clock */
	*(uint32_t *)0x400480F0UL = 1;  /*SYSCON CLKOUTSEL*/
	/* CLKOUT clock divider register: Divide by 2 */
	*(uint32_t *)0x400480F4UL = 2;  /*SYSCON CLKOUTDIV*/
}
