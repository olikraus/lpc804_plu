# /lpc_chip_804

LPC804 system files from LPC804-EX-CODE-MCUXPRESSO.zip from nxp.com
available from here: https://www.nxp.com/products/processors-and-microcontrollers/arm-microcontrollers/general-purpose-mcus/lpc800-cortex-m0-plus-/low-cost-microcontrollers-mcus-based-on-arm-cortex-m0-plus-core:LPC80X?tab=Design_Tools_Tab#t749
Download "LPC804 Example Code Bundle MCUXpresso"

Modifications (i2c.h syscon.h syscon.c i2c.c spi.c spi.h plu.c):
 - Removed includes of "utilities.h" because the funcionality is not required
 - added inclusion of "swm.h" from "spi.h", because "spi.h" makes use of 
    some definitions from "swm.h"
 - Removed "Config_Syspll" declaration from "syscon.h" because it is removed in the src.
 - Alligned "unsigned int" with "uint32_t" declarations in syscon.h and .c 
 - Change the inclusion of "lpc8xx.h" to "LPC8xx.h" in i2c.h
   
# /common

Common (system) files for all/most of the example files. 

This includes the linker script (lpc804.ld) and the startup code (startup.c)
