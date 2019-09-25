# lpc804_plu

This repo contains:
 * Toolchain for the NXP LPC804 Microcontroller
 * Synthesis tool for the Programmable Logic Unit (PLU Compiler)
 * UART Upload tool
 * LPC804 examples, including PLU examples
 

## pluc

"pluc" is the PLU Compiler tool, developed for this project. 
It is derived from my older project DGC and is based as much as possible 
on the DGC sources.

"pluc" is a synthesis compiler, which creates a C-function. This C-function
includes all the setup code required for the LPC804 PLU.

"pluc" accepts boolean expressions (PLA, BEX) and state machines (KISS, BMS) as
input. 
 
"pluc" vs NXP "PLU configuration tool":
 - pluc is just a commandline tool, it reads a specification and writes C code
 - pluc basically does not require any knowledge of the PLU details, just specify the problem. "pluc" will generate an error if it does not fit into the PLU.
 - pluc is available for linux and windows
