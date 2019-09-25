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
 - pluc basically does not require any knowledge of the PLU details, just specify the problem. "pluc" will generate an error if the result does not fit into the PLU.
 - pluc is available for linux and windows
 - pluc does not support high level languages like VHDL or Verilog. However the asumption is, that PLA, BEX, KISS or BMS formats are powerful enough to describe problems for the LPC804 PLU.
 - pluc does not contain a graphical user interface or schematic editor
 - Algorithms inside pluc (dgc) and NXP "PLU configuration tool" (SIS) are both based on the research happend between 1980 and 1990.
 - A good summary of the technical background for pluc is here: https://www2.eecs.berkeley.edu/Pubs/TechRpts/1986/734.html
 
 