echo off
rem make.bat
rem
rem Create and upload hex file for LPC804 on windows operating system.
rem Requires https://launchpad.net/gcc-arm-embedded/ "Win32 Installer"
rem
set TARGET=plu_sw_input
rem Change the following path if required:
set GNUARMPATH="C:\Program Files (x86)\GNU Tools ARM Embedded\5.4 2016q2"

set GCC=%GNUARMPATH%\bin\arm-none-eabi-gcc.exe
set OBJCOPY=%GNUARMPATH%\bin\arm-none-eabi-objcopy.exe
set CCFLAGS=-mthumb -mcpu=cortex-m0plus -Wall -I. -I..\lpc_chip_804 -I..\common  -Os  -ffunction-sections -fdata-sections -std=gnu99
set LDFLAGS=-Wl,--gc-sections  -Wl,--undefined=arm_stack_area -Wl,--undefined=__isr_vector -Wl,-Map=%TARGET%.map --specs=nosys.specs -L..\common -T lpc804.ld
set SRC=
for %%f in (..\lpc_chip_804\*.c) do call set SRC=%%SRC%% %%f
for %%f in (..\common\*.c) do call set SRC=%%SRC%% %%f
for %%f in (*.c) do call set SRC=%%SRC%% %%f
echo on
%GCC% %CCFLAGS%%SRC% %LDFLAGS% -o %TARGET%.elf
%OBJCOPY% -O ihex  %TARGET%.elf %TARGET%.hex
..\..\tools\lpc21isp\lpc21isp.win.x86-64 -verify %TARGET%.hex COM3 115200 14746