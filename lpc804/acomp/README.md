# Compare 0.9V bandgap against ladder value

 * USB UART Converter is connected to pins 6 and 19 of the LPC804
 * LED at pin PIO0_15: Comperator output status
 * LED at pin PIO0_9: Brightness indicates the current ladder value 
 * This example will cycle through all 32 ladder values and output the current ladder value as bightness to PIO0_9 LED
 * If the current ladder output voltage is higher than the 0.9V bandgap, then the LED at PIO0_15 will be on
 * The toolchain for this example project is described
[here](https://drolliblog.wordpress.com/2019/10/04/nxp-lpc804-toolchain-and-blink-project/).

![lpc804_blink_schematic.png](../blink/lpc804_blink_schematic.png)

