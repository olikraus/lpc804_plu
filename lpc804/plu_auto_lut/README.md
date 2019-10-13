# PLU Code Generation 

 * Post: [https://drolliblog.wordpress.com/2019/10/05/nxp-lpc804-plu-lut-configuration/](https://drolliblog.wordpress.com/2019/10/05/nxp-lpc804-plu-lut-configuration/)
 * USB UART Converter is connected to pins 6 and 19 of the LPC804
 * This example will use the LED at pin PIO0_15
 * Assums buttons at PIO0_2 and PIO0_10
 * Implements Boolean AND between PIO0_2 and PIO0_10, output at PIO0_15
 * Code generation: `../../tools/pluc/pluc 'LVLSHFT_IN0 <= PIO0_10; PIO0_22 <= LVLSHFT_OUT0; PIO0_15 <= PIO0_2 & PIO0_22;' -oc plu.c -fn plu`

![lpc804_blink_schematic.png](../blink/lpc804_blink_schematic.png)
