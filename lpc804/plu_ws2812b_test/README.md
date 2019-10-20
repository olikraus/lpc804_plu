# Test for the WS2812b protocol

 * USB UART Converter is connected to pins 6 and 19 of the LPC804
 * This example will use the LED at pin PIO0_15 (data out) and PIO0_9 (active)
 * Assums buttons PIO0_10 (enable) and PIO0_2 (data in)
 * PIO0_2 selects bit 0 or 1, which will be visible in a more or less brighter LED at PIO0_15
 * Pressing PIO0_10 will disable the FSM, both LEDs will be off (FSM in wait state)
 
![lpc804_blink_schematic.png](../blink/lpc804_blink_schematic.png)
