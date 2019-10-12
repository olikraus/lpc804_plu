# Muller-C State Machine Example

 * Post: [https://drolliblog.wordpress.com/2019/10/12/state-machine-implementation-for-the-lpc804-plu/](https://drolliblog.wordpress.com/2019/10/12/state-machine-implementation-for-the-lpc804-plu/)
 * USB UART Converter is connected to pins 6 and 19 of the LPC804
 * This example will use the LED at pin PIO0_15
 * Assums buttons at PIO0_2 and PIO0_10
 * Implements Muller-C state machine, inputs "x" (PIO0_2) and "y" (PIO0_10), output "z" (PIO0_15)
 * The clock signal for the PLU state machine is derived from the system clock (availanle via CLKOUT)
 * Remember, that the buttons are low active!

![muller_c_bms.png](muller_c_bms.png)

![muller_c_bex_bms.png](muller_c_bex_bms.png)
