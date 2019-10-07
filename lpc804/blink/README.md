# Simple Blink Example for the LPC804

 * Post: [https://drolliblog.wordpress.com/2019/10/04/nxp-lpc804-toolchain-and-blink-project/](https://drolliblog.wordpress.com/2019/10/04/nxp-lpc804-toolchain-and-blink-project/)
 * USB UART Converter is connected to pins 6 and 19 of the LPC804
 * A switch at pin 4 selects between in-system programming and user code mode
 * A button at pin 5 will reset the LPC804 and either start the ISP bootloader or the user code (depending on the state at pin 4)
 * This example will use the LED at pin 15
 * See also my [post](https://drolliblog.wordpress.com/2019/10/04/nxp-lpc804-toolchain-and-blink-project/) about this project.



![lpc804_blink_schematic.png](lpc804_blink_schematic.png)

![lpc804_blink_hardware.jpg](lpc804_blink_hardware.jpg)
