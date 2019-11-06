# CTIMER Interrupt Example

 * Description: [https://drolliblog.wordpress.com/2019/10/05/nxp-lpc804-ctimer-and-lvlshft/](https://drolliblog.wordpress.com/2019/10/05/nxp-lpc804-ctimer-and-lvlshft/)
 * USB UART Converter is connected to pins 6 and 19 of the LPC804
 * A switch at pin 4 selects between in-system programming and user code mode
 * A button at pin 5 will reset the LPC804 and either start the ISP bootloader or the user code (depending on the state at pin 4)
 * Code was takenover from "ctimer" example, so it will use the LED at pin PIO0_15
 * Additionally LED at pin PIO0_9 will be toggled by an interrupt procedure.
 
The following steps are required for interrupts:
 1. The handler has to be defined in the code. The handler name is predefined in startup.c: `CTIMER0_Handler()`
 2. The interrupt must be enabled in CTIMER MCR register
 3. The CTIMER NVIC IRQ has to be enabled with `NVIC_EnableIRQ(CTIMER0_IRQn)`
 4. Once the interrupt has happend, it must be cleared in CTIMER IR register by setting the corresponding bit. 
 

Result:
 * LED at PIO0_15 will flash (due to the PWM configuration)
 * LED at PIO0_9 will toggle in sync with the flashing PIO0_15 LED (due to the ctimer interrupt)

![lpc804_blink_schematic.png](../blink/lpc804_blink_schematic.png)

![lpc804_ctimer.png](lpc804_ctimer.png)
