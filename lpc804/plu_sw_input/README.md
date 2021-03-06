# Changing the input value to internal functions via software

 * Internal functions can be connected to GPIOs only
 * As soon as an internal *input* function is connected to a GPIO pin, then the
   GPIO pin is configured input.
 * Because the GPIO port status can be changed only for outputs and because
   the GPIO is automatically forced to be input (see previos point): Writing a GPIO
   input value to an internal function will not work.
 * The only way to change the input value to an internal function connected to
   a GPIO port is changing the pullup/donw and/or by using the inverter function.
   
The example in this folder will demonstrate this:

 * PIO0_2 output is connected to LVLSHFT0 input
 * LVLSHFT0 output is connected to PIO0_10
 * PIO0_10 output is connected to LUT0 input
 * LUT0 output is connected to PIO0_15 (LED)
 * The software can influence the value of the LED only by the following methods:
   * changing the pullup/down at PIO0_2
   * changing the inverter for PIO0_2
   * changing the inverter for PIO0_10
   
Use case: A PLU logic block requires input from the microcontroller:
 1. Connect the PLU Input to a GPIO port
 2. Configure pullup for this GPIO port (direction does not matter, it is forced to input by the PLU Input assginment)
 3. Modify the inverter function status instead of the GPIO value:

| Input | Inverter | PLU Input |
|---|---|---|
| Pull-Up | off | Logical One |
| Pull-Up | on | Logical Zero |
| Pull-Down | off | Logical Zero |
| Pull-Down | on | Logical One |




![lpc804_blink_schematic.png](../blink/lpc804_blink_schematic.png)
