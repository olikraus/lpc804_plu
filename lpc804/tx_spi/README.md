# Transmit SPI Test

 * USB UART Converter is connected to pins 6 and 19 of the LPC804
 * A switch at pin 4 selects between in-system programming and user code mode
 * A button at pin 5 will reset the LPC804 and either start the ISP bootloader or the user code (depending on the state at pin 4)
 * LED at pin PIO0_15: SPI Clock
 * LED at pin PIO0_9: SPI MOSI
 * Button at PIO0_2: Put test data (0x99) into TXDAT register
 * SPI clock speed is below 1Hz, so that the data (0x99) can be observed on the LEDs


![lpc804_blink_schematic.png](../blink/lpc804_blink_schematic.png)

