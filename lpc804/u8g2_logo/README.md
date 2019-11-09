# SSD1306 demo with u8g2 library (software emulated I2C)

 * u8g2 library location: "git clone" for u8g2 and lpc804_plu should be on the same directory folder
 * USB UART Converter is connected to pins 6 and 19 of the LPC804
 * A switch at pin 4 selects between in-system programming and user code mode
 * A button at pin 5 will reset the LPC804 and either start the ISP bootloader or the user code (depending on the state at pin 4)
 * SSD1306 I2C clock is connected to pin 7
 * SSD1306 I2C data is connected to pin 14
 * I2C clock and data lines are pulled up with a 10k resistor to 3.3V
 * The OLED will just show the usual u8g2 logo
 


