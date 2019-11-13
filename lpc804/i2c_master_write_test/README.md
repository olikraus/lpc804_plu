# LPC804 and I2C 

Before using any of the I2C function:

 * Open dran mode needs to be activated on the GPIOs (https://community.nxp.com/message/1231588)
 * Connection between I2C subsystem and GPIO via SWM has to be established first
 
I2C Subsystem:

 * `i2c_write()` will probably only return `I2C_NO_TX_POST_ADR` and `I2C_TIMEOUT_POST_ADR`
 * Assuming that MSTSCLHIGH and MSTSCLLOW are kept as 0, thent he clkdiv value 
can have the following values (`i2c_init()`):


|  input clk |	clkdiv |	Nominal I2C Bus Clock |
|---|---|---|
|  15 Mhz |		37 |		99 KHz |
|  15 Mhz |		9 |		375 KHz |  
|  12 Mhz |		29 |		100 KHz |
|  12 Mhz |		7 |		375 KHz |

 