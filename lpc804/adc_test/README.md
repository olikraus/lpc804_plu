This function will test the "adc_init" and "adc_read" procedures.
See "common/util.h" for details regarding "adc_init" and "adc_read".

This example will output the ADC values via serial interface connected to 
GPIO 0_4 (tx) and 0_0 (rx). If the serial data is received by a USB converter,
then the datastream from the controller can be listed with "minicom -D /dev/ttyUSB0".

The toolchain for this example project is described
[here](https://drolliblog.wordpress.com/2019/10/04/nxp-lpc804-toolchain-and-blink-project/).