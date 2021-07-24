### Display audio FFT, temperature and pressure

Application to operate displaying Audio FFT, Temperatur and Pressure.

## Components

- STM32 Nucleo-F411RE high performance,
- OLED SSD1306 display, 
- BMP280 temperature and pressure sensor,
- microphone.

## I2C

Pins PB8 and PB9 belong to I2C.

We don't need to enable pull-up because 10 kOm on the BMP280 board is enough.

We don't need a translator because all devices are 3.3V.

We move the address to the left so that we have the RW direction bit at the end. HAL accepts 8-bit values. The address is 0-127. That's why you make a define with the address and only in the program move it from one to the left.

Use the I2C scanner to read the address and check the documentation.

## Functions

### BMP280

Read8 - reads the ChipID register,
Read16 - reads 16-bit calibration registers to correct temperature and pressure. These registers are fixed for one sensor. 
Write8 - writes a value to the configure register ctrl_meas. First we have to read this register and swap mode bits, so we don't swap other bits.  On the other hand, the measurement accuracy orsr_t and osrs_o, on 3 bits.
Read32 - read temp_xlsb etc registers which are 24-bit (3-bytes). We read the raw temp and press data.
ReadPressureandTemperature - reads the temperature to then compensate for it and use the converted value in the pressure value calculation. The conversion can be found in the BMP280 sensor documentation.

### OLED

Porting library (Arduino): https://github.com/adafruit/Adafruit_SSD1306

SSD1306_Clear - writes data from the buffer to erase the OLED RAM.
GFX_DrawString - inserts the message for which we are passing space.
SSD_Display - displays data from the buffer

## DMA

Change to fast mode. Sending data to the buffer does not load the uC.

When using more devices we need to check if I2C is free and the same if DMA is free or busy. We have to select an interrupt to make it free. 

BMP refreshed every 100 times per second and OLED refreshed 10 times per second.



