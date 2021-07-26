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

- Read8() - reads the ChipID register,
- Read16() - reads 16-bit calibration registers to correct temperature and pressure. These registers are fixed for one sensor. 
- Write8() - writes a value to the configure register ctrl_meas. First we have to read this register and swap mode bits, so we don't swap other bits.  On the other hand, the measurement accuracy orsr_t and osrs_o, on 3 bits.
- Read32() - read temp_xlsb etc registers which are 24-bit (3-bytes). We read the raw temp and press data.
- ReadPressureandTemperature() - reads the temperature to then compensate for it and use the converted value in the pressure value calculation. The conversion can be found in the BMP280 sensor documentation.

### OLED

Porting library (Arduino): https://github.com/adafruit/Adafruit_SSD1306

- SSD1306_Clear() - writes data from the buffer to erase the OLED RAM.
- GFX_DrawString() - inserts the message for which we are passing space.
- SSD_Display() - displays data from the buffer

## DMA

Change to fast mode. Sending data to the buffer does not load the uC.

When using more devices we need to check if I2C is free and the same if DMA is free or busy. We have to select an interrupt to make it free. 

BMP refreshed every 100 times per second and OLED refreshed 10 times per second.

## Components of FreeRTOS

### Tasks:

- HeartBeatTask
- Bmp280Task
- OledTask

#### Mutex:
- MutexPrint - includes its own printf library dedicated to embedded,
- MutexI2C1 - wherever I2C is used, access for the shuffle is blocked,
- MutexBmpData - when reading from one file and displaying global variables from another,

### Queue:

- QueueBmpData - global variable replaced with FreeRTOS variable in the form of a queue,

We send to OLED 10 times less frequently than we read from BMP. We can't send all the measurements to the queue because it will get clogged quickly.

### Timers:

- TimerBmpData - Instead of reading every 10 values, we use a software timer from the RTOS, which every 100 ms signals with a semaphore that it's time to put something into the queue. 

### Semaphores:

- SemaphoreBmpQueue

Shred the OLED communication into 8 parts. Then after one part is executed a mutex is given away. The yeld function, on the other hand, forces a context change so the BMP280 sensor can read something. 

We send a TaskNotification flag to the task computing the FFT algorithm that it can already perform the computation, in infinite wait mode it starts to compute the FFT. On OLED we send the same way as BMP, that is through the queue. We place the FFT buffers on the FreeRTOS heap using dynamic memory allocation. 

Tip: In particular, be mindful of task priorities and stack size!

![image](https://user-images.githubusercontent.com/37025393/127037106-6e92771e-2723-4a26-83c7-1a7f3cefc77c.png)

