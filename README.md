# PSP-SIO-port-IMU

## RndMnkIII 2018
Example using MPU-6050 sensor as IMU thanks to the Kalman filter library
by Kristian Lauszus, to convert the raw data from the sensor to float values
representing the x and y axis rotation values. This values are converted to a
byte array and transfered to the PSP SIO port using the Teensy 3.2 UART2 at 9600bps.
The UART2 port is connected to the PSP console SIO port using a bidirectional
voltage level converter between for converting from 3.3v Teensy 3.2 logic level
to the PSP SIO 2.5v logic level using a sparkfun logic level shifter.
(https://www.sparkfun.com/products/12009).

![Schematic](https://github.com/RndMnkIII/PSP-SIO-port-IMU/blob/master/diagram.png)

This repository contains two project folders:
- Arduino sketch with the code for the Teensy 3.2 board. Install the Kalman library
  from https://github.com/TKJElectronics/KalmanFilter in the Arduino\libraries folder.
  
- Gcc Make project for the PSP DevKit cross compiler that generates the EBOOT.PBP file 
  for copy to the PSP memory stick PSP\GAME150\celshading folder with the lightmap.raw
  resource file.
