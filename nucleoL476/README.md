# NUCLEOL476 - GPS - IMU

The function is build around a evalboard Nucleo-L476 from ST. Firmware code can be found in firmware directory\
The NucleoL476 board has two features:
* Send raw data from the IMU to the Raspberry via the CAN bus (magnetic, angular rate, linear acceleration, temperature, pressure, humidity)
* Send GPS data to the Raspberry via the CAN bus (latitude, longitude, altitude)

## How to program the board 

Like the NucleoF103 board, you can program the board with the STM32CubeIDE software, by usb connection. To power the board through the usb cable, you have to set the jumper JP5 to U5V. After the programming, don't forget to set the jumper JP5 on E5V so that the card is powered by the general power supply of the car.

## Firmware

Firmware code is split between drivers parts (in Drivers/BSP directory) and application (in Core directory). Every driver is initialized in main function (I/O, Can, Interruptions, memory); Dynamic part of the application can be found in scheduler.c. Scheduler is cooperative and launch periodic tasks, roughtly one for each sensor (one for GPS, one for IMU ...). Shared variables (like latest values from sensors) can be found in globvar.c

## IMU Sensors [X-Nucleo-IKS01A2]

All sensors are accessed via I2C. Available sensors are 
- 3-axis accelerometer (values given in milli g (mg))
- 3-axis gyroscope (values given in milli degree per seconde (mdps))
- 3-axis magnetometer (values given in milli Gauss (mG))
- humidity sensor (values given in percent)
- pressure (values given in hectoPascal (hPa))
- temperature (values given in Celsius degree (degC))

File iks01a2.c/h serve as top level API for accessing sensors

## GPS RTK [Simple RTK2B]

GPS is accessed via uart2. The gnss.c file provides the "ProcessNMEALine" function, which interprets the frames received on UART2 from the GPS. This function updates the global variable `gpsCoords` (latitude, longitude, altitude). Latitude and longitude are in decimal degrees, altitude in meters.


## CAN

CAN driver can be found in drivers/BSP/can. The file "can.h" contains all the CAN IDs. The `CAN_Send` function is used to send data through the CAN bus.
