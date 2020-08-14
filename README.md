MPU9250 on Tiva TM4C1294
======================

<img style="width:200px;float: left;margin:10px;" src="http://des.everbuying.net/uploads/2015/201509/heditor/201509231442464706.jpg">

This repository contains an example project for running MPU9250 IMU module with TM4C1294 microcontroller. __Note__ that the project is for CodeComposer studio. MPU9250 is implemented as C++ singleton.

Code is taken from a [bigger project I worked on](https://github.com/vedranMv/roverRPi3)  and can be integrated with other modules from there (task scheduler and event logger). For clarity those were removed from this example but feel free to check the repository to find out more about them. Event logger allows for tacking the state of a module by tracking chronologically the most common events and their source (_startup_, _initialization_, _error_, etc.). Task scheduler on the other hand, is a simple run-to-completion scheduler that allows one to execute a sequence of tasks based on their starting time.

## Functionality

This library allows one to connect MPU9250 to TM4C1294 mcu over either SPI or I2C. Choosing the communication protocol is done through ``hwconfig.h`` file. Softwarewise, this library supports running the MPU in either the standard way where one read sensor values directly OR with the DMP firmware on-board. Again, use ``hwconfig.h`` to configure whether which version to use.

#### DMP
DMP firmware was released by the InvenSense and can perform 6DOF sensor fusion to compute the quaternions from which it is possible to find commonly used Euler angles. Sadly, DMP doesn't take into consideration magnetometer so, depending on the application it might not be suitable.

#### Direct sensor readings
Reading direct sensor values from MPUs' registers is the most common way of operation. Registers and their content are described in the official [register map](https://store.invensense.com/Datasheets/invensense/RM-MPU-9150A-00-v3.0.pdf). This library then integrates Mahonys' algorithm for calculating quaternions in as a way of performing 9DOF sensor fusion and estimating the attitude of the sensor.

## Wiring in I2C mode

MPU9250 can be connected to I2C2 peripheral of Tiva evaluation kit as follows:


MPU9250       |   EK-TM4C1294XL
--------------|------------------
MPU9250 SDA   | PN4(I2C2SDA)
MPU9250 SCL   | PN5(I2C2SCL)
MPU9250 INT   | PA5(GPIO)
MPU9250 AD0   | GND
MPU9250 VCC   | 3.3V
MPU9250 GND   | GND

I2C2 is used in high-speed mode, 400kHz clock. Additionally, library implements power control functionality through pin PL4. It is meant to control external n-type MOSFET to cut the power to MPU9250. Power control signal is designed as active-high, cutting the power to MPU9250 when it's set low.


## Wiring in SPI mode

MPU9250 can be connected to SPI2 peripheral of Tiva evaluation kit as follows:

MPU9250       |   EK-TM4C1294XL
--------------|------------------
MPU9250 SDA   | PD1(SPI2MOSI)
MPU9250 SCL   | PN5(SPI2CLK)
MPU9250 INT   | PA5(GPIO)
MPU9250 AD0   | PD2(SPI2MOSI)
MPU9250 NCS   | PN2(GPIO used as slave select)
MPU9250 VCC   | 3.3V
MPU9250 GND   | GND

Speed of SPI transfer is set to 1MHz. (sidenote: I have successfully tested up to 60MHz with TM4C1294 after which Tiva cannot generate the clock any more) Additionally, library implements power control functionality through pin PL4. It is meant to control external n-type MOSFET to cut the power to MPU9250. Power control signal is designed as active-high, cutting the power to MPU9250 when it's set low.


## Library

MPU9250 library provided in this example is implemented in C++ and based on the singleton design approach. At the beginning of the program, user grabs the reference to the instance of a singleton and uses it through the rest of the program.

#### DMP mode

![alt tag](https://vedran.ml/public/images/DMP.png)

DMP mode uses InvenSense code to load the DMP firmware on startup and use its sensor fusion for estimating the orientation. Output rate of fusion algorithm is set to 50Hz (can be increased to 200Hz) and the code handles conversion from quaternions to Euler angles.

#### Direct-sensor-reading mode

![alt tag](https://vedran.ml/public/images/DSR.png)

Basic functionality is implemented in form of configuring accelerometer and gyro for 1kHz output rate, performing accelerometer and gyro calibration. Current software interface is rather simplistic and allows for reading direct sensor measurements, reboot the MPU and control its power supply. Furthermore, as mentioned above, [Mahonys' algorithm](https://github.com/PaulStoffregen/MahonyAHRS) is implemented to perform 9DOF sensor fusion and produce orientation. Core functionality of Direct-sensor-reading mode is ported from [SparkFuns' MPU9250 library](https://github.com/sparkfun/SparkFun_MPU-9250_Breakout_Arduino_Library) (but extended with SPI).


At this point there is __no__ magnetometer calibration functionality implemented for any of the modes.

## Example code

``main.cpp`` contains a simple example which demonstrates initialization of the sensor, and a loop which reads sensor data at roughly 1kHz, computes orientation and prints it through serial port.


## Porting the library

Even though the library was developed and tested on TM4C1294 the functional code is fully decoupled from hardware through the use of Hardware Abstraction Layer (HAL). If you want to experiment with support for other board simply create new folder in ``HAL/``, add in the same files as in ``HAL/tm4c1294/``. Keep interface of new HAL the same as that in ``HAL/tm4c1294/``, i.e. use same function names as those in header files ``HAL/tm4c1294/*.h``. Main HAL include file, ``HAL/hal.h``, then uses macros to select the right board and load appropriate board drivers.
