/**
 * api_mpu9250.h
 *
 *
 *  Created on: Jan 26, 2018
 *      Author: Vedran Mikov
 *
 *  API for using MPU9250 IMU sensor. API is board-independent thanks to the
 *  hardware abstraction layer and supports communication with MPU through
 *  both SPI and I2C. Switch between the communication protocol must be done
 *  in HAL as this API has no knowledge of physical communication layer.
 *
 *  @note This implementation is modified version of existing arduino library:
 *      https://github.com/sparkfun/SparkFun_MPU-9250_Breakout_Arduino_Library
 *      Changes to original project were made in order to support SPI interface
 *      instead of commonly used I2C.
 *
 *  @version 1.0.0
 *  V1.0.0
 *  +Creation of file. Tested reading functions for gyro/mag/accel and
 *  initialization. API tested with both SPI & I2C.
 */
#include "hwconfig.h"

//  Compile following section only if hwconfig.h says to include this module
#if !defined(ROVERKERNEL_MPU9250_API_MPU9250_H_) && defined(__HAL_USE_MPU9250_NODMP__)
#define ROVERKERNEL_MPU9250_API_MPU9250_H_


#ifdef __cplusplus
extern "C"
{
#endif

    void    initMPU9250();
    void    initAK8963();

    float   getMres();
    float   getGres();
    float   getAres();

    void    readAccelData(int16_t *);
    void    readGyroData(int16_t *);
    void    readMagData(int16_t *);
    int16_t readTempData();


    void    calibrateMPU9250(float * gyroBias, float * accelBias);
    //  TODO:
    //void    MPU9250SelfTest(float * destination);
    //void    magCalMPU9250(float * dest1, float * dest2);
#ifdef __cplusplus
}
#endif

#endif /* ROVERKERNEL_MPU9250_API_MPU9250_H_ */
