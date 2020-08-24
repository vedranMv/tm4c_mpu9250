/**
 *  mpu9250.cpp
 *
 *  Created on: 25. 3. 2015.
 *      Author: Vedran
 */
#include "mpu9250.h"

#if defined(__HAL_USE_MPU9250_NODMP__)  //  Compile only if module is enabled

#include "HAL/hal.h"
#include "api_mpu9250.h"
#include "libs/myLib.h"
#include "libs/helper_3dmath.h"

#include "registerMap.h"


//  Enable debug information printed on serial port
//#define __DEBUG_SESSION__

#ifdef __DEBUG_SESSION__
#include "serialPort/uartHW.h"

#endif


///-----------------------------------------------------------------------------
///         Functions for returning static instance                     [PUBLIC]
///-----------------------------------------------------------------------------

/**
 * Return reference to a singleton
 * @return reference to an internal static instance
 */
MPU9250& MPU9250::GetI()
{
    static MPU9250 singletonInstance;
    return singletonInstance;
}

/**
 * Return pointer to a singleton
 * @return pointer to a internal static instance
 */
MPU9250* MPU9250::GetP()
{
    return &(MPU9250::GetI());
}

///-----------------------------------------------------------------------------
///         Public functions used for configuring MPU9250               [PUBLIC]
///-----------------------------------------------------------------------------

/**
 * Initialize hardware used by MPU9250
 * Initializes bus for communication with MPU, pin(PA5) to be toggled by MPU9250
 * when it has data available for reading.
 * @return One of MPU_* error codes
 */
int8_t MPU9250::InitHW()
{
    HAL_MPU_Init();
    HAL_MPU_PowerSwitch(true);

    return MPU_SUCCESS;
}

/**
 * Initialize MPU sensor, load DMP firmware and configure DMP output. Prior to
 * any software initialization, this function power-cycles the board
 * @return One of MPU_* error codes
 */
int8_t MPU9250::InitSW()
{
    //  Power cycle MPU chip before every SW initialization
    HAL_MPU_PowerSwitch(false);
    HAL_DelayUS(20000);
    HAL_MPU_PowerSwitch(true);
    HAL_DelayUS(30000);

#ifdef __DEBUG_SESSION__
    DEBUG_WRITE("Starting up initialization\n");
#endif

    initMPU9250();
    initAK8963();

#ifdef __DEBUG_SESSION__
    DEBUG_WRITE("done\n");
#endif

    return MPU_SUCCESS;
}

/**
 * Trigger software reset of the MPU module by writing into corresponding
 * register. Wait for 50ms afterwards for sensor to start up.
 * @return One of MPU_* error codes
 */
int8_t MPU9250::Reset()
{
    HAL_MPU_WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, 1 << 7);
    HAL_DelayUS(50000);

    return MPU_SUCCESS;
}

/**
 * Control power supply of the MPU9250
 * Enable or disable power supply of the MPU9250 using external MOSFET
 * @param en Power state
 * @return One of MPU_* error codes
 */
int8_t MPU9250::Enabled(bool en)
{
    HAL_MPU_PowerSwitch(en);

    return MPU_SUCCESS;
}

/**
 * Check if new sensor data has been received
 * @return true if new sensor data is available
 *        false otherwise
 */
bool MPU9250::IsDataReady()
{
    //  Call HAL to read the sensor interrupt pin
    return HAL_MPU_DataAvail();
}

/**
 * Get ID from MPU, should always return 0x71
 * @return ID value stored in MPU's register
 */
uint8_t MPU9250::GetID()
{
    uint8_t ID;
    ID = HAL_MPU_ReadByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);

    return ID;
}

/**
 * Trigger reading data from MPU9250
 * Read data from MPU9250 and run AHRS algorithm when done
 * @return One of MPU_* error codes
 */
int8_t MPU9250::ReadSensorData()
{
    int16_t gyro[3], accel[3], mag[3];

    //  Read sensor data into buffers
    readAccelData(accel);
    readGyroData(gyro);
    //  Check if we're asked to read magnetometer
    if (_magEn)
        readMagData(mag);
    else
        memset((void*)mag, 0, 3*sizeof(uint16_t));

    //  Conversion from digital sensor readings to actual values
    for (uint8_t i = 0; i < 3; i++)
    {
        _acc[i] = (float)accel[i] * getAres();  //  m/s^2
        _gyro[i] = (float)gyro[i] * getGres();  //  deg/s
        _mag[i] = (float)mag[i] * getMres();    //  mG
    }

    //  Update attitude with new sensor readings
    // MPU9250 magnetometer is oriented differently than IMU
    _ahrs.Update(_gyro[0], _gyro[1], _gyro[2],
                 _acc[0], _acc[1], _acc[2],
                 _mag[1], _mag[0], _mag[2]);

    //  Copy data from AHRS object to this one
    memcpy((void*)_ypr, (void*)_ahrs.ypr, 3*sizeof(float));

    return MPU_SUCCESS;
}

/**
 * Copy orientation from internal buffer to user-provided one
 * @param RPY pointer to float buffer of size 3 to hold roll-pitch-yaw
 * @param inDeg if true RPY returned in degrees, if false in radians
 * @return One of MPU_* error codes
 */
int8_t MPU9250::RPY(float* RPY, bool inDeg)
{
    //  Copy data from internal buffer to a user-provided one, perform
    //  conversion from radians to degrees if asked
    for (uint8_t i = 0; i < 3; i++)
        if (inDeg)
            RPY[i] = _ypr[2-i]*180.0/PI_CONST;
        else
            RPY[i] = _ypr[2-i];

    return MPU_SUCCESS;
}

/**
 * Copy acceleration from internal buffer to user-provided one
 * @param acc Pointer a float array of min. size 3 to store 3-axis acceleration
 *        data
 * @return One of MPU_* error codes
 */
int8_t MPU9250::Acceleration(float *acc)
{
    //  Copy data from internal buffer to a user-provided one
    memcpy((void*)acc, (void*)_acc, sizeof(float)*3);

    return MPU_SUCCESS;
}

/**
 * Copy angular rotation from internal buffer to user-provided one
 * @param gyro Pointer a float array of min. size 3 to store 3-axis rotation
 *        data
 * @return One of MPU_* error codes
 */
int8_t MPU9250::Gyroscope(float *gyro)
{
    //  Copy data from internal buffer to a user-provided one
    memcpy((void*)gyro, (void*)_gyro, sizeof(float)*3);

    return MPU_SUCCESS;
}

/**
 * Copy mag. field strength from internal buffer to user-provided one
 * @param mag Pointer a float array of min. size 3 to store 3-axis mag. field
 *        strength data
 * @return One of MPU_* error codes
 */
int8_t MPU9250::Magnetometer(float *mag)
{
    //  Copy data from internal buffer to a user-provided one
    memcpy((void*)mag, (void*)_mag, sizeof(float)*3);

    return MPU_SUCCESS;
}

/**
 * Configure settings of AHRS algorithm
 * @note Using dT=0 will not update the value of dT in AHRS. This can be used
 * when one wants to update only the gains
 * @param dT Sampling time (time step between measurements)
 * @param kp Proportional gain
 * @param ki Integral gain
 * @return One of MPU_* error codes
 */
int8_t MPU9250::SetupAHRS(float dT, float kp, float ki)
{
    _ahrs.twoKi = 2* ki;
    _ahrs.twoKp = 2* kp;

    if (dT != 0.0f)
        _ahrs.InitSW(dT);

    return MPU_SUCCESS;
}

///-----------------------------------------------------------------------------
///                      Class constructor & destructor              [PROTECTED]
///-----------------------------------------------------------------------------

MPU9250::MPU9250() :  dT(0), _magEn(true), _ahrs()
{
    //  Initialize arrays
    memset((void*)_ypr, 0, 3);
    memset((void*)_acc, 0, 3);
    memset((void*)_gyro, 0, 3);
    memset((void*)_mag, 0, 3);
}

MPU9250::~MPU9250()
{}

#endif  /* __HAL_USE_MPU9250_NODMP__ */
