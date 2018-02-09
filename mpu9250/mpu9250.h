/**
 * MPU9250.h
 *
 *  Created on: 25. 3. 2015.
 *      Author: Vedran Mikov
 *
 *  @version V3.1.0
 *  V1.0 - 25.3.2016
 *  +MPU9250 library now implemented as a C++ object
 *  V1.1 - 25.6.2016
 *  +New class Orientation added in order to provide single interface for position data
 *  V1.2 - 25.2.2017
 *  +Integration with task scheduler
 *  V1.2.1 - 11.3.2017
 *  +Changed MPU9250 class into a singleton
 *  V3.0 - 29.5.2017
 *  +Completely rewriting MPU9250 class, uses built-in digital motion processor
 *  instead of reading raw sensor data.
 *  V3.0.1 - 2.7.2017
 *  +Change include paths for better portability, new way of printing to debug
 *  +Integration with event logger
 *  V3.0.2 - 2.9.2017
 *  +Added soft-reboot for resetting only event logger status
 *  +Moved data processing from ISR to task-scheduler callback
 *  V3.0.3 - 21.9.2017
 *  *Fixed FIFO overflow error in MPU causing occasional glitches when reading
 *  sensor data
 *  V3.0.4 - 13.12.2017
 *  +HAL and hardware support power cycling of MPU. Added power-cycle step in
 *  initialization routine of MPU
 *  -Removed interrupt-based sensor readings; Polling sensor from scheduler
 *  -Removed 'listen' functionality
 *  V3.1.0 - 28.1.2018
 *  +Implemented support for SPI communication with MPU (use hwconfig.h to set
 *  the mode of communication)
 *  +Implemented support for using the MPU module without DMP firmware, getting
 *  raw sensor measurements and computing orientation from them - check
 *  api_mpu9250 files. (use hwconfig.h to select which mode of operation to use,
 *  raw data or DMP)
 *  V3.1.1 - 9.1.2018
 *  +Created interface to read acceleration/gyro/mag data
 *  +Added Mahony algorithm for attitude estimation from sensor data
 */
#include "hwconfig.h"

//  Compile following section only if hwconfig.h says to include this module
#if !defined(ROVERKERNEL_MPU9250_MPU9250_H_) && defined(__HAL_USE_MPU9250__)
#define ROVERKERNEL_MPU9250_MPU9250_H_

//  Enable integration of this library with task scheduler but only if task
//  scheduler is being compiled into this project
#if defined(__HAL_USE_TASKSCH__)
#define __USE_TASK_SCHEDULER__
#endif  /* __HAL_USE_TASKSCH__ */

//  Check if this library is set to use task scheduler
#if defined(__USE_TASK_SCHEDULER__)
    #include "taskScheduler/taskScheduler.h"
    //  Unique identifier of this module as registered in task scheduler
    #define MPU_UID             3
    //  Definitions of ServiceID for service offered by this module
    #define MPU_T_POWERSW         0
    #define MPU_T_GET_DATA        1
    #define MPU_T_REBOOT          2
    #define MPU_T_SOFT_REBOOT     3
    #define MPU_T_AHRS_CONFIG     4
#endif

//  Custom error codes for the library
#define MPU_SUCCESS             0
#define MPU_ERROR               2

#if defined(__HAL_USE_MPU9250_NODMP__)
    //  Mahony AHRS is used for computing orientation without DMP
    #include "MahonyAHRS.h"
#endif


/**
 * Class object for MPU9250 sensor
 */
class MPU9250
{
    friend void _MPU_KernelCallback(void);
    friend void MPUDataHandler(void);
    public:
        static MPU9250& GetI();
        static MPU9250* GetP();

        int8_t  InitHW();
        int8_t  InitSW();
        int8_t  Reset();
        int8_t  Enabled(bool en);
        bool    IsDataReady();
        uint8_t GetID();

        int8_t  ReadSensorData();
        int8_t  RPY(float* RPY, bool inDeg);
        int8_t  Acceleration(float *acc);
        int8_t  Gyroscope(float *gyro);
        int8_t  Magnetometer(float *mag);

        volatile float  dT;


    protected:
        MPU9250();
        ~MPU9250();
        MPU9250(MPU9250 &arg) {}              //  No definition - forbid this
        void operator=(MPU9250 const &arg) {} //  No definition - forbid this

        //  Yaw-Pitch-Roll orientation[Y,P,R] in radians
        volatile float _ypr[3];
        //  Acceleration [x,y,z]
        volatile float _acc[3];
        //  Gyroscope readings [x,y,z]
        volatile float _gyro[3];
        //  Magnetometer readings[x,y,z]
        volatile float _mag[3];
        //  Magnetometer control
        bool _magEn;

#if defined(__HAL_USE_MPU9250_NODMP__)
    private:
        //  Use Mahony algorithm for attitude estimations
        Mahony _ahrs;
    public:
        int8_t SetupAHRS(float dT, float kp, float ki);
#else
    protected:
        volatile float _gv[3];
        volatile float _quat[4];
#endif

        //  Interface with task scheduler - provides memory space and function
        //  to call in order for task scheduler to request service from this module
#if defined(__USE_TASK_SCHEDULER__)
        _kernelEntry _mpuKer;
#endif
};

#endif /* MPU9250_H_ */
