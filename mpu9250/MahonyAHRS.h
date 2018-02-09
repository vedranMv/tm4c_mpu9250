//=============================================================================================
// MahonyAHRS.h
//=============================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=============================================================================================
#include "hwconfig.h"

#if !defined(MahonyAHRS_h) && defined(__HAL_USE_MPU9250_NODMP__)
#define MahonyAHRS_h
#include <math.h>

//--------------------------------------------------------------------------------------------
// Variable declaration

class Mahony {

    public:
             Mahony();
        void InitSW(float sampleTime)
                { _invSampleFreq = sampleTime; }
        void Update(float gx, float gy, float gz, float ax, float ay, float az,
                    float mx, float my, float mz);
        void UpdateNoMag(float gx, float gy, float gz,
                         float ax, float ay, float az);

        //  Yaw-Pitch-Raw orientation in radians
        float ypr[3];
        // Quaternion of sensor frame relative to auxiliary frame
        float q0, q1, q2, q3;
        //  Filter gains
        float twoKp;        // 2 * proportional gain (Kp)
        float twoKi;        // 2 * integral gain (Ki)


    private:
        //  Convert quaternions to YPR
        void            _ComputeAngles();
        static float    _InvSqrt(float x);

        float _integralFBx, _integralFBy, _integralFBz;  // integral error terms scaled by Ki
        float _invSampleFreq;
};

#endif
