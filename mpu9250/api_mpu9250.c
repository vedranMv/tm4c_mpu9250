/**
 * api_mpu9250.c
 *
 *  Created on: Jan 26, 2018
 *      Author: Vedran
 */
#include "api_mpu9250.h"

#if defined(__HAL_USE_MPU9250_NODMP__)       //  Compile only if module is enabled

#include "registerMap.h"
#include "HAL/hal.h"
#include "libs/myLib.h"

//  Local (to this file) variables for holding configuration data
//  Values are defined as enums in "registerMap.h"
uint8_t Gscale = GFS_250DPS;
uint8_t Ascale = AFS_2G;
uint8_t Mscale = MFS_16BITS;
uint8_t Mmode = M_100HZ;


/**
 * Configure MPU9250 accelerometer and gyroscope
 * Configures gyro for 1kHz sampling rate, 42Hz bandwidth and 200Hz output rate
 * and use full scale readings (+/- 250 dps). Configures accel for 1kHz sampling
 * rate, 200Hz output rate and full scale readings (+/- 16g). Finally, configure
 * interrupt pin to be active high, push-pull, held high until cleared and
 * cleared by reading ANY register. I2C bypass is disabled to allow for SPI.
 * Data-ready interrupts are only allowed.
 */
void initMPU9250()
{
    float gBias[3], aBias[3];

    // Enable on first run to calibrate the sensor.
    // calibrateMPU9250(gBias, aBias);

    // wake up device
    // Clear sleep mode bit (6), enable all sensors
    HAL_MPU_WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);
    HAL_DelayUS(1000*100); // Wait for all registers to reset

    // Get stable time source
    // Auto select clock source to be PLL gyroscope reference if ready else
    HAL_MPU_WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
    HAL_DelayUS(1000*200);

    // Configure Gyro and Thermometer
    // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz,
    // respectively;
    // minimum delay time for this setting is 5.9 ms, which means sensor fusion
    // update rates cannot be higher than 1 / 0.0059 = 170 Hz
    // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
    // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!),
    // 8 kHz, or 1 kHz
    HAL_MPU_WriteByte(MPU9250_ADDRESS, CONFIG, 0x03);

    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    // Use a 200 Hz rate; a rate consistent with the filter update rate
    // determined inset in CONFIG above.
    HAL_MPU_WriteByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);

    // Set gyroscope full scale range
    // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are
    // left-shifted into positions 4:3

    // get current GYRO_CONFIG register value
    uint8_t c = HAL_MPU_ReadByte(MPU9250_ADDRESS, GYRO_CONFIG);
    // c = c & ~0xE0; // Clear self-test bits [7:5]
    c = c & ~0x02; // Clear Fchoice bits [1:0]
    c = c & ~0x18; // Clear AFS bits [4:3]
    c = c | Gscale << 3; // Set full scale range for the gyro
    // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of
    // GYRO_CONFIG
    c |= 0x03;
    // Write new GYRO_CONFIG value to register
    HAL_MPU_WriteByte(MPU9250_ADDRESS, GYRO_CONFIG, c);

    // Set accelerometer full-scale range configuration
    // Get current ACCEL_CONFIG register value
    c = HAL_MPU_ReadByte(MPU9250_ADDRESS, ACCEL_CONFIG);
    // c = c & ~0xE0; // Clear self-test bits [7:5]
    c = c & ~0x18;  // Clear AFS bits [4:3]
    c = c | Ascale << 3; // Set full scale range for the accelerometer
    // Write new ACCEL_CONFIG register value
    HAL_MPU_WriteByte(MPU9250_ADDRESS, ACCEL_CONFIG, c);

    // Set accelerometer sample rate configuration
    // It is possible to get a 4 kHz sample rate from the accelerometer by
    // choosing 1 for accel_fchoice_b bit [3]; in this case the bandwidth is
    // 1.13 kHz
    // Get current ACCEL_CONFIG2 register value
    c = HAL_MPU_ReadByte(MPU9250_ADDRESS, ACCEL_CONFIG2);
    c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
    c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
    // Write new ACCEL_CONFIG2 register value
    HAL_MPU_WriteByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c);
    // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
    // but all these rates are further reduced by a factor of 5 to 200 Hz because
    // of the SMPLRT_DIV setting

    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH
    // until interrupt cleared, clear on read of any register, and DISABLE
    // I2C_BYPASS_EN -> otherwise communication with AK8963 doesn't work when
    //  using SPI
    HAL_MPU_WriteByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x30);
    // Enable data ready (bit 0) interrupt
    HAL_MPU_WriteByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);
    HAL_DelayUS(1000*100);
}

/**
 * Initialize AK8963 magnetometer. 16bit data @ 100Hz
 * AK8963 is configured as a slave device of MPU9250 and not directly accessible.
 * Every transaction with AK8963 has to be carried through MPU9250 by configuring
 * its I2C slave communication registers. Result of the I2C transaction is also
 * stored internally in MPU and needs to be read as one would normally read
 * MPU registers.
 */
void initAK8963()
{
    //  Initialization uses I2C channel number 4 for writing data

    //  Configure master I2C clock (400kHz) for MPU to talk to slaves
    HAL_MPU_WriteByte(MPU9250_ADDRESS,  I2C_MST_CTRL, 0x5D);
    //  Enable I2C master
    HAL_MPU_WriteByte(MPU9250_ADDRESS,  USER_CTRL, 0x20);

    //  Stop I2C slave number 4
    HAL_MPU_WriteByte(MPU9250_ADDRESS,  I2C_SLV4_CTRL, 0x00);
    //  Set address for I2C4 slave to that of AK8963, writing mode (MSB=0)
    HAL_MPU_WriteByte(MPU9250_ADDRESS,  I2C_SLV4_ADDR, AK8963_ADDRESS);
    //  Select which register is being updated
    HAL_MPU_WriteByte(MPU9250_ADDRESS,  I2C_SLV4_REG, AK8963_CNTL);
    // Set value to write into the register:
    //      16-bit continuous measurements @ 100Hz
    HAL_MPU_WriteByte(MPU9250_ADDRESS,  I2C_SLV4_DO, Mscale << 4 | Mmode);
    // Trigger write data to slave device 4 -> AK8963
    HAL_MPU_WriteByte(MPU9250_ADDRESS,  I2C_SLV4_CTRL, 0x80);
    HAL_DelayUS(5000);
}

/**
 * Get resolution of magnetometer data. Used to convert digital readings to
 * analog values.
 * @return  Resolution in milliGaus/bit
 */
float getMres()
{
    float mRes;

  switch (Mscale)
  {
    // Possible magnetometer scales (and their register bit settings) are:
    // 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
      mRes = 10.0f * 4912.0f / 8190.0f; // Proper scale to return milliGauss
      break;
    case MFS_16BITS:
      mRes = 10.0f * 4912.0f / 32760.0f; // Proper scale to return milliGauss
      break;
  }

  return mRes;
}

/**
 * Get resolution of gyroscope data. Used to convert digital readings to
 * analog values.
 * @return Resolution in degrees per second(DPS)/bit
 */
float getGres()
{
    float gRes;
    switch (Gscale)
    {
        // Possible gyro scales (and their register bit settings) are:
        // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS (11).
    case GFS_250DPS:
        gRes = 250.0f / 32768.0f;
        break;
    case GFS_500DPS:
        gRes = 500.0f / 32768.0f;
        break;
    case GFS_1000DPS:
        gRes = 1000.0f / 32768.0f;
        break;
    case GFS_2000DPS:
        gRes = 2000.0f / 32768.0f;
        break;
    }

  return gRes;
}

/**
 * Get resolution of accelerometer data. Used to convert digital readings to
 * analog values.
 * @return Resolution in (m/s^2)/bit
 */
float getAres()
{
    float aRes;
    switch (Ascale)
    {
        // Possible accelerometer scales (and their register bit settings) are:
        // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    case AFS_2G:
        aRes = GRAVITY_CONST * 2.0f / 32768.0f;
        break;
    case AFS_4G:
        aRes = GRAVITY_CONST * 4.0f / 32768.0f;
        break;
    case AFS_8G:
        aRes = GRAVITY_CONST * 8.0f / 32768.0f;
        break;
    case AFS_16G:
        aRes = GRAVITY_CONST * 16.0f / 32768.0f;
        break;
    }

  return aRes;
}

/**
 * Read raw accelerometer data into a provided buffer
 * @param destination Buffer to save x, y, z acceleration data (min. size = 3)
 */
void readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  // Read the six raw data registers into data array
  HAL_MPU_ReadBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);

  // Turn the MSB and LSB into a signed 16-bit value
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}

/**
 * Read raw gyroscope data into a provided buffer
 * @param destination Buffer to save x, y, z gyroscope data (min. size = 3)
 */
void readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  // Read the six raw data registers sequentially into data array
  HAL_MPU_ReadBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);

  // Turn the MSB and LSB into a signed 16-bit value
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}

/**
 * Read raw magnetometer data into a provided buffer
 * @param destination Buffer to save x, y, z magnetometer data (min. size = 3)
 */
void readMagData(int16_t * destination)
{
    // x,y,z gyro register data, ST2 register stored here, must read ST2 at end
    // of data acquisition
    uint8_t rawData[7];

    //  TODO: In case it's necessary to always have new data when calling this
    //  function, implement hanging function which reads Status1 register and
    //  checks for DRDY bit -> I noticed it doesn't always work (i.e. new data
    //  is available but bit is 0)

    //  Stop any ongoing I2C0 operations
    HAL_MPU_WriteByte(MPU9250_ADDRESS,  I2C_SLV0_CTRL, 0x00);
    // Set to read from slave address of AK8963
    HAL_MPU_WriteByte(MPU9250_ADDRESS,  I2C_SLV0_ADDR, AK8963_ADDRESS | 0x80);

    // Start reading from X_out, 7 bytes. This will read 6 data registers and
    //  one status register which will tell whether there was an overflow
    //  in measurements
    HAL_MPU_WriteByte(MPU9250_ADDRESS,  I2C_SLV0_REG, AK8963_XOUT_L);
    // Read 7 bytes from I2C slave 0
    HAL_MPU_WriteByte(MPU9250_ADDRESS,  I2C_SLV0_CTRL, 0x87);
    // Move 7 registers from MPU reg to here
    HAL_MPU_ReadBytes(MPU9250_ADDRESS, EXT_SENS_DATA_00, 7, rawData);


    uint8_t c = 0 & rawData[6]; // End data read by reading ST2 register
    // Check if magnetic sensor overflow is set, if not then report data
    if (!(c & 0x08))
    {
      // Turn the MSB and LSB into a signed 16-bit value
      destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];
      // Data stored as little Endian
      destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];
      destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
    }
}

/**
 * Read data from internal temperature sensor
 * @return Temperature data
 */
int16_t readTempData()
{
  uint8_t rawData[2]; // x/y/z gyro register data stored here
  // Read the two raw data registers sequentially into data array
  HAL_MPU_ReadBytes(MPU9250_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);
  // Turn the MSB and LSB into a 16-bit value
  return ((int16_t)rawData[0] << 8) | rawData[1];
}

/**
 * Function which accumulates gyro and accelerometer data after device
 * initialization. It calculates the average of the at-rest readings and then
 * loads the resulting offsets into accelerometer and gyro bias registers.
 * @param gyroBias
 * @param accelBias
 */
void calibrateMPU9250(float * gyroBias, float * accelBias)
{
    uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
    uint16_t ii, packet_count, fifo_count;
    int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

    // reset device
    // Write a one to bit 7 reset bit; toggle reset device
    HAL_MPU_WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, 1<<7);
    HAL_DelayUS(1000*100);

    // get stable time source; Auto select clock source to be PLL gyroscope
    // reference if ready else use the internal oscillator, bits 2:0 = 001
    HAL_MPU_WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
    HAL_MPU_WriteByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
    HAL_DelayUS(1000*200);

    // Configure device for bias calculation
    // Disable all interrupts
    HAL_MPU_WriteByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);
    // Disable FIFO
    HAL_MPU_WriteByte(MPU9250_ADDRESS, FIFO_EN, 0x00);
    // Turn on internal clock source
    HAL_MPU_WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);
    // Disable I2C master
    HAL_MPU_WriteByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00);
    // Disable FIFO and I2C master modes
    HAL_MPU_WriteByte(MPU9250_ADDRESS, USER_CTRL, 0x00);
    // Reset FIFO and DMP
    HAL_MPU_WriteByte(MPU9250_ADDRESS, USER_CTRL, 0x0C);
    HAL_DelayUS(1000*15);

    // Configure MPU6050 gyro and accelerometer for bias calculation
    // Set low-pass filter to 188 Hz
    HAL_MPU_WriteByte(MPU9250_ADDRESS, CONFIG, 0x01);
    // Set sample rate to 1 kHz
    HAL_MPU_WriteByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);
    // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    HAL_MPU_WriteByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);
    // Set accelerometer full-scale to 2 g, maximum sensitivity
    HAL_MPU_WriteByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);

    uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
    uint16_t  accelsensitivity = 16384; // = 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    HAL_MPU_WriteByte(MPU9250_ADDRESS, USER_CTRL, 0x40);  // Enable FIFO
    // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in
    // MPU-9150)
    HAL_MPU_WriteByte(MPU9250_ADDRESS, FIFO_EN, 0x78);
    HAL_DelayUS(1000*40);  // accumulate 40 samples in 40 milliseconds = 480 bytes

    // At end of sample accumulation, turn off FIFO sensor read
    // Disable gyro and accelerometer sensors for FIFO
    HAL_MPU_WriteByte(MPU9250_ADDRESS, FIFO_EN, 0x00);
    // Read FIFO sample count
    HAL_MPU_ReadBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]);
    fifo_count = ((uint16_t)data[0] << 8) | data[1];
    // How many sets of full gyro and accelerometer data for averaging
    packet_count = fifo_count/12;

    for (ii = 0; ii < packet_count; ii++)
    {
        int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
        // Read data for averaging
        HAL_MPU_ReadBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]);
        // Form signed 16-bit integer for each sample in FIFO
        accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  );
        accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  );
        accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  );
        gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  );
        gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  );
        gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]);

        // Sum individual signed 16-bit biases to get accumulated signed 32-bit
        // biases.
        accel_bias[0] += (int32_t) accel_temp[0];
        accel_bias[1] += (int32_t) accel_temp[1];
        accel_bias[2] += (int32_t) accel_temp[2];
        gyro_bias[0]  += (int32_t) gyro_temp[0];
        gyro_bias[1]  += (int32_t) gyro_temp[1];
        gyro_bias[2]  += (int32_t) gyro_temp[2];
    }
    // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[0] /= (int32_t) packet_count;
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;

    // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    if (accel_bias[2] > 0L)
    {
        accel_bias[2] -= (int32_t) accelsensitivity;
    }
    else
    {
        accel_bias[2] += (int32_t) accelsensitivity;
    }

    // Construct the gyro biases for push to the hardware gyro bias registers,
    // which are reset to zero upon device startup.
    // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input
    // format.
    data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF;
    // Biases are additive, so change sign on calculated average gyro biases
    data[1] = (-gyro_bias[0]/4)       & 0xFF;
    data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
    data[3] = (-gyro_bias[1]/4)       & 0xFF;
    data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
    data[5] = (-gyro_bias[2]/4)       & 0xFF;

    // Push gyro biases to hardware registers
    HAL_MPU_WriteByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
    HAL_MPU_WriteByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
    HAL_MPU_WriteByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
    HAL_MPU_WriteByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
    HAL_MPU_WriteByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
    HAL_MPU_WriteByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);

    // Output scaled gyro biases for display in the main program
    gyroBias[0] = (float) gyro_bias[0]/(float) gyrosensitivity;
    gyroBias[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
    gyroBias[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

    // Construct the accelerometer biases for push to the hardware accelerometer
    // bias registers. These registers contain factory trim values which must be
    // added to the calculated accelerometer biases; on boot up these registers
    // will hold non-zero values. In addition, bit 0 of the lower byte must be
    // preserved since it is used for temperature compensation calculations.
    // Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.

    // A place to hold the factory accelerometer trim biases
    int32_t accel_bias_reg[3] = {0, 0, 0};
    // Read factory accelerometer trim values
    HAL_MPU_ReadBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]);
    accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
    HAL_MPU_ReadBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
    accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
    HAL_MPU_ReadBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
    accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

    // Define mask for temperature compensation bit 0 of lower byte of
    // accelerometer bias registers
    uint32_t mask = 1uL;
    // Define array to hold mask bit for each accelerometer bias axis
    uint8_t mask_bit[3] = {0, 0, 0};

    for (ii = 0; ii < 3; ii++)
    {
        // If temperature compensation bit is set, record that fact in mask_bit
        if ((accel_bias_reg[ii] & mask))
        {
            mask_bit[ii] = 0x01;
        }
    }

    // Construct total accelerometer bias, including calculated average
    // accelerometer bias from above
    // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g
    // (16 g full scale)
    accel_bias_reg[0] -= (accel_bias[0]/8);
    accel_bias_reg[1] -= (accel_bias[1]/8);
    accel_bias_reg[2] -= (accel_bias[2]/8);

    data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    data[1] = (accel_bias_reg[0])      & 0xFF;
    // preserve temperature compensation bit when writing back to accelerometer
    // bias registers
    data[1] = data[1] | mask_bit[0];
    data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    data[3] = (accel_bias_reg[1])      & 0xFF;
    // Preserve temperature compensation bit when writing back to accelerometer
    // bias registers
    data[3] = data[3] | mask_bit[1];
    data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    data[5] = (accel_bias_reg[2])      & 0xFF;
    // Preserve temperature compensation bit when writing back to accelerometer
    // bias registers
    data[5] = data[5] | mask_bit[2];

    // Apparently this is not working for the acceleration biases in the MPU-9250
    // Are we handling the temperature correction bit properly?
    // Push accelerometer biases to hardware registers
    HAL_MPU_WriteByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
    HAL_MPU_WriteByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
    HAL_MPU_WriteByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
    HAL_MPU_WriteByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
    HAL_MPU_WriteByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
    HAL_MPU_WriteByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);

    // Output scaled accelerometer biases for display in the main program
    accelBias[0] = (float)accel_bias[0]/(float)accelsensitivity;
    accelBias[1] = (float)accel_bias[1]/(float)accelsensitivity;
    accelBias[2] = (float)accel_bias[2]/(float)accelsensitivity;
}

#endif  /* __HAL_USE_MPU9250_NODMP__ */
