/**
 * hal_mpu_tm4c.h
 *
 *  Created on: Mar 4, 2017
 *      Author: Vedran Mikov
 *
 *  Hardware abstraction layer (HAL) for MPU 9250 IMU and TM4C1294. Implements
 *  basic function for reading and writing data to MPU using either I2C and SPI
 *  communication protocol. Configure the protocol in hwconfig.h file
 *
 *  Hardware dependencies:
 *    * Check hwconfig to find out whether HAL uses I2C2(PN4 as SDA and PN5 as
 *      SCL) or SPI2(PD0 as MISO, PD1 as MOSI, PD3 as SCLK, PN2 as CS)
 *    * GPIO PA5 - Data available interrupt pin (normal input, not interrupt pin)
 *    * GPIO PL4 - Power switch for MPU (active high)
 */
#include "hwconfig.h"

//  Compile following section only if hwconfig.h says to include this module
#if !defined(ROVERKERNEL_HAL_TM4C1294_HAL_MPU_TM4C_H_) && defined(__HAL_USE_MPU9250__)
#define ROVERKERNEL_HAL_TM4C1294_HAL_MPU_TM4C_H_

#ifdef __cplusplus
extern "C"
{
#endif

/**     MPU9250 - related HW API       */
    extern void     HAL_MPU_Init();
    extern void     HAL_MPU_PowerSwitch(bool powerState);
    extern bool     HAL_MPU_DataAvail();

    extern void     HAL_MPU_WriteByte(uint8_t I2Caddress, uint8_t regAddress,
                                      uint8_t data);
    extern uint8_t  HAL_MPU_WriteBytes(uint8_t I2Caddress, uint8_t regAddress,
                                       uint16_t length, uint8_t *data);

    extern uint8_t  HAL_MPU_ReadByte(uint8_t I2Caddress, uint8_t regAddress);
    extern uint8_t  HAL_MPU_ReadBytes(uint8_t I2Caddress, uint8_t regAddress,
                                      uint16_t length, uint8_t* data);

#ifdef __cplusplus
}
#endif

#endif /* ROVERKERNEL_HAL_TM4C1294_HAL_MPU_TM4C_H_ */
