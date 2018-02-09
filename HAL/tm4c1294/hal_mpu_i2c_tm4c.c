/**
 *  hal_mpu_i2c_tm4c.c
 *
 *  I2C drivers for MPU9250 on TM4C1294NCPDT
 *  This file implements communication with MPU9250 IMU by utilizing I2C bus.
 *  I2C2 bus is used in high-speed mode (400kHz) with pins PN4 as SDA and PN5 as
 *  SCL, PA5 as data-ready signal, and PL4 as power-control pin.
 *
 *  Created on: Mar 4, 2017
 *      Author: Vedran
 */
#include "hal_mpu_tm4c.h"

#if defined(__HAL_USE_MPU9250_I2C__)       //  Compile only if module is enabled

#include "libs/myLib.h"
#include "HAL/tm4c1294/hal_common_tm4c.h"

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_timer.h"
#include "inc/hw_ints.h"
#include "inc/hw_gpio.h"

#include "driverlib/rom_map.h"
#include "driverlib/rom.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "utils/uartstdio.h"
#include "driverlib/i2c.h"
#include "driverlib/timer.h"


/**     MPU9250 - related macros        */
#define MPU9250_I2C_BASE I2C2_BASE

/**
 * Initializes I2C2 bus for communication with MPU
 *   * I2C Bus frequency 400kHz, PN4 as SDA, PN5 as SCL
 *   * Pin PL4 as power switch (control external MOSFET to cut-off power to MPU)
 *   * Pin PA5 as input, to receive data-ready signal from MPU
 */
void HAL_MPU_Init(void((*custHook)(void)))
{
    //  Enable peripherals in use. Also reset I2C2 at the end to allow calling
    //  this function at any point in order to reset I2C interface.
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);
    MAP_SysCtlPeripheralReset(SYSCTL_PERIPH_I2C2);

    // Enable I2C communication interface, SCL, SDA lines
    MAP_GPIOPinConfigure(GPIO_PN4_I2C2SDA);
    MAP_GPIOPinConfigure(GPIO_PN5_I2C2SCL);
    MAP_GPIOPinTypeI2CSCL(GPIO_PORTN_BASE, GPIO_PIN_5);
    MAP_GPIOPinTypeI2C(GPIO_PORTN_BASE, GPIO_PIN_4);

    //  Enable I2C master interface
    MAP_I2CMasterEnable(MPU9250_I2C_BASE);

    // Run I2C bus in high-speed mode, 400kHz speed
    MAP_I2CMasterInitExpClk(MPU9250_I2C_BASE, g_ui32SysClock, true);

    //  Configure power-switch pin
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_4);
    MAP_GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_4, 0x00);

    //  Configure interrupt pin to receive output
    //      (not used as actual interrupts)
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_5);
    MAP_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0x00);

}

/**
 * Control power-switch for MPU9250
 * Controls whether or not MPU sensors receives power (n-ch MOSFET as switch)
 * @param powerState Desired state of power switch (active high)
 */
void HAL_MPU_PowerSwitch(bool powerState)
{
    if (powerState)
        MAP_GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_4, 0xFF);
    else
        MAP_GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_4, 0x00);
}

/**
 * Check if MPU has raised interrupt to notify it has new data ready
 * @return true if interrupt pin is active, false otherwise
 */
bool HAL_MPU_DataAvail()
{
    return (MAP_GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_5) != 0);
}

/**
 * Write one byte of data to I2C bus and wait until transmission is over (blocking)
 * @param I2Caddress 7-bit address of I2C device (8. bit is for R/W)
 * @param regAddress Address of register in I2C device to write into
 * @param data Data to write into the register of I2C device
 */
void HAL_MPU_WriteByte(uint8_t I2Caddress, uint8_t regAddress, uint8_t data)
{
    //  Set I2C address of MPU, writing mode
    MAP_I2CMasterSlaveAddrSet(MPU9250_I2C_BASE, I2Caddress, false);
    //  Push register address into a sending buffer
    MAP_I2CMasterDataPut(MPU9250_I2C_BASE, regAddress);
    //  Send start sequence and address, followed by register address
    MAP_I2CMasterControl(MPU9250_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    HAL_DelayUS(4);
    while(MAP_I2CMasterBusy(MPU9250_I2C_BASE));

    //  Send register data to write and stop sequence
    MAP_I2CMasterDataPut(MPU9250_I2C_BASE, data);
    MAP_I2CMasterControl(MPU9250_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    HAL_DelayUS(4);
    while(MAP_I2CMasterBusy(MPU9250_I2C_BASE));
}

/**
 * Send a byte-array of data through I2C bus(blocking)
 * @param I2Caddress 7-bit address of I2C device (8. bit is for R/W)
 * @param regAddress Address of register in I2C device to write into
 * @param data Data buffer of data to send
 * @param length Length of data to send
 * @return 0 so that caller can verify the function didn't hang
 */
uint8_t HAL_MPU_WriteBytes(uint8_t I2Caddress, uint8_t regAddress,
                           uint16_t length, uint8_t *data)
{
    uint16_t i;
    //  Set I2C address of MPU, writing mode
    MAP_I2CMasterSlaveAddrSet(MPU9250_I2C_BASE, I2Caddress, false);
    //  Push register address into a sending buffer
    MAP_I2CMasterDataPut(MPU9250_I2C_BASE, regAddress);
    //  Send start sequence and address, followed by register address. Use burst
    //  mode as we're sending more than 1 byte
    MAP_I2CMasterControl(MPU9250_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    HAL_DelayUS(4);
    while(MAP_I2CMasterBusy(MPU9250_I2C_BASE));

    //  Loop through all data that needs to be sent. Send everything but last
    //  byte
    for (i = 0; i < (length-1); i++)
    {
        MAP_I2CMasterDataPut(MPU9250_I2C_BASE, data[i]);
        MAP_I2CMasterControl(MPU9250_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
        HAL_DelayUS(4);
        while(MAP_I2CMasterBusy(MPU9250_I2C_BASE));
    }

    //  Send last byte together with stop sequence
    MAP_I2CMasterDataPut(MPU9250_I2C_BASE, data[length-1]);
    MAP_I2CMasterControl(MPU9250_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    HAL_DelayUS(4);
    while(MAP_I2CMasterBusy(MPU9250_I2C_BASE));

    return 0;
}

/**
 * Read one byte of data from I2C device
 * @param I2Caddress 7-bit address of I2C device (8. bit is for R/W)
 * @param regAddress Address of register in I2C device to write into
 * @return Data received from I2C device
 */
uint8_t HAL_MPU_ReadByte(uint8_t I2Caddress, uint8_t regAddress)
{
    uint32_t data;

    //  Set I2C address of MPU, reading mode (incorrect)
    //  I'm not sure why is the sending condition requires address in reading,
    //  bute there are problems if it's not done this way :/
    MAP_I2CMasterSlaveAddrSet(MPU9250_I2C_BASE, I2Caddress, false);
    //  Push register address into a sending buffer
    MAP_I2CMasterDataPut(MPU9250_I2C_BASE, regAddress);
    //  Send start sequence and address, followed by register address
    MAP_I2CMasterControl(MPU9250_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    HAL_DelayUS(4);
    while(MAP_I2CMasterBusy(MPU9250_I2C_BASE));

    //  Perform s single receive from I2C bus
    MAP_I2CMasterSlaveAddrSet(MPU9250_I2C_BASE, I2Caddress, true);
    MAP_I2CMasterControl(MPU9250_I2C_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
    HAL_DelayUS(4);
    while(MAP_I2CMasterBusy(MPU9250_I2C_BASE));
    //  Read a byte from receiving buffer
    data = MAP_I2CMasterDataGet(MPU9250_I2C_BASE);

    //  We're dealing with 8-bit data so return only lower 8 bits
    return (data & 0xFF);
}

/**
 * Read several bytes from I2C device
 * @param I2Caddress 7-bit address of I2C device (8. bit is for R/W)
 * @param regAddress address of register in I2C device to write into
 * @param count number of bytes to red
 * @param dest pointer to data buffer in which data is saved after reading
 */
uint8_t HAL_MPU_ReadBytes(uint8_t I2Caddress, uint8_t regAddress,
                          uint16_t length, uint8_t* data)
{
    uint16_t i;

    //  Set I2C address of MPU, reading mode (incorrect)
    //  I'm not sure why is the sending condition requires address in reading,
    //  bute there are problems if it's not done this way :/
    MAP_I2CMasterSlaveAddrSet(MPU9250_I2C_BASE, I2Caddress, false);
    //  Push register address into a sending buffer
    MAP_I2CMasterDataPut(MPU9250_I2C_BASE, regAddress);
    //  Send start sequence and address, followed by register address. Use burst
    //  mode as we're reading more than 1 byte
    MAP_I2CMasterControl(MPU9250_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    HAL_DelayUS(4);
    while(MAP_I2CMasterBusy(MPU9250_I2C_BASE));

    //  Change address to reading mode
    MAP_I2CMasterSlaveAddrSet(MPU9250_I2C_BASE, I2Caddress, true);

    //  Check how many bytes we need to receive
    if (length == 1)
        MAP_I2CMasterControl(MPU9250_I2C_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
    else
    {
        MAP_I2CMasterControl(MPU9250_I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
        HAL_DelayUS(4);
        while(MAP_I2CMasterBusy(MPU9250_I2C_BASE));
        data[0] = (uint8_t)(MAP_I2CMasterDataGet(MPU9250_I2C_BASE) & 0xFF);

        for (i = 1; i < (length-1); i++)
        {
            MAP_I2CMasterControl(MPU9250_I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
            HAL_DelayUS(4);
            while(MAP_I2CMasterBusy(MPU9250_I2C_BASE));
            data[i] = (uint8_t)(MAP_I2CMasterDataGet(MPU9250_I2C_BASE) & 0xFF);
        }
        MAP_I2CMasterControl(MPU9250_I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    }

    HAL_DelayUS(4);
    while(MAP_I2CMasterBusy(MPU9250_I2C_BASE));
    data[length-1] = (uint8_t)(MAP_I2CMasterDataGet(MPU9250_I2C_BASE) & 0xFF);

    return 0;
}


#endif /* __HAL_USE_MPU9250_I2C__ */
