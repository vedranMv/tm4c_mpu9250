/**
 *  hal_mpu_spi_tm4c.c
 *
 *  SPI drivers for MPU9250 on TM4C1294NCPDT
 *  This file implements communication with MPU9250 IMU by utilizing SPI bus.
 *  SPI2 bus is used at 1MHZ speed with PN2 as slave select (configured as GPIO),
 *  PA5 as data-ready signal, and PL4 as power-control pin.
 *
 *  Created on: Mar 4, 2017
 *      Author: Vedran
 */
#include "hal_mpu_tm4c.h"

#if defined(__HAL_USE_MPU9250_SPI__)       //  Compile only if module is enabled

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
#include "driverlib/ssi.h"
#include "driverlib/timer.h"


/**     MPU9250 - related macros        */
#define MPU9250_SPI_BASE SSI2_BASE

/**
 * Initializes SPI2 bus for communication with MPU
 *   * SPI Bus frequency 1MHz, PD0 as MISO, PD1 as MOSI, PD3 as SCLK
 *   * Pin PL4 as power switch (control external MOSFET to cut-off power to MPU)
 *   * Pin PA5 as input, to receive data-ready signal from MPU
 *   * Pin PN2 as slave select, but configured as GPIO and manually toggled
 */
void HAL_MPU_Init(void((*custHook)(void)))
{

    //  Enable peripherals in use. Also reset SSI2 at the end to allow calling
    //  this function at any point in order to reset SPI interface.
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
    MAP_SysCtlPeripheralReset(SYSCTL_PERIPH_SSI2);

    //  Configure SSI2 pins
    MAP_GPIOPinConfigure(GPIO_PD3_SSI2CLK);
    MAP_GPIOPinConfigure(GPIO_PD0_SSI2XDAT1);   //MISO
    MAP_GPIOPinConfigure(GPIO_PD1_SSI2XDAT0);   //MOSI
    MAP_GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3);

    //  Setup SPI: 1MHz, 8 bit data, mode 0
    MAP_SSIConfigSetExpClk(SSI2_BASE, g_ui32SysClock, SSI_FRF_MOTO_MODE_0,
                           SSI_MODE_MASTER, 1000000, 8);
    //  Enable SPI peripheral
    MAP_SSIEnable(SSI2_BASE);

    //  Empty receiving buffer
    uint32_t dummy[1];
    while (MAP_SSIDataGetNonBlocking(SSI2_BASE, &dummy[0]));

    //  Configure power-switch pin
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_4);
    MAP_GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_4, 0x00);

    //  Configure input pin to receive interrupts from MPU
    //      (not used as actual interrupt)
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_5);
    MAP_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0x00);

    //  Configure slave select pin
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_2);
    MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 0xFF);
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
 * Check if MPU has raised an interrupt to notify it has new data ready
 * @return true if interrupt pin is high, false otherwise
 */
bool HAL_MPU_DataAvail()
{
    return (MAP_GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_5) != 0);
}

/**
 * Write one byte of data to SPI bus and wait until transmission is over (blocking)
 * @param I2Caddress (NOT USED) Here for compatibility with I2C HAL implementation
 * @param regAddress Address of register in MPU to write into
 * @param data Data to write into the register
 */
void HAL_MPU_WriteByte(uint8_t I2Caddress, uint8_t regAddress, uint8_t data)
{
    uint32_t dummy[1];

    regAddress = regAddress & 0x7F; //  MSB = 0 for writing operation
    //  Drive CS low and wait one SPI clock cycle
    MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 0x00);
    HAL_DelayUS(1);
    //  Send register address
    SSIDataPut(SSI2_BASE, regAddress);
    //  Send register data
    SSIDataPut(SSI2_BASE, data);
    //  Wait until sending buffer is empty
    while(SSIBusy(SSI2_BASE));
    //  Drive CS high to stop communication
    MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 0xFF);

    //  Empty any received junk from the receive buffer
    while (SSIDataGetNonBlocking(SSI2_BASE, &dummy[0]));
}

/**
 * Send a byte-array of data through SPI bus (blocking)
 * @param I2Caddress (NOT USED) Here for compatibility with I2C HAL implementation
 * @param regAddress Address of a first register in MPU to start writing into
 * @param data Buffer of data to send
 * @param length Length of data to send
 * @return 0 to verify that function didn't hang somewhere
 */
uint8_t HAL_MPU_WriteBytes(uint8_t I2Caddress, uint8_t regAddress,
                           uint16_t length, uint8_t *data)
{
    uint16_t i;
    uint32_t dummy[1];

    regAddress = regAddress & 0x7F; //  MSB = 0 for writing operation
    MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 0x00);
    HAL_DelayUS(1);
    SSIDataPut(SSI2_BASE, regAddress);

    for (i = 0; i < length; i++)
        SSIDataPut(SSI2_BASE, data[i]);

    while(SSIBusy(SSI2_BASE));
    MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 0xFF);

    while (SSIDataGetNonBlocking(SSI2_BASE, &dummy[0]));

    return 0;
}

/**
 * Read one byte of data from SPI device (performs dummy write as well)
 * @param I2Caddress (NOT USED) Here for compatibility with I2C HAL implementation
 * @param regAddress Address of register in MPU to read from
 * @return Byte of data received from SPI device
 */
uint8_t HAL_MPU_ReadByte(uint8_t I2Caddress, uint8_t regAddress)
{
    uint32_t dummy[1], data;

    regAddress = regAddress | 0x80; //  MSB = 1 for reading operation
    MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 0x00);
    HAL_DelayUS(1);

    SSIDataPut(SSI2_BASE, regAddress);
    SSIDataGet(SSI2_BASE, &dummy[0]);
    SSIDataPut(SSI2_BASE, 0x00);
    SSIDataGet(SSI2_BASE, &data);

    while(SSIBusy(SSI2_BASE));
    MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 0xFF);

    while (SSIDataGetNonBlocking(SSI2_BASE, &dummy[0]));

    return (uint8_t)(data & 0xFF);
}

/**
 * Read several bytes from SPI device (performs dummy write as well)
 * @param I2Caddress (NOT USED) Here for compatibility with I2C HAL implementation
 * @param regAddress Address of register in MPU to read from
 * @param count Number of bytes to red
 * @param dest Pointer to data buffer in which data is saved after reading
 */
uint8_t HAL_MPU_ReadBytes(uint8_t I2Caddress, uint8_t regAddress,
                          uint16_t length, uint8_t* data)
{
    uint16_t i;
    uint32_t dummy[1];

    regAddress = regAddress | 0x80; //  MSB = 1 for reading operation
    MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 0x00);
    HAL_DelayUS(1);

    SSIDataPut(SSI2_BASE, regAddress);
    SSIDataGet(SSI2_BASE, &dummy[0]);

    for (i = 0; i < length; i++)
    {
        uint32_t tmp;
        SSIDataPut(SSI2_BASE, 0x00);
        SSIDataGet(SSI2_BASE, &tmp);
        data[i] = tmp & 0xFF;
    }

    while(SSIBusy(SSI2_BASE));
    MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 0xFF);

    while (SSIDataGetNonBlocking(SSI2_BASE, &dummy[0]));

    return 0;
}

#endif /* __HAL_USE_MPU9250_SPI__ */
