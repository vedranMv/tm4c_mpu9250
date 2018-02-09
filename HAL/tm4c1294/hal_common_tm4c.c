/*
 * hal_common_tm4c.c
 *
 *  Created on: Mar 4, 2017
 *      Author: Vedran Mikov
 */
#include "libs/myLib.h"
#include "hal_common_tm4c.h"

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
#include "driverlib/fpu.h"
#include "driverlib/interrupt.h"
#include "driverlib/pwm.h"


uint32_t g_ui32SysClock;

/**
 *  Dummy function to be called to suppress "Unused variable" warnings
 */
void UNUSED (int32_t arg) { }

/**
 * Initialize microcontroller board clock & enable on-board floating-point unit
 */
void HAL_BOARD_CLOCK_Init()
{
    // Set the clock to use on-board 25MHz oscillator and generate 120MHz clock
    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                        SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
                                        SYSCTL_CFG_VCO_480), 120000000);
    //  Enable Floating-point unit (FPU)
    MAP_FPUEnable();
    //FPULazyStackingEnable();
    MAP_FPUStackingEnable();
    //  Enable interrupt handler
    MAP_IntMasterEnable();
}

/**
 * Software-triggered reboot of microcontroller
 */
void HAL_BOARD_Reset()
{
    MAP_SysCtlReset();
}

/**
 * Wait for given amount of us - blocking function
 * @param us time in us to wait
 */
void HAL_DelayUS(uint32_t us)   //1000
{
    float f = 1000000 / (float)us;//  Frequency = 1 / Period

    f = (float)g_ui32SysClock / (3.0 * f);
    MAP_SysCtlDelay((uint32_t)f);
}

/**
 * Calculate load value from timer based on desired time in milliseconds
 * @param ms time in milliseconds
 * @return equivalent number of clock cycles for main oscillator
 */
uint32_t _TM4CMsToCycles(uint32_t ms)
{
    return (ms*(g_ui32SysClock/1000));
}

/**
 * Set desired PWM duty cycle on specific output channel
 * @param id is channel ID of PWM channel affected
 * @param pwm value of PWM pulse (duty cycle) to set
 */
void HAL_SetPWM(uint32_t id, uint32_t pwm)
{
    MAP_PWMPulseWidthSet(PWM0_BASE, id, pwm);
}
/**
 * Get current PWM duty cycle on specific output channel
 * @param id is channel ID of PWM channel affected
 * @return PWM duty cycle at channel id
 */
uint32_t HAL_GetPWM(uint32_t id)
{
    return MAP_PWMPulseWidthGet(PWM0_BASE, id);
}


