/**
 * hal_common_tm4c.h
 *
 *  Created on: Mar 4, 2017
 *      Author: Vedran Mikov
 */
#include "hwconfig.h"

#ifndef ROVERKERNEL_HAL_TM4C1294_HAL_COMMON_TM4C_H_
#define ROVERKERNEL_HAL_TM4C1294_HAL_COMMON_TM4C_H_

#define HAL_OK                  0

#ifdef __cplusplus
extern "C"
{
#endif

/// Global clock variable
extern uint32_t g_ui32SysClock;


extern void         HAL_DelayUS(uint32_t us);
extern void         HAL_BOARD_CLOCK_Init();
extern void         HAL_BOARD_Reset();
extern void         UNUSED (int32_t arg);
extern uint32_t     _TM4CMsToCycles(uint32_t ms);

extern void         HAL_SetPWM(uint32_t id, uint32_t pwm);
extern uint32_t     HAL_GetPWM(uint32_t id);

#ifdef __cplusplus
}
#endif

#endif /* ROVERKERNEL_HAL_TM4C1294_HAL_COMMON_TM4C_H_ */
