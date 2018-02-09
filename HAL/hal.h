/**
 *  hal.h
 *
 *  Created on: 02. 03. 2017.
 *      Author: Vedran Mikov
 *
 *  Hardware abstraction layer providing uniform interface between board support
 *  layer and hardware in there and any higher-level libraries. Acts as a
 *  switcher between HALs for different boards.
 */

#ifndef __HAL_H__
#define __HAL_H__

#include "hwconfig.h"


#ifdef __BOARD_TM4C1294NCPDT__

    #include "tm4c1294/hal_common_tm4c.h"
    #include "tm4c1294/hal_mpu_tm4c.h"


#elif __BOARD_ATMEGA328P__
//TODO: Arduino support
    #include "atmega328p_hal.h"
#endif

#endif  /* __HAL_H__ */
