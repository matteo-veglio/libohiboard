/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2019 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *   Marco Giammarini <m.giammarini@warcomeb.it>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/**
 * @file libohiboard/include/lowpower-timer.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Low Power Timer definitions and prototypes.
 *
 * This file supply a set of function for managing low-power timer.
 */
#ifdef LIBOHIBOARD_LOWPOWER_TIMER

/**
 * @addtogroup LIBOHIBOARD_Driver
 * @{
 */

/**
 * @defgroup LOWPOWER_TIMER Low Power Timer
 * @brief Low power timer HAL driver
 * @{
 */

#ifndef __LOWPOWER_TIMER_H
#define __LOWPOWER_TIMER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"
#include "errors.h"
#include "types.h"
#include "utility.h"
#include "clock.h"

/**
 *
 */
typedef struct _LowPowerTimer_Device* LowPowerTimer_DeviceHandle;

#if defined (LIBOHIBOARD_STM32L4)

#include "hardware/lowpower-timer_STM32L4.h"

#endif

/**
 * The list of the possible peripheral HAL state.
 */
typedef enum _LowPowerTimer_DeviceState
{
    LOWPOWERTIMER_DEVICESTATE_RESET,
    LOWPOWERTIMER_DEVICESTATE_READY,
    LOWPOWERTIMER_DEVICESTATE_BUSY,
    LOWPOWERTIMER_DEVICESTATE_ERROR,

} LowPowerTimer_DeviceState;

/**
 * List of all possible clock prescaler for low-power peripheral.
 */
typedef enum _LowPowerTimer_ClockPrescaler
{
#if defined (LIBOHIBOARD_STM32L4)

    LOWPOWERTIMER_CLOCKPRESCALER_DIV1   = (0x00000000),
    LOWPOWERTIMER_CLOCKPRESCALER_DIV2   = LPTIM_CFGR_PRESC_0,
    LOWPOWERTIMER_CLOCKPRESCALER_DIV4   = LPTIM_CFGR_PRESC_1,
    LOWPOWERTIMER_CLOCKPRESCALER_DIV8   = (LPTIM_CFGR_PRESC_0 | LPTIM_CFGR_PRESC_1),
    LOWPOWERTIMER_CLOCKPRESCALER_DIV16  = LPTIM_CFGR_PRESC_2,
    LOWPOWERTIMER_CLOCKPRESCALER_DIV32  = (LPTIM_CFGR_PRESC_0 | LPTIM_CFGR_PRESC_2),
    LOWPOWERTIMER_CLOCKPRESCALER_DIV64  = (LPTIM_CFGR_PRESC_1 | LPTIM_CFGR_PRESC_2),
    LOWPOWERTIMER_CLOCKPRESCALER_DIV128 = LPTIM_CFGR_PRESC,

#endif
} LowPowerTimer_ClockPrescaler;

/**
 *
 */
typedef enum _Timer_LowPowerClockSource
{
#if defined (LIBOHIBOARD_STM32L4)

    /**< Internal clock selection, using PCLK */
    LOWPOWERTIMER_CLOCKSOURCE_INTERNAL_PCLK  = 0x00000000,
    /**< Internal clock selection, using LSI */
    LOWPOWERTIMER_CLOCKSOURCE_INTERNAL_LSI   = 0x00000001,
    /**< Internal clock selection, using HSI16 */
    LOWPOWERTIMER_CLOCKSOURCE_INTERNAL_HSI16 = 0x00000002,
    /**< Internal clock selection, using LSE */
    LOWPOWERTIMER_CLOCKSOURCE_INTERNAL_LSE   = 0x00000003,
    /**< External clock selection for ULP */
    LOWPOWERTIMER_CLOCKSOURCE_EXTERNAL       = 0xFFFF0000,

#endif
} LowPowerTimer_ClockSource;

/**
 * List of all possible clock polarity type for the counter unit.
 * This value must be used only in case of external clock (ULP).
 */
typedef enum _LowPowerTimer_ClockPolarity
{
#if defined (LIBOHIBOARD_STM32L4)

    LOWPOWERTIMER_CLOCKPOLARITY_RISING  = 0x00000000,
    LOWPOWERTIMER_CLOCKPOLARITY_FALLING = LPTIM_CFGR_CKPOL_0,
    LOWPOWERTIMER_CLOCKPOLARITY_BOTH    = LPTIM_CFGR_CKPOL_1,

#endif
} LowPowerTimer_ClockPolarity;

/**
 *
 */
typedef enum _LowPowerTimer_TriggerSource
{
#if defined (LIBOHIBOARD_STM32L4)

    LOWPOWERTIMER_TRIGGERSOURCE_SOFTWARE = 0x0000FFFF,
    LOWPOWERTIMER_TRIGGERSOURCE_0        = 0x00000000,
    LOWPOWERTIMER_TRIGGERSOURCE_1        = LPTIM_CFGR_TRIGSEL_0,
    LOWPOWERTIMER_TRIGGERSOURCE_2        = LPTIM_CFGR_TRIGSEL_1,
    LOWPOWERTIMER_TRIGGERSOURCE_3        = (LPTIM_CFGR_TRIGSEL_0 | LPTIM_CFGR_TRIGSEL_1),
    LOWPOWERTIMER_TRIGGERSOURCE_4        = LPTIM_CFGR_TRIGSEL_2,
    LOWPOWERTIMER_TRIGGERSOURCE_5        = (LPTIM_CFGR_TRIGSEL_0 | LPTIM_CFGR_TRIGSEL_2),
    LOWPOWERTIMER_TRIGGERSOURCE_6        = (LPTIM_CFGR_TRIGSEL_1 | LPTIM_CFGR_TRIGSEL_2),
    LOWPOWERTIMER_TRIGGERSOURCE_7        = LPTIM_CFGR_TRIGSEL,

#endif
} LowPowerTimer_TriggerSource;

/**
 * The list of the possible update mode of the autoreload and compare registers.
 * The values update are done immediately or after the end of current period.
 */
typedef enum _LowPowerTimer_UpdateMode
{
#if defined (LIBOHIBOARD_STM32L4)

    LOWPOWERTIMER_UPDATEMODE_IMMEDIATE  = 0x00000000,
    LOWPOWERTIMER_UPDATEMODE_END_PERIOD = LPTIM_CFGR_PRELOAD,

#endif
} LowPowerTimer_UpdateMode;

/**
 * The list of the possible source to increment the counter.
 */
typedef enum _LowPowerTimer_CounterSource
{
#if defined (LIBOHIBOARD_STM32L4)

    LOWPOWERTIMER_COUNTERSOURCE_INTERNAL = 0x00000000,
    LOWPOWERTIMER_COUNTERSOURCE_EXTERNAL = LPTIM_CFGR_COUNTMODE,

#endif
} LowPowerTimer_CounterSource;

/**
 * Struct used to configure low-power peripheral.
 */
typedef struct _LowPowerTimer_Config
{
//    Timer_Mode mode;                                /**< Modes of operations. */

    LowPowerTimer_ClockSource clockSource;
    LowPowerTimer_ClockPrescaler prescaler;

    /**
     * Selects the polarity of the active edge for the counter unit.
     * @note This parameter is used only when ULP clock source is used.
     */
    LowPowerTimer_ClockPolarity polarity;

    /** Selects the trigger source. */
    LowPowerTimer_TriggerSource triggerSource;

    /** Update mode type for autoreload and compare register. */
    LowPowerTimer_UpdateMode updateMode;

    /**
     * Specifies whether the counter is incremented each internal event
     * or each external event.
     */
    LowPowerTimer_CounterSource counterSource;


} LowPowerTimer_Config;

/**
 * @defgroup LOWPOWER_TIMER_Configuration_Functions Low-Power Timer configuration functions
 * @brief Functions to initialize and de-initialize a Low-Power Timer peripheral.
 * @{
 */

/**
 * Initialize the Low-Power Timer according to the specified parameters
 * in the @ref LowPowerTimer_Config and initialize the associated handle.
 *
 * @note This function is usable only with low power peripheral.
 *
 * @param[in] dev Low-Power Timer device handle
 * @param[in] config Configuration parameters list.
 * @return ERRORS_NO_ERROR The initialization is ok.
 */
System_Errors LowPowerTimer_init (LowPowerTimer_DeviceHandle dev,
                                  LowPowerTimer_Config *config);


/**
 * @}
 */

/**
 * This function start counter mode.
 *
 * @param[in] dev Low-Power Timer device handle
 * @param[in] counter Specifies the autoreload value.
 *                    It must be a value between 0x0000 and 0xFFFF
 * @return ERRORS_NO_ERROR The initialization is ok.
 */
System_Errors LowPowerTimer_startCounter (LowPowerTimer_DeviceHandle dev,
                                          uint32_t counter);

/**
 * This function stop counter mode.
 *
 * @param[in] dev Low-Power Timer device handle
 * @return ERRORS_NO_ERROR The initialization is ok.
 */
System_Errors LowPowerTimer_stopCounter (LowPowerTimer_DeviceHandle dev);

#ifdef __cplusplus
}
#endif

#endif // __LOWPOWER_H

/**
 * @}
 */

/**
 * @}
 */

#endif // LIBOHIBOARD_LOWPOWER
