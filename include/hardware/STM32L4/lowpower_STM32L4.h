/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2018 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *   Leonardo Morichelli <leonardo.morichelli@live.com>
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
 * @file libohiboard/include/hardware/STM32L4/lowpower_STM32L4.h
 * @author Leonardo Morichelli <leonardo.morichelli@live.com>
 * @brief Low Power useful definitions for STM32L4 series
 */

#ifndef __LOWPOWER_STM32L4_H
#define __LOWPOWER_STM32L4_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

/**
*Define for Lowpower mode CPU1
*/

#if defined (LIBOHIBOARD_STM32WB)

#define PWR_CR1_LPMS_STOP0              (0x00000000u)                         /*!< Stop 0: stop mode with main regulator */
#define PWR_CR1_LPMS_STOP1              (PWR_CR1_LPMS_0)                      /*!< Stop 1: stop mode with low power regulator */
#define PWR_CR1_LPMS_STOP2              (PWR_CR1_LPMS_1)                      /*!< Stop 2: stop mode with low power regulator and VDD12I interruptible digital core domain supply OFF (less peripherals activated than low power mode stop 1 to reduce power consumption)*/
#define PWR_CR1_LPMS_STANDBY            (PWR_CR1_LPMS_0 | PWR_CR1_LPMS_1)     /*!< Standby mode */
#define PWR_CR1_LPMS_SHUTDOWN           (PWR_CR1_LPMS_2)                      /*!< Shutdown mode */
#define PWR_SR1_SBF                     (PWR_EXTSCR_C1SBF)                    /*!< System standby flag for CPU1 */
#define PWR_SR1_SBF_CPU2                (PWR_EXTSCR_C2SBF)                    /*!< System standby flag for CPU2 */
#define PWR_CR3_EIWF_Msk                (PWR_CR3_EWUP)                        /*!< Enable all external Wake-Up lines  */
#define PWR_SCR_CSBF                    (PWR_EXTSCR_C1CSSF)                   /*!< Clear standby and stop flags for CPU1 */

#endif


/**
 * @addtogroup LIBOHIBOARD_Driver
 * @{
 */

#if defined(LIBOHIBOARD_LOWPOWER) && defined (LIBOHIBOARD_STM32L4) || defined (LIBOHIBOARD_STM32WB)

/**
 * @addtogroup LOWPOWER
 * @{
 */

/**
 * @defgroup LOWPOWER_Hardware Low Power specific hardware types
 * @{
 */

/**
 *
 */
typedef enum _LowPower_WakeUpPins
{
    LOWPOWER_WAKEUPPINS_PIN1 = PWR_CR3_EWUP1,
    LOWPOWER_WAKEUPPINS_PIN2 = PWR_CR3_EWUP2,
    LOWPOWER_WAKEUPPINS_PIN3 = PWR_CR3_EWUP3,
    LOWPOWER_WAKEUPPINS_PIN4 = PWR_CR3_EWUP4,
    LOWPOWER_WAKEUPPINS_PIN5 = PWR_CR3_EWUP5,
} LowPower_WakeUpPins;

/**
 *
 */
typedef enum _LowPower_WakeUpEdge
{
    LOWPOWER_WAKEUPEDGE_RISING       = 0u,
    LOWPOWER_WAKEUPEDGE_FALLING_PIN1 = PWR_CR4_WP1,
    LOWPOWER_WAKEUPEDGE_FALLING_PIN2 = PWR_CR4_WP2,
    LOWPOWER_WAKEUPEDGE_FALLING_PIN3 = PWR_CR4_WP3,
    LOWPOWER_WAKEUPEDGE_FALLING_PIN4 = PWR_CR4_WP4,
    LOWPOWER_WAKEUPEDGE_FALLING_PIN5 = PWR_CR4_WP5,
} LowPower_WakeUpEdge;

/** Number of possible power modes. */
#define LOWPOWER_MODE_NUMBER           9

/**
 * List of all possible Low-Power modes for this kind of microcontroller.
 */
typedef enum _LowPower_Mode
{
    LOWPOWER_MODE_RUN,
    LOWPOWER_MODE_LPRUN,
    LOWPOWER_MODE_SLEEP,
    LOWPOWER_MODE_LPSLEEP,
    LOWPOWER_MODE_STOP0,
    LOWPOWER_MODE_STOP1,
    LOWPOWER_MODE_STOP2,
    LOWPOWER_MODE_STANDBY,
    LOWPOWER_MODE_SHUTDOWN,
} LowPower_Mode;

/**
 *
 */
typedef struct _LowPower_ResetFlags
{
	uint32_t pwrWufi                     : 1;  // PWR_SR1_WUFI
	uint32_t pwrStandby                  : 1;  // PWR_SR1_SBF
	uint32_t pwrStandby_cpu2             : 1;  // PWR_SR1_SBF_CPU2

	uint32_t rccLowPowerReset            : 1;  // RCC_CSR_LPWRRSTF
	uint32_t rccWatchdogReset            : 1;  // RCC_CSR_WWDGRSTF
	uint32_t rccIndependentWatchdogReset : 1;  // RCC_CSR_IWDGRSTF
	uint32_t rccSoftwareReset            : 1;  // RCC_CSR_SFTRSTF
	uint32_t rccBOR                      : 1;  // RCC_CSR_BORRSTF
	uint32_t rccPinReset                 : 1;  // RCC_CSR_PINRSTF
	uint32_t rccOptionbyteLoaderReset    : 1;  // RCC_CSR_OBLRSTF
	uint32_t rccFirewallReset            : 1;  // RCC_CSR_FWRSTF

	uint32_t                             : 22; // Not used
} LowPower_ResetFlags;

/**
 *
 */
typedef union _LowPower_ResetControl
{
	uint32_t value;
	LowPower_ResetFlags flags;
} LowPower_ResetControl;

/**
 * @}
 */

/**
 * @}
 */

#endif // LIBOHIBOARD_LOWPOWER & LIBOHIBOARD_STM32L4 || LIBOHIBOARD_STM32WB

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif // __LOWPOWER_STM32L4_H
