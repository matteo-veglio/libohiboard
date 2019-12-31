/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2014-2019 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *  Marco Giammarini <m.giammarini@warcomeb.it>
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
 * @file libohiboard/include/ipcc.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief IPCC definitions and prototypes.
 */

#ifdef LIBOHIBOARD_IPCC

#ifndef __IPCC_H
#define __IPCC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

/**
 * IPCC Flags status
 */
#define FLAG_IPCC_C1TOC2SR_CH1F IPCC_C1TOC2SR_CH1F_Msk /*!< C1 transmit to C2 receive Channel1 status flag before masking */
#define FLAG_IPCC_C1TOC2SR_CH2F IPCC_C1TOC2SR_CH2F_Msk /*!< C1 transmit to C2 receive Channel2 status flag before masking */
#define FLAG_IPCC_C1TOC2SR_CH3F IPCC_C1TOC2SR_CH3F_Msk /*!< C1 transmit to C2 receive Channel3 status flag before masking */
#define FLAG_IPCC_C1TOC2SR_CH4F IPCC_C1TOC2SR_CH4F_Msk /*!< C1 transmit to C2 receive Channel4 status flag before masking */
#define FLAG_IPCC_C1TOC2SR_CH5F IPCC_C1TOC2SR_CH5F_Msk /*!< C1 transmit to C2 receive Channel5 status flag before masking */
#define FLAG_IPCC_C1TOC2SR_CH6F IPCC_C1TOC2SR_CH6F_Msk /*!< C1 transmit to C2 receive Channel6 status flag before masking */
#define FLAG_IPCC_C2TOC1SR_CH1F IPCC_C2TOC1SR_CH1F_Msk /*!< C2 transmit to C1 receive Channel1 status flag before masking */
#define FLAG_IPCC_C2TOC1SR_CH2F IPCC_C2TOC1SR_CH2F_Msk /*!< C2 transmit to C1 receive Channel2 status flag before masking */
#define FLAG_IPCC_C2TOC1SR_CH3F IPCC_C2TOC1SR_CH3F_Msk /*!< C2 transmit to C1 receive Channel3 status flag before masking */
#define FLAG_IPCC_C2TOC1SR_CH4F IPCC_C2TOC1SR_CH4F_Msk /*!< C2 transmit to C1 receive Channel4 status flag before masking */
#define FLAG_IPCC_C2TOC1SR_CH5F IPCC_C2TOC1SR_CH5F_Msk /*!< C2 transmit to C1 receive Channel5 status flag before masking */
#define FLAG_IPCC_C2TOC1SR_CH6F IPCC_C2TOC1SR_CH6F_Msk /*!< C2 transmit to C1 receive Channel6 status flag before masking */

/**
 * IPCC Channels definition
 */
#define IPCC_CHANNEL_1 (0x00000001U) /*!< IPCC Channel 1 */
#define IPCC_CHANNEL_2 (0x00000002U) /*!< IPCC Channel 2 */
#define IPCC_CHANNEL_3 (0x00000004U) /*!< IPCC Channel 3 */
#define IPCC_CHANNEL_4 (0x00000008U) /*!< IPCC Channel 4 */
#define IPCC_CHANNEL_5 (0x00000010U) /*!< IPCC Channel 5 */
#define IPCC_CHANNEL_6 (0x00000020U) /*!< IPCC Channel 6 */

/**
 * IPCC Interrupt Flags
 */
typedef enum _IPCC_IT_Enable
{
    IPCC_ITF_ENABLE_TX_C1,
    IPCC_ITF_ENABLE_RX_C1,
    IPCC_ITF_ENABLE_TX_C2,
    IPCC_ITF_ENABLE_RX_C2,

} IPCC_IT_Enable;

typedef enum _IPCC_IT_Disable
{
    IPCC_ITF_DISABLE_TX_C1,
    IPCC_ITF_DISABLE_RX_C1,
    IPCC_ITF_DISABLE_TX_C2,
    IPCC_ITF_DISABLE_RX_C2,

} IPCC_IT_Disable;

typedef enum _IPCC_IT_Is_Enable
{
    IPCC_ITF_IS_ENABLE_TX_C1,
    IPCC_ITF_IS_ENABLE_RX_C1,
    IPCC_ITF_IS_ENABLE_TX_C2,
    IPCC_ITF_IS_ENABLE_RX_C2,

} IPCC_IT_Is_Enable;

/**
 * IPCC Channel State
 */
typedef enum _CH_State_Enable
{
    IPCC_ENABLE_TX_CH_C1,
    IPCC_ENABLE_RX_CH_C1,
    IPCC_ENABLE_TX_CH_C2,
    IPCC_ENABLE_RX_CH_C2,

} CH_State_Enable;

typedef enum _CH_State_Disable
{
    IPCC_DISABLE_TX_CH_C1,
    IPCC_DISABLE_RX_CH_C1,
    IPCC_DISABLE_TX_CH_C2,
    IPCC_DISABLE_RX_CH_C2,

} CH_State_Disable;

typedef enum _CH_State_Is_Enable
{
    IPCC_IS_ENABLE_TX_CH_C1,
    IPCC_IS_ENABLE_RX_CH_C1,
    IPCC_IS_ENABLE_TX_CH_C2,
    IPCC_IS_ENABLE_RX_CH_C2,

} CH_State_Is_Enable;

typedef enum _IPCC_ClearFlag_CHx
{
    IPCC_CLEAR_C1,
    IPCC_CLEAR_C2,

} IPCC_ClearFlag_CHx;

typedef enum _IPCC_SetFlag_CHx
{
    IPCC_SET_C1,
    IPCC_SET_C2,

} IPCC_SetFlag_CHx;

typedef enum _IPCC_Is_ActiveFlag_CHx
{
    IPCC_IS_ACTIVE_C1,
    IPCC_IS_ACTIVE_C2,

} IPCC_Is_ActiveFlag_CHx;

#ifdef __cplusplus
}
#endif

#endif // __IPCC_H

#endif // LIBOHIBOARD_IPCC
