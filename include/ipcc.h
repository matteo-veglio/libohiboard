/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2020 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *  Matteo Vegliò <veglio.matteo@libero.it>
 *  Alessandro Zacchilli <a.zacchilli@outlook.com>
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
 * @author Matteo Vegliò <veglio.matteo@libero.it>
 * @author Alessandro Zacchilli <a.zacchilli@outlook.com>
 * @brief IPCC definitions and prototypes.
 */

#ifdef LIBOHIBOARD_IPCC

#ifndef __IPCC_H
#define __IPCC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"
#include "errors.h"

/**
 * IPCC Channels definition
 */
typedef enum _Ipcc_Channel
{
    IPCC_CHANNEL_1 = 0x00000001ul, /*!< IPCC Channel 1 */
    IPCC_CHANNEL_2 = 0x00000002ul, /*!< IPCC Channel 2 */
    IPCC_CHANNEL_3 = 0x00000004ul, /*!< IPCC Channel 3 */
    IPCC_CHANNEL_4 = 0x00000008ul, /*!< IPCC Channel 4 */
    IPCC_CHANNEL_5 = 0x00000010ul, /*!< IPCC Channel 5 */
    IPCC_CHANNEL_6 = 0x00000020ul, /*!< IPCC Channel 6 */
} Ipcc_Channel;

/**
 * IPCC Direction Channels list
 */
typedef enum _Ipcc_DirectionChannel
{
    IPCC_DIRECTIONCHANNEL_CPU1_TX,
    IPCC_DIRECTIONCHANNEL_CPU1_RX,
    IPCC_DIRECTIONCHANNEL_CPU2_TX,
    IPCC_DIRECTIONCHANNEL_CPU2_RX,

} Ipcc_DirectionChannel;

typedef enum _Ipcc_CpuChannel
{
    IPCC_CPUCHANNEL_1,
    IPCC_CPUCHANNEL_2,

} Ipcc_CpuChannel;

typedef struct _Ipcc_Device* Ipcc_DeviceHandle;

/**
 * TODO 
 */
void Ipcc_enable (Ipcc_DeviceHandle dev);

/**
 * TODO 
 */
System_Errors Ipcc_init (Ipcc_DeviceHandle dev, pFunc callbackTx, pFunc callbackRx);


void Ipcc_enableInterrupt (Ipcc_DeviceHandle dev, Ipcc_DirectionChannel dir);

void Ipcc_disableInterrupt (Ipcc_DeviceHandle dev, Ipcc_DirectionChannel dir);

uint32_t Ipcc_isInterruptEnable (Ipcc_DeviceHandle dev, Ipcc_DirectionChannel dir);


void Ipcc_enableChannel (Ipcc_DeviceHandle dev, Ipcc_Channel channel, Ipcc_DirectionChannel dir);

void Ipcc_disableChannel (Ipcc_DeviceHandle dev, Ipcc_Channel channel, Ipcc_DirectionChannel dir);

uint32_t Ipcc_isChannelEnable (Ipcc_DeviceHandle dev, Ipcc_Channel channel, Ipcc_DirectionChannel dir);


void Ipcc_clearFlag (Ipcc_DeviceHandle dev, Ipcc_Channel channel, Ipcc_CpuChannel cpu);

void Ipcc_setFlag (Ipcc_DeviceHandle dev, Ipcc_Channel channel, Ipcc_CpuChannel cpu);

uint32_t Ipcc_isFlagActive (Ipcc_DeviceHandle dev, Ipcc_Channel channel, Ipcc_CpuChannel cpu);


bool Ipcc_isIsrTxPending (Ipcc_DeviceHandle dev, Ipcc_Channel channel);

bool Ipcc_isIsrRxPending (Ipcc_DeviceHandle dev, Ipcc_Channel channel);

extern Ipcc_DeviceHandle OB_IPCC;

#ifdef __cplusplus
}
#endif

#endif // __IPCC_H

#endif // LIBOHIBOARD_IPCC
