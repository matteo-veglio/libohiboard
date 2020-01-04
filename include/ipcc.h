/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2014-2019 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *  Marco Giammarini <m.giammarini@warcomeb.it>
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
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
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

/** CPU1 */
#define IPCC_BLE_CMD_CHANNEL                         IPCC_CHANNEL_1
#define IPCC_SYSTEM_CMD_RSP_CHANNEL                  IPCC_CHANNEL_2
#define IPCC_THREAD_OT_CMD_RSP_CHANNEL               IPCC_CHANNEL_3
#define IPCC_MAC_802_15_4_CMD_RSP_CHANNEL            IPCC_CHANNEL_3
#define IPCC_THREAD_CLI_CMD_CHANNEL                  IPCC_CHANNEL_5
#define IPCC_MM_RELEASE_BUFFER_CHANNEL               IPCC_CHANNEL_4
#define IPCC_HCI_ACL_DATA_CHANNEL                    IPCC_CHANNEL_6

/** CPU2 */
#define IPCC_BLE_EVENT_CHANNEL                       IPCC_CHANNEL_1
#define IPCC_SYSTEM_EVENT_CHANNEL                    IPCC_CHANNEL_2
#define IPCC_THREAD_NOTIFICATION_ACK_CHANNEL         IPCC_CHANNEL_3
#define IPCC_MAC_802_15_4_NOTIFICATION_ACK_CHANNEL   IPCC_CHANNEL_3
#define IPCC_TRACES_CHANNEL                          IPCC_CHANNEL_4
#define IPCC_THREAD_CLI_NOTIFICATION_ACK_CHANNEL     IPCC_CHANNEL_5

#define Ipcc_Tx_Pending( channel ) ( !(Ipcc_Is_active_ChannelFlag( OB_IPCC, channel, IPCC_IS_ACTIVE_CHANNELFLAG_CPU1 )) ) &&  (((~(OB_IPCC->regmapCpu1->MR)) & (channel << 16U)))
#define Ipcc_Rx_Pending( channel )  (Ipcc_Is_active_ChannelFlag( OB_IPCC, channel, IPCC_IS_ACTIVE_CHANNELFLAG_CPU2 )) && (((~(OB_IPCC->regmapCpu1->MR)) & (channel << 0U)))


void IPCC_THREAD_EvtNot( void );
void IPCC_BLE_RxEvtNot( void );
void IPCC_SYS_EvtNot( void );
void IPCC_TRACES_EvtNot( void );
void IPCC_OT_CmdEvtNot( void );
void IPCC_SYS_CmdEvtNot( void );
void IPCC_BLE_AclDataAckNot( void );

typedef struct _Ipcc_Device* Ipcc_DeviceHandle;


#ifdef __cplusplus
}
#endif

#endif // __IPCC_H

#endif // LIBOHIBOARD_IPCC
