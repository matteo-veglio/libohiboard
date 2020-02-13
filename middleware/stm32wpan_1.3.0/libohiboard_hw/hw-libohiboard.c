/*
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

#include "libohiboard.h"
#include "hw.h"
#include "mbox_def.h"

/** @defgroup IPCC_LL_EC_Channel Channel
  * @{
  */
#define LL_IPCC_CHANNEL_1 IPCC_CHANNEL_1
#define LL_IPCC_CHANNEL_2 IPCC_CHANNEL_2
#define LL_IPCC_CHANNEL_3 IPCC_CHANNEL_3
#define LL_IPCC_CHANNEL_4 IPCC_CHANNEL_4
#define LL_IPCC_CHANNEL_5 IPCC_CHANNEL_5
#define LL_IPCC_CHANNEL_6 IPCC_CHANNEL_6
/**
  * @}
  */

static void (*FreeBufCb)( void );

static void HW_IPCC_BLE_EvtHandler( void );
static void HW_IPCC_BLE_AclDataEvtHandler( void );
static void HW_IPCC_MM_FreeBufHandler( void );
static void HW_IPCC_SYS_CmdEvtHandler( void );
static void HW_IPCC_SYS_EvtHandler( void );
static void HW_IPCC_TRACES_EvtHandler( void );

#ifdef THREAD_WB
static void HW_IPCC_OT_CmdEvtHandler( void );
static void HW_IPCC_THREAD_NotEvtHandler( void );
static void HW_IPCC_THREAD_CliNotEvtHandler( void );
#endif

#ifdef MAC_802_15_4_WB
static void HW_IPCC_MAC_802_15_4_CmdEvtHandler( void );
static void HW_IPCC_MAC_802_15_4_NotEvtHandler( void );
#endif

#ifdef ZIGBEE_WB
static void HW_IPCC_ZIGBEE_CmdEvtHandler( void );
static void HW_IPCC_ZIGBEE_StackNotifEvtHandler( void );
static void HW_IPCC_ZIGBEE_CliNotifEvtHandler( void );
#endif


void HW_IPCC_Rx_Handler (void)
{
    if (Ipcc_isIsrRxPending(OB_IPCC,HW_IPCC_SYSTEM_EVENT_CHANNEL))
    {
        HW_IPCC_SYS_EvtHandler();
    }
#ifdef MAC_802_15_4_WB
    else if (Ipcc_isIsrRxPending(OB_IPCC,HW_IPCC_MAC_802_15_4_NOTIFICATION_ACK_CHANNEL))
    {
        HW_IPCC_MAC_802_15_4_NotEvtHandler();
    }
#endif /* MAC_802_15_4_WB */
#ifdef THREAD_WB
    else if (Ipcc_isIsrRxPending(OB_IPCC,HW_IPCC_THREAD_NOTIFICATION_ACK_CHANNEL))
    {
        HW_IPCC_THREAD_NotEvtHandler();
    }
    else if (Ipcc_isIsrRxPending(OB_IPCC,HW_IPCC_THREAD_CLI_NOTIFICATION_ACK_CHANNEL))
    {
        HW_IPCC_THREAD_CliNotEvtHandler();
    }
#endif /* THREAD_WB */
#ifdef ZIGBEE_WB
    else if (Ipcc_isIsrRxPending(OB_IPCC,HW_IPCC_THREAD_NOTIFICATION_ACK_CHANNEL))
    {
        HW_IPCC_ZIGBEE_StackNotifEvtHandler();
    }
    else if (Ipcc_isIsrRxPending(OB_IPCC,HW_IPCC_THREAD_CLI_NOTIFICATION_ACK_CHANNEL))
    {
        HW_IPCC_ZIGBEE_CliNotifEvtHandler();
    }
#endif /* ZIGBEE_WB */
    else if (Ipcc_isIsrRxPending(OB_IPCC,HW_IPCC_BLE_EVENT_CHANNEL))
    {
        HW_IPCC_BLE_EvtHandler();
    }
    else if (Ipcc_isIsrRxPending(OB_IPCC,HW_IPCC_TRACES_CHANNEL))
    {
        HW_IPCC_TRACES_EvtHandler();
    }

  return;
}

void HW_IPCC_Tx_Handler( void )
{
    if (Ipcc_isIsrTxPending(OB_IPCC,HW_IPCC_SYSTEM_CMD_RSP_CHANNEL))
    {
        HW_IPCC_SYS_CmdEvtHandler();
    }
#ifdef MAC_802_15_4_WB
    else if (Ipcc_isIsrTxPending(OB_IPCC,HW_IPCC_MAC_802_15_4_CMD_RSP_CHANNEL))
    {
        HW_IPCC_MAC_802_15_4_CmdEvtHandler();
    }
#endif /* MAC_802_15_4_WB */
#ifdef THREAD_WB
    else if (Ipcc_isIsrTxPending(OB_IPCC,HW_IPCC_THREAD_OT_CMD_RSP_CHANNEL))
    {
        HW_IPCC_OT_CmdEvtHandler();
    }
#endif /* THREAD_WB */
#ifdef ZIGBEE_WB
    if (Ipcc_isIsrTxPending(OB_IPCC,HW_IPCC_THREAD_OT_CMD_RSP_CHANNEL))
    {
        HW_IPCC_ZIGBEE_CmdEvtHandler();
    }
#endif /* ZIGBEE_WB */
    else if (Ipcc_isIsrTxPending(OB_IPCC,HW_IPCC_SYSTEM_CMD_RSP_CHANNEL))
    {
        HW_IPCC_SYS_CmdEvtHandler();
    }
    else if (Ipcc_isIsrTxPending(OB_IPCC,HW_IPCC_MM_RELEASE_BUFFER_CHANNEL))
    {
        HW_IPCC_MM_FreeBufHandler();
    }
    else if (Ipcc_isIsrTxPending(OB_IPCC,HW_IPCC_HCI_ACL_DATA_CHANNEL))
    {
        HW_IPCC_BLE_AclDataEvtHandler();
    }

    return;
}

void HW_IPCC_Enable (void)
{
    Ipcc_enable(OB_IPCC);
}

void HW_IPCC_Init (void)
{
    Ipcc_init(OB_IPCC,HW_IPCC_Rx_Handler,HW_IPCC_Tx_Handler);
}

/******************************************************************************
 * BLE Handler
 ******************************************************************************/
void HW_IPCC_BLE_Init( void )
{
    Ipcc_enableChannel(OB_IPCC, HW_IPCC_BLE_EVENT_CHANNEL, IPCC_DIRECTIONCHANNEL_CPU1_RX);
}

void HW_IPCC_BLE_SendCmd( void )
{
    Ipcc_setFlag(OB_IPCC, HW_IPCC_BLE_CMD_CHANNEL, IPCC_CPUCHANNEL_1);
}

static void HW_IPCC_BLE_EvtHandler( void )
{
    HW_IPCC_BLE_RxEvtNot();
    Ipcc_clearFlag(OB_IPCC, HW_IPCC_BLE_EVENT_CHANNEL, IPCC_CPUCHANNEL_1);
}

void HW_IPCC_BLE_SendAclData( void )
{
    Ipcc_setFlag(OB_IPCC, HW_IPCC_HCI_ACL_DATA_CHANNEL, IPCC_CPUCHANNEL_1);
    Ipcc_enableChannel(OB_IPCC, HW_IPCC_HCI_ACL_DATA_CHANNEL, IPCC_DIRECTIONCHANNEL_CPU1_TX);
}

static void HW_IPCC_BLE_AclDataEvtHandler( void )
{
    Ipcc_disableChannel(OB_IPCC, HW_IPCC_HCI_ACL_DATA_CHANNEL, IPCC_DIRECTIONCHANNEL_CPU1_TX);
    HW_IPCC_BLE_AclDataAckNot();
}

__weak void HW_IPCC_BLE_AclDataAckNot( void ){};
__weak void HW_IPCC_BLE_RxEvtNot( void ){};

/******************************************************************************
 * SYSTEM
 ******************************************************************************/
void HW_IPCC_SYS_Init( void )
{
    Ipcc_enableChannel(OB_IPCC,HW_IPCC_SYSTEM_EVENT_CHANNEL,IPCC_DIRECTIONCHANNEL_CPU1_RX);
}

void HW_IPCC_SYS_SendCmd( void )
{
    Ipcc_setFlag(OB_IPCC, HW_IPCC_SYSTEM_CMD_RSP_CHANNEL,IPCC_CPUCHANNEL_1);
    Ipcc_enableChannel( OB_IPCC, HW_IPCC_SYSTEM_CMD_RSP_CHANNEL,IPCC_DIRECTIONCHANNEL_CPU1_TX);
}

static void HW_IPCC_SYS_CmdEvtHandler( void )
{
    Ipcc_disableChannel(OB_IPCC,HW_IPCC_SYSTEM_CMD_RSP_CHANNEL,IPCC_DIRECTIONCHANNEL_CPU1_TX);
    HW_IPCC_SYS_CmdEvtNot();
}

static void HW_IPCC_SYS_EvtHandler( void )
{
    HW_IPCC_SYS_EvtNot();
    Ipcc_clearFlag(OB_IPCC, HW_IPCC_SYSTEM_EVENT_CHANNEL, IPCC_CPUCHANNEL_1);
}

__weak void HW_IPCC_SYS_CmdEvtNot( void ){};
__weak void HW_IPCC_SYS_EvtNot( void ){};

/******************************************************************************
 * MAC 802.15.4
 ******************************************************************************/
#ifdef MAC_802_15_4_WB
void HW_IPCC_MAC_802_15_4_Init( void )
{
    Ipcc_enableChannel(OB_IPCC,HW_IPCC_MAC_802_15_4_NOTIFICATION_ACK_CHANNEL,IPCC_DIRECTIONCHANNEL_CPU1_RX);
}

void HW_IPCC_MAC_802_15_4_SendCmd( void )
{
    Ipcc_setFlag(OB_IPCC, HW_IPCC_MAC_802_15_4_CMD_RSP_CHANNEL,IPCC_CPUCHANNEL_1);
    Ipcc_enableChannel(OB_IPCC,HW_IPCC_MAC_802_15_4_CMD_RSP_CHANNEL,IPCC_DIRECTIONCHANNEL_CPU1_TX);
}

void HW_IPCC_MAC_802_15_4_SendAck( void )
{
    Ipcc_clearFlag(OB_IPCC, HW_IPCC_MAC_802_15_4_NOTIFICATION_ACK_CHANNEL,IPCC_CPUCHANNEL_1);
    Ipcc_enableChannel(OB_IPCC,HW_IPCC_MAC_802_15_4_NOTIFICATION_ACK_CHANNEL,IPCC_DIRECTIONCHANNEL_CPU1_RX);
}

static void HW_IPCC_MAC_802_15_4_CmdEvtHandler( void )
{
    Ipcc_disableChannel(OB_IPCC,HW_IPCC_MAC_802_15_4_CMD_RSP_CHANNEL,IPCC_DIRECTIONCHANNEL_CPU1_TX);
    HW_IPCC_MAC_802_15_4_CmdEvtNot();
}

static void HW_IPCC_MAC_802_15_4_NotEvtHandler( void )
{
    Ipcc_disableChannel(OB_IPCC,HW_IPCC_MAC_802_15_4_NOTIFICATION_ACK_CHANNEL,IPCC_DIRECTIONCHANNEL_CPU1_RX);
    HW_IPCC_MAC_802_15_4_EvtNot();
}

__weak void HW_IPCC_MAC_802_15_4_CmdEvtNot( void ){};
__weak void HW_IPCC_MAC_802_15_4_EvtNot( void ){};

#endif

/******************************************************************************
 * THREAD
 ******************************************************************************/
#ifdef THREAD_WB
void HW_IPCC_THREAD_Init( void )
{
    Ipcc_enableChannel(OB_IPCC,HW_IPCC_THREAD_NOTIFICATION_ACK_CHANNEL,IPCC_DIRECTIONCHANNEL_CPU1_RX);
    Ipcc_enableChannel(OB_IPCC,HW_IPCC_THREAD_CLI_NOTIFICATION_ACK_CHANNEL,IPCC_DIRECTIONCHANNEL_CPU1_RX);
}

void HW_IPCC_OT_SendCmd( void )
{
    Ipcc_setFlag(OB_IPCC, HW_IPCC_THREAD_OT_CMD_RSP_CHANNEL,IPCC_CPUCHANNEL_1);
    Ipcc_enableChannel(OB_IPCC,HW_IPCC_THREAD_OT_CMD_RSP_CHANNEL,IPCC_DIRECTIONCHANNEL_CPU1_TX);
}

void HW_IPCC_CLI_SendCmd( void )
{
    Ipcc_setFlag(OB_IPCC, HW_IPCC_THREAD_CLI_CMD_CHANNEL,IPCC_CPUCHANNEL_1);
}

void HW_IPCC_THREAD_SendAck( void )
{
    Ipcc_clearFlag(OB_IPCC, HW_IPCC_THREAD_NOTIFICATION_ACK_CHANNEL,IPCC_CPUCHANNEL_1);
    Ipcc_enableChannel(OB_IPCC,HW_IPCC_THREAD_NOTIFICATION_ACK_CHANNEL,IPCC_DIRECTIONCHANNEL_CPU1_RX);
}

void HW_IPCC_THREAD_CliSendAck( void )
{
    Ipcc_clearFlag(OB_IPCC, HW_IPCC_THREAD_CLI_NOTIFICATION_ACK_CHANNEL,IPCC_CPUCHANNEL_1);
    Ipcc_enableChannel(OB_IPCC,HW_IPCC_THREAD_CLI_NOTIFICATION_ACK_CHANNEL,IPCC_DIRECTIONCHANNEL_CPU1_RX);
}

static void HW_IPCC_OT_CmdEvtHandler( void )
{
    Ipcc_disableChannel(OB_IPCC,HW_IPCC_THREAD_OT_CMD_RSP_CHANNEL,IPCC_DIRECTIONCHANNEL_CPU1_TX);
    HW_IPCC_OT_CmdEvtNot();
}

static void HW_IPCC_THREAD_NotEvtHandler( void )
{
    Ipcc_disableChannel(OB_IPCC,HW_IPCC_THREAD_NOTIFICATION_ACK_CHANNEL,IPCC_DIRECTIONCHANNEL_CPU1_RX);
    HW_IPCC_THREAD_EvtNot();
}

static void HW_IPCC_THREAD_CliNotEvtHandler( void )
{
    Ipcc_disableChannel(OB_IPCC,HW_IPCC_THREAD_CLI_NOTIFICATION_ACK_CHANNEL,IPCC_DIRECTIONCHANNEL_CPU1_RX);
    HW_IPCC_THREAD_CliEvtNot();
}

__weak void HW_IPCC_OT_CmdEvtNot( void ){};
__weak void HW_IPCC_CLI_CmdEvtNot( void ){};
__weak void HW_IPCC_THREAD_EvtNot( void ){};

#endif /* THREAD_WB */

/******************************************************************************
 * ZIGBEE
 ******************************************************************************/
#ifdef ZIGBEE_WB
void HW_IPCC_ZIGBEE_Init( void )
{
    Ipcc_enableChannel(OB_IPCC,HW_IPCC_THREAD_NOTIFICATION_ACK_CHANNEL,IPCC_DIRECTIONCHANNEL_CPU1_RX);
    Ipcc_enableChannel(OB_IPCC,HW_IPCC_THREAD_CLI_NOTIFICATION_ACK_CHANNEL,IPCC_DIRECTIONCHANNEL_CPU1_RX);
}

void HW_IPCC_ZIGBEE_SendAppliCmd( void )
{
    Ipcc_setFlag(OB_IPCC, HW_IPCC_THREAD_OT_CMD_RSP_CHANNEL,IPCC_CPUCHANNEL_1);
    Ipcc_enableChannel(OB_IPCC,HW_IPCC_THREAD_OT_CMD_RSP_CHANNEL,IPCC_DIRECTIONCHANNEL_CPU1_TX);
}

void HW_IPCC_ZIGBEE_SendCliCmd( void )
{
    Ipcc_setFlag(OB_IPCC, HW_IPCC_THREAD_CLI_CMD_CHANNEL,IPCC_CPUCHANNEL_1);
}

void HW_IPCC_ZIGBEE_SendAppliCmdAck( void )
{
    Ipcc_clearFlag(OB_IPCC, HW_IPCC_THREAD_NOTIFICATION_ACK_CHANNEL,IPCC_CPUCHANNEL_1);
    Ipcc_enableChannel(OB_IPCC,HW_IPCC_THREAD_NOTIFICATION_ACK_CHANNEL,IPCC_DIRECTIONCHANNEL_CPU1_RX);
}

void HW_IPCC_ZIGBEE_SendCliCmdAck( void )
{
    Ipcc_clearFlag(OB_IPCC, HW_IPCC_THREAD_CLI_NOTIFICATION_ACK_CHANNEL,IPCC_CPUCHANNEL_1);
    Ipcc_enableChannel(OB_IPCC,HW_IPCC_THREAD_CLI_NOTIFICATION_ACK_CHANNEL,IPCC_DIRECTIONCHANNEL_CPU1_RX);
}

static void HW_IPCC_ZIGBEE_CmdEvtHandler( void )
{
    Ipcc_disableChannel(OB_IPCC,HW_IPCC_THREAD_OT_CMD_RSP_CHANNEL,IPCC_DIRECTIONCHANNEL_CPU1_TX);
    HW_IPCC_ZIGBEE_AppliCmdNotification();
}

static void HW_IPCC_ZIGBEE_StackNotifEvtHandler( void )
{
    Ipcc_disableChannel(OB_IPCC,HW_IPCC_THREAD_NOTIFICATION_ACK_CHANNEL,IPCC_DIRECTIONCHANNEL_CPU1_RX);
    HW_IPCC_ZIGBEE_AppliAsyncEvtNotification();
}

static void HW_IPCC_ZIGBEE_CliNotifEvtHandler( void )
{
    Ipcc_disableChannel(OB_IPCC,HW_IPCC_THREAD_CLI_NOTIFICATION_ACK_CHANNEL,IPCC_DIRECTIONCHANNEL_CPU1_RX);
    HW_IPCC_ZIGBEE_CliEvtNotification();
}

__weak void HW_IPCC_ZIGBEE_AppliCmdNotification( void ){};
__weak void HW_IPCC_ZIGBEE_AppliAsyncEvtNotification( void ){};
__weak void HW_IPCC_ZIGBEE_CliEvtNotification( void ){};
#endif /* ZIGBEE_WB */

/******************************************************************************
 * MEMORY MANAGER
 ******************************************************************************/
void HW_IPCC_MM_SendFreeBuf( void (*cb)( void ) )
{
    if (Ipcc_isFlagActive(OB_IPCC,HW_IPCC_MM_RELEASE_BUFFER_CHANNEL,IPCC_CPUCHANNEL_1))
    {
        FreeBufCb = cb;
        Ipcc_enableChannel(OB_IPCC,HW_IPCC_MM_RELEASE_BUFFER_CHANNEL,IPCC_DIRECTIONCHANNEL_CPU1_TX);
    }
    else
    {
        cb();
        Ipcc_setFlag(OB_IPCC, HW_IPCC_MM_RELEASE_BUFFER_CHANNEL,IPCC_CPUCHANNEL_1);
    }
}

static void HW_IPCC_MM_FreeBufHandler( void )
{
    Ipcc_disableChannel(OB_IPCC,HW_IPCC_MM_RELEASE_BUFFER_CHANNEL,IPCC_DIRECTIONCHANNEL_CPU1_TX);
    FreeBufCb();

    Ipcc_setFlag(OB_IPCC, HW_IPCC_MM_RELEASE_BUFFER_CHANNEL,IPCC_CPUCHANNEL_1);
}

/******************************************************************************
 * TRACES
 ******************************************************************************/
void HW_IPCC_TRACES_Init( void )
{
    Ipcc_enableChannel(OB_IPCC,HW_IPCC_TRACES_CHANNEL,IPCC_DIRECTIONCHANNEL_CPU1_RX);
}

static void HW_IPCC_TRACES_EvtHandler( void )
{
    HW_IPCC_TRACES_EvtNot();
    Ipcc_clearFlag(OB_IPCC, HW_IPCC_TRACES_CHANNEL,IPCC_CPUCHANNEL_1);
}

__weak void HW_IPCC_TRACES_EvtNot( void ){};
