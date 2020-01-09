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

/**
 * @file libohiboard/source/STM32WB/ipcc_STM32WB.c
 * @author Matteo Vegliò <veglio.matteo@libero.it>
 * @author Alessandro Zacchilli <a.zacchilli@outlook.com>
 * @brief IPCC implementations for STM32WB Series
 */

#if defined (LIBOHIBOARD_IPCC)

#ifdef __cplusplus
extern "C" {
#endif

#include "ipcc.h"

#include "platforms.h"
#include "interrupt.h"
#include "utility.h"

#define IPCC_CLOCK_ENABLE(REG,MASK)      do { \
                                            UTILITY_SET_REGISTER_BIT(REG,MASK); \
                                            asm("nop"); \
                                            (void) UTILITY_READ_REGISTER_BIT(REG,MASK); \
                                         } while (0)

#define IPCC_BOOT_CPU2()                 do { \
                                            UTILITY_SET_REGISTER_BIT(OB_IPCC->regmapPWR->CR4, PWR_CR4_C2BOOT); \
                                         } while (0)

#define IPCC_TX_ISR_PENDING(CHANNEL)     ((!(Ipcc_isFlagActive(OB_IPCC,CHANNEL,IPCC_CPUCHANNEL_1))) && (((~(OB_IPCC->regmapCpu1->MR)) & (CHANNEL << 16ul))))
#define IPCC_RX_ISR_PENDING(CHANNEL)     ((Ipcc_isFlagActive(OB_IPCC,CHANNEL,IPCC_CPUCHANNEL_2)) && (((~(OB_IPCC->regmapCpu1->MR)) & (CHANNEL << 0U))))

typedef struct _Ipcc_Device
{
    IPCC_TypeDef*       regmap;
    IPCC_CommonTypeDef* regmapCpu1;
    IPCC_CommonTypeDef* regmapCpu2;
    PWR_TypeDef*        regmapPWR;

    volatile uint32_t* rccRegisterPtr;      /**< Register for clock enabling. */
    uint32_t rccRegisterEnable;        /**< Register mask for current device. */
} Ippc_Device;

static Ippc_Device ipcc1 =
{
    .regmap     = IPCC,
    .regmapCpu1 = IPCC_C1,
    .regmapCpu2 = IPCC_C2,
    .regmapPWR  = PWR,

    .rccRegisterPtr      = &RCC->AHB3ENR,
    .rccRegisterEnable   = RCC_AHB3ENR_IPCCEN,
};
Ipcc_DeviceHandle OB_IPCC = &ipcc1;

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

static inline void Ipcc_enableInterrupt (Ipcc_DeviceHandle dev, Ipcc_DirectionChannel dir)
{
    switch (dir)
    {
    case IPCC_DIRECTIONCHANNEL_CPU1_TX:
        UTILITY_SET_REGISTER_BIT(dev->regmapCpu1->CR, IPCC_C1CR_TXFIE);
        break;
    case IPCC_DIRECTIONCHANNEL_CPU1_RX:
        UTILITY_SET_REGISTER_BIT(dev->regmapCpu1->CR, IPCC_C1CR_RXOIE);
        break;
    case IPCC_DIRECTIONCHANNEL_CPU2_TX:
        UTILITY_SET_REGISTER_BIT(dev->regmapCpu2->CR, IPCC_C2CR_TXFIE);
        break;
    case IPCC_DIRECTIONCHANNEL_CPU2_RX:
        UTILITY_SET_REGISTER_BIT(dev->regmapCpu2->CR, IPCC_C2CR_RXOIE);
        break;
    }
}

static inline void Ipcc_disableInterrupt (Ipcc_DeviceHandle dev, Ipcc_DirectionChannel dir)
{
    switch (dir)
    {
    case IPCC_DIRECTIONCHANNEL_CPU1_TX:
        UTILITY_CLEAR_REGISTER_BIT(dev->regmapCpu1->CR, IPCC_C1CR_TXFIE);
        break;
    case IPCC_DIRECTIONCHANNEL_CPU1_RX:
        UTILITY_CLEAR_REGISTER_BIT(dev->regmapCpu1->CR, IPCC_C1CR_RXOIE);
        break;
    case IPCC_DIRECTIONCHANNEL_CPU2_TX:
        UTILITY_CLEAR_REGISTER_BIT(dev->regmapCpu2->CR, IPCC_C2CR_TXFIE);
        break;
    case IPCC_DIRECTIONCHANNEL_CPU2_RX:
        UTILITY_CLEAR_REGISTER_BIT(dev->regmapCpu2->CR, IPCC_C2CR_RXOIE);
        break;
    }
}

static inline uint32_t Ipcc_isInterruptEnable (Ipcc_DeviceHandle dev, Ipcc_DirectionChannel dir)
{
    switch (dir)
    {
    case IPCC_DIRECTIONCHANNEL_CPU1_TX:
        return ((UTILITY_READ_REGISTER_BIT(dev->regmapCpu1->CR, IPCC_C1CR_TXFIE) == (IPCC_C1CR_TXFIE)) ? 1ul : 0ul);
    case IPCC_DIRECTIONCHANNEL_CPU1_RX:
        return ((UTILITY_READ_REGISTER_BIT(dev->regmapCpu1->CR, IPCC_C1CR_RXOIE) == (IPCC_C1CR_RXOIE)) ? 1ul : 0ul);
    case IPCC_DIRECTIONCHANNEL_CPU2_TX:
        return ((UTILITY_READ_REGISTER_BIT(dev->regmapCpu2->CR, IPCC_C2CR_TXFIE) == (IPCC_C2CR_TXFIE)) ? 1ul : 0ul);
    case IPCC_DIRECTIONCHANNEL_CPU2_RX:
        return ((UTILITY_READ_REGISTER_BIT(dev->regmapCpu2->CR, IPCC_C2CR_RXOIE) == (IPCC_C2CR_RXOIE)) ? 1ul : 0ul);
    }
}

static inline void Ipcc_enableChannel (Ipcc_DeviceHandle dev, uint32_t channel, Ipcc_DirectionChannel dir)
{
    switch (dir)
    {
    case IPCC_DIRECTIONCHANNEL_CPU1_TX:
        UTILITY_CLEAR_REGISTER_BIT(dev->regmapCpu1->MR, channel << IPCC_C1MR_CH1FM_Pos);
        break;
    case IPCC_DIRECTIONCHANNEL_CPU1_RX:
        UTILITY_CLEAR_REGISTER_BIT(dev->regmapCpu1->MR, channel);
        break;
    case IPCC_DIRECTIONCHANNEL_CPU2_TX:
        UTILITY_CLEAR_REGISTER_BIT(dev->regmapCpu2->MR, channel << IPCC_C2MR_CH1FM_Pos);
        break;
    case IPCC_DIRECTIONCHANNEL_CPU2_RX:
        UTILITY_CLEAR_REGISTER_BIT(dev->regmapCpu2->MR, channel);
        break;
    }
}

static inline void Ipcc_disableChannel (Ipcc_DeviceHandle dev, uint32_t channel, Ipcc_DirectionChannel dir)
{
    switch (dir)
    {
    case IPCC_DIRECTIONCHANNEL_CPU1_TX:
        UTILITY_SET_REGISTER_BIT(dev->regmapCpu1->MR, channel << IPCC_C1MR_CH1FM_Pos);
        break;
    case IPCC_DIRECTIONCHANNEL_CPU1_RX:
        UTILITY_SET_REGISTER_BIT(dev->regmapCpu1->MR, channel);
        break;
    case IPCC_DIRECTIONCHANNEL_CPU2_TX:
        UTILITY_SET_REGISTER_BIT(dev->regmapCpu2->MR, channel << (IPCC_C2MR_CH1FM_Pos));
        break;
    case IPCC_DIRECTIONCHANNEL_CPU2_RX:
        UTILITY_SET_REGISTER_BIT(dev->regmapCpu2->MR, channel);
        break;
    }
}

static inline uint32_t Ipcc_isChannelEnable (Ipcc_DeviceHandle dev, uint32_t channel, Ipcc_DirectionChannel dir)
{
    switch (dir)
    {
    case IPCC_DIRECTIONCHANNEL_CPU1_TX:
        return ((UTILITY_READ_REGISTER_BIT(dev->regmapCpu1->MR, channel << IPCC_C1MR_CH1FM_Pos) != (channel << IPCC_C1MR_CH1FM_Pos)) ? 1UL : 0UL);
    case IPCC_DIRECTIONCHANNEL_CPU1_RX:
        return ((UTILITY_READ_REGISTER_BIT(dev->regmapCpu1->MR, channel) != (channel)) ? 1UL : 0UL);
    case IPCC_DIRECTIONCHANNEL_CPU2_TX:
        return ((UTILITY_READ_REGISTER_BIT(dev->regmapCpu2->MR, channel << IPCC_C2MR_CH1FM_Pos) != (channel << IPCC_C2MR_CH1FM_Pos)) ? 1UL : 0UL);
    case IPCC_DIRECTIONCHANNEL_CPU2_RX:
        return ((UTILITY_READ_REGISTER_BIT(dev->regmapCpu2->MR, channel) != (channel)) ? 1UL : 0UL);
    }
}

static inline void Ipcc_clearFlag (Ipcc_DeviceHandle dev, uint32_t channel, Ipcc_CpuChannel cpu)
{
    switch (cpu)
    {
    case IPCC_CPUCHANNEL_1:
        UTILITY_WRITE_REGISTER(dev->regmapCpu1->SCR, channel);
        break;
    case IPCC_CPUCHANNEL_2:
        UTILITY_WRITE_REGISTER(dev->regmapCpu2->SCR, channel);
        break;
    }
}

static inline void Ipcc_setFlag (Ipcc_DeviceHandle dev, uint32_t channel, Ipcc_CpuChannel cpu)
{
    switch (cpu)
    {
    case IPCC_CPUCHANNEL_1:
        UTILITY_WRITE_REGISTER(dev->regmapCpu1->SCR, channel << IPCC_C1SCR_CH1S_Pos);
        break;
    case IPCC_CPUCHANNEL_2:
        UTILITY_WRITE_REGISTER(dev->regmapCpu2->SCR, channel << IPCC_C2SCR_CH1S_Pos);
        break;
    }
}

static inline uint32_t Ipcc_isFlagActive (Ipcc_DeviceHandle dev, uint32_t channel, Ipcc_CpuChannel cpu)
{
    switch (cpu)
    {
    case IPCC_CPUCHANNEL_1:
        return ((UTILITY_READ_REGISTER_BIT(dev->regmapCpu1->SR, channel) == (channel)) ? 1UL : 0UL);
    case IPCC_CPUCHANNEL_2:
        return ((UTILITY_READ_REGISTER_BIT(dev->regmapCpu2->SR, channel) == (channel)) ? 1UL : 0UL);
    }
}

static void Ipcc_threadNotificationEventHandler (void)
{
    Ipcc_disableChannel (OB_IPCC,IPCC_THREAD_NOTIFICATION_ACK_CHANNEL,IPCC_DIRECTIONCHANNEL_CPU1_RX);

    Ipcc_threadNotificationEvent();

    return;
}

static void Ipcc_bleNotificationEventHandler( void )
{
    Ipcc_bleRxNotificationEvent();

    Ipcc_clearFlag( OB_IPCC, IPCC_BLE_EVENT_CHANNEL, IPCC_CPUCHANNEL_1 );

    return;
}

static void Ipcc_sysNotificationEventHandler( void )
{
    Ipcc_sysNotificationEvent();

    Ipcc_clearFlag( OB_IPCC, IPCC_SYSTEM_EVENT_CHANNEL, IPCC_CPUCHANNEL_1 );

    return;
}

static void Ipcc_tracesNotificationEventHandler( void )
{
    Ipcc_tracesNotificationEvent();

    Ipcc_clearFlag( OB_IPCC, IPCC_TRACES_CHANNEL, IPCC_CPUCHANNEL_1 );

    return;
}

static void Ipcc_threadCliNotificationEventHandler( void )
{
    Ipcc_disableChannel( OB_IPCC, IPCC_THREAD_CLI_NOTIFICATION_ACK_CHANNEL, IPCC_DIRECTIONCHANNEL_CPU1_RX );

    Ipcc_threadCliNotificationEvent();

  return;
}

/*void Ipcc_threadClientNotificationEvent( void )
{
  TL_THREAD_CliNotReceived( (TL_EvtPacket_t*)(TL_RefTable.p_thread_table->clicmdrsp_buffer) );

  return;
}
__weak void TL_THREAD_CliNotReceived( TL_EvtPacket_t * Notbuffer ){}; */

static void Ipcc_otCommandNotificationEventHandler( void )
{
    Ipcc_disableChannel( OB_IPCC, IPCC_THREAD_OT_CMD_RSP_CHANNEL, IPCC_DIRECTIONCHANNEL_CPU1_TX );

    Ipcc_otCommandNotificationEvent();

    return;
}

static void Ipcc_sysCommandNotificationEventHandler( void )
{
    Ipcc_disableChannel( OB_IPCC, IPCC_SYSTEM_CMD_RSP_CHANNEL, IPCC_DIRECTIONCHANNEL_CPU1_TX );

    Ipcc_sysCommandNotificationEvent();

    return;
}

static void (*FreeBufferCb)( void );

static void Ipcc_memoryFreeBufferHandler( void )
{
    Ipcc_disableChannel( OB_IPCC, IPCC_MM_RELEASE_BUFFER_CHANNEL, IPCC_DIRECTIONCHANNEL_CPU1_TX );

    FreeBufferCb();

    Ipcc_setFlag( OB_IPCC, IPCC_MM_RELEASE_BUFFER_CHANNEL, IPCC_CPUCHANNEL_1 );

    return;
}

static void Ipcc_bleAclDataNotificationEventHandler( void )
{
    Ipcc_disableChannel( OB_IPCC, IPCC_HCI_ACL_DATA_CHANNEL, IPCC_DIRECTIONCHANNEL_CPU1_TX );

    Ipcc_bleAclDataAckNotificationEvent();

  return;
}

void Ipcc_enable (Ipcc_DeviceHandle dev)
{
    (void) dev;
    IPCC_BOOT_CPU2();
}

System_Errors Ipcc_init (Ipcc_DeviceHandle dev)
{
    System_Errors err = ERRORS_NO_ERROR;
    IPCC_CLOCK_ENABLE(*dev->rccRegisterPtr,dev->rccRegisterEnable);

    Ipcc_enableInterrupt(dev,IPCC_DIRECTIONCHANNEL_CPU1_TX);
    Ipcc_enableInterrupt(dev,IPCC_DIRECTIONCHANNEL_CPU1_RX );

    Interrupt_enable(INTERRUPT_IPCC_C1_TX);
    Interrupt_enable(INTERRUPT_IPCC_C1_RX);

    return err;
}

void HW_IPCC_Rx_Handler (void)
{
    if (IPCC_RX_ISR_PENDING(IPCC_THREAD_NOTIFICATION_ACK_CHANNEL))
    {
        Ipcc_threadNotificationEventHandler();
    }
    else if (IPCC_RX_ISR_PENDING( IPCC_BLE_EVENT_CHANNEL ))
    {
        Ipcc_bleNotificationEventHandler();
    }
    else if (IPCC_RX_ISR_PENDING( IPCC_SYSTEM_EVENT_CHANNEL ))
    {
        Ipcc_sysNotificationEventHandler();
    }
    else if (IPCC_RX_ISR_PENDING( IPCC_TRACES_CHANNEL ))
    {
        Ipcc_tracesNotificationEventHandler();
    }
    else if (IPCC_RX_ISR_PENDING( IPCC_THREAD_CLI_NOTIFICATION_ACK_CHANNEL ))
    {
        Ipcc_threadCliNotificationEventHandler();
    }
}

void HW_IPCC_Tx_Handler (void)
{
    if (IPCC_TX_ISR_PENDING( IPCC_THREAD_OT_CMD_RSP_CHANNEL ))
    {
        Ipcc_otCommandNotificationEventHandler();
    }
    else if (IPCC_TX_ISR_PENDING( IPCC_SYSTEM_CMD_RSP_CHANNEL ))
    {
        Ipcc_sysCommandNotificationEventHandler();
    }
    else if (IPCC_TX_ISR_PENDING( IPCC_MM_RELEASE_BUFFER_CHANNEL ))
    {
        Ipcc_memoryFreeBufferHandler();
    }
    else if (IPCC_TX_ISR_PENDING( IPCC_HCI_ACL_DATA_CHANNEL ))
    {
        Ipcc_bleAclDataNotificationEventHandler();
    }
}

/*
 * BLE
 */
void Ipcc_ble_init( void )
{
    Ipcc_enableChannel (OB_IPCC, IPCC_BLE_EVENT_CHANNEL, IPCC_DIRECTIONCHANNEL_CPU1_RX);

    return;
}

void Ipcc_ble_sendCommand( void )
{
    Ipcc_setFlag (OB_IPCC, IPCC_BLE_CMD_CHANNEL, IPCC_CPUCHANNEL_1);

    return;
}

void Ipcc_ble_sendAclData( void )
{
    Ipcc_setFlag( OB_IPCC, IPCC_HCI_ACL_DATA_CHANNEL, IPCC_CPUCHANNEL_1 );
    Ipcc_enableChannel( OB_IPCC, IPCC_HCI_ACL_DATA_CHANNEL, IPCC_DIRECTIONCHANNEL_CPU1_TX );

  return;
}

/*
 * SYSTEM
 */
void Ipcc_system_init( void )
{
    Ipcc_enableChannel (OB_IPCC, IPCC_SYSTEM_EVENT_CHANNEL, IPCC_DIRECTIONCHANNEL_CPU1_RX);

    return;
}

void Ipcc_system_sendCommand( void )
{
    Ipcc_setFlag( OB_IPCC, IPCC_SYSTEM_CMD_RSP_CHANNEL, IPCC_CPUCHANNEL_1 );
    Ipcc_enableChannel( OB_IPCC, IPCC_SYSTEM_CMD_RSP_CHANNEL, IPCC_DIRECTIONCHANNEL_CPU1_TX );

    return;
}

void Ipcc_thread_init( void )
{
    Ipcc_enableChannel( OB_IPCC, IPCC_THREAD_NOTIFICATION_ACK_CHANNEL, IPCC_DIRECTIONCHANNEL_CPU1_RX );
    Ipcc_enableChannel( OB_IPCC, IPCC_THREAD_CLI_NOTIFICATION_ACK_CHANNEL, IPCC_DIRECTIONCHANNEL_CPU1_RX );

    return;
}

void Ipcc_ot_sendCommand( void )
{
    Ipcc_setFlag( OB_IPCC, IPCC_THREAD_OT_CMD_RSP_CHANNEL, IPCC_CPUCHANNEL_1 );
    Ipcc_enableChannel( OB_IPCC, IPCC_THREAD_OT_CMD_RSP_CHANNEL, IPCC_DIRECTIONCHANNEL_CPU1_TX );

    return;
}

void Ipcc_cli_sendCommand( void )
{
    Ipcc_setFlag( OB_IPCC, IPCC_THREAD_CLI_CMD_CHANNEL, IPCC_CPUCHANNEL_1 );

    return;
}

void Ipcc_thread_sendAck( void )
{
    Ipcc_clearFlag( OB_IPCC, IPCC_THREAD_NOTIFICATION_ACK_CHANNEL, IPCC_CPUCHANNEL_1 );
    Ipcc_enableChannel( OB_IPCC, IPCC_THREAD_NOTIFICATION_ACK_CHANNEL, IPCC_DIRECTIONCHANNEL_CPU1_RX );

    return;
}

void Ipcc_thread_cliSendAck( void )
{
    Ipcc_clearFlag( OB_IPCC, IPCC_THREAD_CLI_NOTIFICATION_ACK_CHANNEL, IPCC_CPUCHANNEL_1 );
    Ipcc_enableChannel( OB_IPCC, IPCC_THREAD_CLI_NOTIFICATION_ACK_CHANNEL, IPCC_DIRECTIONCHANNEL_CPU1_RX );

  return;
}

/*
 * MEMORY
 */
void Ipcc_memory_sendFreeBuffer( void (*cb)( void ) )
{
    if ( Ipcc_isFlagActive( OB_IPCC, IPCC_MM_RELEASE_BUFFER_CHANNEL, IPCC_CPUCHANNEL_1 ) )
    {
        FreeBufferCb = cb;
        Ipcc_enableChannel( OB_IPCC, IPCC_MM_RELEASE_BUFFER_CHANNEL, IPCC_DIRECTIONCHANNEL_CPU1_TX );
    }
    else
    {
        cb();

        Ipcc_setFlag( OB_IPCC, IPCC_MM_RELEASE_BUFFER_CHANNEL, IPCC_CPUCHANNEL_1 );
    }

    return;
}

/*
 * TRACES
 */
void Ipcc_traces_init( void )
{
    Ipcc_enableChannel( OB_IPCC, IPCC_TRACES_CHANNEL, IPCC_DIRECTIONCHANNEL_CPU1_RX );

    return;
}

__weak void Ipcc_threadNotificationEvent (void){};
__weak void Ipcc_bleRxNotificationEvent( void ){};
__weak void Ipcc_sysNotificationEvent( void ){};
__weak void Ipcc_tracesNotificationEvent( void ){};
__weak void Ipcc_otCommandNotificationEvent( void ){};
__weak void Ipcc_sysCommandNotificationEvent( void ){};
__weak void Ipcc_bleAclDataAckNotificationEvent( void ){};
__weak void Ipcc_threadCliNotificationEvent( void ){};
__weak void Ipcc_cli_commandNotificationEvent( void ){};

#ifdef __cplusplus
}
#endif

#endif // LIBOHIBOARD_IPCC
