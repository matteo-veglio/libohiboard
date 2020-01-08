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
#define IPCC_RX_ISR_PENDING(CHANNEL)     ((Ipcc_isFlagActive(OB_IPCC,CHANNEL,IPCC_CPUCHANNEL_2)) && (((~(OB_IPCC->regmapCpu1->MR)) & (channel << 0U))))

// NOTA: nella riga sopra, è giusto che ci sia CHANNEL2??

/*
static inline void AHB3_GRP1_EnableClock(uint32_t Periphs)
{
  uint32_t tmpreg;
  UTILITY_SET_REGISTER_BIT(RCC->AHB3ENR, Periphs);
  /* Delay after an RCC peripheral clock enabling
  tmpreg = UTILITY_READ_REGISTER_BIT(RCC->AHB3ENR, Periphs);
  (void)tmpreg;
}*/

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
    .regboot    = PWR,

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

static inline void Ipcc_enableChannel (Ipcc_DeviceHandle dev, Ippc_Channel channel, Ipcc_DirectionChannel dir)
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

static inline void Ipcc_disableChannel (Ipcc_DeviceHandle dev, Ippc_Channel channel, Ipcc_DirectionChannel dir)
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

static inline uint32_t Ipcc_isChannelEnable (Ipcc_DeviceHandle dev, Ippc_Channel channel, Ipcc_DirectionChannel dir)
{
    switch (dir)
    {
    case IPCC_CHANNEL_IS_ENABLE_CPU1_TX:
        return ((UTILITY_READ_REGISTER_BIT(dev->regmapCpu1->MR, channel << IPCC_C1MR_CH1FM_Pos) != (channel << IPCC_C1MR_CH1FM_Pos)) ? 1UL : 0UL);
    case IPCC_CHANNEL_IS_ENABLE_CPU1_RX:
        return ((UTILITY_READ_REGISTER_BIT(dev->regmapCpu1->MR, channel) != (channel)) ? 1UL : 0UL);
    case IPCC_CHANNEL_IS_ENABLE_CPU2_TX:
        return ((UTILITY_READ_REGISTER_BIT(dev->regmapCpu2->MR, channel << IPCC_C2MR_CH1FM_Pos) != (channel << IPCC_C2MR_CH1FM_Pos)) ? 1UL : 0UL);
    case IPCC_CHANNEL_IS_ENABLE_CPU2_RX:
        return ((UTILITY_READ_REGISTER_BIT(dev->regmapCpu2->MR, channel) != (channel)) ? 1UL : 0UL);
    }
}

static inline void Ipcc_clearFlag (Ipcc_DeviceHandle dev, Ippc_Channel channel, Ipcc_CpuChannel cpu)
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

static inline void Ipcc_setFlag (Ipcc_DeviceHandle dev, Ippc_Channel channel, Ipcc_CpuChannel cpu)
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

static inline uint32_t Ipcc_isFlagActive (Ipcc_DeviceHandle dev, Ippc_Channel channel, Ipcc_CpuChannel cpu)
{
    switch (cpu)
    {
    case IPCC_CPUCHANNEL_1:
        return ((UTILITY_READ_REGISTER_BIT(dev->regmapCpu1->SR, channel) == (channel)) ? 1UL : 0UL);
    case IPCC_CPUCHANNEL_2:
        return ((UTILITY_READ_REGISTER_BIT(dev->regmapCpu2->SR, channel) == (channel)) ? 1UL : 0UL);
    }
}

/*
 * NOTA: Correggo il codestyle solo di questa, voi fate le altre.
 */
static void Ipcc_threadNotificationEventHandler (void)
{
    Ipcc_disableChannel (OB_IPCC,IPCC_THREAD_NOTIFICATION_ACK_CHANNEL,IPCC_DIRECTIONCHANNEL_CPU1_RX);

    Ipcc_ThreadNotificationEvent();

    return;
}

static void IPCC_BLE_EvtHandler( void )
{
    IPCC_BLE_RxEvtNot();

    Ipcc_clear_ChannelFlag( OB_IPCC, IPCC_BLE_EVENT_CHANNEL, IPCC_CLEAR_CHANNELFLAG_CPU1 );

    return;
}

static void IPCC_SYS_EvtHandler( void )
{
    IPCC_SYS_EvtNot();

    Ipcc_clear_ChannelFlag( OB_IPCC, IPCC_SYSTEM_EVENT_CHANNEL, IPCC_CLEAR_CHANNELFLAG_CPU1 );

    return;
}

static void IPCC_TRACES_EvtHandler( void )
{
    IPCC_TRACES_EvtNot();

    Ipcc_clear_ChannelFlag( OB_IPCC, IPCC_TRACES_CHANNEL, IPCC_CLEAR_CHANNELFLAG_CPU1 );

    return;
}

static void IPCC_THREAD_CliNotEvtHandler( void )
{
    Ipcc_disableChannel( OB_IPCC, IPCC_THREAD_CLI_NOTIFICATION_ACK_CHANNEL, IPCC_CHANNELDISABLE_CPU1_RX );

    IPCC_THREAD_CliEvtNot();

  return;
}

/*void IPCC_THREAD_CliEvtNot( void )
{
  TL_THREAD_CliNotReceived( (TL_EvtPacket_t*)(TL_RefTable.p_thread_table->clicmdrsp_buffer) );

  return;
}
__weak void TL_THREAD_CliNotReceived( TL_EvtPacket_t * Notbuffer ){}; */

static void IPCC_OT_CmdEvtHandler( void )
{
    Ipcc_disableChannel( OB_IPCC, IPCC_THREAD_OT_CMD_RSP_CHANNEL, IPCC_CHANNELDISABLE_CPU1_TX );

    IPCC_OT_CmdEvtNot();

    return;
}

static void IPCC_SYS_CmdEvtHandler( void )
{
    Ipcc_disableChannel( OB_IPCC, IPCC_SYSTEM_CMD_RSP_CHANNEL, IPCC_CHANNELDISABLE_CPU1_TX );

    IPCC_SYS_CmdEvtNot();

    return;
}

static void (*FreeBufCb)( void );

static void IPCC_MM_FreeBufHandler( void )
{
    Ipcc_disableChannel( OB_IPCC, IPCC_MM_RELEASE_BUFFER_CHANNEL, IPCC_CHANNELDISABLE_CPU1_TX );

    FreeBufCb();

    Ipcc_set_ChannelFlag( OB_IPCC, IPCC_MM_RELEASE_BUFFER_CHANNEL, IPCC_SET_CHANNELFLAG_CPU1 );

    return;
}

static void IPCC_BLE_AclDataEvtHandler( void )
{
    Ipcc_disableChannel( OB_IPCC, IPCC_HCI_ACL_DATA_CHANNEL, IPCC_CHANNELDISABLE_CPU1_TX );

    IPCC_BLE_AclDataAckNot();

  return;
}

void Ipcc_enable (Ipcc_DeviceHandle dev)
{
    (void) dev;
    IPCC_BOOT_CPU2();
}

System_Errors Ippc_init (Ipcc_DeviceHandle dev)
{
    System_Errors err = ERRORS_NO_ERROR;
    IPCC_CLOCK_ENABLE(*dev->rccRegisterPtr,dev->rccRegisterEnable);

    Ipcc_enableInterrupt(dev,IPCC_INTERRUPTENABLE_CPU1_TX);
    Ipcc_enableInterrupt(dev,IPCC_INTERRUPTENABLE_CPU1_RX );

    Interrupt_enable(INTERRUPT_IPCC_C1_TX);
    Interrupt_enable(INTERRUPT_IPCC_C1_RX);

    return err;
}

/**
 * Interrupt Handler
 * NOTA: il nome da usare qui è quello di sistema!
 */
void IPCC_C1_RX_IRQHandler (void)
{
    if (IPCC_RX_ISR_PENDING(IPCC_THREAD_NOTIFICATION_ACK_CHANNEL))
    {
        Ipcc_threadNotificationEventHandler(); // Ho fixato solo questa riga!
    }
    else if (IPCC_RX_ISR_PENDING( IPCC_BLE_EVENT_CHANNEL ))
    {
        IPCC_BLE_EvtHandler();
    }
    else if (IPCC_RX_ISR_PENDING( IPCC_SYSTEM_EVENT_CHANNEL ))
    {
        IPCC_SYS_EvtHandler();
    }
    else if (IPCC_RX_ISR_PENDING( IPCC_TRACES_CHANNEL ))
    {
        IPCC_TRACES_EvtHandler();
    }
    else if (IPCC_RX_ISR_PENDING( IPCC_THREAD_CLI_NOTIFICATION_ACK_CHANNEL ))
    {
        IPCC_THREAD_CliNotEvtHandler();
    }
}

void IPCC_C1_TX_IRQHandler (void)
{
    if (IPCC_TX_ISR_PENDING( IPCC_THREAD_OT_CMD_RSP_CHANNEL ))
    {
        IPCC_OT_CmdEvtHandler();
    }
    else if (IPCC_TX_ISR_PENDING( IPCC_SYSTEM_CMD_RSP_CHANNEL ))
    {
        IPCC_SYS_CmdEvtHandler();
    }
    else if (IPCC_TX_ISR_PENDING( IPCC_MM_RELEASE_BUFFER_CHANNEL ))
    {
        IPCC_MM_FreeBufHandler();
    }
    else if (IPCC_TX_ISR_PENDING( IPCC_HCI_ACL_DATA_CHANNEL ))
    {
        IPCC_BLE_AclDataEvtHandler();
    }
}

__weak void Ipcc_ThreadNotificationEvent (void) 
{
    // Void!
}

__weak void IPCC_BLE_RxEvtNot( void ){};
__weak void IPCC_SYS_EvtNot( void ){};
__weak void IPCC_TRACES_EvtNot( void ){};

__weak void IPCC_OT_CmdEvtNot( void ){};
__weak void IPCC_SYS_CmdEvtNot( void ){};
__weak void IPCC_BLE_AclDataAckNot( void ){};

#ifdef __cplusplus
}
#endif

#endif // LIBOHIBOARD_IPCC
