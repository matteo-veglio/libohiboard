/*
 * Copyright (C) 2018-2019 A. C. Open Hardware Ideas Lab
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
 * @file libohiboard/source/STM32WB/ipcc_STM32WB.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
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
    PWR_TypeDef*        regboot;

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
 * IPCC Interrupt State
 */
typedef enum _Ipcc_InterruptChannel
{
    IPCC_INTERRUPTCHANNEL_CPU1_TX,
    IPCC_INTERRUPTCHANNEL_CPU1_RX,
    IPCC_INTERRUPTCHANNEL_CPU2_TX,
    IPCC_INTERRUPTCHANNEL_CPU2_RX,

} Ipcc_InterruptChannel;

#if 0
typedef enum _Ipcc_InterruptDisable
{
    IPCC_INTERRUPTDISABLE_CPU1_TX,
    IPCC_INTERRUPTDISABLE_CPU1_RX,
    IPCC_INTERRUPTDISABLE_CPU2_TX,
    IPCC_INTERRUPTDISABLE_CPU2_RX,

} Ipcc_InterruptDisable;

typedef enum _Ipcc_Interrupt_Is_Enable
{
    IPCC_INTERRUPT_IS_ENABLE_CPU1_TX,
    IPCC_INTERRUPT_IS_ENABLE_CPU1_RX,
    IPCC_INTERRUPT_IS_ENABLE_CPU2_TX,
    IPCC_INTERRUPT_IS_ENABLE_CPU2_RX,

} Ipcc_Interrupt_Is_Enable;

/**
 * IPCC Channel State
 */
typedef enum _Ipcc_ChannelEnable
{
    IPCC_CHANNELENABLE_CPU1_TX,
    IPCC_CHANNELENABLE_CPU1_RX,
    IPCC_CHANNELENABLE_CPU2_TX,
    IPCC_CHANNELENABLE_CPU2_RX,

} Ipcc_ChannelEnable;

typedef enum _Ipcc_ChannelDisable
{
    IPCC_CHANNELDISABLE_CPU1_TX,
    IPCC_CHANNELDISABLE_CPU1_RX,
    IPCC_CHANNELDISABLE_CPU2_TX,
    IPCC_CHANNELDISABLE_CPU2_RX,

} Ipcc_ChannelDisable;

typedef enum _Ipcc_Channel_Is_Enable
{
    IPCC_CHANNEL_IS_ENABLE_CPU1_TX,
    IPCC_CHANNEL_IS_ENABLE_CPU1_RX,
    IPCC_CHANNEL_IS_ENABLE_CPU2_TX,
    IPCC_CHANNEL_IS_ENABLE_CPU2_RX,

} Ipcc_Channel_Is_Enable;
#endif

/**
 * IPCC Interrupt Flags
 */
typedef enum _Ipcc_Clear_ChannelFlag
{
    IPCC_CLEAR_CHANNELFLAG_CPU1,
    IPCC_CLEAR_CHANNELFLAG_CPU2,

} Ipcc_Clear_ChannelFlag;

typedef enum _Ipcc_Set_ChannelFlag
{
    IPCC_SET_CHANNELFLAG_CPU1,
    IPCC_SET_CHANNELFLAG_CPU2,

} Ipcc_Set_ChannelFlag;

typedef enum _Ipcc_Is_Active_ChannelFlag
{
    IPCC_IS_ACTIVE_CHANNELFLAG_CPU1,
    IPCC_IS_ACTIVE_CHANNELFLAG_CPU2,

} Ipcc_Is_Active_ChannelFlag;


static inline void Ipcc_enableInterrupt (Ipcc_DeviceHandle dev, Ipcc_InterruptChannel interrupt)
{
    switch (interrupt)
    {
    case IPCC_INTERRUPTCHANNEL_CPU1_TX:
        UTILITY_SET_REGISTER_BIT(dev->regmapCpu1->CR, IPCC_C1CR_TXFIE);
        break;
    case IPCC_INTERRUPTCHANNEL_CPU1_RX:
        UTILITY_SET_REGISTER_BIT(dev->regmapCpu1->CR, IPCC_C1CR_RXOIE);
        break;
    case IPCC_INTERRUPTCHANNEL_CPU2_TX:
        UTILITY_SET_REGISTER_BIT(dev->regmapCpu2->CR, IPCC_C2CR_TXFIE);
        break;
    case IPCC_INTERRUPTCHANNEL_CPU2_RX:
        UTILITY_SET_REGISTER_BIT(dev->regmapCpu2->CR, IPCC_C2CR_RXOIE);
        break;
    }
}

static inline void Ipcc_disableInterrupt (Ipcc_DeviceHandle dev, Ipcc_InterruptChannel interrupt)
{
    switch (interrupt)
    {
    case IPCC_INTERRUPTCHANNEL_CPU1_TX:
        UTILITY_CLEAR_REGISTER_BIT(dev->regmapCpu1->CR, IPCC_C1CR_TXFIE);
    case IPCC_INTERRUPTCHANNEL_CPU1_RX:
        UTILITY_CLEAR_REGISTER_BIT(dev->regmapCpu1->CR, IPCC_C1CR_RXOIE);
    case IPCC_INTERRUPTCHANNEL_CPU2_TX:
        UTILITY_CLEAR_REGISTER_BIT(dev->regmapCpu2->CR, IPCC_C2CR_TXFIE);
    case IPCC_INTERRUPTCHANNEL_CPU2_RX:
        UTILITY_CLEAR_REGISTER_BIT(dev->regmapCpu2->CR, IPCC_C2CR_RXOIE);
    }
}

static inline uint32_t Ipcc_isInterruptEnable (Ipcc_DeviceHandle dev, Ipcc_InterruptChannel interrupt)
{
    switch (interrupt)
    {
    case IPCC_INTERRUPTCHANNEL_CPU1_TX:
        return ((UTILITY_READ_REGISTER_BIT(dev->regmapCpu1->CR, IPCC_C1CR_TXFIE) == (IPCC_C1CR_TXFIE)) ? 1UL : 0UL);
    case IPCC_INTERRUPTCHANNEL_CPU1_RX:
        return ((UTILITY_READ_REGISTER_BIT(dev->regmapCpu1->CR, IPCC_C1CR_RXOIE) == (IPCC_C1CR_RXOIE)) ? 1UL : 0UL);
    case IPCC_INTERRUPTCHANNEL_CPU2_TX:
        return ((UTILITY_READ_REGISTER_BIT(dev->regmapCpu2->CR, IPCC_C2CR_TXFIE) == (IPCC_C2CR_TXFIE)) ? 1UL : 0UL);
    case IPCC_INTERRUPTCHANNEL_CPU2_RX:
        return ((UTILITY_READ_REGISTER_BIT(dev->regmapCpu2->CR, IPCC_C2CR_RXOIE) == (IPCC_C2CR_RXOIE)) ? 1UL : 0UL);
    }
}

static inline void Ipcc_enableChannel(Ipcc_DeviceHandle dev, uint32_t Channel, Ipcc_ChannelEnable state)
{
    switch (state)
    {
    case IPCC_CHANNELENABLE_CPU1_TX:
        UTILITY_CLEAR_REGISTER_BIT(dev->regmapCpu1->MR, Channel << IPCC_C1MR_CH1FM_Pos);
    case IPCC_CHANNELENABLE_CPU1_RX:
        UTILITY_CLEAR_REGISTER_BIT(dev->regmapCpu1->MR, Channel);
    case IPCC_CHANNELENABLE_CPU2_TX:
        UTILITY_CLEAR_REGISTER_BIT(dev->regmapCpu2->MR, Channel << IPCC_C2MR_CH1FM_Pos);
    case IPCC_CHANNELENABLE_CPU2_RX:
        UTILITY_CLEAR_REGISTER_BIT(dev->regmapCpu2->MR, Channel);
    }
}

static inline void Ipcc_disableChannel(Ipcc_DeviceHandle dev, uint32_t Channel, Ipcc_ChannelDisable state)
{
    switch (state)
    {
    case IPCC_CHANNELDISABLE_CPU1_TX:
        UTILITY_SET_REGISTER_BIT(dev->regmapCpu1->MR, Channel << IPCC_C1MR_CH1FM_Pos);
    case IPCC_CHANNELDISABLE_CPU1_RX:
        UTILITY_SET_REGISTER_BIT(dev->regmapCpu1->MR, Channel);
    case IPCC_CHANNELDISABLE_CPU2_TX:
        UTILITY_SET_REGISTER_BIT(dev->regmapCpu2->MR, Channel << (IPCC_C2MR_CH1FM_Pos));
    case IPCC_CHANNELDISABLE_CPU2_RX:
        UTILITY_SET_REGISTER_BIT(dev->regmapCpu2->MR, Channel);
    }
}

static inline uint32_t Ipcc_Is_enableChannel(Ipcc_DeviceHandle dev, uint32_t Channel, Ipcc_Channel_Is_Enable state)
{
    switch (state)
    {
    case IPCC_CHANNEL_IS_ENABLE_CPU1_TX:
        return ((UTILITY_READ_REGISTER_BIT(dev->regmapCpu1->MR, Channel << IPCC_C1MR_CH1FM_Pos) != (Channel << IPCC_C1MR_CH1FM_Pos)) ? 1UL : 0UL);
    case IPCC_CHANNEL_IS_ENABLE_CPU1_RX:
        return ((UTILITY_READ_REGISTER_BIT(dev->regmapCpu1->MR, Channel) != (Channel)) ? 1UL : 0UL);
    case IPCC_CHANNEL_IS_ENABLE_CPU2_TX:
        return ((UTILITY_READ_REGISTER_BIT(dev->regmapCpu2->MR, Channel << IPCC_C2MR_CH1FM_Pos) != (Channel << IPCC_C2MR_CH1FM_Pos)) ? 1UL : 0UL);
    case IPCC_CHANNEL_IS_ENABLE_CPU2_RX:
        return ((UTILITY_READ_REGISTER_BIT(dev->regmapCpu2->MR, Channel) != (Channel)) ? 1UL : 0UL);
    }
}

static inline void Ipcc_clear_ChannelFlag(Ipcc_DeviceHandle dev, uint32_t Channel, Ipcc_Clear_ChannelFlag flag)
{
    switch (flag)
    {
    case IPCC_CLEAR_CHANNELFLAG_CPU1:
        UTILITY_WRITE_REGISTER(dev->regmapCpu1->SCR, Channel);
    case IPCC_CLEAR_CHANNELFLAG_CPU2:
        UTILITY_WRITE_REGISTER(dev->regmapCpu2->SCR, Channel);
    }
}

static inline void Ipcc_set_ChannelFlag(Ipcc_DeviceHandle dev, uint32_t Channel, Ipcc_Set_ChannelFlag flag)
{
    switch (flag)
    {
    case IPCC_SET_CHANNELFLAG_CPU1:
        UTILITY_WRITE_REGISTER(dev->regmapCpu1->SCR, Channel << IPCC_C1SCR_CH1S_Pos);
    case IPCC_SET_CHANNELFLAG_CPU2:
        UTILITY_WRITE_REGISTER(dev->regmapCpu2->SCR, Channel << IPCC_C2SCR_CH1S_Pos);
    }
}

static inline uint32_t Ipcc_Is_active_ChannelFlag(Ipcc_DeviceHandle dev, uint32_t Channel, Ipcc_Is_Active_ChannelFlag flag)
{
    switch (flag)
    {
    case IPCC_IS_ACTIVE_CHANNELFLAG_CPU1:
        return ((UTILITY_READ_REGISTER_BIT(dev->regmapCpu1->SR, Channel) == (Channel)) ? 1UL : 0UL);
    case IPCC_IS_ACTIVE_CHANNELFLAG_CPU2:
        return ((UTILITY_READ_REGISTER_BIT(dev->regmapCpu2->SR, Channel) == (Channel)) ? 1UL : 0UL);
    }
}

static void IPCC_THREAD_NotEvtHandler( void )
{
    Ipcc_disableChannel( OB_IPCC, IPCC_THREAD_NOTIFICATION_ACK_CHANNEL, IPCC_CHANNELDISABLE_CPU1_RX );

    IPCC_THREAD_EvtNot();

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



static inline void Boot_CPU2(void)
{
    UTILITY_SET_REGISTER_BIT(OB_IPCC->regboot->CR4, PWR_CR4_C2BOOT);
}

void IPCC_Enable( void )
{
    Boot_CPU2();

    return;
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
 */
void Ipcc_Rx_Handler( void )
{
    if (Ipcc_Rx_Pending( IPCC_THREAD_NOTIFICATION_ACK_CHANNEL ))
    {
        IPCC_THREAD_NotEvtHandler();
    }
    else if (Ipcc_Rx_Pending( IPCC_BLE_EVENT_CHANNEL ))
    {
        IPCC_BLE_EvtHandler();
    }
    else if (Ipcc_Rx_Pending( IPCC_SYSTEM_EVENT_CHANNEL ))
    {
        IPCC_SYS_EvtHandler();
    }
    else if (Ipcc_Rx_Pending( IPCC_TRACES_CHANNEL ))
    {
        IPCC_TRACES_EvtHandler();
    }
  else if (Ipcc_Rx_Pending( IPCC_THREAD_CLI_NOTIFICATION_ACK_CHANNEL ))
  {
    IPCC_THREAD_CliNotEvtHandler();
  }

    return;
}

void Ipcc_Tx_Handler( void )
{
    if (Ipcc_Tx_Pending( IPCC_THREAD_OT_CMD_RSP_CHANNEL ))
    {
        IPCC_OT_CmdEvtHandler();
    }
    else if (Ipcc_Tx_Pending( IPCC_SYSTEM_CMD_RSP_CHANNEL ))
    {
        IPCC_SYS_CmdEvtHandler();
    }
    else if (Ipcc_Tx_Pending( IPCC_MM_RELEASE_BUFFER_CHANNEL ))
    {
        IPCC_MM_FreeBufHandler();
    }
    else if (Ipcc_Tx_Pending( IPCC_HCI_ACL_DATA_CHANNEL ))
    {
        IPCC_BLE_AclDataEvtHandler();
    }
    return;
}

__weak void IPCC_THREAD_EvtNot( void ){};
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
