/*
 * Copyright (C) 2018-2019 A. C. Open Hardware Ideas Lab
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
 * @file libohiboard/source/STM32WB/ipcc_STM32WB.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
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

static inline void PWR_EnableBootC2(void)
{
    UTILITY_SET_REGISTER_BIT(PWR->CR4, PWR_CR4_C2BOOT);
}

static inline void AHB3_GRP1_EnableClock(uint32_t Periphs)
{
  uint32_t tmpreg;
  UTILITY_SET_REGISTER_BIT(RCC->AHB3ENR, Periphs);
  /* Delay after an RCC peripheral clock enabling */
  tmpreg = UTILITY_READ_REGISTER_BIT(RCC->AHB3ENR, Periphs);
  (void)tmpreg;
}





static inline void IPCC_ITF_ENABLE(IPCC_TypeDef *ipcc, IPCC_IT_Enable IT_On)
{
    switch (IT_On)
    {
    case IPCC_ITF_ENABLE_TX_C1:
        UTILITY_SET_REGISTER_BIT(ipcc->C1CR, IPCC_C1CR_TXFIE);
    case IPCC_ITF_ENABLE_RX_C1:
        UTILITY_SET_REGISTER_BIT(ipcc->C1CR, IPCC_C1CR_RXOIE);
    case IPCC_ITF_ENABLE_TX_C2:
        UTILITY_SET_REGISTER_BIT(ipcc->C2CR, IPCC_C2CR_TXFIE);
    case IPCC_ITF_ENABLE_RX_C2:
        UTILITY_SET_REGISTER_BIT(ipcc->C2CR, IPCC_C2CR_RXOIE);
    }
}

static inline void IPCC_ITF_DISABLE(IPCC_TypeDef *ipcc, IPCC_IT_Disable IT_Off)
{
    switch (IT_Off)
    {
    case IPCC_ITF_DISABLE_TX_C1:
        UTILITY_CLEAR_REGISTER_BIT(ipcc->C1CR, IPCC_C1CR_TXFIE);
    case IPCC_ITF_DISABLE_RX_C1:
        UTILITY_CLEAR_REGISTER_BIT(ipcc->C1CR, IPCC_C1CR_RXOIE);
    case IPCC_ITF_DISABLE_TX_C2:
        UTILITY_CLEAR_REGISTER_BIT(ipcc->C2CR, IPCC_C2CR_TXFIE);
    case IPCC_ITF_DISABLE_RX_C2:
        UTILITY_CLEAR_REGISTER_BIT(ipcc->C2CR, IPCC_C2CR_RXOIE);
    }
}

static inline uint32_t IPCC_ITF_STATE(IPCC_TypeDef const *const ipcc, IPCC_IT_Is_Enable IT_Is_On)
{
    switch (IT_Is_On)
    {
    case IPCC_ITF_IS_ENABLE_TX_C1:
        return ((UTILITY_READ_REGISTER_BIT(ipcc->C1CR, IPCC_C1CR_TXFIE) == (IPCC_C1CR_TXFIE)) ? 1UL : 0UL);
    case IPCC_ITF_IS_ENABLE_RX_C1:
        return ((UTILITY_READ_REGISTER_BIT(ipcc->C1CR, IPCC_C1CR_RXOIE) == (IPCC_C1CR_RXOIE)) ? 1UL : 0UL);
    case IPCC_ITF_IS_ENABLE_TX_C2:
        return ((UTILITY_READ_REGISTER_BIT(ipcc->C2CR, IPCC_C2CR_TXFIE) == (IPCC_C2CR_TXFIE)) ? 1UL : 0UL);
    case IPCC_ITF_IS_ENABLE_RX_C2:
        return ((UTILITY_READ_REGISTER_BIT(ipcc->C2CR, IPCC_C2CR_RXOIE) == (IPCC_C2CR_RXOIE)) ? 1UL : 0UL);
    }
}

static inline void IPCC_CH_STATE_ENABLE(IPCC_TypeDef *ipcc, uint32_t Channel, CH_State_Enable State_On)
{
    switch (State_On)
    {
    case IPCC_ENABLE_TX_CH_C1:
        UTILITY_CLEAR_REGISTER_BIT(ipcc->C1MR, Channel << IPCC_C1MR_CH1FM_Pos);
    case IPCC_ENABLE_RX_CH_C1:
        UTILITY_CLEAR_REGISTER_BIT(ipcc->C1MR, Channel);
    case IPCC_ENABLE_TX_CH_C2:
        UTILITY_CLEAR_REGISTER_BIT(ipcc->C2MR, Channel << IPCC_C2MR_CH1FM_Pos);
    case IPCC_ENABLE_RX_CH_C2:
        UTILITY_CLEAR_REGISTER_BIT(ipcc->C2MR, Channel);
    }
}

static inline void IPCC_CH_STATE_DISABLE(IPCC_TypeDef *ipcc, uint32_t Channel, CH_State_Disable State_Off)
{
    switch (State_Off)
    {
    case IPCC_DISABLE_TX_CH_C1:
        UTILITY_SET_REGISTER_BIT(ipcc->C1MR, Channel << IPCC_C1MR_CH1FM_Pos);
    case IPCC_DISABLE_RX_CH_C1:
        UTILITY_SET_REGISTER_BIT(ipcc->C1MR, Channel);
    case IPCC_DISABLE_TX_CH_C2:
        UTILITY_SET_REGISTER_BIT(ipcc->C2MR, Channel << (IPCC_C2MR_CH1FM_Pos));
    case IPCC_DISABLE_RX_CH_C2:
        UTILITY_SET_REGISTER_BIT(ipcc->C2MR, Channel);
    }
}

static inline uint32_t IPCC_CH_STATE_IS_ENABLE(IPCC_TypeDef const *const ipcc, uint32_t Channel, CH_State_Is_Enable State_Is_On)
{
    switch (State_Is_On)
    {
    case IPCC_IS_ENABLE_TX_CH_C1:
        return ((UTILITY_READ_REGISTER_BIT(ipcc->C1MR, Channel << IPCC_C1MR_CH1FM_Pos) != (Channel << IPCC_C1MR_CH1FM_Pos)) ? 1UL : 0UL);
    case IPCC_IS_ENABLE_RX_CH_C1:
        return ((UTILITY_READ_REGISTER_BIT(ipcc->C1MR, Channel) != (Channel)) ? 1UL : 0UL);
    case IPCC_IS_ENABLE_TX_CH_C2:
        return ((UTILITY_READ_REGISTER_BIT(ipcc->C2MR, Channel << IPCC_C2MR_CH1FM_Pos) != (Channel << IPCC_C2MR_CH1FM_Pos)) ? 1UL : 0UL);
    case IPCC_IS_ENABLE_RX_CH_C2:
        return ((UTILITY_READ_REGISTER_BIT(ipcc->C2MR, Channel) != (Channel)) ? 1UL : 0UL);
    }
}

static inline void IPCC_CLEAR_FLAG_CHx(IPCC_TypeDef *ipcc, uint32_t Channel, IPCC_ClearFlag_CHx Clear_F)
{
    switch (Clear_F)
    {
    case IPCC_CLEAR_C1:
        UTILITY_WRITE_REGISTER(ipcc->C1SCR, Channel);
    case IPCC_CLEAR_C2:
        UTILITY_WRITE_REGISTER(ipcc->C2SCR, Channel);
    }
}

static inline void IPCC_SET_FLAG_CHx(IPCC_TypeDef *ipcc, uint32_t Channel, IPCC_SetFlag_CHx Set_F)
{
    switch (Set_F)
    {
    case IPCC_SET_C1:
        UTILITY_WRITE_REGISTER(ipcc->C1SCR, Channel << IPCC_C1SCR_CH1S_Pos);
    case IPCC_SET_C2:
        UTILITY_WRITE_REGISTER(ipcc->C2SCR, Channel << IPCC_C2SCR_CH1S_Pos);
    }
}

static inline uint32_t IPCC_IS_ACTIVE_FLAG_CHx(IPCC_TypeDef const *const ipcc, uint32_t Channel, IPCC_Is_ActiveFlag_CHx Is_Active_F)
{
    switch (Is_Active_F)
    {
    case IPCC_IS_ACTIVE_C1:
        return ((UTILITY_READ_REGISTER_BIT(ipcc->C1TOC2SR, Channel) == (Channel)) ? 1UL : 0UL);
    case IPCC_IS_ACTIVE_C2:
        return ((UTILITY_READ_REGISTER_BIT(ipcc->C2TOC1SR, Channel) == (Channel)) ? 1UL : 0UL);
    }
}

static void IPCC_THREAD_NotEvtHandler( void )
{
    IPCC_CH_STATE_DISABLE( IPCC, IPCC_THREAD_NOTIFICATION_ACK_CHANNEL, IPCC_DISABLE_RX_CH_C1 );

    IPCC_THREAD_EvtNot();

    return;
}

static void IPCC_BLE_EvtHandler( void )
{
    IPCC_BLE_RxEvtNot();

    IPCC_CLEAR_FLAG_CHx( IPCC, IPCC_BLE_EVENT_CHANNEL, IPCC_CLEAR_C1 );

    return;
}

static void IPCC_SYS_EvtHandler( void )
{
    IPCC_SYS_EvtNot();

    IPCC_CLEAR_FLAG_CHx( IPCC, IPCC_SYSTEM_EVENT_CHANNEL, IPCC_CLEAR_C1 );

    return;
}

static void IPCC_TRACES_EvtHandler( void )
{
    IPCC_TRACES_EvtNot();

    IPCC_CLEAR_FLAG_CHx( IPCC, IPCC_TRACES_CHANNEL, IPCC_CLEAR_C1 );

    return;
}

static void IPCC_THREAD_CliNotEvtHandler( void )
{
    IPCC_CH_STATE_DISABLE( IPCC, IPCC_THREAD_CLI_NOTIFICATION_ACK_CHANNEL, IPCC_DISABLE_RX_CH_C1 );

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
    IPCC_CH_STATE_DISABLE( IPCC, IPCC_THREAD_OT_CMD_RSP_CHANNEL, IPCC_DISABLE_TX_CH_C1 );

    IPCC_OT_CmdEvtNot();

    return;
}

static void IPCC_SYS_CmdEvtHandler( void )
{
    IPCC_CH_STATE_DISABLE( IPCC, IPCC_SYSTEM_CMD_RSP_CHANNEL, IPCC_DISABLE_TX_CH_C1 );

    IPCC_SYS_CmdEvtNot();

    return;
}

static void (*FreeBufCb)( void );

static void IPCC_MM_FreeBufHandler( void )
{
    IPCC_CH_STATE_DISABLE( IPCC, IPCC_MM_RELEASE_BUFFER_CHANNEL, IPCC_DISABLE_TX_CH_C1 );

    FreeBufCb();

    IPCC_SET_FLAG_CHx( IPCC, IPCC_MM_RELEASE_BUFFER_CHANNEL, IPCC_SET_C1 );

    return;
}

static void IPCC_BLE_AclDataEvtHandler( void )
{
    IPCC_CH_STATE_DISABLE( IPCC, IPCC_HCI_ACL_DATA_CHANNEL, IPCC_DISABLE_TX_CH_C1 );

  IPCC_BLE_AclDataAckNot();

  return;
}





void IPCC_Enable( void )
{
    PWR_EnableBootC2();

    return;
}

void IPCC_Init( void )
{
    AHB3_GRP1_EnableClock( RCC_AHB3ENR_IPCCEN );

    IPCC_ITF_ENABLE( IPCC, IPCC_ITF_ENABLE_RX_C1 );
    IPCC_ITF_ENABLE( IPCC, IPCC_ITF_ENABLE_TX_C1 );

    Interrupt_enable(INTERRUPT_IPCC_C1_RX);
    Interrupt_enable(INTERRUPT_IPCC_C1_TX);

    return;
}



/**
 * Interrupt Handler
 */
void IPCC_Rx_Handler( void )
{
    if (IPCC_RX_PENDING( IPCC_THREAD_NOTIFICATION_ACK_CHANNEL ))
    {
        IPCC_THREAD_NotEvtHandler();
    }
    else if (IPCC_RX_PENDING( IPCC_BLE_EVENT_CHANNEL ))
    {
        IPCC_BLE_EvtHandler();
    }
    else if (IPCC_RX_PENDING( IPCC_SYSTEM_EVENT_CHANNEL ))
    {
        IPCC_SYS_EvtHandler();
    }
    else if (IPCC_RX_PENDING( IPCC_TRACES_CHANNEL ))
    {
        IPCC_TRACES_EvtHandler();
    }
  else if (IPCC_RX_PENDING( IPCC_THREAD_CLI_NOTIFICATION_ACK_CHANNEL ))
  {
    IPCC_THREAD_CliNotEvtHandler();
  }

    return;
}

void IPCC_Tx_Handler( void )
{
    if (IPCC_TX_PENDING( IPCC_THREAD_OT_CMD_RSP_CHANNEL ))
    {
        IPCC_OT_CmdEvtHandler();
    }
    else if (IPCC_TX_PENDING( IPCC_SYSTEM_CMD_RSP_CHANNEL ))
    {
        IPCC_SYS_CmdEvtHandler();
    }
    else if (IPCC_TX_PENDING( IPCC_MM_RELEASE_BUFFER_CHANNEL ))
    {
        IPCC_MM_FreeBufHandler();
    }
    else if (IPCC_TX_PENDING( IPCC_HCI_ACL_DATA_CHANNEL ))
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
