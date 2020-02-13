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

typedef struct _Ipcc_Device
{
    IPCC_TypeDef*       regmap;
    IPCC_CommonTypeDef* regmapCpu1;
    IPCC_CommonTypeDef* regmapCpu2;
    PWR_TypeDef*        regmapPWR;

    volatile uint32_t* rccRegisterPtr;      /**< Register for clock enabling. */
    uint32_t rccRegisterEnable;        /**< Register mask for current device. */

    pFunc callbackTx;
    pFunc callbackRx;
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

inline void __attribute__((always_inline)) Ipcc_enableInterrupt (Ipcc_DeviceHandle dev, Ipcc_DirectionChannel dir)
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

inline void __attribute__((always_inline)) Ipcc_disableInterrupt (Ipcc_DeviceHandle dev, Ipcc_DirectionChannel dir)
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

inline uint32_t __attribute__((always_inline)) Ipcc_isInterruptEnable (Ipcc_DeviceHandle dev, Ipcc_DirectionChannel dir)
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
    return 0;
}

inline void __attribute__((always_inline)) Ipcc_enableChannel (Ipcc_DeviceHandle dev, Ipcc_Channel channel, Ipcc_DirectionChannel dir)
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

inline void __attribute__((always_inline)) Ipcc_disableChannel (Ipcc_DeviceHandle dev, Ipcc_Channel channel, Ipcc_DirectionChannel dir)
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

inline uint32_t __attribute__((always_inline)) Ipcc_isChannelEnable (Ipcc_DeviceHandle dev, Ipcc_Channel channel, Ipcc_DirectionChannel dir)
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
    return 0;
}

inline void __attribute__((always_inline)) Ipcc_clearFlag (Ipcc_DeviceHandle dev, Ipcc_Channel channel, Ipcc_CpuChannel cpu)
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

inline void __attribute__((always_inline)) Ipcc_setFlag (Ipcc_DeviceHandle dev, Ipcc_Channel channel, Ipcc_CpuChannel cpu)
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

inline uint32_t __attribute__((always_inline)) Ipcc_isFlagActive (Ipcc_DeviceHandle dev, Ipcc_Channel channel, Ipcc_CpuChannel cpu)
{
    switch (cpu)
    {
    case IPCC_CPUCHANNEL_1:
        return ((UTILITY_READ_REGISTER_BIT(dev->regmapCpu1->SR, channel) == (channel)) ? 1UL : 0UL);
    case IPCC_CPUCHANNEL_2:
        return ((UTILITY_READ_REGISTER_BIT(dev->regmapCpu2->SR, channel) == (channel)) ? 1UL : 0UL);
    }
    return 0;
}

void Ipcc_enable (Ipcc_DeviceHandle dev)
{
    (void) dev;
    IPCC_BOOT_CPU2();
}

System_Errors Ipcc_init (Ipcc_DeviceHandle dev, pFunc callbackTx, pFunc callbackRx)
{
    System_Errors err = ERRORS_NO_ERROR;
    IPCC_CLOCK_ENABLE(*dev->rccRegisterPtr,dev->rccRegisterEnable);

    Ipcc_enableInterrupt(dev,IPCC_DIRECTIONCHANNEL_CPU1_TX);
    Ipcc_enableInterrupt(dev,IPCC_DIRECTIONCHANNEL_CPU1_RX);

    if (callbackTx != NULL)
    {
        dev->callbackTx = callbackTx;
    }

    if (callbackRx != NULL)
    {
        dev->callbackRx = callbackRx;
    }

    Interrupt_enable(INTERRUPT_IPCC_C1_TX);
    Interrupt_enable(INTERRUPT_IPCC_C1_RX);

    return err;
}

void IPCC_C1_RX_IRQHandler (void)
{
    if (OB_IPCC->callbackRx != NULL)
    {
        OB_IPCC->callbackRx();
    }
}

void IPCC_C1_TX_IRQHandler (void)
{
    if (OB_IPCC->callbackTx != NULL)
    {
        OB_IPCC->callbackTx();
    }
}

bool Ipcc_isIsrTxPending (Ipcc_DeviceHandle dev, Ipcc_Channel channel)
{
    return ((!(Ipcc_isFlagActive(dev,channel,IPCC_CPUCHANNEL_1))) && (((~(dev->regmapCpu1->MR)) & (channel << 16ul))));
}

bool Ipcc_isIsrRxPending (Ipcc_DeviceHandle dev, Ipcc_Channel channel)
{
    return (((Ipcc_isFlagActive(dev,channel,IPCC_CPUCHANNEL_2))) && (((~(dev->regmapCpu1->MR)) & (channel << 0ul ))));
}

#ifdef __cplusplus
}
#endif

#endif // LIBOHIBOARD_IPCC
