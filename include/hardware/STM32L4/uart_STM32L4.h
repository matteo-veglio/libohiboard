/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2018 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *   Marco Giammarini <m.giammarini@warcomeb.it>
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
 * @file libohiboard/include/hardware/STM32L4/uart_STM32L4.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief UART pins and device definitions for STM32L4 series
 */

#ifndef __UART_STM32L4_H
#define __UART_STM32L4_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

#if defined(LIBOHIBOARD_UART) & defined(LIBOHIBOARD_STM32L4)

typedef enum _Uart_RxPins
{
#if defined (LIBOHIBOARD_STM32L476)

// WLCSP72 ballout
// LQFP64
#if defined (LIBOHIBOARD_STM32L476Jx) || \
    defined (LIBOHIBOARD_STM32L476Rx)

    UART_PINS_PA1,
    UART_PINS_PA3,
    UART_PINS_PA10,

    UART_PINS_PB7,
    UART_PINS_PB10_RX,
    UART_PINS_PB11_RX,

    UART_PINS_PC0,
    UART_PINS_PC5,
    UART_PINS_PC11,

    UART_PINS_PD2,

#if defined (LIBOHIBOARD_STM32L476Jx)
    UART_PINS_PG10,
#endif

#endif

#endif // LIBOHIBOARD_STM32L476

    UART_PINS_RXNONE,

} Uart_RxPins;

typedef enum _Uart_TxPins
{
#if defined (LIBOHIBOARD_STM32L476)

// WLCSP72 ballout
// LQFP64
#if defined (LIBOHIBOARD_STM32L476Jx) || \
    defined (LIBOHIBOARD_STM32L476Rx)

    UART_PINS_PA0,
    UART_PINS_PA2,
    UART_PINS_PA9,

    UART_PINS_PB6,
    UART_PINS_PB10_TX,
    UART_PINS_PB11_TX,

    UART_PINS_PC1,
    UART_PINS_PC4,
    UART_PINS_PC10,
    UART_PINS_PC12,

#if defined (LIBOHIBOARD_STM32L476Jx)
    UART_PINS_PG9,
#endif

#endif

#endif // LIBOHIBOARD_STM32L476

    UART_PINS_TXNONE,

} Uart_TxPins;


// WLCSP72 ballout
// LQFP64
#if defined (LIBOHIBOARD_STM32L476Jx) || \
    defined (LIBOHIBOARD_STM32L476Rx)

void LPUART1_IRQHandler(void);
void USART1_IRQHandler(void);
void USART2_IRQHandler(void);
void USART3_IRQHandler(void);
void UART4_IRQHandler(void);
void UART5_IRQHandler(void);

extern Uart_DeviceHandle OB_UART1;
extern Uart_DeviceHandle OB_UART2;
extern Uart_DeviceHandle OB_UART3;
extern Uart_DeviceHandle OB_UART4;
extern Uart_DeviceHandle OB_UART5;
extern Uart_DeviceHandle OB_LPUART1;

#endif // LIBOHIBOARD_STM32L476Jx || LIBOHIBOARD_STM32L476Rx

#endif // LIBOHIBOARD_UART & LIBOHIBOARD_STM32L4

#ifdef __cplusplus
}
#endif

#endif // __SPI_STM32L4_H
