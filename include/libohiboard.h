/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2012-2019 A. C. Open Hardware Ideas Lab
 * 
 * Authors:
 *  Edoardo Bezzeccheri <coolman3@gmail.com>
 *  Marco Giammarini <m.giammarini@warcomeb.it>
 *  Matteo Pirro
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
 * @file libohiboard/include/libohiboard.h
 * @author Edoardo Bezzeccheri <coolman3@gmail.com>
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @author Matteo Pirro
 * @brief Library main file.
 */

/**
 * @mainpage libohiboad
 *
 * The libohiboard is an open-source firmware library for
 * various ARM Cortex-M microcontrollers.
 *
 * @section status Current Development Status
 *
 * In the following table is reported the current development status.
 *
 * <table>
 * <caption id="mcustatus2">MCU Development Status</caption>
 * <tr><th>Peripheral     <th colspan="2">NXP MKL<th colspan="3">NXP MK <th colspan="2">NXP MKV      <th>STM32L4    <th>STM32L0   <th>PIC24FJ 
 * <tr><td>-              <td>MKL15 <td>MKL25    <td>MK10 <td>MK12 <td>MK64      <td>MKV31 <td>MKV46 <td>STM32L476  <td>STM32L073 <td>PIC24FJ1024
 * <tr><td>Clock          <td>-     <td> @ohiwip <td>-    <td>-    <td>-         <td>-     <td>-     <td> @ohidone  <td> @ohidone <td> @ohidone
 * <tr><td>Power Mode     <td>-     <td> @ohiwip <td>-    <td>-    <td>-         <td>-     <td>-     <td> @ohiwip   <td> @ohiwip  <td> @ohiwip
 * <tr><td>GPIO           <td>-     <td>-        <td>-    <td>-    <td>-         <td>-     <td>-     <td> @ohidone  <td> @ohidone <td> @ohidone
 * <tr><td>RTC            <td>-     <td>-        <td>-    <td>-    <td>-         <td>-     <td>-     <td> @ohidone  <td> @ohitodo <td> @ohitodo
 * <tr><td>ADC            <td>-     <td>-        <td>-    <td>-    <td>-         <td>-     <td>-     <td> @ohidone  <td> @ohitodo <td> @ohitodo
 * <tr><td>DAC            <td>-     <td>-        <td>-    <td>-    <td>-         <td>-     <td>-     <td> @ohidone  <td> @ohitodo <td> @ohitodo
 * <tr><td>UART           <td>-     <td>-        <td>-    <td>-    <td>-         <td>-     <td>-     <td> @ohidone  <td> @ohidone <td> @ohidone
 * <tr><td>I2C            <td>-     <td>-        <td>-    <td>-    <td>-         <td>-     <td>-     <td> @ohidone  <td> @ohitodo <td> @ohitodo
 * <tr><td>SPI            <td>-     <td>-        <td>-    <td>-    <td>-         <td>-     <td>-     <td> @ohifix   <td> @ohidone <td> @ohidone
 * <tr><td>Timer          <td>-     <td>-        <td>-    <td>-    <td>-         <td>-     <td>-     <td> @ohiwip   <td> @ohiwip  <td> @ohiwip
 * <tr><td>LowPower Timer <td>-     <td>-        <td>-    <td>-    <td>-         <td>-     <td>-     <td> @ohiwip   <td> @ohidone <td> @ohidone
 * <tr><td>Interrupt      <td>-     <td>-        <td>-    <td>-    <td>-         <td>-     <td>-     <td> @ohidone  <td> @ohidone <td> @ohidone
 * <tr><td>DMA            <td>-     <td>-        <td>-    <td>-    <td>-         <td>-     <td>-     <td> @ohiwip   <td> @ohitodo <td> @ohitodo
 * <tr><td>CAN            <td>-     <td>-        <td>-    <td>-    <td>-         <td>-     <td>-     <td> @ohitodo  <td> -        <td> -
 * <tr><td>Ethernet       <td>-     <td>-        <td>-    <td>-    <td> @ohitodo <td>-     <td>-     <td> -         <td> -        <td> -
 * </table>
 *
 *
 * @section credits Credits
 *
 * Thanks to all person wrote a single row of code, or they suggested a change or gave us advice.
 * The list of people who actively participated is as follows:
 * @li Edoardo Bezzeccheri
 * @li Matteo Civale
 * @li Marco Contigiani
 * @li Simone Giacomucci
 * @li Marco Giammarini (<b>maintainer</b>)
 * @li Leonardo Morichelli
 * @li Nicola Orlandini
 * @li Niccolo' Paolinelli
 * @li Alessio Paolucci
 * @li Matteo Piersantelli
 * @li Matteo Pirro
 * @li Francesco Piunti
 *
 * @section changelog ChangeLog
 *
 * @li <b>v1.1.0</b> of 2018/11/05 - First stable version. No more development
 *     on this version, except for bug fixes.
 * @li <b>v1.0.0-Beta</b> of 2015/02/20 - First release
 *
 * @subpage device "Documented Devices"
 */

#ifndef __LIBOHIBOARD_H
#define __LIBOHIBOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>

#include "types.h"
#include "errors.h"
#include "utility.h"

#if defined (LIBOHIBOARD_NXP_KINETIS)

#if defined (LIBOHIBOARD_MKL15)

/**
 * @page device Documented Devices
 *
 * This documentation was generated for these microcontrollers:
 * @li LIBOHIBOARD_MKL15ZxFM (QFN32)
 * @li LIBOHIBOARD_MKL15ZxFT (QFN48)
 * @li LIBOHIBOARD_MKL15ZxLH (LQFP64)
 * @li LIBOHIBOARD_MKL15ZxLK (LQFP80)
 */


#elif defined (LIBOHIBOARD_MKL25)

/**
 * @page device Documented Devices
 *
 * This documentation was generated for these microcontrollers:
 * @li LIBOHIBOARD_MKL25ZxFM (QFN32)
 * @li LIBOHIBOARD_MKL25ZxFT (QFN48)
 * @li LIBOHIBOARD_MKL25ZxLH (LQFP64)
 * @li LIBOHIBOARD_MKL25ZxLK (LQFP80)
 */

#endif

#elif defined (LIBOHIBOARD_ST_STM32)

#endif

#ifdef LIBOHIBOARD_BASIC

#include "interrupt.h"

#include "gpio.h"

#include "system.h"

#include "clock.h"
#endif

#ifdef LIBOHIBOARD_TIMER
#include "timer.h"
#endif

#ifdef LIBOHIBOARD_PIT
#include "pit.h"
#endif

#ifdef LIBOHIBOARD_UART
#include "uart.h"
#endif

#ifdef LIBOHIBOARD_IIC
#include "i2c.h"
#endif

#ifdef LIBOHIBOARD_SPI
#include "spi.h"
#endif

#ifdef LIBOHIBOARD_ADC
#include "adc.h"
#endif

#ifdef LIBOHIBOARD_DAC
#include "dac.h"
#endif

#ifdef LIBOHIBOARD_ETHERNET
#include "ethernet.h"
#include "ethernet-interface.h"
#include "ethernet-utility.h"
#endif

#ifdef LIBOHIBOARD_RTC
#include "rtc.h"
#endif

#ifdef LIBOHIBOARD_PDB
#include "pdb.h"
#endif

#ifdef LIBOHIBOARD_DMA
#include "dma.h"
#endif

#ifdef LIBOHIBOARD_XBAR
#include "xbar.h"
#endif

#ifdef LIBOHIBOARD_SMC
#include "smc.h"
#endif

#ifdef LIBOHIBOARD_LLWU
#include "llwu.h"
#endif

#ifdef LIBOHIBOARD_FLASH
#include "flash.h"
#endif

#ifdef LIBOHIBOARD_CRITICAL
#include "critical.h"
#endif

#ifdef LIBOHIBOARD_SDHC
#include "sdhc.h"
#endif

#ifdef LIBOHIBOARD_FILTER
#include "filter.h"
#endif

#ifdef LIBOHIBOARD_LOWPOWER
#include "lowpower.h"
#endif

#ifdef LIBOHIBOARD_LOWPOWER_TIMER
#include "lowpower-timer.h"
#endif

#ifdef LIBOHIBOARD_WATCHDOG
#include "watchdog.h"
#endif

#ifdef LIBOHIBOARD_IPCC
#include "ipcc.h"
#endif
#ifndef __XC16
    // stub of ClrWdt invoked from framework
    // WARNING : conflict with microchip builtin methods
    #define ClrWdt() {}
#endif

#include "timeday.h"

#include "comm-utility.h"

#include "utility-buffer.h"

#include "utility-debouncing.h"

#ifdef __cplusplus
}
#endif

#endif // __LIBOHIBOARD_H
