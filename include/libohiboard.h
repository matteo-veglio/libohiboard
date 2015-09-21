/******************************************************************************
 * Copyright (C) 2012-2015 A. C. Open Hardware Ideas Lab
 * 
 * Authors:
 *  Edoardo Bezzeccheri <coolman3@gmail.com>
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
 ******************************************************************************/

/**
 * @file libohiboard/include/libohiboard.h
 * @author Edoardo Bezzeccheri <coolman3@gmail.com>
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Library main file.
 */

#ifndef __LIBOHIBOARD_H
#define __LIBOHIBOARD_H

#include <stdio.h>

#include "types.h"
#include "errors.h"
#include "utility.h"

#include "interrupt.h"

#include "gpio.h"

#include "system.h"

#include "clock.h"

#ifdef LIBOHIBOARD_FTM
#include "ftm.h"
#endif

#ifdef LIBOHIBOARD_UART
#include "uart.h"
#endif

#include "i2c.h"

#include "spi.h"

#include "adc.h"

#ifdef LIBOHIBOARD_DAC
#include "dac.h"
#endif

#include "rtc.h"

#include "timeday.h"

void test();

#endif /* __LIBOHIBOARD_H */
