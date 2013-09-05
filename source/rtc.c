/******************************************************************************
 * Copyright (C) 2012-2013 A. C. Open Hardware Ideas Lab
 * 
 * Author(s):
 *	Marco Giammarini <m.giammarini@warcomeb.it>
 *	
 * Project: libohiboard
 * Package: RTC
 * Version: 0.0
 * 
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>
 ******************************************************************************/

/**
 * @file libohiboard/source/rtc.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief RTC functions implementation.
 */

#include "platforms.h"
#include "utility.h"
#include "rtc.h"

typedef struct Rtc_Device {
    RTC_MemMapPtr 		  regMap;
    
    Rtc_ClockSource       clockSource;
} Rtc_Device;

static Rtc_Device rtc = {
        .regMap        = RTC_BASE_PTR,
        .clockSource   = RTC_SYSTEM_OSCILLATOR,
};
Rtc_DeviceHandle RTC = &rtc;

System_Errors Rtc_init (Rtc_DeviceHandle dev)
{
    RTC_MemMapPtr regmap = dev->regMap;
    Rtc_ClockSource clock = dev->clockSource;
    
    /* Turn on clock */
#if defined(MKL15Z4) || defined(FRDMKL25Z) || defined(FRDMKL05Z)
    SIM_SCGC6 |= SIM_SCGC6_RTC_MASK;
#elif defined(MK60DZ10)
#elif defined(FRDMK20D50M)
#endif
    
    switch(clock)
    {
    case RTC_CLKIN:
#if defined(FRDMKL25Z) || defined(MKL15Z4)
        /* Activate mux for this pin */
        PORTC_PCR1 &= ~PORT_PCR_MUX_MASK;
        PORTC_PCR1 = PORT_PCR_MUX(1);
        /* Select this clock source */
        SIM_SOPT1 &= ~SIM_SOPT1_OSC32KSEL_MASK;
        SIM_SOPT1 |= SIM_SOPT1_OSC32KSEL(2);
#elif defined(MK60DZ10)
#elif defined(FRDMKL05Z)
#elif defined(FRDMK20D50M)
#endif
        break;
    case RTC_SYSTEM_OSCILLATOR:
#if defined(FRDMKL25Z) || defined(MKL15Z4)
        /* Select this clock source */
        SIM_SOPT1 &= ~SIM_SOPT1_OSC32KSEL_MASK;
        SIM_SOPT1 |= SIM_SOPT1_OSC32KSEL(0);
#elif defined(MK60DZ10)
#elif defined(FRDMKL05Z)
#elif defined(FRDMK20D50M)
#endif
        break;
    case RTC_LPO_1kHz:
        /* TODO */
        break;
    }

    /* Configure TSR value to default */
    /* From RM: "TSR with zero is supported, but not recommended" */
    RTC_TSR_REG(regmap) = 1;

    /* Enable counter */
    RTC_SR_REG(regmap) |= RTC_SR_TCE_MASK;

    return ERRORS_NO_ERROR;
}

System_Errors Rtc_setClockSource (Rtc_DeviceHandle dev, Rtc_ClockSource clock)
{
    dev->clockSource = clock;
    return ERRORS_NO_ERROR;
}

/**
 * Set current time with Unix system format.
 * Unix format is the number of seconds since the start of the Unix epoch: 
 * midnight UTC of January 1, 1970 (not counting leap seconds).
 */
void Rtc_setTime (Rtc_DeviceHandle dev, Rtc_Time time)
{
    /* Disable counter */
    RTC_SR_REG(dev->regMap) &= ~RTC_SR_TCE_MASK;
    
    if (time == 0)
        RTC_TSR_REG(dev->regMap) = 1;
    else
        RTC_TSR_REG(dev->regMap) = time;
    
    /* Enable counter */
    RTC_SR_REG(dev->regMap) |= RTC_SR_TCE_MASK;
}

Rtc_Time Rtc_getTime (Rtc_DeviceHandle dev)
{
    return RTC_TSR_REG(dev->regMap);
}