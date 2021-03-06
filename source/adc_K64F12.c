/* Copyright (C) 2014-2016 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *  Alessio Paolucci <a.paolucci89@gmail.com>
 *  Matteo Civale <m.civale@gmail.com>
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
 * @file libohiboard/source/adc_K64F12.c
 * @author Alessio Paolucci <a.paolucci89@gmail.com>
 * @author Matteo Civale <m.civale@gmail.com>
 * @brief ADC functions implementation.
 */

#ifdef LIBOHIBOARD_ADC

#include "platforms.h"
#include "system.h"
#include "interrupt.h"
#include "adc.h"
#include "clock.h"

#if defined (LIBOHIBOARD_K64F12)     || \
    defined (LIBOHIBOARD_FRDMK64F)

//#define ADC0_BASE_PTR ADC0
//#define ADC1_BASE_PTR ADC1

#define ADC_PIN_ENABLED                  1
#define ADC_PIN_DISABLED                 0

#define ADC_MAX_PINS                     28

/*! @brief Pointers to ADC16 bases for each instance. */
static ADC_Type *const s_adc16Bases[] = ADC_BASE_PTRS;

typedef struct Adc_Device {
    /*ADC_MemMapPtr*/ADC_Type * regMap;

    volatile uint32_t* simScgcPtr;    /**< SIM_SCGCx register for the device. */
    uint32_t simScgcBitEnable;       /**< SIM_SCGC enable bit for the device. */

    void (*isr)(void);                     /**< The function pointer for ISR. */
    void (*callback)(void);      /**< The function pointer for user callback. */
    Interrupt_Vector isrNumber;                       /**< ISR vector number. */

    Adc_Pins pins[ADC_MAX_PINS+1];  /**< List of the pin for the ADC channel. */
    volatile uint32_t* pinsPtr[ADC_MAX_PINS];
    Adc_ChannelNumber channelNumber[ADC_MAX_PINS];
    Adc_ChannelMux channelMux[ADC_MAX_PINS];
    uint8_t pinMux[ADC_MAX_PINS];     /**< Mux of the pin of the FTM channel. */

    uint8_t devInitialized;   /**< Indicate that device was been initialized. */
    /** Indicate that device require calibration on Adc_readValue. */
    uint8_t devCalibration;
} Adc_Device;

static Adc_Device adc0_lib = {
     .regMap           = ADC0,//s_adc16Bases[0],//ADC0_BASE_PTR,

     .simScgcPtr       = &SIM_SCGC6,
     .simScgcBitEnable = SIM_SCGC6_ADC0_MASK,

     .pins             = {ADC_PINS_PTE2,
    		              ADC_PINS_PTE3,
    		              ADC_PINS_ADC0_DP1,
                          ADC_PINS_ADC0_DM1,
                          ADC_PINS_ADC0_DP0,
                          ADC_PINS_ADC0_DM0,
                          ADC_PINS_ADC0_DP3,
                          ADC_PINS_ADC0_DM3,
                          ADC_PINS_ADC0_SE22,
                          ADC_PINS_ADC0_SE16,
                          ADC_PINS_ADC0_SE21,
                          ADC_PINS_ADC0_SE23,
                          ADC_PINS_PTE24,
                          ADC_PINS_PTE25,
                          ADC_PINS_PTA7,
                          ADC_PINS_PTA8,
                          ADC_PINS_PTB0,
                          ADC_PINS_PTB1,
                          ADC_PINS_PTB2,
                          ADC_PINS_PTB3,
                          ADC_PINS_PTC0,
                          ADC_PINS_PTC1,
                          ADC_PINS_PTC2,
                          ADC_PINS_PTD1,
                          ADC_PINS_PTD5,
                          ADC_PINS_PTD6,
                          ADC_PINS_NONE,
		        },
      .pinsPtr         = {&PORTE_PCR2,
                          &PORTE_PCR3,
    		              0,//&ADC_PINS_ADC0_DP1,
                          0,//&ADC_PINS_ADC0_DM1,
    		              0,//&ADC_PINS_ADC0_DP0,
                          0,//&ADC_PINS_ADC0_DM0,
    		              0,//&ADC_PINS_ADC0_DP3,
                          0,//&ADC_PINS_ADC0_DM3,
                          0,//&ADC_PINS_ADC0_SE22,
                          0,//&ADC_PINS_ADC0_SE16,
                          0,//&ADC_PINS_ADC0_SE21,
                          0,//&ADC_PINS_ADC0_SE23,
                          &PORTE_PCR24,
                          &PORTE_PCR25,
                          &PORTA_PCR7,
                          &PORTA_PCR8,
                          &PORTB_PCR0,
                          &PORTB_PCR1,
                          &PORTB_PCR2,
                          &PORTB_PCR3,
                          &PORTC_PCR0,
                          &PORTC_PCR1,
                          &PORTC_PCR2,
                          &PORTD_PCR1,
                          &PORTD_PCR5,
                          &PORTD_PCR6,
		        },
        .pinMux        = {0,
        		          0,
        		          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
               },
         .channelNumber  = {ADC_CH_DP2,
        		          ADC_CH_DM2,
        		          ADC_CH_DP1,
                          ADC_CH_DM1,
                          ADC_CH_DP0,
                          ADC_CH_DM0,
                          ADC_CH_DP3,
                          ADC_CH_DM3,
                          ADC_CH_SE22,
                          ADC_CH_SE16,
                          ADC_CH_SE21,
                          ADC_CH_SE23,
                          ADC_CH_SE17,
                          ADC_CH_SE18,
                          ADC_CH_SE10,
                          ADC_CH_SE11,
                          ADC_CH_SE8,
                          ADC_CH_SE9,
                          ADC_CH_SE12,
                          ADC_CH_SE13,
                          ADC_CH_SE14,
                          ADC_CH_SE15,
                          ADC_CH_SE4b,
                          ADC_CH_SE5b,
                          ADC_CH_SE6b,
                          ADC_CH_SE7b,
              },
         .channelMux   = {ADC_CHL_A,
                          ADC_CHL_A,
                          ADC_CHL_A,
                          ADC_CHL_A,
                          ADC_CHL_A,
                          ADC_CHL_A,
                          ADC_CHL_A,
                          ADC_CHL_A,
                          ADC_CHL_A,
                          ADC_CHL_A,
                          ADC_CHL_A,
                          ADC_CHL_A,
                          ADC_CHL_A,
                          ADC_CHL_A,
                          ADC_CHL_A,
                          ADC_CHL_A,
                          ADC_CHL_A,
                          ADC_CHL_A,
                          ADC_CHL_A,
                          ADC_CHL_A,
                          ADC_CHL_A,
                          ADC_CHL_A,
                          ADC_CHL_B,
                          ADC_CHL_B,
                          ADC_CHL_B,
                          ADC_CHL_B,
         },

         .isr              = ADC0_IRQHandler,
         .isrNumber        = INTERRUPT_ADC0,

         .devInitialized = 0,
         .devCalibration = 0,
};
Adc_DeviceHandle OB_ADC0 = &adc0_lib;

static Adc_Device adc1_lib = {
        .regMap          = ADC1,//s_adc16Bases[1],//ADC1_BASE_PTR,

        .simScgcPtr      = &SIM_SCGC3,
        .simScgcBitEnable= SIM_SCGC3_ADC1_MASK,

        .pins            = {ADC_PINS_PTE0,
                            ADC_PINS_PTE1,
                            ADC_PINS_PTE2,
                            ADC_PINS_PTE3,
                            ADC_PINS_ADC1_DP1,
                            ADC_PINS_ADC1_DM1,
                            ADC_PINS_ADC1_DP3,
                            ADC_PINS_ADC1_DM3,
                            ADC_PINS_ADC1_DP0,
                            ADC_PINS_ADC1_DM0,
                            ADC_PINS_ADC1_SE16,
                            ADC_PINS_ADC1_SE18,
                            ADC_PINS_ADC1_SE23,
                            ADC_PINS_PTA17,
                            ADC_PINS_PTB0,
                            ADC_PINS_PTB1,
                            ADC_PINS_PTB4,
                            ADC_PINS_PTB5,
                            ADC_PINS_PTB6,
                            ADC_PINS_PTB7,
                            ADC_PINS_PTB10,
                            ADC_PINS_PTB11,
                            ADC_PINS_PTC8,
                            ADC_PINS_PTC9,
                            ADC_PINS_PTC10,
                            ADC_PINS_PTC11,
                            ADC_PINS_NONE,
        },

        .pinsPtr         = {&PORTE_PCR0,
                            &PORTE_PCR1,
                            &PORTE_PCR2,
                            &PORTE_PCR3,
                            0,//&ADC_PINS_ADC1_DP1,
                            0,//&ADC_PINS_ADC1_DM1,
                            0,//&ADC_PINS_ADC1_DP3,
                            0,//&ADC_PINS_ADC1_DM3,
                            0,//&ADC_PINS_ADC1_DP0,
                            0,//&ADC_PINS_ADC1_DM0,
                            0,//&ADC_PINS_ADC1_SE16,
                            0,//&ADC_PINS_ADC1_SE18,
                            0,//&ADC_PINS_ADC1_SE23,
                            &PORTA_PCR17,
                            &PORTB_PCR0,
                            &PORTB_PCR1,
                            &PORTB_PCR4,
                            &PORTB_PCR5,
                            &PORTB_PCR6,
                            &PORTB_PCR7,
                            &PORTB_PCR10,
                            &PORTB_PCR11,
                            &PORTC_PCR8,
                            &PORTC_PCR9,
                            &PORTC_PCR10,
                            &PORTC_PCR11,
       },

       .pinMux           = {0,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0,
     },
      .channelNumber    = {ADC_CH_SE4a,
    		               ADC_CH_SE5a,
    		               ADC_CH_SE6a,
    		               ADC_CH_SE7a,
                           ADC_CH_DP1,
                           ADC_CH_DM1,
                           ADC_CH_DP3,
                           ADC_CH_DM3,
                           ADC_CH_DP0,
                           ADC_CH_DM0,
                           ADC_CH_SE16,
                           ADC_CH_SE18,
                           ADC_CH_SE23,
                           ADC_CH_SE17,
                           ADC_CH_SE8,
                           ADC_CH_SE9,
                           ADC_CH_SE10,
                           ADC_CH_SE11,
                           ADC_CH_SE12,
                           ADC_CH_SE13,
                           ADC_CH_SE14,
                           ADC_CH_SE15,
                           ADC_CH_SE4b,
                           ADC_CH_SE5b,
                           ADC_CH_SE6b,
                           ADC_CH_SE7b,
       },
       .channelMux      = {ADC_CHL_A,
                           ADC_CHL_A,
                           ADC_CHL_A,
                           ADC_CHL_A,
                           ADC_CHL_A,
                           ADC_CHL_A,
                           ADC_CHL_A,
                           ADC_CHL_A,
                           ADC_CHL_A,
                           ADC_CHL_A,
                           ADC_CHL_A,
                           ADC_CHL_A,
                           ADC_CHL_A,
                           ADC_CHL_A,
                           ADC_CHL_A,
                           ADC_CHL_A,
                           ADC_CHL_A,
                           ADC_CHL_A,
                           ADC_CHL_A,
                           ADC_CHL_A,
                           ADC_CHL_A,
                           ADC_CHL_A,
                           ADC_CHL_A,
                           ADC_CHL_B,
                           ADC_CHL_B,
                           ADC_CHL_B,
                           ADC_CHL_B,
      },

      .isr              = ADC1_IRQHandler,
      .isrNumber        = INTERRUPT_ADC1,

      .devInitialized = 0,
      .devCalibration = 0,
};
Adc_DeviceHandle OB_ADC1 = &adc1_lib;

void ADC0_IRQHandler (void)
{
    OB_ADC0->callback();
    (OB_ADC0->regMap)->R[0];

    if (!((OB_ADC0->regMap)->SC3 & ADC_SC3_ADCO_MASK)
    		&& !((OB_ADC0->regMap)->SC2 & ADC_SC2_ADTRG_MASK))

    	(OB_ADC0->regMap)->SC1[0] |= ADC_SC1_ADCH(ADC_CH_DISABLE);
}

void ADC1_IRQHandler (void)
{
    OB_ADC1->callback();
    (OB_ADC1->regMap)->R[0];

    if (!((OB_ADC1->regMap)->SC3 & ADC_SC3_ADCO_MASK)
    		&& !((OB_ADC1->regMap)->SC2 & ADC_SC2_ADTRG_MASK))

    	(OB_ADC1->regMap)->SC1[0] |= ADC_SC1_ADCH(ADC_CH_DISABLE);
}

/**
 * @brief
 * @param dev Adc device handle to be synchronize.
 * @return A System_Errors elements that indicate the status of initialization.
 */
System_Errors Adc_init (Adc_DeviceHandle dev, void* callback, Adc_Config *config)
{
    /*ADC_MemMapPtr*/ ADC_Type* regmap = dev->regMap;
    System_Errors errore = ERRORS_NO_ERROR;
    //uint8_t clkdiv = 0;

    if (dev->devInitialized) return ERRORS_ADC_DEVICE_JUST_INIT;

    /* Enable the clock to the selected ADC */
    *dev->simScgcPtr |= dev->simScgcBitEnable;

    /* Da controllare, quando attivo l'hardwere trigger mode, se non metto uno dei
     * canali ad un valore diverso da 0x1f l'adc settato per ultimo non parte
     * */
    regmap->SC1[0] = ADC_SC1_ADCH(ADC_CH_DISABLE);
    regmap->SC1[1] = ADC_SC1_ADCH(ADC_CH_DISABLE);

    /*setting clock source and divider*/
    switch (config->clkDiv)
    {
    case 1:
    	regmap->CFG1 |= ADC_CFG1_ADIV(0);
    	break;
    case 2:
    	regmap->CFG1 |= ADC_CFG1_ADIV(1);
    	break;
    case 4:
    	regmap->CFG1 |= ADC_CFG1_ADIV(2);
    	break;
    case 8:
    	regmap->CFG1 |= ADC_CFG1_ADIV(3);
    	break;
    default:
    	return errore = ERRORS_ADC_DIVIDER_NOT_FOUND;
    }

    switch (config->clkSource)
    {
    case ADC_BUS_CLOCK:
    	regmap->CFG1 |= ADC_CFG1_ADICLK(0);
    	break;
    case ADC_BUS_CLOCK_DIV2:
    	regmap->CFG1 |= ADC_CFG1_ADICLK(1);
    	break;
    case ADC_ALTERNATE_CLOCK:
    	regmap->CFG1 |= ADC_CFG1_ADICLK(2);
    	break;
    case ADC_ASYNCHRONOUS_CLOCK:
    	regmap->CFG1 |= ADC_CFG1_ADICLK(3);
    	break;
    }

    /*Setting Sample Time*/
    switch (config->sampleLength)
    {
    case ADC_SHORT_SAMPLE:
    	regmap->CFG1 &= ~(ADC_CFG1_ADLSMP_MASK);
    	break;
    case ADC_LONG_SAMPLE_20:
    	regmap->CFG1 |= ADC_CFG1_ADLSMP_MASK;
    	regmap->CFG2 |= ADC_CFG2_ADLSTS(0);
    	break;
    case ADC_LONG_SAMPLE_12:
    	regmap->CFG1 |= ADC_CFG1_ADLSMP_MASK;
    	regmap->CFG2 |= ADC_CFG2_ADLSTS(1);
    	break;
    case ADC_LONG_SAMPLE_6:
    	regmap->CFG1 |= ADC_CFG1_ADLSMP_MASK;
    	regmap->CFG2 |= ADC_CFG2_ADLSTS(2);
    	break;
    case ADC_LONG_SAMPLE_2:
    	regmap->CFG1 |= ADC_CFG1_ADLSMP_MASK;
    	regmap->CFG2 |= ADC_CFG2_ADLSTS(3);
    	break;
    }

    /*setting convertion speed*/
    switch (config->covertionSpeed)
    {
    case ADC_NORMAL_CONVERTION:
    	regmap->CFG2 &= ~(ADC_CFG2_ADHSC_MASK);
    	break;
    case ADC_HIGH_SPEED_CONVERTION:
    	regmap->CFG2 |= ADC_CFG2_ADHSC_MASK;
    	break;
    }


    /*setting single or continuous convertion*/
    switch (config->contConv)
    {
    case ADC_SINGLE_CONVERTION:
    	regmap->SC3 &= ~(ADC_SC3_ADCO_MASK);
    	break;
    case ADC_CONTINUOUS_CONVERTION:
    	regmap->SC3 |= ADC_SC3_ADCO_MASK;
    	break;
    }

    /*setting resoluton*/
    switch (config->resolution)
    {
    case ADC_RESOLUTION_8BIT:
    	regmap->CFG1 |= ADC_CFG1_MODE(0);
        break;
    case ADC_RESOLUTION_10BIT:
    	regmap->CFG1 |= ADC_CFG1_MODE(2);
        break;
    case ADC_RESOLUTION_12BIT:
    	regmap->CFG1 |= ADC_CFG1_MODE(1);
        break;
    case ADC_RESOLUTION_16BIT:
    	regmap->CFG1 |= ADC_CFG1_MODE(3);
        break;
	}

    /* Select voltage reference*/
    switch (config->voltRef)
    {
    case ADC_VREF:
    	regmap->SC2 = ADC_SC2_REFSEL(0);
        break;
    case ADC_VALT:
    	regmap->SC2 = ADC_SC2_REFSEL(1);
        break;
    }

    /* Select the average */
    switch (config->average)
    {
    case ADC_AVERAGE_1_SAMPLES:
        /* Nothing to do! */
    	regmap->SC3 &= ~ADC_SC3_AVGE_MASK;
        break;
    case ADC_AVERAGE_4_SAMPLES:
    	regmap->SC3 = ADC_SC3_AVGE_MASK | ADC_SC3_AVGS(0);
        break;
    case ADC_AVERAGE_8_SAMPLES:
    	regmap->SC3 = ADC_SC3_AVGE_MASK | ADC_SC3_AVGS(1);
        break;
    case ADC_AVERAGE_16_SAMPLES:
    	regmap->SC3 = ADC_SC3_AVGE_MASK | ADC_SC3_AVGS(2);
        break;
    case ADC_AVERAGE_32_SAMPLES:
    	regmap->SC3 = ADC_SC3_AVGE_MASK | ADC_SC3_AVGS(3);
        break;
    }


    if (config->enableHwTrigger)
    	regmap->SC2 |= ADC_SC2_ADTRG_MASK;
    else
    	regmap->SC2 &= ~ADC_SC2_ADTRG_MASK;


    dev->devInitialized = 1;

    /* Calibration flag */
	if (config->doCalibration)
		Adc_calibration(dev);

    /* If call back exist save it */
    if (callback)
    {
	   dev->callback = callback;
	   /* Enable interrupt */
	   Interrupt_enable(dev->isrNumber);
    }

	return ERRORS_NO_ERROR;
}

void Adc_enablePin (Adc_DeviceHandle dev, Adc_Pins pin)
{
    uint8_t devPinIndex;

    for (devPinIndex = 0; devPinIndex < ADC_MAX_PINS; ++devPinIndex)
    {
        if (dev->pins[devPinIndex] == ADC_PINS_NONE) break;

        /* Pins haven't PORT register */
        if ((pin == ADC_PINS_ADC0_DP1) ||
            (pin == ADC_PINS_ADC0_DM1) ||
            (pin == ADC_PINS_ADC0_DP0) ||
            (pin == ADC_PINS_ADC0_DM0) ||
            (pin == ADC_PINS_ADC0_DP3) ||
            (pin == ADC_PINS_ADC0_DM3) ||
            (pin == ADC_PINS_ADC0_SE22) ||
            (pin == ADC_PINS_ADC0_SE16) ||
            (pin == ADC_PINS_ADC0_SE21) ||
            (pin == ADC_PINS_ADC0_SE23) ||
            (pin == ADC_PINS_ADC1_DP1) ||
            (pin == ADC_PINS_ADC1_DM1) ||
            (pin == ADC_PINS_ADC1_DP3) ||
            (pin == ADC_PINS_ADC1_DM3) ||
            (pin == ADC_PINS_ADC1_DP0) ||
            (pin == ADC_PINS_ADC1_DM0) ||
            (pin == ADC_PINS_ADC1_SE16) ||
            (pin == ADC_PINS_ADC1_SE18) ||
            (pin == ADC_PINS_ADC1_SE23))
            break;

        if (dev->pins[devPinIndex] == pin)
        {
            *(dev->pinsPtr[devPinIndex]) =
                PORT_PCR_MUX(dev->pinMux[devPinIndex]) | PORT_PCR_IRQC(0);
            break;
        }
    }

    /* TODO: It's all? */
}

System_Errors Adc_readValue (Adc_DeviceHandle dev,
                             Adc_ChannelNumber channel,
                             uint16_t *value,
                             Adc_InputType type)
{
    /*ADC_MemMapPtr*/ADC_Type* regmap = dev->regMap;
    uint8_t channelIndex;
    Adc_ChannelMux channelMux;

    if (channel != ADC_CH_DISABLE)
    {
        for (channelIndex = 0; channelIndex < ADC_MAX_PINS; ++channelIndex)
        {
            if (dev->channelNumber[channelIndex] == channel)
            {
                channelMux = dev->channelMux[channelIndex];
                break;
            }
        }

        if (channel > 0x1F)
            channel -= 0x20;

        if (channelMux == ADC_CHL_A)
        	regmap->CFG2 &= ~ADC_CFG2_MUXSEL_MASK;
        else
        	regmap->CFG2 |= ADC_CFG2_MUXSEL_MASK;


        /* Set single-ended or differential input mode! */
        regmap->SC1[0] |= ((type << ADC_SC1_DIFF_SHIFT) & ADC_SC1_DIFF_MASK);

        /*
         * If is there a callback, enable interrupt and go out!
         */
        if(dev->callback)
        {
        	regmap->SC1[0] &= ~(ADC_SC1_AIEN_MASK | ADC_SC1_ADCH_MASK);
        	regmap->SC1[0] |= ADC_SC1_AIEN_MASK | ADC_SC1_ADCH(channel);
            *value = 0;
            return ERRORS_NO_ERROR;
        }

        /* Start conversion */
        regmap->SC1[0] = (regmap->SC1[0] & (~ADC_SC1_ADCH_MASK))|ADC_SC1_ADCH(channel);

        /* wait until conversion ended */
        while ((regmap->SC1[0] & ADC_SC1_COCO_MASK) != ADC_SC1_COCO_MASK);

        *value = (uint16_t) ADC_R_REG(regmap,0);

        /* Disable conversion */
        regmap->SC1[0] = ADC_SC1_ADCH(ADC_CH_DISABLE);

        return ERRORS_NO_ERROR;
    }
    else
    {
        *value = 0;
        return ERRORS_ADC_CHANNEL_BUSY;
    }
}

System_Errors Adc_readValueFromInterrupt (Adc_DeviceHandle dev,
										uint16_t *value)
{
	ADC_Type* regmap = dev->regMap;
    *value = (uint16_t) ADC_R_REG(regmap,0);
}

System_Errors Adc_setHwChannelTrigger (Adc_DeviceHandle dev,
                                       Adc_ChannelConfig* config,
                                       uint8_t numChannel)
{
    uint8_t i;

    if (numChannel > ADC_MAX_CHANNEL_NUMBER)
        return ERRORS_ADC_NUMCH_WRONG;



    (dev->regMap)->SC2 &= ~ADC_SC2_ADTRG_MASK;

    Interrupt_disable(dev->isrNumber);

    for(i=0;i<numChannel;i++)
    {
    	(dev->regMap)->SC1[i] = 0;// ADC_SC1_ADCH(0);
    	(dev->regMap)->SC1[i] = ADC_SC1_ADCH(config[i].channel)|
                                     ADC_SC1_AIEN_MASK |
                                     ((config[i].inputType << ADC_SC1_DIFF_SHIFT) & ADC_SC1_DIFF_MASK);
    }

    (dev->regMap)->SC2 |= ADC_SC2_ADTRG_MASK;

    Interrupt_enable(dev->isrNumber);

    return ERRORS_NO_ERROR;
}



System_Errors Adc_calibration (Adc_DeviceHandle dev)
{
	/*ADC_MemMapPtr*/ADC_Type* regmap = dev->regMap;
	    uint16_t calibration;
	    uint32_t busClock = Clock_getFrequency(CLOCK_BUS);

	    uint32_t regSC3  = ADC_SC3_REG(regmap);
	    uint32_t regSC2  = ADC_SC2_REG(regmap);
	    uint32_t regCFG1 = ADC_CFG1_REG(regmap);

	    if (!dev->devInitialized) return ERRORS_ADC_DEVICE_NOT_INIT;

	    /* Set registers and clock for a better calibration */
	    regmap->SC3 &= ~(ADC_SC3_AVGE_MASK | ADC_SC3_AVGS_MASK | ADC_SC3_ADCO_MASK);
	    regmap->SC3 |= ADC_SC3_AVGE_MASK | ADC_SC3_AVGS(3)|ADC_SC3_CAL_MASK;

	    regmap->SC2 &= ~ADC_SC2_REFSEL_MASK;
	    regmap->SC2 |= ADC_SC2_REFSEL(0);

	    regmap->CFG1 &= ~(ADC_CFG1_ADIV_MASK | ADC_CFG1_ADICLK_MASK);
	    if (busClock < 32000000)
	    	regmap->CFG1 |= ADC_CFG1_ADIV(3);
	    else
	    	regmap->CFG1 |= ADC_CFG1_ADIV(3) | ADC_CFG1_ADICLK_MASK;

	    /* Start calibration */
	    regmap->SC3 |= ADC_SC3_CAL_MASK;
	    /* wait calibration ended */
	    while (regmap->SC3 & ADC_SC3_CAL_MASK){;};
	    /* Check error */
	    if(regmap->SC3 & ADC_SC3_CALF_MASK)
	    {
	        /* Restore register */
	    	regmap->SC3 = regSC3;
	    	regmap->SC3 = regSC2;
	    	regmap->CFG1 = regCFG1;
	        return ERRORS_ADC_CALIBRATION;
	    }

	    calibration = 0;
	    calibration += ADC_CLP0_REG(regmap);
	    calibration += ADC_CLP1_REG(regmap);
	    calibration += ADC_CLP2_REG(regmap);
	    calibration += ADC_CLP3_REG(regmap);
	    calibration += ADC_CLP4_REG(regmap);
	    calibration += ADC_CLPS_REG(regmap);
	    calibration = (calibration >> 1U) | 0x8000U; // divide by 2
	    regmap->PG = calibration;

	    calibration = 0;
	    calibration += ADC_CLM0_REG(regmap);
	    calibration += ADC_CLM1_REG(regmap);
	    calibration += ADC_CLM2_REG(regmap);
	    calibration += ADC_CLM3_REG(regmap);
	    calibration += ADC_CLM4_REG(regmap);
	    calibration += ADC_CLMS_REG(regmap);
	    calibration = (calibration >> 1U) | 0x8000U; // divide by 2
	    regmap->MG = calibration;

	    regmap->SC1[0] = ADC_SC1_ADCH(ADC_CH_DISABLE);

	    /* Restore register */
	    regmap->SC3 = regSC3;
	    regmap->SC2 = regSC2;
	    regmap->CFG1 = regCFG1;

	    dev->devCalibration = 1;

	    return ERRORS_NO_ERROR;
}

#endif // defined (LIBOHIBOARD_K64F12) || defined (LIBOHIBOARD_FRDMK64F)

#endif // LIBOHIBOARD_ADC
