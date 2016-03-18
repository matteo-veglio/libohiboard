/******************************************************************************
 * Copyright (C) 2014-2016 A. C. Open Hardware Ideas Lab
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
 ******************************************************************************/

/**
 * @file libohiboard/include/dma.h
 * @author Matteo Civale <matteo.civale@gmail.com>
 * @brief DMA configuration and parameter.
 */

#ifdef LIBOHIBOARD_DMA

#ifndef __DMA_H
#define __DMA_H

#if defined (LIBOHIBOARD_K64F12)     || \
    defined (LIBOHIBOARD_FRDMK64F)   ||\
    defined (LIBOHIBOARD_K12D5)      ||\
    defined(LIBOHIBOARD_KL25Z4)

#include "platforms.h"
#include "errors.h"
#include "types.h"


typedef enum
{
	SURCE_NOT_USED     = 0x00,

	UART0_RECEIVE      = 0x02,
	UART0_TRANSMIT     = 0x03,

	UART1_RECEIVE      = 0x04,
	UART1_TRANSMIT     = 0x05,

	UART2_RECEIVE      = 0x06,
    UART2_TRANSMIT     = 0x07,

	UART3_RECEIVE      = 0x08,
	UART3_TRANSMIT     = 0x09,

	DAC_UPPER_POINTER  = 0x0A,
	DAC_BOTTOM_POINTER = 0x0B,
	DAC_BOOTH_POINTER  = 0x0C,

	ADC_CONV_COMPLETE  = 0x28,



}Dma_RequestSourceType;


typedef enum
{
	DMA_CHANNEL_0     =0,
	DMA_CHANNEL_1     =1,
	DMA_CHANNEL_2     =2,
	DMA_CHANNEL_3     =3,

#if	defined (LIBOHIBOARD_K64F12)   ||\
	defined (LIBOHIBOARD_FRDMK64F) ||\
	defined (LIBOHIBOARD_K12D5)

	DMA_CHANNEL_4     =4,
	DMA_CHANNEL_5     =5,
	DMA_CHANNEL_6     =6,
	DMA_CHANNEL_7     =7,
	DMA_CHANNEL_8     =8,
	DMA_CHANNEL_9     =9,
	DMA_CHANNEL_10    =10,
	DMA_CHANNEL_11    =11,
	DMA_CHANNEL_12    =12,
	DMA_CHANNEL_13    =13,
	DMA_CHANNEL_14    =14,
	DMA_CHANNEL_15    =15,
#endif
}Dma_ChannelType;

typedef enum
{
    DMA_8BIT    =0x00,
    DMA_16BIT   =0x01,
    DMA_32BIT   =0x02,
    DMA_16BYTE  =0x04,
    DMA_32BYTE  =0x05,

}Dma_DataSizeType;

typedef enum
{
	MODE_STEAL_CYCLE = 0x1,
	MODE_CONTINUOS   = 0x0,

} Dma_TransferModeType;





typedef struct dma_ConfigType
{

    Dma_ChannelType channel;

    Dma_RequestSourceType requestSource;
	uint32_t sourceAddress;
	uint32_t destinationAddress;

	/*source and destination for minor cycle*/
	uint8_t  sourceOff;
	uint8_t  destinationOff;

	/*source and destination data size*/
    Dma_DataSizeType sSize;
    Dma_DataSizeType dSize;

    /*number of byte for request*/
    uint32_t nByteforReq;

    /*counter of major iteration count*/
    uint32_t nOfCycle;

    /* Transfer mode*/
    Dma_TransferModeType transferMode;

	/*this is for trigger by pit*/
    uint8_t enableTimerTrig;

    /* Disable channel after transfer complete */
    bool disableAfterComplete;

    /*last source adjustment*/
    long int  lsAdjust;

    /*last destination adjustment*/
    long int  ldAdjust;


    /*Handler of the peripheral that trigger the DMA */
    void *pHandler;

}dma_ConfigType;




typedef struct Dma_Device* Dma_DeviceHandle;

extern Dma_DeviceHandle OB_DMA0;

/*************************************************************************************************************
 *                                                                                                           *
 *                                 This function initialize a DMA Channel                                     *
 *                                                                                                           *
 *************************************************************************************************************/

System_Errors  dma_init(Dma_DeviceHandle dev, dma_ConfigType* config, void *callback);

/*************************************************************************************************************
 *                                                                                                           *
 *                                 This function initialize a DMA Channel                                     *
 *                                                                                                           *
 *************************************************************************************************************/

void Dma_disableChannel(Dma_DeviceHandle dev, Dma_ChannelType channel);



#endif

#endif /* define __DMA_H */

#endif/* LIBOHIBOARD_DMA */
