/**
********************************************************************************
\file       Zynq_ARM_CortexA9/include/systemComponents.h

\brief      Header file which contains processor specific definitions
            (Zynq ARM Cortex A9 Specific)

This header file contains of platform specific definitions.

Copyright © 2011 BERNECKER + RAINER, AUSTRIA, 5142 EGGELSBERG, B&R STRASSE 1
All rights reserved. All use of this software and documentation is
subject to the License Agreement located at the end of this file below.

*******************************************************************************/

#ifndef SYSTEMCOMPONENTS_H_
#define SYSTEMCOMPONENTS_H_

/******************************************************************************/
/* includes */
#include "xparameters.h"
#include "xparameters_ps.h"

#include "cnApiGlobal.h"
#include "cnApiCfg.h"

#include <string.h>
#include <stdio.h>

/******************************************************************************/
/* defines */
#define SLCR_LOCK			0xF8000004 /**< SLCR Write Protection Lock */
#define SLCR_UNLOCK			0xF8000008 /**< SLCR Write Protection Unlock */
#define AFI_WRCHAN_CTRL2 	0xF800A014
#define AFI_RDCHAN_CTRL2 	0xF800A000
#define FPGA_RST_CNTRL   	0xF8000240

#define SLCR_LOCK_VAL		0x767B
#define SLCR_UNLOCK_VAL		0xDF0D
#define AFI_WRCHAN_CONFIG	0x00000F01 //32 bit enable, 16 beats
#define AFI_RDCHAN_CONFIG	0x00000001 //32 bit enable
#define DEFAULT_PRIORITY	0xa0a0a0a0UL

#define SYNC_INTR_PRIORITY		0x00		//lower the value, higher the priority
#define ASYNC_INTR_PRIORITY		0x01		//lower the value, higher the priority
#define TRIGGER_VALUE			0x0			//For SPI, 0X --> High-level senstive
											//1X --> rising edge (bit 2 is readonly)
#if (XPAR_CPU_ID == 0)
#define TARGET_CPU_VALUE 0x01
#else
#define TARGET_CPU_VALUE 0x02
#endif

#define XSCUGIC_INT_CFG_OFFSET_CALC(InterruptID) \
    (XSCUGIC_INT_CFG_OFFSET + ((InterruptID/16) * 4))

#define XSCUGIC_PRIORITY_OFFSET_CALC(InterruptID) \
    (XSCUGIC_PRIORITY_OFFSET + ((InterruptID/4) * 4))

#define XSCUGIC_SPI_TARGET_OFFSET_CALC(InterruptID) \
    (XSCUGIC_SPI_TARGET_OFFSET + ((InterruptID/4) * 4))

#define XSCUGIC_ENABLE_DISABLE_OFFSET_CALC(Register, InterruptID) \
    (Register + ((InterruptID/32) * 4))

/******************************************************************************/

// PDI DPRAM offset
#ifdef CN_API_USING_SPI
	#define PDI_DPRAM_BASE_AP    0x00                       ///< no base address necessary
#elif defined(CN_API_USING_16BIT) || defined(CN_API_USING_8BIT)
	#define PDI_DPRAM_BASE_AP    XPAR_AXI_EMC_0_S_AXI_MEM0_BASEADDR
#else
	#define PDI_DPRAM_BASE_AP    XPAR_AXI_POWERLINK_0_S_AXI_PDI_AP_BASEADDR           ///< from xparameters.h
#endif /* CN_API_USING_SPI */

#if defined(CN_API_USING_16BIT) || defined(CN_API_USING_8BIT)
  #ifdef XPAR_AP_INTC_SYSTEM_AXI_POWERLINK_0_AP_SYNCIRQ_PIN_INTR
   #define SYNC_IRQ_NUM			XPAR_AP_INTC_SYSTEM_AXI_POWERLINK_0_AP_SYNCIRQ_PIN_INTR
   #define SYNC_IRQ_NUM_MASK	XPAR_SYSTEM_AXI_POWERLINK_0_AP_SYNCIRQ_PIN_MASK
  #endif //XPAR_AP_INTC_SYSTEM_AXI_POWERLINK_0_AP_SYNCIRQ_PIN_INTR
#else
  #ifdef XPAR_FABRIC_AXI_POWERLINK_0_AP_SYNCIRQ_VEC_ID
   #define SYNC_IRQ_NUM			XPAR_FABRIC_AXI_POWERLINK_0_AP_SYNCIRQ_INTR
   //#define SYNC_IRQ_NUM_MASK	XPAR_AXI_POWERLINK_0_AP_SYNCIRQ_MASK
  #endif //XPAR_FABRIC_AXI_POWERLINK_0_AP_SYNCIRQ_VEC_ID
#endif
#if defined(CN_API_USING_16BIT) || defined(CN_API_USING_8BIT)
  #ifdef XPAR_AP_INTC_SYSTEM_AXI_POWERLINK_0_AP_ASYNCIRQ_PIN_INTR
   #define ASYNC_IRQ_NUM		XPAR_AP_INTC_SYSTEM_AXI_POWERLINK_0_AP_ASYNCIRQ_PIN_INTR
   #define ASYNC_IRQ_NUM_MASK	XPAR_SYSTEM_AXI_POWERLINK_0_AP_ASYNCIRQ_PIN_MASK
  #endif //XPAR_AP_INTC_PLB_POWERLINK_0_AP_ASYNCIRQ_INTR
#else
  #ifdef XPAR_FABRIC_AXI_POWERLINK_0_AP_ASYNCIRQ_VEC_ID
   #define ASYNC_IRQ_NUM		XPAR_FABRIC_AXI_POWERLINK_0_AP_ASYNCIRQ_INTR
   //#define ASYNC_IRQ_NUM_MASK	XPAR_AXI_POWERLINK_0_AP_ASYNCIRQ_MASK
  #endif //XPAR_FABRIC_AXI_POWERLINK_0_AP_ASYNCIRQ_VEC_ID
#endif

#ifdef XPAR_AP_OUTPUT_BASEADDR
	#define OUTPORT_AP_BASE_ADDRESS		XPAR_AP_OUTPUT_BASEADDR
#endif

#ifdef XPAR_AP_INPUT_BASEADDR
	#define INPORT_AP_BASE_ADDRESS		XPAR_AP_INPUT_BASEADDR
#endif

/******************************************************************************/
/* typedefs */

/******************************************************************************/
/* external variable declarations */

/******************************************************************************/
/* global variables */

/******************************************************************************/
/* function declarations */
void SysComp_initPeripheral(void);
void SysComp_InitInterrupts(void);

inline void SysComp_enableInterrupts(void);
inline void SysComp_disableInterrupts(void);
void SysComp_freeProcessorCache(void);

int SysComp_initSyncInterrupt(void (*callbackFunc)(void*));
int SysComp_initAsyncInterrupt(void (*callbackFunc)(void*));

inline void SysComp_enableSyncInterrupt(void);
inline void SysComp_disableSyncInterrupt(void);

inline void SysComp_enableAsyncInterrupt(void);
inline void SysComp_disableAsyncInterrupt(void);

#ifdef CN_API_USING_SPI
int SysComp_SPICommand(unsigned char *pTxBuf_p, unsigned char *pRxBuf_p, int iBytes_p);
#endif

void SysComp_writeOutputPort(DWORD dwValue_p);
DWORD SysComp_readInputPort();

#endif /* SYSTEMCOMPONENTS_H_ */

/*******************************************************************************
*
* License Agreement
*
* Copyright © 2011 BERNECKER + RAINER, AUSTRIA, 5142 EGGELSBERG, B&R STRASSE 1
* All rights reserved.
*
* Redistribution and use in source and binary forms,
* with or without modification,
* are permitted provided that the following conditions are met:
*
*   * Redistributions of source code must retain the above copyright notice,
*     this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above copyright notice,
*     this list of conditions and the following disclaimer
*     in the documentation and/or other materials provided with the
*     distribution.
*   * Neither the name of the B&R nor the names of its contributors
*     may be used to endorse or promote products derived from this software
*     without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
* THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
* A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
* THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*******************************************************************************/
/* END-OF-FILE */
