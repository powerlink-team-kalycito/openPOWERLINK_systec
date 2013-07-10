/*
 * hostiflib_arm.h
 *
 *  Created on: May 14, 2013
 *      Author: John
 */

#ifndef HOSTIFLIB_ARM_H_
#define HOSTIFLIB_ARM_H_

/**
********************************************************************************
\file   hostiflib_nios.h

\brief  Host Interface Library - For Nios II target

This header file provides specific macros for Altera Nios II soft-core CPU.

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holders nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
------------------------------------------------------------------------------*/

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
//FIXME:@John Check the appropriate headers
#include <stdint.h>
#include <stdlib.h>
#include <xil_cache.h>
#include <xscugic.h>
#include <xil_exception.h>
#include <unistd.h>
#include <xil_io.h>
#include <xparameters.h>
#include <xil_types.h>

#include "hostiflib_arm_cache.h"


// include section header file for special functions in
// tightly-coupled memory
#include <section-arm.h>//TODO:@John include specific arm files

// include generated header file for memory structure and version filed
#include "hostiflib-zynqmem.h"//TODO:@John include the zynq specific file

void hostif_FlushDCacheRange(u32 dwAddr_p,u16 span_p);
void hostif_InvalidateDCacheRange(u32 dwAddr_p,u16 span_p);

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
//TODO:@John Substitute the ARM specific memory addresses
#define PCP_0_HOSTINTERFACE_0_BASE	0x2C000000
#if defined(PCP_0_HOSTINTERFACE_0_BASE)

#define HOSTIF_PCP_BASE             PCP_0_HOSTINTERFACE_0_BASE //XPAR_PS7_DDR_0_S_AXI_HP0_BASEADDR
#define HOSTIF_HOST_BASE            PCP_0_HOSTINTERFACE_0_BASE //XPAR_PS7_DDR_0_S_AXI_BASEADDR

#define HOSTIF_IRQ_IC_ID            XPAR_PS7_SCUGIC_0_DEVICE_ID			 //FIXME: @Vinod Check Parameter is correct
#define HOSTIF_IRQ                  XPAR_FABRIC_AXI_POWERLINK_0_TCP_IRQ_INTR //FIXME: @Vinod Check Parameter is correct

#elif (defined(PCP_0_HOSTINTERFACE_0_PCP_BASE) && \
       defined(PCP_0_HOSTINTERFACE_0_HOST_BASE))
/* If one Nios II does Pcp and Host (makes no sense, but why not?) */
#define HOSTIF_PCP_BASE             PCP_0_HOSTINTERFACE_0_PCP_BASE
#define HOSTIF_HOST_BASE            PCP_0_HOSTINTERFACE_0_HOST_BASE

#endif
#define HOSTIF_USE_DCACHE 	TRUE
/*By pass Dcache*/
/* Can not be implemented on Zynq*/
#define ARM_BYPASS_DCACHE_MASK    (0x0 << 31)//Memory allocation used upto 31st bit, so 32nd bit is free


/// cache
/* Can not be implemented on Zynq*///TODO:@John Add these functionalities if possible
#define HOSTIF_MAKE_NONCACHEABLE(ptr)  \
    (void*)(((unsigned long)ptr)|ARM_BYPASS_DCACHE_MASK)

/* Can not be implemented on Zynq*///TODO:@John To be made non-cacheable
#define HOSTIF_UNCACHED_MALLOC(size)  xil_uncached_malloc(size)
#define HOSTIF_UNCACHED_FREE(ptr)     xil_uncached_free(ptr)

/// sleep
#define HOSTIF_USLEEP(x)              usleep((unsigned int)x)

/// hw access
#define HOSTIF_RD32(base, offset)         ARM_ZYNQ_RD_32DIRECT(base,(void*)offset)
#define HOSTIF_RD16(base, offset)         ARM_ZYNQ_RD_16DIRECT(base, (void*)offset)
#define HOSTIF_RD8(base, offset)          ARM_ZYNQ_RD_8DIRECT(base, (void*)offset)

#define HOSTIF_WR32(base, offset, dword)  ARM_ZYNQ_WR_32DIRECT(base, (void*)offset, dword)
#define HOSTIF_WR16(base, offset, word)   ARM_ZYNQ_WR_16DIRECT(base, (void*)offset, word)
#define HOSTIF_WR8(base, offset, byte)    ARM_ZYNQ_WR_8DIRECT(base, (void*)offset, byte)


/// irq handling
#define HOSTIF_IRQ_REG(cb, arg)     \
		SysComp_initSyncInterrupt(HOSTIF_IRQ, cb, arg)
    //alt_ic_isr_register(HOSTIF_IRQ_IC_ID, HOSTIF_IRQ, cb, arg, NULL)

#define HOSTIF_IRQ_ENABLE()         \
	XScuGic_EnableIntr(HOSTIF_IRQ_IC_ID, HOSTIF_IRQ);
//    alt_ic_irq_enable(HOSTIF_IRQ_IC_ID, HOSTIF_IRQ)

#define HOSTIF_IRQ_DISABLE()        \
		XScuGic_DisableIntr(HOSTIF_IRQ_IC_ID, HOSTIF_IRQ);
//    alt_ic_irq_disable(HOSTIF_IRQ_IC_ID, HOSTIF_IRQ)


//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------


#endif /* HOSTIFLIB_ARM_H_ */
