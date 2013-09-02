/**
********************************************************************************
\file   dualprocshm-arm.h

\brief  Dual Processor Library Target support Header - For ARM target

This header file provides specific macros for Zynq ARM(ps7_cortexa9_0) CPU.

*******************************************************************************/
/*------------------------------------------------------------------------------
Copyright (c) 2012 Kalycito Infotech Private Limited
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


#ifndef _INC_DUALPROCSHM_ARM_H_
#define _INC_DUALPROCSHM_ARM_H_


//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <stdint.h>
#include <stdlib.h>
#include <xil_cache.h>
#include <xscugic.h>
#include <xil_exception.h>
#include <unistd.h>
#include <xil_io.h>
#include <xparameters.h>
#include <xil_types.h>
#include <systemComponents.h>

// include section header file for special functions in
// tightly-coupled memory
//#include <section-arm.h>//TODO:Check if this is needed with no TCM

// include generated header file for memory structure and version filed
#include "dualprocshm-zynqmem.h"//TODO:@John include the zynq specific file

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

#if defined(DUALPROCSHM_CMS_BASE)

#define DUALPROCSHM_CMS_BASE_PCP         DUALPROCSHM_CMS_BASE
#define DUALPROCSHM_CMS_BASE_HOST        DUALPROCSHM_CMS_BASE

#define DUALPROCSHM_IRQ_IC_ID            XPAR_PS7_SCUGIC_0_DEVICE_ID                 //FIXME: @Vinod Check Parameter is correct
#define DUALPROCSHM_IRQ                  XPAR_FABRIC_AXI_POWERLINK_0_TCP_IRQ_INTR    //FIXME: @Vinod Check Parameter is correct

#else

#warning "Common Memory base is assumed! Set the correct address!"

#define DUALPROCSHM_CMS_BASE_PCP             0x10000000
#define DUALPROCSHM_CMS_BASE_HOST            0x10000000
#define DUALPROCSHM_IRQ_IC_ID            0
#define DUALPROCSHM_IRQ                  0

#endif


/// cache
#define DUALPROCSHM_MALLOC(size)            malloc(size)
#define DUALPROCSHM_FREE(ptr)               free(ptr)

/// sleep
#define DUALPROCSHM_USLEEP(x)               usleep((unsigned int)x)

/// hw access
#define DPSHM_RD32(base, offset)           ARM_READ32(base,offset)
#define DPSHM_RD16(base, offset)           ARM_READ16(base,offset)
#define DPSHM_RD8(base, offset)            ARM_READ8(base,offset)

#define DPSHM_WR32(base, offset, dword)    ARM_WRITE32(base, offset, dword)
#define DPSHM_WR16(base, offset, word)     ARM_WRITE16(base, offset, word)
#define DPSHM_WR8(base, offset, byte)      ARM_WRITE8(base, offset, byte)


/// irq handling
#define DUALPROCSHM_REG_IRQHDL(cb, arg)     \
                    SysComp_initSyncInterrupt(DUALPROCSHM_IRQ, cb, arg)

#define DUALPROCSHM_EN_IRQ()         \
                    XScuGic_EnableIntr(DUALPROCSHM_IRQ_IC_ID, DUALPROCSHM_IRQ);

#define DUALPROCSHM_DISABLE_IRQ()        \
                    XScuGic_DisableIntr(DUALPROCSHM_IRQ_IC_ID, DUALPROCSHM_IRQ);

#define DUALPROCSHM_FLUSH_DCACHE_RANGE(base,range) \
                    Xil_DCacheFlushRange(base, range);

#define DUALPROCSHM_INVALIDATE_DCACHE_RANGE(base,range) \
                    Xil_DCacheInvalidateRange(base, range);


//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------


#endif /* _INC_DUALPROCSHM_ARM_H_ */
