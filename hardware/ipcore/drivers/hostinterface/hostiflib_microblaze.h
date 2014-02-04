/**
********************************************************************************
\file   hostiflib_microblaze.h

\brief  Host Interface Library - For Microblaze target

This header file provides specific macros for Xilinx Microblaze CPU.

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
#ifndef _INC_HOST_IF_MICROBLAZE_H_
#define _INC_HOST_IF_MICROBLAZE_H_


//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <stddef.h>
#include <xil_types.h>
#include <xil_cache.h>
#include "xintc_l.h"
#include <usleep.h>
#include <xil_io.h>
#include <xparameters.h>
#include <EplInc.h>
// include section header file for special functions in
// tightly-coupled memory
#include <section-microblaze.h>

// TODO : This file is handwritten
#include "hostiflib-mem_xilinx.h"

int hostiflib_RegisterHandler (u32 BaseAddress, int InterruptId,
        XInterruptHandler Handler, void *CallBackRef);

#if defined(XPAR_AXI_HOSTINTERFACE_0_BASEADDR)

#define HOSTIF_PCP_BASE     XPAR_AXI_HOSTINTERFACE_0_BASEADDR
#define HOSTIF_HOST_BASE    XPAR_AXI_HOSTINTERFACE_0_HOST_BASEADDR

//TODO: Assign Interrupt on Microblaze if Host?
#ifdef XPAR_HOST_INTC_DEVICE_ID

#define HOSTIF_IRQ_IC_ID    XPAR_HOST_INTC_BASEADDR
#define HOSTIF_IRQ          XPAR_HOST_INTC_AXI_HOSTINTERFACE_0_IRQOUT_IRQ_INTR

#else

#define HOSTIF_IRQ_IC_ID    -1
#define HOSTIF_IRQ          -1

#endif //XPAR_HOST_INTC_DEVICE_ID

#elif (defined(PCP_0_HOSTINTERFACE_0_PCP_BASE) && \
       defined(PCP_0_HOSTINTERFACE_0_HOST_BASE))

#define HOSTIF_PCP_BASE     PCP_0_HOSTINTERFACE_0_PCP_BASE
#define HOSTIF_HOST_BASE    PCP_0_HOSTINTERFACE_0_HOST_BASE

#else

#warning "Host Interface base is assumed! Set the correct address!"

#define HOSTIF_PCP_BASE             0x10000000
#define HOSTIF_HOST_BASE            0x10000000
#define HOSTIF_IRQ_IC_ID            0
#define HOSTIF_IRQ                  0

#endif

#if (XPAR_MICROBLAZE_USE_DCACHE == 1)
#define HOSTIF_SYNC_DCACHE           TRUE
#else
#define HOSTIF_SYNC_DCACHE           FALSE
#endif

#define HOSTIF_MAKE_NONCACHEABLE(ptr)       (void *) ptr

#define HOSTIF_UNCACHED_MALLOC(size)        malloc(size)
#define HOSTIF_UNCACHED_FREE(ptr)           free(ptr)

/// sleep
#define HOSTIF_USLEEP(x)                    usleep(x)

/// hw access
#define HOSTIF_RD32(base, offset)           MB_READ32(base, offset);
#define HOSTIF_RD16(base, offset)           MB_READ16(base, offset);
#define HOSTIF_RD8(base, offset)            MB_READ8(base, offset);

#define HOSTIF_WR32(base, offset, dword)    MB_WRITE32(base,offset,dword);
#define HOSTIF_WR16(base, offset, word)     MB_WRITE16(base,offset,word);
#define HOSTIF_WR8(base, offset, byte)      MB_WRITE8(base,offset,byte);



#define HOSTIF_IRQ_REG(cb, arg)  \
                    hostiflib_RegisterHandler(HOSTIF_IRQ_IC_ID,HOSTIF_IRQ,cb,arg)
#define HOSTIF_IRQ_ENABLE()      \
                    UINT32 mask= Xil_In32(HOSTIF_IRQ_IC_ID + XIN_IER_OFFSET);\
                    mask = (mask | (1 << HOSTIF_IRQ));              \
                    XIntc_EnableIntr(HOSTIF_IRQ_IC_ID,mask);//FIXME: Change the base address and mask as required

#define HOSTIF_IRQ_DISABLE()     \
                            UINT32 mask= Xil_In32(HOSTIF_IRQ_IC_ID + XIN_IER_OFFSET); \
                            mask = (mask & ~(1 << HOSTIF_IRQ));   \
                            XIntc_DisableIntr(HOSTIF_IRQ_IC_ID,mask); //FIXME: Change the base address and mask as required

#define HOSTIF_FLUSH_DCACHE_RANGE(base,range) \
                    microblaze_flush_dcache_range(base, range);

#define HOSTIF_INVALIDATE_DCACHE_RANGE(base,range) \
                    microblaze_invalidate_dcache_range(base, range);


#endif /* _INC_HOST_IF_MICROBLAZE_H_ */
