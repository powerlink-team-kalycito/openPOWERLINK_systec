/**
********************************************************************************
\file   dualprocshm-microblaze.h

\brief  Dual processor Library Target support header - For Microblaze target

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
#ifndef _INC_DUALPROCSHM_MICROBLAZE_H_
#define _INC_DUALPROCSHM_MICROBLAZE_H_


//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <stddef.h>
#include <xil_types.h>
#include <xil_cache.h>
#include "xintc_l.h"
#include <xilinx_usleep.h>
#include <xil_io.h>
#include <xparameters.h>
// include section header file for special functions in
// tightly-coupled memory
#include <section-microblaze.h> // TODO : Is this needed in absence
                                //        of TCM?

// Memory offset file for Zynq architecture
// NOTE : This file is handwritten
#include "dualprocshm-zynqmem.h"

int dualprocshm_RegisterHandler (u32 BaseAddress, int InterruptId,
	   XInterruptHandler Handler, void *CallBackRef);

#if defined(DUALPROCSHM_CMS_BASE)

#define DUALPROCSHM_CMS_BASE_PCP        DUALPROCSHM_CMS_BASE
#define DUALPROCSHM_CMS_BASE_HOST       DUALPROCSHM_CMS_BASE

//TODO: Assign Interrupt on Microblaze if Host?
#define DUALPROCSHM_IRQ_IC_ID    -1
#define DUALPROCSHM_IRQ          -1

#else

#warning "Common Memory base is assumed! Set the correct address!"

#define DUALPROCSHM_CMS_BASE_PCP             0x10000000
#define DUALPROCSHM_CMS_BASE_HOST            0x10000000
#define DUALPROCSHM_IRQ_IC_ID            0
#define DUALPROCSHM_IRQ                  0

#endif

// Added here to support Cache specific operation required in Zynq
// in absence of cache handling BSP routines
#if (XPAR_MICROBLAZE_USE_DCACHE == 1)
#define DUALPROCSHM_SYNC_DCACHE           TRUE
#else
#define DUALPROCSHM_SYNC_DCACHE           FALSE
#endif



#define DUALPROCSHM_MALLOC(size)            malloc(size)
#define DUALPROCSHM_FREE(ptr)               free(ptr)

/// sleep
#define DUALPROCSHM_USLEEP(x)               usleep(x)

/// hw access
#define DPSHM_RD32(base, offset)            MB_READ32(base, offset);
#define DPSHM_RD16(base, offset)            MB_READ16(base, offset);
#define DPSHM_RD8(base, offset)             MB_READ8(base, offset);

#define DPSHM_WR32(base, offset, dword)     MB_WRITE32(base,offset,dword);
#define DPSHM_WR16(base, offset, word)      MB_WRITE16(base,offset,word);
#define DPSHM_WR8(base, offset, byte)       MB_WRITE8(base,offset,byte);



#define DUALPROCSHM_REG_IRQHDL(cb, arg)  \
                    dualprocshm_RegisterHandler(DUALPROCSHM_IRQ_IC_ID,DUALPROCSHM_IRQ,cb,arg)
#define DUALPROCSHM_EN_IRQ()      \
                    XIntc_EnableIntr(DUALPROCSHM_IRQ_IC_ID,DUALPROCSHM_IRQ) //FIXME: Change the base address and mask as required

#define DUALPROCSHM_DISABLE_IRQ()     \
                    XIntc_DisableIntr(DUALPROCSHM_IRQ_IC_ID,DUALPROCSHM_IRQ) //FIXME: Change the base address and mask as required

#define DUALPROCSHM_FLUSH_DCACHE_RANGE(base,range) \
                    microblaze_flush_dcache_range(base, range);

#define DUALPROCSHM_INVALIDATE_DCACHE_RANGE(base,range) \
                    microblaze_invalidate_dcache_range(base, range);


#endif /* _INC_DUALPROCSHM_MICROBLAZE_H_ */
