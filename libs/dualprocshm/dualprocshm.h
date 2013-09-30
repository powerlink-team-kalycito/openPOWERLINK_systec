/**
********************************************************************************
\file   dualprocshm.h

\brief  Dual Processor Library - Header

Dual processor library provides routines for initialization of memory
and interrupt resources for a shared memory interface between dual
processor.

*******************************************************************************/
/*------------------------------------------------------------------------------
Copyright (c) 2012 Kalycito Infotech Private Limited
              www.kalycito.com
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
#ifndef _INC_DUALPROCSHM_H_
#define _INC_DUALPROCSHM_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include "dualprocshm-target.h"

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

#define  DUALPROC_DYNBUF_COUNT       2 ///< number of supported dynamic buffers
//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
/**
\brief Return codes
*/
typedef enum eDualprocReturn
{
    kDualprocSuccessful     = 0x0000,       ///< no error / successful run
    kDualprocNoResource     = 0x0001,       ///< resource could not be created
    kDualprocInvalidParameter = 0x0002,     ///< function parameter invalid
    kDualprocWrongProcInst  = 0x0003,       ///< Processor instance wrong
    kDualprocHwWriteError   = 0x0004,       ///< write to hw failed
    kDualprocBufferOverflow = 0x0005,       ///< buffer size overflow
    kDualprocBufferEmpty    = 0x0006,       ///< buffer is empty
    kDualprocBufferError    = 0x0007,       ///< buffer is faulty

    kDualprocUnspecError    = 0xFFFF        ///< unspecified error
} tDualprocReturn;

/**
\brief Processor instance

The processor instance determines if the caller is the Pcp or the Host.
*/
typedef enum eDualProcInstance
{
    kDualProcPcp        = 0,            ///< instance on PCP
    kDualProcHost       = 1,            ///< instance on Host

} tDualProcInstance;

/**
\brief Driver instance configuration

Configures the driver instance.
*/
typedef struct sDualprocConfig
{
    tDualProcInstance   ProcInstance; ///< Processor instance (Pcp/Host)
    UINT16              commMemSize;  ///< minimum size of common memory
    UINT8               procId;

} tDualprocConfig;

typedef struct sDualprocMemInst
{
    UINT16     span;   ///< span of the dynamic buffer
    UINT8      lock;   ///< lock for memory
    UINT8      resv;   ///< reserved byte;
}tDualprocMemInst;


/**
\brief Driver Instance
*/
typedef void* tDualprocDrvInstance;


//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

tDualprocReturn dualprocshm_create (tDualprocConfig *pConfig_p, tDualprocDrvInstance *ppInstance_p);
tDualprocReturn dualprocshm_delete (tDualprocDrvInstance pInstance_p);
tDualprocDrvInstance dualprocshm_getDrvInst (tDualProcInstance Instance_p);
tDualprocReturn dualprocshm_getMemory(tDualprocDrvInstance pInstance_p, UINT8 Id_p,
                                      UINT8 **ppAddr_p, size_t *pSize_p, BOOL fAlloc_p);
tDualprocReturn dualprocshm_freeMemory(tDualprocDrvInstance pInstance_p, UINT8 Id_p,
                                                                    BOOL fFree_p);
tDualprocReturn dualprocshm_writeData(tDualprocDrvInstance pInstance_p, UINT8 Id_p,
                                        UINT32 offset_p,size_t Size_p, UINT8* pData_p);
tDualprocReturn dualprocshm_readData(tDualprocDrvInstance pInstance_p, UINT8 Id_p,
                                        UINT32 offset_p,size_t Size_p, UINT8* pData_p);
tDualprocReturn dualprocshm_readDataCommon(tDualprocDrvInstance pInstance_p,UINT32 offset_p,
                                                            size_t Size_p,UINT8* pData_p);
tDualprocReturn dualprocshm_writeDataCommon(tDualprocDrvInstance pInstance_p,UINT32 offset_p,
                                                            size_t  Size_p, UINT8* pData_p);


tDualprocReturn dualprocshm_acquireBuffLock(tDualprocDrvInstance pInstance_p, UINT8 Id_p);
tDualprocReturn dualprocshm_releaseBuffLock(tDualprocDrvInstance pInstance_p, UINT8 Id_p);

#ifdef __cplusplus
}
#endif
#endif  // _INC_DUALPROCSHM_H_
