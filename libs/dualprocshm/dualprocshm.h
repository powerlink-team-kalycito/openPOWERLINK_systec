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
\brief Interrupt source

The enum determines the address interrupt source
*/
typedef enum eDualprocIrqSrc
{
    kDualprocIrqSrcSync       = 0,        ///< sync irq
    kDualprocIrqSrcEvent      = 1,        ///< event irq
    kDualprocIrqSrcAsyncTx    = 2,        ///< async tx irq
    kDualprocIrqSrcAsyncRx    = 3,        ///< async rx irq
    kDualprocIrqSrcLast                   ///< last entry in enum
} tDualprocIrqSrc;

/**
\brief Function type definition for interrupt callback

This function callback is called for a given interrupt source, registered by the
host.
*/
typedef void (*tDualprocIrqCb) (void *pArg_p);

/**
\brief Resource Id

The Resource ids for the resources.
*/
typedef enum eDualprocResourceId
{
    kDualprocResIdDynBuff0 = 0,   ///< dynamic buffer 1
    kDualprocResIdDynBuff1,       ///< dynamic buffer 2
    kDualprocResIdErrCount ,      ///< error counters
    kDualprocResIdTxNmtQueue,     ///< NMT TX queue
    kDualprocResIdTxGenQueue,     ///< generic TX queue
    kDualprocResIdTxSyncQueue,    ///< sync TX queue
    kDualprocResIdTxVethQueue,    ///< VEth TX queue
    kDualprocResIdRxVethQueue,    ///< VEth RX queue
    kDualprocResIdK2UQueue,       ///< K2U Queue
    kDualprocResIdU2KQueue,       ///< U2K Queue
    kDualprocResIdPdo,            ///< Pdo
    kDualprocResIdLast            ///< last instance id
} tDualprocResourceId;

/**
\brief Driver instance configuration

Configures the driver instance.
*/
typedef struct sDualprocConfig
{
    tDualProcInstance   ProcInstance; ///< Processor instance (Pcp/Host)

} tDualprocConfig;

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
tDualprocDrvInstance dualprocshm_getInstance (tDualProcInstance Instance_p);

tDualprocReturn dualprocshm_process (tDualprocDrvInstance pInstance_p);

tDualprocReturn dualprocshm_irqRegHdl (tDualprocDrvInstance pInstance_p,
        tDualprocIrqSrc irqSrc_p, tDualprocIrqCb pfnCb_p);
tDualprocReturn dualprocshm_irqSourceEnable (tDualprocDrvInstance pInstance_p,
        tDualprocIrqSrc irqSrc_p, BOOL fEnable_p);
tDualprocReturn dualprocshm_irqMasterEnable (tDualprocDrvInstance pInstance_p,
        BOOL fEnable_p);

tDualprocReturn dualprocshm_setMagic (tDualprocDrvInstance pInstance_p, UINT16 magic_p);
tDualprocReturn dualprocshm_getMagic (tDualprocDrvInstance pInstance_p, UINT16 *pMagic_p);
tDualprocReturn dualprocshm_setCommand (tDualprocDrvInstance pInstance_p, UINT16 cmd_p);
tDualprocReturn dualprocshm_getCommand (tDualprocDrvInstance pInstance_p, UINT16 *pCmd_p);
tDualprocReturn dualprocshm_setStatus (tDualprocDrvInstance pInstance_p, UINT16 status_p);
tDualprocReturn dualprocshm_getStatus (tDualprocDrvInstance pInstance_p, UINT16 *pStatus_p);
tDualprocReturn dualprocshm_setRetVal (tDualprocDrvInstance pInstance_p, UINT16 retval_p);
tDualprocReturn dualprocshm_getRetVal (tDualprocDrvInstance pInstance_p, UINT16 *pRetval_p);
tDualprocReturn dualprocshm_setHeartbeat (tDualprocDrvInstance pInstance_p, UINT16 heartbeat_p);
tDualprocReturn dualprocshm_getHeartbeat (tDualprocDrvInstance pInstance_p, UINT16 *pHeartbeat_p);
tDualprocReturn dualprocshm_getDynRes(tDualprocDrvInstance pInstance_p,tDualprocResourceId resId_p,
                                        UINT8 **ppAddr_p, UINT16 *pSize_p);
tDualprocReturn dualprocshm_acquireQueueLock(UINT8 queueId_p);
tDualprocReturn dualprocshm_releaseQueueLock(UINT8 queueId_p);

#ifdef __cplusplus
}
#endif
#endif  // _INC_DUALPROCSHM_H_
