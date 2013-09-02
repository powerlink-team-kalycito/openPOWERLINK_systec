/**
********************************************************************************
\file   dualprocshm_l.h

\brief  Dual Processor Library - Low level driver Header

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

#ifndef _INC_DUALPROCSHM_L_H_
#define _INC_DUALPROCSHM__L_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include "dualprocshm-target.h"
#include "dualprocshm.h"
//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
/**
\brief Common Memory structure - Status/Control

The control sub-registers provide basic Pcp-to-Host communication features.
*/
typedef struct sCmsCont
{
    volatile UINT16     magic;      ///< enable the bridge logic
    volatile UINT16     status;     ///< reserved
    volatile UINT16     heartbeat;  ///< heart beat word
    volatile UINT16     command;    ///< command word
    volatile UINT16     retval;     ///< return word
    UINT16              RESERVED0;  ///< reserved
} tCmsCont;

/**
\brief Common Memory structure - Synchronization/Lock

Interrupt synchronization and locks for queues.
*/
typedef struct sCmsSyncLock
{
    UINT16              irqEnable;   ///< enable irqs
    volatile UINT16     irqMasterEnable; ///< enable master irq
    union
    {
       volatile UINT16 irqSet;      ///< set irq (Pcp)
       volatile UINT16 irqAck;      ///< acknowledge irq (Host)
       volatile UINT16 irqPending;  ///< pending irq
    };
    volatile UINT16 queueLock; //TODO: @gks revise this

} tCmsSyncLock;

/**
\brief Common Memory structure - Dynamic Shared Memory Address

The dynamic buffer's address registers.
*/
typedef struct sCmsDynRes
{
    volatile UINT32     DynBuf0;          ///< base of dynamic buffer 1
    volatile UINT32     DynBuf1;          ///< base of dynamic buffer 2
    volatile UINT32     errorCounterAddr; ///< base of error counters
    volatile UINT32     txNmtQAddr;       ///< base of Tx NMT queue
    volatile UINT32     txGenQAddr;       ///< base of Tx generic queue
    volatile UINT32     txSyncQAddr;      ///< base of Tx sync queue
    volatile UINT32     txVethQAddr;      ///< base of Tx Virtual Ethernet queue
    volatile UINT32     rxVethQAddr;      ///< base of Rx Virtual Ethernet queue
    volatile UINT32     K2UQAddr;         ///< base of Kernel-to-User queue
    volatile UINT32     U2KQAddr;         ///< base of User-to-Kernel queue
    volatile UINT32     PDOAddr;          ///< base of Pdo buffers
} tCmsDynRes;

/**
\brief Common Memory structure

Common memory structure for Control and status exchange.
*/
typedef struct sCommonMemStruc
{
    tCmsCont        controlStatus;      ///< control and status registers
    tCmsSyncLock    syncLock;           ///< synchronization and control registers
    tCmsDynRes      dynSharedRes;       ///< dynamic shared memory resources
}tCommonMemStruc;
//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

void dualprocshm_initMem (UINT8* pAaddr_p);
UINT16 dualprocshm_readMagic (void);
void dualprocshm_writeMagic (UINT16 val_p);
UINT16 dualprocshm_readCommand (void);
void dualprocshm_writeCommand (UINT16 val_p);
UINT16 dualprocshm_readStatus (void);
void dualprocshm_writeStatus (UINT16 val_p);
UINT16 dualprocshm_readHeartbeat (void);
void dualprocshm_writeHeartbeat (UINT16 val_p);
UINT16 dualprocshm_readRetVal (void);
void dualprocshm_writeRetVal (UINT16 val_p);
void dualprocshm_ackIrq (UINT16 val_p);
UINT16 dualprocshm_readPendingIrq (void);
void dualprocshm_setIrq (UINT16 val_p);
UINT16 dualprocshm_readQueueLock (void);
void dualprocshm_writeQueueLock (UINT16 val_p);
UINT32 dualprocshm_readDynResBuff0 (void);
void dualprocshm_writeDynResBuff0 (UINT32 addr_p);
UINT32 dualprocshm_readDynResBuff1 (void);
void dualprocshm_writeDynResBuff1 (UINT32 addr_p);
UINT32 dualprocshm_readDynResErrCnt (void);
void dualprocshm_writeDynResErrCnt (UINT32 addr_p);
UINT32 dualprocshm_readDynResTxNmtQ (void);
void dualprocshm_writeDynResTxNmtQ (UINT32 addr_p);
UINT32 dualprocshm_readDynResTxGenQ (void);
void dualprocshm_writeDynResTxGenQ (UINT32 addr_p);
UINT32 dualprocshm_readDynResTxSyncQ (void);
void dualprocshm_writeDynResTxSyncQ (UINT32 addr_p);
UINT32 dualprocshm_readDynResTxVethQ (void);
void dualprocshm_writeDynResTxVethQ (UINT32 addr_p);
UINT32 dualprocshm_readDynResRxVethQ (void);
void dualprocshm_writeDynResRxVethQ (UINT32 addr_p);
UINT32 dualprocshm_readDynResK2UQ (void);
void dualprocshm_writeDynResK2UQ (UINT32 addr_p);
UINT32 dualprocshm_readDynResU2KQ (void);
void dualprocshm_writeDynResU2KQ (UINT32 addr_p);
UINT32 dualprocshm_readDynResPdo (void);
void dualprocshm_writeDynResPdo (UINT32 addr_p);


#ifdef __cplusplus
}
#endif

#endif // _INC_DUALPROCSHM__L_H_
