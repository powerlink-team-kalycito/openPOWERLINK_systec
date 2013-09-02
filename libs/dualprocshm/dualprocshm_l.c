/**
********************************************************************************
\file   dualprocshm_l.c

\brief  Dual processor shared memory library - Low Level Driver Source

The Dual processor shared memory Low Level Driver provides access to the
common memory register structure.

\ingroup module_dualprocshm
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

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include "dualprocshm_l.h"
#include "dualprocshm-target.h"


//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//-----------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
tCommonMemStruc *pCommonMemStruc_l;
//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
void dualprocshm_initMem(UINT8* pAaddr_p)
{
    if(pAaddr_p != NULL)
        pCommonMemStruc_l = (tCommonMemStruc *)pAaddr_p;
}
//------------------------------------------------------------------------------
/**
\brief  Read magic field

\return The function returns the magic field.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
UINT16 dualprocshm_readMagic (void)
{
    return DPSHM_RD16(&pCommonMemStruc_l->controlStatus,
            offsetof(tCmsCont, magic));
}
//------------------------------------------------------------------------------
/**
\brief  Write magic field

\param  val_p    pattern to be written

\return The function returns the magic field.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
void dualprocshm_writeMagic (UINT16 val_p)
{
   DPSHM_WR16(&pCommonMemStruc_l->controlStatus,
            offsetof(tCmsCont, magic),val_p);
}
//------------------------------------------------------------------------------
/**
\brief  Read command field

\return The function returns the command field.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
UINT16 dualprocshm_readCommand (void)
{
    return DPSHM_RD16(&pCommonMemStruc_l->controlStatus,
            offsetof(tCmsCont, command));
}
//------------------------------------------------------------------------------
/**
\brief  Write command field

\param  val_p    pattern to be written

\return The function returns the command field.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
void dualprocshm_writeCommand (UINT16 val_p)
{
   DPSHM_WR16(&pCommonMemStruc_l->controlStatus,
            offsetof(tCmsCont, command),val_p);
}
//------------------------------------------------------------------------------
/**
\brief  Read status field

\return The function returns the status field.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
UINT16 dualprocshm_readStatus (void)
{
    return DPSHM_RD16(&pCommonMemStruc_l->controlStatus,
            offsetof(tCmsCont, status));
}
//------------------------------------------------------------------------------
/**
\brief  Write status field

\param  val_p    pattern to be written

\return The function returns the status field.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
void dualprocshm_writeStatus (UINT16 val_p)
{
    DPSHM_WR16(&pCommonMemStruc_l->controlStatus,
            offsetof(tCmsCont, status),val_p);
}
//------------------------------------------------------------------------------
/**
\brief  Read heart beat field

\return The function returns the heart beat field.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
UINT16 dualprocshm_readHeartbeat (void)
{
    return DPSHM_RD16(&pCommonMemStruc_l->controlStatus,
            offsetof(tCmsCont, heartbeat));
}
//------------------------------------------------------------------------------
/**
\brief  Write heart beat field

\param  val_p    pattern to be written

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
void dualprocshm_writeHeartbeat (UINT16 val_p)
{
    DPSHM_WR16(&pCommonMemStruc_l->controlStatus,
            offsetof(tCmsCont, heartbeat),val_p);
}
//------------------------------------------------------------------------------
/**
\brief  Read return value field

\return The function returns the return value field.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
UINT16 dualprocshm_readRetVal (void)
{
    return DPSHM_RD16(&pCommonMemStruc_l->controlStatus,
            offsetof(tCmsCont, retval));
}
//------------------------------------------------------------------------------
/**
\brief  Write return value field

\param  val_p    pattern to be written

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
void dualprocshm_writeRetVal (UINT16 val_p)
{
    DPSHM_WR16(&pCommonMemStruc_l->controlStatus,
            offsetof(tCmsCont, retval),val_p);
}
//------------------------------------------------------------------------------
/**
\brief  Acknowledge interrupt from PCP

\param  val_p   pattern to be written

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
void dualprocshm_ackIrq (UINT16 val_p)
{
    DPSHM_WR16(&pCommonMemStruc_l->syncLock,
            offsetof(tCmsSyncLock, irqAck),val_p);
}
//------------------------------------------------------------------------------
/**
\brief  Read pending interrupts from PCP

\param  val_p   pattern to be written

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
UINT16 dualprocshm_readPendingIrq (void)
{
    return DPSHM_RD16(&pCommonMemStruc_l->syncLock,
            offsetof(tCmsSyncLock, irqPending));
}
//------------------------------------------------------------------------------
/**
\brief  Set interrupts to HOST

\param  val_p   pattern to be written

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
void dualprocshm_setIrq (UINT16 val_p)
{
    DPSHM_WR16(&pCommonMemStruc_l->syncLock,
            offsetof(tCmsSyncLock, irqSet),val_p);
}
//------------------------------------------------------------------------------
/**
\brief  Read queue lock register

\return The function returns the queue lock field.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
UINT16 dualprocshm_readQueueLock (void)
{
    return DPSHM_RD16(&pCommonMemStruc_l->syncLock,
            offsetof(tCmsSyncLock, queueLock));
}
//------------------------------------------------------------------------------
/**
\brief  write queue lock register

\param  val_p   pattern to be written

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
void dualprocshm_writeQueueLock (UINT16 val_p)
{
    DPSHM_WR16(&pCommonMemStruc_l->syncLock,
            offsetof(tCmsSyncLock, queueLock),val_p);
}
//------------------------------------------------------------------------------
/**
\brief  Read dynamic buffer 1 register

\return The function returns the dynamic buff 1 field.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
UINT32 dualprocshm_readDynResBuff0 (void)
{
    return DPSHM_RD32(&pCommonMemStruc_l->dynSharedRes,
            offsetof(tCmsDynRes, DynBuf0));
}
//------------------------------------------------------------------------------
/**
\brief  Write dynamic buffer 1 register

\param  addr_p address to be written

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
void dualprocshm_writeDynResBuff0 (UINT32 addr_p)
{
    DPSHM_WR32(&pCommonMemStruc_l->dynSharedRes,
            offsetof(tCmsDynRes, DynBuf0),addr_p);
}
//------------------------------------------------------------------------------
/**
\brief  Read dynamic buffer 2 register

\return The function returns the dynamic buff 2 field.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
UINT32 dualprocshm_readDynResBuff1 (void)
{
    return DPSHM_RD32(&pCommonMemStruc_l->dynSharedRes,
            offsetof(tCmsDynRes, DynBuf1));
}
//------------------------------------------------------------------------------
/**
\brief  Write dynamic buffer 2 register

\param  addr_p address to be written

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
void dualprocshm_writeDynResBuff1 (UINT32 addr_p)
{
    DPSHM_WR32(&pCommonMemStruc_l->dynSharedRes,
            offsetof(tCmsDynRes, DynBuf1),addr_p);
}
//------------------------------------------------------------------------------
/**
\brief  Read error counter register

\return The function returns the error counter field.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
UINT32 dualprocshm_readDynResErrCnt (void)
{
    return DPSHM_RD32(&pCommonMemStruc_l->dynSharedRes,
            offsetof(tCmsDynRes, errorCounterAddr));
}
//------------------------------------------------------------------------------
/**
\brief  Write error counter register

\param  addr_p address to be written

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
void dualprocshm_writeDynResErrCnt (UINT32 addr_p)
{
    DPSHM_WR32(&pCommonMemStruc_l->dynSharedRes,
            offsetof(tCmsDynRes, errorCounterAddr),addr_p);
}
//------------------------------------------------------------------------------
/**
\brief  Read Tx NMT queue register

\return The function returns the Tx NMT queue address field.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
UINT32 dualprocshm_readDynResTxNmtQ (void)
{
    return DPSHM_RD32(&pCommonMemStruc_l->dynSharedRes,
            offsetof(tCmsDynRes, txNmtQAddr));
}
//------------------------------------------------------------------------------
/**
\brief  Write Tx NMT queue register

\param  addr_p address to be written

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
void dualprocshm_writeDynResTxNmtQ (UINT32 addr_p)
{
    DPSHM_WR32(&pCommonMemStruc_l->dynSharedRes,
            offsetof(tCmsDynRes, txNmtQAddr),addr_p);
}
//------------------------------------------------------------------------------
/**
\brief  Read Tx generic queue register

\return The function returns the Tx generic queue address field.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
UINT32 dualprocshm_readDynResTxGenQ (void)
{
    return DPSHM_RD32(&pCommonMemStruc_l->dynSharedRes,
            offsetof(tCmsDynRes, txGenQAddr));
}
//------------------------------------------------------------------------------
/**
\brief  Write Tx generic queue register

\param  addr_p address to be written

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
void dualprocshm_writeDynResTxGenQ (UINT32 addr_p)
{
    DPSHM_WR32(&pCommonMemStruc_l->dynSharedRes,
            offsetof(tCmsDynRes, txGenQAddr),addr_p);
}
//------------------------------------------------------------------------------
/**
\brief  Read Tx Sync queue register

\return The function returns the Tx Sync queue address field.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
UINT32 dualprocshm_readDynResTxSyncQ (void)
{
    return DPSHM_RD32(&pCommonMemStruc_l->dynSharedRes,
            offsetof(tCmsDynRes, txSyncQAddr));
}
//------------------------------------------------------------------------------
/**
\brief  Write Tx Sync queue register

\param  addr_p address to be written

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
void dualprocshm_writeDynResTxSyncQ (UINT32 addr_p)
{
    DPSHM_WR32(&pCommonMemStruc_l->dynSharedRes,
            offsetof(tCmsDynRes, txSyncQAddr),addr_p);
}
//------------------------------------------------------------------------------
/**
\brief  Read Tx Virtual Ethernet queue register

\return The function returns the Tx virtual Ethernet queue address field.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
UINT32 dualprocshm_readDynResTxVethQ (void)
{
    return DPSHM_RD32(&pCommonMemStruc_l->dynSharedRes,
            offsetof(tCmsDynRes, txVethQAddr));
}
//------------------------------------------------------------------------------
/**
\brief  Write Tx Virtual Ethernet queue register

\param  addr_p address to be written

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
void dualprocshm_writeDynResTxVethQ (UINT32 addr_p)
{
    DPSHM_WR32(&pCommonMemStruc_l->dynSharedRes,
            offsetof(tCmsDynRes, txVethQAddr),addr_p);
}
//------------------------------------------------------------------------------
/**
\brief  Read Rx Virtual Ethernet queue register

\return The function returns the Rx virtual Ethernet queue address field.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
UINT32 dualprocshm_readDynResRxVethQ (void)
{
    return DPSHM_RD32(&pCommonMemStruc_l->dynSharedRes,
            offsetof(tCmsDynRes, rxVethQAddr));
}
//------------------------------------------------------------------------------
/**
\brief  Write Rx Virtual Ethernet queue register

\param  addr_p address to be written

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
void dualprocshm_writeDynResRxVethQ (UINT32 addr_p)
{
    DPSHM_WR32(&pCommonMemStruc_l->dynSharedRes,
            offsetof(tCmsDynRes, rxVethQAddr),addr_p);
}
//------------------------------------------------------------------------------
/**
\brief  Read Kernel to User event queue register

\return The function returns the Kernel to User event queue address field.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
UINT32 dualprocshm_readDynResK2UQ (void)
{
    return DPSHM_RD32(&pCommonMemStruc_l->dynSharedRes,
            offsetof(tCmsDynRes, K2UQAddr));
}
//------------------------------------------------------------------------------
/**
\brief  Write Kernel to User event queue register

\param  addr_p address to be written

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
void dualprocshm_writeDynResK2UQ (UINT32 addr_p)
{
    DPSHM_WR32(&pCommonMemStruc_l->dynSharedRes,
            offsetof(tCmsDynRes, K2UQAddr),addr_p);
}
//------------------------------------------------------------------------------
/**
\brief  Read User to Kernel event queue register

\return The function returns the User to Kernel event queue address field.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
UINT32 dualprocshm_readDynResU2KQ (void)
{
    return DPSHM_RD32(&pCommonMemStruc_l->dynSharedRes,
            offsetof(tCmsDynRes, U2KQAddr));
}
//------------------------------------------------------------------------------
/**
\brief  Write User to Kernel event queue register

\param  addr_p address to be written

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
void dualprocshm_writeDynResU2KQ (UINT32 addr_p)
{
    DPSHM_WR32(&pCommonMemStruc_l->dynSharedRes,
            offsetof(tCmsDynRes, U2KQAddr),addr_p);
}
//------------------------------------------------------------------------------
/**
\brief  Read TPDO address register

\return The function returns the TPDO address field.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
UINT32 dualprocshm_readDynResPdo (void)
{
    return DPSHM_RD32(&pCommonMemStruc_l->dynSharedRes,
            offsetof(tCmsDynRes, PDOAddr));
}
//------------------------------------------------------------------------------
/**
\brief  Write TPDO address register

\param  addr_p      address to be written

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
void dualprocshm_writeDynResPdo (UINT32 addr_p)
{
    DPSHM_WR32(&pCommonMemStruc_l->dynSharedRes,
            offsetof(tCmsDynRes, PDOAddr),addr_p);
}
