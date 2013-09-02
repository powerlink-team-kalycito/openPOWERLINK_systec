/**
********************************************************************************
\file   pdoucalsync-noosdual.c

\brief  Dual Processor PDO CAL user sync module

This file contains the dual processor shared memory sync implementation for
the PDO user CAL module.

\ingroup module_pdoucal
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
#include <EplInc.h>
#include <pdo.h>
#include <user/pdoucal.h>

#include <dualprocshm.h>

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
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tEplSyncCb pfnSyncCb_l = NULL;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void dualprocshmIrqSyncCb(void *pArg_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize PDO user CAL sync module

The function initializes the PDO user CAL sync module

\param  pfnSyncCb_p             function that is called in case of sync event

\return The function returns a tEplKernel error code.

\ingroup module_pdoucal
*/
//------------------------------------------------------------------------------
tEplKernel pdoucal_initSync(tEplSyncCb pfnSyncCb_p)
{
    tDualprocReturn dualRet;
    tDualprocDrvInstance pDrvInstance = dualprocshm_getInstance(kDualProcHost);

    if(pDrvInstance == NULL)
    {
        EPL_DBGLVL_ERROR_TRACE("%s: Could not find Driver instance!\n", __func__);
        return kEplNoResource;
    }

    dualRet = dualprocshm_irqRegHdl(pDrvInstance, kDualprocIrqSrcSync, dualprocshmIrqSyncCb);
    if(dualRet != kDualprocSuccessful)
    {
        EPL_DBGLVL_ERROR_TRACE("%s: Enable irq not possible!\n", __func__);
        return kEplNoResource;
    }

    pfnSyncCb_l = pfnSyncCb_p;

    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Cleanup PDO user CAL sync module

The function cleans up the PDO user CAL sync module
*/
//------------------------------------------------------------------------------
void pdoucal_exitSync(void)
{
    tDualprocReturn dualRet;
    tDualprocDrvInstance pDrvInstance = dualprocshm_getInstance(kDualProcHost);

    if(pDrvInstance == NULL)
    {
        EPL_DBGLVL_ERROR_TRACE("%s: Could not find Driver instance!\n", __func__);
        return;
    }

    dualRet = dualprocshm_irqRegHdl(pDrvInstance, kDualprocIrqSrcSync, NULL);
    if(dualRet != kDualprocSuccessful)
        EPL_DBGLVL_ERROR_TRACE("%s: Disable irq not possible (%d)!\n", __func__, dualRet);
}

//------------------------------------------------------------------------------
/**
\brief  Wait for a sync event

The function waits for a sync event.

\param  timeout_p       Specifies a timeout in microseconds. If 0 it waits
                        forever.

\return The function returns a tEplKernel error code.
\retval kEplSuccessful      Successfully received sync event
\retval kEplGeneralError    Error while waiting on sync event
*/
//------------------------------------------------------------------------------
tEplKernel pdoucal_waitSyncEvent(ULONG timeout_p)
{
    UNUSED_PARAMETER(timeout_p);
    return kEplSuccessful;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Synchronization callback called by Dual Processor library

\param  pArg_p                  Argument pointer provides dual processor driver instance
                                instance
*/
//------------------------------------------------------------------------------
static void dualprocshmIrqSyncCb(void *pArg_p)
{
    UNUSED_PARAMETER(pArg_p);

    pfnSyncCb_l();
}
