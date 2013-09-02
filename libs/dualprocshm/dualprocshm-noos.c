/**
********************************************************************************
\file   dualprocshm_noos.c

\brief  Dual Processor Library - Using shared memory

Dual processor library provides routines for initialization of memory
and interrupt resources for a shared memory interface between dual
processor.

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
#include "dualprocshm.h"
#include "dualprocshm_l.h"

#include <stdlib.h>
#include <string.h>
//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define DUALPROC_INSTANCE_COUNT     2   ///< number of supported instances
#define DUALPROCSHM_QUEUE_COUNT     12  ///< number of supported queues per driver
                                        ///< instance
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
// local types
//------------------------------------------------------------------------------

/**
\brief Function type to set the address of a Resource(queue/buffers)

This function type enables to set the corresponding dynamic shared memory address
register for a resource. The address of the dynamic shared memory address registers
is given with pDualProcScBase_p and the address itself with addr_p.
*/
typedef void (*tSetDynRes) (UINT32 addr_p);

/**
\brief Function type to get the address of a Resource(queue/buffers)

This function type enables to get the address set in the dynamic shared buffer
address register. The address of the dynamic shared memory address registers is
given with DualProcScBase_p.
*/
typedef UINT32 (*tGetDynRes) ();

/**
\brief Structure for dual processor dynamic resources(queue/buffers)

This structure defines for each dynamic resources instance the set and get
functions. Additionally the base and span is provided.
*/
typedef struct sDualprocDynRes
{
    tSetDynRes  pfnSetDynRes; ///< this function sets the dynamic buffer base to hardware
    tGetDynRes  pfnGetDynRes; ///< this function gets the dynamic buffer base to hardware
    UINT8*      pBase;        ///< base of the dynamic buffer
    UINT16      span;         ///< span of the dynamic buffer
} tDualprocDynResConfig;

/**
\brief Dynamic buffer configuration for Pcp

Stores the default configuration settings for the dynamic buffers
*/
const tDualprocDynResConfig aDynResInit[kDualprocResIdLast] =
{
    {   /* kDualprocResIdDynBuff0 */
        dualprocshm_writeDynResBuff0, dualprocshm_readDynResBuff0,
        NULL, DUALPROCSHM_SIZE_DYNBUF0
    },
    {   /* kDualprocResIdDynBuff1 */
        dualprocshm_writeDynResBuff1, dualprocshm_readDynResBuff1,
        NULL, DUALPROCSHM_SIZE_DYNBUF1
    },
    {   /* kDualprocResIdErrCount */
        dualprocshm_writeDynResErrCnt, dualprocshm_readDynResErrCnt,
        NULL, DUALPROCSHM_SIZE_ERRORCOUNTER
    },
    {   /* kDualprocResIdTxNmtQueue */
        dualprocshm_writeDynResTxNmtQ, dualprocshm_readDynResTxNmtQ,
        NULL, DUALPROCSHM_SIZE_TXNMTQ
    },
    {   /* kDualprocResIdTxGenQueue */
        dualprocshm_writeDynResTxGenQ, dualprocshm_readDynResTxGenQ,
        NULL, DUALPROCSHM_SIZE_TXGENQ
    },
    {   /* kDualprocResIdTxSyncQueue */
        dualprocshm_writeDynResTxSyncQ, dualprocshm_readDynResTxSyncQ,
        NULL, DUALPROCSHM_SIZE_TXSYNCQ
    },
    {   /* kDualprocResIdTxVethQueue */
        dualprocshm_writeDynResTxVethQ, dualprocshm_readDynResTxVethQ,
        NULL, DUALPROCSHM_SIZE_TXVETHQ
    },
    {   /* kDualprocResIdRxVethQueue */
        dualprocshm_writeDynResRxVethQ, dualprocshm_readDynResRxVethQ,
        NULL, DUALPROCSHM_SIZE_RXVETHQ
    },
    {   /* kDualprocResIdK2UQueue */
        dualprocshm_writeDynResK2UQ, dualprocshm_readDynResK2UQ,
        NULL, DUALPROCSHM_SIZE_K2UQ
    },
    {   /* kDualprocResIdU2KQueue */
        dualprocshm_writeDynResU2KQ, dualprocshm_readDynResU2KQ,
        NULL, DUALPROCSHM_SIZE_U2KQ
    },
    {   /* kDualprocResIdPdo */
        dualprocshm_writeDynResPdo, dualprocshm_readDynResPdo,
        NULL, DUALPROCSHM_SIZE_PDO
    }
};

/**
\brief Dual Processor Instance

Holds the configuration passed to the instance at creation.
*/
typedef struct sDualProcDrv
{
    tDualprocConfig         config;       ///< copy of configuration
    UINT8                   *pBase;       ///< base address of host interface

    int                     iDynResEntries; ///< number of dynamic buffers (Pcp/Host)
    tDualprocDynResConfig*  pDynResTbl;  ///< dynamic buffer table (Pcp/Host)
//    UINT8*                  apDynBufHost[DUALPROC_DYNBUF_COUNT]; ///< DynBuf acquired by Host
//TODO: gks: decide on using process queue for circular buffer based queues

 //   tQueueProcess       aQueueProcessTable[DUALPROCSHM_QUEUE_COUNT]; ///< queue process table, processed by dualprocshm_process()
//    int                 iQueueProcessEntries; ///< number of entries in aQueueProcessTable

    tDualprocIrqCb        apfnIrqCb[kDualprocIrqSrcLast];
    UINT16                activeIrq;        ///< list of active interrupts

} tDualProcDrv;
//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
/**
\brief Instance array

This array holds all Dual Processor Driver instances available.
*/
static tDualProcDrv *paDualProcDrvInstance[DUALPROC_INSTANCE_COUNT] =
{
    NULL
};
//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void dualprocshmIrqHandler (void *pArg_p);
static tDualprocReturn allocateDynResources (tDualProcDrv *pDrvInst_p);
static tDualprocReturn freeDynResources (tDualProcDrv *pDrvInst_p);
static tDualprocReturn setDynResources (tDualProcDrv *pDrvInst_p);
static tDualprocReturn getDynResources (tDualProcDrv *pDrvInst_p);
static void getResAddr (tDualProcDrv *pDrvInst_p, tDualprocResourceId resId_p,UINT8** ppAddr_p);
static UINT16 getResSpan (tDualProcDrv *pDrvInst_p, tDualprocResourceId resId_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Create a dual processor driver instance

This function creates a dual processor driver instance, and
initializes it depending on the pConfig_p parameters.

\param  pConfig_p               The caller provides the configuration
                                parameters with this pointer.
\param  ppInstance_p            The function returns with this double-pointer
                                the created instance pointer. (return)

\return tDualprocReturn
\retval kDualprocSuccessful         The dual processor driver is configured successfully
                                    with the provided parameters.
\retval kDualprocInvalidParameter   The caller has provided incorrect parameters.
\retval kDualprocNoResource         Heap allocation was impossible or to many
                                    instances are present.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
tDualprocReturn dualprocshm_create (tDualprocConfig *pConfig_p, tDualprocDrvInstance *ppInstance_p)
{
    tDualprocReturn  ret = kDualprocSuccessful;
    tDualProcDrv     *pDrvInst = NULL;
    int              iIndex;
    if(pConfig_p->ProcInstance != kDualProcPcp && pConfig_p->ProcInstance != kDualProcHost )
    {
        TRACE("Inst %x\n",pConfig_p->ProcInstance);
        return kDualprocInvalidParameter;
    }

    //create driver instance
    pDrvInst = (tDualProcDrv*) malloc(sizeof(tDualProcDrv));

    if(NULL == pDrvInst)
    {
        ret = kDualprocNoResource;
        goto Exit;
    }

    memset(pDrvInst, 0, sizeof(tDualProcDrv));

    // store the configuration

    pDrvInst->config = *pConfig_p;

    pDrvInst->iDynResEntries = kDualprocResIdLast;
    pDrvInst->pDynResTbl = (tDualprocDynResConfig*)aDynResInit;



    if(pDrvInst->config.ProcInstance == kDualProcPcp)
    {
        // store the base address of common memory strucutre PCP
        pDrvInst->pBase = (UINT8*) DUALPROCSHM_CMS_BASE_PCP;

        memset(pDrvInst->pBase,0,sizeof(tCommonMemStruc));
        // initialize lower layer driver
        dualprocshm_initMem(pDrvInst->pBase);
    }
    else
    {
        // store the base address of common memory strucutre HOST
        pDrvInst->pBase = (UINT8*) DUALPROCSHM_CMS_BASE_HOST;

        // initialize lower layer driver
        dualprocshm_initMem(pDrvInst->pBase);
    }

    // allocate dynamic resources if PCP
    if(pDrvInst->config.ProcInstance == kDualProcPcp)
    {
        ret = allocateDynResources(pDrvInst);
        if(kDualprocSuccessful != ret )
        {
            goto Exit;
        }
    }
    else // get them from dynamic shared memory registers
    {
        ret = getDynResources(pDrvInst);

        if(kDualprocSuccessful != ret )
        {
            goto Exit;
        }
    }

    // store driver instance in array
    for(iIndex = 0; iIndex < DUALPROC_INSTANCE_COUNT; iIndex++)
    {
        if(paDualProcDrvInstance[iIndex] == NULL)
        {
           // free entry found
            paDualProcDrvInstance[iIndex] = pDrvInst;

           break;
        }
    }

    if(DUALPROC_INSTANCE_COUNT == iIndex)
    {
        ret = kDualprocNoResource;
        goto Exit;
    }

    if( pDrvInst->config.ProcInstance  == kDualProcHost )
    {
        DUALPROCSHM_REG_IRQHDL(dualprocshmIrqHandler, (void*)pDrvInst);
    }

    // Return the driver instance
    *ppInstance_p = pDrvInst;


Exit:
    if(ret != kDualprocSuccessful)
    {
        dualprocshm_delete(pDrvInst);
    }

    return ret;
}
//------------------------------------------------------------------------------
/**
\brief  Delete dual processor driver instance

This function deletes a dual processor driver instance.

\param  pInstance_p             The driver instance that should be
                                deleted

\return tDualprocReturn
\retval kDualprocSuccessful       The driver instance is deleted successfully.
\retval kDualprocInvalidParameter The caller has provided incorrect parameters.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
tDualprocReturn dualprocshm_delete (tDualprocDrvInstance pInstance_p)
{
    tDualprocReturn ret = kDualprocSuccessful;
    tDualProcDrv    *pDrvInst = (tDualProcDrv*) pInstance_p;
    int iIndex;

    if(pInstance_p == NULL)
    {
        ret = kDualprocInvalidParameter;
        goto Exit;
    }

    if( kDualProcHost == pDrvInst->config.ProcInstance )
    {
        // disable Sync Irq
        DUALPROCSHM_DISABLE_IRQ();

        // unregister the ISR
        DUALPROCSHM_REG_IRQHDL(NULL, NULL);
    }

    for(iIndex = 0; iIndex < DUALPROC_INSTANCE_COUNT; iIndex++)
    {
        if(pDrvInst == paDualProcDrvInstance[iIndex])
        {
           // delete the driver instance
           paDualProcDrvInstance[iIndex] = NULL;
           break;
        }
    }

    if( kDualProcPcp == pDrvInst->config.ProcInstance )
    {
        freeDynResources(pDrvInst);
    }

    free(pDrvInst);

Exit:
    return ret;
}
//------------------------------------------------------------------------------
/**
\brief  Returns the driver instance of the given processor instance

If the instance is not found NULL is returned

\param  Instance_p              Processor instance

\return tDualprocDrvInstance
\retval NULL                    driver instance not found

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
tDualprocDrvInstance dualprocshm_getInstance (tDualProcInstance Instance_p)
{
    tDualProcDrv    *pDrvInst = NULL;
    int             iIndex;

    for(iIndex = 0; iIndex < DUALPROC_INSTANCE_COUNT; iIndex++)
    {
        pDrvInst = (tDualProcDrv *)paDualProcDrvInstance[iIndex];

        if(Instance_p == pDrvInst->config.ProcInstance)
        {
            break;
        }
    }

    return pDrvInst;
}
//------------------------------------------------------------------------------
/**
\brief  This function processes all resources of dual processor driver

This function processes the configured resources of the dual processor driver
like the queues.

\param  pInstance_p             dual processor driver instance

\return tDualprocReturn
\retval kDualprocSuccessful       The process function exit without errors.
\retval kDualprocInvalidParameter The caller has provided incorrect parameters.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
tDualprocReturn dualprocshm_process (tDualprocDrvInstance pInstance_p)
{
    tDualprocReturn ret = kDualprocSuccessful;
    tDualProcDrv    *pDrvInst = (tDualProcDrv*)pInstance_p;

    if(pDrvInst == NULL)
    {
        ret = kDualprocInvalidParameter;
        goto Exit;
    }
    //TODO: Handle Queue processing here

Exit:
    return ret;
}
//------------------------------------------------------------------------------
/**
\brief  This function adds an irq handler for the corresponding irq source

This function adds an irq handler function for the corresponding irq source.
Note: The provided callback is invoked within the interrupt context!
If the provided callback is NULL, then the irq source is disabled.

\param  pInstance_p             dual processor driver instance
\param  irqSrc_p                irq source that should invoke the callback
\param  callback_p              callback that is invoked

\return tDualprocReturn
\retval kDualprocSuccessful       The process function exit without errors.
\retval kDualprocInvalidParameter The caller has provided incorrect parameters.
\retval kDualprocWrongProcInst    Only the host may call this function.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
tDualprocReturn dualprocshm_irqRegHdl (tDualprocDrvInstance pInstance_p,
        tDualprocIrqSrc irqSrc_p, tDualprocIrqCb pfnCb_p)
{
    tDualprocReturn ret = kDualprocSuccessful;
    tDualProcDrv    *pDrvInst = (tDualProcDrv*)pInstance_p;
    UINT16 irqEnableVal = pDrvInst->activeIrq;

    // Only host can register ISR
    if(kDualProcHost != pDrvInst->config.ProcInstance)
    {
        ret = kDualprocWrongProcInst;
        goto Exit;
    }
    if(NULL != pfnCb_p)
    {
        irqEnableVal |= (1 << irqSrc_p);
    }
    else
    {
        irqEnableVal &= ~(1 << irqSrc_p);
    }

    pDrvInst->apfnIrqCb[irqSrc_p] = pfnCb_p;
    pDrvInst->activeIrq = irqEnableVal;

Exit:
    return ret;
}
//------------------------------------------------------------------------------
/**
\brief  This function enables an irq source

This function enables an irq source from the Pcp side.

\param  pInstance_p             dual processor instance
\param  irqSrc_p                irq source to be controlled
\param  fEnable_p               enable the irq source (TRUE)

\return tDualprocReturn
\retval kDualprocSuccessful       The process function exit without errors.
\retval kDualprocInvalidParameter The caller has provided incorrect parameters.
\retval kDualprocWrongProcInst    Only the host may call this function.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
tDualprocReturn dualprocshm_irqSourceEnable (tDualprocDrvInstance pInstance_p,
        tDualprocIrqSrc irqSrc_p, BOOL fEnable_p)
{
    tDualprocReturn ret = kDualprocSuccessful;
    tDualProcDrv    *pDrvInst = (tDualProcDrv*)pInstance_p;
    UINT16 irqEnableVal = pDrvInst->activeIrq;

    if(pInstance_p == NULL)
    {
        ret = kDualprocInvalidParameter;
        goto Exit;
    }

    if(fEnable_p)
    {
        // Add irqSrc_p to active interrupt list
        irqEnableVal |= (1 << irqSrc_p);
    }
    else
    {
        // remove irqSrc_p from active list
        irqEnableVal &= ~(1 << irqSrc_p);;
    }
    // add the new Irq to the list for handling
    pDrvInst->activeIrq = irqEnableVal;

Exit:
    return ret;
}
//------------------------------------------------------------------------------
/**
\brief  This function controls the master irq enable

This function allows the host to enable or disable all irq sources from the
PCP.

\param  pInstance_p             host interface instance
\param  fEnable_p               enable the master irq (TRUE)

\return tDualprocReturn
\retval kDualprocSuccessful       The process function exit without errors.
\retval kDualprocInvalidParameter The caller has provided incorrect parameters.
\retval kDualprocWrongProcInst    Only the host may call this function.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
tDualprocReturn dualprocshm_irqMasterEnable (tDualprocDrvInstance pInstance_p,
        BOOL fEnable_p)
{
    tDualprocReturn ret = kDualprocSuccessful;
    tDualProcDrv    *pDrvInst = (tDualProcDrv*)pInstance_p;

    if(pInstance_p == NULL)
    {
        ret = kDualprocInvalidParameter;
        goto Exit;
    }

    if(kDualProcHost == pDrvInst->config.ProcInstance)
    {
        if(fEnable_p)
        {
            // Enable sync interrupts
            DUALPROCSHM_EN_IRQ();
        }
        else
        {
            DUALPROCSHM_DISABLE_IRQ();
        }
    }
    else
    {
        ret = kDualprocWrongProcInst;
        goto Exit;
    }

Exit:
    return ret;
}
//------------------------------------------------------------------------------
/**
\brief  This function sets magic in control register

\param  pInstance_p             dual processor driver instance
\param  magic_p                 magic pattern

\return tDualprocReturn
\retval kDualprocSuccessful       The process function exit without errors.
\retval kDualprocInvalidParameter The caller has provided incorrect parameters.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
tDualprocReturn dualprocshm_setMagic (tDualprocDrvInstance pInstance_p, UINT16 magic_p)
{
    tDualprocReturn ret = kDualprocSuccessful;
    tDualProcDrv    *pDrvInst = (tDualProcDrv*)pInstance_p;

    if(pDrvInst == NULL)
    {
        ret = kDualprocInvalidParameter;
        goto Exit;
    }

    dualprocshm_writeMagic(magic_p);

Exit:
    return ret;
}
//------------------------------------------------------------------------------
/**
\brief  This function reads magic from control register

\param  pInstance_p             dual processor driver instance
\param  pMagic_p                return magic pattern

\return tDualprocReturn
\retval kDualprocSuccessful       The process function exit without errors.
\retval kDualprocInvalidParameter The caller has provided incorrect parameters.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
tDualprocReturn dualprocshm_getMagic (tDualprocDrvInstance pInstance_p, UINT16 *pMagic_p)
{
    tDualprocReturn ret = kDualprocSuccessful;
    tDualProcDrv    *pDrvInst = (tDualProcDrv*)pInstance_p;

    if(pDrvInst == NULL)
    {
        ret = kDualprocInvalidParameter;
        goto Exit;
    }

    *pMagic_p = dualprocshm_readMagic();

Exit:
    return ret;
}
//------------------------------------------------------------------------------
/**
\brief  This function sets a command in control register

\param  pInstance_p             dual processor driver instance
\param  cmd_p                   command

\return tDualprocReturn
\retval kDualprocSuccessful       The process function exit without errors.
\retval kDualprocInvalidParameter The caller has provided incorrect parameters.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
tDualprocReturn dualprocshm_setCommand (tDualprocDrvInstance pInstance_p, UINT16 cmd_p)
{
    tDualprocReturn ret = kDualprocSuccessful;
    tDualProcDrv    *pDrvInst = (tDualProcDrv*)pInstance_p;

    if(pDrvInst == NULL)
    {
        ret = kDualprocInvalidParameter;
        goto Exit;
    }

    dualprocshm_writeCommand(cmd_p);
Exit:
    return ret;
}
//------------------------------------------------------------------------------
/**
\brief  This function reads a command from control register

\param  pInstance_p             dual processor driver instance
\param  cmd_p                   command

\return tDualprocReturn
\retval kDualprocSuccessful       The process function exit without errors.
\retval kDualprocInvalidParameter The caller has provided incorrect parameters.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
tDualprocReturn dualprocshm_getCommand (tDualprocDrvInstance pInstance_p, UINT16 *pCmd_p)
{
    tDualprocReturn ret = kDualprocSuccessful;
    tDualProcDrv    *pDrvInst = (tDualProcDrv*)pInstance_p;

    if(( NULL == pDrvInst )|| ( NULL == pCmd_p))
    {
        ret = kDualprocInvalidParameter;
        goto Exit;
    }

    *pCmd_p = dualprocshm_readCommand();
Exit:
    return ret;
}
//------------------------------------------------------------------------------
/**
\brief  This function writes status info in control register

\param  pInstance_p             dual processor driver instance
\param  status_p                status

\return tDualprocReturn
\retval kDualprocSuccessful       The process function exit without errors.
\retval kDualprocInvalidParameter The caller has provided incorrect parameters.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
tDualprocReturn dualprocshm_setStatus (tDualprocDrvInstance pInstance_p, UINT16 status_p)
{
    tDualprocReturn ret = kDualprocSuccessful;
    tDualProcDrv    *pDrvInst = (tDualProcDrv*)pInstance_p;

    if(pInstance_p == NULL)
    {
        ret = kDualprocInvalidParameter;
        goto Exit;
    }

    if(pDrvInst->config.ProcInstance != kDualProcPcp)
    {
        // host can't set Status field
        ret = kDualprocWrongProcInst;
        goto Exit;
    }

    dualprocshm_writeStatus(status_p);

Exit:
    return ret;
}
//------------------------------------------------------------------------------
/**
\brief  This function reads status info from control register

\param  pInstance_p             dual processor driver instance
\param  pStatus_p               Read status value

\return tDualprocReturn
\retval kDualprocSuccessful       The process function exit without errors.
\retval kDualprocInvalidParameter The caller has provided incorrect parameters.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
tDualprocReturn dualprocshm_getStatus (tDualprocDrvInstance pInstance_p, UINT16 *pStatus_p)
{
    tDualprocReturn ret = kDualprocSuccessful;
    tDualProcDrv    *pDrvInst = (tDualProcDrv*)pInstance_p;

    if(( NULL == pDrvInst )|| ( NULL == pStatus_p))
    {
        ret = kDualprocInvalidParameter;
        goto Exit;
    }

    *pStatus_p = dualprocshm_readStatus();

Exit:
    return ret;
}
//------------------------------------------------------------------------------
/**
\brief  This function writes the return value in control register

\param  pInstance_p             dual processor driver instance
\param  retval_p                return value

\return tDualprocReturn
\retval kDualprocSuccessful       The process function exit without errors.
\retval kDualprocInvalidParameter The caller has provided incorrect parameters.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
tDualprocReturn dualprocshm_setRetVal (tDualprocDrvInstance pInstance_p, UINT16 retval_p)
{
    tDualprocReturn ret = kDualprocSuccessful;
    tDualProcDrv    *pDrvInst = (tDualProcDrv*)pInstance_p;

    if(pDrvInst == NULL)
    {
        ret = kDualprocInvalidParameter;
        goto Exit;
    }

    dualprocshm_writeRetVal(retval_p);

Exit:
    return ret;
}
//------------------------------------------------------------------------------
/**
\brief  This function reads the return value from control register

\param  pInstance_p             dual processor driver instance
\param  pRetval_p               read return value

\return tDualprocReturn
\retval kDualprocSuccessful       The process function exit without errors.
\retval kDualprocInvalidParameter The caller has provided incorrect parameters.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
tDualprocReturn dualprocshm_getRetVal (tDualprocDrvInstance pInstance_p, UINT16 *pRetval_p)
{
    tDualprocReturn ret = kDualprocSuccessful;
    tDualProcDrv    *pDrvInst = (tDualProcDrv*)pInstance_p;

    if(( NULL == pDrvInst )|| ( NULL == pRetval_p))
    {
        ret = kDualprocInvalidParameter;
        goto Exit;
    }

    *pRetval_p = dualprocshm_readRetVal();

Exit:
    return ret;
}
//------------------------------------------------------------------------------
/**
\brief  This function sets the heart beat value in control register

Note that only the Pcp is allowed to write to this register!

\param  pInstance_p             dual processor driver instance
\param  heartbeat_p             heart beat value

\return tDualprocReturn
\retval kDualprocSuccessful       The process function exit without errors.
\retval kDualprocInvalidParameter The caller has provided incorrect parameters.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
tDualprocReturn dualprocshm_setHeartbeat (tDualprocDrvInstance pInstance_p, UINT16 heartbeat_p)
{
    tDualprocReturn ret = kDualprocSuccessful;
    tDualProcDrv    *pDrvInst = (tDualProcDrv*)pInstance_p;

    if(pInstance_p == NULL)
    {
        ret = kDualprocInvalidParameter;
        goto Exit;
    }

    if(pDrvInst->config.ProcInstance != kDualProcPcp)
    {
        // host can't set heart beat
        ret = kDualprocWrongProcInst;
        goto Exit;
    }

    dualprocshm_writeHeartbeat(heartbeat_p);
   // TRACE("Write Heart %x",heartbeat_p);
Exit:
    return ret;
}
//------------------------------------------------------------------------------
/**
\brief  This function reads the heart beat value from control register

\param  pInstance_p             dual processor driver instance
\param  pHeartbeat_p            heart beat value

\return tDualprocReturn
\retval kDualprocSuccessful       The process function exit without errors.
\retval kDualprocInvalidParameter The caller has provided incorrect parameters.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
tDualprocReturn dualprocshm_getHeartbeat (tDualprocDrvInstance pInstance_p, UINT16 *pHeartbeat_p)
{
    tDualprocReturn ret = kDualprocSuccessful;
    tDualProcDrv    *pDrvInst = (tDualProcDrv*)pInstance_p;

    if(( NULL == pDrvInst )|| ( NULL == pHeartbeat_p))
    {
        ret = kDualprocInvalidParameter;
        goto Exit;
    }

    *pHeartbeat_p = dualprocshm_readHeartbeat();

Exit:
    return ret;
}
//------------------------------------------------------------------------------
/**
\brief  This function retrieves the address and size of a dynamic resource

\param  pInstance_p             dual processor driver instance
\param  resId_p                 resource Id
\param  ppAddr_p                return address of the resource
\param  pSize_p                 return size of the resource

\return tDualprocReturn
\retval kDualprocSuccessful       The process function exit without errors.
\retval kDualprocInvalidParameter The caller has provided incorrect parameters.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
tDualprocReturn dualprocshm_getDynRes(tDualprocDrvInstance pInstance_p,tDualprocResourceId resId_p,
                                        UINT8 **ppAddr_p, UINT16 *pSize_p)
{
    tDualprocReturn     ret = kDualprocSuccessful ;
    tDualProcDrv *pDrvInst = (tDualProcDrv *)pInstance_p;

    if(pInstance_p == NULL || pSize_p == NULL || ppAddr_p == NULL )
    {
        ret = kDualprocInvalidParameter;
        goto Exit;
    }

    // get the resource address
    // getResAddr(pDrvInst,resId_p,ppAddr_p);
    *ppAddr_p = pDrvInst->pDynResTbl[resId_p].pBase;
    // TODO : @gks add a check for size
    // *pSize_p = getResSpan(pDrvInst,resId_p,pSize_p);
    *pSize_p = pDrvInst->pDynResTbl[resId_p].span;

Exit:
    return ret;
}
//------------------------------------------------------------------------------
/**
\brief  This function acquires a lock for the specified queue

\param  pInstance_p             dual processor driver instance
\param  queueId_p               queue_Id

\return tDualprocReturn
\retval kDualprocSuccessful       The process function exit without errors.
\retval kDualprocInvalidParameter The caller has provided incorrect parameters.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
tDualprocReturn dualprocshm_acquireQueueLock(UINT8 queueId_p)
{
    tDualprocReturn     ret = kDualprocSuccessful ;
    UINT16      lockState, acquireLock = 0;

    if(queueId_p > kDualprocResIdLast)
    {
        ret = kDualprocInvalidParameter;
        goto Exit;
    }

    acquireLock = (1 << queueId_p );
    lockState = dualprocshm_readQueueLock();
    while((lockState & acquireLock))
    {
        lockState = dualprocshm_readQueueLock();
    }

    lockState |= acquireLock;

    dualprocshm_writeQueueLock(lockState);
Exit:
    return ret;
}
//------------------------------------------------------------------------------
/**
\brief  This function release a lock for the specified queue

\param  pInstance_p             dual processor driver instance
\param  queueId_p               queue_Id

\return tDualprocReturn
\retval kDualprocSuccessful       The process function exit without errors.
\retval kDualprocInvalidParameter The caller has provided incorrect parameters.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
tDualprocReturn dualprocshm_releaseQueueLock(UINT8 queueId_p)
{
    tDualprocReturn     ret = kDualprocSuccessful ;
    UINT16      lockState;
    if(queueId_p > kDualprocResIdLast)
    {
        ret = kDualprocInvalidParameter;
        goto Exit;
    }

    lockState = dualprocshm_readQueueLock();
    lockState &= ~(1 << queueId_p );
    dualprocshm_writeQueueLock(lockState);
Exit:
    return ret;
}
//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Sync Interrupt Handler

This is the sync interrupt handler which should by called by the system if the
irq signal is asserted by the HOST/PCP. This handler acknowledges the processed
interrupt sources and calls the corresponding callbacks registered with
dualprocshm_irqRegHdl().

\param  pArg_p                  The system caller should provide the host
                                interface instance with this parameter.
*/
//------------------------------------------------------------------------------
static void dualprocshmIrqHandler (void *pArg_p)
{
    tDualProcDrv    *pDrvInst = (tDualProcDrv*)pArg_p;
    UINT16          pendingIrq;
    UINT16          activeIrq = pDrvInst->activeIrq;
    UINT16          mask;
    int             index;

    if(pArg_p == NULL )
    {
        goto Exit;
    }
    pendingIrq = dualprocshm_readPendingIrq();

    for(index = 0 ; index < kDualprocIrqSrcLast ; index++)
    {
        mask = (1 << index);
        if((pendingIrq & mask) && (activeIrq & mask))
        {
            // acknowledge interrupt
            dualprocshm_ackIrq(mask);

            // call the registered callback
            if(pDrvInst->apfnIrqCb[index] != NULL)
                pDrvInst->apfnIrqCb[index](pArg_p);
        }
    }

Exit:
    return;
}
//------------------------------------------------------------------------------
/**
\brief  Allocate the dynamic Resources in heap memory

This function allocates the memory necessary for the dynamic resources in the heap
on Pcp side. Note that this function does not allocate memory for buffers that
can be set from host side, instead buffers like K2U-Queue or Error Counters.

\param  pDrvInst_p               Dual processor Driver instance

\return The function returns tDualprocReturn error code.
*/
//------------------------------------------------------------------------------
static tDualprocReturn allocateDynResources (tDualProcDrv *pDrvInst_p)
{
    tDualprocReturn ret = kDualprocSuccessful;
    int             index;
    UINT8           *pBase;
    UINT16          span;

    for(index = 0; index < pDrvInst_p->iDynResEntries ; index++)
    {
        pBase = NULL;
        span = pDrvInst_p->pDynResTbl[index].span;

        if(span > 0)
          pBase = (UINT8*)DUALPROCSHM_MALLOC(span);

        if(pBase == NULL)
        {
            ret = kDualprocNoResource;
            goto Exit;
        }

        memset(pBase,0,span);

        pDrvInst_p->pDynResTbl[index].pBase = pBase;
    }

    ret = setDynResources(pDrvInst_p);

Exit:

    if(ret != kDualprocSuccessful)
    {
       // index points to failed one
       while(index != 0)
       {
           // TODO: @gks check this
           free(pDrvInst_p->pDynResTbl[--index].pBase);

           pDrvInst_p->pDynResTbl[index].pBase = NULL;
       }
    }

    return ret;

}
//------------------------------------------------------------------------------
/**
\brief  Free the dynamic resources in heap memory

This function frees the memory necessary for the dynamic buffers in the heap
on Pcp side. Note that this function does not free memory for buffers that
can be set from host side, instead buffers like K2U-Queue or Error Counters.

\param  pDrvInst_p               Dual processor driver instance

\return The function returns tDualprocReturn error code.
*/
//------------------------------------------------------------------------------
static tDualprocReturn freeDynResources (tDualProcDrv *pDrvInst_p)
{
    tDualprocReturn ret = kDualprocSuccessful;
    int             index;

    for(index = 0; index < pDrvInst_p->iDynResEntries ; index++)
    {
        DUALPROCSHM_FREE(pDrvInst_p->pDynResTbl[index].pBase);
        pDrvInst_p->pDynResTbl[index].pBase = NULL;
    }

    ret = setDynResources(pDrvInst_p);

    return ret;
}
//------------------------------------------------------------------------------
/**
\brief  Set the address of the dynamic resources

This function sets the base addresses of the dynamic buffers in Pcp environment
into the dynamic shared memory registers of common memory structure.

\param  pDrvInst_p               driver instance

\return The function returns tDualprocReturn error code.
*/
//------------------------------------------------------------------------------
static tDualprocReturn setDynResources (tDualProcDrv *pDrvInst_p)
{
    tDualprocReturn ret = kDualprocSuccessful;
    int             index;
    UINT32          addr;

    for(index = 0; index < pDrvInst_p->iDynResEntries ; index++)
    {
        addr =  (UINT32) pDrvInst_p->pDynResTbl[index].pBase;

        pDrvInst_p->pDynResTbl[index].pfnSetDynRes(addr);
    }

    return ret;
}
//------------------------------------------------------------------------------
/**
\brief  Get the address of the dynamic resources

This function reads the base addresses of the dynamic buffers in PCP environment
from the dynamic shared memory registers to be used by HOST.

\param  pDrvInst_p               driver instance

\return The function returns tDualprocReturn error code.
*/
//------------------------------------------------------------------------------
static tDualprocReturn getDynResources (tDualProcDrv *pDrvInst_p)
{
    tDualprocReturn ret = kDualprocSuccessful;
    int             index;
    UINT8*          pAddr;

    for(index = 0; index < pDrvInst_p->iDynResEntries ; index++)
    {
        pAddr =  (UINT8 *) pDrvInst_p->pDynResTbl[index].pfnGetDynRes();

        pDrvInst_p->pDynResTbl[index].pBase = pAddr;
    }

    return ret;
}
//------------------------------------------------------------------------------
/**
\brief  Get the address of the Resource(queue/buffer)

This function is used to retrieve the base addresses of a resource from
dynamic shared memory registers.

\param  resId_p               driver instance

\return The function returns base address of the resource.
*/
//------------------------------------------------------------------------------
static void getResAddr (tDualProcDrv *pDrvInst_p, tDualprocResourceId resId_p,UINT8** ppAddr_p)
{
    *ppAddr_p = pDrvInst_p->pDynResTbl[resId_p].pBase;
}
//------------------------------------------------------------------------------
/**
\brief  Get the length of the Resource(queue/buffer)

This function is used to retrieve the length of a resource allocated in
dynamically.

\param  resId_p               driver instance

\return The function returns base address of the resource.
*/
//------------------------------------------------------------------------------
static UINT16 getResSpan (tDualProcDrv *pDrvInst_p, tDualprocResourceId resId_p)
{
    UINT16 span;

    span = pDrvInst_p->pDynResTbl[resId_p].span;

    return span;
}

//------------------------------------------------------------------------------
/**
\brief  Add queue instance to processing list

This function adds a queue instance to the processing table of the host
interface instance.
If the function dualprocshm_process() is called, the queue process table is
processed.

\param  Instance_p              driver instance
\param  QueueProcess_p          Entry to be added to queue process table

\return The function returns tDualprocReturn error code.
*/
/*
//------------------------------------------------------------------------------
static tDualprocReturn addQueueProcess (tDualprocDrvInstance Instance_p,
        tQueueProcess QueueProcess_p)
{
    // TODO : @gks check this implementation
    tDualprocReturn ret = kDualprocNoResource;
    tDualProcDrv *pDrvInst = (tDualProcDrv *)Instance_p;
    int index;

    for(index = 0; index < DUALPROCSHM_QUEUE_COUNT; index++)
    {
        if(pDrvInst->aQueueProcessTable[index].pInstance == NULL)
        {
            pDrvInst->aQueueProcessTable[index] = QueueProcess_p;
            pDrvInst->iQueueProcessEntries++;
            ret = kDualprocSuccessful;
            goto Exit;
        }
    }

Exit:
    return ret;
}
*/
//------------------------------------------------------------------------------
/**
\brief  Remove queue instance from processing list

This function removes a queue instance from the processing table of the host
interface instance.

\param  Instance_p              driver instance
\param  QueueProcess_p          Entry to be removed from queue process table

\return The function returns tDualprocReturn error code.
*/
/*//------------------------------------------------------------------------------
static tDualprocReturn removeQueueProcess (tDualprocDrvInstance Instance_p,
        tQueueProcess QueueProcess_p)
{
    tDualprocReturn ret = kDualprocNoResource;
    tDualProcDrv *pDrvInst = (tDualProcDrv *)Instance_p;
    int index;

    for(index = 0; index < DUALPROCSHM_QUEUE_COUNT; index++)
    {
        if(pDrvInst->aQueueProcessTable[i].pInstance ==
                QueueProcess_p.pInstance)
        {
            memset(&pDrvInst->aQueueProcessTable[i], 0, sizeof(tQueueProcess));
            pDrvInst->iQueueProcessEntries--;
            ret = kDualprocSuccessful;
            goto Exit;
        }
    }

Exit:
    return ret;
}
*/
