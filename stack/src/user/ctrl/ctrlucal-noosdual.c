/**
********************************************************************************
\file   ctrlucal-noosdual.c

\brief  User control CAL module using a dual processor shared memory library

This file contains an implementation of the user control CAL module which uses
a shared memory block for communication with the kernel layer.

\ingroup module_ctrlucal
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
#include <unistd.h>
#include <stddef.h>

#include <Epl.h>
#include <EplTarget.h>
#include <ctrl.h>
#include <ctrlcal.h>
#include <dualprocshm.h>
#include <user/ctrlucal.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define CMD_TIMEOUT_CNT     500     // loop counter for command timeout
#define CTRL_MAGIC          0xA5A5
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
#define CTRL_PROC_ID            0xFA
#define DUALPROCSHM_DYNBUFF_ID  0x09
#define TARGET_MAX_INTERRUPTS   4
//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

/**
\brief Control buffer - Status/Control

The control sub-registers provide basic Pcp-to-Host communication features.
*/
typedef struct sCtrlBuff
{
    volatile UINT16     magic;      ///< enable the bridge logic
    volatile UINT16     status;     ///< reserved
    volatile UINT16     heartbeat;  ///< heart beat word
    volatile UINT16     command;    ///< command word
    volatile UINT16     retval;     ///< return word
    UINT16              resv;        ///< reserved
    UINT16              irqEnable;  ///< enable irqs
    union
    {
       volatile UINT16 irqSet;      ///< set irq (Pcp)
       volatile UINT16 irqAck;      ///< acknowledge irq (Host)
       volatile UINT16 irqPending;  ///< pending irq
    };
} tCtrlBuff;

/**
\brief Function type definition for target interrupt callback

This function callback is called for a given interrupt source, registered by
a specific user layer module.
*/
typedef void (*tTargetIrqCb) (void);

typedef struct
{
    tDualprocDrvInstance dualProcDrvInst;
    UINT8*               initParamBase;
    size_t               initParamBuffSize;
    BOOL                 fIrqMasterEnable;
    tTargetIrqCb         apfnIrqCb[TARGET_MAX_INTERRUPTS];
}tCtrluCalInstance;


//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tCtrluCalInstance   instance_l;
//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void targetInterruptHandler ( void* pArg_p );

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize user control CAL module

The function initializes the user control CAL module.

\return The function returns a tEplKernel error code.

\ingroup module_ctrlucal
*/
//------------------------------------------------------------------------------
tEplKernel ctrlucal_init(void)
{
    tEplKernel      ret = kEplSuccessful;
    tDualprocReturn dualRet;
    tDualprocConfig dualProcConfig;

    EPL_MEMSET(&instance_l,0,sizeof(tCtrluCalInstance));

    EPL_MEMSET(&dualProcConfig,0,sizeof(tDualprocConfig));

    dualProcConfig.ProcInstance = kDualProcHost;
    dualProcConfig.procId = CTRL_PROC_ID;

    dualRet = dualprocshm_create(&dualProcConfig,&instance_l.dualProcDrvInst);
    if(dualRet != kDualprocSuccessful)
    {
        EPL_DBGLVL_ERROR_TRACE(" {%s} Could not create dual processor driver instance (0x%X)\n",\
                                        __func__,dualRet );
        dualprocshm_delete(instance_l.dualProcDrvInst);
        ret = kEplNoResource;
        goto Exit;
    }

    dualRet = dualprocshm_getMemory(instance_l.dualProcDrvInst,DUALPROCSHM_DYNBUFF_ID,
                                    &instance_l.initParamBase,&instance_l.initParamBuffSize,FALSE);
    if(dualRet != kDualprocSuccessful)
    {
        EPL_DBGLVL_ERROR_TRACE("{%s} Error Retrieving dynamic buff %x\n ",__func__,dualRet);
        ret = kEplNoResource;
        goto Exit;
    }

    // Disable the Interrupts from PCP
    instance_l.fIrqMasterEnable =  FALSE;

    if(target_regSyncIrqHdl(targetInterruptHandler, (void*)&instance_l) != 0)
    {
        EPL_DBGLVL_ERROR_TRACE("Interrupt\n ");
        ret = kEplNoResource;
        goto Exit;
    }

    // enable system irq
    target_enableSyncIrq(TRUE);



Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Cleanup user control CAL module

The function cleans-up the user control CAL module.

\ingroup module_ctrlucal
*/
//------------------------------------------------------------------------------
void ctrlucal_exit (void)
{
    tDualprocReturn dualRet;

    instance_l.fIrqMasterEnable =  FALSE;

    // disable system irq
    target_enableSyncIrq(FALSE);
    target_regSyncIrqHdl(NULL, NULL);


    dualRet = dualprocshm_freeMemory(instance_l.dualProcDrvInst,DUALPROCSHM_DYNBUFF_ID,FALSE);
    if(dualRet != kDualprocSuccessful)
    {
        EPL_DBGLVL_ERROR_TRACE("Unable to free memory (0x%X)\n",dualRet);
    }

    dualRet = dualprocshm_delete(instance_l.dualProcDrvInst);
    if(dualRet != kDualprocSuccessful)
    {
        EPL_DBGLVL_ERROR_TRACE ("Could not delete dual proc driver inst (0x%X)\n", dualRet);
    }

    instance_l.initParamBuffSize = 0;
    instance_l.initParamBase = NULL;
}

//------------------------------------------------------------------------------
/**
\brief  Process user control CAL module

This function provides processing time for the CAL module.

\return The function returns a tEplKernel error code.

\ingroup module_ctrlucal
*/
//------------------------------------------------------------------------------
tEplKernel ctrlucal_process (void)
{
    if (instance_l.fIrqMasterEnable ==  FALSE)
    {
        instance_l.fIrqMasterEnable = TRUE;
    }

    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief    Execute a ctrl command

The function executes a control command in the kernel stack.

\param  cmd_p            Command to execute

\return The function returns a tEplKernel error code.

\ingroup module_ctrlucal
*/
//------------------------------------------------------------------------------
tEplKernel ctrlucal_executeCmd(tCtrlCmdType cmd_p)
{
    UINT16              cmd;
    UINT16              retVal;
    int                 timeout;

    /* write command into shared buffer */
    cmd = cmd_p;
    retVal = 0;
    if(dualprocshm_writeDataCommon(instance_l.dualProcDrvInst,offsetof(tCtrlBuff,retval), \
        sizeof(retVal),(UINT8 *)&retVal) != kDualprocSuccessful )
        return kEplGeneralError;

    if(dualprocshm_writeDataCommon(instance_l.dualProcDrvInst,offsetof(tCtrlBuff,command), \
            sizeof(cmd),(UINT8 *)&cmd) != kDualprocSuccessful )
        return kEplGeneralError;

    /* wait for response */
    for (timeout = 0; timeout < CMD_TIMEOUT_CNT; timeout++)
    {
        target_msleep(10);

        if(dualprocshm_readDataCommon(instance_l.dualProcDrvInst,offsetof(tCtrlBuff,command), \
                sizeof(cmd),(UINT8 *)&cmd) != kDualprocSuccessful )
            return kEplGeneralError;
        if (cmd == 0)
        {
            if(dualprocshm_readDataCommon(instance_l.dualProcDrvInst,offsetof(tCtrlBuff,retval), \
                    sizeof(retVal),(UINT8 *)&retVal) != kDualprocSuccessful )
                return kEplGeneralError;
            else
                return retVal;
        }
    }

    TRACE("%s() Timeout waiting for return!\n", __func__);
    return kEplGeneralError;
}


//------------------------------------------------------------------------------
/**
\brief Check state of kernel stack

The function checks the state of the kernel stack. If it is already running
it tries to shutdown.

\return The function returns a tEplKernel error code.
\retval kEplSuccessful  If kernel stack is initialized
\retval kEplNoResource  If kernel stack is not running or in wrong state

\ingroup module_ctrlucal
*/
//------------------------------------------------------------------------------
tEplKernel ctrlucal_checkKernelStack(void)
{
    UINT16              kernelStatus;
    tEplKernel          ret;
    UINT16              magic;


    TRACE ("Checking for kernel stack...\n");

    if(dualprocshm_readDataCommon(instance_l.dualProcDrvInst,offsetof(tCtrlBuff,magic), \
            sizeof(magic),(UINT8 *)&magic) != kDualprocSuccessful)
        return kEplGeneralError;

    if ( magic != CTRL_MAGIC)
    {
        TRACE ("Kernel daemon not running! Exiting...\n");
        return kEplNoResource;
    }

    kernelStatus = ctrlucal_getStatus();

    switch(kernelStatus)
    {
        case kCtrlStatusReady:
            TRACE("-> Kernel Stack is ready\n");
            ret = kEplSuccessful;
            break;

        case kCtrlStatusRunning:
            /* try to shutdown kernel stack */

            TRACE("-> Try to shutdown Kernel Stack\n");
            ret = ctrlucal_executeCmd(kCtrlCleanupStack);
            if (ret != kEplSuccessful)
            {
                ret = kEplNoResource;
                break;
            }

            target_msleep(1000);

            kernelStatus = ctrlucal_getStatus();
            if (kernelStatus != kCtrlStatusReady)
            {
                ret = kEplNoResource;
            }
            break;

        default:
            ret = kEplNoResource;
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Get status of kernel stack

The function gets the status of the kernel stack

\return The function returns the kernel status.

\ingroup module_ctrlucal
*/
//------------------------------------------------------------------------------
UINT16 ctrlucal_getStatus(void)
{
    UINT16          status;

    if (dualprocshm_readDataCommon(instance_l.dualProcDrvInst,offsetof(tCtrlBuff,status), \
            sizeof(status),(UINT8 *)&status) == kDualprocSuccessful)
        return status;
    else
        return kCtrlStatusUnavailable;
}

//------------------------------------------------------------------------------
/**
\brief Get the heartbeat of the kernel stack

The function reads the heartbeat genereated by the kernel stack.

\return The function returns the heartbeat counter.

\ingroup module_ctrlucal
*/
//------------------------------------------------------------------------------
UINT16 ctrlucal_getHeartbeat(void)
{
    UINT16      heartbeat;

    if(dualprocshm_readDataCommon(instance_l.dualProcDrvInst,offsetof(tCtrlBuff,heartbeat), \
            sizeof(heartbeat),(UINT8 *)&heartbeat) == kDualprocSuccessful)
       return heartbeat;
    else
       return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Store the init parameters for kernel use

The function stores the openPOWERLINK initialization parameter so that they
can be accessed by the kernel stack.

\param  pInitParam_p        Specifies where to read the init parameters.

\ingroup module_ctrlucal
*/
//------------------------------------------------------------------------------
void ctrlucal_storeInitParam(tCtrlInitParam* pInitParam_p)
{
    if(instance_l.initParamBase != NULL)
    {
        dualprocshm_writeData(instance_l.dualProcDrvInst,DUALPROCSHM_DYNBUFF_ID,0, \
                                       sizeof(tCtrlInitParam),(UINT8 *)pInitParam_p);
    }
}

//------------------------------------------------------------------------------
/**
\brief  Read the init parameters from kernel

The function reads the initialization parameter from the kernel stack.

\param  pInitParam_p        Specifies where to store the read init parameters.

\return The function returns a tEplKernel error code. It returns always
        kEplSuccessful!

\ingroup module_ctrlucal
*/
//------------------------------------------------------------------------------
tEplKernel ctrlucal_readInitParam(tCtrlInitParam* pInitParam_p)
{
    tDualprocReturn dualRet;

    if(instance_l.initParamBase == NULL)
        return kEplNoResource;

    dualRet = dualprocshm_readData(instance_l.dualProcDrvInst,DUALPROCSHM_DYNBUFF_ID,0, \
            sizeof(tCtrlInitParam),(UINT8 *)pInitParam_p) ;

    if (dualRet!= kDualprocSuccessful)
    {
        EPL_DBGLVL_ERROR_TRACE("Cannot read initparam (0x%X)\n",dualRet);
        return kEplGeneralError;
    }

    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Register interrupt handler

The function registers a interrupt handler for the specified interrupt.

\param  irqId_p        interrupt id.
\param  pfnIrqHandler_p  interrpt handler

\return The function returns a tEplKernel error code. It returns always
        kEplSuccessful!

\ingroup module_ctrlucal
*/
//------------------------------------------------------------------------------
tEplKernel ctrlucal_registerHandler(UINT8 irqId_p,void* pfnIrqHandler_p)
{
    UINT16 irqEnableVal;

    if(irqId_p > TARGET_MAX_INTERRUPTS)
        return kEplInvalidOperation;

    if(dualprocshm_readDataCommon(instance_l.dualProcDrvInst,offsetof(tCtrlBuff,irqEnable), \
                sizeof(irqEnableVal),(UINT8 *)&irqEnableVal) != kDualprocSuccessful)
    {
        return kEplInvalidOperation;
    }

    if(pfnIrqHandler_p != NULL)
        irqEnableVal |= (1 << irqId_p);
    else
        irqEnableVal &= ~(1 << irqId_p);

    instance_l.apfnIrqCb[irqId_p] = (tTargetIrqCb)pfnIrqHandler_p;

    if(dualprocshm_writeDataCommon(instance_l.dualProcDrvInst,offsetof(tCtrlBuff,irqEnable), \
                sizeof(irqEnableVal),(UINT8 *)&irqEnableVal) != kDualprocSuccessful)
    {
        return kEplInvalidOperation;
    }

    return kEplSuccessful;
}
//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
//------------------------------------------------------------------------------
/**
\brief  Control Interrupt Handler

This is the host interrupt handler which should by called by the system if the
irq signal is asserted by PCP. This will be used to handle multiple interrupts sources
with a single interrupt line available. This handler acknowledges the processed
interrupt sources and calls the corresponding callbacks registered with
ctrlucal_registerHandler().

\param  pArg_p                  The system caller should provide the control module
                                instance with this parameter.
*/
//------------------------------------------------------------------------------
static void targetInterruptHandler ( void* pArg_p )
{
    UINT16 pendings;
    UINT16 mask;
    int i;

    UNUSED_PARAMETER(pArg_p);

    if(dualprocshm_readDataCommon(instance_l.dualProcDrvInst,offsetof(tCtrlBuff,irqPending), \
                sizeof(pendings),(UINT8 *)&pendings) != kDualprocSuccessful)
    {
        return;
    }

    for(i=0; i < TARGET_MAX_INTERRUPTS; i++)
    {
        mask = 1 << i;

        //ack irq source first
        if(pendings & mask)
        {
            pendings &= ~mask;
            dualprocshm_writeDataCommon(instance_l.dualProcDrvInst,offsetof(tCtrlBuff,irqAck), \
                                        sizeof(pendings),(UINT8 *)&pendings);
        }


        //then try to execute the callback
        if(instance_l.apfnIrqCb[i] != NULL)
            instance_l.apfnIrqCb[i]();
    }

}
