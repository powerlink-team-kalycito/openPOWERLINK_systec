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

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
typedef struct
{
    tDualprocDrvInstance dualProcDrvInst;
    UINT8*               initParamBase;
    UINT16               initParamBuffSize;
    BOOL                 fIrqMasterEnable;
}tCtrlkCalInstance;
//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tCtrlkCalInstance   instance_l;
//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

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

    EPL_MEMSET(&instance_l,0,sizeof(tCtrlkCalInstance));

    EPL_MEMSET(&dualProcConfig,0,sizeof(tDualprocConfig));

    dualProcConfig.ProcInstance = kDualProcHost;

    dualRet = dualprocshm_create(&dualProcConfig,&instance_l.dualProcDrvInst);
    if(dualRet != kDualprocSuccessful)
    {
        EPL_DBGLVL_ERROR_TRACE(" {%s} Could not create dual processor driver instance (0x%X)\n",\
                                        __func__,dualRet );
        dualprocshm_delete(instance_l.dualProcDrvInst);
        ret = kEplNoResource;
        goto Exit;
    }

    dualRet = dualprocshm_getDynRes(instance_l.dualProcDrvInst,kDualprocResIdDynBuff0,&instance_l.initParamBase, \
                            &instance_l.initParamBuffSize);
    if(dualRet != kDualprocSuccessful)
    {
        ret = kEplNoResource;
        goto Exit;
    }

    instance_l.fIrqMasterEnable =  FALSE;

    // Disable the Interrupts from PCP
    dualRet = dualprocshm_irqMasterEnable(instance_l.dualProcDrvInst, instance_l.fIrqMasterEnable);
    if(dualRet != kDualprocSuccessful)
    {
        EPL_DBGLVL_ERROR_TRACE ("Could not disable Master Irq (0x%X)\n", dualRet);
        ret = kEplNoResource;
        goto Exit;
    }

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

    // Disable the Interrupts from PCP
    dualRet = dualprocshm_irqMasterEnable(instance_l.dualProcDrvInst, \
                            instance_l.fIrqMasterEnable);
    if(dualRet != kDualprocSuccessful)
    {
        EPL_DBGLVL_ERROR_TRACE ("Could not disable Master Irq (0x%X)\n", dualRet);
    }

    dualRet = dualprocshm_delete(instance_l.dualProcDrvInst);
    if(dualRet != kDualprocSuccessful)
    {
        EPL_DBGLVL_ERROR_TRACE ("Could not delete dual proc driver inst (0x%X)\n", dualRet);
    }

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

        // Enable interrupts from PCP

        if(dualprocshm_irqMasterEnable(instance_l.dualProcDrvInst, \
                instance_l.fIrqMasterEnable) != kDualprocSuccessful)
        {
            EPL_DBGLVL_ERROR_TRACE ("Could not delete Enable Master \n");
        }
    }
    //TODO : @gks can this be moved ??
  //  if(dualprocshm_process(instance_l.dualProcDrvInst) != kDualprocSuccessful)
   // {
   //     EPL_DBGLVL_ERROR_TRACE ("Could not initialize Dual processor driver \n");
   //    return kEplInvalidOperation;
  //  }

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
    //printf("Cmd %x\n",cmd);
    if(dualprocshm_setRetVal(instance_l.dualProcDrvInst,retVal) \
            != kDualprocSuccessful )
        return kEplGeneralError;

    if(dualprocshm_setCommand(instance_l.dualProcDrvInst,cmd) \
            != kDualprocSuccessful )
        return kEplGeneralError;

    /* wait for response */
    for (timeout = 0; timeout < CMD_TIMEOUT_CNT; timeout++)
    {
        target_msleep(10);

        if(dualprocshm_getCommand(instance_l.dualProcDrvInst,&cmd) \
                != kDualprocSuccessful )
            return kEplGeneralError;
        //printf("Cmd %x\n",cmd);
        if (cmd == 0)
        {
            if(dualprocshm_getRetVal(instance_l.dualProcDrvInst,&retVal)\
                    != kDualprocSuccessful )
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

    if(dualprocshm_getMagic(instance_l.dualProcDrvInst,&magic) \
            != kDualprocSuccessful)
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

    if ((dualprocshm_getStatus(instance_l.dualProcDrvInst,&status)) \
           == kDualprocSuccessful)
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

    if( dualprocshm_getHeartbeat(instance_l.dualProcDrvInst,&heartbeat)\
            == kDualprocSuccessful)
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
        EPL_MEMCPY(instance_l.initParamBase,pInitParam_p,sizeof(tCtrlInitParam));
        TARGET_FLUSH_DCACHE(instance_l.initParamBase,sizeof(tCtrlInitParam));
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
    if(instance_l.initParamBase == NULL)
        return kEplNoResource;

    TARGET_INVALIDATE_DCACHE(instance_l.initParamBase, sizeof(tCtrlInitParam));

    EPL_MEMCPY(pInitParam_p, instance_l.initParamBase, sizeof(tCtrlInitParam));

    return kEplSuccessful;
}


//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//

