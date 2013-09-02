/**
********************************************************************************
\file   ctrlkcal-noosdual.c

\brief  Kernel control CAL module using a dual processor shared memory library

This file contains an implementation of the kernel control CAL module which uses
a dual processor shared memory for communication with the user layer.

\ingroup module_ctrlkcal
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

#include <ctrl.h>
#include <ctrlkcal.h>
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
#define CTRL_MAGIC                      0xA5A5
//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
typedef struct
{
    tDualprocDrvInstance dualProcDrvInst;
    UINT8*               initParamBase;
    UINT16               initParamBuffSize;
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
\brief  Initialize kernel control CAL module

The function initializes the kernel control CAL module. It initializes the
control memory block and the underlaying CAL module used for implementing
the memory block access functions.

\return The function returns a tEplKernel error code.

\ingroup module_ctrlkcal
*/
//------------------------------------------------------------------------------
tEplKernel ctrlkcal_init (void)
{
    tEplKernel      ret = kEplSuccessful;
    tDualprocReturn dualRet;
    tDualprocConfig dualProcConfig;

    EPL_MEMSET(&instance_l,0,sizeof(tCtrlkCalInstance));

    EPL_MEMSET(&dualProcConfig,0,sizeof(tDualprocConfig));

    dualProcConfig.ProcInstance = kDualProcPcp;

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

    dualRet = dualprocshm_setMagic(instance_l.dualProcDrvInst,CTRL_MAGIC);
    if(dualRet != kDualprocSuccessful)
    {
        EPL_DBGLVL_ERROR_TRACE(" {%s} Could not create write magic (0x%X)\n",\
                                            __func__,dualRet );
        ret = kEplNoResource;
        goto Exit;
    }

    ctrlkcal_setStatus(kCtrlStatusReady);


Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Cleanup kernel control CAL module

The function cleans up the kernel control CAL module. It resets the control
memory block and cleans up the underlaying CAL module used for implementing
the memory block access functions.

\ingroup module_ctrlkcal
*/
//------------------------------------------------------------------------------
void ctrlkcal_exit (void)
{
    tDualprocReturn dualRet;

    dualRet = dualprocshm_delete(instance_l.dualProcDrvInst);
    if(dualRet != kDualprocSuccessful)
    {
        EPL_DBGLVL_ERROR_TRACE("Could not delete dual processor driver (0x%X)\n", dualRet);
    }

    instance_l.initParamBuffSize = 0;
    instance_l.initParamBase = NULL;
}

//------------------------------------------------------------------------------
/**
\brief  Process kernel control CAL module

This function provides processing time for the CAL module.

\return The function returns a tEplKernel error code.

\ingroup module_ctrlkcal
*/
//------------------------------------------------------------------------------
tEplKernel ctrlkcal_process (void)
{
    UINT16 status;
   // if(dualprocshm_process(instance_l.dualProcDrvInst) \
  //         != kDualprocSuccessful)
  //  {
  //      EPL_DBGLVL_ERROR_TRACE ("Could not initialize Dual processor driver \n");
  //     return kEplInvalidOperation;
  //  }
    dualprocshm_getStatus(instance_l.dualProcDrvInst,&status);
    if(status == kCtrlStatusRunning )
    {
        eventkcal_process();
    }
    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Get control command

The function reads a control command stored by the user in the control memory
block to execute a kernel control function.

\param  pCmd_p            The command to be executed.

\return The function returns a tEplKernel error code.

\ingroup module_ctrlkcal
*/
//------------------------------------------------------------------------------
tEplKernel ctrlkcal_getCmd (tCtrlCmdType *pCmd_p)
{
    UINT16          cmd;

    if(dualprocshm_getCommand(instance_l.dualProcDrvInst,&cmd) \
            != kDualprocSuccessful)
        return kEplGeneralError;

    *pCmd_p = (tCtrlCmdType)cmd;

    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Send a return value

The function sends the return value of an executed command to the user stack
by storing it in the control memory block.

\param  retval_p            Return value to send.

\ingroup module_ctrlkcal
*/
//------------------------------------------------------------------------------
void ctrlkcal_sendReturn(UINT16 retval_p)
{
    UINT16          cmd = 0;

    dualprocshm_setRetVal(instance_l.dualProcDrvInst,retval_p);

    dualprocshm_setCommand(instance_l.dualProcDrvInst,cmd);

}

//------------------------------------------------------------------------------
/**
\brief  Set the kernel stack status

The function stores the status of the kernel stack in the control memory block.

\param  status_p                Status to set.

\ingroup module_ctrlkcal
*/
//------------------------------------------------------------------------------
void ctrlkcal_setStatus (UINT16 status_p)
{
    dualprocshm_setStatus(instance_l.dualProcDrvInst,status_p);
}

//------------------------------------------------------------------------------
/**
\brief  Update the heartbeat counter

The function updates it's heartbeat counter in the control memory block which
can be used by the user stack to detect if the kernel stack is still running.

\param  heartbeat_p         Heartbeat counter to store in the control memory
                            block.
\ingroup module_ctrlkcal
*/
//------------------------------------------------------------------------------
void ctrlkcal_updateHeartbeat (UINT16 heartbeat_p)
{
    dualprocshm_setHeartbeat(instance_l.dualProcDrvInst,heartbeat_p);
}

//------------------------------------------------------------------------------
/**
\brief  Store the init parameters for user stack

The function stores the openPOWERLINK initialization parameter so that they
can be accessed by the user stack. It is used to notify the user stack about
parameters modified in the kernel stack.

\param  pInitParam_p        Specifies where to read the init parameters.

\ingroup module_ctrlkcal

*/
//------------------------------------------------------------------------------
void ctrlkcal_storeInitParam(tCtrlInitParam* pInitParam_p)
{
    if(instance_l.initParamBase != NULL)
    {
        EPL_MEMCPY(instance_l.initParamBase,pInitParam_p,sizeof(tCtrlInitParam));
        TARGET_FLUSH_DCACHE(instance_l.initParamBase,sizeof(tCtrlInitParam));
    }
}

//------------------------------------------------------------------------------
/**
\brief  Read the init parameters from user stack

The function reads the initialization parameter from the user stack.

\param  pInitParam_p        Specifies where to store the read init parameters.

\return The function returns a tEplKernel error code.

\ingroup module_ctrlkcal
*/
//------------------------------------------------------------------------------
tEplKernel ctrlkcal_readInitParam(tCtrlInitParam* pInitParam_p)
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

