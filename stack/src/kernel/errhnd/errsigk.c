/**
********************************************************************************
\file   errsigk.c

\brief  Implementation of kernel error signaling module

This module implements the kernel part of the error signaling module.
It is responsible for storing and sending the errors on CN to MN.

\ingroup module_errhndk
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, Kalycito Infotech Private Limited
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
#include <nmt.h>
#include <Benchmark.h>
#include <obd.h>
#include <kernel/eventk.h>
#include <kernel/dllk.h>

#include <errhnd.h>
#include <kernel/errhndk.h>
#include <Epl.h>
#include "errhndkcal.h"

#include "kernel/errsigk.h"
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
static tEplKernel allocateErrStatusBuffers(void);
static tEplKernel freeErrStatusBuffers(void);
//============================================================================//
//          P R I V A T E   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

#define MAX_STATUS_FRAMES            3
#define MAX_STATUS_ENTRY_PER_BUFFER  5
//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

typedef enum
{
    kNoBuffer = 0,          ///< No ErrorSignaling buffer is initialised
    kBuffersEmpty,          ///< No ErrorSignaling buffer is used
    kBuffersAvailable,      ///< Some buffers excluding the reserved one,
                            ///	 with ErrorSignaling module are occupied
    kBuffersFull,           ///< All buffers excluding the reserved one,
                            ///	 with ErrorSignaling module are occupied
    kReservedBufferFull     ///< All buffers including the reserved one,
                            ///	 with ErrorSignaling module are occupied
}tErrSigkBufferStatus;

/**
\brief  instance of kernel error signaller

The structure defines the instance variables of the kernel error signaller
*/
typedef struct sErrSigkInstance
{
    tErrSigkBufferStatus    m_Status;               ///< current status of the Error Status Buffer
    tErrSigkBuffer*         m_ErrorBufferHead;      ///< The first Error Status Buffer
    tErrSigkBuffer*         m_pCurrentErrorBuffer;  ///< Current Error Status Buffer used for storing status entries
    tErrSigkBuffer*         m_pReservedErrorBuffer; ///< Reserved Error Status Buffer for storing any sudden error status changes
}tErrSigkInstance;

//------------------------------------------------------------------------------
// module local vars
//------------------------------------------------------------------------------
static tErrSigkInstance instance_l;
//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------



//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Initialize kernel error signaller module

The function initializes the kernel error signaller module.

\return Returns a tEplKernel error code.

\ingroup module_errsigk
*/
//------------------------------------------------------------------------------
tEplKernel errsigk_init(void)
{
    tEplKernel      ret;

    ret = kEplSuccessful;
    //TODO: Reset Error queue and error objects: requires posting event to user side.
    instance_l.m_Status = kNoBuffer;
    instance_l.m_ErrorBufferHead = NULL;
    instance_l.m_pCurrentErrorBuffer = NULL;
    instance_l.m_pReservedErrorBuffer = NULL;

    return ret;
}


//------------------------------------------------------------------------------
/**
\brief    Stops kernel error signaller module

The function stops the kernel error signaller module.

\return Returns a tEplKernel error code.

\ingroup module_errsigk
*/
//------------------------------------------------------------------------------
tEplKernel errsigk_exit(void)
{
    tEplKernel      ret;

    ret = kEplSuccessful;
    //TODO: Reset Error queue and error objects: requires posting event to user side.
    if (instance_l.m_Status != kNoBuffer)
    {
        if((ret = freeErrStatusBuffers()) != kEplSuccessful)
        {
            goto Exit;
        }
    }
    instance_l.m_Status = kNoBuffer;
    instance_l.m_ErrorBufferHead = NULL;
    instance_l.m_pCurrentErrorBuffer = NULL;
    instance_l.m_pReservedErrorBuffer = NULL;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    creates buffers kernel error signaller module

The function initialises error status buffers for the kernel
error signaller module when called by dllk and decides the ownership of the
buffers

\param      dllErrStatusBuffer      Pointer to where the dllk-owned error status
                                    buffer shall be stored

\return Returns a tEplKernel error code.

\ingroup module_errsigk
*/
//------------------------------------------------------------------------------
tEplKernel errsigk_createErrStatusBuffers(tErrSigkBuffer** dllErrStatusBuffer)
{
    tEplKernel      ret;

    ret = kEplSuccessful;

    if (instance_l.m_Status != kNoBuffer)
    {
        ret = kEplInvalidEvent;
        goto Exit;
    }
    ret = allocateErrStatusBuffers();

    if (ret != kEplSuccessful)
    {
        goto Exit;
    }
    instance_l.m_Status = kBuffersEmpty;
    (*dllErrStatusBuffer) = instance_l.m_ErrorBufferHead;
    (*dllErrStatusBuffer)->m_uiOwner = kOwnerDll;
    instance_l.m_pCurrentErrorBuffer = (*dllErrStatusBuffer)->m_pNextErrorBuffer;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Deallocates the buffers in kernel error signaller module

The function deallocates error status buffers and resets the resets the
error signalling module structure

\param      dllErrStatusBuffer      Pointer to where the dllk-owned error status
                                    buffer

\return Returns a tEplKernel error code.

\ingroup module_errsigk
*/
//------------------------------------------------------------------------------
tEplKernel errsigk_cleanErrStatusBuffers(tErrSigkBuffer** dllErrStatusBuffer)
{
    tEplKernel      ret;

    ret = kEplSuccessful;

    if (instance_l.m_Status == kNoBuffer)
    {
        ret = kEplInvalidEvent;
        goto Exit;
    }
    ret = freeErrStatusBuffers();

    if (ret != kEplSuccessful)
    {
        goto Exit;
    }
    instance_l.m_Status = kNoBuffer;
    (*dllErrStatusBuffer) = NULL;
    instance_l.m_ErrorBufferHead = NULL;
    instance_l.m_pCurrentErrorBuffer = NULL;

Exit:
    return ret;


}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name    private functions
/// \{

//------------------------------------------------------------------------------
/**
\brief    allocates the buffers for error signaller module

The function allocates the error status buffers.

\return Returns a tEplKernel error code.

\ingroup module_errsigk
*/
//------------------------------------------------------------------------------

static tEplKernel allocateErrStatusBuffers(void)
{
    tEplKernel      ret;
    UINT8           loopCount;
    tErrSigkBuffer* currentErrSigkBuffer;
    tErrSigkBuffer* previousErrSigkBuffer;

    ret = kEplSuccessful;

    currentErrSigkBuffer = (tErrSigkBuffer*) EPL_MALLOC(sizeof (tErrSigkBuffer));

    if (currentErrSigkBuffer == NULL)
    {
        ret = kEplNoResource;
        goto Exit;
    }

    EPL_MEMSET(currentErrSigkBuffer,0,sizeof(tErrSigkBuffer));
    
    currentErrSigkBuffer->m_uiOwner = kOwnerReserved;
    instance_l.m_pReservedErrorBuffer = currentErrSigkBuffer;

    for (loopCount = 1; loopCount < MAX_STATUS_FRAMES; loopCount++)
    {
        currentErrSigkBuffer = (tErrSigkBuffer*) EPL_MALLOC(sizeof (tErrSigkBuffer));

        if (currentErrSigkBuffer == NULL)
        {
            ret = kEplNoResource;
            break;
        }

        EPL_MEMSET(currentErrSigkBuffer,0,sizeof (tErrSigkBuffer));
        currentErrSigkBuffer->m_uiOwner = kOwnerErrSigk;
        if (loopCount == 1)
        {
            instance_l.m_ErrorBufferHead = currentErrSigkBuffer;
        }
        else
        {
            if (loopCount == (MAX_STATUS_FRAMES - 1))
            {
                currentErrSigkBuffer->m_pNextErrorBuffer = instance_l.m_ErrorBufferHead;
            }

            previousErrSigkBuffer->m_pNextErrorBuffer = currentErrSigkBuffer;
        }
        previousErrSigkBuffer = currentErrSigkBuffer;
       // printf("ErrBufs: #%d- %x\n", loopCount, currentErrSigkBuffer);
    }

Exit:

    return ret;
}


//------------------------------------------------------------------------------
/**
\brief    deallocates the buffers for error signaller module

The function deallocates the error status buffers.

\return Returns a tEplKernel error code.

\ingroup module_errsigk
*/
//------------------------------------------------------------------------------

static tEplKernel freeErrStatusBuffers(void)
{
    tEplKernel      ret;
    UINT8           loopCount;
    tErrSigkBuffer* currentErrSigkBuffer;
    tErrSigkBuffer* nextErrSigkBuffer;

    ret = kEplSuccessful;

    currentErrSigkBuffer = (tErrSigkBuffer*) instance_l.m_pReservedErrorBuffer;

    if (currentErrSigkBuffer != NULL)
    {
       EPL_FREE(currentErrSigkBuffer);
    }
    instance_l.m_pReservedErrorBuffer = NULL;

    for (loopCount = 1; loopCount < (MAX_STATUS_FRAMES - 1); loopCount++)
    {


        if (loopCount == 1)
        {
            currentErrSigkBuffer = (tErrSigkBuffer*) instance_l.m_ErrorBufferHead;
        }

        nextErrSigkBuffer = currentErrSigkBuffer->m_pNextErrorBuffer;

        if (currentErrSigkBuffer != NULL)
        {
           EPL_FREE(currentErrSigkBuffer);
        }
        currentErrSigkBuffer = nextErrSigkBuffer;
    }

    return ret;
}
/// \}
