/**
********************************************************************************
\file   eventucal-noosdual.c

\brief  User event CAL module using shared memory on NON-OS systems

This user event CAL module implementation uses circular buffers and direct calls
on non-OS systems running on dual processor with shared memory interface.

\see eventucalintf-circbuf.c

\ingroup module_eventucal
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013 Kalycito Infotech Private Limited
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
#include <Epl.h>
#include <user/eventu.h>
#include <user/eventucal.h>
#include <user/eventucalintf.h>

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
/**
\brief Kernel event CAL instance type

The structure contains all necessary information needed by the kernel event
CAL module.
*/
typedef struct
{
    BOOL                    fInitialized;
} tEventuCalInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tEventuCalInstance       instance_l;             ///< Instance variable of kernel event CAL module

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Initialize architecture specific stuff of user event CAL module

The function initializes the architecture specific stuff of the user event
CAL module.

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          If function executes correctly
\retval other error codes       If an error occurred

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
tEplKernel eventucal_init (void)
{
    EPL_MEMSET(&instance_l, 0, sizeof(tEventuCalInstance));

    if (eventucal_initQueueCircbuf(kEventQueueU2K) != kEplSuccessful)
        goto Exit;
    if (eventucal_initQueueCircbuf(kEventQueueK2U) != kEplSuccessful)
        goto Exit;

    instance_l.fInitialized = TRUE;
    return kEplSuccessful;

Exit:
    eventucal_exitQueueCircbuf(kEventQueueK2U);
    eventucal_exitQueueCircbuf(kEventQueueU2K);

    return kEplNoResource;
}

//------------------------------------------------------------------------------
/**
\brief    Cleanup kernel event CAL module

The function cleans up the kernel event CAL module. For cleanup it calls the exit
functions of the queue implementations for each used queue.

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          If function executes correctly
\retval other error codes       If an error occurred

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
tEplKernel eventucal_exit (void)
{
    if (instance_l.fInitialized == TRUE)
    {
        eventucal_exitQueueCircbuf(kEventQueueK2U);
        eventucal_exitQueueCircbuf(kEventQueueU2K);
    }
    instance_l.fInitialized = FALSE;

    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief    Post kernel event

This function posts a event to a queue. It is called from the generic kernel
event post function in the event handler. Depending on the sink the appropriate
queue post function is called.

\param  pEvent_p                Event to be posted.

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          If function executes correctly
\retval other error codes       If an error occurred

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
tEplKernel eventucal_postKernelEvent (tEplEvent *pEvent_p)
{
    tEplKernel      ret;
   /* TRACE("U2K type:%s(%d) sink:%s(%d) size:%d!\n",
                   EplGetEventTypeStr(pEvent_p->m_EventType), pEvent_p->m_EventType,
                   EplGetEventSinkStr(pEvent_p->m_EventSink), pEvent_p->m_EventSink,
                   pEvent_p->m_uiSize);*/

    EplTgtEnableGlobalInterrupt(FALSE);
    ret = eventucal_postEventCircbuf(kEventQueueU2K,pEvent_p);
    EplTgtEnableGlobalInterrupt(TRUE);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Post user event

This function posts a event to a queue. It is called from the generic kernel
event post function in the event handler. Depending on the sink the appropriate
queue post function is called.

\param  pEvent_p                Event to be posted.

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          If function executes correctly
\retval other error codes       If an error occurred

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
tEplKernel eventucal_postUserEvent (tEplEvent *pEvent_p)
{
    tEplKernel      ret;

    /*TRACE(" type:%s(%d) sink:%s(%d) size:%d!\n",
                   EplGetEventTypeStr(pEvent_p->m_EventType), pEvent_p->m_EventType,
                   EplGetEventSinkStr(pEvent_p->m_EventSink), pEvent_p->m_EventSink,
                   pEvent_p->m_uiSize);*/
    ret = eventu_process(pEvent_p);

    return ret;
}


//------------------------------------------------------------------------------
/**
\brief  Process function of user CAL module

This function will be called by the systems process function.

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
void eventucal_process(void)
{
    if (eventucal_getEventCountCircbuf(kEventQueueK2U) > 0)
    {
        // Disable interrupts to avoid deadlocks
        EplTgtEnableGlobalInterrupt(FALSE);
        eventucal_processEventCircbuf(kEventQueueK2U);
        EplTgtEnableGlobalInterrupt(TRUE);
    }
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}

