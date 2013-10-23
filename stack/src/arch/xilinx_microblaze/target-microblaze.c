/**
********************************************************************************
\file   target-microblaze.c

\brief  Target specific functions for Microblaze without OS

This target depending module provides several functions that are necessary for
systems without OS and not using shared buffer library.

\ingroup module_target
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <Epl.h>
#include <xparameters.h>
#include "xilinx_usleep.h"
#include "xilinx_irq.h"
#include "systemComponents.h"
//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define TGTCONIO_MS_IN_US(x)    (x*1000U)

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

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Returns current system tick

This function returns the current system tick determined by the system timer.

\return The function returns the system tick in milliseconds

\ingroup module_target
*/
//------------------------------------------------------------------------------
DWORD PUBLIC EplTgtGetTickCountMs (void)
{
    DWORD dwTicks;

    dwTicks = getMSCount();

    return dwTicks;
}

//------------------------------------------------------------------------------
/**
\brief    Enable global interrupt

This function enables/disables global interrupts.

\param  fEnable_p               TRUE = enable interrupts
                                FALSE = disable interrupts

\ingroup module_target
*/
//------------------------------------------------------------------------------
void PUBLIC EplTgtEnableGlobalInterrupt (BYTE fEnable_p)
{
    static int              iLockCount = 0;

    if (fEnable_p != FALSE)
    {   // restore interrupts
        if (--iLockCount == 0)
        {
            enableInterruptMaster();
        }
    }
    else
    {   // disable interrupts
        if (iLockCount == 0)
        {
            disableInterruptMaster();
        }
        iLockCount++;
    }
}

//------------------------------------------------------------------------------
/**
\brief    Check if CPU is in interrupt context

This function obtains if the CPU is in interrupt context.

\return This function returns the current CPU interrupt context status
\retval TRUE    CPU is in interrupt context
\retval FALSE   CPU is NOT in interrupt context

\ingroup module_target
*/
//------------------------------------------------------------------------------
BYTE PUBLIC EplTgtIsInterruptContext (void)
{
    // No real interrupt context check is performed.
    // This would be possible with a flag in the ISR, only.
    // For now, the global interrupt enable flag is checked.

    DWORD dwGIE;

    dwGIE = Xil_In32(XPAR_PCP_INTC_BASEADDR + XIN_MER_OFFSET) & \
            XIN_INT_MASTER_ENABLE_MASK;

    if(dwGIE == 0)
    {
        //master enable is off
        return TRUE;
    }
    else
    {
        //master enable is on
        return FALSE;
    }
}

//------------------------------------------------------------------------------
/**
\brief  Initialize target specific stuff

The function initialize target specific stuff which is needed to run the
openPOWERLINK stack.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
tEplKernel target_init(void)
{
    // Initialize peripherals
    SysComp_initPeripheral();

    // Initialize interrupts
    SysComp_enableInterrupts();

    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Cleanup target specific stuff

The function cleans-up target specific stuff.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
tEplKernel target_cleanup(void)
{
    //TODO jz: do stuff in systemComponents.c here?

    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief Sleep for the specified number of milliseconds

The function makes the calling thread sleep until the number of specified
milliseconds have elapsed.

\param  milliSecond_p       Number of milliseconds to sleep

\ingroup module_target
*/
//------------------------------------------------------------------------------
void target_msleep (unsigned int milliSecond_p)
{
    usleep(TGTCONIO_MS_IN_US(milliSecond_p));
}
//------------------------------------------------------------------------------
/**
\brief Register synchronization interrupt handler

The function registers the ISR for target specific synchronization interrupt
used by the application for PDO and event synchronization.

\param  callback_p              Interrupt handler
\param  pArg_p                  Argument to be passed while calling the handler

\return The function returns the error code as a integer value
\retval 0 if able to register
\retval other if not

\ingroup module_target
*/
//------------------------------------------------------------------------------
int target_regSyncIrqHdl( void* callback_p,void* pArg_p)
{
    // todo gks: Add Target interrupt registration for sync here
    return 0;
}

//------------------------------------------------------------------------------
/**
\brief Sync interrupt control rroutine

The function is used to enable or disable the sync interrupt

\param  fEnable_p              enable if TRUE, disable if FALSE

\ingroup module_target
*/
//------------------------------------------------------------------------------
void target_enableSyncIrq(BOOL fEnable_p)
{
    // todo gks Add interrupt handling
}
//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
