/**
********************************************************************************
\file   target-arm.c

\brief  Target specific functions for ARM on Zynq without OS

This target depending module provides several functions that are necessary for
systems without OS and not using the shared buffer library.

\ingroup module_target
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, Kalycito Infotech Pvt Ltd, Coimbatore
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
#include <global.h>
#include <xscugic.h>
#include <xtime_l.h>
#include "systemComponents.h"
#include "xil_cache.h"
#include "xil_types.h"
#include "xscugic.h"
#include "xil_io.h"
#include "xil_exception.h"
#include <unistd.h>


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
#define TGTCONIO_MS_IN_US(x)         (x*1000U)
#define TARGET_SYNC_IRQ_ID           XPAR_PS7_SCUGIC_0_DEVICE_ID
#define TARGET_SYNC_IRQ              XPAR_FABRIC_AXI_POWERLINK_0_TCP_IRQ_INTR
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
UINT32 PUBLIC EplTgtGetTickCountMs(void)
{
    UINT32 dwTicks;
    XTime* ticks;
    /*Uses global timer functions*/

    XTime_GetTime(ticks);
    /*Select the lower 32 bit of the timer value*/
    dwTicks = (UINT32)(((2000 * (*ticks))/XPAR_CPU_CORTEXA9_CORE_CLOCK_FREQ_HZ));

    return dwTicks;
}

//------------------------------------------------------------------------------
/**
\brief    Enables global interrupt

This function enabels/disables global interrupts.

\param  fEnable_p               TRUE = enable interrupts
                                FALSE = disable interrupts

\ingroup module_target
*/
//------------------------------------------------------------------------------
void EplTgtEnableGlobalInterrupt (UINT8 fEnable_p)
{
    if(fEnable_p == TRUE)
    {
        SysComp_enableInterrupts();
    }
    else
    {
        SysComp_disableInterrupts();
    }
}

//------------------------------------------------------------------------------
/**
\brief    Checks if CPU is in interrupt context

This function obtains if the CPU is in interrupt context.

\return This function returns the current CPU interrupt context status
\retval TRUE        CPU is in interrupt context
\retval FALSE       CPU is NOT in interrupt context

\ingroup module_target
*/
//------------------------------------------------------------------------------
UINT8 EplTgtIsInterruptContext (void)
{
    // No real interrupt context check is performed.
    // This would be possible with a flag in the ISR, only.
    // For now, the global interrupt enable flag is checked.

    // Read the distributor state
    u32 Distributor_state = Xil_In32(XPAR_PS7_SCUGIC_0_DIST_BASEADDR + XSCUGIC_DIST_EN_OFFSET);
    // Read the DP (Distributor) and CP (CPU interface) state
    u32 CPUif_state = Xil_In32(XPAR_SCUGIC_0_CPU_BASEADDR + XSCUGIC_CONTROL_OFFSET);

    if(Distributor_state && CPUif_state)
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}
//------------------------------------------------------------------------------
/**
\brief  Initialize target specific stuff

The function initialize target specific stuff which is needed to run the
openPOWERLINK stack.

\return The function returns a tEplKernel error code.

\ingroup module_target
*/
//------------------------------------------------------------------------------
tEplKernel target_init(void)
{
u32 version = 0;
#if defined(__arm__)

    Xil_DCacheFlush();
    SysComp_initPeripheral();

    return kEplSuccessful;
#else
    // Add Here any other platform specific code
	return kEplSuccessful;
#endif
}

//------------------------------------------------------------------------------
/**
\brief  Cleanup target specific stuff

The function cleans-up target specific stuff.

\return The function returns a tEplKernel error code.

\ingroup module_target
*/
//------------------------------------------------------------------------------
tEplKernel target_cleanup(void)
{
    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief Sleep for the specified number of milliseconds

The function makes the calling thread sleep until the number of specified
milliseconds have elapsed.

\param  milliSecond_p              Number of milliseconds to sleep

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
    return SysComp_initSyncInterrupt(TARGET_SYNC_IRQ,(Xil_InterruptHandler) callback_p,pArg_p);
}
//------------------------------------------------------------------------------
/**
\brief Sync interrupt control routine

The function is used to enable or disable the sync interrupt

\param  fEnable_p              enable if TRUE, disable if FALSE

\ingroup module_target
*/
//------------------------------------------------------------------------------
void target_enableSyncIrq(BOOL fEnable_p)
{
    if(fEnable_p)
    {
        XScuGic_EnableIntr(TARGET_SYNC_IRQ_ID, TARGET_SYNC_IRQ);
    }
    else
    {
        XScuGic_DisableIntr(TARGET_SYNC_IRQ_ID, TARGET_SYNC_IRQ);
    }

}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
