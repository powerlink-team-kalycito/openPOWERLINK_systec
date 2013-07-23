/**
********************************************************************************
\file       systemComponents.c

\brief      Module which contains processor specific definitions
            (Zynq-ARM version)

This module contains of platform specific definitions.

Copyright © 2011 BERNECKER + RAINER, AUSTRIA, 5142 EGGELSBERG, B&R STRASSE 1
All rights reserved. All use of this software and documentation is
subject to the License Agreement located at the end of this file below.

*******************************************************************************/

/*******************************************************************************/
/* includes */
#include "systemComponents.h"

#include "xil_cache.h"
#include "xil_types.h"
#include "xil_io.h"
#include "xil_exception.h"
#include "hostiflib_arm.h"
#include "EplTarget.h"
#include <unistd.h>


/******************************************************************************/
/* defines */

/******************************************************************************/
/* typedefs */

/******************************************************************************/
/* external variable declarations */

extern XScuGic_Config XScuGic_ConfigTable[];

/******************************************************************************/
/* global variables */

XScuGic sGicInstance_l;

/******************************************************************************/
/* function declarations */
static int Gic_CfgInitialize(XScuGic *pInstancePtr_p, unsigned int uiDeviceID);
static void Gic_InitDistributor (XScuGic_Config *pConfig_p, int iCpuID_p);
static void Gic_InitCpuInterface (XScuGic_Config *Config);
static void Gic_StubHandler(void *CallBackRef);
static void Gic_InterruptHandler(XScuGic *InstancePtr);

/******************************************************************************/
/* private functions */

/******************************************************************************/
/* functions */

/**
********************************************************************************
\brief  init the peripherals of the ARM processor

This function init's the peripherals of the ARM like cache and the interrupt
controller.
*******************************************************************************/
void SysComp_initPeripheral(void)
{
	/*Release MB!*/

	Xil_Out32(SLCR_UNLOCK, SLCR_UNLOCK_VAL);
    Xil_Out32(FPGA_RST_CNTRL,0);
    Xil_Out32(SLCR_LOCK, SLCR_LOCK_VAL);

    Xil_ICacheEnable();
    Xil_DCacheEnable();

    // Initialize Interrupts
	SysComp_InitInterrupts();

}

/**
********************************************************************************
\brief enables the interrupts

This function enables the interrupts of the Host processor on ARM
*******************************************************************************/
inline void SysComp_enableInterrupts(void)
{
	//Global interrupt enable
	//Distributor global enable
	Xil_Out32((XPAR_PS7_SCUGIC_0_DIST_BASEADDR + XSCUGIC_DIST_EN_OFFSET), XSCUGIC_EN_INT_MASK);	
	//CPU interface global enable
	Xil_Out32((XPAR_SCUGIC_0_CPU_BASEADDR + XSCUGIC_CONTROL_OFFSET), XSCUGIC_CNTR_EN_S_MASK);
}

/**
********************************************************************************
\brief disables the interrupts

This function disables the interrupts of the Host processor on ARM
*******************************************************************************/
inline void SysComp_disableInterrupts(void)
{
	// Disable all interrupts from the distributor
	Xil_Out32((XPAR_PS7_SCUGIC_0_DIST_BASEADDR + XSCUGIC_DIST_EN_OFFSET), 0UL);
	// Reset the DP (Distributor) and CP (CPU interface)
	Xil_Out32((XPAR_SCUGIC_0_CPU_BASEADDR + XSCUGIC_CONTROL_OFFSET), 0UL);
}

/**
********************************************************************************
\brief  initialize synchronous interrupt

SysComp_initSyncInterrupt() initializes the synchronous interrupt. The timing parameters
will be initialized, the interrupt handler will be connected and the interrupt
will be enabled.

\param  Int_p                    Interrupt ID of the sync interrupt
\param  callbackFunc             The callback of the sync interrupt
\param  Arg_p                    argument to be passed while calling callback

\return int
\retval OK                      on success
\retval ERROR                   if interrupt couldn't be connected
*******************************************************************************/
int SysComp_initSyncInterrupt(int Int_p,Xil_InterruptHandler callbackFunc_p,void* Arg_p)
{

#ifdef _USE_HIGH_PRIO_
	//If priority value is not set, then interrupts are dispatched based on IDs
	XScuGic_SetPriorityTriggerType(&sGicInstance_l, SYNC_INTR_ID, 
								SYNC_INTR_PRIORITY, TRIGGER_VALUE);
#endif

	//register sync irq handler
    XScuGic_RegisterHandler(ARM_IRQ_IC_BASE, Int_p,	callbackFunc_p, Arg_p);
    //enable the sync interrupt
    XScuGic_EnableIntr(ARM_IRQ_IC_DIST_BASE, Int_p);


    return OK;
}

/**
********************************************************************************
\brief  Enable synchronous interrupt

SysComp_enableSyncInterrupt() enables the synchronous interrupt.
*******************************************************************************/
inline void SysComp_enableSyncInterrupt(void)
{
#ifdef XPAR_FABRIC_AXI_POWERLINK_0_AP_SYNCIRQ_VEC_ID
   XScuGic_Enable(&sGicInstance_l, SYNC_IRQ_NUM);
#endif
}

/**
********************************************************************************
\brief  Disable synchronous interrupt

SysComp_disableSyncInterrupt() disable the synchronous interrupt.
*******************************************************************************/
inline void SysComp_disableSyncInterrupt(void)
{
#ifdef XPAR_FABRIC_AXI_POWERLINK_0_AP_SYNCIRQ_VEC_ID
	XScuGic_Disable(&sGicInstance_l, SYNC_IRQ_NUM);
	//TODO: Check if we have to disconnect handler
#endif
}
/**
********************************************************************************
\brief  initialize asynchronous interrupt

SysComp_initAsyncInterrupt() initializes the asynchronous interrupt. The interrupt handler
will be connected and the interrupt will be enabled.

\param  callbackFunc             The callback of the async interrupt

\return int
\retval OK                       on success
\retval ERROR                    if interrupt couldn't be connected
*******************************************************************************/
int SysComp_initAsyncInterrupt(void (*callbackFunc)(void*))
{

#ifdef _USE_HIGH_PRIO_
	//If priority value is not set, then interrupts are dispatched based on IDs
	XScuGic_SetPriorityTriggerType(&sGicInstance_l, SYNC_INTR_ID, 
								ASYNC_INTR_PRIORITY, TRIGGER_VALUE);
#endif

#ifdef XPAR_FABRIC_AXI_POWERLINK_0_AP_ASYNCIRQ_VEC_ID
	/* register interrupt handler */
    XScuGic_Connect(&sGicInstance_l, ASYNC_IRQ_NUM,
            (Xil_InterruptHandler)callbackFunc, 0);

   //enable the sync interrupt
   XScuGic_Enable(&sGicInstance_l, ASYNC_IRQ_NUM);
#endif
   return OK;
}

/**
********************************************************************************
\brief  Enable synchronous interrupt

SysComp_enableSyncInterrupt() enables the synchronous interrupt.
*******************************************************************************/
inline void SysComp_enableAsyncInterrupt(void)
{
#ifdef XPAR_FABRIC_AXI_POWERLINK_0_AP_ASYNCIRQ_VEC_ID
   XScuGic_Enable(&sGicInstance_l, ASYNC_IRQ_NUM);
#endif
}

/**
********************************************************************************
\brief  Disable synchronous interrupt

SysComp_disableSyncInterrupt() disable the synchronous interrupt.
*******************************************************************************/
inline void SysComp_disableAsyncInterrupt(void)
{
#ifdef XPAR_FABRIC_AXI_POWERLINK_0_AP_ASYNCIRQ_VEC_ID
	XScuGic_Disable(&sGicInstance_l, ASYNC_IRQ_NUM);
	//TODO: Check if we have to disconnect handler
#endif
}

#ifdef MN_API_USING_SPI
/**
********************************************************************************
\brief  Execute SPI command

SysComp_SPICommand() sends an SPI command to the SPI master by using the
alt_avalon_spi_command function

\param  pTxBuf_p             A pointer to the buffer to send
\param  pRxBuf_p             A pointer to the buffer where the data should be stored
\param  iBytes_p             The number of bytes to send or receive

\return int
\retval OK                       on success
\retval ERROR                    in case of an error
*******************************************************************************/
int SysComp_SPICommand(unsigned char *pTxBuf_p, unsigned char *pRxBuf_p, int iBytes_p)
{
	//TODO: to be integrated later	 
	return OK;
}
#endif //CN_API_USING_SPI

/**
********************************************************************************
\brief  write a value to the output port

This function writes a value to the output port of the AP

\param  dwValue_p       the value to write
*******************************************************************************/
void SysComp_writeOutputPort(DWORD dwValue_p)
{
    #ifdef OUTPORT_AP_BASE_ADDRESS
        XGpio_WriteReg(OUTPORT_AP_BASE_ADDRESS, XGPIO_DATA_OFFSET, dwValue_p);
    #endif
}

/**
********************************************************************************
\brief  read a value from the input port

This function reads a value from the input port of the AP

\return  DWORD
\retval  dwValue              the value of the input port
*******************************************************************************/
DWORD SysComp_readInputPort(void)
{
    DWORD dwValue = 0;

    #ifdef INPORT_AP_BASE_ADDRESS
        dwValue = XGpio_ReadReg(INPORT_AP_BASE_ADDRESS, XGPIO_DATA_OFFSET);
    #endif

    return dwValue;
}

/**
*****************************************************************************
\brief		This function initializes the distributor of the GIC on ARM

				- Write the trigger mode, priority and target CPU
				- All interrupt sources are disabled
				- Enable the distributor


\param	pConfig_p		
\param	iCpuID_p		
******************************************************************************/
static void Gic_InitDistributor (XScuGic_Config *pConfig_p, int iCpuID_p)
{
	int iIntId;

	XScuGic_WriteReg(pConfig_p->DistBaseAddress, XSCUGIC_DIST_EN_OFFSET, 0UL);

	/*
	 * Set the security domains in the int_security registers for non-secure
	 * interrupts. All are secure, so leave at the default. Set to 1 for
	 * non-secure interrupts.
	 */


	/*
	 * For the Shared Peripheral Interrupts INT_ID[MAX..32], set:
	 */

	/*
	 * 1. The trigger mode in the int_config register
	 * Only write to the SPI interrupts, so start at 32
	 */
	for (iIntId = 32; iIntId < XSCUGIC_MAX_NUM_INTR_INPUTS; iIntId+=16)
	{
	/*
	 * Each INT_ID uses two bits, or 16 INT_ID per register
	 * Set them all to be level sensitive, active HIGH.
	 */
		XScuGic_WriteReg(pConfig_p->DistBaseAddress,
			XSCUGIC_INT_CFG_OFFSET_CALC(iIntId), 0UL);
	}

	for (iIntId = 0; iIntId < XSCUGIC_MAX_NUM_INTR_INPUTS; iIntId+=4)
	{
		/*
		 * 2. The priority using int the priority_level register
		 * The priority_level and spi_target registers use one byte per
		 * INT_ID.
		 * Write a default value that can be changed elsewhere.
		 */
		XScuGic_WriteReg(pConfig_p->DistBaseAddress,
				XSCUGIC_PRIORITY_OFFSET_CALC(iIntId),
				DEFAULT_PRIORITY);
	}

	for (iIntId = 32; iIntId < XSCUGIC_MAX_NUM_INTR_INPUTS; iIntId+=4)
	{
		/*
		 * 3. The CPU interface in the spi_target register
		 * Only write to the SPI interrupts, so start at 32
		 */
		iCpuID_p |= iCpuID_p << 8;
		iCpuID_p |= iCpuID_p << 16;

		XScuGic_WriteReg(pConfig_p->DistBaseAddress,
 				XSCUGIC_SPI_TARGET_OFFSET_CALC(iIntId), iCpuID_p);
	}

	for (iIntId = 0; iIntId < XSCUGIC_MAX_NUM_INTR_INPUTS; iIntId+=32)
	{
	/*
	 * 4. Enable the SPI using the enable_set register. Leave all disabled
	 * for now.
	 */
		XScuGic_WriteReg(pConfig_p->DistBaseAddress,
		XSCUGIC_ENABLE_DISABLE_OFFSET_CALC(XSCUGIC_DISABLE_OFFSET,
				iIntId),
		0xFFFFFFFFUL);

	}

	XScuGic_WriteReg(pConfig_p->DistBaseAddress, XSCUGIC_DIST_EN_OFFSET,
						XSCUGIC_EN_INT_MASK);
}

/**
*****************************************************************************
\brief		This function initializes the CPU Interface of the GIC

					-Set the priority of the CPU
					-Enable the CPU interface

\param		Config	 Pointer to config structure
******************************************************************************/
static void Gic_InitCpuInterface (XScuGic_Config *Config)
{
	/*
	 * Program the priority mask of the CPU using the Priority mask
	 * register
	 */
	XScuGic_WriteReg(Config->CpuBaseAddress, XSCUGIC_CPU_PRIOR_OFFSET,
									0xF0);

	/*
	 * If the CPU operates in both security domains, set parameters in the
	 * control_s register.
	 * 1. Set FIQen=1 to use FIQ for secure interrupts,
	 * 2. Program the AckCtl bit
	 * 3. Program the SBPR bit to select the binary pointer behavior
	 * 4. Set EnableS = 1 to enable secure interrupts
	 * 5. Set EnbleNS = 1 to enable non secure interrupts
	 */

	/*
	 * If the CPU operates only in the secure domain, setup the
	 * control_s register.
	 * 1. Set FIQen=1,
	 * 2. Set EnableS=1, to enable the CPU interface to signal secure .
	 * interrupts Only enable the IRQ output unless secure interrupts
	 * are needed.
	 */
	XScuGic_WriteReg(Config->CpuBaseAddress, XSCUGIC_CONTROL_OFFSET, 0x07);
}

/**
*****************************************************************************
\brief	callback stub

A stub for the asynchronous callback. The stub is here in case the upper
layers forget to set the handler.

\param	CallBackRef 		is a pointer to the upper layer callback reference
******************************************************************************/
static void Gic_StubHandler(void *CallBackRef)
{
	/*
	 * verify that the inputs are valid
	 */
	Xil_AssertVoid(CallBackRef != NULL);

	/*
	 * Indicate another unhandled interrupt for stats
	 */
	((XScuGic *)CallBackRef)->UnhandledInterrupts++;
}


/**
*****************************************************************************
\brief Interrupt Handler

This function is the primary interrupt handler for the driver.  It must be
connected to the interrupt source such that it is called when an interrupt of
the interrupt controller is active. It will resolve which interrupts are
active and enabled and call the appropriate interrupt handler. It uses
the Interrupt Type information to determine when to acknowledge the interrupt.
Highest priority interrupts are serviced first.

This function assumes that an interrupt vector table has been previously
initialized.  It does not verify that entries in the table are valid before
calling an interrupt handler.

This handler also defaults to having nested interrupts disabled. The define
is XSCUGIC_NESTED_INTERRUPTS and if it is not defined, the option to service
all of the interrupts is honored. If nested interrupts are enabled, the
standard interrupt processing will always only service one interrupt and then
return.

 \param		InstancePtr 		Is a pointer to the XScuGic instance.
******************************************************************************/
static void Gic_InterruptHandler(XScuGic *InstancePtr)
{

    u32 IntID;
    u32 IntIDFull;

    XScuGic_VectorTableEntry *TablePtr;

    /* Assert that the pointer to the instance is valid
     */
    Xil_AssertVoid(InstancePtr != NULL);

    /*
     * Read the int_ack register to identify the highest priority interrupt ID
     * and make sure it is valid. Reading Int_Ack will clear the interrupt
     * in the GIC.
     */
    IntIDFull = XScuGic_CPUReadReg(InstancePtr, XSCUGIC_INT_ACK_OFFSET);
    IntID = IntIDFull & XSCUGIC_ACK_INTID_MASK;
    if(XSCUGIC_MAX_NUM_INTR_INPUTS < IntID){
    	goto IntrExit;
    }

    /*
     * If the interrupt is shared, do some locking here if there are multiple
     * processors.
     */
    /*
     * If pre-eption is required:
     * Re-enable pre-emption by setting the CPSR I bit for non-secure ,
     * interrupts or the F bit for secure interrupts
     */

    /*
     * If we need to change security domains, issue a SMC instruction here.
     */

    /*
     * Execute the ISR. Jump into the Interrupt service routine based on the
     * IRQSource. A software trigger is cleared by the ACK.
     */
        TablePtr = &(InstancePtr->Config->HandlerTable[IntID]);
        TablePtr->Handler(TablePtr->CallBackRef);

IntrExit:
    /*
     * Write to the EOI register, we are all done here.
     * Let this function return, the boot code will restore the stack.
     */
    XScuGic_CPUWriteReg(InstancePtr, XSCUGIC_EOI_OFFSET, IntIDFull);

    /*
     * Return from the interrupt. Change security domains could happen here.
     */
}

/**
*****************************************************************************
\brief	This function initializes a specific interrupt controller instance

			- initialize fields of the XScuGic structure
			- initial vector table with stub function calls
			- init Distributor and CPU interface

\param	pInstancePtr_p		Instance of GIC
\param	uiDeviceID			Device ID of GIC
\return	int
\retval XST_SUCCESS			On success
\retval XST_FAILURE			On error
******************************************************************************/
static int Gic_CfgInitialize(XScuGic *pInstancePtr_p, unsigned int uiDeviceID)
{
	static XScuGic_Config *pConfig;
	int iIntId;
	int iStatus = XST_SUCCESS;

	if (NULL == pInstancePtr_p)
	{
		iStatus = XST_FAILURE;
	}

	pConfig = &XScuGic_ConfigTable[uiDeviceID];
	pInstancePtr_p->IsReady = 0;
	pInstancePtr_p->Config = pConfig;

	for (iIntId = 0; iIntId < XSCUGIC_MAX_NUM_INTR_INPUTS; iIntId ++)
	{
		/*
		 * Initalize the handler to point to a stub to handle an
		 * interrupt which has not been connected to a handler. Only
		 * initialize it if the handler is 0 which means it was not
		 * initialized statically by the tools/user. Set the callback
		 * reference to this instance so that unhandled interrupts
		 * can be tracked.
		 */
		if ((pInstancePtr_p->Config->HandlerTable[iIntId].Handler == 0))
		{
			pInstancePtr_p->Config->HandlerTable[iIntId].Handler =
							Gic_StubHandler;
		}
		pInstancePtr_p->Config->HandlerTable[iIntId].CallBackRef =
								pInstancePtr_p;
	}



	Gic_InitDistributor(pConfig, TARGET_CPU_VALUE);

	Gic_InitCpuInterface(pConfig);

	pInstancePtr_p->IsReady = XIL_COMPONENT_IS_READY;

	return iStatus;
}

/**
********************************************************************************************
\brief	Setup interrupt controller

This function sets up the interrupt and exception handling for interrupt controller
********************************************************************************************/
void SysComp_InitInterrupts(void)
{
	int iStatus;

	iStatus = Gic_CfgInitialize(&sGicInstance_l,XPAR_PS7_SCUGIC_0_DEVICE_ID);
	if (iStatus != XST_SUCCESS)
	{
		return;
	}

    // CPU interrupt interface & distributor has been enabled before this point
	Xil_ExceptionInit();

	/*
	 * Connect the interrupt controller interrupt handler to the hardware
	 * interrupt handling logic in the processor.
	 */
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_DATA_ABORT_INT,
				(Xil_ExceptionHandler)Gic_InterruptHandler,
				&sGicInstance_l);
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_IRQ_INT,
				(Xil_ExceptionHandler)Gic_InterruptHandler,
				&sGicInstance_l);

	Xil_ExceptionEnable();
}

/*******************************************************************************
*
* License Agreement
*
* Copyright © 2011 BERNECKER + RAINER, AUSTRIA, 5142 EGGELSBERG, B&R STRASSE 1
* All rights reserved.
*
* Redistribution and use in source and binary forms,
* with or without modification,
* are permitted provided that the following conditions are met:
*
*   * Redistributions of source code must retain the above copyright notice,
*     this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above copyright notice,
*     this list of conditions and the following disclaimer
*     in the documentation and/or other materials provided with the
*     distribution.
*   * Neither the name of the B&R nor the names of its contributors
*     may be used to endorse or promote products derived from this software
*     without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
* THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
* A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
* THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*******************************************************************************/
/* END-OF-FILE */

