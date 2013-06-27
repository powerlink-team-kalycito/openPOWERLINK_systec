/**
********************************************************************************
\file   hostiflib_microblaze.c

\brief  Host Interface Library Support File - For Microblaze target

This file provides specific funtion definition for Xilinx Microblaze CPU.

*******************************************************************************/

#include "hostiflib_microblaze.h"


int hostiflib_RegisterHandler (u32 BaseAddress, int InterruptId,
	   XInterruptHandler Handler, void *CallBackRef)
{
	//TODO: Cleanup
	//printf("[PCP]Host IF Register IRQ \n\r");
	XIntc_RegisterHandler(BaseAddress,InterruptId,Handler,CallBackRef);
	return 1;
}
