/**
********************************************************************************
\file   hostiflib_microblaze.c

\brief  Host Interface Library Support File - For Microblaze target

This file provides specific funtion definition for Xilinx Microblaze CPU.

*******************************************************************************/

#include "hostiflib_microblaze.h"
#include "Benchmark.h"

int hostiflib_RegisterHandler (u32 BaseAddress, int InterruptId,
	   XInterruptHandler Handler, void *CallBackRef)
{
	//TODO: Cleanup
	//printf("[PCP]Host IF Register IRQ \n\r");
	XIntc_RegisterHandler(BaseAddress,InterruptId,Handler,CallBackRef);
	return 1;
}
void hostif_FlushDCacheRange(u32 dwAddr_p,u16 span_p)
{
	//BENCHMARK_MOD_02_SET(2);
	microblaze_flush_dcache_range(dwAddr_p, span_p);
	//BENCHMARK_MOD_02_RESET(2);
}

void hostif_InvalidateDCacheRange(u32 dwAddr_p,u16 span_p)
{
	microblaze_invalidate_dcache_range(dwAddr_p, span_p);
}

u32 MB_READ32(u32 dwBase_p,u32 offset_p)
{

	microblaze_invalidate_dcache_range((dwBase_p+offset_p), 4);
	return Xil_In32(dwBase_p+offset_p);
}
u16 MB_READ16(u32 dwBase_p,u32 offset_p)
{

	microblaze_invalidate_dcache_range((dwBase_p+offset_p), 2);
	return Xil_In16(dwBase_p+offset_p);

}

u8 MB_READ8(u32 dwBase_p,u32 offset_p)
{
	microblaze_invalidate_dcache_range((dwBase_p+offset_p), 1);
	return Xil_In8(dwBase_p+offset_p);
}

void MB_WRITE32(u32 dwBase_p,u32 offset_p,u32 Val_p)
{
	Xil_Out32((dwBase_p+offset_p),Val_p);
	microblaze_flush_dcache_range((dwBase_p+offset_p), 4);
}

void MB_WRITE16(u32 dwBase_p,u32 offset_p,u16 Val_p)
{
	Xil_Out16((dwBase_p+offset_p),Val_p);
	microblaze_flush_dcache_range((dwBase_p+offset_p), 2);
}

void MB_WRITE8(u32 dwBase_p,u32 offset_p,u8 Val_p)
{
	Xil_Out8((dwBase_p+offset_p),Val_p);
	microblaze_flush_dcache_range((dwBase_p+offset_p), 1);
}
