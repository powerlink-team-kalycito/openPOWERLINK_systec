/*
 * hostiflib_arm.c
 *
 *  Created on: May 14, 2013
 *      Author: John
 */
#include "hostiflib_arm.h"

void hostif_FlushDCacheRange(u32 dwAddr_p,u16 span_p)
{
	Xil_DCacheFlushRange(dwAddr_p, span_p);
}

void hostif_InvalidateDCacheRange(u32 dwAddr_p,u16 span_p)
{
	Xil_DCacheInvalidateRange(dwAddr_p, span_p);
}

u32 ARM_ZYNQ_RD_32DIRECT(void* base,void* offset)
{
	u32 Address = (u32)base + (u32)offset;
	Xil_DCacheInvalidateRange(Address,4);
	return Xil_In32(Address);;
}
u16 ARM_ZYNQ_RD_16DIRECT(void* base,void* offset)
{
	u32 Address = (u32)base + (u32)offset;
	Xil_DCacheInvalidateRange(Address,2);
	return Xil_In16(Address);;
}
u8  ARM_ZYNQ_RD_8DIRECT(void* base, void* offset)
{
	u32 Address = (u32)base + (u32)offset;
	Xil_DCacheInvalidateRange(Address,1);
	return Xil_In8(Address);;
}

void ARM_ZYNQ_WR_32DIRECT(void* base,void* offset,u32 dword)
{
	u32 Address = (u32)base + (u32)offset;
	Xil_Out32(Address, dword);
	Xil_DCacheFlushRange(Address,4);
}
void ARM_ZYNQ_WR_16DIRECT(void* base,void* offset,u16 word)
{
	u32 Address = (u32)base + (u32)offset;
	Xil_Out16(Address, word);
	Xil_DCacheFlushRange(Address,2);
}
void ARM_ZYNQ_WR_8DIRECT(void* base,void* offset,u8 byte)
{
	u32 Address = (u32)base + (u32)offset;
	Xil_Out8(Address, byte);
	Xil_DCacheFlushRange(Address,1);
}

volatile void* xil_uncached_malloc (size_t size)
{
  void* ptr;

  ptr = malloc (size);

  Xil_DCacheFlushRange((unsigned int)ptr, size);

  return ptr ? (volatile void*) (((u32) ptr) | ARM_BYPASS_DCACHE_MASK) : NULL;
}


/*
 * Free a block of uncached memory.
 */

void xil_uncached_free (volatile void* ptr)
{
  free ((void*) (((u32) ptr) & ~ARM_BYPASS_DCACHE_MASK));
}
