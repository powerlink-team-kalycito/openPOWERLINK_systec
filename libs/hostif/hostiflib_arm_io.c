/*
 * hostiflib_arm_io.c
 *
 *  Created on: May 14, 2013
 *      Author: John
 */
#include "hostiflib_arm_io.h"

u32 ARM_ZYNQ_RD_32DIRECT(void* base,void* offset)
{
	u32 Address = (u32)base + (u32)offset;
	u32 Read = Xil_In32(Address);
	return Read;
}
u16 ARM_ZYNQ_RD_16DIRECT(void* base,void* offset)
{
	u32 Address = (u32)base + (u32)offset;
	u16 Read = Xil_In16(Address);
	return Read;
}
u8  ARM_ZYNQ_RD_8DIRECT(void* base, void* offset)
{
	u32 Address = (u32)base + (u32)offset;
	u8 Read = Xil_In8(Address);
	return Read;
}

void ARM_ZYNQ_WR_32DIRECT(void* base,void* offset,u32 dword)
{
	u32 Address = (u32)base + (u32)offset;
	Xil_Out32(Address, dword);
}
void ARM_ZYNQ_WR_16DIRECT(void* base,void* offset,u16 dword)
{
	u32 Address = (u32)base + (u32)offset;
	Xil_Out16(Address, dword);
}
void ARM_ZYNQ_WR_8DIRECT(void* base,void* offset,u8 dword)
{
	u32 Address = (u32)base + (u32)offset;
	Xil_Out8(Address, dword);
}
