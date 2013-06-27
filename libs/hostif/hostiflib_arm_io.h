/*
 * hostiflib_arm_io.h
 *
 *  Created on: May 14, 2013
 *      Author: John
 */

#ifndef HOSTIFLIB_ARM_IO_H_
#define HOSTIFLIB_ARM_IO_H_

#include <xil_io.h>
#include <xbasic_types.h>

u32 ARM_ZYNQ_RD_32DIRECT(void* base,void* offset);
u16 ARM_ZYNQ_RD_16DIRECT(void* base,void* offset);
u8  ARM_ZYNQ_RD_8DIRECT(void* base,void* offset);

void ARM_ZYNQ_WR_32DIRECT(void* base,void* offset,u32 dword);
void ARM_ZYNQ_WR_16DIRECT(void* base,void* offset,u16 dword);
void ARM_ZYNQ_WR_8DIRECT(void* base,void* offset,u8 dword);

#endif /* ARM_IO_H_ */
