/*
 * hostiflib_arm_exception.h
 *
 *  Created on: May 15, 2013
 *      Author: John
 */

#ifndef HOSTIFLIB_ARM_EXCEPTION_H_
#define HOSTIFLIB_ARM_EXCEPTION_H_

#include <xscugic_hw.h>

int arm_register_handler(u32 HOSTIF_IRQ_IC_ID, u32 HOSTIF_IRQ, Xil_ExceptionHandler* CbFunction, void* Arg);

#endif /* HOSTIFLIB_ARM_EXCEPTION_H_ */
