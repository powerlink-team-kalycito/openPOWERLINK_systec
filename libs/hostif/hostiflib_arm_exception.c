/*
 * hostiflib_arm_exception.c
 *
 *  Created on: May 15, 2013
 *      Author: John
 */

#include "hostiflib_arm_exception.h"


int arm_register_handler(u32 HOSTIF_IRQ_IC_ID, u32 HOSTIF_IRQ, Xil_ExceptionHandler* CbFunction, void* Arg)
{
	XScuGic_RegisterHandler(HOSTIF_IRQ_IC_ID, HOSTIF_IRQ,
			(Xil_InterruptHandler)CbFunction, Arg);
	XScuGic_EnableIntr(HOSTIF_IRQ_IC_ID, HOSTIF_IRQ);

	/*The functions return void*/
	return 0;//FIXME:@John: See if this can be verified(success or failure) from any register
}
