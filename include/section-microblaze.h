/**
********************************************************************************
\file   section-microblaze.h

\brief  Macros for special function linking for Xilinx Microblaze

This header file defines macros for Xilinx Microblaze targets to link specific
functions to local memory.

Copyright (c) 2012, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2012, SYSTEC electronik GmbH
Copyright (c) 2012, Kalycito Infotech Private Ltd.
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
*******************************************************************************/

#ifndef _INC_SECTION_MICROBLAZE_H_
#define _INC_SECTION_MICROBLAZE_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//TODO: Avoid Internal memory
//FIXME: @Vinod No internal memoery for Zynq  ???
#define XIL_INTERNAL_RAM    __attribute__((section(".local_memory")))
//#define XIL_PLK_MN //TODO: @Vinod:From where this define is coming ???
#ifdef XIL_PLK_MN
//TODO: Added by Vinod
	#define SECTION_OMETHLIB_RX_IRQ_HDL     ALT_INTERNAL_RAM
	#define SECTION_OMETHLIB_TX_IRQ_HDL     ALT_INTERNAL_RAM
	#define SECTION_EDRVOPENMAC_RX_HOOK     ALT_INTERNAL_RAM
	#define SECTION_EDRVOPENMAC_IRQ_HDL     ALT_INTERNAL_RAM
	#define SECTION_DLLK_FRAME_RCVD_CB      ALT_INTERNAL_RAM
	#define SECTION_EVENT_GET_HDL_FOR_SINK  ALT_INTERNAL_RAM
	#define SECTION_EVENTK_POST             ALT_INTERNAL_RAM
	#define SECTION_EVENTKCAL_POST          ALT_INTERNAL_RAM
	#define SECTION_EVENTCAL_DIRECT_POST    ALT_INTERNAL_RAM
	#define SECTION_EVENTCAL_HOSTIF_POST    ALT_INTERNAL_RAM
	#define SECTION_EVENTK_PROCESS          ALT_INTERNAL_RAM
	#define SECTION_EVENTKCAL_RXHDL         ALT_INTERNAL_RAM
	#define SECTION_EVENTCAL_HOSTIF_RXHDL   ALT_INTERNAL_RAM
	#define SECTION_PDOK_PROCESS_TPDO_CB    ALT_INTERNAL_RAM
	#define SECTION_PDOKCAL_READ_TPDO       ALT_INTERNAL_RAM
	#define SECTION_PDOK_PROCESS_RPDO       ALT_INTERNAL_RAM
	#define SECTION_PDOKCAL_WRITE_RPDO      ALT_INTERNAL_RAM
	#define SECTION_EDRVCYC_TIMER_CB        ALT_INTERNAL_RAM
	#define SECTION_HRTIMER_IRQ_HDL         ALT_INTERNAL_RAM
	#define SECTION_HRTIMER_MODTIMER        ALT_INTERNAL_RAM
	#define SECTION_DLLK_PROCESS            ALT_INTERNAL_RAM
	#define SECTION_DLLK_PROCESS_CYCFIN     ALT_INTERNAL_RAM
	#define SECTION_DLLK_PROCESS_SYNC       ALT_INTERNAL_RAM
	#define SECTION_DLLKCAL_GETSOAREQ       ALT_INTERNAL_RAM
	#define SECTION_ERRHNDK_DECRCNTERS      ALT_INTERNAL_RAM
	#define SECTION_ERRHNDKCAL_GETMNCNT     ALT_INTERNAL_RAM
	#define SECTION_ERRHNDKCAL_SETMNCNT     ALT_INTERNAL_RAM

#else
    /* TODO:
     * Find optimal setting again due to revised stack design!
     */
	#if (defined(XIL_NO_OPT_LEVEL) || defined(XIL_OPT_LEVEL_1) || \
	    defined(XIL_OPT_LEVEL_2)   || defined(XIL_OPT_LEVEL_3) || \
	    defined(XIL_OPT_LEVEL_SIZE)                               )

	    #define SECTION_PDOK_PROCESS_TPDO_CB    XIL_INTERNAL_RAM
	    #define SECTION_PDOK_COPY_TPDO          XIL_INTERNAL_RAM
	    #define SECTION_PDOK_PROCESS_RPDO       XIL_INTERNAL_RAM
	    #define SECTION_EVENTK_PROCESS          XIL_INTERNAL_RAM
	    #define SECTION_EVENTK_POST             XIL_INTERNAL_RAM
	    #define SECTION_OMETHLIB_RX_IRQ_HDL     XIL_INTERNAL_RAM
	    #define SECTION_OMETHLIB_TX_IRQ_HDL     XIL_INTERNAL_RAM
	    #define SECTION_EDRVOPENMAC_RX_HOOK     XIL_INTERNAL_RAM
	    #define SECTION_EDRVOPENMAC_IRQ_HDL     XIL_INTERNAL_RAM
	    #define SECTION_MAIN_APP_CB_SYNC        XIL_INTERNAL_RAM
		//TODO: @Vinod : Fixed by adding this define, Is it make sense ?
		#define SECTION_EVENTKCAL_HOSTIF_RXHDL	XIL_INTERNAL_RAM
	#endif

	#if (defined(XIL_OPT_LEVEL_3) || defined(XIL_OPT_LEVEL_SIZE))
	    #define SECTION_DLLK_FRAME_RCVD_CB      XIL_INTERNAL_RAM
	#endif
#endif

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#endif /* _INC_SECTION_MICROBLAZE_H_ */
