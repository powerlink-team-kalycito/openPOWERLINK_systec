/**
********************************************************************************
\file       /libs/hostif/hostiflib-zynqmem

\brief      Hostif interface memory offsets and size 

This module contains of platform specific definitions.
*******************************************************************************/


#ifndef __DUALPROCSHM_MEM_H__
#define __DUALPROCSHM_MEM_H__

/* BASE ADDRESSES */
#include "EplDef.h"
#include "dllcal.h"
#define DUALPROCSHM_CMS_BASE    0x2C000000

/* SIZE */
#define DUALPROCSHM_SIZE_DYNBUF0         2048
#define DUALPROCSHM_SIZE_DYNBUF1         2048
#define DUALPROCSHM_SIZE_ERRORCOUNTER    3108
#define DUALPROCSHM_SIZE_TXNMTQ          DLLCAL_BUFFER_SIZE_TX_NMT
#define DUALPROCSHM_SIZE_TXGENQ          DLLCAL_BUFFER_SIZE_TX_GEN
#define DUALPROCSHM_SIZE_TXSYNCQ         DLLCAL_BUFFER_SIZE_TX_SYNC
#define DUALPROCSHM_SIZE_TXVETHQ         2064
#define DUALPROCSHM_SIZE_RXVETHQ         1040
#define DUALPROCSHM_SIZE_K2UQ            EVENT_SIZE_CIRCBUF_KERNEL_TO_USER
#define DUALPROCSHM_SIZE_U2KQ            EVENT_SIZE_CIRCBUF_USER_TO_KERNEL
#define DUALPROCSHM_SIZE_PDO             10240   // This shall be increased to support
                                               // higher PDO size (eg. 1490)

#endif //__DUALPROCSHM_MEM_H__
