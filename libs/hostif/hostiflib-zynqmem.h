/**
********************************************************************************
\file       /libs/hostif/hostiflib-zynqmem

\brief      Hostif interface memory offsets and size 

This module contains of platform specific definitions.
*******************************************************************************/


#ifndef __HOSTIF_MEM_H__
#define __HOSTIF_MEM_H__

/* BASE ADDRESSES */

#define HOSTINTERFACE_0_BASE	0x2C000000


/* VERSION */
#define HOSTIF_VERSION_MAJOR            0
#define HOSTIF_VERSION_MINOR            0
#define HOSTIF_VERSION_REVISION         1
#define HOSTIF_VERSION_COUNT            44

/* BASE */
#define HOSTIF_BASE_DYNBUF0			NULL
#define HOSTIF_BASE_DYNBUF1			NULL
#define HOSTIF_BASE_ERRORCOUNTER	NULL
#define HOSTIF_BASE_TXNMTQ			NULL
#define HOSTIF_BASE_TXGENQ			NULL
#define HOSTIF_BASE_TXSYNCQ			NULL
#define HOSTIF_BASE_TXVETHQ			NULL
#define HOSTIF_BASE_RXVETHQ			NULL
#define HOSTIF_BASE_K2UQ			NULL
#define HOSTIF_BASE_U2KQ			NULL
#define HOSTIF_BASE_TPDO			NULL
#define HOSTIF_BASE_RPDO			NULL

/* SIZE */
#define HOSTIF_SIZE_DYNBUF0 2048
#define HOSTIF_SIZE_DYNBUF1 2048
#define HOSTIF_SIZE_ERRORCOUNTER 3108
#define HOSTIF_SIZE_TXNMTQ 2064
#define HOSTIF_SIZE_TXGENQ 2064
#define HOSTIF_SIZE_TXSYNCQ 2064
#define HOSTIF_SIZE_TXVETHQ 2064
#define HOSTIF_SIZE_RXVETHQ 1040
#define HOSTIF_SIZE_K2UQ 8208
#define HOSTIF_SIZE_U2KQ 8208
#define HOSTIF_SIZE_TPDO 1024
#define HOSTIF_SIZE_RPDO 10240


#define HOSTIF_SC_INFO_OFFS_MAGIC         0x0000U
#define HOSTIF_SC_INFO_OFFS_VERSION       0x0004U
#define HOSTIF_SC_INFO_OFFS_CMD           0x0204U
#endif
