/**
********************************************************************************
\file   lfqueue.c

\brief  This is the implementation of a lock-free queue.

The lock-free queue implementation enables an independent producer- and
consumer-process to access the shared resource without critical sections.
This lock-free queue does not support multiple producer- and consumer-processes!

\ingroup module_hostiflib
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
------------------------------------------------------------------------------*/

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include "lfqueue.h"
#include "event.h" //TODO" CLeanup
//#include <xil_cache.h>

#include <stdlib.h>
#include <string.h>

//TODO clean
#include "dllcal.h"

#include <Benchmark.h> // TODO: Review
//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define ENTRY_MIN_SIZE      4U      ///< UINT32-alignment

#define ALIGN32(ptr)        (((UINT32)(ptr) + 3U) & 0xFFFFFFFCU)
                                    ///< aligns the pointer to UINT32 (4 byte)
#define UNALIGNED32(ptr)    ((UINT32)(ptr) & 3U)
                                    ///< checks if the pointer is UINT32-aligned

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

/**
\brief Queue entry header

The queue entry header enables to identify a valid queue entry and its size.
For future enhancements a reserved UINT32 is available.
Note: This struct has to be UINT32-aligned!
*/
typedef struct sEntryHeader
{
    UINT16          magic;           ///< Magic word to identify an entry
    UINT16          payloadSize;     ///< Provides the entry's payload size
    UINT8           aReserved[4];    ///< reserved
} tEntryHeader;

/**
\brief Queue entry

The queue entry is built out of a header and its payload.
*/
typedef struct sQueueEntry
{
    tEntryHeader    header;           ///< Queue entry header
    UINT8           aPayload[QUEUE_MAX_PAYLOAD]; ///< Entry payload
} tQueueEntry;

/**
\brief Queue index data type

Determines the data type of an index (write and read).
*/
typedef UINT16 tIndex;

/**
\brief Both queue indices data type

Determines the data type of both indices (write and read).
*/
typedef UINT32 tBothIndices;

/**
\brief Queue indices

These are the queue indices
*/
typedef union Indices
{
    struct
    {
        tIndex          write;        ///< write index
        tIndex          read;         ///< read index
    };
    tBothIndices        bothIndices;  ///< combined indices
} tIndices;

/**
\brief Queue indices hw access

These are the queue indices
*/
typedef union uIndicesHw
{
    volatile tIndices       set;      ///< use this for setting individual
    volatile tBothIndices   get;      ///< use this to get indices
    volatile tBothIndices   reset;    ///< use this to reset indices
} tIndicesHw;

typedef enum eQueueState
{
    kQueueStateInvalid      = 0,        ///< queue invalid
    kQueueStateReset        = 1,        ///< queue is in reset
    kQueueStateOperational  = 2,        ///< queue is operational

} tQueueState;

/**
\brief Queue buffer header

This is the header of a queue buffer, with a size of 16 byte.
*/
typedef struct sQueueBufferHdr
{
    volatile UINT8      state;       ///< queue state
    volatile UINT8      aReserved[3];
    tIndicesHw          spaceIndices; ///< gives the offset within the queue
    tIndicesHw          entryIndices; ///< gives the number of entries
    volatile UINT32     reserved;
} tQueueBufferHdr;

/**
\brief Queue buffer header

This structure is shared by the queue consumer and producer.
Note that the data section starting with data must have a span of power 2
(e.g. 1 kB, 2 kB or 4 kB).
*/
typedef struct sQueueBuffer
{
    tQueueBufferHdr     header;   ///< queue buffer header
    volatile UINT8      data;    ///< start of data section
} tQueueBuffer;

/**
\brief Queue instance type

The queue instance type holds the queue configuration, buffer information
and a local copy of the indices.
Note that the members IndicesLocal_m, freeSpace and usedSpace will be
updated to the most current state of the shared queue with the function
getQueueState.
*/
typedef struct sQueue
{
    tQueueConfig    config;           ///< Copy of the queue configuration
    UINT8           *pBase;           ///< Queue base address
    UINT16          span;            ///< Queue span
    struct
    {
        tIndices    spaceIndices;     ///< Local copy of memory space indices
        UINT16      freeSpace;       ///< Local copy of free memory space
        UINT16      usedSpace;       ///< Local copy of used memory space
        tIndices    entryIndices;     ///< Local copy of entry indices
        UINT16      usedEntries;     ///< Local copy of used entries
    } local;
    UINT16          maxEntries;      ///< Maximum addressable entries
    UINT16          queueBufferSpan; ///< queue buffer span
    tQueueBuffer    *pQueueBuffer;    ///< pointer to queue buffer
} tQueue;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void freePtr(void *p);

static void getHwQueueBufferHeader (tQueue *pQueue_p);
static tQueueState getHwQueueState (tQueue *pQueue_p);
static void setHwQueueState (tQueue *pQueue_p, tQueueState State_p);
static void setHwQueueWrite (tQueue *pQueue_p);
static void setHwQueueRead (tQueue *pQueue_p);
static void resetHwQueue (tQueue *pQueue_p);

static UINT16 getOffsetInCirBuffer (tQueue *pQueue_p, UINT16 index_p);

static BOOL checkMagicValid (tEntryHeader *pHeader_p);
static BOOL checkPayloadFitable (tQueue *pQueue_p, UINT16 payloadSize_p);
static BOOL checkQueueEmpty (tQueue *pQueue_p);

static void writeHeader (tQueue *pQueue_p, tEntryHeader *pHeader_p);
static void writeData (tQueue *pQueue_p, UINT8 *pData_p, UINT16 size_p);
static void writeCirMemory (tQueue *pQueue_p, UINT16 offset_p,
        UINT8 *pSrc_p, UINT16 srcSpan_p);

static void readHeader (tQueue *pQueue_p, tEntryHeader *pHeader_p);
static void readData (tQueue *pQueue_p, UINT8 *pData_p, UINT16 size_p);
static void readCirMemory (tQueue *pQueue_p, UINT16 offset_p,
        UINT8 *pDst_p, UINT16 dstSpan_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Create a lock-free queue instance

This function creates a lock-free queue instance, and completely initializes
the queue depending on the pQueueConfig_p parameter. After this function call
the queue is usable.

\param  pQueueConfig_p          The caller provides configuration parameters
                                with this parameter.
\param  ppInstance_p            The function returns with this double-pointer
                                the created instance pointer. (return)

\return tQueueReturn
\retval kQueueSuccessful        The queue is created successfully with the
                                provided configuration parameters
\retval kQueueInvalidParamter   If the parameter pointers are NULL
\retval kQueueAlignment         the allocated buffer queue does not satisfy
                                UINT32-alignment
\retval kQueueNoResource        Either the heap allocation fails or the queue
                                size exceeds the limit QUEUE_MAX_ENTRIES
\retval kQueueHwError           The queue producer is not able to reset the
                                queue indices (e.g. hardware error)

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tQueueReturn lfq_create (tQueueConfig *pQueueConfig_p,
        tQueueInstance *ppInstance_p)
{
    tQueue *pQueue = NULL;

    if(pQueueConfig_p == NULL || ppInstance_p == NULL)
        return kQueueInvalidParameter;

    if(UNALIGNED32(pQueueConfig_p->span - sizeof(tQueueBufferHdr)))
        return kQueueAlignment;

    pQueue = (tQueue*)malloc(sizeof(tQueue));

    if(pQueue == NULL)
        return kQueueNoResource;

    memset(pQueue, 0, sizeof(tQueue));

    /// store configuration
    pQueue->config = *pQueueConfig_p;

    /// allocate memory for queue
    pQueue->span = pQueue->config.span;

    if(pQueue->config.fAllocHeap != FALSE)
        pQueue->pBase = (UINT8*)HOSTIF_UNCACHED_MALLOC(pQueue->span);
    else
        pQueue->pBase = pQueue->config.pBase;

    /// kill instance if allocation is faulty
    if(pQueue->pBase == NULL || UNALIGNED32(pQueue->pBase))
    {
        lfq_delete(pQueue);
        return kQueueNoResource;
    }

    /// set queue buffer
    pQueue->pQueueBuffer = (tQueueBuffer*)pQueue->pBase;
    pQueue->queueBufferSpan = pQueue->span - sizeof(tQueueBufferHdr);

    /// initialize max entries (=mask)
    pQueue->maxEntries = pQueue->queueBufferSpan / ENTRY_MIN_SIZE;

    if(pQueue->maxEntries > QUEUE_MAX_ENTRIES)
    {
        lfq_delete(pQueue);
        return kQueueNoResource;
    }

    /// initialize queue header
    switch(pQueue->config.queueRole)
    {
        case kQueueProducer:
            break;
        case kQueueConsumer:
        case kQueueBoth:
        {
            setHwQueueState(pQueue, kQueueStateReset);

            resetHwQueue(pQueue);

            /// check if initializing was successful (queue must be empty)
            getHwQueueBufferHeader(pQueue);

            if(!checkQueueEmpty(pQueue))
                return kQueueHwError;

            setHwQueueState(pQueue, kQueueStateOperational);
        }
    }

    /// return initialized queue instance
    *ppInstance_p = pQueue;

    return kQueueSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief    Delete a lock-free queue instance

This function deletes a lock-free queue instance. After this function call
the queue must not be used!

\param  pInstance_p             The queue instance that should be deleted

\return tQueueReturn
\retval kQueueSuccessful        The queue instance is deleted successfully
\retval kQueueInvalidParamter   The queue instance is invalid

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tQueueReturn lfq_delete (tQueueInstance pInstance_p)
{
    tQueue *pQueue = (tQueue*)pInstance_p;

    if(pQueue == NULL)
        return kQueueInvalidParameter;

    switch(pQueue->config.queueRole)
    {
        case kQueueProducer:
            break;
        case kQueueConsumer:
        case kQueueBoth:
            setHwQueueState(pQueue, kQueueStateInvalid);
    }

    /// free memory for queue
    if(pQueue->config.fAllocHeap != FALSE)
        HOSTIF_UNCACHED_FREE(pQueue->pBase);

    /// free memory for queue instance
    freePtr(pQueue);

    return kQueueSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief    Resets the queue instance

This function resets the queue of the given instance

\param  pInstance_p             The queue instance that should be reset

\return tQueueReturn
\retval kQueueSuccessful        The returned base address is valid
\retval kQueueInvalidParamter   If the parameter pointers are NULL
\retval kQueueWrongCaller       Consumer is not allowed to reset queue

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tQueueReturn lfq_reset (tQueueInstance pInstance_p)
{
    tQueueReturn Ret = kQueueSuccessful;
    tQueue *pQueue = (tQueue*)pInstance_p;

    if(pQueue == NULL)
    {
        Ret = kQueueInvalidParameter;
        goto Exit;
    }

    if(pQueue->config.queueRole == kQueueProducer)
    {
        Ret = kQueueWrongCaller;
        goto Exit;
    }

    /// signalize reset of the queue
    setHwQueueState(pQueue, kQueueStateReset);

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief    Checks if the queue instance is empty

This function returns the empty state of the queue instance (TRUE = is empty).

\param  pInstance_p             The queue instance of interest
\param  pfIsEmpty_p             Provides the queue empty state

\return tQueueReturn
\retval kQueueSuccessful        The returned base address is valid
\retval kQueueInvalidParamter   If the parameter pointers are NULL

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tQueueReturn lfq_checkEmpty (tQueueInstance pInstance_p,
        BOOL *pfIsEmpty_p)
{
    tQueueReturn Ret = kQueueSuccessful;
    tQueue *pQueue = (tQueue*)pInstance_p;

    if(pQueue == NULL || pfIsEmpty_p == NULL)
    {
        Ret = kQueueInvalidParameter;
        goto Exit;
    }

    switch(getHwQueueState(pQueue))
    {
        case kQueueStateOperational:
            break;
        default:
            /// queue is not operational, hence, empty!
            *pfIsEmpty_p = TRUE;
            goto Exit;
    }

    getHwQueueBufferHeader(pQueue);

    *pfIsEmpty_p = checkQueueEmpty(pQueue);

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief    Obtains the number of entries available in the given queue

This function returns the number of entries in the queue instance.

\param  pInstance_p             The queue instance of interest
\param  pEntryCount_p           Provides the number of added entries

\return tQueueReturn
\retval kQueueSuccessful        The returned base address is valid
\retval kQueueInvalidParamter   If the parameter pointers are NULL

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tQueueReturn lfq_getEntryCount (tQueueInstance pInstance_p,
        UINT16 *pEntryCount_p)
{
    tQueueReturn Ret = kQueueSuccessful;
    tQueue *pQueue = (tQueue*)pInstance_p;

    if(pQueue == NULL || pEntryCount_p == NULL)
    {
        Ret = kQueueInvalidParameter;
        goto Exit;
    }

    getHwQueueBufferHeader(pQueue);

    *pEntryCount_p = pQueue->local.usedEntries;

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief    Enqueue an entry into the queue instance

This function enqueues an entry (pEntry_p) into the given queue instance
(pInstance_p).
Note that the entry which pEntry_p points to has to be initialized with the
payload and its size (magic and reserved are set by this function).

\param  pInstance_p             The queue instance of interest
\param  pData_p                 data to be inserted
\param  size_p                  size of data to be inserted

\return tQueueReturn
\retval kQueueSuccessful        entry is enqueued successfully
\retval kQueueInvalidParamter   If the parameter pointers are NULL
\retval kQueueAlignment         The entry is not UINT32 aligned
\retval kQueueFull              The queue instance is full
\retval kQueueHwError           Queue invalid

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tQueueReturn lfq_entryEnqueue (tQueueInstance pInstance_p,
        UINT8 *pData_p, UINT16 size_p)
{
    tQueue *pQueue = (tQueue*)pInstance_p;
    UINT16 entryPayloadSize;
    tEntryHeader entryHeader;
    //TODO: CLean UP Below



    if(pQueue == NULL || pData_p == NULL || size_p > QUEUE_MAX_PAYLOAD)
        return kQueueInvalidParameter;

    switch(getHwQueueState(pQueue))
    {
        case kQueueStateOperational:
            break;
        case kQueueStateReset:
            /// queue was reset by consumer, producer reactivates queue
            setHwQueueState(pQueue, kQueueStateOperational);
            break;
        default:
        case kQueueStateInvalid:
            return kQueueHwError;
    }

    if(UNALIGNED32(pData_p))
        return kQueueAlignment;

    getHwQueueBufferHeader(pQueue);

#ifdef __AP__
  //  printf("EN AP get Local %x Hif %x B %x\n",pQueue->local.spaceIndices.bothIndices,
    	//	pQueue->pQueueBuffer->header.spaceIndices.get,pQueue->pQueueBuffer);
#elif __PCP__
   // printf("EN PCP get Local %x Hif %x B %x\n",pQueue->local.spaceIndices.bothIndices,
    		//pQueue->pQueueBuffer->header.spaceIndices.get,pQueue->pQueueBuffer);
#endif
    entryPayloadSize = ALIGN32(size_p);

    if(!checkPayloadFitable(pQueue, entryPayloadSize))
        return kQueueFull;

    /// prepare header
    entryHeader.magic = QUEUE_MAGIC;
    entryHeader.payloadSize = entryPayloadSize;
    BENCHMARK_MOD_02_SET(2);
    memset(entryHeader.aReserved, 0, sizeof(entryHeader.aReserved));
    BENCHMARK_MOD_02_RESET(2);
    BENCHMARK_MOD_02_SET(2);
    writeHeader(pQueue, &entryHeader);
    BENCHMARK_MOD_02_RESET(2);
    writeData(pQueue, pData_p, entryPayloadSize);
    /// new element is written
    pQueue->local.entryIndices.write += 1;

    /// the new indices are written to hw only if the queue is still operational
    if(getHwQueueState(pQueue) != kQueueStateOperational)
        return kQueueSuccessful;

    setHwQueueWrite(pQueue);
#ifdef __AP__
   // printf("AP Set Write Local %x Hif %x B %x\n",pQueue->local.spaceIndices.bothIndices,
    	//	pQueue->pQueueBuffer->header.spaceIndices.set,pQueue->pQueueBuffer);
#elif __PCP__
    //printf("PCP Set Write Local %x Hif %x B %x\n",pQueue->local.spaceIndices.bothIndices,
    	//	pQueue->pQueueBuffer->header.spaceIndices.set,pQueue->pQueueBuffer);
#endif
    return kQueueSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief    Dequeue an entry from the queue instance

This function dequeues an entry from the given queue instance to the provided
entry buffer (pEntry_p). The caller has to provide the buffer size, to which the
pointer pEntry_p points to.

\param  pInstance_p             The queue instance of interest
\param  pData_p                 buffer to be used for extracting next entry
\param  pSize_p                 size of the buffer, returns the actual size of
                                the entry

\return tQueueReturn
\retval kQueueSuccessful        The returned base address is valid
\retval kQueueInvalidParamter   If the parameter pointers are NULL
\retval kQueueAlignment         The entry buffer is not UINT32 aligned
\retval kQueueEmpty             The queue instance is empty
\retval kQueueInvalidEntry      The read entry of the queue instance is
                                invalid (magic queue word is wrong!)
\retval kQueueNoResource        The provided entry buffer is too small

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tQueueReturn lfq_entryDequeue (tQueueInstance pInstance_p,
        UINT8 *pData_p, UINT16 *pSize_p)
{
    tQueue *pQueue = (tQueue*)pInstance_p;
    tEntryHeader EntryHeader;
    UINT16 size;

    if(pQueue == NULL || pData_p == NULL)
        return kQueueInvalidParameter;

    /// not operational queues are empty for the consumer
    if(getHwQueueState(pQueue) != kQueueStateOperational)
        return kQueueEmpty;

    if(UNALIGNED32(pData_p))
        return kQueueAlignment;

    getHwQueueBufferHeader(pQueue);
#ifdef __AP__
    //printf("DN AP get Local %x Hif %x B %x\n",pQueue->local.spaceIndices.bothIndices,
    		//pQueue->pQueueBuffer->header.spaceIndices.get,pQueue->pQueueBuffer);
#elif __PCP__
   // printf("DN PCP get Local %x Hif %x B %x\n",pQueue->local.spaceIndices.bothIndices,
    	//	pQueue->pQueueBuffer->header.spaceIndices.get,pQueue->pQueueBuffer);
#endif
    if(checkQueueEmpty(pQueue))
        return kQueueEmpty;
    //Xil_DCacheFlush(); //TODO: @Vinod Cleanup Cache flush
    readHeader(pQueue, &EntryHeader);

    if(!checkMagicValid(&EntryHeader))
        return kQueueInvalidEntry;

    size = ALIGN32(EntryHeader.payloadSize);

    if(size > *pSize_p)
        return kQueueNoResource;

    readData(pQueue, pData_p, size);

    /// element is read
    pQueue->local.entryIndices.read += 1;

    setHwQueueRead(pQueue);
#ifdef __AP__
   // printf("AP Set Read Local %x Hif %x B %x\n",(u32)pQueue->local.spaceIndices.bothIndices,
    	//	pQueue->pQueueBuffer->header.spaceIndices,pQueue->pQueueBuffer);
#elif __PCP__
  //  printf("PCP Set Read Local %x Hif %x B %x\n",(u32)pQueue->local.spaceIndices.bothIndices,
  //  		pQueue->pQueueBuffer->header.spaceIndices,pQueue->pQueueBuffer);
#endif
    /// return entry size
    *pSize_p = size;
    //TODO: Clean Up



    return kQueueSuccessful;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Free pointers which are not NULL

\param  p                       Pointer to be freed
*/
//------------------------------------------------------------------------------
static void freePtr(void *p)
{
    if(p != NULL)
        free(p);
}

//------------------------------------------------------------------------------
/**
\brief    Get queue buffer header from shared memory

This function reads the queue buffer instance header from the shared memory
and writes it to the queue instance.
This ensures reading queue indices consistently.

\param  pQueue_p                The queue instance of interest
*/
//------------------------------------------------------------------------------
static void getHwQueueBufferHeader (tQueue *pQueue_p)
{

    pQueue_p->local.spaceIndices.bothIndices =
            HOSTIF_RD32(pQueue_p->pQueueBuffer,
            offsetof(tQueueBuffer, header.spaceIndices));

    pQueue_p->local.entryIndices.bothIndices =
            HOSTIF_RD32(pQueue_p->pQueueBuffer,
                    offsetof(tQueueBuffer, header.entryIndices));

    pQueue_p->local.usedSpace = pQueue_p->local.spaceIndices.write -
            pQueue_p->local.spaceIndices.read;

    pQueue_p->local.freeSpace =
            pQueue_p->maxEntries -pQueue_p->local.usedSpace;

    pQueue_p->local.usedEntries = pQueue_p->local.entryIndices.write -
            pQueue_p->local.entryIndices.read;

}

//------------------------------------------------------------------------------
/**
\brief    Get queue buffer state from shared memory

\param  pQueue_p                The queue instance of interest

\return The function returns the state of the queue instance.
*/
//------------------------------------------------------------------------------
static tQueueState getHwQueueState (tQueue *pQueue_p)
{
    return (tQueueState)HOSTIF_RD8(pQueue_p->pQueueBuffer,
            offsetof(tQueueBuffer, header.state));
}

//------------------------------------------------------------------------------
/**
\brief    Set queue buffer state to shared memory

\param  pQueue_p                The queue instance of interest
\param  State_p                 Queue state to be written
*/
//------------------------------------------------------------------------------
static void setHwQueueState (tQueue *pQueue_p, tQueueState State_p)
{

    HOSTIF_WR8(pQueue_p->pQueueBuffer,
            offsetof(tQueueBuffer, header.state), (UINT8)State_p);

}

//------------------------------------------------------------------------------
/**
\brief    Set queue buffer write index from local instance to shared memory

This function writes the local write indices to the shared memory.

\param  pQueue_p                The queue instance of interest
*/
//------------------------------------------------------------------------------
static void setHwQueueWrite (tQueue *pQueue_p)
{


    HOSTIF_WR16(pQueue_p->pQueueBuffer,
            offsetof(tQueueBuffer, header.spaceIndices.set.write),
            pQueue_p->local.spaceIndices.write);

    HOSTIF_WR16(pQueue_p->pQueueBuffer,
            offsetof(tQueueBuffer, header.entryIndices.set.write),
            pQueue_p->local.entryIndices.write);

#if (HOSTIF_USE_DCACHE != FALSE)

    hostif_FlushDCacheRange((DWORD)pQueue_p->pQueueBuffer,sizeof(tQueueBufferHdr));

#endif

}

//------------------------------------------------------------------------------
/**
\brief    Set queue buffer read index from local instance to shared memory

This function writes the local read indices to the shared memory.

\param  pQueue_p                The queue instance of interest
*/
//------------------------------------------------------------------------------
static void setHwQueueRead (tQueue *pQueue_p)
{

    HOSTIF_WR16(pQueue_p->pQueueBuffer,
            offsetof(tQueueBuffer, header.spaceIndices.set.read),
            pQueue_p->local.spaceIndices.read);

    HOSTIF_WR16(pQueue_p->pQueueBuffer,
            offsetof(tQueueBuffer, header.entryIndices.set.read),
            pQueue_p->local.entryIndices.read);
#if (HOSTIF_USE_DCACHE != FALSE)

    hostif_FlushDCacheRange((DWORD)pQueue_p->pQueueBuffer, sizeof(tQueueBufferHdr));

#endif

}

//------------------------------------------------------------------------------
/**
\brief    Reset the queue instance

This function resets the queue buffer instance by writing directly to the shared
memory region.

\param  pQueue_p                The queue instance of interest
*/
//------------------------------------------------------------------------------
static void resetHwQueue (tQueue *pQueue_p)
{

    HOSTIF_WR32(pQueue_p->pQueueBuffer, offsetof(tQueueBuffer,
            header.spaceIndices.reset), 0);

    HOSTIF_WR32(pQueue_p->pQueueBuffer, offsetof(tQueueBuffer,
            header.entryIndices.reset), 0);
#if (HOSTIF_USE_DCACHE != FALSE)

    hostif_FlushDCacheRange((DWORD)pQueue_p->pQueueBuffer, sizeof(tQueueBufferHdr));
#endif
}

//------------------------------------------------------------------------------
/**
\brief    Get offset in circular buffer

This function returns the offset of an index of a queue instance.

\param  pQueue_p                The queue instance of interest
\param  index_p                 The index of interest

\return The function returns the offset of the specified index.
*/
//------------------------------------------------------------------------------
static UINT16 getOffsetInCirBuffer (tQueue *pQueue_p, UINT16 index_p)
{
    return (index_p & (pQueue_p->maxEntries - 1)) * ENTRY_MIN_SIZE;
}

//------------------------------------------------------------------------------
/**
\brief    Check if a queue entry has a valid magic word

This function checks the magic word of a queue entry header.

\param  pHeader_p               Reference to the queue entry header.

\return The function returns TRUE if the queue entry header magic is valid.
*/
//------------------------------------------------------------------------------
static BOOL checkMagicValid (tEntryHeader *pHeader_p)
{
    if(pHeader_p->magic != QUEUE_MAGIC)
    {
        return FALSE;
    }
    else
    {
        return TRUE;
    }
}

//------------------------------------------------------------------------------
/**
\brief    Check if a entry will fit into the queue instance

This function checks if the specified payloadSize_p will fit into the queue
instance.

\param  pQueue_p                Queue instance of interest
\param  payloadSize_p           Payload size of the queue entry

\return The function returns TRUE if the entry will fit.
*/
//------------------------------------------------------------------------------
static BOOL checkPayloadFitable (tQueue *pQueue_p, UINT16 payloadSize_p)
{
    UINT16 space = (sizeof(tEntryHeader) + payloadSize_p) / ENTRY_MIN_SIZE;

    if(space > pQueue_p->local.freeSpace)
    {
        return FALSE;
    }
    else
    {
        return TRUE;
    }
}

//------------------------------------------------------------------------------
/**
\brief    Check if the queue instance is empty

Note that the queue empty state is checked with the local copy! In order to get
the most current empty state, the function getHwQueueBufferHeader() must be
called.

\param  pQueue_p                Queue instance of interest

\return The function returns TRUE if the queue instance is empty.
*/
//------------------------------------------------------------------------------
static BOOL checkQueueEmpty (tQueue *pQueue_p)
{
    return (pQueue_p->local.usedSpace == 0);
}

//------------------------------------------------------------------------------
/**
\brief    Write queue entry header to queue instance

\param  pQueue_p                Queue instance of interest
\param  pHeader_p               Reference to header to be written
*/
//------------------------------------------------------------------------------
static void writeHeader (tQueue *pQueue_p, tEntryHeader *pHeader_p)
{

    UINT16 offset = getOffsetInCirBuffer(pQueue_p,
            pQueue_p->local.spaceIndices.write);

    writeCirMemory(pQueue_p, offset, (UINT8*)pHeader_p, sizeof(tEntryHeader));

    pQueue_p->local.spaceIndices.write += sizeof(tEntryHeader) /
            ENTRY_MIN_SIZE;
}

//------------------------------------------------------------------------------
/**
\brief    Write queue entry payload to queue instance

\param  pQueue_p                Queue instance of interest
\param  pData_p                 Payload data to be written
\param  size_p                  Size of payload data to be written
*/
//------------------------------------------------------------------------------
static void writeData (tQueue *pQueue_p, UINT8 *pData_p, UINT16 size_p)
{

    UINT16 offset = getOffsetInCirBuffer(pQueue_p,
            pQueue_p->local.spaceIndices.write);

    writeCirMemory(pQueue_p, offset, pData_p, size_p);


    pQueue_p->local.spaceIndices.write += size_p / ENTRY_MIN_SIZE;
}

//------------------------------------------------------------------------------
/**
\brief    Write data to circular memory

This function writes data from a source to a circular memory.

\param  pQueue_p                Queue instance of interest
\param  offset_p                Offset where to start writing to
\param  pSrc_p                  Source data base address
\param  srcSpan_p               Source data size
*/
//------------------------------------------------------------------------------
static void writeCirMemory (tQueue *pQueue_p, UINT16 offset_p,
        UINT8 *pSrc_p, UINT16 srcSpan_p)
{
	tEplEvent*          pEplEvent; //TODO: clean
    UINT8 *pDst = (UINT8*)(&pQueue_p->pQueueBuffer->data);
    UINT16 part;
    UINT8 flag =0;
#if defined(__AP__)
   // ULONG dataSize = srcSpan_p - sizeof(tEplEvent) ;
   // printf(" En AP:nmt:%x Sink %x Size %d \n",((u32 *)pSrc_p)[0],((u32 *)pSrc_p)[1],srcSpan_p);
#elif defined(__PCP1__)
    //printf("EN PCP:nmt:%x Size %d\n",(u32)pSrc_p[4],srcSpan_p);
#endif

#ifdef __AP1__
    //TODO: Cleanup
    pEplEvent = (tEplEvent*) (pSrc_p);
     if(pEplEvent->m_EventType == kEplEventTypeDllkIssueReq)
     {	tDllCalIssueRequest*	pRequest;

        	pRequest = (tDllCalIssueRequest *)(pDst + offset_p + sizeof(tEplEvent));
        	printf("Eb %x-%x \n", pRequest->nodeId,pRequest);
        	pRequest->nodeId = 0;
        	flag = 1;
     }
#endif
    if(offset_p + srcSpan_p <= pQueue_p->queueBufferSpan)
    {

        memcpy(pDst + offset_p, pSrc_p, srcSpan_p);
#if (HOSTIF_USE_DCACHE != FALSE)

        hostif_FlushDCacheRange((UINT32)(pDst + offset_p), srcSpan_p);

#endif
    }
    else
    {
    	if(flag)
    	    	{
    	    		printf("Copy\n");
    	    	}
        /// mind the circular nature of this buffer!
        part = pQueue_p->queueBufferSpan - offset_p;

        /// copy to the buffer's end
        memcpy(pDst + offset_p, pSrc_p, part);

        /// copy the rest starting at the buffer's head
        memcpy(pDst, pSrc_p + part, srcSpan_p - part);
#if (HOSTIF_USE_DCACHE != FALSE)
        hostif_FlushDCacheRange((UINT32)(pDst + offset_p), part);
        hostif_FlushDCacheRange((UINT32)(pDst), srcSpan_p - part);
#endif
    }
#ifdef __AP1__
    //TODO: Cleanup
    pEplEvent = (tEplEvent*) (pDst + offset_p);
     if(pEplEvent->m_EventType == kEplEventTypeDllkIssueReq)
     {	tDllCalIssueRequest*	pRequest;

        	pRequest = (tDllCalIssueRequest *)(pDst + offset_p + sizeof(tEplEvent));
        	//printf("Ea %x-%x \n", pRequest->nodeId,pRequest);

     }
#endif

#ifdef __AP1__
    //TODO: Cleanup
    pEplEvent = (tEplEvent*) (pDst + offset_p);
     if(pEplEvent->m_EventType == kEplEventTypeDllkIssueReq)
     {	tDllCalIssueRequest*	pRequest;

        	pRequest = (tDllCalIssueRequest *)(pDst + offset_p + sizeof(tEplEvent));
        	printf("E %x-%x \n", pRequest->nodeId,pRequest);

     }
     pEplEvent = (tEplEvent*) (pSrc_p);
          if(pEplEvent->m_EventType == kEplEventTypeDllkIssueReq)
          {	tDllCalIssueRequest*	pRequest;

             	pRequest = (tDllCalIssueRequest *)(pSrc_p + sizeof(tEplEvent));
              // 	printf("ESrc %x-%x \n", pRequest->nodeId,pRequest);

          }
#endif
#if defined(__AP__)
   // ULONG dataSize = srcSpan_p - sizeof(tEplEvent) ;
   // printf("Af AP:nmt:%x sink %x-%x\n",((u32 *)(pDst + offset_p ))[0],((u32 *)(pDst + offset_p ))[1],pQueue_p->pQueueBuffer);
    //printf("Af AP:Write Offset %x b %x\n",(pDst + offset_p),pQueue_p->pQueueBuffer);
#elif defined(__PCP1__)
    //printf("Af PCP:nmt:%x-%x\n",(u32)pDst[offset_p + 4],pQueue_p->pQueueBuffer);
    //printf("Af PCP:Write Offset %x b %x\n",(pDst + offset_p),pQueue_p->pQueueBuffer);
#endif
}

//------------------------------------------------------------------------------
/**
\brief    Read queue entry header from queue instance

\param  pQueue_p                Queue instance of interest
\param  pHeader_p               Reference to data buffer that will be filled
                                with the read header
*/
//------------------------------------------------------------------------------
static void readHeader (tQueue *pQueue_p, tEntryHeader *pHeader_p)
{
    UINT16 offset = getOffsetInCirBuffer(pQueue_p,
            pQueue_p->local.spaceIndices.read);

    readCirMemory(pQueue_p, offset, (UINT8*)pHeader_p, sizeof(tEntryHeader));

    pQueue_p->local.spaceIndices.read += sizeof(tEntryHeader) /
            ENTRY_MIN_SIZE;
}

//------------------------------------------------------------------------------
/**
\brief    Read queue entry payload from queue instance

\param  pQueue_p                Queue instance of interest
\param  pData_p                 Reference to data buffer that will be filled
                                with the read payload
\param  size_p                  Size of data buffer
*/
//------------------------------------------------------------------------------
static void readData (tQueue *pQueue_p, UINT8 *pData_p, UINT16 size_p)
{

    UINT16 offset = getOffsetInCirBuffer(pQueue_p,
            pQueue_p->local.spaceIndices.read);

    readCirMemory(pQueue_p, offset, pData_p, size_p);

    pQueue_p->local.spaceIndices.read += size_p / ENTRY_MIN_SIZE;
}

//------------------------------------------------------------------------------
/**
\brief    Read data from circular memory

This function reads data from a circular memory.

\param  pQueue_p                Queue instance of interest
\param  offset_p                Offset where to start reading from
\param  pDst_p                  Destination data base address
\param  dstSpan_p               Destination data size
*/
//------------------------------------------------------------------------------
static void readCirMemory (tQueue *pQueue_p, UINT16 offset_p,
        UINT8 *pDst_p, UINT16 dstSpan_p)
{
	tEplEvent*          pEplEvent; //TODO : clean
    UINT8 *pSrc = (UINT8*)(&pQueue_p->pQueueBuffer->data);
    UINT16 part;

#if defined(__AP1__)
   // printf(" Bef AP:nmt:%x Size %d \n",*(pSrc + offset_p),dstSpan_p);
#elif defined(__PCP__)
  //  printf("Bef PCP:nmt:%x Sink %x Size %d \n",((u32 *)(pSrc + offset_p))[0],((u32 *)(pSrc + offset_p))[1],dstSpan_p);
#endif

    if(offset_p + dstSpan_p <= pQueue_p->queueBufferSpan)
    {
#if (HOSTIF_USE_DCACHE != FALSE)
       	hostif_InvalidateDCacheRange((UINT32)(pSrc + offset_p), dstSpan_p);
#endif
        memcpy(pDst_p, pSrc + offset_p, dstSpan_p);
    }
    else
    {

        /// mind the circular nature of this buffer!
        part = pQueue_p->queueBufferSpan - offset_p;
#if (HOSTIF_USE_DCACHE != FALSE)

        hostif_InvalidateDCacheRange((UINT32)(pSrc + offset_p), part);
        hostif_InvalidateDCacheRange((UINT32)(pSrc),dstSpan_p - part);

#endif
        /// copy until the buffer's end
        memcpy(pDst_p, pSrc + offset_p, part);

        /// copy the rest starting at the buffer's head
        memcpy(pDst_p + part, pSrc, dstSpan_p - part);
    }

#ifdef __PCP1__
    //TODO: Cleanup
#error
    pEplEvent = (tEplEvent*) (pSrc + offset_p);
     if(pEplEvent->m_EventType == kEplEventTypeDllkIssueReq)
     {	tDllCalIssueRequest*	pRequest;

        	pRequest = (tDllCalIssueRequest *)(tDllCalIssueRequest *)(pSrc + offset_p + sizeof(tEplEvent));;

        	printf("ER %x-%x \n", pRequest->nodeId,pRequest);

     }
     pEplEvent = (tEplEvent*) (pDst_p);
         if(pEplEvent->m_EventType == kEplEventTypeDllkIssueReq)
         {	tDllCalIssueRequest*	pRequest;

            	pRequest = (tDllCalIssueRequest *)(pDst_p + sizeof(tEplEvent));;
            	printf("ERDst %x-%x \n", pRequest->nodeId,pRequest);

         }

#endif
#if defined(__AP1__)
   // printf(" DN AP:nmt:%x-%x \n",(u32 *)pDst_p)[0],pQueue_p->pQueueBuffer);
   //printf("AP Read Offset %x B %x\n",(pSrc + offset_p),pQueue_p->pQueueBuffer);
#elif defined(__PCP__)

      // printf("DN PCP:nmt:%x Sink%x-%x\n",((u32 *)pDst_p)[0],((u32 *)pDst_p)[1],pQueue_p->pQueueBuffer);
       //printf("PCP Read Offset %x B %x\n",(pSrc + offset_p),pQueue_p->pQueueBuffer);
#endif
}
