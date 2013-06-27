/*
 * hostiflib_arm_cache.c
 *
 *  Created on: May 14, 2013
 *      Author: John
 */

#include "hostiflib_arm.h"




/*
 * Allocate a block of uncached memory.
 */

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
