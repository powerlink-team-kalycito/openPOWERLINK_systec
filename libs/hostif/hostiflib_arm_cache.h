/*
 * hostiflib_arm_cache.h
 *
 *  Created on: May 14, 2013
 *      Author: John
 */

#ifndef HOSTIFLIB_ARM_CACHE_H_
#define HOSTIFLIB_CACHE_H_

void xil_uncached_free (volatile void*);
volatile void* xil_uncached_malloc (size_t);

#endif /* ARM_CACHE_H_ */
