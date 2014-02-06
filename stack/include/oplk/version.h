/**
********************************************************************************
\file   version.h

\brief  openPOWERLINK version definitions

The file contains definitions describing the openPOWERLINK version.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2013, SYSTEC electronic GmbH
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

#ifndef _INC_oplk_version_H_
#define _INC_oplk_version_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/oplkinc.h>

// NOTE:
// All version macros should contain the same version number. But do not use
// defines instead of the numbers. Because the macro EPL_STRING_VERSION() can not
// convert a define to a string.
//
// Format: maj.min.build
//         maj            = major version
//             min        = minor version (will be set to 0 if major version will be incremented)
//                 build  = current build (will be set to 0 if minor version will be incremented)
//
#define EPL_DEFINED_STACK_VERSION   EPL_STACK_VERSION   (2, 0, 0)
#define EPL_DEFINED_OBJ1018_VERSION EPL_OBJ1018_VERSION (2, 0, 0)
#define EPL_DEFINED_STRING_VERSION  EPL_STRING_VERSION  (2, 0, 0, "pre1")


// -----------------------------------------------------------------------------
#define EPL_PRODUCT_NAME            "EPL V2"
#define EPL_PRODUCT_VERSION         EPL_DEFINED_STRING_VERSION
#define EPL_PRODUCT_MANUFACTURER    "SYS TEC electronic GmbH"

#define EPL_PRODUCT_KEY             "SO-1083"
#define EPL_PRODUCT_DESCRIPTION     "openPOWERLINK Protocol Stack Source"

#endif /* _INC_oplk_version_H_ */
