/**
********************************************************************************
\file   debugstr.c

\brief  Debug String module

This file implements the debug string module. It is used to convert a lot
of openPOWERLINK enumerations and error codes into descriptive strings.

\ingroup module_debugstr
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2010 E. Dumas
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
#include <oplk/oplkinc.h>
#include <oplk/oplk.h>
#include <oplk/nmt.h>

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

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
typedef struct
{
    tNmtState           nmtState;
    char*               sNmtState;
} tNmtStateInfo;

typedef struct
{
    tEplApiEventType    apiEvent;
    char*               sApiEvent;
} tApiEventInfo;

typedef struct
{
    tEplKernel          key;
    char*               sName;
} tRetValInfo;

typedef struct
{
    UINT16              key;
    char*               sName;
} tEmergErrCodeInfo;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

static char* invalidStr_l = "INVALID";

// text strings for POWERLINK events
static char* nmtEventStr_l[] =
{
    "NmtEventNoEvent",              //
    "NmtEventDllMePres",            //
    "NmtEventDllMePresTimeout",     //
    "NmtEventDllMeAsndTimeout",     //
    "NmtEventDllMeSoaSent",         //
    "NmtEventDllMeSocTrig",         //
    "NmtEventDllMeSoaTrig",         //
    "NmtEventDllCeSoc",             //
    "NmtEventDllCePreq",            //
    "NmtEventDllCePres",            //
    "NmtEventDllCeSoa",             //
    "NmtEventDllCeAsnd",            //
    "NmtEventDllCeFrameTimeout",    //
    "0xd",                          // reserved
    "0xe",                          // reserved
    "0xf",                          // reserved
    "NmtEventSwReset",              // NMT_GT1, NMT_GT2, NMT_GT8
    "NmtEventResetNode",            //
    "NmtEventResetCom",             //
    "NmtEventResetConfig",          //
    "NmtEventEnterPreOperational2", //
    "NmtEventEnableReadyToOperate", //
    "NmtEventStartNode",            // NMT_CT7
    "NmtEventStopNode",             //
    "0x18",                         // reserved
    "0x19",                         // reserved
    "0x1A",                         // reserved
    "0x1B",                         // reserved
    "0x1C",                         // reserved
    "0x1D",                         // reserved
    "0x1E",                         // reserved
    "0x1F",                         // reserved
    "NmtEventEnterResetApp",        //
    "NmtEventEnterResetCom",        //
    "NmtEventInternComError",       // NMT_GT6, internal communication error -> enter ResetCommunication
    "NmtEventEnterResetConfig",     //
    "NmtEventEnterCsNotActive",     //
    "NmtEventEnterMsNotActive",     //
    "NmtEventTimerBasicEthernet",   // NMT_CT3; timer triggered state change (NotActive -> BasicEth)
    "NmtEventTimerMsPreOp1",        // enter PreOp1 on MN (NotActive -> MsPreOp1)
    "NmtEventNmtCycleError",        // NMT_CT11, NMT_MT6; error during cycle -> enter PreOp1
    "NmtEventTimerMsPreOp2",        // enter PreOp2 on MN (MsPreOp1 -> MsPreOp2 if NmtEventAllMandatoryCNIdent)
    "NmtEventAllMandatoryCNIdent",  // enter PreOp2 on MN if NmtEventTimerMsPreOp2
    "NmtEventEnterReadyToOperate",  // application ready for the state ReadyToOp
    "NmtEventEnterMsOperational",   // enter Operational on MN
    "NmtEventSwitchOff",            // enter state Off
    "NmtEventCriticalError",        // enter state Off because of critical error
};

// text strings for POWERLINK event sources
static char* eventSourceStr_l[] =
{
    "0",                        // reserved

    // kernelspace modules
    "EventSourceDllk",          // Dllk module
    "EventSourceNmtk",          // Nmtk module
    "EventSourceObdk",          // Obdk module
    "EventSourcePdok",          // Pdok module
    "EventSourceTimerk",        // Timerk module
    "EventSourceEventk",        // Eventk module
    "EventSourceSyncCb",        // sync-Cb
    "EventSourceErrk",          // Error handler module
    "0x09", "0x0a", "0x0b",     // reserved
    "0x0c", "0x0d", "0x0e",     // reserved
    "0x0f",                     // reserved

    // userspace modules
    "EventSourceDllu",          // Dllu module
    "EventSourceNmtu",          // Nmtu module
    "EventSourceNmtCnu",        // NmtCnu module
    "EventSourceNmtMnu",        // NmtMnu module
    "EventSourceObdu",          // Obdu module
    "EventSourceSdoUdp",        // Sdo/Udp module
    "EventSourceSdoAsnd",       // Sdo/Asnd module
    "EventSourceSdoAsySeq",     // Sdo asynchronous Sequence Layer module
    "EventSourceSdoCom",        // Sdo command layer module
    "EventSourceTimeru",        // Timeru module
    "EventSourceCfgMau",        // CfgMau module
    "EventSourceEventu",        // Eventu module
    "EventSourceEplApi",        // Api module
    "EventSourceLedu",          // Ledu module
    "EventSourceGw309Ascii",    // GW309ASCII module
};

// text strings for POWERLINK event sinks
static char* eventSinkStr_l[] =
{
    "EventSinkSync",
    "EventSinkNmtk",
    "EventSinkDllk",
    "EventSinkDlluCal",
    "EventSinkDllkCal",
    "EventSinkPdok",
    "EventSinkNmtu",
    "EventSinkErrk",
    "EventSinkErru",
    "EventSinkSdoAsySeq",
    "EventSinkNmtMnu",
    "EventSinkLedu",
    "EventSinkPdokCal",
    "EventSinkGw309Ascii",
    "EventSinkApi"
};

// text strings for POWERLINK event types
static char* eventTypeStr_l[] =
{
    "0",                                // reserved
    "EventTypeNmtEvent",                // NMT event
    "EventTypePdoRx",                   // PDO frame received event (PRes/PReq)
    "EventTypePdoTx",                   // PDO frame transmitted event (PRes/PReq)
    "EventTypePdoSoa",                  // SoA frame received event (isochronous phase completed)
    "EventTypeSync",                    // Sync event (e.g. SoC or anticipated SoC)
    "EventTypeTimer",                   // Timer event
    "EventTypeHeartbeat",               // Heartbeat event
    "EventTypeHistoryEntry",            // Error history entry event
    "EventTypeDllkFlag1",               // DLL kernel Flag 1 changed event
    "EventTypeDllkFillTx",              // DLL kernel fill TxBuffer event
    "EventTypeDllkPresReady",           // DLL kernel PRes ready event
    "EventTypeError",                   // Error event for API layer
    "EventTypeNmtStateChange",          // indicate change of NMT-State
    "EventTypeDllError",                // DLL error event for Error handler
    "EventTypeAsndRx",                  // received ASnd frame for DLL user module
    "EventTypeDllkServFilter",          // configure ServiceIdFilter
    "EventTypeDllkIdentity",            // configure Identity
    "EventTypeDllkConfig",              // configure ConfigParam
    "EventTypeDllkIssueReq",            // issue Ident/Status request
    "EventTypeDllkAddNode",             // add node to isochronous phase
    "EventTypeDllkDelNode",             // remove node from isochronous phase
    "EventTypeDllkConfigNode",          // configures parameters of node
    "EventTypeDllkStartReducedCycle",   // start reduced EPL cycle on MN
    "EventTypeNmtMnuNmtCmdSent",        // NMT command was actually sent
    "EventTypeApiUserDef",              // user-defined event
    "EventTypeDllkCycleFinish",         // SoA sent, cycle finished
    "0x1B", "0x1C", "0x1D",             // reserved
    "0x1E", "0x1F",                     // reserved
    "EventTypePdokAlloc",               // alloc PDOs
    "EventTypePdokConfig",              // configure PDO channel
    "EventTypeNmtMnuNodeCmd",           // trigger NMT node command
    "EventTypeGw309AsciiReq",           // GW309ASCII request
    "EventTypeNmtMnuNodeAdded",         // node was added to isochronous phase by DLL
    "EventTypePdokSetupPdoBuf",         // dealloc PDOs
    "EventTypePdokControlSync"          // enable/disable the pdokcal sync trigger (arg is pointer to BOOL)
};

// text strings for POWERLINK states
static tNmtStateInfo nmtStateInfo_l[] =
{
    { kNmtGsOff,                 "NmtGsOff"                  },
    { kNmtGsInitialising,        "NmtGsInitializing"         },
    { kNmtGsResetApplication,    "NmtGsResetApplication"     },
    { kNmtGsResetCommunication,  "NmtGsResetCommunication"   },
    { kNmtGsResetConfiguration,  "NmtGsResetConfiguration"   },
    { kNmtCsNotActive,           "NmtCsNotActive"            },
    { kNmtCsPreOperational1,     "NmtCsPreOperational1"      },
    { kNmtCsStopped,             "NmtCsStopped"              },
    { kNmtCsPreOperational2,     "NmtCsPreOperational2"      },
    { kNmtCsReadyToOperate,      "NmtCsReadyToOperate"       },
    { kNmtCsOperational,         "NmtCsOperational"          },
    { kNmtCsBasicEthernet,       "NmtCsBasicEthernet"        },
    { kNmtMsNotActive,           "NmtMsNotActive"            },
    { kNmtMsPreOperational1,     "NmtMsPreOperational1"      },
    { kNmtMsPreOperational2,     "NmtMsPreOperational2"      },
    { kNmtMsReadyToOperate,      "NmtMsReadyToOperate"       },
    { kNmtMsOperational,         "NmtMsOperational"          },
    { kNmtMsBasicEthernet,       "NmtMsBasicEthernet"        },
};

// text strings for API events
static tApiEventInfo apiEventInfo_l[] =
{
    { kEplApiEventUserDef,          "User defined"                      },
    { kEplApiEventNmtStateChange,   "NMT state change"                  },
    { kEplApiEventCriticalError,    "CRITICAL error -> stack halted"    },
    { kEplApiEventWarning,          "Warning"                           },
    { kEplApiEventHistoryEntry,     "History entry"                     },
    { kEplApiEventNode,             "Node event"                        },
    { kEplApiEventBoot,             "Boot event"                        },
    { kEplApiEventSdo,              "SDO event"                         },
    { kEplApiEventObdAccess,        "OBD access"                        },
    { kEplApiEventLed,              "LED event"                         },
    { kEplApiEventCfmProgress,      "CFM progress"                      },
    { kEplApiEventCfmResult,        "CFM result"                        },
};

// text strings for values of type tEplKernel
static tRetValInfo retValInfo_l[] =
{
    /* area for generic errors 0x0000 - 0x000F */
    { kEplSuccessful,               "no error/successful run"},
    { kEplIllegalInstance,          "the called instance does not exist"},
    { kEplInvalidInstanceParam,     "invalid instance parameter"},
    { kEplNoFreeInstance,           "XxxAddInstance was called but no free instance is available"},
    { kEplWrongSignature,           "wrong signature while writing to object 0x1010 or 0x1011"},
    { kEplInvalidOperation,         "operation not allowed in this situation"},
    { kEplInvalidNodeId,            "invalid NodeId was specified"},
    { kEplNoResource,               "resource could not be created (Windows, PxROS, ...)"},
    { kEplShutdown,                 "stack is shutting down"},
    { kEplReject,                   "reject the subsequent command"},
    { kEplRetry,                    "retry this command"},
    { kEplInvalidEvent,             "invalid event was posted to process function"},

    /* area for EDRV module 0x0010 - 0x001F */
    { kEplEdrvNoFreeTxDesc,         "no free Tx descriptor available"},
    { kEplEdrvInvalidCycleLen,      "invalid cycle length (e.g. 0)"},
    { kEplEdrvInitError,            "initialisation error"},
    { kEplEdrvNoFreeBufEntry,       "no free entry in internal buffer table for Tx frames"},
    { kEplEdrvBufNotExisting,       "specified Tx buffer does not exist"},
    { kEplEdrvInvalidRxBuf,         "specified Rx buffer is invalid"},
    { kEplEdrvInvalidParam,         "invalid parameter in function call"},
    { kEplEdrvNextTxListNotEmpty,   "next Tx buffer list is not empty, i.e. still in use"},
    { kEplEdrvCurTxListEmpty,       "current Tx buffer list is empty, i.e. DLL didn't provide one"},
    { kEplEdrvTxListNotFinishedYet, "current Tx buffer list has not been finished yet, but new cycle has started"},


    /* area for DLL module 0x0020 - 0x002F */
    { kEplDllOutOfMemory,           "out of memory"},
    { kEplDllIllegalHdl,            "illegal handle for a TxFrame was passed"},
    { kEplDllCbAsyncRegistered,     "handler for non-EPL frames was already registered before"},
    { kEplDllAsyncSyncReqFull,      "buffer for SyncRequests is full"},
    { kEplDllAsyncTxBufferEmpty,    "transmit buffer for asynchronous frames is empty"},
    { kEplDllAsyncTxBufferFull,     "transmit buffer for asynchronous frames is full"},
    { kEplDllNoNodeInfo,            "MN: too less space in the internal node info structure"},
    { kEplDllInvalidParam,          "invalid parameters passed to function"},
    { kEplDllInvalidAsndServiceId,  "invalid AsndServiceId specified"},
    { kEplDllTxBufNotReady,         "TxBuffer (e.g. for PReq) is not ready yet"},
    { kEplDllTxFrameInvalid,        "TxFrame (e.g. for PReq) is invalid or does not exist"},

    /* area for OBD module 0x0030 - 0x003F */
    { kEplObdIllegalPart,           "unknown OD part"},
    { kEplObdIndexNotExist,         "object index does not exist in OD"},
    { kEplObdSubindexNotExist,      "subindex does not exist in object index"},
    { kEplObdReadViolation,         "read access to a write-only object"},
    { kEplObdWriteViolation,        "write access to a read-only object"},
    { kEplObdAccessViolation,       "access not allowed"},
    { kEplObdUnknownObjectType,     "object type not defined/known"},
    { kEplObdVarEntryNotExist,      "object does not contain VarEntry structure"},
    { kEplObdValueTooLow,           "value to write to an object is too low"},
    { kEplObdValueTooHigh,          "value to write to an object is too high"},
    { kEplObdValueLengthError,      "value to write is to long or to short"},
    { kEplObdErrnoSet,              "file I/O error occurred and errno is set"},
    { kEplObdInvalidDcf,            "device configuration file (CDC) is not valid"},
    { kEplObdOutOfMemory,           "out of memory"},
    { kEplObdNoConfigData,          "no configuration data present (CDC is empty)"},


    /* area for NMT module 0x0040 - 0x004F */
    { kEplNmtUnknownCommand,        "unknown NMT command"},
    { kEplNmtInvalidFramePointer,   "pointer to the frame is not valid"},
    { kEplNmtInvalidEvent,          "invalid event send to NMT-modul"},
    { kEplNmtInvalidState,          "unknown state in NMT-State-Maschine"},
    { kEplNmtInvalidParam,          "invalid parameters specified"},
    { kEplNmtSyncReqRejected,       "SyncReq could not be issued"},

    /* area for SDO/UDP module 0x0050 - 0x005F */
    { kEplSdoUdpMissCb,             "missing callback-function pointer during init of module"},
    { kEplSdoUdpNoSocket,           "error during init of socket"},
    { kEplSdoUdpSocketError,        "error during usage of socket"},
    { kEplSdoUdpThreadError,        "error during start of listen thread"},
    { kEplSdoUdpNoFreeHandle,       "no free connection handle for Udp"},
    { kEplSdoUdpSendError,          "Error during send of frame"},
    { kEplSdoUdpInvalidHdl,         "the connection handle is invalid"},

    /* area for SDO Sequence layer module 0x0060 - 0x006F */
    { kEplSdoSeqMissCb,             "no callback-function assign"},
    { kEplSdoSeqNoFreeHandle,       "no free handle for connection"},
    { kEplSdoSeqInvalidHdl,         "invalid handle in SDO sequence layer"},
    { kEplSdoSeqUnsupportedProt,    "unsupported Protocol selected"},
    { kEplSdoSeqNoFreeHistory,      "no free entry in history"},
    { kEplSdoSeqFrameSizeError,     "the size of the frames is not correct"},
    { kEplSdoSeqRequestAckNeeded,   "indicates that the history buffer is full and a ack request is needed"},
    { kEplSdoSeqInvalidFrame,       "frame not valid"},
    { kEplSdoSeqConnectionBusy,     "connection is busy -> retry later"},
    { kEplSdoSeqInvalidEvent,       "invalid event received"},

    /* area for SDO Command Layer Module 0x0070 - 0x007F */
    { kEplSdoComUnsupportedProt,    "unsupported Protocol selected"},
    { kEplSdoComNoFreeHandle,       "no free handle for connection"},
    { kEplSdoComInvalidServiceType, "invalid SDO service type specified"},
    { kEplSdoComInvalidHandle,      "handle invalid"},
    { kEplSdoComInvalidSendType,    "the stated to of frame to send is not possible"},
    { kEplSdoComNotResponsible,     "internal error: command layer handle is not responsible for this event from sequence layer"},
    { kEplSdoComHandleExists,       "handle to same node already exists"},
    { kEplSdoComHandleBusy,         "transfer via this handle is already running"},
    { kEplSdoComInvalidParam,       "invalid parameters passed to function"},

    /* area for EPL Event-Modul 0x0080 - 0x008F */
    { kEplEventUnknownSink,         "unknown sink for event"},
    { kEplEventPostError,           "error during post of event"},
    { kEplEventReadError,           "error during reading of event from queue"},
    { kEplEventWrongSize,           "event arg has wrong size"},

    /* area for EPL Timer Modul 0x0090 - 0x009F */
    { kEplTimerInvalidHandle,       "invalid handle for timer"},
    { kEplTimerNoTimerCreated,      "no timer was created caused by an error"},
    { kEplTimerThreadError,         "process thread could not be created"},

    /* area for EPL SDO/Asnd Module 0x00A0 - 0x0AF */
    { kEplSdoAsndInvalidNodeId,     "node id is invalid"},
    { kEplSdoAsndNoFreeHandle,      "no free handle for connection"},
    { kEplSdoAsndInvalidHandle,     "handle for connection is invalid"},

    /* area for PDO module 0x00B0 - 0x00BF  */
    { kEplPdoNotExist,              "selected PDO does not exist"},
    { kEplPdoLengthExceeded,        "length of PDO mapping exceeds the current payload limit"},
    { kEplPdoGranularityMismatch,   "configured PDO granularity is not equal to supported granularity"},
    { kEplPdoInitError,             "error during initialisation of PDO module"},
    { kEplPdoConfWhileEnabled,      "PDO configuration cannot be changed while it is enabled"},
    { kEplPdoErrorMapp,             "invalid PDO mapping"},
    { kEplPdoVarNotFound,           "the referenced object in a PDO mapping does not exist"},
    { kEplPdoVarNotMappable,        "the referenced object in a PDO mapping is not mappable"},
    { kEplPdoSizeMismatch,          "bit size of object mapping is larger than the object size"},
    { kEplPdoTooManyTxPdos,         "there exits more than one TPDO on CN"},
    { kEplPdoInvalidObjIndex,       "invalid object index used for PDO mapping or communication parameter"},
    { kEplPdoTooManyPdos,           "there exit to many PDOs"},

    /* Configuration manager module 0x00C0 - 0x00CF */
    { kEplCfmConfigError,           "error in configuration manager"},
    { kEplCfmSdocTimeOutError,      "error in configuration manager, Sdo timeout"},
    { kEplCfmInvalidDcf,            "device configuration file (CDC) is not valid"},
    { kEplCfmUnsupportedDcf,        "unsupported DCF format"},
    { kEplCfmConfigWithErrors,      "configuration finished with errors"},
    { kEplCfmNoFreeConfig,          "no free configuration entry"},
    { kEplCfmNoConfigData,          "no configuration data present"},
    { kEplCfmUnsuppDatatypeDcf,     "unsupported datatype found in dcf -> this entry was not configured"},

    { kEplApiTaskDeferred,          "EPL performs task in background and informs the application (or vice-versa), when it is finished"},
    { kEplApiInvalidParam,          "passed invalid parameters to a function (e.g. invalid node id)"},
    { kEplApiNoObdInitRam,          "no function pointer for ObdInitRam supplied"},
    { kEplApiSdoBusyIntern,         "the SDO channel to this node is internally used by the stack (e.g. the CFM) and currently not available for the application."},
    { kEplApiPIAlreadyAllocated,    "process image is already allocated"},
    { kEplApiPIOutOfMemory,         "process image: out of memory"},
    { kEplApiPISizeExceeded,        "process image: variable linking or copy job exceeds the size of the PI"},
    { kEplApiPINotAllocated,        "process image is not allocated"},
    { kEplApiPIJobQueueFull,        "process image: job queue is full"},
    { kEplApiPIJobQueueEmpty,       "process image: job queue is empty"},
    { kEplApiPIInvalidJobSize,      "process image: invalid job size"},
    { kEplApiPIInvalidPIPointer,    "process image: pointer to application's process image is invalid"},
    { kEplApiPINonBlockingNotSupp,  "process image: non-blocking copy jobs are not supported on this target"},
};

static const tEmergErrCodeInfo   emergErrCodeInfo_l[] =
{
    { EPL_E_NO_ERROR,                    "EPL_E_NO_ERROR"               },

    // 0xFxxx manufacturer specific error codes
    { EPL_E_NMT_NO_IDENT_RES,            "EPL_E_NMT_NO_IDENT_RES"       },
    { EPL_E_NMT_NO_STATUS_RES,           "EPL_E_NMT_NO_STATUS_RES"      },

    // 0x816x HW errors
    { EPL_E_DLL_BAD_PHYS_MODE,           "EPL_E_DLL_BAD_PHYS_MODE"      },
    { EPL_E_DLL_COLLISION,               "EPL_E_DLL_COLLISION"          },
    { EPL_E_DLL_COLLISION_TH,            "EPL_E_DLL_COLLISION_TH"       },
    { EPL_E_DLL_CRC_TH,                  "EPL_E_DLL_CRC_TH "            },
    { EPL_E_DLL_LOSS_OF_LINK,            "EPL_E_DLL_LOSS_OF_LINK",      },
    { EPL_E_DLL_MAC_BUFFER,              "EPL_E_DLL_MAC_BUFFER",        },

    // 0x82xx Protocol errors
    { EPL_E_DLL_ADDRESS_CONFLICT,        "EPL_E_DLL_ADDRESS_CONFLICT"   },
    { EPL_E_DLL_MULTIPLE_MN,             "EPL_E_DLL_MULTIPLE_MN"        },

    // 0x821x Frame size errors
    { EPL_E_PDO_SHORT_RX,                "EPL_E_PDO_SHORT_RX"           },
    { EPL_E_PDO_MAP_VERS,                "EPL_E_PDO_MAP_VERS"           },
    { EPL_E_NMT_ASND_MTU_DIF,            "EPL_E_NMT_ASND_MTU_DIF"       },
    { EPL_E_NMT_ASND_MTU_LIM,            "EPL_E_NMT_ASND_MTU_LIM"       },
    { EPL_E_NMT_ASND_TX_LIM,             "EPL_E_NMT_ASND_TX_LIM"        },

    // 0x823x Timing errors
    { EPL_E_NMT_CYCLE_LEN,               "EPL_E_NMT_CYCLE_LEN"          },
    { EPL_E_DLL_CYCLE_EXCEED,            "EPL_E_DLL_CYCLE_EXCEED"       },
    { EPL_E_DLL_CYCLE_EXCEED_TH,         "EPL_E_DLL_CYCLE_EXCEED_TH"    },
    { EPL_E_NMT_IDLE_LIM,                "EPL_E_NMT_IDLE_LIM"           },
    { EPL_E_DLL_JITTER_TH,               "EPL_E_DLL_JITTER_TH"          },
    { EPL_E_DLL_LATE_PRES_TH,            "EPL_E_DLL_LATE_PRES_TH"       },
    { EPL_E_NMT_PREQ_CN,                 "EPL_E_NMT_PREQ_CN"            },
    { EPL_E_NMT_PREQ_LIM,                "EPL_E_NMT_PREQ_LIM"           },
    { EPL_E_NMT_PRES_CN,                 "EPL_E_NMT_PRES_CN"            },
    { EPL_E_NMT_PRES_RX_LIM,             "EPL_E_NMT_PRES_RX_LIM"        },
    { EPL_E_NMT_PRES_TX_LIM,             "EPL_E_NMT_PRES_TX_LIM"        },

    // 0x824x Frame errors
    { EPL_E_DLL_INVALID_FORMAT,          "EPL_E_DLL_INVALID_FORMAT"     },
    { EPL_E_DLL_LOSS_PREQ_TH,            "EPL_E_DLL_LOSS_PREQ_TH"       },
    { EPL_E_DLL_LOSS_PRES_TH,            "EPL_E_DLL_LOSS_PRES_TH"       },
    { EPL_E_DLL_LOSS_SOA_TH,             "EPL_E_DLL_LOSS_SOA_TH"        },
    { EPL_E_DLL_LOSS_SOC_TH,             "EPL_E_DLL_LOSS_SOC_TH"        },

    // 0x84xx BootUp Errors
    { EPL_E_NMT_BA1,                     "EPL_E_NMT_BA1"                },
    { EPL_E_NMT_BA1_NO_MN_SUPPORT,       "EPL_E_NMT_BA1_NO_MN_SUPPORT"  },
    { EPL_E_NMT_BPO1,                    "EPL_E_NMT_BPO1"               },
    { EPL_E_NMT_BPO1_GET_IDENT,          "EPL_E_NMT_BPO1_GET_IDENT"     },
    { EPL_E_NMT_BPO1_DEVICE_TYPE,        "EPL_E_NMT_BPO1_DEVICE_TYPE"   },
    { EPL_E_NMT_BPO1_VENDOR_ID,          "EPL_E_NMT_BPO1_VENDOR_ID"     },
    { EPL_E_NMT_BPO1_PRODUCT_CODE,       "EPL_E_NMT_BPO1_PRODUCT_CODE"  },
    { EPL_E_NMT_BPO1_REVISION_NO,        "EPL_E_NMT_BPO1_REVISION_NO"   },
    { EPL_E_NMT_BPO1_SERIAL_NO,          "EPL_E_NMT_BPO1_SERIAL_NO"     },
    { EPL_E_NMT_BPO1_CF_VERIFY,          "EPL_E_NMT_BPO1_CF_VERIFY"     },
    { EPL_E_NMT_BPO2,                    "EPL_E_NMT_BPO2"               },
    { EPL_E_NMT_BRO,                     "EPL_E_NMT_BRO"                },
    { EPL_E_NMT_WRONG_STATE,             "EPL_E_NMT_WRONG_STATE"        },
};

// text strings for NMT node events
static char *EplNmtNodeEvtTypeStr_g[] =
{
    "Found",                    // 0x00
    "Update software",          // 0x01
    "Check configuration",      // 0x02
    "Update configuration",     // 0x03
    "Verify configuration",     // 0x04
    "Ready to start",           // 0x05
    "NMT state",                // 0x06
    "NMT error",                // 0x07
};

// text strings for NMT boot events
static char *EplNmtBootEvtTypeStr_g[] =
{
    "Boot step 1 finished",     // 0x00     PreOp2 is possible
    "Boot step 2 finished",     // 0x01     ReadyToOp is possible for MN
    "EnableReadyToOp",          // 0x02     ReadyToOP is possible for CN
    "CheckComFinish",           // 0x03     Operational is possible
    "Operational",              // 0x04     all mandatory CNs are Operational
    "Error",                    // 0x05
};

// text strings for SDO command layer connection states
static char *EplSdoComConStateStr_g[] =
{
    "NotActive",                // 0x00
    "Running",                  // 0x01
    "TxAborted",                // 0x02
    "RxAborted",                // 0x03
    "Finished",                 // 0x04
    "LowerLayerAbort",          // 0x05
};


//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Return the string of the specified event

The function returns the string describing the specified event.

\param  nmtEvent_p          Event to print

\return The function returns a string describing the specified event.

\ingroup module_debugstr
*/
//------------------------------------------------------------------------------
char* debugstr_getNmtEventStr(tNmtEvent nmtEvent_p)
{
    if (nmtEvent_p >= tabentries(nmtEventStr_l))
    {
        return invalidStr_l;
    }
    else
    {
        return nmtEventStr_l[nmtEvent_p];
    }
}

//------------------------------------------------------------------------------
/**
\brief  Return the string of the specified event source

The function returns the string describing the specified event source.

\param  eventSrc_p          Event source to print

\return The function returns a string describing the specified event source.

\ingroup module_debugstr
*/
//------------------------------------------------------------------------------
char* debugstr_getEventSourceStr(tEplEventSource eventSrc_p)
{
    if (eventSrc_p >= tabentries(eventSourceStr_l))
    {
        return invalidStr_l;
    }
    else
    {
        return eventSourceStr_l[eventSrc_p];
    }
}

//------------------------------------------------------------------------------
/**
\brief  Return the string of the specified event sink

The function returns the string describing the specified event sink.

\param  eventSink_p         Event sink to print

\return The function returns a string describing the specified event sink.

\ingroup module_debugstr
*/
//------------------------------------------------------------------------------
char* debugstr_getEventSinkStr(tEplEventSink eventSink_p)
{
    if (eventSink_p >= tabentries(eventSinkStr_l))
    {
        return invalidStr_l;
    }
    else
    {
        return eventSinkStr_l[eventSink_p];
    }
}

//------------------------------------------------------------------------------
/**
\brief  Return the string of the specified event type

The function returns the string describing the specified event type.

\param  eventType_p         Event type to print

\return The function returns a string describing the specified event type.

\ingroup module_debug
*/
//------------------------------------------------------------------------------
char* debugstr_getEventTypeStr(tEplEventType eventType_p)
{
    if (eventType_p >= tabentries(eventTypeStr_l))
    {
        return invalidStr_l;
    }
    else
    {
        return eventTypeStr_l[eventType_p];
    }
}

//------------------------------------------------------------------------------
/**
\brief  Return the string of the specified NMT state

The function returns the string describing the specified NMT state.

\param  nmtState_p         NMT state to print

\return The function returns a string describing the specified NMT state.

\ingroup module_debugstr
*/
//------------------------------------------------------------------------------
char* debugstr_getNmtStateStr(tNmtState nmtState_p)
{
    unsigned int         i;

    for (i = 0; i < tabentries(nmtStateInfo_l); i++)
    {
        if (nmtStateInfo_l[i].nmtState == nmtState_p)
            return (nmtStateInfo_l[i].sNmtState);
    }
    return invalidStr_l;
}

//------------------------------------------------------------------------------
/**
\brief  Return the string of the specified API event

The function returns the string describing the specified API event.

\param  ApiEvent_p         API event to print

\return The function returns a string describing the specified API event.

\ingroup module_debugstr
*/
//------------------------------------------------------------------------------
char* debugstr_getApiEventStr(tEplApiEventType ApiEvent_p)
{
    UINT        i;

    for (i = 0; i < tabentries(apiEventInfo_l); i++)
    {
        if (apiEventInfo_l[i].apiEvent == ApiEvent_p)
            return (apiEventInfo_l[i].sApiEvent);
    }
    return invalidStr_l;
}

//------------------------------------------------------------------------------
/**
\brief  Return the string of the specified NMT node event

The function returns the string describing the specified NMT node event.

\param  NodeEventType_p         NMT node event to print

\return The function returns a string describing the specified NMT node event.

\ingroup module_debugstr
*/
//------------------------------------------------------------------------------
char* debugstr_getNmtNodeEventTypeStr(tNmtNodeEvent NodeEventType_p )
{
    if( NodeEventType_p >= tabentries(EplNmtNodeEvtTypeStr_g) )
    {
        return  invalidStr_l;
    }
    else
    {
        return  EplNmtNodeEvtTypeStr_g[ NodeEventType_p ];
    }
}

//------------------------------------------------------------------------------
/**
\brief  Return the string of the specified NMT boot event

The function returns the string describing the specified NMT boot event.

\param  BootEventType_p         NMT boot event to print

\return The function returns a string describing the specified NMT boot event.

\ingroup module_debugstr
*/
//------------------------------------------------------------------------------
char* debugstr_getNmtBootEventTypeStr(tNmtBootEvent BootEventType_p )
{
    if( BootEventType_p >= tabentries(EplNmtBootEvtTypeStr_g) )
    {
        return  invalidStr_l;
    }
    else
    {
        return  EplNmtBootEvtTypeStr_g[ BootEventType_p ];
    }
}

//------------------------------------------------------------------------------
/**
\brief  Return the string of the specified SDO command connection state

The function returns the string describing the specified SDO command connection
state.

\param  SdoComConState_p         SDO command connection state to print

\return The function returns a string describing the specified SDO command
connection state.

\ingroup module_debugstr
*/
//------------------------------------------------------------------------------
char* debugstr_getSdoComConStateStr(tSdoComConState SdoComConState_p)
{
    if( SdoComConState_p >= tabentries(EplSdoComConStateStr_g) )
    {
        return  invalidStr_l;
    }
    else
    {
        return  EplSdoComConStateStr_g[ SdoComConState_p ];
    }
}

//------------------------------------------------------------------------------
/**
\brief  Return the string of the specified SDO command connection state

The function returns the string describing the given entry of type tEplKernel.

\param  EplKernel_p         tEplKernel value to print

\return The function returns a string describing the specified tEplKernel type.

\ingroup module_debugstr
*/
//------------------------------------------------------------------------------
char* debugstr_getRetValStr(tEplKernel EplKernel_p)
{
    UINT        i;

    for (i = 0; i < tabentries(retValInfo_l); i++)
    {
        if (retValInfo_l[i].key == EplKernel_p)
            return (retValInfo_l[i].sName);
    }
    return invalidStr_l;
}

//------------------------------------------------------------------------------
/**
\brief  Return the string describing the specified emergency error code

The function returns the string describing the specified emergency error code.

\param  EmergErrCode_p       emergency error code value to print

\return The function returns a string describing the specified emergency error
code.

\ingroup module_debugstr
*/
//------------------------------------------------------------------------------
char* debugstr_getEmergErrCodeStr(UINT16 emergErrCode_p)
{

    UINT        i;

    for (i = 0; i < tabentries(emergErrCodeInfo_l); i++)
    {
        if (emergErrCodeInfo_l[i].key == emergErrCode_p)
            return (emergErrCodeInfo_l[i].sName);
    }
    return invalidStr_l;
}

