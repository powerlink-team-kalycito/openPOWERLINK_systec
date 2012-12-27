/****************************************************************************

    Copyright (c) 2012 Kalycito Infotech Private Limited

    Project: openPOWERLINK

    Description: Zynq's Triple Timer(TTC1)Module driver to support High Resolution 
                 timer.

    License:

        Redistribution and use in source and binary forms, with or without
        modification, are permitted provided that the following conditions
        are met:

        1. Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.

        2. Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.

        3. Neither the name of Kalycito Infotech nor the names of its
        contributors may be used to endorse or promote products derived
        from this software without prior written permission. For written
        permission, please contact info@kalycito.com.

        THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
        "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
        LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
        FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
        COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
        INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
        BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
        LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
        CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
        LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
        ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
        POSSIBILITY OF SUCH DAMAGE.

    Severability Clause:

    If a provision of this License is or becomes illegal, invalid or
    unenforceable in any jurisdiction, that shall not affect:
    
        1. The validity or enforceability in that jurisdiction of any other
        provision of this License; or
        2. The validity or enforceability in other jurisdictions of that or
        any other provision of this License.

    Based on
        1. TimerHighreskX86.c, TimerHighReskAT91.c and mach-zynq/timer.c
****************************************************************************/
#include "EplInc.h"
#include "kernel/EplTimerHighResk.h"
#include "Benchmark.h"

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/major.h>
#include <linux/version.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/irq.h>
#include <linux/interrupt.h>

/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          G L O B A L   D E F I N I T I O N S                            */
/*                                                                         */
/*                                                                         */
/***************************************************************************/

//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------
#define TIMER_COUNT               2               /* max 2 timers selectable */
#define TIMER_MIN_VAL_SINGLE      5000ULL         /* min 5us */
#define TIMER_MIN_VAL_CYCLE       100000ULL       /* min 100us */

#define TTC_RESOLUTION_FACTOR     144

#define XTTC_BASE                 0xf8002000
#define SIZE                      0x00001000
#define XTTC1_TIMERBASE           0x00000000
#define XTTC2_TIMERBASE           0x04
#define XTTC_IRQ                  69
#define XTTC1                     0
#define XTTC2                     1

/*
 * Timer Register Offset Definitions Timer 
 */
#define XTTCPSS_CLK_CNTRL_OFFSET          0x00      /* Clock Control Reg, RW */
#define XTTCPSS_CNT_CNTRL_OFFSET          0x0C      /* Counter Control Reg, RW */
#define XTTCPSS_COUNT_VAL_OFFSET          0x18      /* Counter Value Reg, RO */
#define XTTCPSS_INTR_VAL_OFFSET           0x24      /* Interval Count Reg, RW */
#define XTTCPSS_MATCH_1_OFFSET            0x30      /* Match 1 Value Reg, RW */
#define XTTCPSS_MATCH_2_OFFSET            0x3C      /* Match 2 Value Reg, RW */
#define XTTCPSS_MATCH_3_OFFSET            0x48      /* Match 3 Value Reg, RW */
#define XTTCPSS_ISR_OFFSET                0x54      /* Interrupt Status Reg, RO */
#define XTTCPSS_IER_OFFSET                0x60      /* Interrupt Enable Reg, RW */

#define XTTCPSS_INTR_MATCH_1              0x02
#define XTTCPSS_INTR_MATCH_2              0x04
#define XTTCPSS_INTR_INTERVAL             0x01
#define XTTCPSS_CLEAR                     0x0000
#define XTTCPSS_CNT_CNTRL_EN_WAVE         0x20
#define XTTCPSS_CNT_CNTRL_RST             0x10
#define XTTCPSS_CNT_CNTRL_MATCH           0x08
#define XTTCPSS_CNT_CNTRL_DECR            0x04
#define XTTCPSS_CNT_CNTRL_INTERVAL        0x02
#define XTTCPSS_CNT_CNTRL_DISABLE         0x01
#define XTTCPSS_CNT_CNTRL_DISABLE_MASK    0x1

/* Setup the timers to use pre-scaling, using a fixed value for now that will work
 * across most input frequency, but it may need to be more dynamic
 */
#define PRESCALE_EXPONENT                 4        /* 2 ^ PRESCALE_EXPONENT = PRESCALE */
#define PRESCALE                          16   /* The exponent must match this */
#define CLK_CNTRL_PRESCALE             (((PRESCALE_EXPONENT - 1) << 1) | 0x1)

#define USEC_TO_COUNT(ullTimeout)      ((unsigned int)ullTimeout/TTC_RESOLUTION_FACTOR)

#define XTTCPSS_READ_REG(uiIndex,dwOffset)            __raw_readl(pXttc_base_addr_g[uiIndex] + dwOffset)

#define XTTCPSS_WRITE_REG(uiIndex,dwOffset, wVal)    __raw_writel(wVal,(pXttc_base_addr_g[uiIndex] + dwOffset))

#define TIMERHDL_MASK               0x0FFFFFFF
#define TIMERHDL_SHIFT              28
#define HDL_TO_IDX(Hdl)             ((Hdl >> TIMERHDL_SHIFT) - 1)
#define HDL_INIT(Idx)               ((Idx + 1) << TIMERHDL_SHIFT)
#define HDL_INC(Hdl)                (((Hdl + 1) & TIMERHDL_MASK) \
                                    | (Hdl & ~TIMERHDL_MASK))

//---------------------------------------------------------------------------
// module global types
//---------------------------------------------------------------------------

typedef struct
{
    tEplTimerEventArg   m_EventArg;
    tEplTimerkCallback  m_pfnCallback;
    UINT                m_uiIndex;
    BOOL                m_fContinuously;
} tEplTimerHighReskTimerInfo;

typedef struct
{
    tEplTimerHighReskTimerInfo  m_aTimerInfo[TIMER_COUNT];
    void*                       m_pIoAddr;      // pointer to register space of Timer / Counter unit
} tEplTimerHighReskInstance;
//---------------------------------------------------------------------------
// local vars
//---------------------------------------------------------------------------

static tEplTimerHighReskInstance EplTimerHighReskInstance_l;
void __iomem *pXttc_base_addr_g[TIMER_COUNT];

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------

static irqreturn_t TgtTimerCounterIsr(int nIrqNum_p, void* pDevInstData_p);

//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:    EplTimerHighReskInit()
//
// Description: initializes the high resolution timer module.
//
// Parameters:  void
//
// Return:      tEplKernel      = error code
//
// State:       not tested
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTimerHighReskInit(void)
{
    tEplKernel Ret;

    Ret = EplTimerHighReskAddInstance();

    return Ret;

}

//---------------------------------------------------------------------------
//
// Function:    EplTimerHighReskAddInstance()
//
// Description: initializes the high resolution timer module.
//
// Parameters:  void
//
// Return:      tEplKernel      = error code
//
// State:       not tested
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTimerHighReskAddInstance(void)
{
    tEplKernel                  Ret = kEplSuccessful;
    INT                         iResult = 0;
    UINT                        uiIndex;
    tEplTimerHighReskTimerInfo* pTimerInfo;
    BYTE                        bReg;
    INT                         iIrq;
    EPL_MEMSET(&EplTimerHighReskInstance_l, 0,
            sizeof(EplTimerHighReskInstance_l));

    EplTimerHighReskInstance_l.m_pIoAddr = ioremap(XTTC_BASE, SIZE);
    if (NULL == EplTimerHighReskInstance_l.m_pIoAddr)
    {
        Ret = kEplNoResource;
        goto Exit;
    }
    PRINTF("%s: IoAddr=%p\n", __func__, EplTimerHighReskInstance_l.m_pIoAddr);

    pXttc_base_addr_g[XTTC1] = (void *) (EplTimerHighReskInstance_l.m_pIoAddr
            + XTTC1_TIMERBASE);
    pXttc_base_addr_g[XTTC2] = (void *) (EplTimerHighReskInstance_l.m_pIoAddr
            + XTTC2_TIMERBASE);

    pTimerInfo = &EplTimerHighReskInstance_l.m_aTimerInfo[0];

    for (uiIndex = 0; uiIndex < TIMER_COUNT; uiIndex++, pTimerInfo++)
    {

        pTimerInfo->m_uiIndex = uiIndex;
        iIrq = XTTC_IRQ + uiIndex;
        iResult = request_irq(iIrq, TgtTimerCounterIsr, IRQF_DISABLED, "Zynq",
                pTimerInfo);
        PRINTF("%s: interrupt%d registered (return=%d)\n", __func__,
                pTimerInfo->m_uiIndex, iResult);

        if (iResult != 0)
        {
            iounmap(EplTimerHighReskInstance_l.m_pIoAddr);
            Ret = kEplNoResource;
            goto Exit;
        }

        bReg = 0;
        bReg = XTTCPSS_READ_REG(uiIndex,XTTCPSS_CNT_CNTRL_OFFSET);
        bReg = XTTCPSS_CNT_CNTRL_DISABLE;
        XTTCPSS_WRITE_REG(uiIndex, XTTCPSS_CNT_CNTRL_OFFSET, bReg);

        bReg = XTTCPSS_READ_REG(uiIndex,XTTCPSS_CLK_CNTRL_OFFSET);
        bReg = CLK_CNTRL_PRESCALE;
        XTTCPSS_WRITE_REG(uiIndex, XTTCPSS_CLK_CNTRL_OFFSET, bReg);

        // set the necessary counter control param , for now we keep the timer disabled
        bReg = 0;
        bReg = XTTCPSS_READ_REG(uiIndex,XTTCPSS_CNT_CNTRL_OFFSET);
        bReg |= (XTTCPSS_CNT_CNTRL_EN_WAVE);
        XTTCPSS_WRITE_REG(uiIndex, XTTCPSS_CNT_CNTRL_OFFSET, bReg);

        // clear all interrupts
        bReg = 0;
        XTTCPSS_WRITE_REG(uiIndex, XTTCPSS_IER_OFFSET, bReg);
    }

    Exit: return Ret;
}

//---------------------------------------------------------------------------
//
// Function:    EplTimerHighReskDelInstance()
//
// Description: shuts down the high resolution timer module.
//
// Parameters:  void
//
// Return:      tEplKernel      = error code
//
// State:       not tested
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTimerHighReskDelInstance(void)
{
    tEplKernel                  Ret = kEplSuccessful;
    BYTE                        bReg;
    UINT                        uiIndex;
    tEplTimerHighReskTimerInfo* pTimerInfo;
    INT                         iIrq;

    pTimerInfo = &EplTimerHighReskInstance_l.m_aTimerInfo[0];
    for (uiIndex = 0; uiIndex < TIMER_COUNT; uiIndex++, pTimerInfo++)
    {
        bReg = 0;
        bReg = XTTCPSS_READ_REG(pTimerInfo->m_uiIndex,XTTCPSS_CNT_CNTRL_OFFSET);
        // set the necessary counter control param , for now we keep the timer disabled
        bReg = (XTTCPSS_CNT_CNTRL_DISABLE);
        XTTCPSS_WRITE_REG(pTimerInfo->m_uiIndex, XTTCPSS_CNT_CNTRL_OFFSET,
                bReg);
        iIrq = XTTC_IRQ + pTimerInfo->m_uiIndex;
        free_irq(iIrq, pTimerInfo);
    }

    iounmap(EplTimerHighReskInstance_l.m_pIoAddr);

    return Ret;
}

//---------------------------------------------------------------------------
//
// Function:    EplTimerHighReskModifyTimerNs()
//
// Description: modifies the timeout of the timer with the specified handle.
//              If the handle the pointer points to is zero, the timer must
//              be created first.
//              If it is not possible to stop the old timer,
//              this function always assures that the old timer does not
//              trigger the callback function with the same handle as the new
//              timer. That means the callback function must check the passed
//              handle with the one returned by this function. If these are
//              unequal, the call can be discarded.
//
// Parameters:  pTimerHdl_p     = pointer to timer handle
//              ullTimeNs_p     = relative timeout in [ns]
//              pfnCallback_p   = callback function, which is called mutual
//                                exclusive with the Edrv callback functions
//                                (Rx and Tx).
//              ulArgument_p    = user-specific argument
//              fContinuously_p = if TRUE, callback function will be called
//                                continuously;
//                                otherwise, it is a oneshot timer.
//
// Return:      tEplKernel      = error code
//
// State:       not tested
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTimerHighReskModifyTimerNs(tEplTimerHdl* pTimerHdl_p,
        unsigned long long ullTimeNs_p, tEplTimerkCallback pfnCallback_p,
        unsigned long ulArgument_p, BOOL fContinuously_p)
{
    tEplKernel                  Ret = kEplSuccessful;
    tEplTimerHighReskTimerInfo* pTimerInfo;
    WORD                        wCounter = 0;
    UINT                        uiIndex;
    BYTE                        bReg;
    WORD                        wCurCntr = 0;
    // check pointer to handle
    if (pTimerHdl_p == NULL)
    {
        Ret = kEplTimerInvalidHandle;
        goto Exit;
    }

    if (*pTimerHdl_p == 0)
    {     // no timer created yet
          // search free timer info structure
        pTimerInfo = &EplTimerHighReskInstance_l.m_aTimerInfo[0];
        for (uiIndex = 0; uiIndex < TIMER_COUNT; uiIndex++, pTimerInfo++)
        {
            if (pTimerInfo->m_EventArg.m_TimerHdl == 0)
            {     // free structure found
                break;
            }
        }
        if (uiIndex >= TIMER_COUNT)
        {     // no free structure found
            Ret = kEplTimerNoTimerCreated;
            goto Exit;
        }
        pTimerInfo->m_EventArg.m_TimerHdl = HDL_INIT(uiIndex);
    }
    else
    {
        uiIndex = HDL_TO_IDX(*pTimerHdl_p);
        ;
        if (uiIndex >= TIMER_COUNT)
        {     // invalid handle
            Ret = kEplTimerInvalidHandle;
            goto Exit;
        }
        pTimerInfo = &EplTimerHighReskInstance_l.m_aTimerInfo[uiIndex];
    }

    // increment timer handle (if timer expires right after this statement,
    // the user would detect an unknown timer handle and discard it)
    pTimerInfo->m_EventArg.m_TimerHdl =
            HDL_INC(pTimerInfo->m_EventArg.m_TimerHdl);
    *pTimerHdl_p = pTimerInfo->m_EventArg.m_TimerHdl;

    // Adjust the Timeout if its to small
    if (fContinuously_p != FALSE)
    {
        if (ullTimeNs_p < TIMER_MIN_VAL_CYCLE)
        {
            ullTimeNs_p = TIMER_MIN_VAL_CYCLE;
        }
    }
    else
    {
        if (ullTimeNs_p < TIMER_MIN_VAL_SINGLE)
        {
            ullTimeNs_p = TIMER_MIN_VAL_SINGLE;
        }
    }

    // Get the counter value from the timeout  
    wCounter = (WORD)USEC_TO_COUNT(ullTimeNs_p);
    if (wCounter > 0xFFFF)
    {
        Ret = kEplTimerNoTimerCreated;
        goto Exit;
    }
    pTimerInfo->m_EventArg.m_Arg.m_dwVal = ulArgument_p;
    pTimerInfo->m_pfnCallback = pfnCallback_p;

    // disable the Timer 
    bReg = 0;
    bReg = XTTCPSS_READ_REG(pTimerInfo->m_uiIndex,XTTCPSS_CNT_CNTRL_OFFSET);
    bReg |= XTTCPSS_CNT_CNTRL_DISABLE;
    XTTCPSS_WRITE_REG(pTimerInfo->m_uiIndex, XTTCPSS_CNT_CNTRL_OFFSET, bReg);

    if (fContinuously_p != FALSE)
    {

        pTimerInfo->m_fContinuously = fContinuously_p;
        // Set the interval for continuos timer 
        XTTCPSS_WRITE_REG(pTimerInfo->m_uiIndex, XTTCPSS_INTR_VAL_OFFSET,
                wCounter);

        // Enable the interval interrupt
        bReg = XTTCPSS_READ_REG(pTimerInfo->m_uiIndex,XTTCPSS_IER_OFFSET);
        bReg = XTTCPSS_INTR_INTERVAL;
        XTTCPSS_WRITE_REG(pTimerInfo->m_uiIndex, XTTCPSS_IER_OFFSET, bReg);

        // Set specific values in Counter control reg
        bReg = XTTCPSS_READ_REG(pTimerInfo->m_uiIndex,XTTCPSS_CNT_CNTRL_OFFSET);
        bReg |= (XTTCPSS_CNT_CNTRL_RST | XTTCPSS_CNT_CNTRL_INTERVAL);
        XTTCPSS_WRITE_REG(pTimerInfo->m_uiIndex, XTTCPSS_CNT_CNTRL_OFFSET,
                bReg);

    }
    else
    {
        pTimerInfo->m_fContinuously = fContinuously_p;

        // Set match counter for oneshot timer
        XTTCPSS_WRITE_REG(pTimerInfo->m_uiIndex, XTTCPSS_MATCH_1_OFFSET,
                wCounter);
        // Enable the Match1 interrupt
        bReg = XTTCPSS_READ_REG(pTimerInfo->m_uiIndex,XTTCPSS_IER_OFFSET);
        bReg = XTTCPSS_INTR_MATCH_1;
        XTTCPSS_WRITE_REG(pTimerInfo->m_uiIndex, XTTCPSS_IER_OFFSET, bReg);

        // Set specific values in Counter control reg
        bReg = XTTCPSS_READ_REG(pTimerInfo->m_uiIndex,XTTCPSS_CNT_CNTRL_OFFSET);
        bReg |= (XTTCPSS_CNT_CNTRL_MATCH | XTTCPSS_CNT_CNTRL_RST);
        XTTCPSS_WRITE_REG(pTimerInfo->m_uiIndex, XTTCPSS_CNT_CNTRL_OFFSET,
                bReg);
    }

    // Re-enable the timer
    bReg = XTTCPSS_READ_REG(pTimerInfo->m_uiIndex,XTTCPSS_CNT_CNTRL_OFFSET);
    bReg &= ~XTTCPSS_CNT_CNTRL_DISABLE;
    XTTCPSS_WRITE_REG(pTimerInfo->m_uiIndex, XTTCPSS_CNT_CNTRL_OFFSET, bReg);

    Exit: return Ret;
}

//---------------------------------------------------------------------------
//
// Function:    EplTimerHighReskDeleteTimer()
//
// Description: deletes the timer with the specified handle. Afterward the
//              handle is set to zero.
//
// Parameters:  pTimerHdl_p     = pointer to timer handle
//
// Return:      tEplKernel      = error code
//
// State:       not tested
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTimerHighReskDeleteTimer(tEplTimerHdl* pTimerHdl_p)
{
    tEplKernel                  Ret = kEplSuccessful;
    UINT                        uiIndex;
    tEplTimerHighReskTimerInfo* pTimerInfo;
    BYTE                        bReg;
    // check pointer to handle
    if (pTimerHdl_p == NULL)
    {
        Ret = kEplTimerInvalidHandle;
        goto Exit;
    }

    if (*pTimerHdl_p == 0)
    {     // no timer created yet
        goto Exit;
    }
    else
    {
        uiIndex = HDL_TO_IDX(*pTimerHdl_p);
        if (uiIndex >= TIMER_COUNT)
        {     // invalid handle
            Ret = kEplTimerInvalidHandle;
            goto Exit;
        }
        pTimerInfo = &EplTimerHighReskInstance_l.m_aTimerInfo[uiIndex];
        if (pTimerInfo->m_EventArg.m_TimerHdl != *pTimerHdl_p)
        {     // invalid handle
            goto Exit;
        }
    }

    *pTimerHdl_p = 0;
    pTimerInfo->m_EventArg.m_TimerHdl = 0;
    pTimerInfo->m_pfnCallback = NULL;

    // Disable the interrupt for this timer
    bReg = XTTCPSS_READ_REG(pTimerInfo->m_uiIndex,XTTCPSS_IER_OFFSET);
    bReg = 0;
    XTTCPSS_WRITE_REG(pTimerInfo->m_uiIndex, XTTCPSS_IER_OFFSET, bReg);
    // Stop the timer

    bReg = XTTCPSS_READ_REG(pTimerInfo->m_uiIndex,XTTCPSS_CNT_CNTRL_OFFSET);
    bReg = 0;
    bReg |= (XTTCPSS_CNT_CNTRL_DISABLE | XTTCPSS_CNT_CNTRL_EN_WAVE);
    XTTCPSS_WRITE_REG(pTimerInfo->m_uiIndex, XTTCPSS_CNT_CNTRL_OFFSET, bReg);

    Exit: return Ret;

}

//---------------------------------------------------------------------------
//
// Function:    TgtTimerCounterIsr()
//
// Description: target specific interrupt handler for slice timer and
//              general purpose timer.
//
// Parameters:  nIrqNum_p       = IRQ number
//              pDevInstData_p  = pointer to instance structure
//              ptRegs_p        = pointer to register structure
//
// Return:      irqreturn_t
//
// State:       not tested
//
//---------------------------------------------------------------------------

static irqreturn_t TgtTimerCounterIsr(int nIrqNum_p, void* pDevInstData_p)
{
    UINT                        uiIndex;
    tEplTimerHighReskTimerInfo* pTimerInfo = (tEplTimerHighReskTimerInfo*) pDevInstData_p;
    BYTE                        bReg = 0;
    tEplTimerHdl                OrgTimerHdl;
    WORD                        count;

    bReg = XTTCPSS_READ_REG(pTimerInfo->m_uiIndex,XTTCPSS_ISR_OFFSET);
    if (bReg == 0)
    {
        return IRQ_NONE;
    }
    // Acknowledge the Interrupt
    XTTCPSS_WRITE_REG(pTimerInfo->m_uiIndex, XTTCPSS_ISR_OFFSET, bReg);
    uiIndex = HDL_TO_IDX(pTimerInfo->m_EventArg.m_TimerHdl);

    if (uiIndex >= TIMER_COUNT)
    {     // invalid handle
        goto Exit;
    }

    if (!(bReg & (XTTCPSS_INTR_MATCH_1)) && !(bReg & XTTCPSS_INTR_INTERVAL))
    {
        // unknown interrupt
        goto Exit;
    }

    if (!pTimerInfo->m_fContinuously)
    {
        // Disable the interrupt for this timer
        bReg = XTTCPSS_READ_REG(pTimerInfo->m_uiIndex,XTTCPSS_IER_OFFSET);
        bReg = 0;
        XTTCPSS_WRITE_REG(pTimerInfo->m_uiIndex, XTTCPSS_IER_OFFSET, bReg);

        // stop the timer
        bReg = 0;
        bReg = XTTCPSS_READ_REG(pTimerInfo->m_uiIndex,XTTCPSS_CNT_CNTRL_OFFSET);
        bReg |= XTTCPSS_CNT_CNTRL_DISABLE;
        XTTCPSS_WRITE_REG(pTimerInfo->m_uiIndex, XTTCPSS_CNT_CNTRL_OFFSET,
                bReg);

        // Clear the Match counter
        XTTCPSS_WRITE_REG(pTimerInfo->m_uiIndex, XTTCPSS_MATCH_1_OFFSET,
                XTTCPSS_CLEAR);

    }

    if (pTimerInfo->m_pfnCallback != NULL)
    {
        // call the timer callback
        pTimerInfo->m_pfnCallback(&pTimerInfo->m_EventArg);
    }

    Exit:

    return IRQ_HANDLED;
}
