/****************************************************************************

    Copyright (c) 2012 Kalycito Infotech Private Limited

    Project: openPOWERLINK

    Description: Ethernet driver for Gigabit Ethernet Controller (GEM) 
                 on Zynq.

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
        1. Edrv82573.c and xilinx_emacps.c
****************************************************************************/
#include "global.h"
#include "EplInc.h"
#include "edrv.h"

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/of_address.h>
#include <linux/of_mdio.h>
#include <linux/phy.h>
#include <linux/mii.h>
#include <mach/board.h>
#include <mach/slcr.h>

#define EDRV_DRV_NAME                                    "epl"
#define EDRV_READ_REG(dwOffset)                         __raw_readl(EdrvInstance_l.m_pIoAddr + dwOffset)
#define EDRV_WRITE_REG(dwOffset, dwVal)                 __raw_writel(dwVal, EdrvInstance_l.m_pIoAddr + dwOffset)

//Reading and writing to descriptor memory
#define EDRV_DESC_WRITE(dwDescBase,dwOffset, dwVal)                __raw_writel(dwVal, ((u8 *)dwDescBase + dwOffset))
#define EDRV_DESC_READ(dwDescBase,dwOffset)                        __raw_readl((u8 *)dwDescBase + dwOffset)

#define EDRV_DESC_ADDR_OFFSET                   0x00000000
#define EDRV_DESC_CNTRL_OFFSET                  0x00000004

//Tx proto test (hardcoded SoCs sent out)
#define __EDRV_TX_TEST
//enable prints for register values in MAC
#define __REG_DEBUG

#define MARVELL_PHY_ADDR                0x07
#define PHY_CONTROL_REG_OFFSET          0x00
#define PHY_STATUS_REG_OFFSET           0x01
#define PHY_LINK_SPEED_100              0x2000
#define PHY_RESET                       0x8000                 
#define MII_MARVELL_PHY_PAGE            22
#define MII_M1116R_CONTROL_REG_MAC      21
#define MII_M1011_PHY_SCR               0x10
#define MII_M1011_PHY_SCR_AUTO_CROSS    0x0060

//Register Defines
#define EDRV_NET_CNTRL_REG              0x0
#define EDRV_NET_CONFIG_REG             0x4
#define EDRV_NET_STATUS_REG             0x8
#define EDRV_TX_STATUS_REG              0x14
#define EDRV_RXQBASE_REG                0x18
#define EDRV_RX_STATUS_REG              0x20
#define EDRV_TXQBASE_REG                0x1c
#define EDRV_INTR_DIS_REG               0x2C
#define EDRV_INTR_EN_REG                0x28
#define EDRV_INTR_STATUS_REG            0x24
#define EDRV_LATE_COLL_REG              0x144
#define EDRV_DEF_TX_REG                 0x148
#define EDRV_FRAMES_TX_REG              0x108 
#define EDRV_INTR_MASK_REG              0x00000030 
#define EDRV_HASHL_REG                  0x80
#define EDRV_HASHH_REG                  0x84
#define EDRV_INTR_EN_MASK               0x03FC7FFF

#define EDRV_LADDR1L_REG                0x88
#define EDRV_LADDR1H_REG                0x8C
#define EDRV_DMACFG_REG                 0x10
//Reg Masks
#define EDRV_NWCTRL_TXEN_MASK           0x00000008
#define EDRV_NWCTLR_RXEN_MASK           0x00000004
#define EDRV_NWCTLR_LOOPEN_MASK         0x00000002
#define EDRV_NWCTLR_MDEN_MASK           0x00000010
#define EDRV_NWCTLR_STARTTX_MASK        0x00000200
// Network Config Register
#define EDRV_NWCFG_MDCCLK_SHIFT         18
#define EDRV_NWCFG_100_MASK             0x00000001
#define EDRV_NWCFG_COPYALLEN_MASK       0x00000010
#define EDRV_NWCFG_FCSREMOVE_MASK       0x00020000
#define EDRV_NWCFG_MULTICASTEN_MASK     0x00000040
#define EDRV_NWCFG_UNICASTEN_MASK       0x00000080
#define EDRV_NWCFG_COPYALLEN_MASK       0x00000010
#define EDRV_NWCFG_FCSREMOVE_MASK       0x00020000

// Network Status register
#define EDRV_NWSR_MDIOIDLE_MASK         0x00000004

// DMA configuration register
#define EDRV_DMA_BURST_AHBINCR16_MASK   0x00000010      //AHB INCR16
#define EDRV_DMA_BURST_MASK             0x00000001      //SINGLE AHB burst
#define EDRV_DMA_BURST_AHBINCR4_MASK    0x00000004      //AHB INCR4 burst
#define EDRV_DMA_SWAP_MGMT_MASK         0x00000040      //Desc access 
#define EDRV_DMA_SWAP_PKTEN_MASK        0x00000080      //Pkt buffer
#define EDRV_DMA_RXPKT_BUFSIZE_MASK     0x00000300      //pkt buffer-8kb
#define EDRV_DMA_TXPKT_BUFSIZE4_MASK    0x00000400      //pkt buffer-4kb
#define EDRV_DMA_TXPKT_BUFSIZE2_MASK    0x00000000      //pkt buffer-2kb
#define EDRV_DMA_RXBUFF_SIZE_MASK       0x00180000      //Set Rx buff size as 1536 bytes
//Interrupt define
#define EDRV_TX_USED_BIT_READ           0x00000008
#define EDRV_TX_COMPLETE_READ           0x00000080
#define EDRV_RX_COMPLETE_READ           0x00000002
#define EDRV_TX_AHB_CORR_READ           0x00000040
#define EDRV_TX_BUFF_UNDRUN_READ        0x00000010
#define EDRV_TX_LATE_COL_READ           0x00000020
#define EDRV_RX_OVERUN_READ             0x00000400
#define EDRV_RX_USED_BIT_READ           0x00000004

// Descriptor defines
#define EDRV_DESC_LAST_BUFF_MASK        (1 << 15)
#define EDRV_TX_DESC_WRAP_MASK          0x40000000
#define EDRV_DESC_USED_BIT_MASK         0x80000000
#define EDRV_DESC_AHB_ERROR_MASK        0x08000000
#define EDRV_DESC_LATE_COLL_MASK        0x04000000
#define EDRV_RX_FRAME_LENGTH_MASK       0x00001FFF
#define EDRV_TX_FRAME_LENGTH_MASK       0x00003FFF
#define EDRV_RX_FRAME_START_MASK        0x00004000
#define EDRV_TX_FRAME_END_MASK          0x00008000

#define EDRV_MAX_TX_DESCRIPTOR          128             //Max no of Desc in mem
#define EDRV_MAX_RX_DESCRIPTOR          128             //Max no of Desc in mem
#define EDRV_MAX_TX_DESC_LEN           (EDRV_MAX_TX_DESCRIPTOR - 1)     //one slot to diff full
#define EDRV_MAX_RX_DESC_LEN           (EDRV_MAX_RX_DESCRIPTOR - 1) 

#ifndef EDRV_MAX_TX_BUFFERS
#define EDRV_MAX_TX_BUFFERS             42              //up-to 128 buffers are supported per frame
#endif   

#ifndef EDRV_MAX_RX_BUFFERS  
#define EDRV_MAX_RX_BUFFERS             128
#endif

#define EDRV_MAX_FRAME_SIZE             0x600    //1536
#define EDRV_TX_BUFFER_SIZE             (EDRV_MAX_TX_BUFFERS * EDRV_MAX_FRAME_SIZE) // n * (MTU + 14 + 4)
#define EDRV_RX_BUFFER_SIZE             (EDRV_MAX_RX_BUFFERS * EDRV_MAX_FRAME_SIZE)

#define EDRV_TX_DESCS_SIZE              (EDRV_MAX_TX_DESCRIPTOR * sizeof(tEdrvTxDesc))    
#define EDRV_RX_DESCS_SIZE              (EDRV_MAX_RX_DESCRIPTOR * sizeof (tEdrvRxDesc))

#define EDRV_RXBUF_CLEAR_USED_MASK      0xFFFFFFFE
#define EDRV_RXBUF_WRAP_MASK            0x00000002
#define EDRV_TXBUF_USED_MASK            0x80000000
#define EDRV_TXBUF_WRAP_MASK            0x40000000

//Multicast filtering through hashing
#define _MULTICASTEN_MODE_
//#define _PROMISCUOUS_MODE_

// PHY Maintenance defines
#define EDRV_PHYMNTNC_OFFSET            0x00000034      // PHY Maintenance Reg offset
#define EDRV_PHYMNTNC_OP_MASK           0x40020000      // clause_22 and must_10 bits mask
#define EDRV_PHYMNTNC_OP_READ           0x20000000      // Set Read bit
#define EDRV_PHYMNTNC_OP_WRITE          0x10000000      // Set Write Bit
#define EDRV_PHYMNTNC_PHYAD_SHIFT       23              // shift bits to form PHY address
#define EDRV_PHYMNTNC_PHYREG_SHIFT      18              // shift bits to form PHY register
#define EDRV_PHYMNTNC_DATA_MASK         0x0000FFFF      // data bits
#define XSLCR_EMAC0_CLK_CTRL_OFFSET     0x140           /* EMAC0 Reference Clk Control */
#define XSLCR_EMAC1_CLK_CTRL_OFFSET     0x144           /* EMAC1 Reference Clk Control */
#define EDRV_SLCR_DIV_MASK              0xFC0FC0FF

#define EDRV_IXR_FRAMERX_MASK           0x00000002
// MDC clock division - set according to pclk speed.
enum
{
    MDC_DIV_8 = 0,
    MDC_DIV_16,
    MDC_DIV_32,
    MDC_DIV_48,
    MDC_DIV_64,
    MDC_DIV_96,
    MDC_DIV_128,
    MDC_DIV_224
};

typedef struct
{
    UINT        m_uiTxBufferAddr;
    UINT        m_uiTxCntrlStatus; /*len+status+CRC*/

} tEdrvTxDesc;

typedef struct
{
    UINT        m_uiRxBufferAddr;
    UINT        m_uiStatus;               //may require additional fields

} tEdrvRxDesc;

typedef struct
{
    struct platform_device*   m_pPltDev;
    void*                     m_pIoAddr;      // pointer to register space of Ethernet controller
    tEdrvInitParam            m_InitParam;
    resource_size_t           m_MmIoAddr;
    resource_size_t           m_MmIoSize;
    dma_addr_t                m_pRxDescDma;
    dma_addr_t                m_pTxDescDma;
    BYTE                      abMacAddr[6];
    BOOL                      m_afTxBufUsed[EDRV_MAX_TX_BUFFERS];
    tEdrvTxBuffer*            m_apTxBuffer[EDRV_MAX_TX_DESCRIPTOR];
    BYTE*                     m_apbRxBufInDesc[EDRV_MAX_RX_DESCRIPTOR];
    UINT                      m_uiDescHead;
    UINT                      m_uiDescTail;
    void *                    m_pTxDescVirt;
    void *                    m_pRxDescVirt;
    void *                    m_pTxBuffer;
    void *                    m_pRxBuffer;
    tEdrvTxDesc *             m_pTxDescAddr;
    tEdrvRxDesc *             m_pRxDescAddr;
    UINT                      m_uiRxDescHead;
    UINT                      m_uiRxDescTail;
    UINT                      m_slcr_div0_100Mbps;
    UINT                      m_slcr_div1_100Mbps;
    UINT                      m_Irq;
} tEdrvInstance;

tEdrvInstance EdrvInstance_l;

#ifdef __EDRV_TX_TEST
unsigned char abSocData[60] = { 0x01, 0x11, 0x1e, 0x00, 0x00, 0x01, 0x00, 0x03,
        0xc0, 0xa8, 0x64, 0xf0, 0x88, 0xab, 0x01, 0xff, 0xf0, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00 };
#endif

#ifdef CONFIG_OF
static struct of_device_id xemacps_of_match[] =
{
    {   .compatible = "xlnx,ps7-ethernet-1.00.a",},     //__devinitdata creates warning!
    { /* end of table */}               //keep devinit in separate data section,
                                        //linker is not able to link
};
MODULE_DEVICE_TABLE(of, xemacps_of_match);
#else
#define xemacps_of_match NULL
#endif /* CONFIG_OF */

static int EdrvInitOne(struct platform_device *pDev_p);
static int EdrvRemoveOne(struct platform_device *pDev_p);
static irqreturn_t EdrvIntrHandler(int iIrqNum, void *pInstData);
UINT EdrvCalcHashAddr(BYTE * pbMAC_p);

static struct platform_driver EdrvDriver = {

.probe = EdrvInitOne, .remove = EdrvRemoveOne, .suspend = NULL,     //Not handling power management functions
        .resume = NULL,     //Not handling power management functions
        .driver = { .name = EDRV_DRV_NAME, .owner = THIS_MODULE,
                .of_match_table = xemacps_of_match,     //This function is to check the device
                },				    //from the device tree
        };

//---------------------------------------------------------------------------
//
// Function:    EdrvMacPsWrite
//
// Description: initializes one PCI device
//
// Parameters:  pPciDev             = pointer to corresponding PCI device structure
//              pId                 = PCI device ID
//
// Returns:     (int)               = error code
//
// State:
//
//---------------------------------------------------------------------------
static int EdrvMacPsWrite(INT iMiiId_p, INT iPhyreg_p, WORD wValue_p)
{
    DWORD           dwRegVal = 0;
    volatile DWORD  dwPhyIdle;

    dwRegVal |= EDRV_PHYMNTNC_OP_MASK;
    dwRegVal |= EDRV_PHYMNTNC_OP_WRITE;
    dwRegVal |= (iMiiId_p << EDRV_PHYMNTNC_PHYAD_SHIFT);
    dwRegVal |= (iPhyreg_p << EDRV_PHYMNTNC_PHYREG_SHIFT);
    dwRegVal |= wValue_p;

    EDRV_WRITE_REG(EDRV_PHYMNTNC_OFFSET, dwRegVal);

    // wait for completion of transfer

    do
    {
        cpu_relax();
        dwPhyIdle = EDRV_READ_REG(EDRV_NET_STATUS_REG);
    } while ((dwPhyIdle & EDRV_NWSR_MDIOIDLE_MASK) == 0);     //wait PHYIDLE is set 1

    return 0;
}
//---------------------------------------------------------------------------
//
// Function:    EdrvMacPsRead
//
// Description: read from PHY registers
//
// Parameters:  iMiiId_p    = PHY address
//              iPhyreg_p   = PHY register
//
// Returns:     (int)      = error code
//
// State:
//
//---------------------------------------------------------------------------
static int EdrvMacPsRead(INT iMiiId_p, INT iPhyreg_p)
{

    DWORD           dwRegVal = 0;
    INT             iValue = 0;
    volatile DWORD  dwPhyIdle;

    dwRegVal |= EDRV_PHYMNTNC_OP_MASK;
    dwRegVal |= EDRV_PHYMNTNC_OP_READ;
    dwRegVal |= (iMiiId_p << EDRV_PHYMNTNC_PHYAD_SHIFT);
    dwRegVal |= (iPhyreg_p << EDRV_PHYMNTNC_PHYREG_SHIFT);

    EDRV_WRITE_REG(EDRV_PHYMNTNC_OFFSET, dwRegVal);

    // wait till the value is being read

    do
    {
        cpu_relax();
        dwPhyIdle = EDRV_READ_REG(EDRV_NET_STATUS_REG);

    } while ((dwPhyIdle & EDRV_NWSR_MDIOIDLE_MASK) == 0);     //wait PHYIDLE is set 1

    iValue = (EDRV_READ_REG(EDRV_PHYMNTNC_OFFSET) & EDRV_PHYMNTNC_DATA_MASK);

    return iValue;

}
//---------------------------------------------------------------------------
//
// Function:    EdrvInit
//

// Description: function for init of the Ethernet controller
//
// Parameters:  pEdrvInitParam_p    = pointer to struct including the init-parameters
//

// Returns:     Errorcode           = kEplSuccessful
//                                  = kEplNoResource
//
// State:

//
//---------------------------------------------------------------------------
tEplKernel EdrvInit(tEdrvInitParam * pEdrvInitParam_p)
{
    tEplKernel  EplRet = kEplSuccessful;
    INT         iRet;
    INT         iLoop;

    EPL_MEMSET(&EdrvInstance_l, 0x0, sizeof(EdrvInstance_l));
    EdrvInstance_l.m_InitParam = *pEdrvInitParam_p;

    printk("(%s) Registering the driver to the kernel...", __func__);

    /*TODO:This function can be replaced with platform_driver_probe 
     to reduce memory footprints */
    iRet = platform_driver_register(&EdrvDriver);
    if (iRet != 0)
    {
        EplRet = kEplNoResource;
        printk("Failed \n");

    }
    printk("Done \n");

    // local MAC address might have been changed in EdrvInitOne
    EPL_MEMCPY(pEdrvInitParam_p->m_abMyMacAddr,
            EdrvInstance_l.m_InitParam.m_abMyMacAddr, 6);

    printk("Local MAC = ");
    for (iLoop = 0; iLoop < 6; iLoop++)
    {
        printk("0x%02x ", pEdrvInitParam_p->m_abMyMacAddr[iLoop]);
    }
    printk("\n");

    return EplRet;
}

//---------------------------------------------------------------------------
//
// Function:    EdrvChangeFilter
//

// Description: Change all rx-filters or one specific rx-filter
//              of the openMAC
//
// Parameters:  pFilter_p           = pointer to array of filter entries

//              uiCount_p           = number of filters in array
//              uiEntryChanged_p    = selects one specific filter which is
//                                    to be changed. If value is equal to
//                                    or larger than uiCount_p, all entries

//                                    are selected.
//              uiChangeFlags_p     = If one specific entry is selected,
//                                    these flag bits show which filter
//                                    properties have been changed.

//                                    available flags:
//                                      EDRV_FILTER_CHANGE_MASK
//                                      EDRV_FILTER_CHANGE_VALUE
//                                      EDRV_FILTER_CHANGE_STATE

//                                      EDRV_FILTER_CHANGE_AUTO_RESPONSE
//                                    if auto-response delay is supported:
//                                      EDRV_FILTER_CHANGE_AUTO_RESPONSE_DELAY
//

// Returns:     Errorcode           = kEplSuccessful
//                                  = kEplEdrvInvalidParam
//
// State:

//
//---------------------------------------------------------------------------
tEplKernel EdrvChangeFilter(tEdrvFilter* pFilter_p, UINT uiCount_p,

UINT uiEntryChanged_p, UINT uiChangeFlags_p)
{
    tEplKernel Ret = kEplSuccessful;

    return Ret;
}
//---------------------------------------------------------------------------
//
// Function:    EdrvDefineRxMacAddrEntry
//
// Description: Set a multicast entry into the Ethernet controller
//
// Parameters:  pbMacAddr_p     = pointer to multicast entry to set
//
// Returns:     Errorcode       = kEplSuccessful
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel EdrvDefineRxMacAddrEntry(BYTE * pbMacAddr_p)
{
    tEplKernel      Ret = kEplSuccessful;
    UINT            uiHashValue;
    UINT            uiHashRegLVal = 0;
    UINT            uiHashRegHVal = 0;

    uiHashValue = EdrvCalcHashAddr(pbMacAddr_p);
    uiHashRegLVal = EDRV_READ_REG(EDRV_HASHL_REG);
    uiHashRegHVal = EDRV_READ_REG(EDRV_HASHL_REG);
    if (uiHashValue < 32)
    {
        uiHashRegLVal |= (1 << uiHashValue);
    }
    else
    {
        uiHashRegHVal |= (1 << (uiHashValue - 32));
    }

    EDRV_WRITE_REG(EDRV_HASHL_REG, uiHashRegLVal);
    EDRV_WRITE_REG(EDRV_HASHH_REG, uiHashRegHVal);
    return Ret;
}
//---------------------------------------------------------------------------
//
// Function:    EdrvUndefineRxMacAddrEntry
//
// Description: Reset a multicast entry in the Ethernet controller
//
// Parameters:  pbMacAddr_p     = pointer to multicast entry to reset
//
// Returns:     Errorcode       = kEplSuccessful
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel EdrvUndefineRxMacAddrEntry(BYTE * pbMacAddr_p)
{
    tEplKernel      Ret = kEplSuccessful;
    UINT            uiHashValue;
    UINT            uiHashRegLVal = 0;
    UINT            uiHashRegHVal = 0;

    uiHashValue = EdrvCalcHashAddr(pbMacAddr_p);

    uiHashRegLVal = EDRV_READ_REG(EDRV_HASHL_REG);
    uiHashRegHVal = EDRV_READ_REG(EDRV_HASHL_REG);

    if (uiHashValue < 32)
    {
        uiHashRegLVal &= ~(1 << uiHashValue);
    }
    else
    {
        uiHashRegHVal &= ~(1 << (uiHashValue - 32));
    }

    EDRV_WRITE_REG(EDRV_HASHL_REG, uiHashRegLVal);
    EDRV_WRITE_REG(EDRV_HASHH_REG, uiHashRegHVal);

    return Ret;
}
//---------------------------------------------------------------------------
//
// Function:    EdrvAllocTxMsgBuffer
//
// Description: Register a Tx-Buffer
//
// Parameters:  pBuffer_p   = pointer to Buffer structure
//
// Returns:     Errorcode   = kEplSuccessful
//                          = kEplEdrvNoFreeBufEntry
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel EdrvAllocTxMsgBuffer(tEdrvTxBuffer * pBuffer_p)
{
    tEplKernel  Ret = kEplSuccessful;
    INT         iChannel;

    if (pBuffer_p->m_uiMaxBufferLen > EDRV_MAX_FRAME_SIZE)
    {
        Ret = kEplEdrvNoFreeBufEntry;
        goto Exit;
    }

    if (EdrvInstance_l.m_pTxBuffer == NULL)
    {
        printk("%s Tx buffers currently not allocated\n", __FUNCTION__);
        Ret = kEplEdrvNoFreeBufEntry;
        goto Exit;
    }

    // search a free Tx buffer with appropriate size
    for (iChannel = 0; iChannel < EDRV_MAX_TX_BUFFERS; iChannel++)
    {
        if (EdrvInstance_l.m_afTxBufUsed[iChannel] == FALSE)
        {
            // free channel found
            EdrvInstance_l.m_afTxBufUsed[iChannel] = TRUE;
            pBuffer_p->m_BufferNumber.m_dwVal = iChannel;
            pBuffer_p->m_pbBuffer = EdrvInstance_l.m_pTxBuffer
                    + (iChannel * EDRV_MAX_FRAME_SIZE);
            pBuffer_p->m_uiMaxBufferLen = EDRV_MAX_FRAME_SIZE;
            break;
        }
    }
    if (iChannel >= EDRV_MAX_TX_BUFFERS)
    {
        Ret = kEplEdrvNoFreeBufEntry;
        goto Exit;
    }

    Exit: return Ret;
}
//---------------------------------------------------------------------------
//
// Function:    EdrvReleaseTxMsgBuffer
//
// Description: Register a Tx-Buffer
//
// Parameters:  pBuffer_p   = pointer to Buffer structure
//
// Returns:     Errorcode   = kEplSuccessful
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel EdrvReleaseTxMsgBuffer(tEdrvTxBuffer * pBuffer_p)
{
    UINT uiBufferNumber;

    uiBufferNumber = pBuffer_p->m_BufferNumber.m_dwVal;

    if (uiBufferNumber < EDRV_MAX_TX_BUFFERS)
    {
        EdrvInstance_l.m_afTxBufUsed[uiBufferNumber] = FALSE;
    }

    return kEplSuccessful;

}
//---------------------------------------------------------------------------
//
// Function:    EdrvSendTxMsg
//
// Description: immediately starts the transmission of the buffer
//
// Parameters:  pBuffer_p   = buffer descriptor to transmit
//
// Returns:     Errorcode   = kEplSuccessful
//
// State:

//
//---------------------------------------------------------------------------
tEplKernel EdrvSendTxMsg(tEdrvTxBuffer * pBuffer_p)
{
    tEplKernel      Ret = kEplSuccessful;
    UINT            uiBufferNumber;
    tEdrvTxDesc     *pTxDesc;
    dma_addr_t      TxDmaAddr;
    DWORD           dwReg;

    uiBufferNumber = pBuffer_p->m_BufferNumber.m_dwVal;

    if ((uiBufferNumber >= EDRV_MAX_TX_BUFFERS)
            || (EdrvInstance_l.m_afTxBufUsed[uiBufferNumber] == FALSE))
    {
        Ret = kEplEdrvBufNotExisting;
        goto Exit;
    }

    if (((EdrvInstance_l.m_uiDescTail + 1) & (EDRV_MAX_TX_DESC_LEN))== EdrvInstance_l.m_uiDescHead)
    {
        Ret = kEplEdrvNoFreeTxDesc;
        printk("{%s}->no free Desc\n", __func__);
        goto Exit;
    }

    EdrvInstance_l.m_apTxBuffer[EdrvInstance_l.m_uiDescTail] = pBuffer_p;     //saved for call back

    if (pBuffer_p->m_uiTxMsgLen < 60)
    {
        pBuffer_p->m_uiTxMsgLen = 60;
    }
    pTxDesc = &EdrvInstance_l.m_pTxDescAddr[EdrvInstance_l.m_uiDescTail];
    TxDmaAddr = dma_map_single(&EdrvInstance_l.m_pPltDev->dev,
            pBuffer_p->m_pbBuffer, pBuffer_p->m_uiTxMsgLen, DMA_TO_DEVICE);
    EDRV_DESC_WRITE(pTxDesc, EDRV_DESC_ADDR_OFFSET, TxDmaAddr);
    wmb();     //no re-ordering!

    dwReg = 0;
    dwReg = EDRV_DESC_READ(pTxDesc, EDRV_DESC_CNTRL_OFFSET);
    dwReg &= ~EDRV_DESC_USED_BIT_MASK;     //clear the used bit
    dwReg &= ~EDRV_TX_FRAME_LENGTH_MASK;
    dwReg |= (pBuffer_p->m_uiTxMsgLen | EDRV_DESC_LAST_BUFF_MASK);     //last buffer in the frame
    EDRV_DESC_WRITE(pTxDesc, EDRV_DESC_CNTRL_OFFSET, dwReg);
    wmb();     //no re-ordering!

    dwReg = 0;
    dwReg = EDRV_READ_REG(EDRV_NET_CNTRL_REG);
    dwReg |= EDRV_NWCTLR_STARTTX_MASK;
    EDRV_WRITE_REG(EDRV_NET_CNTRL_REG, dwReg);
    EdrvInstance_l.m_uiDescTail = (EdrvInstance_l.m_uiDescTail + 1)
            & (EDRV_MAX_TX_DESC_LEN);     //scale len to size

    Exit: return Ret;
}
//---------------------------------------------------------------------------
//
// Function:     EdrvIntrHandler
//
// Description:  interrupt handler
//
// Parameters:   void
//
// Returns:      void
//
// State:
//
//---------------------------------------------------------------------------
static irqreturn_t EdrvIntrHandler(INT iIrqNum, void *pInstData)
{

    DWORD           dwIsrStatus;
    DWORD           dwStat = 0;
    irqreturn_t     iRet = IRQ_HANDLED;
    tEdrvTxDesc     *pTxDesc;
    tEdrvTxBuffer   *pTxBuffer;
    tEdrvRxDesc     *pRxDesc;
    //unsigned int   uiOrigHeadDesc; 

    dwIsrStatus = EDRV_READ_REG(EDRV_INTR_STATUS_REG);
    //not a shared handler, yet!
    if (0 == dwIsrStatus)
    {
        iRet = IRQ_NONE;
        goto Exit;
    }
    else
    {
        //Acknowledge interrupt
        EDRV_WRITE_REG(EDRV_INTR_STATUS_REG, dwIsrStatus);
    }

    //process Rx with higher priority
    if (dwIsrStatus & (EDRV_RX_COMPLETE_READ))
    {
        //uiOrigHeadDesc = EdrvInstance_l.m_uiRxDescHead;
        if (dwIsrStatus & (EDRV_RX_OVERUN_READ | EDRV_RX_USED_BIT_READ))
        {
            //ERROR: Shd we abort?
            goto Exit;
        }
        pRxDesc = &EdrvInstance_l.m_pRxDescAddr[EdrvInstance_l.m_uiRxDescHead];
        while (pRxDesc->m_uiRxBufferAddr & ~EDRV_RXBUF_CLEAR_USED_MASK)
        {
            tEdrvRxBuffer RxBuffer;
            WORD wFrameLen;
            DWORD dwReg;

            dwReg = 0;
            dwReg = EDRV_DESC_READ(pRxDesc,EDRV_DESC_CNTRL_OFFSET);
            wFrameLen = dwReg & EDRV_RX_FRAME_LENGTH_MASK;

            RxBuffer.m_uiRxMsgLen = AmiGetWordFromLe(&wFrameLen);
            RxBuffer.m_pbBuffer =
                    EdrvInstance_l.m_apbRxBufInDesc[EdrvInstance_l.m_uiRxDescHead];
            dma_sync_single_for_cpu(&EdrvInstance_l.m_pPltDev->dev,
                    pRxDesc->m_uiRxBufferAddr, wFrameLen, DMA_FROM_DEVICE);
            EdrvInstance_l.m_InitParam.m_pfnRxHandler(&RxBuffer);

            //clear the status
            dwReg = 0;
            EDRV_DESC_WRITE(pRxDesc, EDRV_DESC_CNTRL_OFFSET, dwReg);
            //clear used bit
            dwReg = EDRV_DESC_READ(pRxDesc,EDRV_DESC_ADDR_OFFSET);
            dwReg &= EDRV_RXBUF_CLEAR_USED_MASK;

            if ((EDRV_MAX_RX_DESCRIPTOR - 1) == EdrvInstance_l.m_uiRxDescHead)
            {
                dwReg |= EDRV_RXBUF_WRAP_MASK;
            }
            EDRV_DESC_WRITE(pRxDesc, EDRV_DESC_ADDR_OFFSET, dwReg);
            EdrvInstance_l.m_uiRxDescHead = (EdrvInstance_l.m_uiRxDescHead + 1)
                    & EDRV_MAX_RX_DESC_LEN;
            pRxDesc =
                    &EdrvInstance_l.m_pRxDescAddr[EdrvInstance_l.m_uiRxDescHead];
            if (EdrvInstance_l.m_uiRxDescHead == 0)
            {
                EdrvInstance_l.m_uiRxDescTail = EDRV_MAX_RX_DESC_LEN;
            }
            else
            {
                EdrvInstance_l.m_uiRxDescTail = EdrvInstance_l.m_uiRxDescHead
                        - 1;
            }
        }
    }

    //process Tx
    if (dwIsrStatus & (EDRV_TX_COMPLETE_READ))
    {
        do
        {
            pTxDesc =
                    &EdrvInstance_l.m_pTxDescAddr[EdrvInstance_l.m_uiDescHead];
            if (pTxDesc == NULL)
            {
                //TODO: Check the validity of this check
                break;
            }
            dwStat = EDRV_DESC_READ(pTxDesc, EDRV_DESC_CNTRL_OFFSET);
            if (!(dwStat & EDRV_DESC_USED_BIT_MASK))
            {
                break;
            }
            if (dwStat & EDRV_DESC_AHB_ERROR_MASK)
            {
                //ERROR: Shd we abort?
            }
            else if (dwStat & EDRV_DESC_LATE_COLL_MASK)
            {
                //ERROR: Late coll, prone in gigabit mode
            }
            else
            {
                pTxBuffer =
                        EdrvInstance_l.m_apTxBuffer[EdrvInstance_l.m_uiDescHead];
                if (pTxBuffer == NULL)
                {
                    break;

                }
                EdrvInstance_l.m_apTxBuffer[EdrvInstance_l.m_uiDescHead] = NULL;
                EdrvInstance_l.m_uiDescHead = ((EdrvInstance_l.m_uiDescHead + 1)
                        & EDRV_MAX_TX_DESC_LEN);
                dma_unmap_single(&EdrvInstance_l.m_pPltDev->dev,
                        pTxDesc->m_uiTxBufferAddr, pTxBuffer->m_uiTxMsgLen,
                        DMA_TO_DEVICE);
                if (NULL != pTxBuffer)
                {
                    // Call Tx handler of Data link layer
                    if (pTxBuffer->m_pfnTxHandler != NULL)
                    {
                        pTxBuffer->m_pfnTxHandler(pTxBuffer);
                    }
                }
            }

        } while (EdrvInstance_l.m_uiDescHead != EdrvInstance_l.m_uiDescTail);
    }

    Exit: return iRet;
}
//---------------------------------------------------------------------------
//
// Function:    EdrvInitOne
//
// Description: initializes platform device
//
// Parameters:  pDev_p             = pointer to corresponding platform device structure
// 
// Returns:     (int)               = error code
//
// State:
//
//---------------------------------------------------------------------------

static int EdrvInitOne(struct platform_device *pDev_p)
{
    struct resource     *pResIoMem;
    struct resource     *pResIrq;
    INT                 iRet = 0, iLoop = 0;
    DWORD               dwReg = 0;
    DWORD               dwMacLAddr;
    WORD                wMacHAddr;
    dma_addr_t          RxDmaAddr;
    WORD                wCtrl;
    INT                 iValue;
    const void          *pProp;

    if (NULL == pDev_p)
    {
        printk("%s device (%s) discarded\n", __func__, pDev_p->name);
        iRet = -ENODEV;
        goto Exit;
    }

    if (NULL != EdrvInstance_l.m_pPltDev)
    {
        printk("%s device (%s) already registered\n", __func__, pDev_p->name);
        iRet = -ENODEV;
        goto Exit;
    }

    //create local instance
    EdrvInstance_l.m_pPltDev = pDev_p;

    printk("(%s) IOMEM Resource initialization...", __func__);
    pResIoMem = platform_get_resource(pDev_p, IORESOURCE_MEM, 0);
    if (NULL == pResIoMem)
    {
        printk("Failed \n");
        iRet = -ENODEV;
        goto Exit;
    }
    printk("Done \n");

    printk("(%s) IRQ Resource initialization...", __func__);
    pResIrq = platform_get_resource(pDev_p, IORESOURCE_IRQ, 0);
    if (NULL == pResIrq)
    {
        printk("Failed \n");
        iRet = -ENODEV;
        goto Exit;
    }
    printk("Done \n");

    /*Local instance copy to clear mem*/
    EdrvInstance_l.m_MmIoAddr = pResIoMem->start;
    EdrvInstance_l.m_MmIoSize = (pResIoMem->end - pResIoMem->start + 1);

    /*Obtain the region exclusively for Edrv*/
    if (!request_mem_region(EdrvInstance_l.m_MmIoAddr,
            EdrvInstance_l.m_MmIoSize, EDRV_DRV_NAME))
    {
        printk("Req mem region failed \n");
        iRet = -ENOMEM;
        goto Exit;
    }
    printk("MEM_RESOURCE: Start 0x(%X), End 0x(%X) \n", pResIoMem->start,
            pResIoMem->end);

    /*Physical memory mapped to virtual memory*/
    EdrvInstance_l.m_pIoAddr = ioremap(pResIoMem->start,
            (pResIoMem->end - pResIoMem->start + 1));
    if (NULL == EdrvInstance_l.m_pIoAddr)
    {
        printk("Ioremap failed \n");
        iRet = -EIO;
        goto Exit;
    }

    //Request IRQ
    printk("Requesting IRQ resource ...");

    if (request_irq(pResIrq->start, EdrvIntrHandler, IRQF_SAMPLE_RANDOM,
            pDev_p->name, pDev_p))
    {
        printk("Failed \n");
        iRet = -EIO;
        goto Exit;
    }
    EdrvInstance_l.m_Irq = pResIrq->start;
    printk("Done \n");

    //Begin by clearing the registers ----------------->
    //Disable Tx and Rx circuit
    EDRV_WRITE_REG(EDRV_NET_CNTRL_REG, 0x0);
    //Disble interrupts        
    EDRV_WRITE_REG(EDRV_INTR_DIS_REG, ~0x0);
    //clear Tx and Rx status registers
    EDRV_WRITE_REG(EDRV_TX_STATUS_REG, ~0x0);
    EDRV_WRITE_REG(EDRV_RX_STATUS_REG, ~0x0);
    //Register clean done ----------------------------->

    pProp = of_get_property(pDev_p->dev.of_node, "xlnx,slcr-div0-100Mbps",
            NULL);
    if (pProp)
    {
        EdrvInstance_l.m_slcr_div0_100Mbps = (u32) be32_to_cpup(pProp);
    }

    pProp = of_get_property(pDev_p->dev.of_node, "xlnx,slcr-div1-100Mbps",
            NULL);
    if (pProp)
    {
        EdrvInstance_l.m_slcr_div1_100Mbps = (u32) be32_to_cpup(pProp);
    }

    // Set MDC clock Division to be used

    dwReg = EDRV_READ_REG(EDRV_NET_CONFIG_REG);
    dwReg = 0;
    dwReg |= (MDC_DIV_224 << EDRV_NWCFG_MDCCLK_SHIFT);
    dwReg |= EDRV_NWCFG_100_MASK;     // set speed to 100Mbps

#ifdef _PROMISCUOUS_MODE_
    dwReg |= EDRV_NWCFG_COPYALLEN_MASK;
#elif defined _MULTICASTEN_MODE_
    dwReg |= EDRV_NWCFG_MULTICASTEN_MASK;
#elif defined __UNICASTEN_MODE_
    dwReg |= EDRV_NWCFG_UNICASTEN_MASK;
#endif
    dwReg |= EDRV_NWCFG_FCSREMOVE_MASK;
    EDRV_WRITE_REG(EDRV_NET_CONFIG_REG, dwReg);

    //Enable MDIO port
    dwReg = 0;
    dwReg = EDRV_READ_REG(EDRV_NET_CNTRL_REG);
    dwReg |= EDRV_NWCTLR_MDEN_MASK;
    EDRV_WRITE_REG(EDRV_NET_CNTRL_REG, dwReg);

    //Configure Slcr registers
    dwReg = xslcr_read(XSLCR_EMAC0_CLK_CTRL_OFFSET);
    dwReg &= EDRV_SLCR_DIV_MASK;
    dwReg |= ((EdrvInstance_l.m_slcr_div1_100Mbps) << 20);
    dwReg |= ((EdrvInstance_l.m_slcr_div0_100Mbps) << 8);
    xslcr_write((u32) XSLCR_EMAC0_CLK_CTRL_OFFSET, dwReg);

    //Set the MAC address
    if ((EdrvInstance_l.m_InitParam.m_abMyMacAddr[0] != 0)
            | (EdrvInstance_l.m_InitParam.m_abMyMacAddr[1] != 0)
            | (EdrvInstance_l.m_InitParam.m_abMyMacAddr[2] != 0)
            | (EdrvInstance_l.m_InitParam.m_abMyMacAddr[3] != 0)
            | (EdrvInstance_l.m_InitParam.m_abMyMacAddr[4] != 0)
            | (EdrvInstance_l.m_InitParam.m_abMyMacAddr[5] != 0))
    {
        dwMacLAddr = 0;
        dwMacLAddr |= EdrvInstance_l.m_InitParam.m_abMyMacAddr[5];
        dwMacLAddr |= EdrvInstance_l.m_InitParam.m_abMyMacAddr[4] << 8;
        dwMacLAddr |= EdrvInstance_l.m_InitParam.m_abMyMacAddr[3] << 16;
        dwMacLAddr |= EdrvInstance_l.m_InitParam.m_abMyMacAddr[2] << 24;
        EDRV_WRITE_REG(EDRV_LADDR1L_REG, dwMacLAddr);
        wMacHAddr = 0;
        wMacHAddr |= EdrvInstance_l.m_InitParam.m_abMyMacAddr[1];
        wMacHAddr |= EdrvInstance_l.m_InitParam.m_abMyMacAddr[0] << 8;
        EDRV_WRITE_REG(EDRV_LADDR1H_REG, wMacHAddr);
    }
    else
    {
        dwMacLAddr = 0;
        dwMacLAddr = EDRV_READ_REG(EDRV_LADDR1L_REG);
        EdrvInstance_l.m_InitParam.m_abMyMacAddr[0] = (dwMacLAddr & 0xFF);
        EdrvInstance_l.m_InitParam.m_abMyMacAddr[1] =
                ((dwMacLAddr >> 8) & 0xFF);
        EdrvInstance_l.m_InitParam.m_abMyMacAddr[2] =
                ((dwMacLAddr >> 16) & 0xFF);
        EdrvInstance_l.m_InitParam.m_abMyMacAddr[3] =
                ((dwMacLAddr >> 24) & 0xFF);
        wMacHAddr = 0;
        wMacHAddr = EDRV_READ_REG(EDRV_LADDR1H_REG);
        EdrvInstance_l.m_InitParam.m_abMyMacAddr[4] = (wMacHAddr & 0xFF);
        EdrvInstance_l.m_InitParam.m_abMyMacAddr[5] = ((wMacHAddr >> 8) & 0xFF);
    }

    //allocate the Tx and Rx buffer
    /*For Buffer allocation should the memory be dma'ble
     Currently doing a kzalloc here*/
    //Zeros mem before returing pointer
    printk("Allocation of Tx Buffers ...");
    EdrvInstance_l.m_pTxBuffer = kzalloc(EDRV_TX_BUFFER_SIZE, GFP_KERNEL);
    if (!EdrvInstance_l.m_pTxBuffer)
    {
        printk("Failed \n");
        iRet = -ENOMEM;
        goto Exit;
    }
    printk("Done \n");

    //allocate tx and rx buffer descriptors
    printk("Allocation of Tx Desc ...");
    EdrvInstance_l.m_pTxDescVirt = dma_alloc_coherent(&pDev_p->dev,
            EDRV_TX_DESCS_SIZE, &EdrvInstance_l.m_pTxDescDma, GFP_KERNEL);
    if (NULL == EdrvInstance_l.m_pTxDescVirt)
    {
        printk("Failed \n");
        iRet = -ENOMEM;
        goto Exit;
    }
    printk("Done \n");

    printk("Allocation of Rx Desc ...");
    EdrvInstance_l.m_pRxDescVirt = dma_alloc_coherent(&pDev_p->dev,
            EDRV_RX_DESCS_SIZE, &EdrvInstance_l.m_pRxDescDma, GFP_KERNEL);
    if (NULL == EdrvInstance_l.m_pRxDescVirt)
    {
        printk("Failed \n");
        iRet = -ENOMEM;
        goto Exit;
    }
    printk("Done \n");
    EdrvInstance_l.m_pRxDescAddr = (tEdrvRxDesc *) EdrvInstance_l.m_pRxDescVirt;

    printk("Allocation of Rx Buffers ...");
    EdrvInstance_l.m_pRxBuffer = kzalloc(EDRV_RX_BUFFER_SIZE, GFP_KERNEL);
    if (!EdrvInstance_l.m_pRxBuffer)
    {
        printk("Failed \n");
        iRet = -ENOMEM;
        goto Exit;
    }
    printk("Done \n");

    for (iLoop = 0; iLoop < EDRV_MAX_RX_DESCRIPTOR; iLoop++)
    {
        RxDmaAddr = dma_map_single(&pDev_p->dev,
                (EdrvInstance_l.m_pRxBuffer + (iLoop * EDRV_MAX_FRAME_SIZE)),
                EDRV_MAX_FRAME_SIZE, DMA_FROM_DEVICE);
        if (!RxDmaAddr)
        {
            printk("RxDmaAddr Allocation failed\n");
            iRet = -EIO;
            goto Exit;
        }
        EdrvInstance_l.m_apbRxBufInDesc[iLoop] = (EdrvInstance_l.m_pRxBuffer
                + (iLoop * EDRV_MAX_FRAME_SIZE));
        EdrvInstance_l.m_pRxDescAddr[iLoop].m_uiRxBufferAddr = RxDmaAddr;
        EdrvInstance_l.m_pRxDescAddr[iLoop].m_uiRxBufferAddr &=
                EDRV_RXBUF_CLEAR_USED_MASK;

        if ((EDRV_MAX_RX_DESCRIPTOR - 1) == iLoop)
        {
            EdrvInstance_l.m_pRxDescAddr[iLoop].m_uiRxBufferAddr |=
                    EDRV_RXBUF_WRAP_MASK;
        }
    }

    // Set Dma parameters 
    dwReg = 0;
    dwReg |= EDRV_DMA_BURST_MASK;
    dwReg |= EDRV_DMA_RXPKT_BUFSIZE_MASK;
    dwReg |= EDRV_DMA_TXPKT_BUFSIZE2_MASK;
    dwReg |= EDRV_DMA_RXBUFF_SIZE_MASK;

    EDRV_WRITE_REG(EDRV_DMACFG_REG, dwReg);

    dwReg = 0;
    dwReg = EDRV_READ_REG(EDRV_NET_CNTRL_REG);
    dwReg |= EDRV_NWCTRL_TXEN_MASK;     // Enable TX
    EDRV_WRITE_REG(EDRV_NET_CNTRL_REG, dwReg);

    // Set the TxQptr
    EDRV_WRITE_REG(EDRV_TXQBASE_REG, EdrvInstance_l.m_pTxDescDma);
    // Set the RxQptr
    EDRV_WRITE_REG(EDRV_RXQBASE_REG, EdrvInstance_l.m_pRxDescDma);

    EdrvInstance_l.m_pTxDescAddr = (tEdrvTxDesc *) EdrvInstance_l.m_pTxDescVirt;

    for (iLoop = 0; iLoop < EDRV_MAX_TX_DESCRIPTOR; iLoop++)
    {
        EDRV_DESC_WRITE(&EdrvInstance_l.m_pTxDescAddr[iLoop],
                EDRV_DESC_CNTRL_OFFSET, EDRV_DESC_USED_BIT_MASK);     //Set the used bit
        if (iLoop == (EDRV_MAX_TX_DESCRIPTOR - 1))
        {
            EDRV_DESC_WRITE(&EdrvInstance_l.m_pTxDescAddr[iLoop],
                    EDRV_DESC_CNTRL_OFFSET, EDRV_TX_DESC_WRAP_MASK);     //Wrap set for last desc
        }
    }

    // Initialize the Phy
    EdrvMacPsWrite(MARVELL_PHY_ADDR, PHY_CONTROL_REG_OFFSET,
            PHY_LINK_SPEED_100);
    wCtrl = EdrvMacPsRead(MARVELL_PHY_ADDR, PHY_CONTROL_REG_OFFSET);
    wCtrl |= PHY_RESET;        //PHY reset
    EdrvMacPsWrite(MARVELL_PHY_ADDR, PHY_CONTROL_REG_OFFSET, wCtrl);

    msleep(10);

    // Enable Rx Circuit
    dwReg = 0;
    dwReg = EDRV_READ_REG(EDRV_NET_CNTRL_REG);
    dwReg |= EDRV_NWCTLR_RXEN_MASK;
    EDRV_WRITE_REG(EDRV_NET_CNTRL_REG, dwReg);

    //enable Tx_used_bit read INTR
    EDRV_WRITE_REG(EDRV_INTR_EN_REG,
            (EDRV_TX_AHB_CORR_READ | EDRV_RX_COMPLETE_READ | EDRV_TX_COMPLETE_READ | EDRV_TX_BUFF_UNDRUN_READ | EDRV_TX_LATE_COL_READ | EDRV_RX_OVERUN_READ));

    Exit: return iRet;
}
//---------------------------------------------------------------------------
//
// Function:    EdrvShutdown

//
// Description: Shutdown the Ethernet controller
//
// Parameters:  void

//
// Returns:     Errorcode   = kEplSuccessful
//
// State:

//
//---------------------------------------------------------------------------
tEplKernel EdrvShutdown(void)
{
    printk("%s calling pci_unregister_driver()\n", __FUNCTION__);
    platform_driver_unregister(&EdrvDriver);
    return kEplSuccessful;
}
//---------------------------------------------------------------------------
//
// Function:    EdrvRemoveOne
//
// Description: shuts down platform device
//
// Parameters:  pDev_p             = pointer to corresponding platform device structure
//
// Returns:     (int)
//
// State:
//
//---------------------------------------------------------------------------
static int EdrvRemoveOne(struct platform_device *pDev_p)
{
    INT iLoop;

    if (pDev_p != EdrvInstance_l.m_pPltDev)
    {
        BUG_ON(EdrvInstance_l.m_pPltDev != pDev_p);
        goto Exit;
    }

    if (NULL != EdrvInstance_l.m_pTxBuffer)
    {
        kfree(EdrvInstance_l.m_pTxBuffer);
        EdrvInstance_l.m_pTxBuffer = NULL;
    }
    if (NULL != EdrvInstance_l.m_pRxBuffer)
    {
        kfree(EdrvInstance_l.m_pRxBuffer);
        EdrvInstance_l.m_pRxBuffer = NULL;
    }
    if (NULL != EdrvInstance_l.m_pTxDescVirt)
    {
        dma_free_coherent(&pDev_p->dev, EDRV_TX_DESCS_SIZE,
                EdrvInstance_l.m_pTxDescVirt, EdrvInstance_l.m_pTxDescDma);
        EdrvInstance_l.m_pTxDescVirt = NULL;
    }

    for (iLoop = 0; iLoop < EDRV_MAX_RX_DESCRIPTOR; iLoop++)
    {
        dma_unmap_single(&pDev_p->dev,
                EdrvInstance_l.m_pRxDescAddr[iLoop].m_uiRxBufferAddr,
                EDRV_MAX_FRAME_SIZE, DMA_FROM_DEVICE);
    }
    if (NULL != EdrvInstance_l.m_pRxDescVirt)
    {
        dma_free_coherent(&pDev_p->dev, EDRV_TX_DESCS_SIZE,
                EdrvInstance_l.m_pRxDescVirt, EdrvInstance_l.m_pRxDescDma);
        EdrvInstance_l.m_pRxDescVirt = NULL;
    }
    EDRV_WRITE_REG(EDRV_INTR_DIS_REG, ~0x0);
    free_irq(EdrvInstance_l.m_Irq, pDev_p);
    release_mem_region(EdrvInstance_l.m_MmIoAddr, EdrvInstance_l.m_MmIoSize);
    iounmap(EdrvInstance_l.m_pIoAddr);
    Exit: return 0;
}

//---------------------------------------------------------------------------
//
// Function:    EdrvGetBitFromMac
//
// Description: function returns the bit value at the bit position
//
//
// Parameters:  pbMAC_p - pointer to MAC address uiBitPos - bit position
//
// Returns:     value at the bit position
//
// State:       
//
//---------------------------------------------------------------------------
UINT EdrvGetBitFromMac(BYTE * pbMAC_p, UINT uiBitPos)
{
    BYTE    bMacAdd;
    UINT    uiBitVal;

    bMacAdd = (*(pbMAC_p + (uiBitPos / 8)));
    uiBitVal = ((bMacAdd >> (uiBitPos & 0x7)) & 0x01);
    return uiBitVal;
}
//---------------------------------------------------------------------------
//
// Function:    EdrvCalcHashAddr
//
// Description: function calculates the entry for the hash-table from MAC
//              address
//
//
// Parameters:  pbMAC_p - pointer to MAC address
//
// Returns:     hash value
//
// State:       
//
//---------------------------------------------------------------------------
unsigned int EdrvCalcHashAddr(BYTE * pbMAC_p)
{
    UINT    uiHashIndex = 0;
    INT     iLoop;
    for (iLoop = 0; iLoop <= 5; iLoop++)
    {
        uiHashIndex |= (EdrvGetBitFromMac(pbMAC_p, (0 + iLoop))
                ^ EdrvGetBitFromMac(pbMAC_p, (6 + iLoop))
                ^ EdrvGetBitFromMac(pbMAC_p, (12 + iLoop))
                ^ EdrvGetBitFromMac(pbMAC_p, (18 + iLoop))
                ^ EdrvGetBitFromMac(pbMAC_p, (24 + iLoop))
                ^ EdrvGetBitFromMac(pbMAC_p, (30 + iLoop))
                ^ EdrvGetBitFromMac(pbMAC_p, (36 + iLoop))
                ^ EdrvGetBitFromMac(pbMAC_p, (42 + iLoop))) << iLoop;

    }
    return uiHashIndex;
}

