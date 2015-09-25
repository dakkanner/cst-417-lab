/*******************************************************************************
  MAC Module (Microchip PIC32MX5-7) for Microchip TCP/IP Stack
*******************************************************************************/

/*******************************************************************************
FileName:   eth_pic32_int_mac.c
Copyright © 2012 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/

#include "tcpip_private.h"


#include "tcpip_mac_private.h"
#include "tcpip/tcpip_mac.h"
#include "tcpip/tcpip_mac_object.h"


// Compile only for PIC32MX with Ethernet MAC interface
#if ( ((__PIC32_FEATURE_SET__ >= 500) && (__PIC32_FEATURE_SET__ <= 799) && defined (_ETH)) && defined(TCPIP_IF_PIC32INT) )

// running on PIC32MX5-7 family with embedded ETHC

#include <peripheral/eth.h>

#include "tcpip/eth_pic32_ext_phy.h"

#include "eth_pic32_int_mac.h"

/** D E F I N I T I O N S ****************************************************/

#define TCPIP_THIS_MODULE_ID    TCPIP_MODULE_MAC_PIC32INT

#define ETHER_IP    (0x00u)
#define ETHER_ARP   (0x06u)



/******************************************************************************
 * Prototypes
 ******************************************************************************/
static void		_TxAckCallback(void* pPktBuff, int buffIx, void* fParam);			// Eth tx buffer acnowledge function
static bool		_LinkReconfigure(void);						// link reconfiguration

static void*    _MacCallocCallback( size_t nitems, size_t size, void* param );
static void     _MacFreeCallback(  void* ptr, void* param );

static void     _MACDeinit(PIC32_EMB_MAC_DCPT* pMacD );
static void     _MACCleanup(PIC32_EMB_MAC_DCPT* pMacD );


static    TCPIP_MAC_RES       PIC32MACClose( TCPIP_MAC_HANDLE hMac );
static    int                 PIC32MACGetHeader(TCPIP_MAC_HANDLE hMac, MAC_ADDR *remote, uint16_t* type);
static    void                PIC32MACSetReadPtrInRx(TCPIP_MAC_HANDLE hMac, uint16_t offset);
static    TCPIP_MAC_PTR_TYPE  PIC32MACSetWritePtr(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_PTR_TYPE address);
static    TCPIP_MAC_PTR_TYPE  PIC32MACGetReadPtrInRx(TCPIP_MAC_HANDLE hMac);
static    TCPIP_MAC_PTR_TYPE  PIC32MACSetReadPtr(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_PTR_TYPE address);
static    TCPIP_MAC_PTR_TYPE  PIC32MACSetBaseReadPtr(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_PTR_TYPE address);
static    uint16_t            PIC32MACGetArray(TCPIP_MAC_HANDLE hMac, uint8_t *val, uint16_t len);
static    void                PIC32MACDiscardRx(TCPIP_MAC_HANDLE hMac);
static    void                PIC32MACPutHeader(TCPIP_MAC_HANDLE hMac, MAC_ADDR *remote, uint16_t type, uint16_t dataLen);
static    bool                PIC32MACIsTxReady(TCPIP_MAC_HANDLE hMac);
static    void                PIC32MACPutArray(TCPIP_MAC_HANDLE hMac, uint8_t *val, uint16_t len);
static    void                PIC32MACFlush(TCPIP_MAC_HANDLE hMac);
static    bool                PIC32MACCheckLink(TCPIP_MAC_HANDLE hMac);
static    TCPIP_MAC_PTR_TYPE  PIC32MACGetTxBaseAddr(TCPIP_MAC_HANDLE hMac);
static    uint16_t            PIC32MACCalcRxChecksum(TCPIP_MAC_HANDLE hMac, uint16_t offset, uint16_t len);
static    uint16_t            PIC32MACCalcIPBufferChecksum(TCPIP_MAC_HANDLE hMac, uint16_t len);
static    void                PIC32MACSetRXHashTableEntry(TCPIP_MAC_HANDLE hMac, MAC_ADDR DestMACAddr);
static    bool                PIC32MACPowerMode(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_POWER_MODE pwrMode);

#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
TCPIP_MAC_EVENT_RESULT    PIC32MACEventInit(TCPIP_MAC_HANDLE hMac, int intPri, int intSubPri);
TCPIP_MAC_EVENT_RESULT    PIC32MACEventDeInit(TCPIP_MAC_HANDLE hMac);
TCPIP_MAC_EVENT_RESULT    PIC32MACEventSetNotifyEvents(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents);
TCPIP_MAC_EVENT_RESULT    PIC32MACEventClearNotifyEvents(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents);
TCPIP_MAC_EVENT_RESULT    PIC32MACEventAck(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents);
TCPIP_EVENT               PIC32MACEventGetPending(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup);
TCPIP_MAC_EVENT_RESULT    PIC32MACEventSetNotifyHandler(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, pMacEventF eventHandler, void* hParam);
#endif  // (TCPIP_STACK_USE_EVENT_NOTIFICATION)


/******************************************************************************
 * PIC32 MAC object implementation
 ******************************************************************************/


// the embedded PIC32 MAC object
static const TCPIP_MAC_OBJECT _pic32_emb_mac_obj = 
{
    PIC32MACClose,
    PIC32MACGetHeader,
    PIC32MACSetReadPtrInRx,
    PIC32MACSetWritePtr,
    PIC32MACGetReadPtrInRx,
    PIC32MACSetBaseReadPtr,
    PIC32MACSetReadPtr,
    PIC32MACGetArray,
    PIC32MACDiscardRx,
    PIC32MACPutHeader,
    PIC32MACIsTxReady,
    PIC32MACPutArray,
    PIC32MACFlush,
    PIC32MACCheckLink,
    PIC32MACGetTxBaseAddr,
    PIC32MACCalcRxChecksum,
    PIC32MACCalcIPBufferChecksum,
    PIC32MACSetRXHashTableEntry,
    PIC32MACPowerMode,
#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
    PIC32MACEventSetNotifyEvents,
    PIC32MACEventClearNotifyEvents,
    PIC32MACEventAck,
    PIC32MACEventGetPending,
    PIC32MACEventSetNotifyHandler,
#endif  // (TCPIP_STACK_USE_EVENT_NOTIFICATION)
};


// the embedded PIC32 MAC descriptor
// no support for multiple instances


// the embedded PIC32 MAC descriptor
// to support multiple instances
// create an array/list of MAC_DCPT structures
static PIC32_EMB_MAC_DCPT _pic32_emb_mac_dcpt[1] = 
{
    {
        &_pic32_emb_mac_obj,
        // specific PIC32 MAC data 
        {
            0,                                                                                  // _macFlags
        }
    }
};
static const TCPIP_MODULE_MAC_PIC32INT_CONFIG _pic32_emb_mac_DefaultConfig = 
{
    EMAC_TX_BUFF_SIZE,      // txBuffSize
    EMAC_TX_DESCRIPTORS,    // nTxDescriptors
    EMAC_RX_BUFF_SIZE,      // rxBuffSize
    EMAC_RX_DESCRIPTORS,    // nRxDescriptors
};

extern __inline__ int __attribute__((always_inline)) _PIC32MacIdToIx(TCPIP_STACK_MODULE macId)
{
    // update for multiple instances
    return (macId == TCPIP_MODULE_MAC_PIC32INT)?0:-1; 
}

/*
 * interface functions
 *
*/


/****************************************************************************
 * Function:        PIC32MACInitialize
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *                
 * Side Effects:    None
 *
 * Overview:        This function initializes the Eth controller, the MAC and the associated PHY.
 *                  It should be called to be able to schedule any Eth transmit or receive operation.
 *
 * Note:            Only one client per MAC supported. 
 *****************************************************************************/
TCPIP_MAC_RES PIC32MACInitialize(const TCPIP_STACK_MODULE_CTRL* const stackData,
                                 const TCPIP_MODULE_MAC_PIC32INT_CONFIG* initData )
{
    union
    {
        double      align;              // alignement
        uint8_t		addr[6];            // address itself
    }alignMacAddress;        // aligned MAC address

    int		ix;
	eEthRes		ethRes, phyInitRes;
	uint8_t		useFactMACAddr[6] = {0x00, 0x04, 0xa3, 0x00, 0x00, 0x00};		// to check if factory programmed MAC address needed
	uint8_t		unsetMACAddr[6] =   {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};		// not set MAC address

    unsigned char*  pRxBuff;
    void*           allocPtr;
    TCPIP_MAC_RES   initRes;
    int		        macIx;
    PIC32_EMB_MAC_DCPT* pMacD;


    macIx = _PIC32MacIdToIx(stackData->pNetIf->macId);

    if(macIx < 0 )
    {   
        return TCPIP_MAC_RES_TYPE_ERR;      // no such type supported
    }
   
    pMacD = _pic32_emb_mac_dcpt + macIx;
    
    if((pMacD->mData._macFlags & PIC32_MAC_FLAG_OPEN) != 0)
    { 
        return TCPIP_MAC_RES_IS_BUSY;     // have a client connected
    }

    if(stackData->memH == 0)
    {
        return TCPIP_MAC_RES_ALLOC_ERR;     // not possible without dynamic memory!        
    }
    

    // init the MAC object
    memset(&pMacD->mData, 0x0, sizeof(pMacD->mData)); 
    pMacD->mData._macIx = macIx;
	pMacD->mData._linkPrev = ETH_LINK_ST_DOWN;
    
#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
    pMacD->mData._macIntSrc = SYS_INT_Source(SYS_MODULE_ETH_1 + pMacD->mData._macIx); 
#endif  // defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
	
    // use initialization data
    pMacD->mData._AllocH = stackData->memH;

    if(initData == 0)
    {
        initData = &_pic32_emb_mac_DefaultConfig;
    }

    pMacD->mData._nTxDescriptors = initData->nTxDescriptors;
    pMacD->mData._TxBuffSize = ((initData->txBuffSize + sizeof(MAC_ETHERNET_HEADER) + sizeof(int)-1) / sizeof(int)) * sizeof(int);        // adjust size of the TX buffer

    pMacD->mData._nRxDescriptors = initData->nRxDescriptors;
    pMacD->mData._RxBuffSize = ((initData->rxBuffSize + 15) / 16) * 16;   // adjust size of the RX buffer

    
	while(1)
	{
		eEthLinkStat	linkStat;
		eEthOpenFlags	oFlags, linkFlags;
		eEthMacPauseType pauseType;
		eEthPhyCfgFlags cfgFlags;


		
	#ifdef PHY_RMII
		cfgFlags=ETH_PHY_CFG_RMII;
	#else
		cfgFlags=ETH_PHY_CFG_MII;		
	#endif
		
	#ifdef PHY_CONFIG_ALTERNATE
		cfgFlags|=ETH_PHY_CFG_ALTERNATE;
	#else
		cfgFlags|=ETH_PHY_CFG_DEFAULT;	
	#endif


	#if ETH_CFG_LINK
			oFlags=ETH_CFG_AUTO?ETH_OPEN_AUTO:0;
			oFlags|=ETH_CFG_10?ETH_OPEN_10:0;
			oFlags|=ETH_CFG_100?ETH_OPEN_100:0;
			oFlags|=ETH_CFG_HDUPLEX?ETH_OPEN_HDUPLEX:0;
			oFlags|=ETH_CFG_FDUPLEX?ETH_OPEN_FDUPLEX:0;
			if(ETH_CFG_AUTO_MDIX)
			{
				oFlags|=ETH_OPEN_MDIX_AUTO;
			}
			else
			{
				oFlags|=ETH_CFG_SWAP_MDIX?ETH_OPEN_MDIX_SWAP:ETH_OPEN_MDIX_NORM;
			}			
	#else
		oFlags= ETH_OPEN_DEFAULT;
	#endif // ETH_CFG_LINK

		
		pauseType=(oFlags&ETH_OPEN_FDUPLEX)?ETH_MAC_PAUSE_CPBL_MASK:ETH_MAC_PAUSE_TYPE_NONE;
		
		// start the initialization sequence	
		EthInit();

		phyInitRes=EthPhyInit(oFlags, cfgFlags, &linkFlags);
		
		// let the auto-negotiation (if any) take place
		// continue the initialization

    	// set the TX buffers
        allocPtr = TCPIP_HEAP_Calloc(pMacD->mData._AllocH, pMacD->mData._nTxDescriptors, sizeof(sEthTxDcpt) + pMacD->mData._TxBuffSize);
        if(allocPtr == 0)
        {
            initRes = TCPIP_MAC_RES_ALLOC_ERR;
            break;
        }
        else if(!IS_KVA(allocPtr))
        {   // only kernel space buffers accepted
            TCPIP_HEAP_Free(pMacD->mData._AllocH, allocPtr);
            initRes = TCPIP_MAC_RES_ALLOC_ERR;
            break;
        }

        if(IS_KVA0(allocPtr))
        {   // switch to k1
            pMacD->mData._macFlags |= PIC32_MAC_FLAG_K0_HEAP;
            allocPtr = KVA0_TO_KVA1(allocPtr);
        }

        pMacD->mData._TxDescriptors = (sEthTxDcpt*)allocPtr;
        pMacD->mData._pTxCurrDcpt = pMacD->mData._TxDescriptors;
        pMacD->mData._pNextTxDcpt = (sEthTxDcpt*)((char*)pMacD->mData._pTxCurrDcpt + sizeof(sEthTxDcpt) + pMacD->mData._TxBuffSize);
        pMacD->mData._pLastTxDcpt = (sEthTxDcpt*)((char*)pMacD->mData._TxDescriptors + pMacD->mData._nTxDescriptors * (sizeof(sEthTxDcpt) + pMacD->mData._TxBuffSize));

    	// set the RX buffers
        allocPtr = TCPIP_HEAP_Calloc(pMacD->mData._AllocH, pMacD->mData._nRxDescriptors, pMacD->mData._RxBuffSize);
        if(allocPtr == 0)
        {
            initRes = TCPIP_MAC_RES_ALLOC_ERR;
            break;
        }
        if((pMacD->mData._macFlags & PIC32_MAC_FLAG_K0_HEAP) != 0)
        {
            allocPtr = KVA0_TO_KVA1(allocPtr);
        }
        pMacD->mData._RxBuffers = (unsigned char*)allocPtr;

        
		EthRxFiltersClr(ETH_FILT_ALL_FILTERS);
		EthRxFiltersSet(ETH_FILT_CRC_ERR_REJECT|ETH_FILT_RUNT_REJECT|ETH_FILT_ME_UCAST_ACCEPT|ETH_FILT_MCAST_ACCEPT|ETH_FILT_BCAST_ACCEPT);

		// set the MAC address
        memcpy(alignMacAddress.addr, stackData->pNetIf->netMACAddr.v, sizeof(alignMacAddress.addr));
        if(memcmp(alignMacAddress.addr, useFactMACAddr, sizeof(useFactMACAddr))==0 || memcmp(alignMacAddress.addr, unsetMACAddr, sizeof(unsetMACAddr))==0 )
		{	// use the factory programmed address existent in the MAC
            unsigned short* pS=(unsigned short*)alignMacAddress.addr;        
            *pS++=EMACxSA2;
            *pS++=EMACxSA1;
            *pS=EMACxSA0;
            memcpy(stackData->pNetIf->netMACAddr.v, alignMacAddress.addr, sizeof(alignMacAddress.addr));
		}
        else
        {   // use the supplied address
			EthMACSetAddress(alignMacAddress.addr);                
        }
				
		if(EthDescriptorsPoolAdd(pMacD->mData._nTxDescriptors, ETH_DCPT_TYPE_TX, _MacCallocCallback, (void*)pMacD)!=pMacD->mData._nTxDescriptors)
		{
            initRes = TCPIP_MAC_RES_ALLOC_ERR;
            break;
		}

		if(EthDescriptorsPoolAdd(pMacD->mData._nRxDescriptors, ETH_DCPT_TYPE_RX, _MacCallocCallback, (void*)pMacD)!=pMacD->mData._nRxDescriptors)
		{
            initRes = TCPIP_MAC_RES_ALLOC_ERR;
            break;
		}

		EthRxSetBufferSize(pMacD->mData._RxBuffSize);

		// set the RX buffers as permanent receive buffers
        pRxBuff = pMacD->mData._RxBuffers;
		for(ix=0, ethRes=ETH_RES_OK; ix<pMacD->mData._nRxDescriptors && ethRes==ETH_RES_OK; ix++)
		{
			ethRes=EthRxBuffersAppend((void*)&pRxBuff, 1, ETH_BUFF_FLAG_RX_STICKY);
			pRxBuff += pMacD->mData._RxBuffSize;
		}

		if(ethRes!=ETH_RES_OK)
		{
            initRes = TCPIP_MAC_RES_ALLOC_ERR;
            break;
		}

		if(phyInitRes != ETH_RES_OK)
		{
            initRes = TCPIP_MAC_RES_PHY_INIT_FAIL;
            break;
		}

        // PHY was detected properly
        pMacD->mData._linkPresent=1;
        if(oFlags&ETH_OPEN_AUTO)
        {	// we'll just wait for the negotiation to be done
            pMacD->mData._linkNegotiation=1;	// performing the negotiation
            linkStat=_LinkReconfigure()?ETH_LINK_ST_UP:ETH_LINK_ST_DOWN;	// if negotiation not done yet we need to try it next time
        }
        else
        {	// no need of negotiation results; just update the MAC
            EthMACOpen(linkFlags, pauseType);
            linkStat = EthPhyGetLinkStatus(0) & ETH_LINK_ST_UP;
        }

        pMacD->mData._linkUpdTick = SYS_TICK_Get();		// the last time we performed the link read
        pMacD->mData._linkPrev = linkStat;


#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
        if(PIC32MACEventInit(pMacD, TCPIP_EVENT_IPL, TCPIP_EVENT_SIPL) != TCPIP_MAC_EVRES_OK)
        {
            initRes = TCPIP_MAC_RES_EVENT_INIT_FAIL;
            break;
        }
#endif
        
        initRes = TCPIP_MAC_RES_OK;
		break;
	}


    if(initRes != TCPIP_MAC_RES_OK)
    {
        _MACDeinit(pMacD);        
    }
    
	return initRes;

}


/****************************************************************************
 * Function:        PIC32MACDeinitialize
 *
 * PreCondition:    None
 *
 * Input:           stackData - standard stack initialization structure
 *
 * Output:          TCPIP_MAC_RES
 *                
 * Side Effects:    None
 *
 * Overview:        This function de-initializes the Eth MAC controller.
 *
 * Note:            None 
 *****************************************************************************/
TCPIP_MAC_RES PIC32MACDeinitialize(const TCPIP_STACK_MODULE_CTRL* const stackData )
{
    int                 macIx;

    macIx = _PIC32MacIdToIx(stackData->pNetIf->macId);

    if(macIx < 0 )
    {   
        return TCPIP_MAC_RES_TYPE_ERR;      // no such type supported
    }
   
    _MACDeinit(_pic32_emb_mac_dcpt + macIx);
    
    return TCPIP_MAC_RES_OK;
}

size_t PIC32MACGetConfig(TCPIP_STACK_MODULE modId, void* configBuff, size_t buffSize, size_t* pConfigSize)
{
    int macIx = _PIC32MacIdToIx(modId);

    if(pConfigSize)
    {
        *pConfigSize =  (macIx >= 0)?sizeof(TCPIP_MODULE_MAC_PIC32INT_CONFIG):0;
    }

    if(macIx >= 0)
    {
        if(configBuff && buffSize >= sizeof(TCPIP_MODULE_MAC_PIC32INT_CONFIG))
        {   // can copy the data
            TCPIP_MODULE_MAC_PIC32INT_CONFIG* pMacConfig = (TCPIP_MODULE_MAC_PIC32INT_CONFIG*)configBuff;
            PIC32_EMB_MAC_DCPT* pMacD = _pic32_emb_mac_dcpt + macIx;

            pMacConfig->txBuffSize = pMacD->mData._TxBuffSize;
            pMacConfig->nTxDescriptors = pMacD->mData._nTxDescriptors;
            pMacConfig->rxBuffSize = pMacD->mData._RxBuffSize;
            pMacConfig->nRxDescriptors = pMacD->mData._nRxDescriptors;

            return sizeof(TCPIP_MODULE_MAC_PIC32INT_CONFIG);
        }

        return 0;
    }

    return -1;  // unknown module
}

/****************************************************************************
 * Function:        PIC32MACOpen
 *
 * PreCondition:    PIC32MACInitialize has been called
 *
 * Input:           macId - standard MAC id
 *
 * Output:          TCPIP_MAC_RES
 *                
 * Side Effects:    None
 *
 * Overview:        This function returns a client handle.
 *
 * Note:            Currently only one client is supported 
 *****************************************************************************/
TCPIP_MAC_HANDLE PIC32MACOpen( TCPIP_STACK_MODULE macId )
{
    int                 macIx;
    PIC32_EMB_MAC_DCPT* pMacD;
    TCPIP_MAC_HANDLE    hMac = 0;

    macIx = _PIC32MacIdToIx(macId);

    if(macIx >= 0 )
    {   
        pMacD = _pic32_emb_mac_dcpt + macIx;
        if((pMacD->mData._macFlags & PIC32_MAC_FLAG_OPEN) == 0)
        {   // only one client for now
            pMacD->mData._macFlags |= PIC32_MAC_FLAG_OPEN;
            hMac = pMacD;
        }
    }
   
    return hMac;
}


/****************************************************************************
 * Function:        PIC32MACClose
 *
 * PreCondition:    PIC32MACInitialize has been called
 *
 * Input:           hMac - MAC handle
 *
 * Output:          TCPIP_MAC_RES
 *                
 * Side Effects:    None
 *
 * Overview:        This function closes a client handle.
 *
 * Note:            Currently only one client is supported 
 *****************************************************************************/
static TCPIP_MAC_RES PIC32MACClose( TCPIP_MAC_HANDLE hMac )
{
    PIC32_EMB_MAC_DCPT* pMacD;

    pMacD = (PIC32_EMB_MAC_DCPT*)hMac;
    pMacD->mData._macFlags &= ~PIC32_MAC_FLAG_OPEN;



    return TCPIP_MAC_RES_OK;
}


/****************************************************************************
 * Function:        PIC32MACGetTxBaseAddr
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          TX buffer base address
 *                
 * Side Effects:    None
 *
 * Overview:        This function returns the address of the current TX buffer.
 *
 * Note:            The returned value could be 0 if currently there's no available TX buffer. 
 *****************************************************************************/
static TCPIP_MAC_PTR_TYPE PIC32MACGetTxBaseAddr(TCPIP_MAC_HANDLE hMac )
{
    PIC32_EMB_MAC_DCPT*     pMacD = (PIC32_EMB_MAC_DCPT*)hMac;
	return pMacD->mData._pTxCurrDcpt?(TCPIP_MAC_PTR_TYPE)pMacD->mData._pTxCurrDcpt->dataBuff:0;
}

/**************************
 * TX functions
 ***********************************************/

/****************************************************************************
 * Function:        PIC32MACSetWritePtr
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          old write pointer
 *                
 * Side Effects:    None
 *
 * Overview:        This function sets the new write pointer.
 *
 * Note:            None
 *****************************************************************************/
static TCPIP_MAC_PTR_TYPE PIC32MACSetWritePtr(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_PTR_TYPE address)
{
	unsigned char* oldPtr;
    PIC32_EMB_MAC_DCPT*     pMacD = (PIC32_EMB_MAC_DCPT*)hMac;

	oldPtr=pMacD->mData._CurrWrPtr;
	pMacD->mData._CurrWrPtr=(unsigned char*)address;
	return (TCPIP_MAC_PTR_TYPE)oldPtr;
}


/******************************************************************************
 * Function:        bool PIC32MACIsTxReady(TCPIP_MAC_HANDLE hMac )
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          true: If data can be inserted in the current TX buffer
 *                  false: there is no free TX buffer
 *
 * Side Effects:    None
 *
 * Overview:        Checks if there is an available current TX buffer
 *
 * Note:            None
 *****************************************************************************/
static bool PIC32MACIsTxReady(TCPIP_MAC_HANDLE hMac )
{
    sEthTxDcpt*   p;
    PIC32_EMB_MAC_DCPT*     pMacD = (PIC32_EMB_MAC_DCPT*)hMac;
    
	EthTxAcknowledgeBuffer(0, _TxAckCallback, 0);		// acknowledge everything

	if(pMacD->mData._pTxCurrDcpt == 0)
	{
		for(p = pMacD->mData._pNextTxDcpt; p < pMacD->mData._pLastTxDcpt; )
		{			
			if(p->txBusy==0)
			{	// found a non busy descriptor
				pMacD->mData._pTxCurrDcpt = p;
				break;
			}
            p = (sEthTxDcpt*)((char*)p + sizeof(sEthTxDcpt) + pMacD->mData._TxBuffSize);
		}
        
		if(pMacD->mData._pTxCurrDcpt == 0)
		{
			for(p = pMacD->mData._TxDescriptors; p < pMacD->mData._pNextTxDcpt; )
			{
				if(p->txBusy==0)
				{	// found a non busy descriptor
					pMacD->mData._pTxCurrDcpt = p;
					break;
				}
                p = (sEthTxDcpt*)((char*)p + sizeof(sEthTxDcpt) + pMacD->mData._TxBuffSize);
			}
		}
        
        if( pMacD->mData._pTxCurrDcpt != 0)
        {
            pMacD->mData._pNextTxDcpt = (sEthTxDcpt*)((char*)pMacD->mData._pTxCurrDcpt + sizeof(sEthTxDcpt) + pMacD->mData._TxBuffSize);
        }
	}


	if( pMacD->mData._pTxCurrDcpt != 0)
    {
        return 1;
    }
    else
	{
		pMacD->mData._stackMgrTxNotReady++;
        return 0;
	}
	
}

/******************************************************************************
 * Function:        void PIC32MACPutArray(TCPIP_MAC_HANDLE hMac, uint8_t* buff, uint16_t len)
 *
 * PreCondition:    None
 *
 * Input:           buff - buffer to be written
 *                  len - buffer length
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Writes a buffer to the current write location and updates the write pointer. 
 *
 * Note:            None
 *****************************************************************************/
static void PIC32MACPutArray(TCPIP_MAC_HANDLE hMac, uint8_t *buff, uint16_t len)
{
    PIC32_EMB_MAC_DCPT*     pMacD = (PIC32_EMB_MAC_DCPT*)hMac;
	memcpy(pMacD->mData._CurrWrPtr, buff, len);
	pMacD->mData._CurrWrPtr+=len;
}


/******************************************************************************
 * Function:        void PIC32MACPutHeader(TCPIP_MAC_HANDLE hMac, (MAC_ADDR *remote, uint16_t type, uint16_t dataLen)
 *
 * PreCondition:    None
 *
 * Input:           remote - Pointer to memory which contains the destination MAC address (6 bytes)
 *                  type - packet type: ETHERTYPE_IPV4/6, ETHERTYPE_ARP
 *                  dataLen - ethernet frame payload
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:       Sets the write pointer at the beginning of the current TX buffer
 *                 and sets the ETH header and the frame length. Updates the write pointer
 *
 * Note:            Assumes there is an available TX buffer, i.e. PIC32MACIsTxReady() returned !0
 *****************************************************************************/
static void PIC32MACPutHeader(TCPIP_MAC_HANDLE hMac, MAC_ADDR *remote, uint16_t type, uint16_t dataLen)
{
    PIC32_EMB_MAC_DCPT*     pMacD = (PIC32_EMB_MAC_DCPT*)hMac;

	pMacD->mData._TxCurrSize=dataLen+sizeof(MAC_ETHERNET_HEADER);
	pMacD->mData._CurrWrPtr=(unsigned char*)pMacD->mData._pTxCurrDcpt->dataBuff;		// point at the beg of the buffer
    TCPIP_NET_IF* pNetIf;
       	
    pNetIf = _TCPIPStackMacToNet(hMac);

	memcpy(pMacD->mData._CurrWrPtr, remote, sizeof(*remote));
	pMacD->mData._CurrWrPtr+=sizeof(*remote);
	memcpy(pMacD->mData._CurrWrPtr, &pNetIf->netMACAddr, sizeof(pNetIf->netMACAddr));
	pMacD->mData._CurrWrPtr+=sizeof(pNetIf->netMACAddr);

	
	*pMacD->mData._CurrWrPtr++= (type >> 8)& 0xFF;
	*pMacD->mData._CurrWrPtr++= (type & 0xFF);
	
}



static void PIC32MACFlush(TCPIP_MAC_HANDLE hMac )
{
    PIC32_EMB_MAC_DCPT*     pMacD = (PIC32_EMB_MAC_DCPT*)hMac;

	if(pMacD->mData._pTxCurrDcpt && pMacD->mData._TxCurrSize)
	{	// there is a buffer to transmit
		pMacD->mData._pTxCurrDcpt->txBusy=1;	
		EthTxSendBuffer((void*)pMacD->mData._pTxCurrDcpt->dataBuff, pMacD->mData._TxCurrSize);
		// res should be ETH_RES_OK since we made sure we had a descriptor available
		// by the call to PIC32MACIsTxReady and the number of the buffers matches the number of descriptors
		pMacD->mData._pTxCurrDcpt=0;
		pMacD->mData._TxCurrSize=0;
	}
}

/**************************
 * RX functions
 ***********************************************/


/******************************************************************************
 * Function:        void PIC32MACDiscardRx(TCPIP_MAC_HANDLE hMac )
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Marks the last received packet (obtained using 
 *                  PIC32MACGetHeader())as being processed and frees the buffer
 *                  memory associated with it.
 *                  It acknowledges the ETHC.
 *
 * Note:            Is is safe to call this function multiple times between
 *                  PIC32MACGetHeader() calls.  Extra packets won't be thrown away 
 *                  until PIC32MACGetHeader() makes it available.
 *****************************************************************************/
static void PIC32MACDiscardRx(TCPIP_MAC_HANDLE hMac )
{
    PIC32_EMB_MAC_DCPT*     pMacD = (PIC32_EMB_MAC_DCPT*)hMac;

	if(pMacD->mData._pRxCurrBuff)
	{	// an already existing packet
		EthRxAcknowledgeBuffer(pMacD->mData._pRxCurrBuff, 0, 0);
		pMacD->mData._pRxCurrBuff=0;
        pMacD->mData._RxCurrSize=0; 
        pMacD->mData._CurrRdPtr = 0;   

		pMacD->mData._stackMgrRxDiscarded++;
	}	
}



/******************************************************************************
 * Function:        int PIC32MACGetHeader(TCPIP_MAC_HANDLE hMac, MAC_ADDR *remote, uint16_t* type)
 *
 * PreCondition:    None
 *
 * Input:           *remote: Location to store the Source MAC address of the
 *                           received frame.
 *                  *type: Location of a uint16_t to store the constant
 *                         ETHERTYPE_UNKNOWN, ETHERTYPE_IPVx, or ETHERTYPE_ARP, 
 *                         representing the contents of the Ethernet type
 *                         field.
 *
 * Output:          !0: If a packet of this size is waiting in the RX buffer.  The
 *                        remote, and type values are updated.
 *                  0: If a packet was not pending.  remote and type are
 *                         not changed.
 *
 * Side Effects:    Last packet is discarded if PIC32MACDiscardRx() hasn't already
 *                  been called.
 *
 * Overview:        Checks if a new RX packet is available and returns the packet payload size
 *                  (without the Ethernet frame header)
 *
 * Note:            Sets the read pointer at the beginning of the new packet
 *****************************************************************************/
static int PIC32MACGetHeader(TCPIP_MAC_HANDLE hMac, MAC_ADDR *remote, uint16_t * type)
{
	void*			pNewPkt;
	const sEthRxPktStat*	pRxPktStat;
	eEthRes			res;
    PIC32_EMB_MAC_DCPT*     pMacD = (PIC32_EMB_MAC_DCPT*)hMac;


	PIC32MACDiscardRx(hMac);		// discard/acknowledge the old RX buffer, if any
	
	res=EthRxGetBuffer(&pNewPkt, &pRxPktStat);
	
	if(res==ETH_RES_OK)
	{	// available packet; minimum check

		if(pRxPktStat->rxOk && !pRxPktStat->runtPkt && !pRxPktStat->crcError)
		{	// valid packet;
			TCPIP_UINT16_VAL newType;
			pMacD->mData._pRxCurrBuff=pNewPkt;
			pMacD->mData._RxCurrSize=pRxPktStat->rxBytes-sizeof(MAC_ETHERNET_HEADER);
			pMacD->mData._CurrRdPtr=pMacD->mData._pRxCurrBuff+sizeof(MAC_ETHERNET_HEADER);	// skip the packet header
			// set the packet type
			memcpy(remote, &((MAC_ETHERNET_HEADER*)pNewPkt)->SourceMACAddr, sizeof(*remote));
			*type = ETHERTYPE_UNKNOWN;
			newType=((MAC_ETHERNET_HEADER*)pNewPkt)->Type;
            newType.Val = TCPIP_HELPER_ntohs (newType.Val);
            if( (newType.Val == ETHERTYPE_IPV4) || (newType.Val == ETHERTYPE_IPV6) || 
                (newType.Val == ETHERTYPE_ARP) )
            {
                *type = newType.Val;
            }
			
			pMacD->mData._stackMgrRxOkPkts++;
		}
	}

	if(pMacD->mData._pRxCurrBuff==0 && pNewPkt)
	{	// failed packet, discard
		EthRxAcknowledgeBuffer(pNewPkt, 0, 0);
		pMacD->mData._stackMgrRxBadPkts++;
	}
		
	
	return pMacD->mData._RxCurrSize;
}



/******************************************************************************
 * Function:        void PIC32MACSetReadPtrInRx(TCPIP_MAC_HANDLE hMac, uint16_t offset)
 *
 * PreCondition:    A packet has been obtained by calling PIC32MACGetHeader() and
 *                  getting a true result.
 *
 * Input:           offset: uint16_t specifying how many bytes beyond the Ethernet
 *                          header's type field to relocate the read pointer.
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The current read pointer is updated.  All calls to
 *                  PIC32MACGetArray() will use these new values.
 *
 * Note:            
 ******************************************************************************/
static void PIC32MACSetReadPtrInRx(TCPIP_MAC_HANDLE hMac, uint16_t offset)
{
    PIC32_EMB_MAC_DCPT*     pMacD = (PIC32_EMB_MAC_DCPT*)hMac;
	pMacD->mData._CurrRdPtr=pMacD->mData._pRxCurrBuff+sizeof(MAC_ETHERNET_HEADER)+offset;
}


/****************************************************************************
 * Function:        PIC32MACSetReadPtr
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          old read pointer
 *                
 * Side Effects:    None
 *
 * Overview:        This function sets the new read pointer value.
 *
 * Note:            None
 *****************************************************************************/
static TCPIP_MAC_PTR_TYPE PIC32MACSetReadPtr(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_PTR_TYPE address)
{
	unsigned char* oldPtr;
    PIC32_EMB_MAC_DCPT*     pMacD = (PIC32_EMB_MAC_DCPT*)hMac;

	oldPtr=pMacD->mData._CurrRdPtr;
	pMacD->mData._CurrRdPtr=(unsigned char*)address;
	return (TCPIP_MAC_PTR_TYPE)oldPtr;
}

/****************************************************************************
 * Function:        PIC32MACSetBaseReadPtr
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          old read pointer
 *                
 * Side Effects:    None
 *
 * Overview:        This function sets the new base read pointer value.
 *
 * Note:            None
 *****************************************************************************/
static TCPIP_MAC_PTR_TYPE PIC32MACSetBaseReadPtr(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_PTR_TYPE address)
{
	unsigned char* oldPtr;
    PIC32_EMB_MAC_DCPT*     pMacD = (PIC32_EMB_MAC_DCPT*)hMac;

	oldPtr=pMacD->mData._pRxCurrBuff;
	pMacD->mData._pRxCurrBuff=(unsigned char*)address;
	return (TCPIP_MAC_PTR_TYPE)oldPtr;
}


/******************************************************************************
 * Function:        uint16_t PIC32MACGetArray(TCPIP_MAC_HANDLE hMac, uint8_t *address, uint16_t len)
 *
 * PreCondition:    A valid packet should have been obtained or the read pointer properly set.
 *
 * Input:           address: Pointer to storage location
 *                  len:  Number of bytes to read from the data buffer.
 *
 * Output:          number of bytes copied to the data buffer.
 *
 * Side Effects:    None
 *
 * Overview:        Copies data in the supplied buffer.
 *
 * Note:            The read pointer is updated
 *
 *****************************************************************************/
static uint16_t PIC32MACGetArray(TCPIP_MAC_HANDLE hMac, uint8_t *address, uint16_t len)
{
    PIC32_EMB_MAC_DCPT*     pMacD = (PIC32_EMB_MAC_DCPT*)hMac;

	if(address)
	{
		memcpy(address, pMacD->mData._CurrRdPtr, len);
	}

	pMacD->mData._CurrRdPtr+=len;
	return len;
}

static TCPIP_MAC_PTR_TYPE PIC32MACGetReadPtrInRx(TCPIP_MAC_HANDLE hMac)
{
    PIC32_EMB_MAC_DCPT*     pMacD = (PIC32_EMB_MAC_DCPT*)hMac;

    return (TCPIP_MAC_PTR_TYPE)pMacD->mData._CurrRdPtr;
}

/******************************************************************************
 * Function:        uint16_t PIC32MACCalcIPBufferChecksum(TCPIP_MAC_HANDLE hMac, uint16_t len)
 *
 * PreCondition:    Read buffer pointer set to starting of checksum data
 *
 * Input:           len: Total number of bytes to calculate the checksum over.
 *
 * Output:          16-bit checksum as defined by RFC 793
 *
 * Side Effects:    None
 *
 * Overview:        This function performs a checksum calculation of the buffer
 *                  pointed by the current value of the read pointer.
 *
 * Note:            None
 *****************************************************************************/
static uint16_t PIC32MACCalcIPBufferChecksum(TCPIP_MAC_HANDLE hMac, uint16_t len)
{
    PIC32_EMB_MAC_DCPT*     pMacD = (PIC32_EMB_MAC_DCPT*)hMac;
	return TCPIP_Helper_CalcIPChecksum(pMacD->mData._CurrRdPtr, len, 0);
}


/******************************************************************************
 * Function:        uint16_t PIC32MACCalcRxChecksum(TCPIP_MAC_HANDLE hMac, uint16_t offset, uint16_t len)
 *
 * PreCondition:    None
 *
 * Input:           offset  - Number of bytes beyond the beginning of the
 *                          Ethernet data (first byte after the type field)
 *                          where the checksum should begin
 *                  len     - Total number of bytes to include in the checksum
 *
 * Output:          16-bit checksum as defined by RFC 793.
 *
 * Side Effects:    None
 *
 * Overview:        This function performs a checksum calculation in the current receive buffer.
 *
 * Note:            None
 *****************************************************************************/
static uint16_t PIC32MACCalcRxChecksum(TCPIP_MAC_HANDLE hMac, uint16_t offset, uint16_t len)
{
    PIC32_EMB_MAC_DCPT*     pMacD = (PIC32_EMB_MAC_DCPT*)hMac;
	return TCPIP_Helper_CalcIPChecksum(pMacD->mData._pRxCurrBuff+sizeof(MAC_ETHERNET_HEADER)+offset, len, 0);
}

	// verify the link status

/****************************************************************************
 * Function:        PIC32MACCheckLink
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          true if the link is up
 *                  false otherwise
 *
 * Side Effects:    None
 *
 * Overview:        This function periodically checks the link status
 *                  performing the MAC reconfiguration if the link went up
 *                  after being down.
 *
 * Note:            If auto negotiation is enabled the MAC we may have to be reconfigured.
 *****************************************************************************/
static bool PIC32MACCheckLink(TCPIP_MAC_HANDLE hMac)
{
    eEthLinkStat	    linkCurr;
    PIC32_EMB_MAC_DCPT* pMacD = (PIC32_EMB_MAC_DCPT*)hMac;

    if(pMacD->mData._linkPresent == 0)
    {
        return false;
    }

    linkCurr = EthPhyGetLinkStatus(0) & ETH_LINK_ST_UP;
    if(pMacD->mData._linkPrev != linkCurr)
    {   // PHY state changed 

        if(linkCurr == ETH_LINK_ST_UP)
        {   // possibly link up now

            if(pMacD->mData._linkNegotiation)
            {	// if the auto-negotiation is turned on we have to re-negotiate
                // but the link is actually up only after the re-negotiation is done
                // if negotiation not done yet we need to try again
                linkCurr=_LinkReconfigure()?ETH_LINK_ST_UP:ETH_LINK_ST_DOWN;	
            }
        }
        // else link down, not much we can do about it


        // update the new stat
        pMacD->mData._linkPrev = linkCurr;
    }
    // else same old state
    // 
    return linkCurr == ETH_LINK_ST_UP;
}

    
/******************************************************************************
 * Function:        void PIC32MACSetRXHashTableEntry(TCPIP_MAC_HANDLE hMac, MAC_ADDR DestMACAddr)
 *
 * PreCondition:    MACInitialize() should have been called.
 *
 * Input:           DestMACAddr: 6 byte group destination MAC address to allow 
 *                  through the Hash Table Filter.  If DestMACAddr 
 *                  is set to 00-00-00-00-00-00, then the hash 
 *                  table will be cleared of all entries and the 
 *                  filter will be disabled.
 *
 * Output:          Sets the appropriate bit in the ETHHT0/1 registers to allow 
 *                  packets sent to DestMACAddr to be received and enabled the 
 *                  Hash Table receive filter (if not already).
 *
 * Side Effects:    None
 *
 * Overview:        Calculates a CRC-32 using polynomial 0x4C11DB7 and then,
 *                  using bits 28:23 of the CRC, sets the appropriate bit in 
 *                  the ETHHT0-ETHHT1 registers.
 *
 * Note:            This code is commented out to save code space on systems 
 *                  that do not need this function.  Change the 
 *                  "#if TCPIP_STACK_USE_ZEROCONF_MDNS_SD" line to "#if 1" to 
 *                  uncomment it, assuming you aren't using the Zeroconf module, 
 *                  which requires mutlicast support and enables this function 
 *                  automatically.
 *
 *                  There is no way to individually unset destination MAC 
 *                  addresses from the hash table since it is possible to have 
 *                  a hash collision and therefore multiple MAC addresses 
 *                  relying on the same hash table bit.  The stack would have 
 *                  to individually store each 6 byte MAC address to support 
 *                  this feature, which would waste a lot of RAM and be 
 *                  unnecessary in most applications.  As a simple compromise, 
 *                  you can call PIC32MACSetRXHashTableEntry() using a 
 *                  00-00-00-00-00-00 destination MAC address, which will clear 
 *                  the entire hash table and disable the hash table filter.  
 *                  This will allow you to then readd the necessary destination 
 *                  addresses.
 *****************************************************************************/
static void PIC32MACSetRXHashTableEntry(TCPIP_MAC_HANDLE hMac, MAC_ADDR DestMACAddr)
{
      volatile unsigned int*    pHTSet;
      uint8_t                      hVal;
      int                       i, j;
      TCPIP_UINT32_VAL                 crc = {0xFFFFFFFF};
      uint8_t                      nullMACAddr[6] =   {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

      // Clear the Hash Table bits and disable the Hash Table Filter if a special 
      // 00-00-00-00-00-00 destination MAC address is provided.
      if( memcmp(DestMACAddr.v, nullMACAddr, sizeof(nullMACAddr))==0 )
      {
            // Disable the Hash Table receive filter and clear the hash table
            EthRxFiltersClr(ETH_FILT_HTBL_ACCEPT);
            EthRxFiltersHTSet(0ull);
            return;
      }
 
 
      // Calculate a CRC-32 over the 6 byte MAC address 
      // using polynomial 0x4C11DB7
      for(i = 0; i < sizeof(MAC_ADDR); i++)
      {
            uint8_t  crcnext;
      
            // shift in 8 bits
            for(j = 0; j < 8; j++)
            {
                  crcnext = 0;
                  if(((TCPIP_UINT8_VAL*)&(crc.v[3]))->bits.b7)
                        crcnext = 1;
                  crcnext ^= (((TCPIP_UINT8_VAL*)&DestMACAddr.v[i])->bits.b0);
      
                  crc.Val <<= 1;
                  if(crcnext)
                        crc.Val ^= 0x4C11DB7;
                  // next bit
                  DestMACAddr.v[i] >>= 1;
            }
      }
      
      // CRC-32 calculated, now extract bits 28:23
      // Bit 28 defines what HT register is affected: ETHHT0 or ETHHT1
      // Bits 27:23 define the bit offset within the ETHHT register
      pHTSet = (crc.bits.b28)? &ETHHT1SET : &ETHHT0SET;
      hVal = (crc.Val >> 23)&0x1f;
      *pHTSet = 1 << hVal;
      
      // Enable that the Hash Table receive filter
      EthRxFiltersSet(ETH_FILT_HTBL_ACCEPT);
      
}

static bool PIC32MACPowerMode(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_POWER_MODE pwrMode)
{   // not supported
    return true;
}

/**************************
 * local functions and helpers
 ***********************************************/

/*********************************************************************
* Function:        void	_TxAckCallback(void* pPktBuff, int buffIx, void* fParam)
 *
 * PreCondition:    None
 * 
 * Input:           pPktBuff - tx buffer to be acknowledged
 *                  buffIx   - buffer index, when packet spans multiple buffers
 *                  fParam   - optional parameter specified when EthTxAcknowledgeBuffer() called 
 * 
 * Output:          None
 * 
 * Side Effects:    None
 * 
 * Overview:        TX acknowledge call back function.
 *                  Called by the Eth MAC when TX buffers are acknoledged (as a result of a call to EthTxAckBuffer).
 * 
 * Note:            None
 ********************************************************************/
static void	_TxAckCallback(void* pPktBuff, int buffIx, void* fParam)
{
	volatile sEthTxDcpt*	pDcpt;

	pDcpt=(sEthTxDcpt*)((char*)pPktBuff-offsetof(sEthTxDcpt, dataBuff));

	pDcpt->txBusy=0;

}

/*********************************************************************
* Function:        void* _MacCallocCallback( size_t nitems, size_t size, void* param )
 *
 * PreCondition:    None
 * 
 * Input:           nitems - number of items to be allocated
 *                  size   - size of each item
 *                  param  - optional parameter specified when EthDescriptorsPoolAdd() called 
 * 
 * Output:          pointer to the allocated memory of NULL if allocation failed
 * 
 * Side Effects:    None
 * 
 * Overview:        Memory allocation callback.
 * 
 * Note:            None
 ********************************************************************/
static void* _MacCallocCallback( size_t nitems, size_t size, void* param )
{
    PIC32_EMB_MAC_DCPT*     pMacD = (PIC32_EMB_MAC_DCPT*)param;
    void* allocPtr = TCPIP_HEAP_Calloc(pMacD->mData._AllocH, nitems, size);
    if(allocPtr != 0 && (pMacD->mData._macFlags & PIC32_MAC_FLAG_K0_HEAP) != 0)
    {
        allocPtr = KVA0_TO_KVA1(allocPtr);
    }

    return allocPtr;
}



/*********************************************************************
* Function:        void _MacFreeCallback(  void* ptr, void* param )
 *
 * PreCondition:    None
 * 
 * Input:           ptr -  pointer to be freed
 *                  param  - optional parameter specified when EthDescriptorsPoolCleanUp() called 
 * 
 * Output:          None
 * 
 * Side Effects:    None
 * 
 * Overview:        Memory free callback.
 * 
 * Note:            None
 ********************************************************************/
static void _MacFreeCallback(  void* ptr, void* param )
{
    if(ptr)
    {
        PIC32_EMB_MAC_DCPT*     pMacD = (PIC32_EMB_MAC_DCPT*)param;
        if((pMacD->mData._macFlags & PIC32_MAC_FLAG_K0_HEAP) != 0)
        {
            ptr = KVA1_TO_KVA0(ptr);
        }
        TCPIP_HEAP_Free( pMacD->mData._AllocH, ptr);
    }
}


/*********************************************************************
* Function:        bool	_LinkReconfigure(void)
 *
 * PreCondition:    None
 * 
 * Input:           None
 * 
 * Output:          true if negotiation succeeded and MAC was updated
 *                  false otherwise
 * 
 * Side Effects:    None
 * 
 * Overview:        Performs re-configuration after auto-negotiation performed.
 * 
 * Note:            None
 ********************************************************************/
static bool _LinkReconfigure(void)
{

	eEthOpenFlags	linkFlags;
	eEthLinkStat	linkStat;
	eEthMacPauseType pauseType;
	eEthRes		phyRes;
	bool		success = false;


	phyRes=EthPhyNegotiationComplete(0);	// see if negotiation complete
	if(phyRes==ETH_RES_OK)
	{	
		linkStat=EthPhyGetNegotiationResult(&linkFlags, &pauseType);
		if(linkStat&ETH_LINK_ST_UP)
		{	// negotiation succeeded; properly update the MAC
            linkFlags|=(EthPhyGetHwConfigFlags()&ETH_PHY_CFG_RMII)?ETH_OPEN_RMII:ETH_OPEN_MII;                       
			EthMACOpen(linkFlags, pauseType);
			success = true;
		}
	}

	return success;
}

static void _MACDeinit(PIC32_EMB_MAC_DCPT* pMacD )
{
    _MACCleanup(pMacD);
    
    EthClose(ETH_CLOSE_DEFAULT);
#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
    PIC32MACEventDeInit(pMacD);
#endif

    pMacD->mData._macFlags = 0;
    
}

static void _MACCleanup(PIC32_EMB_MAC_DCPT* pMacD )
{
    
    if(pMacD->mData._AllocH)
    {
        void* allocPtr;
        EthDescriptorsPoolCleanUp(ETH_DCPT_TYPE_ALL,  _MacFreeCallback, (void*)pMacD);
        if(pMacD->mData._RxBuffers)
        {
            if((pMacD->mData._macFlags & PIC32_MAC_FLAG_K0_HEAP) != 0)
            {
                allocPtr = KVA1_TO_KVA0(pMacD->mData._RxBuffers);
            }
            else
            {
                allocPtr = (void*)pMacD->mData._RxBuffers;
            }

            TCPIP_HEAP_Free( pMacD->mData._AllocH, allocPtr);
            pMacD->mData._RxBuffers = 0;
        }
        if(pMacD->mData._TxDescriptors)
        {
            if((pMacD->mData._macFlags & PIC32_MAC_FLAG_K0_HEAP) != 0)
            {
                allocPtr = KVA1_TO_KVA0(pMacD->mData._TxDescriptors);
            }
            else
            {
                allocPtr = (void*)pMacD->mData._TxDescriptors;
            }

            TCPIP_HEAP_Free( pMacD->mData._AllocH, allocPtr);
            pMacD->mData._TxDescriptors = 0;
        }
    }
}


#endif  // ( ((__PIC32_FEATURE_SET__ >= 500) && (__PIC32_FEATURE_SET__ <= 799) && defined (_ETH)) && defined(TCPIP_IF_PIC32INT) )


