/*******************************************************************************
  ENCx24J600 Driver Medium Access Control (MAC)

  Summary:
    Layer Module  for Microchip TCP/IP Stack
    
  Description:
    PIC32 implementation for multiple MAC support
*******************************************************************************/

/*******************************************************************************
FileName:   encx24_mac.c
Copyright ?2012 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED �AS IS?WITHOUT WARRANTY OF ANY KIND,
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
#include "tcpip/tcpip_mac_object.h"
#include "tcpip/encx24j600.h"
#include "encx24_mac.h"

// Compile only for ENCX24J60 MAC interface
#if defined(TCPIP_IF_ENCX24J600)


// Function proto declaration
static TCPIP_MAC_RES       ENCX24_MACClose(TCPIP_MAC_HANDLE hMac);
static int                 ENCX24_MACGetHeader(TCPIP_MAC_HANDLE hMac, MAC_ADDR *remote, uint16_t* type);
static void                ENCX24_MACSetReadPtrInRx(TCPIP_MAC_HANDLE hMac, uint16_t offset);
static TCPIP_MAC_PTR_TYPE  ENCX24_MACSetWritePtr(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_PTR_TYPE address);
static TCPIP_MAC_PTR_TYPE  ENCX24_MACSetReadPtr(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_PTR_TYPE address);
static uint16_t            ENCX24_MACGetArray(TCPIP_MAC_HANDLE hMac, uint8_t *val, uint16_t len);
static void                ENCX24_MACDiscardRx(TCPIP_MAC_HANDLE hMac);
static void                ENCX24_MACPutHeader(TCPIP_MAC_HANDLE hMac, MAC_ADDR *remote, uint16_t type, uint16_t dataLen);
static bool                ENCX24_MACIsTxReady(TCPIP_MAC_HANDLE hMac);
static void                ENCX24_MACPutArray(TCPIP_MAC_HANDLE hMac, uint8_t *val, uint16_t len);
static void                ENCX24_MACFlush(TCPIP_MAC_HANDLE hMac);
static bool                ENCX24_MACCheckLink(TCPIP_MAC_HANDLE hMac);
static TCPIP_MAC_PTR_TYPE  ENCX24_MACGetTxBaseAddr(TCPIP_MAC_HANDLE hMac);
static TCPIP_MAC_PTR_TYPE  ENCX24_MACGetReadPtrInRx(TCPIP_MAC_HANDLE hMac);
static uint16_t            ENCX24_MACCalcRxChecksum(TCPIP_MAC_HANDLE hMac, uint16_t offset, uint16_t len);
static uint16_t            ENCX24_MACCalcIPBufferChecksum(TCPIP_MAC_HANDLE hMac, uint16_t len);
static void                ENCX24_MACSetRXHashTableEntry(TCPIP_MAC_HANDLE hMac, MAC_ADDR DestMACAddr);
static bool                ENCX24_MACPowerMode(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_POWER_MODE pwrMode);
#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
static TCPIP_MAC_EVENT_RESULT ENCX24_MACEventSetNotifyEvents(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents);
static TCPIP_MAC_EVENT_RESULT ENCX24_MACEventClearNotifyEvents(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents);
static TCPIP_MAC_EVENT_RESULT ENCX24_MACEventAck(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents);
static TCPIP_EVENT        ENCX24_MACEventGetPending(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup);
static TCPIP_MAC_EVENT_RESULT ENCX24_MACEventSetNotifyHandler(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, pMacEventF eventHandler, void* hParam);
#endif  // (TCPIP_STACK_USE_EVENT_NOTIFICATION)

static void     _ENCX24_Deinitialize(ENCX24_MAC_DCPT* pDcpt );
    
// the PIC32 ENCX24J60 MAC descriptor
// no support for multiple instances
static const TCPIP_MAC_OBJECT _pic32_encx24_mac_obj = 
{
    ENCX24_MACClose,
    ENCX24_MACGetHeader,
    ENCX24_MACSetReadPtrInRx,
    ENCX24_MACSetWritePtr,
    ENCX24_MACGetReadPtrInRx,
    0,    
    ENCX24_MACSetReadPtr,
    ENCX24_MACGetArray,
    ENCX24_MACDiscardRx,
    ENCX24_MACPutHeader,
    ENCX24_MACIsTxReady,
    ENCX24_MACPutArray,
    ENCX24_MACFlush,
    ENCX24_MACCheckLink,
    ENCX24_MACGetTxBaseAddr,
    ENCX24_MACCalcRxChecksum,
    ENCX24_MACCalcIPBufferChecksum,
    ENCX24_MACSetRXHashTableEntry,
    ENCX24_MACPowerMode,
#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
    ENCX24_MACEventSetNotifyEvents,
    ENCX24_MACEventClearNotifyEvents,
    ENCX24_MACEventAck,
    ENCX24_MACEventGetPending,
    ENCX24_MACEventSetNotifyHandler,
#endif  // (TCPIP_STACK_USE_EVENT_NOTIFICATION)
};

// only one hardware instance for now!
static ENCX24_MAC_DCPT _pic32_encx24_mac_dcpt[1] = 
{
    {
        &_pic32_encx24_mac_obj,
        // specific ENCX24 MAC data 
        0,                                  // pNetIf
        0,									// _linkPrev
        0,                                  // isOpen
    }
};

////////////////////////////////////////////////////////////////////////////////
/*******************************************************************************
 *
 * interface functions
 *
*******************************************************************************/
////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
  Function:
	ENCX24_MACInitialize

  Summary:
	This function initializes the NIC, the MAC and the associated PHY.

  Description:
    This function initializes the NIC, the MAC and the associated PHY. It should
    be called to be able to schedule any Eth transmit or receive operation.

  Precondition:
	None

  Parameters:
	stackData - ponter to stack initialization data structure
	initData - ponter to MAC initialization data structure

  Returns:
  	TCPIP_MAC_RES
  	
  Remarks:
	Only one client per MAC supported..
*******************************************************************************/

TCPIP_MAC_RES ENCX24_MACInitialize(const TCPIP_STACK_MODULE_CTRL* const stackData,
                                    const TCPIP_MODULE_MAC_ENCJ600_CONFIG * const initData)
{
	ENCX24_MAC_DCPT* pDcpt;
	TCPIP_MAC_RES		res;

	if(stackData->pNetIf->macId != TCPIP_MODULE_MAC_ENCJ600)
	{
		return TCPIP_MAC_RES_TYPE_ERR;		// no other type supported
	}

	pDcpt = _pic32_encx24_mac_dcpt + 0; // no other instance supported
	if(pDcpt->isOpen != 0)
	{ 
		return TCPIP_MAC_RES_IS_BUSY;	  // have a client connected
	}

	pDcpt->pNetIf = stackData->pNetIf;
	res =  ENCX24J600_MACInit(pDcpt->pNetIf);

	if(res !=  TCPIP_MAC_RES_OK)
	{
		_ENCX24_Deinitialize(pDcpt);
	}

	return res;
}

/*******************************************************************************
  Function:
	ENCX24_MACDeinitialize

  Summary:
	This function de-initializes the Eth MAC controller.

  Description:
    This function de-initializes the Eth MAC controller.

  Precondition:
	None

  Parameters:
	stackData - ponter to stack initialization data structure

  Returns:
  	TCPIP_MAC_RES
  	
  Remarks:
	None
*******************************************************************************/

TCPIP_MAC_RES ENCX24_MACDeinitialize(const TCPIP_STACK_MODULE_CTRL* const stackData )
{
	
	if(stackData->pNetIf->macId != TCPIP_MODULE_MAC_ENCJ600)
	{
		return TCPIP_MAC_RES_TYPE_ERR;		// no other type supported
	}

	_ENCX24_Deinitialize(_pic32_encx24_mac_dcpt + 0); // no other instance supported

	return TCPIP_MAC_RES_OK;
}

/*******************************************************************************
  Function:
	ENCX24_MACOpen

  Summary:
	This function returns a client handle.

  Description:
    This function returns a client handle.

  Precondition:
	ENCX24_MACInitialize has been called

  Parameters:
	macId    - standard MAC ID

  Returns:
  	TCPIP_MAC_RES
  	
  Remarks:
	Currently only one client is supported 
*******************************************************************************/

TCPIP_MAC_HANDLE ENCX24_MACOpen( TCPIP_STACK_MODULE macId )
{    
	ENCX24_MAC_DCPT* pMacD;
    TCPIP_MAC_HANDLE    hMac = 0;

    if(macId == TCPIP_MODULE_MAC_ENCJ600)
    {
        pMacD = _pic32_encx24_mac_dcpt + 0; // no other instance supported
        if((pMacD->isOpen) == 0)
        {   // only one client for now
            pMacD->isOpen = 1;
            hMac = pMacD;
        }
    }
   
    return hMac;
}

 /*******************************************************************************
  Function:
	ENCX24_MACClose

  Summary:
	This function closes a client handle.

  Description:
    This function closes a client handle.

  Precondition:
	ENCX24_MACInitialize has been called

  Parameters:
	hMac - MAC handle

  Returns:
  	TCPIP_MAC_RES
  	
  Remarks:
	Currently only one client is supported 
*******************************************************************************/
static TCPIP_MAC_RES ENCX24_MACClose( TCPIP_MAC_HANDLE hMac )
{
	 ENCX24_MAC_DCPT* pMacD = (ENCX24_MAC_DCPT*)hMac;
	 
	 pMacD->isOpen = 0;
	 
	 return TCPIP_MAC_RES_OK;
}

/*******************************************************************************
  Function:
	ENCX24_MACGetTxBaseAddr

  Summary:
	This function returns the address of the current TX buffer

  Description:
	This function returns the address of the current TX buffer

  Precondition:
	None

  Parameters:
	hMac - MAC handle
	
  Returns:
	TX buffer base address
	
  Remarks:
	The returned value could be 0 if currently there's no available TX buffer. 
*******************************************************************************/

static TCPIP_MAC_PTR_TYPE ENCX24_MACGetTxBaseAddr(TCPIP_MAC_HANDLE hMac)
{
	return (TCPIP_MAC_PTR_TYPE)BASE_TX_ADDR;
}


static TCPIP_MAC_PTR_TYPE ENCX24_MACGetReadPtrInRx(TCPIP_MAC_HANDLE hMac)
{
    return ENCX24J600_MACGetReadPtrInRx(hMac);
}


////////////////////////////////////////////////////////////////////////////////
/*******************************************************************************
 *
 * TX functions
 *
*******************************************************************************/
////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
  Function:
	ENCX24_MACSetWritePtr

  Summary:
	This function sets the new write pointer.

  Description:
	This function sets the new write pointer.

  Precondition:
	None

  Parameters:
	hMac - MAC handle
    address - new address
	
  Returns:
	TX buffer size
	
  Remarks:
	None	
*******************************************************************************/

static TCPIP_MAC_PTR_TYPE ENCX24_MACSetWritePtr(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_PTR_TYPE address)
{
	return ENCX24J600_MACSetWritePtr(address);
}

/*******************************************************************************
  Function:
	bool ENCX24_MACIsTxReady(TCPIP_MAC_HANDLE hMac )

  Summary:
	Checks if there is an available current TX buffer.

  Description:
	Checks if there is an available current TX buffer.
	
  Precondition:
	None

  Parameters:
	hMac - MAC handle
	
  Returns:
	true: If data can be inserted in the current TX buffer
	false: there is no free TX buffer
	
  Remarks:
	None	
*******************************************************************************/

static bool ENCX24_MACIsTxReady(TCPIP_MAC_HANDLE hMac)
{
	return ENCX24J600_MACIsTxReady();
}

/*******************************************************************************
  Function:
	void ENCX24_MACPutArray(TCPIP_MAC_HANDLE hMac, uint8_t *val, uint16_t len)

  Summary:
	Writes a buffer to the current write location and updates the write pointer.
	
  Description:
	Writes a buffer to the current write location and updates the write pointer.
	
  Precondition:
	None

  Parameters:
	hMac - MAC handle
	buff - buffer to be written
	len - buffer length
	
  Returns:
	None
	
  Remarks:
	None	
*******************************************************************************/

static void ENCX24_MACPutArray(TCPIP_MAC_HANDLE hMac, uint8_t *val, uint16_t len)
{
	ENCX24J600_MACPutArray(val, len);
}

/*******************************************************************************
  Function:
	void ENCX24_MACPutHeader(TCPIP_MAC_HANDLE hMac, (MAC_ADDR *remote, uint16_t type, uint16_t dataLen)

  Summary:
	Sets the write pointer at the beginning of the current TX buffer
    and sets the ETH header and the frame length. Updates the write pointer
	
  Description:
	Sets the write pointer at the beginning of the current TX buffer
    and sets the ETH header and the frame length. Updates the write pointer
	
  Precondition:
	None

  Parameters:
	hMac - MAC handle
	remote - Pointer to memory which contains the destination MAC address (6 bytes)
    type - packet type: ETHERTYPE_IPV4/6, ETHERTYPE_ARP
	
  Returns:
	None
	
  Remarks:
	Assumes there is an available TX buffer, i.e. PIC32MACIsTxReady() returned !0	
*******************************************************************************/

static void ENCX24_MACPutHeader(TCPIP_MAC_HANDLE hMac, MAC_ADDR *remote, uint16_t type, uint16_t dataLen)
{
	ENCX24_MAC_DCPT* pDcpt;

	pDcpt = _pic32_encx24_mac_dcpt + 0; // no other instance supported

	ENCX24J600_MACPutHeader(pDcpt->pNetIf, remote, type, dataLen);
}

static void ENCX24_MACFlush(TCPIP_MAC_HANDLE hMac)
{
	ENCX24J600_MACFlush();
}

////////////////////////////////////////////////////////////////////////////////
/*******************************************************************************
 *
 * RX functions
 *
*******************************************************************************/
////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
  Function:
	void ENCX24_MACDiscardRx(TCPIP_MAC_HANDLE hMac )

  Summary:
	Marks the last received packet (obtained using ENCX24_MACGetHeader())as being 
	processed and frees the buffer memory associated with it. 
	It acknowledges the ETHC.
	
  Description:
	Marks the last received packet (obtained using ENCX24_MACGetHeader())as being 
	processed and frees the buffer memory associated with it. 
	It acknowledges the ETHC.
	
  Precondition:
	None

  Parameters:
	hMac - MAC handle
	
  Returns:
	None
	
  Remarks:
	Is is safe to call this function multiple times between
    ENCX24_MACGetHeader() calls.  Extra packets won't be thrown away 
    until ENCX24_MACGetHeader() makes it available.	
*******************************************************************************/

static void ENCX24_MACDiscardRx(TCPIP_MAC_HANDLE hMac)
{
	ENCX24J600_MACDiscardRx();
}

/*******************************************************************************
  Function:
	int ENCX24_MACGetHeader(TCPIP_MAC_HANDLE hMac, MAC_ADDR *remote, uint16_t* type)

  Summary:
	Checks if a new RX packet is available and returns the packet payload size
   (without the Ethernet frame header)
	
  Description:
	Checks if a new RX packet is available and returns the packet payload size
   (without the Ethernet frame header)
	
  Precondition:
	None

  Parameters:
	hMac - MAC handle
	*remote: Location to store the Source MAC address of the received frame.
	*type: Location of a uint16_t to store the constant ETHERTYPE_UNKNOWN, 
		   ETHERTYPE_IPVx, or ETHERTYPE_ARP, representing the contents of the 
		   Ethernet type field
	
  Returns:
	 !0: If a packet of this size is waiting in the RX buffer.  The remote, and 
		 type values are updated.
     0: If a packet was not pending.  remote and type are not changed.
	
  Remarks:
	Last packet is discarded if ENCX24_MACDiscardRx() hasn't already been called;
	Sets the read pointer at the beginning of the new packet.
*******************************************************************************/

static int ENCX24_MACGetHeader(TCPIP_MAC_HANDLE hMac, MAC_ADDR *remote, uint16_t* type)
{
	return ENCX24J600_MACGetHeader(remote, type);
}
	
/*******************************************************************************
  Function:
	void ENCX24_MACSetReadPtrInRx(TCPIP_MAC_HANDLE hMac, uint16_t offset)

  Summary:
	 The current read pointer is updated.  All calls to 
	 ENCX24_MACGetArray() will use these new values.
	
  Description:
	 The current read pointer is updated.  All calls to 
	 ENCX24_MACGetArray() will use these new values.
	
  Precondition:
	A packet has been obtained by calling ENCX24_MACGetHeader() and getting a true
	result.

  Parameters:
	hMac - MAC handle
	offset: uint16_t specifying how many bytes beyond the Ethernet header's type
	        field to relocate the read pointer.
	
  Returns:
	None
	
  Remarks:
	Last packet is discarded if ENCX24_MACDiscardRx() hasn't already been called;
	Sets the read pointer at the beginning of the new packet.
*******************************************************************************/

static void ENCX24_MACSetReadPtrInRx(TCPIP_MAC_HANDLE hMac, uint16_t offset)
{
	ENCX24J600_MACSetReadPtrInRx(offset);
}

/*******************************************************************************
  Function:
	ENCX24_MACSetReadPtr

  Summary:
	 This function sets the new read pointer value.
	
  Description:
	 This function sets the new read pointer value.
	
  Precondition:
	None

  Parameters:
	hMac - MAC handle
	address - new address to be used
	
  Returns:
	old read pointer
	
  Remarks:
	None
*******************************************************************************/

static TCPIP_MAC_PTR_TYPE ENCX24_MACSetReadPtr(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_PTR_TYPE address)
{
	return ENCX24J600_MACSetReadPtr(address);
}

/*******************************************************************************
  Function:
	ENCX24_MACGetArray(TCPIP_MAC_HANDLE hMac, uint8_t *address, uint16_t len)

  Summary:
	 Copies data in the supplied buffer.
	
  Description:
	 Copies data in the supplied buffer.
	
  Precondition:
	A valid packet should have been obtained or the read pointer properly set.

  Parameters:
	hMac - MAC handle
	*val: Pointer to storage location
    len:  Number of bytes to read from the data buffer.
	
  Returns:
	Byte read from the current read pointer location
	
  Remarks:
	The read pointer is updated
*******************************************************************************/

static uint16_t ENCX24_MACGetArray(TCPIP_MAC_HANDLE hMac, uint8_t *val, uint16_t len)
{
	return ENCX24J600_MACGetArray(val, len);
}

/*******************************************************************************
  Function:
	ENCX24_MACCalcIPBufferChecksum(TCPIP_MAC_HANDLE hMac, uint16_t len)

  Summary:
	 This function performs a checksum calculation of the buffer pointed by the 
	 current value of the read pointer.
	
  Description:
	 This function performs a checksum calculation of the buffer pointed by the 
	 current value of the read pointer.
	
  Precondition:
	Read buffer pointer set to starting of checksum data

  Parameters:
	hMac - MAC handle
	len: Total number of bytes to calculate the checksum over.
	
  Returns:
	16-bit checksum as defined by RFC 793
	
  Remarks:
	None
*******************************************************************************/

static uint16_t ENCX24_MACCalcIPBufferChecksum(TCPIP_MAC_HANDLE hMac, uint16_t len)
{
	return ENCX24J600_CalcIPBufferChecksum(len);
}

/*******************************************************************************
  Function:
	ENCX24_MACCalcRxChecksum(TCPIP_MAC_HANDLE hMac, uint16_t offset, uint16_t len)

  Summary:
	 This function performs a checksum calculation in the current receive buffer.
	
  Description:
	 This function performs a checksum calculation in the current receive buffer.
	
  Precondition:
	Read buffer pointer set to starting of checksum data

  Parameters:
	hMac - MAC handle
	offset  - Number of bytes beyond the beginning of the Ethernet data 
			(first byte after the type field)nwhere the checksum should begin
    len     - Total number of bytes to include in the checksum
	
  Returns:
	16-bit checksum as defined by RFC 793.
	
  Remarks:
	None
*******************************************************************************/

static uint16_t ENCX24_MACCalcRxChecksum(TCPIP_MAC_HANDLE hMac, uint16_t offset, uint16_t len)
{
	return ENCX24J600_MACCalcRxChecksum(offset, len);
}

/*******************************************************************************
  Function:
	bool ENCX24_MACCheckLink(TCPIP_MAC_HANDLE hMac)

  Summary:
	This function periodically checks the link statusperforming the MAC 
	reconfiguration if the link went up after being down.
	
  Description:
	This function periodically checks the link statusperforming the MAC 
	reconfiguration if the link went up after being down.
	
  Precondition:
	None
	
  Parameters:
	hMac - MAC handle
	
  Returns:
	None
	
  Remarks:
	If auto negotiation is enabled the MAC we may have to be reconfigured.
*******************************************************************************/

static bool ENCX24_MACCheckLink(TCPIP_MAC_HANDLE hMac)
{
    int	    linkCurr;
    ENCX24_MAC_DCPT* pMacD = (ENCX24_MAC_DCPT*)hMac;

    linkCurr = ENCX24J600_MACCheckLink();
    if(pMacD->_linkPrev != linkCurr)
    {   // PHY state changed 
		if (linkCurr == 1) {
		}

        // update the new stat
        pMacD->_linkPrev = linkCurr;
    }
    // else same old state
    // 
    return (linkCurr==1);

}

/*******************************************************************************
  Function:
	void ENCX24_MACSetRXHashTableEntry(TCPIP_MAC_HANDLE hMac, MAC_ADDR DestMACAddr)

  Summary:
	Calculates a CRC-32 using polynomial 0x4C11DB7 and then,using bits 28:23 of
	the CRC, sets the appropriate bit in the ETHHT0-ETHHT1 registers.
	
  Description:
	Calculates a CRC-32 using polynomial 0x4C11DB7 and then,using bits 28:23 of
	the CRC, sets the appropriate bit in the ETHHT0-ETHHT1 registers.
	
  Precondition:
	MACInitialize() should have been called.
	
  Parameters:
	hMac - MAC handle
	DestMACAddr: 6 byte group destination MAC address to allow through the Hash
			Table Filter.  If DestMACAddr is set to 00-00-00-00-00-00, then the 
			hash table will be cleared of all entries and the filter will be 
			disabled.
	
  Returns:
	Sets the appropriate bit in the ETHHT0/1 registers to allow packets sent to 
	DestMACAddr to be received and enabled the Hash Table receive filter (if not 
	already).
	
  Remarks:
	This code is commented out to save code space on systems that do not need 
	this function.  Change the "#if TCPIP_STACK_USE_ZEROCONF_MDNS_SD" line to "#if 1" 
	to uncomment it, assuming you aren't using the Zeroconf module, which 
	requires mutlicast support and enables this function automatically.
    There is no way to individually unset destination MAC addresses from the 
    hash table since it is possible to have a hash collision and therefore 
    multiple MAC addresses relying on the same hash table bit.  The stack would
    have to individually store each 6 byte MAC address to support this feature, 
    which would waste a lot of RAM and be unnecessary in most applications.  As 
    a simple compromise,you can call ENCX24_MACSetRXHashTableEntry() using a 
    00-00-00-00-00-00 destination MAC address, which will clear the entire hash 
    table and disable the hash table filter. This will allow you to then readd 
    the necessary destination addresses.
*******************************************************************************/

static void ENCX24_MACSetRXHashTableEntry(TCPIP_MAC_HANDLE hMac, MAC_ADDR DestMACAddr)
{
#if defined(TCPIP_STACK_USE_ZEROCONF_MDNS_SD)
	ENCX24J600_SetRXHashTableEntry(DestMACAddr);
#endif
}

static bool ENCX24_MACPowerMode(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_POWER_MODE pwrMode)
{
    switch(pwrMode)
    {
        case TCPIP_MAC_POWER_FULL:
            ENCX24J600_MACPowerUp();
            break;

        case TCPIP_MAC_POWER_DOWN:
            ENCX24J600_MACPowerDown();
            break;

        default:
            return false;
    }

    return true;
}

#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
static TCPIP_MAC_EVENT_RESULT ENCX24_MACEventSetNotifyEvents(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents)
{
	return TCPIP_MAC_EVRES_GROUP_ERR;
}
static TCPIP_MAC_EVENT_RESULT ENCX24_MACEventClearNotifyEvents(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents)
{
	return TCPIP_MAC_EVRES_GROUP_ERR;
}
static TCPIP_MAC_EVENT_RESULT ENCX24_MACEventAck(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents)
{
	return TCPIP_MAC_EVRES_GROUP_ERR;
}
static TCPIP_EVENT        	  ENCX24_MACEventGetPending(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup)
{
	return TCPIP_EV_NONE;
}
static TCPIP_MAC_EVENT_RESULT ENCX24_MACEventSetNotifyHandler(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, pMacEventF eventHandler, void* hParam)
{
	return TCPIP_MAC_EVRES_GROUP_ERR;
}
#endif  // (TCPIP_STACK_USE_EVENT_NOTIFICATION)

static void _ENCX24_Deinitialize(ENCX24_MAC_DCPT* pDcpt )
{
}

#endif

