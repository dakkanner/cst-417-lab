/*******************************************************************************
  Internet Control Message Protocol (ICMP) Server

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    - Provides "ping" diagnostics
    - Reference: RFC 792
*******************************************************************************/

/*******************************************************************************
FileName:   ICMP.c
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


#if defined(TCPIP_STACK_USE_IPV4)
#if defined(TCPIP_STACK_USE_ICMP_SERVER) || defined(TCPIP_STACK_USE_ICMP_CLIENT)

#define TCPIP_THIS_MODULE_ID    TCPIP_MODULE_ICMP
#include "tcpip_notify.h"

// ICMP Header Structure
typedef struct
{
    uint8_t vType;
    uint8_t vCode;
    uint16_t wChecksum;
    uint16_t wIdentifier;
    uint16_t wSequenceNumber;
} ICMP_HEADER;


static int          icmpInitCount = 0;      // ICMP module initialization count

static const void*      icmpMemH = 0;        // memory handle


#if defined(TCPIP_STACK_USE_ICMP_CLIENT)

// Callback function for informing the upper-layer protocols about ICMP events
typedef void (*icmpCallback) (TCPIP_NET_HANDLE hNetIf, IPV4_ADDR * remoteIP, void * data);

static SINGLE_LIST      icmpRegisteredUsers = { 0 };
//
// ICMP callback registration
typedef struct  _TAG_ICMP_LIST_NODE
{
	struct _TAG_ICMP_LIST_NODE* next;		// next node in list
                                            // makes it valid SGL_LIST_NODE node
    icmpCallback                callback;   // handler to be called for ICMP event
}ICMP_LIST_NODE;


// ICMP Packet Structure
typedef struct
{
	uint8_t vType;
	uint8_t vCode;
	uint16_t wChecksum;
	uint16_t wIdentifier;
	uint16_t wSequenceNumber;
	uint32_t wData;
} ICMP_PACKET;

#endif

// local prototypes
static void _ICMPAckPacket(void* pktHandle, bool sent, void* ackParam);
static IPV4_PACKET * _ICMPAllocateTxPacketStruct (TCPIP_NET_IF * pNetIf);

#if defined(TCPIP_STACK_USE_ICMP_CLIENT)
static void _ICMPNotifyClients(TCPIP_NET_HANDLE hNetIf, IPV4_ADDR * remoteIP, void * data);
#endif  // defined(TCPIP_STACK_USE_ICMP_CLIENT)


bool ICMPInitialize(const TCPIP_STACK_MODULE_CTRL* const stackCtrl, const ICMP_MODULE_GONFIG* const pIcmpInit)
{
    if(stackCtrl->stackAction == TCPIP_STACK_ACTION_IF_UP)
    {   // interface restart
        return true;
    }

    // stack start up
    if(icmpInitCount == 0)
    {   // first time we're run
        icmpMemH = stackCtrl->memH;
#if defined(TCPIP_STACK_USE_ICMP_CLIENT)
        SingleListInit(&icmpRegisteredUsers);
#endif  // defined(TCPIP_STACK_USE_ICMP_CLIENT)
    }

    // interface init
    // postpone packet allocation until the ICMPSendEchoRequest is called
    //

    icmpInitCount++;
    return true;
}


void ICMPDeinitialize(const TCPIP_STACK_MODULE_CTRL* const stackCtrl)
{
    // if(stackCtrl->stackAction == TCPIP_STACK_ACTION_DEINIT) // stack shut down
    // if(stackCtrl->stackAction == TCPIP_STACK_ACTION_IF_DOWN) // interface down

    // one way or another this interface is going down

    if(stackCtrl->stackAction == TCPIP_STACK_ACTION_DEINIT)
    {   // stack shut down
        if(icmpInitCount > 0)
        {   // we're up and running
            if(--icmpInitCount == 0)
            {   // all closed
                // release resources
#if defined(TCPIP_STACK_USE_ICMP_CLIENT)
                TCPIP_NotificationRemoveAll(&icmpRegisteredUsers, icmpMemH);
#endif  // defined(TCPIP_STACK_USE_ICMP_CLIENT)
                icmpMemH = 0;
            }
        }
    }

}

#if defined(TCPIP_STACK_USE_ICMP_CLIENT)
ICMP_HANDLE ICMPRegisterCallback (void (*callback)(TCPIP_NET_HANDLE hNetIf, IPV4_ADDR * remoteIP, void * data))
{
    if(icmpMemH)
    {
        ICMP_LIST_NODE* newNode = (ICMP_LIST_NODE*)TCPIP_NotificationAdd(&icmpRegisteredUsers, icmpMemH, sizeof(*newNode));
        if(newNode)
        {
            newNode->callback = callback;
            return newNode;
        }
    }

    return 0;
}


bool ICMPDeRegisterCallback(ICMP_HANDLE hIcmp)
{
    if(hIcmp && icmpMemH)
    {
        if(TCPIP_NotificationRemove((SGL_LIST_NODE*)hIcmp, &icmpRegisteredUsers, icmpMemH))
        {
            return true;
        }
    }

    return false;


}
#else
ICMP_HANDLE ICMPRegisterCallback (void (*callback)(TCPIP_NET_HANDLE hNetIf, IPV4_ADDR * remoteIP, void * data))
{
    return 0;
}

bool ICMPDeRegisterCallback(ICMP_HANDLE hIcmp)
{
    return false;
}

#endif  // defined(TCPIP_STACK_USE_ICMP_CLIENT)



static IPV4_PACKET * _ICMPAllocateTxPacketStruct (TCPIP_NET_IF * pNetIf)
{
    IPV4_PACKET * ptrPacket;

    ptrPacket = TCPIP_IPV4_AllocateTxPacket (pNetIf, _ICMPAckPacket, 0);

    if (ptrPacket == NULL)
    {
        return NULL;
    }

    if (!TCPIP_IPV4_PutUpperLayerHeader (ptrPacket, NULL, sizeof (ICMP_HEADER), IP_PROT_ICMP, IPV4_NO_UPPER_LAYER_CHECKSUM))
    {
        TCPIP_IPV4_FreePacket (ptrPacket);
        return NULL;
    }

    return ptrPacket;
}

// packet deallocation function
// packet was transmitted by the IP layer
static void _ICMPAckPacket(void* pktHandle, bool sent, void* ackParam)
{
    if(pktHandle)
    {
        TCPIP_IPV4_FreePacket((IPV4_PACKET*)pktHandle);
    }
}

#if defined (TCPIP_STACK_USE_ICMP_CLIENT)
bool ICMPSendEchoRequest (NODE_INFO * remoteNode, uint16_t sequenceNumber, uint16_t identifier)
{
    uint32_t data = 0x44332211;
    ICMP_HEADER *   pICMPHeader;
    TCPIP_NET_IF *  pNetIf;
    IPV4_PACKET *     pTxPkt;

    pNetIf = (TCPIP_NET_IF*)TCPIP_STACK_GetDefaultNet();
    if(!TCPIP_IPV4_IsTxReady(pNetIf))
    {
        return false;
    }

    if ((pTxPkt = _ICMPAllocateTxPacketStruct(pNetIf)) == 0)
    {   // failed to allocate a new packet
        return false;
    }


    if (TCPIP_IPV4_IsTxPutReady(pTxPkt, 4) < 4)
    {   // packet too small?
        TCPIP_IPV4_FreePacket(pTxPkt);
        return false;
    }

    pICMPHeader = TCPIP_IPV4_GetUpperLayerHeaderPtr (pTxPkt);

    pICMPHeader->vType = 0x08;  // 0x08: Echo (ping) request
    pICMPHeader->vCode = 0x00;
    pICMPHeader->wChecksum = 0x0000;
    pICMPHeader->wIdentifier = TCPIP_HELPER_htons(identifier);
    pICMPHeader->wSequenceNumber = TCPIP_HELPER_htons(sequenceNumber);

    TCPIP_IPV4_PutArray(pTxPkt, (uint8_t *)&data, 4);

    TCPIP_IPV4_SetDestAddress (pTxPkt, remoteNode->IPAddr.Val);

    pICMPHeader->wChecksum = TCPIP_IPV4_CalculatePayloadChecksum (pTxPkt);

	TCPIP_IPV4_PutHeader(pTxPkt, IP_PROT_ICMP);

	TCPIP_IPV4_Flush(pTxPkt, &remoteNode->MACAddr);

    return true;
}

#endif

/*********************************************************************
 * Function:        void ICMPProcess(TCPIP_NET_IF* pNetIf)
 *
 * PreCondition:    MAC buffer contains ICMP type packet.
 *
 * Input:           hMac: interfaceon which the request was received
 *                  *remote: Pointer to a NODE_INFO structure of the 
 *					ping requester
 *					len: Count of how many bytes the ping header and 
 *					payload are in this IP packet
 *
 * Output:          Generates an echo reply, if requested
 *					Validates and sets ICMPFlags.bReplyValid if a 
 *					correct ping response to one of ours is received.
 *
 * Side Effects:    None
 *
 * Overview:        None
 *
 * Note:            None
 ********************************************************************/
void ICMPProcess(TCPIP_NET_IF* pNetIf, NODE_INFO *remote, uint16_t len)
{
	TCPIP_UINT32_VAL dwVal;
    TCPIP_MAC_HANDLE hMac;
    ICMP_HEADER * headerPtr;
    int          netIx;     
    
    hMac = _TCPIPStackNetToMac(pNetIf);
    netIx = _TCPIPStackNetIx(pNetIf);
    
    // Obtain the ICMP header Type, Code, and Checksum fields
    TCPIP_IPV4_GetArray(hMac, (uint8_t*)&dwVal, sizeof(dwVal));
	
	// See if this is an ICMP echo (ping) request
	if(dwVal.w[0] == 0x0008u)
	{
        IPV4_PACKET *pTxPkt = 0;
        bool txSuccess = false;     

        while(true)
        {
            // Validate the checksum
            // The checksum data includes the precomputed checksum in the header
            // so a valid packet will always have a checksum of 0x0000
            if(TCPIP_IPV4_CalcRxChecksum(hMac, 0+sizeof(IPV4_HEADER), len))
            {
                break;
            }

            // Check that the TX hardware can actually transmit 
            if(!TCPIP_IPV4_IsTxReady(pNetIf))
            {
                break;
            }

            if ((pTxPkt = _ICMPAllocateTxPacketStruct(pNetIf)) == 0)
            {   // failed to allocate a new packet
                break;
            }

            headerPtr = (ICMP_HEADER *)TCPIP_IPV4_GetUpperLayerHeaderPtr(pTxPkt);

            // Calculate new Type, Code, and Checksum values
            headerPtr->vType = 0x00;	// Type: 0 (ICMP echo/ping reply)
            headerPtr->vCode = dwVal.v[1];
            dwVal.v[2] += 8;	// Subtract 0x0800 from the checksum
            if(dwVal.v[2] < 8u)
            {
                dwVal.v[3]++;
                if(dwVal.v[3] == 0u)
                    dwVal.v[2]++;
            }

            headerPtr->wChecksum = dwVal.w[1];

            TCPIP_IPV4_GetArray (hMac, (uint8_t *)&headerPtr->wIdentifier, 4);

            // Create IP header in TX memory
            TCPIP_IPV4_PutHeader(pTxPkt, IP_PROT_ICMP);

            TCPIP_IPV4_SetDestAddress (pTxPkt, remote->IPAddr.Val);

            if (TCPIP_IPV4_IsTxPutReady(pTxPkt, len - sizeof(ICMP_HEADER)) < len - sizeof(ICMP_HEADER))
            {   // packet too small?
                break;
            }

            // Copy ICMP response into the TX memory
            TCPIP_IPV4_PutRxData (hMac, pTxPkt, len - 8);

            // Transmit the echo reply packet
            TCPIP_IPV4_Flush(pTxPkt, &remote->MACAddr);
            txSuccess = true;
            break;
        }

        if(!txSuccess)
        {
			TCPIP_IPV4_DiscardRx(hMac);
            if(pTxPkt)
            {
                TCPIP_IPV4_FreePacket(pTxPkt);
            }
        }
	}
#if defined(TCPIP_STACK_USE_ICMP_CLIENT)
	else if(dwVal.w[0] == 0x0000u)	// See if this an ICMP Echo reply to our request
	{
		// Get the sequence number and identifier fields
		TCPIP_IPV4_GetArray(hMac, (uint8_t*)&dwVal, sizeof(dwVal));

		// Validate the ICMP checksum field
	    TCPIP_IPV4_SetRxBuffer(pNetIf, 0);
		if(MACCalcIPBufferChecksum(hMac, sizeof(ICMP_PACKET)))	// Two bytes of payload were sent in the echo request
			return;

		// Send a message to the application-level Ping driver that we've received an Echo Reply
        _ICMPNotifyClients(pNetIf, &remote->IPAddr, (void *)&dwVal);
	}
#endif
}

#if defined(TCPIP_STACK_USE_ICMP_CLIENT)

static void _ICMPNotifyClients(TCPIP_NET_HANDLE hNetIf, IPV4_ADDR * remoteIP, void * data)
{
    ICMP_LIST_NODE* dNode;

    for(dNode = (ICMP_LIST_NODE*)icmpRegisteredUsers.head; dNode != 0; dNode = dNode->next)
    {
        (*dNode->callback)(hNetIf, remoteIP, data);
    }

}

#endif //#if defined(TCPIP_STACK_USE_ICMP_CLIENT)

#endif //#if defined(TCPIP_STACK_USE_ICMP_SERVER) || defined(TCPIP_STACK_USE_ICMP_CLIENT)
#endif  // defined(TCPIP_STACK_USE_IPV4)


