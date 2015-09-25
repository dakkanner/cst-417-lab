/*******************************************************************************
  IPv6 private API for Microchip TCP/IP Stack

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  ipv6_private.h 
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

#ifndef _IPV6_PRIVATE_H_
#define _IPV6_PRIVATE_H_

#define ETHERTYPE_IPV4      	(0x0800u)
#define ETHERTYPE_IPV6          (0x86DDu)

// misc IP functions
// 



// Allocate a data segment header
IPV6_DATA_SEGMENT_HEADER * TCPIP_IPV6_AllocateDataSegmentHeader (uint16_t len);

// Gets the IP-layer pseudo-header checksum
unsigned short TCPIP_IPV6_GetPseudoHeaderChecksum (IPV6_PACKET * pkt);

void TCPIP_IPV6_InsertIPPacketSegment (IPV6_DATA_SEGMENT_HEADER * ptrSegment, IPV6_PACKET * ptrPacket, IPV6_SEGMENT_TYPE);
IPV6_DATA_SEGMENT_HEADER * TCPIP_IPV6_GetDataSegmentByType (IPV6_PACKET * ptrPacket, IPV6_SEGMENT_TYPE type);
void * TCPIP_IPV6_GetDataSegmentContentsByType (IPV6_PACKET * ptrPacket, IPV6_SEGMENT_TYPE type);

// Flush the chain of data segments to the MAC TX buffer.
void TCPIP_IPV6_FlushDataSegments (TCPIP_MAC_HANDLE hMac, IPV6_PACKET * pkt);

// Deallocate all of the data segment headers and dynamically allocated payload segments in an IPV6_PACKET payload
void TCPIP_IPV6_FreePacketData (IPV6_PACKET * ptrPacket);
// Free a fragmented packet's reassembly buffer
void TCPIP_IPV6_FreeFragmentBuffer (void * ptrFragment);


    // IPV6 event registration
    
    typedef struct  _TAG_IPV6_LIST_NODE
    {
    	struct _TAG_IPV6_LIST_NODE*		next;		// next node in list
                                                    // makes it valid SGL_LIST_NODE node
        IPV6_EVENT_HANDLER              handler;    // handler to be called for event
        const void*                     hParam;     // handler parameter
        TCPIP_NET_HANDLE                hNet;       // interface that's registered for
                                                    // 0 if all    
    }IPV6_LIST_NODE;
    
    #define TCPIP_IPV6_GetOptionHeader(h,data, count)   MACGetArray (h,(unsigned char *)data, count << 3)
    uint8_t TCPIP_IPV6_ProcessHopByHopOptionsHeader (TCPIP_NET_IF * pNetIf, uint8_t * nextHeader, uint16_t * length);
    uint8_t TCPIP_IPV6_ProcessDestinationOptionsHeader (TCPIP_NET_IF * pNetIf, uint8_t * nextHeader, uint16_t * length);
    uint8_t TCPIP_IPV6_ProcessRoutingHeader (TCPIP_NET_IF * pNetIf, uint8_t * nextHeader, uint16_t * length);
    uint8_t TCPIP_IPV6_ProcessFragmentationHeader (TCPIP_NET_IF * pNetIf, IPV6_ADDR * source, IPV6_ADDR * dest, uint8_t * nextHeader, uint16_t dataCount, uint16_t headerLen, MAC_ADDR * remoteMACAddr, uint16_t previousHeader);
        
    IPV6_ADDR_STRUCT *  TCPIP_IPV6_FindSolicitedNodeMulticastAddress(TCPIP_NET_IF * pNetIf, IPV6_ADDR * addr, unsigned char listType);

        
    #define IPV6_ADDR_POLICY_TABLE_LEN      (sizeof (gPolicyTable) / sizeof (IPV6_ADDRESS_POLICY))
    unsigned char TCPIP_IPV6_DAS_GetPolicy (const IPV6_ADDR * addr, unsigned char * label, unsigned char * precedence, unsigned char * prefixLen);

    unsigned short TCPIP_IPV6_PutArrayHelper (IPV6_PACKET * pkt, const void * dataSource, uint8_t dataType, unsigned short len);
    #define TCPIP_IPV6_PutRxData(mac,pkt,len)        TCPIP_IPV6_PutArrayHelper(pkt, &mac, IPV6_DATA_NETWORK_FIFO, len)
        
    void TCPIP_IPV6_FragmentTask (void);
    void TCPIP_IPV6_UpdateTimestampsTask (void);

    bool TCPIP_IPV6_TransmitPacketInFragments (IPV6_PACKET * pkt, uint16_t mtu);


#endif // _IPV6_PRIVATE_H_



