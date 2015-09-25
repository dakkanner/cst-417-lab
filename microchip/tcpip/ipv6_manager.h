/*******************************************************************************
  IPv6 private manager API for Microchip TCP/IP Stack

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  ipv6_manager.h 
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

#ifndef _IPV6_MANAGER_H_
#define _IPV6_MANAGER_H_

// Stack structures

// IPv6 Pseudo header (needed for TCP and UDP checksum calculations/verification)
typedef struct
{
    IPV6_ADDR SourceAddress;
    IPV6_ADDR DestAddress;
    unsigned long PacketLength;
    unsigned short zero1;
    unsigned char zero2;
    unsigned char NextHeader;
} IPV6_PSEUDO_HEADER;

typedef struct
{
    IPV6_HEAP_NDP_DR_ENTRY * currentDefaultRouter;
    DOUBLE_LIST listIpv6UnicastAddresses;         // IPV6_ADDR_STRUCT list
    DOUBLE_LIST listIpv6MulticastAddresses;       // IPV6_ADDR_STRUCT list
    DOUBLE_LIST listIpv6TentativeAddresses;       // IPV6_ADDR_STRUCT list
    uint32_t baseReachableTime;
    uint32_t reachableTime;
    uint32_t retransmitTime;
    uint32_t mtuIncreaseTimer;
    uint16_t linkMTU;
    uint16_t multicastMTU;
    uint8_t curHopLimit;
    SINGLE_LIST listNeighborCache;                // IPV6_HEAP_NDP_NC_ENTRY list
    SINGLE_LIST listDefaultRouter;                // IPV6_HEAP_NDP_DR_ENTRY list
    SINGLE_LIST listDestinationCache;             // IPV6_HEAP_NDP_DC_ENTRY list
    SINGLE_LIST listPrefixList;                   // IPV6_HEAP_NDP_PL_ENTRY list
    SINGLE_LIST rxFragments;                      // IPV6_RX_FRAGMENT_BUFFER list
    uint8_t initState;
    uint8_t policyPreferTempOrPublic;
} IPV6_INTERFACE_CONFIG;



// stack private API
// 
   
bool TCPIP_IPV6_TaskPending (void);

void TCPIP_IPV6_Task (void);

bool TCPIP_IPV6_Initialize(const TCPIP_STACK_MODULE_CTRL* const pStackInit, const IPV6_MODULE_CONFIG* pIpv6Init);
void TCPIP_IPV6_Deinitialize(const TCPIP_STACK_MODULE_CTRL* const stackInit);
void TCPIP_IPV6_InitializeTask (void);
void TCPIP_IPV6_InitTmo (SYS_TICK curSysTick);
bool TCPIP_IPV6_InitTaskPending (void);
void TCPIP_IPV6_InitializeStop (TCPIP_NET_IF * pNetIf);

bool TCPIP_IPV6_Process (TCPIP_NET_IF * pNetIf, MAC_ADDR * remoteMAC);


// misc IP functions
// 


#define TCPIP_IPV6_IsTxReady(pNet)       MACIsTxReady(_TCPIPStackNetToMac(pNet))
void    TCPIP_IPV6_SetTxBuffer(TCPIP_NET_IF* pNet, uint16_t offset);
bool    TCPIP_IPV6_GetHeader(TCPIP_NET_IF* pNet, IPV6_ADDR * localIP, IPV6_ADDR * remoteIPAddr, uint8_t *protocol, uint16_t *len, uint8_t * hopLimit);
#define TCPIP_IPV6_DiscardRx(a)         MACDiscardRx(a)

#define TCPIP_IPV6_GetReadPtrInRx(n)     MACGetReadPtrInRx( _TCPIPStackNetToMac((TCPIP_NET_IF *) n))

#define TCPIP_IPV6_SetReadPtr(n, o)      MACSetReadPtr( _TCPIPStackNetToMac((TCPIP_NET_IF *) n), o)

#define TCPIP_IPV6_CalcRxChecksum(a,b,c) MACCalcRxChecksum(a,b,c)


bool TCPIP_IPV6_CopyTxPacketStruct (IPV6_PACKET * destination, IPV6_PACKET * source);

void TCPIP_IPV6_SetPacketIPProtocol (IPV6_PACKET * pkt);

// Adds the upper layer header (TCP/UDP/ICMP/ICMPv6) to the packet tx structure
IPV6_DATA_SEGMENT_HEADER * TCPIP_IPV6_PutUpperLayerHeader (IPV6_PACKET * pkt, void * header, unsigned short len, unsigned char type, unsigned short checksumOffset);

// Populate the IPV6_PACKET ptrIPHeader and upperLayerHeaderType fields.
// This is a change from the original TCPIP_IPV6_PutHeader function; the old function
// would also write the Ethernet and IP header to the MAC TX buffer, including
// the link-layer address.  Now the link-layer address is passed in in the TCPIP_IPV6_Flush
// function.  The length of remoteIPAddr is 4 or 16, depending on pkt's IP protocol
// type.
void TCPIP_IPV6_PutHeader(IPV6_PACKET * pkt, uint8_t protocol);

// Interface functions
// Gets the payload length of a TX packet
unsigned short TCPIP_IPV6_GetPayloadLength (IPV6_PACKET * pkt);

// Returns the pointer to the upper-layer header
void * TCPIP_IPV6_GetUpperLayerHeaderPtr(IPV6_PACKET * pkt);

#define TCPIP_IPV6_GetUpperLayerHeaderLen(s) (s->upperLayerHeaderLen)

// Calcualtes the 1's complement checksum over the packet payload
unsigned short TCPIP_IPV6_CalculatePayloadChecksum (IPV6_PACKET * pkt);

unsigned short TCPIP_IPV6_PutArrayHelper (IPV6_PACKET * pkt, const void * dataSource, uint8_t dataType, unsigned short len);
#define TCPIP_IPV6_PutRxData(mac,pkt,len)        TCPIP_IPV6_PutArrayHelper(pkt, &mac, IPV6_DATA_NETWORK_FIFO, len)

// Resets the IPV6_PACKET state associated with a socket after transmitting a packet so that it
// can be used to transmit additional packets.
void TCPIP_IPV6_ResetTransmitPacketState (IPV6_PACKET * pkt);

// Transmits a packet section
bool TCPIP_IPV6_TransmitPacket (IPV6_PACKET * pkt);

#define TCPIP_IPV6_IsPacketQueued(p)       (((IPV6_PACKET *)p)->flags.queued)

void TCPIP_IPV6_SingleListFree (void * list);


void TCPIP_IPV6_SetHopLimit(IPV6_PACKET * ptrPacket, uint8_t hopLimit);

void TCPIP_IPV6_SetRxBuffer(TCPIP_NET_IF* pNet, uint16_t Offset);

#define TCPIP_IPV6_GetHash(d,a,b)  ((((IPV6_ADDR *)d)->w[0] + ((IPV6_ADDR *)d)->w[1] + ((IPV6_ADDR *)d)->w[2] + ((IPV6_ADDR *)d)->w[3] + ((IPV6_ADDR *)d)->w[4] + ((IPV6_ADDR *)d)->w[5] + ((IPV6_ADDR *)d)->w[6] + ((IPV6_ADDR *)d)->w[7] + a) ^ b)


IPV6_ADDR_STRUCT *  TCPIP_IPV6_FindAddress(TCPIP_NET_IF * pNetIf, IPV6_ADDR * addr, unsigned char listType);
bool                TCPIP_IPV6_AddressIsSolicitedNodeMulticast (IPV6_ADDR * address);

IPV6_ADDRESS_TYPE TCPIP_IPV6_GetAddressType (TCPIP_NET_IF * pNetIf, IPV6_ADDR * address);

void TCPIP_IPV6_SendError (TCPIP_NET_IF * pNetIf, IPV6_ADDR * localIP, IPV6_ADDR * remoteIP, uint8_t code, uint8_t type, uint32_t additionalData, uint16_t packetLen);

void TCPIP_IPV6_NotifyClients(TCPIP_NET_IF* pNetIf, IPV6_EVENT_TYPE evType);

IPV6_INTERFACE_CONFIG* TCPIP_IPV6_GetInterfaceConfig(TCPIP_NET_IF* pNetIf);

void TCPIP_IPV6_DoubleListFree (void * list);


void TCPIP_IPV6_TmoHandler(SYS_TICK curSysTick);

#endif // _IPV6_MANAGER_H_



