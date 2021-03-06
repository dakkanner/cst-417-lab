/*******************************************************************************
  IPV4 private manager API for Microchip TCP/IP Stack

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  ipv4_manager.h 
Copyright � 2012 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED �AS IS� WITHOUT WARRANTY OF ANY KIND,
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

#ifndef _IPV4_MANAGER_H_
#define _IPV4_MANAGER_H_

// Stack structures

// IP Pseudo header as defined by RFC 793 (needed for TCP and UDP 
// checksum calculations/verification)
typedef struct
{
    IPV4_ADDR SourceAddress;
    IPV4_ADDR DestAddress;
    uint8_t Zero;
    uint8_t Protocol;
    uint16_t Length;
} IPV4_PSEUDO_HEADER;


// stack private API
// 
   
bool TCPIP_IPV4_TaskPending (void);

void TCPIP_IPV4_Task (void);

// misc IP functions
// 

#define TCPIP_IPV4_SwapPseudoHeader(h)  (h.Length = TCPIP_HELPER_htons(h.Length))

#define TCPIP_IPV4_IsTxReady(pNet)       MACIsTxReady(_TCPIPStackNetToMac(pNet))
void    TCPIP_IPV4_SetTxBuffer(TCPIP_NET_IF* pNet, uint16_t offset);
bool    TCPIP_IPV4_GetHeader(TCPIP_NET_IF* pNet, IPV4_ADDR *localIP, NODE_INFO *remote, uint8_t *protocol, uint16_t *len);
#define TCPIP_IPV4_DiscardRx(a)         MACDiscardRx(a)

#define TCPIP_IPV4_SetReadPtr(n, o)      MACSetReadPtr( _TCPIPStackNetToMac((TCPIP_NET_IF *) n), o)

#define TCPIP_IPV4_CalcRxChecksum(a,b,c) MACCalcRxChecksum(a,b,c)

bool TCPIP_IPV4_Initialize(const TCPIP_STACK_MODULE_CTRL* const stackInit, const IPV4_MODULE_CONFIG* pIpInit);
void TCPIP_IPV4_DeInitialize(const TCPIP_STACK_MODULE_CTRL* const stackInit);

void TCPIP_IPV4_SetRxBuffer(TCPIP_NET_IF* pNet, uint16_t Offset);

bool TCPIP_IPV4_CopyTxPacketStruct (IPV4_PACKET * destination, IPV4_PACKET * source);

void TCPIP_IPV4_SetPacketIPProtocol (IPV4_PACKET * pkt);

// Adds the upper layer header (TCP/UDP/ICMP) to the packet tx structure
IPV4_DATA_SEGMENT_HEADER * TCPIP_IPV4_PutUpperLayerHeader (IPV4_PACKET * pkt, void * header, unsigned short len, unsigned char type, unsigned short checksumOffset);

// Populate the IPV4_PACKET ptrIPHeader and upperLayerHeaderType fields.
// This is a change from the original TCPIP_IPV4_PutHeader function; the old function
// would also write the Ethernet and IP header to the MAC TX buffer, including
// the link-layer address.  Now the link-layer address is passed in in the TCPIP_IPV4_Flush
// function.  The length of remoteIPAddr is 4 or 16, depending on pkt's IP protocol
// type.
void TCPIP_IPV4_PutHeader(IPV4_PACKET * pkt, uint8_t protocol);

// Interface functions
// Gets the payload length of a TX packet
unsigned short TCPIP_IPV4_GetPayloadLength (IPV4_PACKET * pkt);

// Returns the pointer to the upper-layer header
void * TCPIP_IPV4_GetUpperLayerHeaderPtr(IPV4_PACKET * pkt);

#define TCPIP_IPV4_GetUpperLayerHeaderLen(s) (s->upperLayerHeaderLen)

// Calcualtes the 1's complement checksum over the packet payload
unsigned short TCPIP_IPV4_CalculatePayloadChecksum (IPV4_PACKET * pkt);

unsigned short TCPIP_IPV4_PutArrayHelper (IPV4_PACKET * pkt, const void * dataSource, uint8_t dataType, unsigned short len);
#define TCPIP_IPV4_PutRxData(mac,pkt,len)        TCPIP_IPV4_PutArrayHelper(pkt, &mac, IPV4_DATA_NETWORK_FIFO, len)

// Resets the IPV4_PACKET state associated with a socket after transmitting a packet so that it
// can be used to transmit additional packets.
void TCPIP_IPV4_ResetTransmitPacketState (IPV4_PACKET * pkt);

// Transmits a packet section
bool TCPIP_IPV4_TransmitPacket (IPV4_PACKET * pkt);

#define TCPIP_IPV4_IsPacketQueued(p)       (((IPV4_PACKET *)p)->flags.queued)

void TCPIP_IPV4_SingleListFree (void * list);


void TCPIP_IPV4_TmoHandler(SYS_TICK curSysTick);

#endif // _IPV4_MANAGER_H_



