/*******************************************************************************
  IPv4 private API for Microchip TCP/IP Stack

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  ipv4_private.h 
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

#ifndef _IPV4_PRIVATE_H_
#define _IPV4_PRIVATE_H_

#define ETHERTYPE_IPV4      	(0x0800u)
#define ETHERTYPE_IPV6          (0x86DDu)

// misc IP functions
// 

#define TCPIP_IPV4_SwapPseudoHeader(h)  (h.Length = TCPIP_HELPER_htons(h.Length))


bool TCPIP_IPV4_GetHeader(TCPIP_NET_IF* pNet, IPV4_ADDR *localIP, NODE_INFO *remote, uint8_t *protocol, uint16_t *len);

// Allocate a data segment header
IPV4_DATA_SEGMENT_HEADER * TCPIP_IPV4_AllocateDataSegmentHeader (uint16_t len);

// Gets the IP-layer pseudo-header checksum
unsigned short TCPIP_IPV4_GetPseudoHeaderChecksum (IPV4_PACKET * pkt);

void TCPIP_IPV4_InsertIPPacketSegment (IPV4_DATA_SEGMENT_HEADER * ptrSegment, IPV4_PACKET * ptrPacket, IPV4_SEGMENT_TYPE);
IPV4_DATA_SEGMENT_HEADER * TCPIP_IPV4_GetDataSegmentByType (IPV4_PACKET * ptrPacket, IPV4_SEGMENT_TYPE type);
void * TCPIP_IPV4_GetDataSegmentContentsByType (IPV4_PACKET * ptrPacket, IPV4_SEGMENT_TYPE type);

// Flush the chain of data segments to the MAC TX buffer.
void TCPIP_IPV4_FlushDataSegments (TCPIP_MAC_HANDLE hMac, IPV4_PACKET * pkt);

// Deallocate all of the data segment headers and dynamically allocated payload segments in an IPV4_PACKET payload
void TCPIP_IPV4_FreePacketData (IPV4_PACKET * ptrPacket);


#endif // _IPV4_PRIVATE_H_



