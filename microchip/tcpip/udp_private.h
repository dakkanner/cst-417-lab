/*******************************************************************************
  UDP Module private definitions

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  udp_private.h 
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

#ifndef __UDP_PRIVATE_H_
#define __UDP_PRIVATE_H_

#define UDP_CHECKSUM_OFFSET     6u

// UDP States 
typedef enum
{
    UDP_CLOSED,				// Socket is idle and unallocated
	UDP_GATEWAY_SEND_ARP,	// Special state for UDP client mode sockets
	UDP_GATEWAY_GET_ARP,	// Special state for UDP client mode sockets	
	UDP_OPENED
} UDP_STATE;


// Stores information about a current UDP socket
typedef struct
{
    uint32_t		remoteHost;		// RAM or const pointer to a hostname string (ex: "www.microchip.com")
    UDP_PORT    remotePort;		// Remote node's UDP port number
    UDP_PORT    localPort;		// Local UDP port number, or 0 when free
    TCPIP_NET_IF* pSktNet;       // local interface
    UDP_STATE   smState;			// State of this socket
    SYS_TICK    retryInterval;
    uint8_t        retryCount;
    union
    {
        struct
        {
            char bRemoteHostIsROM : 1;	// Remote host is stored in const
            char looseRemPort     : 1;  // allows receiving data from any socket having the same destination Port
            // but irrespective its local port
            char looseNetIf       : 1;  // allows receiving data on any interface        
        };
        uint8_t Val;
    }flags;
    IP_ADDRESS_TYPE addType;     // IPV4/6 socket type;
    SYS_TICK    eventTime;
    void*       pTxPkt;     // IPV4_PACKET/IPV6_PACKET
} UDP_SOCKET_DCPT;


/****************************************************************************
  Section:
	External Global Variables
  ***************************************************************************/
#if !defined(__UDP_C)
    extern UDP_SOCKET_DCPT*  UDPSocketDcpt;
#endif

// Stores the header of a UDP packet
typedef struct
{
    UDP_PORT    SourcePort;				// Source UDP port
    UDP_PORT    DestinationPort;		// Destination UDP port
    uint16_t        Length;					// Length of data
    uint16_t        Checksum;				// UDP checksum of the data
} UDP_HEADER;


#endif  // __UDP_PRIVATE_H_



