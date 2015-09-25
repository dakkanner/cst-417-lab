/*******************************************************************************
  UDP Module manager - private stack API

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  udp_manager.h 
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

#ifndef __UDP__MANAGER_H_
#define __UDP__MANAGER_H_


bool UDPInit(const TCPIP_STACK_MODULE_CTRL* const stackInit, const UDP_MODULE_CONFIG* pUdpInit);
void UDPDeInit(const TCPIP_STACK_MODULE_CTRL* const stackInit);

void UDPTask(TCPIP_NET_IF* pNetIf);



bool UDPProcessIPv4(TCPIP_NET_IF* pNetIf, NODE_INFO *remoteNode, IPV4_ADDR *localIP, uint16_t len);
bool UDPProcessIPv6(TCPIP_NET_IF * pNetIf, IPV6_ADDR * localIP, IPV6_ADDR * remoteIP, uint16_t dataLen, uint16_t headerLen);

void UDPResetHeader(UDP_HEADER * h);



// sets the source IP address of a packet
bool UDPSetSourceIPAddress(UDP_SOCKET s, IP_ADDRESS_TYPE addType, IP_MULTI_ADDRESS* localAddress);


// sets the destination IP address of a packet
bool UDPSetDestinationIPAddress(UDP_SOCKET s, IP_ADDRESS_TYPE addType, IP_MULTI_ADDRESS* remoteAddress);



void UDPDiscardNet(TCPIP_NET_IF* pNetIf);


  


#endif // __UDP__MANAGER_H_


