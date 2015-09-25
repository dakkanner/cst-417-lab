/*******************************************************************************
  TCP Manager internal stack API

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  tcp_manager.h 
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

#ifndef __TCP_MANAGER_H_
#define __TCP_MANAGER_H_


/****************************************************************************
  Section:
	Type Definitions
  ***************************************************************************/


/****************************************************************************
  Section:
	Function Declarations
  ***************************************************************************/

bool TCPInit(const TCPIP_STACK_MODULE_CTRL* const stackInit, const TCP_MODULE_CONFIG* pTcpInit);
void TCPDeInit(const TCPIP_STACK_MODULE_CTRL* const stackInit);
bool TCPProcessIPv4(TCPIP_NET_IF* pNetIf, NODE_INFO* remote, IPV4_ADDR* localIP, uint16_t len);
bool TCPProcessIPv6(TCPIP_NET_IF* pPktIf, IPV6_ADDR * localIP, IPV6_ADDR * remoteIP, uint16_t dataLen, uint16_t headerLen);
void TCPTick(void);
bool TCPTaskPending(void);

bool TCPSetSourceIPAddress(TCP_SOCKET s, IP_ADDRESS_TYPE addType, IP_MULTI_ADDRESS* localAddress);

bool TCPSetDestinationIPAddress(TCP_SOCKET s, IP_ADDRESS_TYPE addType, IP_MULTI_ADDRESS* remoteAddress);

int     TCPSocketsNo(void);



// SSL related

void        TcpSSLMessageTransmit(TCP_SOCKET hTCP);

uint8_t     TcpSocketGetSSLId(TCP_SOCKET hTCP);

void        TCPSSLPutRecordHeader(TCP_SOCKET hTCP, uint8_t* hdr, bool recDone);
uint16_t    TCPSSLGetPendingTxSize(TCP_SOCKET hTCP);
void        TCPSSLHandleIncoming(TCP_SOCKET hTCP);

bool        TCPRequestSSLMessage(TCP_SOCKET hTCP, uint8_t msg);

void        TCPSSLDecryptMAC(TCP_SOCKET hTCP, ARCFOUR_CTX* ctx, uint16_t len);

void        TCPSSLInPlaceMACEncrypt(TCP_SOCKET hTCP, ARCFOUR_CTX* ctx, uint8_t* MACSecret, uint16_t len);



#endif  // __TCP_MANAGER_H_
