/*******************************************************************************
  BSD internal stack API Header File

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  berkeley_manager.h 
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

#ifndef __BERKELEY_MANAGER_H_
#define __BERKELEY_MANAGER_H_


typedef enum
{
    SKT_CLOSED,   			// Socket closed state indicating a free descriptor
    SKT_CREATED, 			// Socket created state for TCP and UDP sockets
    SKT_BOUND,   			// Socket bound state for TCP and UDP sockets
    SKT_BSD_LISTEN,			// Listening state for TCP BSD listener handle "socket"
    SKT_LISTEN,  			// TCP server listen state
    SKT_IN_PROGRESS, 		// TCP client connection in progress state
    SKT_EST,  				// TCP client or server established state
    SKT_DISCONNECTED		// TCP client or server no longer connected to the remote host (but was historically)
} BSD_SCK_STATE; // Berkeley Socket (BSD) states

struct BSDSocket
{
    int            SocketType; // Socket type
    BSD_SCK_STATE  bsdState; //Socket state
    uint16_t           localPort; //local port
    uint16_t           remotePort; //remote port
    uint32_t          remoteIP; //remote IP
    int            backlog; // maximum number or client connection
    bool           isServer; // server/client check
    TCP_SOCKET     SocketID; // Socket ID
    uint32_t          localIP; // bound address
}; // Berkeley Socket structure



bool BerkeleySocketInit(const TCPIP_STACK_MODULE_CTRL* const stackData,
                        const BERKELEY_MODULE_GONFIG* berkeleyData);

void BerkeleySocketDeInit(const TCPIP_STACK_MODULE_CTRL* const stackData);

#endif  // __BERKELEY_MANAGER_H_


