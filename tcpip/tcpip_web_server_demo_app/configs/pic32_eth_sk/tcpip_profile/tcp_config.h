/*******************************************************************************
  Transmission Control Protocol (TCP) Configuration file

  Summary:
    TCP configuration file
    
  Description:
    This file contains the TCP module configuration options
    
*******************************************************************************/

/*******************************************************************************
FileName:   tcp_config.h
Copyright © 2011 released Microchip Technology Inc.  All rights 
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

#ifndef _TCP_CONFIG_H_
#define _TCP_CONFIG_H_

// First port number for randomized local port number selection
// Use the dynamic port range defined by IANA consists of the 49152-65535 range
// and is meant for the selection of ephemeral ports (RFC 6056).
// Adjust to your needs but stay within the IANA range 
#define LOCAL_TCP_PORT_START_NUMBER (49152)

// Last port number for randomized local port number selection
#define LOCAL_TCP_PORT_END_NUMBER   (65535)

// TCP Maximum Segment Size for TX.  The TX maximum segment size is actually 
// governed by the remote node's MSS option advertised during connection 
// establishment.  However, if the remote node specifies an unhandlably large 
// MSS (ex: > Ethernet MTU), this define sets a hard limit so that we don't 
// cause any TX buffer overflows.  If the remote node does not advirtise a MSS 
// option, all TX segments are fixed at 536 bytes maximum.
#define TCP_MAX_SEG_SIZE_TX			(1460u)

// TCP Maximum Segment Size for RX (MSS).
// This value is advertised during TCP connection establishment
// and the remote node should obey it.
// The value has to be set in such a way to avoid IP layer fragmentation
// from causing packet loss.
// However, raising its value can enhance performance at the (small) risk of introducing 
// incompatibility with certain special remote nodes (ex: ones connected via a 
// slow dial up modem).
// On Ethernet networks the standard value is 1460.
// On dial-up links, etc. the default values should be 536.
// Adjust these values according to your network.
// The stack will use the local value for local destination networks
// and the default value for nonlocal networks. 
#define TCP_MAX_SEG_SIZE_RX_LOCAL   			(1460)
#define TCP_MAX_SEG_SIZE_RX_NON_LOCAL			(536)

// default socket Tx buffer size
#define TCP_SOCKET_DEFAULT_TX_SIZE		512			

// default socket Rx buffer size
#define TCP_SOCKET_DEFAULT_RX_SIZE		512			
		

// TCP Timeout and retransmit numbers
// All timeouts in milliseconds
// Timeout to retransmit unacked data, ms
#define TCP_START_TIMEOUT_VAL   	(1000ul)

// Timeout for delayed-acknowledgement algorithm, ms
#define TCP_DELAYED_ACK_TIMEOUT		(100ul)

// Timeout for FIN WAIT 2 state, ms
#define TCP_FIN_WAIT_2_TIMEOUT		(5000ul)

// Timeout for keep-alive messages when no traffic is sent, ms
#define TCP_KEEP_ALIVE_TIMEOUT		(10000ul)

// Timeout for the CLOSE_WAIT state, ms
#define TCP_CLOSE_WAIT_TIMEOUT		(200ul)

// Maximum number of retransmission attempts
#define TCP_MAX_RETRIES			(5u)

// Maximum number of keep-alive messages that can be sent 
// without receiving a response before automatically closing 
// the connection
#define TCP_MAX_UNACKED_KEEP_ALIVES	(6u)

// Smaller than all other retries to reduce SYN flood DoS duration
#define TCP_MAX_SYN_RETRIES		(2u)

// Timeout before automatically transmitting unflushed data, ms; Default 40 ms
#define TCP_AUTO_TRANSMIT_TIMEOUT_VAL	(40ul)

// Timeout before automatically transmitting a window update due to a TCPGet() or TCPGetArray() function call, ms
#define TCP_WINDOW_UPDATE_TIMEOUT_VAL	(200ul)

//	The maximum number of sockets to create in the stack.
//	When defining TCP_MAX_SOCKETS take into account
//	the number of interfaces the stack is supporting
#define TCP_MAX_SOCKETS 			(10)



// The TCP task processing rate: number of miliseconds to generate an TCP tick.
// Used by the TCP state machine
// Note: the System Tick resolution in system_config.h (SYS_TICKS_PER_SECOND) has to 
// be fine enough to allow for this TCP tick granularity.  
#define TCP_TASK_TICK_RATE        (5)     // 5 ms default rate 


/* Reference only:
// TCP layer configuration/initialization
typedef struct
{
    int             nSockets;       // number of sockets to be created
    uint16_t        sktTxBuffSize;  // size of the socket tx buffer
    uint16_t        sktRxBuffSize;  // size of the socket rx buffer
}TCP_MODULE_CONFIG;
*/

#ifdef TCPIP_STACK_MODULE_CONFIGURATION
const TCP_MODULE_CONFIG  tcpConfigData = 
{
    TCP_MAX_SOCKETS,
    TCP_SOCKET_DEFAULT_TX_SIZE,
    TCP_SOCKET_DEFAULT_RX_SIZE,
     
};
#else
extern const TCP_MODULE_CONFIG  tcpConfigData;
#endif  // TCPIP_STACK_MODULE_CONFIGURATION

#endif  // _TCP_CONFIG_H_
