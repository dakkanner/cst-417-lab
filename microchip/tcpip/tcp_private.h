/*******************************************************************************
  TCP Module private defs for Microchip TCP/IP Stack

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  tcp_private.h
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

#ifndef _TCP_PRIVATE_H_
#define _TCP_PRIVATE_H_


/****************************************************************************
  Section:
	Type Definitions
  ***************************************************************************/


#define TCP_CHECKSUM_OFFSET (16u)


// The minimum value a RX or TX TCP buffer can have
// The value of 25 is chosen to accomodate the SSL connections
#define TCP_MIN_RX_BUFF_SIZE    (25)
#define TCP_MIN_TX_BUFF_SIZE    (25)

// for efficiency reasons, 
// any request to change a TX/RX buffer size
// that results in a difference less than this limit
// will be ignored
#define TCP_MIN_BUFF_CHANGE     (32)

/****************************************************************************
  Section:
	State Machine Variables
  ***************************************************************************/

// TCP States as defined by RFC 793
typedef enum
{
	TCP_GATEWAY_SEND_ARP,	    // Special state for TCP client mode sockets
	TCP_GATEWAY_GET_ARP,	    // Special state for TCP client mode sockets

    TCP_LISTEN,				    // Socket is listening for connections
    TCP_SYN_SENT,			    // A SYN has been sent, awaiting an SYN+ACK
    TCP_SYN_RECEIVED,		    // A SYN has been received, awaiting an ACK
    TCP_ESTABLISHED,		    // Socket is connected and connection is established
    TCP_FIN_WAIT_1,			    // FIN WAIT state 1
    TCP_FIN_WAIT_2,			    // FIN WAIT state 2
    TCP_CLOSING,			    // Socket is closing
//	TCP_TIME_WAIT, state is not implemented
	TCP_CLOSE_WAIT,			    // Waiting to close the socket
    TCP_LAST_ACK,			    // The final ACK has been sent

    TCP_CLOSED_BUT_RESERVED,    // Special state for TCP client mode sockets.
                                // Socket is idle, but still allocated
                                // pending application closure of the handle.
} TCP_STATE;

typedef enum
{
	SSL_NONE = 0,			// No security is enabled
	SSL_HANDSHAKING,		// Handshake is progressing (no application data allowed)
	SSL_ESTABLISHED,		// Connection is established and secured
	SSL_CLOSED				// Connection has been closed (no applicaiton data is allowed)
} SSL_STATE;

/****************************************************************************
  Section:
	TCB Definitions
  ***************************************************************************/

// TCP Control Block (TCB) stub data storage.  Stubs are stored in local PIC RAM for speed.
typedef struct
{
	uint8_t*            txStart;		    // First byte of TX buffer
	uint8_t*            txEnd;			    // Last byte of TX buffer
	uint8_t*            txHead;			    // Head pointer for TX
	uint8_t*            txTail;			    // Tail pointer for TX
	uint8_t*	        txUnackedTail;	    // TX tail pointer for data that is not yet acked
	uint8_t*            rxStart;		    // First byte of RX buffer.
	uint8_t*            rxEnd;			    // Last byte of RX buffer
	uint8_t*            rxHead;			    // Head pointer for RX
	uint8_t*            rxTail;			    // Tail pointer for RX
    SYS_TICK            eventTime;			// Packet retransmissions, state changes
	SYS_TICK            eventTime2;		    // Window updates, automatic transmission
    SYS_TICK            delayedACKTime;     // Delayed Acknowledgement timer
    SYS_TICK            closeWaitTime;		// TCP_CLOSE_WAIT timeout timer
    uint16_t            smState;			// TCP_STATE: State of this socket
    struct
    {
	    uint16_t vUnackedKeepalives : 3;		// Count of how many keepalives have been sent with no response
        uint16_t bServer : 1;					// Socket should return to listening state when closed
		uint16_t bTimerEnabled	: 1;			// Timer is enabled
		uint16_t bTimer2Enabled : 1;			// Second timer is enabled
		uint16_t bDelayedACKTimerEnabled : 1;	// DelayedACK timer is enabled
		uint16_t bOneSegmentReceived : 1;		// A segment has been received
		uint16_t bHalfFullFlush : 1;			// Flush is for being half full
		uint16_t bTXASAP : 1;					// Transmit as soon as possible (for Flush)
		uint16_t bTXASAPWithoutTimerReset : 1;	// Transmit as soon as possible (for Flush), but do not reset retransmission timers
		uint16_t bTXFIN : 1;					// FIN needs to be transmitted
		uint16_t bSocketReset : 1;				// Socket has been reset (self-clearing semaphore)
		uint16_t bSSLHandshaking : 1;			// Socket is in an SSL handshake
		uint16_t bInitialized : 1;				// Future expansion
		uint16_t failedDisconnect : 1;			// Failed to send a FIN
    } Flags;
	TCPIP_UINT16_VAL remoteHash;	                // Consists of remoteIP, remotePort, localPort for connected sockets.
                                                    // It is a localPort number only for listening server sockets.

#if defined (TCPIP_STACK_USE_SSL)
    uint8_t*            sslTxHead;		            // Position of data being written in next SSL application record
    						                        // Also serves as cache of localSSLPort when smState = TCP_LISTENING
    uint8_t*            sslRxHead;		            // Position of incoming data not yet handled by SSL
    TCPIP_UINT16_VAL	localSSLPort;			    // Local SSL port number (for listening sockets)
    uint8_t             sslStubID;			        // Which sslStub is associated with this connection
    uint8_t             sslReqMessage;		        // Currently requested SSL message
#endif // defined (TCPIP_STACK_USE_SSL)

    TCPIP_NET_IF*       pSktNet;                    // which interface this socket is bound to
    void *              pTxPkt;                     // Transmit packet state
    IP_ADDRESS_TYPE     addType;                    // IPV4/6 socket type;
    // 
	SYS_TICK            retryInterval;			    // How long to wait before retrying transmission
	uint32_t		    MySEQ;					    // Local sequence number
	uint32_t		    RemoteSEQ;				    // Remote sequence number
    uint32_t            remoteHost;
    TCPIP_UINT16_VAL    remotePort;			    	// Remote port number
    TCPIP_UINT16_VAL	localPort;				    // Local port number
	uint16_t		    remoteWindow;			    // Remote window size
	uint16_t		    localWindow;			    // last advertised window size
	uint16_t		    wFutureDataSize;		    // How much out-of-order data has been received
	int16_t		        sHoleSize;				    // Size of the hole, or -1 for none exists.  (0 indicates hole has just been filled)
	uint16_t		    wRemoteMSS;				    // Maximum Segment Size option advertised by the remote node during initial handshaking
	uint16_t		    localMSS;				    // our advertised MSS
    TCP_SOCKET          sktIx;                      // socket number
     
    struct
    {
        uint16_t bFINSent : 1;		            // A FIN has been sent
		uint16_t bSYNSent : 1;		            // A SYN has been sent
		uint16_t bRXNoneACKed1 : 1;	            // A duplicate ACK was likely received
		uint16_t bRXNoneACKed2 : 1;	            // A second duplicate ACK was likely received
		unsigned char openAddType : 2;		    // the address type used at open
		uint16_t nonLinger : 1; 		        // linger option
		uint16_t nonGraceful : 1; 		        // graceful close
        uint16_t ackSent :1;                    // acknowledge sent in this pass
        uint16_t seqInc :1;                     // sequence number incremented after FIN ack 
        uint16_t forceKill :1;                  // socket should be killed, even if it's server socket 
        uint16_t forceFlush :1;                 // flush data any time caller writes to the socket
                                                // i.e. disable Nagle
        uint16_t reserved :4;                   // reserved for future use 
    } flags;
	uint8_t		        retryCount;				// Counter for transmission retries
    
} TCB_STUB;




#endif  // _TCP_PRIVATE_H_
