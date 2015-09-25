/*******************************************************************************
  Transmission Control Protocol (TCP) Communications Layer

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Provides reliable, handshaked transport of application stream 
     oriented data with flow control
    -Reference: RFC 793
*******************************************************************************/

/*******************************************************************************
FileName:   tcp.c
Copyright 2012 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS?WITHOUT WARRANTY OF ANY KIND,
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


#include "tcpip_private.h"
#include "tcp_private.h"

#if defined(TCPIP_STACK_USE_TCP)

#define TCPIP_THIS_MODULE_ID    TCPIP_MODULE_TCP

/****************************************************************************
  Section:
	TCP Header Data Types
  ***************************************************************************/

#define FIN             (0x01)		// FIN Flag as defined in RFC
#define SYN             (0x02)		// SYN Flag as defined in RFC
#define RST             (0x04)		// Reset Flag as defined in RFC
#define PSH             (0x08)		// Push Flag as defined in RFC
#define ACK             (0x10)		// Acknowledge Flag as defined in RFC
#define URG             (0x20)		// Urgent Flag as defined in RFC


// TCP Header Data Structure
typedef struct
{
	uint16_t    SourcePort;		// Local port number
	uint16_t    DestPort;		// Remote port number
	uint32_t   SeqNumber;		// Local sequence number
	uint32_t   AckNumber;		// Acknowledging remote sequence number

	struct
	{
		unsigned char Reserved3      : 4;
		unsigned char Val            : 4;
	} DataOffset;			// Data offset flags nibble

	union
	{
		struct
		{
			unsigned char flagFIN    : 1;
			unsigned char flagSYN    : 1;
			unsigned char flagRST    : 1;
			unsigned char flagPSH    : 1;
			unsigned char flagACK    : 1;
			unsigned char flagURG    : 1;
			unsigned char Reserved2  : 2;
		} bits;
		uint8_t byte;
	} Flags;				// TCP Flags as defined in RFC

	uint16_t    Window;			// Local free RX buffer window
	uint16_t    Checksum;		// Data payload checksum
	uint16_t    UrgentPointer;	// Urgent pointer
} TCP_HEADER;

#define TCP_OPTIONS_END_OF_LIST     (0x00u)		// End of List TCP Option Flag
#define TCP_OPTIONS_NO_OP           (0x01u)		// No Op TCP Option
#define TCP_OPTIONS_MAX_SEG_SIZE    (0x02u)		// Maximum segment size TCP flag
typedef struct
{
	uint8_t        Kind;							// Type of option
	uint8_t        Length;							// Length
	TCPIP_UINT16_VAL    MaxSegSize;						// Maximum segment size
} TCP_OPTIONS;									// TCP Options data structure							

// Indicates if this packet is a retransmission (no reset) or a new packet (reset required)
#define SENDTCP_RESET_TIMERS	0x01
// Instead of transmitting normal data, a garbage octet is transmitted according to RFC 1122 section 4.2.3.6
#define SENDTCP_KEEP_ALIVE		0x02

// Internal _TcpSend result
typedef enum
{
    // positive success codes
    _TCP_SEND_MAC_BUSY  = 2,    // MAC is busy; however the operation can be retried
    _TCP_SEND_NOT_CONN  = 1,    // socket is not connected, no data is actually sent 
    _TCP_SEND_OK        = 0,    // success

    // negative failure codes
    _TCP_SEND_NO_SPACE  = -1,   // not enough space in the packet to set all the data
                                // most probably an out of memory condition
    _TCP_SEND_NO_PKT    = -2,   // socket has no packet, cannot send data
}_TCP_SEND_RES;

/****************************************************************************
  Section:
	TCB Definitions
  ***************************************************************************/


static TCB_STUB** TCBStubs = 0;

static int        tcpInitCount = 0;                 // initialization counter

static const void*  tcpHeapH = 0;                    // memory allocation handle
static unsigned int TcpSockets;                      // number of sockets in the current TCP configuration

static int                  tcpTickPending = 0;        // TCP processing tick
static SystemTickHandle     tcpTimerHandle = 0;

static uint16_t             tcpDefTxSize;               // default size of the TX buffer
static uint16_t             tcpDefRxSize;               // default size of the RX buffer

static uint16_t             tcpMssLocal;                // default size of the MSS for local networks
static uint16_t             tcpMssNonLocal;             // default size of the MSS for nonlocal networks


static const TCP_MODULE_CONFIG  tcpConfigDefault = 
{
    TCP_MAX_SOCKETS,
    TCP_SOCKET_DEFAULT_TX_SIZE,
    TCP_SOCKET_DEFAULT_RX_SIZE,
     
};


/****************************************************************************
  Section:
	Function Prototypes
  ***************************************************************************/

static void _TcpMac2RamCopy(TCPIP_NET_IF* pNet, uint8_t* ptrDest, TCPIP_MAC_PTR_TYPE ptrSource, uint16_t wLength);


static _TCP_SEND_RES _TcpSend(TCB_STUB* pSkt, uint8_t vTCPFlags, uint8_t vSendFlags);
static void _TcpHandleSeg(TCB_STUB* pSkt, TCP_HEADER* h, uint16_t len);
static TCB_STUB* _TcpFindMatchingSocket(TCPIP_NET_IF* pPktIf, TCP_HEADER* h, void * localIP, void * remoteIP, MAC_ADDR * remoteMACAddr, IP_ADDRESS_TYPE addressType);
static void _TcpSwapHeader(TCP_HEADER* header);
static void _TcpCloseSocket(TCB_STUB* pSkt);
static void _TcpSocketInitialize(TCB_STUB* pSkt, TCP_SOCKET hTCP, uint8_t* txBuff, uint16_t txBuffSize, uint8_t* rxBuff, uint16_t rxBuffSize);
static void _TcpSocketSetIdleState(TCB_STUB* pSkt);
static void _TcpTmoHandler(SYS_TICK currSysTick);

static void _TcpCleanup(void);
static void _TcpAbort(TCB_STUB* pSkt, bool forceClose);
static _TCP_SEND_RES _TcpDisconnect(TCB_STUB* pSkt, bool signalFIN);


static TCP_PORT     TcpAllocateEphemeralPort(void);
static bool         TCPIsAvailablePort(TCP_PORT port);

static uint16_t     TcpClientIpv4RemoteHash(IPV4_ADDR* pAdd, TCB_STUB* pSkt);


typedef enum
{
    TCP_OPEN_SERVER,    // create a server socket
    TCP_OPEN_CLIENT,    // create a client socket

}TCP_OPEN_TYPE;


static TCP_SOCKET   TCPOpen(IP_ADDRESS_TYPE addType, TCP_OPEN_TYPE opType, UDP_PORT port, IP_MULTI_ADDRESS* address);

#if defined (TCPIP_STACK_USE_IPV4)
static IPV4_PACKET*     TCPv4AllocateTxPacketStruct (TCB_STUB * stub);
static void             _Tcpv4AckFnc (void * pkt, bool sent, void * param);

#endif  // defined (TCPIP_STACK_USE_IPV4)


#if defined (TCPIP_STACK_USE_IPV6)
static IPV6_PACKET*     TCPv6AllocateTxPacketStruct (TCB_STUB * stub);
static void             _Tcpv6AckFnc (void * pkt, bool sent, void * param);

#endif  // defined (TCPIP_STACK_USE_IPV6)


static bool TCP_IsTxReady(TCB_STUB * stub);
static void * TCP_GetUpperLayerHeaderPtr(TCB_STUB * stub);
static unsigned short TCP_IsTxPutReady (TCB_STUB * stub, unsigned short count);
static unsigned short TCP_PutArray(TCB_STUB * stub, const void * dataSource, unsigned short len);
static unsigned short TCP_SetPayload(TCB_STUB * stub, uint8_t* payload, unsigned short len);
static void TCP_PutHeader(TCB_STUB * stub, uint8_t protocol);
static bool TCP_Flush(TCB_STUB * stub, MAC_ADDR * remoteMACAddr);
static bool TCP_IsPacketQueued(TCB_STUB * stub);
static bool TCP_CopyTxPacketStruct (void * destination, TCB_STUB * stub);
static void TCP_FreePacket(void * pkt, IP_ADDRESS_TYPE addType);
static void TCP_ResetTransmitPacketState (TCB_STUB * stub);

static void         _TcpFlush(TCB_STUB* pSkt);

static void         _TcpDiscardTx(TCB_STUB* pSkt);

static uint16_t     _TCPIsPutReady(TCB_STUB* pSkt);

static uint16_t     _TCPIsGetReady(TCB_STUB* pSkt);

static uint16_t     _TCPGetRxFIFOFree(TCB_STUB* pSkt);

static bool         _TCPSendWinIncUpdate(TCB_STUB* pSkt);

/*static __inline__*/static  void /*__attribute__((always_inline))*/ _TcpSocketBind(TCB_STUB* pSkt, TCPIP_NET_IF* pNet)
{
    pSkt->pSktNet = pNet;
    switch(pSkt->addType)
    {
#if defined (TCPIP_STACK_USE_IPV6)
        case IP_ADDRESS_TYPE_IPV6:
            ((IPV6_PACKET*)pSkt->pTxPkt)->netIfH = pNet;
            break;
#endif  // defined (TCPIP_STACK_USE_IPV6)

#if defined (TCPIP_STACK_USE_IPV4)
        case IP_ADDRESS_TYPE_IPV4:
            ((IPV4_PACKET*)pSkt->pTxPkt)->netIfH = pNet;
            break;
#endif  // defined (TCPIP_STACK_USE_IPV4)

        default:
            break;
    }
}

/*static __inline__*/static  TCB_STUB* /*__attribute__((always_inline))*/ _TcpSocketChk(TCP_SOCKET hTCP)
{
    if(hTCP >= 0 && hTCP < TcpSockets)
    {
        return TCBStubs[hTCP];
    }

    return 0;
}


/*static __inline__*/static  void /*__attribute__((always_inline))*/ _TcpSocketKill(TCB_STUB* pSkt)
{
    TCBStubs[pSkt->sktIx] = 0;
    TCPIP_HEAP_Free(tcpHeapH, (void*)pSkt->rxStart);
    TCPIP_HEAP_Free(tcpHeapH, (void*)pSkt->txStart);
    TCPIP_HEAP_Free(tcpHeapH, pSkt);
}

/*****************************************************************************
  Function:
	bool TCPInit(const TCPIP_STACK_MODULE_CTRL* const stackInit, TCP_MODULE_CONFIG* pTcpInit)

  Summary:
	Initializes the TCP module.

  Description:
	Initializes the TCP module.  This function sets up the TCP buffers
	in memory and initializes each socket to the CLOSED state.  If
	insufficient memory was allocated for the TCP sockets, the function
	will call the TCPIPError() error function that can be captured by the debugger
    and will return false.

  Precondition:
	None

  Parameters:
    stackInit   - pointer to stack initialization data; contains heap, interfaces, etc

    pTcpInit    - pointer to a TCP initialization structure containing:
                    - nSockets:         number of sockets to be created
                    - sktTxBuffSize:    default TX buffer size
                    - sktRxBuffSize:    default RX buffer size
  Returns:
  	true if initialization succeeded
    false otherwise
  	
  Remarks:
   None 
  ***************************************************************************/
bool TCPInit(const TCPIP_STACK_MODULE_CTRL* const stackInit, const TCP_MODULE_CONFIG* pTcpInit)
{
    int      nSockets;
    
    if(stackInit->stackAction == TCPIP_STACK_ACTION_IF_UP)
    {   // interface start up
        return true;    // do not store per interface data
    }

    // stack start up
    if(tcpInitCount != 0)
    {   // initialize just once
        tcpInitCount++;
        return true;
    }
    
    if(stackInit->memH == 0)
    {
        SYS_ERROR(SYS_ERROR_ERROR, "TCP NULL dynamic allocation handle");
        return false;
    }

    tcpHeapH = stackInit->memH;

    // select default configuration if init data is missing
    if(pTcpInit == 0)
    {
        pTcpInit = &tcpConfigDefault;
    }

    nSockets = pTcpInit->nSockets?pTcpInit->nSockets:TCP_MAX_SOCKETS;
    // default initialization
    tcpDefTxSize = pTcpInit->sktTxBuffSize;
    tcpDefRxSize = pTcpInit->sktRxBuffSize;

    TCBStubs = (TCB_STUB**)TCPIP_HEAP_Calloc(tcpHeapH, nSockets, sizeof(*TCBStubs));
    if(TCBStubs == 0)
    {
        SYS_ERROR(SYS_ERROR_ERROR, " TCP Dynamic allocation failed");
        return false;
    }


    TcpSockets = nSockets;

    // create the TCP timer
    tcpTimerHandle = SYS_TICK_TimerCreate(_TcpTmoHandler);
    if(tcpTimerHandle)
    {
        tcpTickPending = 0;
        SYS_TICK_TimerSetRate(tcpTimerHandle, ((SYS_TICK_ResolutionGet() * TCP_TASK_TICK_RATE) + 999 )/1000);
    }
    else
    {   // cannot create the TCP timer
        _TcpCleanup();
        return false;
    }
    
    // initialize some default data
    tcpMssLocal = TCP_MAX_SEG_SIZE_RX_LOCAL; 
    tcpMssNonLocal = TCP_MAX_SEG_SIZE_RX_NON_LOCAL;

    tcpInitCount++;

    return true;
}

/*****************************************************************************
  Function:
	void TCPDeInit(const TCPIP_STACK_MODULE_CTRL* const stackInit)

  Summary:
	De-Initializes the TCP module.

  Description:
	De-Initializes the TCP module.
    This function initializes each socket to the CLOSED state.
    If dynamic memory was allocated for the TCP sockets, the function
	will deallocate it.

  Precondition:
	TCPInit() should have been called

  Parameters:
    stackInit   - pointer to stack initialization data; contains heap, interfaces, etc
                  and interface that's going down

  Returns:
    None
    
  Remarks:
  ***************************************************************************/
void TCPDeInit(const TCPIP_STACK_MODULE_CTRL* const stackInit)
{
    int       ix;
    TCB_STUB* pSkt;


    // interface is going down
    for(ix = 0; ix < TcpSockets; ix++)
    {
        pSkt = TCBStubs[ix]; 
        if(pSkt && pSkt->pSktNet == stackInit->pNetIf)  
        {
            _TcpAbort(pSkt, true);
        }
    }


    if(stackInit->stackAction == TCPIP_STACK_ACTION_DEINIT)
    {   // stack shut down
        if(tcpInitCount > 0)
        {   // we're up and running

            if(--tcpInitCount == 0)
            {   // all closed
                // release resources
                _TcpCleanup();
            }
        }
    }
	
}

// performs the clean-up of resources
static void _TcpCleanup(void)
{
    int ix;
    TCB_STUB* pSkt;

    if(TCBStubs)
    {
        // all sockets should be closed here
        // just a sanity check
        for(ix = 0; ix < TcpSockets; ix++)
        {
            pSkt = TCBStubs[ix]; 
            if(pSkt) 
            {
                _TcpAbort(pSkt, true);
            }
        }
    }

    TCPIP_HEAP_Free(tcpHeapH, TCBStubs);
    TCBStubs = 0;

    TcpSockets = 0;

    if(tcpTimerHandle)
    {
        SYS_TICK_TimerDelete(tcpTimerHandle);
        tcpTimerHandle = 0;
        tcpTickPending = 0;
    }

}

static void _TcpTmoHandler(SYS_TICK currSysTick)
{
    tcpTickPending++;
}
    

// returns true if service needed
// called by the stack manager
bool TCPTaskPending(void)
{
    return tcpTickPending != 0;
}


bool TCPSetConnectionMSS(uint16_t mssLocal, uint16_t mssNonLocal)
{

    tcpMssLocal = mssLocal; 
    tcpMssNonLocal = mssNonLocal;

    return true;
}


/*****************************************************************************
  Function:
	 TCP_SOCKET TCPOpenServer(IP_ADDRESS_TYPE addType, TCP_PORT localPort,  IP_MULTI_ADDRESS* localAddress)

  Summary:
	Opens a TCP socket as a server.
	
  Description:
	Provides a unified method for opening TCP server sockets. 

	Sockets are statically/dynamically allocated on boot, and can be claimed with this
    function and freed using TCPClose.

  Precondition:
    TCP is initialized.

  Parameters:
    IP_ADDRESS_TYPE addType			-	The type of address being used. Example: IP_ADDRESS_TYPE_IPV4.
    TCP_PORT localPort				-	TCP port to listen on for connections.
    IP_MULTI_ADDRESS* localAddress	-	Local address to use.
	
  Returns:
 	INVALID_SOCKET -  No sockets of the specified type were available to be
                      opened.
    Otherwise -       A TCP_SOCKET handle. Save this handle and use it when
                      calling all other TCP APIs. 
 ***************************************************************************/
TCP_SOCKET TCPOpenServer(IP_ADDRESS_TYPE addType, TCP_PORT localPort,  IP_MULTI_ADDRESS* localAddress)
{
    TCP_SOCKET  skt;
    TCPIP_NET_IF* pDefIf = 0;

#if !defined (TCPIP_STACK_USE_IPV6)
   if(addType == IP_ADDRESS_TYPE_IPV6)
   {
       return INVALID_SOCKET;
   } 
   else
   {
       addType = IP_ADDRESS_TYPE_IPV4;
   }
#endif  // defined (TCPIP_STACK_USE_IPV6)
    
#if !defined (TCPIP_STACK_USE_IPV4)
   if(addType == IP_ADDRESS_TYPE_IPV4)
   {
       return INVALID_SOCKET;
   }
   else
   {
       addType = IP_ADDRESS_TYPE_IPV6;
   }
#endif  // defined (TCPIP_STACK_USE_IPV4)

#if defined (TCPIP_STACK_USE_IPV4)
    if(addType == IP_ADDRESS_TYPE_IPV4 && localAddress != 0 && localAddress->v4Add.Val != 0)
    {
        pDefIf = _TCPIPStackIpAddToNet(&localAddress->v4Add, false);
        if(pDefIf == 0)
        {    // no such interface
            return INVALID_UDP_SOCKET;
        }
    }
#endif  // defined (TCPIP_STACK_USE_IPV4)

#if defined (TCPIP_STACK_USE_IPV6)
   if(addType == IP_ADDRESS_TYPE_IPV6 && localAddress != 0)
   {
       int netIx, netNo;
       
       netNo = TCPIP_STACK_NetworksNo();
       for(netIx =0; netIx < netNo; netIx++)
       {
           pDefIf = (TCPIP_NET_IF*)TCPIP_STACK_IxToNet(netIx);
           if(TCPIP_IPV6_FindAddress(pDefIf, &localAddress->v6Add, IPV6_ADDR_TYPE_UNICAST) != 0)
           {    // found interface
              break;
           }
       }

       if(pDefIf == 0)
       {    // no such interface
           return INVALID_UDP_SOCKET;
       }
   }
#endif  // defined (TCPIP_STACK_USE_IPV6)

    skt = TCPOpen(addType, TCP_OPEN_SERVER, localPort, 0);
    if(pDefIf != 0 && skt != INVALID_SOCKET)
    {
        _TcpSocketBind(TCBStubs[skt], pDefIf);
    }

    return skt;

}

/*****************************************************************************
  Function:
	 TCP_SOCKET TCPOpenClient(IP_ADDRESS_TYPE addType, TCP_PORT remotePort, IP_MULTI_ADDRESS* remoteAddress)

  Summary:
	Opens a TCP socket as a client.
	
  Description:
	Provides a unified method for opening TCP client sockets. 

	Sockets are statically/dynamically allocated on boot, and can be claimed with this
    function and freed using TCPAbort or TCPClose.

  Precondition:
    TCP is initialized.

  Parameters:
    IP_ADDRESS_TYPE addType			-	The type of address being used. Example: IP_ADDRESS_TYPE_IPV4.
    TCP_PORT remotePort				-	TCP port to connect to.
                         				  The local port for client sockets will be automatically picked
                                          by the TCP module.
    IP_MULTI_ADDRESS* remoteAddress	-	The remote address to be used.
	
  Returns:
 	INVALID_SOCKET -  No sockets of the specified type were available to be
                      opened.
    Otherwise -       A TCP_SOCKET handle. Save this handle and use it when
                      calling all other TCP APIs. 

 Remarks:
    IP_ADDRESS_TYPE_ANY is not supported!
 ***************************************************************************/
TCP_SOCKET TCPOpenClient(IP_ADDRESS_TYPE addType, TCP_PORT remotePort, IP_MULTI_ADDRESS* remoteAddress)
{
    TCP_SOCKET  skt;

#if !defined (TCPIP_STACK_USE_IPV6)
    if(addType == IP_ADDRESS_TYPE_IPV6)
    {
        return INVALID_SOCKET; 
    }
    else
    {
        addType = IP_ADDRESS_TYPE_IPV4;
    }
#endif  // defined (TCPIP_STACK_USE_IPV6)

#if !defined (TCPIP_STACK_USE_IPV4)
    if(addType == IP_ADDRESS_TYPE_IPV4)
    {
        return INVALID_SOCKET; 
    }
    else
    {
        addType = IP_ADDRESS_TYPE_IPV6;
    }
#endif  // defined (TCPIP_STACK_USE_IPV4)

    if(addType == IP_ADDRESS_TYPE_ANY)
    {
        return INVALID_SOCKET; 
    }


    skt = TCPOpen(addType, TCP_OPEN_CLIENT, remotePort, remoteAddress);
    if(skt != INVALID_UDP_SOCKET)
    {   
        _TcpSocketBind(TCBStubs[skt], (TCPIP_NET_IF*)TCPIP_STACK_GetDefaultNet());
    }

    return skt;
}


/****************************************************************************
  Section:
	Connection Management Functions
  ***************************************************************************/


/*****************************************************************************
  Function:
	TCP_SOCKET TCPOpen(IP_ADDRESS_TYPE addType, TCP_OPEN_TYPE opType, UDP_PORT port, IP_MULTI_ADDRESS* address)
    
  Summary:
    Opens a TCP socket for listening or as a client.

  Description:
    Provides a unified method for opening TCP sockets. This function can
    open both client and server sockets.
    It accepts an IP address in binary form.
    
    Sockets are dynamically allocated with this function
    and freed using TCPAbort or TCPClose.

  Conditions:
    TCP is initialized.

  Input:
    addType     - IPv4/IPv6 address type spec
    opType      - TCP_OPEN_SERVER/TCP_OPEN_CLIENT ro open a server or a client socket
    port        - for a client socket this is the remote port
                  for a server socket this is the local port the server is listening on
    hostAddress - A big endian IP address
                  for a client socket this is the address of the host to connect to
                  for a server socket this is not currently used
                  (it may be used in the future for local binding though).
                 

  Return Values:
    INVALID_SOCKET -  No sockets of the specified type were available to be
                      opened.
    Otherwise -       A TCP_SOCKET handle. Save this handle and use it when
                      calling all other TCP APIs.

  Remarks:
	When finished using the TCP socket handle, call the TCPClose() function to free the 
	socket and delete the handle.

    IP_ADDRESS_TYPE_ANY is supported for server sockets only!

  *****************************************************************************/
static TCP_SOCKET TCPOpen(IP_ADDRESS_TYPE addType, TCP_OPEN_TYPE opType, UDP_PORT port, IP_MULTI_ADDRESS* hostAddress)
{
    TCB_STUB*  pSkt;
    TCP_SOCKET hTCP;
    TCP_PORT   localPort, remotePort;     
    uint8_t    *txBuff, *rxBuff;

    if(opType == TCP_OPEN_CLIENT)
    {
        localPort = 0;
        remotePort = port;
    }
    else
    {
        localPort = port;
        remotePort = 0;
    }

    // allocate an ephemeral port for both server and client
    if(localPort == 0)
    {
        localPort = TcpAllocateEphemeralPort();
        if(localPort  == 0)
        {
            return INVALID_SOCKET;
        }
    }


    pSkt = (TCB_STUB*)1;
    for(hTCP = 0; hTCP < TcpSockets; hTCP++)
    {
        pSkt = TCBStubs[hTCP];
        if(pSkt == 0)
        {   // found an empty slot
            break;
        }
    }


    if(pSkt != 0)
    {   // all slots taken
        return INVALID_SOCKET;
    }

    pSkt = (TCB_STUB*)TCPIP_HEAP_Calloc(tcpHeapH, 1, sizeof(*pSkt));
    txBuff = (uint8_t*)TCPIP_HEAP_Malloc(tcpHeapH, tcpDefTxSize + 1);
    rxBuff = (uint8_t*)TCPIP_HEAP_Malloc(tcpHeapH, tcpDefRxSize + 1);

    if(pSkt == 0 || txBuff == 0 || rxBuff == 0)
    {   // out of memory
        TCPIP_HEAP_Free(tcpHeapH, rxBuff);
        TCPIP_HEAP_Free(tcpHeapH, txBuff);
        TCPIP_HEAP_Free(tcpHeapH, pSkt);
        return INVALID_SOCKET;
    }

    _TcpSocketInitialize(pSkt, hTCP, txBuff, tcpDefTxSize, rxBuff, tcpDefRxSize);
    // set all the proper members 
    _TcpSocketSetIdleState(pSkt);

    switch(addType)
    {
#if defined (TCPIP_STACK_USE_IPV6)
        case IP_ADDRESS_TYPE_IPV6:
            pSkt->pTxPkt = TCPv6AllocateTxPacketStruct (pSkt);
            break;
#endif  // defined (TCPIP_STACK_USE_IPV6)

#if defined (TCPIP_STACK_USE_IPV4)
        case IP_ADDRESS_TYPE_IPV4:
            pSkt->pTxPkt = TCPv4AllocateTxPacketStruct (pSkt);
            break;
#endif  // defined (TCPIP_STACK_USE_IPV4)

        default:
            pSkt->addType = IP_ADDRESS_TYPE_ANY;
            pSkt->pTxPkt = 0;   // default for IP_ADDRESS_TYPE_ANY
            break;
    }

    if (pSkt->pTxPkt == 0 && pSkt->addType != IP_ADDRESS_TYPE_ANY)
    {   // failed to allocate memory
        _TcpSocketKill(pSkt);
        return INVALID_SOCKET;
    }

    pSkt->flags.openAddType = addType;

    if(opType == TCP_OPEN_SERVER)
    {
        pSkt->localPort.Val = localPort;
        pSkt->Flags.bServer = true;
        pSkt->smState = TCP_LISTEN;
        pSkt->remoteHash.Val = localPort;
#if defined(TCPIP_STACK_USE_SSL_SERVER)
        pSkt->localSSLPort.Val = 0;
#endif
    }
    // Handle all the client mode socket types
    else
    {   // TCP_CLIENT_SOCKET
        // Set the non-zero TCB fields
        pSkt->localPort.Val = localPort;
        pSkt->remotePort.Val = remotePort;

        // Flag to start the DNS, ARP, SYN processes
        pSkt->eventTime = SYS_TICK_Get();
        pSkt->Flags.bTimerEnabled = 1;

        switch(addType)
        {
#if defined (TCPIP_STACK_USE_IPV4)
            case IP_ADDRESS_TYPE_IPV4:
                // hostAddress is a literal IP address.
                // This doesn't need DNS and can skip directly to the Gateway ARPing step.
                {
                    IPV4_ADDR v4Add;
                    v4Add.Val = hostAddress?hostAddress->v4Add.Val:0;
                    pSkt->remoteHash.Val = TcpClientIpv4RemoteHash(&v4Add, pSkt);
                    TCPIP_IPV4_SetDestAddress(pSkt->pTxPkt, v4Add.Val);
                    TCPIP_IPV4_SetPacketIPProtocol (pSkt->pTxPkt);
                    pSkt->retryCount = 0;
                    pSkt->retryInterval = (SYS_TICK_TicksPerSecondGet()/4);
                    pSkt->smState = TCP_GATEWAY_SEND_ARP;
                }
                break;
#endif  // defined (TCPIP_STACK_USE_IPV4)

#if defined (TCPIP_STACK_USE_IPV6)
            case IP_ADDRESS_TYPE_IPV6:
                // hostAddress is a pointer to an IPv6 address. 
                // This doesn't need DNS.
                pSkt->remoteHash.Val = TCPIP_IPV6_GetHash(&hostAddress->v6Add, remotePort, pSkt->localPort.Val);
                TCPIP_IPV6_SetDestAddress(pSkt->pTxPkt, &hostAddress->v6Add);
                TCPIP_IPV6_SetPacketIPProtocol (pSkt->pTxPkt);
                pSkt->retryCount = 0;
                pSkt->retryInterval = (SYS_TICK_TicksPerSecondGet()/4);
                pSkt->smState = TCP_SYN_SENT;
                _TcpSend(pSkt, SYN, SENDTCP_RESET_TIMERS);
                break;
#endif  // defined (TCPIP_STACK_USE_IPV6)

            default:
                break;  // IP_ADDRESS_TYPE_ANY
        }
    }

    return hTCP;		

}

#if defined (TCPIP_STACK_USE_IPV4)
static IPV4_PACKET * TCPv4AllocateTxPacketStruct (TCB_STUB * pSkt)
{
    IPV4_PACKET * pkt;

    pkt = TCPIP_IPV4_AllocateTxPacket (pSkt->pSktNet, _Tcpv4AckFnc, pSkt);

    if (pkt == NULL)
        return NULL;

    if (!TCPIP_IPV4_PutUpperLayerHeader (pkt, NULL, sizeof (TCP_HEADER), IP_PROT_TCP, TCP_CHECKSUM_OFFSET))
    {
        TCPIP_IPV4_FreePacket (pkt);
        return NULL;
    }

    pSkt->addType = IP_ADDRESS_TYPE_IPV4;
    return pkt;
}

static void _Tcpv4AckFnc (void * pkt, bool sent, void * param)
{
    if (pkt)
    {
        TCB_STUB* pSkt = (TCB_STUB*)param;
        if(pSkt == TCBStubs[pSkt->sktIx] && pSkt->pTxPkt == pkt)
        {
            TCPIP_IPV4_ResetTransmitPacketState (pSkt->pTxPkt);
        }
        else
        {
            TCPIP_IPV4_FreePacket (pkt);
        }
    }
}

#endif  // defined (TCPIP_STACK_USE_IPV4)

#if defined (TCPIP_STACK_USE_IPV6)
static IPV6_PACKET * TCPv6AllocateTxPacketStruct (TCB_STUB * pSkt)
{
    IPV6_PACKET * pkt;

    pkt = TCPIP_IPV6_AllocateTxPacket (pSkt->pSktNet, _Tcpv6AckFnc, pSkt);

    if (pkt == NULL)
        return NULL;

    if (!TCPIP_IPV6_PutUpperLayerHeader (pkt, NULL, sizeof (TCP_HEADER), IP_PROT_TCP, TCP_CHECKSUM_OFFSET))
    {
        TCPIP_IPV6_FreePacket (pkt);
        return NULL;
    }

    pSkt->addType = IP_ADDRESS_TYPE_IPV6;
    return pkt;
}

static void _Tcpv6AckFnc (void * pkt, bool sent, void * param)
{
    if (pkt)
    {
        TCB_STUB* pSkt = (TCB_STUB*)param;
        if(pSkt == TCBStubs[pSkt->sktIx] && pSkt->pTxPkt == pkt)
        {
            TCPIP_IPV6_ResetTransmitPacketState (pSkt->pTxPkt);
        }
        else
        {
            TCPIP_IPV6_FreePacket (pkt);
        }
    }
}

#endif  // defined (TCPIP_STACK_USE_IPV6)


/*****************************************************************************
  Function:
	bool TCPWasReset(TCP_SOCKET hTCP)

  Summary:
	Self-clearing semaphore inidicating socket reset.

  Description:
	This function is a self-clearing semaphore indicating whether or not
	a socket has been disconnected since the previous call.  This function
	works for all possible disconnections: a call to TCPDisconnect, a FIN 
	from the remote node, or an acknowledgement timeout caused by the loss
	of a network link.  It also returns true after the first call to TCPInit.
	Applications should use this function to reset their state machines.
	
	This function was added due to the possibility of an error when relying
	on TCPIsConnected returing false to check for a condition requiring a
	state machine reset.  If a socket is closed (due to a FIN ACK) and then
	immediately reopened (due to a the arrival of a new SYN) in the same
	cycle of the stack, calls to TCPIsConnected by the application will 
	never return false even though the socket has been disconnected.  This 
	can cause errors for protocols such as HTTP in which a client will 
	immediately open a new connection upon closing of a prior one.  Relying
	on this function instead allows applications to trap those conditions 
	and properly reset their internal state for the new connection.

  Precondition:
	TCP is initialized.

  Parameters:
	hTCP - The socket to check.

  Return Values:
  	true - The socket has been disconnected since the previous call.
  	false - The socket has not been disconnected since the previous call.
  ***************************************************************************/
bool TCPWasReset(TCP_SOCKET hTCP)
{
    TCB_STUB* pSkt = _TcpSocketChk(hTCP); 

    if(pSkt)
    {
        if(pSkt->Flags.bSocketReset)
        {
            pSkt->Flags.bSocketReset = 0;
            return true;
        }	
        return false;
    }

    // if no such socket, probably it was closed and disappeared
    return true;
}


/*****************************************************************************
  Function:
	bool TCPIsConnected(TCP_SOCKET hTCP)

  Summary:
	Determines if a socket has an established connection.

  Description:
	This function determines if a socket has an established connection to 
	a remote node.  Call this function after calling TCPOpen to determine 
	when the connection is set up and ready for use.  This function was 
	historically used to check for disconnections, but TCPWasReset is now a
	more appropriate solution. 

  Precondition:
	TCP is initialized.

  Parameters:
	hTCP - The socket to check.

  Return Values:
  	true - The socket has an established connection to a remote node.
  	false - The socket is not currently connected.

  Remarks:
	A socket is said to be connected only if it is in the TCP_ESTABLISHED
	state.  Sockets in the process of opening or closing will return false.
  ***************************************************************************/
bool TCPIsConnected(TCP_SOCKET hTCP)
{
    TCB_STUB* pSkt = _TcpSocketChk(hTCP); 
    TCP_STATE smState;

    if(pSkt)
    {
        smState = pSkt->smState;
        if(smState == TCP_ESTABLISHED || smState == TCP_FIN_WAIT_1 || smState == TCP_FIN_WAIT_2 || smState == TCP_CLOSE_WAIT)
        {
            return true;
        }
    }

    return false;
}


// This function closes a connection to a remote node by sending a FIN
// (if currently connected).
bool TCPDisconnect(TCP_SOCKET hTCP)
{
    TCB_STUB* pSkt = _TcpSocketChk(hTCP); 
	
    if(pSkt)
    {
        _TCP_SEND_RES sRes = _TcpDisconnect(pSkt, true);
        return sRes >= 0;
    }

    return false;
}


static _TCP_SEND_RES _TcpDisconnect(TCB_STUB* pSkt, bool signalFIN)
{
    uint8_t         tcpFlags;
    bool            sendData;
    _TCP_SEND_RES   sendRes;

	switch(pSkt->smState)
	{

		case TCP_SYN_RECEIVED:
		case TCP_ESTABLISHED:
#if defined(TCPIP_STACK_USE_SSL)
            // When disconnecting SSL sockets, send a close_notify so we can resume later
            if(pSkt->Flags.failedDisconnect == 0)
            { 
                if(pSkt->sslStubID != SSL_INVALID_ID)
                {
                    // Flush pending data and send close_notify
                    SSLTxRecord(pSkt->sktIx, pSkt->sslStubID, SSL_APPLICATION);
                    SSLTxMessage(pSkt->sktIx, pSkt->sslStubID, SSL_ALERT_CLOSE_NOTIFY);
                }
            }
#endif  // defined(TCPIP_STACK_USE_SSL)

            // fall through
		case TCP_CLOSE_WAIT:
			// Send the FIN.
            // If the socket linger is on we should keep the socket opened (for the specified timeout)
            // until all the queued TX data can be sent in the background
            // Otherwise the queued TX data is discarded.
            // However since we don't support background operation for now:
            // - if linger is on: we'll try to send all the data before sending FIN
            // - if linger is off: discard the TX data, just send FIN
            //
            //
            //
            if(pSkt->flags.nonLinger != 0)
            {
                _TcpDiscardTx(pSkt);
            }

            // send something out only if we need a FIN or we have some data
            sendData = (signalFIN || (pSkt->txHead != pSkt->txUnackedTail));

            if(sendData)
            {
                // Transmit our last data.
                // This is done in a loop to ensure that if we have 
                // more data wating in the TX FIFO than can be sent in a single 
                // packet (due to the remote Max Segment Size packet size limit), 
                // we will keep generating more packets until either all data gets 
                // transmitted or the remote node's receive window fills up.
                tcpFlags = signalFIN? FIN | ACK : ACK;
                do
                {   
                    sendRes = _TcpSend(pSkt, tcpFlags, SENDTCP_RESET_TIMERS);
                    if(sendRes < 0 || pSkt->remoteWindow == 0u)
                        break;
                } while(pSkt->txHead != pSkt->txUnackedTail);
            }
            else
            {
                sendRes = _TCP_SEND_OK; 
            }
			
            if(sendRes < 0)
            {   
                // let the user know
                // another attempt may be done
                // if it's followed by an close/abort won't matter anyway
                pSkt->Flags.failedDisconnect = 1;
            }
            else
            {
                pSkt->Flags.failedDisconnect = 0;
                pSkt->smState = pSkt->smState == TCP_CLOSE_WAIT?TCP_LAST_ACK:TCP_FIN_WAIT_1;
            }
			break;

		case TCP_FIN_WAIT_1:
		case TCP_FIN_WAIT_2:
		case TCP_CLOSING:
		case TCP_LAST_ACK:
            sendRes = _TCP_SEND_OK;
            break;
			
		case TCP_CLOSED_BUT_RESERVED:
            // special client socket state
            _TcpCloseSocket(pSkt);
            sendRes = _TCP_SEND_OK;
            break;

		default:
            sendRes = _TCP_SEND_NOT_CONN;
			break;
	}

    return sendRes;

}

// aborts a connection sending a RST to the other end of the socket
void TCPAbort(TCP_SOCKET hTCP, bool killSocket)
{
    TCB_STUB* pSkt = _TcpSocketChk(hTCP); 
	
    if(pSkt)
    {
        _TcpAbort(pSkt, killSocket);
    }

}

// internal function to abort a connection
static void _TcpAbort(TCB_STUB* pSkt, bool forceClose)
{
    bool    sktReset = false;

	switch(pSkt->smState)
	{
		case TCP_SYN_RECEIVED:
		case TCP_ESTABLISHED:
		case TCP_CLOSE_WAIT:
		case TCP_FIN_WAIT_1:
		case TCP_FIN_WAIT_2:
		case TCP_LAST_ACK:
            sktReset = true;
			break;

		default:
			break;
	}

    if(sktReset)
    {
        _TcpDiscardTx(pSkt);
        _TcpSend(pSkt, RST | ACK, 0);
    }

    if(forceClose)
    {
        pSkt->flags.forceKill = 1;
    }

    _TcpCloseSocket(pSkt);
}



// Disconnects an open socket and destroys the socket handle
// including server mode socket handles.
void TCPClose(TCP_SOCKET hTCP)
{
    TCB_STUB* pSkt = _TcpSocketChk(hTCP); 

    if(pSkt)
    {
        bool needAbort = false;

        if(pSkt->flags.nonGraceful == 0)
        {   // end gracefully but close anyway
            pSkt->flags.forceKill = 1;
            _TCP_SEND_RES sRes = _TcpDisconnect(pSkt, true);
            if(sRes != _TCP_SEND_OK)
            {   // either failed or socket is not connected
                needAbort = true;
            }
        }
        else
        {
            needAbort = true;
        }

        if(needAbort)
        {
            _TcpAbort(pSkt, true);
        }
    }

}


/*****************************************************************************
  Function:
	bool TCPGetSocketInfo(TCP_SOCKET hTCP, TCP_SOCKET_INFO* remoteInfo)

  Summary:
	Obtains information about a currently open socket.

  Description:
	Fills the provided TCP_SOCKET_INFO structure associated with this socket.  This 
	contains the NODE_INFO structure with IP and MAC address (or gateway
	MAC) and the remote port.

  Precondition:
	TCP is initialized and the socket is connected.

  Parameters:
	hTCP - The socket to check.

  Returns:
    true if the call succeeded
    false if no such socket or the socket is not opened
  ***************************************************************************/
bool TCPGetSocketInfo(TCP_SOCKET hTCP, TCP_SOCKET_INFO* remoteInfo)
{

    TCB_STUB* pSkt = _TcpSocketChk(hTCP); 
	
    if(pSkt == 0)
    {
        return false;
    }

    while(true)
    {
#if defined (TCPIP_STACK_USE_IPV6)
        if (pSkt->addType == IP_ADDRESS_TYPE_IPV6)
        {
            memcpy((void*)&remoteInfo->remoteIPaddress.v6Add.v, (void*)TCPIP_IPV6_GetDestAddress(pSkt->pTxPkt), sizeof(IPV6_ADDR));
            memcpy((void*)&remoteInfo->localIPaddress.v6Add.v, (void*)TCPIP_IPV6_GetSourceAddress(pSkt->pTxPkt), sizeof(IPV6_ADDR));
            remoteInfo->addressType = IP_ADDRESS_TYPE_IPV6;
            break;
        }
#endif  // defined (TCPIP_STACK_USE_IPV6)

#if defined (TCPIP_STACK_USE_IPV4)
        if (pSkt->addType == IP_ADDRESS_TYPE_IPV4)
        {
            remoteInfo->remoteIPaddress.v4Add = TCPIP_IPV4_GetDestAddress(pSkt->pTxPkt);
            remoteInfo->localIPaddress.v4Add = TCPIP_IPV4_GetSourceAddress(pSkt->pTxPkt);
            remoteInfo->addressType = IP_ADDRESS_TYPE_IPV4;
        }
#endif  // defined (TCPIP_STACK_USE_IPV4)
        break;
    }

	remoteInfo->remotePort = pSkt->remotePort.Val;
	remoteInfo->localPort = pSkt->localPort.Val;

	return true;
}

int TCPSocketsNo(void)
{
    return TcpSockets;
}

#if defined(TCPIP_STACK_USE_SSL)
uint8_t TcpSocketGetSSLId(TCP_SOCKET hTCP)
{

    TCB_STUB* pSkt = TCBStubs[hTCP];
    if(pSkt)
    {
        return pSkt->sslStubID;
    }

    return SSL_INVALID_ID;
}


// Handle SSL Message Transmission
void TcpSSLMessageTransmit(TCP_SOCKET hTCP)
{

    TCB_STUB* pSkt = TCBStubs[hTCP];
    if(pSkt && pSkt->sslStubID != SSL_INVALID_ID)
    {
        // If unsent data is waiting, transmit it as an application record
        if(pSkt->sslTxHead != pSkt->txHead && TCPSSLGetPendingTxSize(hTCP) != 0u)
        {
            SSLTxRecord(hTCP, pSkt->sslStubID, SSL_APPLICATION);
        }

        // If an SSL message is requested, send it now
        if(pSkt->sslReqMessage != SSL_NO_MESSAGE)
        {
            SSLTxMessage(hTCP, pSkt->sslStubID, pSkt->sslReqMessage);
        }
    }
}



#endif  // defined(TCPIP_STACK_USE_SSL)

/****************************************************************************
  Section:
	Transmit Functions
  ***************************************************************************/

/*****************************************************************************
  Function:
	void TCPFlush(TCP_SOCKET hTCP)

  Summary:
	Immediately transmits all pending TX data.

  Description:
	This function immediately transmits all pending TX data with a PSH 
	flag.  If this function is not called, data will automatically be sent
	when either a) the TX buffer is half full or b) the 
	TCP_AUTO_TRANSMIT_TIMEOUT_VAL (default: 40ms) has elapsed.

  Precondition:
	TCP is initialized and the socket is connected.

  Parameters:
	hTCP - The socket whose data is to be transmitted.

  Returns:
	None

  Remarks:
	SSL application data is automatically flushed, so this function has 
	no effect for SSL sockets.
  ***************************************************************************/
void TCPFlush(TCP_SOCKET hTCP)
{
    TCB_STUB* pSkt = _TcpSocketChk(hTCP); 

    if(pSkt)
    {
        _TcpFlush(pSkt);
    }
}

static void _TcpFlush(TCB_STUB* pSkt)
{
    // NOTE: Pending SSL data will NOT be transferred here
    if(pSkt->txHead != pSkt->txUnackedTail && pSkt->remoteWindow != 0)
    {   // The check remoteWindow != 0 stops us sending lots of
        // ACKs with len == 0, when the other host is slow
        // Send the TCP segment with all unacked bytes
        _TcpSend(pSkt, ACK, SENDTCP_RESET_TIMERS);
    }
}


static void _TcpDiscardTx(TCB_STUB* pSkt)
{
    // Empty the TX buffer
    pSkt->txHead = pSkt->txTail = pSkt->txUnackedTail = pSkt->txStart;
}

/*****************************************************************************
  Function:
	uint16_t TCPIsPutReady(TCP_SOCKET hTCP)

  Summary:
	Determines how much free space is available in the TCP TX buffer.

  Description:
	Call this function to determine how many bytes can be written to the 
	TCP TX buffer.  If this function returns zero, the application must 
	return to the main stack loop before continuing in order to transmit
	more data.

  Precondition:
	TCP is initialized.

  Parameters:
	hTCP - The socket to check.

  Returns:
	The number of bytes available to be written in the TCP TX buffer.
  ***************************************************************************/
uint16_t TCPIsPutReady(TCP_SOCKET hTCP)
{
    TCB_STUB* pSkt = _TcpSocketChk(hTCP); 
	
    if(pSkt)
    {   // user is asking for more space: flush the buffer if can send at least txBuff/4
        int32_t  canSend = pSkt->txHead - pSkt->txUnackedTail;
        if(canSend < 0)
        {
            canSend += pSkt->txEnd - pSkt->txStart;
        }

        if(canSend >= ((pSkt->txEnd-pSkt->txStart) >> 2))
        {
            _TcpFlush(pSkt);	
        }

        return _TCPIsPutReady(pSkt);
    }

    return 0;
}


static uint16_t _TCPIsPutReady(TCB_STUB* pSkt)
{
    if(pSkt->pTxPkt == 0)
    {   // can happen if it is a server socket and opened with IP_ADDRESS_TYPE_ANY
        // and no client connected to it
        return 0;
    }

    switch(pSkt->addType)
    {   
#if defined (TCPIP_STACK_USE_IPV6)
        case IP_ADDRESS_TYPE_IPV6:
            if( ((IPV6_PACKET*)pSkt->pTxPkt)->flags.queued)
            {   // Try to allocate a new transmit packet
                IPV6_PACKET * tempPtr = TCPv6AllocateTxPacketStruct (pSkt);
                if (tempPtr == 0)
                {   // cannot allocate packet
                    // better chances next time or when the existing packet is unqueued
                    return 0;
                }

                if (!TCPIP_IPV6_CopyTxPacketStruct (tempPtr, (IPV6_PACKET*)&pSkt->pTxPkt))
                {
                    TCPIP_IPV6_FreePacket (tempPtr);
                    return 0;
                }
                pSkt->pTxPkt = tempPtr;
            }
                break;
#endif  // defined (TCPIP_STACK_USE_IPV6)

#if defined (TCPIP_STACK_USE_IPV4)
        case IP_ADDRESS_TYPE_IPV4:
            if( ((IPV4_PACKET*)pSkt->pTxPkt)->flags.queued)
            {   // Try to allocate a new transmit packet
                IPV4_PACKET * tempPtr = TCPv4AllocateTxPacketStruct (pSkt);
                if (tempPtr == 0)
                {   // cannot allocate packet
                    // better chances next time or when the existing packet is unqueued
                    return 0;
                }

                if (!TCPIP_IPV4_CopyTxPacketStruct (tempPtr, (IPV4_PACKET*)&pSkt->pTxPkt))
                {
                    TCPIP_IPV4_FreePacket (tempPtr);
                    return 0;
                }
                pSkt->pTxPkt = tempPtr;
            }
            break;
#endif  // defined (TCPIP_STACK_USE_IPV4)
        default:
            return 0;   // server socket with IP_ADDRESS_TYPE_ANY
    }

	// Unconnected sockets shouldn't be transmitting anything.
	if(!( (pSkt->smState == TCP_ESTABLISHED) || (pSkt->smState == TCP_CLOSE_WAIT) ))
		return 0;

	// Calculate the free space in this socket's TX FIFO
	#if defined(TCPIP_STACK_USE_SSL)
	if(pSkt->sslStubID != SSL_INVALID_ID)
	{// Use sslTxHead as the head pointer when SSL is active
		uint16_t rem;
		
		// Find out raw free space
		if(pSkt->sslTxHead >= pSkt->txTail)
			rem = (pSkt->txEnd - pSkt->txStart - 1) - (pSkt->sslTxHead - pSkt->txTail);
		else
			rem = pSkt->txTail - pSkt->sslTxHead - 1;
		
		// Reserve space for a new MAC and header
		if(rem > 22u)
			return rem - 22;
		else
			return 0;
	}
	#endif
	
	if(pSkt->txHead >= pSkt->txTail)
    {
		return (pSkt->txEnd - pSkt->txStart - 1) - (pSkt->txHead - pSkt->txTail);
    }

	return pSkt->txTail - pSkt->txHead - 1;
}


uint16_t TCPPut(TCP_SOCKET hTCP, uint8_t byte)
{
    return TCPPutArray(hTCP, &byte, 1);
}

/*****************************************************************************
  Function:
	uint16_t TCPPutArray(TCP_SOCKET hTCP, const uint8_t* data, uint16_t len)

  Description:
	Writes an array from RAM to a TCP socket.

  Precondition:
	TCP is initialized.

  Parameters:
	hTCP - The socket to which data is to be written.
	data - Pointer to the array to be written.
	len  - Number of bytes to be written.

  Returns:
	The number of bytes written to the socket.  If less than len, the
	buffer became full or the socket is not conected.
  ***************************************************************************/
uint16_t TCPPutArray(TCP_SOCKET hTCP, const uint8_t* data, uint16_t len)
{
	uint16_t wActualLen;
	uint16_t wFreeTXSpace;
	uint16_t wRightLen = 0;

    TCB_STUB* pSkt = _TcpSocketChk(hTCP); 
	
    if(pSkt == 0)
    {
        return 0;
    }

	wFreeTXSpace = _TCPIsPutReady(pSkt);
	if(wFreeTXSpace == 0u)
	{
		_TcpFlush(pSkt);
		return 0;
	}

	wActualLen = wFreeTXSpace;
	if(wFreeTXSpace > len)
		wActualLen = len;

	// Send all current bytes if we are crossing half full
	// This is required to improve performance with the delayed 
	// acknowledgement algorithm
	if((!pSkt->Flags.bHalfFullFlush) && (wFreeTXSpace <= ((pSkt->txEnd-pSkt->txStart)>>1)))
	{
		_TcpFlush(pSkt);	
		pSkt->Flags.bHalfFullFlush = true;
	}
	
#if defined(TCPIP_STACK_USE_SSL)
	if(pSkt->sslStubID != SSL_INVALID_ID)
	{
		// See if we need a two part put
		if(pSkt->sslTxHead + wActualLen >= pSkt->txEnd)
		{
			wRightLen = pSkt->txEnd-pSkt->sslTxHead;
            memcpy((uint8_t*)pSkt->sslTxHead, data, wRightLen);
			data += wRightLen;
			wActualLen -= wRightLen;
			pSkt->sslTxHead = pSkt->txStart;
		}
	
        memcpy((uint8_t*)pSkt->sslTxHead, data, wActualLen);
		pSkt->sslTxHead += wActualLen;
	}
	else
	{
		// See if we need a two part put
		if(pSkt->txHead + wActualLen >= pSkt->txEnd)
		{
			wRightLen = pSkt->txEnd-pSkt->txHead;
            memcpy((uint8_t*)pSkt->txHead, data, wRightLen);
			data += wRightLen;
			wActualLen -= wRightLen;
			pSkt->txHead = pSkt->txStart;
		}
	
        memcpy((uint8_t*)pSkt->txHead, data, wActualLen);
		pSkt->txHead += wActualLen;
	}
#else
	// See if we need a two part put
	if(pSkt->txHead + wActualLen >= pSkt->txEnd)
	{
		wRightLen = pSkt->txEnd-pSkt->txHead;
        memcpy((uint8_t*)pSkt->txHead, data, wRightLen);
		data += wRightLen;
		wActualLen -= wRightLen;
		pSkt->txHead = pSkt->txStart;
	}

    memcpy((uint8_t*)pSkt->txHead, data, wActualLen);
	pSkt->txHead += wActualLen;
#endif  // defined(TCPIP_STACK_USE_SSL)

	// Send these bytes right now if forceFlush or we are out of TX buffer space
	if(pSkt->flags.forceFlush != 0 || wFreeTXSpace <= len)
	{
		_TcpFlush(pSkt);
	}
	// If not already enabled, start a timer so this data will 
	// eventually get sent even if the application doens't call
	// TCPFlush()
	else if(!pSkt->Flags.bTimer2Enabled)
	{
		pSkt->Flags.bTimer2Enabled = true;
		pSkt->eventTime2 = SYS_TICK_Get() + (TCP_AUTO_TRANSMIT_TIMEOUT_VAL * SYS_TICK_TicksPerSecondGet())/1000;
	}

	return wActualLen + wRightLen;
}

/*****************************************************************************
  Function:
	const uint8_t* TCPPutString(TCP_SOCKET hTCP, const uint8_t* data)

  Description:
	Writes a null-terminated string from RAM to a TCP socket.  The 
	null-terminator is not copied to the socket.

  Precondition:
	TCP is initialized.

  Parameters:
	hTCP - The socket to which data is to be written.
	data - Pointer to the string to be written.

  Returns:
	Pointer to the byte following the last byte written to the socket.  If
	this pointer does not dereference to a NUL byte, the buffer became full
	or the socket is not connected.

  Remarks:
	The return value of this function differs from that of TCPPutArray.  To
	write long strings in a single state, initialize the *data pointer to the
	first byte, then call this function repeatedly (breaking to the main 
	stack loop after each call) until the return value dereferences to a NUL
	byte.  Save the return value as the new starting *data pointer otherwise.
  ***************************************************************************/
const uint8_t* TCPPutString(TCP_SOCKET hTCP, const uint8_t* data)
{
	return data + TCPPutArray(hTCP, data, strlen((char*)data));
}

/*****************************************************************************
  Function:
	uint16_t TCPGetTxFIFOFull(TCP_SOCKET hTCP)

  Description:
	Determines how many bytes are pending in the TCP TX FIFO.

  Precondition:
	TCP is initialized.

  Parameters:
	hTCP - The socket to check.

  Returns:
	Number of bytes pending to be flushed in the TCP TX FIFO.
  ***************************************************************************/
uint16_t TCPGetTxFIFOFull(TCP_SOCKET hTCP)
{
    TCB_STUB* pSkt = _TcpSocketChk(hTCP); 
	
    if(pSkt)
    {
        uint16_t wDataLen;
        uint16_t wFIFOSize;

        // Calculate total usable FIFO size
        wFIFOSize = pSkt->txEnd - pSkt->txStart - 1;

        // Find out how many data bytes are free in the TX FIFO
        wDataLen = _TCPIsPutReady(pSkt);

        return wFIFOSize - wDataLen;
    }

    return 0;
}

/****************************************************************************
  Section:
	Receive Functions
  ***************************************************************************/

/*****************************************************************************
  Function:
	uint16_t TCPDiscard(TCP_SOCKET hTCP)

  Description:
	Discards any pending data in the TCP RX FIFO.

  Precondition:
	TCP is initialized.

  Parameters:
	hTCP - The socket whose RX FIFO is to be cleared.

  Returns:
	Number of bytes that have been discarded from the RX buffer.
  ***************************************************************************/
uint16_t TCPDiscard(TCP_SOCKET hTCP)
{
    uint16_t nBytes = 0;

    TCB_STUB* pSkt = _TcpSocketChk(hTCP); 

    if(pSkt)
    {
        nBytes = _TCPIsGetReady(pSkt);
        if(nBytes)
        {
            // Delete all data in the RX buffer
            pSkt->rxTail = pSkt->rxHead;
            _TCPSendWinIncUpdate(pSkt);
        }
    }

    return nBytes;
}

// checks if a Window increased update is needed
// returns true if it was issued, false otherwise
// tries to avoid the Silly Window Syndrome (SWS) on the RX side
// it will send the Win update if all RX buffer is available
static bool _TCPSendWinIncUpdate(TCB_STUB* pSkt)
{
    uint16_t    oldWin, newWin, minWinInc, rxBuffSz;
    bool    toAdvertise = false;

    // previously advertised window
    oldWin = pSkt->localWindow;

    // new window is all the available RX buffer
    newWin = _TCPGetRxFIFOFree(pSkt);
    rxBuffSz = pSkt->rxEnd - pSkt->rxStart;

    if(newWin == rxBuffSz)
    {   // whole RX buffer available, advertise
        toAdvertise = true;
    }
    else if(newWin > oldWin)
    {   // should always be the case; just a sanity check
        minWinInc = rxBuffSz >> 1; // half RX buffer
        if(minWinInc > pSkt->localMSS)
        {
            minWinInc = pSkt->localMSS;    // minimum of MSS, 1/2 RX buffer
        }

        if((newWin - oldWin) > minWinInc)
        {   // Send a Window update message to the remote node
            toAdvertise = true;
        }
    }

    if(toAdvertise)
    {
        _TcpSend(pSkt, ACK, SENDTCP_RESET_TIMERS);
        return true;
    }

    return false;
}

/*****************************************************************************
  Function:
	void uint16_t TCPIsGetReady(TCP_SOCKET hTCP)

  Summary:
	Determines how many bytes can be read from the TCP RX buffer.

  Description:
	Call this function to determine how many bytes can be read from the 
	TCP RX buffer.  If this function returns zero, the application must 
	return to the main stack loop before continuing in order to wait for
	more data to arrive.

  Precondition:
	TCP is initialized.

  Parameters:
	hTCP - The socket to check.

  Returns:
	The number of bytes available to be read from the TCP RX buffer.
  ***************************************************************************/
uint16_t TCPIsGetReady(TCP_SOCKET hTCP)
{
    TCB_STUB* pSkt = _TcpSocketChk(hTCP); 
	
    if(pSkt)
    {
        return _TCPIsGetReady(pSkt);
    }

    return 0;
}

static uint16_t _TCPIsGetReady(TCB_STUB* pSkt)
{	
	if(pSkt->rxHead >= pSkt->rxTail)
    {
		return pSkt->rxHead - pSkt->rxTail;
    }

    return (pSkt->rxEnd - pSkt->rxTail + 1) + (pSkt->rxHead - pSkt->rxStart);
}

uint16_t TCPGet(TCP_SOCKET hTCP, uint8_t* byte)
{
    return TCPGetArray(hTCP, byte, 1);
}


/*****************************************************************************
  Function:
	uint16_t TCPGetArray(TCP_SOCKET hTCP, uint8_t* buffer, uint16_t len)

  Description:
	Reads an array of data bytes from a TCP socket's receive FIFO.  The data 
	is removed from the FIFO in the process.

  Precondition:
	TCP is initialized.

  Parameters:
	hTCP - The socket from which data is to be read.
	buffer - Pointer to the array to store data that was read.
	len  - Number of bytes to be read.

  Returns:
	The number of bytes read from the socket.  If less than len, the
	RX FIFO buffer became empty or the socket is not conected.
  ***************************************************************************/
uint16_t TCPGetArray(TCP_SOCKET hTCP, uint8_t* buffer, uint16_t len)
{
	uint16_t wGetReadyCount;
	uint16_t RightLen = 0;
    TCB_STUB* pSkt = _TcpSocketChk(hTCP); 
	
	// See if there is any data which can be read
    if(pSkt == 0 || (wGetReadyCount = _TCPIsGetReady(pSkt)) == 0)
    {
        return 0;
    }

	// Make sure we don't try to read more data than is available
	if(len > wGetReadyCount)
    {
		len = wGetReadyCount;
    }

	// See if we need a two part get
	if(pSkt->rxTail + len > pSkt->rxEnd)
	{
		RightLen = pSkt->rxEnd - pSkt->rxTail + 1;
		if(buffer)
		{
			memcpy(buffer, (uint8_t*)pSkt->rxTail, RightLen);
			buffer += RightLen;
		}
		len -= RightLen;
		pSkt->rxTail = pSkt->rxStart;
	}

	if(buffer)
    {
		memcpy(buffer, (uint8_t*)pSkt->rxTail, len);
    }
	pSkt->rxTail += len;
	len += RightLen;

    if(!_TCPSendWinIncUpdate(pSkt))
    {   // not enough data freed to generate a window update
        if(wGetReadyCount - len <= len)
        {   // Send a window update if we've run low on data
            pSkt->Flags.bTXASAPWithoutTimerReset = 1;
        }
        else if(!pSkt->Flags.bTimer2Enabled)
            // If not already enabled, start a timer so a window 
            // update will get sent to the remote node at some point
        {
            pSkt->Flags.bTimer2Enabled = true;
            pSkt->eventTime2 = SYS_TICK_Get() + (TCP_WINDOW_UPDATE_TIMEOUT_VAL * SYS_TICK_TicksPerSecondGet())/1000;
        }
    }

	return len;
}


/*****************************************************************************
  Function:
	uint16_t TCPGetRxFIFOFree(TCP_SOCKET hTCP)

  Description:
	Determines how many bytes are free in the RX FIFO.

  Precondition:
	TCP is initialized.

  Parameters:
	hTCP - The socket to check.

  Returns:
	The number of bytes free in the TCP RX FIFO.  If zero, no additional 
	data can be received until the application removes some data using one
	of the TCPGet family functions.
  ***************************************************************************/
uint16_t TCPGetRxFIFOFree(TCP_SOCKET hTCP)
{
    TCB_STUB* pSkt = _TcpSocketChk(hTCP); 
	
    if(pSkt != 0)
    {
        return _TCPGetRxFIFOFree(pSkt);
    }

    return 0;
}


static uint16_t _TCPGetRxFIFOFree(TCB_STUB* pSkt)
{

    uint16_t wDataLen;
    uint16_t wFIFOSize;

    // Calculate total usable FIFO size
    wFIFOSize = pSkt->rxEnd - pSkt->rxStart;

#if defined(TCPIP_STACK_USE_SSL)
    {
        uint8_t* SSLtemp = pSkt->rxHead;

        // Move SSL pointer to determine full buffer size
        if(pSkt->sslStubID != SSL_INVALID_ID)
        {
            pSkt->rxHead = pSkt->sslRxHead;
        }

        // Find out how many data bytes are actually in the RX FIFO
        wDataLen = _TCPIsGetReady(pSkt);

        // Move SSL pointer back to proper location (if we changed it)
        pSkt->rxHead = SSLtemp;
    }
#else
    {
        // Find out how many data bytes are actually in the RX FIFO
        wDataLen = _TCPIsGetReady(pSkt);
    }
#endif

    // Perform the calculation	
    return wFIFOSize - wDataLen;
}

/*****************************************************************************
  Function:
	uint16_t TCPPeekArray(TCP_SOCKET hTCP, uint8_t *vBuffer, uint16_t wLen, uint16_t wStart)

  Summary:
  	Reads a specified number of data bytes from the TCP RX FIFO without 
  	removing them from the buffer.

  Description:
	Reads a specified number of data bytes from the TCP RX FIFO without 
  	removing them from the buffer.  No TCP control actions are taken as a 
  	result of this function (ex: no window update is sent to the remote node).
  	
  Precondition:
	TCP is initialized.

  Parameters:
	hTCP - The socket to peak from (read without removing from stream).
	vBuffer - Destination to write the peeked data bytes.
	wLen - Length of bytes to peak from the RX FIFO and copy to vBuffer.
	wStart - Zero-indexed starting position within the FIFO to start peeking 
		from.

  Return Values:
	Number of bytes actually peeked from the stream and copied to vBuffer.  
	This value can be less than wLen if wStart + wLen is greater than the 
	deepest possible character in the RX FIFO.

  Remarks:
  	None
  ***************************************************************************/
uint16_t TCPPeekArray(TCP_SOCKET hTCP, uint8_t *vBuffer, uint16_t wLen, uint16_t wStart)
{
    uint8_t* ptrRead;
    uint16_t w;
    uint16_t wBytesUntilWrap;
    TCB_STUB* pSkt = _TcpSocketChk(hTCP); 

    if(pSkt == 0 || wLen == 0)
    {
        return 0;
    }

    // Find out how many bytes are in the RX FIFO and decrease read length 
    // if the start offset + read length is beyond the end of the FIFO
    w = _TCPIsGetReady(pSkt);
    if(wStart + wLen > w)
    {
        wLen = w - wStart;
    }

    // Find the read start location
    ptrRead = pSkt->rxTail + wStart;
    if(ptrRead > pSkt->rxEnd)
    {
        ptrRead -= pSkt->rxEnd - pSkt->rxStart + 1;
    }

    // Calculate how many bytes can be read in a single go
    wBytesUntilWrap = pSkt->rxEnd - ptrRead + 1;
    if(wLen <= wBytesUntilWrap)
    {
        // Read all at once
        memcpy(vBuffer, ptrRead, wLen);
    }
    else
    {
        // Read all bytes up to the wrap position and then read remaining bytes 
        // at the start of the buffer
        memcpy(vBuffer, ptrRead, wBytesUntilWrap);
        memcpy(vBuffer + wBytesUntilWrap, (uint8_t*)pSkt->rxStart, wLen - wBytesUntilWrap);
    }

    return wLen;
}

/*****************************************************************************
  Function:
	uint8_t TCPPeek(TCP_SOCKET hTCP, uint16_t wStart)

  Summary:
  	Peaks at one byte in the TCP RX FIFO without removing it from the buffer.

  Description:
	Peaks at one byte in the TCP RX FIFO without removing it from the buffer.
  	
  Precondition:
	TCP is initialized.

  Parameters:
	hTCP - The socket to peak from (read without removing from stream).
	wStart - Zero-indexed starting position within the FIFO to peek from.

  Return Values:
	Byte peeked from the RX FIFO.  If there is no data in the buffer or an 
	illegal wStart starting offset is given, then an indeterminate value is 
	returned.  The caller must ensure that valid parameters are passed to avoid 
	(i.e ensure that TCPIsGetReady() returns a number that is less than wStart 
	before calling TCPPeek()).

  Remarks:
  	Use the TCPPeekArray() function to read more than one byte.  It will 
  	perform better than calling TCPPeek() in a loop.
  ***************************************************************************/
uint8_t TCPPeek(TCP_SOCKET hTCP, uint16_t wStart)
{
	uint8_t i;
	
	TCPPeekArray(hTCP, &i, 1, wStart);
	return i;
}


/****************************************************************************
  Section:
	Search Functions
  ***************************************************************************/

/*****************************************************************************
  Function:
	uint16_t TCPFindArray(TCP_SOCKET hTCP, uint8_t* cFindArray, uint16_t wLen, 
						uint16_t wStart, uint16_t wSearchLen, bool bTextCompare)

  Summary:
  	Searches for a string in the TCP RX buffer.

  Description:
	This function finds the first occurrance of an array of bytes in the
	TCP RX buffer.  It can be used by an application to abstract searches 
	out of their own application code.  For increased efficiency, the 
	function is capable of limiting the scope of search to a specific
	range of bytes.  It can also perform a case-insensitive search if
	required.
	
	For example, if the buffer contains "I love PIC MCUs!" and the search
	array is "love" with a length of 4, a value of 2 will be returned.

  Precondition:
	TCP is initialized.

  Parameters:
	hTCP - The socket to search within.
	cFindArray - The array of bytes to find in the buffer.
	wLen - Length of cFindArray.
	wStart - Zero-indexed starting position within the buffer.
	wSearchLen - Length from wStart to search in the buffer.
	bTextCompare - true for case-insensitive text search, false for binary search

  Return Values:
	0xFFFF - Search array not found
	Otherwise - Zero-indexed position of the first occurrance

  Remarks:
	Since this function usually must transfer data from external storage
	to internal RAM for comparison, its performance degrades significantly
	when the buffer is full and the array is not found.  For better 
	performance, try to search for characters that are expected to exist or
	limit the scope of the search as much as possible.  The HTTP2 module, 
	for example, uses this function to parse headers.  However, it searches 
	for newlines, then the separating colon, then reads the header name to 
	RAM for final comparison.  This has proven to be significantly faster  
	than searching for full header name strings outright.
  ***************************************************************************/
uint16_t TCPFindArray(TCP_SOCKET hTCP, const uint8_t* cFindArray, uint16_t wLen, uint16_t wStart, uint16_t wSearchLen, bool bTextCompare)
{
	uint8_t* ptrRead;
	uint16_t wDataLen;
	uint16_t wBytesUntilWrap;
	uint8_t* ptrLocation;
	uint16_t wLenStart;
	const uint8_t *cFindArrayStart;
	uint8_t i, j, k;
	bool isFinding;
	uint8_t buffer[32];

    TCB_STUB* pSkt = _TcpSocketChk(hTCP); 
	
    if(pSkt == 0 || wLen == 0)
    {
        return 0;
    }



	// Find out how many bytes are in the RX FIFO and return 
	// immediately if we won't possibly find a match
	wDataLen = _TCPIsGetReady(pSkt) - wStart;
	if(wDataLen < wLen)
    {
		return 0xFFFFu;
    }

	if(wSearchLen && (wDataLen > wSearchLen))
    {
		wDataLen = wSearchLen;
    }

	ptrLocation = pSkt->rxTail + wStart;
	if(ptrLocation > pSkt->rxEnd)
    {
		ptrLocation -= pSkt->rxEnd - pSkt->rxStart + 1;
    }
	ptrRead = ptrLocation;
	wBytesUntilWrap = pSkt->rxEnd - ptrLocation + 1;
	wLenStart = wLen;
	cFindArrayStart = cFindArray;
	j = *cFindArray++;
	isFinding = false;
	if(bTextCompare)
	{
		if(j >= 'a' && j <= 'z')
        {
			j += 'A'-'a';
        }
	}

	// Search for the array
	while(1)
	{
		// Figure out how big of a chunk to read
		k = sizeof(buffer);
		if(k > wBytesUntilWrap)
        {
			k = wBytesUntilWrap;
        }
		if((uint16_t)k > wDataLen)
        {
			k = wDataLen;
        }

		// Read a chunk of data into the buffer
		memcpy(buffer, ptrRead, k);
		ptrRead += k;
		wBytesUntilWrap -= k;

		if(wBytesUntilWrap == 0u)
		{
			ptrRead = pSkt->rxStart;
			wBytesUntilWrap = 0xFFFFu;
		}

		// Convert everything to uppercase
		if(bTextCompare)
		{
			for(i = 0; i < k; i++)
			{
				if(buffer[i] >= 'a' && buffer[i] <= 'z')
                {
					buffer[i] += 'A'-'a';
                }

				if(j == buffer[i])
				{
					if(--wLen == 0u)
                    {
						return wStart-wLenStart + i + 1;
                    }
					j = *cFindArray++;
					isFinding = true;
					if(j >= 'a' && j <= 'z')
                    {
						j += 'A'-'a';
                    }
				}
				else
				{
					wLen = wLenStart;
					if(isFinding)
					{
						cFindArray = cFindArrayStart;
						j = *cFindArray++;
						if(j >= 'a' && j <= 'z')
                        {
							j += 'A'-'a';
                        }
						isFinding = false;
					}
				}
			}
		}
		else	// Compare as is
		{
			for(i = 0; i < k; i++)
			{
				if(j == buffer[i])
				{
					if(--wLen == 0u)
                    {
						return wStart-wLenStart + i + 1;
                    }
					j = *cFindArray++;
					isFinding = true;
				}
				else
				{
					wLen = wLenStart;
					if(isFinding)
					{
						cFindArray = cFindArrayStart;
						j = *cFindArray++;
						isFinding = false;
					}
				}
			}
		}

		// Check to see if it is impossible to find a match
		wDataLen -= k;
		if(wDataLen < wLen)
        {
			return 0xFFFFu;
        }

		wStart += k;
	}
}

/*****************************************************************************
  Function:
	uint16_t TCPFind(TCP_SOCKET hTCP, uint8_t cFind,
						uint16_t wStart, uint16_t wSearchLen, bool bTextCompare)

  Summary:
  	Searches for a byte in the TCP RX buffer.

  Description:
	This function finds the first occurrance of a byte in the TCP RX
	buffer.  It can be used by an application to abstract searches 
	out of their own application code.  For increased efficiency, the 
	function is capable of limiting the scope of search to a specific
	range of bytes.  It can also perform a case-insensitive search if
	required.
	
	For example, if the buffer contains "I love PIC MCUs!" and the cFind
	byte is ' ', a value of 1 will be returned.

  Precondition:
	TCP is initialized.

  Parameters:
	hTCP - The socket to search within.
	cFind - The byte to find in the buffer.
	wStart - Zero-indexed starting position within the buffer.
	wSearchLen - Length from wStart to search in the buffer.
	bTextCompare - true for case-insensitive text search, false for binary search

  Return Values:
	0xFFFF - Search array not found
	Otherwise - Zero-indexed position of the first occurrance

  Remarks:
	Since this function usually must transfer data from external storage
	to internal RAM for comparison, its performance degrades significantly
	when the buffer is full and the array is not found.  For better 
	performance, try to search for characters that are expected to exist or
	limit the scope of the search as much as possible.  The HTTP2 module, 
	for example, uses this function to parse headers.  However, it searches 
	for newlines, then the separating colon, then reads the header name to 
	RAM for final comparison.  This has proven to be significantly faster  
	than searching for full header name strings outright.
  ***************************************************************************/
uint16_t TCPFind(TCP_SOCKET hTCP, uint8_t cFind, uint16_t wStart, uint16_t wSearchLen, bool bTextCompare)
{
	return TCPFindArray(hTCP, &cFind, sizeof(cFind), wStart, wSearchLen, bTextCompare);
}



/****************************************************************************
  Section:
	Data Processing Functions
  ***************************************************************************/

/*****************************************************************************
  Function:
	void TCPTick(void)

  Summary:
  	Performs periodic TCP tasks.

  Description:
	This function performs any required periodic TCP tasks.  Each 
	socket's state machine is checked, and any elapsed timeout periods
	are handled.

  Precondition:
	TCP is initialized.

  Parameters:
	None

  Returns:
	None
  ***************************************************************************/
void TCPTick(void)
{
	TCP_SOCKET hTCP;
	bool bRetransmit;
	bool bCloseSocket;
	uint8_t vFlags;
	uint16_t w;
    TCB_STUB* pSkt; 

	// Periodically all "not closed" sockets must perform timed operations
	for(hTCP = 0; hTCP < TcpSockets; hTCP++)
	{
        pSkt = TCBStubs[hTCP];
        if(pSkt)
        {   // existing socket
            vFlags = 0x00;
            bRetransmit = false;
            bCloseSocket = false;

            // Transmit ASAP data if the medium is available
            if(pSkt->Flags.bTXASAP || pSkt->Flags.bTXASAPWithoutTimerReset)
            {
                if(TCP_IsTxReady(pSkt))
                {
                    vFlags = ACK;
                    bRetransmit = pSkt->Flags.bTXASAPWithoutTimerReset;
                }
            }

            // Perform any needed window updates and data transmissions
            if(pSkt->Flags.bTimer2Enabled)
            {
                // See if the timeout has occured, and we need to send a new window update and pending data
                if(SYS_TICK_Get() >= pSkt->eventTime2)
                {
                    vFlags = ACK;
                }
            }

            // Process Delayed ACKnowledgement timer
            if(pSkt->Flags.bDelayedACKTimerEnabled)
            {
                // See if the timeout has occured and delayed ACK needs to be sent
                if(SYS_TICK_Get() >= pSkt->delayedACKTime)
                {
                    vFlags = ACK;
                }
            }

            // Process TCP_CLOSE_WAIT timer
            if(pSkt->smState == TCP_CLOSE_WAIT)
            {
                // Automatically close the socket on our end if the application 
                // fails to call TCPDisconnect() is a reasonable amount of time.
                if(SYS_TICK_Get() >= pSkt->closeWaitTime)
                {
                    vFlags = FIN | ACK;
                    pSkt->smState = TCP_LAST_ACK;
                }
            }

            // Process FIN_WAIT2 timer
            if(pSkt->smState == TCP_FIN_WAIT_2)
            {
                // Automatically reset the socket on our end if the other side 
                // fails to call close its connection within the TCP_FIN_WAIT_2_TIMEOUT
                if(SYS_TICK_Get() >= pSkt->closeWaitTime)
                {
                    _TcpSend(pSkt, RST | ACK, SENDTCP_RESET_TIMERS);
                    _TcpCloseSocket(pSkt);
                    continue;
                }
            }



            if(vFlags)
            {
                _TcpSend(pSkt, vFlags, bRetransmit ? 0 : SENDTCP_RESET_TIMERS);
            }

            // The TCP_LISTEN, and sometimes the TCP_ESTABLISHED 
            // state don't need any timeout events, so see if the timer is enabled
            if(!pSkt->Flags.bTimerEnabled)
            {
#if defined(TCP_KEEP_ALIVE_TIMEOUT)
                // Only the established state has any use for keep-alives
                if(pSkt->smState == TCP_ESTABLISHED)
                {
                    // If timeout has not occured, do not do anything.
                    if(SYS_TICK_Get() < pSkt->eventTime)
                    {
                        continue;
                    }

                    // If timeout has occured and the connection appears to be dead (no 
                    // responses from remote node at all), close the connection so the 
                    // application doesn't sit around indefinitely with a useless socket 
                    // that it thinks is still open
                    if(pSkt->Flags.vUnackedKeepalives == TCP_MAX_UNACKED_KEEP_ALIVES)
                    {
                        vFlags = pSkt->Flags.bServer;

                        // Force an immediate FIN and RST transmission
                        // Also back in the listening state immediately if a server socket.
                        _TcpDisconnect(pSkt, true);
                        pSkt->Flags.bServer = 1;    // force client socket non-closing
                        _TcpAbort(pSkt, false);

                        // Prevent client mode sockets from getting reused by other applications.  
                        // The application must call TCPDisconnect()/TCPAbort() with the handle to free this 
                        // socket (and the handle associated with it)
                        if(!vFlags)
                        {
                            pSkt->Flags.bServer = 0;    // restore the client socket
                            pSkt->smState = TCP_CLOSED_BUT_RESERVED;
                        }

                        continue;
                    }

                    // Otherwise, if a timeout occured, simply send a keep-alive packet
                    _TcpSend(pSkt, ACK, SENDTCP_KEEP_ALIVE);
                    pSkt->eventTime = SYS_TICK_Get() + (TCP_KEEP_ALIVE_TIMEOUT * SYS_TICK_TicksPerSecondGet())/1000;
                }
#endif
                continue;
            }

            // If timeout has not occured, do not do anything.
            if(SYS_TICK_Get() < pSkt->eventTime)
            {
                continue;
            }
            
            // A timeout has occured.  Respond to this timeout condition
            // depending on what state this socket is in.
            switch(pSkt->smState)
            {
                case TCP_GATEWAY_SEND_ARP:
                    // Obtain the MAC address associated with the server's IP address (either direct MAC address on same subnet, or the MAC address of the Gateway machine)
                    pSkt->eventTime2 = SYS_TICK_Get();
                    ARPResolve(pSkt->pSktNet, (IPV4_ADDR *)&(((IPV4_PACKET*)pSkt->pTxPkt)->ipv4Header.DestAddress));
                    pSkt->smState = TCP_GATEWAY_GET_ARP;
                    break;

                case TCP_GATEWAY_GET_ARP:
                    // Wait for the MAC address to finish being obtained
                    if(!ARPIsResolved(pSkt->pSktNet, (IPV4_ADDR *)&(((IPV4_PACKET*)pSkt->pTxPkt)->ipv4Header.DestAddress), &((IPV4_PACKET*)pSkt->pTxPkt)->remoteMACAddr))
                    {
                        // Time out if too much time is spent in this state
                        // Note that this will continuously send out ARP 
                        // requests for an infinite time if the Gateway 
                        // never responds
                        if(SYS_TICK_Get() - pSkt->eventTime2 > pSkt->retryInterval)
                        {
                            // Exponentially increase timeout until we reach 6 attempts then stay constant
                            if(pSkt->retryCount < 6u)
                            {
                                pSkt->retryCount++;
                                pSkt->retryInterval <<= 1;
                            }

                            // Retransmit ARP request
                            pSkt->smState = TCP_GATEWAY_SEND_ARP;
                        }
                        break;
                    }

                    // Send out SYN connection request to remote node
                    // This automatically disables the Timer from 
                    // continuously firing for this socket
                    vFlags = SYN;
                    bRetransmit = false;
                    pSkt->smState = TCP_SYN_SENT;
                    break;

                case TCP_SYN_SENT:
                    // Keep sending SYN until we hear from remote node.
                    // This may be for infinite time, in that case
                    // caller must detect it and do something.
                    vFlags = SYN;
                    bRetransmit = true;

                    // NOTE : This if statement was only in 5.36
                    // Exponentially increase timeout until we reach TCP_MAX_RETRIES attempts then stay constant
                    if(pSkt->retryCount >= (TCP_MAX_RETRIES - 1))
                    {
                        pSkt->retryCount = TCP_MAX_RETRIES - 1;
                        pSkt->retryInterval = ((TCP_START_TIMEOUT_VAL * SYS_TICK_TicksPerSecondGet())/1000) << (TCP_MAX_RETRIES-1);
                    }
                    break;

                case TCP_SYN_RECEIVED:
                    // We must receive ACK before timeout expires.
                    // If not, resend SYN+ACK.
                    // Abort, if maximum attempts counts are reached.
                    if(pSkt->retryCount < TCP_MAX_SYN_RETRIES)
                    {
                        vFlags = SYN | ACK;
                        bRetransmit = true;
                    }
                    else
                    {
                        if(pSkt->Flags.bServer)
                        {
                            vFlags = RST | ACK;
                            bCloseSocket = true;
                        }
                        else
                        {
                            vFlags = SYN;
                        }
                    }
                    break;

                case TCP_ESTABLISHED:
                case TCP_CLOSE_WAIT:
                    // Retransmit any unacknowledged data
                    if(pSkt->retryCount < TCP_MAX_RETRIES)
                    {
                        vFlags = ACK;
                        bRetransmit = true;
                    }
                    else
                    {
                        // No response back for too long, close connection
                        // This could happen, for instance, if the communication 
                        // medium was lost
                        pSkt->smState = TCP_FIN_WAIT_1;
                        vFlags = FIN | ACK;
                    }
                    break;

                case TCP_FIN_WAIT_1:
                    if(pSkt->retryCount < TCP_MAX_RETRIES)
                    {
                        // Send another FIN
                        vFlags = FIN | ACK;
                        bRetransmit = true;
                    }
                    else
                    {
                        // Close on our own, we can't seem to communicate 
                        // with the remote node anymore
                        vFlags = RST | ACK;
                        bCloseSocket = true;
                    }
                    break;

                case TCP_CLOSING:
                    if(pSkt->retryCount < TCP_MAX_RETRIES)
                    {
                        // Send another ACK+FIN (the FIN is retransmitted 
                        // automatically since it hasn't been acknowledged by 
                        // the remote node yet)
                        vFlags = ACK;
                        bRetransmit = true;
                    }
                    else
                    {
                        // Close on our own, we can't seem to communicate 
                        // with the remote node anymore
                        vFlags = RST | ACK;
                        bCloseSocket = true;
                    }
                    break;

                    //			case TCP_TIME_WAIT:
                    //				// Wait around for a while (2MSL) and then close
                    //				bCloseSocket = true;
                    //				break;
                    //			

                case TCP_LAST_ACK:
                    // Send some more FINs or close anyway
                    if(pSkt->retryCount < TCP_MAX_RETRIES)
                    {
                        vFlags = FIN | ACK;
                        bRetransmit = true;
                    }
                    else
                    {
                        vFlags = RST | ACK;
                        bCloseSocket = true;
                    }
                    break;

                default:
                    break;
            }

            if(vFlags)
            {
                // Transmit all unacknowledged data over again
                if(bRetransmit)
                {
                    // Set the appropriate retry time
                    pSkt->retryCount++;
                    pSkt->retryInterval <<= 1;

                    // Calculate how many bytes we have to roll back and retransmit
                    w = pSkt->txUnackedTail - pSkt->txTail;
                    if(pSkt->txUnackedTail < pSkt->txTail)
                        w += pSkt->txEnd - pSkt->txStart;

                    // Perform roll back of local SEQuence counter, remote window 
                    // adjustment, and cause all unacknowledged data to be 
                    // retransmitted by moving the unacked tail pointer.
                    pSkt->MySEQ -= w;
                    pSkt->remoteWindow += w;
                    pSkt->txUnackedTail = pSkt->txTail;		
                    _TcpSend(pSkt, vFlags, 0);
                }
                else
                {
                    _TcpSend(pSkt, vFlags, SENDTCP_RESET_TIMERS);
                }

            }

            if(bCloseSocket)
            {
                _TcpCloseSocket(pSkt);
            }
        }
    }

    tcpTickPending = 0;
}


/*****************************************************************************
  Function:
	bool TCPProcessIPv4(TCPIP_NET_IF* pPktIf, NODE_INFO* remote, IPV4_ADDR* localIP, uint16_t len)

  Summary:
  	Handles incoming TCP segments.

  Description:
	This function handles incoming TCP segments.  When a segment arrives, it
	is compared to open sockets using a hash of the remote port and IP.  
	On a match, the data is passed to _TcpHandleSeg for further processing.

  Precondition:
	TCP is initialized and a TCP segment is ready in the MAC buffer.

  Parameters:
    pPktIf -  interface that has packets
    remote - Remote NODE_INFO structure
	localIP - This stack's IP address (for header checking)
	len - Total length of the waiting TCP segment

  Return Values:
	true - the segment was properly handled.
	false - otherwise
  ***************************************************************************/
bool TCPProcessIPv4(TCPIP_NET_IF* pPktIf, NODE_INFO* remote, IPV4_ADDR* localIP, uint16_t len)
{
	TCP_HEADER      TCPHeader;
	IPV4_PSEUDO_HEADER   pseudoHeader;
	TCPIP_UINT16_VAL        checksum1;
	TCPIP_UINT16_VAL        checksum2;
	uint8_t            optionsSize;
    TCPIP_MAC_HANDLE hMac;
    TCB_STUB* pSkt; 


	// Calculate IP pseudoheader checksum.
	pseudoHeader.SourceAddress      = remote->IPAddr;
	pseudoHeader.DestAddress        = *localIP;
	pseudoHeader.Zero               = 0x0;
	pseudoHeader.Protocol           = IP_PROT_TCP;
	pseudoHeader.Length          	= len;

	TCPIP_IPV4_SwapPseudoHeader(pseudoHeader);

	checksum1.Val = ~TCPIP_Helper_CalcIPChecksum((uint8_t*)&pseudoHeader, sizeof(pseudoHeader), 0);

    hMac = _TCPIPStackNetToMac(pPktIf);
	// Now calculate TCP packet checksum in NIC RAM - should match
	// pesudo header checksum
	checksum2.Val = MACCalcIPBufferChecksum(hMac, len);

	// Compare checksums.
	if(checksum1.Val != checksum2.Val)
	{
		TCPIP_IPV4_DiscardRx(hMac);
		return true;
	}

#if defined(DEBUG_GENERATE_RX_LOSS)
	// Throw RX packets away randomly
	if(LFSRRand() > DEBUG_GENERATE_RX_LOSS)
	{
		TCPIP_IPV4_DiscardRx(hMac);
		return true;
	}
#endif

	// Retrieve TCP header.
	TCPIP_IPV4_SetRxBuffer(pPktIf, 0);
	TCPIP_IPV4_GetArray(hMac, (uint8_t*)&TCPHeader, sizeof(TCPHeader));
	_TcpSwapHeader(&TCPHeader);


	// Skip over options to retrieve data bytes
	optionsSize = (uint8_t)((TCPHeader.DataOffset.Val << 2) - sizeof(TCPHeader));
	len = len - optionsSize - sizeof(TCPHeader);

	// Find matching socket.
	pSkt = _TcpFindMatchingSocket(pPktIf, &TCPHeader, (void *)&(remote->IPAddr), localIP, &remote->MACAddr, IP_ADDRESS_TYPE_IPV4);
    if(pSkt)
	{
		#if defined(TCPIP_STACK_USE_SSL)
		uint8_t* prevRxHead;
		// For SSL connections, show _TcpHandleSeg() the full data buffer
		prevRxHead = pSkt->rxHead;
		if(pSkt->sslStubID != SSL_INVALID_ID)
        {
			pSkt->rxHead = pSkt->sslRxHead;
        }
		#endif
		
		_TcpHandleSeg(pSkt, &TCPHeader, len);
		
		#if defined(TCPIP_STACK_USE_SSL)
		if(pSkt->sslStubID != SSL_INVALID_ID)
		{
			// Restore the buffer state
			pSkt->sslRxHead = pSkt->rxHead;
			pSkt->rxHead = prevRxHead;

			// Process the new SSL data, using the currently loaded stub
			TCPSSLHandleIncoming(pSkt->sktIx);
		}
		#endif
	}
//	else
//	{
//		// NOTE: RFC 793 specifies that if the socket is closed and a segment 
//		// arrives, we should send back a RST if the RST bit in the incoming 
//		// packet is not set.  Instead, we will just silently ignore such a 
//		// packet since this is what firewalls do on purpose to enhance 
//		// security.
//		//if(!TCPHeader.Flags.bits.flagRST)
//		//	_TcpSend(pSkt, RST, SENDTCP_RESET_TIMERS);
//	}

	// Finished with this packet, discard it and free the Ethernet RAM for new packets
	TCPIP_IPV4_DiscardRx(hMac);

	return true;
}

#if defined (TCPIP_STACK_USE_IPV6)
bool TCPProcessIPv6(TCPIP_NET_IF* pPktIf, IPV6_ADDR * localIP, IPV6_ADDR * remoteIP, uint16_t dataLen, uint16_t headerLen)
{
	TCP_HEADER      TCPHeader;
	IPV6_PSEUDO_HEADER   pseudoHeader;
	TCPIP_UINT16_VAL        checksum1;
	TCPIP_UINT16_VAL        checksum2;
	uint8_t            optionsSize;
    TCPIP_MAC_HANDLE hMac;
    TCB_STUB* pSkt; 

	// Calculate IP pseudoheader checksum.
    memcpy (&pseudoHeader.SourceAddress, remoteIP, sizeof (IPV6_ADDR));
    memcpy (&pseudoHeader.DestAddress, localIP, sizeof (IPV6_ADDR));
    // Total payload length is the length of data + extension headers
    pseudoHeader.PacketLength = TCPIP_HELPER_htons(dataLen);
    pseudoHeader.zero1 = 0;
    pseudoHeader.zero2 = 0;
    pseudoHeader.NextHeader = IP_PROT_TCP;

	checksum1.Val = ~TCPIP_Helper_CalcIPChecksum((uint8_t*)&pseudoHeader, sizeof(pseudoHeader), 0);

    hMac = _TCPIPStackNetToMac(pPktIf);
    // Reset the RX buffer so the checksum will encompass the necessary headers
    TCPIP_IPV6_SetRxBuffer (pPktIf, 0);
	// Now calculate TCP packet checksum in NIC RAM - should match
	// pesudo header checksum
	checksum2.Val = MACCalcIPBufferChecksum(hMac, dataLen);

	// Compare checksums.
	if(checksum1.Val != checksum2.Val)
	{
		TCPIP_IPV6_DiscardRx(hMac);
		return true;
	}

#if defined(DEBUG_GENERATE_RX_LOSS)
	// Throw RX packets away randomly
	if(LFSRRand() > DEBUG_GENERATE_RX_LOSS)
	{
		TCPIP_IPV6_DiscardRx(hMac);
		return true;
	}
#endif

	// Retrieve TCP header.
	TCPIP_IPV6_SetRxBuffer(pPktIf, headerLen);
	TCPIP_IPV6_GetArray(hMac, (uint8_t*)&TCPHeader, sizeof(TCPHeader));
	_TcpSwapHeader(&TCPHeader);

	// Skip over options to retrieve data bytes
	optionsSize = (uint8_t)((TCPHeader.DataOffset.Val << 2) - sizeof(TCPHeader));
	dataLen = dataLen - optionsSize - sizeof(TCPHeader);
	// Find matching socket.
	pSkt = _TcpFindMatchingSocket(pPktIf, &TCPHeader, remoteIP, localIP, NULL, IP_ADDRESS_TYPE_IPV6);
    if(pSkt)
	{
		#if defined(TCPIP_STACK_USE_SSL)
		uint8_t* prevRxHead;
		// For SSL connections, show _TcpHandleSeg() the full data buffer
		prevRxHead = pSkt->rxHead;
		if(pSkt->sslStubID != SSL_INVALID_ID)
        {
			pSkt->rxHead = pSkt->sslRxHead;
        }
		#endif
		
		_TcpHandleSeg(pSkt, &TCPHeader, dataLen);
		
		#if defined(TCPIP_STACK_USE_SSL)
		if(pSkt->sslStubID != SSL_INVALID_ID)
		{
			// Restore the buffer state
			pSkt->sslRxHead = pSkt->rxHead;
			pSkt->rxHead = prevRxHead;

			// Process the new SSL data, using the currently loaded stub
			TCPSSLHandleIncoming(pSkt->sktIx);
		}
		#endif
	}
	else
	{
        // Send ICMP Destination Unreachable Code 4 (Port unreachable) and discard packet
        TCPIP_IPV6_SendError (pPktIf, localIP, remoteIP, ICMPV6_ERR_DU_PORT_UNREACHABLE, ICMPV6_ERROR_DEST_UNREACHABLE, 0x00000000, dataLen + headerLen + sizeof (IPV6_HEADER));
	}

	// Finished with this packet, discard it and free the Ethernet RAM for new packets
	TCPIP_IPV6_DiscardRx(hMac);

	return true;
}
#endif  // defined (TCPIP_STACK_USE_IPV6)

/*****************************************************************************
  Function:
	static bool _TcpSend(pSkt, uint8_t vTCPFlags, uint8_t vSendFlags)

  Summary:
	Transmits a TPC segment.

  Description:
	This function assembles and transmits a TCP segment, including any 
	pending data.  It also supports retransmissions, keep-alives, and 
	other packet types.

  Precondition:
	TCP is initialized.

  Parameters:
	vTCPFlags - Additional TCP flags to include
	vSendFlags - Any combinations of SENDTCP_* constants to modify the
				 transmit behavior or contents.

  Returns:
	_TCP_SEND_OK for success, a _TCP_SEND_RES code otherwise
  ***************************************************************************/
static _TCP_SEND_RES _TcpSend(TCB_STUB* pSkt, uint8_t vTCPFlags, uint8_t vSendFlags)
{
	TCP_HEADER *    header;
	TCP_OPTIONS     options;
	uint16_t 		len, len2, mss;
    TCPIP_MAC_HANDLE hMac;
    void*   tempPkt;
	
	//  Make sure that we can write to the MAC transmit area
    if(!TCP_IsTxReady(pSkt))
    {
        return _TCP_SEND_MAC_BUSY;
    }

	// FINs must be handled specially
	if(vTCPFlags & FIN)
	{
		pSkt->Flags.bTXFIN = 1;
		vTCPFlags &= ~FIN;
	}

	// Status will now be synched, disable automatic future 
	// status transmissions
	pSkt->Flags.bTimer2Enabled = 0;
	pSkt->Flags.bDelayedACKTimerEnabled = 0;
	pSkt->Flags.bOneSegmentReceived = 0;
	pSkt->Flags.bTXASAP = 0;
	pSkt->Flags.bTXASAPWithoutTimerReset = 0;
	pSkt->Flags.bHalfFullFlush = 0;

    hMac = _TCPIPStackNetToMac(pSkt->pSktNet);

    header = TCP_GetUpperLayerHeaderPtr(pSkt);
	
	if (header != NULL) // Something was wrong...
	{

    header->DataOffset.Val = 0;

	// Put all socket application data in the TX space
	if(vTCPFlags & (SYN | RST))
	{
		// Don't put any data in SYN and RST messages
		len = 0;

    	// Insert the MSS (Maximum Segment Size) TCP option if this is SYN packet
    	if(vTCPFlags & SYN)
    	{
            if (TCP_IsTxPutReady (pSkt, sizeof (options)) < sizeof (options))
            {
                return _TCP_SEND_NO_SPACE;
            }
    		options.Kind = TCP_OPTIONS_MAX_SEG_SIZE;
    		options.Length = 0x04;
    
    		// Load MSS and swap to big endian
            if(_TCPIPStackIsLocalNetwork(pSkt->pSktNet, TCPIP_IPV4_GetDestAddress(pSkt->pTxPkt)))
            {
                mss = tcpMssLocal; 
            }
            else
            {
                mss = tcpMssNonLocal; 
            }
            options.MaxSegSize.Val = (((mss)&0x00FF)<<8) | (((mss)&0xFF00)>>8);
            pSkt->localMSS = mss;
    
    		header->DataOffset.Val   += sizeof(options) >> 2;
    
    		TCP_PutArray(pSkt, (uint8_t*)&options, sizeof(options));
    	}
	}
	else
	{
		// Begin copying any application data over to the TX space
		if(pSkt->txHead == pSkt->txUnackedTail)
		{
			// All caught up on data TX, no real data for this packet
			len = 0;
		}
		else if(pSkt->txHead > pSkt->txUnackedTail)
		{
			len = pSkt->txHead - pSkt->txUnackedTail;
			if(len > pSkt->remoteWindow)
            {
				len = pSkt->remoteWindow;
            }

			if(len > pSkt->wRemoteMSS)
			{
				len = pSkt->wRemoteMSS;
				pSkt->Flags.bTXASAPWithoutTimerReset = 1;
			}

            TCP_SetPayload (pSkt, pSkt->txUnackedTail, len);
			// Copy application data into the raw TX buffer
			pSkt->txUnackedTail += len;
		}
		else
		{
			len2 = pSkt->txEnd - pSkt->txUnackedTail;
			len = len2 + pSkt->txHead - pSkt->txStart;

			if(len > pSkt->remoteWindow)
				len = pSkt->remoteWindow;

			if(len > pSkt->wRemoteMSS)
			{
				len = pSkt->wRemoteMSS;
				pSkt->Flags.bTXASAPWithoutTimerReset = 1;
			}

            if (len2 > len)
                len2 = len;

            TCP_SetPayload (pSkt, pSkt->txUnackedTail, len2);

			// Copy application data into the raw TX buffer
			len2 = len - len2;
	
			// Copy any left over chunks of application data over
			if(len2)
			{
                TCP_SetPayload (pSkt, pSkt->txStart, len2);
			}

			pSkt->txUnackedTail += len;
			if(pSkt->txUnackedTail >= pSkt->txEnd)
            {
				pSkt->txUnackedTail -= pSkt->txEnd-pSkt->txStart;
            }
		}

		// If we are to transmit a FIN, make sure we can put one in this packet
		if(pSkt->Flags.bTXFIN)
		{
			if((len != pSkt->remoteWindow) && (len != pSkt->wRemoteMSS))
            {
				vTCPFlags |= FIN;
            }
		}
	}

	// Ensure that all packets with data of some kind are 
	// retransmitted by TCPTick() until acknowledged
	// Pure ACK packets with no data are not ACKed back in TCP
	if(len || (vTCPFlags & (SYN | FIN)))
	{
		// Transmitting data, update remote window variable to reflect smaller 
		// window.
		pSkt->remoteWindow -= len;

		// Push (PSH) all data for enhanced responsiveness on 
		// the remote end, especially with GUIs
		if(len)
        {
			vTCPFlags |= PSH;
        }

		if(vSendFlags & SENDTCP_RESET_TIMERS)
		{
			pSkt->retryCount = 0;
			pSkt->retryInterval = (TCP_START_TIMEOUT_VAL * SYS_TICK_TicksPerSecondGet())/1000;
		}	

		pSkt->eventTime = SYS_TICK_Get() + pSkt->retryInterval;
		pSkt->Flags.bTimerEnabled = 1;
	}
	else if(vSendFlags & SENDTCP_KEEP_ALIVE)
	{
		// Increment Keep Alive TX counter to handle disconnection if not response is returned
		pSkt->Flags.vUnackedKeepalives++;
		
		// Generate a dummy byte
		pSkt->MySEQ -= 1;
		len = 1;
	}
	else if(pSkt->Flags.bTimerEnabled) 
	{
		// If we have data to transmit, but the remote RX window is zero, 
		// so we aren't transmitting any right now then make sure to not 
		// extend the retry counter or timer.  This will stall our TX 
		// with a periodic ACK sent to the remote node.
		if(!(vSendFlags & SENDTCP_RESET_TIMERS))
		{
			// Roll back retry counters since we can't send anything, 
			// but only if we incremented it in the first place
			if(pSkt->retryCount)
			{
				pSkt->retryCount--;
				pSkt->retryInterval >>= 1;
			}
		}
	
		pSkt->eventTime = SYS_TICK_Get() + pSkt->retryInterval;
	}

	header->SourcePort			= pSkt->localPort.Val;
	header->DestPort			= pSkt->remotePort.Val;
	header->SeqNumber			= pSkt->MySEQ;
	header->AckNumber			= pSkt->RemoteSEQ;
	header->Flags.bits.Reserved2	= 0;
	header->DataOffset.Reserved3	= 0;
	header->Flags.byte			= vTCPFlags;
	header->UrgentPointer        = 0;

	// Update our send sequence number and ensure retransmissions 
	// of SYNs and FINs use the right sequence number
  	pSkt->MySEQ += (uint32_t)len;
	if(vTCPFlags & SYN)
	{
        len += sizeof(options);

		// SEG.ACK needs to be zero for the first SYN packet for compatibility 
		// with certain paranoid TCP/IP stacks, even though the ACK flag isn't 
		// set (indicating that the AckNumber field is unused).
		if(!(vTCPFlags & ACK))
        {
			header->AckNumber = 0;
        }

		if(pSkt->flags.bSYNSent)
        {
			header->SeqNumber--;
        }
		else
		{
			pSkt->MySEQ++;
			pSkt->flags.bSYNSent = 1;
		}
	}

	if(vTCPFlags & FIN)
	{
        pSkt->flags.bFINSent = 1;   // do not advance the seq no for FIN!
	}

	if(vTCPFlags & ACK)
	{
        pSkt->flags.ackSent = 1;   // store the ACK already sent
	}

	// Calculate the amount of free space in the RX buffer area of this socket
	if(pSkt->rxHead >= pSkt->rxTail)
    {
		header->Window = (pSkt->rxEnd - pSkt->rxStart) - (pSkt->rxHead - pSkt->rxTail);
    }
	else
    {
		header->Window = pSkt->rxTail - pSkt->rxHead - 1;
    }
    pSkt->localWindow = header->Window; // store the last advertised window

	_TcpSwapHeader(header);


	len += sizeof(TCP_HEADER);
	header->DataOffset.Val   += sizeof(TCP_HEADER) >> 2;

	// Write IP header
	TCP_PutHeader(pSkt, IP_PROT_TCP);

	// Update the TCP checksum
	// Calculate IP pseudoheader checksum.
    //header->Checksum = ~TCP_GetPseudoHeaderChecksum (pSkt->pTxPkt);
    //header->Checksum = TCP_CalculatePayloadChecksum (pSkt->pTxPkt);

#if defined(DEBUG_GENERATE_TX_LOSS)
	// Damage TCP checksums on TX packets randomly
	if(LFSRRand() > DEBUG_GENERATE_TX_LOSS)
	{
		header->Checksum++;
	}
#endif
        
	// Physically start the packet transmission over the network
    TCP_Flush (pSkt, NULL);

    if (TCP_IsPacketQueued(pSkt))
    {
        switch(pSkt->addType)
        {
#if defined (TCPIP_STACK_USE_IPV6)
            case IP_ADDRESS_TYPE_IPV6:
                tempPkt = TCPv6AllocateTxPacketStruct (pSkt);
                break;
#endif  // defined (TCPIP_STACK_USE_IPV6)

#if defined (TCPIP_STACK_USE_IPV4)
            case IP_ADDRESS_TYPE_IPV4:
                tempPkt = TCPv4AllocateTxPacketStruct (pSkt);
                break;
#endif  // defined (TCPIP_STACK_USE_IPV4)
            default:
                tempPkt = 0;
                break;
        }

        if(tempPkt)
        {
            if (!TCP_CopyTxPacketStruct (tempPkt, pSkt))
            {
                TCP_FreePacket (tempPkt, pSkt->addType);
            }
            else
            {
                pSkt->pTxPkt = tempPkt;
            }
        }
    }
    else
    {
        TCP_ResetTransmitPacketState (pSkt);
    }

    return _TCP_SEND_OK;

	}

    return _TCP_SEND_NO_PKT;
}

/*****************************************************************************
  Function:
	static TCB_STUB* _TcpFindMatchingSocket(TCPIP_NET_IF* pPktIf, TCP_HEADER* h, NODE_INFO* remote)

  Summary:
	Finds a suitable socket for a TCP segment.

  Description:
	This function searches through the sockets and attempts to match one with
	a given TCP header and NODE_INFO structure.
    If a socket is found, a valid socket pointer it is returned. 
	Otherwise, a 0 pointer is returned.
	
  Precondition:
	TCP is initialized.

  Parameters:
    pPktIf - interface the TCP packet/header belongs to  
	h - TCP header to be matched against
	remote - The remote node who sent this header

  Return Values:
	a socket pointer - A match was found 
	0 - No suitable socket was found 
  ***************************************************************************/
static TCB_STUB* _TcpFindMatchingSocket(TCPIP_NET_IF* pPktIf, TCP_HEADER* h, void * remoteIP, void * localIP, MAC_ADDR * remoteMACAddr, IP_ADDRESS_TYPE addressType)
{
	TCP_SOCKET hTCP;
	uint16_t hash;
#if defined(TCPIP_STACK_USE_SSL_SERVER)
    int  isSslSkt = 0, isForceBound;
#endif  // defined(TCPIP_STACK_USE_SSL_SERVER)
    TCB_STUB* pSkt, *partialSkt;


	// Prevent connections on invalid port 0
	if(h->DestPort == 0)
    {
		return 0;
    }

	partialSkt = 0;

    switch(addressType)
    {
#if defined TCPIP_STACK_USE_IPV6
        case IP_ADDRESS_TYPE_IPV6:
            hash = TCPIP_IPV6_GetHash(remoteIP,h->SourcePort,h->DestPort);
            break;  // OK
#endif  // defined (TCPIP_STACK_USE_IPV6)

#if defined (TCPIP_STACK_USE_IPV4)
        case IP_ADDRESS_TYPE_IPV4:
            hash = (((IPV4_ADDR *)remoteIP)->w[1] + ((IPV4_ADDR *)remoteIP)->w[0] + h->SourcePort) ^ h->DestPort;
            break;  // OK
#endif  // defined (TCPIP_STACK_USE_IPV4)

        default:
            return 0;  // shouldn't happen
    }

	// Loop through all sockets looking for a socket that is expecting this 
	// packet or can handle it.
	for(hTCP = 0; hTCP < TcpSockets; hTCP++ )
    {
        pSkt = TCBStubs[hTCP];

        if(pSkt == 0)
        {
            continue;
        }

        if( (pSkt->addType == IP_ADDRESS_TYPE_ANY || pSkt->addType == addressType) &&
                (pSkt->pSktNet == 0 || pSkt->pSktNet == pPktIf) )
        {   // both network interface and address type match

            bool found = false;

            if(pSkt->smState == TCP_LISTEN)
            {
                // For listening ports, check if this is the correct port
                if(pSkt->remoteHash.Val == h->DestPort)
                {
                    partialSkt = pSkt;
#if defined(TCPIP_STACK_USE_SSL_SERVER)
                    isSslSkt = false;
#endif // defined(TCPIP_STACK_USE_SSL_SERVER)
                }

#if defined(TCPIP_STACK_USE_SSL_SERVER)
                // Check the SSL port as well for SSL Servers
                // 0 is defined as an invalid port number
                if(pSkt->sslTxHead == (uint8_t*)(uint32_t)h->DestPort)
                {
                    partialSkt = pSkt;
                    isSslSkt = true;
                }
#endif
                continue;
            }
            else if(pSkt->remoteHash.Val != hash)
            {// Ignore if the hash doesn't match
                continue;
            }

            while(	h->DestPort == pSkt->localPort.Val && h->SourcePort == pSkt->remotePort.Val )  
            {

#if defined (TCPIP_STACK_USE_IPV6)
                if (addressType == IP_ADDRESS_TYPE_IPV6)
                {
                    if (!memcmp (TCPIP_IPV6_GetDestAddress(pSkt->pTxPkt), remoteIP, sizeof (IPV6_ADDR)))
                    {
                        found = true;
                    }
                    break;
                }
#endif  // defined (TCPIP_STACK_USE_IPV6)

#if defined (TCPIP_STACK_USE_IPV4)
                if (addressType == IP_ADDRESS_TYPE_IPV4)
                {
                    if (TCPIP_IPV4_GetDestAddress(pSkt->pTxPkt).Val == ((IPV4_ADDR *)remoteIP)->Val)
                    {
                        found = true;
                    }
                    break;
                }
#endif  // defined (TCPIP_STACK_USE_IPV4)

                break;
            }

            if(found)
            { 
                _TcpSocketBind(pSkt, pPktIf);
                return pSkt;    // bind to the correct interface
            }
        }
    }


	// If there is a partial match, then a listening socket is currently 
	// available.  Set up the extended TCB with the info needed 
	// to establish a connection and return this socket to the 
	// caller.
	while(partialSkt != 0)
    {
        pSkt = partialSkt;

        if(pSkt->pSktNet == 0)
        {   // unbound socket; bind it to this interface
            _TcpSocketBind(pSkt, pPktIf);
#if defined(TCPIP_STACK_USE_SSL_SERVER)
            isForceBound = 1;    // if the SSL connection fails we'll have to unbound!
#endif  // defined(TCPIP_STACK_USE_SSL_SERVER)
        }

#if defined(TCPIP_STACK_USE_SSL_SERVER)
        // Check the SSL port as well for SSL Servers
        // 0 is defined as an invalid port number
        if(isSslSkt)
        {
            // Try to start an SSL session.  If no stubs are available,
            // we can't service this request right now, so ignore it.
            if(!TCPStartSSLServer(pSkt->sktIx))
            {
                if(isForceBound)
                {
                    _TcpSocketBind(pSkt, 0);  // remove the binding
                }
                break;
            }
        }
#endif  // defined(TCPIP_STACK_USE_SSL_SERVER)


        switch(addressType)
        {
#if defined (TCPIP_STACK_USE_IPV6)
            case IP_ADDRESS_TYPE_IPV6:
                if(pSkt->pTxPkt == 0)
                {   // could be a server socket opened with IP_ADDRESS_TYPE_ANY
                    if((pSkt->pTxPkt = TCPv6AllocateTxPacketStruct (pSkt)) != 0)
                    {
                        TCPIP_IPV6_SetPacketIPProtocol (pSkt->pTxPkt);
                    }
                }

                if(pSkt->pTxPkt != 0)
                {   
                    TCPIP_IPV6_SetDestAddress (pSkt->pTxPkt, remoteIP);
                    TCPIP_IPV6_SetSourceAddress(pSkt->pTxPkt, &(((IPV6_ADDR_STRUCT *)localIP)->address));
                }
                break;
#endif  // defined (TCPIP_STACK_USE_IPV6)

#if defined (TCPIP_STACK_USE_IPV4)
            case IP_ADDRESS_TYPE_IPV4:
                if(pSkt->pTxPkt == 0)
                {   // could be a server socket opened with IP_ADDRESS_TYPE_ANY
                    if((pSkt->pTxPkt = TCPv4AllocateTxPacketStruct (pSkt)) != 0)
                    {
                        TCPIP_IPV4_SetPacketIPProtocol (pSkt->pTxPkt);
                    }
                }

                if(pSkt->pTxPkt != 0)
                {   
                    TCPIP_IPV4_SetDestAddress (pSkt->pTxPkt, ((IPV4_ADDR *)remoteIP)->Val);
                    memcpy((void*)&((IPV4_PACKET*)pSkt->pTxPkt)->remoteMACAddr, (void*)remoteMACAddr, sizeof(MAC_ADDR));
                }
                break;
#endif  // defined (TCPIP_STACK_USE_IPV4)

            default:    // shouldn't happen
                break;
        }

        if(pSkt->pTxPkt != 0)
        {
            pSkt->remoteHash.Val = hash;
            pSkt->remotePort.Val = h->SourcePort;
            pSkt->localPort.Val = h->DestPort;
            pSkt->txUnackedTail	= pSkt->txStart;

            // All done, and we have a match
            return pSkt;
        }
    }
		
	return 0;

}



/*****************************************************************************
  Function:
	static void _TcpSwapHeader(TCP_HEADER* header)

  Summary:
	Swaps endian-ness of a TCP header.

  Description:
	This function changes the endian-ness of a given TCP header for comparison.

  Precondition:
	None

  Parameters:
	header - The TCP header that is to be swapped

  Returns:
	None
  ***************************************************************************/
static void _TcpSwapHeader(TCP_HEADER* header)
{
	header->SourcePort      = TCPIP_HELPER_ntohs(header->SourcePort);
	header->DestPort        = TCPIP_HELPER_ntohs(header->DestPort);
	header->SeqNumber       = TCPIP_HELPER_ntohl(header->SeqNumber);
	header->AckNumber       = TCPIP_HELPER_ntohl(header->AckNumber);
	header->Window          = TCPIP_HELPER_ntohs(header->Window);
	header->Checksum        = TCPIP_HELPER_ntohs(header->Checksum);
	header->UrgentPointer   = TCPIP_HELPER_ntohs(header->UrgentPointer);
}


// initialize a socket
// the socket index and the sizes of its TX/RX buffers are passed as parameters
static void _TcpSocketInitialize(TCB_STUB* pSkt, TCP_SOCKET hTCP, uint8_t* txBuff, uint16_t txBuffSize, uint8_t* rxBuff, uint16_t rxBuffSize)
{
    // clear everything
    memset(pSkt, 0, sizeof(*pSkt));

    pSkt->sktIx = hTCP;     // hTCP is the index of this socket!

    // set all socket FIFO addresses
    pSkt->txStart	= txBuff;
    pSkt->txEnd	    = txBuff + txBuffSize + 1;
    pSkt->rxStart	= rxBuff;
    pSkt->rxEnd		= rxBuff + rxBuffSize;

#if defined(TCPIP_STACK_USE_SSL)
    pSkt->sslStubID = SSL_INVALID_ID;
#endif		

    // leave it unbound 
    // the binding will be handled dynamically
    // pSkt->pSktNet = 0;
    // Start out assuming worst case Maximum Segment Size (changes when MSS 
    // option is received from remote node)
    pSkt->wRemoteMSS = 536;

    TCBStubs[hTCP] = pSkt;  // store it
    
}

// set the default socket state
// socket should have been initialized
static void _TcpSocketSetIdleState(TCB_STUB* pSkt)
{

	pSkt->remoteHash.Val = pSkt->localPort.Val;
	pSkt->txHead = pSkt->txStart;
	pSkt->txTail = pSkt->txStart;
	pSkt->txUnackedTail = pSkt->txStart;
	pSkt->rxHead = pSkt->rxStart;
	pSkt->rxTail = pSkt->rxStart;
	pSkt->Flags.vUnackedKeepalives = 0;
	pSkt->Flags.bTimerEnabled = 0;
	pSkt->Flags.bTimer2Enabled = 0;
	pSkt->Flags.bDelayedACKTimerEnabled = 0;
	pSkt->Flags.bOneSegmentReceived = 0;
	pSkt->Flags.bHalfFullFlush = 0;
	pSkt->Flags.bTXASAP = 0;
	pSkt->Flags.bTXASAPWithoutTimerReset = 0;
	pSkt->Flags.bTXFIN = 0;
	pSkt->Flags.bSocketReset = 1;

#if defined(TCPIP_STACK_USE_SSL)
    // Reset the SSL buffer pointers
    pSkt->sslRxHead = pSkt->rxStart;
    pSkt->sslTxHead = pSkt->txStart;
#endif
	
#if defined(TCPIP_STACK_USE_SSL_SERVER)
    pSkt->sslTxHead = (uint8_t*)(uint32_t)pSkt->localSSLPort.Val;
#endif

	pSkt->flags.bFINSent = 0;
    pSkt->flags.seqInc = 0;
	pSkt->flags.bSYNSent = 0;
	pSkt->flags.bRXNoneACKed1 = 0;
	pSkt->flags.bRXNoneACKed2 = 0;
    pSkt->MySEQ = ((uint32_t)LFSRRand() << 16) | LFSRRand();
	pSkt->sHoleSize = -1;
	pSkt->remoteWindow = 1;

    pSkt->pSktNet = 0;

}

/*****************************************************************************
  Function:
	static void _TcpCloseSocket(void)

  Summary:
	Closes a TCP socket.

  Description:
	This function closes a TCP socket.  All socket state information is 
	reset, and any buffered bytes are discarded.
    If this is a server socket it will continue listening.
   
  Precondition:
	None.

  Parameters:
	None

  Returns:
	None
  ***************************************************************************/
static void _TcpCloseSocket(TCB_STUB* pSkt)
{
    // a socket server will listen after this Close operation
    bool    sktIsKilled;
    bool    freePkt;

    if(pSkt->Flags.bServer !=  0 && pSkt->flags.forceKill == 0)
    {   // server socket won't be killed
        sktIsKilled = false;
        // for unspecified address type the buffer gets de-allocated; next time it could be different
        freePkt = (pSkt->flags.openAddType == IP_ADDRESS_TYPE_ANY)?true:false;
    }
    else
    {
        sktIsKilled = true;
        freePkt = true;
    }


#if defined(TCPIP_STACK_USE_SSL)
	// If SSL is active, then we need to close it
	if(pSkt->sslStubID != SSL_INVALID_ID)
    {
        SSLTerminate(pSkt->sktIx, pSkt->sslStubID);
        pSkt->sslStubID = SSL_INVALID_ID;
        // Swap the SSL port and local port back to proper values
        pSkt->remoteHash.Val = pSkt->localSSLPort.Val;
        pSkt->localSSLPort.Val = pSkt->localPort.Val;
        pSkt->localPort.Val = pSkt->remoteHash.Val;
    }
#endif // defined(TCPIP_STACK_USE_SSL)

    while(pSkt->pTxPkt != NULL)
    {
#if defined (TCPIP_STACK_USE_IPV6)
        if(pSkt->addType == IP_ADDRESS_TYPE_IPV6)
        {
            if (!TCPIP_IPV6_IsPacketQueued(pSkt->pTxPkt))
            {
                if(freePkt)
                {
                    TCPIP_IPV6_FreePacket (pSkt->pTxPkt);
                    pSkt->pTxPkt = 0;
                }
                else
                {   // reused
                    TCPIP_IPV6_ResetTransmitPacketState (pSkt->pTxPkt);
                }
            }
            else
            {   // packet queued somewhere
                if(freePkt)
                {   // it will be freed by the acknowledge
                    pSkt->pTxPkt = 0;
                }
            }
            break;
        }
#endif  // defined (TCPIP_STACK_USE_IPV6)

#if defined (TCPIP_STACK_USE_IPV4)
        if(pSkt->addType == IP_ADDRESS_TYPE_IPV4)
        {
            if (!TCPIP_IPV4_IsPacketQueued(pSkt->pTxPkt))
            {
                if(freePkt)
                {
                    TCPIP_IPV4_FreePacket (pSkt->pTxPkt);
                    pSkt->pTxPkt = 0;
                }
                else
                {   // reused
                    TCPIP_IPV4_ResetTransmitPacketState (pSkt->pTxPkt);
                }
            }
            else
            {   // packet queued somewhere
                if(freePkt)
                {   // it will be freed by the acknowledge
                    pSkt->pTxPkt = 0;
                }
            }
        }
#endif  // defined (TCPIP_STACK_USE_IPV4)
        break;
    }

    if(sktIsKilled)
    {
        _TcpSocketKill(pSkt);
    }
    else
    {
        _TcpSocketSetIdleState(pSkt);
        pSkt->addType = (IP_ADDRESS_TYPE)pSkt->flags.openAddType;   // restore initial setting
        pSkt->smState = TCP_LISTEN;
    }
}


/*****************************************************************************
  Function:
	static uint16_t GetMaxSegSizeOption(TCPIP_NET_IF* pNet)

  Summary:
	Obtains the Maximum Segment Size (MSS) TCP Option out of the TCP header 
	for the current socket.

  Description:
	Parses the current TCP packet header and extracts the Maximum Segment Size 
	option.  

  Precondition:
	Must be called while a TCP packet is present and being processed via 
	_TcpHandleSeg() and only if the the TCP SYN flag is set.

  Parameters:
	hMac   - MAC interface for multi-homed hosts

  Returns:
	Maximum segment size option value.  If illegal or not present, a failsafe 
	value of 536 is returned.  If the option is larger than the 
	TCP_MAX_SEG_SIZE_TX upper limit, then TCP_MAX_SEG_SIZE_TX is returned.

  Remarks:
	The internal MAC Read Pointer is moved but not restored.
  ***************************************************************************/
static uint16_t GetMaxSegSizeOption(TCPIP_NET_IF* pNet)
{
    uint8_t vOptionsBytes;
    uint8_t vOption;
    uint8_t mData[2];
    uint16_t wMSS;
    TCPIP_MAC_HANDLE hMac;

    hMac = _TCPIPStackNetToMac(pNet);
    // Find out how many options bytes are in this packet.
    TCPIP_IPV4_SetRxBuffer(pNet, 2+2+4+4);	// Seek to data offset field, skipping Source port (2), Destination port (2), Sequence number (4), and Acknowledgement number (4)
    MACGetArray(hMac, &vOptionsBytes, 1);
    vOptionsBytes = ((vOptionsBytes&0xF0)>>2) - sizeof(TCP_HEADER);

    // Return minimum Maximum Segment Size value of 536 bytes if none are 
    // present
    if(vOptionsBytes == 0u)
        return 536;

    // Seek to beginning of options
    MACGetArray(hMac, NULL, 7);

    // Search for the Maximum Segment Size option	
    while(vOptionsBytes--)
    {
        MACGetArray(hMac, &vOption, 1);

        if(vOption == 0u)	// End of Options list
            break;

        if(vOption == 1u)	// NOP option
            continue;

        if(vOption == 2u)	// Maximum Segment Size option
        {
            if(vOptionsBytes < 3u)
                break;

            wMSS = 0;

            // Get option length
            MACGetArray(hMac, &vOption, 1);
            if(vOption == 4u)
            {// Retrieve MSS and swap value to little endian
                MACGetArray(hMac, mData, 2);
                ((uint8_t*)&wMSS)[1] = mData[0];
                ((uint8_t*)&wMSS)[0] = mData[1];
            }

            if(wMSS < 536u)
                break;
            if(wMSS > TCP_MAX_SEG_SIZE_TX)
                return TCP_MAX_SEG_SIZE_TX;
            else 
                return wMSS;
        }
        else
        { // Assume this is a multi byte option and throw it way
            if(vOptionsBytes < 2u)
                break;
            MACGetArray(hMac, &vOption, 1);
            if(vOptionsBytes < vOption)
                break;
            MACGetArray(hMac, NULL, vOption);
            vOptionsBytes -= vOption;
        }

    }

    // Did not find MSS option, return worst case default
    return 536;
}

/*****************************************************************************
  Function:
	static void _TcpHandleSeg(TCB_STUB* pSkt, TCP_HEADER* h, uint16_t len)

  Summary:
	Processes an incoming TCP segment.

  Description:
	Once an incoming segment has been matched to a socket, this function
	performs the necessary processing with the data.  Depending on the 
	segment and the state, this may include copying data to the TCP buffer,
	re-assembling out-of order packets, continuing an initialization or 
	closing handshake, or closing the socket altogether.

  Precondition:
	TCP is initialized and the current TCP stub is already synced.

  Parameters:
	h - The TCP header for this packet
	len - The total buffer length of this segment

  Returns:
	None
  ***************************************************************************/
static void _TcpHandleSeg(TCB_STUB* pSkt, TCP_HEADER* h, uint16_t len)
{
    uint32_t dwTemp;
    uint8_t* ptrTemp;
    uint32_t wTemp;
    int32_t lMissingBytes;
    uint16_t wMissingBytes;
    uint16_t wFreeSpace;
    uint8_t localHeaderFlags;
    uint32_t localAckNumber;
    uint32_t localSeqNumber;
    uint16_t wSegmentLength;
    bool bSegmentAcceptable;
    uint16_t wNewWindow;

    // Cache a few variables in local RAM.  
    // PIC18s take a fair amount of code and execution time to 
    // dereference pointers frequently.
    localHeaderFlags = h->Flags.byte;
    localAckNumber = h->AckNumber;
    localSeqNumber = h->SeqNumber;

    // We received a packet, reset the keep alive timer and count
#if defined(TCP_KEEP_ALIVE_TIMEOUT)
    pSkt->Flags.vUnackedKeepalives = 0;
    if(!pSkt->Flags.bTimerEnabled)
    {
        pSkt->eventTime = SYS_TICK_Get() + (TCP_KEEP_ALIVE_TIMEOUT * SYS_TICK_TicksPerSecondGet())/1000;
    }
#endif

    pSkt->flags.ackSent = 0;   // clear the ACK already sent

    // Handle TCP_LISTEN and TCP_SYN_SENT states
    // Both of these states will return, so code following this 
    // state machine need not check explicitly for these two 
    // states.
    switch(pSkt->smState)
    {
        case TCP_LISTEN:
            // First: check RST flag
            if(localHeaderFlags & RST)
            {
                _TcpCloseSocket(pSkt);	// Unbind remote IP address/port info
                return;
            }

            // Second: check ACK flag, which would be invalid
            if(localHeaderFlags & ACK)
            {
                // Use a believable sequence number and reset the remote node
                pSkt->MySEQ = localAckNumber;
                _TcpSend(pSkt, RST, 0);
                _TcpCloseSocket(pSkt);	// Unbind remote IP address/port info
                return;
            }

            // Third: check for SYN flag, which is what we're looking for
            if(localHeaderFlags & SYN)
            {
                // We now have a sequence number for the remote node
                pSkt->RemoteSEQ = localSeqNumber + 1;

                // Get MSS option
                pSkt->wRemoteMSS = GetMaxSegSizeOption(pSkt->pSktNet);

                // Set Initial Send Sequence (ISS) number
                // Nothing to do on this step... ISS already set in _TcpCloseSocket()

                // Respond with SYN + ACK
                _TcpSend(pSkt, SYN | ACK, SENDTCP_RESET_TIMERS);
                pSkt->smState = TCP_SYN_RECEIVED;
            }
            else
            {
                _TcpCloseSocket(pSkt);	// Unbind remote IP address/port info
            }

            // Fourth: check for other text and control
            // Nothing to do since we don't support this
            return;

        case TCP_SYN_SENT:
            // Second: check the RST bit
            // This is out of order because this stack has no API for 
            // notifying the application that the connection seems to 
            // be failing.  Instead, the application must time out and 
            // the stack will just keep trying in the mean time.
            if(localHeaderFlags & RST)
            {
                return;
            }

            // First: check ACK bit
            if(localHeaderFlags & ACK)
            {
                if(localAckNumber != pSkt->MySEQ)
                {
                    // Send a RST packet with SEQ = SEG.ACK, but retain our SEQ 
                    // number for arivial of any other SYN+ACK packets
                    localSeqNumber = pSkt->MySEQ;	// Save our original SEQ number
                    pSkt->MySEQ = localAckNumber;	// Set SEQ = SEG.ACK
                    _TcpSend(pSkt, RST, SENDTCP_RESET_TIMERS);		// Send the RST
                    pSkt->MySEQ = localSeqNumber;	// Restore original SEQ number
                    return;
                }
            }

#if defined (TCPIP_STACK_USE_IPV6)
            if (pSkt->addType == IP_ADDRESS_TYPE_IPV6)
            {
                TCPIP_NDP_NeighborConfirmReachability (pSkt->pSktNet, TCPIP_IPV6_GetDestAddress(pSkt->pTxPkt));
            }
#endif  // defined (TCPIP_STACK_USE_IPV6)

            // Third: check the security and precedence
            // No such feature in this stack.  We want to accept all connections.

            // Fourth: check the SYN bit
            if(localHeaderFlags & SYN)
            {
                // We now have an initial sequence number and window size
                pSkt->RemoteSEQ = localSeqNumber + 1;
                pSkt->remoteWindow = h->Window;

                // Get MSS option
                pSkt->wRemoteMSS = GetMaxSegSizeOption(pSkt->pSktNet);

                if(localHeaderFlags & ACK)
                {
                    _TcpSend(pSkt, ACK, SENDTCP_RESET_TIMERS);
                    pSkt->smState = TCP_ESTABLISHED;
                    // Set up keep-alive timer
#if defined(TCP_KEEP_ALIVE_TIMEOUT)
                    pSkt->eventTime = SYS_TICK_Get() + (TCP_KEEP_ALIVE_TIMEOUT * SYS_TICK_TicksPerSecondGet())/1000;
#endif
                    pSkt->Flags.bTimerEnabled = 0;
                }
                else
                {
                    _TcpSend(pSkt, SYN | ACK, SENDTCP_RESET_TIMERS);
                    pSkt->smState = TCP_SYN_RECEIVED;
                }
            }

            // Fifth: drop the segment if neither SYN or RST is set
            return;

        default:
            break;
    }

    //
    // First: check the sequence number
    //
    wSegmentLength = len;
    if(localHeaderFlags & FIN)
    {
        wSegmentLength++;
    }
    if(localHeaderFlags & SYN)
    {
        wSegmentLength++;
    }

    // Calculate the RX FIFO space
    if(pSkt->rxHead >= pSkt->rxTail)
    {
        wFreeSpace = (pSkt->rxEnd - pSkt->rxStart) - (pSkt->rxHead - pSkt->rxTail);
    }
    else
    {
        wFreeSpace = pSkt->rxTail - pSkt->rxHead - 1;
    }

    // Calculate the number of bytes ahead of our head pointer this segment skips
    lMissingBytes = localSeqNumber - pSkt->RemoteSEQ;
    wMissingBytes = (uint16_t)lMissingBytes;

    // Run TCP acceptability tests to verify that this packet has a valid sequence number
    bSegmentAcceptable = false;
    if(wSegmentLength)
    {
        // Check to see if we have free space, and if so, if any of the data falls within the freespace
        if(wFreeSpace)
        {
            // RCV.NXT =< SEG.SEQ < RCV.NXT+RCV.WND
            if((lMissingBytes >= (int32_t)0) && (wFreeSpace > (uint32_t)lMissingBytes))
            {
                bSegmentAcceptable = true;
            }
            else
            {
                // RCV.NXT =< SEG.SEQ+SEG.LEN-1 < RCV.NXT+RCV.WND
                if((lMissingBytes + (int32_t)wSegmentLength > (int32_t)0) && (lMissingBytes <= (int32_t)(int16_t)(wFreeSpace - wSegmentLength)))
                {
                    bSegmentAcceptable = true;
                }
            }

            if((lMissingBytes < (int32_t)wFreeSpace) && ((int16_t)wMissingBytes + (int16_t)wSegmentLength > (int16_t)0))
            {
                bSegmentAcceptable = true;
            }
        }
        // Segments with data are not acceptable if we have no free space
    }
    else
    {
        // Zero length packets are acceptable if they fall within our free space window
        // SEG.SEQ = RCV.NXT
        if(lMissingBytes == 0)
        {
            bSegmentAcceptable = true;
        }
        else
        {
            // RCV.NXT =< SEG.SEQ < RCV.NXT+RCV.WND
            if((lMissingBytes >= (int32_t)0) && (wFreeSpace > (uint32_t)lMissingBytes))
            {
                bSegmentAcceptable = true;
            }
        }
    }

    if(!bSegmentAcceptable)
    {
        // Unacceptable segment, drop it and respond appropriately
        if(!(localHeaderFlags & RST)) 
        {
            _TcpSend(pSkt, ACK, SENDTCP_RESET_TIMERS);
        }
        return;
    }


    //
    // Second: check the RST bit
    //
    //
    // Fourth: check the SYN bit
    //
    // Note, that since the third step is not implemented, we can 
    // combine this second and fourth step into a single operation.
    if(localHeaderFlags & (RST | SYN))
    {
        _TcpCloseSocket(pSkt);
        return;
    }

    //
    // Third: check the security and precedence
    //
    // Feature not supported.  Let's process this segment.

    //
    // Fifth: check the ACK bit
    //
    if(!(localHeaderFlags & ACK))
    {
        return;
    }

#if defined (TCPIP_STACK_USE_IPV6)
    // If we've received an ACK, update neighbor reachability
    if (pSkt->addType == IP_ADDRESS_TYPE_IPV6)
    {
        TCPIP_NDP_NeighborConfirmReachability (pSkt->pSktNet, TCPIP_IPV6_GetDestAddress(pSkt->pTxPkt));
    }
#endif  // defined (TCPIP_STACK_USE_IPV6)

    switch(pSkt->smState)
    {
        case TCP_SYN_RECEIVED:
            if(localAckNumber != pSkt->MySEQ)
            {
                // Send a RST packet with SEQ = SEG.ACK, but retain our SEQ 
                // number for arivial of any other correct packets
                localSeqNumber = pSkt->MySEQ;	// Save our original SEQ number
                pSkt->MySEQ = localAckNumber;	// Set SEQ = SEG.ACK
                _TcpSend(pSkt, RST, SENDTCP_RESET_TIMERS);		// Send the RST
                pSkt->MySEQ = localSeqNumber;	// Restore original SEQ number
                return;
            }
            pSkt->smState = TCP_ESTABLISHED;
            // No break

        case TCP_ESTABLISHED:
        case TCP_FIN_WAIT_1:
        case TCP_FIN_WAIT_2:
        case TCP_CLOSE_WAIT:
        case TCP_CLOSING:
            // Calculate what the highest possible SEQ number in our TX FIFO is
            wTemp = pSkt->txHead - pSkt->txUnackedTail;
            if((int32_t)wTemp < 0)
            {
                wTemp += pSkt->txEnd - pSkt->txStart;
            }
            dwTemp = pSkt->MySEQ + (uint32_t)wTemp;

            // Drop the packet if it ACKs something we haven't sent
            dwTemp = (int32_t)localAckNumber - (int32_t)dwTemp;
            if((int32_t)dwTemp > 0)
            {   // acknowledged more than we've sent??
                if(!pSkt->flags.bFINSent || pSkt->flags.seqInc || dwTemp != 1)
                {
                    _TcpSend(pSkt, ACK, 0);
                    return;
                }
                else
                {
                    pSkt->MySEQ++;      // since we didn't count the FIN
                    pSkt->flags.seqInc = 1;
                }
            }

            // Throw away all ACKnowledged TX data:
            // Calculate what the last acknowledged sequence number was (ignoring any FINs we sent)
            dwTemp = pSkt->MySEQ - (int32_t)(int16_t)(pSkt->txUnackedTail - pSkt->txTail);
            if(pSkt->txUnackedTail < pSkt->txTail)
            {
                dwTemp -= pSkt->txEnd - pSkt->txStart;
            }

            // Calcluate how many bytes were ACKed with this packet
            dwTemp = localAckNumber - dwTemp;
            if(((int32_t)(dwTemp) > (int32_t)0) && (dwTemp <= pSkt->txEnd - pSkt->txStart))
            {
                pSkt->flags.bRXNoneACKed1 = 0;
                pSkt->flags.bRXNoneACKed2 = 0;
                pSkt->Flags.bHalfFullFlush = false;

                // Bytes ACKed, free up the TX FIFO space
                ptrTemp = pSkt->txTail;
                pSkt->txTail += dwTemp;
                if(pSkt->txUnackedTail >= ptrTemp)
                {
                    if(pSkt->txUnackedTail < pSkt->txTail)
                    {
                        pSkt->MySEQ += pSkt->txTail - pSkt->txUnackedTail;
                        pSkt->txUnackedTail = pSkt->txTail;
                    }
                }
                else
                {
                    ptrTemp = pSkt->txUnackedTail + (pSkt->txEnd - pSkt->txStart);
                    if(ptrTemp < pSkt->txTail)
                    {
                        pSkt->MySEQ += pSkt->txTail - ptrTemp;
                        pSkt->txUnackedTail = pSkt->txTail;
                    }
                }
                if(pSkt->txTail >= pSkt->txEnd)
                {
                    pSkt->txTail -= pSkt->txEnd - pSkt->txStart;
                }
                if(pSkt->txUnackedTail >= pSkt->txEnd)
                {
                    pSkt->txUnackedTail -= pSkt->txEnd - pSkt->txStart;
                }
            }
            else
            {
                // See if we have outstanding TX data that is waiting for an ACK
                if(pSkt->txTail != pSkt->txUnackedTail)
                {
                    if(pSkt->flags.bRXNoneACKed1)
                    {
                        if(pSkt->flags.bRXNoneACKed2)
                        {
                            // Set up to perform a fast retransmission
                            // Roll back unacknowledged TX tail pointer to cause retransmit to occur
                            pSkt->MySEQ -= (int32_t)(int16_t)(pSkt->txUnackedTail - pSkt->txTail);
                            if(pSkt->txUnackedTail < pSkt->txTail)
                            {
                                pSkt->MySEQ -= (int32_t)(int16_t)(pSkt->txEnd - pSkt->txStart);
                            }
                            pSkt->txUnackedTail = pSkt->txTail;
                            pSkt->Flags.bTXASAPWithoutTimerReset = 1;
                        }
                        pSkt->flags.bRXNoneACKed2 = 1;
                    }
                    pSkt->flags.bRXNoneACKed1 = 1;
                }
            }

            // No need to keep our retransmit timer going if we have nothing that needs ACKing anymore
            if(pSkt->txTail == pSkt->txHead)
            {
                // Make sure there isn't a "FIN byte in our TX FIFO"
                if(pSkt->Flags.bTXFIN == 0u)
                {
                    // Convert retransmission timer to keep-alive timer
#if defined(TCP_KEEP_ALIVE_TIMEOUT)
                    pSkt->eventTime = SYS_TICK_Get() + (TCP_KEEP_ALIVE_TIMEOUT * SYS_TICK_TicksPerSecondGet())/1000;
#endif
                    pSkt->Flags.bTimerEnabled = 0;
                }
                else
                {
                    // "Throw away" FIN byte from our TX FIFO if it has been ACKed
                    if((pSkt->MySEQ == localAckNumber) && pSkt->flags.bFINSent)
                    {
                        pSkt->Flags.bTimerEnabled = 0;
                        pSkt->Flags.bTXFIN = 0;
                    }
                }
            }

            // The window size advertised in this packet is adjusted to account 
            // for any bytes that we have transmitted but haven't been ACKed yet 
            // by this segment.
            wNewWindow = h->Window - ((uint16_t)(pSkt->MySEQ - localAckNumber));

            // Update the local stored copy of the RemoteWindow.
            // If previously we had a zero window, and now we don't, then 
            // immediately send whatever was pending.
            if((pSkt->remoteWindow == 0u) && wNewWindow)
            {
                pSkt->Flags.bTXASAP = 1;
            }
            pSkt->remoteWindow = wNewWindow;

            // A couple of states must do all of the TCP_ESTABLISHED stuff, but also a little more
            if(pSkt->smState == TCP_FIN_WAIT_1)
            {
                // Check to see if our FIN has been ACKnowledged
                if((pSkt->MySEQ == localAckNumber) && pSkt->flags.bFINSent)
                {
                    // Reset our timer for forced closure if the remote node 
                    // doesn't send us a FIN in a timely manner.
                    pSkt->closeWaitTime = SYS_TICK_Get() + (TCP_FIN_WAIT_2_TIMEOUT * SYS_TICK_TicksPerSecondGet())/1000;
                    pSkt->Flags.bTimerEnabled = 1;
                    pSkt->smState = TCP_FIN_WAIT_2;
                }
            }
            else if(pSkt->smState == TCP_CLOSING)
            {
                // Check to see if our FIN has been ACKnowledged
                if(pSkt->MySEQ == localAckNumber)
                {
                    // RFC not recommended: We should be going to 
                    // the TCP_TIME_WAIT state right here and 
                    // starting a 2MSL timer, but since we have so 
                    // few precious sockets, we can't afford to 
                    // leave a socket waiting around doing nothing 
                    // for a long time.  If the remote node does 
                    // not recieve this ACK, it'll have to figure 
                    // out on it's own that the connection is now 
                    // closed.
                    _TcpCloseSocket(pSkt);
                }

                return;
            }

            break;

        case TCP_LAST_ACK:
            // Check to see if our FIN has been ACKnowledged
            if(pSkt->MySEQ + 1 == localAckNumber)
            {
                _TcpCloseSocket(pSkt);
            }
            return;

            //		case TCP_TIME_WAIT:
            //			// Nothing is supposed to arrive here.  If it does, reset the quiet timer.
            //			_TcpSend(pSkt, ACK, SENDTCP_RESET_TIMERS);
            //			return;

        default:
            break;
    }

    //
    // Sixth: Check the URG bit
    //
    // Urgent packets are not supported in this stack, so we
    // will throw them away instead
    if(localHeaderFlags & URG)
    {
        return;
    }

    //
    // Seventh: Process the segment text
    //
    // Throw data away if in a state that doesn't accept data
    if(pSkt->smState == TCP_CLOSE_WAIT || pSkt->smState == TCP_CLOSING || pSkt->smState == TCP_LAST_ACK)
    {
        return;
    }
    //	if(pSkt->smState == TCP_TIME_WAIT)
    //		return;

    // Copy any valid segment data into our RX FIFO, if any
    if(len)
    {
        // See if there are bytes we must skip
        if((int16_t)wMissingBytes <= 0)
        {
            // Position packet read pointer to start of useful data area.
            switch(pSkt->addType)
            {
#if defined (TCPIP_STACK_USE_IPV6)
                case IP_ADDRESS_TYPE_IPV6:
                    TCPIP_IPV6_SetRxBuffer(pSkt->pSktNet, (h->DataOffset.Val << 2) - wMissingBytes);
                    break;
#endif  // defined (TCPIP_STACK_USE_IPV6)

#if defined (TCPIP_STACK_USE_IPV4)
                case IP_ADDRESS_TYPE_IPV4:
                    TCPIP_IPV4_SetRxBuffer(pSkt->pSktNet, (h->DataOffset.Val << 2) - wMissingBytes);
                    break;
#endif  // defined (TCPIP_STACK_USE_IPV4)
                default:
                    return; // should not happen
            }

            len += wMissingBytes;		

            // Truncate packets that would overflow our TCP RX FIFO
            // and request a retransmit by sending a duplicate ACK
            if(len > wFreeSpace)
            {
                len = wFreeSpace;
            }

            pSkt->RemoteSEQ += (uint32_t)len;

            // Copy the application data from the packet into the socket RX FIFO
            // See if we need a two part copy (spans rxEnd->rxStart)
            if(pSkt->rxHead + len > pSkt->rxEnd)
            {
                wTemp = pSkt->rxEnd - pSkt->rxHead + 1;
                _TcpMac2RamCopy(pSkt->pSktNet, pSkt->rxHead, (TCPIP_MAC_PTR_TYPE)-1, wTemp);
                _TcpMac2RamCopy(pSkt->pSktNet, pSkt->rxStart, (TCPIP_MAC_PTR_TYPE)-1, len - wTemp);
                pSkt->rxHead = pSkt->rxStart + (len - wTemp);
            }
            else
            {
                _TcpMac2RamCopy(pSkt->pSktNet, pSkt->rxHead, (TCPIP_MAC_PTR_TYPE)-1, len);
                pSkt->rxHead += len;
            }

            // See if we have a hole and other data waiting already in the RX FIFO
            if(pSkt->sHoleSize != -1)
            {
                pSkt->sHoleSize -= len;
                wTemp = pSkt->wFutureDataSize + pSkt->sHoleSize;

                // See if we just closed up a hole, and if so, advance head pointer
                if((int32_t)wTemp < 0)
                {
                    pSkt->sHoleSize = -1;
                }
                else if(pSkt->sHoleSize <= 0)
                {
                    pSkt->RemoteSEQ += wTemp;
                    pSkt->rxHead += wTemp;
                    if(pSkt->rxHead > pSkt->rxEnd)
                    {
                        pSkt->rxHead -= pSkt->rxEnd - pSkt->rxStart + 1;							
                    }
                    pSkt->sHoleSize = -1;
                }
            }
        } // This packet is out of order or we lost a packet, see if we can generate a hole to accomodate it
        else if((int16_t)wMissingBytes > 0)
        {
            // Truncate packets that would overflow our TCP RX FIFO
            if(len + wMissingBytes > wFreeSpace)
            {
                len = wFreeSpace - wMissingBytes;
            }

            // Position packet read pointer to start of useful data area.
            switch(pSkt->addType)
            {
#if defined (TCPIP_STACK_USE_IPV6)
                case IP_ADDRESS_TYPE_IPV6:
                    TCPIP_IPV6_SetRxBuffer(pSkt->pSktNet, h->DataOffset.Val << 2);
                    break;
#endif  // defined (TCPIP_STACK_USE_IPV6)

#if defined (TCPIP_STACK_USE_IPV4)
                case IP_ADDRESS_TYPE_IPV4:
                    TCPIP_IPV4_SetRxBuffer(pSkt->pSktNet, h->DataOffset.Val << 2);
                    break;
#endif  // defined (TCPIP_STACK_USE_IPV4)
                default:
                    break;
            }


            // See if we need a two part copy (spans rxEnd->rxStart)
            if(pSkt->rxHead + wMissingBytes + len > pSkt->rxEnd)
            {
                // Calculate number of data bytes to copy before wraparound
                wTemp = pSkt->rxEnd - pSkt->rxHead + 1 - wMissingBytes;
                if((int32_t)wTemp >= 0)
                {
                    _TcpMac2RamCopy(pSkt->pSktNet, pSkt->rxHead + wMissingBytes, (TCPIP_MAC_PTR_TYPE)-1, wTemp);
                    _TcpMac2RamCopy(pSkt->pSktNet, pSkt->rxStart, (TCPIP_MAC_PTR_TYPE)-1, len - wTemp);
                }
                else
                {
                    _TcpMac2RamCopy(pSkt->pSktNet, pSkt->rxHead + wMissingBytes - (pSkt->rxEnd - pSkt->rxStart + 1), (TCPIP_MAC_PTR_TYPE)-1, len);
                }
            }
            else
            {
                _TcpMac2RamCopy(pSkt->pSktNet, pSkt->rxHead + wMissingBytes, (TCPIP_MAC_PTR_TYPE)-1, len);
            }

            // Record the hole is here
            if(pSkt->sHoleSize == -1)
            {
                pSkt->sHoleSize = wMissingBytes;
                pSkt->wFutureDataSize = len;
            }
            else
            {
                // We already have a hole, see if we can shrink the hole 
                // or extend the future data size
                if(wMissingBytes < (uint16_t)pSkt->sHoleSize)
                {
                    if((wMissingBytes + len > (uint16_t)pSkt->sHoleSize + pSkt->wFutureDataSize) || (wMissingBytes + len < (uint16_t)pSkt->sHoleSize))
                    {
                        pSkt->wFutureDataSize = len;
                    }
                    else
                    {
                        pSkt->wFutureDataSize = (uint16_t)pSkt->sHoleSize + pSkt->wFutureDataSize - wMissingBytes;
                    }
                    pSkt->sHoleSize = wMissingBytes;
                }
                else if(wMissingBytes + len > (uint16_t)pSkt->sHoleSize + pSkt->wFutureDataSize)
                {
                    // Make sure that there isn't a second hole between 
                    // our future data and this TCP segment's future data
                    if(wMissingBytes <= (uint16_t)pSkt->sHoleSize + pSkt->wFutureDataSize)
                    {
                        pSkt->wFutureDataSize += wMissingBytes + len - (uint16_t)pSkt->sHoleSize - pSkt->wFutureDataSize;
                    }
                }

            }
        }
    }

    // Send back an ACK of the data (+SYN | FIN) we just received, 
    // if any.  To minimize bandwidth waste, we are implementing 
    // the delayed acknowledgement algorithm here, only sending 
    // back an immediate ACK if this is the second segment received.  
    // Otherwise, a 200ms timer will cause the ACK to be transmitted.
    if(wSegmentLength)
    {
        // For non-established sockets, let's delete all data in 
        // the RX buffer immediately after receiving it.  This is 
        // not really how TCP was intended to operate since a 
        // socket cannot receive any response after it sends a FIN,
        // but our TCP application API doesn't readily accomodate
        // receiving data after calling TCPDisconnect(), which 
        // invalidates the application TCP handle.  By deleting all 
        // data, we'll ensure that the RX window is nonzero and 
        // the remote node will be able to send us a FIN response, 
        // which needs an RX window of at least 1.
        if(pSkt->smState != TCP_ESTABLISHED && pSkt->smState != TCP_FIN_WAIT_1 && pSkt->smState != TCP_FIN_WAIT_2)
        {
            pSkt->rxTail = pSkt->rxHead;
        }

        if(pSkt->Flags.bOneSegmentReceived)
        {
            _TcpSend(pSkt, ACK, SENDTCP_RESET_TIMERS);
            // bOneSegmentReceived is cleared in _TcpSend(pSkt, ), so no need here
        }
        else
        {
            pSkt->Flags.bOneSegmentReceived = true;	

            // Do not send an ACK immediately back.  Instead, we will 
            // perform delayed acknowledgements.  To do this, we will 
            // just start a timer
            if(!pSkt->Flags.bDelayedACKTimerEnabled)
            {
                pSkt->Flags.bDelayedACKTimerEnabled = 1;
                pSkt->delayedACKTime = SYS_TICK_Get() + (TCP_DELAYED_ACK_TIMEOUT * SYS_TICK_TicksPerSecondGet())/1000;
            }
        }
    }

    //
    // Eighth: check the FIN bit
    //
    if(localHeaderFlags & FIN)
    {
        // Note: Since we don't have a good means of storing "FIN bytes" 
        // in our TCP RX FIFO, we must ensure that FINs are processed 
        // in-order.
        if(pSkt->RemoteSEQ + 1 == localSeqNumber + (uint32_t)wSegmentLength)
        {
            // FINs are treated as one byte of data for ACK sequencing
            pSkt->RemoteSEQ++;

            switch(pSkt->smState)
            {
                case TCP_SYN_RECEIVED:
                    // RFC in exact: Our API has no need for the user 
                    // to explicitly close a socket that never really 
                    // got opened fully in the first place, so just 
                    // transmit a FIN automatically and jump to 
                    // TCP_LAST_ACK
                    pSkt->smState = TCP_LAST_ACK;
                    _TcpSend(pSkt, FIN | ACK, SENDTCP_RESET_TIMERS);
                    return;

                case TCP_ESTABLISHED:
                    // Go to TCP_CLOSE_WAIT state
                    pSkt->smState = TCP_CLOSE_WAIT;

                    // For legacy applications that don't call 
                    // TCPDisconnect() as needed and expect the TCP/IP 
                    // Stack to automatically close sockets when the 
                    // remote node sends a FIN, let's start a timer so 
                    // that we will eventually close the socket automatically
                    pSkt->closeWaitTime = SYS_TICK_Get() + (TCP_CLOSE_WAIT_TIMEOUT * SYS_TICK_TicksPerSecondGet())/1000;
                    if(pSkt->flags.ackSent)
                    {   // don't send another ack to the FIN if we already did that
                        return;
                    }

                    break;

                case TCP_FIN_WAIT_1:
                    if(pSkt->MySEQ == localAckNumber)
                    {
                        // RFC not recommended: We should be going to 
                        // the TCP_TIME_WAIT state right here and 
                        // starting a 2MSL timer, but since we have so 
                        // few precious sockets, we can't afford to 
                        // leave a socket waiting around doing nothing 
                        // for a long time.  If the remote node does 
                        // not recieve this ACK, it'll have to figure 
                        // out on it's own that the connection is now 
                        // closed.
                        _TcpSend(pSkt, ACK, 0);
                        _TcpCloseSocket(pSkt);
                        return;
                    }
                    else
                    {
                        pSkt->smState = TCP_CLOSING;
                    }
                    break;

                case TCP_FIN_WAIT_2:
                    // RFC not recommended: We should be going to 
                    // the TCP_TIME_WAIT state right here and 
                    // starting a 2MSL timer, but since we have so 
                    // few precious sockets, we can't afford to 
                    // leave a socket waiting around doing nothing 
                    // for a long time.  If the remote node does 
                    // not recieve this ACK, it'll have to figure 
                    // out on it's own that the connection is now 
                    // closed.
                    _TcpSend(pSkt, ACK, 0);
                    _TcpCloseSocket(pSkt);
                    return;

                default:
                    break;
            }

            // Acknowledge receipt of FIN
            _TcpSend(pSkt, ACK, SENDTCP_RESET_TIMERS);
        }
    }
}

/****************************************************************************
  Section:
	Buffer Management Functions
  ***************************************************************************/

/*****************************************************************************
  Function:
	bool TCPAdjustFIFOSize(TCP_SOCKET hTCP, uint16_t wMinRXSize, 
							uint16_t wMinTXSize, TCP_ADJUST_FLAGS vFlags)

  Summary:
	Adjusts the relative sizes of the RX and TX buffers.

  Description:
	This function can be used to simultaneously adjust the sizes
    of the RX and TX FIFOs.

    Adjusting the size of the TX/RX FIFO on the fly can allow for optimal transmission speed 
	for one-sided application protocols.
    For example, HTTP typically begins by receiving large amounts of data from the client,
    then switches to serving large amounts of data back.
    Adjusting the FIFO at these points can increase performance in systems that have 
    limited resources.
    Once the FIFOs are adjusted, a window update is sent.
	
    The TCP_ADJUST_FLAGS control the distribution of the remaining available space
    between the TX and RX FIFOs.
	If neither or both of TCP_ADJUST_GIVE_REST_TO_TX and 
	TCP_ADJUST_GIVE_REST_TO_RX are set, the function distributes the
	remaining space (if any) equally.
    If the new requested FIFOs space is greater that the old existing FIFOs space
    the TCP_ADJUST_GIVE_REST_TO_TX and TCP_ADJUST_GIVE_REST_TO_RX are ignored.
	
    TCP_ADJUST_PRESERVE_RX and TCP_ADJUST_PRESERVE_TX request the
    preserving of the existing data.
	Existing data can be preserved as long as the old data in the buffer
    does not exceed the capacity of the new buffer.

  Precondition:
	TCP is initialized.

  Parameters:
	hTCP		- The socket to be adjusted
	wMinRXSize	- Minimum number of bytes for the RX FIFO
	wMinTXSize 	- Minimum number of bytes for the TX FIFO
	vFlags		- If TCP_ADJUST_TX_ONLY or TCP_ADJUST_RX_ONLY  are not set,
                    then the TX and RX bufferrs are evaluated together and any
                    combination of the following flags is valid:
                    TCP_ADJUST_GIVE_REST_TO_RX, TCP_ADJUST_GIVE_REST_TO_TX
                    TCP_ADJUST_PRESERVE_RX, TCP_ADJUST_PRESERVE_TX
                - If TCP_ADJUST_TX_ONLY or TCP_ADJUST_RX_ONLY is set
                    then TX and RX buffers are treated individually
                    and TCP_ADJUST_GIVE_REST_TO_TX, TCP_ADJUST_GIVE_REST_TO_RX
                    values are irrelevant.
                - TCP_ADJUST_TX_ONLY and TCP_ADJUST_RX_ONLY both set is invalid.
                   

  Return Values:
	true - The FIFOs were adjusted successfully
	false - New RX and/or TX buffers could not be allocated
            and therefore the socket was left unchanged.

  Remarks:
	The function will automatically maintain minimal values for both TX and RX buffers.

    To avoid having a socket with no associated buffers, the function first
    allocates the new buffers and, if succeeded, it frees the old ones.

    The TX and RX FIFOs (buffers) associated with a socket
    are completely separate and independent.
    However, when TCP_ADJUST_TX_ONLY or TCP_ADJUST_RX_ONLY are not set,
    for the purpose of this function, the TX and RX FIFOs are considered
    to be contiguous so that the total FIFO space is divided between
    the TX and RX FIFOs.
    This provides bacward compatibility with previous versions of this function.

    The new flags TCP_ADJUST_TX_ONLY and TCP_ADJUST_RX_ONLY allow changing
    the size of TX and RX buffers independently.
    This is the preferred option.

    TX or RX associated buffer sizes can be changed too using the socket options.
    See TCPSetOptions. 
  ***************************************************************************/
bool TCPAdjustFIFOSize(TCP_SOCKET hTCP, uint16_t wMinRXSize, uint16_t wMinTXSize, TCP_ADJUST_FLAGS vFlags)
{
    uint16_t    oldTxSize, pendTxEnd, pendTxBeg, txUnackOffs;
    uint16_t    oldRxSize, avlblRxEnd, avlblRxBeg;
    uint16_t    diffChange;
    uint8_t     *newTxBuff, *newRxBuff;
    bool        adjustFail;
    
    if((vFlags & (TCP_ADJUST_TX_ONLY | TCP_ADJUST_RX_ONLY)) == (TCP_ADJUST_TX_ONLY | TCP_ADJUST_RX_ONLY))
    {   // invalid option
        return false;
    }

    TCB_STUB* pSkt = _TcpSocketChk(hTCP); 

    if(pSkt == 0)
    {
        return false;
    }

    // minimum size check
    if(wMinRXSize < TCP_MIN_RX_BUFF_SIZE)
    {
        wMinRXSize = TCP_MIN_RX_BUFF_SIZE;
    }
    if(wMinTXSize < TCP_MIN_TX_BUFF_SIZE)
    {
        wMinTXSize = TCP_MIN_TX_BUFF_SIZE;
    }
    

    oldTxSize = pSkt->txEnd - pSkt->txStart -1;
    oldRxSize = pSkt->rxEnd - pSkt->rxStart;


    if((vFlags & TCP_ADJUST_TX_ONLY) != 0)
    {   // preserve RX buffer
        wMinRXSize = oldRxSize;
    }
    else if((vFlags & TCP_ADJUST_RX_ONLY) != 0)
    {   // preserve TX buffer
        wMinTXSize = oldTxSize;
    }
    else if(oldTxSize + oldRxSize > wMinRXSize + wMinTXSize)
    {   // change both buffers relative to the old cumulated size
        // OK, we have some available space left
        uint16_t leftSpace = (oldTxSize + oldRxSize) - (wMinRXSize + wMinTXSize);

        // Set both allocation flags if none set
        TCP_ADJUST_FLAGS equalMask = (TCP_ADJUST_GIVE_REST_TO_TX | TCP_ADJUST_GIVE_REST_TO_RX);

        if((vFlags & equalMask) == 0 || (vFlags & equalMask) == equalMask)
        {   // distribute what's left equally
            wMinRXSize += (leftSpace + 1) >> 1;
            wMinTXSize += (leftSpace +1) >> 1;
        }
        else if((vFlags & equalMask) == TCP_ADJUST_GIVE_REST_TO_TX)
        {
            wMinTXSize += leftSpace;
        }
        else
        {
            wMinRXSize += leftSpace;
        }
    }

    if(wMinTXSize >= oldTxSize)
    {
        diffChange = wMinTXSize - oldTxSize;
    }
    else
    {
        diffChange = oldTxSize - wMinTXSize;
    }

    if(diffChange >= TCP_MIN_BUFF_CHANGE)
    {
        newTxBuff = (uint8_t*)TCPIP_HEAP_Malloc(tcpHeapH, wMinTXSize + 1);
        if(newTxBuff == 0)
        {    // fail, out of memory
            return false;
        }
    }
    else
    {   // same TX size, no change needed
        newTxBuff = 0;
    }

    if(wMinRXSize >= oldRxSize)
    {
        diffChange = wMinRXSize - oldRxSize;
    }
    else
    {
        diffChange = oldRxSize - wMinRXSize;
    }

    if(diffChange >= TCP_MIN_BUFF_CHANGE)
    {

        newRxBuff = (uint8_t*)TCPIP_HEAP_Malloc(tcpHeapH, wMinRXSize + 1);
        if(newRxBuff == 0)
        {    // fail, out of memory
            TCPIP_HEAP_Free(tcpHeapH, newTxBuff);
            return false;
        }
    }
    else
    {   // same RX size, no change needed
        newRxBuff = 0;
    }


    adjustFail = false;

    // process TX data; assume no copy, discard 
    pendTxEnd = pendTxBeg = 0;
    txUnackOffs = 0;
    while(newTxBuff != 0)
    {
        if((vFlags & TCP_ADJUST_PRESERVE_TX) != 0)
        {
            // calculate the pending data in the TX buffer 
            uint8_t*    txHead;
            uint8_t*    srcOffs = 0;

#if defined(TCPIP_STACK_USE_SSL)
            if(pSkt->sslStubID != SSL_INVALID_ID)
            {
                txHead = pSkt->sslTxHead;
            }
            else
#endif
            {
                txHead = pSkt->txHead;
            }
            
            if(txHead < pSkt->txTail)
            {
                pendTxEnd = pSkt->txEnd - pSkt->txTail;
                pendTxBeg = txHead - pSkt->txStart;
                srcOffs = pSkt->txStart;
            }
            else if(txHead > pSkt->txTail)
            {
                pendTxEnd = 0;
                pendTxBeg = txHead - pSkt->txTail;
                srcOffs = pSkt->txTail;
            }

            if((pendTxEnd + pendTxBeg) != 0)
            {   // need data copying
                if(pendTxEnd + pendTxBeg > wMinTXSize)
                {   // fail, not enough room
                    adjustFail = true;
                    break;
                }

                if(pendTxEnd)
                {
                    memcpy(newTxBuff, pSkt->txTail, pendTxEnd);
                }
                if(pendTxBeg)
                {
                    memcpy(newTxBuff + pendTxEnd,  srcOffs, pendTxBeg);
                }


                txUnackOffs = pSkt->txUnackedTail - pSkt->txTail;
                if(pSkt->txUnackedTail < pSkt->txTail)
                {
                    txUnackOffs += pSkt->txEnd - pSkt->txStart;
                }
            }
        }
        break;
    }

    // process the RX data
    // assume no copy, discard 
    avlblRxEnd = avlblRxBeg = 0;
    while(adjustFail != true && newRxBuff != 0)
    {
        if((vFlags & TCP_ADJUST_PRESERVE_RX) != 0)
        {   // calculate alavilable data in the RX buffer
            uint8_t*    rxHead;
            uint8_t*    srcOffs = 0;

#if defined(TCPIP_STACK_USE_SSL)
            if(pSkt->sslStubID != SSL_INVALID_ID)
            {
                rxHead = pSkt->sslRxHead;
            }
            else
#endif
            {
                rxHead = pSkt->rxHead;
            }

            // preserve out-of-order pending data
            if(pSkt->sHoleSize != -1)
            {
                rxHead += pSkt->sHoleSize + pSkt->wFutureDataSize;
                if(rxHead > pSkt->rxEnd)
                {
                    rxHead -= pSkt->rxEnd - pSkt->rxStart + 1;
                }
            }

            if(rxHead > pSkt->rxTail)
            {
                avlblRxEnd = 0;
                avlblRxBeg = rxHead - pSkt->rxTail;
                srcOffs = pSkt->rxTail;
            }
            else if(rxHead < pSkt->rxTail)
            {
                avlblRxEnd = pSkt->rxEnd + 1 - pSkt->rxTail;
                avlblRxBeg = rxHead - pSkt->rxStart;
                srcOffs = pSkt->rxStart;
            }

            if((avlblRxEnd + avlblRxBeg) != 0)
            {   // need data copying

                if(avlblRxEnd + avlblRxBeg > wMinRXSize)
                {   // fail, not enough room
                    adjustFail = true;
                    break;
                }

                if(avlblRxEnd)
                {
                    memcpy(newRxBuff, pSkt->rxTail, avlblRxEnd);
                }
                if(avlblRxBeg)
                {
                    memcpy(newRxBuff + avlblRxEnd, srcOffs, avlblRxBeg);
                }
            }
        }

        break;
    }

    if(adjustFail == true)
    {
        TCPIP_HEAP_Free(tcpHeapH, newRxBuff);
        TCPIP_HEAP_Free(tcpHeapH, newTxBuff);
        return false;
    }

    // success

    // adjust new TX pointers
    if(newTxBuff)
    {
        TCPIP_HEAP_Free(tcpHeapH, (void*)pSkt->txStart);
        pSkt->txStart =  newTxBuff;
        pSkt->txEnd = newTxBuff + wMinTXSize + 1;
        pSkt->txTail = pSkt->txStart;
        pSkt->txHead = pSkt->txStart + (pendTxEnd + pendTxBeg);
        pSkt->txUnackedTail = pSkt->txTail + txUnackOffs;
#if defined(TCPIP_STACK_USE_SSL)
        if(pSkt->sslStubID != SSL_INVALID_ID)
        {
            pSkt->sslTxHead = pSkt->txHead + 5;
        }
#endif
    }

    // adjust new RX pointers
    if(newRxBuff)
    {
        TCPIP_HEAP_Free(tcpHeapH, (void*)pSkt->rxStart);
        pSkt->rxStart = newRxBuff;
        pSkt->rxEnd = newRxBuff + wMinRXSize;
        pSkt->rxTail = pSkt->rxStart;
        pSkt->rxHead = pSkt->rxStart + (avlblRxEnd + avlblRxBeg);
#if defined(TCPIP_STACK_USE_SSL)
        pSkt->sslRxHead = pSkt->rxHead;
#endif
    }

    // Send a window update to notify remote node of change
    if(newTxBuff || newRxBuff)
    {
        if(pSkt->smState == TCP_ESTABLISHED)
        {
            _TcpSend(pSkt, ACK, SENDTCP_RESET_TIMERS);
        }
    }
    

    return true;
}

/*****************************************************************************
  Function:
	static void _TcpMac2RamCopy(TCPIP_NET_IF* pNet, TCPIP_MAC_PTR_TYPE ptrDest, TCPIP_MAC_PTR_TYPE ptrSource, uint16_t wLength)

  Summary:
	Copies data from MAC to RAM.

  Description:
	This function copies data from MAC to a TCP socket buffer 

  Precondition:
	TCP is initialized.

  Parameters:
    pNet        - interface to use 
    ptrDest		- RAM address to write to
	ptrSource	- MAC address to copy from
	wLength		- Number of bytes to copy

  Returns:
	None

  Remarks:
  ***************************************************************************/
static void _TcpMac2RamCopy(TCPIP_NET_IF* pNet, uint8_t* ptrDest, TCPIP_MAC_PTR_TYPE ptrSource, uint16_t wLength)
{
    TCPIP_MAC_HANDLE    hMac;

    hMac = _TCPIPStackNetToMac(pNet);

    if(hMac ==0 )
    {
        SYS_ERROR(SYS_ERROR_ERROR,  "_TcpMac2RamCopy: ETH MAC->RAM copy required with no valid MAC!");
        return;
    }

    if(ptrSource != (TCPIP_MAC_PTR_TYPE)-1)
    {
        MACSetReadPtr(hMac, ptrSource);
    }
    MACGetArray(hMac, ptrDest, wLength);

}


/****************************************************************************
  Section:
	SSL Functions
  ***************************************************************************/

/*****************************************************************************
  Function:
	bool TCPStartSSLClient(TCP_SOCKET hTCP, uint8_t* host)

  Summary:
	Begins an SSL client session.

  Description:
	This function escalates the current connection to an SSL secured 
	connection by initiating an SSL client handshake.

  Precondition:
	TCP is initialized and hTCP is already connected.

  Parameters:
	hTCP		- TCP connection to secure
	host		- Expected host name on certificate (currently ignored)

  Return Values:
	true 		- an SSL connection was initiated
	false 		- Insufficient SSL resources (stubs) were available

  Remarks:
	The host parameter is currently ignored and is not validated.
  ***************************************************************************/
#if defined(TCPIP_STACK_USE_SSL_CLIENT)
bool TCPStartSSLClient(TCP_SOCKET hTCP, uint8_t* host)
{
    int i;
    TCB_STUB* pSkt = _TcpSocketChk(hTCP); 

    if(pSkt == 0)
    {
        return false;
    }

    // Make sure SSL is not established already
    if(pSkt->sslStubID != SSL_INVALID_ID)
    {
        return false;
    }

    // Try to start the session
    if(pSkt->pSktNet == 0)
    {   // unbound socket; bind it to the default interface
        _TcpSocketBind(pSkt, (TCPIP_NET_IF*)TCPIP_STACK_GetDefaultNet());
    }
    pSkt->sslStubID = SSLStartSession(hTCP, NULL, 0);

    // Make sure a session stub was obtained
    if(pSkt->sslStubID == SSL_INVALID_ID)
    {
        return false;
    }

    // Mark connection as handshaking and return
    pSkt->sslReqMessage = SSL_CLIENT_HELLO;
    pSkt->sslRxHead = pSkt->rxHead;
    pSkt->sslTxHead = pSkt->txHead;
    pSkt->Flags.bSSLHandshaking = 1;
    for(i = 0; i < 5u; i++)
    {// Skip first 5 bytes in TX for the record header
        if(++pSkt->sslTxHead >= pSkt->txEnd)
        {
            pSkt->sslTxHead = pSkt->txStart;
        }
    }
    return true;
}
#endif // SSL Client

/*****************************************************************************
  Function:
	bool TCPStartSSLClientEx(TCP_SOCKET hTCP, uint8_t* host, uint8_t * buffer, uint8_t suppDataType)

  Summary:
	Begins an SSL client session.

  Description:
	This function escalates the current connection to an SSL secured 
	connection by initiating an SSL client handshake.

  Precondition:
	TCP is initialized and hTCP is already connected.

  Parameters:
	hTCP			- TCP connection to secure
	host			- Expected host name on certificate (currently ignored)
	buffer      	- Buffer for supplementary data return
	suppDataType 	- Type of supplementary data to copy

  Return Values:
	true 		- an SSL connection was initiated
	false 		- Insufficient SSL resources (stubs) were available

  Remarks:
	The host parameter is currently ignored and is not validated.
  ***************************************************************************/
#if defined(TCPIP_STACK_USE_SSL_CLIENT)
bool TCPStartSSLClientEx(TCP_SOCKET hTCP, uint8_t* host, void * buffer, uint8_t suppDataType)
{
    int i;
    TCB_STUB* pSkt = _TcpSocketChk(hTCP); 

    if(pSkt == 0)
    {
        return false;
    }

    // Make sure SSL is not established already
    if(pSkt->sslStubID != SSL_INVALID_ID)
    {
        return false;
    }

    // Try to start the session
    if(pSkt->pSktNet == 0)
    {   // unbound socket; bind it to the default interface
        _TcpSocketBind(pSkt, (TCPIP_NET_IF*)TCPIP_STACK_GetDefaultNet());
    }
    pSkt->sslStubID = SSLStartSession(hTCP, buffer, suppDataType);

    // Make sure a session stub was obtained
    if(pSkt->sslStubID == SSL_INVALID_ID)
    {
        return false;
    }

    // Mark connection as handshaking and return
    pSkt->sslReqMessage = SSL_CLIENT_HELLO;
    pSkt->sslRxHead = pSkt->rxHead;
    pSkt->sslTxHead = pSkt->txHead;
    pSkt->Flags.bSSLHandshaking = 1;
    for(i = 0; i < 5u; i++)
    {// Skip first 5 bytes in TX for the record header
        if(++pSkt->sslTxHead >= pSkt->txEnd)
        {
            pSkt->sslTxHead = pSkt->txStart;
        }
    }
    return true;
}
#endif // SSL Client

/*****************************************************************************
  Function:
	bool TCPStartSSLServer(TCP_SOCKET hTCP)

  Summary:
	Begins an SSL server session.

  Description:
	This function sets up an SSL server session when a new connection is
	established on an SSL port.

  Precondition:
	TCP is initialized and hTCP is already connected.

  Parameters:
	hTCP		- TCP connection to secure

  Return Values:
	true		- an SSL connection was initiated
	false		- Insufficient SSL resources (stubs) were available
  ***************************************************************************/
#if defined(TCPIP_STACK_USE_SSL_SERVER)
bool TCPStartSSLServer(TCP_SOCKET hTCP)
{
    int i;
    TCB_STUB* pSkt = _TcpSocketChk(hTCP); 

    if(pSkt == 0)
    {
        return false;
    }

    // Make sure SSL is not established already
    if(pSkt->sslStubID != SSL_INVALID_ID)
    {
        return true;
    }

    // Try to start the session
    pSkt->sslStubID = SSLStartSession(hTCP, NULL, 0);

    // Make sure a session stub was obtained
    if(pSkt->sslStubID == SSL_INVALID_ID)
    {
        return false;
    }

    // Swap the localPort and localSSLPort
    pSkt->remoteHash.Val = pSkt->localPort.Val;
    pSkt->localPort.Val = pSkt->localSSLPort.Val;
    pSkt->localSSLPort.Val = pSkt->remoteHash.Val;	

    // Mark connection as handshaking and return
    pSkt->sslReqMessage = SSL_NO_MESSAGE;
    pSkt->sslRxHead = pSkt->rxHead;
    pSkt->sslTxHead = pSkt->txHead;
    pSkt->Flags.bSSLHandshaking = 1;
    for(i = 0; i < 5u; i++)
    {// Skip first 5 bytes in TX for the record header
        if(++pSkt->sslTxHead >= pSkt->txEnd)
        {
            pSkt->sslTxHead = pSkt->txStart;
        }
    }
    return true;
}
#endif // SSL Client

/*****************************************************************************
  Function:
	bool TCPAddSSLListener(TCP_SOCKET hTCP, uint16_t port)

  Summary:
	Listens for SSL connection on a specific port.

  Description:
	This function adds an additional listening port to a TCP connection.  
	Connections made on this alternate port will be secured via SSL.

  Precondition:
	TCP is initialized and hTCP is listening.

  Parameters:
	hTCP		- TCP connection to secure
	port		- SSL port to listen on

  Return Values:
	true		- SSL port was added.
	false		- The socket was not a listening socket.
  ***************************************************************************/
#if defined(TCPIP_STACK_USE_SSL_SERVER)
bool TCPAddSSLListener(TCP_SOCKET hTCP, uint16_t port)
{
    TCB_STUB* pSkt = _TcpSocketChk(hTCP); 
	
    if(pSkt == 0 || pSkt->smState != TCP_LISTEN)
    {
        return false;
    }
	
	pSkt->localSSLPort.Val = port;
	pSkt->sslTxHead = (uint8_t*)(uint32_t)port;

	return true;
}
#endif // SSL Server

/*****************************************************************************
  Function:
	bool TCPRequestSSLMessage(TCP_SOCKET hTCP, uint8_t msg)

  Summary:
	Requests an SSL message to be transmitted.

  Description:
	This function is called to request that a specific SSL message be
	transmitted.  This message should only be called by the SSL module.
	
  Precondition:
	TCP is initialized.

  Parameters:
	hTCP		- TCP connection to use
	msg			- One of the SSL_MESSAGE types to transmit.

  Return Values:
	true		- The message was requested.
	false		- Another message is already pending transmission.
  ***************************************************************************/
#if defined(TCPIP_STACK_USE_SSL)
bool TCPRequestSSLMessage(TCP_SOCKET hTCP, uint8_t msg)
{
    TCB_STUB* pSkt = _TcpSocketChk(hTCP); 

    if(pSkt == 0)
    {
        return false;
    }

    if(msg == SSL_NO_MESSAGE || pSkt->sslReqMessage == SSL_NO_MESSAGE)
    {
        pSkt->sslReqMessage = msg;
        return true;
    }

    return false;
}
#endif // SSL

/*****************************************************************************
  Function:
	bool TCPSSLIsHandshaking(TCP_SOCKET hTCP)

  Summary:
	Determines if an SSL session is still handshaking.

  Description:
	Call this function after calling TCPStartSSLClient until false is
	returned.  Then your application may continue with its normal data
	transfer (which is now secured).
	
  Precondition:
	TCP is initialized and hTCP is connected.

  Parameters:
	hTCP		- TCP connection to check

  Return Values:
	true		- SSL handshake is still progressing
	false		- SSL handshake has completed
  ***************************************************************************/
#if defined(TCPIP_STACK_USE_SSL)
bool TCPSSLIsHandshaking(TCP_SOCKET hTCP)
{
    TCB_STUB* pSkt = _TcpSocketChk(hTCP); 
	
    if(pSkt == 0)
    {
        return false;
    }

	return pSkt->Flags.bSSLHandshaking;	
}
#endif // SSL

/*****************************************************************************
  Function:
	bool TCPIsSSL(TCP_SOCKET hTCP)

  Summary:
	Determines if a TCP connection is secured with SSL.

  Description:
	Call this function to determine whether or not a TCP connection is 
	secured with SSL.
	
  Precondition:
	TCP is initialized and hTCP is connected.

  Parameters:
	hTCP		- TCP connection to check

  Return Values:
	true		- Connection is secured via SSL
	false		- Connection is not secured
  ***************************************************************************/
#if defined(TCPIP_STACK_USE_SSL)
bool TCPIsSSL(TCP_SOCKET hTCP)
{
    TCB_STUB* pSkt = _TcpSocketChk(hTCP); 
	
    if(pSkt == 0 || pSkt->sslStubID == SSL_INVALID_ID)
    {
        return false;
    }

	return true;
}
#endif // SSL

/*****************************************************************************
  Function:
	void TCPSSLHandshakeComplete(TCP_SOCKET hTCP)

  Summary:
	Clears the SSL handshake flag.

  Description:
	This function clears the flag indicating that an SSL handshake is
	complete.
	
  Precondition:
	TCP is initialized and hTCP is connected.

  Parameters:
	hTCP		- TCP connection to set

  Returns:
	None

  Remarks:
	This function should never be called by an application.  It is used 
	only by the SSL module itself.
  ***************************************************************************/
#if defined(TCPIP_STACK_USE_SSL)
void TCPSSLHandshakeComplete(TCP_SOCKET hTCP)
{

    TCB_STUB* pSkt = _TcpSocketChk(hTCP); 
	
    if(pSkt == 0)
    {
        return;
    }
	pSkt->Flags.bSSLHandshaking = 0;
}
#endif // SSL

/*****************************************************************************
  Function:
	void TCPSSLDecryptMAC(TCP_SOCKET hTCP, ARCFOUR_CTX* ctx, uint16_t len)

  Summary:
	Decrypts and MACs data arriving via SSL.

  Description:
	This function decrypts data in the TCP buffer and calculates the MAC over
	the data.  All data is left in the exact same location in the TCP buffer.
	It is called to help process incoming SSL records.
	
  Precondition:
	TCP is initialized, hTCP is connected, and ctx's Sbox is loaded.

  Parameters:
	hTCP		- TCP connection to decrypt in
	ctx			- ARCFOUR encryption context to use
	len 		- Number of bytes to crypt
	inPlace		- true to write back in place, false to write at end of
					currently visible data.

  Returns:
	None

  Remarks:
	This function should never be called by an application.  It is used 
	only by the SSL module itself.
  ***************************************************************************/
#if defined(TCPIP_STACK_USE_SSL)
void TCPSSLDecryptMAC(TCP_SOCKET hTCP, ARCFOUR_CTX* ctx, uint16_t len)
{
    uint8_t *wSrc, *wDest;
    unsigned int wBlockLen, wTemp;
    uint8_t buffer[32];

    TCB_STUB* pSkt = _TcpSocketChk(hTCP); 

    if(pSkt == 0)
    {
        return;
    }

    wSrc = pSkt->rxTail;
    wDest = wSrc;

    // Handle 32 bytes at a time
    while(len)
    {
        // Determine how many bytes we can read
        wBlockLen = sizeof(buffer);
        if(wBlockLen > len)
        { // Don't do more than we should
            wBlockLen = len;
        }

        // Read those bytes to a buffer
        if(wSrc + wBlockLen > pSkt->rxEnd)
        {// Two part read
            wTemp = pSkt->rxEnd - wSrc + 1;
            memcpy(buffer, (uint8_t*)wSrc, wTemp);
            memcpy(buffer+wTemp, (uint8_t*)pSkt->rxStart, wBlockLen - wTemp);
            wSrc = pSkt->rxStart + wBlockLen - wTemp;
        }
        else
        {
            memcpy(buffer, (uint8_t*)wSrc, wBlockLen);
            wSrc += wBlockLen;
        }

        // Decrypt and hash
        ARCFOURCrypt(ctx, buffer, wBlockLen);
        SSLMACAdd(buffer, wBlockLen);

        // Write decrypted bytes back
        if(wDest + wBlockLen > pSkt->rxEnd)
        {// Two part write
            wTemp = pSkt->rxEnd - wDest + 1;
            memcpy((uint8_t*)wDest, buffer, wTemp);
            memcpy((uint8_t*)pSkt->rxStart, buffer+wTemp, wBlockLen - wTemp);
            wDest = pSkt->rxStart + wBlockLen - wTemp;
        }
        else
        {
            memcpy((uint8_t*)wDest, buffer, wBlockLen);
            wDest += wBlockLen;
        }

        // Update the length remaining
        len -= wBlockLen;
    }
}	
#endif // SSL

/*****************************************************************************
  Function:
	void TCPSSLInPlaceMACEncrypt(TCP_SOCKET hTCP, ARCFOUR_CTX* ctx, 
									uint8_t* MACSecret, uint16_t len)

  Summary:
	Encrypts and MACs data in place in the TCP TX buffer.

  Description:
	This function encrypts data in the TCP buffer while calcuating a MAC.  
	When encryption is finished, the MAC is appended to the buffer and 
	the record will be ready to transmit.
	
  Precondition:
	TCP is initialized, hTCP is connected, and ctx's Sbox is loaded.

  Parameters:
	hTCP		- TCP connection to encrypt in
	ctx			- ARCFOUR encryption context to use
	MACSecret	- MAC encryption secret to use
	len 		- Number of bytes to crypt

  Returns:
	None

  Remarks:
	This function should never be called by an application.  It is used 
	only by the SSL module itself.
  ***************************************************************************/
#if defined(TCPIP_STACK_USE_SSL)
void TCPSSLInPlaceMACEncrypt(TCP_SOCKET hTCP, ARCFOUR_CTX* ctx, uint8_t* MACSecret, uint16_t len)
{
	uint8_t* pos;
	unsigned int blockLen;
	uint8_t buffer[32];
	
    TCB_STUB* pSkt = _TcpSocketChk(hTCP); 
	
    if(pSkt == 0)
    {
        return;
    }

	pos = pSkt->txHead;
	for(blockLen = 0; blockLen < 5u; blockLen++)
	{// Skips first 5 bytes for the header
		if(++pos >= pSkt->txEnd)
        {
			pos = pSkt->txStart;
        }
	}
	
	// Handle 32 bytes at a time
	while(len)
	{
		// Determine how many bytes we can read
		blockLen = sizeof(buffer);
		if(blockLen > len)
        { // Don't do more than we should
			blockLen = len;
        }
		if(blockLen > pSkt->txEnd - pos)
        { // Don't pass the end
			blockLen = pSkt->txEnd - pos;
        }
		
		// Read those bytes to a buffer
		memcpy(buffer, (uint8_t*)pos, blockLen);
		
		// Hash and encrypt
		SSLMACAdd(buffer, blockLen);
		ARCFOURCrypt(ctx, buffer, blockLen);
		
		// Put them back
		memcpy((uint8_t*)pos, buffer, blockLen);
		
		// Update the pointers
		pos += blockLen;
		len -= blockLen;
		if(pos >= pSkt->txEnd)
        {
			pos = pSkt->txStart;
        }
	}
	
	// Calculate and add the MAC
	SSLMACCalc(MACSecret, buffer);
	ARCFOURCrypt(ctx, buffer, 16);

	// Write the MAC to the TX FIFO
	// Can't use TCPPutArray here because TCPIsPutReady() saves 16 bytes for the MAC
	// TCPPut* functions use this to prevent writing too much data.  Therefore, the
	// functionality is duplicated here.
	
	len = 16;
	blockLen = 0;
	// See if we need a two part put
	if(pSkt->sslTxHead + len >= pSkt->txEnd)
	{
		blockLen = pSkt->txEnd-pSkt->sslTxHead;
		memcpy((uint8_t*)pSkt->sslTxHead, buffer, blockLen);
		len -= blockLen;
		pSkt->sslTxHead = pSkt->txStart;
	}
	
	memcpy((uint8_t*)pSkt->sslTxHead, &buffer[blockLen], len);
	pSkt->sslTxHead += len;

}	
#endif // SSL

/*****************************************************************************
  Function:
	void TCPSSLPutRecordHeader(TCP_SOCKET hTCP, uint8_t* hdr, bool recDone)

  Summary:
	Writes an SSL record header and sends an SSL record.

  Description:
	This function writes an SSL record header to the pending TCP SSL data, 
	then indicates that the data is ready to be sent by moving the txHead
	pointer.
	
	If the record is complete, set recDone to true.  The sslTxHead 
	pointer will be moved forward 5 bytes to leave space for a future 
	record header.  If the record is only partially sent, use false and
	to leave the pointer where it is so that more data can be added
	to the record.  Partial records can only be used for the 
	SERVER_CERTIFICATE handshake message.
	
  Precondition:
	TCP is initialized, and hTCP is connected with an active SSL session.

  Parameters:
	hTCP		- TCP connection to write the header and transmit with
	hdr			- Record header (5 bytes) to send or NULL to just 
				  move the pointerctx
	recDone		- true if the record is done, false otherwise

  Returns:
	None

  Remarks:
	This function should never be called by an application.  It is used 
	only by the SSL module itself.
  ***************************************************************************/
#if defined(TCPIP_STACK_USE_SSL)
void TCPSSLPutRecordHeader(TCP_SOCKET hTCP, uint8_t* hdr, bool recDone)
{
    uint8_t i;

    // Set up the pointers
    TCB_STUB* pSkt = _TcpSocketChk(hTCP); 

    if(pSkt == 0)
    {
        return;
    }

    // Write the header if needed
    if(hdr)
    {// This is a new record, so insert the header
        for(i = 0; i < 5u; i++)
        {
            *(uint8_t*)pSkt->txHead = *(hdr+i);
            if(++pSkt->txHead >= pSkt->txEnd)
            {
                pSkt->txHead = pSkt->txStart;
            }
        }
    }

    // Move the txHead pointer to indicate what data is ready
    // Also, flush just the header, then all the data.  This shotguns two 
    // packets down the line, therefore causing immediate ACKs by the 
    // remote node.  Reconnect handshakes are as much as 60% faster now.
    _TcpFlush(pSkt);
    pSkt->txHead = pSkt->sslTxHead;
    _TcpFlush(pSkt);

    // If this record is done, move the sslTxHead forward
    // to accomodate the next record header
    if(recDone)
    {
        for(i = 0; i < 5u; i++)
        {// Skip first 5 bytes in TX for the record header
            if(++pSkt->sslTxHead >= pSkt->txEnd)
            {
                pSkt->sslTxHead = pSkt->txStart;
            }
        }
    }
}	
#endif // SSL

/*****************************************************************************
  Function:
	uint16_t TCPSSLGetPendingTxSize(TCP_SOCKET hTCP)

  Summary:
	Determines how many bytes are pending for a future SSL record.

  Description:
	This function determines how many bytes are pending for a future SSL
	record.
	
  Precondition:
	TCP is initialized, and hTCP is connected with an active SSL connection.

  Parameters:
	hTCP		- TCP connection to check

  Returns:
	None
  ***************************************************************************/
#if defined(TCPIP_STACK_USE_SSL)
uint16_t TCPSSLGetPendingTxSize(TCP_SOCKET hTCP)
{
    TCB_STUB* pSkt = _TcpSocketChk(hTCP); 

    if(pSkt == 0)
    {
        return 0;
    }

    // Non-SSL connections have no pending SSL data
    //if(pSkt->sslStubID == SSL_INVALID_ID)
    //	return 0;

    // Determine how many bytes are waiting to be written in this record
    if(pSkt->sslTxHead > pSkt->txHead)
    {
        return pSkt->sslTxHead - pSkt->txHead - 5;
    }
    else
    {
        return (pSkt->txEnd - pSkt->txStart - 1) - (pSkt->txHead - pSkt->sslTxHead - 1) - 5;
    }
}
#endif


/*****************************************************************************
  Function:
	void TCPSSLHandleIncoming(TCP_SOCKET hTCP)

  Summary:
	Hands newly arrive TCP data to the SSL module for processing.

  Description:
	This function processes incoming TCP data as an SSL record and 
	performs any necessary repositioning and decrypting.
	
  Precondition:
	TCP is initialized, and hTCP is connected with an active SSL session.

  Parameters:
	hTCP		- TCP connection to handle incoming data on

  Returns:
	None

  Remarks:
	This function should never be called by an application.  It is used 
	only by the SSL module itself.
  ***************************************************************************/
#if defined(TCPIP_STACK_USE_SSL)
void TCPSSLHandleIncoming(TCP_SOCKET hTCP)
{
    uint8_t  *prevRxTail, *nextRxHead, *startRxTail, *wSrc, *wDest;
    uint16_t wToMove, wLen, wSSLBytesThatPoofed, wDecryptedBytes;

    TCB_STUB* pSkt = _TcpSocketChk(hTCP); 

    if(pSkt == 0)
    {
        return;
    }


    // If new data is waiting
    if(pSkt->sslRxHead != pSkt->rxHead)
    {
        // Reconfigure pointers for SSL use
        prevRxTail = pSkt->rxTail;
        nextRxHead = pSkt->rxHead;
        pSkt->rxTail = pSkt->rxHead;
        pSkt->rxHead = pSkt->sslRxHead;

        do
        {
            startRxTail = pSkt->rxTail;

            // Handle incoming data.  This function performs deframing of the 
            // SSL records, decryption, and MAC verification.
            wSSLBytesThatPoofed = _TCPIsGetReady(pSkt);
            wDecryptedBytes = SSLRxRecord(hTCP, pSkt->sslStubID);
            wSSLBytesThatPoofed -= _TCPIsGetReady(pSkt);

            // Now need to move data to fill the SSL header/MAC/padding hole, 
            // if there is one
            if(wSSLBytesThatPoofed)
            {	
                // Calculate how big the SSL hole is
                if(pSkt->sHoleSize == -1)
                {// Just need to move pending SSL data
                    wToMove = _TCPIsGetReady(pSkt);
                }
                else
                {// A TCP hole exists, so move all data
                    wToMove = _TCPIsGetReady(pSkt) + pSkt->sHoleSize + pSkt->wFutureDataSize;
                }

                // Start with the destination as the startRxTail and source as current rxTail
                wDest = startRxTail;
                wSrc = pSkt->rxTail;

                // If data exists between the end of the buffer and 
                // the destination, then move it forward
                if(wSrc > wDest)
                {
                    wLen = pSkt->rxEnd - wSrc + 1;
                    if(wLen > wToMove)
                    {
                        wLen = wToMove;
                    }
                    memcpy(wDest, wSrc, wLen);
                    wDest += wLen;
                    wSrc = pSkt->rxStart;
                    wToMove -= wLen;
                }

                // If data remains to be moved, fill in to end of buffer
                if(wToMove)
                {
                    wLen = pSkt->rxEnd - wDest + 1;
                    if(wLen > wToMove)
                    {
                        wLen = wToMove;
                    }
                    memcpy(wDest, wSrc, wLen);
                    wDest = pSkt->rxStart;
                    wSrc += wLen;
                    wToMove -= wLen;
                }

                // If data still remains, copy from from front + len to front
                if(wToMove)
                {
                    memcpy(wDest, wSrc, wToMove);
                }

                // Since bytes poofed, we need to move the head pointers 
                // backwards by an equal amount.
                pSkt->rxHead -= wSSLBytesThatPoofed;
                if(pSkt->rxHead < pSkt->rxStart)
                {
                    pSkt->rxHead += pSkt->rxEnd - pSkt->rxStart + 1;
                }
                pSkt->sslRxHead = pSkt->rxHead;
            }

            // Move tail pointer forward by the number of decrypted bytes ready 
            // for the application (but not poofed bytes)
            pSkt->rxTail = startRxTail + wDecryptedBytes;
            if(pSkt->rxTail > pSkt->rxEnd)
            {
                pSkt->rxTail -= pSkt->rxEnd - pSkt->rxStart + 1;
            }
            nextRxHead += wDecryptedBytes;

            // Loop until SSLRxRecord() runs out of data and stops doing 
            // anything
        } while(wSSLBytesThatPoofed || (startRxTail != pSkt->rxTail));

        // Restore TCP buffer pointers to point to the decrypted application data 
        // only
        if(nextRxHead > pSkt->rxEnd)
        {
            nextRxHead -= pSkt->rxEnd - pSkt->rxStart + 1;
        }
        pSkt->rxTail = prevRxTail;
        pSkt->rxHead = nextRxHead;
    }
}	
#endif

/*****************************************************************************
  Function:
	void TCPSocketSetNet(TCP_SOCKET hTCP, TCPIP_NET_HANDLE hNet)

  Summary:
	Sets the interface for an TCP socket
	
  Description:
	This function sets the network interface for an TCP socket

  Precondition:
	TCP socket should have been opened with TCPOpen().
    hTCP - valid socket

  Parameters:
	hTCP - The TCP socket
   	hNet - interface handle.
	
  Returns:
    None.

  Note: None
  ***************************************************************************/
void TCPSocketSetNet(TCP_SOCKET hTCP, TCPIP_NET_HANDLE hNet)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNet(hNet);
    TCB_STUB* pSkt = _TcpSocketChk(hTCP); 
	
    if(pSkt && pNetIf)
    {
        _TcpSocketBind(pSkt, pNetIf);
    }
}

/*****************************************************************************
  Function:
	TCPIP_NET_IF* TCPSocketGetNet(TCP_SOCKET hTCP)

  Summary:
	Gets the MAC interface of an TCP socket
	
  Description:
	This function returns the MAC interface id of an TCP socket

  Precondition:
	TCP socket should have been opened with TCPOpen().
    hTCP - valid socket

  Parameters:
	hTCP - The TCP socket
	
  Returns:
    None.
  ***************************************************************************/
TCPIP_NET_HANDLE  TCPSocketGetNet(TCP_SOCKET hTCP)
{
    TCB_STUB* pSkt = _TcpSocketChk(hTCP); 
	
    if(pSkt == 0)
    {
        return 0;
    }

    return pSkt->pSktNet;
}

// allocates a new ephemeral port number
// returns 0 on error
static TCP_PORT TcpAllocateEphemeralPort(void)
{
    int      num_ephemeral;
    int      count;
    TCP_PORT next_ephemeral;


    count = num_ephemeral = LOCAL_TCP_PORT_END_NUMBER - LOCAL_TCP_PORT_START_NUMBER + 1;

    next_ephemeral = LOCAL_TCP_PORT_START_NUMBER + (SYS_Rand() % num_ephemeral);

    while(count--)
    {
        if(TCPIsAvailablePort(next_ephemeral))
        {
            return next_ephemeral;
        }

        if (next_ephemeral == LOCAL_TCP_PORT_END_NUMBER)
        {
            next_ephemeral = LOCAL_TCP_PORT_START_NUMBER;
        }
        else
        {
            next_ephemeral++;
        }
    }

    return 0;   // not found

}

static bool TCPIsAvailablePort(TCP_PORT port)
{
    int sktIx;
    TCB_STUB* pSkt; 

    // Find an available socket that matches the specified socket type
    for(sktIx = 0; sktIx < TcpSockets; sktIx++)
    {
        pSkt = TCBStubs[sktIx];
        // Sockets that are in use will be in a non-closed state
        if(pSkt && pSkt->localPort.Val == port)
        {
            return false;
        }
    }

    return true;
}

/*****************************************************************************
  Function:
	TCPBind(TCP_SOCKET s, IP_ADDRESS_TYPE addType, TCP_PORT localPort,  IP_MULTI_ADDRESS* localAddress)

  Summary:
    Bind a socket to a local address
    This function is meant for client sockets.
    It is similar to TCPSocketSetNet() that assigns a specific source interface for a socket.
    If localPort is 0 the stack will assign a unique local port

  Description:
    Sockets don't need specific binding, it is done automatically by the stack
    However, specific binding can be requested using these functions.
    Works for both client and server sockets.
    TBD: the call should fail if the socket is already bound to an interface
    (a server socket is connected or a client socket already sent the data on an interface).
    Implementation pending
  ***************************************************************************/
bool TCPBind(TCP_SOCKET hTCP, IP_ADDRESS_TYPE addType, TCP_PORT localPort,  IP_MULTI_ADDRESS* localAddress)
{
    TCPIP_NET_IF* pSktIf = 0;
    TCB_STUB* pSkt = _TcpSocketChk(hTCP); 

    if(pSkt == 0)
    {
        return false;
    }

    if(localPort == 0)
    {
        localPort = TcpAllocateEphemeralPort();
        if(localPort == 0)
        {
            return false;
        }
    }
    else if(!TCPIsAvailablePort(localPort))
    {
        return false;
    }


    if(localAddress && localAddress->v4Add.Val != 0)
    {
        pSktIf = _TCPIPStackIpAddToNet(&localAddress->v4Add, false);
        if(pSktIf == 0)
        {    // no such interface
            return false;
        }
    }
    else
    {
        if(pSkt->Flags.bServer == 0)
        {
            pSktIf = (TCPIP_NET_IF*)TCPIP_STACK_GetDefaultNet();
        }
    }

    _TcpSocketBind(pSkt, pSktIf);
    pSkt->localPort.Val = localPort;

    // recalculate the MYTCBStub remote hash value
    if(pSkt->Flags.bServer)
    {   // server socket
        pSkt->remoteHash.Val = localPort;
    }
    else
    {   // client socket
        IPV4_ADDR v4Add = TCPIP_IPV4_GetDestAddress(pSkt->pTxPkt);
        pSkt->remoteHash.Val = TcpClientIpv4RemoteHash(&v4Add, pSkt);
    }

    return true;
}

/*****************************************************************************
  Function:
	bool TCPRemoteBind(TCP_SOCKET hTCP, IP_ADDRESS_TYPE addType, TCP_PORT remotePort,  IP_MULTI_ADDRESS* remoteAddress)

  Summary:
    Bind a socket to a remote address
    This function is meant for server sockets.

  Description:
    Sockets don't need specific remote binding, they should accept connections on any incoming interface.
    Thus the binding is done automatically by the stack.
    However, specific remote binding can be requested using these functions.
    For a server socket it can be used to restrict accepting connections from  a specific remote host.
    For a client socket it will just change the default binding done when the socket was opened.
    TBD: the call should fail if the socket is already bound to an interface
    (a server socket is connected or a client socket already sent the data on an interface).
    Implementation pending
  ***************************************************************************/
bool TCPRemoteBind(TCP_SOCKET hTCP, IP_ADDRESS_TYPE addType, TCP_PORT remotePort,  IP_MULTI_ADDRESS* remoteAddress)
{
    TCB_STUB* pSkt = _TcpSocketChk(hTCP); 

    if(pSkt)
    {

        if(TCPSetDestinationIPAddress(hTCP, addType, remoteAddress))
        {
            pSkt->remotePort.Val = remotePort;
            return true;
        }
    }

    return false;
}

// Allows setting options to a socket like Rx/Tx buffer size, etc
bool TCPSetOptions(TCP_SOCKET hTCP, TCP_SOCKET_OPTION option, void* optParam)
{
    TCB_STUB* pSkt = _TcpSocketChk(hTCP); 

    if(pSkt)
    {
        switch(option)
        {
            case TCP_OPTION_LINGER:
                if(optParam)
                {
                    TCP_OPTION_LINGER_DATA* pLData = (TCP_OPTION_LINGER_DATA*)optParam;
                    pSkt->flags.nonLinger = pLData->lingerEnable == 0;
                    pSkt->flags.nonGraceful = pLData->gracefulEnable == 0;
                    return true;
                }
                return false;

            case TCP_OPTION_RX_BUFF:
                return TCPAdjustFIFOSize(hTCP, (uint16_t)((unsigned int)optParam), 0, TCP_ADJUST_RX_ONLY | TCP_ADJUST_PRESERVE_RX);

            case TCP_OPTION_TX_BUFF:
                return TCPAdjustFIFOSize(hTCP, 0, (uint16_t)((unsigned int)optParam), TCP_ADJUST_TX_ONLY | TCP_ADJUST_PRESERVE_TX);

            case TCP_OPTION_NODELAY:
                pSkt->flags.forceFlush = (int)optParam != 0;
                return true;

            default:
                return false;   // not supported option
        }
    }

    return false;
}

bool TCPGetOptions(TCP_SOCKET hTCP, TCP_SOCKET_OPTION option, void* optParam)
{
    TCB_STUB* pSkt = _TcpSocketChk(hTCP); 

    if(pSkt && optParam)
    {
        switch(option)
        {
            case TCP_OPTION_LINGER:
                {
                    TCP_OPTION_LINGER_DATA* pLData = (TCP_OPTION_LINGER_DATA*)optParam;
                    pLData->lingerEnable = pSkt->flags.nonLinger == 0;
                    pLData->gracefulEnable = pSkt->flags.nonGraceful == 0;
                    pLData->lingerTmo = 0;  // not supported
                }
                return true;

            case TCP_OPTION_RX_BUFF:
                *(uint16_t*)optParam = pSkt->rxEnd - pSkt->rxStart;
                return true;

            case TCP_OPTION_TX_BUFF:
                *(uint16_t*)optParam = pSkt->txEnd - pSkt->txStart + 1;
                return true;

            case TCP_OPTION_NODELAY:
                *(bool*)optParam = pSkt->flags.forceFlush != 0;
                return true;

            default:
                return false;   // not supported option
        }
    }

    return false;
}


bool TCPSetDestinationIPAddress(TCP_SOCKET hTCP, IP_ADDRESS_TYPE addType, IP_MULTI_ADDRESS* remoteAddress)
{
    TCB_STUB* pSkt = _TcpSocketChk(hTCP); 

    if(pSkt == 0)
    {
        return false;
    }


    switch(pSkt->addType)
    {
#if defined (TCPIP_STACK_USE_IPV6)
        case IP_ADDRESS_TYPE_IPV6:
            TCPIP_IPV6_SetDestAddress (pSkt->pTxPkt, remoteAddress?&remoteAddress->v6Add:0);
            break;
#endif  // defined (TCPIP_STACK_USE_IPV6)

#if defined (TCPIP_STACK_USE_IPV4)
        case IP_ADDRESS_TYPE_IPV4:
            TCPIP_IPV4_SetDestAddress (pSkt->pTxPkt, remoteAddress?remoteAddress->v4Add.Val:0);
            break;
#endif  // defined (TCPIP_STACK_USE_IPV4)
        default:
            return false;
    }

    return true;
}


// sets the source IP address of a packet
bool TCPSetSourceIPAddress(TCP_SOCKET hTCP, IP_ADDRESS_TYPE addType, IP_MULTI_ADDRESS* localAddress)
{
    TCB_STUB* pSkt = _TcpSocketChk(hTCP); 

    if(pSkt == 0)
    {
        return false;
    }


    switch(pSkt->addType)
    {
#if defined (TCPIP_STACK_USE_IPV6)
        case IP_ADDRESS_TYPE_IPV6:
            TCPIP_IPV6_SetSourceAddress(pSkt->pTxPkt, localAddress?&localAddress->v6Add:0);
            break;
#endif  // defined (TCPIP_STACK_USE_IPV6)

#if defined (TCPIP_STACK_USE_IPV4)
        case IP_ADDRESS_TYPE_IPV4:
            TCPIP_IPV4_SetSourceAddress(pSkt->pTxPkt, localAddress?localAddress->v4Add.Val:0);
            break;
#endif  // defined (TCPIP_STACK_USE_IPV4)
        default:
            return false;
    }

    return true;
}

// calculates a client socket remote hash value
static uint16_t TcpClientIpv4RemoteHash(IPV4_ADDR* pAdd, TCB_STUB* pSkt)
{
    return (pAdd->w[1] + pAdd->w[0] + pSkt->remotePort.Val) ^ pSkt->localPort.Val;
}

//
static bool TCP_IsTxReady(TCB_STUB * pSkt)
{
    if(pSkt->pSktNet)
    {
        switch(pSkt->addType)
        {
#if defined (TCPIP_STACK_USE_IPV6)
            case IP_ADDRESS_TYPE_IPV6:
                return TCPIP_IPV6_IsTxReady(pSkt->pSktNet);
#endif  // defined (TCPIP_STACK_USE_IPV6)

#if defined (TCPIP_STACK_USE_IPV4)
            case IP_ADDRESS_TYPE_IPV4:
                return TCPIP_IPV4_IsTxReady(pSkt->pSktNet);
#endif  // defined (TCPIP_STACK_USE_IPV4)

            default:
                break;
        }
    }

    return false;
}

static void * TCP_GetUpperLayerHeaderPtr(TCB_STUB * pSkt)
{
    if(pSkt->pTxPkt)
    {
        switch(pSkt->addType)
        {
#if defined (TCPIP_STACK_USE_IPV6)
            case IP_ADDRESS_TYPE_IPV6:
                return TCPIP_IPV6_GetUpperLayerHeaderPtr(pSkt->pTxPkt);
#endif  // defined (TCPIP_STACK_USE_IPV6)

#if defined (TCPIP_STACK_USE_IPV4)
            case IP_ADDRESS_TYPE_IPV4:
                return TCPIP_IPV4_GetUpperLayerHeaderPtr(pSkt->pTxPkt);
#endif  // defined (TCPIP_STACK_USE_IPV4)

            default:
                break;
        }
    }

    return 0;
}

static unsigned short TCP_IsTxPutReady (TCB_STUB * pSkt, unsigned short count)
{
    if(pSkt->pTxPkt)
    {
        switch(pSkt->addType)
        {
#if defined (TCPIP_STACK_USE_IPV6)
            case IP_ADDRESS_TYPE_IPV6:
                return TCPIP_IPV6_IsTxPutReady (pSkt->pTxPkt, count);
#endif  // defined (TCPIP_STACK_USE_IPV6)

#if defined (TCPIP_STACK_USE_IPV4)
            case IP_ADDRESS_TYPE_IPV4:
                return TCPIP_IPV4_IsTxPutReady (pSkt->pTxPkt, count);
#endif  // defined (TCPIP_STACK_USE_IPV4)

            default:
                break;
        }
    }
    return 0;
}

static unsigned short TCP_PutArray(TCB_STUB * pSkt, const void * dataSource, unsigned short len)
{
    if(pSkt->pTxPkt)
    {
        switch(pSkt->addType)
        {
#if defined (TCPIP_STACK_USE_IPV6)
            case IP_ADDRESS_TYPE_IPV6:
                return TCPIP_IPV6_PutArray(pSkt->pTxPkt, dataSource, len);
#endif  // defined (TCPIP_STACK_USE_IPV6)

#if defined (TCPIP_STACK_USE_IPV4)
            case IP_ADDRESS_TYPE_IPV4:
                return TCPIP_IPV4_PutArray(pSkt->pTxPkt, dataSource, len);
#endif  // defined (TCPIP_STACK_USE_IPV4)

            default:
                break;
        }
    }

    return 0;
}

static unsigned short TCP_SetPayload(TCB_STUB * pSkt, uint8_t* payload, unsigned short len)
{
    if(pSkt->pTxPkt)
    {
        switch(pSkt->addType)
        {
#if defined (TCPIP_STACK_USE_IPV6)
            case IP_ADDRESS_TYPE_IPV6:
                return TCPIP_IPV6_SetPayload(pSkt->pTxPkt, payload, len);
#endif  // defined (TCPIP_STACK_USE_IPV6)

#if defined (TCPIP_STACK_USE_IPV4)
            case IP_ADDRESS_TYPE_IPV4:
                return TCPIP_IPV4_SetPayload(pSkt->pTxPkt, payload, len);
#endif  // defined (TCPIP_STACK_USE_IPV4)

            default:
                break;
        }
    }

    return 0;
}

static void TCP_PutHeader(TCB_STUB * pSkt, uint8_t protocol)
{
    if(pSkt->pTxPkt)
    {
        switch(pSkt->addType)
        {
#if defined (TCPIP_STACK_USE_IPV6)
            case IP_ADDRESS_TYPE_IPV6:
                return TCPIP_IPV6_PutHeader(pSkt->pTxPkt, protocol);
#endif  // defined (TCPIP_STACK_USE_IPV6)

#if defined (TCPIP_STACK_USE_IPV4)
            case IP_ADDRESS_TYPE_IPV4:
                return TCPIP_IPV4_PutHeader(pSkt->pTxPkt, protocol);
#endif  // defined (TCPIP_STACK_USE_IPV4)

            default:
                break;
        }
    }
}

static bool TCP_Flush(TCB_STUB * pSkt, MAC_ADDR * remoteMACAddr)
{
    if(pSkt->pTxPkt)
    {
        switch(pSkt->addType)
        {
#if defined (TCPIP_STACK_USE_IPV6)
            case IP_ADDRESS_TYPE_IPV6:
                return TCPIP_IPV6_Flush(pSkt->pTxPkt, remoteMACAddr);
#endif  // defined (TCPIP_STACK_USE_IPV6)

#if defined (TCPIP_STACK_USE_IPV4)
            case IP_ADDRESS_TYPE_IPV4:
                return TCPIP_IPV4_Flush(pSkt->pTxPkt, remoteMACAddr);
#endif  // defined (TCPIP_STACK_USE_IPV4)

            default:
                break;
        }
    }
    return false;
}

static bool TCP_IsPacketQueued(TCB_STUB * pSkt)
{
    if(pSkt->pTxPkt)
    {
        switch(pSkt->addType)
        {
#if defined (TCPIP_STACK_USE_IPV6)
            case IP_ADDRESS_TYPE_IPV6:
                return TCPIP_IPV6_IsPacketQueued(pSkt->pTxPkt);
#endif  // defined (TCPIP_STACK_USE_IPV6)

#if defined (TCPIP_STACK_USE_IPV4)
            case IP_ADDRESS_TYPE_IPV4:
                return TCPIP_IPV4_IsPacketQueued(pSkt->pTxPkt);
#endif  // defined (TCPIP_STACK_USE_IPV4)

            default:
                break;
        }
    }
    return false;
}

static bool TCP_CopyTxPacketStruct (void * destination, TCB_STUB * pSkt)
{
    if(pSkt->pTxPkt)
    {
        switch(pSkt->addType)
        {
#if defined (TCPIP_STACK_USE_IPV6)
            case IP_ADDRESS_TYPE_IPV6:
                return TCPIP_IPV6_CopyTxPacketStruct (destination, pSkt->pTxPkt);
#endif  // defined (TCPIP_STACK_USE_IPV6)

#if defined (TCPIP_STACK_USE_IPV4)
            case IP_ADDRESS_TYPE_IPV4:
                return TCPIP_IPV4_CopyTxPacketStruct (destination, pSkt->pTxPkt);
#endif  // defined (TCPIP_STACK_USE_IPV4)

            default:
                break;
        }
    }
    return false;
}

static void TCP_FreePacket(void * pkt, IP_ADDRESS_TYPE addType)
{
    switch(addType)
    {
#if defined (TCPIP_STACK_USE_IPV6)
        case IP_ADDRESS_TYPE_IPV6:
            TCPIP_IPV6_FreePacket(pkt);
            break;
#endif  // defined (TCPIP_STACK_USE_IPV6)

#if defined (TCPIP_STACK_USE_IPV4)
        case IP_ADDRESS_TYPE_IPV4:
            TCPIP_IPV4_FreePacket(pkt);
            break;
#endif  // defined (TCPIP_STACK_USE_IPV4)

        default:
            break;
    }
}

static void TCP_ResetTransmitPacketState (TCB_STUB * pSkt)
{
    if(pSkt->pTxPkt)
    {
        switch(pSkt->addType)
        {
#if defined (TCPIP_STACK_USE_IPV6)
            case IP_ADDRESS_TYPE_IPV6:
                TCPIP_IPV6_ResetTransmitPacketState (pSkt->pTxPkt);
                break;
#endif  // defined (TCPIP_STACK_USE_IPV6)

#if defined (TCPIP_STACK_USE_IPV4)
            case IP_ADDRESS_TYPE_IPV4:
                TCPIP_IPV4_ResetTransmitPacketState (pSkt->pTxPkt);
                break;
#endif  // defined (TCPIP_STACK_USE_IPV4)

            default:
                break;
        }
    }
}


#endif //#if defined(TCPIP_STACK_USE_TCP)

