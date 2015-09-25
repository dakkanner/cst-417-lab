/*******************************************************************************
  TCP/IP iperf implementation

  Summary:
    Runs iperf client and server
    
  Description:
    - Implements iperf benchmarking
*******************************************************************************/

/*******************************************************************************
FileName:  iperf.c 
Copyright ?2012 released Microchip Technology Inc.  All rights
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

#include <string.h> 

#include "tcpip_private.h"

#include "iperf.h"


#if defined(TCPIP_STACK_USE_IPERF)

#define TCPIP_THIS_MODULE_ID    TCPIP_MODULE_TCPIP_IPERF

//****************************************************************************
// CONSTANTS (Defines and enums)
//****************************************************************************

#define UDP_FIN_RETRANSMIT_COUNT		10u     // iperf retransmits 10 times the last UDP packet,
#define UDP_FIN_RETRANSMIT_PERIOD       10      // at 10ms apart.
#define TIMING_ERROR_MARGIN              2      // Account for msec tick uncertainty.


// TCP Maximum Segment Size - MSS;
#define IPERF_TCP_MSS  TCP_MAX_SEG_SIZE_TX


typedef enum {
    UDP_PROTOCOL = 1,
    TCP_PROTOCOL
} tIperfProto;

enum {
    IPERF_STANDBY_STATE=1,

    IPERF_RX_START_STATE,
    IPERF_UDP_RX_STATE,
    IPERF_UDP_RX_DRAIN_STATE,
    IPERF_UDP_RX_DONE_STATE,
    IPERF_TCP_RX_LISTEN_STATE,
    IPERF_TCP_RX_STATE,
    IPERF_TCP_RX_DONE_STATE,
    IPERF_RX_DONE_STATE,

    IPERF_TX_START_STATE,
    IPERF_TX_ARP_RESOLVE_STATE,

    IPERF_TCP_TX_OPEN_STATE,
    IPERF_TCP_TX_CONNECT_STATE,
    IPERF_TCP_TX_SEGMENT_STATE,
    IPERF_TCP_TX_DONE_STATE,

    IPERF_UDP_TX_OPEN_STATE,
    IPERF_UDP_TX_DATAGRAM_STATE,
    IPERF_UDP_TX_DONE_STATE
};

typedef enum
{
    INTERVAL_REPORT,
    SUBTOTAL_REPORT,
    SESSION_REPORT
} tIperfReport;

typedef enum
{
    IPERF_TX_WAIT   = 0,    // wait some more for transmitting
    IPERF_TX_OK,            // can go ahead and transmit
    IPERF_TX_FAIL,          // cannot get TX resources, abort
}tIperfTxResult;

//****************************************************************************
// LOCAL DATA TYPES                                                             
//****************************************************************************

/* tIperfState */
typedef struct
{
    uint32_t		mInterval;		// -i
    uint32_t		mAmount;		// -n
    uint32_t		mDuration;		// -t. Default = 10*TICK_SECOND msec
    uint32_t		mDatagramSize;	// -l
    uint16_t		mMSS;			// -M
    bool		mServerMode;    // -s or -c
    bool   		mBufLenSet;     // -l
    tIperfProto	mProtocol;		// -b or -u
    uint16_t		mServerPort;	// -p

    uint32_t		mTxRate;		// -b or
                                // -x: NONE-STANDARD IPERF OPTION. Max Tx bps rate for TCP.

    double			totalLen; // mTotalLen
    long 			pktId; 		// datagramID
    long			lastPktId; // lastDatagramID
    uint32_t		errorCount;
    uint32_t		outofOrder;

    TCP_SOCKET tcpServerSock;
    TCP_SOCKET tcpClientSock;

    UDP_SOCKET udpSock;

    TCP_SOCKET_INFO  remoteSide;
    MAC_ADDR         remoteMACAddr; // remote host MAC address
    uint16_t		 localPort;
    IPV4_ADDR        localAddr;     // local address/interface to use

    //struct sockaddr_in remoteAddr;
   // int 		remoteAddrlen;
    //	tSocketAddr remoteAddr;

    // Calculated packet period, in msec, to reflect the target bit rate.
    uint32_t		mPktPeriod;

    uint32_t		startTime;
    uint32_t		stopTime;
    uint32_t		nextTxTime;
    //uint32_t		remoteStartTime;
    //uint32_t		remoteStopTime;

    uint8_t			nAttempts;
    uint32_t		pktCount;
    bool			stopRequested;


    uint32_t		lastCheckPktCount;  // Check if pktCount changes within mInterval; or kIperfRxTimeOut.
    long			lastCheckPktId;
    uint32_t		lastCheckErrorCount;
    uint32_t		lastCheckTotalLen;
    uint32_t		lastCheckTime;
    uint8_t 		statusReported;

//	long		mPendingACK;		// number of outstanding TCP ACKs
//	uint8_t		mRetransmit;

    uint8_t 		state;

    uint32_t      timer;
    uint16_t      remainingTxData;
    uint16_t      availUDPTxSpace;

    bool     	isLastTransmit;

    uint32_t    txWaitTick;

    uint32_t    txBuffSize;
    uint32_t    rxBuffSize;

    // console that invoked iperf
    _CMDIO_DEV_NODE* pCmdIO;
    // interface to use
    TCPIP_NET_HANDLE pNetIf;
} tIperfState;



//
// Data structure used by iperf protocol
//

#define HEADER_VERSION1 0x80000000

typedef struct
{
    long id;
    uint32_t tv_sec;
    uint32_t tv_usec;
} tIperfPktInfo; 	// In the original Iperf, this is the "UDP_datagram" structure.

// tUDP_datagram
typedef struct
{
    uint32_t flags;
    uint32_t total_len1;
    uint32_t total_len2;
    uint32_t stop_sec;
    uint32_t stop_usec;
    uint32_t error_cnt;
    uint32_t outorder_cnt;
    uint32_t datagrams;
    uint32_t jitter1;
    uint32_t jitter2;
} tServerHdr;

typedef struct
{
    uint32_t flags;
    uint32_t numThreads;
    uint32_t mPort;
    uint32_t bufferlen;
    uint32_t mWinBand;
    uint32_t mAmount;
} tClientHdr;

extern NODE_INFO remoteNode;    
//****************************************************************************
// LOCAL GLOBALS                                                             
//****************************************************************************

#define MAX_BUFFER   (sizeof(tIperfPktInfo) + sizeof(tServerHdr))
uint8_t  g_bfr[ MAX_BUFFER ];


static tIperfState gIperfState;

static int CommandIperfStart(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv);
static int CommandIperfStop(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv);
static int CommandIperfNetIf(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv);
static int CommandIperfSize(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv);
// Iperf command table
static const _SYS_CMD_DCPT    iperfCmdTbl[]=
{
    {"iperf",   	CommandIperfStart, 	":  <iperf> start cmd"},
    {"iperfk",  	CommandIperfStop,	": <iperfk> kill cmd"},
    {"iperfi",  	CommandIperfNetIf,	": <iperfi address>  interface cmd"},
    {"iperfs",  	CommandIperfSize,	": <iperfs tx/rx size> tx/rx size cmd"}
};


//****************************************************************************
// LOCAL Prototypes                                                             
//****************************************************************************
static void StateMachineUdpTxDone(void);
static void StateMachineTcpTxDone(void);
static void GenericTxDone(void);
static void StateMachineUdpTxDatagram(void);
static void StateMachineTcpTxSegment(void);
static void GenericTxEnd(void);
static void UdpTxFillDatagram(void);
static void TcpTxFillSegment(void);
static tIperfTxResult GenericTxStart(void);
static void GenericTxHeaderPreparation(uint8_t *pData, bool isTheLastTransmit);
static void StateMachineTCPTxOpen(void);
static void StateMachineUDPTxOpen(void);
static void StateMachineTxArpResolve(void);
static void StateMachineTxStart(void);
static void StateMachineRxDone(void);
static void StateMachineTcpRxDone(void);
static void StateMachineUdpRxDone(void);
static void StateMachineUdpRxDrain(void);
static void StateMachineUdpRx(void);
static void StateMachineTcpRx(void);
static void StateMachineTcpListen(void);
static void StateMachineRxStart(void);
static void ReportBW_Jitter_Loss(tIperfReport reportType);
static void ascii_to_u32s(char *ptr, uint32_t *values, uint8_t count);
static void ResetIperfCounters(void);
static void StateMachineTCPTxConnect(void);

//****************************************************************************
// Implementation: public functions
//****************************************************************************

bool TCPIP_IPERF_Initialize(const TCPIP_STACK_MODULE_CTRL* const stackCtrl, const void* initData)
{
    memset( &gIperfState, 0, sizeof(tIperfState) );

    gIperfState.state = IPERF_STANDBY_STATE;
    gIperfState.stopRequested = false;

    gIperfState.tcpClientSock = INVALID_SOCKET;
    gIperfState.tcpServerSock = INVALID_SOCKET;
    gIperfState.udpSock = INVALID_SOCKET;
    gIperfState.txBuffSize = IPERF_TX_BUFFER_SIZE;
    gIperfState.rxBuffSize = IPERF_RX_BUFFER_SIZE;
    
    return _SYS_COMMAND_ADDGRP(iperfCmdTbl, sizeof(iperfCmdTbl)/sizeof(*iperfCmdTbl), "iperf", ": iperf commands");

}

void TCPIP_IPERF_Deinitialize(const TCPIP_STACK_MODULE_CTRL* const stackCtrl)
{
}

void TCPIP_IPERF_Task(void)
{
	if (gIperfState.state == IPERF_STANDBY_STATE)
    {
       return;
    }

    switch ( gIperfState.state )
    {
        /********************/
        /* RX Client States */
        /********************/

        case IPERF_RX_START_STATE:

            StateMachineRxStart();

            break;

        case IPERF_UDP_RX_STATE:

            StateMachineUdpRx();

            break;

        case IPERF_UDP_RX_DRAIN_STATE:

            StateMachineUdpRxDrain();

            break;


        case IPERF_UDP_RX_DONE_STATE:

            StateMachineUdpRxDone();

            break;


        case IPERF_TCP_RX_LISTEN_STATE:

            StateMachineTcpListen();

            break;

        case IPERF_TCP_RX_STATE:

            StateMachineTcpRx();

            break;

        case IPERF_TCP_RX_DONE_STATE:

            StateMachineTcpRxDone();

            break;

        case IPERF_RX_DONE_STATE:

            StateMachineRxDone();

            break;


       /********************/
       /* TX Client states */
       /********************/


        case IPERF_TX_START_STATE:

            StateMachineTxStart();

            break;


        case IPERF_TX_ARP_RESOLVE_STATE:

           StateMachineTxArpResolve();

           break;

        case IPERF_UDP_TX_OPEN_STATE:

            StateMachineUDPTxOpen();

            break;

        case IPERF_TCP_TX_OPEN_STATE:

            StateMachineTCPTxOpen();

            break;

        case IPERF_TCP_TX_CONNECT_STATE:

            StateMachineTCPTxConnect();

            break;

        case IPERF_TCP_TX_SEGMENT_STATE:

            StateMachineTcpTxSegment();

            break;

        case IPERF_UDP_TX_DATAGRAM_STATE:

            StateMachineUdpTxDatagram();

            break;

        case IPERF_TCP_TX_DONE_STATE:

            StateMachineTcpTxDone();

            break;


        case IPERF_UDP_TX_DONE_STATE:

            StateMachineUdpTxDone();

            break;

		default:
			gIperfState.state = IPERF_STANDBY_STATE;
			break;

        }
}


//****************************************************************************
// Implementation: local functions
//****************************************************************************

static void ResetIperfCounters(void)
{
    // gIperfState.mAmount = 0;
    // gIperfState.mDuration = 10*1000; // -t: default 10 sec
    // gIperfState.mInterval = 1000; 	// -i: default 1 sec
    gIperfState.mMSS = IPERF_TCP_MSS;
    gIperfState.mDatagramSize = 1470; // -l: default 1470 bytes. UDP datagram size.
    gIperfState.totalLen = 0;
    gIperfState.pktId = 0;
    gIperfState.lastPktId = 0;
    gIperfState.errorCount = 0;
    gIperfState.outofOrder = 0;
    gIperfState.pktCount = 0;
    gIperfState.statusReported = 0;
    gIperfState.startTime = 0;
    gIperfState.stopTime = 0;

    gIperfState.lastCheckPktCount = 0;
    gIperfState.lastCheckPktId = 0;
    gIperfState.lastCheckErrorCount = 0;
    gIperfState.lastCheckTotalLen = 0;
    gIperfState.lastCheckTime = 0;

    gIperfState.isLastTransmit = false;

    gIperfState.txWaitTick = 0;
//	gIperfState.mPendingACK = 0;
//	gIperfState.mRetransmit = 0;

}

static void ascii_to_u32s(char *ptr, uint32_t *values, uint8_t count)
{
    uint8_t i;
    uint32_t tmp;

    // Convert "123.456_78_90k", with count set to 4,  to
    // unsigned 32-bit numbers 123, 456, 78 and 90000, and
    // store them in the values array.

    for (i = 0; i < count; i++)
    {
        tmp = 0;

        while ( (*ptr > (int8_t)'9') || (*ptr < (int8_t)'0') )
        {
            if ( (*ptr == (int8_t)' ') || (*ptr == (int8_t)0) ) return; // terminates at blank or NULL.

            ptr++;
        }

        while ( (*ptr <= (int8_t)'9') && (*ptr >= (int8_t)'0') )
        {
            tmp = tmp*10 + *ptr - '0';
            ptr++;
        }
        if ( (*ptr == (int8_t)'k') || (*ptr == (int8_t)'K') )
        {
            tmp = tmp * 1000;
            ptr++;
        }
        else if ( (*ptr == (int8_t)'m') || (*ptr == (int8_t)'M') )
        {
            tmp = tmp * 1000 * 1000;
            ptr++;
        }

        values[i] = tmp;
    }
}

//
// Report bandwith, jitter, and packet loss stastistics.
// Used by in both server and client modes.
//
// Todo: implement the jitter report.
//

static void ReportBW_Jitter_Loss(tIperfReport reportType)
{
    uint32_t nAttempted;
    uint32_t nDropped;
    double kbps;
    uint32_t currentTime;
    uint32_t sec;
	uint32_t msec;
    const void* cmdIoParam = gIperfState.pCmdIO->cmdIoParam;

    currentTime = SYS_TICK_Get();

    switch ( reportType )
    {
        case INTERVAL_REPORT:

            nDropped = gIperfState.errorCount - gIperfState.lastCheckErrorCount;

            // bits-per-msec == Kbps

            sec = (currentTime- gIperfState.lastCheckTime)/SYS_TICK_TicksPerSecondGet();
			msec = ((double) (currentTime - gIperfState.lastCheckTime)) / (((double)(SYS_TICK_TicksPerSecondGet()))/1000);

            if ( gIperfState.state == (uint8_t)IPERF_UDP_TX_DONE_STATE )
            {
               nAttempted = (gIperfState.lastPktId - gIperfState.lastCheckPktId) + nDropped;
            }
            else
            {
                nAttempted = gIperfState.pktId - gIperfState.lastCheckPktId;
            }

			if ( msec == 0u )
            {
                kbps = 0;
            }
            else
            {
				kbps = ((gIperfState.totalLen - gIperfState.lastCheckTotalLen)*((double) 8)) / msec;
            }

            sec = (gIperfState.lastCheckTime - gIperfState.startTime)/SYS_TICK_TicksPerSecondGet();

            (gIperfState.pCmdIO->pCmdApi->print)(cmdIoParam, "    - [%2lu- %2lu sec] %3lu/ %3lu (%2lu%%)    %4lu Kbps\r\n",
                      (unsigned long)sec, 
                      (unsigned long)sec + ( (unsigned long) (gIperfState.mInterval/SYS_TICK_TicksPerSecondGet()) ),
                      (unsigned long)nDropped,
                      (unsigned long)nAttempted,
                      (unsigned long)nDropped*100/(unsigned long)nAttempted,
                      (unsigned long) (kbps + ((double) 0.5)));
            break;


        case SUBTOTAL_REPORT:
            // intentional fall-through
        case SESSION_REPORT:

           nDropped = gIperfState.errorCount;

           if (gIperfState.state == (uint8_t)IPERF_UDP_TX_DONE_STATE)
            {
               nAttempted = gIperfState.lastPktId + nDropped;
            }
            else
            {
                nAttempted = gIperfState.lastPktId;
            }

			msec = ((double) (gIperfState.stopTime - gIperfState.startTime)) / (((double)(SYS_TICK_TicksPerSecondGet()))/1000);

			if ( msec == 0u )
            {
                kbps = 0;
            }
            else
            {
   				kbps = (gIperfState.totalLen * ((double) 8)) / msec;
            }

            (gIperfState.pCmdIO->pCmdApi->print)(cmdIoParam, "    - [0.0- %lu.%lu sec] %3lu/ %3lu (%2lu%%)    %4lu Kbps\r\n",
                             (unsigned long)(msec/1000),
                             (unsigned long)((msec%1000)/100),
                             (unsigned long)nDropped,
                             (unsigned long)nAttempted,
                             (nAttempted == 0u) ? 0 : ((unsigned long)nDropped*100/(unsigned long)nAttempted),
                             (unsigned long) (kbps + ((double) 0.5)));
            break;
    }

    if ( reportType == 	SESSION_REPORT )
    {
      (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: Session completed ...");
    }

    gIperfState.lastCheckPktId = gIperfState.pktId;
    gIperfState.lastCheckErrorCount = gIperfState.errorCount;
    gIperfState.lastCheckPktCount = gIperfState.pktCount;
    gIperfState.lastCheckTime = currentTime;
    gIperfState.lastCheckTotalLen = gIperfState.totalLen;
}


static void StateMachineRxStart(void)
{
    const void* cmdIoParam = gIperfState.pCmdIO->cmdIoParam;
    if ( !gIperfState.mServerMode )
    {
        (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: Unsupported Configuration\r\n");
        gIperfState.state = IPERF_STANDBY_STATE;
        return;
    }


    switch ( gIperfState.mProtocol )
    {
    case TCP_PROTOCOL:	// TCP
        /* TCP Server sockets are allocated for entire runtime duration, a call to disconnect does not free them */
        /* therefore a subsequent N+1 open will fail */
        if ( (gIperfState.tcpServerSock == INVALID_SOCKET) &&
            (gIperfState.tcpServerSock = TCPOpenServer(IP_ADDRESS_TYPE_IPV4, gIperfState.mServerPort, (IP_MULTI_ADDRESS*)&gIperfState.localAddr)) == INVALID_SOCKET )
        {
            /* error case */
            (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: Create TCP socket failed\r\n");
            gIperfState.state = IPERF_STANDBY_STATE;
            return;
        }

        if(!TCPSetOptions(gIperfState.tcpServerSock, TCP_OPTION_RX_BUFF, (void*)gIperfState.rxBuffSize))
        {
            (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: Set of RX buffer size failed\r\n");
        }

        gIperfState.state = IPERF_TCP_RX_LISTEN_STATE;
        break;

    case UDP_PROTOCOL:	// UDP
        if ( (gIperfState.udpSock = UDPOpenServer(IP_ADDRESS_TYPE_IPV4, gIperfState.mServerPort, (IP_MULTI_ADDRESS*)&gIperfState.localAddr)) == INVALID_UDP_SOCKET )
        {
            /* error case */
            (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: Create UDP socket failed\r\n");
            gIperfState.state = IPERF_STANDBY_STATE;
            return;
        }

        gIperfState.state = IPERF_UDP_RX_STATE;
        break;

    default:
        (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: Protocol error\r\n");
        gIperfState.state = IPERF_STANDBY_STATE;
        return;
    }
}


static void StateMachineTcpListen(void)
{

   if ( gIperfState.stopRequested == true )
   {
        gIperfState.state = IPERF_RX_DONE_STATE;
        return;
   }

   if( TCPIsConnected(gIperfState.tcpServerSock) )
   {
      TCP_SOCKET_INFO tcpSocketInfo;
	  TCPGetSocketInfo( gIperfState.tcpServerSock, &tcpSocketInfo);
      memcpy ( (void *) &gIperfState.remoteSide, &tcpSocketInfo, sizeof ( TCP_SOCKET_INFO) );
      gIperfState.state = IPERF_TCP_RX_STATE;

      /* clear the stack's reset flag */
      TCPWasReset(gIperfState.tcpServerSock);
   }
}


static void StateMachineTcpRx(void)
{
    uint16_t length;
    const void* cmdIoParam = gIperfState.pCmdIO->cmdIoParam;

    if( (length = TCPIsGetReady(gIperfState.tcpServerSock)) == 0 )
    {

      if ( TCPWasReset(gIperfState.tcpServerSock)  )
      {
          gIperfState.stopTime = SYS_TICK_Get();
          gIperfState.state = IPERF_TCP_RX_DONE_STATE;
          return;
      }

    }
    else
    {
       if ( gIperfState.pktId == 0)
       {
          IPV4_ADDR lclAddress;
          
          // This is the first rx pkt.
          (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "\r\niperf: Session started ...\r\n");

          gIperfState.startTime = SYS_TICK_Get();
          gIperfState.lastCheckTime = 	gIperfState.startTime;

          gIperfState.lastCheckPktId = gIperfState.pktId;
          lclAddress.Val = TCPIP_STACK_NetAddress(TCPSocketGetNet(gIperfState.tcpServerSock));

          (gIperfState.pCmdIO->pCmdApi->print)(cmdIoParam, "    - Local  %u.%u.%u.%u port %u connected with\r\n",
                   lclAddress.v[0],
                   lclAddress.v[1],
                   lclAddress.v[2],
                   lclAddress.v[3],
                   gIperfState.mServerPort);

          (gIperfState.pCmdIO->pCmdApi->print)(cmdIoParam, "    - Remote %u.%u.%u.%u port %u\r\n",
                   gIperfState.remoteSide.remoteIPaddress.v4Add.v[0],
                   gIperfState.remoteSide.remoteIPaddress.v4Add.v[1],
                   gIperfState.remoteSide.remoteIPaddress.v4Add.v[2],
                   gIperfState.remoteSide.remoteIPaddress.v4Add.v[3],
                   gIperfState.remoteSide.remotePort );
       }

       gIperfState.pktId++;
       gIperfState.pktCount++;
       gIperfState.lastPktId = gIperfState.pktId;
       gIperfState.totalLen += length;

       /* read the remaining datagram payload */
       /* a UdpDiscard would be disingenuous, because it would not reflect the bandwidth at L7 */
       while ( length > 0 )
       {
          uint16_t chunk;

          if ( length <  (uint16_t)MAX_BUFFER )
            chunk = length;
          else
            chunk = MAX_BUFFER;

          TCPGetArray( gIperfState.tcpServerSock, (uint8_t*)g_bfr, chunk);
          length -= chunk;
       }

    }

    if ((gIperfState.pktId != (long)0) &&
       (SYS_TICK_Get() > (gIperfState.lastCheckTime + gIperfState.mInterval)) )
    {
         // Time to report statistics
         ReportBW_Jitter_Loss(INTERVAL_REPORT);
    }

    if ( gIperfState.stopRequested == true )
    {
       gIperfState.state = IPERF_TCP_RX_DONE_STATE;
       gIperfState.stopTime = SYS_TICK_Get();

       return;
    }
}


static void StateMachineUdpRx(void)
{
    uint16_t length =0;
    tIperfPktInfo *pPktInfo;
    UDP_SOCKET_INFO UdpSkt;
    IPV4_ADDR      lclIpAddr, remIpAddr;
    const void* cmdIoParam = gIperfState.pCmdIO->cmdIoParam;


    // Do nothing if no data is waiting
    if( (length = UDPIsGetReady(gIperfState.udpSock)) >= (uint16_t)(sizeof(tIperfPktInfo)) )
    {

       /* The GetArray should not fail... */
       if ( UDPGetArray(gIperfState.udpSock, (uint8_t*)g_bfr, sizeof(tIperfPktInfo)) != sizeof(tIperfPktInfo) )
       {
          (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: UDP Get Array Failed\r\n");
          gIperfState.state = IPERF_UDP_RX_DRAIN_STATE;
          return;
       }

       pPktInfo = (tIperfPktInfo *) g_bfr;
       gIperfState.pktId = TCPIP_HELPER_htonl(pPktInfo->id);

       if ( (gIperfState.pktCount == (uint32_t)0) && (gIperfState.pktId < (long)0) )
       {
          // Ignore retransmits from previous session.
          UDPDiscard(gIperfState.udpSock);
          return;
       }

       gIperfState.pktCount++;
       if (gIperfState.pktCount == (uint32_t)1 )
       {
          // The first pkt is used to set up the server,
          // does not count as a data pkt.

          (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "\r\niperf: Session started ...");

          if ( gIperfState.pktId != 0 )
          {
             // We have lost a few packets before the first pkt arrived.
             (gIperfState.pCmdIO->pCmdApi->print)(cmdIoParam, "iperf: - First pkt id = %ld (should be 0)\r\n",
                              gIperfState.pktId);
             // The first data pkt starts with id = 1.
             gIperfState.errorCount	+= 	gIperfState.pktId - 1;
          }

          gIperfState.lastPktId = gIperfState.pktId;

		  UDPGetSocketInfo(gIperfState.udpSock, &UdpSkt);
          lclIpAddr.Val = TCPIP_STACK_NetAddress(UDPSocketGetNet(gIperfState.udpSock));
          remIpAddr.Val = remoteNode.IPAddr.Val;
          (gIperfState.pCmdIO->pCmdApi->print)(cmdIoParam, "    - Local  %u.%u.%u.%u port %u connected with\r\n",
                           lclIpAddr.v[0],
                           lclIpAddr.v[1],
                           lclIpAddr.v[2],
                           lclIpAddr.v[3],
                           gIperfState.mServerPort);
		  
          (gIperfState.pCmdIO->pCmdApi->print)(cmdIoParam, "    - Remote %u.%u.%u.%u port %u\r\n",      
                           remIpAddr.v[0],
                           remIpAddr.v[1],
                           remIpAddr.v[2],
                           remIpAddr.v[3],
                           UdpSkt.remotePort);

          // Store the remote info so we can send the iperf "UDP-FIN-ACK" msg
          gIperfState.remoteSide.remoteIPaddress.v4Add.Val = remIpAddr.Val;
          gIperfState.remoteSide.remotePort =  UdpSkt.remotePort;

          gIperfState.startTime = SYS_TICK_Get();
          //gIperfState.remoteStartTime = IPERFTOHL(pPktInfo->tv_sec);

          gIperfState.lastCheckTime = 	gIperfState.startTime;

          gIperfState.lastCheckPktId = gIperfState.pktId;
          gIperfState.lastCheckPktCount = gIperfState.pktCount;
          gIperfState.lastCheckErrorCount = gIperfState.errorCount;

          UDPDiscard(gIperfState.udpSock);

          return;

      }

      gIperfState.totalLen += length;

      if ( gIperfState.pktId < 0 )
      {
         // this is the last datagram
         gIperfState.pktId = - gIperfState.pktId;

         gIperfState.stopTime = SYS_TICK_Get();
        //gIperfState.remoteStopTime = IPERFTOHL(pPktInfo->tv_sec);

        gIperfState.nAttempts = 0;
        gIperfState.state = IPERF_UDP_RX_DRAIN_STATE;
      }

      if ( gIperfState.pktId != gIperfState.lastPktId+1 )
      {
         if ( gIperfState.pktId < gIperfState.lastPktId+1 )
         {
            gIperfState.outofOrder++;
         }
         else
         {
            gIperfState.errorCount += gIperfState.pktId - (gIperfState.lastPktId+1);
         }
      }

      // never decrease pktId (e.g. if we get an out-of-order packet)
      if ( gIperfState.pktId == gIperfState.lastPktId )
      {
         (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: Recv duplicated pkt\r\n");
      }

      if ( gIperfState.pktId > gIperfState.lastPktId )
      {
         gIperfState.lastPktId = gIperfState.pktId;
      }

      /* read the remaining datagram payload - the full payload */
      /* a UdpDiscard would be disingenuous, because it would not reflect the bandwidth at L7 */
      length -=  sizeof(tIperfPktInfo);
      while ( length > 0 )
      {
         uint16_t chunk;

         if ( length <  (uint16_t)MAX_BUFFER )
            chunk = length;
         else
            chunk = MAX_BUFFER;

         UDPGetArray(gIperfState.udpSock, (uint8_t*)g_bfr, chunk);
         length -= chunk;
      }


    }  /* end got a datagram */

    if ( (gIperfState.pktCount != (uint32_t)0) &&
         (SYS_TICK_Get() > (gIperfState.lastCheckTime + gIperfState.mInterval)) )
    {
        if ( gIperfState.pktCount == gIperfState.lastCheckPktCount )
        {
          // No events in gIperfState.mInterval msec, we timed out
          (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: Rx timed out\r\n");

          gIperfState.stopTime = SYS_TICK_Get();

          gIperfState.nAttempts = 0;
          gIperfState.state = IPERF_UDP_RX_DRAIN_STATE;
        }
        else
        {
          ReportBW_Jitter_Loss(INTERVAL_REPORT);
        }
    }

    if ( gIperfState.stopRequested == true )
    {
        gIperfState.state = IPERF_UDP_RX_DRAIN_STATE;
        return;
    }

}


static void StateMachineUdpRxDrain(void)
{
    if( UDPIsGetReady(gIperfState.udpSock) > (uint8_t)0 )
    {
         UDPDiscard(gIperfState.udpSock);
         return;
    }

   /* If iperf kill was done, just jump to closing the socket */
   if ( gIperfState.stopRequested )
     gIperfState.state = IPERF_RX_DONE_STATE;
   else /* go ahead an generate reports, etc */
     gIperfState.state = IPERF_UDP_RX_DONE_STATE;

}


static void StateMachineUdpRxDone(void)
{
    tIperfPktInfo *pPktInfo;
    tServerHdr *pServer_hdr;
    uint16_t  tmp;
    float tmp2;


    if ( gIperfState.statusReported == 0u )
    {
        ReportBW_Jitter_Loss(SESSION_REPORT);
        gIperfState.statusReported = 1;
    }

    /* Drain any waiting pkts */
    if (  UDPIsGetReady(gIperfState.udpSock)  )
    {
        UDPDiscard(gIperfState.udpSock);
        return;
    }

    // Send the iperf UDP "FIN-ACK" 10 times.
    if ( gIperfState.nAttempts++ > 10u )
    {
        gIperfState.state = IPERF_RX_DONE_STATE;
        return;
    }

     /* Make sure space is available to TX the ACK packet of 128 bytes */
    if ( UDPIsTxPutReady(gIperfState.udpSock, 128 ) > 0u )
    {

      pPktInfo = (tIperfPktInfo *) g_bfr;
      pServer_hdr = (tServerHdr *) (pPktInfo +1);

      pPktInfo->id = TCPIP_HELPER_htonl( -gIperfState.lastPktId );
      pPktInfo->tv_sec = 0;
      pPktInfo->tv_usec = 0;

      pServer_hdr->flags = TCPIP_HELPER_htonl(HEADER_VERSION1);
      pServer_hdr->total_len1 = 0;
      pServer_hdr->total_len2 = TCPIP_HELPER_htonl( (uint32_t) gIperfState.totalLen);

      pServer_hdr->stop_sec =  TCPIP_HELPER_htonl( (uint32_t) (gIperfState.stopTime - gIperfState.startTime)/SYS_TICK_TicksPerSecondGet());

      /* get the remainder of the ticks using modulus */
      tmp2 = ((gIperfState.stopTime - gIperfState.startTime)%SYS_TICK_TicksPerSecondGet());

      /* normalize  to uSecs */
      tmp2 =  tmp2*1000/SYS_TICK_TicksPerSecondGet(); /* Convert to mSec */
      tmp2 *= 1000;   /* 1000 uSecs per mSec */


      pServer_hdr->stop_usec = TCPIP_HELPER_htonl( (uint32_t) tmp2 );
      pServer_hdr->error_cnt = TCPIP_HELPER_htonl( (uint32_t)  gIperfState.errorCount);;
      pServer_hdr->outorder_cnt = TCPIP_HELPER_htonl( (uint32_t) gIperfState.outofOrder);
      pServer_hdr->datagrams = TCPIP_HELPER_htonl( (uint32_t) gIperfState.lastPktId);
      pServer_hdr->jitter1 = 0;
      pServer_hdr->jitter2 = 0;

      UDPPutArray(gIperfState.udpSock, (uint8_t*)g_bfr, MAX_BUFFER);

      for ( tmp=0; tmp < (128 - MAX_BUFFER); tmp++)
        UDPPut(gIperfState.udpSock, 0x00);

      UDPFlush(gIperfState.udpSock );

    }

}

static void StateMachineTcpRxDone(void)
{
    if ( gIperfState.statusReported == 0u )
    {
        ReportBW_Jitter_Loss(SESSION_REPORT);
        gIperfState.statusReported = 1;
    }

    gIperfState.state = IPERF_RX_DONE_STATE;
}

static void StateMachineRxDone(void)
{
    const void* cmdIoParam = gIperfState.pCmdIO->cmdIoParam;

   if ( gIperfState.mProtocol == UDP_PROTOCOL )
   {
     UDPClose(  gIperfState.udpSock );
   }
   else
   {

     TCPClose( gIperfState.tcpServerSock );
	 gIperfState.tcpServerSock = INVALID_SOCKET;
   }

    (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "\r\niperf: Rx done. Socket closed.\r\n");

    // Clear statistics
    ResetIperfCounters();

    // In server mode, continue to accept new session requests ...

    if ((gIperfState.mServerMode == true) 	&&
        (gIperfState.stopRequested == false) )
    {
        (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: Ready for the next session.\r\n");

        gIperfState.state = IPERF_RX_START_STATE;
    }
    else
    {
        (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: completed.\r\n");
        gIperfState.state = IPERF_STANDBY_STATE;
    }

}




/******************************/
/* TX CLIENT CODE BEGINS HERE */
/******************************/


static void StateMachineTxStart(void)
{

   ARPResolve(gIperfState.pNetIf, &gIperfState.remoteSide.remoteIPaddress.v4Add);
   gIperfState.state = IPERF_TX_ARP_RESOLVE_STATE;
   gIperfState.timer = SYS_TICK_Get();

}

static void StateMachineTxArpResolve(void)
{
    const void* cmdIoParam = gIperfState.pCmdIO->cmdIoParam;

  if ( gIperfState.stopRequested == true )
  {
     (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: client session closed.\r\n");
     gIperfState.state = IPERF_STANDBY_STATE;
     return;
  }

  if(!ARPIsResolved(gIperfState.pNetIf, &gIperfState.remoteSide.remoteIPaddress.v4Add, &gIperfState.remoteMACAddr))
  {
      /* every 3 seconds print a warning */
      if( SYS_TICK_Get() - gIperfState.timer > 5*SYS_TICK_TicksPerSecondGet() )
      {
         (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: ARP unable to resolve the MAC address of remote side.\r\n");
         gIperfState.timer = SYS_TICK_Get();
      }
      return;
  }

  (gIperfState.pCmdIO->pCmdApi->print)(cmdIoParam, "    - RemoteNode MAC: %x %x %x %x %x %x\r\n",
           gIperfState.remoteMACAddr.v[0],
           gIperfState.remoteMACAddr.v[1],
           gIperfState.remoteMACAddr.v[2],
           gIperfState.remoteMACAddr.v[3],
           gIperfState.remoteMACAddr.v[4],
           gIperfState.remoteMACAddr.v[5]);
  
  if ( gIperfState.mProtocol == UDP_PROTOCOL )
     gIperfState.state = IPERF_UDP_TX_OPEN_STATE;
  else
     gIperfState.state = IPERF_TCP_TX_OPEN_STATE;

}

static void StateMachineUDPTxOpen(void)
{	
	UDP_SOCKET_INFO UdpSkt;
    const void* cmdIoParam = gIperfState.pCmdIO->cmdIoParam;
	
    if ( (gIperfState.udpSock = UDPOpenClient(IP_ADDRESS_TYPE_IPV4, gIperfState.mServerPort, (IP_MULTI_ADDRESS*)&gIperfState.remoteSide.remoteIPaddress.v4Add)) == INVALID_UDP_SOCKET )
    {
        /* error case */
        (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: Create UDP socket failed\r\n");
        gIperfState.state = IPERF_STANDBY_STATE;
        return;
    }

	
    UDPSocketSetNet(gIperfState.udpSock, gIperfState.pNetIf);
	UDPGetSocketInfo(gIperfState.udpSock, &UdpSkt);
    gIperfState.localPort = UdpSkt.localPort;

    (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "---------------------------------------------------------\r\n");
    (gIperfState.pCmdIO->pCmdApi->print)(cmdIoParam,  "iperf: Client connecting to %u.%u.%u.%u, UDP port %u\r\n",
              gIperfState.remoteSide.remoteIPaddress.v4Add.v[0],
              gIperfState.remoteSide.remoteIPaddress.v4Add.v[1],
              gIperfState.remoteSide.remoteIPaddress.v4Add.v[2],
              gIperfState.remoteSide.remoteIPaddress.v4Add.v[3],
              gIperfState.mServerPort );
    gIperfState.state = IPERF_UDP_TX_DATAGRAM_STATE;

    gIperfState.startTime = SYS_TICK_Get();

     // Wait for a few seconds before first TCP tx, so we can resolve ARP.
    gIperfState.nextTxTime = gIperfState.startTime + gIperfState.mPktPeriod;

}

static void StateMachineTCPTxOpen(void)
{
    const void* cmdIoParam = gIperfState.pCmdIO->cmdIoParam;

   if  ( (gIperfState.tcpClientSock = TCPOpenClient(IP_ADDRESS_TYPE_IPV4, gIperfState.mServerPort, &gIperfState.remoteSide.remoteIPaddress)) == INVALID_SOCKET )
   {
       /* error case */
        (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: Create TCP socket failed\r\n");
        gIperfState.state = IPERF_STANDBY_STATE;
        return;
    }

   TCPSocketSetNet(gIperfState.tcpClientSock, gIperfState.pNetIf);
    gIperfState.localPort = IPERF_TCP_LOCAL_PORT_START_NUMBER;

    (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "---------------------------------------------------------\r\n");
    (gIperfState.pCmdIO->pCmdApi->print)(cmdIoParam, "iperf: Client connecting to %u.%u.%u.%u, TCP port %u\r\n",
              gIperfState.remoteSide.remoteIPaddress.v4Add.v[0],
              gIperfState.remoteSide.remoteIPaddress.v4Add.v[1],
              gIperfState.remoteSide.remoteIPaddress.v4Add.v[2],
              gIperfState.remoteSide.remoteIPaddress.v4Add.v[3],
              gIperfState.mServerPort );

    gIperfState.state =  IPERF_TCP_TX_CONNECT_STATE;

    if(!TCPSetOptions(gIperfState.tcpClientSock, TCP_OPTION_TX_BUFF, (void*)gIperfState.txBuffSize))
    {
        (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: Set of TX buffer size failed\r\n");
    }

    gIperfState.timer = SYS_TICK_Get();
}


static void StateMachineTCPTxConnect(void)
{

    const void* cmdIoParam = gIperfState.pCmdIO->cmdIoParam;
    if ( gIperfState.stopRequested == true )
    {
        gIperfState.state = IPERF_TCP_TX_DONE_STATE;
        return;
    }

    if( !TCPIsConnected(gIperfState.tcpClientSock) )
    {

        // Time out if too much time is spent in this state
        if(SYS_TICK_Get()- gIperfState.timer > 5*SYS_TICK_TicksPerSecondGet())
        {
            TCPClose(gIperfState.tcpClientSock);
            gIperfState.tcpClientSock = INVALID_SOCKET;
            (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: TCP Client connection timeout\r\n");
            gIperfState.state = IPERF_TCP_TX_DONE_STATE;
        }

        return;
    }

    /* reset the reset flag so we don't get a false positive */
    TCPWasReset(gIperfState.tcpClientSock);

    gIperfState.startTime = SYS_TICK_Get();
    gIperfState.nextTxTime = gIperfState.startTime + gIperfState.mPktPeriod;
    gIperfState.state = IPERF_TCP_TX_SEGMENT_STATE;
}



static void GenericTxHeaderPreparation(uint8_t *pData, bool isTheLastTransmit)
{
    tIperfPktInfo *pPktInfo = NULL;
    uint32_t currentTime;
    tClientHdr *pClientHdr = NULL;
    long tmp2;

    if ( gIperfState.pktId == 0 ) {
        // The first tx packet
    }

    switch ( gIperfState.mProtocol )
    {
        case TCP_PROTOCOL: // TCP

            // Required by iperf.
            pClientHdr = (tClientHdr *) pData;

            // We borrow the same tIperfPktInfo structure to embed
            // some useful (non-standard iperf) meta info.
            // Unfortunately, the order has to be reversed.

            pPktInfo = (tIperfPktInfo *) (pClientHdr + 1);

            break;

        case UDP_PROTOCOL: // UDP

            // Both are required by iperf.

            pPktInfo = (tIperfPktInfo *) pData;
            pClientHdr = (tClientHdr *) (pPktInfo + 1);

            break;
    }

    // Client header:
    // Needed for all UDP packets.
    // For TCP, only the first two segments need this info. However,
    // there seems to be no harm to put it to all segments though.

    pClientHdr->flags = TCPIP_HELPER_htonl( (uint32_t) 0);
    pClientHdr->numThreads = TCPIP_HELPER_htonl((uint32_t) 1);
    pClientHdr->mPort = TCPIP_HELPER_htonl((uint32_t) gIperfState.mServerPort);
    pClientHdr->bufferlen = TCPIP_HELPER_htonl( (uint32_t) 0);
    pClientHdr->mWinBand = TCPIP_HELPER_htonl(gIperfState.mTxRate);

    if ( gIperfState.mAmount != 0u )
    {
        pClientHdr->mAmount = TCPIP_HELPER_htonl(gIperfState.mAmount);
    }
    else
    {
        pClientHdr->mAmount = TCPIP_HELPER_htonl( - (long) (gIperfState.mDuration/10) );
    }

    // Additional info: needed for UDP only.
    // No harm to put it to all TCP segments though.

    if ( isTheLastTransmit == true )
    {
        // The last UDP tx packet. Some caveats:
        // 1. Iperf uses a negative Id for the last tx packet.
        // 2. Its id should not change during retransmit.

        pPktInfo->id = - ( (long) (gIperfState.pktId - gIperfState.nAttempts) );
    }
    else
    {
        pPktInfo->id = gIperfState.pktId;
    }

    pPktInfo->id = TCPIP_HELPER_htonl(pPktInfo->id);

    currentTime = SYS_TICK_Get();

    pPktInfo->tv_sec = TCPIP_HELPER_htonl(currentTime / SYS_TICK_TicksPerSecondGet());

    /* get the remainder of the ticks using modulus */
    tmp2 = ((gIperfState.stopTime - gIperfState.startTime)%SYS_TICK_TicksPerSecondGet());

    /* normalize  to uSecs */
    tmp2 =  tmp2*1000/SYS_TICK_TicksPerSecondGet(); /* Convert to mSec */
    tmp2 *= 1000;   /* 1000 uSecs per mSec */


    pPktInfo->tv_usec = TCPIP_HELPER_htonl( tmp2 );

    return;
}



static tIperfTxResult GenericTxStart(void)
{
    uint32_t currentTime;
    bool iperfKilled;
    const void* cmdIoParam = gIperfState.pCmdIO->cmdIoParam;

    currentTime = SYS_TICK_Get();

    if ( currentTime < (gIperfState.nextTxTime - TIMING_ERROR_MARGIN))
    {
        // Wait until we are scheduled to Tx.
        return IPERF_TX_WAIT;
    }

    iperfKilled = gIperfState.stopRequested;
    if ((iperfKilled == true) ||
        ((gIperfState.mDuration != 0u) &&
         (currentTime > (gIperfState.startTime + gIperfState.mDuration))) ||
        ((gIperfState.mAmount != 0u) &&
         (gIperfState.totalLen > gIperfState.mAmount)))
    {
        // Prepare to transmit the last packet.
        // Although the last packet needs to be retransmitted UDP_FIN_RETRANSMIT_COUNT times,
        // if we are in UDP mode.

         gIperfState.isLastTransmit = true;
    }

    if ( gIperfState.pktId == 0 )
    {
        // The first pkt is going out ...

        // Reset startTime, to get a more accurate report.

        gIperfState.startTime = currentTime;
        gIperfState.nextTxTime = gIperfState.startTime;

        gIperfState.lastCheckTime = 	gIperfState.startTime;

        gIperfState.lastCheckPktId = gIperfState.pktId;
        gIperfState.lastCheckPktCount = gIperfState.pktCount;
        gIperfState.lastCheckErrorCount = gIperfState.errorCount;
        gIperfState.nAttempts = 0;
    }

    if ( gIperfState.mProtocol == TCP_PROTOCOL )
    {
       /* Manage socket */
       if( TCPIsGetReady(gIperfState.tcpClientSock) > 0u )
       {
          TCPDiscard(gIperfState.tcpClientSock);
          return IPERF_TX_WAIT;
       }

       if ( TCPWasReset(gIperfState.tcpClientSock) )
       {
         // We don't close the socket. We wait for user to "kill iperf" explicitly.
         (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "\r\niperf: Warning, TCP server disconnect detected\r\n");
       }

       if  (( TCPIsPutReady(gIperfState.tcpClientSock) <= gIperfState.mMSS ) && (!iperfKilled))
          return IPERF_TX_WAIT;

    }
    else
    {
       /* Manage socket */
       if( UDPIsGetReady(gIperfState.udpSock) > 0u )
       {
          UDPDiscard(gIperfState.udpSock);
          return IPERF_TX_WAIT;
       }

       if ( UDPIsTxPutReady(gIperfState.udpSock, gIperfState.mDatagramSize) < gIperfState.mDatagramSize )
       {
          if(gIperfState.txWaitTick == 0)
          {     // beginning wait period
              gIperfState.txWaitTick = SYS_TICK_Get() + ((IPERF_TX_WAIT_TMO * SYS_TICK_ResolutionGet()) + 999)/1000;
              return IPERF_TX_WAIT;
          }
          else if(SYS_TICK_Get() < gIperfState.txWaitTick)
          { // wait some more
              return IPERF_TX_WAIT;
          }
          
          // TX ready timeout
          (gIperfState.pCmdIO->pCmdApi->print)(cmdIoParam, "iperf: Failed to get %d bytes socket TX space\r\n", gIperfState.mDatagramSize);
          return IPERF_TX_FAIL;
       }
       else
       {    // reset retry tick counter
          gIperfState.txWaitTick = 0;
       }

    }


    // One Tx per mPktPeriod msec.
    gIperfState.nextTxTime = currentTime + gIperfState.mPktPeriod;

    GenericTxHeaderPreparation(g_bfr, gIperfState.isLastTransmit);

    if ( gIperfState.mProtocol == UDP_PROTOCOL )
    {

        gIperfState.remainingTxData = (gIperfState.mDatagramSize - MAX_BUFFER);

        if ( UDPPutArray(gIperfState.udpSock, g_bfr, MAX_BUFFER) != MAX_BUFFER )
        {
            (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: Socket send failed\r\n");
            gIperfState.errorCount++;
            return IPERF_TX_FAIL;
        }

    }
    else
    {

        gIperfState.remainingTxData = (gIperfState.mMSS - MAX_BUFFER);

        if (( TCPPutArray(gIperfState.tcpClientSock, (uint8_t*) g_bfr, MAX_BUFFER) != MAX_BUFFER ) && (!iperfKilled))
        {
            (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: Socket send failed\r\n");
            gIperfState.errorCount++;
            return IPERF_TX_FAIL;
        }

    }

     return IPERF_TX_OK;

}


/* This routine does a piecewise send, because the entire RAM buffer may not be available for putArray */
static void TcpTxFillSegment(void)
{
  uint16_t chunk;

  /* Fill the buffer with ASCII char T */
  memset( g_bfr, 0x54, MAX_BUFFER);

  while( gIperfState.remainingTxData > 0u )
  {
    chunk = MAX_BUFFER;

    /* finish case where we get more than is needed */
    if ( gIperfState.remainingTxData < MAX_BUFFER )
      chunk = gIperfState.remainingTxData;

    gIperfState.remainingTxData -= chunk;

    if ( TCPPutArray( gIperfState.tcpClientSock, (uint8_t *) g_bfr, chunk) != chunk )
      return;

  }

}

/* This routine does a piece wis send, because the entire RAM pkt buffer may not be available for putArray */
static void UdpTxFillDatagram(void)
{

  uint16_t chunk;

  /* Fill the buffer with ASCII char U */
  memset( g_bfr, 0x55, MAX_BUFFER);

  while( gIperfState.remainingTxData > 0u )
  {
    chunk = MAX_BUFFER;

    /* finish case where we get more than is needed */
    if ( gIperfState.remainingTxData < MAX_BUFFER )
      chunk = gIperfState.remainingTxData;

    gIperfState.remainingTxData -= chunk;

    if (  UDPPutArray(gIperfState.udpSock, (uint8_t *) g_bfr, chunk) != chunk )
     return;

  }

}



static void GenericTxEnd(void)
{
    const void* cmdIoParam = gIperfState.pCmdIO->cmdIoParam;

    if(  gIperfState.remainingTxData  > 0u )
    {
        /* unhandled error */
        (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: Socket send failed\r\n");
        gIperfState.errorCount++;
    }
    else
    {
        // send successful.

        if ( gIperfState.pktCount == 0u )
        {
            // first tx pkt

            IPV4_ADDR lclAddress;

            (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "\n\riperf: Session started ...\r\n");

            lclAddress.Val = TCPIP_STACK_NetAddress(gIperfState.pNetIf);

            (gIperfState.pCmdIO->pCmdApi->print)(cmdIoParam, "    - Local  %u.%u.%u.%u port %u connected with\r\n",
                    lclAddress.v[0],
                    lclAddress.v[1],
                    lclAddress.v[2],
                    lclAddress.v[3],
                    gIperfState.localPort);

            (gIperfState.pCmdIO->pCmdApi->print)(cmdIoParam, "    - Remote %u.%u.%u.%u port %u\r\n",
                    gIperfState.remoteSide.remoteIPaddress.v4Add.v[0],
                    gIperfState.remoteSide.remoteIPaddress.v4Add.v[1],
                    gIperfState.remoteSide.remoteIPaddress.v4Add.v[2],
                    gIperfState.remoteSide.remoteIPaddress.v4Add.v[3],
                    gIperfState.mServerPort );

            (gIperfState.pCmdIO->pCmdApi->print)(cmdIoParam, "    - Target rate = %ld bps, period = %ld ms\r\n",
                    (unsigned long)gIperfState.mTxRate, 
                    (unsigned long)(gIperfState.mPktPeriod*1000/SYS_TICK_TicksPerSecondGet()) );

        }

        gIperfState.pktId++;
        gIperfState.pktCount++;

        if ( gIperfState.mProtocol == UDP_PROTOCOL )
        {
            gIperfState.totalLen += gIperfState.mDatagramSize;
        }
        else
        {

            gIperfState.totalLen += gIperfState.mMSS;
        }


    }

    gIperfState.lastPktId = gIperfState.pktId - 1;



    if ( SYS_TICK_Get() > (gIperfState.lastCheckTime + gIperfState.mInterval - TIMING_ERROR_MARGIN) )
    {
        // Time to report statistics
        ReportBW_Jitter_Loss(INTERVAL_REPORT);

        //gIperfState.lastCheckPktCount = gIperfState.pktCount;
    }


    if ( gIperfState.isLastTransmit == true )
    {
        if ((gIperfState.mProtocol == (tIperfProto)UDP_PROTOCOL) &&
                (++gIperfState.nAttempts < UDP_FIN_RETRANSMIT_COUNT) ) {

            if ( gIperfState.nAttempts == 1u )
            {
                // So the normal pkt statistics is not mixed with the retransmited last pkt.
                gIperfState.stopTime = SYS_TICK_Get();

                ReportBW_Jitter_Loss(SUBTOTAL_REPORT);
                (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "    -----------------------------------------\r\n"); 
            }

            // Don't follow the same transmision rate during retransmit.
            gIperfState.mPktPeriod = UDP_FIN_RETRANSMIT_PERIOD;
        }
        else
        {
            gIperfState.state = (gIperfState.mProtocol == UDP_PROTOCOL) ? IPERF_UDP_TX_DONE_STATE : IPERF_TCP_TX_DONE_STATE;
            gIperfState.stopTime = SYS_TICK_Get();
        }
    }

}

static void StateMachineTcpTxSegment(void)
{
    tIperfTxResult txRes = GenericTxStart();

    if(txRes == IPERF_TX_OK)
    {   // go ahead and transmit
       TcpTxFillSegment();
       TCPFlush(gIperfState.tcpClientSock);
       GenericTxEnd();
    }
    else if(txRes == IPERF_TX_FAIL)
    {
        gIperfState.state = IPERF_TCP_TX_DONE_STATE;
        gIperfState.stopTime = SYS_TICK_Get();
    }
    // else wait...
}

static void StateMachineUdpTxDatagram(void)
{
    tIperfTxResult txRes = GenericTxStart();

    if ( txRes == IPERF_TX_OK )
    {   // go ahead and transmit
       UdpTxFillDatagram();
       UDPFlush(gIperfState.udpSock);
       GenericTxEnd();
    }
    else if(txRes == IPERF_TX_FAIL)
    {
        gIperfState.state = IPERF_UDP_TX_DONE_STATE;
        gIperfState.stopTime = SYS_TICK_Get();
    }
    // else wait...
}


static void GenericTxDone(void)
{
    const void* cmdIoParam = gIperfState.pCmdIO->cmdIoParam;
    if ( gIperfState.statusReported == 0u )
    {
        ReportBW_Jitter_Loss(SESSION_REPORT);
        gIperfState.statusReported = 1;
    }

    gIperfState.state = IPERF_STANDBY_STATE;

    (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: Tx done. Socket closed.\r\n");

    // Clear statistics
    ResetIperfCounters();

    //(gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "    Back to standby mode.\r\n");
    (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: completed.\r\n");

}

static void StateMachineTcpTxDone(void)
{
    GenericTxDone();

    // close gracefully
    TCPClose(gIperfState.tcpClientSock);
    gIperfState.tcpClientSock = INVALID_SOCKET;
}

static void StateMachineUdpTxDone(void)
{

    GenericTxDone();

    UDPClose(gIperfState.udpSock );
}

static int CommandIperfStart(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv)
{
    uint8_t i;
    char *ptr;
    uint32_t values[4];
    uint16_t payloadSize;
    float pktRate;
    const void* cmdIoParam = pCmdIO->cmdIoParam;

    // Check if iperf is still running
    if ( gIperfState.state != (uint8_t)IPERF_STANDBY_STATE )
    {
            (*pCmdIO->pCmdApi->msg)(cmdIoParam, "\r\niperf: session already started\r\n");
            return false;
    }

	// preparation for new iperf test
    gIperfState.mServerMode = false;
    gIperfState.mProtocol = TCP_PROTOCOL;   			// default is TCP mode.
    gIperfState.stopRequested = false;

    gIperfState.mServerPort = IPERF_SERVER_PORT;		// -p. default: server port 5001

    gIperfState.mTxRate = ((uint32_t) 500)*((uint32_t) 1000);		// -b or -x. Target tx rate.
    // KS: default tx rate for iperf is actually 1Mbps. Here we set it to 500Kbps instead.

    gIperfState.mAmount = 0;			// -n: default 0.
    gIperfState.mDuration = ((uint32_t) 10)*((uint32_t) SYS_TICK_TicksPerSecondGet()); // -t: default 10 sec.
    gIperfState.mInterval =  SYS_TICK_TicksPerSecondGet(); 	// -i: default 1 sec.

    // remember the console we've been invoked from
    gIperfState.pCmdIO = pCmdIO;

    
    // Initialize statistics

    ResetIperfCounters();

    for (i = 1; i < argc; i++)
    {
        if ((memcmp(argv[i], "-s", 2) == 0) || (memcmp(argv[i], "--server", 5) == 0) )
        {
            // Function as an iperf server.

            gIperfState.mServerMode = true;
        }
        else if ((memcmp(argv[i], "-u", 2) == 0) || (memcmp(argv[i], "--udp", 5) == 0) )
        {
            // iperf UDP mode.
            gIperfState.mProtocol = UDP_PROTOCOL;
        }
        else if ((memcmp(argv[i], "-b", 2) == 0) || (memcmp(argv[i], "--bandwidth", 5) == 0) )
        {
            // iperf UDP mode.
            gIperfState.mProtocol = UDP_PROTOCOL;

            // Next argument should be the target rate, in bps.
            i++;
            ptr = argv[i];

            ascii_to_u32s(ptr, values, 1);

            gIperfState.mTxRate = values[0];
        }
        else if ((memcmp(argv[i], "-x", 2) == 0) || (memcmp(argv[i], "--xmitrate", 5) == 0) )
        {
            // NON-STANDARD IPERF OPTION. Set the max TCP tx rate.
            // Next argument should be the target rate, in bps.
            i++;
            ptr = argv[i];

            ascii_to_u32s(ptr, values, 1);

            gIperfState.mTxRate = values[0];
        }
        else if ((memcmp(argv[i], "-c", 2) == 0) || (memcmp(argv[i], "--client", 5) == 0) )
        {
            // Function as an iperf client.
            gIperfState.mServerMode = false;

            // Next argument should be the server IP, such as "192.168.1.100".
            i++;
            ptr = argv[i];
            ascii_to_u32s(ptr, values, 4);

            gIperfState.remoteSide.remoteIPaddress.v4Add.v[0] = values[0];
            gIperfState.remoteSide.remoteIPaddress.v4Add.v[1] = values[1];
            gIperfState.remoteSide.remoteIPaddress.v4Add.v[2] = values[2];
            gIperfState.remoteSide.remoteIPaddress.v4Add.v[3] = values[3]; 
        }
        else if ((memcmp(argv[i], "-t", 2) == 0) || (memcmp(argv[i], "--time", 5) == 0) )
        {
            // Next argument should be the (client tx) duration, in seconds.
            i++;
            ptr = argv[i];
            ascii_to_u32s(ptr, values, 1);

            gIperfState.mDuration = values[0]*SYS_TICK_TicksPerSecondGet();
            gIperfState.mAmount = 0;
        }
        else if ((memcmp(argv[i], "-n", 2) == 0) || (memcmp(argv[i], "--num", 5) == 0) )
        {
            // Next argument should be the (client tx) size, in bytes.
            i++;
            ptr = argv[i];
            ascii_to_u32s(ptr, values, 1);

            gIperfState.mAmount = values[0];
            gIperfState.mDuration = 0;
        }


        else if ((memcmp(argv[i], "-M", 2) == 0) ||
                (memcmp(argv[i], "--mss", 5) == 0) )
        {
            // Next argument should be the (client tcp tx) MSS size, in bytes.

            i++;
            ptr = argv[i];

            ascii_to_u32s(ptr, values, 1);

            gIperfState.mMSS = values[0];
        }

        else if ((memcmp(argv[i], "-i", 2) == 0) || (memcmp(argv[i], "--interval", 5) == 0) )
        {
            // Next argument should be the report interval, in seconds.
            i++;
            ptr = argv[i];
            ascii_to_u32s(ptr, values, 1);

            gIperfState.mInterval = values[0]*SYS_TICK_TicksPerSecondGet(); // Convert to msec
        }
        else if ((memcmp(argv[i], "-l", 2) == 0) || (memcmp(argv[i], "--len", 5) == 0) )
        {
            // Next argument should be the buffer length, in bytes.
            // This is used as the UDP datagram size.
            i++;
            ptr = argv[i];
            ascii_to_u32s(ptr, values, 1);

            if ( values[0] <  MAX_BUFFER  )
            {
               (gIperfState.pCmdIO->pCmdApi->print)(cmdIoParam, "iperf: The minimum datagram size is %d\r\n", (int)MAX_BUFFER);
               return false;
            }

            gIperfState.mDatagramSize = values[0];
        }
    }

    switch (gIperfState.mServerMode)
    {
        case 0:
            // iperf client

            // set the interface to work on
            if((gIperfState.pNetIf = _TCPIPStackIpAddToNet(&gIperfState.localAddr, false)) == 0)
            {
                (*pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: Using the default interface!\r\n");
                gIperfState.pNetIf = TCPIP_STACK_GetDefaultNet();
            }

            payloadSize = 	(gIperfState.mProtocol == UDP_PROTOCOL) ?
                            gIperfState.mDatagramSize : gIperfState.mMSS;

            pktRate =  (float) (gIperfState.mTxRate / 8) / (float) payloadSize;
            gIperfState.mPktPeriod =  (uint32_t) ( (float) SYS_TICK_TicksPerSecondGet() / pktRate );

            gIperfState.state = IPERF_TX_START_STATE;
            break;

        case 1:
            // iperf server
            (*pCmdIO->pCmdApi->msg)(cmdIoParam, "---------------------------------------------------------\r\n");

            (*pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: Server listening on ");
            if (gIperfState.mProtocol == UDP_PROTOCOL)
            {
                (*pCmdIO->pCmdApi->msg)(cmdIoParam, (const char *)"UDP");
            }    
            else
            {
                (*pCmdIO->pCmdApi->msg)(cmdIoParam, (const char *)"TCP");
            }    
            (gIperfState.pCmdIO->pCmdApi->print)(cmdIoParam, " port %d\r\n", gIperfState.mServerPort);
            gIperfState.state = IPERF_RX_START_STATE;
            break;
    }
    
    return true;
}

static int CommandIperfStop(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv)
{
    const void* cmdIoParam = pCmdIO->cmdIoParam;
    if (argc > 1)
    {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: too many args\r\n");
        return false;
    }

    if (gIperfState.state == IPERF_STANDBY_STATE)
    {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "\r\niperf: not started yet!\r\n");
        return false;
    }

    gIperfState.stopRequested = true;
    (*pCmdIO->pCmdApi->msg)(cmdIoParam, "\r\niperf: trying to stop...\r\n");

    return true;
}

static int CommandIperfNetIf(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv)
{
    IPV4_ADDR   ipAddr;
    const void* cmdIoParam = pCmdIO->cmdIoParam;

    if (argc != 2)
    {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "iperfi usage: iperfi address\r\n");
        return false;
    }

    if (gIperfState.state != IPERF_STANDBY_STATE)
    {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "\r\niperfi: cannot change the ip address while running!\r\n");
        return false;
    }


    // argument should be the IP address, such as "192.168.1.100".
    // use "0" for any interface (server mode only)
    if(!TCPIP_HELPER_StringToIPAddress(argv[1], &ipAddr))
    {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "iperfi: use a valid IP address!\r\n");
        return false;
    }

    gIperfState.localAddr.Val = ipAddr.Val;
    (*pCmdIO->pCmdApi->msg)(cmdIoParam, "iperfi: OK, set the IP address\r\n");

    return true;


}

static int CommandIperfSize(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv)
{
    bool        setTx;
    uint32_t    buffSize;

    const void* cmdIoParam = pCmdIO->cmdIoParam;


    if (argc != 3)
    {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "iperfs usage: iperfs tx/rx size\r\n");
        return false;
    }

    if(strcmp(argv[1], "tx") == 0)
    {
        setTx = true;
    }
    else if(strcmp(argv[1], "rx") == 0)
    {
        setTx = false;
    }
    else
    {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "iperfs: enter tx or rx\r\n");
        return false;
    }

    buffSize = atoi(argv[2]);
    if(buffSize <= 0 || buffSize >= 65536)
    {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "iperfs: 0 < size < 65536\r\n");
        return false;
    }

    if(setTx)
    {
        gIperfState.txBuffSize = buffSize;
    }
    else
    {
        gIperfState.rxBuffSize = buffSize;
    }

    (*pCmdIO->pCmdApi->print)(cmdIoParam, "iperfs: OK, set %s size to %d\r\n", argv[1], buffSize);

    return true;



}

#endif  // defined(TCPIP_STACK_USE_IPERF)

