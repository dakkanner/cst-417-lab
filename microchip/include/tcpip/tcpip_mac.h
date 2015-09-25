/*******************************************************************************
  MAC Module Defs for Microchip Stack

  Summary:
    
  Description:
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
FileName:  tcpip_mac.h 
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
//DOM-IGNORE-END

#ifndef __TCPIP_MAC_H_
#define __TCPIP_MAC_H_

#include <stdint.h>
#include <stdbool.h>

// *****************************************************************************
// *****************************************************************************
// Section: MAC interface definitions
// *****************************************************************************
// *****************************************************************************

// Structure to contain a MAC address
typedef struct __attribute__((__packed__))
{
    uint8_t v[6];
} MAC_ADDR;


// A generic structure representing the Ethernet header starting all Ethernet
// frames
typedef struct  __attribute__((aligned(2), packed))
{
	MAC_ADDR            DestMACAddr;
	MAC_ADDR            SourceMACAddr;
	TCPIP_UINT16_VAL    Type;
} MAC_ETHERNET_HEADER;
        
// MAC initialization data
// Each supported MAC has its own specific init/configuration data

typedef struct
{
    // add necessary initialization data here
}TCPIP_MODULE_MAC_ENCJ60_CONFIG;


typedef struct
{
    // add necessary initialization data here
}TCPIP_MODULE_MAC_ENCJ600_CONFIG;

typedef struct
{
    // add necessary initialization data here
}TCPIP_MODULE_MAC_97J60_CONFIG;

typedef struct
{
    int             txBuffSize;     // size of the corresponding TX buffer
    int             nTxDescriptors; // number of TX descriptors
    int             rxBuffSize;     // size of the corresponding RX buffer
    int             nRxDescriptors; // number of RX descriptors
}TCPIP_MODULE_MAC_PIC32INT_CONFIG;

typedef struct
{
    // add necessary initialization data here
}TCPIP_MODULE_MAC_MRF24W_CONFIG;


// Address structure for a node
typedef struct __attribute__((__packed__))
{
    IPV4_ADDR     IPAddr;
    MAC_ADDR    MACAddr;
} NODE_INFO;

typedef const void* TCPIP_MAC_HANDLE;     // handle to a MAC

// *****************************************************************************
/*  Received Packet Status

  Summary:
    Status of a received packet.

  Description:
    This structure contains the status of a received packet.
  
  Notes:
*/

typedef struct __attribute__ ((__packed__))
{

    // correct checksum filled in
    unsigned        chksumOk        : 1;
    // Packet payload checksum
    unsigned        pktChecksum     :16;

    // Runt packet received
    unsigned        runtPkt         : 1;

    // Unicast, not me packet,
    unsigned        notMeUcast      : 1;

    // Hash table match
    unsigned        htMatch         : 1;

    // Magic packet match
    unsigned        magicMatch      : 1;

    // Pattern match match
    unsigned        pmMatch         : 1;

    // Unicast match
    unsigned        uMatch          : 1;

    // Broadcast match
    unsigned        bMatch          : 1;

    // Multicast match
    unsigned        mMatch          : 1;

    // Received bytes
    unsigned        rxBytes         :16;

    // CRC error in packet
    unsigned        crcError        : 1;

    // Receive length check error
    unsigned        lenError        : 1;

    // Receive length out of range
    unsigned        lenRange        : 1;

    // Receive OK
    unsigned        rxOk            : 1;

    // Multicast packet
    unsigned        mcast           : 1;

    // Broadcast packet
    unsigned        bcast           : 1;

    // Control frame received
    unsigned        rxCtrl          : 1;

    // Received VLAN tagged frame
    unsigned        rxVLAN          : 1;

}MAC_RX_PKT_STAT;


// *****************************************************************************
/*  MAC Packet

  Summary:
    A TX/RX packet descriptor.

  Description:
    Structure of a buffer transferred with the MAC.
  
  Remarks:
    None
*/
typedef struct __attribute__ ((__packed__)) _tag_MAC_PACKET
{
    struct _tag_MAC_PACKET* next;           // multi-segment/multi-packet support
    size_t                  buffSize;       // available buffer size;
    size_t                  pktSize;        // actual packet size
    unsigned short          pktFlags;       // flags:
                                            //      rx checksum is supported
                                            //      pkt is internal/external
                                            //      pkt needs ack
                                            //      zero copy intended
                                            //      1st segment, last segment - for multi-segment/multi-packet support
                                            //      etc.
    unsigned short          pktChksum;      // rx - packet checksum, if supported
    MAC_ETHERNET_HEADER     header;         // standard Ethernet header
    uint32_t                buffer[0];      // payload
}MAC_PACKET;


// prototype of a MAC notification handler
// The MAC client can register a handler with the MAC
// Once an TX/RX packet is done, the MAC will call the registered notification handler
// The handler has to be short and fast.
// It is meant for setting an event flag, not for lengthy processing!
typedef void    (*MAC_NOTIFY_HANDLER)(TCPIP_MAC_HANDLE hMac,  MAC_PACKET* pkt,  const void* param);

// profile only
typedef struct
{
    int     nTotPkts;         // number of total packets supported
    int     nPendingPkts;     // number of pending packets
    size_t  nTotBytes;        // number of total bytes supported
    size_t  nAvlblBytes;      // number of available bytes
}TCPIP_MAC_PROFILE;        

typedef enum
{
    TCPIP_MAC_RES_OK                = 0,   // operation successful
    // benign operation results - positive codes
    TCPIP_MAC_RES_PENDING           = 1,    // operation is pending upon some hardware resource
                                            // call again to completion
    // error codes - negative
    TCPIP_MAC_RES_TYPE_ERR          = -1,   // unsupported type
    TCPIP_MAC_RES_IS_BUSY           = -2,   // device is in use
    TCPIP_MAC_RES_INIT_FAIL         = -3,   // generic initialization failure
    TCPIP_MAC_RES_PHY_INIT_FAIL     = -4,   // PHY initialization failure
    TCPIP_MAC_RES_EVENT_INIT_FAIL   = -5,   // Event system initialization failure
    TCPIP_MAC_RES_OP_ERR            = -6,   // unsupported operation
    TCPIP_MAC_RES_ALLOC_ERR         = -7,   // memory allocation error
    TCPIP_MAC_RES_INSTANCE_ERR      = -8,   // already instantiated, initialized error
    
    
}TCPIP_MAC_RES;         // list of return codes from MAC functions


/****************************
 *  Interface functions
 **************************************/


//TCPIP_MAC_RES       MACInitialize(TCPIP_MAC_ID macId,
//                                  const TCPIP_STACK_MODULE_GONFIG* const stackData,
//                                  const void* const moduleData );
// Moved to tcp_mac_private.h

// function to de-initialize a MAC
//TCPIP_MAC_RES       MACDeinitialize(TCPIP_MAC_ID macId );
// Moved to tcp_mac_private.h

// function to open a MAC and get a client handle
// TCPIP_MAC_HANDLE    MACOpen( TCPIP_MAC_ID macId );
// Moved to tcp_mac_private.h

// all subsequent MAC functions take the hMac as the 1st parameter 
// hMac is a handle to a MAC
// This way multiple instances of the same MAC could be created
// for platforms that support it
// Moved to tcp_mac_private.h

// closes a MAC client
TCPIP_MAC_RES       MACClose(TCPIP_MAC_HANDLE hMac);

// transfer interface

// TX - functions
TCPIP_MAC_RES       MACTxPacket(TCPIP_MAC_HANDLE hMac, MAC_PACKET * ptrPacket);

// TX ready
// Note: no real need for this, MACTxPacket() will tell if busy or not
// Could be misleading!!!
bool                MACIsTxReady(TCPIP_MAC_HANDLE hMac);

// registers the TX done notification to be called
// Note: single client only!!!
TCPIP_MAC_RES       MACTxRegisterNotifyHandler(TCPIP_MAC_HANDLE hMac, MAC_NOTIFY_HANDLER pNotify, void* param);

// fills a profile struct with data
TCPIP_MAC_RES       MACTxProfile(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_PROFILE* pProf);


// RX - functions
//

// returns a pending RX packet if exists
TCPIP_MAC_RES      MACRxGetPacket (MAC_PACKET* pkt, const MAC_RX_PKT_STAT* pPktStat);


// acknowledge a RX packet
// if pkt == NULL, then all the pending RX buffers will be acknowledged
// (and discarded) (and possibly re-inserted in the MACs RX queue - if supported).
TCPIP_MAC_RES       MACRxAcknowledge(TCPIP_MAC_HANDLE hMac, MAC_PACKET* pkt);

// registers the RX  notification to be called
TCPIP_MAC_RES       MACRxRegisterNotifyHandler(TCPIP_MAC_HANDLE hMac, MAC_NOTIFY_HANDLER pNotify, void* param);

// RX ready -  at least one packet available
// Note: no real need for this, MACRxGetPacketDcpt() will tell if available or not
bool               MACIsRxReady(TCPIP_MAC_HANDLE hMac);

// fills a profile struct with data
TCPIP_MAC_RES       MACRxProfile(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_PROFILE* pProf);

/****************************
 *  Link Interface functions
 **************************************/

bool                MACCheckLink(TCPIP_MAC_HANDLE hMac);


/****************************
 *  RX Filters Interface functions
 **************************************/
/* MAC Receive Filter Flags
   Multiple values can be OR-ed together
   */

typedef enum
{
    // Frames with wrong CRC are accepted
    TCPIP_MAC_FILT_CRC_ERR_ACCEPT,

    // Runt frames accepted
    TCPIP_MAC_FILT_RUNT_ACCEPT,

    // Frames with wrong CRC are rejected
    TCPIP_MAC_FILT_CRC_ERR_REJECT,

    // Runt frames rejected
    TCPIP_MAC_FILT_RUNT_REJECT,

    // Me unicast accepted
    TCPIP_MAC_FILT_ME_UCAST_ACCEPT,

    // Not me unicast accepted
    TCPIP_MAC_FILT_NOTME_UCAST_ACCEPT,

    // Multicast accepted
    TCPIP_MAC_FILT_MCAST_ACCEPT,

    // Broadcast accepted
    TCPIP_MAC_FILT_BCAST_ACCEPT,

    // Hash table matches destination address accepted
    TCPIP_MAC_FILT_HTBL_ACCEPT,

    // Magic packet accepted
    TCPIP_MAC_FILT_MAGICP_ACCEPT,


    // All Filters
    TCPIP_MAC_FILT_ALL_FILTERS = TCPIP_MAC_FILT_CRC_ERR_ACCEPT  | TCPIP_MAC_FILT_RUNT_ACCEPT        | 
        TCPIP_MAC_FILT_CRC_ERR_REJECT  | TCPIP_MAC_FILT_RUNT_REJECT        |
        TCPIP_MAC_FILT_ME_UCAST_ACCEPT | TCPIP_MAC_FILT_NOTME_UCAST_ACCEPT |
        TCPIP_MAC_FILT_MCAST_ACCEPT    | TCPIP_MAC_FILT_BCAST_ACCEPT       |
        TCPIP_MAC_FILT_HTBL_ACCEPT     | TCPIP_MAC_FILT_MAGICP_ACCEPT

} TCPIP_MAC_FILTERS;


typedef enum
{
    // Simple Pattern Match accepted
    TCPIP_MAC_FILT_PMATCH_ACCEPT = 1,

    // Pattern Match AND destination==me
    TCPIP_MAC_FILT_PMATCH_ME_UCAST_ACCEPT,

    // Pattern Match AND destination!=me
    TCPIP_MAC_FILT_PMATCH_NOTME_UCAST_ACCEPT,

    // Pattern Match AND destination!=unicast
    TCPIP_MAC_FILT_PMATCH_MCAST_ACCEPT,

    // Pattern Match AND destination==unicast
    TCPIP_MAC_FILT_PMATCH_NOT_MCAST_ACCEPT,

    // Pattern Match AND destination==broadcast
    TCPIP_MAC_FILT_PMATCH_BCAST_ACCEPT,

    // Pattern Match AND destination!=broadcast
    TCPIP_MAC_FILT_PMATCH_NOT_BCAST_ACCEPT,

    // Pattern Match AND hash table filter match (irrespective of the 
    // TCPIP_MAC_FILT_HTBL_ACCEPT setting)
    TCPIP_MAC_FILT_PMATCH_HTBL_ACCEPT,

    // Pattern Match AND packet ==magic packet
    TCPIP_MAC_FILT_PMATCH_MAGICP_ACCEPT,

    // If set, the pattern must NOT match for a successful Pattern Match to 
    // occur!
    TCPIP_MAC_FILT_PMATCH_INVERT = 0x80000000

} TCPIP_MAC_PAT_MATCH;

typedef enum
{
    TCPIP_MAC_FILT_NOP,     // no operation
    TCPIP_MAC_FILT_SET,     // set filters
    TCPIP_MAC_FILT_CLR,     // clear filters
    TCPIP_MAC_FILT_WR,      // write filters
}TCPIP_MAC_FILT_OP;     // filter operation

// perform a filter operation
TCPIP_MAC_RES        MACRxFilterOperation (TCPIP_MAC_HANDLE hMac,  TCPIP_MAC_FILT_OP op, TCPIP_MAC_FILTERS rxFilters );

// set the hash table filter
TCPIP_MAC_RES        MACRxFilterSetHashTableEntry(TCPIP_MAC_HANDLE hMac, MAC_ADDR* DestMACAddr);

// set the Pattern Match filter
TCPIP_MAC_RES        MACRxFilterPatternMode (TCPIP_MAC_HANDLE hMac, TCPIP_MAC_FILT_OP op, TCPIP_MAC_PAT_MATCH mode,
        unsigned long long matchMask, unsigned int matchOffs, unsigned int matchChecksum );

    
/****************************
 *  Flow Control Functions
 **************************************/

typedef enum
{
    // Auto flow control
    TCPIP_MAC_FC_AUTO = 0,

    // Software flow control
    TCPIP_MAC_FC_SOFTWARE

} TCPIP_MAC_FC_TYPE;


//    This function sets the pause value to be used with manual or auto flow 
//    control.
TCPIP_MAC_RES        MACFCSetPauseValue (TCPIP_MAC_HANDLE hMac, unsigned int pauseBytes );


// Enables or disables selected flow control.
TCPIP_MAC_RES       MACFCEnable ( TCPIP_MAC_HANDLE hMac, TCPIP_MAC_FC_TYPE fcType, bool enable );


// sets the values for the watermarks used in the automatic flow
TCPIP_MAC_RES       MACFCSetRxWMark ( TCPIP_MAC_HANDLE hMac, int fullWM, int emptyWM );

/****************************
 *  Misc Control functions
 **************************************/


// supported MAC power mode state
typedef enum
{
    TCPIP_MAC_POWER_NONE,     // unknown power mode; 
    TCPIP_MAC_POWER_FULL,     // up and running; valid for init/re-init
    TCPIP_MAC_POWER_LOW,      // low power mode; valid for init/re-init
    TCPIP_MAC_POWER_DOWN,     // interface is down; 
}TCPIP_MAC_POWER_MODE;


uint16_t            MACCalcRxChecksum(TCPIP_MAC_HANDLE hMac, uint16_t offset, uint16_t len);
uint16_t            MACCalcIPBufferChecksum(TCPIP_MAC_HANDLE hMac, uint16_t len); 

void                MACSetRXHashTableEntry(TCPIP_MAC_HANDLE hMac, MAC_ADDR DestMACAddr); 

bool 	            MACPowerMode(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_POWER_MODE pwrMode);


/****************************
 *  Event Notification functions
 **************************************/

#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
#include "tcpip/tcpip_mac_events.h"

TCPIP_MAC_EVENT_RESULT    MACEventSetNotifyEvents(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents);
TCPIP_MAC_EVENT_RESULT    MACEventClearNotifyEvents(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents);
TCPIP_MAC_EVENT_RESULT    MACEventAck(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents);
TCPIP_EVENT               MACEventGetPending(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup);
TCPIP_MAC_EVENT_RESULT    MACEventSetNotifyHandler(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, pMacEventF eventHandler, void* hParam);
#endif  // (TCPIP_STACK_USE_EVENT_NOTIFICATION)


/****************************
 *  Functions for MACs that support on chip memory
 **************************************/

#if defined(__PIC32MX__)
	typedef uint32_t    TCPIP_MAC_PTR_TYPE;
#elif defined(__C30__)
	typedef uint16_t    TCPIP_MAC_PTR_TYPE;
#endif


// Read functions
int                 MACGetHeader(TCPIP_MAC_HANDLE hMac, MAC_ADDR *remote, uint16_t* type);
void                MACSetReadPtrInRx(TCPIP_MAC_HANDLE hMac, uint16_t offset);
TCPIP_MAC_PTR_TYPE  MACGetReadPtrInRx(TCPIP_MAC_HANDLE hMac);
TCPIP_MAC_PTR_TYPE  MACSetReadPtr(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_PTR_TYPE address);
TCPIP_MAC_PTR_TYPE  MACSetBaseReadPtr(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_PTR_TYPE address);
uint16_t            MACGetArray(TCPIP_MAC_HANDLE hMac, uint8_t *val, uint16_t len);
void                MACDiscardRx(TCPIP_MAC_HANDLE hMac);

// Write functions

TCPIP_MAC_PTR_TYPE  MACSetWritePtr(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_PTR_TYPE address);
void                MACPutHeader(TCPIP_MAC_HANDLE hMac, MAC_ADDR *remote, uint16_t type, uint16_t dataLen);
void                MACPutArray(TCPIP_MAC_HANDLE hMac, uint8_t *val, uint16_t len);
void                MACFlush(TCPIP_MAC_HANDLE hMac);

TCPIP_MAC_PTR_TYPE  MACGetTxBaseAddr(TCPIP_MAC_HANDLE hMac);    



	
#endif // __TCPIP_MAC_H_
