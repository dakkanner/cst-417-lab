/*******************************************************************************
  IPv6 Defs for Microchip TCP/IP Stack

  Summary:
    
  Description:
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
FileName:  ipv6.h 
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

#ifndef __IPV6_H_
#define __IPV6_H_


// *****************************************************************************
// *****************************************************************************
// Section: public definitions
// *****************************************************************************
// *****************************************************************************

#include "link_list.h"

// *****************************************************************************
/* Defines a list of next header types

  Summary:
    None

  Description:
    None

  Remarks:
    None
 */
typedef enum
{
    IPV6_PROT_HOP_BY_HOP_OPTIONS_HEADER = (0u), // IPv6 Hop-by-Hop Opt. Header
    IPV6_PROT_ICMP = (1u),
    IPV6_PROT_TCP = (6u),
    IPV6_PROT_UDP = (17u),
    IPV6_PROT_IPV6 = (41u), // IPv6 Protocol
    IPV6_PROT_ROUTING_HEADER = (43u), // IPv6 Routing Header
    IPV6_PROT_FRAGMENTATION_HEADER = (44u), // IPv6 Fragmentation Header
    IPV6_PROT_ENCAPSULATING_SECURITY_PAYLOAD_HEADER = (50u), // Encapsulating Security Payload Header
    IPV6_PROT_AUTHENTICATION_HEADER = (51u), // Authentication Header
    IPV6_PROT_ICMPV6 = (58u), // ICMPv6 Protocol
    IPV6_PROT_NONE = (59u), // No next header
    IPV6_PROT_DESTINATION_OPTIONS_HEADER = (60u) // Destination Options Header
} IPV6_NEXT_HEADER_TYPE;

#define IP_VERSION_6    (1u)
#define IP_VERSION_4    (0u)

typedef enum {
    IPV6_ACTION_NONE = 0,
    IPV6_ACTION_DISCARD_SILENT,
    IPV6_ACTION_DISCARD_PP_0,
    IPV6_ACTION_DISCARD_PP_2,
    IPV6_ACTION_DISCARD_PP_2_NOT_MC,
    IPV6_ACTION_BEGIN_EX_HEADER_PROCESSING
} IPV6_ACTION;

// IPv6 Type-length-value type code for the Pad 1 option
#define IPV6_TLV_PAD_1                          0u
// IPv6 Type-length-value type code for the Pad N option
#define IPV6_TLV_PAD_N                          1u
// IPv6 Type-length-value type code for the Hop-by-hop "Jumbogram Payload" option
#define IPV6_TLV_HBHO_PAYLOAD_JUMBOGRAM         0xC2u
// IPv6 Type-length-value type code for the Hop-by-hop "Router Alert" option
#define IPV6_TLV_HBHO_ROUTER_ALERT              0x05u

// IPv6 action code for the unrecognized option reaction to skip the option
#define IPV6_TLV_UNREC_OPT_SKIP_OPTION          0b00
// IPv6 action code for the unrecognized option reaction to discard the packet silently
#define IPV6_TLV_UNREC_OPT_DISCARD_SILENT       0b01
// IPv6 action code for the unrecognized option reaction to discard the packet and send 
// an ICMP parameter problem message
#define IPV6_TLV_UNREC_OPT_DISCARD_PP           0b10
// IPv6 action code for the unrecognized option reaction to discard the packet and send 
// an ICMP parameter problem message is the destination addr isn't a multicast address
#define IPV6_TLV_UNREC_OPT_DISCARD_PP_NOT_MC    0b11

#define IPV6_HEADER_OFFSET_PAYLOAD_LENGTH       (0x04u)
#define IPV6_HEADER_OFFSET_NEXT_HEADER          (0x06u)
#define IPV6_HEADER_OFFSET_SOURCE_ADDR          (0x08u)
#define IPV6_HEADER_OFFSET_DEST_ADDR            (0x08u + sizeof (IPV6_ADDR))

typedef enum
{
    IPV6_PREFER_PUBLIC_ADDRESSES = 0,
    IPV6_PREFER_TEMPORARY_ADDRESSES
} IPV6_ADDRESS_PREFERENCE;

typedef enum
{
    TYPE_IPV6_HEADER = 1u,
    TYPE_IPV6_EX_HEADER_HOP_BY_HOP_OPTIONS,
    TYPE_IPV6_EX_HEADER_DESTINATION_OPTIONS_1,
    TYPE_IPV6_EX_HEADER_ROUTING,
    TYPE_IPV6_EX_HEADER_FRAGMENT,
    TYPE_IPV6_EX_HEADER_AUTHENTICATION_HEADER,
    TYPE_IPV6_EX_HEADER_ENCAPSULATING_SECURITY_PAYLOAD,
    TYPE_IPV6_EX_HEADER_DESTINATION_OPTIONS_2,
    TYPE_IPV6_UPPER_LAYER_HEADER,
    TYPE_IPV6_UPPER_LAYER_PAYLOAD,
    TYPE_IPV6_BEGINNING_OF_WRITABLE_PART,
    TYPE_IPV6_END_OF_LIST
} IPV6_SEGMENT_TYPE;

extern const IPV6_ADDR IPV6_FIXED_ADDR_UNSPECIFIED;
extern const IPV6_ADDR IPV6_FIXED_ADDR_ALL_NODES_MULTICAST;
extern const IPV6_ADDR IPV6_FIXED_ADDR_ALL_ROUTER_MULTICAST;
extern const IPV6_ADDR IPV6_SOLICITED_NODE_MULTICAST;

typedef union
{
    unsigned char b;

    struct
    {
        unsigned option : 6;
        unsigned unrecognizedAction : 2;
    } bits;
} IPV6_TLV_OPTION_TYPE;

typedef union
{
    unsigned char byte;

    struct
    {
        unsigned scope : 4;
        unsigned type : 2;
    } bits;
} IPV6_ADDRESS_TYPE;

typedef struct
{
    IPV6_ADDR address;
    unsigned char prefixLength;
    unsigned char precedence;
    unsigned char label;
} IPV6_ADDRESS_POLICY;

typedef struct _IPV6_DATA_SEGMENT_HEADER
{
    uint8_t* dataLocation; // Location of the data to transmit
    unsigned short segmentSize; // Size of this data segment
    unsigned short segmentLen; // Number of bytes of data in this segment
    unsigned char memory; // Type: IPV6_DATA_NONE, IPV6_DATA_DYNAMIC_BUFFER, IPV6_DATA_NETWORK_FIFO, IPV6_DATA_PIC_RAM
    unsigned char segmentType; // Type of segment contents
    struct _IPV6_DATA_SEGMENT_HEADER * nextSegment; // Pointer to the next data segment
    void * data[0]; // Optional buffer space
} IPV6_DATA_SEGMENT_HEADER;

typedef struct _IPV6_RX_FRAGMENT_BUFFER {
    struct _IPV6_RX_FRAGMENT_BUFFER * next; // Next fragmented packet
    uint32_t identification; // Fragment id
    uint16_t bytesInPacket; // Number of bytes written to packet
    uint16_t packetSize; // Packet size (packet is complete when this matches bytesInPacket)
    uint16_t firstFragmentLength; // Length of the first fragment
    uint8_t secondsRemaining; // Number of seconds remaining during which the fragment can be reassembled
    uint8_t packet[1500]; // Packet information
} IPV6_RX_FRAGMENT_BUFFER;

typedef struct __attribute__((__packed__))
{
    uint8_t nextHeader;
    uint8_t reserved;

    union {

        struct __attribute__((__packed__))
        {
            unsigned m : 1;
            unsigned reserved2 : 2;
            unsigned fragmentOffset : 13;
        }
        bits;
        uint16_t w;
    } offsetM;
    uint32_t identification;
} IPV6_FRAGMENT_HEADER;

#define IPV6_DATA_NONE                (0x0u)          // The data segment is unused
#define IPV6_DATA_DYNAMIC_BUFFER      (0x1u)          // Data to transmit is allocated in dynamically allocated RAM
#define IPV6_DATA_NETWORK_FIFO        (0x2u)          // Data to transmit is stored in the Network Controller's FIFOs
#define IPV6_DATA_PIC_RAM             (0x3u)          // Data to transmit is stored in PIC RAM

#define IPV6_NO_UPPER_LAYER_CHECKSUM          (0xFFFFu)


// *****************************************************************************
/* IPv6 packet header definition

  Summary:
    None

  Description:
    None

  Remarks:
    None
 */
typedef struct
{
    unsigned long V_T_F;
    unsigned short PayloadLength;
    unsigned char NextHeader;
    unsigned char HopLimit;
    IPV6_ADDR SourceAddress;
    IPV6_ADDR DestAddress;
} IPV6_HEADER;


typedef enum {
    IPV6_EVENT_ADDRESS_ADDED = 1,
    IPV6_EVENT_ADDRESS_REMOVED,
} IPV6_EVENT_TYPE;

typedef const void * IPV6_HANDLE;

typedef struct
{
}IPV6_MODULE_CONFIG;

// *****************************************************************************
/* Prototype of an IPv6 event handler

  Summary:
    Clients can register a handler with the IPv6 service.

  Description:
    Once an IPv6 event occurs the IPv6 service will call the registered handler
    The handler has to be short and fast.
    It is meant for setting an event flag, not for lengthy processing!

  Remarks:
    None
 */
typedef void (*IPV6_EVENT_HANDLER)(TCPIP_NET_HANDLE hNet, IPV6_EVENT_TYPE evType, const void* param);


// *****************************************************************************
/* Packet allocation callback function

  Summary:
    None

  Description:
    1st parameter will be an IPV6_PACKET*
    2nd parameter is supplied by the caller

  Remarks:
    None
*/
typedef void (*IPV6_PACKET_ACK_FNC)(void*, bool, void*);

// Packet structure/state tracking for IPv6 packets

typedef struct _IPV6_PACKET
{
    struct _IPV6_PACKET * next;                   // Next packet in a queue
    unsigned short payloadLen;                  // Amount of data in payload buffer
    unsigned short headerLen;                   // Total header length (IP header + IPv6 Extension headers)
    unsigned short upperLayerHeaderLen;         // Total length of the upper layer header
    unsigned short upperLayerChecksumOffset;    // Offset of the upper layer checksum
    unsigned char upperLayerHeaderType;         // Type definition for the upper-layer heaer

    union {

        struct {
            unsigned reserved :         3;
            unsigned useUnspecAddr :    1; // This packet should use the unspecified address
            unsigned sourceSpecified :  1; // The upper layer or application layer specified a source address
            unsigned queued :           1; // Packet has been queued
            unsigned addressType :      2; // IP_ADDRESS_TYPE_IPV6 or IP_ADDRESS_TYPE_IPV4
        };
        unsigned char val;
    } flags;
    MAC_ADDR remoteMACAddr; // The packet's remote MAC address
    IPV6_PACKET_ACK_FNC ackFnc; // function to be called when done with the packet
    void* ackParam; // parameter to be used
    void * neighbor;                            // The neighbor that the message was received from
    unsigned short offsetInSegment;             // Offset used for storing fragment transmission information
    SYS_TICK queuedPacketTimeout;
    IPV6_DATA_SEGMENT_HEADER payload;
    TCPIP_NET_HANDLE netIfH; // Net
    IPV6_HEADER ipv6Header;
} IPV6_PACKET;


// *****************************************************************************
// *****************************************************************************
// Section: public API
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************

/*
  Function:
        bool TCPIP_IPV6_InterfaceIsReady (TCPIP_NET_HANDLE netH)

  Summary:
        Determines if an interface is ready for IPv6 transactions.

  Description:
        Determines if an interface is ready for IPv6 transactions.

  Precondition:
        None

  Parameters:
        pNetIf - The interface to check

  Returns:
        true if the interface has IPv6 functionality available, false otherwise
  	
  Remarks:
        None
 */
bool TCPIP_IPV6_InterfaceIsReady(TCPIP_NET_HANDLE netH);

// *****************************************************************************

/*
  Function:
        IPV6_ADDR_STRUCT * TCPIP_IPV6_DAS_SelectSourceAddress (TCPIP_NET_HANDLE hNetIf,
        IPV6_ADDR * dest, IPV6_ADDR * requestedSource)

  Summary:
        Determines the appropriate source address for a given destination
    address.

  Description:
        Determines the appropriate source address for a given destination
    address.

  Precondition:
        None

  Parameters:
        hNetIf - The given interface.
    dest - The destination address.
    requestedSource - A specified source.

  Returns:
        IPV6_ADDR_STRUCT * - The selected source address.
  	
  Remarks:
        None
 */
IPV6_ADDR_STRUCT * TCPIP_IPV6_DAS_SelectSourceAddress(TCPIP_NET_HANDLE hNetIf, IPV6_ADDR * dest, IPV6_ADDR * requestedSource);

// *****************************************************************************

/*
  Function:
        IPV6_PACKET * TCPIP_IPV6_AllocateTxPacket (TCPIP_NET_HANDLE netH, 
    IPV6_PACKET_ACK_FNC ackFnc, void* ackParam);


  Summary:
    Dynamically allocates a packet for transmitting IP protocol data.

  Description:
    Dynamically allocates a packet for transmitting IP protocol data.	

  Precondition:
        None

  Parameters:
        pNetIf - Interface of the outgoing packet.
    ackFnc      - function to be called when IP is done with the TX packet
                  (finished transmitting)
    ackParam    - parameter to be used for this callback
                  This has meaning only for the caller of the
                  TCPIP_IPV6_AllocateTxPacket

  Returns:
        IPV6_PACKET * - Pointer to the allocated packet.
  	
  Remarks:
        None
 */
IPV6_PACKET * TCPIP_IPV6_AllocateTxPacket(TCPIP_NET_HANDLE netH, IPV6_PACKET_ACK_FNC ackFnc, void* ackParam);

// *****************************************************************************

/*
  Function:
        unsigned short TCPIP_IPV6_IsTxPutReady (IPV6_PACKET * ptrPacket,
        unsigned short count)

  Summary:
        Determines whether a TX packet can be written to.

  Description:
        Determines whether a TX packet can be written to.  This function will
    allocate additional space to the packet to accomodate the user.	

  Precondition:
        None

  Parameters:
        ptrPacket - The packet to check.
    count - The amount of writable space to check for,

  Returns:
        unsigned short - The amount of space available.
  	
  Remarks:
        None
 */
unsigned short TCPIP_IPV6_IsTxPutReady(IPV6_PACKET * pkt, unsigned short count);

// *****************************************************************************

/*
  Function:
        void TCPIP_IPV6_FreePacket (IPV6_PACKET * ptrPacket)

  Summary:
        Frees a TCP/IP Packet structure from dynamic memory.

  Description:
        Frees a TCP/IP Packet structure from dynamic memory.

  Precondition:
        None

  Parameters:
        ptrPacket - The packet to free.

  Returns:
        None
  	
  Remarks:
        None
 */
void TCPIP_IPV6_FreePacket(IPV6_PACKET * pkt);

// *****************************************************************************

/*
  Function:
        bool TCPIP_IPV6_Put (IPV6_PACKET * pkt, unsigned char v)

  Summary:
        Writes a character of data to a packet.

  Description:
    Writes a character of data to a packet.

  Precondition:
        None

  Parameters:
        pkt - The packet.
        v - The characeter.

  Returns:
    true if the character was written, false otherwise
  	
  Remarks:
        None
 */
bool TCPIP_IPV6_Put(IPV6_PACKET * pkt, unsigned char v);

// *****************************************************************************

/*
  Function:
        unsigned short TCPIP_IPV6_PutArray (IPV6_PACKET * ptrPacket,
        const void * dataSource, unsigned short len)

  Summary:
        Writes data to a packet

  Description:
        Writes data to an outgoing packet.

  Precondition:
        The TCPIP_IPV6_IsTxPutReady function must have returned a value greater
    than or equal to 'len.'

  Parameters:
        ptrPacket - The packet.
    dataSource - Pointer to the data to copy to the packet.
    len - Length of the data.

  Returns:
        unsigned short - The number of bytes of data written.
  	
  Remarks:
        None
 */
unsigned short TCPIP_IPV6_PutArrayHelper(IPV6_PACKET * pkt, const void * dataSource, uint8_t dataType, unsigned short len);
#define TCPIP_IPV6_PutArray(pkt,data,len)        TCPIP_IPV6_PutArrayHelper(pkt, data, IPV6_DATA_PIC_RAM, len)

// *****************************************************************************

/*
  Function:
        unsigned short TCPIP_IPV6_SetPayload (IPV6_PACKET * ptrPacket,
        uint8_t* payload, unsigned short len)

  Summary:
        Appends a buffer of non-volatile data in the PIC RAM to the end of a
    packet.

  Description:
        This function will append a segment of data in PIC RAM on to the end
    of a TX packet that is being constructed without copying the data.  The 
    data must be maintained in RAM until the packet has been transmitted 
    (verified by the acknowledge function).

  Precondition:
        None

  Parameters:
        ptrPacket - The packet.
    payload - Address of the data payload.
    len - Length of the data payload

  Returns:
        unsigned short - The amount of data added to the packet length.
  	
  Remarks:
        This function is useful for adding payloads to outgoing packets without
    copying them if the data is in another preexisting buffer (i.e. TCP).
 */
unsigned short TCPIP_IPV6_SetPayload(IPV6_PACKET * pkt, uint8_t* payload, unsigned short len);


// *****************************************************************************

/*
  Function:
        bool TCPIP_IPV6_Flush (IPV6_PACKET * ptrPacket, MAC_ADDR * remoteMACAddr)

  Summary:
        Flushes an IP TX packet.

  Description:
        Flushes an IP TX packet.  Determines the link-layer address if necessary.
    Calculates the upper-layer checksum if necessary.	

  Precondition:
        None

  Parameters:
        ptrPacket - The packet to flush.
    remoteMACAddr - An optional explicity specified MAC address (for IPv4).

  Returns:
        bool - True if the packet has been transmitted, false if the packet
        has been queued.
  	
  Remarks:
        None
 */
bool TCPIP_IPV6_Flush(IPV6_PACKET * pkt, MAC_ADDR * remoteMACAddr/*, pMACNotifyF pNofify*/);


// *****************************************************************************

/*
  Function:
        uint8_t TCPIP_IPV6_Get (TCPIP_MAC_HANDLE hMac, uint8_t* pData)

  Summary:
        Reads the next byte of data from the specified MAC.

  Description:
    Reads a character of data from a packet.

  Precondition:
        None

  Parameters:
        hMac - The MAC to read data from

  Returns:
    The data read.
  	
  Remarks:
        None
 */
#define TCPIP_IPV6_Get(hMac, pData)                do{MACGetArray(hMac, pData, 1);}while(0)

// *****************************************************************************

/*
  Function:
        uint8_t TCPIP_IPV6_GetArray (TCPIP_MAC_HANDLE hMac, uint8_t *val,
        uint16_t len);

  Summary:
        Reads the next byte of data from the specified MAC.

  Description:
    Reads a character of data from a packet.

  Precondition:
        None

  Parameters:
        hMac - The MAC to read data from
    val - The buffer to store the data
    len - The amount of data to read

  Returns:
    uint8_t - The number of bytes read.
  	
  Remarks:
        None
 */
#define TCPIP_IPV6_GetArray(hMac, val, len)     MACGetArray(hMac, val, len)



IPV6_ADDR *  TCPIP_IPV6_GetDestAddress(IPV6_PACKET * p);

void  TCPIP_IPV6_SetDestAddress(IPV6_PACKET * p, IPV6_ADDR * addr);

void  TCPIP_IPV6_SetSourceAddress(IPV6_PACKET * p, IPV6_ADDR * addr);

IPV6_ADDR *  TCPIP_IPV6_GetSourceAddress(IPV6_PACKET * p);

// *****************************************************************************

/*
  Function:
        IPV6_ADDR_STRUCT * TCPIP_IPV6_AddUnicastAddress (TCPIP_NET_HANDLE netH,
        IPV6_ADDR * address, uint8_t skipProcessing)

  Summary:
        Adds a unicast address to a specified interface

  Description:
        Adds a unicast address to a specified interface.  Starts duplicate address
    detection if necessary.

  Precondition:
        None

  Parameters:
        pNetIf - The interface to add the address to.
    address - The address to add.
    skipProcessing - true to skip Duplicate address detection, false otherwise

  Returns:
        IPV6_ADDR_STRUCT * - Pointer to the structure of the newly allocated
        address
   
  Remarks:
        None
 */
IPV6_ADDR_STRUCT * TCPIP_IPV6_AddUnicastAddress(TCPIP_NET_HANDLE netH, IPV6_ADDR * address, uint8_t skipProcessing);

// *****************************************************************************

/*
 Function:
       void IPv6RemoveUnicastAddress (TCPIP_NET_HANDLE netH, IPV6_ADDR * address)

 Summary:
       Removed a configured unicast address from an interface.

 Description:
       Removed a configured unicast address from an interface.

 Precondition:
       None

 Parameters:
       pNetIf - The interface.
   address - The address

 Returns:
       None
  	
 Remarks:
       None
 */
void TCPIP_IPV6_RemoveUnicastAddress(TCPIP_NET_HANDLE netH, IPV6_ADDR * address);

// *****************************************************************************

/*
 Function:
       IPV6_ADDR_STRUCT * TCPIP_IPV6_AddMulticastListener (TCPIP_NET_HANDLE netH,
       IPV6_ADDR * address)

 Summary:
       Adds a multicast listener to an interface.

 Description:
       Adds a multicast listener to an interface.

 Precondition:
       None

 Parameters:
       pNetIf - The interface to add the address to.
   address - The new listener

 Returns:
       IPV6_ADDR_STRUCT * - Pointer to the new listener, or NULL
  	
 Remarks:
       None
 */
IPV6_ADDR_STRUCT * TCPIP_IPV6_AddMulticastListener(TCPIP_NET_HANDLE hNet, IPV6_ADDR * address);

// *****************************************************************************

/*
  Function:
        void IPv6RemoveMulticastListener (TCPIP_NET_HANDLE netH, IPV6_ADDR * address)

  Summary:
        Removes a multicast listener from a given interface.

  Description:
        Removes a multicast listener from a given interface.

  Precondition:
        None

  Parameters:
        pNetIf - The interface
    address - The address

  Returns:
        None
  	
  Remarks:
        None
 */
void TCPIP_IPV6_RemoveMulticastListener(TCPIP_NET_HANDLE netH, IPV6_ADDR * address);


IPV6_HANDLE TCPIP_IPV6_RegisterHandler(TCPIP_NET_HANDLE hNet, IPV6_EVENT_HANDLER handler, const void* hParam);

bool TCPIP_IPV6_DeRegisterHandler(IPV6_HANDLE hIpv6);


#endif // __IPV6_H_ 



