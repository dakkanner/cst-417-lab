/*******************************************************************************
  IPv4 Defs for Microchip TCP/IP Stack

  Summary:
    
  Description:
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
FileName:  ipv4.h 
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

#ifndef __IPV4_H_
#define __IPV4_H_


// *****************************************************************************
// *****************************************************************************
// Section: public definitions
// *****************************************************************************
// *****************************************************************************

#include "link_list.h"

typedef enum
{
    IP_PROT_ICMP = (1u),
    IP_PROT_TCP = (6u),
    IP_PROT_UDP = (17u),
} IPV4_HEADER_TYPE;


typedef enum
{
    TYPE_IPV4_HEADER = 1u,
    TYPE_IPV4_EX_HEADER_HOP_BY_HOP_OPTIONS,
    TYPE_IPV4_EX_HEADER_DESTINATION_OPTIONS_1,
    TYPE_IPV4_EX_HEADER_ROUTING,
    TYPE_IPV4_EX_HEADER_FRAGMENT,
    TYPE_IPV4_EX_HEADER_AUTHENTICATION_HEADER,
    TYPE_IPV4_EX_HEADER_ENCAPSULATING_SECURITY_PAYLOAD,
    TYPE_IPV4_EX_HEADER_DESTINATION_OPTIONS_2,
    TYPE_IPV4_UPPER_LAYER_HEADER,
    TYPE_IPV4_UPPER_LAYER_PAYLOAD,
    TYPE_IPV4_BEGINNING_OF_WRITABLE_PART,
    TYPE_IPV4_END_OF_LIST
} IPV4_SEGMENT_TYPE;


typedef struct _IPV4_DATA_SEGMENT_HEADER
{
    uint8_t* dataLocation; // Location of the data to transmit
    unsigned short segmentSize; // Size of this data segment
    unsigned short segmentLen; // Number of bytes of data in this segment
    unsigned char memory; // Type: IPV4_DATA_NONE, IPV4_DATA_DYNAMIC_BUFFER, IPV4_DATA_NETWORK_FIFO, IPV4_DATA_PIC_RAM
    unsigned char segmentType; // Type of segment contents
    struct _IPV4_DATA_SEGMENT_HEADER * nextSegment; // Pointer to the next data segment
    void * data[0]; // Optional buffer space
} IPV4_DATA_SEGMENT_HEADER;


#define IPV4_DATA_NONE                (0x0u)          // The data segment is unused
#define IPV4_DATA_DYNAMIC_BUFFER      (0x1u)          // Data to transmit is allocated in dynamically allocated RAM
#define IPV4_DATA_NETWORK_FIFO        (0x2u)          // Data to transmit is stored in the Network Controller's FIFOs
#define IPV4_DATA_PIC_RAM             (0x3u)          // Data to transmit is stored in PIC RAM

#define IPV4_NO_UPPER_LAYER_CHECKSUM          (0xFFFFu)


// *****************************************************************************
/* IPv4 packet header definition

  Summary:
    None

  Description:
    None

  Remarks:
    None
 */
typedef struct
{
    uint8_t VersionIHL;
    uint8_t TypeOfService;
    uint16_t TotalLength;
    uint16_t Identification;
    uint16_t FragmentInfo;
    uint8_t TimeToLive;
    uint8_t Protocol;
    uint16_t HeaderChecksum;
    IPV4_ADDR SourceAddress;
    IPV4_ADDR DestAddress;
} IPV4_HEADER;

typedef struct
{
}IPV4_MODULE_CONFIG;


// *****************************************************************************
/* Packet allocation callback function

  Summary:
    None

  Description:
    1st parameter will be an IPV4_PACKET*
    2nd parameter is supplied by the caller

  Remarks:
    None
*/
typedef void (*IPV4_PACKET_ACK_FNC)(void*, bool, void*);

// Packet structure/state tracking for IPv4 packets

typedef struct _IPV4_PACKET
{
    struct _IPV4_PACKET * next;                   // Next packet in a queue
    unsigned short payloadLen;                  // Amount of data in payload buffer
    unsigned short headerLen;                   // Total header length (IPv4 header)
    unsigned short upperLayerHeaderLen;         // Total length of the upper layer header
    unsigned short upperLayerChecksumOffset;    // Offset of the upper layer checksum
    unsigned char upperLayerHeaderType;         // Type definition for the upper-layer heaer

    union {

        struct {
            unsigned reserved :         3;
            unsigned useUnspecAddr :    1; // This packet should use the unspecified address
            unsigned sourceSpecified :  1; // The upper layer or application layer specified a source address
            unsigned queued :           1; // Packet has been queued
            unsigned addressType :      2; // IP_ADDRESS_TYPE_IPV4
        };
        unsigned char val;
    } flags;
    MAC_ADDR remoteMACAddr; // The packet's remote MAC address
    IPV4_PACKET_ACK_FNC ackFnc; // function to be called when done with the packet
    void* ackParam; // parameter to be used
    SYS_TICK queuedPacketTimeout;
    IPV4_DATA_SEGMENT_HEADER payload;
    TCPIP_NET_HANDLE netIfH; // Net
    IPV4_HEADER ipv4Header;
} IPV4_PACKET;


// *****************************************************************************
// *****************************************************************************
// Section: public API
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************

/*
  Function:
        IPV4_PACKET * TCPIP_IPV4_AllocateTxPacket (TCPIP_NET_HANDLE netH, 
    IPV4_PACKET_ACK_FNC ackFnc, void* ackParam);


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
                  TCPIP_IPV4_AllocateTxPacket

  Returns:
        IPV4_PACKET * - Pointer to the allocated packet.
      
  Remarks:
        None
 */
IPV4_PACKET * TCPIP_IPV4_AllocateTxPacket(TCPIP_NET_HANDLE netH, IPV4_PACKET_ACK_FNC ackFnc, void* ackParam);

// *****************************************************************************

/*
  Function:
        unsigned short TCPIP_IPV4_IsTxPutReady (IPV4_PACKET * ptrPacket,
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
unsigned short TCPIP_IPV4_IsTxPutReady(IPV4_PACKET * pkt, unsigned short count);

// *****************************************************************************

/*
  Function:
        void TCPIP_IPV4_FreePacket (IPV4_PACKET * ptrPacket)

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
void TCPIP_IPV4_FreePacket(IPV4_PACKET * pkt);

// *****************************************************************************

/*
  Function:
        bool TCPIP_IPV4_Put (IPV4_PACKET * pkt, unsigned char v)

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
bool TCPIP_IPV4_Put(IPV4_PACKET * pkt, unsigned char v);

// *****************************************************************************

/*
  Function:
        unsigned short TCPIP_IPV4_PutArray (IPV4_PACKET * ptrPacket,
        const void * dataSource, unsigned short len)

  Summary:
        Writes data to a packet

  Description:
        Writes data to an outgoing packet.

  Precondition:
        The TCPIP_IPV4_IsTxPutReady function must have returned a value greater
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
unsigned short TCPIP_IPV4_PutArrayHelper(IPV4_PACKET * pkt, const void * dataSource, uint8_t dataType, unsigned short len);
#define TCPIP_IPV4_PutArray(pkt,data,len)        TCPIP_IPV4_PutArrayHelper(pkt, data, IPV4_DATA_PIC_RAM, len)

// *****************************************************************************

/*
  Function:
        unsigned short TCPIP_IPV4_SetPayload (IPV4_PACKET * ptrPacket,
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
unsigned short TCPIP_IPV4_SetPayload(IPV4_PACKET * pkt, uint8_t* payload, unsigned short len);


// *****************************************************************************

/*
  Function:
        bool TCPIP_IPV4_Flush (IPV4_PACKET * ptrPacket, MAC_ADDR * remoteMACAddr)

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
bool TCPIP_IPV4_Flush(IPV4_PACKET * pkt, MAC_ADDR * remoteMACAddr/*, pMACNotifyF pNofify*/);


// *****************************************************************************

/*
  Function:
        uint8_t TCPIP_IPV4_GetArray (TCPIP_MAC_HANDLE hMac, uint8_t *val,
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
#define TCPIP_IPV4_GetArray(hMac, val, len)     MACGetArray(hMac, val, len)

void  TCPIP_IPV4_SetDestAddress(IPV4_PACKET * p, uint32_t addrValue);

IPV4_ADDR  TCPIP_IPV4_GetDestAddress(IPV4_PACKET * p);

void  TCPIP_IPV4_SetSourceAddress(IPV4_PACKET * p, uint32_t addrValue);

IPV4_ADDR  TCPIP_IPV4_GetSourceAddress(IPV4_PACKET * p);


#endif // __IPV4_H_ 



