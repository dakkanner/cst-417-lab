/*******************************************************************************
  Internet Protocol (IP) Version 4 Communications Layer

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Provides a transport for TCP, UDP, and ICMP messages
    -Reference: RFC 791
*******************************************************************************/

/*******************************************************************************
FileName:   ipv4.c
Copyright © 2012 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED ?AS IS? WITHOUT WARRANTY OF ANY KIND,
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
#include "ipv4_private.h"

#include "tcpip_mac_private.h"

#if defined(TCPIP_STACK_USE_IPV4)

#define TCPIP_THIS_MODULE_ID    TCPIP_MODULE_IPV4
#include "tcpip_notify.h"

// This is left shifted by 4.  Actual value is 0x04.
#define IPv4_VERSION        (0x40u)

// IHL (Internet Header Length) is # of 32 bit words in a header.
// Since, we do not support options, our IP header length will be
// minimum i.e. 20 bytes : IHL = 20 / 4 = 5.
#define IP_IHL              (0x05)

#define IP_SERVICE_NW_CTRL  (0x07)
#define IP_SERVICE_IN_CTRL  (0x06)
#define IP_SERVICE_ECP      (0x05)
#define IP_SERVICE_OVR      (0x04)
#define IP_SERVICE_FLASH    (0x03)
#define IP_SERVICE_IMM      (0x02)
#define IP_SERVICE_PRIOR    (0x01)
#define IP_SERVICE_ROUTINE  (0x00)

#define IP_SERVICE_N_DELAY  (0x00)
#define IP_SERCICE_L_DELAY  (0x08)
#define IP_SERVICE_N_THRPT  (0x00)
#define IP_SERVICE_H_THRPT  (0x10)
#define IP_SERVICE_N_RELIB  (0x00)
#define IP_SERVICE_H_RELIB  (0x20)

#define IP_SERVICE          (IP_SERVICE_ROUTINE | IP_SERVICE_N_DELAY)

#if defined(TCPIP_STACK_USE_ZEROCONF_MDNS_SD)
  #define MY_IP_TTL           (255)  // Time-To-Live in hops 
  // IP TTL is set to 255 for Multicast DNS compatibility. See mDNS-draft-08, section 4.
#else
  #define MY_IP_TTL           (100)  // Time-To-Live in hops
#endif


static SystemTickHandle     ipv4TimerHandle = 0;          // Handle for the IP task timer
static int                  ipv4TickPending = 0;          // Pending flag for the IP task

static uint16_t             ipv4Identifier = 0;           // Static identifier value for IPv4 headers
static const void*          ipv4MemH = 0;                     // memory handle

SINGLE_LIST                 ipv4QueuedPackets = { 0 };


static uint16_t             ipv4InitCount = 0;


/************************************************************************/
/****************               Prototypes               ****************/
/************************************************************************/
static void SwapIPV4Header(IPV4_HEADER* h);


static void TCPIP_IPV4_QueuePacket (IPV4_PACKET * pkt, SINGLE_LIST* pList, int queueLimit, int tmoSeconds);

static void TCPIP_IPV4_QueuedPacketTransmitTask (SINGLE_LIST* pList);

/*****************************************************************************
  Function:
	bool TCPIP_IPV4_Initialize(const TCPIP_STACK_MODULE_CTRL* const stackInit, 
        const IPV4_MODULE_CONFIG* pIpInit)

  Summary:
	Initializes the IP module.

  Description:
	Initializes the IP module.  Sets the dynamic heap used by this module.	

  Precondition:
	None

  Parameters:
	stackInit - Stack initialization parameters
    pIpInit - Unused supplementary data.

  Returns:
  	true
  	
  Remarks:
	None
  ***************************************************************************/
bool TCPIP_IPV4_Initialize(const TCPIP_STACK_MODULE_CTRL* const stackInit, const IPV4_MODULE_CONFIG* pIpInit)
{
    if(stackInit->stackAction == TCPIP_STACK_ACTION_IF_UP)
    {   // interface restart
        return true;
    }


    if (!ipv4InitCount)
    {
        ipv4MemH = stackInit->memH;
        SingleListInit (&ipv4QueuedPackets);

        if (ipv4TimerHandle == 0)
        {
            ipv4TimerHandle = SYS_TICK_TimerCreate(TCPIP_IPV4_TmoHandler);
        }

        if (ipv4TimerHandle == 0)
        {
            TCPIP_IPV4_DeInitialize(stackInit->memH);
            return false;
        }

        SYS_TICK_TimerSetRate (ipv4TimerHandle, ((SYS_TICK_ResolutionGet() * IPV4_TASK_PROCESS_RATE) + 999)/1000);
        ipv4TickPending = 0;

    }

    ipv4InitCount++;


    return true;
}

/*****************************************************************************
  Function:
	void TCPIP_IPV4_DeInitialize(const TCPIP_STACK_MODULE_CTRL* const stackInit)

  Summary:
	Deinitializes the IP module.

  Description:
	Deinitializes the IP module.	

  Precondition:
	None

  Parameters:
	stackInit - Stack initialization data

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_IPV4_DeInitialize(const TCPIP_STACK_MODULE_CTRL* const stackInit)
{
    if(stackInit->stackAction == TCPIP_STACK_ACTION_DEINIT)
    {   // stack shut down

        ipv4InitCount = (ipv4InitCount > 0 ? ipv4InitCount - 1 : 0);
        if (!ipv4InitCount)
        {
            if(ipv4TimerHandle)
            {
                SYS_TICK_TimerDelete(ipv4TimerHandle);
                ipv4TimerHandle = 0;
            }

            TCPIP_IPV4_SingleListFree (&ipv4QueuedPackets);
            ipv4MemH = 0;
        }
    }
}

/*****************************************************************************
  Function:
	IPV4_PACKET * TCPIP_IPV4_AllocateTxPacket (TCPIP_NET_HANDLE netH, IPV4_PACKET_ACK_FNC ackFnc, void* ackParam);


  Summary:
    Dynamically allocates a packet for transmitting IP protocol data.

  Description:
    Dynamically allocates a packet for transmitting IP protocol data.	

  Precondition:
	None

  Parameters:
	netH        - Interface of the outgoing packet.
    ackFnc      - function to be called when IP is done with the TX packet
                  (finished transmitting)
    ackParam    - parameter to be used for this callback
                  This has meaning only for the caller of the
                  TCPIP_IPV4_AllocateTxPacket

  Returns:
  	IPV4_PACKET * - Pointer to the allocated packet.
  	
  Remarks:
	None
  ***************************************************************************/
IPV4_PACKET * TCPIP_IPV4_AllocateTxPacket (TCPIP_NET_HANDLE netH, IPV4_PACKET_ACK_FNC ackFnc, void* ackParam)
{
    TCPIP_NET_IF * pNetIf =  _TCPIPStackHandleToNet(netH);
    IPV4_PACKET * ptrPacket = (IPV4_PACKET *)TCPIP_HEAP_Malloc (ipv4MemH, sizeof (IPV4_PACKET));

    if (ptrPacket == NULL)
        return NULL;

    ptrPacket->next = 0;
    ptrPacket->netIfH = pNetIf;
    ptrPacket->flags.val = 0;
    ptrPacket->upperLayerChecksumOffset = IPV4_NO_UPPER_LAYER_CHECKSUM;
    
    ptrPacket->payload.dataLocation = (uint8_t*)&ptrPacket->ipv4Header;
    ptrPacket->payload.segmentSize = sizeof (IPV4_HEADER);
    ptrPacket->payload.segmentLen = sizeof (IPV4_HEADER);

    ptrPacket->payload.memory = IPV4_DATA_PIC_RAM;
    ptrPacket->payload.segmentType = TYPE_IPV4_HEADER;
    ptrPacket->payload.nextSegment = NULL;

    TCPIP_IPV4_SetPacketIPProtocol (ptrPacket);

    ptrPacket->headerLen = 0;
    ptrPacket->flags.queued = false;
    ptrPacket->flags.sourceSpecified = false;
    ptrPacket->flags.useUnspecAddr = false;

    ptrPacket->payloadLen = 0;
    ptrPacket->upperLayerHeaderLen = 0;
    memset (&ptrPacket->remoteMACAddr, 0x00, sizeof (MAC_ADDR));

    ptrPacket->ackFnc = ackFnc;
    ptrPacket->ackParam = ackParam;

    return ptrPacket;
}

/*****************************************************************************
  Function:
	bool TCPIP_IPV4_CopyTxPacketStruct (IPV4_PACKET * destination, 
        IPV4_PACKET * source)

  Summary:
	Copies data from one IPV4_PACKET to another.

  Description:
	This function will copy essential transmission data from one IP packet 
    to another.  This includes the upper-layer header and IP header 
    information.

  Precondition:
	None

  Parameters:
	destination - The destination packet.
    source - The source packet

  Returns:
  	true if the data was copied successfully, false otherwise
  	
  Remarks:
	None
  ***************************************************************************/
bool TCPIP_IPV4_CopyTxPacketStruct (IPV4_PACKET * destination, IPV4_PACKET * source)
{
    IPV4_DATA_SEGMENT_HEADER *ptrSegmentSource, *ptrSegmentDest;

    if (destination == NULL || source == NULL)
    {
        return false;
    }

    memcpy ((void *)&destination->remoteMACAddr, (void *)&source->remoteMACAddr, sizeof (MAC_ADDR));
    destination->flags.useUnspecAddr = source->flags.useUnspecAddr;
    destination->flags.sourceSpecified = source->flags.sourceSpecified;
    destination->flags.addressType = source->flags.addressType;


    memcpy ((void *)&destination->payload, (void *)&source->payload, sizeof (IPV4_DATA_SEGMENT_HEADER));

    destination->payload.nextSegment = NULL;

    memcpy ((void *)&destination->ipv4Header, (void *)&source->ipv4Header, sizeof (IPV4_HEADER));
    destination->ipv4Header.TotalLength = 0x0000;
    destination->ipv4Header.HeaderChecksum = 0x0000;

    destination->payload.dataLocation = (uint8_t*)&destination->ipv4Header;

    if ((ptrSegmentSource = TCPIP_IPV4_GetDataSegmentByType (source, TYPE_IPV4_UPPER_LAYER_HEADER)) != NULL)
    {
        if ((ptrSegmentDest = TCPIP_IPV4_GetDataSegmentByType (destination, ptrSegmentSource->segmentType)) == NULL)
        {
            ptrSegmentDest = TCPIP_IPV4_AllocateDataSegmentHeader (ptrSegmentSource->segmentSize);
            if (ptrSegmentDest == NULL)
                return false;
            ptrSegmentDest->nextSegment = NULL;
            TCPIP_IPV4_InsertIPPacketSegment (ptrSegmentDest, destination, ptrSegmentSource->segmentType);
        }

        memcpy ((void *)ptrSegmentDest->dataLocation, (void *)ptrSegmentSource->dataLocation, ptrSegmentSource->segmentLen);
        ptrSegmentDest->segmentLen = ptrSegmentSource->segmentLen;

        destination->upperLayerHeaderLen = source->upperLayerHeaderLen;
        destination->upperLayerChecksumOffset = source->upperLayerChecksumOffset;
        destination->upperLayerHeaderType = source->upperLayerHeaderType;        
    }

    return true;
}

/*****************************************************************************
  Function:
	void TCPIP_IPV4_SetPacketIPProtocol (IPV4_PACKET * ptrPacket) 

  Summary:
	Sets the packet IP protocol for a TX packet to IPv4.

  Description:
	Sets the packet IP protocol for a TX packet to IPv4.	

  Precondition:
	None

  Parameters:
	ptrPacket - Pointer to the target packet.

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_IPV4_SetPacketIPProtocol (IPV4_PACKET * ptrPacket)
{

    ptrPacket->payload.segmentLen = sizeof (IPV4_HEADER);            

    ptrPacket->flags.addressType = IP_ADDRESS_TYPE_IPV4;
}

void TCPIP_IPV4_PutHeader(IPV4_PACKET * ptrPacket, uint8_t protocol)
{
    if(ptrPacket->flags.addressType != IP_ADDRESS_TYPE_IPV4)
    {
        return;
    }

    void * ptrSegment = TCPIP_IPV4_GetDataSegmentContentsByType (ptrPacket, TYPE_IPV4_HEADER);

    if (ptrSegment == NULL)
        return;

    ((IPV4_HEADER *)ptrSegment)->VersionIHL       = IPv4_VERSION | IP_IHL;
    ((IPV4_HEADER *)ptrSegment)->TypeOfService    = IP_SERVICE;
    ((IPV4_HEADER *)ptrSegment)->Identification   = ++ipv4Identifier;
    ((IPV4_HEADER *)ptrSegment)->FragmentInfo     = 0;
    ((IPV4_HEADER *)ptrSegment)->TimeToLive       = MY_IP_TTL;
    ((IPV4_HEADER *)ptrSegment)->Protocol         = protocol;
    ((IPV4_HEADER *)ptrSegment)->HeaderChecksum   = 0;

    if (!ptrPacket->flags.sourceSpecified)
        ((IPV4_HEADER *)ptrSegment)->SourceAddress 	= ((TCPIP_NET_IF*)ptrPacket->netIfH)->netIPAddr;

    ptrPacket->flags.sourceSpecified = false;

    // Note : The upper layer protocol will already have copied the destination address into the packet header
}

/*****************************************************************************
  Function:
	IPV4_DATA_SEGMENT_HEADER * TCPIP_IPV4_PutUpperLayerHeader (
        IPV4_PACKET * ptrPacket, void * header, unsigned short len, 
        unsigned char type, unsigned short checksumOffset)

  Summary:
	Adds an upper layer header to an IP TX packet.

  Description:
	This function will add an upper layer header to the chain of segments 
    in an IPV4_PACKET structure.  It will also initialize the packet's 
    upper-layer header variables.

  Precondition:
	None

  Parameters:
	ptrPacket - The packet that the upper layer header applies to.
    header - Pointer to the header data.
    len - Length of the header.
    type - Standard upper layer header type.
    checksumOffset - Offset of the upper-layer checksum in the header, or 
        IPV4_NO_UPPER_LAYER_CHECKSUM.

  Returns:
  	IPV4_DATA_SEGMENT_HEADER * - Pointer to the segment in the packet that 
        contains the header information.
  	
  Remarks:
	This function will automatically allocate memory to store the header data.
  ***************************************************************************/
IPV4_DATA_SEGMENT_HEADER * TCPIP_IPV4_PutUpperLayerHeader (IPV4_PACKET * ptrPacket, void * header, unsigned short len, unsigned char type, unsigned short checksumOffset)
{
    IPV4_DATA_SEGMENT_HEADER * ptrSegment = TCPIP_IPV4_AllocateDataSegmentHeader(len);

    if (ptrSegment == NULL)
        return NULL;

    ptrSegment->segmentLen = len;

    if (header != NULL)
        memcpy (ptrSegment->data, header, len);

    ptrPacket->upperLayerHeaderLen = len;
    ptrPacket->upperLayerHeaderType = type;
    ptrPacket->upperLayerChecksumOffset = checksumOffset;

    TCPIP_IPV4_InsertIPPacketSegment (ptrSegment, ptrPacket, TYPE_IPV4_UPPER_LAYER_HEADER);

    return ptrSegment;
}

/*****************************************************************************
  Function:
	void TCPIP_IPV4_InsertIPPacketSegment (IPV4_DATA_SEGMENT_HEADER * ptrSegment, 
        IPV4_PACKET * ptrPacket, IPV4_SEGMENT_TYPE type)

  Summary:
	Inserts a data segment into an IPV4_PACKET structure.

  Description:
	This function inserts a data segment into an IPV4_PACKET structure.  It will 
    update the next header fields in existing segments, if applicable.

  Precondition:
	None

  Parameters:
	ptrSegment - The segment to insert.
    ptrPacket - The packet to insert the segment into.
    type - The segment type.  Defined by IPV4_SEGMENT_TYPE enumeration.

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_IPV4_InsertIPPacketSegment (IPV4_DATA_SEGMENT_HEADER * ptrSegment, IPV4_PACKET * ptrPacket, IPV4_SEGMENT_TYPE type)
{
    IPV4_DATA_SEGMENT_HEADER * tempSegment;

    if ((ptrSegment == NULL) || (ptrPacket == NULL))
        return;

    tempSegment = &ptrPacket->payload; 

    ptrSegment->segmentType = type;

    if (type == TYPE_IPV4_END_OF_LIST)
    {
        while (tempSegment->nextSegment != NULL)
            tempSegment = tempSegment->nextSegment;
    }
    else
    {
        while ((tempSegment->nextSegment != NULL) && (ptrSegment->segmentType > tempSegment->nextSegment->segmentType))
            tempSegment = tempSegment->nextSegment;
    }
    ptrSegment->nextSegment = tempSegment->nextSegment;
    tempSegment->nextSegment = ptrSegment;

    if (type == TYPE_IPV4_END_OF_LIST)
    {
        ptrSegment->segmentType = TYPE_IPV4_UPPER_LAYER_PAYLOAD;
    }

}

/*****************************************************************************
  Function:
	IPV4_DATA_SEGMENT_HEADER * TCPIP_IPV4_GetDataSegmentByType (
        IPV4_PACKET * ptrPacket, IPV4_SEGMENT_TYPE type)

  Summary:
	Returns the data segment header of a segment of specified type from a 
    packet.

  Description:
	Returns the data segment header of a segment of specified type from a 
    packet.	

  Precondition:
	None

  Parameters:
	ptrPacket - The packet to search.
    type - IPV4_SEGMENT_TYPE segment type value to search for.

  Returns:
  	IPV4_DATA_SEGMENT_HEADER * - Pointer to the specified segment type, or NULL.
  	
  Remarks:
	There is a special IPV4_SEGMENT_TYPE value defined:
        - TYPE_IPV4_BEGINNING_OF_WRITABLE_PART searches for the first upper layer 
            payload segment to which data can be written.
  ***************************************************************************/
IPV4_DATA_SEGMENT_HEADER * TCPIP_IPV4_GetDataSegmentByType (IPV4_PACKET * ptrPacket, IPV4_SEGMENT_TYPE type)
{
    IPV4_DATA_SEGMENT_HEADER * ptrSegment;

    if (ptrPacket == NULL)
        return NULL;

    ptrSegment = &ptrPacket->payload;

    if (type != TYPE_IPV4_BEGINNING_OF_WRITABLE_PART)
    {
        while ((ptrSegment != NULL) && (ptrSegment->segmentType < type))
        {
            ptrSegment = ptrSegment->nextSegment;
        }
    }
    else
    {
        IPV4_DATA_SEGMENT_HEADER * ptrTempSegment;

        type = TYPE_IPV4_UPPER_LAYER_PAYLOAD;

        while ((ptrSegment != NULL) && (ptrSegment->segmentType < type))
        {
            ptrSegment = ptrSegment->nextSegment;
        }

        while (ptrSegment != NULL)
        {
            // Move ptrSegment to the next dynamically allocated segment
            while ((ptrSegment != NULL) && (ptrSegment->memory != IPV4_DATA_DYNAMIC_BUFFER))
                ptrSegment = ptrSegment->nextSegment;
            // Break out of the loop if the pointer gets to the end of the linked list
            if (ptrSegment == NULL)
                break;
    
            // Initialize ptrTempSegment to the first segment after the beginning of the dynamically allocated segments
            ptrTempSegment = ptrSegment->nextSegment;
            // Check to see if the dynamically allocated section is contiguous at the end of the linked list (or find the break)
            while ((ptrTempSegment != NULL) && (ptrTempSegment->memory == IPV4_DATA_DYNAMIC_BUFFER))
                ptrTempSegment = ptrTempSegment->nextSegment;
    
            if (ptrTempSegment != NULL)
            {
                // If there is a non-dynamic segment, continue in the loop until we find the final sublist
                // of dynamically allocated segments
                ptrSegment = ptrTempSegment;
            }
            else
            {
                // If we have reached the final sublist of dynamic segments, advance to the 
                // section that is writable.
                ptrTempSegment = ptrSegment->nextSegment;
                while (ptrTempSegment != NULL)
                {
                    if (ptrTempSegment->segmentLen != 0)
                        ptrSegment = ptrTempSegment;
                    ptrTempSegment = ptrTempSegment->nextSegment;
                }
                break;
            }
        }
    }

    if (ptrSegment == NULL)
        return NULL;
    else if (ptrSegment->segmentType == type)
        return ptrSegment;
    else
        return NULL;
}

/*****************************************************************************
  Function:
	IPV4_DATA_SEGMENT_HEADER * TCPIP_IPV4_GetDataSegmentContentsByType (
        IPV4_PACKET * ptrPacket, IPV4_SEGMENT_TYPE type)

  Summary:
	Returns a pointer to the contents of a segment of specified type from a 
    packet.

  Description:
	Returns a pointer to the contents of a segment of specified type from a 
    packet.

  Precondition:
	None

  Parameters:
	ptrPacket - The packet to search.
    type - IPV4_SEGMENT_TYPE segment type value to search for.

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void * TCPIP_IPV4_GetDataSegmentContentsByType (IPV4_PACKET * ptrPacket, IPV4_SEGMENT_TYPE type)
{
    IPV4_DATA_SEGMENT_HEADER * ptr;

    ptr = TCPIP_IPV4_GetDataSegmentByType(ptrPacket, type);

    if (ptr != NULL)
    {
        return (void *)ptr->dataLocation;
    }
    else
        return NULL;
}

/*****************************************************************************
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
  ***************************************************************************/
unsigned short TCPIP_IPV4_IsTxPutReady (IPV4_PACKET * ptrPacket, unsigned short count)
{
    IPV4_DATA_SEGMENT_HEADER * ptrSegment;
    unsigned short payloadSize = 0;
    unsigned short availableSpace;

    if (ptrPacket == NULL)
        return 0;

    payloadSize = (count > IP_DEFAULT_ALLOCATION_BLOCK_SIZE)?count:IP_DEFAULT_ALLOCATION_BLOCK_SIZE;

    ptrSegment = TCPIP_IPV4_GetDataSegmentByType (ptrPacket, TYPE_IPV4_UPPER_LAYER_PAYLOAD);

    // Verify that there is a valid upper layer payload
    if (ptrSegment == NULL)
    {
        ptrSegment = TCPIP_IPV4_AllocateDataSegmentHeader(payloadSize);
        if (ptrSegment == NULL)
        {
            return 0;
        }

        TCPIP_IPV4_InsertIPPacketSegment (ptrSegment, ptrPacket, TYPE_IPV4_END_OF_LIST);

        return payloadSize;        
    }

    ptrSegment = TCPIP_IPV4_GetDataSegmentByType (ptrPacket, TYPE_IPV4_BEGINNING_OF_WRITABLE_PART);

    availableSpace = 0;

    if (ptrSegment != NULL)
    {
        while (ptrSegment != NULL)
        {
            availableSpace += ptrSegment->segmentSize - ptrSegment->segmentLen;
            ptrSegment = ptrSegment->nextSegment;
        }

        if (availableSpace >= count)
        {
            return availableSpace;
        }
    }
    
    // allocate new payload
    payloadSize -= availableSpace;
    if(payloadSize < IP_DEFAULT_ALLOCATION_BLOCK_SIZE)
    {
        payloadSize = IP_DEFAULT_ALLOCATION_BLOCK_SIZE;
    }

    ptrSegment = TCPIP_IPV4_AllocateDataSegmentHeader(payloadSize);
    if (ptrSegment == NULL)
    {
        return 0;
    }

    TCPIP_IPV4_InsertIPPacketSegment (ptrSegment, ptrPacket, TYPE_IPV4_END_OF_LIST);

    return availableSpace + payloadSize ;        
}

/*****************************************************************************
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
  ***************************************************************************/
bool TCPIP_IPV4_Put (IPV4_PACKET * pkt, unsigned char v)
{
    return (TCPIP_IPV4_PutArray (pkt, &v, 1) == 1)?true:false;
}

/*****************************************************************************
  Function:
	unsigned short TCPIP_IPV4_PutArrayHelper (IPV4_PACKET * ptrPacket, 
        const void * dataSource, uint8_t dataType, unsigned short len)

  Summary:
	Helper function to write data to a packet.

  Description:
	Helper function to write data to a packet.	

  Precondition:
	None

  Parameters:
	ptrPacket - The packet.
    dataSource - The address of the data on its medium.
    dataType - Descriptor of the data type (dynamic memory on PIC, in a 
                    network FIFO, in static PIC RAM)
    len - Length of the data.

  Returns:
  	unsigned short - The number of bytes of data written.
  	
  Remarks:
	None
  ***************************************************************************/
unsigned short TCPIP_IPV4_PutArrayHelper (IPV4_PACKET * ptrPacket, const void * dataSource, uint8_t dataType, unsigned short len)
{
    IPV4_DATA_SEGMENT_HEADER * ptrSegment;
    uint8_t * dataLocation;
    unsigned short txSize = 0;

    if (ptrPacket == NULL)
        return 0;

    ptrSegment = TCPIP_IPV4_GetDataSegmentByType (ptrPacket, TYPE_IPV4_BEGINNING_OF_WRITABLE_PART);

    if (ptrSegment == NULL)
        return 0;

    dataLocation = (uint8_t *)ptrSegment->dataLocation;

    if (ptrSegment->segmentLen + len > ptrSegment->segmentSize)
    {
        txSize = ptrSegment->segmentSize - ptrSegment->segmentLen;
        if (dataType == IPV4_DATA_PIC_RAM)
            memcpy ((void *)(dataLocation + ptrSegment->segmentLen), dataSource, txSize);
        else if (dataType == IPV4_DATA_NETWORK_FIFO)
            MACGetArray(*((TCPIP_MAC_HANDLE *)dataSource), dataLocation + ptrSegment->segmentLen, txSize);
        else
            return 0;
        ptrSegment->segmentLen += txSize;
        ptrPacket->payloadLen  += txSize;

        ptrSegment = TCPIP_IPV4_AllocateDataSegmentHeader(len - txSize);
        if (ptrSegment == NULL)
            return txSize;

        TCPIP_IPV4_InsertIPPacketSegment (ptrSegment, ptrPacket, TYPE_IPV4_END_OF_LIST);
        dataLocation = (uint8_t *)ptrSegment->dataLocation;
        len -= txSize;
    }

    if ((ptrSegment->segmentLen + len <= ptrSegment->segmentSize))
    {
        if (dataType == IPV4_DATA_PIC_RAM)
            memcpy ((void *)(dataLocation + ptrSegment->segmentLen), dataSource, len);
        else if (dataType == IPV4_DATA_NETWORK_FIFO)
            MACGetArray(*((TCPIP_MAC_HANDLE *)dataSource), dataLocation + ptrSegment->segmentLen, len);
        else
            return 0;
        ptrSegment->segmentLen += len;
        ptrPacket->payloadLen += len;
        return len + txSize;
    }
    return 0;
}

/*****************************************************************************
  Function:
	unsigned short TCPIP_IPV4_SetPayload (IPV4_PACKET * ptrPacket, 
        uint8_t* payload, unsigned short len)

  Summary:
	Allocates a segment on the end of a packet segment chain and uses it to 
    address pre-buffered data.

  Description:
	This function will allocate a data segment header and append it to the 
    end of a chain of segments in a TX packet.  It will set the data ptr in 
    the packet segment to a pre-existing buffer of data.

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
  ***************************************************************************/
unsigned short TCPIP_IPV4_SetPayload (IPV4_PACKET * ptrPacket, uint8_t* payload, unsigned short len)
{
    IPV4_DATA_SEGMENT_HEADER * ptrSegment;

    ptrSegment = TCPIP_IPV4_AllocateDataSegmentHeader (0);
    if(ptrSegment == 0)
    {
        return 0;
    }

    ptrSegment->nextSegment = NULL;
    ptrSegment->dataLocation = payload;
    ptrSegment->segmentSize = len;
    ptrSegment->segmentLen = len;
    ptrSegment->memory = IPV4_DATA_PIC_RAM;

    TCPIP_IPV4_InsertIPPacketSegment (ptrSegment, ptrPacket, TYPE_IPV4_END_OF_LIST);

    ptrPacket->payloadLen += len;

    return len;
}

/*****************************************************************************
  Function:
	unsigned short TCPIP_IPV4_GetPseudoHeaderChecksum (IPV4_PACKET * ptrPacket)

  Summary:
	Returns the 16-bit checksum of the pseudo-header for an IP packet.

  Description:
	Returns the 16-bit checksum of the pseudo-header for an IP packet.	

  Precondition:
	None

  Parameters:
	ptrPacket - The packet

  Returns:
  	unsigned short - The checksum
  	
  Remarks:
	A flag in the packet is used to detrmine the address type (IPv4/6) for 
    this calculation.
  ***************************************************************************/
unsigned short TCPIP_IPV4_GetPseudoHeaderChecksum (IPV4_PACKET * ptrPacket)
{
    if (ptrPacket->flags.addressType == IP_ADDRESS_TYPE_IPV4)
    {
        IPV4_PSEUDO_HEADER pseudoHeader;

        pseudoHeader.SourceAddress = TCPIP_IPV4_GetSourceAddress (ptrPacket);
        pseudoHeader.DestAddress = TCPIP_IPV4_GetDestAddress (ptrPacket);
        pseudoHeader.Zero = 0;
        pseudoHeader.Protocol = ptrPacket->upperLayerHeaderType;
        pseudoHeader.Length = TCPIP_HELPER_ntohs (ptrPacket->upperLayerHeaderLen + ptrPacket->payloadLen);

        return TCPIP_Helper_CalcIPChecksum ((uint8_t *)&pseudoHeader, sizeof (pseudoHeader), 0);
    }
    return 0;
}


/*****************************************************************************
  Function:
	unsigned short TCPIP_IPV4_CalculatePayloadChecksum (IPV4_PACKET * ptrPacket)

  Summary:
	Calculates the checksum of an upper layer payload for a TX packet.

  Description:
	Calculates the checksum of an upper layer payload for a TX packet.

  Precondition:
	None

  Parameters:
	ptrPacket - The packet

  Returns:
  	unsigned short - The checksum.
  	
  Remarks:
	None
  ***************************************************************************/
unsigned short TCPIP_IPV4_CalculatePayloadChecksum (IPV4_PACKET * ptrPacket)
{
    IPV4_DATA_SEGMENT_HEADER * ptrSegment;
    TCPIP_UINT32_VAL checksum;
    TCPIP_MAC_HANDLE hMac;
    uint16_t tempChecksum = 0;
    unsigned short checksumByteCount = 0;

    ptrSegment = TCPIP_IPV4_GetDataSegmentByType (ptrPacket, TYPE_IPV4_UPPER_LAYER_HEADER);

    checksum.Val = 0;
    if (ptrSegment != NULL)
    {
        checksum.w[0] = ~TCPIP_Helper_CalcIPChecksum((uint8_t *)ptrSegment->dataLocation, ptrPacket->upperLayerHeaderLen, 0);
        checksumByteCount += ptrPacket->upperLayerHeaderLen;
    }

    ptrSegment = TCPIP_IPV4_GetDataSegmentByType (ptrPacket, TYPE_IPV4_UPPER_LAYER_PAYLOAD);

    while (ptrSegment != NULL)
    {
        switch (ptrSegment->memory)
        {
            case IPV4_DATA_DYNAMIC_BUFFER:
            case IPV4_DATA_PIC_RAM:
                tempChecksum = ~TCPIP_Helper_CalcIPChecksum((uint8_t *)ptrSegment->dataLocation, ptrSegment->segmentLen, 0);
                if (checksumByteCount % 2)
                {
                    tempChecksum = TCPIP_HELPER_htons(tempChecksum);
                }
                checksumByteCount += ptrSegment->segmentLen;
                break;
            case IPV4_DATA_NETWORK_FIFO:
                hMac = _TCPIPStackNetToMac ((TCPIP_NET_IF*)ptrPacket->netIfH);
                MACSetReadPtr (hMac, (TCPIP_MAC_PTR_TYPE)ptrSegment->dataLocation);
                tempChecksum = ~MACCalcIPBufferChecksum(hMac, ptrSegment->segmentLen);
                if (checksumByteCount % 2)
                {
                    tempChecksum = TCPIP_HELPER_htons(tempChecksum);
                }
                checksumByteCount += ptrSegment->segmentLen;
                break;
            case IPV4_DATA_NONE:
                tempChecksum = 0;
                break;
        }
        checksum.Val += (uint32_t)tempChecksum;
        ptrSegment = ptrSegment->nextSegment;
    }

    checksum.Val = (uint32_t)checksum.w[0] + (uint32_t)checksum.w[1];
    checksum.w[0] += checksum.w[1];
    return ~checksum.w[0];
}

/*****************************************************************************
  Function:
	void TCPIP_IPV4_ResetTransmitPacketState (IPV4_PACKET * pkt)

  Summary:
	Removes payload data from a TX packet structure so it can be reused by 
    a socket for furthur communications.

  Description:
	Removes payload data from a TX packet structure so it can be reused by 
    a socket for furthur communications.	

  Precondition:
	None

  Parameters:
	pkt - The packet to reset.

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_IPV4_ResetTransmitPacketState (IPV4_PACKET * pkt)
{
    IPV4_DATA_SEGMENT_HEADER * segmentHeader = &pkt->payload;
    IPV4_DATA_SEGMENT_HEADER * segmentHeader2 = 0;

    while ((segmentHeader != NULL) && (segmentHeader->segmentType != TYPE_IPV4_UPPER_LAYER_PAYLOAD))
    {
        segmentHeader2 = segmentHeader;
        segmentHeader = segmentHeader->nextSegment;
    }

    if(segmentHeader2)
    {
        segmentHeader2->nextSegment = NULL;
    }

    while (segmentHeader != NULL)
    {
        segmentHeader2 = segmentHeader->nextSegment;
        TCPIP_HEAP_Free (ipv4MemH, segmentHeader);
        segmentHeader = segmentHeader2;
    }

    pkt->flags.queued = false;
    pkt->payloadLen = 0;
}


/*****************************************************************************
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
  ***************************************************************************/
bool TCPIP_IPV4_Flush (IPV4_PACKET * ptrPacket, MAC_ADDR * remoteMACAddr)
{
    uint16_t * checksumPointer;
    bool       toTxPkt = false;

    if(ptrPacket == 0 || ptrPacket->flags.addressType != IP_ADDRESS_TYPE_IPV4)
    {
        return false;
    }

    // Write the Ethernet Header to the MAC TX Buffer
    IPV4_HEADER * ptrIpHeader = TCPIP_IPV4_GetDataSegmentContentsByType (ptrPacket, TYPE_IPV4_HEADER);

    // Transmit IPv4 Packet
    ptrIpHeader->TotalLength = sizeof (IPV4_HEADER) + ptrPacket->upperLayerHeaderLen + ptrPacket->payloadLen;

    SwapIPV4Header (ptrIpHeader);

    ptrIpHeader->HeaderChecksum = TCPIP_Helper_CalcIPChecksum((uint8_t *)ptrIpHeader, sizeof (IPV4_HEADER), 0);

    if (ptrPacket->upperLayerChecksumOffset != IPV4_NO_UPPER_LAYER_CHECKSUM)
    {
        checksumPointer = TCPIP_IPV4_GetDataSegmentContentsByType (ptrPacket, TYPE_IPV4_UPPER_LAYER_HEADER);
        checksumPointer = (uint16_t *)(((uint8_t *)checksumPointer) + ptrPacket->upperLayerChecksumOffset);
        *checksumPointer = ~TCPIP_IPV4_GetPseudoHeaderChecksum (ptrPacket);
        *checksumPointer = TCPIP_IPV4_CalculatePayloadChecksum (ptrPacket);
    }

    if (remoteMACAddr != NULL)
    {
        memcpy (&ptrPacket->remoteMACAddr, remoteMACAddr, sizeof (MAC_ADDR));
    }

    toTxPkt = true;

    if(toTxPkt)
    {   // packet needs to be transmitted

        if(TCPIP_IPV4_TransmitPacket (ptrPacket))
        {   // success
            if (ptrPacket->ackFnc)
            {
                (*ptrPacket->ackFnc)(ptrPacket, true, ptrPacket->ackParam);
            }
            return true;
        }
        else
        {
            TCPIP_IPV4_QueuePacket (ptrPacket, &ipv4QueuedPackets, IPV4_QUEUED_PACKET_LIMIT, IPV4_QUEUED_PACKET_TIMEOUT);
        }
    }

    return false;
}

/*****************************************************************************
  Function:
	bool TCPIP_IPV4_TransmitPacket (IPV4_PACKET * pkt)

  Summary:
	Writes the formatted packet to the MAC layer and flushes it.

  Description:
	Writes the formatted packet to the MAC layer and flushes it.

  Precondition:
	None

  Parameters:
	pkt - The packet to transmit

  Returns:
  	bool - true if the packet was transmitted, false if the MAC is unlinked
  	
  Remarks:
	None
  ***************************************************************************/
bool TCPIP_IPV4_TransmitPacket (IPV4_PACKET * pkt)
{
    TCPIP_MAC_HANDLE hMac;
 
    if (pkt->flags.addressType != IP_ADDRESS_TYPE_IPV4)
    {
        return false;
    }

    hMac = _TCPIPStackNetToMac ((TCPIP_NET_IF*)pkt->netIfH);
    
    if (!_TCPIPStackIsNetLinked((TCPIP_NET_IF*)pkt->netIfH))
    {
        return false;
    }

    if (!TCPIP_IPV4_IsTxReady((TCPIP_NET_IF*)pkt->netIfH))
    {
        return false;
    }

    // Initialize MAC TX Write Pointer
    MACSetWritePtr (hMac, MACGetTxBaseAddr(hMac));

    // Write Ethernet Header to the MAC TX Buffer
    MACPutHeader (hMac, &pkt->remoteMACAddr, ETHERTYPE_IPV4, sizeof (IPV4_HEADER) + pkt->upperLayerHeaderLen + pkt->payloadLen);

    // Write the payload data to the MAC
    TCPIP_IPV4_FlushDataSegments(hMac, pkt);

    // Transmit the packet
    MACFlush (hMac);

    pkt->flags.queued = false;

    return true;
}


/*****************************************************************************
  Function:
	void TCPIP_IPV4_FlushDataSegments (TCPIP_MAC_HANDLE hMac, 
        IPV4_PACKET * ptrPacket)

  Summary:
	Helper function for TCPIP_IPV4_TransmitPacket that writes all segments in 
    the packet data segment chain to the MAC layer.

  Description:
	Helper function for TCPIP_IPV4_TransmitPacket that writes all segments in 
    the packet data segment chain to the MAC layer.

  Precondition:
	None

  Parameters:
	hMac - The target MAC handle.
    ptrPacket - The packet.

  Returns:
  	None.
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_IPV4_FlushDataSegments (TCPIP_MAC_HANDLE hMac, IPV4_PACKET * ptrPacket)
{
    IPV4_DATA_SEGMENT_HEADER * segmentHeader = &ptrPacket->payload;

    while (segmentHeader != NULL)
    {
        MACPutArray(hMac, (uint8_t *)segmentHeader->dataLocation, segmentHeader->segmentLen);
        segmentHeader = segmentHeader->nextSegment;
    }
}

/*****************************************************************************
  Function:
	IPV4_DATA_SEGMENT_HEADER * TCPIP_IPV4_AllocateDataSegmentHeader (uint16_t len)

  Summary:
	Allocates a data segment header and an optional payload

  Description:
	Allocates a data segment header and an optional payload	

  Precondition:
	None

  Parameters:
	len - Length of the optional dynamic payload to allocate for this segment.

  Returns:
  	IPV4_DATA_SEGMENT_HEADER * - Pointer to the new segment.
  	
  Remarks:
	None
  ***************************************************************************/
IPV4_DATA_SEGMENT_HEADER * TCPIP_IPV4_AllocateDataSegmentHeader (uint16_t len)
{
    IPV4_DATA_SEGMENT_HEADER * ptrSegment = (IPV4_DATA_SEGMENT_HEADER *)TCPIP_HEAP_Malloc (ipv4MemH, sizeof (IPV4_DATA_SEGMENT_HEADER) + len);

    if (ptrSegment == NULL)
    {
        return NULL;
    }

    if (len)
    {
        ptrSegment->dataLocation = (uint8_t*)ptrSegment->data;
        ptrSegment->segmentSize = len;
        ptrSegment->segmentLen = 0;
        ptrSegment->memory = IPV4_DATA_DYNAMIC_BUFFER;
    }
    else
    {
        ptrSegment->memory = IPV4_DATA_NONE;
        ptrSegment->dataLocation = (uint8_t*)NULL;
        ptrSegment->segmentSize = 0;
        ptrSegment->segmentLen = 0;
    }
    ptrSegment->nextSegment = NULL;

    return ptrSegment;
}

/*****************************************************************************
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
  ***************************************************************************/
void TCPIP_IPV4_FreePacket (IPV4_PACKET * ptrPacket)
{
    if (ptrPacket == NULL)
        return;

    TCPIP_IPV4_FreePacketData (ptrPacket);
    TCPIP_HEAP_Free (ipv4MemH, ptrPacket);
}

/*****************************************************************************
  Function:
	void TCPIP_IPV4_FreePacketData (IPV4_PACKET * ptrPacket)

  Summary:
	Frees all dynamically allocated structures used by an IPV4_PACKET struct.

  Description:
	Frees all dynamically allocated structures used by an IPV4_PACKET struct.

  Precondition:
	None

  Parameters:
	ptrPacket - The packet to free.

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_IPV4_FreePacketData (IPV4_PACKET * ptrPacket)
{
    IPV4_DATA_SEGMENT_HEADER * ptrSegment;
    IPV4_DATA_SEGMENT_HEADER * ptrSegment2;

    if (ptrPacket == NULL)
        return;

    // Set the initial segment to the segment after the IP header (which shouldn't be deallocated
    ptrSegment = ptrPacket->payload.nextSegment;

    while (ptrSegment != NULL)
    {
        ptrSegment2 = ptrSegment->nextSegment;
        TCPIP_HEAP_Free (ipv4MemH, ptrSegment);
        ptrSegment = ptrSegment2;
    }
}

void TCPIP_IPV4_SetTxBuffer(TCPIP_NET_IF* pNet, uint16_t offset)
{
    TCPIP_MAC_HANDLE hMac = _TCPIPStackNetToMac(pNet);

    MACSetWritePtr(hMac, offset + MACGetTxBaseAddr(hMac) + sizeof(MAC_ETHERNET_HEADER) + sizeof(IPV4_HEADER));
}


/*********************************************************************
 * Function:        bool TCPIP_IPV4_GetHeader( TCPIP_NET_IF* pNet,
 *                                    IPV4_ADDR    *localIP,
 *                                    NODE_INFO  *remote,
 *                                    uint8_t        *Protocol,
 *                                    uint16_t        *len)
 *
 * PreCondition:    MACGetHeader() == true
 *
 * Input:           pNet       - the interface for multihomed hosts
 *                  localIP     - Local node IP Address as received
 *                                in current IP header.
 *                                If this information is not required
 *                                caller may pass NULL value.
 *                  remote      - Remote node info
 *                  Protocol    - Current packet protocol
 *                  len         - Current packet data length
 *
 * Output:          true, if valid packet was received
 *                  false otherwise
 *
 * Side Effects:    None
 *
 * Note:            Only one IP message can be received.
 *                  Caller may not transmit and receive a message
 *                  at the same time.
 *
 ********************************************************************/
bool TCPIP_IPV4_GetHeader(TCPIP_NET_IF* pNet, IPV4_ADDR *localIP, NODE_INFO *remote, uint8_t *protocol, uint16_t *len)
{
    uint8_t IPHeaderLen;
    TCPIP_UINT16_VAL    CalcChecksum;
    IPV4_HEADER   header;

#if defined(NON_MCHP_MAC)
    TCPIP_UINT16_VAL    ReceivedChecksum;
    uint16_t        checksums[2];
    uint8_t        optionsLen;
	#define MAX_OPTIONS_LEN     (40u)            // As per RFC 791.
    uint8_t        options[MAX_OPTIONS_LEN];
#endif
    TCPIP_MAC_HANDLE hMac;

    hMac = _TCPIPStackNetToMac(pNet);

    // Read IP header.
    MACGetArray(hMac, (uint8_t*)&header, sizeof(header));

    // Make sure that this is an IPv4 packet.
    if((header.VersionIHL & 0xf0) != IPv4_VERSION)
    	return false;

	// Throw this packet away if it is a fragment.  
	// We don't have enough RAM for IP fragment reconstruction.
	if(header.FragmentInfo & 0xFF1F)
		return false;

	IPHeaderLen = (header.VersionIHL & 0x0f) << 2;

#if !defined(NON_MCHP_MAC)
	// Validate the IP header.  If it is correct, the checksum 
	// will come out to 0x0000 (because the header contains a 
	// precomputed checksum).  A corrupt header will have a 
	// nonzero checksum.
	CalcChecksum.Val = MACCalcRxChecksum(hMac, 0, IPHeaderLen);

	// Seek to the end of the IP header
	MACSetReadPtrInRx(hMac, IPHeaderLen);

    if(CalcChecksum.Val)
#else
    // Calculate options length in this header, if there is any.
    // IHL is in terms of numbers of 32-bit words; i.e. actual
    // length is 4 times IHL.
    optionsLen = IPHeaderLen - sizeof(header);

    // If there is any option(s), read it so that we can include them
    // in checksum calculation.
    if ( optionsLen > MAX_OPTIONS_LEN )
        return false;

    if ( optionsLen > 0u )
        MACGetArray(hMac, options, optionsLen);

    // Save header checksum; clear it and recalculate it ourselves.
    ReceivedChecksum.Val = header.HeaderChecksum;
    header.HeaderChecksum = 0;

    // Calculate checksum of header including options bytes.
    checksums[0] = ~TCPIP_Helper_CalcIPChecksum((uint8_t*)&header, sizeof(header), 0);

    // Calculate Options checksum too, if they are present.
    if ( optionsLen > 0u )
        checksums[1] = ~TCPIP_Helper_CalcIPChecksum((uint8_t*)options, optionsLen, 0);
    else
        checksums[1] = 0;

    CalcChecksum.Val  = TCPIP_Helper_CalcIPChecksum((uint8_t*)checksums,
                                            2 * sizeof(uint16_t), 0);

    // Make sure that checksum is correct
    if ( ReceivedChecksum.Val != CalcChecksum.Val )
#endif
    {
        // Bad packet. The function caller will be notified by means of the false 
        // return value and it should discard the packet.
        return false;
    }

    // Network to host conversion.
    SwapIPV4Header(&header);

    // If caller is intrested, return destination IP address
    // as seen in this IP header.
    if ( localIP )
        localIP->Val    = header.DestAddress.Val;

    remote->IPAddr.Val  = header.SourceAddress.Val;
    *protocol           = header.Protocol;
    *len 				= header.TotalLength - IPHeaderLen;

    return true;
}


/*********************************************************************
 * Function:        IPSetRxBuffer(TCPIP_NET_IF* pNet, uint16_t Offset)
 *
 * PreCondition:    None 
 *
 * Input:           Offset from beginning of IP data field
 *
 * Output:          Next Read/Write access to receive buffer is
 *                  set to Offset 
 *
 * Side Effects:    None
 *
 * Note:            None
 *
 ********************************************************************/
void TCPIP_IPV4_SetRxBuffer(TCPIP_NET_IF* pNet, uint16_t Offset) 
{
	MACSetReadPtrInRx(_TCPIPStackNetToMac(pNet), Offset + sizeof (IPV4_HEADER));
}


/*****************************************************************************
  Function:
	static void SwapIPV4Header(IPV4_HEADER* h)

  Summary:
	Swaps the endinaness of parameters in an IPv4 header.

  Description:
	Swaps the endinaness of parameters in an IPv4 header.	

  Precondition:
	None

  Parameters:
	h - The header

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
static void SwapIPV4Header(IPV4_HEADER* h)
{
    h->TotalLength      = TCPIP_HELPER_ntohs(h->TotalLength);
    h->Identification   = TCPIP_HELPER_ntohs(h->Identification);
    h->HeaderChecksum   = TCPIP_HELPER_ntohs(h->HeaderChecksum);
}





/*****************************************************************************
  Function:
	void TCPIP_IPV4_QueuePacket (IPV4_PACKET * pkt, SINGLE_LIST* pList, int queueLimit, int tmoSeconds)

  Summary:
	Queues a packet for future transmission.

  Description:
	Queues a packet for future transmission.

  Precondition:
	None

  Parameters:
    pkt         - The packet to queue.
    pList       - list to use for queuing
    queueLimit  - the max number of packets in the queue.
                  Packets will be removed when the number is reached.
    tmoSeconds  - timeout to set for the entry                  

  Returns:
  	None

  Remarks:
	For IPv6 queuing the tmo has to be 0!
    The queue is processed separately by the NDP
  ***************************************************************************/
static void TCPIP_IPV4_QueuePacket (IPV4_PACKET * pkt, SINGLE_LIST* pList, int queueLimit, int tmoSeconds)
{
    IPV4_PACKET * ptrPacket;

    ptrPacket = (IPV4_PACKET *)pList->head;

    while ((ptrPacket != NULL) && (ptrPacket != pkt))
    {
        ptrPacket = ptrPacket->next;
    }

    if (ptrPacket == NULL)
    {
        if (pList->nNodes == queueLimit)
        {
            ptrPacket = (IPV4_PACKET *)SingleListRemoveHead (pList);
            if (ptrPacket->ackFnc)
            {
                (*ptrPacket->ackFnc)(ptrPacket, false, ptrPacket->ackParam);
            }
        }
        SingleListAddTail (pList, (SGL_LIST_NODE *)pkt);
        pkt->flags.queued = true;
        pkt->queuedPacketTimeout = SYS_TICK_Get() + (SYS_TICK_ResolutionGet() * tmoSeconds);
    }
}




/*****************************************************************************
  Function:
	void TCPIP_IPV4_TmoHandler(SYS_TICK curSysTick)

  Summary:
	Timeout handler for the IP task system timer.

  Description:
	Timeout handler for the IP task system timer.

  Precondition:
	None

  Parameters:
	curSysTick - The current system tick

  Returns:
  	None

  Remarks:
	None
  ***************************************************************************/
void TCPIP_IPV4_TmoHandler(SYS_TICK curSysTick)
{
    ipv4TickPending++;
}

/*****************************************************************************
  Function:
	bool TCPIP_IPV4_TaskPending (void)

  Summary:
	Determines if an IP task is pending

  Description:
	Determines if an IP task is pending

  Precondition:
	None

  Parameters:
	None

  Returns:
    bool - 1 if a task is pending, 0 otherwise

  Remarks:
	None
  ***************************************************************************/
bool TCPIP_IPV4_TaskPending (void)
{
    return (ipv4TickPending == 0)?0:1;
}

/*****************************************************************************
  Function:
	void TCPIP_IPV4_Task (void)

  Summary:
	IPv4 task function

  Description:
	IPv4 task function

  Precondition:
	None

  Parameters:
	None

  Returns:
  	None

  Remarks:
	None
  ***************************************************************************/
void TCPIP_IPV4_Task (void)
{
    ipv4TickPending--;
    TCPIP_IPV4_QueuedPacketTransmitTask (&ipv4QueuedPackets);
}

/*****************************************************************************
  Function:
	void TCPIP_IPV4_QueuedPacketTransmitTask (void)

  Summary:
        Task to transmit/time-out IPv4 queued packets

  Description:
        Task to transmit/time-out IPv4 queued packets

  Precondition:
	None

  Parameters:
	None

  Returns:
  	None

  Remarks:
	None
  ***************************************************************************/

static void TCPIP_IPV4_QueuedPacketTransmitTask (SINGLE_LIST* pList)
{
    IPV4_PACKET * queuedPacket;
    SYS_TICK tick = SYS_TICK_Get();

    while (pList->nNodes != 0)
    {
        queuedPacket = (IPV4_PACKET *)pList->head;
        if ((long)(tick - queuedPacket->queuedPacketTimeout) > 0)
        {
            // Remove packet
            queuedPacket->flags.queued = false;
            SingleListRemoveHead (pList);
            if (queuedPacket->ackFnc)
            {
                (*queuedPacket->ackFnc)(queuedPacket, false, queuedPacket->ackParam);
            }
        }
        else
        {
            if (TCPIP_IPV4_TransmitPacket(queuedPacket))
            {
                queuedPacket->flags.queued = false;
                SingleListRemoveHead (pList);
                if (queuedPacket->ackFnc)
                {
                    (*queuedPacket->ackFnc)(queuedPacket, true, queuedPacket->ackParam);
                }
            }
            else
            {
                return;
            }
        }
    }
}







/*****************************************************************************
  Function:
	void * TCPIP_IPV4_GetUpperLayerHeaderPtr(IPV4_PACKET * pkt)

  Summary:
	Returns a pointer to the upper layer header segment in a packet.

  Description:
	Returns a pointer to the upper layer header segment in a packet.	

  Precondition:
	None

  Parameters:
	pkt - The packet.

  Returns:
  	void * - Pointer to the upper layer header.
  	
  Remarks:
	None
  ***************************************************************************/
void * TCPIP_IPV4_GetUpperLayerHeaderPtr(IPV4_PACKET * pkt)
{
    return TCPIP_IPV4_GetDataSegmentContentsByType(pkt, TYPE_IPV4_UPPER_LAYER_HEADER);
}

/*****************************************************************************
  Function:
	unsigned short TCPIP_IPV4_GetPayloadLength (IPV4_PACKET * pkt)

  Summary:
	Returns the current payload length of a packet.

  Description:
	Returns the current payload length of a packet.	

  Precondition:
	None

  Parameters:
	pkt - The packet.

  Returns:
  	unsigned short - The current payload length, in bytes.
  	
  Remarks:
	None
  ***************************************************************************/
unsigned short TCPIP_IPV4_GetPayloadLength (IPV4_PACKET * pkt)
{
    return pkt->payloadLen + pkt->upperLayerHeaderLen;
}


void  TCPIP_IPV4_SetDestAddress(IPV4_PACKET * p, uint32_t addrValue)
{
    p->ipv4Header.DestAddress.Val = addrValue;
}

IPV4_ADDR  TCPIP_IPV4_GetDestAddress(IPV4_PACKET * p)
{
    return p->ipv4Header.DestAddress;
}

void  TCPIP_IPV4_SetSourceAddress(IPV4_PACKET * p, uint32_t addrValue)
{
    p->ipv4Header.SourceAddress.Val = addrValue;
    p->flags.sourceSpecified = true;
}

IPV4_ADDR  TCPIP_IPV4_GetSourceAddress(IPV4_PACKET * p)
{
    return p->ipv4Header.SourceAddress;
}


/*****************************************************************************
  Function:
	void TCPIP_IPV4_SingleListFree (void * list)

  Summary:
	Frees and deallocates all nodes from a singly linked list.

  Description:
	Frees and deallocates all nodes from a singly linked list.

  Precondition:
	None

  Parameters:
	list - The list.

  Returns:
  	None

  Remarks:
	None
  ***************************************************************************/
void TCPIP_IPV4_SingleListFree (void * list)
{
    SINGLE_LIST * sList = (SINGLE_LIST *) list;
    SGL_LIST_NODE * node;
    while ((node = SingleListRemoveHead (sList)))
        TCPIP_HEAP_Free (ipv4MemH, node);
}

#endif  // defined(TCPIP_STACK_USE_IPV4)


