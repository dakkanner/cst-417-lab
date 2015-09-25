
/*******************************************************************************
  Address Resolution Protocol (ARP) Header file


  Company:
    Microchip Technology Incorported

  File Name:
   arp.h

  Summary:
    Address Resolution Protocol (ARP) Header file

  Description:
    This source file contains the ARP module API
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
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

#ifndef __ARP_H_
#define __ARP_H_


// *****************************************************************************
// *****************************************************************************
// Section: API definitions
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************

/*
Various Definitions for Success and Failure Codes

  Summary:
    None

  Description:
    None

  Remarks:
    None
*/
typedef enum
{
    // success codes
    ARP_RES_OK                  = 0,    // operation succeeded
    ARP_RES_ENTRY_NEW,                  // operation succeeded and a new entry was added
    ARP_RES_ENTRY_SOLVED,               // the required entry is already solved
    ARP_RES_ENTRY_QUEUED,               // the required entry was already queued
    ARP_RES_ENTRY_EXIST,                // the required entry was already cached
    ARP_RES_PERM_QUOTA_EXCEED,          // info: the quota of permanent entries was exceeded


    // failure codes
    ARP_RES_NO_ENTRY            = -1,   // no such entry exists
    ARP_RES_CACHE_FULL          = -2,   // the cache is full and no entry could be
                                        // removed to make room
    ARP_RES_TX_FAILED           = -3,   // failed to transmit an ARP message
    ARP_RES_BAD_INDEX           = -4,   // bad query index    
    ARP_RES_BAD_ADDRESS         = -5,   // bad IP address specified 
    ARP_RES_NO_INTERFACE        = -6,   // no such interface exists   
    ARP_RES_BAD_TYPE            = -7,   // no such type is valid/exists   
}ARP_RESULT;


// *****************************************************************************
/*
Type of ARP entry

  Summary:
    None

  Description:
    None

  Remarks:
    None
*/
typedef enum
{
    ARP_ENTRY_TYPE_INVALID,             // empty entry
    ARP_ENTRY_TYPE_PERMANENT,           // entry valid and permanent
    ARP_ENTRY_TYPE_COMPLETE,            // entry valid
    ARP_ENTRY_TYPE_INCOMPLETE,          // entry not resolved yet
    ARP_ENTRY_TYPE_ANY,                 // any busy entry (PERMANENT|COMPLETE|INCOMPLETE) 
    ARP_ENTRY_TYPE_TOTAL,               // total entries - the number of entries the cache can store 
}ARP_ENTRY_TYPE;

typedef struct
{
    ARP_ENTRY_TYPE  entryType;      // what entry type
    IPV4_ADDR         entryIpAdd;     // the entry IP address
    MAC_ADDR        entryHwAdd;     // the entry hardware address
}ARP_ENTRY_QUERY;


// *****************************************************************************
/*
Events

  Summary:
    events reported by ARP

  Description:
    None

  Remarks:
    possibly multiple events can be set,
    where it makes sense
*/
typedef enum
{
    ARP_EVENT_SOLVED        = 0x01,     // a queued cache entry was solved;                        
    ARP_EVENT_UPDATED       = 0x02,     // an existent cache entry was updated
    ARP_EVENT_PERM_UPDATE   = 0x04,     // an update for an permanent entry was received
                                        // however the permanent entry was not updated    
    ARP_EVENT_TMO           = 0x08,     // an entry could not be solved and a tmo occurred                                        
}ARP_EVENT_TYPE;


typedef struct
{
    // specific ARP params
    size_t  cacheEntries;   // cache entries for this interface
    bool    deleteOld;      // delete old cache if still in place,
                            // else don't re-initialize it
    int     entrySolvedTmo; // solved entry removed after this tmo
                            // if not referenced - seconds
    int     entryPendingTmo;// timeout for a pending to be solved entry in the cache, in seconds
    int     entryRetryTmo;  // timeout for resending an ARP request for a pending entry - seconds
                            // 1 sec < tmo < entryPendingTmo
    int     permQuota;      // max percentage of permanent entries allowed in the cache - %
    int     purgeThres;     // purge threshold - %
    int     purgeQuanta;    // no of entries to delete once the threshold is reached
}ARP_MODULE_CONFIG;





// *****************************************************************************
/*
Notification handler

  Summary:
    Notification handler that can be called when a specific entry is resolved.

  Description:
    None

  Remarks:
    The param member significance is module dependent.
    It can be an IP address, pointer to some other structure, etc.
    The handler is called when an event of some sort occurs for a particular IP address entry.
    If pNetIf == 0 then the notification is called for events on any interface
*/
typedef void    (*ARP_EVENT_HANDLER)(TCPIP_NET_HANDLE hNet, const IPV4_ADDR* ipAdd, const MAC_ADDR* MACAddr, ARP_EVENT_TYPE evType, const void* param);


// *****************************************************************************
/*
ARP Handle

  Summary:
    A handle that a client can use.

  Description:
    None

  Remarks:
    This handle can be used by the client after the event handler has been registered.
*/
typedef const void* ARP_HANDLE;

/*
Type of ARP operation

  Summary:
    None

  Description:
    Operation to be performed by an ARP probe

  Remarks:
    Used for low level functionality, ARPProbe
*/
typedef enum
{
    ARP_OPERATION_REQ       = 1,        // ARP request
    ARP_OPERATION_RESP      = 2,        // ARP respone
}ARP_OPERATION_TYPE;



// *****************************************************************************
// *****************************************************************************
// Section: ARP Routines
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
/*
Function:
    ARP_HANDLE ARPRegisterHandler(TCPIP_NET_HANDLE hNet, ARP_EVENT_HANDLER handler, const void* hParam)

  Summary:
    Register an ARP resolve handler

  Description:
    NONE

  Precondition:
    ARP module should have been initialized

  Parameters:
    hNet        -   Specifies interface to register on. 
                    Use hNet == 0 to register on all interfaces available.
    handler     -   Handler to be called for event.
    hParam      -   The hParam is passed by the client and will be used by the 
                    ARP when the notification is made. It is used for per-thread 
                    content or if more modules, for example, share the same handler
                    and need a way to differentiate the callback.

  Returns:
    ARP_HANDLE 
        On Success - Returns a valid handle
        On Failure - null handle

  Example:
     None

  Remarks:
    NONE
*/
ARP_HANDLE      ARPRegisterHandler(TCPIP_NET_HANDLE hNet, ARP_EVENT_HANDLER handler, const void* hParam);

// *****************************************************************************
/*
Function:
    bool ARPDeRegisterHandler(ARP_HANDLE hArp)

  Summary:
    deregister the event handler

  Description:
    None

  Precondition:
    ARP module should have been initialized

  Parameters:
    hArp    -   ARP Handle

  Returns:
    bool
        On Success - true
        On Failure - false (If no such handler registered)

  Example:
    None

  Remarks:
    None
*/
bool            ARPDeRegisterHandler(ARP_HANDLE hArp);


// *****************************************************************************
/*
  Function:
    ARP_RESULT ARPResolve(TCPIP_NET_HANDLE hNet, IPV4_ADDR* IPAddr)

  Summary:
    Transmits an ARP request to resolve an IP address.

  Description:
    This function transmits and ARP request to determine the hardware
    address of a given IP address.
    Upon the address resolution it calls the registered handler
    (if available) with the supplied notification parameter (if != 0)

  Precondition:
    ARP module should have been initialized

  Parameters:
    hNet   - interface to use
    IPAddr - The IP address to be resolved.  The address must be specified
             in network byte order (big endian).

  Returns:
    ARP_RES_ENTRY_SOLVED     - if the required entry is already solved
    ARP_RES_ENTRY_QUEUED     - if the required entry was already queued
    ARP_RES_ENTRY_NEW        - if the operation succeeded and a new entry
                               was added (and queued for resolving)
    ARP_RES_CACHE_FULL       - if new entry could not be inserted,
                               the cache was full
    ARP_RES_BAD_ADDRESS      - bad address specified
    ARP_RES_NO_INTERFACE     - no such interface

  Remarks:

    To retrieve the ARP query result, call the ARPIsResolved() function.
  ***************************************************************************/
ARP_RESULT      ARPResolve(TCPIP_NET_HANDLE hNet, IPV4_ADDR* IPAddr);


// *****************************************************************************
/*
  Function:
    bool ARPIsResolved(TCPIP_NET_HANDLE hNet, IPV4_ADDR* IPAddr, MAC_ADDR* MACAddr)

  Summary:
    Determines if an ARP request has been resolved yet.

  Description:
    This function checks if an ARP request has been resolved yet, and if
    so, stores the resolved MAC address in the pointer provided.

  Precondition:
    ARP module should have been initialized

  Parameters:
    hNet   - interface to use
    IPAddr - The IP address to be resolved.  This must match the IP address
             provided to the ARPResolve() function call.
    MACAddr - A buffer to store the corresponding MAC address retrieved from
             the ARP query.

  Return Values:
    true - The IP address has been resolved and MACAddr MAC address field
           indicates the response.
    false - The IP address is not yet resolved.  Try calling ARPIsResolved()
           again at a later time.  If you don't get a response after a
           application specific timeout period, you may want to call
           ARPResolve() again to transmit another ARP query (in case if the
           original query or response was lost on the network).  If you never
           receive an ARP response, this may indicate that the IP address
           isn't in use.

  Remarks:
  ***************************************************************************/
bool            ARPIsResolved(TCPIP_NET_HANDLE hNet, IPV4_ADDR* IPAddr, MAC_ADDR* MACAddr);

/*
  Function:
    ARP_RESULT ARPProbe(TCPIP_NET_HANDLE hNet, IPV4_ADDR* IPAddr, IPV4_ADDR* srcAddr, ARP_OPERATION_TYPE opType)

  Summary:
    Transmits an ARP probe to resolve an IP address.


  Description:
    This function transmits and ARP probe to determine the hardware
    address of a given IP address.
    The packet will use the type of operation and the source address 
    specified as parameters.

  Precondition:
    ARP module should have been initialized

  Parameters:
    hNet    - interface to use
    IPAddr  - The IP address to be resolved.  The address must be specified
              in network byte order (big endian).
    srcAddr - The source address to be used in the ARP packet
    opTYpe  - Operation code to be set in the outgoing ARP packet    

  Returns:
    ARP_RES_ENTRY_SOLVED     - if the required entry is already solved
    ARP_RES_ENTRY_QUEUED     - if the required entry was already queued
    ARP_RES_ENTRY_NEW        - if the operation succeeded and a new entry
                               was added (and queued for resolving)
    ARP_RES_CACHE_FULL       - if new entry could not be inserted,
                               the cache was full
    ARP_RES_BAD_ADDRESS      - bad address specified
    ARP_RES_NO_INTERFACE     - no such interface

  Remarks:

    This function is a more advanced version of ARPResolve.
    It allows the caller to specify the operation type and the source address
    of the outgoiong AARP packet.

    No check is done for IPAddr to be valid.

    To retrieve the ARP query result, call the ARPIsResolved() function.
  ***************************************************************************/
ARP_RESULT ARPProbe(TCPIP_NET_HANDLE hNet
                    , IPV4_ADDR* IPAddr
                    , IPV4_ADDR* srcAddr
                    , ARP_OPERATION_TYPE opType);

// *****************************************************************************
// *****************************************************************************
// Section: cache manipulation routines
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
/*
Function:
    ARP_RESULT ARPEntrySet(TCPIP_NET_HANDLE hNet, IPV4_ADDR* ipAdd, MAC_ADDR* hwAdd, bool perm)

  Summary:
    Adds an ARP cache entry for the specified interface.

  Description:
    Can be added as permanent. Not subject to timeouts.
    If cache is full, an entry will be deleted to make room.

  Precondition:
    ARP module should have been initialized

  Parameters:
    hNet    -   Interface to use
    ipAdd   -   The ip address
    hwAdd   -   
    perm    -   

  Returns:
    On Success - ARP_RES_OK/ARP_RES_ENTRY_EXIST
    ON Failure - An Error 
        (for example, cache is full with permanent entries that cannot be 
        purged or the permanent quota exceeded)  

  Example:
    None

  Remarks:
    None
*/
ARP_RESULT      ARPEntrySet(TCPIP_NET_HANDLE hNet, IPV4_ADDR* ipAdd, MAC_ADDR* hwAdd, bool perm);


// *****************************************************************************
/*
Function:
    ARP_RESULT ARPEntryGet(TCPIP_NET_HANDLE hNet, IPV4_ADDR* ipAdd, MAC_ADDR* pHwAdd)

  Summary:
    gets the current mapping for an IP address

  Description:
    None

  Precondition:
    ARP module should have been initialized

  Parameters:
    hNet    -   Interface to use
    ipAdd   -   The ip address to get entries from
    pHwAdd  -   The hardware address

  Returns:
    ARP_RESULT
        On Success - ARP_RES_OK
        On Failure - ARP_RES_NO_ENTRY (if no such mapping exists)

  Example:
    None

  Remarks:
    similar to ARPIsResolved()
*/
ARP_RESULT      ARPEntryGet(TCPIP_NET_HANDLE hNet, IPV4_ADDR* ipAdd, MAC_ADDR* pHwAdd);


// *****************************************************************************
/*
Function:
    ARP_RESULT ARPEntryRemove(TCPIP_NET_HANDLE hNet, IPV4_ADDR* ipAdd);

  Summary:
    removes the mapping of an address, even a permanent one

  Description:
    None

  Precondition:
    ARP module should have been initialized

  Parameters:
    hNet    -   Interface to use
    ipAdd   -   ip Address to remove entries for

  Returns:
    ARP_RESULT
        On Success - 
        On Failure - ARP_RES_NO_ENTRY (if no such mapping exists)

  Example:
    None

  Remarks:
    None
*/
ARP_RESULT      ARPEntryRemove(TCPIP_NET_HANDLE hNet,  IPV4_ADDR* ipAdd);


// *****************************************************************************
/*
Function:
    ARP_RESULT ARPEntryRemoveAll(TCPIP_NET_HANDLE hNet)

  Summary:
    removes all the mapping belonging to an interface

  Description:
    None

  Precondition:
    ARP module should have been initialized

  Parameters:
    hNet    -   network interface handle

  Returns:
    ARP_RES_OK

  Example:
    None

  Remarks:
    None
*/
ARP_RESULT      ARPEntryRemoveAll(TCPIP_NET_HANDLE hNet);


// *****************************************************************************
/*
Function:
    ARP_RESULT ARPEntryRemoveNet(TCPIP_NET_HANDLE hNet, IPV4_ADDR* ipAdd, IPV4_ADDR* mask, ARP_ENTRY_TYPE type)

  Summary:
    Removes all the entries belonging to a network interface.

  Description:
    if(entry->type == type and entry->ipAdd & mask == ipAdd & mask)
        then remove entry

  Precondition:
    ARP module should have been initialized

  Parameters:
    hNet    -   Interface handle to use
    ipAdd   -   ip address
    mask    -   ip address of mask
    type    -   type of entries to remove:  
        valid types:    ARP_ENTRY_TYPE_PERMANENT
                        ARP_ENTRY_TYPE_COMPLETE
                        ARP_ENTRY_TYPE_INCOMPLETE
                        ARP_ENTRY_TYPE_ANY

  Returns:
    ARP_RES_OK

  Example:
    None

  Remarks:
    None
*/
ARP_RESULT      ARPEntryRemoveNet(TCPIP_NET_HANDLE hNet, IPV4_ADDR* ipAdd, IPV4_ADDR* mask , ARP_ENTRY_TYPE type);


// *****************************************************************************
/*
Function:
    ARP_RESULT ARPEntryQuery(TCPIP_NET_HANDLE hNet, size_t index, ARP_ENTRY_QUERY* pArpQuery)

  Summary:
    Querries an ARP cache entry using the index of the cache line.

  Description:
    None

  Precondition:
    ARP module should have been initialized
    The index has to be a valid one i.e. < then ARPCacheGetEntriesNo()
    populates the supplied query routine if not NULL

  Parameters:
    hNet        -   Interface handle to use
    index       -   Index to cache
    pArpQuery   -   entry type, ip address, hardware address

  Returns:
    On Success - ARP_RES_OK
    On Failure - ARP_RES_BAD_INDEX (if index is out of range)
        use it for displaying the cache contents

  Example:
    None

  Remarks:
    None
*/
ARP_RESULT      ARPEntryQuery(TCPIP_NET_HANDLE hNet, size_t index, ARP_ENTRY_QUERY* pArpQuery);


// *****************************************************************************
/*
Function:
    size_t ARPCacheGetEntriesNo(TCPIP_NET_HANDLE hNet, ARP_ENTRY_TYPE type);

  Summary:
    Used to retrieve the number of entereies for a specific interface

  Description:
    None

  Precondition:
    ARP module should have been initialized

  Parameters:
    hNet    -   Interface to use
    type    -   Type of ARP entry

  Returns:
    The number of entries of the specified type per interface

  Example:
    None

  Remarks:
    None
*/
size_t          ARPCacheGetEntriesNo(TCPIP_NET_HANDLE hNet, ARP_ENTRY_TYPE type);


// *****************************************************************************
/*
Function
    ARP_RESULT ARPCacheSetThreshold(TCPIP_NET_HANDLE hNet, int purgeThres, int purgeEntries);

  Summary:
    Sets the cache threshold for the specified interface in %

  Description:
    once the number of entries in the cache is > than the threshold
    a number of purgeEntries (usually one) will be discarded

  Precondition:
    ARP module should have been initialized

  Parameters:
    hNet            -   Interface handle to use
    purgeThres      -   Threshold to start cache purging
    purgeEntries    -   Number of entries to purge

  Returns:
    ARP_RES_OK

  Example:
    None

  Remarks:
    None
*/
ARP_RESULT ARPCacheSetThreshold(TCPIP_NET_HANDLE hNet, int purgeThres, int purgeEntries);



#endif  // __ARP_H_



