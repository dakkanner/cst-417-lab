/*******************************************************************************
  Internet Protocol (IP) Version 6 Communications Layer

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Provides a transport for TCP, UDP, and ICMP messages
    -Reference: RFC 
*******************************************************************************/

/*******************************************************************************
FileName:   ipv6.c
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
#include "ipv6_private.h"

#include "tcpip_mac_private.h"

#if defined(TCPIP_STACK_USE_IPV6)

#define TCPIP_THIS_MODULE_ID    TCPIP_MODULE_IPV6
#include "tcpip_notify.h"

// This is left shifted by 4.  Actual value is 0x04.
#define IPv4_VERSION        (0x40u)
// This is left shifted by 4.  Actual value is 0x06.
#define IPv6_VERSION        (0x60u)

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

// The IPv6 unspecified address
const IPV6_ADDR IPV6_FIXED_ADDR_UNSPECIFIED = {{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
// The IPv6 all-nodes multicast addres
const IPV6_ADDR IPV6_FIXED_ADDR_ALL_NODES_MULTICAST = {{0xFF, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01}};
// The IPv6 all-routers multicast address
const IPV6_ADDR IPV6_FIXED_ADDR_ALL_ROUTER_MULTICAST = {{0xFF, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02}};

// The IPv6 solicited node multicast address mask
const IPV6_ADDR IPV6_SOLICITED_NODE_MULTICAST = {{0xFF, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0x00, 0x00, 0x00}};

static SystemTickHandle     ipv6InitTimerHandle = 0;      // Handle for the IPv6 initialization task timer
static int                  ipv6InitTickPending = 0;      // Pending flag for the IPv6 intialization task
static int                  ipv6InitCount = 0;            // Indicator of how many interfaces are initializing the IPv6 module

static uint32_t             fragmentId = 0;             // Static ID to use when sending fragmented packets

static int                  nStackIfs = 0;              // number of interfaces the stack is currently running on

static SINGLE_LIST          ipv6RegisteredUsers = { 0 };

static SINGLE_LIST          mcastQueue = { 0 };

static int                  ipv6ModuleInitCount = 0;      // Indicator of how many times the IP module has been initialized


static SystemTickHandle     ipv6TimerHandle = 0;          // Handle for the IP task timer
static int                  ipv6TickPending = 0;          // Pending flag for the IP task

static const void*          ipv6MemH = 0;                     // memory handle

SINGLE_LIST                 ipv6QueuedPackets = { 0 };

// Enumeration defining IPv6 initialization states
enum
{
    IPV6_INIT_STATE_NONE = 0,                           // IPv6 initialization is not in progress
    IPV6_INIT_STATE_INITIALIZE,                         // Initializes IPv6 variables
    IPV6_INIT_STATE_DAD,                                // Duplicate address detection is being performed on the interfaces link-local unicast address
    IPV6_INIT_STATE_SOLICIT_ROUTER,                     // The interface is soliciting routers
    IPV6_INIT_STATE_DONE,                               // The interface is up
    IPV6_INIT_STATE_FAIL                                // The initialization has failed
} IPV6_INIT_STATE;

// Index of address selection rules for IPv6 default address selection
typedef enum
{
    ADDR_INVALID = 0,
    ADDR_USER_SPECIFIED,
    ADDR_INVALID_ADDRESS_SPECIFIED,             
    ADDR_UNDEFINED,
    ADDR_STILL_VALID,
    ADDR_SEL_RULE_1,
    ADDR_SEL_RULE_2,
    ADDR_SEL_RULE_3,
    ADDR_SEL_RULE_4,
    ADDR_SEL_RULE_5,
    ADDR_SEL_RULE_6,
    ADDR_SEL_RULE_7,
    ADDR_SEL_RULE_8,
    ADDR_SEL_RULE_9,
    ADDR_SEL_RULE_10,
} IPV6_ADDR_SEL_INDEX;

// IPv6 address policy table for default address selection
const IPV6_ADDRESS_POLICY gPolicyTable[] = {
    {{{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01}},128,50,0},          // Loopback address
    {{{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},0,40,1},            // Unspecified address
    {{{0x20, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},16,30,2},           // 2002::/15 - 6to4
    {{{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},96,20,3},           // IPv4-compatible address
    {{{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00}},96,10,4},           // IPv4-mapped address
    {{{0x20, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},32,5,5},            // 2001::/32 - Teredo tunneling
    {{},0xFF,0,0},
    {{},0xFF,0,0},
    {{},0xFF,0,0},
    {{},0xFF,0,0},
};

// Array of global configuration and state variables for an IPv6 interface
static IPV6_INTERFACE_CONFIG* ipv6Config = 0;

/************************************************************************/
/****************               Prototypes               ****************/
/************************************************************************/

    // Free all of the dynamically linked lists in an IPv6 net configuration
    void IPv6FreeConfigLists (IPV6_INTERFACE_CONFIG * pNetIf);
    // Returns true if addressTwo is preferred over addressOne
    unsigned char IPv6ASCompareSourceAddresses(TCPIP_NET_IF * pNetIf, IPV6_ADDR_STRUCT * addressOne, IPV6_ADDR_STRUCT * addressTwo, IPV6_ADDR * dest, IPV6_ADDR_SEL_INDEX rule);
    // performs resources cleanup
    static void _TCPIP_IPV6_Cleanup(const void* memH);

static void TCPIP_IPV6_QueuedPacketTransmitTask (SINGLE_LIST* pList);

static void TCPIP_IPV6_QueuePacket(IPV6_PACKET * pkt, SINGLE_LIST* pList, int queueLimit, int tmoSeconds);

/*****************************************************************************
  Function:
	bool TCPIP_IPV6_Initialize(
        const TCPIP_STACK_MODULE_CTRL* const pStackInit, 
        const IPV6_MODULE_CONFIG* pIpv6Init)

  Summary:
	Initializes an IPv6 interface.

  Description:
	This function initializes an interface for IPv6 operation.  It will 
    create the task timer for the IP task and the IPv6 initialization task 
    (if it hasn't already been created by a previous call to this function). 

  Precondition:
	The IP module initialization function must have been called.

  Parameters:
	pStackInit - Stack initialization parameters
    pIpv6Init - Unused supplementary data field

  Returns:
  	true if interface is being initialized, false otherwise
  	
  Remarks:
	None
  ***************************************************************************/
bool TCPIP_IPV6_Initialize(const TCPIP_STACK_MODULE_CTRL* const pStackInit, const IPV6_MODULE_CONFIG* pIpv6Init)
{
    TCPIP_NET_IF* pNetIf;
    IPV6_INTERFACE_CONFIG*  pIpv6Config;

    if(pStackInit->stackAction == TCPIP_STACK_ACTION_IF_UP)
    {   // interface going up
        return true;
    }

    // stack initialization
    if(ipv6ModuleInitCount == 0)
    {   // 1st time we run

        SingleListInit (&mcastQueue);
        ipv6MemH = pStackInit->memH;
        SingleListInit (&ipv6QueuedPackets);
        // save the max number of interfaces the stack is working on
        nStackIfs = pStackInit->nIfs;

        // Initialize the global fragment ID
        fragmentId = 0;

        ipv6Config = (IPV6_INTERFACE_CONFIG*)TCPIP_HEAP_Calloc(pStackInit->memH, pStackInit->nIfs, sizeof(*ipv6Config));
        if(ipv6Config == 0)
        {   // failed
            return 0;
        }

        if (ipv6TimerHandle == 0)
        {
            ipv6TimerHandle = SYS_TICK_TimerCreate(TCPIP_IPV6_TmoHandler);
        }

        if (ipv6TimerHandle == 0)
        {
            _TCPIP_IPV6_Cleanup(pStackInit->memH);
            return false;
        }

        SYS_TICK_TimerSetRate (ipv6TimerHandle, ((SYS_TICK_ResolutionGet() * IPV6_TASK_PROCESS_RATE) + 999)/1000);
        ipv6TickPending = 0;

        if (ipv6InitTimerHandle == 0)
        {
            ipv6InitTimerHandle = SYS_TICK_TimerCreate(TCPIP_IPV6_InitTmo);
        }
        
        if (ipv6InitTimerHandle == 0)
        {
            _TCPIP_IPV6_Cleanup(pStackInit->memH);
            return false;
        }

        // Set the interrupt time for this task to 1/32 of a second
        SYS_TICK_TimerSetRate (ipv6InitTimerHandle, ((SYS_TICK_ResolutionGet() * IPV6_INIT_TASK_PROCESS_RATE) + 999)/1000);
        ipv6InitTickPending = 0;
    }
        

    // init this interface
    pNetIf = pStackInit->pNetIf;

    // Check to see if this interface is already being initialized
    if (pNetIf->Flags.bIPv6InConfig == false)
    {

        pNetIf->Flags.bIPv6InConfig = true;
        pIpv6Config = ipv6Config + pStackInit->netIx;

        // Initialize the IPv6 parameters for this net
        pIpv6Config->initState = IPV6_INIT_STATE_INITIALIZE;
        DoubleListInit (&pIpv6Config->listIpv6UnicastAddresses);
        DoubleListInit (&pIpv6Config->listIpv6MulticastAddresses);
        DoubleListInit (&pIpv6Config->listIpv6TentativeAddresses);
        SingleListInit (&pIpv6Config->listNeighborCache);
        SingleListInit (&pIpv6Config->listDefaultRouter);
        SingleListInit (&pIpv6Config->listDestinationCache);
        SingleListInit (&pIpv6Config->listPrefixList);
        SingleListInit (&pIpv6Config->rxFragments);

        pIpv6Config->currentDefaultRouter = NULL;
        pIpv6Config->baseReachableTime = IPV6_DEFAULT_BASE_REACHABLE_TIME;
        pIpv6Config->reachableTime = IPV6_DEFAULT_BASE_REACHABLE_TIME;
        pIpv6Config->retransmitTime = IPV6_DEFAULT_RETRANSMIT_TIME;
        pIpv6Config->linkMTU = IPV6_DEFAULT_LINK_MTU;
        pIpv6Config->multicastMTU = IPV6_DEFAULT_LINK_MTU;
        pIpv6Config->mtuIncreaseTimer = 0;
        pIpv6Config->curHopLimit = IPV6_DEFAULT_CUR_HOP_LIMIT;

        pIpv6Config->policyPreferTempOrPublic = IPV6_PREFER_PUBLIC_ADDRESSES;
    }

    ipv6InitCount++;
    ipv6ModuleInitCount++;
    return true;
}

static void _TCPIP_IPV6_Cleanup(const void* memH)
{
    if(ipv6TimerHandle)
    {
        SYS_TICK_TimerDelete(ipv6TimerHandle);
        ipv6TimerHandle = 0;
    }

    if(ipv6InitTimerHandle)
    {
        SYS_TICK_TimerDelete(ipv6InitTimerHandle);
        ipv6InitTimerHandle = 0;
    }

    if(ipv6Config)
    {
        TCPIP_HEAP_Free(memH, ipv6Config);
        ipv6Config = 0;
    }

    TCPIP_IPV6_SingleListFree (&ipv6QueuedPackets);
    ipv6MemH = 0;
}

/*****************************************************************************
  Function:
	void TCPIP_IPV6_InitTmo (SYS_TICK curSysTick)

  Summary:
	Timeout function for IPv6 initialization task.

  Description:
	This function will be called by the system timer to indicate to the 
    stack that the IPv6 initialization task function should be called.

  Precondition:
	One or more interfaces must be undergoing IPv6 initialization.

  Parameters:
	curSysTick - The current system tick.

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_IPV6_InitTmo (SYS_TICK curSysTick)
{
    ipv6InitTickPending++;
}

/*****************************************************************************
  Function:
	bool TCPIP_IPV6_InitTaskPending (void)

  Summary:
	Indicates that the IPv6 initialization task function must be called.

  Description:
	Indicates that the IPv6 initialization task function must be called.	

  Precondition:
	None

  Parameters:
	None

  Returns:
  	true if a task is pending, false otherwise
  	
  Remarks:
	None
  ***************************************************************************/
bool TCPIP_IPV6_InitTaskPending (void)
{
    return (ipv6InitTickPending == 0)?0:1;
}

/*****************************************************************************
  Function:
	void TCPIP_IPV6_Deinitialize(const TCPIP_STACK_MODULE_CTRL* const stackCtrl)

  Summary:
	Disables IPv6 functionality on the specified interface.

  Description:
	This function will disable IPv6 functionality on a specified interface. 
    It will free any dynamically allocated structures.

  Precondition:
	None

  Parameters:
	stackCtrl - Stack initialization parameters

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_IPV6_Deinitialize(const TCPIP_STACK_MODULE_CTRL* const stackCtrl)
{
    TCPIP_NET_IF* pNetIf;

    if(stackCtrl->stackAction == TCPIP_STACK_ACTION_IF_DOWN)
    {   // interface shutdown
        return ;
    }

    // stack shut down
    if(ipv6ModuleInitCount)
    {
        pNetIf = stackCtrl->pNetIf;
        TCPIP_IPV6_InitializeStop (pNetIf);

        IPv6FreeConfigLists (ipv6Config + stackCtrl->netIx);
        TCPIP_IPV6_SingleListFree (&mcastQueue);

        pNetIf->Flags.bIPv6Enabled = false;

        if(--ipv6ModuleInitCount == 0)
        {
            _TCPIP_IPV6_Cleanup(stackCtrl->memH);
            nStackIfs = 0;
        }
    }
}

/*****************************************************************************
  Function:
	void TCPIP_IPV6_InitializeStop (TCPIP_NET_IF * pNetIf)

  Summary:
	Stops IPv6 initialization

  Description:
	This function will halt IPv6 initialization for a specified interface. 
    If it determines that no interfaces are initializing IPv6, it will 
    destroy the IPv6 initialization task system timer.

  Precondition:
	None

  Parameters:
	pNetIf - The interface to top initialization for.

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_IPV6_InitializeStop (TCPIP_NET_IF * pNetIf)
{
    if(ipv6InitCount)
    {   // We're up and running
        if (pNetIf->Flags.bIPv6InConfig == true)
        {
            pNetIf->Flags.bIPv6InConfig = false;
            if (--ipv6InitCount == 0)
            {   // release
                if(ipv6InitTimerHandle)
                {
                    SYS_TICK_TimerDelete(ipv6InitTimerHandle);
                    ipv6InitTimerHandle = 0;
                    ipv6InitTickPending = 0;
                }
            }
        }
    }
}

/*****************************************************************************
  Function:
	void TCPIP_IPV6_InitializeTask (void)

  Summary:
	Task function for IPv6 initialization.

  Description:
    Task function for IPv6 initialization.

  Precondition:
	None

  Parameters:
	None

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_IPV6_InitializeTask (void)
{
    IPV6_ADDR_STRUCT * localAddressPointer;
    IPV6_ADDR linkLocalAddress = {{0xFE, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0 | 0x02, 0, 0, 0xFF, 0xFE, 0, 0, 0}};
    int netIx;
    TCPIP_NET_IF * pNetIf;
    IPV6_INTERFACE_CONFIG*  pIpv6Config;
    
    for (netIx = 0; netIx < TCPIP_STACK_NetworksNo(); netIx++)
    {
        pNetIf = (TCPIP_NET_IF*)TCPIP_STACK_IxToNet (netIx);
        pIpv6Config = ipv6Config + netIx;

        if ((pNetIf->Flags.bIPv6InConfig == true) && (_TCPIPStackIsNetLinked(pNetIf)))
        {
            linkLocalAddress.v[8] = pNetIf->netMACAddr.v[0] | 0x02;
            linkLocalAddress.v[9] = pNetIf->netMACAddr.v[1];
            linkLocalAddress.v[10] = pNetIf->netMACAddr.v[2];
            linkLocalAddress.v[13] = pNetIf->netMACAddr.v[3];
            linkLocalAddress.v[14] = pNetIf->netMACAddr.v[4];
            linkLocalAddress.v[15] = pNetIf->netMACAddr.v[5];

            switch (pIpv6Config->initState)
            {
                case IPV6_INIT_STATE_INITIALIZE:
                    // Add the all-nodes multicast listener to this node
                    localAddressPointer = TCPIP_IPV6_AddMulticastListener (pNetIf, (IPV6_ADDR *)&IPV6_FIXED_ADDR_ALL_NODES_MULTICAST);
                    if (localAddressPointer == NULL)
                    {
                        pIpv6Config->initState = IPV6_INIT_STATE_FAIL;
                        break;
                    }
                    // Configure link-local address
                    localAddressPointer = TCPIP_IPV6_AddUnicastAddress (pNetIf, &linkLocalAddress, false);
                    if (localAddressPointer == NULL)
                    {
                        pIpv6Config->initState = IPV6_INIT_STATE_FAIL;
                        break;
                    }
        
                    // Enable IPv6 functionality for now so we can process ICMPv6 messages for stateless address autoconfiguration
                    pNetIf->Flags.bIPv6Enabled = true;
                    pIpv6Config->initState = IPV6_INIT_STATE_DAD;
                    break;
                case IPV6_INIT_STATE_DAD:
                    if (TCPIP_IPV6_FindAddress (pNetIf, &linkLocalAddress, IPV6_ADDR_TYPE_UNICAST))
                    {
                        pIpv6Config->initState = IPV6_INIT_STATE_SOLICIT_ROUTER;
                    }
                    else if (TCPIP_IPV6_FindAddress (pNetIf, &linkLocalAddress, IPV6_ADDR_TYPE_UNICAST_TENTATIVE) == NULL)
                    {
                        pIpv6Config->initState = IPV6_INIT_STATE_FAIL;
                    }
                    break;
                case IPV6_INIT_STATE_SOLICIT_ROUTER:
                    TCPIP_NDP_RS_Start(pNetIf);
                    pIpv6Config->initState = IPV6_INIT_STATE_DONE;            
                case IPV6_INIT_STATE_DONE:
                    TCPIP_IPV6_InitializeStop (pNetIf);
                    break;
                case IPV6_INIT_STATE_FAIL:
                    IPv6FreeConfigLists (pIpv6Config);
                    TCPIP_IPV6_InitializeStop (pNetIf);
                    pNetIf->Flags.bIPv6Enabled = 0;
                    break;
                default:
                    break;
            }
        }
    }
}

/*****************************************************************************
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
  ***************************************************************************/
bool TCPIP_IPV6_InterfaceIsReady (TCPIP_NET_HANDLE netH)
{
    TCPIP_NET_IF * pNetIf =  _TCPIPStackHandleToNet(netH);

    if(pNetIf != 0 && pNetIf->Flags.bInterfaceEnabled)
    {
        if (!pNetIf->Flags.bIPv6InConfig && pNetIf->Flags.bIPv6Enabled)
        {
            return true;
        }
    }

    return false;
}




/*****************************************************************************
  Function:
	IPV6_PACKET * TCPIP_IPV6_AllocateTxPacket (TCPIP_NET_HANDLE netH, IPV6_PACKET_ACK_FNC ackFnc, void* ackParam);


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
                  TCPIP_IPV6_AllocateTxPacket

  Returns:
  	IPV6_PACKET * - Pointer to the allocated packet.
  	
  Remarks:
	None
  ***************************************************************************/
IPV6_PACKET * TCPIP_IPV6_AllocateTxPacket (TCPIP_NET_HANDLE netH, IPV6_PACKET_ACK_FNC ackFnc, void* ackParam)
{
    TCPIP_NET_IF * pNetIf =  _TCPIPStackHandleToNet(netH);
    IPV6_PACKET * ptrPacket = (IPV6_PACKET *)TCPIP_HEAP_Malloc (ipv6MemH, sizeof (IPV6_PACKET));

    if (ptrPacket == NULL)
        return NULL;

    ptrPacket->next = 0;
    ptrPacket->netIfH = pNetIf;
    ptrPacket->flags.val = 0;
    ptrPacket->upperLayerChecksumOffset = IPV6_NO_UPPER_LAYER_CHECKSUM;
    
    ptrPacket->payload.dataLocation = (uint8_t*)&ptrPacket->ipv6Header;
    ptrPacket->payload.segmentSize = sizeof (IPV6_HEADER);
    ptrPacket->payload.segmentLen = sizeof (IPV6_HEADER);
    ptrPacket->offsetInSegment = 0;

    ptrPacket->payload.memory = IPV6_DATA_PIC_RAM;
    ptrPacket->payload.segmentType = TYPE_IPV6_HEADER;
    ptrPacket->payload.nextSegment = NULL;

    TCPIP_IPV6_SetPacketIPProtocol (ptrPacket);

    ptrPacket->headerLen = 0;
    ptrPacket->flags.queued = false;
    ptrPacket->flags.sourceSpecified = false;
    ptrPacket->flags.useUnspecAddr = false;

    ptrPacket->neighbor = NULL;

    ptrPacket->payloadLen = 0;
    ptrPacket->upperLayerHeaderLen = 0;
    memset (&ptrPacket->remoteMACAddr, 0x00, sizeof (MAC_ADDR));

    ptrPacket->ackFnc = ackFnc;
    ptrPacket->ackParam = ackParam;

    return ptrPacket;
}

/*****************************************************************************
  Function:
	bool TCPIP_IPV6_CopyTxPacketStruct (IPV6_PACKET * destination, 
        IPV6_PACKET * source)

  Summary:
	Copies data from one IPV6_PACKET to another.

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
bool TCPIP_IPV6_CopyTxPacketStruct (IPV6_PACKET * destination, IPV6_PACKET * source)
{
    IPV6_DATA_SEGMENT_HEADER *ptrSegmentSource, *ptrSegmentDest;

    if (destination == NULL || source == NULL)
    {
        return false;
    }
    if(destination->flags.addressType != source->flags.addressType || destination->flags.addressType != IP_ADDRESS_TYPE_IPV6)
    {
        return false;
    }

    memcpy ((void *)&destination->remoteMACAddr, (void *)&source->remoteMACAddr, sizeof (MAC_ADDR));
    destination->flags.useUnspecAddr = source->flags.useUnspecAddr;
    destination->flags.sourceSpecified = source->flags.sourceSpecified;
    destination->flags.addressType = IP_ADDRESS_TYPE_IPV6;


    memcpy ((void *)&destination->payload, (void *)&source->payload, sizeof (IPV6_DATA_SEGMENT_HEADER));

    destination->payload.nextSegment = NULL;

    destination->neighbor = source->neighbor;
    memcpy ((void *)&destination->ipv6Header, (void *)&source->ipv6Header, sizeof (IPV6_HEADER));
    destination->ipv6Header.PayloadLength = 0x0000;

    destination->payload.dataLocation = (uint8_t*)&destination->ipv6Header;

    if ((ptrSegmentSource = TCPIP_IPV6_GetDataSegmentByType (source, TYPE_IPV6_UPPER_LAYER_HEADER)) != NULL)
    {
        if ((ptrSegmentDest = TCPIP_IPV6_GetDataSegmentByType (destination, ptrSegmentSource->segmentType)) == NULL)
        {
            ptrSegmentDest = TCPIP_IPV6_AllocateDataSegmentHeader (ptrSegmentSource->segmentSize);
            if (ptrSegmentDest == NULL)
                return false;
            ptrSegmentDest->nextSegment = NULL;
            TCPIP_IPV6_InsertIPPacketSegment (ptrSegmentDest, destination, ptrSegmentSource->segmentType);
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
	void TCPIP_IPV6_SetPacketIPProtocol (IPV6_PACKET * ptrPacket)
  Summary:
	Sets the packet IP protocol for a TX packet to IPv4 or IPv6.

  Description:
	Sets the packet IP protocol for a TX packet to IPv4 or IPv6.	

  Precondition:
	None

  Parameters:
	ptrPacket - Pointer to the target packet.

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_IPV6_SetPacketIPProtocol (IPV6_PACKET * ptrPacket)
{

    ptrPacket->payload.segmentLen = sizeof (IPV6_HEADER);
    ptrPacket->flags.addressType = IP_ADDRESS_TYPE_IPV6;
}

void TCPIP_IPV6_PutHeader(IPV6_PACKET * ptrPacket, uint8_t protocol)
{
    if(ptrPacket->flags.addressType != IP_ADDRESS_TYPE_IPV6)
    {
        return;
    }

    void * ptrSegment = TCPIP_IPV6_GetDataSegmentContentsByType (ptrPacket, TYPE_IPV6_HEADER);

    if (ptrSegment == NULL)
        return;


    ((IPV6_HEADER *)ptrSegment)->V_T_F = (uint32_t)IPv6_VERSION;
    ((IPV6_HEADER *)ptrSegment)->HopLimit = 0;
}

/*****************************************************************************
  Function:
	IPV6_DATA_SEGMENT_HEADER * TCPIP_IPV6_PutUpperLayerHeader (
        IPV6_PACKET * ptrPacket, void * header, unsigned short len, 
        unsigned char type, unsigned short checksumOffset)

  Summary:
	Adds an upper layer header to an IP TX packet.

  Description:
	This function will add an upper layer header to the chain of segments 
    in an IPV6_PACKET structure.  It will also initialize the packet's 
    upper-layer header variables.

  Precondition:
	None

  Parameters:
	ptrPacket - The packet that the upper layer header applies to.
    header - Pointer to the header data.
    len - Length of the header.
    type - Standard upper layer header type.
    checksumOffset - Offset of the upper-layer checksum in the header, or 
        IPV6_NO_UPPER_LAYER_CHECKSUM.

  Returns:
  	IPV6_DATA_SEGMENT_HEADER * - Pointer to the segment in the packet that 
        contains the header information.
  	
  Remarks:
	This function will automatically allocate memory to store the header data.
  ***************************************************************************/
IPV6_DATA_SEGMENT_HEADER * TCPIP_IPV6_PutUpperLayerHeader (IPV6_PACKET * ptrPacket, void * header, unsigned short len, unsigned char type, unsigned short checksumOffset)
{
    IPV6_DATA_SEGMENT_HEADER * ptrSegment = TCPIP_IPV6_AllocateDataSegmentHeader(len);

    if (ptrSegment == NULL)
        return NULL;

    ptrSegment->segmentLen = len;

    if (header != NULL)
        memcpy (ptrSegment->data, header, len);

    ptrPacket->upperLayerHeaderLen = len;
    ptrPacket->upperLayerHeaderType = type;
    ptrPacket->upperLayerChecksumOffset = checksumOffset;

    TCPIP_IPV6_InsertIPPacketSegment (ptrSegment, ptrPacket, TYPE_IPV6_UPPER_LAYER_HEADER);

    return ptrSegment;
}

/*****************************************************************************
  Function:
	void TCPIP_IPV6_InsertIPPacketSegment (IPV6_DATA_SEGMENT_HEADER * ptrSegment, 
        IPV6_PACKET * ptrPacket, IPV6_SEGMENT_TYPE type)

  Summary:
	Inserts a data segment into an IPV6_PACKET structure.

  Description:
	This function inserts a data segment into an IPV6_PACKET structure.  It will 
    update the next header fields in existing segments, if applicable.

  Precondition:
	None

  Parameters:
	ptrSegment - The segment to insert.
    ptrPacket - The packet to insert the segment into.
    type - The segment type.  Defined by IPV6_SEGMENT_TYPE enumeration.

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_IPV6_InsertIPPacketSegment (IPV6_DATA_SEGMENT_HEADER * ptrSegment, IPV6_PACKET * ptrPacket, IPV6_SEGMENT_TYPE type)
{
    IPV6_DATA_SEGMENT_HEADER * tempSegment;
    uint8_t * nextHeader;

    if ((ptrSegment == NULL) || (ptrPacket == NULL))
        return;

    tempSegment = &ptrPacket->payload; 

    ptrSegment->segmentType = type;

    if (type == TYPE_IPV6_END_OF_LIST)
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

    if (type == TYPE_IPV6_END_OF_LIST)
    {
        ptrSegment->segmentType = TYPE_IPV6_UPPER_LAYER_PAYLOAD;
    }

    if (tempSegment->segmentType < TYPE_IPV6_UPPER_LAYER_HEADER)
    {
        if (tempSegment == &ptrPacket->payload)
        {
            nextHeader = &ptrPacket->ipv6Header.NextHeader;
        }
        else
        {
            nextHeader = (uint8_t *)&tempSegment->data;
        }
        switch (ptrSegment->segmentType)
        {
            case TYPE_IPV6_EX_HEADER_HOP_BY_HOP_OPTIONS:
                *nextHeader = IPV6_PROT_HOP_BY_HOP_OPTIONS_HEADER;
                break;
            case TYPE_IPV6_EX_HEADER_DESTINATION_OPTIONS_1:
            case TYPE_IPV6_EX_HEADER_DESTINATION_OPTIONS_2:
                *nextHeader = IPV6_PROT_DESTINATION_OPTIONS_HEADER;
                break;
            case TYPE_IPV6_EX_HEADER_ROUTING:
                *nextHeader = IPV6_PROT_ROUTING_HEADER;
                break;
            case TYPE_IPV6_EX_HEADER_FRAGMENT:
                *nextHeader = IPV6_PROT_FRAGMENTATION_HEADER;
                break;
            case TYPE_IPV6_EX_HEADER_AUTHENTICATION_HEADER:
                *nextHeader = IPV6_PROT_AUTHENTICATION_HEADER;
                break;
            case TYPE_IPV6_EX_HEADER_ENCAPSULATING_SECURITY_PAYLOAD:
                *nextHeader = IPV6_PROT_ENCAPSULATING_SECURITY_PAYLOAD_HEADER;
                break;
            default:
                *nextHeader = ptrPacket->upperLayerHeaderType;
                break;
        }
    }

    if (ptrSegment->segmentType < TYPE_IPV6_UPPER_LAYER_HEADER)
    {
        nextHeader = (uint8_t *)&ptrSegment->data;
        if (ptrSegment->nextSegment == NULL)
        {
            *nextHeader = IPV6_PROT_NONE;
        }
        else
        {
            switch (ptrSegment->nextSegment->segmentType)
            {
                case TYPE_IPV6_EX_HEADER_HOP_BY_HOP_OPTIONS:
                    *nextHeader = IPV6_PROT_HOP_BY_HOP_OPTIONS_HEADER;
                    break;
                case TYPE_IPV6_EX_HEADER_DESTINATION_OPTIONS_1:
                case TYPE_IPV6_EX_HEADER_DESTINATION_OPTIONS_2:
                    *nextHeader = IPV6_PROT_DESTINATION_OPTIONS_HEADER;
                    break;
                case TYPE_IPV6_EX_HEADER_ROUTING:
                    *nextHeader = IPV6_PROT_ROUTING_HEADER;
                    break;
                case TYPE_IPV6_EX_HEADER_FRAGMENT:
                    *nextHeader = IPV6_PROT_FRAGMENTATION_HEADER;
                    break;
                case TYPE_IPV6_EX_HEADER_AUTHENTICATION_HEADER:
                    *nextHeader = IPV6_PROT_AUTHENTICATION_HEADER;
                    break;
                case TYPE_IPV6_EX_HEADER_ENCAPSULATING_SECURITY_PAYLOAD:
                    *nextHeader = IPV6_PROT_ENCAPSULATING_SECURITY_PAYLOAD_HEADER;
                    break;
                default:
                    *nextHeader = ptrPacket->upperLayerHeaderType;
                    break;
            }
        }
    }
}

/*****************************************************************************
  Function:
	IPV6_DATA_SEGMENT_HEADER * TCPIP_IPV6_GetDataSegmentByType (
        IPV6_PACKET * ptrPacket, IPV6_SEGMENT_TYPE type)

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
    type - IPV6_SEGMENT_TYPE segment type value to search for.

  Returns:
  	IPV6_DATA_SEGMENT_HEADER * - Pointer to the specified segment type, or NULL.
  	
  Remarks:
	There is a special IPV6_SEGMENT_TYPE value defined:
        - TYPE_IPV6_BEGINNING_OF_WRITABLE_PART searches for the first upper layer 
            payload segment to which data can be written.
  ***************************************************************************/
IPV6_DATA_SEGMENT_HEADER * TCPIP_IPV6_GetDataSegmentByType (IPV6_PACKET * ptrPacket, IPV6_SEGMENT_TYPE type)
{
    IPV6_DATA_SEGMENT_HEADER * ptrSegment;

    if (ptrPacket == NULL)
        return NULL;

    ptrSegment = &ptrPacket->payload;

    if (type != TYPE_IPV6_BEGINNING_OF_WRITABLE_PART)
    {
        while ((ptrSegment != NULL) && (ptrSegment->segmentType < type))
        {
            ptrSegment = ptrSegment->nextSegment;
        }
    }
    else
    {
        IPV6_DATA_SEGMENT_HEADER * ptrTempSegment;

        type = TYPE_IPV6_UPPER_LAYER_PAYLOAD;

        while ((ptrSegment != NULL) && (ptrSegment->segmentType < type))
        {
            ptrSegment = ptrSegment->nextSegment;
        }

        while (ptrSegment != NULL)
        {
            // Move ptrSegment to the next dynamically allocated segment
            while ((ptrSegment != NULL) && (ptrSegment->memory != IPV6_DATA_DYNAMIC_BUFFER))
                ptrSegment = ptrSegment->nextSegment;
            // Break out of the loop if the pointer gets to the end of the linked list
            if (ptrSegment == NULL)
                break;
    
            // Initialize ptrTempSegment to the first segment after the beginning of the dynamically allocated segments
            ptrTempSegment = ptrSegment->nextSegment;
            // Check to see if the dynamically allocated section is contiguous at the end of the linked list (or find the break)
            while ((ptrTempSegment != NULL) && (ptrTempSegment->memory == IPV6_DATA_DYNAMIC_BUFFER))
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
	IPV6_DATA_SEGMENT_HEADER * TCPIP_IPV6_GetDataSegmentContentsByType (
        IPV6_PACKET * ptrPacket, IPV6_SEGMENT_TYPE type)

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
    type - IPV6_SEGMENT_TYPE segment type value to search for.

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void * TCPIP_IPV6_GetDataSegmentContentsByType (IPV6_PACKET * ptrPacket, IPV6_SEGMENT_TYPE type)
{
    IPV6_DATA_SEGMENT_HEADER * ptr;

    ptr = TCPIP_IPV6_GetDataSegmentByType(ptrPacket, type);

    if (ptr != NULL)
    {
        return (void *)ptr->dataLocation;
    }
    else
        return NULL;
}

/*****************************************************************************
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
  ***************************************************************************/
unsigned short TCPIP_IPV6_IsTxPutReady (IPV6_PACKET * ptrPacket, unsigned short count)
{
    IPV6_DATA_SEGMENT_HEADER * ptrSegment;
    unsigned short payloadSize = 0;
    unsigned short availableSpace;

    if (ptrPacket == NULL)
        return 0;

    payloadSize = (count > IP_DEFAULT_ALLOCATION_BLOCK_SIZE)?count:IP_DEFAULT_ALLOCATION_BLOCK_SIZE;

    ptrSegment = TCPIP_IPV6_GetDataSegmentByType (ptrPacket, TYPE_IPV6_UPPER_LAYER_PAYLOAD);

    // Verify that there is a valid upper layer payload
    if (ptrSegment == NULL)
    {
        ptrSegment = TCPIP_IPV6_AllocateDataSegmentHeader(payloadSize);
        if (ptrSegment == NULL)
        {
            return 0;
        }

        TCPIP_IPV6_InsertIPPacketSegment (ptrSegment, ptrPacket, TYPE_IPV6_END_OF_LIST);

        return payloadSize;        
    }

    ptrSegment = TCPIP_IPV6_GetDataSegmentByType (ptrPacket, TYPE_IPV6_BEGINNING_OF_WRITABLE_PART);

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

    ptrSegment = TCPIP_IPV6_AllocateDataSegmentHeader(payloadSize);
    if (ptrSegment == NULL)
    {
        return 0;
    }

    TCPIP_IPV6_InsertIPPacketSegment (ptrSegment, ptrPacket, TYPE_IPV6_END_OF_LIST);

    return availableSpace + payloadSize ;        
}

/*****************************************************************************
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
  ***************************************************************************/
bool TCPIP_IPV6_Put (IPV6_PACKET * pkt, unsigned char v)
{
    return (TCPIP_IPV6_PutArray (pkt, &v, 1) == 1)?true:false;
}

/*****************************************************************************
  Function:
	unsigned short TCPIP_IPV6_PutArrayHelper (IPV6_PACKET * ptrPacket, 
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
unsigned short TCPIP_IPV6_PutArrayHelper (IPV6_PACKET * ptrPacket, const void * dataSource, uint8_t dataType, unsigned short len)
{
    IPV6_DATA_SEGMENT_HEADER * ptrSegment;
    uint8_t * dataLocation;
    unsigned short txSize = 0;

    if (ptrPacket == NULL)
        return 0;

    ptrSegment = TCPIP_IPV6_GetDataSegmentByType (ptrPacket, TYPE_IPV6_BEGINNING_OF_WRITABLE_PART);

    if (ptrSegment == NULL)
        return 0;

    dataLocation = (uint8_t *)ptrSegment->dataLocation;

    if (ptrSegment->segmentLen + len > ptrSegment->segmentSize)
    {
        txSize = ptrSegment->segmentSize - ptrSegment->segmentLen;
        if (dataType == IPV6_DATA_PIC_RAM)
            memcpy ((void *)(dataLocation + ptrSegment->segmentLen), dataSource, txSize);
        else if (dataType == IPV6_DATA_NETWORK_FIFO)
            MACGetArray(*((TCPIP_MAC_HANDLE *)dataSource), dataLocation + ptrSegment->segmentLen, txSize);
        else
            return 0;
        ptrSegment->segmentLen += txSize;
        ptrPacket->payloadLen += len;

        ptrSegment = TCPIP_IPV6_AllocateDataSegmentHeader(len - txSize);
        if (ptrSegment == NULL)
            return txSize;

        TCPIP_IPV6_InsertIPPacketSegment (ptrSegment, ptrPacket, TYPE_IPV6_END_OF_LIST);
        dataLocation = (uint8_t *)ptrSegment->dataLocation;
        len -= txSize;
    }

    if ((ptrSegment->segmentLen + len <= ptrSegment->segmentSize))
    {
        if (dataType == IPV6_DATA_PIC_RAM)
            memcpy ((void *)(dataLocation + ptrSegment->segmentLen), dataSource, len);
        else if (dataType == IPV6_DATA_NETWORK_FIFO)
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
	unsigned short TCPIP_IPV6_SetPayload (IPV6_PACKET * ptrPacket, 
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
unsigned short TCPIP_IPV6_SetPayload (IPV6_PACKET * ptrPacket, uint8_t* payload, unsigned short len)
{
    IPV6_DATA_SEGMENT_HEADER * ptrSegment;

    ptrSegment = TCPIP_IPV6_AllocateDataSegmentHeader (0);
    if(ptrSegment == 0)
    {
        return 0;
    }

    ptrSegment->nextSegment = NULL;
    ptrSegment->dataLocation = payload;
    ptrSegment->segmentSize = len;
    ptrSegment->segmentLen = len;
    ptrSegment->memory = IPV6_DATA_PIC_RAM;

    TCPIP_IPV6_InsertIPPacketSegment (ptrSegment, ptrPacket, TYPE_IPV6_END_OF_LIST);

    ptrPacket->payloadLen += len;

    return len;
}

/*****************************************************************************
  Function:
	unsigned short TCPIP_IPV6_GetPseudoHeaderChecksum (IPV6_PACKET * ptrPacket)

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
unsigned short TCPIP_IPV6_GetPseudoHeaderChecksum (IPV6_PACKET * ptrPacket)
{
    if (ptrPacket->flags.addressType == IP_ADDRESS_TYPE_IPV6)
    {
        IPV6_PSEUDO_HEADER pseudoHeader;

        pseudoHeader.zero1 = 0;
        pseudoHeader.zero2 = 0;
        pseudoHeader.PacketLength = TCPIP_HELPER_ntohs (ptrPacket->upperLayerHeaderLen + ptrPacket->payloadLen);
        pseudoHeader.NextHeader = ptrPacket->upperLayerHeaderType;
        memcpy((void *)&pseudoHeader.SourceAddress, (void *)TCPIP_IPV6_GetSourceAddress (ptrPacket), sizeof (IPV6_ADDR));
        memcpy((void *)&pseudoHeader.DestAddress, (void *)TCPIP_IPV6_GetDestAddress (ptrPacket), sizeof (IPV6_ADDR));
        return TCPIP_Helper_CalcIPChecksum ((void *)&pseudoHeader, sizeof (pseudoHeader), 0);
    }

    return 0;
}

/*****************************************************************************
  Function:
	void TCPIP_IPV6_SetHopLimit(IPV6_PACKET * ptrPacket, uint8_t hopLimit)

  Summary:
	Sets the hop limit of a TX packet to a specific value.

  Description:
	Sets the hop limit of a TX packet to a specific value.	

  Precondition:
	None

  Parameters:
	ptrPacket - The TX packet.
    hopLimit - The new hop limit value.

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_IPV6_SetHopLimit(IPV6_PACKET * ptrPacket, uint8_t hopLimit)
{
    IPV6_HEADER * ptrHeader = TCPIP_IPV6_GetDataSegmentContentsByType (ptrPacket, TYPE_IPV6_HEADER);
    ptrHeader->HopLimit = hopLimit;
}

/*****************************************************************************
  Function:
	unsigned short TCPIP_IPV6_CalculatePayloadChecksum (IPV6_PACKET * ptrPacket)

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
unsigned short TCPIP_IPV6_CalculatePayloadChecksum (IPV6_PACKET * ptrPacket)
{
    IPV6_DATA_SEGMENT_HEADER * ptrSegment;
    TCPIP_UINT32_VAL checksum;
    TCPIP_MAC_HANDLE hMac;
    uint16_t tempChecksum = 0;
    unsigned short checksumByteCount = 0;

    ptrSegment = TCPIP_IPV6_GetDataSegmentByType (ptrPacket, TYPE_IPV6_UPPER_LAYER_HEADER);

    checksum.Val = 0;
    if (ptrSegment != NULL)
    {
        checksum.w[0] = ~TCPIP_Helper_CalcIPChecksum((uint8_t *)ptrSegment->dataLocation, ptrPacket->upperLayerHeaderLen, 0);
        checksumByteCount += ptrPacket->upperLayerHeaderLen;
    }

    ptrSegment = TCPIP_IPV6_GetDataSegmentByType (ptrPacket, TYPE_IPV6_UPPER_LAYER_PAYLOAD);

    while (ptrSegment != NULL)
    {
        switch (ptrSegment->memory)
        {
            case IPV6_DATA_DYNAMIC_BUFFER:
            case IPV6_DATA_PIC_RAM:
                tempChecksum = ~TCPIP_Helper_CalcIPChecksum((uint8_t *)ptrSegment->dataLocation, ptrSegment->segmentLen, 0);
                if (checksumByteCount % 2)
                {
                    tempChecksum = TCPIP_HELPER_htons(tempChecksum);
                }
                checksumByteCount += ptrSegment->segmentLen;
                break;
            case IPV6_DATA_NETWORK_FIFO:
                hMac = _TCPIPStackNetToMac ((TCPIP_NET_IF*)ptrPacket->netIfH);
                MACSetReadPtr (hMac, (TCPIP_MAC_PTR_TYPE)ptrSegment->dataLocation);
                tempChecksum = ~MACCalcIPBufferChecksum(hMac, ptrSegment->segmentLen);
                if (checksumByteCount % 2)
                {
                    tempChecksum = TCPIP_HELPER_htons(tempChecksum);
                }
                checksumByteCount += ptrSegment->segmentLen;
                break;
            case IPV6_DATA_NONE:
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
	void TCPIP_IPV6_ResetTransmitPacketState (IPV6_PACKET * pkt)

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
void TCPIP_IPV6_ResetTransmitPacketState (IPV6_PACKET * pkt)
{
    IPV6_DATA_SEGMENT_HEADER * segmentHeader = &pkt->payload;
    IPV6_DATA_SEGMENT_HEADER * segmentHeader2 = 0;

    while ((segmentHeader != NULL) && (segmentHeader->segmentType != TYPE_IPV6_UPPER_LAYER_PAYLOAD))
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
        TCPIP_HEAP_Free (ipv6MemH, segmentHeader);
        segmentHeader = segmentHeader2;
    }

    pkt->flags.queued = false;
    pkt->payloadLen = 0;
    pkt->offsetInSegment = 0;
}

/*****************************************************************************
  Function:
	bool TCPIP_IPV6_AddressIsSolicitedNodeMulticast (IPV6_ADDR * address)

  Summary:
	Determines if the specified address is a solicited node multicast address.

  Description:
	Determines if the specified address is a solicited node multicast address.

  Precondition:
	None

  Parameters:
	address - The address.

  Returns:
  	bool - true if the address is a solciited-node multicast address, false 
        otherwise
  	
  Remarks:
	None
  ***************************************************************************/
bool TCPIP_IPV6_AddressIsSolicitedNodeMulticast (IPV6_ADDR * address)
{
    uint8_t solNodeMulticastFragment[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0x00, 0x00, 0x00};

    if ((address->v[0] == 0xFF) && ((address->v[1] & 0x02) == 0x02) && (memcmp (solNodeMulticastFragment, &address->v[2], sizeof (IPV6_ADDR) - 5) == 0))
    {
        return true;
    }
    return false;
}

/*****************************************************************************
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
  ***************************************************************************/
bool TCPIP_IPV6_Flush (IPV6_PACKET * ptrPacket, MAC_ADDR * remoteMACAddr)
{
    IPV6_HEAP_NDP_NC_ENTRY * neighborPointer = 0;
    uint16_t *  checksumPointer;
    bool        toTxPkt = false;
    bool        mcastPkt = false;

    if(ptrPacket == 0 || ptrPacket->flags.addressType != IP_ADDRESS_TYPE_IPV6)
    {
        return false;
    }

    // Write the Ethernet Header to the MAC TX Buffer
    IPV6_ADDR_STRUCT * sourceAddress;
    IPV6_ADDR * destinationAddress = TCPIP_IPV6_GetDestAddress(ptrPacket);
    IPV6_HEADER * ptrIpHeader = TCPIP_IPV6_GetDataSegmentContentsByType (ptrPacket, TYPE_IPV6_HEADER);

    if (ptrPacket->headerLen == 0u)
    {
        ptrIpHeader->NextHeader = ptrPacket->upperLayerHeaderType; 
    }

    ptrIpHeader->PayloadLength = TCPIP_HELPER_htons (ptrPacket->payloadLen + ptrPacket->upperLayerHeaderLen + ptrPacket->headerLen);

    if (ptrIpHeader->HopLimit == 0)
        ptrIpHeader->HopLimit = (ipv6Config + _TCPIPStackNetIx((TCPIP_NET_IF*)ptrPacket->netIfH))->curHopLimit;

    // Check to see if a source address was specified
    if (ptrPacket->flags.sourceSpecified)
    {
        if (ptrPacket->flags.useUnspecAddr)
        {
            TCPIP_IPV6_SetSourceAddress(ptrPacket, (IPV6_ADDR *)&IPV6_FIXED_ADDR_UNSPECIFIED);                    
            ptrPacket->flags.sourceSpecified = true;
        }
        else
        {
            sourceAddress = TCPIP_IPV6_DAS_SelectSourceAddress (ptrPacket->netIfH, destinationAddress, &ptrIpHeader->SourceAddress);
            if (sourceAddress == NULL)
            {
                ptrPacket->flags.sourceSpecified = false;
            }
        }
    }

    if (!ptrPacket->flags.sourceSpecified)
    {
        sourceAddress = TCPIP_IPV6_DAS_SelectSourceAddress (ptrPacket->netIfH, destinationAddress, NULL);
        if (sourceAddress != NULL)
        {
            TCPIP_IPV6_SetSourceAddress(ptrPacket, &(sourceAddress->address));
        }
        else
        {
            // This should never happen; we should always have at least our link-local auto-configured address
            TCPIP_IPV6_SetSourceAddress(ptrPacket, (IPV6_ADDR *)&IPV6_FIXED_ADDR_UNSPECIFIED);
        }
    }

    if (ptrPacket->upperLayerChecksumOffset != IPV6_NO_UPPER_LAYER_CHECKSUM)
    {
        checksumPointer = TCPIP_IPV6_GetDataSegmentContentsByType (ptrPacket, TYPE_IPV6_UPPER_LAYER_HEADER);
        if (checksumPointer != NULL)
        {
            checksumPointer = (uint16_t *)(((uint8_t *)checksumPointer) + ptrPacket->upperLayerChecksumOffset);
            *checksumPointer = ~TCPIP_IPV6_GetPseudoHeaderChecksum (ptrPacket);
            *checksumPointer = TCPIP_IPV6_CalculatePayloadChecksum (ptrPacket);
        }
    }

    if (destinationAddress->v[0] == 0xFF)
    {
        mcastPkt = true;
        // Determine the appropriate link-local address for a multicast address
        ptrPacket->remoteMACAddr.v[0] = 0x33;
        ptrPacket->remoteMACAddr.v[1] = 0x33;
        ptrPacket->remoteMACAddr.v[2] = destinationAddress->v[12];
        ptrPacket->remoteMACAddr.v[3] = destinationAddress->v[13];
        ptrPacket->remoteMACAddr.v[4] = destinationAddress->v[14];
        ptrPacket->remoteMACAddr.v[5] = destinationAddress->v[15];

        toTxPkt = true;
    }
    else
    {
        // Determine the appropriate neighbor to transmit the unicast packet to
        if ((neighborPointer = ptrPacket->neighbor) == NULL)
        {
            neighborPointer = TCPIP_NDP_GetNextHop ((TCPIP_NET_IF*)ptrPacket->netIfH, destinationAddress);
        }

        if (neighborPointer == NULL)
        {
            // The device could not determine a next hop address
            return false;
        }

        ptrPacket->neighbor = neighborPointer;

        switch (neighborPointer->reachabilityState)
        {
            case NDP_STATE_STALE:
                TCPIP_NDP_SetReachability ((TCPIP_NET_IF*)ptrPacket->netIfH, neighborPointer, NDP_STATE_DELAY);\
                    // Fall through
            case NDP_STATE_REACHABLE:
            case NDP_STATE_DELAY:
            case NDP_STATE_PROBE:
                    memcpy (&ptrPacket->remoteMACAddr, &neighborPointer->remoteMACAddr, sizeof (MAC_ADDR));
                    toTxPkt = true;
                    break;
            case NDP_STATE_INCOMPLETE:
                    TCPIP_NDP_ResolveAddress (neighborPointer);
                    TCPIP_IPV6_QueuePacket(ptrPacket, &neighborPointer->queuedPackets, IPV6_QUEUE_NEIGHBOR_PACKET_LIMIT, 0);
                    return false;
            default:
                    TCPIP_NDP_SetReachability ((TCPIP_NET_IF*)ptrPacket->netIfH, neighborPointer, NDP_STATE_INCOMPLETE);
                    TCPIP_NDP_ResolveAddress (neighborPointer);
                    TCPIP_IPV6_QueuePacket(ptrPacket, &neighborPointer->queuedPackets, IPV6_QUEUE_NEIGHBOR_PACKET_LIMIT, 0);
                    return false;
        }
    }

    if(toTxPkt)
    {   // packet needs to be transmitted

        if(TCPIP_IPV6_TransmitPacket (ptrPacket))
        {   // success
            if (ptrPacket->ackFnc)
            {
                (*ptrPacket->ackFnc)(ptrPacket, true, ptrPacket->ackParam);
            }
            return true;
        }
        else
        {
            if(mcastPkt)
            {
                TCPIP_IPV6_QueuePacket(ptrPacket, &mcastQueue, IPV6_QUEUE_MCAST_PACKET_LIMIT, IPV6_QUEUED_MCAST_PACKET_TIMEOUT);
            }
            else
            {
                TCPIP_IPV6_QueuePacket(ptrPacket, &neighborPointer->queuedPackets, IPV6_QUEUE_NEIGHBOR_PACKET_LIMIT, 0);
            }
        }
    }

    return false;
}

/*****************************************************************************
  Function:
	bool TCPIP_IPV6_TransmitPacket (IPV6_PACKET * pkt)

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
bool TCPIP_IPV6_TransmitPacket (IPV6_PACKET * pkt)
{
    TCPIP_MAC_HANDLE hMac;
    uint16_t mtu;
    IPV6_HEAP_NDP_DC_ENTRY * ptrDestination;
 
    if (pkt->flags.addressType != IP_ADDRESS_TYPE_IPV6)
    {
        return false;
    }

    hMac = _TCPIPStackNetToMac ((TCPIP_NET_IF*)pkt->netIfH);
    
    if (!_TCPIPStackIsNetLinked((TCPIP_NET_IF*)pkt->netIfH))
    {
        return false;
    }

    if (!TCPIP_IPV6_IsTxReady((TCPIP_NET_IF*)pkt->netIfH))
    {
        return false;
    }

    if (pkt->ipv6Header.DestAddress.v[0] == 0xFF)
    {
        mtu = (ipv6Config + _TCPIPStackNetIx((TCPIP_NET_IF*)pkt->netIfH))->multicastMTU;
    }
    else
    {
        ptrDestination = TCPIP_NDP_FindRemoteNode ((TCPIP_NET_IF*)pkt->netIfH, &pkt->ipv6Header.DestAddress, IPV6_HEAP_NDP_DC_ID);
        if (ptrDestination)
        {
            mtu = ptrDestination->pathMTU;
        }
        else
        {
            mtu = (ipv6Config + _TCPIPStackNetIx((TCPIP_NET_IF*)pkt->netIfH))->linkMTU;
        }
    }

    if (TCPIP_HELPER_htons(pkt->ipv6Header.PayloadLength) + sizeof (IPV6_HEADER) + pkt->headerLen > mtu)
    {
        return TCPIP_IPV6_TransmitPacketInFragments(pkt, mtu);
    }


    // Initialize MAC TX Write Pointer
    MACSetWritePtr (hMac, MACGetTxBaseAddr(hMac));

    // Write the Ethernet Header to the MAC TX Buffer
    MACPutHeader (hMac, &pkt->remoteMACAddr, ETHERTYPE_IPV6, sizeof (IPV6_HEADER) + pkt->upperLayerHeaderLen + pkt->headerLen + pkt->payloadLen);

    // Write the payload data to the MAC
    TCPIP_IPV6_FlushDataSegments(hMac, pkt);

    // Transmit the packet
    MACFlush (hMac);

    pkt->flags.queued = false;

    return true;
}

/*****************************************************************************
  Function:
	bool TCPIP_IPV6_TransmitPacketInFragments (IPV6_PACKET * pkt, uint16_t mtu)

  Summary:
	Transmits a packet in fragments across a link with an MTU less than the 
    packet size.

  Description:
	Transmits a packet in fragments across a link with an MTU less than the 
    packet size.	

  Precondition:
	None

  Parameters:
	pkt - The packet
    mtu - The link MTU.

  Returns:
  	bool - true if the packet was transmitted, false otherwise.
  	
  Remarks:
	None
  ***************************************************************************/
bool TCPIP_IPV6_TransmitPacketInFragments (IPV6_PACKET * pkt, uint16_t mtu)
{
    TCPIP_MAC_HANDLE hMac;
    IPV6_FRAGMENT_HEADER * ptrFragmentHeader = 0;
    IPV6_DATA_SEGMENT_HEADER * ptrSegment;
    IPV6_DATA_SEGMENT_HEADER * ptrFragmentablePart;
    uint16_t currentPayloadLen;
    uint16_t sentPayloadLen = pkt->offsetInSegment;
    uint16_t totalPayloadLen;
    uint16_t unfragmentableLen = 0;
    uint16_t temp;
    uint16_t offsetInSegment = 0;

    hMac = _TCPIPStackNetToMac ((TCPIP_NET_IF*)pkt->netIfH);

    if (!_TCPIPStackIsNetLinked((TCPIP_NET_IF*)pkt->netIfH))
    {
        // Discard the packet if this interface isn't even linked
        return false;
    }
    if (!TCPIP_IPV6_IsTxReady((TCPIP_NET_IF*)pkt->netIfH))
    {
        // Try to queue this packet
        // If it has already been queued, the Queue function will skip it
        return false;
    }

    if ((ptrSegment = TCPIP_IPV6_GetDataSegmentContentsByType(pkt, TYPE_IPV6_EX_HEADER_FRAGMENT)) == NULL)
    {

        ptrSegment = TCPIP_IPV6_AllocateDataSegmentHeader (sizeof (IPV6_FRAGMENT_HEADER));

        if (ptrSegment == NULL)
        {
            return false;
        }

        ptrSegment->segmentLen = sizeof (IPV6_FRAGMENT_HEADER);

        TCPIP_IPV6_InsertIPPacketSegment (ptrSegment, pkt, TYPE_IPV6_EX_HEADER_FRAGMENT);

        ptrFragmentHeader = (IPV6_FRAGMENT_HEADER *)ptrSegment->data;

        ptrFragmentHeader->reserved = 0;
        ptrFragmentHeader->offsetM.bits.reserved2 = 0;
        ptrFragmentHeader->offsetM.bits.m = 0;
        ptrFragmentHeader->offsetM.bits.fragmentOffset = 0;
        ptrFragmentHeader->identification = TCPIP_HELPER_htonl(fragmentId);
        fragmentId++;
        pkt->headerLen += sizeof (IPV6_FRAGMENT_HEADER);
    }

    totalPayloadLen = pkt->payloadLen + pkt->headerLen + pkt->upperLayerHeaderLen + sizeof (IPV6_HEADER);

    // If a router specified that the path MTU is less than the minimum link MTU 
    // we just need to add a fragmentation header to the packet
    if (mtu < IPV6_MINIMUM_LINK_MTU)
    {
        if (totalPayloadLen < IPV6_DEFAULT_LINK_MTU)
            mtu = IPV6_DEFAULT_LINK_MTU;
    }

    ptrSegment = &pkt->payload;
    while ((ptrSegment != NULL) && (ptrSegment->segmentType <= TYPE_IPV6_EX_HEADER_FRAGMENT))
    {
        unfragmentableLen += ptrSegment->segmentLen;
        ptrSegment = ptrSegment->nextSegment;
    }

    ptrFragmentablePart = ptrSegment;

    do
    {
        currentPayloadLen = unfragmentableLen;
        ptrSegment = ptrFragmentablePart;

        // Determine the length of the current payload
        while (ptrSegment != NULL)
        {
            if (currentPayloadLen + (ptrSegment->segmentLen - offsetInSegment) <= mtu)
            {
                currentPayloadLen += (ptrSegment->segmentLen - offsetInSegment);
            }
            else
            {
                if (mtu - currentPayloadLen > 8)
                {
                    currentPayloadLen = mtu - (mtu & 0b111);
                }
                else
                {
                    if (currentPayloadLen % 8 != 0)
                    {
                        if ((ptrSegment->segmentLen - offsetInSegment) > (8 - (currentPayloadLen & 0b111)))
                        {
                            currentPayloadLen += (8 - (currentPayloadLen & 0b111));
                        }
                        else
                        {
                            currentPayloadLen -= (currentPayloadLen & 0b111);
                        }
                    }
                }
                break;
            }
            ptrSegment = ptrSegment->nextSegment;
        }

        // Set M flag
        if (sentPayloadLen + currentPayloadLen == totalPayloadLen)
        {
            ptrFragmentHeader->offsetM.bits.m = 0;
        }
        else
        {
            ptrFragmentHeader->offsetM.bits.m = 1;
        }

        // Set fragment offset
        ptrFragmentHeader->offsetM.bits.fragmentOffset = sentPayloadLen >> 3;

        ptrFragmentHeader->offsetM.w = TCPIP_HELPER_htons(ptrFragmentHeader->offsetM.w);

        // Calculate new payload length        
        pkt->ipv6Header.PayloadLength = TCPIP_HELPER_htons(currentPayloadLen - sizeof (IPV6_HEADER));

        if (!MACIsTxReady (hMac))
        {
            pkt->offsetInSegment = sentPayloadLen;
            return false;
        }

        // Initialize MAC TX Write Pointer
        MACSetWritePtr (hMac, MACGetTxBaseAddr(hMac));

        // Write the Ethernet Header to the MAC TX Buffer
        MACPutHeader (hMac, &pkt->remoteMACAddr, ETHERTYPE_IPV6, currentPayloadLen);

        // Write the unfragmentable part
        ptrSegment = &pkt->payload;
        temp = unfragmentableLen;
        while (temp)
        {
            if (ptrSegment->segmentLen < temp)
            {
                MACPutArray(hMac, (uint8_t *)ptrSegment->dataLocation, ptrSegment->segmentLen);
                temp -= ptrSegment->segmentLen;
                ptrSegment = ptrSegment->nextSegment;
            }
            else
            {
                MACPutArray(hMac, (uint8_t *)ptrSegment->dataLocation, temp);
                break;
            }
        }

        // Write the fragmentable part
        ptrSegment = ptrFragmentablePart;
        temp = currentPayloadLen - unfragmentableLen;
        while (temp)
        {
            if (ptrSegment->segmentLen < temp)
            {
                MACPutArray(hMac, (uint8_t *)ptrSegment->dataLocation + offsetInSegment, ptrSegment->segmentLen);
                temp -= ptrSegment->segmentLen;
                ptrSegment = ptrSegment->nextSegment;
                offsetInSegment = 0;
            }
            else
            {
                MACPutArray(hMac, (uint8_t *)ptrSegment->dataLocation + offsetInSegment, temp);
                if (temp == ptrSegment->segmentLen)
                {
                    ptrSegment = ptrSegment->nextSegment;
                    offsetInSegment = 0;
                }
                else
                {
                    offsetInSegment = temp;
                }
                break;
            }
        }
        ptrFragmentablePart = ptrSegment;

        // Transmit the packet
        MACFlush (hMac);

        sentPayloadLen += currentPayloadLen - unfragmentableLen;
    } while (sentPayloadLen + unfragmentableLen != totalPayloadLen);

    pkt->flags.queued = false;

    return true;
}


/*****************************************************************************
  Function:
	void TCPIP_IPV6_FlushDataSegments (TCPIP_MAC_HANDLE hMac, 
        IPV6_PACKET * ptrPacket)

  Summary:
	Helper function for TCPIP_IPV6_TransmitPacket that writes all segments in 
    the packet data segment chain to the MAC layer.

  Description:
	Helper function for TCPIP_IPV6_TransmitPacket that writes all segments in 
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
void TCPIP_IPV6_FlushDataSegments (TCPIP_MAC_HANDLE hMac, IPV6_PACKET * ptrPacket)
{
    IPV6_DATA_SEGMENT_HEADER * segmentHeader = &ptrPacket->payload;

    while (segmentHeader != NULL)
    {
        MACPutArray(hMac, (uint8_t *)segmentHeader->dataLocation, segmentHeader->segmentLen);
        segmentHeader = segmentHeader->nextSegment;
    }
}

/*****************************************************************************
  Function:
	IPV6_DATA_SEGMENT_HEADER * TCPIP_IPV6_AllocateDataSegmentHeader (uint16_t len)

  Summary:
	Allocates a data segment header and an optional payload

  Description:
	Allocates a data segment header and an optional payload	

  Precondition:
	None

  Parameters:
	len - Length of the optional dynamic payload to allocate for this segment.

  Returns:
  	IPV6_DATA_SEGMENT_HEADER * - Pointer to the new segment.
  	
  Remarks:
	None
  ***************************************************************************/
IPV6_DATA_SEGMENT_HEADER * TCPIP_IPV6_AllocateDataSegmentHeader (uint16_t len)
{
    IPV6_DATA_SEGMENT_HEADER * ptrSegment = (IPV6_DATA_SEGMENT_HEADER *)TCPIP_HEAP_Malloc (ipv6MemH, sizeof (IPV6_DATA_SEGMENT_HEADER) + len);

    if (ptrSegment == NULL)
    {
        return NULL;
    }

    if (len)
    {
        ptrSegment->dataLocation = (uint8_t*)ptrSegment->data;
        ptrSegment->segmentSize = len;
        ptrSegment->segmentLen = 0;
        ptrSegment->memory = IPV6_DATA_DYNAMIC_BUFFER;
    }
    else
    {
        ptrSegment->memory = IPV6_DATA_NONE;
        ptrSegment->dataLocation = (uint8_t*)NULL;
        ptrSegment->segmentSize = 0;
        ptrSegment->segmentLen = 0;
    }
    ptrSegment->nextSegment = NULL;

    return ptrSegment;
}

/*****************************************************************************
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
  ***************************************************************************/
void TCPIP_IPV6_FreePacket (IPV6_PACKET * ptrPacket)
{
    if (ptrPacket == NULL)
        return;

    TCPIP_IPV6_FreePacketData (ptrPacket);
    TCPIP_HEAP_Free (ipv6MemH, ptrPacket);
}

/*****************************************************************************
  Function:
	void TCPIP_IPV6_FreePacketData (IPV6_PACKET * ptrPacket)

  Summary:
	Frees all dynamically allocated structures used by an IPV6_PACKET struct.

  Description:
	Frees all dynamically allocated structures used by an IPV6_PACKET struct.

  Precondition:
	None

  Parameters:
	ptrPacket - The packet to free.

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_IPV6_FreePacketData (IPV6_PACKET * ptrPacket)
{
    IPV6_DATA_SEGMENT_HEADER * ptrSegment;
    IPV6_DATA_SEGMENT_HEADER * ptrSegment2;

    if (ptrPacket == NULL)
        return;

    // Set the initial segment to the segment after the IP header (which shouldn't be deallocated
    ptrSegment = ptrPacket->payload.nextSegment;

    while (ptrSegment != NULL)
    {
        ptrSegment2 = ptrSegment->nextSegment;
        TCPIP_HEAP_Free (ipv6MemH, ptrSegment);
        ptrSegment = ptrSegment2;
    }
}

void TCPIP_IPV6_SetTxBuffer(TCPIP_NET_IF* pNet, uint16_t offset)
{
    TCPIP_MAC_HANDLE hMac = _TCPIPStackNetToMac(pNet);

    MACSetWritePtr(hMac, offset + MACGetTxBaseAddr(hMac) + sizeof(MAC_ETHERNET_HEADER) + sizeof(IPV4_HEADER));
}


/*****************************************************************************
  Function:
	bool TCPIP_IPV6_GetHeader(TCPIP_NET_IF * pNetIf, IPV6_ADDR * localIPAddr, 
        IPV6_ADDR * remoteIPAddr, uint8_t *protocol, uint16_t *len, 
        uint8_t * hopLimit)

  Summary:
	Reads IPv6 header information from the MAC.

  Description:
	Reads IPv6 header information from the MAC.

  Precondition:
	None

  Parameters:
	pNetIf - The interface to check for a header.
    localIPAddr - The incoming packet's destination address.
    remoteIPAddr - The incoming packet's source address.
    protocol - Return value for the next header protocol.
    len - Return value for the payload length.
    hopLimit - Return value for the hop limit of the packet

  Returns:
  	bool - True if a packet header was read, false otherwise.
  	
  Remarks:
	None
  ***************************************************************************/
bool TCPIP_IPV6_GetHeader(TCPIP_NET_IF * pNetIf, IPV6_ADDR * localIPAddr, IPV6_ADDR * remoteIPAddr, uint8_t *protocol, uint16_t *len, uint8_t * hopLimit)
{
    IPV6_HEADER header;
    TCPIP_MAC_HANDLE hMac;

    hMac = _TCPIPStackNetToMac(pNetIf);

    // Read IP header.
    MACGetArray(hMac, (uint8_t*)&header, sizeof(header));

    // Make sure that this is an IPv6 packet.
    if ((header.V_T_F & 0x000000F0) != IPv6_VERSION)
    	return false;

    *hopLimit = header.HopLimit;

    *len = TCPIP_HELPER_ntohs(header.PayloadLength);

    *protocol = header.NextHeader;

    memcpy(localIPAddr, &header.DestAddress, sizeof (IPV6_ADDR));

    memcpy(remoteIPAddr, &header.SourceAddress, sizeof (IPV6_ADDR));

    return true;
}


/*****************************************************************************
  Function:
    void TCPIP_IPV6_SetRxBuffer(TCPIP_NET_IF* pNet, uint16_t Offset) 

  Summary:
	Sets the read pointer for the current RX buffer (IPv6 version).

  Description:
	Sets the read pointer for the current RX buffer to the specified offset
    after an IPv6 header.	

  Precondition:
	None

  Parameters:
	pNet - The interface to set the pointer for.
    Offset - The offset to set.

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_IPV6_SetRxBuffer(TCPIP_NET_IF* pNet, uint16_t Offset) 
{
	MACSetReadPtrInRx(_TCPIPStackNetToMac(pNet), Offset + sizeof (IPV6_HEADER));
}


/*****************************************************************************
  Function:
	void IPv6FreeConfigLists (IPV6_INTERFACE_CONFIG * pNetIf)

  Summary:
	Frees all dynamically allocated structures from linked lists used 
    for IPv6 configuration on an interface.

  Description:
	Frees all dynamically allocated structures from linked lists used 
    for IPv6 configuration on an interface.	

  Precondition:
	None

  Parameters:
	pNetIf - The interface.

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void IPv6FreeConfigLists (IPV6_INTERFACE_CONFIG * pNetIf)
{
    TCPIP_IPV6_SingleListFree(&pNetIf->listDestinationCache);
    TCPIP_IPV6_SingleListFree(&pNetIf->listNeighborCache);
    TCPIP_IPV6_SingleListFree(&pNetIf->listDefaultRouter);
    TCPIP_IPV6_SingleListFree(&pNetIf->listPrefixList);
    TCPIP_IPV6_SingleListFree(&pNetIf->rxFragments);
    TCPIP_IPV6_DoubleListFree(&pNetIf->listIpv6UnicastAddresses);
    TCPIP_IPV6_DoubleListFree(&pNetIf->listIpv6MulticastAddresses);
    TCPIP_IPV6_DoubleListFree(&pNetIf->listIpv6TentativeAddresses);
}

/*****************************************************************************
  Function:
	IPV6_ADDR_STRUCT * TCPIP_IPV6_FindAddress(TCPIP_NET_IF * pNetIf, IPV6_ADDR * addr, 
        unsigned char listType)

  Summary:
	Finds a local IPv6 address.

  Description:
	This function finds a local address in the list of tentative, unicast, 
    or multicast addresses.

  Precondition:
	None

  Parameters:
	pNetIf - The interface to check for the address.
    addr - The address to check for.
    listType - IPV6_ADDR_TYPE_UNICAST_TENTATIVE, IPV6_ADDR_TYPE_UNICAST, 
            IPV6_ADDR_TYPE_MULTICAST

  Returns:
  	IPV6_ADDR_STRUCT * - Pointer to the found address, or NULL
  	
  Remarks:
	None
  ***************************************************************************/
IPV6_ADDR_STRUCT * TCPIP_IPV6_FindAddress(TCPIP_NET_IF * pNetIf, IPV6_ADDR * addr, unsigned char listType)
{
    IPV6_ADDR_STRUCT * nextEntryPointer; 
    IPV6_INTERFACE_CONFIG*  pIpv6Config;

    if (!pNetIf)
    {
        return NULL;
    }
    else
    {
        pIpv6Config = ipv6Config + _TCPIPStackNetIx (pNetIf);
    }

    switch (listType)
    {
        case IPV6_ADDR_TYPE_UNICAST_TENTATIVE:
            nextEntryPointer = (IPV6_ADDR_STRUCT *)pIpv6Config->listIpv6TentativeAddresses.head;
            break;
        case IPV6_ADDR_TYPE_UNICAST:
            nextEntryPointer = (IPV6_ADDR_STRUCT *)pIpv6Config->listIpv6UnicastAddresses.head;
            break;
        case IPV6_ADDR_TYPE_MULTICAST:
            nextEntryPointer = (IPV6_ADDR_STRUCT *)pIpv6Config->listIpv6MulticastAddresses.head;
            break;

        default:
            nextEntryPointer = 0;
            break;
    }

    while (nextEntryPointer != NULL)
    {
        if (!memcmp (addr, &(nextEntryPointer->address), sizeof (IPV6_ADDR)))
        {
            return nextEntryPointer;
        }
        nextEntryPointer = nextEntryPointer->next;
    }
    return NULL;
}

/*****************************************************************************
  Function:
    IPV6_ADDR_STRUCT * TCPIP_IPV6_FindSolicitedNodeMulticastAddress(
        TCPIP_NET_IF * pNetIf, IPV6_ADDR * addr, unsigned char listType)

  Summary:
    Finds a unicast address based on a given solicited-node multicast address

  Description:
    Finds a unicast address based on a given solicited-node multicast address

  Precondition:
	None

  Parameters:
	pNetIf - The interface to check for the address.
    addr - The address to check for.
    listType - IPV6_ADDR_TYPE_UNICAST_TENTATIVE or IPV6_ADDR_TYPE_UNICAST

  Returns:
  	IPV6_ADDR_STRUCT * - Pointer to the found address, or NULL
  	
  Remarks:
	None
  ***************************************************************************/
IPV6_ADDR_STRUCT * TCPIP_IPV6_FindSolicitedNodeMulticastAddress(TCPIP_NET_IF * pNetIf, IPV6_ADDR * addr, unsigned char listType)
{
    IPV6_ADDR_STRUCT * nextEntryPointer; 
    IPV6_INTERFACE_CONFIG*  pIpv6Config;

    if (!pNetIf)
    {
        return NULL;
    }
    else
    {
        pIpv6Config = ipv6Config + _TCPIPStackNetIx (pNetIf);
    }

    switch (listType)
    {
        case IPV6_ADDR_TYPE_UNICAST_TENTATIVE:
            nextEntryPointer = (IPV6_ADDR_STRUCT *)pIpv6Config->listIpv6TentativeAddresses.head;
            break;
        case IPV6_ADDR_TYPE_UNICAST:
            nextEntryPointer = (IPV6_ADDR_STRUCT *)pIpv6Config->listIpv6UnicastAddresses.head;
            break;

        default:
            nextEntryPointer = 0;
    }

    while (nextEntryPointer != NULL)
    {
        if (!memcmp (&addr->v[13], &(nextEntryPointer->address.v[13]), 3))
        {
            return nextEntryPointer;
        }
        nextEntryPointer = nextEntryPointer->next;
    }
    return NULL;
}

/*****************************************************************************
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
  ***************************************************************************/
IPV6_ADDR_STRUCT * TCPIP_IPV6_AddMulticastListener (TCPIP_NET_HANDLE hNet, IPV6_ADDR * address)
{
    IPV6_ADDR_STRUCT * entryLocation;
    IPV6_ADDRESS_TYPE addressType;
    TCPIP_NET_IF* pNetIf = (TCPIP_NET_IF*)hNet;
    
    if (!pNetIf)
    {
        return NULL;
    }

    if ((entryLocation = TCPIP_IPV6_FindAddress (pNetIf, address, IPV6_ADDR_TYPE_MULTICAST)) == NULL)
    {
        entryLocation = (IPV6_ADDR_STRUCT *) TCPIP_HEAP_Malloc (ipv6MemH, sizeof (IPV6_ADDR_STRUCT));

        if (entryLocation != NULL)
        {
            memcpy (&entryLocation->address, address, sizeof (IPV6_ADDR));
            entryLocation->flags.type = IPV6_ADDR_TYPE_MULTICAST;
            addressType = TCPIP_IPV6_GetAddressType(pNetIf, address);
            entryLocation->flags.scope = addressType.bits.scope;
            TCPIP_NDP_LinkedListEntryInsert (pNetIf, entryLocation, IPV6_HEAP_ADDR_MULTICAST_ID);
            TCPIP_IPV6_NotifyClients(pNetIf, IPV6_EVENT_ADDRESS_ADDED);
        }
    }

    return entryLocation;
}

/*****************************************************************************
  Function:
	void TCPIP_IPV6_QueuePacket (IPV6_PACKET * pkt, SINGLE_LIST* pList, int queueLimit, int tmoSeconds)

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
static void TCPIP_IPV6_QueuePacket(IPV6_PACKET * pkt, SINGLE_LIST* pList, int queueLimit, int tmoSeconds)
{
    IPV6_PACKET * ptrPacket;

    ptrPacket = (IPV6_PACKET *)pList->head;

    while ((ptrPacket != NULL) && (ptrPacket != pkt))
    {
        ptrPacket = ptrPacket->next;
    }

    if (ptrPacket == NULL)
    {
        if (pList->nNodes == queueLimit)
        {
            ptrPacket = (IPV6_PACKET *)SingleListRemoveHead (pList);
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
	IPV6_ADDRESS_TYPE TCPIP_IPV6_GetAddressType (TCPIP_NET_IF * pNetIf, 
        IPV6_ADDR * address)

  Summary:
	Gets the scope and type of an IPv6 address.

  Description:
	Gets the scope and type of an IPv6 address.	

  Precondition:
	None

  Parameters:
	pNetIf - The interface (used to check for link-locality)
    address - The address to check.

  Returns:
  	IPV6_ADDRESS_TYPE - The address scope and type
  	
  Remarks:
	None
  ***************************************************************************/
IPV6_ADDRESS_TYPE TCPIP_IPV6_GetAddressType (TCPIP_NET_IF * pNetIf, IPV6_ADDR * address)
{
    uint8_t b;
    IPV6_ADDRESS_TYPE returnVal;

    if (address->v[0] == 0xFF)
    {
        // First byte == 0xFF -> multicast
        returnVal.bits.type = IPV6_ADDR_TYPE_MULTICAST;
        b = address->v[1];
        b &= 0x0F;
        switch (b)
        {
            case 1:
            case 2:
            case 4:
            case 5:
            case 8:
            case 0x0E:
                returnVal.bits.scope = b;
                break;
            default:
                returnVal.bits.scope = IPV6_ADDR_SCOPE_UNKNOWN;
                break;
        }
    }
    else
    {
        // First byte != 0xFF -> unicast or anycast
        // Impossible to determine if it's an anycast addr unless
        // it's specifically identified as one somewhere
        returnVal.bits.type = IPV6_ADDR_TYPE_UNICAST;

        if (((address->v[0] == 0xFE) && (address->v[1] == 0x80)) || TCPIP_NDP_PrefixOnLinkStatus(pNetIf, address))
        {
            returnVal.bits.scope = IPV6_ADDR_SCOPE_LINK_LOCAL;
        }
        // Compare to loopback address
        else if ((address->d[0] == 0x00000000) && (address->d[1] == 0x00000000) && (address->d[2] == 0x00000000) && (address->v[12] == 0x00) && \
                     (address->v[13] == 0x00) && (address->v[14] == 0x00) && (address->v[15] == 0x01))
        {
            // This counts as link-local unicast
            returnVal.bits.scope = IPV6_ADDR_SCOPE_INTERFACE_LOCAL;
        }
        else
        {
            returnVal.bits.scope = IPV6_ADDR_SCOPE_GLOBAL;
        }
    }

    return returnVal;
}

/*****************************************************************************
  Function:
	void IPv6RemoveUnicastAddress (TCPIP_NET_HANDLE netH, IPV6_ADDR * address)

  Summary:
	Removed a configured unicast address from an interface.

  Description:
	Removed a configured unicast address from an interface.	

  Precondition:
	None

  Parameters:
	netH    - The interface.
    address - The address

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void IPv6RemoveUnicastAddress (TCPIP_NET_HANDLE netH, IPV6_ADDR * address)
{
    IPV6_ADDR_STRUCT * entryLocation;
    TCPIP_NET_IF * pNetIf =  _TCPIPStackHandleToNet(netH);

    if(pNetIf)
    {

        entryLocation = TCPIP_IPV6_FindAddress (pNetIf, address, IPV6_ADDR_TYPE_UNICAST);
        if (entryLocation != NULL)
        {
            TCPIP_NDP_LinkedListEntryRemove (pNetIf, entryLocation, IPV6_HEAP_ADDR_UNICAST_ID);
            TCPIP_IPV6_NotifyClients(pNetIf, IPV6_EVENT_ADDRESS_REMOVED);
        }
    }
}

/*****************************************************************************
  Function:
	void IPv6RemoveMulticastListener (TCPIP_NET_HANDLE netH, IPV6_ADDR * address)

  Summary:
	Removes a multicast listener from a given interface.

  Description:
	Removes a multicast listener from a given interface.	

  Precondition:
	None

  Parameters:
	netH  - The interface
    address - The address

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void IPv6RemoveMulticastListener (TCPIP_NET_HANDLE netH, IPV6_ADDR * address)
{
    IPV6_ADDR_STRUCT * entryLocation;
    TCPIP_NET_IF * pNetIf =  _TCPIPStackHandleToNet(netH);

    if(pNetIf != 0)
    {
        entryLocation = TCPIP_IPV6_FindAddress (pNetIf, address, IPV6_ADDR_TYPE_MULTICAST);
        if (entryLocation != NULL)
        {
            TCPIP_NDP_LinkedListEntryRemove (pNetIf, entryLocation, IPV6_HEAP_ADDR_MULTICAST_ID);
            TCPIP_IPV6_NotifyClients(pNetIf, IPV6_EVENT_ADDRESS_REMOVED);
        }
    }

}

/*****************************************************************************
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
	netH            - The interface to add the address to.
    address         - The address to add.
    skipProcessing  - true to skip Duplicate address detection, false otherwise

  Returns:
  	IPV6_ADDR_STRUCT * - Pointer to the structure of the newly allocated
        address
   
  Remarks:
	None
  ***************************************************************************/
IPV6_ADDR_STRUCT * TCPIP_IPV6_AddUnicastAddress (TCPIP_NET_HANDLE netH, IPV6_ADDR * address, uint8_t skipProcessing)
{
    IPV6_ADDR_STRUCT * entryLocation = 0;
    unsigned char label, precedence, prefixLen;
    IPV6_ADDRESS_TYPE i;
    TCPIP_NET_IF * pNetIf =  _TCPIPStackHandleToNet(netH);

    if(pNetIf)
    {

        if (((entryLocation = TCPIP_IPV6_FindAddress (pNetIf, address, IPV6_ADDR_TYPE_UNICAST)) == NULL) && 
                ((entryLocation = TCPIP_IPV6_FindAddress (pNetIf, address, IPV6_ADDR_TYPE_UNICAST_TENTATIVE)) == NULL))
        {
            entryLocation = (IPV6_ADDR_STRUCT *) TCPIP_HEAP_Malloc (ipv6MemH, sizeof (IPV6_ADDR_STRUCT));

            if (entryLocation != NULL)
            {
                memcpy (&entryLocation->address, address, sizeof (IPV6_ADDR));
                i = TCPIP_IPV6_GetAddressType (pNetIf, address);
                entryLocation->flags.type = i.bits.type;
                entryLocation->flags.scope = i.bits.scope;
                if (TCPIP_IPV6_DAS_GetPolicy(address, &label, &precedence, &prefixLen))
                {
                    entryLocation->flags.precedence = precedence;
                    entryLocation->flags.label = label & 0x0F;
                }
                else
                {
                    entryLocation->flags.precedence = 0x00;
                    entryLocation->flags.label = 0xF;                    
                }
                entryLocation->flags.temporary = 0;

                // Set the Stateless Address Autoconfiguration variables to default values.
                // The Stateless Address AutoConfiguration function will set it to something else
                // if necessary.
                entryLocation->validLifetime = 0xFFFFFFFF;
                entryLocation->preferredLifetime = 0xFFFFFFFF;
                entryLocation->prefixLen = 0;
                // The skipProcessing flag indicates that the address doesn't need duplicate address
                // detection or an associated solicited node multicast address.
                // This can be used to add loopback addresses, for example.
                if (!skipProcessing)
                {
                    TCPIP_NDP_LinkedListEntryInsert (pNetIf, entryLocation, IPV6_HEAP_ADDR_UNICAST_TENTATIVE_ID);
                    if (TCPIP_NDP_DAD_DetectDuplicateAddress (pNetIf, entryLocation) == -1)
                    {
                        TCPIP_NDP_LinkedListEntryRemove (pNetIf, entryLocation, IPV6_HEAP_ADDR_UNICAST_TENTATIVE_ID);
                        entryLocation = NULL;
                    }
                }
                else
                {
                    TCPIP_NDP_LinkedListEntryInsert (pNetIf, entryLocation, IPV6_HEAP_ADDR_UNICAST_ID);
                    TCPIP_IPV6_NotifyClients(pNetIf, IPV6_EVENT_ADDRESS_ADDED);
                }
            }
        }
    }

    return entryLocation;
}

/*****************************************************************************
  Function:
	uint8_t TCPIP_IPV6_ProcessHopByHopOptionsHeader (TCPIP_NET_IF * pNetIf, 
        uint8_t * nextHeader, uint16_t * length)

  Summary:
	Processes an IPv6 Hop-by-hop Extension header.

  Description:
	Processes an IPv6 Hop-by-hop Extension header.	

  Precondition:
	None

  Parameters:
	pNetIf - The interface the packet with this header was received on.
    nextHeader - Return value for the next header.
    length - Return value for the header length.

  Returns:
  	uint8_t - An action code for the TCPIP_IPV6_Process function.
  	
  Remarks:
	None
  ***************************************************************************/
uint8_t TCPIP_IPV6_ProcessHopByHopOptionsHeader (TCPIP_NET_IF * pNetIf, uint8_t * nextHeader, uint16_t * length)
{
    uint8_t headerLength;
    uint8_t option;
    uint8_t optionLength;
    uint8_t data[8];
    uint8_t i;
    uint8_t j;
    TCPIP_MAC_HANDLE hMac = _TCPIPStackNetToMac (pNetIf);

    TCPIP_IPV6_GetOptionHeader (hMac, data, 1);

    *nextHeader = data[0];
    headerLength = data[1] + 1;
    *length = (uint16_t)headerLength << 3;

    option = data[2];

    i = 3;

    do
    {
        switch (option)
        {
            case IPV6_TLV_PAD_1:
                // If this option is present, let the post-switch code load new
                // data if necessary and get the next option
                break;
            case IPV6_TLV_PAD_N:
                optionLength = data[i++];
                if (optionLength <= (8-i))
                {
                    i+= optionLength;
                }
                else
                {
                    optionLength -= (8-i);
                    j = optionLength >> 3;
                    headerLength -= j;
                    while (j--)
                        TCPIP_IPV6_GetOptionHeader (hMac, data, 1);
                    optionLength -= (j << 3);
                    i = optionLength;
                }
                break;
            case IPV6_TLV_HBHO_PAYLOAD_JUMBOGRAM:
                break;
            case IPV6_TLV_HBHO_ROUTER_ALERT:
                break;
            default:
                switch (((IPV6_TLV_OPTION_TYPE)option).bits.unrecognizedAction)
                {
                    case IPV6_TLV_UNREC_OPT_SKIP_OPTION:
                        // Ignore this option (treat it like a padding option)
                        optionLength = data[i++];
                        if (optionLength <= (8-i))
                        {
                            i+= optionLength;
                        }
                        else
                        {
                            optionLength -= (8-i);
                            j = optionLength >> 3;
                            headerLength -= j;
                            while (j--)
                                TCPIP_IPV6_GetOptionHeader (hMac, data, 1);
                            optionLength -= (j << 3);
                            i = optionLength;
                        }
                        break;
                    case IPV6_TLV_UNREC_OPT_DISCARD_SILENT:
                        *length = *length - (headerLength << 3) + --i;
                        return IPV6_ACTION_DISCARD_SILENT;
                        break;
                    case IPV6_TLV_UNREC_OPT_DISCARD_PP:
                        *length = *length - (headerLength << 3) + --i;
                        return IPV6_ACTION_DISCARD_PP_2;
                        break;
                    case IPV6_TLV_UNREC_OPT_DISCARD_PP_NOT_MC:
                        *length = *length - (headerLength << 3) + --i;
                        return IPV6_ACTION_DISCARD_PP_2_NOT_MC;
                        break;
                }
                break;
        }
        if (i == 8)
        {
            headerLength--;
            i = 0;
            if (headerLength != 0)
                TCPIP_IPV6_GetOptionHeader (hMac, data, 1);
        }
        option = data[i++];
    }while (headerLength);

    return IPV6_ACTION_NONE;
}

/*****************************************************************************
  Function:
	uint8_t TCPIP_IPV6_ProcessDestinationOptionsHeader (TCPIP_NET_IF * pNetIf, 
        uint8_t * nextHeader, uint16_t * length)

  Summary:
	Processes an IPv6 Destination Extension header.

  Description:
	Processes an IPv6 Destination Extension header.	

  Precondition:
	None

  Parameters:
	pNetIf - The interface the packet with this header was received on.
    nextHeader - Return value for the next header.
    length - Return value for the header length.

  Returns:
  	uint8_t - An action code for the TCPIP_IPV6_Process function.
  	
  Remarks:
	None
  ***************************************************************************/
uint8_t TCPIP_IPV6_ProcessDestinationOptionsHeader (TCPIP_NET_IF * pNetIf, uint8_t * nextHeader, uint16_t * length)
{
    uint8_t headerLength;
    uint8_t option;
    uint8_t optionLength;
    uint8_t data[8];
    uint8_t i;
    uint8_t j;
    TCPIP_MAC_HANDLE hMac = _TCPIPStackNetToMac (pNetIf);

    TCPIP_IPV6_GetOptionHeader (hMac, data, 1);

    *nextHeader = data[0];
    headerLength = data[1] + 1;
    *length = (uint16_t)headerLength << 3;

    option = data[2];

    i = 3;

    do
    {
        switch (option)
        {
            // There are only two options current defined, and they're padding options
            case IPV6_TLV_PAD_1:
                // If this option is present, let the post-switch code load new
                // data if necessary and get the next option
                break;
            case IPV6_TLV_PAD_N:
                optionLength = data[i++];
                if (optionLength <= (8-i))
                {
                    i+= optionLength;
                }
                else
                {
                    optionLength -= (8-i);
                    j = optionLength >> 3;
                    headerLength -= (j + 1);
                    while (j--)
                        TCPIP_IPV6_GetOptionHeader (hMac, data, 1);
                    optionLength -= (j << 3);
                    i = optionLength;
                }
                break;
            default:
                switch (((IPV6_TLV_OPTION_TYPE)option).bits.unrecognizedAction)
                {
                    case IPV6_TLV_UNREC_OPT_SKIP_OPTION:
                        // Ignore this option (treat it like a padding option)
                        optionLength = data[i++];
                        if (optionLength <= (8-i))
                        {
                            i+= optionLength;
                        }
                        else
                        {
                            optionLength -= (8-i);
                            j = optionLength >> 3;
                            headerLength -= (j + 1);
                            while (j--)
                                TCPIP_IPV6_GetOptionHeader (hMac, data, 1);
                            optionLength -= (j << 3);
                            i = optionLength;
                        }
                        break;
                    case IPV6_TLV_UNREC_OPT_DISCARD_SILENT:
                        // Return the offset of the error in this header in the headerLength parameter
                        *length = *length - (headerLength << 3) + --i;
                        return IPV6_ACTION_DISCARD_SILENT;
                        break;
                    case IPV6_TLV_UNREC_OPT_DISCARD_PP:
                        // Return the offset of the error in this header in the headerLength parameter
                        *length = *length - (headerLength << 3) + --i;
                        return IPV6_ACTION_DISCARD_PP_2;
                        break;
                    case IPV6_TLV_UNREC_OPT_DISCARD_PP_NOT_MC:
                        // Return the offset of the error in this header in the headerLength parameter
                        *length = *length - (headerLength << 3) + --i;
                        return IPV6_ACTION_DISCARD_PP_2_NOT_MC;
                        break;
                }
                break;
        }
        if (i == 8)
        {
            headerLength--;
            i = 0;
            if (headerLength != 0)
                TCPIP_IPV6_GetOptionHeader (hMac, data, 1);
        }
        option = data[i++];
    }while (headerLength);

    return IPV6_ACTION_NONE;
}

/*****************************************************************************
  Function:
	void TCPIP_IPV6_FreeFragmentBuffer (void * ptrFragment)

  Summary:
	Frees a fragment processing buffer.

  Description:
	Frees a fragment processing buffer.	

  Precondition:
	None

  Parameters:
    ptrFragment - The fragment to free.

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_IPV6_FreeFragmentBuffer (void * ptrFragment)
{
    if (ptrFragment == NULL)
        return;

    TCPIP_HEAP_Free (ipv6MemH, ptrFragment);
}

/*****************************************************************************
  Function:
    uint8_t TCPIP_IPV6_ProcessFragmentationHeader (TCPIP_NET_IF * pNetIf, 
        IPV6_ADDR * remoteIP, IPV6_ADDR * localIP, uint8_t * nextHeader, 
        uint16_t dataCount, uint16_t headerLen, MAC_ADDR * remoteMACAddr, 
        uint16_t previousHeaderLen)

  Summary:
	Processes an IPv6 Fragmentation Extension header.

  Description:
	Processes an IPv6 Fragmentation Extension header.  This will usually results
    in the packet data being cached in a fragment buffer.

  Precondition:
	None

  Parameters:
	pNetIf - The interface the packet with this header was received on.
    remoteIP - The packet's source IP address
    localIP - The packet's destination IP address
    nextHeader - Return value for the next header
    dataCount - Length of the fragment header and everything after it
    headerLen - Length of all headers before the fragmentation header (including
        the IPv6 header)
    remoteMACAddr - The sender's MAC address
    previousHeaderLen - Length of the previous extension header (for finding 
        next header value)

  Returns:
  	uint8_t - An action code for the TCPIP_IPV6_Process function.
  	
  Remarks:
	None
  ***************************************************************************/
uint8_t TCPIP_IPV6_ProcessFragmentationHeader (TCPIP_NET_IF * pNetIf, IPV6_ADDR * remoteIP, IPV6_ADDR * localIP, uint8_t * nextHeader, uint16_t dataCount, uint16_t headerLen, MAC_ADDR * remoteMACAddr, uint16_t previousHeaderLen)
{
    IPV6_RX_FRAGMENT_BUFFER * ptrFragment;
    IPV6_FRAGMENT_HEADER fragmentHeader;
    TCPIP_MAC_HANDLE  hMac;
    IPV6_INTERFACE_CONFIG*  pIpv6Config;
    
    
    if ((pNetIf == NULL) || (dataCount < sizeof (IPV6_FRAGMENT_HEADER)))
        return IPV6_ACTION_DISCARD_SILENT;

    pIpv6Config = ipv6Config + _TCPIPStackNetIx (pNetIf);
    hMac = _TCPIPStackNetToMac(pNetIf);

    TCPIP_IPV6_GetArray(hMac, (void *)&fragmentHeader, 8);
    dataCount -= sizeof (IPV6_FRAGMENT_HEADER);

    fragmentHeader.offsetM.w = TCPIP_HELPER_ntohs (fragmentHeader.offsetM.w);

    // Set fragment buffer pointer to the head of the linked list of fragmented packets    
    ptrFragment = (IPV6_RX_FRAGMENT_BUFFER *)pIpv6Config->rxFragments.head;

    // Find a packet being reassembled that matches this fragmented packet
    while (ptrFragment != NULL)
    {
        if (ptrFragment->identification == fragmentHeader.identification)
        {
            if ((!memcmp (ptrFragment->packet + IPV6_HEADER_OFFSET_SOURCE_ADDR, remoteIP, sizeof (IPV6_ADDR))) &&
                (!memcmp (ptrFragment->packet + IPV6_HEADER_OFFSET_DEST_ADDR, localIP, sizeof (IPV6_ADDR))))
            {
                break;
            }
        }
        ptrFragment = ptrFragment->next;
    }

    // If no existing fragment was found, this is the first fragment in the packet.
    // Create a fragment buffer for it and store the unfragmentable part.
    if (ptrFragment == NULL)
    {
        ptrFragment = (IPV6_RX_FRAGMENT_BUFFER *)TCPIP_HEAP_Malloc (ipv6MemH, sizeof (IPV6_RX_FRAGMENT_BUFFER));

        if (ptrFragment == NULL)
            return IPV6_ACTION_DISCARD_SILENT;

        ptrFragment->next = NULL;
        // The RFC specifies that the fragments must be reassembled in one minute or less
        ptrFragment->secondsRemaining = 60;;
        ptrFragment->identification = fragmentHeader.identification;
        ptrFragment->packetSize = headerLen;
        ptrFragment->bytesInPacket = headerLen;
        ptrFragment->firstFragmentLength = 0;

        // Reset the packet's read pointer
        MACSetReadPtrInRx(hMac, 0);

        // Copy the unfragmentable part of the packet data into the fragment buffer
        MACGetArray(hMac, ptrFragment->packet, headerLen);

        // Set the packet's read pointer to skip the fragment header
        MACSetReadPtrInRx(hMac, headerLen + sizeof (IPV6_FRAGMENT_HEADER));

        if (headerLen == sizeof (IPV6_HEADER))
            ptrFragment->packet[IPV6_HEADER_OFFSET_NEXT_HEADER] = fragmentHeader.nextHeader;
        else
            ptrFragment->packet[headerLen - previousHeaderLen] = fragmentHeader.nextHeader;

        SingleListAddTail(&pIpv6Config->rxFragments, (SGL_LIST_NODE *)ptrFragment);

    }

    if (dataCount)
    {
        if (fragmentHeader.offsetM.bits.fragmentOffset == 0)
        {
            ptrFragment->firstFragmentLength = dataCount + headerLen;
        }

        if ((headerLen + (fragmentHeader.offsetM.bits.fragmentOffset << 3) + dataCount) > 1500u)
        {
            TCPIP_IPV6_SendError (pNetIf, localIP, remoteIP, ICMPV6_ERR_PP_ERRONEOUS_HEADER, ICMPV6_ERROR_PARAMETER_PROBLEM, TCPIP_HELPER_htonl(headerLen + 2), dataCount + headerLen + sizeof (IPV6_FRAGMENT_HEADER));
            return IPV6_ACTION_DISCARD_SILENT;
        }

        if (fragmentHeader.offsetM.bits.m)
        {
            // More fragments
            // Check to ensure the packet's payload length is a multiple of eight bytes.
            if (((headerLen + dataCount) % 8) != 0)
            {
                TCPIP_IPV6_SendError (pNetIf, localIP, remoteIP, ICMPV6_ERR_PP_ERRONEOUS_HEADER, ICMPV6_ERROR_PARAMETER_PROBLEM, TCPIP_HELPER_htonl(IPV6_HEADER_OFFSET_PAYLOAD_LENGTH), dataCount + headerLen + sizeof (IPV6_FRAGMENT_HEADER));
                return IPV6_ACTION_DISCARD_SILENT;
            }
            MACGetArray(hMac, ptrFragment->packet + headerLen + (fragmentHeader.offsetM.bits.fragmentOffset << 3), dataCount);
            ptrFragment->bytesInPacket += dataCount;
        }
        else
        {
            // No more fragments
            MACGetArray(hMac, ptrFragment->packet + headerLen + (fragmentHeader.offsetM.bits.fragmentOffset << 3), dataCount);
            ptrFragment->bytesInPacket += dataCount;
            ptrFragment->packetSize += (fragmentHeader.offsetM.bits.fragmentOffset << 3) + dataCount;
        }
    }

    // Todo : Just for safety we may want to insert a check to prevent this packet from being sent
    // Todo : if packetSize == headerLen.  This would occur if a fragment packet with no fragmentable part
    // Todo : was received.  That should never happen, though.
    if (ptrFragment->packetSize == ptrFragment->bytesInPacket)
    {
        TCPIP_MAC_PTR_TYPE tempReadPtr;
        TCPIP_MAC_PTR_TYPE tempBaseReadPtr;

        // Subtract the length of the IPV6 header from the payload
        ptrFragment->packetSize -= sizeof (IPV6_HEADER);

        ptrFragment->packet[IPV6_HEADER_OFFSET_PAYLOAD_LENGTH] = ((TCPIP_UINT16_VAL)ptrFragment->packetSize).v[1];
        ptrFragment->packet[IPV6_HEADER_OFFSET_PAYLOAD_LENGTH + 1] = ((TCPIP_UINT16_VAL)ptrFragment->packetSize).v[0];

        tempReadPtr = MACSetReadPtr (hMac, (TCPIP_MAC_PTR_TYPE)ptrFragment->packet);
        tempBaseReadPtr = MACSetBaseReadPtr (hMac, (TCPIP_MAC_PTR_TYPE)ptrFragment->packet - sizeof (MAC_ETHERNET_HEADER));

        TCPIP_IPV6_Process (pNetIf, remoteMACAddr);

        MACSetReadPtr (hMac, tempReadPtr);
        MACSetBaseReadPtr (hMac, tempBaseReadPtr);

        SingleListRemoveNode(&pIpv6Config->rxFragments, (SGL_LIST_NODE *)ptrFragment);

        TCPIP_IPV6_FreeFragmentBuffer (ptrFragment);
    }

    return IPV6_ACTION_DISCARD_SILENT;
}



/*****************************************************************************
  Function:
	void TCPIP_IPV6_FragmentTask (void)

  Summary:
	IPv6 fragment processing task function.

  Description:
	IPv6 fragment processing task function.

  Precondition:
	None

  Parameters:
	None

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_IPV6_FragmentTask (void)
{
    IPV6_RX_FRAGMENT_BUFFER * ptrRxFragment;
    IPV6_RX_FRAGMENT_BUFFER * ptrNextRxFragment;
    TCPIP_NET_IF * pNetIf;
    int netIx;
    TCPIP_MAC_HANDLE  hMac;
    IPV6_INTERFACE_CONFIG*  pIpv6Config;

    for (netIx = 0; netIx < nStackIfs; netIx++)
    {
        pNetIf = (TCPIP_NET_IF*)TCPIP_STACK_IxToNet(netIx);
        pIpv6Config = ipv6Config + netIx;
        if(_TCPIPStackIsNetUp(pNetIf))
        {
            ptrRxFragment = (IPV6_RX_FRAGMENT_BUFFER *)pIpv6Config->rxFragments.head;
            while (ptrRxFragment != NULL)
            {
                ptrRxFragment->secondsRemaining--;
                if ((char)(ptrRxFragment->secondsRemaining) == 0)
                {
                    if (ptrRxFragment->firstFragmentLength != 0)
                    {
                        TCPIP_MAC_PTR_TYPE tempReadPtr;
                        TCPIP_MAC_PTR_TYPE tempBaseReadPtr;

                        hMac = _TCPIPStackNetToMac(pNetIf);
                        tempReadPtr = MACSetReadPtr (hMac, (TCPIP_MAC_PTR_TYPE)ptrRxFragment->packet);
                        tempBaseReadPtr = MACSetBaseReadPtr (hMac, (TCPIP_MAC_PTR_TYPE)ptrRxFragment->packet - sizeof (MAC_ETHERNET_HEADER));

                        // If we received the first fragment, send it and a Time Exceeded error message
                        TCPIP_IPV6_SendError (pNetIf, (IPV6_ADDR *)&ptrRxFragment->packet[IPV6_HEADER_OFFSET_DEST_ADDR], (IPV6_ADDR *)&ptrRxFragment->packet[IPV6_HEADER_OFFSET_SOURCE_ADDR], ICMPV6_ERR_TE_FRAG_ASSEMBLY_TIME_EXCEEDED, ICMPV6_ERROR_TIME_EXCEEDED, 0, ptrRxFragment->firstFragmentLength);

                        MACSetReadPtr (hMac, tempReadPtr);
                        MACSetBaseReadPtr (hMac, tempBaseReadPtr);
                    }
                    SingleListRemoveNode(&pIpv6Config->rxFragments, (SGL_LIST_NODE *)ptrRxFragment);
                    ptrNextRxFragment = ptrRxFragment->next;
                    TCPIP_IPV6_FreeFragmentBuffer (ptrRxFragment);
                    ptrRxFragment = ptrNextRxFragment;
                }
                else
                    ptrRxFragment = ptrRxFragment->next;
            }
        }
    }
}



/*****************************************************************************
  Function:
	void TCPIP_IPV6_TmoHandler(SYS_TICK curSysTick)

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
void TCPIP_IPV6_TmoHandler(SYS_TICK curSysTick)
{
    ipv6TickPending++;
}

/*****************************************************************************
  Function:
	bool TCPIP_IPV6_TaskPending (void)

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
bool TCPIP_IPV6_TaskPending (void)
{
    return (ipv6TickPending == 0)?0:1;
}

/*****************************************************************************
  Function:
	void TCPIP_IPV6_Task (void)

  Summary:
	IPv6 task function

  Description:
	IPv6 task function

  Precondition:
	None

  Parameters:
	None

  Returns:
  	None

  Remarks:
	None
  ***************************************************************************/
void TCPIP_IPV6_Task (void)
{
    ipv6TickPending--;
    TCPIP_IPV6_FragmentTask();
    TCPIP_IPV6_UpdateTimestampsTask();
    TCPIP_IPV6_QueuedPacketTransmitTask(&mcastQueue);
    TCPIP_IPV6_QueuedPacketTransmitTask(&ipv6QueuedPackets);
}

/*****************************************************************************
  Function:
	void TCPIP_IPV6_QueuedPacketTransmitTask (SINGLE_LIST* pList)

  Summary:
        Task to transmit/time-out IPv6 queued packets

  Description:
        Task to transmit/time-out IPv6 queued packets

  Precondition:
	None

  Parameters:
	pList   - list to process/transmit

  Returns:
  	None

  Remarks:
	None
  ***************************************************************************/
static void TCPIP_IPV6_QueuedPacketTransmitTask (SINGLE_LIST* pList)
{
    IPV6_PACKET * queuedPacket;
    SYS_TICK tick = SYS_TICK_Get();

    while (pList->nNodes != 0)
    {
        queuedPacket = (IPV6_PACKET *)pList->head;
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
            if (TCPIP_IPV6_TransmitPacket(queuedPacket))
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
	uint8_t TCPIP_IPV6_ProcessRoutingHeader (TCPIP_NET_IF * pNetIf, 
        uint8_t * nextHeader, uint16_t * length)

  Summary:
	Processes an IPv6 Routing Extension header.

  Description:
	Processes an IPv6 Routing Extension header.	

  Precondition:
	None

  Parameters:
	pNetIf - The interface the packet with this header was received on.
    nextHeader - Return value for the next header.
    length - Return value for the header length.

  Returns:
  	uint8_t - An action code for the TCPIP_IPV6_Process function.
  	
  Remarks:
	None
  ***************************************************************************/
uint8_t TCPIP_IPV6_ProcessRoutingHeader (TCPIP_NET_IF * pNetIf, uint8_t * nextHeader, uint16_t * length)
{
    uint8_t headerLength;
    uint8_t routingType;
    uint8_t segmentsLeft;
    uint8_t data[8];
    uint8_t i;
    TCPIP_MAC_HANDLE hMac = _TCPIPStackNetToMac (pNetIf);

    TCPIP_IPV6_GetOptionHeader (hMac, data, 1);

    *nextHeader = data[0];
    headerLength = data[1];
    *length = ((uint16_t)headerLength << 3) + 8;

    routingType = data[2];
    segmentsLeft = data[3];

    i = 4;

    switch (routingType)
    {
        // Type 0 routing headers were deprecated and we aren't a router,
        // so we don't support any routing options
        default:
            if (segmentsLeft == 0)
                break;
            else
            {
                // Set the 'length' parameter to the offset of the routingType field.
                // This will allow the TCPIP_IPV6_Process function to find the correct offset
                // for the ICMPv6 Parameter Problem error message.
                *length = 2;
                return IPV6_ACTION_DISCARD_PP_0;
            }
    }

    // If we get here, ignore the rest of the header
    // Since the header size is a multiple of 8 bytes, 
    // just discard the rest of the bytes in the first
    // 8-byte unit and read the full header size 
    while (headerLength--)
        TCPIP_IPV6_GetOptionHeader (hMac, data, 1);

    return IPV6_ACTION_NONE;
}

/*****************************************************************************
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
  ***************************************************************************/
IPV6_ADDR_STRUCT * TCPIP_IPV6_DAS_SelectSourceAddress (TCPIP_NET_HANDLE hNetIf, IPV6_ADDR * dest, IPV6_ADDR * requestedSource)
{
    IPV6_ADDR_STRUCT * currentSource;
    IPV6_ADDR_STRUCT * previousSource;
    uint8_t ruleCounter = ADDR_SEL_RULE_8;
    IPV6_INTERFACE_CONFIG*  pIpv6Config;
	
	TCPIP_NET_IF * pNetIf = _TCPIPStackHandleToNet(hNetIf);
    if (pNetIf == NULL)
    {
        return NULL;
    }
    else
    {
        pIpv6Config = ipv6Config + _TCPIPStackNetIx (pNetIf);
    }

    // Check to see if the user is trying to force an address
    if (requestedSource != NULL)
    {
        currentSource = TCPIP_IPV6_FindAddress(pNetIf, requestedSource, IPV6_ADDR_TYPE_UNICAST);
        if (currentSource != NULL)
        {
            return currentSource;
        }
        else
        {
            return NULL;
        }
    } // End manual address selection

    // Simple case: there are no local addresses (this should never happen)
    if (DoubleListIsEmpty(&pIpv6Config->listIpv6UnicastAddresses))
        return NULL;

    // Simple case: there's only one source address in the list
    if (DoubleListCount (&pIpv6Config->listIpv6UnicastAddresses) == 1)
        return (IPV6_ADDR_STRUCT *)pIpv6Config->listIpv6UnicastAddresses.head;

    // Complex case: Sort the addresses we found using the Address Selection rules

    // Sort the linked list.
    // There are 8 sorting rules.  Starting with the last rule and working to the most
    // important, using a stable sorting algorithm, will produce a sorted list most
    // efficiently.  The best average run time we'll get with a stable sort with O(1) 
    // memory usage is O(n^2), so we'll use an insertion sort.  This will usually be 
    // most efficient for small lists (which should be the typical case).

    do
    {
        // We know that the list has at least two elements, so these pointers will both have non-null values
        previousSource = (IPV6_ADDR_STRUCT *)pIpv6Config->listIpv6UnicastAddresses.head;
        currentSource = previousSource->next;
        
        do
        {
            // Set previousSource to the node before the current node being evaluated
            previousSource = currentSource->prev;
    
            // Advance backwards through the list until we don't prefer currentSource over previousSource
            // (or until there are no previous source addresses)
            // The IPv6ASCompareSourceAddresses function will return true if we prefer currentSource over previousSource
            while ((previousSource != NULL) && (IPv6ASCompareSourceAddresses (pNetIf, previousSource, currentSource, dest, ruleCounter)))
            {
                previousSource = previousSource->prev;
            }
            
            // Move the currentSource node into the list after previousSource (or to the head if previousSource == NULL)
            currentSource = TCPIP_NDP_UnicastAddressMove (pNetIf, currentSource, previousSource);
        } while (currentSource != NULL);
    
        ruleCounter--;
        // Skip rules 4 and 5 for now; we don't support Mobile IPv6 or multiple interfaces at this time
        if (ruleCounter == ADDR_SEL_RULE_5)
            ruleCounter = ADDR_SEL_RULE_3;
    } while (ruleCounter >= ADDR_SEL_RULE_1);
    
    return (IPV6_ADDR_STRUCT *)pIpv6Config->listIpv6UnicastAddresses.head;
}

/*****************************************************************************
  Function:
	uint8_t TCPIP_IPV6_DAS_GetPolicy (const IPV6_ADDR * addr, uint8_t * label, 
        uint8_t * precedence, uint8_t * prefixLen)

  Summary:
	Gets the default address selection policy for an address.

  Description:
	Gets the default address selection policy for an address.	

  Precondition:
	None

  Parameters:
	addr - The given address
    label - Return value for the address label
    precedence - Return value for the precedence
    prefixLen - Return value for the prefix length

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
uint8_t TCPIP_IPV6_DAS_GetPolicy (const IPV6_ADDR * addr, uint8_t * label, uint8_t * precedence, uint8_t * prefixLen)
{
    uint8_t i;
    uint8_t prefixMatch = 0;
    uint8_t matchingPrefix = 0xFF;

    for (i = 0; i < IPV6_ADDR_POLICY_TABLE_LEN; i++)
    {
        if (gPolicyTable[i].prefixLength != 0xFF)
        {
            // If we get to the 0-length prefix and we haven't found a
            // matching prefix, then assume a match
            if (gPolicyTable[i].prefixLength == 0)
            {
                if (prefixMatch == 0)
                    matchingPrefix = i;
            }
            else if (gPolicyTable[i].prefixLength > prefixMatch)
            {
                if (TCPIP_Helper_FindCommonPrefix ((uint8_t *)addr, (uint8_t *)&gPolicyTable[i].address, 16) >= gPolicyTable[i].prefixLength)
                {
                    matchingPrefix = i;
                    prefixMatch = gPolicyTable[i].prefixLength;
                }
            }
        }
    }

    if (matchingPrefix == 0xFF)
        return false;
    else
    {
        if (label != NULL)
            *label = gPolicyTable[matchingPrefix].label;
        if (precedence != NULL)
            *precedence = gPolicyTable[matchingPrefix].precedence;
        if (prefixLen != NULL)
            *prefixLen = gPolicyTable[matchingPrefix].prefixLength;
    }
    return true;
}

/*****************************************************************************
  Function:
	unsigned char IPv6ASCompareSourceAddresses(TCPIP_NET_IF * pNetIf, 
        IPV6_ADDR_STRUCT * addressOne, IPV6_ADDR_STRUCT * addressTwo, 
        IPV6_ADDR * dest, IPV6_ADDR_SEL_INDEX rule)

  Summary:
	Compares two IPv6 addresses using specified rules to determine which 
    ones are prefereable.

  Description:
	Compares two IPv6 addresses using specified rules to determine which 
    ones are prefereable.

  Precondition:
	None

  Parameters:
	pNetIf - The given interface
    addressOne - One address to compare
    addressTwo - The other address to compare
    dest - A destination address (used for some comparisons)
    rule - The address comparison rule to use

  Returns:
  	bool - true if addressTwo is preferred over addressOne, false otherwise
  	
  Remarks:
	None
  ***************************************************************************/
unsigned char IPv6ASCompareSourceAddresses(TCPIP_NET_IF * pNetIf, IPV6_ADDR_STRUCT * addressOne, IPV6_ADDR_STRUCT * addressTwo, IPV6_ADDR * dest, IPV6_ADDR_SEL_INDEX rule)
{
    unsigned char policy1 = 0;
    unsigned char policy2 = 0;
    unsigned char destPolicy;
    IPV6_ADDRESS_TYPE destScope;
    IPV6_INTERFACE_CONFIG*  pIpv6Config;

    if (pNetIf == NULL)
        return false;

    pIpv6Config = ipv6Config + _TCPIPStackNetIx (pNetIf);

    switch (rule)
    {
        case ADDR_SEL_RULE_1:
            // We can assume the the addresses are different; the function to add a local
            // address won't add a new one if it's already in the IPv6 Heap.
            if (memcmp ((void *)&(addressTwo->address), (void *)dest, 16) == 0)
            {
                return true;
            }
            break;
        case ADDR_SEL_RULE_2:
            if (addressOne->flags.scope != addressTwo->flags.scope)
            {
                destScope = TCPIP_IPV6_GetAddressType (pNetIf, dest);
                destPolicy = destScope.bits.scope;

                if (addressOne->flags.scope < addressTwo->flags.scope)
                {
                    if (addressOne->flags.scope < destPolicy)
                    {
                        return true;
                    }
                }
                else
                {
                    if (addressTwo->flags.scope >= destPolicy)
                    {
                        return true;
                    }
                }
            }
            break;
        case ADDR_SEL_RULE_3:
            if (addressTwo->preferredLifetime && !(addressOne->preferredLifetime))
            {
                return true;
            }
            break;
        case ADDR_SEL_RULE_4:
            // We aren't supporting Mobile IPv6 at this time                
            break;
        case ADDR_SEL_RULE_5:
            // We aren't supporting multiple interfaces at this time
            break;
        case ADDR_SEL_RULE_6:
            if (!TCPIP_IPV6_DAS_GetPolicy (dest, &destPolicy, NULL, NULL))
            {
                // If there's no policy that corresponds to the destination, skip this step
                break;
            }
            if (!TCPIP_IPV6_DAS_GetPolicy (&(addressOne->address), &policy1, NULL, NULL))
            {
                if (TCPIP_IPV6_DAS_GetPolicy (&(addressTwo->address), &policy2, NULL, NULL))
                {
                    if (destPolicy == policy2)
                        return true;
                }
            }
            else
            {
                if (!TCPIP_IPV6_DAS_GetPolicy (&(addressTwo->address), &policy2, NULL, NULL))
                {
                    if (destPolicy == policy1)
                        return false;
                }
            }
            if (policy1 != policy2)
            {
                if (destPolicy == policy2)
                    return true;
            }
            break;
        case ADDR_SEL_RULE_7:
            if (addressOne->flags.temporary != addressTwo->flags.temporary)
            {
                if (((addressTwo->flags.temporary == false) && (pIpv6Config->policyPreferTempOrPublic == IPV6_PREFER_PUBLIC_ADDRESSES)) ||
                    ((addressTwo->flags.temporary == true) && (pIpv6Config->policyPreferTempOrPublic == IPV6_PREFER_TEMPORARY_ADDRESSES)))
                {
                    return true;
                }
            }
            break;
        case ADDR_SEL_RULE_8:
            policy1 = TCPIP_Helper_FindCommonPrefix ((uint8_t *)dest, (uint8_t *)&(addressOne->address), 16);
            policy2 = TCPIP_Helper_FindCommonPrefix ((uint8_t *)dest, (uint8_t *)&(addressTwo->address), 16);
            if (policy2 > policy1)
            {
                return true;
            }
            break;

        default:
            break;
    }
    // If there's no reason to prefer addressTwo, return false
    return false;
}

/*****************************************************************************
  Function:
	void TCPIP_IPV6_UpdateTimestampsTask (void)

  Summary:
	Task to update timestamps and check for validity for NDP structures.

  Description:
	Task to update timestamps and check for validity for NDP structures.

  Precondition:
	None

  Parameters:
	None

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_IPV6_UpdateTimestampsTask (void)
{
    unsigned long timeElapsed;
    unsigned long currentTickTime = SYS_TICK_Get();
    unsigned long correctedCurrentTime;
    int i;
    IPV6_HEAP_NDP_PL_ENTRY * ptrPrefix;
    IPV6_HEAP_NDP_PL_ENTRY * tempPrefix;
    IPV6_ADDR_STRUCT * ptrAddress;
    IPV6_ADDR_STRUCT * tempAddress;
    IPV6_HEAP_NDP_DR_ENTRY * ptrRouter;
    IPV6_HEAP_NDP_DC_ENTRY * ptrDestination;
    TCPIP_NET_IF * pNetIf;
    IPV6_INTERFACE_CONFIG*  pIpv6Config;

    for (i = 0; i < nStackIfs; i++)
    {
        pNetIf = (TCPIP_NET_IF*)TCPIP_STACK_IxToNet(i);
        pIpv6Config = ipv6Config + i;
        if(_TCPIPStackIsNetUp(pNetIf))
        {
            // Update prefixes
            ptrPrefix = (IPV6_HEAP_NDP_PL_ENTRY *)pIpv6Config->listPrefixList.head;

            while (ptrPrefix != NULL)
            {
                timeElapsed = currentTickTime - ptrPrefix->lastTickTime;
                timeElapsed /= SYS_TICK_ResolutionGet();
                correctedCurrentTime = currentTickTime - (timeElapsed % SYS_TICK_ResolutionGet());

                ptrPrefix->lastTickTime = correctedCurrentTime;
                if (timeElapsed < ptrPrefix->validLifetime)
                {
                    ptrPrefix->validLifetime -= timeElapsed;
                    tempPrefix = ptrPrefix->next;
                }
                else
                {
                    tempPrefix = ptrPrefix->next;
                    TCPIP_NDP_LinkedListEntryRemove ((TCPIP_NET_IF*)TCPIP_STACK_IxToNet(i), ptrPrefix, IPV6_HEAP_NDP_PL_ID);
                }
                ptrPrefix = tempPrefix;
            }

            // Update addresses
            ptrAddress = (IPV6_ADDR_STRUCT *)pIpv6Config->listIpv6UnicastAddresses.head;

            while (ptrAddress != NULL)
            {
                timeElapsed = currentTickTime - ptrAddress->lastTickTime;
                timeElapsed /= SYS_TICK_ResolutionGet();
                correctedCurrentTime = currentTickTime - (timeElapsed % SYS_TICK_ResolutionGet());

                ptrAddress->lastTickTime = correctedCurrentTime;
                if (timeElapsed < ptrAddress->preferredLifetime)
                {
                    ptrAddress->preferredLifetime -= timeElapsed;
                }
                else
                {
                    ptrAddress->preferredLifetime = 0;
                }
                if (timeElapsed < ptrAddress->validLifetime)
                {
                    ptrAddress->validLifetime -= timeElapsed;    
                    tempAddress = ptrAddress->next;
                }
                else
                {
                    tempAddress = ptrAddress->next;
                    IPv6RemoveUnicastAddress ((TCPIP_NET_IF*)TCPIP_STACK_IxToNet(i), &ptrAddress->address);
                }
                ptrAddress = tempAddress;

            }

            // Update default routers
            ptrRouter = (IPV6_HEAP_NDP_DR_ENTRY *)pIpv6Config->listDefaultRouter.head;

            while (ptrRouter != NULL)
            {
                if ((long)(currentTickTime - ptrRouter->tickTimer) >= SYS_TICK_ResolutionGet())
                {
                    ptrRouter->tickTimer += SYS_TICK_ResolutionGet();
                    ptrRouter->invalidationTimer--;
                    if (!ptrRouter->invalidationTimer)
                    {
                        ptrRouter = TCPIP_NDP_LinkedListEntryRemove ((TCPIP_NET_IF*)TCPIP_STACK_IxToNet(i), ptrRouter, IPV6_HEAP_NDP_DR_ID);
                    }
                }
                else
                {
                    ptrRouter = ptrRouter->next;
                }
            }

            // Try to periodically increase path MTUs
            ptrDestination = (IPV6_HEAP_NDP_DC_ENTRY *)pIpv6Config->listDestinationCache.head;

            while (ptrDestination != NULL)
            {
                if (ptrDestination->pathMTUIncreaseTimer != 0)
                {
                    if ((long)(currentTickTime - ptrDestination->pathMTUIncreaseTimer) > 0)
                    {
                        ptrDestination->pathMTU = IPV6_DEFAULT_LINK_MTU;
                        ptrDestination->pathMTUIncreaseTimer = 0;
                    }
                }
                ptrDestination = ptrDestination->next;
            }

            if (pIpv6Config->mtuIncreaseTimer != 0)
            {
                if ((long)(currentTickTime - pIpv6Config->mtuIncreaseTimer) > 0)
                {
                    pIpv6Config->linkMTU = IPV6_DEFAULT_LINK_MTU;
                    pIpv6Config->multicastMTU = IPV6_DEFAULT_LINK_MTU;
                    pIpv6Config->mtuIncreaseTimer = 0;
                }
            }
        }
    }
}


/*****************************************************************************
  Function:
	void TCPIP_IPV6_Process (TCPIP_NET_IF * pNetIf, MAC_ADDR * remoteMAC)

  Summary:
	Processes incoming IPv6 packets/

  Description:
	This function processes incoming IPv6 packets.

  Precondition:
	None

  Parameters:
	pNetIf - The interface the packet was received on
    remoteMAC - The remote node's MAC address

  Returns:
  	true - if UDP packets processed and the TCPIP dispatcher loop needs to be broken
    fale - other packets types 
  	
  Remarks:
    None

  ***************************************************************************/
bool TCPIP_IPV6_Process (TCPIP_NET_IF * pNetIf, MAC_ADDR * remoteMAC)
{
    IPV6_ADDR tempLocalIPv6Addr;
    IPV6_ADDR tempRemoteIPv6Addr;
    IPV6_ADDR_STRUCT * localAddressPointer;
    uint16_t headerLen = 0;
    uint16_t extensionHeaderLen;
    uint8_t action;
    uint8_t hopLimit;
    uint8_t addrType = 0;
    uint16_t currentOffset = 0;
    uint8_t cIPFrameType;
    uint16_t dataCount;

    // Break out of this processing function is IPv6 is not enabled on this node
    if (!pNetIf->Flags.bIPv6Enabled)
        return false;

    // Get the relevant IPv6 header parameters
    if (!TCPIP_IPV6_GetHeader(pNetIf, &tempLocalIPv6Addr, &tempRemoteIPv6Addr, &cIPFrameType, &dataCount, &hopLimit))
        return false;

    currentOffset += sizeof (IPV6_HEADER);

    // Determine if the address corresponds to one of the addresses used by our node
    if (tempLocalIPv6Addr.v[0] == 0xFF)
    {
        // Determine if the address is a solicited node multicast address
        if (TCPIP_IPV6_AddressIsSolicitedNodeMulticast (&tempLocalIPv6Addr))
        {
            // Determine if we are listening to this address
            if ((localAddressPointer = TCPIP_IPV6_FindSolicitedNodeMulticastAddress(pNetIf, &tempLocalIPv6Addr, IPV6_ADDR_TYPE_UNICAST)) != NULL)
            {
                addrType = IPV6_ADDR_TYPE_SOLICITED_NODE_MULTICAST;
            }
            else if ((localAddressPointer = TCPIP_IPV6_FindSolicitedNodeMulticastAddress(pNetIf, &tempLocalIPv6Addr, IPV6_ADDR_TYPE_UNICAST_TENTATIVE)) != NULL)
            {
                addrType = IPV6_ADDR_TYPE_UNICAST_TENTATIVE;
            }
        }
        else
        {
            // Find the address in the list of multicast addresses we're listening to
            localAddressPointer = TCPIP_IPV6_FindAddress(pNetIf, &tempLocalIPv6Addr, IPV6_ADDR_TYPE_MULTICAST);
            addrType = IPV6_ADDR_TYPE_MULTICAST;
        }
    }
    else
    {
        // Find the address in the list of unicast addresses assigned to our node
        localAddressPointer = TCPIP_IPV6_FindAddress (pNetIf, &tempLocalIPv6Addr, IPV6_ADDR_TYPE_UNICAST);
        addrType = IPV6_ADDR_TYPE_UNICAST;
    }

    // If the packet's destination address isn't one of the unicast/multicast addresses assigned to this node, check to see if it is a 
    // tentative address (ICMPv6/NDP still needs to receive packets addressed to tentative addresses for duplicate address detection).
    // If it is not tentative, return.
    if (localAddressPointer == NULL)
    {
        // If we didn't find a matching configured address try to find one in the tentative address list
        if ((localAddressPointer = TCPIP_IPV6_FindAddress (pNetIf, &tempLocalIPv6Addr, IPV6_ADDR_TYPE_UNICAST_TENTATIVE)) != NULL)
        {
            addrType = IPV6_ADDR_TYPE_UNICAST_TENTATIVE;
        }
        else
        {
            return false;
        }
    }

    extensionHeaderLen = 0;
    action = IPV6_ACTION_BEGIN_EX_HEADER_PROCESSING;

    // Process frame
    while (cIPFrameType != IPV6_PROT_NONE)
    {
        switch (cIPFrameType)
        {
            // Process the frame's hop-by-hop options header
            case IPV6_PROT_HOP_BY_HOP_OPTIONS_HEADER:
                // Action should only equal 0xFF immediately after processing the IPv6 header.
                // The hop-by-hop options header must occur only after the IPv6 header.
                if (action != IPV6_ACTION_BEGIN_EX_HEADER_PROCESSING)
                {
                    goto ParameterProblem;
                }
                action = TCPIP_IPV6_ProcessHopByHopOptionsHeader (pNetIf, &cIPFrameType, &headerLen);
                dataCount -= headerLen;
                extensionHeaderLen += headerLen;
                break;
            // Process the frame's routing header
            case IPV6_PROT_ROUTING_HEADER:
                action = TCPIP_IPV6_ProcessRoutingHeader (pNetIf, &cIPFrameType, &headerLen);
                dataCount -= headerLen;
                extensionHeaderLen += headerLen;
                break;
            // Process the frame's fragmentation header
            case IPV6_PROT_FRAGMENTATION_HEADER:
                action = TCPIP_IPV6_ProcessFragmentationHeader (pNetIf, &tempRemoteIPv6Addr, &tempLocalIPv6Addr, &cIPFrameType, dataCount, extensionHeaderLen + sizeof (IPV6_HEADER), remoteMAC, headerLen);
                //action = IPV6_ACTION_DISCARD_SILENT;
                break;
            // Process the frame's ESP header
            case IPV6_PROT_ENCAPSULATING_SECURITY_PAYLOAD_HEADER:
                action = IPV6_ACTION_DISCARD_SILENT;
                break;
            // Process the frame's Authentication header
            case IPV6_PROT_AUTHENTICATION_HEADER:
                action = IPV6_ACTION_DISCARD_SILENT;
                break;
            // Process the frame's destination options header
            case IPV6_PROT_DESTINATION_OPTIONS_HEADER:
                action = TCPIP_IPV6_ProcessDestinationOptionsHeader(pNetIf, &cIPFrameType, &headerLen);
                dataCount -= headerLen;
                extensionHeaderLen += headerLen;
                break;
            // Process the frame's TCP header and payload
            case IPV6_PROT_TCP:
#if defined (TCPIP_STACK_USE_TCP)
                // If the address is tentative, do not process the TCP packet
                if (addrType == IPV6_ADDR_TYPE_UNICAST_TENTATIVE)
                {
                    cIPFrameType = IPV6_PROT_NONE;
                    break;
                }
                TCPProcessIPv6(pNetIf, &tempLocalIPv6Addr, &tempRemoteIPv6Addr, dataCount, extensionHeaderLen);
#endif
                cIPFrameType = IPV6_PROT_NONE;
                action = 0;
                break;
            // Process the frame's UDP header and payload
            case IPV6_PROT_UDP:
#if defined (TCPIP_STACK_USE_UDP)
                // If the address is tentative, do not process the TCP packet
                if (addrType == IPV6_ADDR_TYPE_UNICAST_TENTATIVE)
                {
                    cIPFrameType = IPV6_PROT_NONE;
                    break;
                }
                // Process the UDP packet
                return UDPProcessIPv6(pNetIf, &tempLocalIPv6Addr, &tempRemoteIPv6Addr, dataCount, extensionHeaderLen);
#else
                action = 0;
                return false;                
#endif
            // Process the frame's ICMPv6 header and payload
            case IPV6_PROT_ICMPV6:
                // Process the ICMPv6 packet
                TCPIP_ICMPV6_Process(pNetIf, localAddressPointer, &tempLocalIPv6Addr, &tempRemoteIPv6Addr, dataCount, extensionHeaderLen, hopLimit, addrType);
                cIPFrameType = IPV6_PROT_NONE;
                action = 0;
                break;
            // Unknown header type
            default:
ParameterProblem:
                // Send ICMP Parameter Problem Code 1 and discard packet
                {
                    // Action should only equal IPV6_ACTION_BEGIN_EX_HEADER_PROCESSING if we haven't been through this loop once
                    if (action == IPV6_ACTION_BEGIN_EX_HEADER_PROCESSING)
                        currentOffset = IPV6_HEADER_OFFSET_NEXT_HEADER;
                    else
                        currentOffset -= headerLen;

                    TCPIP_IPV6_SendError (pNetIf, &tempLocalIPv6Addr, &tempRemoteIPv6Addr, ICMPV6_ERR_PP_UNRECOGNIZED_NEXT_HEADER, ICMPV6_ERROR_PARAMETER_PROBLEM, TCPIP_HELPER_htonl((uint32_t)currentOffset), dataCount + extensionHeaderLen + sizeof (IPV6_HEADER));
                    cIPFrameType = IPV6_PROT_NONE;
                }
                break;
        }

        // Add the length of the header to the current offset
        // If there was a parameter problem, this value will indicate the offset 
        // in the header at which the parameter problem occured.
        currentOffset += headerLen;

        // Take an action depending on the result of our header processing
        switch (action)
        {
            // Silently discard the packet
            case IPV6_ACTION_DISCARD_SILENT:
                cIPFrameType = IPV6_PROT_NONE;
                break;
            // Discard the packet and send an ICMPv6 Parameter Problem, code 0 error message to the packet's source address
            case IPV6_ACTION_DISCARD_PP_0:
                TCPIP_IPV6_SendError (pNetIf, &tempLocalIPv6Addr, &tempRemoteIPv6Addr, ICMPV6_ERR_PP_ERRONEOUS_HEADER, ICMPV6_ERROR_PARAMETER_PROBLEM, TCPIP_HELPER_htonl((uint32_t)currentOffset), dataCount + extensionHeaderLen + sizeof (IPV6_HEADER));
                cIPFrameType = IPV6_PROT_NONE;
                break;
            // Discard the packet and send an ICMPv6 Parameter Problem, code 2 error message to the packet's source address
            case IPV6_ACTION_DISCARD_PP_2:
                TCPIP_IPV6_SendError (pNetIf, &tempLocalIPv6Addr, &tempRemoteIPv6Addr, ICMPV6_ERR_PP_UNRECOGNIZED_IPV6_OPTION, ICMPV6_ERROR_PARAMETER_PROBLEM, TCPIP_HELPER_htonl((uint32_t)currentOffset), dataCount + extensionHeaderLen + sizeof (IPV6_HEADER));
                cIPFrameType = IPV6_PROT_NONE;
                break;
            // Discard the packet and send an ICMPv6 Parameter Problem, code 2 error message to the packet's source address if
            // the packet's destination address is not a multicast address.
            case IPV6_ACTION_DISCARD_PP_2_NOT_MC:
                // Check to ensure the packet's destination address wasn't a multicast address
                if (tempLocalIPv6Addr.v[0] != 0xFF)
                {
                    TCPIP_IPV6_SendError (pNetIf, &tempLocalIPv6Addr, &tempRemoteIPv6Addr, ICMPV6_ERR_PP_UNRECOGNIZED_IPV6_OPTION, ICMPV6_ERROR_PARAMETER_PROBLEM, TCPIP_HELPER_htonl((uint32_t)currentOffset), dataCount + extensionHeaderLen + sizeof (IPV6_HEADER));
                }
                // Discard the packet
                cIPFrameType = IPV6_PROT_NONE;
                break;
            // No action was required
            case IPV6_ACTION_NONE:
            default:
                break;
        }
    }

    return false;
}

/*****************************************************************************
  Function:
	void TCPIP_IPV6_SendError (TCPIP_NET_IF * pNetIf, IPV6_ADDR * localIP, 
        IPV6_ADDR * remoteIP, uint8_t code, uint8_t type, 
        uint32_t additionalData, uint16_t packetLen)

  Summary:
	Sends an ICMPv6 error message to a remote node.

  Description:
	Sends an ICMPv6 error message to a remote node.	

  Precondition:
	None

  Parameters:
	pNetIf - The outgoing interface to send the message on.
    localIP - The local IP address to use.
    remoteIP - The destination IP address to use.
    code - The error code value.
    type - The error type.
    additionalData - Additional ICMPv6 header data.
    packetLen - Length of packet data to include from the packet RX buffer.

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_IPV6_SendError (TCPIP_NET_IF * pNetIf, IPV6_ADDR * localIP, IPV6_ADDR * remoteIP, uint8_t code, uint8_t type, uint32_t additionalData, uint16_t packetLen)
{
    IPV6_PACKET * pkt;

    if (packetLen + sizeof (IPV6_HEADER) + sizeof (ICMPV6_HEADER_ERROR) > IPV6_MINIMUM_LINK_MTU)
        packetLen = IPV6_MINIMUM_LINK_MTU - (sizeof (IPV6_HEADER) + sizeof (ICMPV6_HEADER_ERROR));

    // An ICMPv6 error message MUST NOT be sent as a result of receiving a packet destined to an IPv6 multicast address
    // Exception: Packet Too Big message, Parameter Problem code 2 message if the unrecognized option has its unrecognized action bits set to 0b10.
    // The unrecognized action bits were already examined in the option processing functions.
    if ((localIP->v[0] == 0xFF) && (type != ICMPV6_ERROR_PACKET_TOO_BIG) && ((type != ICMPV6_ERROR_PARAMETER_PROBLEM) || (code != ICMPV6_ERR_PP_UNRECOGNIZED_IPV6_OPTION)))
        return;

    // An ICMPv6 error message MUST NOT be sent as a result of receiving a packet whose source address does not uniquely identify a single node.
    if ((remoteIP->v[0] == 0xFF) || !memcmp (remoteIP, (IPV6_ADDR *)&IPV6_FIXED_ADDR_UNSPECIFIED, sizeof (IPV6_ADDR)))
        return;

    pkt = TCPIP_ICMPV6_PutHeaderError (pNetIf, localIP, remoteIP, code, type, additionalData);
    if (pkt != NULL)
    {
        TCPIP_MAC_HANDLE  hMac = _TCPIPStackNetToMac(pNetIf);
        
        MACSetReadPtrInRx (hMac, 0);
        if (TCPIP_IPV6_IsTxPutReady(pkt, packetLen) < packetLen)
        {
            TCPIP_IPV6_FreePacket (pkt);
            return;
        }
        TCPIP_IPV6_PutRxData (hMac, pkt, packetLen);
        TCPIP_ICMPV6_Flush (pkt);
    }
}


/*****************************************************************************
  Function:
	void * TCPIP_IPV6_GetUpperLayerHeaderPtr(IPV6_PACKET * pkt)

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
void * TCPIP_IPV6_GetUpperLayerHeaderPtr(IPV6_PACKET * pkt)
{
    return TCPIP_IPV6_GetDataSegmentContentsByType(pkt, TYPE_IPV6_UPPER_LAYER_HEADER);
}

/*****************************************************************************
  Function:
	unsigned short TCPIP_IPV6_GetPayloadLength (IPV6_PACKET * pkt)

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
unsigned short TCPIP_IPV6_GetPayloadLength (IPV6_PACKET * pkt)
{
    return pkt->payloadLen + pkt->upperLayerHeaderLen;
}



IPV6_ADDR *  TCPIP_IPV6_GetDestAddress(IPV6_PACKET * p)
{
    return &p->ipv6Header.DestAddress;
}
void  TCPIP_IPV6_SetDestAddress(IPV6_PACKET * p, IPV6_ADDR * addr)
{
    if(addr)
    {
        memcpy (&p->ipv6Header.DestAddress, (void *)addr, sizeof (IPV6_ADDR));
    }
    else
    {
        memset (&p->ipv6Header.DestAddress, 0x0, sizeof (IPV6_ADDR));
    }
}
void  TCPIP_IPV6_SetSourceAddress(IPV6_PACKET * p, IPV6_ADDR * addr)
{
    if(addr)
    {
        memcpy (&p->ipv6Header.SourceAddress, addr, sizeof(IPV6_ADDR));
        p->flags.sourceSpecified = true;
    }
    else
    {
        memset (&p->ipv6Header.SourceAddress, 0x0, sizeof(IPV6_ADDR));
        p->flags.sourceSpecified = false;
    }
}
IPV6_ADDR *  TCPIP_IPV6_GetSourceAddress(IPV6_PACKET * p)
{
    return &p->ipv6Header.SourceAddress;
}


// Register an IPv6 event handler
// Use hNet == 0 to register on all interfaces available
IPV6_HANDLE TCPIP_IPV6_RegisterHandler(TCPIP_NET_HANDLE hNet, IPV6_EVENT_HANDLER handler, const void* hParam)
{
    if(ipv6MemH)
    {
        IPV6_LIST_NODE* newNode = (IPV6_LIST_NODE*)TCPIP_NotificationAdd(&ipv6RegisteredUsers, ipv6MemH, sizeof(*newNode));

        if(newNode)
        {
            newNode->handler = handler;
            newNode->hParam = hParam;
            newNode->hNet = hNet;
            return newNode;
        }
    }

    return 0;
}

// deregister the event handler
bool TCPIP_IPV6_DeRegisterHandler(IPV6_HANDLE hIpv6)
{
    if(hIpv6 && ipv6MemH)
    {
        if(TCPIP_NotificationRemove((SGL_LIST_NODE*)hIpv6, &ipv6RegisteredUsers, ipv6MemH))
        {
            return true;
        }
    }

    return false;
}

void TCPIP_IPV6_NotifyClients(TCPIP_NET_IF* pNetIf, IPV6_EVENT_TYPE evType)
{
    IPV6_LIST_NODE* iNode;

    for(iNode = (IPV6_LIST_NODE*)ipv6RegisteredUsers.head; iNode != 0; iNode = iNode->next)
    {
        if(iNode->hNet == 0 || iNode->hNet == pNetIf)
        {   // trigger event
            (*iNode->handler)(pNetIf, evType, iNode->hParam);
        }
    }
    
}

IPV6_INTERFACE_CONFIG* TCPIP_IPV6_GetInterfaceConfig(TCPIP_NET_IF* pNetIf)
{
    return ipv6Config + _TCPIPStackNetIx (pNetIf);
}

/*****************************************************************************
  Function:
	void TCPIP_IPV6_DoubleListFree (void * list)

  Summary:
	Frees and deallocates all nodes from a doubly linked list.

  Description:
	Frees and deallocates all nodes from a doubly linked list.

  Precondition:
	None

  Parameters:
	list - The list.

  Returns:
  	None

  Remarks:
	None
  ***************************************************************************/
void TCPIP_IPV6_DoubleListFree (void * list)
{
    DOUBLE_LIST * dList = (DOUBLE_LIST *) list;
    DBL_LIST_NODE * node;
    while ((node = DoubleListRemoveHead (dList)))
        TCPIP_HEAP_Free (ipv6MemH, node);
}


/*****************************************************************************
  Function:
	void TCPIP_IPV6_SingleListFree (void * list)

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
void TCPIP_IPV6_SingleListFree (void * list)
{
    SINGLE_LIST * sList = (SINGLE_LIST *) list;
    SGL_LIST_NODE * node;
    while ((node = SingleListRemoveHead (sList)))
        TCPIP_HEAP_Free (ipv6MemH, node);
}

#endif  // defined(TCPIP_STACK_USE_IPV6)


