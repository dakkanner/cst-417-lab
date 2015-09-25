/*******************************************************************************
  Dynamic Host Configuration Protocol (DHCP) Server

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    - Provides automatic IP address, subnet mask, gateway address, 
      DNS server address, and other configuration parameters on DHCP 
      enabled networks.
    - Reference: RFC 2131, 2132
*******************************************************************************/

/*******************************************************************************
FileName:   DHCPs.c
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
#define __DHCPS_C

#include "tcpip_private.h"
#include "dhcp_private.h"
#include "tcpip\udp.h"
#if defined(TCPIP_STACK_USE_DHCP_SERVER)

#define TCPIP_THIS_MODULE_ID    TCPIP_MODULE_DHCP_SERVER
#define DHCP_LEASE_DURATION     60
// DHCP Control Block.  Lease IP address is derived from index into DCB array.
typedef struct
{
    uint32_t     LeaseExpires;    // Expiration time for this lease
    MAC_ADDR    ClientMAC;        // Client's MAC address.  Multicase bit is used to determine if a lease is given out or not
    enum
    {
        LEASE_UNUSED = 0,
        LEASE_REQUESTED,
        LEASE_GRANTED
    } smLease;                    // Status of this lease
} DHCP_CONTROL_BLOCK;



typedef enum
{
    DHCP_SERVER_DISABLE,
    DHCP_SERVER_OPEN_SOCKET,
    DHCP_SERVER_LISTEN,
}DHCP_SRVR_STAT;

typedef struct
{
    UDP_SOCKET      uSkt;               // Socket used by DHCP Server
    IPV4_ADDR            dhcpNextLease;        // IP Address to provide for next lease
    DHCP_SRVR_STAT  smServer;           // server state machine status
    bool            enabled;            // Whether or not the DHCP server is enabled
#if defined(TCPIP_STACK_USE_ZEROCONF_LINK_LOCAL)
    bool            leaseAvailable;
#endif
}DHCP_SRVR_DCPT;    // DHCP server descriptor
    


typedef struct
{
    MAC_ADDR ClientMAC;
    IPV4_ADDR     Client_Addr;
    bool         isUsed;
    uint32_t     Client_Lease_Time;
}DHCP_IP_POOL;
#define MAX_DCHP_CLIENTS_NUMBER 16
DHCP_IP_POOL DhcpIpPool[MAX_DCHP_CLIENTS_NUMBER];

// DHCP is running on all interfaces
static DHCP_SRVR_DCPT*      dhcpSDcpt = 0;
static const void*          dhcpSMemH = 0;        // memory handle

static int                  dhcpSInitCount = 0;     // initialization count


static void DHCPReplyToDiscovery(BOOTP_HEADER *Header, int netIx);
static void DHCPReplyToRequest(BOOTP_HEADER *Header, bool bAccept, int netIx, bool bRenew);
static bool isIpAddrInPool(IPV4_ADDR ipaddr) ;
static IPV4_ADDR GetIPAddrFromIndex_DhcpPool(int8_t index);
static int8_t preAssign_ToDHCPClient_FromPool(BOOTP_HEADER *Header);
static int8_t postAssign_ToDHCPClient_FromPool(MAC_ADDR *macAddr, IPV4_ADDR *ipv4Addr);
static void renew_dhcps_Pool(void);
static bool Compare_MAC_addr(const MAC_ADDR *macAddr1, const MAC_ADDR *macAddr2);
static int8_t getIndexByMacaddr_DhcpPool(const MAC_ADDR *MacAddr);
static bool isMacAddr_Effective(const MAC_ADDR *macAddr);


/*****************************************************************************
  Function:
    bool DHCPServerInit(const TCPIP_STACK_MODULE_CTRL* const stackCtrl, const DHCPS_MODULE_CONFIG* pDhcpConfig);

  Summary:
    Resets the DHCP server module for the specified interface.

  Description:
    Resets the DHCP server module for the specified interface.

  Precondition:
    None

  Parameters:
    stackCtrl - pointer to stack structure specifying the interface to initialize

  Returns:
    None

  Remarks:
    This function should be called internally just once per interface
    by the stack manager.
***************************************************************************/
bool DHCPServerInit(const TCPIP_STACK_MODULE_CTRL* const stackCtrl, const DHCPS_MODULE_CONFIG* pDhcpConfig)
{    
    int i;
    if(stackCtrl->stackAction == TCPIP_STACK_ACTION_IF_UP)
    {   // interface restart
        return true;
    }

    // stack init
    
    if(dhcpSInitCount == 0)
    {   // first time we're run
        DHCP_SRVR_DCPT* pServer;

        // store the memory allocation handle
        dhcpSMemH = stackCtrl->memH;
        
        dhcpSDcpt = (DHCP_SRVR_DCPT*)TCPIP_HEAP_Calloc(dhcpSMemH,  stackCtrl->nIfs, sizeof(DHCP_SRVR_DCPT));
        if(dhcpSDcpt == 0)
        {   // failed
            return false;
        }

        // initialize each instance
        int ix;
        for(ix = 0, pServer = dhcpSDcpt; ix < stackCtrl->nIfs; ix++, pServer++)
        {
            pServer->uSkt = INVALID_UDP_SOCKET;
            pServer->enabled = true;
            pServer->smServer = DHCP_SERVER_OPEN_SOCKET;
        }
    }
            
    // Reset state machine and flags to default values

    dhcpSInitCount++;
    //init ip pool
    for(i = 0;i < MAX_DCHP_CLIENTS_NUMBER; i++)
    {
        DhcpIpPool[i].isUsed = false;
        DhcpIpPool[i].Client_Lease_Time = 0; //   1 hour
        #if defined(MRF24WG)
            #if  (WF_DEFAULT_NETWORK_TYPE == WF_NETWORK_TYPE_SOFT_AP  )
            {
                DhcpIpPool[i].Client_Addr.v[0] = 192;
                DhcpIpPool[i].Client_Addr.v[1] = 168;
            }
            #elif (WF_DEFAULT_NETWORK_TYPE == WF_NETWORK_TYPE_ADHOC)
            {
                DhcpIpPool[i].Client_Addr.v[0] = 169;
                DhcpIpPool[i].Client_Addr.v[1] = 254;
            }
            #endif
        #endif // MRF24WG
        DhcpIpPool[i].Client_Addr.v[2] = 0;
        DhcpIpPool[i].Client_Addr.v[3] = 100+i;
        DhcpIpPool[i].ClientMAC.v[0]=0;
        DhcpIpPool[i].ClientMAC.v[1]=0;
        DhcpIpPool[i].ClientMAC.v[2]=0;
        DhcpIpPool[i].ClientMAC.v[3]=0;
        DhcpIpPool[i].ClientMAC.v[4]=0;
        DhcpIpPool[i].ClientMAC.v[5]=0;
    }

    return true;
}

/*****************************************************************************
  Function:
    bool DHCPServerDeInit(const TCPIP_STACK_MODULE_CTRL* const stackCtrl);

  Summary:
    Turns off the DHCP server module for the specified interface.

  Description:
    Closes out UDP socket.

  Precondition:
    None

  Parameters:
    stackData - pointer to stack structure specifying the interface to deinitialize

  Returns:
    None

  Remarks:
    This function should be called internally just once per interface
    by the stack manager.
***************************************************************************/
void DHCPServerDeInit(const TCPIP_STACK_MODULE_CTRL* const stackCtrl)
{
}


typedef enum
{
    UDP_OPEN_SERVER,    // create a server socket
    UDP_OPEN_CLIENT,    // create a client socket

}UDP_OPEN_TYPE;
/*****************************************************************************
  Function:
    bool DHCPServerTask(TCPIP_NET_IF* pNetIf)

  Summary:
    Performs periodic DHCP server tasks.

  Description:
    This function performs any periodic tasks requied by the DHCP server
    module, such as processing DHCP requests and distributing IP addresses.

  Precondition:
    None

  Parameters:
    pNetIf   - interface

  Returns:
    None
  ***************************************************************************/
bool DHCPServerTask(TCPIP_NET_IF* pNetIf)
{
    uint8_t                 i;
    uint8_t                Option, Len;
    BOOTP_HEADER        BOOTPHeader;
    uint32_t                dw;
    bool                bAccept, bRenew;
    int                 netIx;
    UDP_SOCKET          s;

#if defined(TCPIP_STACK_USE_DHCP_CLIENT)
    // Make sure we don't clobber anyone else's DHCP server
    if(DHCPIsServerDetected(pNetIf))
        return true;
#endif

    netIx = _TCPIPStackNetIx(pNetIf);
    if(!dhcpSDcpt[netIx].enabled)
        return false;

    s = dhcpSDcpt[netIx].uSkt;

    renew_dhcps_Pool();

    switch(dhcpSDcpt[netIx].smServer)
    {
        case DHCP_SERVER_DISABLE:
            break;
        case DHCP_SERVER_OPEN_SOCKET:
            // Obtain a UDP socket to listen/transmit on
            dhcpSDcpt[netIx].uSkt = UDPOpenServer(IP_ADDRESS_TYPE_IPV4, DHCP_SERVER_PORT,  NULL);
            if(dhcpSDcpt[netIx].uSkt == INVALID_UDP_SOCKET)
                break;

            UDPSocketSetNet(dhcpSDcpt[netIx].uSkt, pNetIf);
            // Decide which address to lease out
            // Note that this needs to be changed if we are to
            // support more than one lease
            dhcpSDcpt[netIx].dhcpNextLease.Val = (pNetIf->netIPAddr.Val & pNetIf->netMask.Val) + 0x02000000;
            if(dhcpSDcpt[netIx].dhcpNextLease.v[3] == 255u)
                dhcpSDcpt[netIx].dhcpNextLease.v[3] += 0x03;
            if(dhcpSDcpt[netIx].dhcpNextLease.v[3] == 0u)
                dhcpSDcpt[netIx].dhcpNextLease.v[3] += 0x02;

            dhcpSDcpt[netIx].smServer++;

        case DHCP_SERVER_LISTEN:
            // Check to see if a valid DHCP packet has arrived
            if(UDPIsGetReady(s) < 241u)
                break;

            // Retrieve the BOOTP header
            UDPGetArray(s, (uint8_t*)&BOOTPHeader, sizeof(BOOTPHeader));
            if(true == isIpAddrInPool(BOOTPHeader.ClientIP)){bRenew= true; bAccept = true;}
            else if(BOOTPHeader.ClientIP.Val == 0x00000000u) {bRenew = false; bAccept = true;}
            else                                             {bRenew = false; bAccept = false;}
            //bAccept = (BOOTPHeader.ClientIP.Val == dhcpSDcpt[netIx].dhcpNextLease.Val) || (BOOTPHeader.ClientIP.Val == 0x00000000u);

            // Validate first three fields
            if(BOOTPHeader.MessageType != 1u)
                break;
            if(BOOTPHeader.HardwareType != 1u)
                break;
            if(BOOTPHeader.HardwareLen != 6u)
                break;

            // Throw away 10 unused bytes of hardware address,
            // server host name, and boot file name -- unsupported/not needed.
            for(i = 0; i < 64+128+(16-sizeof(MAC_ADDR)); i++)
            {
                UDPGet(s, &Option);
            }

            // Obtain Magic Cookie and verify
            UDPGetArray(s, (uint8_t*)&dw, sizeof(uint32_t));
            if(dw != 0x63538263ul)
                break;

            // Obtain options
            while(1)
            {
                // Get option type
                if(!UDPGet(s, &Option))
                    break;
                if(Option == DHCP_END_OPTION)
                    break;

                // Get option length
                UDPGet(s, &Len);

                // Process option
                switch(Option)
                {
                    case DHCP_MESSAGE_TYPE:
                        UDPGet(s, &i);
                        switch(i)
                        {
                            case DHCP_DISCOVER_MESSAGE:
                                DHCPReplyToDiscovery(&BOOTPHeader, netIx);
                                break;

                            case DHCP_REQUEST_MESSAGE:
                            // NOTE : This #if section was missing from 5.36
                                #if 0// defined(TCPIP_STACK_USE_ZEROCONF_LINK_LOCAL)
                                if ( (BOOTPHeader.ClientIP.Val == 0x00000000u) &&
                                     (dhcpSDcpt[netIx].leaseAvailable == false) )
                                {
                                    // Lease available only to the current lease holder
                                    break;
                                }
                                #endif

                                DHCPReplyToRequest(&BOOTPHeader, bAccept, netIx, bRenew);


                            // NOTE : This #if section was missing from 5.36
                                #if 0 //defined(TCPIP_STACK_USE_ZEROCONF_LINK_LOCAL)
                                dhcpSDcpt[netIx].leaseAvailable = false;
                                #endif

                                break;

                            // Need to handle these if supporting more than one DHCP lease
                            case DHCP_RELEASE_MESSAGE:
                            case DHCP_DECLINE_MESSAGE:
                                break;
                        }
                        break;

                    case DHCP_PARAM_REQUEST_IP_ADDRESS:
                        if(Len == 4u)
                        {
                            // Get the requested IP address and see if it is the one we have on offer.
                            UDPGetArray(s, (uint8_t*)&dw, 4);
                            Len -= 4;
                            bAccept = (dw == dhcpSDcpt[netIx].dhcpNextLease.Val);
                        }
                        break;

                    case DHCP_END_OPTION:
                        UDPDiscard(s);
                        return true;
                }

                // Remove any unprocessed bytes that we don't care about
                while(Len--)
                {
                    UDPGet(s, &i);
                }
            }

            UDPDiscard(s);
            break;
    }
    return true;
}


/*****************************************************************************
  Function:
    static void DHCPReplyToDiscovery(BOOTP_HEADER *Header, int netIx)

  Summary:
    Replies to a DHCP Discover message.

  Description:
    This function replies to a DHCP Discover message by sending out a
    DHCP Offer message.

  Precondition:
    None

  Parameters:
    Header - the BootP header this is in response to.
    netIx   - interface index

  Returns:
      None
  ***************************************************************************/
static void DHCPReplyToDiscovery(BOOTP_HEADER *Header, int netIx)
{
    uint8_t         i;
    TCPIP_NET_IF*     pNetIf;
    int8_t            IndexOfPool;
    UDP_SOCKET      s;
    IPV4_ADDR       ipAddr;
    // Set the correct socket to active and ensure that
    // enough space is available to generate the DHCP response
    s = dhcpSDcpt[netIx].uSkt;
    if(UDPIsTxPutReady(s, 300) < 300u)
        return;

    pNetIf = (TCPIP_NET_IF*)UDPSocketGetNet(s);

    // find in pool
    IndexOfPool = preAssign_ToDHCPClient_FromPool(Header);
    if( -1 == IndexOfPool) return;

    // Begin putting the BOOTP Header and DHCP options
    UDPPut(s, BOOT_REPLY);            // Message Type: 2 (BOOTP Reply)
    // Reply with the same Hardware Type, Hardware Address Length, Hops, and Transaction ID fields
    UDPPutArray(s, (uint8_t*)&(Header->HardwareType), 7);
    UDPPut(s, 0x00);                // Seconds Elapsed: 0 (Not used)
    UDPPut(s, 0x00);                // Seconds Elapsed: 0 (Not used)
    UDPPutArray(s, (uint8_t*)&(Header->BootpFlags), sizeof(Header->BootpFlags));
    UDPPut(s, 0x00);                // Your (client) IP Address: 0.0.0.0 (none yet assigned)
    UDPPut(s, 0x00);                // Your (client) IP Address: 0.0.0.0 (none yet assigned)
    UDPPut(s, 0x00);                // Your (client) IP Address: 0.0.0.0 (none yet assigned)
    UDPPut(s, 0x00);                // Your (client) IP Address: 0.0.0.0 (none yet assigned)
    //UDPPutArray(s, (uint8_t*)&dhcpSDcpt[netIx].dhcpNextLease, sizeof(IPV4_ADDR));    // Lease IP address to give out
    ipAddr = GetIPAddrFromIndex_DhcpPool(IndexOfPool);
    UDPPutArray(s, (uint8_t*)&ipAddr, sizeof(IPV4_ADDR));    // Lease IP address to give out
    UDPPut(s, 0x00);                // Next Server IP Address: 0.0.0.0 (not used)
    UDPPut(s, 0x00);                // Next Server IP Address: 0.0.0.0 (not used)
    UDPPut(s, 0x00);                // Next Server IP Address: 0.0.0.0 (not used)
    UDPPut(s, 0x00);                // Next Server IP Address: 0.0.0.0 (not used)
    UDPPut(s, 0x00);                // Relay Agent IP Address: 0.0.0.0 (not used)
    UDPPut(s, 0x00);                // Relay Agent IP Address: 0.0.0.0 (not used)
    UDPPut(s, 0x00);                // Relay Agent IP Address: 0.0.0.0 (not used)
    UDPPut(s, 0x00);                // Relay Agent IP Address: 0.0.0.0 (not used)
    UDPPutArray(s, (uint8_t*)&(Header->ClientMAC), sizeof(MAC_ADDR));    // Client MAC address: Same as given by client
    for(i = 0; i < 64+128+(16-sizeof(MAC_ADDR)); i++)    // Remaining 10 bytes of client hardware address, server host name: Null string (not used)
        UDPPut(s, 0x00);                                    // Boot filename: Null string (not used)
    UDPPut(s, 0x63);                // Magic Cookie: 0x63538263
    UDPPut(s, 0x82);                // Magic Cookie: 0x63538263
    UDPPut(s, 0x53);                // Magic Cookie: 0x63538263
    UDPPut(s, 0x63);                // Magic Cookie: 0x63538263

    // Options: DHCP Offer
    UDPPut(s, DHCP_MESSAGE_TYPE);
    UDPPut(s, 1);
    UDPPut(s, DHCP_OFFER_MESSAGE);

    // Option: Subnet Mask
    UDPPut(s, DHCP_SUBNET_MASK);
    UDPPut(s, sizeof(IPV4_ADDR));
    UDPPutArray(s, (uint8_t*)&pNetIf->netMask, sizeof(IPV4_ADDR));

    // Option: Lease duration
    UDPPut(s, DHCP_IP_LEASE_TIME);
    UDPPut(s, 4);
    UDPPut(s, (DHCP_LEASE_DURATION>>24) & 0xFF);
    UDPPut(s, (DHCP_LEASE_DURATION>>16) & 0xFF);
    UDPPut(s, (DHCP_LEASE_DURATION>>8) & 0xFF);
    UDPPut(s, (DHCP_LEASE_DURATION) & 0xFF);

    // Option: Server identifier
    UDPPut(s, DHCP_SERVER_IDENTIFIER);
    UDPPut(s, sizeof(IPV4_ADDR));
    UDPPutArray(s, (uint8_t*)&pNetIf->netIPAddr, sizeof(IPV4_ADDR));

    // Option: Router/Gateway address
    UDPPut(s, DHCP_ROUTER);
    UDPPut(s, sizeof(IPV4_ADDR));
    UDPPutArray(s, (uint8_t*)&pNetIf->netIPAddr, sizeof(IPV4_ADDR));

    // Option: DNS server address
    UDPPut(s, DHCP_DNS);
    UDPPut(s, sizeof(IPV4_ADDR));
    UDPPutArray(s, (uint8_t*)&pNetIf->netIPAddr, sizeof(IPV4_ADDR));

    // No more options, mark ending
    UDPPut(s, DHCP_END_OPTION);

    // Add zero padding to ensure compatibility with old BOOTP relays that discard small packets (<300 UDP octets)
    while(UDPGetTxCount(s) < 300u)
        UDPPut(s, 0);

    // Force remote destination address to be the broadcast address, regardless
    // of what the node's source IP address was (to ensure we don't try to
    // unicast to 0.0.0.0).
    UDPSetBcastIPV4Address( s,UDP_BCAST_NETWORK_LIMITED,pNetIf);
    //UDPSetSourceIPAddress(s,IP_ADDRESS_TYPE_IPV4,&(pNetIf->netIPAddr));
    IP_MULTI_ADDRESS tmp_MultiAddr; tmp_MultiAddr.v4Add = pNetIf->netIPAddr;
    UDPSetSourceIPAddress(s,IP_ADDRESS_TYPE_IPV4,&tmp_MultiAddr);

    // Transmit the packet
    UDPFlush(s);
}


/*****************************************************************************
  Function:
    static void DHCPReplyToRequest(BOOTP_HEADER *Header, bool bAccept, int netIx, bool bRenew)

  Summary:
    Replies to a DHCP Request message.

  Description:
    This function replies to a DHCP Request message by sending out a
    DHCP Acknowledge message.

  Precondition:
    None

  Parameters:
    Header - the BootP header this is in response to.
    bAccept - whether or not we've accepted this request
    netIx   - interface index

  Returns:
      None
  
  Internal:
    Needs to support more than one simultaneous lease in the future.
  ***************************************************************************/
static void DHCPReplyToRequest(BOOTP_HEADER *Header, bool bAccept, int netIx, bool bRenew)
{
    uint8_t         i;
    TCPIP_NET_IF*     pNetIf;
    UDP_SOCKET      s;
    int8_t indexOfPool;
    IPV4_ADDR       ipAddr;
    // Set the correct socket to active and ensure that
    // enough space is available to generate the DHCP response
    s = dhcpSDcpt[netIx].uSkt;
    if(UDPIsTxPutReady(s, 300) < 300u)
        return;
    
    pNetIf = (TCPIP_NET_IF*)UDPSocketGetNet(s);

    // Search through all remaining options and look for the Requested IP address field
    // Obtain options
    while(UDPIsGetReady(s))
    {
        uint8_t Option, Len;
        uint32_t dw;
        MAC_ADDR tmp_MacAddr;

        // Get option type
        if(!UDPGet(s, &Option))
            break;
        if(Option == DHCP_END_OPTION)
            break;

        // Get option length
        UDPGet(s, &Len);

        // Process option
        if(bRenew)
        {
            if((Option == DHCP_PARAM_REQUEST_CLIENT_ID) && (Len == 7u))
            {
                // Get the requested IP address and see if it is the one we have on offer.  If not, we should send back a NAK, but since there could be some other DHCP server offering this address, we'll just silently ignore this request.
                UDPGet(s, &i);
                UDPGetArray(s, (uint8_t*)&tmp_MacAddr, 6);
                Len -= 7;
                indexOfPool = getIndexByMacaddr_DhcpPool(&tmp_MacAddr);//(&tmp_MacAddr,(IPV4_ADDR*)&Header->);
                if(-1 != indexOfPool)
                {
                    if(GetIPAddrFromIndex_DhcpPool(indexOfPool).Val ==  Header->ClientIP.Val)
                        postAssign_ToDHCPClient_FromPool(&tmp_MacAddr, &(Header->ClientIP));
                    else
                        bAccept = false;
                }
                else
                {
                    bAccept = false;
                }

                break;
            }
        }
        else
        {
            if((Option == DHCP_PARAM_REQUEST_IP_ADDRESS) && (Len == 4u))
            {
                // Get the requested IP address and see if it is the one we have on offer.  If not, we should send back a NAK, but since there could be some other DHCP server offering this address, we'll just silently ignore this request.
                UDPGetArray(s, (uint8_t*)&dw, 4);
                Len -= 4;
                indexOfPool = postAssign_ToDHCPClient_FromPool(&(Header->ClientMAC),(IPV4_ADDR*)&dw);
                if( -1 == indexOfPool)
                {
                    bAccept = false;
                }
                break;
            }
        }
        // Remove the unprocessed bytes that we don't care about
        while(Len--)
        {
            UDPGet(s, &i);
        }
    }

    // Begin putting the BOOTP Header and DHCP options
    UDPPut(s, BOOT_REPLY);            // Message Type: 2 (BOOTP Reply)
    // Reply with the same Hardware Type, Hardware Address Length, Hops, and Transaction ID fields
    UDPPutArray(s, (uint8_t*)&(Header->HardwareType), 7);
    UDPPut(s, 0x00);                // Seconds Elapsed: 0 (Not used)
    UDPPut(s, 0x00);                // Seconds Elapsed: 0 (Not used)
    UDPPutArray(s, (uint8_t*)&(Header->BootpFlags), sizeof(Header->BootpFlags));
    UDPPutArray(s, (uint8_t*)&(Header->ClientIP), sizeof(IPV4_ADDR));// Your (client) IP Address:
    if(bAccept)        ipAddr = GetIPAddrFromIndex_DhcpPool(indexOfPool);
    else             ipAddr.Val=0u;
    UDPPutArray(s, (uint8_t*)&ipAddr, sizeof(IPV4_ADDR));    // Lease IP address to give out
    UDPPut(s, 0x00);                // Next Server IP Address: 0.0.0.0 (not used)
    UDPPut(s, 0x00);                // Next Server IP Address: 0.0.0.0 (not used)
    UDPPut(s, 0x00);                // Next Server IP Address: 0.0.0.0 (not used)
    UDPPut(s, 0x00);                // Next Server IP Address: 0.0.0.0 (not used)
    UDPPut(s, 0x00);                // Relay Agent IP Address: 0.0.0.0 (not used)
    UDPPut(s, 0x00);                // Relay Agent IP Address: 0.0.0.0 (not used)
    UDPPut(s, 0x00);                // Relay Agent IP Address: 0.0.0.0 (not used)
    UDPPut(s, 0x00);                // Relay Agent IP Address: 0.0.0.0 (not used)
    UDPPutArray(s, (uint8_t*)&(Header->ClientMAC), sizeof(MAC_ADDR));    // Client MAC address: Same as given by client
    for(i = 0; i < 64+128+(16-sizeof(MAC_ADDR)); i++)    // Remaining 10 bytes of client hardware address, server host name: Null string (not used)
        UDPPut(s, 0x00);                                    // Boot filename: Null string (not used)
    UDPPut(s, 0x63);                // Magic Cookie: 0x63538263
    UDPPut(s, 0x82);                // Magic Cookie: 0x63538263
    UDPPut(s, 0x53);                // Magic Cookie: 0x63538263
    UDPPut(s, 0x63);                // Magic Cookie: 0x63538263

    // Options: DHCP lease ACKnowledge
    if(bAccept)
    {
        UDPPut(s, DHCP_OPTION_ACK_MESSAGE);
        UDPPut(s, 1);
        UDPPut(s, DHCP_ACK_MESSAGE);
    }
    else    // Send a NACK
    {
        UDPPut(s, DHCP_OPTION_ACK_MESSAGE);
        UDPPut(s, 1);
        UDPPut(s, DHCP_NAK_MESSAGE);
    }

    // Option: Lease duration
    UDPPut(s, DHCP_IP_LEASE_TIME);
    UDPPut(s, 4);
    UDPPut(s, (DHCP_LEASE_DURATION>>24) & 0xFF);
    UDPPut(s, (DHCP_LEASE_DURATION>>16) & 0xFF);
    UDPPut(s, (DHCP_LEASE_DURATION>>8) & 0xFF);
    UDPPut(s, (DHCP_LEASE_DURATION) & 0xFF);

    // Option: Server identifier
    UDPPut(s, DHCP_SERVER_IDENTIFIER);
    UDPPut(s, sizeof(IPV4_ADDR));
    UDPPutArray(s, (uint8_t*)&pNetIf->netIPAddr, sizeof(IPV4_ADDR));

    // Option: Subnet Mask
    UDPPut(s, DHCP_SUBNET_MASK);
    UDPPut(s, sizeof(IPV4_ADDR));
    UDPPutArray(s, (uint8_t*)&pNetIf->netMask, sizeof(IPV4_ADDR));

    // Option: Router/Gateway address
    UDPPut(s, DHCP_ROUTER);
    UDPPut(s, sizeof(IPV4_ADDR));
    UDPPutArray(s, (uint8_t*)&pNetIf->netIPAddr, sizeof(IPV4_ADDR));

    // Option: DNS server address
    UDPPut(s, DHCP_DNS);
    UDPPut(s, sizeof(IPV4_ADDR));
    UDPPutArray(s, (uint8_t*)&pNetIf->netIPAddr, sizeof(IPV4_ADDR));

    // No more options, mark ending
    UDPPut(s, DHCP_END_OPTION);

    // Add zero padding to ensure compatibility with old BOOTP relays that discard small packets (<300 UDP octets)
    while(UDPGetTxCount(s) < 300u)
        UDPPut(s, 0);

    // Force remote destination address to be the broadcast address, regardless
    // of what the node's source IP address was (to ensure we don't try to
    // unicast to 0.0.0.0).
    if(false == bRenew)
        UDPSetBcastIPV4Address( s,UDP_BCAST_NETWORK_LIMITED,pNetIf);
    //UDPSetSourceIPAddress(s,IP_ADDRESS_TYPE_IPV4,&(pNetIf->netIPAddr));
    IP_MULTI_ADDRESS tmp_MultiAddr; tmp_MultiAddr.v4Add = pNetIf->netIPAddr;
    UDPSetSourceIPAddress(s,IP_ADDRESS_TYPE_IPV4,&tmp_MultiAddr);
    // Transmit the packet
    UDPFlush(s);
}
static int8_t getIndexByMacaddr_DhcpPool(const MAC_ADDR *MacAddr)
{
    int i;
    if(false == isMacAddr_Effective(MacAddr)) return -1;
    for(i=0;i<MAX_DCHP_CLIENTS_NUMBER;i++)
    {
        if(true == Compare_MAC_addr(&DhcpIpPool[i].ClientMAC, MacAddr)) return i;
    }
    return -1;
}
static bool isIpAddrInPool(IPV4_ADDR ipaddr)
{
    int i;
    for(i=0;i<MAX_DCHP_CLIENTS_NUMBER;i++)
    {
        if(DhcpIpPool[i].Client_Addr.Val == ipaddr.Val)
        {
            return true;
        }
    }
    return false;
}

static IPV4_ADDR GetIPAddrFromIndex_DhcpPool(int8_t index)
{
    IPV4_ADDR tmpIpAddr;
    tmpIpAddr.Val=0u;
    if(index > MAX_DCHP_CLIENTS_NUMBER) return tmpIpAddr;
    return DhcpIpPool[index].Client_Addr;
}
static bool Compare_MAC_addr(const MAC_ADDR *macAddr1, const MAC_ADDR *macAddr2)
{
    int i;
    for(i=0;i<6;i++)
    {
        if(macAddr1->v[i] != macAddr2->v[i]) return false;
    }
    return true;
}
static bool isMacAddr_Effective(const MAC_ADDR *macAddr)
{
    int i;
    for(i=0;i<6;i++)
    {
        if(macAddr->v[i] != 0) return true;
    }
    return false;
}
static int8_t preAssign_ToDHCPClient_FromPool(BOOTP_HEADER *Header)
{
    int i;
    // if MAC==00:00:00:00:00:00, then return -1
    if(false == isMacAddr_Effective(&(Header->ClientMAC))) return -1;
    // Find in Pool, look for the same MAC addr
    for(i=0;i<MAX_DCHP_CLIENTS_NUMBER;i++)
    {
        if(true == Compare_MAC_addr(&DhcpIpPool[i].ClientMAC, &Header->ClientMAC))
        {
            //if(true == DhcpIpPool[i].isUsed) return -1;
            //DhcpIpPool[i].isUsed = true;
            return i;
        }
    }
    // Find in pool, look for a empty MAC addr
    for(i=0;i<MAX_DCHP_CLIENTS_NUMBER;i++)
    {
        if(false == isMacAddr_Effective(&DhcpIpPool[i].ClientMAC))
        {  // this is empty MAC in pool
            int j;
            for(j=0;j<6;j++)  DhcpIpPool[i].ClientMAC.v[j] = Header->ClientMAC.v[j];
            //DhcpIpPool[i].isUsed = true;
            return i;
        }
    }
    #if 1
    // Find in pool, look for a unsued item
    for(i=0;i<MAX_DCHP_CLIENTS_NUMBER;i++)
    {
        if(false == DhcpIpPool[i].isUsed)
        {  // this is unused MAC in pool
            int j;
            for(j=0;j<6;j++)  DhcpIpPool[i].ClientMAC.v[j] = Header->ClientMAC.v[j];
            //DhcpIpPool[i].isUsed = true;
            return i;
        }
    }
    #endif
    return -1;

}
static int8_t postAssign_ToDHCPClient_FromPool(MAC_ADDR *macAddr, IPV4_ADDR *ipv4Addr)
{
    int i;
    for(i=0;i<MAX_DCHP_CLIENTS_NUMBER;i++)
    {
        if(ipv4Addr->Val == DhcpIpPool[i].Client_Addr.Val)
        {
            if(true == Compare_MAC_addr(macAddr,&DhcpIpPool[i].ClientMAC))
            {
                DhcpIpPool[i].isUsed = true;
                DhcpIpPool[i].Client_Lease_Time = DHCP_LEASE_DURATION;
                return i;
            }
            else
                return -1;
        }
    }
    return -1;
}
static void renew_dhcps_Pool(void)
{
    static uint32_t dhcp_timer=0;
    uint32_t current_timer = SYS_TICK_Get()/SYS_TICKS_PER_SECOND;
    int i;
    if((current_timer - dhcp_timer)<1)
    {
        return;
    }
    dhcp_timer = current_timer;
    for(i=0;i<MAX_DCHP_CLIENTS_NUMBER;i++)
    {
        if(DhcpIpPool[i].isUsed == false) continue;

        if(DhcpIpPool[i].Client_Lease_Time != 0) DhcpIpPool[i].Client_Lease_Time --;
        if(DhcpIpPool[i].Client_Lease_Time == 0)
        {
            DhcpIpPool[i].isUsed = false;
            DhcpIpPool[i].ClientMAC.v[0]=00;
            DhcpIpPool[i].ClientMAC.v[1]=00;
            DhcpIpPool[i].ClientMAC.v[2]=00;
            DhcpIpPool[i].ClientMAC.v[3]=00;
            DhcpIpPool[i].ClientMAC.v[4]=00;
            DhcpIpPool[i].ClientMAC.v[5]=00;
        }
    }
}
bool DHCPServerDisable(TCPIP_NET_HANDLE hNet)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNet(hNet);
    if(pNetIf)
    {
        //  Now stop DHCP server
        int  netIx = _TCPIPStackNetIx(pNetIf);

        dhcpSDcpt[netIx].smServer = DHCP_SERVER_DISABLE;
        if( dhcpSDcpt[netIx].uSkt != INVALID_UDP_SOCKET)
        {
            UDPClose(dhcpSDcpt[netIx].uSkt);
            dhcpSDcpt[netIx].uSkt = INVALID_UDP_SOCKET;
        }

        return true;
    }

    return false;
}

bool DHCPServerEnable(TCPIP_NET_HANDLE hNet)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNet(hNet);
    if(pNetIf)
    {
        //  Now Enable DHCP server
        int  netIx = _TCPIPStackNetIx(pNetIf);

        if(dhcpSDcpt[netIx].smServer == DHCP_SERVER_DISABLE)
        {
            dhcpSDcpt[netIx].smServer = DHCP_SERVER_OPEN_SOCKET;
        }

        return true;
    }

    return false;
}
bool DHCPServerIsEnabled(TCPIP_NET_HANDLE hNet)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNet(hNet);
    if(pNetIf)
    {
        int  netIx = _TCPIPStackNetIx(pNetIf);
        return (dhcpSDcpt[netIx].smServer == DHCP_SERVER_DISABLE)?false:true;
    }

    return false;
}


#else
bool DHCPServerDisable(TCPIP_NET_HANDLE hNet){return false;}
bool DHCPServerEnable(TCPIP_NET_HANDLE hNet){return false;}
bool DHCPServerIsEnabled(TCPIP_NET_HANDLE hNet) {return false;}


#endif //#if defined(TCPIP_STACK_USE_DHCP_SERVER)
