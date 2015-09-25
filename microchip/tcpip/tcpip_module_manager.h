/*******************************************************************************
  TCPIP modules manager file

  Summary:
    Internal TCPIP stack module manager file

  Description:
    This header file contains the function prototypes and definitions of the
    TCPIP stack manager services
*******************************************************************************/

/*******************************************************************************
FileName:   tcpip_module_manager.h
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

#ifndef __TCPIP_MODULE_MANAGER_H_
#define __TCPIP_MODULE_MANAGER_H_

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>


// definitions
//


// ************* stack supported modules and their attached functionality **************
//

// ******* table with TCPIP stack modules **************
// We use directly the functions names (pointers) rather than providing registration functions
// so that we can create this table in const space
//

// initialization function
// if the module has initialization to do, this function
// will be called. It should return a result to indicate
// if the initialization was successful. If not, the
// interface will not be completed.
typedef bool    (*tcpipModuleInitFunc)(const TCPIP_STACK_MODULE_CTRL* const, const void* );

// de-initialization function
// if the module needs to clean up when the module is
// brought down, this function will be called. It should
// return a result to indicate that everything has been
// cleaned up.
typedef void    (*tcpipModuleDeInitFunc)(const TCPIP_STACK_MODULE_CTRL * const);


// descriptor of an TCPIP stack module entry
// module that's part of the stack
// each module has an ID and init/deinit functions
//
typedef struct
{
    TCPIP_STACK_MODULE       moduleId;           // module identification
    tcpipModuleInitFunc      initFunc;           // initialization function
    tcpipModuleDeInitFunc    deInitFunc;         // de-initialization function
}TCPIP_STACK_MODULE_ENTRY;


static const TCPIP_STACK_MODULE_ENTRY  TCPIP_STACK_MODULE_ENTRY_TBL [] =
{
    //ModuleID                  //InitFunc                                      //DeInitFunc
#if defined(TCPIP_STACK_USE_IPV4)
    {TCPIP_MODULE_IPV4,          (tcpipModuleInitFunc)TCPIP_IPV4_Initialize,    TCPIP_IPV4_DeInitialize},       // TCPIP_MODULE_IPV4,
#endif
#if defined(TCPIP_STACK_USE_ICMP_CLIENT) || defined(TCPIP_STACK_USE_ICMP_SERVER)
    {TCPIP_MODULE_ICMP,          (tcpipModuleInitFunc)ICMPInitialize,           ICMPDeinitialize},              // TCPIP_MODULE_ICMP,
#endif
    {TCPIP_MODULE_ARP,           (tcpipModuleInitFunc)ARPInitialize,            ARPDeinitialize},               // TCPIP_MODULE_ARP,
#if defined(TCPIP_STACK_USE_IPV6)
    {TCPIP_MODULE_IPV6,          (tcpipModuleInitFunc)TCPIP_IPV6_Initialize,    TCPIP_IPV6_Deinitialize},       // TCPIP_MODULE_IPV6
    {TCPIP_MODULE_ICMPV6,        (tcpipModuleInitFunc)TCPIP_ICMPV6_Initialize,  TCPIP_ICMPV6_Deinitialize},     // TCPIP_MODULE_ICMPV6
    {TCPIP_MODULE_NDP,           (tcpipModuleInitFunc)TCPIP_NDP_Initialize,     TCPIP_NDP_Deinitialize},        // TCPIP_MODULE_NDP
#endif
#if defined(TCPIP_STACK_USE_UDP)
    {TCPIP_MODULE_UDP,           (tcpipModuleInitFunc)UDPInit,                  UDPDeInit},                     // TCPIP_MODULE_UDP,
#endif
#if defined(TCPIP_STACK_USE_TCP)
    {TCPIP_MODULE_TCP,           (tcpipModuleInitFunc)TCPInit,                  TCPDeInit},                     //  TCPIP_MODULE_TCP,
#endif
#if defined(TCPIP_STACK_USE_DHCP_CLIENT)
    {TCPIP_MODULE_DHCP_CLIENT,   (tcpipModuleInitFunc)DHCPInit,                 DHCPDeInit},                    // TCPIP_MODULE_DHCP_CLIENT,
#endif
#if defined(TCPIP_STACK_USE_DHCP_SERVER)
    {TCPIP_MODULE_DHCP_SERVER,   (tcpipModuleInitFunc)DHCPServerInit,           DHCPServerDeInit},              // TCPIP_MODULE_DHCP_SERVER,
#endif
#if defined(TCPIP_STACK_USE_ANNOUNCE)
    {TCPIP_MODULE_ANNOUNCE,      (tcpipModuleInitFunc)TCPIP_ANNOUNCE_Init,      TCPIP_ANNOUNCE_DeInit},         // TCPIP_MODULE_ANNOUNCE,
#endif
#if defined(TCPIP_STACK_USE_DNS)
    {TCPIP_MODULE_DNS_CLIENT,    (tcpipModuleInitFunc)DNSClientInit,            DNSClientDeInit},               // TCPIP_MODULE_DNS_CLIENT,
#endif
#if defined(TCPIP_STACK_USE_NBNS)
    {TCPIP_MODULE_NBNS,          (tcpipModuleInitFunc)NBNSInit,                 NBNSDeInit},                    // TCPIP_MODULE_NBNS
#endif
#if defined(TCPIP_STACK_USE_SMTP_CLIENT)
    {TCPIP_MODULE_SMTP_CLIENT,   (tcpipModuleInitFunc)SMTPClientInit,           SMTPClientDeInit},              // TCPIP_MODULE_SMTP_CLIENT,
#endif
#if defined(TCPIP_STACK_USE_SNTP_CLIENT)
    {TCPIP_MODULE_SNTP,          (tcpipModuleInitFunc)SNTPInit,                 SNTPDeInit},                    // TCPIP_MODULE_SNTP,
#endif
#if defined(TCPIP_STACK_USE_BERKELEY_API)
    {TCPIP_MODULE_BERKELEY,      (tcpipModuleInitFunc)BerkeleySocketInit,       BerkeleySocketDeInit},          // TCPIP_MODULE_BERKELEY,
#endif
#if defined(TCPIP_STACK_USE_HTTP2_SERVER)
    {TCPIP_MODULE_HTTP_SERVER,   (tcpipModuleInitFunc)HTTPInit,                 HTTPDeInit},                    // TCPIP_MODULE_HTTP_SERVER,
#endif
#if defined(TCPIP_STACK_USE_TELNET_SERVER)
    {TCPIP_MODULE_TELNET_SERVER, (tcpipModuleInitFunc)TelnetInit,               TelnetDeInit},                  // TCPIP_MODULE_TELNET_SERVER,
#endif
#if defined(TCPIP_STACK_USE_SSL_SERVER) || defined(TCPIP_STACK_USE_SSL_CLIENT)
    {TCPIP_MODULE_SSL,           (tcpipModuleInitFunc)SSLInit,                  SSLDeInit},                     // TCPIP_MODULE_SSL,
#endif
#if defined(TCPIP_STACK_USE_RSA)
    {TCPIP_MODULE_RSA,           (tcpipModuleInitFunc)RSAInit,                  RSADeInit},                     // TCPIP_MODULE_RSA,
#endif
#if defined(TCPIP_STACK_USE_FTP_SERVER)
    {TCPIP_MODULE_FTP_SERVER,    (tcpipModuleInitFunc)TCPIP_FTP_ServerInit,     TCPIP_FTP_DeInit},              // TCPIP_MODULE_FTP_SERVER,
#endif
#if defined(TCPIP_STACK_USE_SNMP_SERVER)
    {TCPIP_MODULE_SNMP_SERVER,   (tcpipModuleInitFunc)SNMPInit,                 SNMPDeInit},                    // TCPIP_MODULE_SNMP_SERVER,
#endif
#if defined(TCPIP_STACK_USE_DNS_SERVER)
    {TCPIP_MODULE_DNS_SERVER,    (tcpipModuleInitFunc)DNSServerInit,            DNSServerDeInit},               // TCPIP_MODULE_DNS_SERVER,
#endif
#if defined(TCPIP_STACK_USE_DYNAMICDNS_CLIENT)
    {TCPIP_MODULE_DYNDNS_CLIENT, (tcpipModuleInitFunc)DDNSInit,                 DDNSDeInit},                    // TCPIP_MODULE_DYNDNS_CLIENT,
#endif
#if defined(TCPIP_STACK_USE_REBOOT_SERVER)
    {TCPIP_MODULE_REBOOT_SERVER, (tcpipModuleInitFunc)TCPIP_RebootInit,         TCPIP_RebootDeInit},            // TCPIP_MODULE_REBOOT_SERVER,
#endif
#if defined(TCPIP_STACK_USE_ZEROCONF_LINK_LOCAL)
    {TCPIP_MODULE_ZCLL,          (tcpipModuleInitFunc)ZeroconfLLInitialize,     ZeroconfLLDeinitialize},        // TCPIP_MODULE_ZCLL,
#if defined(TCPIP_STACK_USE_ZEROCONF_MDNS_SD)
    {TCPIP_MODULE_MDNS,          (tcpipModuleInitFunc)mDNSInitialize,           mDNSDeinitialize},              // TCPIP_MODULE_MDNS,
#endif
#endif
#if defined(TCPIP_STACK_COMMAND_ENABLE)
    {TCPIP_MODULE_TCPIP_COMMAND, (tcpipModuleInitFunc)TCPIPCommandsInit,        TCPIPCommandsDeInit},           //    TCPIP_MODULE_TCPIP_COMMAND
#endif
#if defined(TCPIP_STACK_USE_IPERF)
    {TCPIP_MODULE_TCPIP_IPERF,   (tcpipModuleInitFunc)TCPIP_IPERF_Initialize,   TCPIP_IPERF_Deinitialize},      //    TCPIP_MODULE_TCPIP_IPERF
#endif
        // Add other stack modules here

};

// *********** a stack module that exposes an asynchronous handle ****************

// function that is called by the stack manager to detect
// if the corresponding module has some asynchronous event
// pending. It should return true if an asynchronous
// event is pending, false otherwise.
// This could be a timer event created by the module
// or other specific module events
typedef bool    (*tcpipModuleAsyncPending)(void);

// asynchronous event handler
// the stack calls it when there's an asynchronous event pending
// it should clear the pending status
typedef void    (*tcpipModuleAsyncHandler)(void);


typedef struct
{
    tcpipModuleAsyncPending     asyncPending;       // returns true if attention needed
    tcpipModuleAsyncHandler     asyncHandler;       // attention handler
}TCPIP_STACK_ASYNC_MODULE_ENTRY;

// table containing all the modules having asynchronous handlers
// although this table does not contain the module ID
// it is maintained in the same order as TCPIP_STACK_MODULE_ENTRY_TBL!

static const TCPIP_STACK_ASYNC_MODULE_ENTRY  TCPIP_STACK_MODULE_ASYNC_TBL [] =
{
    // asyncPending       // asyncHandler
#if defined(TCPIP_STACK_USE_IPV4)
    {TCPIP_IPV4_TaskPending,     TCPIP_IPV4_Task},              // TCPIP_MODULE_IPV4,
#endif
    {ARPTaskPending,             ARPTask},                      // TCPIP_MODULE_ARP,
#if defined(TCPIP_STACK_USE_IPV6)
    {TCPIP_IPV6_InitTaskPending, TCPIP_IPV6_InitializeTask},    // TCPIP_MODULE_IPV6
    {TCPIP_IPV6_TaskPending,     TCPIP_IPV6_Task},              // TCPIP_MODULE_IPV6
    {TCPIP_NDP_DAD_TaskPending,  TCPIP_NDP_DAD_Task},           // TCPIP_MODULE_NDP
    {TCPIP_NDP_RS_TaskPending,   TCPIP_NDP_RS_Task},            // TCPIP_MODULE_NDP
    {TCPIP_NDP_NUD_TaskPending,  TCPIP_NDP_NUD_Task},           // TCPIP_MODULE_NDP
#endif
#if defined(TCPIP_STACK_USE_TCP)
    {TCPTaskPending,             TCPTick},                      // TCPIP_MODULE_TCP,
#endif
#if defined(TCPIP_STACK_USE_ANNOUNCE)
    {TCPIP_ANNOUNCE_TaskPending, TCPIP_ANNOUNCE_Send},          // TCPIP_MODULE_ANNOUNCE,
#endif
#if defined(TCPIP_STACK_USE_DNS) && (DNS_CLIENT_VERSION_NO >= 2)
    {DNSClientTaskPending,       DNSClientTask},                // TCPIP_MODULE_DNS_CLIENT,
#endif
#if defined(TCPIP_STACK_USE_SSL_SERVER) || defined(TCPIP_STACK_USE_SSL_CLIENT)
    {SSLTaskPending,             SSLTask},                      // TCPIP_MODULE_SSL,
#endif

#if defined(TCPIP_IF_MRF24W)
    {WiFiAsyncTaskPending, WiFiAsyncTask},
#endif // TCPIP_IF_MRF24W

#if defined(TCPIP_STACK_USE_DHCP_SERVER)
	{DHCPServerTaskPending,	DHCPLeaseTimeTask},				// TCPIP_MODULE_DHCP_SERVER,
#endif



// Add other needed services here needing asynchronous handlers

};

// *************** a stack module that exposes an asynchronous handle *************


// synchronous event handler
// the stack calls it when there's an MAC event pending
typedef bool    (*tcpipModuleSyncHandler)(TCPIP_NET_IF* pNetIf);



typedef struct
{
    tcpipModuleSyncHandler      syncHandler;        // synchronous handler
}TCPIP_STACK_SYNC_MODULE_ENTRY;

// table containing all the modules having synchronous handlers
// although this table does not contain the module ID
// it is maintained in the same order as TCPIP_STACK_MODULE_ENTRY_TBL!

#define temp_synch(handler)      ((tcpipModuleSyncHandler)handler)

static const TCPIP_STACK_SYNC_MODULE_ENTRY  TCPIP_STACK_MODULE_SYNC_TBL [] =
{
    // syncHandler
#if defined(TCPIP_STACK_USE_DHCP_SERVER)
    {DHCPServerTask},                                   // TCPIP_MODULE_DHCP_SERVER,
#endif
#if defined(TCPIP_STACK_USE_ANNOUNCE)
    {TCPIP_ANNOUNCE_Task},                              // TCPIP_MODULE_ANNOUNCE,
#endif
#if defined(TCPIP_STACK_USE_DNS) && (DNS_CLIENT_VERSION_NO < 2)
    {temp_synch(DNSClientTask)},                        // TCPIP_MODULE_DNS_CLIENT
#endif
#if defined(TCPIP_STACK_USE_NBNS)
    {NBNSTask},                                         // TCPIP_MODULE_NBNS
#endif
#if defined(TCPIP_STACK_USE_SMTP_CLIENT)
    {temp_synch(SMTPClientTask)},                       // TCPIP_MODULE_SMTP_CLIENT,
#endif
#if defined(TCPIP_STACK_USE_SNTP_CLIENT)
    {SNTPClient},                                       // TCPIP_MODULE_SNTP,
#endif
#if defined(TCPIP_STACK_USE_HTTP2_SERVER)
    {temp_synch(HTTPServer)},                           // TCPIP_MODULE_HTTP_SERVER,
#endif
#if defined(TCPIP_STACK_USE_TELNET_SERVER)
    {TelnetTask},                                       // TCPIP_MODULE_TELNET_SERVER,
#endif
#if defined(TCPIP_STACK_USE_FTP_SERVER)
    {TCPIP_FTP_Server},                                 // TCPIP_MODULE_FTP_SERVER,
#endif
#if defined(TCPIP_STACK_USE_SNMP_SERVER)
    {SNMPTask},                                         // TCPIP_MODULE_SNMP_SERVER,
#endif
#if defined(TCPIP_STACK_USE_DNS_SERVER)
    {temp_synch(DNSServerTask)},                        // TCPIP_MODULE_DNS_SERVER,
#endif
#if defined(TCPIP_STACK_USE_DYNAMICDNS_CLIENT)
    {temp_synch(DDNSTask)},                             // TCPIP_MODULE_DYNDNS_CLIENT,
#endif
#if defined(TCPIP_STACK_USE_REBOOT_SERVER)
    {TCPIP_RebootTask},                                 // TCPIP_MODULE_REBOOT_SERVER,
#endif
#if defined(TCPIP_STACK_USE_ZEROCONF_LINK_LOCAL)
    {ZeroconfLLProcess},                                // TCPIP_MODULE_ZCLL,
#if defined(TCPIP_STACK_USE_ZEROCONF_MDNS_SD)
    {mDNSProcess},                                      // TCPIP_MODULE_MDNS,
#endif
#endif
#if defined(TCPIP_STACK_USE_IPERF)
    {temp_synch(TCPIP_IPERF_Task)},                     // TCPIP_MODULE_TCPIP_IPERF
#endif
    // Add other needed services having synchronous handlers

};

// *************** a stack module that exposes an get configuration function *************


// get configuration function
// each module could have its own function to retrieve the current configuration
typedef size_t (*tcpipModuleGetConfig)(TCPIP_STACK_MODULE modId, void* configBuff, size_t buffSize, size_t* pConfigSize);


typedef struct
{
    TCPIP_STACK_MODULE      moduleId;       // module identification
    tcpipModuleGetConfig    getConfig;      // get Config function
}TCPIP_STACK_GET_CONFIG_MODULE_ENTRY;

// table containing all the modules having get configuration handlers
static const TCPIP_STACK_GET_CONFIG_MODULE_ENTRY  TCPIP_STACK_MODULE_GET_CONFIG_ENTRY_TBL [] =
{
    //moduleId                  //getConfig
#if defined(TCPIP_IF_PIC32INT)
    {TCPIP_MODULE_MAC_PIC32INT, PIC32MACGetConfig},          // TCPIP_MODULE_MAC_PIC32INT
#endif
#if defined(TCPIP_IF_MRF24W)
    {TCPIP_MODULE_MAC_MRF24W,   MRF24W_MACGetConfig},        // TCPIP_MODULE_MAC_MRF24W
#endif

    // Add other modules get config handlers
};

// **************** descriptor of an TCPIP stack MAC entry *************
// (MAC module supported by the stack)
//

// MAC initialization function
// Returns a result to indicate that the initialization was successful.
// If it fails, the stack won't turn up that interface
// If the operation needs to wait for the hardware, the initialization
// fuction can return a pending code
typedef TCPIP_MAC_RES    (*tcpipMacInitFunc)(const TCPIP_STACK_MODULE_CTRL* const, const void* );

// MAC de-initialization function
// Returns a result to indicate that everything has been
// cleaned up.
typedef TCPIP_MAC_RES    (*tcpipMacDeInitFunc)(const TCPIP_STACK_MODULE_CTRL * const);

// function to open a MAC and get a client handle
typedef TCPIP_MAC_HANDLE    (*tcpipMacOpenFunc)( TCPIP_STACK_MODULE macId );



typedef struct
{
    TCPIP_STACK_MODULE          moduleId;           // MAC module identification
    const char*                 interfaceName;      // corresponding interface name
    tcpipMacInitFunc            initFunc;           // initialization function
    tcpipMacDeInitFunc          deInitFunc;         // de-initialization function
    tcpipMacOpenFunc            openFunc;           // open function
}TCPIP_STACK_MODULE_MAC_ENTRY;

// table with TCPIP stack MAC modules
// We use functions pointers rather than variable pointers so that we can create this table in const
// Also, the name of the functions is fixed rather than providing registration functions
//

static const TCPIP_STACK_MODULE_MAC_ENTRY  TCPIP_STACK_MODULE_MAC_ENTRY_TBL [] =
{
    //ModuleID                  //interfaceName               //InitFunc                                       //DeInitFunc                // openFunc
#if defined(TCPIP_IF_ENC28J60)
    {TCPIP_MODULE_MAC_ENCJ60,   TCPIP_STACK_IF_NAME_ENCJ60,   (tcpipMacInitFunc)ENC28_MACInitialize,        ENC28_MACDeinitialize,      ENC28_MACOpen},         // TCPIP_MODULE_MAC_ENCJ60
#endif
#if defined(TCPIP_IF_ENCX24J600)
    {TCPIP_MODULE_MAC_ENCJ600,  TCPIP_STACK_IF_NAME_ENCJ600,  (tcpipMacInitFunc)ENCX24_MACInitialize,       ENCX24_MACDeinitialize,     ENCX24_MACOpen},        // TCPIP_MODULE_MAC_ENCJ600
#endif
#if defined(TCPIP_IF_97J60)
    {TCPIP_MODULE_MAC_97J60,    TCPIP_STACK_IF_NAME_97J60,    (tcpipMacInitFunc)PIC87J60_MACInitialize,     PIC87J60_MACDeinitialize,   PIC87J60_MACOpen},      // TCPIP_MODULE_MAC_97J60
#endif
#if defined(TCPIP_IF_PIC32INT)
    {TCPIP_MODULE_MAC_PIC32INT, TCPIP_STACK_IF_NAME_PIC32INT, (tcpipMacInitFunc)PIC32MACInitialize,         PIC32MACDeinitialize,       PIC32MACOpen},          // TCPIP_MODULE_MAC_PIC32INT
#endif
#if defined(TCPIP_IF_MRF24W)
    {TCPIP_MODULE_MAC_MRF24W,   TCPIP_STACK_IF_NAME_MRF24W,   (tcpipMacInitFunc)MRF24W_MACInitialize,       MRF24W_MACDeinitialize,     MRF24W_MACOpen},        // TCPIP_MODULE_MAC_MRF24W
#endif
};



// Connection event handler definition.
// The stack calls the handler when a new connection event occurs.
// Note that this call will carry only connection events!
typedef void    (*tcpipModuleConnHandler)(TCPIP_NET_IF* pNetIf, TCPIP_EVENT connEvent);



// Since the modules that need connection notification is
// known to the stack manager no dynamic approach is taken.
// But simply a call table is maintained.
static const tcpipModuleConnHandler  TCPIP_STACK_CONN_EVENT_TBL [] =
{
#if defined(TCPIP_STACK_USE_DHCP_CLIENT)
    DHCPConnectionHandler,
#endif // defined(TCPIP_STACK_USE_DHCP_CLIENT)

    // add other needed handlers here
};



#endif //  __TCPIP_MODULE_MANAGER_H_








