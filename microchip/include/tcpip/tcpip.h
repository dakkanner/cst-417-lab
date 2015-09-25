/*******************************************************************************
  Microchip TCP/IP Stack Include File

  Summary:

    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  tcpip.h 
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

#ifndef __TCPIP_H__
#define __TCPIP_H__

#define TCPIP_STACK_VERSION 		"v6.0.04-Beta"		// TCP/IP stack version

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "hardware_config.h"
        
#include "system/system_services.h"

#include "tcpip/tcpip_types.h"

/*******************************************************************
 * List of the TCPIP stack supported addresses
 *   The following enumeration lists all the address types supported
 *   by the TCPIP stack.
 *******************************************************************/

// Definition to represent an IPv4 address
typedef union
{
    uint32_t Val;
    uint16_t w[2];
    uint8_t  v[4];
} IPV4_ADDR;
// backwards compatibility definition
typedef IPV4_ADDR   IP_ADDR;

// Definition to represent an IPv6 address
typedef union
{
    uint8_t  v[16];
    uint16_t w[8];
    uint32_t d[4];
} IPV6_ADDR;

// Definition of the supported address types
typedef enum
{
    IP_ADDRESS_TYPE_ANY = 0,    // either IPv4 or IPv6; unspecified;
    IP_ADDRESS_TYPE_IPV4,       // IPv4 address type
    IP_ADDRESS_TYPE_IPV6        // IPv6 address type
}IP_ADDRESS_TYPE;
    

// definition to represent multiple IP addresses
typedef union
{
    IPV4_ADDR v4Add;
    IPV6_ADDR v6Add;
}IP_MULTI_ADDRESS;
    
// IPv6 addresses definitions

typedef enum
{
    IPV6_ADDR_SCOPE_UNKNOWN         = 0x00,
    IPV6_ADDR_SCOPE_INTERFACE_LOCAL = 0x01,
    IPV6_ADDR_SCOPE_LINK_LOCAL      = 0x02,
    IPV6_ADDR_SCOPE_ADMIN_LOCAL     = 0x04,
    IPV6_ADDR_SCOPE_SITE_LOCAL      = 0x05,
    IPV6_ADDR_SCOPE_ORG_LOCAL       = 0x08,
    IPV6_ADDR_SCOPE_GLOBAL          = 0x0E,
}IPV6_ADDR_SCOPE;

typedef enum
{
    IPV6_ADDR_TYPE_UNKNOWN                  = 0,           // Invalid/unknown address type
    IPV6_ADDR_TYPE_UNICAST                  = 0x01,        // Only link-local and global are currently valid for unicast
    IPV6_ADDR_TYPE_ANYCAST                  = 0x02,
    IPV6_ADDR_TYPE_MULTICAST                = 0x03,
    IPV6_ADDR_TYPE_SOLICITED_NODE_MULTICAST = 0x04,
    IPV6_ADDR_TYPE_UNICAST_TENTATIVE        = 0x05,
}IPV6_ADDR_TYPE;



typedef struct __attribute__((__packed__)) _IPV6_ADDR_STRUCT
{
    struct _IPV6_ADDR_STRUCT * next;
    struct _IPV6_ADDR_STRUCT * prev;
    IPV6_ADDR address;
    unsigned long validLifetime;
    unsigned long preferredLifetime;
    unsigned long lastTickTime;
    unsigned char prefixLen;
    struct __attribute__((__packed__))
    {
        unsigned char precedence;                   // Allow preferences
        unsigned scope                  :4;         // Link-local, site-local, global.
        unsigned label                  :4;         // Policy label
        unsigned type                   :2;         // Uni-, Any-, Multi-cast
        unsigned temporary              :1;         // Indicates that the address is temporary (not public)
    }flags;
} IPV6_ADDR_STRUCT;

typedef const void*   IPV6_ADDR_HANDLE;


/*******************************************************************
 * List of the TCPIP stack supported modules
 *   The following enumeration lists all the modules supported
 *   by the TCPIP stack.
 *******************************************************************/

typedef enum
{
    TCPIP_MODULE_NONE              = 0, // unspecified/unknown module
    // 
    TCPIP_MODULE_IPV4,
    TCPIP_MODULE_ICMP,      
    TCPIP_MODULE_ARP,
	TCPIP_MODULE_IPV6,
	TCPIP_MODULE_ICMPV6,
	TCPIP_MODULE_NDP,
    TCPIP_MODULE_UDP,
    TCPIP_MODULE_TCP,
    TCPIP_MODULE_DHCP_CLIENT,
    TCPIP_MODULE_DHCP_SERVER,
    TCPIP_MODULE_ANNOUNCE,
    TCPIP_MODULE_DNS_CLIENT,
    TCPIP_MODULE_DNS_SERVER,
    TCPIP_MODULE_ZCLL,               // Zero Config Link Local
    TCPIP_MODULE_MDNS,              // mDNS
	TCPIP_MODULE_NBNS,
    TCPIP_MODULE_SMTP_CLIENT,
    TCPIP_MODULE_SNTP,
    TCPIP_MODULE_FTP_SERVER,
    TCPIP_MODULE_HTTP_SERVER,
    TCPIP_MODULE_TELNET_SERVER,
    TCPIP_MODULE_RSA,
    TCPIP_MODULE_SSL,
    TCPIP_MODULE_SNMP_SERVER,
	TCPIP_MODULE_SNMPV3_SERVER,
    TCPIP_MODULE_DYNDNS_CLIENT,
    TCPIP_MODULE_BERKELEY,
    TCPIP_MODULE_REBOOT_SERVER,
	TCPIP_MODULE_TCPIP_COMMAND,
	TCPIP_MODULE_TCPIP_IPERF,
	TCPIP_MODULE_TCPIP_MANAGER,     // stack manager
    // add other modules here
    //
    // list of supported MAC modules
    TCPIP_MODULE_MAC_NONE           = 0x1000,          // unspecified/unknown MAC

    // External ENC28J60 device: room for 16 ENCJ60 devices
    TCPIP_MODULE_MAC_ENCJ60         = 0x1010,
    TCPIP_MODULE_MAC_ENCJ60_0       = 0x1010,   // alternate numbered name 

    // External ENCX24J600 device: room for 16 ENCJ600 devices 
    TCPIP_MODULE_MAC_ENCJ600        = 0x1020,
    TCPIP_MODULE_MAC_ENCJ600_0      = 0x1030,   // alternate numbered name 

    // ETH97J60 device: room for 16 97J60 devices
    TCPIP_MODULE_MAC_97J60          = 0x1030,
    TCPIP_MODULE_MAC_97J60_0        = 0x1030,  // alternate numbered name  

     // internal/embedded PIC32 MAC: room for 16 PIC32 devices
    TCPIP_MODULE_MAC_PIC32INT       = 0x1040,       
    TCPIP_MODULE_MAC_PIC32INT_0     = 0x1040,    // alternate numbered name 

    // MRF24W WiFi MAC:  room for 16 MRF24W devices
    TCPIP_MODULE_MAC_MRF24W         = 0x1050,
    TCPIP_MODULE_MAC_MRF24W_0       = 0x1050,   // alternate numbered name

}TCPIP_STACK_MODULE;


/*******************************************************************
 * List of the TCPIP stack supported network interfaces
 *   The following enumeration lists the names of all the interfaces supported
 *   by the TCPIP stack.
 *******************************************************************/

#define TCPIP_STACK_IF_NAME_NONE           0
#define TCPIP_STACK_IF_NAME_ENCJ60         "ENCJ60"
#define TCPIP_STACK_IF_NAME_ENCJ600        "ENCJ600"
#define TCPIP_STACK_IF_NAME_97J60          "97J60"
#define TCPIP_STACK_IF_NAME_PIC32INT       "PIC32INT"
#define TCPIP_STACK_IF_NAME_MRF24W         "MRF24W"

/*******************************************************************
 * List of the supported TCPIP network configuration power modes
 *******************************************************************/

#define TCPIP_STACK_IF_POWER_NONE       0
#define TCPIP_STACK_IF_POWER_FULL       "full"  // up and running;
#define TCPIP_STACK_IF_POWER_LOW        "low"   // low power mode; not supported now
#define TCPIP_STACK_IF_POWER_DOWN       "down"  // powered down, not started

/*******************************************************************
 * List of the supported TCPIP network configuration start up flags
 *******************************************************************/
typedef enum
{
    TCPIP_NETWORK_CONFIG_DHCP_CLIENT_ON       = 0x0001,   // DHCP client enabled on this interface
    TCPIP_NETWORK_CONFIG_ZCLL_ON              = 0x0002,   // ZeroConf LinkLocal enabled on this interface
    TCPIP_NETWORK_CONFIG_DHCP_SERVER_ON       = 0x0004,   // DHCP server enabled on this interface
                                                          // Note: DHCPc, DHCPs and ZCLL are conflicting and
                                                          // cannot be more than one enabled on the same interface!
                                                          // The order of priorities is: DHCPc, ZCLL, DHCPs in case of conflict
                                                          //
    // add other configuration flags here
    // Note: valid values are 0x0001 - 0x8000; 16 bits only!
}TCPIP_NETWORK_CONFIG_FLAGS;


// =======================================================================
//   multi-homed hosts Network Addressing Configuration
// =======================================================================

typedef struct
{
     char*     interface;  // valid names - one of TCPIP_STACK_IF_NAME_xxx: "ENCJ60", "ENCJ600", "97J60", "PIC32INT", "MRF24W"; 
     char*     hostName;   // "MCHPBOARD"
     char*     macAddr;    // use "00:04:a3:00:00:00" or 0 for the factory pre-programmed address
     char*     ipAddr;     // "169.254.1.1"
     char*     ipMask;     // "255.255.0.0"
     char*     gateway;    // "169.254.1.1"
     char*     priDNS;     // "169.254.1.1"
     char*     secondDNS;  // "0.0.0.0"
     char*     powerMode;  // valid options: one of TCPIP_STACK_IF_POWER_xxx values 
     TCPIP_NETWORK_CONFIG_FLAGS   startFlags;     // flags for interface start up
}TCPIP_NETWORK_CONFIG;


// TCPIP module initialization/configuration structure
// Each stack module will be configured
// with a user defined initialization/configuration structure 
typedef struct
{
	TCPIP_STACK_MODULE		moduleId;
	const void * const		configData;
}TCPIP_STACK_MODULE_CONFIG;


/*******************************************************************
 * Configuration Rules Enforcement
 *   The following section enforces requirements for modules based 
 *   on configurations selected in tcpip_config.h
 *******************************************************************/

#include "tcpip/tcpip_mac.h"
#include "tcpip/tcpip_manager.h"


#include "tcpip/tcpip_helpers.h"
#include "tcpip/tcpip_events.h"
#include "tcpip/tcpip_mac_events.h"


#include "tcpip/ndp.h"
#include "tcpip/ipv4.h"
#include "tcpip/ipv6.h"
#include "tcpip/icmpv6.h"
#include "tcpip/arp.h"
#include "tcpip/rsa.h"
#include "tcpip/arcfour.h"
#include "tcpip/udp.h"
#include "tcpip/tcp.h"
#include "tcpip/berkeley_api.h"
#include "tcpip/dhcp.h"
#include "tcpip/dhcps.h"
#include "tcpip/zero_conf_link_local.h"
#include "tcpip/zero_conf_multicast_dns.h"
#include "tcpip/dns.h"
#include "tcpip/ftp.h"
#include "tcpip/icmp.h"
#include "tcpip/nbns.h"
#include "tcpip/dns.h"
#include "tcpip/dyn_dns.h"
#include "tcpip/telnet.h"
#include "tcpip/smtp.h"
#include "tcpip/tftpc.h"
#include "tcpip/sntp.h"
#include "tcpip/ssl.h"
#include "tcpip/tcpip_storage.h"
#include "tcpip/http2.h"
#include "tcpip/snmp.h"
#include "tcpip/snmpv3.h"
#include "mib.h"

#include "tcpip/wf_api.h"
#include "../../tcpip/wifi/wf_mac.h"


#include "system/fs/sys_fs.h"


#endif  // __TCPIP_H__
