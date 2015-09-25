/*******************************************************************************
Hardware specific definitions

  Summary:
    
  Description:

*******************************************************************************/

/*******************************************************************************
FileName: 	tcpip_modules_config.h
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

#ifndef __TCPIP_MODULES_CONFIG_H_
#define __TCPIP_MODULES_CONFIG_H_

 // =======================================================================
//   Include all the modules configuration options
// =======================================================================
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "tcpip_config.h"

// =======================================================================
//   Enable the modules configuration symbol
// =======================================================================

#if defined(TCPIP_STACK_USE_ANNOUNCE)
#include "tcpip_announce_config.h"
#endif

#include "arp_config.h"

#if defined(TCPIP_STACK_USE_BERKELEY_API)
#include "berekely_api_config.h"
#endif

#if defined(TCPIP_STACK_USE_DHCP_SERVER)
#include "dhcps_config.h"
#endif

#if defined(TCPIP_STACK_USE_DHCP_CLIENT)
#include "dhcp_config.h"
#endif

#if defined(TCPIP_STACK_USE_DNS_SERVER)
#include "dnss_config.h"
#endif

#if defined(TCPIP_STACK_USE_DNS)
#include "dns_config.h"
#endif

#if defined(TCPIP_STACK_USE_DYNAMICDNS_CLIENT)
#include "dyndns_config.h"
#endif

#if defined(TCPIP_STACK_USE_FTP_SERVER)
#include "ftp_config.h"
#endif

#if defined(TCPIP_STACK_USE_HTTP2_SERVER)
#include "http2_config.h"
#endif

#if defined(TCPIP_STACK_USE_IPV6)
#include "icmpv6_config.h"
#include "ndp_config.h"
#endif


#if defined(TCPIP_STACK_USE_ICMP_SERVER) || defined(TCPIP_STACK_USE_ICMP_CLIENT)
#include "icmp_config.h"
#endif

#include "iperf_config.h"

#include "ip_config.h"

#include "tcpip_mac_config.h"

#if defined(TCPIP_STACK_USE_NBNS)
#include "nbns_config.h"
#endif

#if defined(TCPIP_STACK_USE_REBOOT_SERVER)
#include "tcpip_reboot_config.h"
#endif

#if defined(TCPIP_STACK_USE_SMTP_CLIENT)
#include "smtp_config.h"
#endif

#if defined(TCPIP_STACK_USE_SNMPV3_SERVER)
#include "snmpv3_config.h"
#endif

#if defined(TCPIP_STACK_USE_SNMP_SERVER)
#include "snmp_config.h"
#endif

#if defined(TCPIP_STACK_USE_SNTP_CLIENT)
#include "sntp_config.h"
#endif

#if defined(TCPIP_STACK_USE_SSL_SERVER) || defined(TCPIP_STACK_USE_SSL_CLIENT)
#include "ssl_config.h"
#endif

#if defined(TCPIP_STACK_USE_TCP)
#include "tcp_config.h"
#endif

#if defined(TCPIP_STACK_USE_STORAGE)
#include "tcpip_storage_config.h"
#endif

#if defined(TCPIP_STACK_USE_TELNET_SERVER)
#include "telnet_config.h"
#endif

#if defined(TCPIP_STACK_USE_TFTP_CLIENT)
#include "tftpc_config.h"
#endif


#if defined(TCPIP_STACK_USE_UDP)
#include "udp_config.h"
#endif

#include "wf_config.h"
#include "network_config.h"

#include "tcpip/tcpip.h"

 
#ifdef TCPIP_STACK_MODULE_CONFIGURATION
const TCPIP_STACK_MODULE_CONFIG TCPIP_STACK_MODULE_CONFIG_TBL [] =
{
	 //ModuleID				     // configData
#if defined(TCPIP_STACK_USE_IPV4)
	{TCPIP_MODULE_IPV4,		     0},
#endif
#if defined(TCPIP_STACK_USE_ICMP_CLIENT) || defined(TCPIP_STACK_USE_ICMP_SERVER)		
    {TCPIP_MODULE_ICMP,          0},                           // TCPIP_MODULE_ICMP
#endif
    {TCPIP_MODULE_ARP,           &arpConfigData},              // TCPIP_MODULE_ARP
#if defined(TCPIP_STACK_USE_IPV6)
	{TCPIP_MODULE_IPV6,		     0},                           // TCPIP_MODULE_IPV6
    {TCPIP_MODULE_ICMPV6,	     0},                           // TCPIP_MODULE_ICMPV6
    {TCPIP_MODULE_NDP,           0},                           // TCPIP_MODULE_NDP
#endif
#if defined(TCPIP_STACK_USE_UDP)
    {TCPIP_MODULE_UDP,           &udpConfigData},              // TCPIP_MODULE_UDP,
#endif
#if defined(TCPIP_STACK_USE_TCP)
    {TCPIP_MODULE_TCP,           &tcpConfigData},              // TCPIP_MODULE_TCP,
#endif    
#if defined(TCPIP_STACK_USE_DHCP_CLIENT)
    {TCPIP_MODULE_DHCP_CLIENT,   &dhcpConfigData},             // TCPIP_MODULE_DHCP_CLIENT,
#endif
#if defined(TCPIP_STACK_USE_DHCP_SERVER)
    {TCPIP_MODULE_DHCP_SERVER,   0},                           // TCPIP_MODULE_DHCP_SERVER,
#if defined(TCPIP_STACK_USE_AUTOIP)
	{TCPIP_MODULE_AUTOIP,		 0},                           // TCPIP_MODULE_AUTOIP
#endif
#endif
#if defined(TCPIP_STACK_USE_ANNOUNCE)
    {TCPIP_MODULE_ANNOUNCE,      0},                           // TCPIP_MODULE_ANNOUNCE,
#endif
#if defined(TCPIP_STACK_USE_DNS)
    {TCPIP_MODULE_DNS_CLIENT,    0},                           // TCPIP_MODULE_DNS_CLIENT,
#endif // TCPIP_STACK_USE_DNS
#if defined(TCPIP_STACK_USE_NBNS)
	{TCPIP_MODULE_NBNS,			 0},                           // TCPIP_MODULE_NBNS
#endif
#if defined(TCPIP_STACK_USE_SNTP_CLIENT)
    {TCPIP_MODULE_SNTP,         0},                            // TCPIP_MODULE_SNTP,
#endif
#if defined(TCPIP_STACK_USE_BERKELEY_API)
    {TCPIP_MODULE_BERKELEY,      0},                           // TCPIP_MODULE_BERKELEY,
#endif
#if defined(TCPIP_STACK_USE_HTTP2_SERVER)
    {TCPIP_MODULE_HTTP_SERVER,   &httpConfigData},              // TCPIP_MODULE_HTTP_SERVER,
#endif
#if defined(TCPIP_STACK_USE_TELNET_SERVER)
    {TCPIP_MODULE_TELNET_SERVER,   0},                        // TCPIP_MODULE_TELNET_SERVER,
#endif
#if defined(TCPIP_STACK_USE_SSL_SERVER) || defined(TCPIP_STACK_USE_SSL_CLIENT)
    {TCPIP_MODULE_RSA,           0},                           // TCPIP_MODULE_RSA,
#endif
#if defined(TCPIP_STACK_USE_SSL_SERVER) || defined(TCPIP_STACK_USE_SSL_CLIENT)
    {TCPIP_MODULE_SSL,           0},                           // TCPIP_MODULE_SSL,
#endif
#if defined(TCPIP_STACK_USE_FTP_SERVER)
    {TCPIP_MODULE_FTP_SERVER,    0},                           // TCPIP_MODULE_FTP,
#endif
#if defined(TCPIP_STACK_USE_SNMP_SERVER)
    {TCPIP_MODULE_SNMP_SERVER,   0},                           // TCPIP_MODULE_SNMP_SERVER,
#endif
#if defined(TCPIP_STACK_USE_DYNAMICDNS_CLIENT)
    {TCPIP_MODULE_DYNDNS_CLIENT, 0},                           // TCPIP_MODULE_DYNDNS_CLIENT,
#endif
#if defined(TCPIP_STACK_USE_REBOOT_SERVER)
    {TCPIP_MODULE_REBOOT_SERVER, 0},                           // TCPIP_MODULE_REBOOT_SERVER,
#endif
#if defined(TCPIP_STACK_USE_ZEROCONF_LINK_LOCAL)
    {TCPIP_MODULE_ZCLL, 0},                                    // TCPIP_MODULE_ZCLL,
    {TCPIP_MODULE_MDNS, 0},                                    // TCPIP_MODULE_MDNS,
#endif
#if defined(TCPIP_STACK_COMMAND_ENABLE)
	{TCPIP_MODULE_TCPIP_COMMAND, 0},						   // TCPIP_MODULE_TCPIP_COMMAND
#endif


    // MAC modules
#if defined(TCPIP_IF_ENC28J60)
    {TCPIP_MODULE_MAC_ENCJ60, &macENCJ60ConfigData},         // TCPIP_MODULE_MAC_ENCJ60
#endif
#if defined(TCPIP_IF_ENCX24J600)
    {TCPIP_MODULE_MAC_ENCJ600, &macENCJ600ConfigData},       // TCPIP_MODULE_MAC_ENCJ600
#endif
#if defined(TCPIP_IF_97J60)
    {TCPIP_MODULE_MAC_97J60, &mac97J60ConfigData},           // TCPIP_MODULE_MAC_97J60
#endif
#if defined(TCPIP_IF_PIC32INT)
    {TCPIP_MODULE_MAC_PIC32INT, &macPIC32INTConfigData},     // TCPIP_MODULE_MAC_PIC32INT
#endif
#if defined(TCPIP_IF_MRF24W)
    {TCPIP_MODULE_MAC_MRF24W, &macMRF24WConfigData},         // TCPIP_MODULE_MAC_MRF24W
#endif
};
#else
extern const TCPIP_STACK_MODULE_CONFIG TCPIP_STACK_MODULE_CONFIG_TBL [];
#endif  // TCPIP_STACK_MODULE_CONFIGURATION

#endif
