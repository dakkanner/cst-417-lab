/*******************************************************************************
  tcpip Configuration File

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:   tcpip_config_private.h
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

#ifndef __TCPIP_CONFIG_PRIVATE_H_
#define __TCPIP_CONFIG_PRIVATE_H_


#include "tcpip_config.h"
#include "tcpip_announce_config.h"
#include "arp_config.h"
#include "berekely_api_config.h"
#include "dhcp_config.h"
#include "dhcps_config.h"
#include "dns_config.h"
#include "dnss_config.h"
#include "dyndns_config.h"
#include "ftp_config.h"
#include "http2_config.h"
#include "icmp_config.h"
#include "icmpv6_config.h"
#include "ip_config.h"
#include "tcpip_mac_config.h"
#include "nbns_config.h"
#include "ndp_config.h"
#include "tcpip_reboot_config.h"
#include "smtp_config.h"
#include "snmp_config.h"
#include "snmpv3_config.h"
#include "sntp_config.h"
#include "ssl_config.h"
#include "tcp_config.h"
#include "tcpip_storage_config.h"
#include "telnet_config.h"
#include "tftpc_config.h"
#include "udp_config.h"
#include "tcpip_cmd_config.h"
#include "wf_config.h"
#include "iperf_config.h"

#include "network_config.h"
#include "hash_tbl.h"
// Checks for configuration errors //

#if !defined(TCPIP_STACK_USE_IPV4) && !defined(TCPIP_STACK_USE_IPV6)
    #error "The TCPIP configuration requires that either IPv4 or IPv6 should be selected!"
    #error "Define TCPIP_STACK_USE_IPV4 or TCPIP_STACK_USE_IPV6 and rebuild."
#endif


#if !defined(TCPIP_STACK_DRAM_SIZE)
    #error "This stack uses dynamic memory allocation."
    #error "Define TCPIP_STACK_DRAM_SIZE in the tcpip_config.h"
    #error "allocate enough heap space for your project."
#elif defined (__PIC32MX__)
    #if (TCPIP_STACK_DRAM_SIZE < 8*1024)
        #warning "You need at least 8KB of heap space for the TCPIP stack to run properly."
    #endif
#elif defined(__C30__)
    #if (TCPIP_STACK_DRAM_SIZE < 4*1024)
        #warning "You need at least 4KB of heap space for the TCPIP stack to run properly."
    #endif
#endif

// IPv4 check
#if defined(TCPIP_STACK_USE_ICMP_SERVER) || \
	defined(TCPIP_STACK_USE_ICMP_CLIENT) || \
	defined(TCPIP_STACK_USE_DHCP_CLIENT) || \
	defined(TCPIP_STACK_USE_DNS) || \
	defined(TCPIP_STACK_USE_NBNS) || \
	defined(TCPIP_STACK_USE_DHCP_SERVER) || \
	defined(TCPIP_STACK_USE_DNS_SERVER) || \
	defined(TCPIP_STACK_USE_ZEROCONF_MDNS_SD)
    #if !defined(TCPIP_STACK_USE_IPV4)
        #error "The TCPIP configuration requires that TCPIP_STACK_USE_IPV4 should be defined."
        #error "Define TCPIP_STACK_USE_IPV4 and rebuild."
    #endif
#endif


////
// HTTP Check
////
// Include modules required by specific HTTP demos
#if defined(HTTP_FILE_UPLOAD) && !defined(HTTP_USE_POST)
    #error HTTP_FILE_UPLOAD requires HTTP_USE_POST (and external storage) to be defined 
#endif

#if (!defined(HTTP_MAX_CONNECTIONS) || HTTP_MAX_CONNECTIONS <= 0)
    #error Invalid HTTP_MAX_CONNECTIONS value specified.
#endif

////
// DNS Check
////
// Make sure that the DNS client is enabled if services require it
#if defined(TCPIP_STACK_USE_SNTP_CLIENT) || \
	defined(TCPIP_STACK_USE_DYNAMICDNS_CLIENT) || \
	defined(TCPIP_STACK_USE_SMTP_CLIENT)
    #if !defined(TCPIP_STACK_USE_DNS)
        #error "The TCPIP configuration requires that TCPIP_STACK_USE_DNS should be defined."
        #error "Define TCPIP_STACK_USE_DNS and rebuild."
    #endif
#endif

////
// RSA Check
////
// When using SSL server, enable RSA decryption
#if defined(TCPIP_STACK_USE_SSL_SERVER)
	#if !defined TCPIP_STACK_USE_RSA_DECRYPT
    	#define TCPIP_STACK_USE_RSA_DECRYPT
    #endif
	#if !defined (TCPIP_STACK_USE_SSL)
    	#define TCPIP_STACK_USE_SSL
    #endif
#endif

// When using SSL client, enable RSA encryption
#if defined(TCPIP_STACK_USE_SSL_CLIENT)
	#if !defined (TCPIP_STACK_USE_RSA_ENCRYPT)
    	#define TCPIP_STACK_USE_RSA_ENCRYPT
    #endif
	#if !defined (TCPIP_STACK_USE_SSL)
    	#define TCPIP_STACK_USE_SSL
    #endif
#endif

// When using either RSA operation, include the RSA module
#if defined(TCPIP_STACK_USE_RSA_ENCRYPT) || defined(TCPIP_STACK_USE_RSA_DECRYPT)
	#if !defined (TCPIP_STACK_USE_RSA)
    	#define TCPIP_STACK_USE_RSA
    #endif
	#if !defined (TCPIP_STACK_USE_BIGINT)
    	#define TCPIP_STACK_USE_BIGINT
    #endif
#endif


#if defined(TCPIP_STACK_USE_RSA_ENCRYPT)
	#define BI_USE_CONSTRUCTOR
	#define BI_USE_ZERO
	#define BI_USE_MOD
	#define BI_USE_COMPARE
	#define BI_USE_MAG_DIFF
	#define BI_USE_MAG
	#define BI_USE_MSB
	#define BI_USE_MULTIPLY
	#define BI_USE_SQUARE
	#define BI_USE_COPY
#endif

#if defined(TCPIP_STACK_USE_RSA_DECRYPT)
	#define BI_USE_CONSTRUCTOR
	#define BI_USE_ZERO
	#define BI_USE_COMPARE
	#define BI_USE_MAG
	#define BI_USE_MSB
	#define BI_USE_MULTIPLY
	#define BI_USE_SQUARE
	#define BI_USE_ADD
	#define BI_USE_SUBTRACT
	#define BI_USE_COPY

    #define BI_USE_MAG_DIFF
    #define BI_USE_MOD
#endif


////
// SSL Check
//// 
#if defined(TCPIP_STACK_USE_SSL_SERVER) || defined(TCPIP_STACK_USE_SSL_CLIENT)
    #if (!defined(SSL_MAX_CONNECTIONS) || SSL_MAX_CONNECTIONS <= 0 )
        #error Invalid SSL_MAX_CONNECTIONS value specified.
    #endif
    #if (!defined(SSL_MAX_SESSIONS) || SSL_MAX_SESSIONS <= 0)
        #error Invalid SSL_MAX_SESSIONS value specified.
    #endif
    #if (!defined(SSL_MAX_HASHES) || SSL_MAX_HASHES <= 0 )
        #error Invalid SSL_MAX_HASHES value specified.
    #elif (SSL_MAX_HASHES > 16)
        #undef SSL_MAX_HASHES
        #define SSL_MAX_HASHES 16
    #endif
#else
    #undef SSL_MAX_CONNECTIONS
    #undef SSL_MAX_SESSIONS
    #define SSL_MAX_CONNECTIONS  0
    #define SSL_MAX_SESSIONS  0
#endif // TCPIP_STACK_USE_SSL_SERVER || TCPIP_STACK_USE_SSL_CLIENT

// If using SSL (either), include the rest of the support modules
#if defined(TCPIP_STACK_USE_SSL)
	#if !defined (TCPIP_STACK_USE_ARCFOUR)
    	#define TCPIP_STACK_USE_ARCFOUR
    #endif
	#if !defined (TCPIP_STACK_USE_MD5)
    	#define TCPIP_STACK_USE_MD5
    #endif
	#if !defined (TCPIP_STACK_USE_SHA1)
    	#define TCPIP_STACK_USE_SHA1
    #endif
	#if !defined (TCPIP_STACK_USE_RANDOM)
    	#define TCPIP_STACK_USE_RANDOM
    #endif
#endif


////
// FTP Check
////
#if defined(TCPIP_STACK_USE_FTP_SERVER)
    #if (!defined(FTP_MAX_CONNECTIONS) || FTP_MAX_CONNECTIONS <= 0)
        #error Invalid FTP_MAX_CONNECTIONS value specified.
    #endif
#endif // TCPIP_STACK_USE_FTP_SERVER

////
// SMTP Check
////
#if defined(TCPIP_STACK_USE_SMTP_CLIENT)
    #if (!defined(MAX_SMTP_CONNECTIONS) || MAX_SMTP_CONNECTIONS <= 0)
        #error Invalid MAX_SMTP_CONNECTIONS value specified.
    #endif
#else
    #undef  MAX_SMTP_CONNECTIONS
    #define MAX_SMTP_CONNECTIONS  0
#endif // TCPIP_STACK_USE_SMTP_CLIENT

////
// TCP Check
////

// Make sure that TCPIP_STACK_USE_TCP is defined if a service 
// depends on it
#if defined(TCPIP_STACK_USE_HTTP2_SERVER) || \
	defined(TCPIP_STACK_USE_FTP_SERVER) || \
	defined(TCPIP_STACK_USE_TELNET_SERVER) || \
	defined(TCPIP_STACK_USE_SMTP_CLIENT) || \
	defined(TCPIP_STACK_USE_DYNAMICDNS_CLIENT) || \
	defined(TCPIP_STACK_USE_BERKELEY_API) || \
	defined(TCPIP_STACK_USE_SSL_CLIENT) || \
	defined(TCPIP_STACK_USE_SSL_SERVER)
    #if !defined(TCPIP_STACK_USE_TCP)
        #error "The TCPIP configuration requires that TCPIP_STACK_USE_TCP should be defined."
        #error "Define TCPIP_STACK_USE_TCP and rebuild."
    #endif

    #if (!defined(TCP_MAX_SOCKETS) || TCP_MAX_SOCKETS <= 0)
        #error Invalid TCP_MAX_SOCKETS value specified.
    #endif  // !defined(TCP_MAX_SOCKETS)

    #if (!defined(TCP_MAX_SEG_SIZE_RX_LOCAL) || TCP_MAX_SEG_SIZE_RX_LOCAL <= 0)
        #error Invalid TCP_MAX_SEG_SIZE_RX_LOCAL value specified.
    #endif  // !defined(TCP_MAX_SEG_SIZE_RX_LOCAL)

    #if (!defined(TCP_MAX_SEG_SIZE_RX_NON_LOCAL) || TCP_MAX_SEG_SIZE_RX_NON_LOCAL <= 0)
        #error Invalid TCP_MAX_SEG_SIZE_RX_NON_LOCAL value specified.
    #endif  // !defined(TCP_MAX_SEG_SIZE_RX_NON_LOCAL)

#endif



////
// TELNET Check
////
#if defined(TCPIP_STACK_USE_TELNET_SERVER)
    #if (!defined(MAX_TELNET_CONNECTIONS) || MAX_TELNET_CONNECTIONS <= 0)
        #error Invalid MAX_TELNET_CONNECTIONS value specified.
    #elif !defined(SYS_COMMAND_ENABLE)
        #error telnet service needs the SYS_COMMAND_ENABLE to be defined!
        #error Define system_config.h::SYS_COMMAND_ENABLE and rebuild!
    #endif
#else
    #undef  MAX_TELNET_CONNECTIONS
    #define MAX_TELNET_CONNECTIONS  0
#endif // TCPIP_STACK_USE_TELNET_SERVER


////
// tcpip console Check
////
#if defined(TCPIP_STACK_COMMAND_ENABLE)
    #if !defined(SYS_COMMAND_ENABLE)
        #error tcpip console service needs the SYS_COMMAND_ENABLE to be defined!
        #error Define system_config.h::SYS_COMMAND_ENABLE and rebuild!
    #endif
#endif // TCPIP_STACK_COMMAND_ENABLE

////
// tcpip iperf Check
////
#if defined(TCPIP_STACK_USE_IPERF)
    #if !defined(TCPIP_STACK_COMMAND_ENABLE)
        #error tcpip iperf service needs the TCPIP_STACK_COMMAND_ENABLE to be defined!
        #error Define tcpip_config.h::TCPIP_STACK_COMMAND_ENABLE and rebuild!
    #endif
#endif // TCPIP_STACK_USE_IPERF


////
// Berkely API Check
////
#if defined(TCPIP_STACK_USE_BERKELEY_API)
    #if (!defined(MAX_BSD_SERVER_CONNECTIONS) || MAX_BSD_SERVER_CONNECTIONS <= 0)
        #error Invalid MAX_BSD_SERVER_CONNECTIONS value specified.
    #endif
    #if (!defined(MAX_BSD_CLIENT_CONNECTIONS) || MAX_BSD_CLIENT_CONNECTIONS <= 0)
        #error Invalid MAX_BSD_CLIENT_CONNECTIONS value specified.
    #endif
#else
    #undef  MAX_BSD_SERVER_CONNECTIONS
    #undef  MAX_BSD_CLIENT_CONNECTIONS
    #define MAX_BSD_SERVER_CONNECTIONS  0
    #define MAX_BSD_CLIENT_CONNECTIONS  0
#endif // TCPIP_STACK_USE_BERKELEY_API


////
// UDP Check
////

// Make sure that TCPIP_STACK_USE_UDP is defined if a service 
// depends on it
#if defined(TCPIP_STACK_USE_DHCP_CLIENT) || \
	defined(TCPIP_STACK_USE_DHCP_SERVER) || \
	defined(TCPIP_STACK_USE_DNS) || \
	defined(TCPIP_STACK_USE_NBNS) || \
	defined(TCPIP_STACK_USE_SNMP_SERVER) || \
	defined(TCPIP_STACK_USE_SNMPV3_SERVER) || \
	defined(TCPIP_STACK_USE_TFTP_CLIENT) || \
	defined(TCPIP_STACK_USE_ANNOUNCE) || \
	defined(TCPIP_STACK_USE_SNTP_CLIENT) || \
	defined(TCPIP_STACK_USE_BERKELEY_API)
    #if !defined(TCPIP_STACK_USE_UDP)
        #error "The TCPIP configuration requires that TCPIP_STACK_USE_UDP should be defined."
        #error "Define TCPIP_STACK_USE_UDP and rebuild."
    #endif
#endif


// SNMP Check
#if defined(TCPIP_STACK_USE_SNMPV3_SERVER)
    #if !defined (TCPIP_STACK_USE_SNMP_SERVER)
        #error "The TCPIP configuration requires that TCPIP_STACK_USE_SNMP_SERVER should be defined."
        #error "Define TCPIP_STACK_USE_SNMP_SERVER and rebuild."
    #endif
#endif

// Storage Check
#if defined(TCPIP_STACK_USE_STORAGE)
    #if (TCPIP_STORAGE_STACK_CHECK_BUILD_VERSION)
        #define _TCPIP_STACK_CHECK_STORAGE_VERSION
        #define _TCPIP_STORAGE_USER_LABEL_SIZE  2
    #else 
        #define _TCPIP_STORAGE_USER_LABEL_SIZE  TCPIP_STORAGE_USER_LABEL_SIZE   // use default
    #endif
#endif  // defined(TCPIP_STACK_USE_STORAGE)


// ZeroConf check
//
#if defined (TCPIP_STACK_USE_ZEROCONF_MDNS_SD)
    #if !defined(TCPIP_STACK_USE_ZEROCONF_LINK_LOCAL)
        #error "The TCPIP configuration requires that TCPIP_STACK_USE_ZEROCONF_LINK_LOCAL should be defined."
        #error "Define TCPIP_STACK_USE_ZEROCONF_LINK_LOCAL and rebuild."
    #endif  // !defined(TCPIP_STACK_USE_ZEROCONF_LINK_LOCAL)
#endif  // defined (TCPIP_STACK_USE_ZEROCONF_MDNS_SD)

// DHCP server check
#if defined (TCPIP_STACK_USE_DHCP_SERVER)
    #if !defined(OA_HASH_DYNAMIC_KEY_MANIPULATION) 
        #error "TCPIP/Stack internal build error. The stack settings are incompatible with the DHCP server build."
    #endif  // !defined(OA_HASH_DYNAMIC_KEY_MANIPULATION)
#endif  // defined (TCPIP_STACK_USE_DHCP_SERVER)

/*****************************************************************************
The stack state machine ticks rate, miliseconds.
Used by the stack state machine to check for the MAC link, etc
Note: the System Tick resolution in system_config.h (SYS_TICKS_PER_SECOND) has to 
be fine enough to allow for this stack tick granularity.  
 *****************************************************************************/
#if !defined(TCPIP_STACK_TICK_RATE)
    #define TCPIP_STACK_TICK_RATE        (100)        // 100 ms default tick 
#endif


#endif  //  __TCPIP_CONFIG_PRIVATE_H_

