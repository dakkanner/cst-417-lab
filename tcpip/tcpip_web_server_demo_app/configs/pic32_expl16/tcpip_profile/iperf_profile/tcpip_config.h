/*******************************************************************************
Microchip TCP/IP Stack Demo Application Configuration Header

  Summary:
    
  Description:

*******************************************************************************/

/*******************************************************************************
FileName: 	tcpip_config.h
Copyright ?2012 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS?WITHOUT WARRANTY OF ANY KIND,
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

#ifndef __TCPIP_CONFIG_H_
#define __TCPIP_CONFIG_H_

// =======================================================================
//   Stack Configuration/Build Options
// =======================================================================

// TCPIP Stack Module Selection
//   Uncomment or comment the following lines to enable or
//   disabled the following high-level application modules.

#define TCPIP_STACK_USE_IPV4                    // enable IPv4 functionality
#define TCPIP_STACK_USE_ICMP_SERVER			// Ping query and response capability
//#define TCPIP_STACK_USE_HTTP2_SERVER			// New HTTP server with POST, Cookies, Authentication, etc.
//#define TCPIP_STACK_USE_SSL_SERVER			// SSL server socket support 
//#define TCPIP_STACK_USE_SSL_CLIENT			// SSL client socket support 
#define TCPIP_STACK_USE_DHCP_CLIENT			// Dynamic Host Configuration Protocol client for obtaining IP address and other parameters
//#define TCPIP_STACK_USE_SMTP_CLIENT			// Simple Mail Transfer Protocol for sending email
#define TCPIP_STACK_USE_TELNET_SERVER			// Telnet server
//#define TCPIP_STACK_USE_ANNOUNCE				// Microchip Embedded Ethernet Device Discoverer server/client
#define TCPIP_STACK_USE_DNS					// Domain Name Service Client for resolving hostname strings to IP addresses
#define TCPIP_STACK_USE_NBNS					// NetBIOS Name Service Server for repsonding to NBNS hostname broadcast queries
//#define TCPIP_STACK_USE_REBOOT_SERVER			// Module for resetting this PIC remotely.  Primarily useful for a Bootloader.
//#define TCPIP_STACK_USE_SNTP_CLIENT			// Simple Network Time Protocol for obtaining current date/time from Internet
//#define TCPIP_STACK_USE_DYNAMICDNS_CLIENT		// Dynamic DNS client updater module
//#define TCPIP_STACK_USE_BERKELEY_API			// Berekely Sockets APIs are available
#define TCPIP_STACK_USE_IPV6                    // enable IPv6 functionality
#define TCPIP_STACK_USE_TCP                     // Enable the TCP module
#define TCPIP_STACK_USE_UDP                     // Enable the UDP module
//#define TCPIP_STACK_USE_FTP_SERVER			// File Transfer Protocol
//#define TCPIP_STACK_USE_SNMP_SERVER			// Simple Network Management Protocol v2C Community Agent
//#define TCPIP_STACK_USE_SNMPV3_SERVER         // SNMP v3 agent
#define TCPIP_STACK_COMMAND_ENABLE              // TCPIP_COMMANDS for network configuration or debug
#define TCPIP_STACK_USE_IPERF                   // Enable the iperf module for standard network benchmarking
//#define TCPIP_STACK_USE_ZEROCONF_LINK_LOCAL		// Zeroconf IPv4 Link-Local Addressing
//#define TCPIP_STACK_USE_ZEROCONF_MDNS_SD		// Zeroconf mDNS and mDNS service discovery
//#define TCPIP_STACK_USE_ICMP_CLIENT			// Ping transmission capability
//#define TCPIP_STACK_USE_DHCP_SERVER			// Single host DHCP server

// =======================================================================
//   NOT PORTED/TESTED. DO NOT ENABLE!
// =======================================================================

//#define TCPIP_STACK_USE_TFTP_CLIENT			// Trivial File Transfer Protocol client
//#define TCPIP_STACK_USE_DNS_SERVER			// Domain Name Service Server for redirection to the local device


// =======================================================================
//   OBSOLETE SERVICES. NOT RECOMMENDED FOR NEW DESIGNS. DO NOT ENABLE!
// =======================================================================

//#define TCPIP_STACK_USE_STORAGE			    // Enable the TCPIP storage service
                                                // needs hardware support: EEPROM/SPIFlash
                                                // NOTE: This service will be replaced with a system level database for
                                                // storing module settings and parameters.

// =======================================================================
//   Dynamic memory allocation Options
// =======================================================================
// This is the total amount of dynamic memory that the TCPIP stack will use:
// for MAC buffers, for TCP and UDP sockets, etc.
#define TCPIP_STACK_DRAM_SIZE       			(12288) 

// The minimum amount of dynamic memory left for run time allocation by the stack (IP, UDP, etc)
// This is just a warning threshold.
// If after all the modules are initialized the amount of memory available in the TCPIP heap
// is less then TCPIP_STACK_DRAM_RUN_LIMIT then a warning will be displayed
// (if the debug channel is enabled)
#define TCPIP_STACK_DRAM_RUN_LIMIT       	(2048)

// enable debugging of the an allocation call that failed.
// If the system debug is enabled (SYS_DEBUG_ENABLE) the stack will issue a
// warning message at the system debug channel
#define TCPIP_STACK_DRAM_DEBUG_ENABLE

    // enable tracing of the allocated memory by each module
    // the stack will trace all the memory allocated by a module
    // and various statistics
    //#define TCPIP_STACK_DRAM_TRACE_ENABLE
    // number of trace slots
    // there is on slot needed per module that allocates memory from the heap
    #define TCPIP_STACK_DRAM_TRACE_SLOTS    12

// =======================================================================
//   Event Notifications Options
//   ENC624J600, ENC24J60 and MRF24W do not support (yet) event notifications
// =======================================================================
//#define TCPIP_STACK_USE_EVENT_NOTIFICATION

    // The default interrupt priority to use for the TCPIP interrupts
	#define	TCPIP_EVENT_IPL			5
	#define	TCPIP_EVENT_SIPL		1


#endif  // __TCPIP_CONFIG_H_


