/*******************************************************************************
  Dynamic Host Configuration Protocol (DHCPS) Configuration file

  Summary:
    DHCPS configuration file

  Description:
    This file contains the DHCPS module configuration options

*******************************************************************************/

/*******************************************************************************
FileName:   dhcps_config.h
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

#ifndef _DHCPS_CONFIG_H_
#define _DHCPS_CONFIG_H_

// DHCPS task processing rate, in seconds
// the DHCPS module will process a timer event with this rate
// for maintaining its own queues, processing timeouts, etc.
//
#define DHCPS_TASK_PROCESS_RATE              (1)



// number of entries in the cache
// default number of entries per interface
#define DHCPS_LEASE_ENTRIES_DEFAULT       5
// timeout for a solved entry in the cache, in seconds
// the entry will be removed if the tmo elapsed
// and the entry has not been referenced again
#define DHCPS_LEASE_SOLVED_ENTRY_TMO      (20 * 60)

// The entry should be removed from the entry if there is no REQUEST after OFFER
#define DHCPS_LEASE_REMOVED_BEFORE_ACK		(60)
#define DHCPS_LEASE_DURATION				DHCPS_LEASE_SOLVED_ENTRY_TMO

//Start of IP address Range , network_config.h ipaddress and this start of IP address should be in same SUBNET
// RECOMENDED - network_config.h ipaddress should be 192.168.100.1 if DHCP server ip address range starts
// from 192.168.100.100.


#define DHCPS_IP_ADDRESS_RANGE_START		"192.168.100.100" // Address range is starting from 100, because the from 1 to 100 is reserved. Staart address(192.168.100.1)
															// should be used for server address
#define DHCP_SERVER_IP_ADDRESS				"192.168.100.1"     // First three octet should match to the TP address range for Server address
																//and the begining address should be reserved for server address
#define DHCP_SERVER_NETMASK_ADDRESS			"255.255.255.0"    // Net mask value

#define DHCP_SERVER_GATEWAY_ADDRESS			"192.168.100.1"  // as we are in same network, Gateway address is same as server IP address.

#if defined(TCPIP_STACK_USE_DNS)
#define DHCP_SERVER_PRIMARY_DNS_ADDRESS		"192.168.100.1" // DNS Primary address
#define DHCP_SERVER_SECONDARY_DNS_ADDRESS	"192.168.100.1" // DNS Secondary address
#endif


#ifdef TCPIP_STACK_MODULE_CONFIGURATION
const DHCPS_MODULE_CONFIG dhcpsConfigData =
{
	true,
	true,
	DHCPS_LEASE_ENTRIES_DEFAULT,
    DHCPS_LEASE_SOLVED_ENTRY_TMO,
    DHCPS_IP_ADDRESS_RANGE_START,
};
#else
extern const DHCPS_MODULE_CONFIG dhcpsConfigData;

#endif  // TCPIP_STACK_MODULE_CONFIGURATION




#endif  // _DHCPS_CONFIG_H_



