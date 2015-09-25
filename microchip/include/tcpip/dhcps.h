
/*******************************************************************************
  DHCP server API for Microchip TCP/IP Stack

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  dhcps.h 
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


#ifndef __DHCPS_H
#define __DHCPS_H


typedef struct
{
    bool 	enabled;   // enable DHCP server
	bool 	deleteOldLease;  // delete old cache if still in place,
    // specific DHCP params
    size_t  leaseEntries;   // max number of lease entries 
    int     entrySolvedTmo; // solved entry removed after this tmo
                            // if not referenced - seconds
	char 	*startIpAddressRange; // start value of the IP address  for DHCP clients.
}DHCPS_MODULE_CONFIG;


bool DHCPServerIsEnabled(TCPIP_NET_HANDLE hNet);


bool DHCPServerDisable(TCPIP_NET_HANDLE hNet);
bool DHCPServerEnable(TCPIP_NET_HANDLE hNet);


// listing of the leases
//

typedef struct
{
    MAC_ADDR    hwAdd;
    IPV4_ADDR   ipAddress;
    uint32_t    leaseTime;
}DHCPS_LEASE_ENTRY;

typedef enum
{
	DHCP_SERVER_POOL_ENTRY_ALL,
	DHCP_SERVER_POOL_ENTRY_IN_USE
}DHCP_SERVER_POOL_ENTRY_TYPE;



typedef const void* DHCPS_LEASE_HANDLE;


// returns a lease entry and allows iteration through the whole list of leases
// if leaseHandle it starts from the begining
// returns a non-zero DHCPS_LEASE_HANDLE to be used in the subsequent calls or 0 if end of list or wrong interface, or DHCP server not running on that interface
DHCPS_LEASE_HANDLE  DHCPServerLeaseEntryGet(TCPIP_NET_HANDLE netH, DHCPS_LEASE_ENTRY* pLeaseEntry, DHCPS_LEASE_HANDLE leaseHandle);
bool DCHPServerRemovePoolEntries(TCPIP_NET_HANDLE netH, DHCP_SERVER_POOL_ENTRY_TYPE type);
int DCHPServerGetPoolEntries(TCPIP_NET_HANDLE netH, DHCP_SERVER_POOL_ENTRY_TYPE type);
#endif // __DHCPS_H

