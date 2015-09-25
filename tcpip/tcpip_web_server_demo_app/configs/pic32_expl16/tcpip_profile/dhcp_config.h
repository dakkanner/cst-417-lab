/*******************************************************************************
  Dynamic Host Configuration Protocol (DCHP) Configuration file

  Summary:
    DCHP configuration file
    
  Description:
    This file contains the DCHP module configuration options
    
*******************************************************************************/

/*******************************************************************************
FileName:   dhcp_config.h
Copyright © 2011 released Microchip Technology Inc.  All rights
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

#ifndef _DCHP_CONFIG_H_
#define _DCHP_CONFIG_H_


// Defines how long to wait before a DHCP request times out, seconds
#define DHCP_TIMEOUT				(2ul)

// UDP client local port for DHCP Client transactions
#define DHCP_CLIENT_PORT                (68u)
// UDP remote port for DHCP Server
#define DHCP_SERVER_PORT                (67u)

// enable/disable the DHCP client at stack start up
// Note: the interface initialization setting in TCPIP_NETWORK_CONFIG takes precedence!
#define DHCP_CLIENT_ENABLED        1


/*
 * Reference only: DHCP module configuration structure
 *
typedef struct
{
    bool    dhcpEnable;     // enable the DHCP client;
                            // Note: this setting is overridden by the
                            // interface initialization in TCPIP_NETWORK_CONFIG!
    int     dhcpTmo;        // timeout to wait for a DHCP request, seconds
    int     dhcpCliPort;    // client port for DHCP client transactions
    int     dhcpSrvPort;    // remote server port for DHCP server messages
}DHCP_MODULE_CONFIG;
*/

// This is a template of how the DHCP module should be initialized and
// the parameters that it needs.
#ifdef TCPIP_STACK_MODULE_CONFIGURATION
const DHCP_MODULE_CONFIG dhcpConfigData = 
{
    DHCP_CLIENT_ENABLED,
    DHCP_TIMEOUT,
    DHCP_CLIENT_PORT,
    DHCP_SERVER_PORT,
};
#else
extern const DHCP_MODULE_CONFIG dhcpConfigData;
#endif  // TCPIP_STACK_MODULE_CONFIGURATION




#endif  // _DCHP_CONFIG_H_
