/*******************************************************************************
Microchip TCP/IP Stack Network Configuration Header

  Summary:
    
  Description:

*******************************************************************************/

/*******************************************************************************
FileName: 	network_config.h
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

#ifndef __NETWORK_PROFILE_H_
#define __NETWORK_PROFILE_H_

// =======================================================================
//   Network Interfaces Configuration Options
// =======================================================================

// Defalut values
// These values are used only if the application doesn't supply
// run time parameters for the TCPIP stack!

// the default number of interfaces
// Always 1 if no runtime parameter is supplied!
#define TCPIP_NETWORK_DEFAULT_INTERFACES_NO      1

// default values for the default interface
// Adjust them as needed.
#define TCPIP_NETWORK_DEFAULT_INTERFACE_NAME    "ENCJ60"
#define TCPIP_NETWORK_DEFAULT_HOST_NAME         "MCHPBOARD_E"
#define TCPIP_NETWORK_DEFAULT_MAC_ADDR          0
#define TCPIP_NETWORK_DEFAULT_IP_ADDRESS        "192.168.100.115"
#define TCPIP_NETWORK_DEFAULT_IP_MASK           "255.255.255.0"
#define TCPIP_NETWORK_DEFAULT_GATEWAY           "192.168.100.1"
#define TCPIP_NETWORK_DEFAULT_DNS               "192.168.100.1"
#define TCPIP_NETWORK_DEFAULT_SECOND_DNS        "0.0.0.0"
#define TCPIP_NETWORK_DEFAULT_POWER_MODE        "full"
#define TCPIP_NETWORK_INTERFACE_FLAGS   (TCPIP_NETWORK_CONFIG_DHCP_CLIENT_ON)


#ifdef TCPIP_STACK_MODULE_CONFIGURATION
// default configuration per each supported interface
const TCPIP_NETWORK_CONFIG __attribute__((unused))  TCPIP_HOSTS_CONFIGURATION[] =
{
    {
        "ENCJ60",               // interface
        "MCHPBOARD_E",          // hostName
        "00:04:a3:00:00:00",    // macAddr: use "00:04:a3:00:00:00" or 0 for the factory pre-programmed address
        "169.254.1.2",          // ipAddr 
        "255.255.0.0",          // ipMask
        "169.254.1.1",          // gateway
        "169.254.1.1",          // priDNS
        "0.0.0.0",              // secondDNS
        "full",                 // powerMode
        (TCPIP_NETWORK_CONFIG_DHCP_CLIENT_ON),                   // start up flags
    }, 
    {
        "MRF24W",            // interface
        "MCHPBOARD_W",          // hostName
        "00:04:a3:00:00:00",    // macAddr: use "00:04:a3:00:00:00" or 0 for the factory pre-programmed address
        "192.168.2.105",        // ipAddr 
        "255.255.255.0",        // ipMask
        "192.168.2.1",          // gateway
        "192.168.2.1",          // priDNS
        "0.0.0.0",              // secondDNS
        "full",                 // powerMode
        (TCPIP_NETWORK_CONFIG_DHCP_CLIENT_ON),                   // start up flags
    },
    
};
#else
// if other modules need access to the default configuration
extern const TCPIP_NETWORK_CONFIG __attribute__((unused))  TCPIP_HOSTS_CONFIGURATION[];
#endif  // TCPIP_STACK_MODULE_CONFIGURATION





#endif  // __NETWORK_PROFILE_H_

