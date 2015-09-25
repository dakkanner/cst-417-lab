/*******************************************************************************
  Headers for TCPIP Demo App

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  main_demo.h 
Copyright � 2012 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED �AS IS� WITHOUT WARRANTY OF ANY KIND,
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

#ifndef _MAINDEMO_H
#define _MAINDEMO_H

#include "tcpip/tcpip.h"
#include "common/helpers.h"

// enable the demo-applications that you want to run
#define APP_USE_GENERIC_TCP_CLIENT_DEMO
#define APP_USE_GENERIC_TCP_SERVER_DEMO
#define APP_USE_SMTP_CLIENT_DEMO
#define APP_USE_PING_DEMO
//#define APP_USE_SNMP_TRAP_DEMO
//#define APP_USE_SNMP_V2_TRAP_DEMO
#define APP_USE_BERKELEY_API_DEMO




void SMTPDemo(void);
void SNMPTrapDemo(void);
void SNMPV2TrapDemo(void);
void GenericTCPClient(void);
void GenericTCPServer(void);
void GenericTCPServer_IPv6(void);
void BerkeleyTCPClientDemo(void);
void BerkeleyTCPServerDemo(void);
void BerkeleyUDPClientDemo(uint32_t localIP);

// Ping Demo prototypes
bool Ping4 (char * target);
void PingProcessIPv4 (TCPIP_NET_HANDLE hNetIf, IPV4_ADDR * remoteIP, void * data);
#if defined (TCPIP_STACK_USE_IPV6)
    bool Ping6 (char * target);
    bool PingProcessIPv6 (ICMPV6_HEADER_ECHO * header, IPV6_ADDR * remoteIP, IPV6_ADDR * localIP);
#endif
void PingDemoTask (void);

#define PING_HOST           "ww1.microchip.com"	// Address that ICMP client will ping.  If the DNS client module is not available in the stack, then this hostname is ignored and the local gateway IP address will be pinged instead.
#define PING_HOST_IPV6      "ipv6.google.com"   // Address that ICMPv6 client will ping.


// An actual function defined in MainDemo.c for displaying the current IP 
// address on the UART and/or LCD.
void DisplayIPValue(IPV4_ADDR IPVal);


#endif // _MAINDEMO_H
