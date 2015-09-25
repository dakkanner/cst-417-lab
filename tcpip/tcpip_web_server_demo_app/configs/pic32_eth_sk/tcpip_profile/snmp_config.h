/*******************************************************************************
  Simple Network Managment Protocol (SNMP) Configuration file

  Summary:
    SNMP configuration file
    
  Description:
    This file contains the SNMP module configuration options
    
*******************************************************************************/

/*******************************************************************************
FileName:   snmp_config.h
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

#ifndef _SNMP_CONFIG_H_
#define _SNMP_CONFIG_H_


//Trap information.
//This macro will be used to avoid SNMP OID memory buffer corruption 
#define SNMP_TRAP_COMMUNITY_MAX_LEN_MEM_USE   (8)

//This table maintains list of intereseted receivers
//who should receive notifications when some interesting
//event occurs.
#define TRAP_TABLE_SIZE         (2)
#define TRAP_COMMUNITY_MAX_LEN       (SNMP_TRAP_COMMUNITY_MAX_LEN_MEM_USE+1)


// for multi-homed hosts, the default SNMP interface
#define MY_DEFAULT_SNMP_IF             "PIC32INT"

//Name of the bib file for snmp
#define SNMP_BIB_FILE_NAME		"snmp.bib"

//Change this to match your OID string length.
#define OID_MAX_LEN			(18)

// SNMP MIN and MAX message 484 bytes in size
// As per RFC 3411 snmpEngineMaxMessageSize and 
// RFC 1157 ( section 4- protocol specification )
// and implementation  supports more than 484 whenever
// feasible.
#define SNMP_MAX_MSG_SIZE		484

/* Update the Non record id OID value 
   which is part of CustomSnmpDemo.c file
*/
#define SNMP_MAX_NON_REC_ID_OID  3

// This is the maximum length for community string.
// Application must ensure that this length is observed.
// SNMP module adds one byte extra after SNMP_COMMUNITY_MAX_LEN
// for adding '\0' NULL character.
#define SNMP_COMMUNITY_MAX_LEN  	(8u)
#define SNMP_MAX_COMMUNITY_SUPPORT	(3u)
#define NOTIFY_COMMUNITY_LEN		(SNMP_COMMUNITY_MAX_LEN)

// Default SNMPv2C community names.  These can be overridden at run time if
// alternate strings are present in external EEPROM or Flash (actual
// strings coould be stored by the TCPIP storage service.
// These strings are case sensitive.
// An empty string means disabled (not matchable).
// For application security, these default community names should not be
// used, but should all be disabled to force the end user to select unique
// community names.  These defaults are provided only to make it easier to
// start development.  Specifying more strings than
// SNMP_MAX_COMMUNITY_SUPPORT will result in the later strings being
// ignored (but still wasting program memory).  Specifying fewer strings is
// legal, as long as at least one is present.  A string larger than
// SNMP_COMMUNITY_MAX_LEN bytes will be ignored.
#define SNMP_READ_COMMUNITIES		{"public", "read", ""}
#define END_OF_SNMP_READ_COMMUNITIES
#define SNMP_WRITE_COMMUNITIES     	{"private", "write", "public"}
#define END_OF_SNMP_WRITE_COMMUNITIES


// Comment following line if SNMP TRAP support is needed
//#define SNMP_TRAP_DISABLED

#if defined(TCPIP_STACK_USE_SNMP_SERVER) && !defined(SNMP_TRAP_DISABLED)
	#define SNMP_STACK_USE_V2_TRAP
#if defined(TCPIP_STACK_USE_SNMPV3_SERVER)
	#define SNMP_V1_V2_TRAP_WITH_SNMPV3
#endif
#endif 


typedef struct
{
	// SNMPv2C Read community names
	// SNMP_COMMUNITY_MAX_LEN (8) + 1 null termination byte
	uint8_t readCommunity[SNMP_MAX_COMMUNITY_SUPPORT][SNMP_COMMUNITY_MAX_LEN+1]; 

	// SNMPv2C Write community names
	// SNMP_COMMUNITY_MAX_LEN (8) + 1 null termination byte
	uint8_t writeCommunity[SNMP_MAX_COMMUNITY_SUPPORT][SNMP_COMMUNITY_MAX_LEN+1];

	uint32_t SnmpEngineBootRcrd;
}SNMP_NET_CONFIG;



#endif  // _SNMP_CONFIG_H_
