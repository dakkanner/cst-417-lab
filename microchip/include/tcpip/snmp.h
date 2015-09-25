/*******************************************************************************
  SNMP Defs for Microchip TCP/IP Stack

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  SNMP.h 
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

#ifndef SNMP_H
#define SNMP_H

#include "tcpip/tcpip.h"
#include "snmp_config.h"
#include "snmp_private.h"



/****************************************************************************
  Section:
	Macros and Definitions 
  ***************************************************************************/

#define SNMP_START_OF_VAR       (0)
#define SNMP_END_OF_VAR         (0xff)
#define SNMP_INDEX_INVALID      (0xff)

//This is the SNMP OID variable id.
//This id is assigned via MIB file.  Only dynamic and AgentID
//variables can contian ID.  MIB2BIB utility enforces this
//rules when BIB was generated.
//typedef int SNMP_ID;

//typedef uint32_t SNMP_ID;
//typedef uint8_t SNMP_INDEX;

// Section:  SNMP agent version types
#define SNMP_V1                 (0)
#define SNMP_V2C				(1)
#define SNMP_V3					(3)

typedef enum
{
	IPV4_SNMP_TRAP=1,
	IPV6_SNMP_TRAP,
}SNMP_TRAP_IP_ADDRESS_TYPE;

// Section:  SNMP trap notification information for agent
typedef struct 
{
	char community[NOTIFY_COMMUNITY_LEN];	//Community name array
	uint8_t communityLen;						//Community name length
	SNMP_ID agentIDVar; 					//Agent id for trap identification
	uint8_t notificationCode;					//Trap notification code
	UDP_SOCKET socket;						//Udp socket number 
	TCPIP_UINT32_VAL timestamp;					//Time stamp for trap
#if defined(SNMP_STACK_USE_V2_TRAP) || defined(SNMP_V1_V2_TRAP_WITH_SNMPV3) 
	SNMP_ID trapIDVar;						// SNMPV2 specific trap
#endif	
	TCPIP_NET_HANDLE snmpTrapInf; // interface we use for the SNMP TRAP

} SNMP_NOTIFY_INFO;


typedef struct tSNMP_TRAP_INFO
{
   uint8_t Size;
   struct
   {
       uint8_t communityLen;					//Community name length
       char community[TRAP_COMMUNITY_MAX_LEN];	//Community name array
       IPV4_ADDR IPAddress;					//IP address to which trap to be sent
       struct
       {
           unsigned int bEnabled : 1;		//Trap enabled flag	
       } Flags;
   } table[TRAP_TABLE_SIZE];				
} SNMP_TRAP_INFO;

#ifdef TCPIP_STACK_USE_IPV6
typedef struct tIPV6_SNMP_TRAP_INFO
{
   uint8_t Size;
   struct
   {
       uint8_t communityLen;					//Community name length
       char community[TRAP_COMMUNITY_MAX_LEN];	//Community name array
       IPV6_ADDR IPv6Address;					//IPv6 address to which trap to be sent
       struct
       {
           unsigned int bEnabled : 1;		//Trap enabled flag	
       } Flags;
   } table[TRAP_TABLE_SIZE];				
} IPV6_SNMP_TRAP_INFO;

#endif  /* TCPIP_STACK_USE_IPV6 */

typedef union
{
    uint32_t dword;				//double word value
    uint16_t  word;					//word value
    uint8_t  byte;					//byte value
    uint8_t  v[sizeof(uint32_t)];		//byte array
} SNMP_VAL;


typedef enum
{
	READ_COMMUNITY=1,		//Read only community	
	WRITE_COMMUNITY=2,		//Read write community
	INVALID_COMMUNITY=3			//Community invalid
}SNMP_COMMUNITY_TYPE;


typedef enum
{
	COLD_START 			=0x0,
	WARM_START			=0x1,
	LINK_DOWN			=0x2,
	LINK_UP				=0x3,
	AUTH_FAILURE		=0x4,	
	EGP_NEBOR_LOSS		=0x5,
	ENTERPRISE_SPECIFIC	=0x6
	
} GENERIC_TRAP_NOTIFICATION_TYPE; 


typedef enum
{
	VENDOR_TRAP_DEFAULT 	=0x0,
	BUTTON_PUSH_EVENT		=0x1,
	POT_READING_MORE_512	=0x2
} VENDOR_SPECIFIC_TRAP_NOTIFICATION_TYPE;



typedef struct
{
	uint8_t oidstr[16];
	uint8_t version;
}SNMPNONMIBRECDINFO;


typedef struct
{
} SNMP_MODULE_GONFIG;

// 	   // the only if that runs SNMP


/****************************************************************************
  Section:
	Global Variables
  ***************************************************************************/

typedef struct
{
	
	uint8_t gOIDCorrespondingSnmpMibID;

	#if defined(SNMP_STACK_USE_V2_TRAP)
	uint8_t	gSetTrapSendFlag;
	#endif
	
	bool getZeroInstance;
	uint8_t appendZeroToOID;

	uint8_t gSendTrapFlag;
	uint8_t gGenericTrapNotification;
	uint8_t gSpecificTrapNotification;
	SNMP_NOTIFY_INFO SNMPNotifyInfo; //notify info for trap
	SNMPBUFFERDATA outPduBufData;
	
	SNMPBUFFERDATA trapPduOutBufData;
	SNMP_NET_CONFIG snmpNetConfig;

}SNMP_STACK_DCPT_STUB;


typedef struct 
{

SNMP_STACK_DCPT_STUB* snmpStkDynMemStubPtr;
const void* snmpHeapMemHandler; 
TCPIP_SNMP_DCPT* snmpDcptPtr;
}SNMP_PROCESSING_MEM_INFO_PTRS;



/****************************************************************************
  Section:
	Function Prototypes
  ***************************************************************************/

#if !defined(SNMP_TRAP_DISABLED)
TCPIP_NET_HANDLE SNMPUdpClientGetNet(void);
void SNMPSendTrap(void);
bool SNMPNotify(SNMP_ID var, SNMP_VAL val, SNMP_INDEX index);
bool SNMPIsNotifyReady(IP_MULTI_ADDRESS* remoteHost,SNMP_TRAP_IP_ADDRESS_TYPE eTrapMultiAddressType);
void SNMPNotifyPrepare(IP_MULTI_ADDRESS* remoteHost, char* community, uint8_t communityLen, SNMP_ID agentIDVar, uint8_t notificationCode, uint32_t timestamp);
#endif

void SNMPGetPktProcessingDynMemStubPtrs( SNMP_PROCESSING_MEM_INFO_PTRS * dynMemInfoPtr);
SYS_TICK SNMPGetTrapTime(void);

bool SNMPSetVar(SNMP_ID var, SNMP_INDEX index,uint8_t ref, SNMP_VAL val);
bool SNMPGetVar(SNMP_ID var, SNMP_INDEX index,uint8_t* ref, SNMP_VAL* val);
bool SNMPGetNextIndex(SNMP_ID var, SNMP_INDEX* index);
bool SNMPGetExactIndex(SNMP_ID var, SNMP_INDEX *index);
bool SNMPIsValidSetLen(SNMP_ID var, uint8_t len,uint8_t index);

uint8_t SNMPValidateCommunity(uint8_t* community);
uint8_t*  SNMPRetrieveWriteCommunity(int index);
uint8_t*  SNMPRetrieveReadCommunity(int index);


#endif
