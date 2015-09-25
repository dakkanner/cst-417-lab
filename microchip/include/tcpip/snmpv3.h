/*******************************************************************************

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  SNMPv3.h 
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

#ifndef SNMPV3_H
#define SNMPV3_H

#include "tcpip/tcpip.h"
#include "snmpv3_config.h"
#include "snmpv3_private.h"
#include "tcpip/snmp.h"


typedef struct 
{
	uint16_t UserInfoDataBaseIndx; 

	uint8_t SnmpEngineID[32]; //Reserving 32 bytes for the snmpEngineID as the octet string length can vary form 5 to 32 //**
	uint8_t SnmpEngnIDLength;//**

	uint16_t SnmpMsgBufSeekPos;
	uint16_t ScopedPduDataPos;//**

	uint32_t SnmpEngineTimeOffset;//**
	uint32_t SnmpEngineBoots;//The number of times that the SNMP engine has (re-)initialized itself since snmpEngineID was last configured.//**

	TCPIP_UINT16_VAL UsmStatsEngineID; 	//**
	TCPIP_UINT32_VAL AuthoritativeSnmpEngineBoots;//**
	TCPIP_UINT32_VAL AuthoritativeSnmpEngnTime;//**
	TCPIP_UINT32_VAL IncmngSnmpPduMsgID;//**
	TCPIP_UINT32_VAL SnmpEngineTime;//The number of seconds since the value of the SnmpEngineBoots object last changed//**
	TCPIP_UINT32_VAL SnmpEngnMaxMsgSize;//**

	SNMPV3_REQUEST_WHOLEMSG InPduWholeMsgBuf;//**
	SNMPV3_RESPONSE_WHOLEMSG OUTPduWholeMsgBuf;//**
	SNMPV3_RESPONSE_WHOLEMSG TrapOUTPduWholeMsgBuf;

	//snmv3 global database for trap table
	snmpV3TrapConfigDataBase Snmpv3TrapConfigData[SNMPV3_USM_MAX_USER];

	SNMPV3MSGDATA ScopedPduRequstBuf;
	SNMPV3MSGDATA ScopedPduRespnsBuf;
	SNMPV3MSGDATA PduHeaderBuf;
	SNMPV3MSGDATA TrapMsgHeaderBuf;
	SNMPV3MSGDATA TrapScopdPduRespnsBuf;

	dispatcherProcessPdu incomingPdu;

	uint8_t  SnmpSecurityLevel; 
	uint8_t  SnmpRespnsSecrtyFlg;

	uint8_t SnmpInMsgAuthParmStrng[12+1];
	uint8_t SnmpInMsgAuthParamLen;
	uint8_t snmpInMsgPrvParamStrng[8+1];
	uint8_t SnmpInMsgPrivParmLen;

	uint8_t SnmpOutMsgAuthParaStrng[12+1];
	uint8_t SnmpOutMsgAuthParmLen;
	uint8_t SnmpOutMsgPrvParmStrng[8+1];
	uint8_t SnmpOutMsgPrivParmLen;

	uint32_t SnmpEngnSecurityModel;//Maximum range (2^31-1), RFC3411    
	uint32_t SnmpEngnMsgProcessModel;//Maximum range (2^31-1), RFC3411 

	SecuritySysProcessIncomingMsg SecurtyPrimtvesOfIncmngPdu;//**

	snmpV3EngnUserDataBase UserInfoDataBase[SNMPV3_USM_MAX_USER];//**

}SNMPV3_STACK_DCPT_STUB;


typedef struct 
{

SNMPV3_STACK_DCPT_STUB * snmpv3StkProcessingDynMemStubPtr;
const void* snmpHeapMemHandler; 
}
SNMPV3_PROCESSING_MEM_INFO_PTRS;



void SNMPv3ComputeHMACIpadOpadForAuthLoclzedKey(uint8_t userDBIndex);


 bool SNMPv3CmprTrapSecNameAndSecLvlWithUSMDb(uint8_t tragetIndex,uint8_t userTrapSecLen,
			uint8_t *userTrapSecurityName,STD_BASED_SNMPV3_SECURITY_LEVEL securityLevel);

 void SNMPv3USMAuthPrivPswdLocalization(uint8_t userDBIndex);




 bool SNMPv3Notify(SNMP_ID var, SNMP_VAL val, SNMP_INDEX index,uint8_t targetIndex);
 void SNMPv3GetPktProcessingDynMemStubPtrs( SNMPV3_PROCESSING_MEM_INFO_PTRS * dynMemInfoPtr);

#endif
