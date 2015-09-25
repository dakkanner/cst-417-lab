/*******************************************************************************
 *
 *  Simple Network Management Protocol (SNMP) Version 3 Agent 
 *  
 *  Module for Microchip TCP/IP Stack
 *	 -Provides SNMPv3 API for doing stuff
 *	
 *	-Reference: RFCs 3410, 3411, 3412, 3413, 3414 
*******************************************************************************
 * FileName:  snmpv3.c 
 * Copyright © 2012 released Microchip Technology Inc.  All rights
 * reserved.
 *
 * Microchip licenses to you the right to use, modify, copy and distribute
 * Software only when embedded on a Microchip microcontroller or digital signal
 * controller that is integrated into your product or third party product
 * (pursuant to the sublicense terms in the accompanying license agreement).
 *
 * You should refer to the license agreement accompanying this Software for
 * additional information regarding your rights and obligations.
 *
 * SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
 * MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
 * IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
 * CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
 * OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
 * INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
 * CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
 * SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
 * (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/

#include "tcpip_private.h"

#if defined(TCPIP_STACK_USE_SNMPV3_SERVER)
#include "tcpip/snmpv3.h"

#define TCPIP_THIS_MODULE_ID    TCPIP_MODULE_SNMPV3_SERVER

static SNMPV3_STACK_DCPT_STUB * Snmpv3StackDcptStubPtr=0;

#if defined(SNMP_V1_V2_TRAP_WITH_SNMPV3)&& !defined(SNMP_TRAP_DISABLED)
static uint32_t Snmpv3FileDescrptr;
#endif

typedef struct 
{
	uint8_t privAndAuthFlag:2;
    uint8_t reportableFlag :1;
}snmpV3MsgFlagsBitWise;


/*   
SNMP_ENGINE_MAX_MSG_SIZE is determined as the minimum of the max msg size
values supported among all of the transports available to and supported by the engine. 
*/
#define SNMP_ENGINE_MAX_MSG_SIZE	1024 
#define SET_SNMPV3_GET_PDU_INDEX  (Snmpv3StackDcptStubPtr->ScopedPduRequstBuf.length++)


/*Length of the SNMPv3 msg header(x) = Header length (2 bytes) 
+ MSGID size (type(1 byte) + length of value(1 byte)+4 bytes value)
+ msgMAXSIZE(type + length of value +4 bytes value) 
+ msg flag(type + length of value +1 byte value)
+ security model type(type + length of value +1 byte value) */
#define MSGGLOBAL_HEADER_LEN(x)	( x= (2 \
							  +1+1+4 \
							  +1+1+4 \
							  +1+1+1  \
							  +1+1+1)\
						    )
/*Length of SNMPv3 authoratative msg header length = 
Header length ( 2 + 2 bytes)  + engineID ( snmpEngnIDLength bytes)
+ engine boot( 4 bytes)+ engine time(4 bytes)
+security name (securityPrimitivesOfIncomingPdu value)
+authentication parameters (snmpOutMsgAuthParamLen value)
+privacy parameters (snmpOutMsgAuthParamLen value)*/
#define MSG_AUTHORITATIVE_HEADER_LEN(x)   ( x=(2+2 \
								     +1+1+Snmpv3StackDcptStubPtr->SnmpEngnIDLength \
								     +1+1+4 \
								     +1+1+4 \
								     +1+1+Snmpv3StackDcptStubPtr->SecurtyPrimtvesOfIncmngPdu.securityNameLength \
								     +1+1+Snmpv3StackDcptStubPtr->SnmpOutMsgAuthParmLen \
								     +1+1+Snmpv3StackDcptStubPtr->SnmpOutMsgPrivParmLen) \
								)






static bool SNMPv3CheckIfValidInt(uint32_t* val);
static bool SNMPv3CheckIfv3ValidStructure(uint16_t* dataLen);
static uint8_t SNMPv3FindOIDsFrmIncmingV3Req(uint16_t pdulen);

static void SNMPv3ConstructReportPdu(SNMPV3MSGDATA *dynScopedBufPtr);
static bool SNMPv3CheckIfv3ValidOID(uint8_t* oid, uint8_t* len);
static bool SNMPv3CheckIfv3ASNNull(void);
static void SNMPv3SetErrorStatus(uint16_t errorStatusOffset,
                           uint16_t errorIndexOffset,
                           SNMP_ERR_STATUS errorStatus,
                           uint8_t errorIndex,SNMPV3MSGDATA *dynScopedPduPutBuf);

static bool SNMPv3CheckIfValidIntDataType(uint8_t* wholeMsgPtr,uint16_t* pos, uint32_t* val );
static bool SNMPv3CheckIfValidV3StructAnd4ByteDataLen(uint8_t* wholeMsgPtr,uint16_t* pos, uint16_t* dataLen );
static uint8_t SNMPv3CheckIfValidAuthStructure(uint16_t* dataLen);
static void SNMPv3FormulateEngnID(uint8_t fifthOctectIdentifier,TCPIP_NET_IF* snmpIntf);
extern void SaveAppConfig(void);



/****************************************************************************
  Function:
	void SNMPv3Init(TCPIP_NET_IF* netIf)
	
  Summary:
  	Initialize the Snmpv3 agent Username and Security Values and SNMPv3 engine ID.

  Description:
	SNMPv3 Engine ID initilization happens with proper interface details.
	So this API is called from SNMPTask() .
	  	 		 		  	
  Precondition:
   	SNMPInit(); is called. 
		
  Parameters:
  	netIf - Interface  details
  	
  Return Values:
	None

  Remarks:
  	None
***************************************************************************/
void SNMPv3Init(TCPIP_NET_IF* netIf)
{
	uint8_t userDBIndex,fifthOctectIdentifier;
	SNMPV3_PROCESSING_MEM_INFO_PTRS snmpv3PktProcessingMemPntr; 


	SNMPv3GetPktProcessingDynMemStubPtrs(&snmpv3PktProcessingMemPntr);

	Snmpv3StackDcptStubPtr=snmpv3PktProcessingMemPntr.snmpv3StkProcessingDynMemStubPtr;
	

	SNMPv3InitializeUserDataBase();

	fifthOctectIdentifier=MAC_ADDR_ENGN_ID;
	
	SNMPv3FormulateEngnID(fifthOctectIdentifier,netIf);

	for(userDBIndex=0;userDBIndex<SNMPV3_USM_MAX_USER;userDBIndex++)
	{
	
		SNMPv3USMAuthPrivPswdLocalization(userDBIndex);
		SNMPv3ComputeHMACIpadOpadForAuthLoclzedKey(userDBIndex);

	}
}




/****************************************************************************
  Function:
	uint8_t SNMPv3ConfigEngnSecrtyModel(uint32_t securityModelInRequest)
	
  Summary:
  	Updates the Snmp agent security model value.

  Description:
	Checks for the security model value the in the request pdu against the security 
	model value specified in the RFC3411 max range.
  	Updates the snmp agent global variable 'SnmpEngnSecurityModel' for storing 
  	security model value reuqested in the incoming request from the SNMP managers.
	  	 		 		  	
  Precondition:
   	SNMPInit(); is called. The agent is recognised in the network as the SNMPv3 node.
   	Snmp request is received.
		
  Parameters:
  	securityModelInRequest : this value is populated from the incoming request PDUs.
  	
  Return Values:
	true:	If the incoming PDU has valid security model value.
	false:    If the incoming PDU has invalid or un supported security model.

  Remarks:
  	'SnmpEngnSecurityModel' is value to uniquely identiy a security model of the 
  	security sub system whithin the SNMP management architecture. This value is 
  	used by the SNMP engine to send respond or inform PDUs to the other SNMP nodes.
  	'SnmpEngnSecurityModel' value is used for interoperability. 
***************************************************************************/
uint8_t SNMPv3ConfigEngnSecrtyModel(uint32_t securityModelInRequest)
{
	if(securityModelInRequest < 0x80000000  )
	{
		
		if(securityModelInRequest > ANY_SECUTIRY_MODEL && 
		   securityModelInRequest <= SNMPV3_USM_SECURITY_MODEL
		   /* && User can add the Enterprise specific Security Model if reuired and is implemented */ )
		{
			Snmpv3StackDcptStubPtr->SnmpEngnSecurityModel=securityModelInRequest;
			return true;
		}
		else
			return false;
	}
	else
		return false;
}


/****************************************************************************
  Function:
	uint8_t SNMPv3EngnDecodeMsgFlags(uint8_t msgFlasgInRequest)
	
  Summary:
  	Returns the message flag value in the received incoming snmp request pdu. 

  Description:
	Compare the message flag value the in the request pdu against the Octet's 
	least significant three bits: Reportable, PrivFlag, AuthFlag. Return the message flag
	value to message processing model to take corresponding decision for the 
	message processing flow.  
		  	 		 		  	
  Precondition:
   	SNMPInit(); is called. The agent is recognised in the network as the SNMPv3 node.
	Snmp request is received.	

  Parameters:
  	'msgFlasgInRequest' is value populated from the incoming request PDUs.
  	
  Return Values:
	INVALID_MSG: If the incoming PDU has undefined message flag value  or 
	REPORT_FLAG_AND_SECURITY_LEVEL_FLAGS: if not INVALID_MSG, then 
	any other matching value in the enum except INVALID_MSG.

  Remarks:
  	'msgFlasgInRequest' is compared to possible combinations of snmp message flags.
  	The return value from this routine is utilised by the message processing unit to 
  	decide on the course of action to be taken while processing the incoming request. 
***************************************************************************/

uint8_t SNMPv3EngnDecodeMsgFlags(uint8_t msgFlasgInRequest)
{
	if(msgFlasgInRequest > REPORT2REQ_PRIVACY_AND_AUTH_PROVIDED) /*if all msgFlags are SET*/
	{
		return INVALID_MSG;
	}
	else if( (msgFlasgInRequest & NO_REPORT_NO_PRIVACY_NO_AUTH) == NO_REPORT_NO_PRIVACY_NO_AUTH)
	{
		return NO_REPORT_NO_PRIVACY_NO_AUTH;
	}
	else if( (msgFlasgInRequest & NO_REPORT_NO_PRIVACY_BUT_AUTH_PROVIDED) == NO_REPORT_NO_PRIVACY_BUT_AUTH_PROVIDED)
	{
		return NO_REPORT_NO_PRIVACY_BUT_AUTH_PROVIDED;
	}
	else if( (msgFlasgInRequest & NO_REPORT_PRIVACY_PROVIDED_BUT_NO_AUTH) == NO_REPORT_PRIVACY_PROVIDED_BUT_NO_AUTH)
	{
		return INVALID_MSG;
	}
	else if( (msgFlasgInRequest & NO_REPORT_PRIVACY_AND_AUTH_PROVIDED) == NO_REPORT_PRIVACY_AND_AUTH_PROVIDED)
	{
		return NO_REPORT_PRIVACY_AND_AUTH_PROVIDED;
	}

	else if( (msgFlasgInRequest & REPORT2REQ_NO_PRIVACY_NO_AUTH) == REPORT2REQ_NO_PRIVACY_NO_AUTH)
	{
		return REPORT2REQ_NO_PRIVACY_NO_AUTH;
	}
	else if( (msgFlasgInRequest & REPORT2REQ_NO_PRIVACY_BUT_AUTH_PROVIDED) == REPORT2REQ_NO_PRIVACY_BUT_AUTH_PROVIDED)
	{
		return REPORT2REQ_NO_PRIVACY_BUT_AUTH_PROVIDED;
	}
	else if( (msgFlasgInRequest & REPORT2REQ_PRIVACY_PROVIDED_BUT_NO_AUTH) == REPORT2REQ_PRIVACY_PROVIDED_BUT_NO_AUTH)
	{
		return INVALID_MSG;
	}
	else if( (msgFlasgInRequest & REPORT2REQ_PRIVACY_AND_AUTH_PROVIDED) == REPORT2REQ_PRIVACY_AND_AUTH_PROVIDED)
	{
		return REPORT2REQ_PRIVACY_AND_AUTH_PROVIDED;
	}
	else
		return INVALID_MSG;
}



/****************************************************************************
  Function:
	uint32_t SNMPv3TrackAuthEngineTimeTick(void)
	
  Summary:
  	Returns the internal tick timer value to be used by 'SnmpEngineTime'.
	
  Description:
  	
  Precondition:
   	None

  Parameters:
  	None
  	
  Return Values:
	timeStamp : uint32_t value of the timer ticks 

  Remarks:
	None
***************************************************************************/
uint32_t SNMPv3TrackAuthEngineTimeTick(void)
{

    SYS_TICK    timeStamp;

    // convert the current system tick to 10 ms units
    timeStamp = (SYS_TICK_Get() * 100ull)/SYS_TICK_TicksPerSecondGet();

    return timeStamp;

}


/****************************************************************************
  Function:
	static void SNMPv3FormulateEngnID(uint8_t fifthOctectIdentifier,TCPIP_NET_IF* snmpIntf )
	
  Summary:
  	Formulates the SnmpEngineID for the SNMPV3 engine.

  Description:
  	Formulates the SnmpEngineID depending on value of  'fifthOctectIdentifier'.
	as MAC_ADDR_ENGN_ID using the application MAC address. 
  	'fifthOctectIdentifier' defualt set to MAC_ADDR_ENGN_ID as the following octets 
  	used for the SnmpEngineID are of mac address.

	User can set this octet of their choice to fomulate new SnmpEngineID.
	fifthOctectIdentifier=IPV4_ADDR_ENGN_ID; 
	
	If 
	fifthOctectIdentifier=ADMIN_ASSIGNED_TEXT;  or
	fifthOctectIdentifier=ADMIN_ASSIGNED_OCTETS;
	then the following octets should be provided by the administrator through some 
	custom application interface mechanism.
	API parameter 'fifthOctectIdentifier' has to be upated in the intefrace API before 
	passing through SNMPv3FormulateEngnID().  
  	 		 		  	
  Precondition:
   	InitAppConfig(); is called. 
		
  Parameters:
  	fifthOctectIdentifier : Value of the 5th octet in the SnmpEngineID which indicates 
  	how the rest (6th and following octets) are formatted.
  	
  Return Values:
	None

  Remarks:
	Authentication and encryption keys are generated using corresponding passwords and 
	SnmpEngineID. If the SnmpEngineID is newly configured, then the auth and privacy keys 
	would also change. Hence while using this API to change the SnmpEngineID dynamically,
	care should be taken to update the new localized keys at the agent as well as at the manager.
***************************************************************************/
static void SNMPv3FormulateEngnID(uint8_t fifthOctectIdentifier,TCPIP_NET_IF* snmpIntf )
{

uint8_t* dummyptr;
unsigned int i;

	SNMP_PROCESSING_MEM_INFO_PTRS snmpPktProcsMemPtrsInfo; 
	SNMP_STACK_DCPT_STUB * snmpStkDcptMemStubPtr=0;		
	//Modify this private enterprise assigned number to your organization number asssigned by IANA 
	TCPIP_UINT32_VAL mchpPvtEntpriseAssignedNumber; 

	mchpPvtEntpriseAssignedNumber.Val = 0x42C7; //microchip = 17095.
						
	SNMPGetPktProcessingDynMemStubPtrs(&snmpPktProcsMemPtrsInfo);
	snmpStkDcptMemStubPtr=snmpPktProcsMemPtrsInfo.snmpStkDynMemStubPtr;


	//Set the first bit as '1b' .  Refer to RFC3411 section5 Page# 41	
	mchpPvtEntpriseAssignedNumber.Val = ((0x80000000) | mchpPvtEntpriseAssignedNumber.Val);

	Snmpv3StackDcptStubPtr->SnmpEngineID[0]=mchpPvtEntpriseAssignedNumber.v[3];
	Snmpv3StackDcptStubPtr->SnmpEngineID[1]=mchpPvtEntpriseAssignedNumber.v[2];
	Snmpv3StackDcptStubPtr->SnmpEngineID[2]=mchpPvtEntpriseAssignedNumber.v[1];
	Snmpv3StackDcptStubPtr->SnmpEngineID[3]=mchpPvtEntpriseAssignedNumber.v[0];

	//Refer to RFC3411 section5 Page# 41	
	fifthOctectIdentifier=MAC_ADDR_ENGN_ID;
	
	Snmpv3StackDcptStubPtr->SnmpEngineID[4]= fifthOctectIdentifier;

	if(fifthOctectIdentifier == MAC_ADDR_ENGN_ID)
	{
		for(i=0;i<6/*sizeof(MAC_ADDR)*/;i++)
		{
			Snmpv3StackDcptStubPtr->SnmpEngineID[5+i]=snmpIntf->netMACAddr.v[i];
		}
		
		Snmpv3StackDcptStubPtr->SnmpEngineID[5+6/*sizeof(MAC_ADDR)*/]='\0';
		Snmpv3StackDcptStubPtr->SnmpEngnIDLength=4/* 4 Bytes of IANA Pvt Enterprise Assigned Number*/
						+1/* 1 Byte for fifthOctectIdentifier*/
						+6/*sizeof(MAC_ADDR)*/;
	}
	else if(fifthOctectIdentifier == IPV4_ADDR_ENGN_ID)
	{
		dummyptr= (uint8_t*)strncpy((char *)&Snmpv3StackDcptStubPtr->SnmpEngineID[5], (const char *) &snmpIntf->netMACAddr, sizeof(IP_ADDR));
	
		Snmpv3StackDcptStubPtr->SnmpEngineID[5+4/*sizeof(IP_ADDR)*/]='\0';
		Snmpv3StackDcptStubPtr->SnmpEngnIDLength=4/* 4 Bytes of IANA Pvt Enterprise Assigned Number*/
						+1/* 1 Byte for fifthOctectIdentifier*/
						+4 /*sizeof(IP_ADDR)*/;
	}
	else if((fifthOctectIdentifier == ADMIN_ASSIGNED_TEXT )||(fifthOctectIdentifier == ADMIN_ASSIGNED_OCTETS))
	{

		//Interface API updates the  Snmpv3StackDcptStubPtr->SnmpEngineID[4] = fifthOctectIdentifier 
		//and Snmpv3StackDcptStubPtr->SnmpEngineID[5] onwords with the corresponding octet string or value.
		;
		//Snmpv3StackDcptStubPtr->SnmpEngnIDLength=strlen((const char*) Snmpv3StackDcptStubPtr->SnmpEngineID);
	}
		
	
	//Increment the SnmpEngineBoots record as Snmpv3StackDcptStubPtr->SnmpEngineID reconfigured
	Snmpv3StackDcptStubPtr->SnmpEngineBoots+=1;

	//Increment the snmpEngineBootRcrd to be stored in the non volatile memory 
	snmpStkDcptMemStubPtr->snmpNetConfig.SnmpEngineBootRcrd=Snmpv3StackDcptStubPtr->SnmpEngineBoots;

	//Store the new incremented boot record to the non volatile memory
//	SaveAppConfig();

	//Reset the snmEngineTime as SnmpEngineBoots incremented
	Snmpv3StackDcptStubPtr->SnmpEngineTime.Val=0;

	Snmpv3StackDcptStubPtr->SnmpEngineTimeOffset=SNMPv3TrackAuthEngineTimeTick();
}




/****************************************************************************
  Function:
	void SNMPv3MaintainEngineBootsRcrd(void)
	
  Summary:
  	Updates the snmp engine boots counter for the SNMPV3 engine.

  Description:
  	Updates the global SnmpEngineBoots counter variable with the boots counter stored in the 
  	non volatile memory. Also increments and stores the new incremented value  to the non 
  	volatile memoryafter the SNMP engine initialization at the power cycle.
  	 		 		  	
  Precondition:
   	InitAppConfig(); is called.	

  Parameters:
  	None
  	
  Return Values:
	None

  Remarks:
	Should be called only during tcp/ip stack initialization and only after the InitAppConfig();
	is called.
***************************************************************************/
void SNMPv3MaintainEngineBootsRcrd(void)
{
	SNMP_PROCESSING_MEM_INFO_PTRS snmpPktProcsMemPtrsInfo; 
	SNMP_STACK_DCPT_STUB * snmpStkDcptMemStubPtr=0;		
						
	SNMPGetPktProcessingDynMemStubPtrs(&snmpPktProcsMemPtrsInfo);
	snmpStkDcptMemStubPtr=snmpPktProcsMemPtrsInfo.snmpStkDynMemStubPtr;

	//Increment the snmpEngineBootRcrd as due to power cycle snmp egine reinitialization happened 
	//to be stored in the non volatile memory 
	snmpStkDcptMemStubPtr->snmpNetConfig.SnmpEngineBootRcrd+=1;

	//Assgined this counter value to Global cntr which will track the snmp engine boot in real time.
	//'snmpEnigineBoots' can incremement in case of 'SnmpEngineTime' overflow or adminstrator configuring  
	//'SnmpEngineID' with new 'fifthOctectIdentifier' through administrative interface.
	Snmpv3StackDcptStubPtr->SnmpEngineBoots=snmpStkDcptMemStubPtr->snmpNetConfig.SnmpEngineBootRcrd;

	//Store the new incremented boot record to the non volatile memory
	//SaveAppConfig();


}



/****************************************************************************
  Function:
	void SNMPv3GetAuthEngineTime(void)
	
  Summary:
  	Updates the snmp engine time variable 'SnmpEngineTime' for the SNMPV3 engine. 
	
  Description:
  	'SnmpEngineTime' is used for Timeliness checking for Message level security. Snmp 
  	engine keep updating the ''SnmpEngineTime' variable for checking the time window 
  	for the requrest and responses/inform etc. This routine also updates SnmpEngineBoots
  	in scenarios of internal timer reset or 'SnmpEngineTime' cntr ovrflowed 
  	the (2^31 -1) value specified in RFC3411. 
  	 		 		  	
  Precondition:
   	SNMPInit(); is called.	

  Parameters:
  	None
  	
  Return Values:
	None

  Remarks:
	This routine is called every time the rx/tx PDU processing is handled  by the SNMP agent.
	Updates the 'SnmpEngineTime' and requires frequnet access to internal timer registers. 
***************************************************************************/
void SNMPv3GetAuthEngineTime(void)
{
	Snmpv3StackDcptStubPtr->SnmpEngineTime.Val=SNMPv3TrackAuthEngineTimeTick()-Snmpv3StackDcptStubPtr->SnmpEngineTimeOffset;

	if((SNMPv3TrackAuthEngineTimeTick() < Snmpv3StackDcptStubPtr->SnmpEngineTimeOffset)/* Internal Timer Reset occured*/
	   ||(Snmpv3StackDcptStubPtr->SnmpEngineTime.Val > 2147483647 /* (2^31 -1) Refer RFC 3411 Section 5 */))
	{
		/*This means the SnmpEngineTime cntr ovrflowed the (2^31 -1) value 
		    or the internal Tick timer Reset occured*/
		Snmpv3StackDcptStubPtr->SnmpEngineTime.Val=0;
		Snmpv3StackDcptStubPtr->SnmpEngineTimeOffset=0;

		//Increment the SnmpEngineBoots counter
		SNMPv3MaintainEngineBootsRcrd();
		
	}


}




/****************************************************************************
  Function:
	uint8_t SNMPv3NegotiateEngnMaxMsgSize(uint32_t maxMsgSizeInRequest)
	
  Summary:
  	Snmp engine max pdu message size is updated for the pdu recipient from this
  	Snmp agent.
	
  Description:
  	This routine defines the maximum size PDU that could be generated or received 
  	from this SNMP agent. The maximum size is limited to the maximum pdu size the
  	recipient can receive and process or originator can sent. For this SNMP engine, 
  	maximum message size is limited between 484 and 'SNMP_ENGINE_MAX_MSG_SIZE'. 
   	SNMP_ENGINE_MAX_MSG_SIZE is determined as the minimum of the maximum 
   	message size values supported among all of the transports available to and supported 
   	by the engine. 
   	
   Precondition:
   	The SNMP engine should have received a message from the other Snmp node 
   	notifying the maximum message size they can receive and process or send. 
 

  Parameters:
  	maxMsgSizeInRequest: incoming SNMPv3 request max msg size value
  	
  Return Values:
	true:  If the incoming message size is in the predefined range
	false: If the incoming maximum messgae size is less than 484 bytes 

  Remarks:
	SNMP_ENGINE_MAX_MSG_SIZE should not be more than 0x80000000 (2^31 -1).
	
***************************************************************************/
uint8_t SNMPv3NegotiateEngnMaxMsgSize(uint32_t maxMsgSizeInRequest)
{

	Snmpv3StackDcptStubPtr->SnmpEngnMaxMsgSize.Val=SNMP_ENGINE_MAX_MSG_SIZE;//"The maximum length in octets of an SNMP message ranges 484 to (2^31-1), send or receive and process.RFC3411


	if(maxMsgSizeInRequest > 0x80000000 || maxMsgSizeInRequest< 484)

		return false;

	else if(maxMsgSizeInRequest < SNMP_ENGINE_MAX_MSG_SIZE ) 
	{
		Snmpv3StackDcptStubPtr->SnmpEngnMaxMsgSize.Val=maxMsgSizeInRequest;
		return true;
	}
	else
		Snmpv3StackDcptStubPtr->SnmpEngnMaxMsgSize.Val=SNMP_ENGINE_MAX_MSG_SIZE;
		return true;
}


/****************************************************************************
  Function:
	bool SNMPv3CopyDataToProcessBuff(uint8_t val ,SNMPV3MSGDATA *putbuf)
	
  Summary:
  	Copies uint8_t data to dynamically allocated memory buffer.

  Description:
	The SNMPv3 stack implementation uses dynamically allocated memory buffer for
	processing of request and response packets. This routine copies the uint8_t data to the 
	allocated buffer and updates the offset length couter. 
		  	 		 		  	
  Precondition:
	The SNMPv3 stack has sucessfully allocated dynamic memory buffer from the Heap
   	
  Parameters:
  	val: uint8_t value to be written to the buffer
  	putbuf: pointer to the dynamically allocated buffer to which the 'val' to be written 
  	
  Return Values:
	true: if successfully write to the buffer
	false: failure in writing to the buffer
	
  Remarks:
  	This routine is used by the SNMPv3 stack. If required to be used by the application
  	code, valid pointers should be passed to this routine. 
  	
***************************************************************************/
bool SNMPv3CopyDataToProcessBuff(uint8_t val ,SNMPV3MSGDATA *putbuf)
{
	if(putbuf->maxlength > putbuf->length)
	{
		putbuf->head[putbuf->length] = (uint8_t)val;
		putbuf->length++;
		return true;
	}
	else
	{
		return false;
	}

}

/****************************************************************************
  Function:
	bool SNMPv3GetProcessBuffData(SNMPV3MSGDATA getbuf,uint16_t pos)
	
  Summary:
  	Reads uint8_t data from dynamically allocated memory buffer.

  Description:
	The SNMPv3 stack implementation uses dynamically allocated memory buffer for
	processing of request and response packets. This routine reads the uint8_t data from 
	the allocated buffer at the positions (offset) provided.
		  	 		 		  	
  Precondition:
	The SNMPv3 stack has sucessfully allocated dynamic memory buffer from the Heap
   	
  Parameters:
  	getbuf: Structure from where to read the data byte.
  	pos: position in the buffer from which the data to be read 
  	
  Return Values:
	uint8_t: 1 byte value read
	
  Remarks:
  	The read position offset is required to be provided every time the routine is called.
  	This API do not increment the buffer read offset automatically, everytime it is called. 
  	
***************************************************************************/	
uint8_t SNMPv3GetProcessBuffData(SNMPV3MSGDATA getbuf,uint16_t pos)
{
	return (uint8_t)(getbuf.head[pos]);
}


/****************************************************************************
  Function:
	uint8_t SNMPv3GetWholeMsgBufferData(uint8_t* getbuf, uint16_t* pos)
	
  Summary:
  	Reads uint8_t data from dynamically allocated memory buffer.

  Description:
	The SNMPv3 stack implementation uses dynamically allocated memory buffer for
	processing of request and response packets. This routine reads the uint8_t data from 
	the allocated buffer at the positions (offset) provided.
		  	 		 		  	
  Precondition:
	The SNMPv3 stack has sucessfully allocated dynamic memory buffer from the Heap
   	
  Parameters:
  	getbuf: pointer to the buffer where the request or repsonse data is stored.
  	pos: pointer to the position in the buffer from which the data to be read 
  	
  Return Values:
	uint8_t: 1 byte value read
	
  Remarks:
  	This API increments the read offset every time it is called. Hence useful in the 
  	consecutive reads.  
  	
***************************************************************************/	
uint8_t SNMPv3GetWholeMsgBufferData(uint8_t* getbuf, uint16_t* pos)
{
uint8_t* testPtr;
uint16_t* posPtr;

	posPtr=pos; 
	testPtr=getbuf+*posPtr;
	
	*pos=(*posPtr+1);
	return (uint8_t)*(testPtr);
}


/****************************************************************************
  Function: 
  	SNMP_ACTION SNMPv3MsgProcessingModelProcessPDU(uint8_t inOutPdu)

  Summary:
  	This routine collects or populates the message processing model infomation 
  	from the received SNMPv3 request PDU or to the response PDU respectively.
  
  Description:
	The recievd SNMPv3 PDU or the transmit PDU header has message processing
	data bytes infomration. This routine retrievs the messgae processing model 
	infomration from the stored pdu or write the appropriate msg proc info to the 
	repsonse msg buffer.
		  	 		 		  	
  Precondition:
	Valid SNMPv3 request msg is received.
   	
  Parameters:
  	inOutPdu: indicates whether the incomig PDU is to be read for msg proc values 
  	to be retrieved or the response PDU is to be populated with these values 
  	
  Return Values:
	SNMP_NO_CREATION: Failure due to improper msg processing information format in
					      the received PDU or failure in constructing the response PDU.
	SNMP_NO_ERR: The message processing infomration retrieval or response PDU 
				   fomration is successful
				   
  Remarks:
  	The messgae processing model parameters like 'msgID', 'msgMaxSize', 'msgFlags' and 
  	'msgSecurityModel' decides the SNMPv3 engine processing modalities regarding 
  	request or response PDU  	
***************************************************************************/
SNMP_ERR_STATUS SNMPv3MsgProcessingModelProcessPDU(INOUT_SNMP_PDU inOutPdu)
{
	TCPIP_UINT32_VAL tempLen={0};
	uint8_t tempData=0;	
	uint8_t *ptr=NULL;
	uint16_t tempPos=0,snmpv3Headerlength=0;
	uint8_t snmpv3MsgGlobalHeaderlength=0;
	uint8_t snmpv3MsgAuthHedaerLength = 0;	

	  
	SNMPV3_PROCESSING_MEM_INFO_PTRS snmpv3PktProcessingMemPntr; 
	SNMPv3GetPktProcessingDynMemStubPtrs(&snmpv3PktProcessingMemPntr);
					

	if(inOutPdu == SNMP_REQUEST_PDU)
	{
		tempPos=Snmpv3StackDcptStubPtr->SnmpMsgBufSeekPos=0x00;
		if (!SNMPv3CheckIfValidV3StructAnd4ByteDataLen(Snmpv3StackDcptStubPtr->InPduWholeMsgBuf.snmpMsgHead,&tempPos,(uint16_t*)&tempLen.w[0]))
			return SNMP_NO_CREATION;
		
		//Check and collect "msgID"
		if(! SNMPv3CheckIfValidIntDataType(Snmpv3StackDcptStubPtr->InPduWholeMsgBuf.snmpMsgHead,&tempPos,&tempLen.Val))
			return SNMP_NO_CREATION;	

		Snmpv3StackDcptStubPtr->IncmngSnmpPduMsgID.Val= tempLen.Val;

		//Check and collect "msgMaxSize" 
		if(! SNMPv3CheckIfValidIntDataType(Snmpv3StackDcptStubPtr->InPduWholeMsgBuf.snmpMsgHead,&tempPos,&tempLen.Val))
			return SNMP_NO_CREATION;	
		
		Snmpv3StackDcptStubPtr->incomingPdu.maxSizeResponseScopedPDU.Val = tempLen.Val;

		if(SNMPv3NegotiateEngnMaxMsgSize(tempLen.Val)==false)
			return SNMP_NO_CREATION; 	

		//Check and collect "msgFlags" 
		tempData = SNMPv3GetWholeMsgBufferData(Snmpv3StackDcptStubPtr->InPduWholeMsgBuf.snmpMsgHead,&tempPos);
		
		if ( !IS_OCTET_STRING(tempData) )
			return false;

		tempData=SNMPv3GetWholeMsgBufferData(Snmpv3StackDcptStubPtr->InPduWholeMsgBuf.snmpMsgHead,&tempPos);//Length byte of "msgFlags"
		tempData=SNMPv3GetWholeMsgBufferData(Snmpv3StackDcptStubPtr->InPduWholeMsgBuf.snmpMsgHead,&tempPos);//"msgFlag"

		Snmpv3StackDcptStubPtr->SnmpSecurityLevel=tempData;
		Snmpv3StackDcptStubPtr->incomingPdu.securityLevel=Snmpv3StackDcptStubPtr->SnmpSecurityLevel;
		Snmpv3StackDcptStubPtr->SecurtyPrimtvesOfIncmngPdu.securityLevel=Snmpv3StackDcptStubPtr->SnmpSecurityLevel;

		//Check and collect "msgSecurityModel"	
		if(! SNMPv3CheckIfValidIntDataType(Snmpv3StackDcptStubPtr->InPduWholeMsgBuf.snmpMsgHead,&tempPos,(uint32_t*)&tempLen.Val))
			return SNMP_NO_CREATION;	

		if(SNMPv3ConfigEngnSecrtyModel(tempLen.Val))
		{
			Snmpv3StackDcptStubPtr->incomingPdu.securityModel= tempLen.Val;
			Snmpv3StackDcptStubPtr->SecurtyPrimtvesOfIncmngPdu.securityModel= (uint8_t)tempLen.Val;
		}
		else
			return SNMP_NO_CREATION;

		Snmpv3StackDcptStubPtr->SnmpMsgBufSeekPos=tempPos;	
	}
	else if(inOutPdu == SNMP_RESPONSE_PDU)
	{
		bool retBuf=true;

		snmpv3Headerlength = MSGGLOBAL_HEADER_LEN(snmpv3MsgGlobalHeaderlength)+
							  MSG_AUTHORITATIVE_HEADER_LEN(snmpv3MsgAuthHedaerLength);

		ptr = Snmpv3StackDcptStubPtr->PduHeaderBuf.head = (uint8_t *)(TCPIP_HEAP_Calloc(snmpv3PktProcessingMemPntr.snmpHeapMemHandler,1,(size_t)snmpv3Headerlength+5));
		if(ptr == NULL)
			return SNMP_NO_CREATION;
		
		Snmpv3StackDcptStubPtr->PduHeaderBuf.length = 0;
		Snmpv3StackDcptStubPtr->PduHeaderBuf.maxlength = snmpv3Headerlength+1;

		//message header
		SNMPv3CopyDataToProcessBuff(STRUCTURE,&Snmpv3StackDcptStubPtr->PduHeaderBuf);
		SNMPv3CopyDataToProcessBuff(MSGGLOBAL_HEADER_LEN(snmpv3MsgGlobalHeaderlength)-2,&Snmpv3StackDcptStubPtr->PduHeaderBuf);

		//Put "msgID" type ASN_INT of length 4 bytes	
		SNMPv3CopyDataToProcessBuff(ASN_INT,&Snmpv3StackDcptStubPtr->PduHeaderBuf);		
		SNMPv3CopyDataToProcessBuff(0x04,&Snmpv3StackDcptStubPtr->PduHeaderBuf);
		SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->IncmngSnmpPduMsgID.v[3],&Snmpv3StackDcptStubPtr->PduHeaderBuf);
		SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->IncmngSnmpPduMsgID.v[2],&Snmpv3StackDcptStubPtr->PduHeaderBuf);
		SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->IncmngSnmpPduMsgID.v[1],&Snmpv3StackDcptStubPtr->PduHeaderBuf);
		SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->IncmngSnmpPduMsgID.v[0],&Snmpv3StackDcptStubPtr->PduHeaderBuf);

		//Put "msgMaxSize"  type ASN_INT of length 4 bytes
		SNMPv3CopyDataToProcessBuff(ASN_INT,&Snmpv3StackDcptStubPtr->PduHeaderBuf);
		SNMPv3CopyDataToProcessBuff(0x04,&Snmpv3StackDcptStubPtr->PduHeaderBuf);
		SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->SnmpEngnMaxMsgSize.v[3],&Snmpv3StackDcptStubPtr->PduHeaderBuf);
		SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->SnmpEngnMaxMsgSize.v[2],&Snmpv3StackDcptStubPtr->PduHeaderBuf);
		SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->SnmpEngnMaxMsgSize.v[1],&Snmpv3StackDcptStubPtr->PduHeaderBuf);
		SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->SnmpEngnMaxMsgSize.v[0],&Snmpv3StackDcptStubPtr->PduHeaderBuf);

		//Put "msgFlags"  type octet_string 
		SNMPv3CopyDataToProcessBuff(OCTET_STRING,&Snmpv3StackDcptStubPtr->PduHeaderBuf);
		SNMPv3CopyDataToProcessBuff(0x01,&Snmpv3StackDcptStubPtr->PduHeaderBuf);

		if((Snmpv3StackDcptStubPtr->SnmpSecurityLevel & 0x03)==0x03) // Rxed pkt is authenticated and encrypted
		{
			Snmpv3StackDcptStubPtr->SnmpRespnsSecrtyFlg=0x03;
			SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->SnmpRespnsSecrtyFlg,&Snmpv3StackDcptStubPtr->PduHeaderBuf);//Rsponse "msgFlags" value as Reportable, Encrypted and Authenticated Bits are not set. 
		}
		else if ((Snmpv3StackDcptStubPtr->SnmpSecurityLevel & 0x01)==0x01) // Rxed pkt is  authenticated and no priv
		{
			Snmpv3StackDcptStubPtr->SnmpRespnsSecrtyFlg=0x01;
			SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->SnmpRespnsSecrtyFlg,&Snmpv3StackDcptStubPtr->PduHeaderBuf);//Rsponse "msgFlags" value as Reportable, Encrypted and Authenticated Bits are not set. 
		}
		else
		{			
			Snmpv3StackDcptStubPtr->SnmpRespnsSecrtyFlg=0x00;
			SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->SnmpRespnsSecrtyFlg,&Snmpv3StackDcptStubPtr->PduHeaderBuf);
		}
		//Put "msgSecurityModel"	
		SNMPv3CopyDataToProcessBuff(ASN_INT,&Snmpv3StackDcptStubPtr->PduHeaderBuf);
		SNMPv3CopyDataToProcessBuff(0x01,&Snmpv3StackDcptStubPtr->PduHeaderBuf);
		retBuf = SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->SnmpEngnSecurityModel,&Snmpv3StackDcptStubPtr->PduHeaderBuf);
		if(retBuf != true)
			return SNMP_NO_CREATION;
	}
	return SNMP_NO_ERR;
}



/****************************************************************************
  Function: 
  	SNMP_ERR_STATUS SNMPv3UserSecurityModelProcessPDU(uint8_t inOutPdu)

  Summary:
  	This routine collects or populates the security model parametrs infomation 
  	from the received SNMPv3 request PDU or to the response PDU respectively.
  
  Description:
	The recievd SNMPv3 PDU or the transmit PDU header has message security
	data bytes infomration. This routine retrievs the messgae security parameters
	infomration from the stored incoming pdu or write the appropriate security
	model info to the repsonse msg buffer.
		  	 		 		  	
  Precondition:
	Valid SNMPv3 request msg is received.
   	
  Parameters:
  	inOutPdu: indicates whether the incomig PDU is to be read for user security
  	model to be retrieved or the response PDU to be populated with these values 
  	
  Return Values:
	SNMP_NO_CREATION: Failure due to improper security model processing information 
					     format in the received PDU or failure in constructing the response PDU.
	SNMP_NO_ERR: The user security model retrieval or response PDU fomration is successful
				   
  Remarks:
  	The user security parameter constitute the vital information for the message  
	authentication and privacy of the message.
  	The user security model parameters header structure
	MsgAuthEngnID+MsgAuthEngnBoots+MsgAuthEngnTime
	+MsgUserName+MsgAuthParam+MsgPrivParam
***************************************************************************/
SNMP_ERR_STATUS SNMPv3UserSecurityModelProcessPDU(INOUT_SNMP_PDU inOutPdu)
{
	TCPIP_UINT32_VAL tempLen={0};
	uint8_t* ptr=NULL;	
	uint8_t engnIdCntr=0;	
	uint8_t tempData=0,putCntr=0;
	uint16_t tempPos;

	SNMPV3_PROCESSING_MEM_INFO_PTRS snmpv3PktProcessingMemPntr; 
	SNMPv3GetPktProcessingDynMemStubPtrs(&snmpv3PktProcessingMemPntr);
	

	tempPos=Snmpv3StackDcptStubPtr->SnmpMsgBufSeekPos;

	if(inOutPdu == SNMP_REQUEST_PDU)
	{
	
		tempData = SNMPv3GetWholeMsgBufferData(Snmpv3StackDcptStubPtr->InPduWholeMsgBuf.snmpMsgHead,&tempPos);
		 if ( !IS_OCTET_STRING(tempData) )
		    return SNMP_NO_CREATION;
		 
		//Msg security Parameter length
		tempLen.Val=SNMPv3GetWholeMsgBufferData(Snmpv3StackDcptStubPtr->InPduWholeMsgBuf.snmpMsgHead,&tempPos);	
			
		 //Start collecting the security parameters from the incoming PDU.
		 //Check if the security parametrs are binded in ASN structure format 
		if (!SNMPv3CheckIfValidV3StructAnd4ByteDataLen(Snmpv3StackDcptStubPtr->InPduWholeMsgBuf.snmpMsgHead,&tempPos,(uint16_t*)&tempLen.w[0]))
			return SNMP_NO_CREATION;

		 //Collect "msgAuthoritiveEngineID" 
		tempData=SNMPv3GetWholeMsgBufferData(Snmpv3StackDcptStubPtr->InPduWholeMsgBuf.snmpMsgHead,&tempPos);	
		 if ( !IS_OCTET_STRING(tempData) )
		    return false;
		 
		tempData=SNMPv3GetWholeMsgBufferData(Snmpv3StackDcptStubPtr->InPduWholeMsgBuf.snmpMsgHead,&tempPos);	

		Snmpv3StackDcptStubPtr->SecurtyPrimtvesOfIncmngPdu.securityEngineIDLen=tempData;
		if(tempData == 0x00)
		{
			tempData=0;
		}
		else
		{
			Snmpv3StackDcptStubPtr->SecurtyPrimtvesOfIncmngPdu.securityEngineID=ptr=
				(uint8_t *)TCPIP_HEAP_Calloc(snmpv3PktProcessingMemPntr.snmpHeapMemHandler,1,(size_t)tempData+5);
			if(Snmpv3StackDcptStubPtr->SecurtyPrimtvesOfIncmngPdu.securityEngineID == NULL)
				return SNMP_NO_CREATION;
			while( tempData--)
			{
		       *ptr++ = SNMPv3GetWholeMsgBufferData(Snmpv3StackDcptStubPtr->InPduWholeMsgBuf.snmpMsgHead,&tempPos);	
			}	
		}

		//Check and collect "msgAuthoritiveEngineBoots"	
		if(! SNMPv3CheckIfValidIntDataType(Snmpv3StackDcptStubPtr->InPduWholeMsgBuf.snmpMsgHead,&tempPos,&tempLen.Val))
		return SNMP_NO_CREATION; 

		Snmpv3StackDcptStubPtr->AuthoritativeSnmpEngineBoots.Val= tempLen.Val;

		//Check and collect "msgAuthoritiveEngineTime"	
			 if(! SNMPv3CheckIfValidIntDataType(Snmpv3StackDcptStubPtr->InPduWholeMsgBuf.snmpMsgHead,&tempPos,&tempLen.Val))
			return SNMP_NO_CREATION;	
		 
		Snmpv3StackDcptStubPtr->AuthoritativeSnmpEngnTime.Val= tempLen.Val;


		//Collect "msgUserName"	
		tempData = SNMPv3GetWholeMsgBufferData(Snmpv3StackDcptStubPtr->InPduWholeMsgBuf.snmpMsgHead,&tempPos);	
		 if ( !IS_OCTET_STRING(tempData) )
		    return false;
		 
		tempData = SNMPv3GetWholeMsgBufferData(Snmpv3StackDcptStubPtr->InPduWholeMsgBuf.snmpMsgHead,&tempPos);	
		Snmpv3StackDcptStubPtr->SecurtyPrimtvesOfIncmngPdu.securityNameLength=tempData;

		if(tempData == 0x00)
		{
			tempData=0;
		}
		else
		{
			Snmpv3StackDcptStubPtr->SecurtyPrimtvesOfIncmngPdu.securityName=ptr=
				(uint8_t *)TCPIP_HEAP_Calloc(snmpv3PktProcessingMemPntr.snmpHeapMemHandler,1,(size_t)tempData+5);
			if(Snmpv3StackDcptStubPtr->SecurtyPrimtvesOfIncmngPdu.securityName == NULL)
				return SNMP_NO_CREATION;
			while( tempData--)
			{
			   *ptr++ = SNMPv3GetWholeMsgBufferData(Snmpv3StackDcptStubPtr->InPduWholeMsgBuf.snmpMsgHead,&tempPos);	
			}	
		
		}


		//validate user security name with security level
		if(!SNMPv3ValidateSecNameAndSecLevel())
			return SNMP_NO_CREATION;
		
		//Validate if the "msgAuthoritiveEngineID" matches to this agent's SNMP Engine ID
		if(!SNMPv3ValidateSnmpEngnId())
			return SNMP_NO_CREATION; 

		//Check and collect "msgAuthenticationParameters"	
		tempData = SNMPv3GetWholeMsgBufferData(Snmpv3StackDcptStubPtr->InPduWholeMsgBuf.snmpMsgHead,&tempPos);	
		 if ( !IS_OCTET_STRING(tempData) )
		    return false;

		 //(SnmpInMsgAuthParamLen should be 12 bytes if using HAMC-MD5-96 or HMAC-SHA-96)
		Snmpv3StackDcptStubPtr->SnmpInMsgAuthParamLen=tempData= SNMPv3GetWholeMsgBufferData(Snmpv3StackDcptStubPtr->InPduWholeMsgBuf.snmpMsgHead,&tempPos);	

		if((Snmpv3StackDcptStubPtr->SnmpSecurityLevel&0x01)==0x01)//If message is authenticated
		if(Snmpv3StackDcptStubPtr->SnmpInMsgAuthParamLen !=12 /* if using HAMC-MD5-96 or HMAC-SHA-96 */)
			return SNMP_NO_CREATION;
		
		if(tempData != 0x00)
		{
			ptr=Snmpv3StackDcptStubPtr->SnmpInMsgAuthParmStrng;
			
			Snmpv3StackDcptStubPtr->InPduWholeMsgBuf.msgAuthParamOffsetInWholeMsg=tempPos;//From snmpMsgHead;
			while( tempData--)
			{
			   *ptr++= SNMPv3GetWholeMsgBufferData(Snmpv3StackDcptStubPtr->InPduWholeMsgBuf.snmpMsgHead,&tempPos);	
			   
			}	
		}

		
		//Check and collect "msgPrivacyParameters"	
		tempData = SNMPv3GetWholeMsgBufferData(Snmpv3StackDcptStubPtr->InPduWholeMsgBuf.snmpMsgHead,&tempPos);	
		 if ( !IS_OCTET_STRING(tempData) )
		    return false;

		//(SnmpInMsgPrivParmLen should be 8 bytes) 
		Snmpv3StackDcptStubPtr->SnmpInMsgPrivParmLen=tempData =SNMPv3GetWholeMsgBufferData(Snmpv3StackDcptStubPtr->InPduWholeMsgBuf.snmpMsgHead,&tempPos);	
		if((Snmpv3StackDcptStubPtr->SnmpSecurityLevel&0x02)==0x02)//If message is encrypted
		if(Snmpv3StackDcptStubPtr->SnmpInMsgPrivParmLen !=8)
			return SNMP_NO_CREATION;
		
		if(tempData != 0x00)
		{
			ptr=Snmpv3StackDcptStubPtr->snmpInMsgPrvParamStrng;
			
			while( tempData--)
			{
			   *ptr++ = SNMPv3GetWholeMsgBufferData(Snmpv3StackDcptStubPtr->InPduWholeMsgBuf.snmpMsgHead,&tempPos);	
			}	


			//This is a secured request. Compute the AES decryption IV 
			SNMPv3UsmAesEncryptDecrptInitVector(SNMP_REQUEST_PDU);
		}

		/* global variable to find out how many times SNMPv3 engine id has been validated*/
		Snmpv3StackDcptStubPtr->UsmStatsEngineID.Val++;
		Snmpv3StackDcptStubPtr->SnmpMsgBufSeekPos=tempPos;
		
	}
	else if(inOutPdu == SNMP_RESPONSE_PDU)
	{
		uint16_t snmpv3MsgAuthHeaderLength=0;
		bool   retBuf=true;
		uint16_t msgHeaderOffset1=0;
		uint16_t msgHeaderOffset2=0;
		uint16_t tempMsgHeaderOffset=0;
		
		SNMPv3CopyDataToProcessBuff(OCTET_STRING,&Snmpv3StackDcptStubPtr->PduHeaderBuf);  //Security Parameter string 	
		msgHeaderOffset1 = Snmpv3StackDcptStubPtr->PduHeaderBuf.length;
		SNMPv3CopyDataToProcessBuff(MSG_AUTHORITATIVE_HEADER_LEN(snmpv3MsgAuthHeaderLength)-2,&Snmpv3StackDcptStubPtr->PduHeaderBuf);
		SNMPv3CopyDataToProcessBuff(STRUCTURE,&Snmpv3StackDcptStubPtr->PduHeaderBuf);
		msgHeaderOffset2 = Snmpv3StackDcptStubPtr->PduHeaderBuf.length;
		SNMPv3CopyDataToProcessBuff(MSG_AUTHORITATIVE_HEADER_LEN(snmpv3MsgAuthHeaderLength)-4,&Snmpv3StackDcptStubPtr->PduHeaderBuf);
		
		//Put "msgAuthoritiveEngineID"	
		SNMPv3CopyDataToProcessBuff(OCTET_STRING,&Snmpv3StackDcptStubPtr->PduHeaderBuf);	
		SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->SnmpEngnIDLength,&Snmpv3StackDcptStubPtr->PduHeaderBuf); //Integer Length
		tempData=Snmpv3StackDcptStubPtr->SnmpEngnIDLength;
		for(;engnIdCntr<tempData;engnIdCntr++)
		{
			SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->SnmpEngineID[engnIdCntr],&Snmpv3StackDcptStubPtr->PduHeaderBuf);
		}
		
		//Put "msgAuthoritiveEngineBoots" 
		SNMPv3CopyDataToProcessBuff(ASN_INT,&Snmpv3StackDcptStubPtr->PduHeaderBuf);	
		SNMPv3CopyDataToProcessBuff(0x04,&Snmpv3StackDcptStubPtr->PduHeaderBuf);
		SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->SnmpEngineBoots>>24,&Snmpv3StackDcptStubPtr->PduHeaderBuf);
		SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->SnmpEngineBoots>>16,&Snmpv3StackDcptStubPtr->PduHeaderBuf);
		SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->SnmpEngineBoots>>8,&Snmpv3StackDcptStubPtr->PduHeaderBuf);
		SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->SnmpEngineBoots,&Snmpv3StackDcptStubPtr->PduHeaderBuf);

		//Put "msgAuthoritiveEngineTime" 
		SNMPv3GetAuthEngineTime();
		SNMPv3CopyDataToProcessBuff(ASN_INT,&Snmpv3StackDcptStubPtr->PduHeaderBuf);	
		SNMPv3CopyDataToProcessBuff(0x04,&Snmpv3StackDcptStubPtr->PduHeaderBuf); 
		SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->SnmpEngineTime.v[3],&Snmpv3StackDcptStubPtr->PduHeaderBuf);
		SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->SnmpEngineTime.v[2],&Snmpv3StackDcptStubPtr->PduHeaderBuf);
		SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->SnmpEngineTime.v[1],&Snmpv3StackDcptStubPtr->PduHeaderBuf);
		SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->SnmpEngineTime.v[0],&Snmpv3StackDcptStubPtr->PduHeaderBuf);


		//Put "msgUserName"	
		SNMPv3CopyDataToProcessBuff(OCTET_STRING,&Snmpv3StackDcptStubPtr->PduHeaderBuf);	
		SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->SecurtyPrimtvesOfIncmngPdu.securityNameLength,&Snmpv3StackDcptStubPtr->PduHeaderBuf);
		tempData=Snmpv3StackDcptStubPtr->SecurtyPrimtvesOfIncmngPdu.securityNameLength ;
		if(Snmpv3StackDcptStubPtr->SecurtyPrimtvesOfIncmngPdu.securityNameLength != 0)
		{
			ptr= Snmpv3StackDcptStubPtr->SecurtyPrimtvesOfIncmngPdu.securityName;
			for(putCntr=0;putCntr<tempData;putCntr++)
			{
				//SNMPv3CopyDataToProcessBuff(ptr[putCntr],&Snmpv3StackDcptStubPtr->PduHeaderBuf);
				SNMPv3CopyDataToProcessBuff(*(ptr+putCntr),&Snmpv3StackDcptStubPtr->PduHeaderBuf);
			}
		}

		putCntr = 0;
		
		//SNMPv3UsmOutMsgAuthParam(SNMPV3_HAMC_MD5/*Hash Accoding to user*/);
		//Put "msgAuthenticationParameters"	
		SNMPv3CopyDataToProcessBuff(OCTET_STRING,&Snmpv3StackDcptStubPtr->PduHeaderBuf);
		if((Snmpv3StackDcptStubPtr->SnmpSecurityLevel & 0x01) == 0x01)
		{
			SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->SnmpOutMsgAuthParmLen,&Snmpv3StackDcptStubPtr->PduHeaderBuf); //Not supported with the Alpha Release.

			Snmpv3StackDcptStubPtr->PduHeaderBuf.msgAuthParamOffset=Snmpv3StackDcptStubPtr->PduHeaderBuf.length;

			//Put 0x00 to msgAuthenticationParameters, Once the response WholeMsg is 
			//populated, this offset can be updated with the new msgAuthenticationParam
			for(;putCntr<Snmpv3StackDcptStubPtr->SnmpOutMsgAuthParmLen;putCntr++)
			SNMPv3CopyDataToProcessBuff(0x00,&Snmpv3StackDcptStubPtr->PduHeaderBuf);//RFC3414 Section 6.3.2 Page#56 Step3
		}
		else
		{
			SNMPv3CopyDataToProcessBuff(0x00,&Snmpv3StackDcptStubPtr->PduHeaderBuf); //Not supported with the Alpha Release.
		}
		putCntr = 0;
		SNMPv3USMOutMsgPrivParam();
		
		//Put "msgPrivacyParameters" 
		SNMPv3CopyDataToProcessBuff(OCTET_STRING,&Snmpv3StackDcptStubPtr->PduHeaderBuf); 
		if((Snmpv3StackDcptStubPtr->SnmpSecurityLevel & 0x02) == 0x02)
		{
			SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->SnmpOutMsgPrivParmLen,&Snmpv3StackDcptStubPtr->PduHeaderBuf); //Not supported with the Alpha Release
			for(;putCntr<Snmpv3StackDcptStubPtr->SnmpOutMsgPrivParmLen;putCntr++)
				retBuf = SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->SnmpOutMsgPrvParmStrng[putCntr],&Snmpv3StackDcptStubPtr->PduHeaderBuf);
		}
		else
		{
			SNMPv3CopyDataToProcessBuff(0x00,&Snmpv3StackDcptStubPtr->PduHeaderBuf); //Not supported with the Alpha Release.
		}
		if(retBuf != true)
			return SNMP_NO_CREATION;
		tempMsgHeaderOffset = Snmpv3StackDcptStubPtr->PduHeaderBuf.length;
		Snmpv3StackDcptStubPtr->PduHeaderBuf.length = msgHeaderOffset2;
		SNMPv3CopyDataToProcessBuff((tempMsgHeaderOffset-msgHeaderOffset2)-1,&Snmpv3StackDcptStubPtr->PduHeaderBuf); 
		Snmpv3StackDcptStubPtr->PduHeaderBuf.length = tempMsgHeaderOffset;

		tempMsgHeaderOffset = Snmpv3StackDcptStubPtr->PduHeaderBuf.length;
		Snmpv3StackDcptStubPtr->PduHeaderBuf.length = msgHeaderOffset1;
		SNMPv3CopyDataToProcessBuff((tempMsgHeaderOffset-msgHeaderOffset1)-1,&Snmpv3StackDcptStubPtr->PduHeaderBuf);
		Snmpv3StackDcptStubPtr->PduHeaderBuf.length = tempMsgHeaderOffset;
	}
	return SNMP_NO_ERR;
}


/****************************************************************************
  Function: 
  	SNMP_ERR_STATUS SNMPv3ScopedPduProcessing(uint8_t inOutPdu)

  Summary:
  	This routine collects  the scoped pdu header information from the 
  	received SNMPv3 request PDU or populates to the response PDU respectively.
  
  Description:
	The recievd SNMPv3 PDU or the transmit PDU header has scoped pdu parameters
	like 'contextEngineID' 'context name' etc. This routine retrievs these parameters
	infomration from the stored incoming pdu or write the appropriate dynamically 
	allocated memory for the transmit response PDU.
		  	 		 		  	
  Precondition:
	Valid SNMPv3 request msg is received.
   	
  Parameters:
  	inOutPdu: indicates whether the incomig PDU is to be read for scoped pdu
  	paraemters to be retrieved or the response PDU to be populated with these values 
  	
  Return Values:
	SNMP_NO_CREATION: Failure due to improper scoped pdu information format in the  
					     PDU or failure in constructing the response PDU.
	SNMP_NO_ERR: The scoped parameters retrieval or response PDU fomration 
				   is successful
				   
  Remarks:
  	The scoped pDu parameters 
	msg data : - <contextEngineID><context name>[<data> == <pdutype><request id>
	<error status><error index><varbinds>
***************************************************************************/
SNMP_ERR_STATUS SNMPv3ScopedPduProcessing(INOUT_SNMP_PDU inOutPdu)
{
	TCPIP_UINT32_VAL tempLen={0};
	uint8_t* ptr=NULL;
	uint16_t msgDataLen=0;
	SNMPV3MSGDATA scopedPtr={NULL,0,0,0};

	uint16_t 	contextIDlen=0;
	uint16_t	contextNameLength=0;
	uint16_t	snmpv3Headerlength=0,snmpv3MsgGlobalHeaderlength=0;
	uint16_t	snmpv3MsgAuthHedaerLength=0;

	SNMPV3_PROCESSING_MEM_INFO_PTRS snmpv3PktProcessingMemPntr; 
	
	SNMPv3GetPktProcessingDynMemStubPtrs(&snmpv3PktProcessingMemPntr);


	if(inOutPdu == SNMP_REQUEST_PDU)
	{
		if ( !SNMPv3CheckIfValidAuthStructure((uint16_t*)&tempLen) )
		{
			return SNMP_NO_CREATION;
		}

		Snmpv3StackDcptStubPtr->InPduWholeMsgBuf.scopedPduOffset=Snmpv3StackDcptStubPtr->SnmpMsgBufSeekPos;
		

		Snmpv3StackDcptStubPtr->ScopedPduRequstBuf.head=Snmpv3StackDcptStubPtr->InPduWholeMsgBuf.snmpMsgHead+Snmpv3StackDcptStubPtr->SnmpMsgBufSeekPos;
		Snmpv3StackDcptStubPtr->ScopedPduRequstBuf.length = 0;
		Snmpv3StackDcptStubPtr->ScopedPduRequstBuf.maxlength = tempLen.Val+1;
	}
	else if(inOutPdu == SNMP_RESPONSE_PDU)
	{		
			
		snmpv3Headerlength = MSGGLOBAL_HEADER_LEN(snmpv3MsgGlobalHeaderlength)+
									  MSG_AUTHORITATIVE_HEADER_LEN(snmpv3MsgAuthHedaerLength);
		
		msgDataLen = SNMP_MAX_MSG_SIZE - snmpv3Headerlength;
		ptr = Snmpv3StackDcptStubPtr->ScopedPduRespnsBuf.head =
			(uint8_t*)(TCPIP_HEAP_Calloc(snmpv3PktProcessingMemPntr.snmpHeapMemHandler,1,(size_t)msgDataLen+5));
		if(ptr == NULL)
		{
			return SNMP_NO_CREATION;
		}
		Snmpv3StackDcptStubPtr->ScopedPduRespnsBuf.length = 0;
		Snmpv3StackDcptStubPtr->ScopedPduRespnsBuf.maxlength = msgDataLen;
		Snmpv3StackDcptStubPtr->ScopedPduDataPos = 0;


    	//Start collecting the plaint text Scoped PDU data byte from the WholeMsg buffer
		//Check if the plain text scoped pdu data bytes are binded in ASN structure format 
		scopedPtr = Snmpv3StackDcptStubPtr->ScopedPduRespnsBuf;
		SNMPv3CopyDataToProcessBuff(STRUCTURE,&scopedPtr); // First item to Response buffer is packet structure
		SNMPv3CopyDataToProcessBuff(0x82,&scopedPtr); 
		SNMPv3CopyDataToProcessBuff(0,&scopedPtr);
		SNMPv3CopyDataToProcessBuff(0,&scopedPtr);

		if((Snmpv3StackDcptStubPtr->SnmpSecurityLevel & 0x02)==0x02)
		{
			contextIDlen=SNMPv3GetProcessBuffData(Snmpv3StackDcptStubPtr->ScopedPduRequstBuf,Snmpv3StackDcptStubPtr->ScopedPduDataPos++) ;
			contextIDlen=SNMPv3GetProcessBuffData(Snmpv3StackDcptStubPtr->ScopedPduRequstBuf,Snmpv3StackDcptStubPtr->ScopedPduDataPos++) ;
			if(contextIDlen == 0x81)
			{
				SNMPv3GetProcessBuffData(Snmpv3StackDcptStubPtr->ScopedPduRequstBuf,Snmpv3StackDcptStubPtr->ScopedPduDataPos++) ;
			}
			else if(contextIDlen == 0x82)
			{
				SNMPv3GetProcessBuffData(Snmpv3StackDcptStubPtr->ScopedPduRequstBuf,Snmpv3StackDcptStubPtr->ScopedPduDataPos++) ;
				SNMPv3GetProcessBuffData(Snmpv3StackDcptStubPtr->ScopedPduRequstBuf,Snmpv3StackDcptStubPtr->ScopedPduDataPos++) ;
			}
		}
		
		//Collect context engine id
		if (!IS_OCTET_STRING(SNMPv3GetProcessBuffData(Snmpv3StackDcptStubPtr->ScopedPduRequstBuf,Snmpv3StackDcptStubPtr->ScopedPduDataPos) ))
		{
			return SNMP_NO_CREATION;
		}
		SNMPv3CopyDataToProcessBuff(OCTET_STRING,&scopedPtr);

		contextIDlen = SNMPv3GetProcessBuffData(Snmpv3StackDcptStubPtr->ScopedPduRequstBuf,++Snmpv3StackDcptStubPtr->ScopedPduDataPos);
		if(contextIDlen == 0)
		{			
			SNMPv3CopyDataToProcessBuff(0,&scopedPtr);
		}
		else
		{
			//copy context engine id from a local buffer			
			SNMPv3CopyDataToProcessBuff(contextIDlen,&scopedPtr);
			while(contextIDlen!=0)	
			{
				SNMPv3CopyDataToProcessBuff(SNMPv3GetProcessBuffData(Snmpv3StackDcptStubPtr->ScopedPduRequstBuf,++Snmpv3StackDcptStubPtr->ScopedPduDataPos),&scopedPtr);
				contextIDlen-=1;
			}
		}

		//Check and collect "contextName" 
		 if (!IS_OCTET_STRING(SNMPv3GetProcessBuffData(Snmpv3StackDcptStubPtr->ScopedPduRequstBuf,++Snmpv3StackDcptStubPtr->ScopedPduDataPos) ))
		{
		   return SNMP_NO_CREATION;
		}
		SNMPv3CopyDataToProcessBuff(OCTET_STRING,&scopedPtr);
		contextNameLength = (uint16_t)SNMPv3GetProcessBuffData(Snmpv3StackDcptStubPtr->ScopedPduRequstBuf,++Snmpv3StackDcptStubPtr->ScopedPduDataPos);
		if(contextNameLength == 0x00)
		{
			SNMPv3CopyDataToProcessBuff(0x00,&scopedPtr);
		}
		else
		{
			SNMPv3CopyDataToProcessBuff(contextNameLength,&scopedPtr);
			while(contextNameLength!=0x00)
			{
				SNMPv3CopyDataToProcessBuff(SNMPv3GetProcessBuffData(Snmpv3StackDcptStubPtr->ScopedPduRequstBuf,++Snmpv3StackDcptStubPtr->ScopedPduDataPos),&scopedPtr);
				contextNameLength-=1;
			}	
		}

		Snmpv3StackDcptStubPtr->ScopedPduRespnsBuf.length = scopedPtr.length;

	}
	
	return SNMP_NO_ERR;
}



/****************************************************************************
  Function:
	void SNMPv3FreeDynAllocMem(void)
	
  Summary:
  	Allocated dynamic memory freeing is done by this routine.
	
  Description:
  	On the successful completion of the processing of the SNMPv3 request, or the 
  	failure in the processing due to improper PDU formats, the allocated dynamic
  	memory is required to be freed. This routine calls the free(), to deallocate memory.
	
  Precondition:
	The dyanmic memory buffer is allocated.
	
  Parameters:
  	None
  	
  Return Values:
	None
	
  Remarks:
	The SNMPv3 stack does uses the dynamic memory extensively for different 
       processing needs, hence incoming and outgoing pdu memory buffers are created.
	This routine checks for the memory is being allocated before it attempts for the deallocation. 
 ***************************************************************************/
void SNMPv3FreeDynAllocMem(void)
{

//	char tempMsg[15];

	SNMPV3_PROCESSING_MEM_INFO_PTRS snmpv3PktProcessingMemPntr; 
	SNMPv3GetPktProcessingDynMemStubPtrs(&snmpv3PktProcessingMemPntr);


	if(Snmpv3StackDcptStubPtr->PduHeaderBuf.head != NULL)
	{
		TCPIP_HEAP_Free(snmpv3PktProcessingMemPntr.snmpHeapMemHandler, Snmpv3StackDcptStubPtr->PduHeaderBuf.head);
		Snmpv3StackDcptStubPtr->PduHeaderBuf.head=NULL;
		Snmpv3StackDcptStubPtr->PduHeaderBuf.length=0x00;
	}

	if(Snmpv3StackDcptStubPtr->ScopedPduRespnsBuf.head != NULL)
	{
		TCPIP_HEAP_Free(snmpv3PktProcessingMemPntr.snmpHeapMemHandler, Snmpv3StackDcptStubPtr->ScopedPduRespnsBuf.head);
		Snmpv3StackDcptStubPtr->ScopedPduRespnsBuf.head=NULL;
		Snmpv3StackDcptStubPtr->ScopedPduRespnsBuf.length = 0;
	}

	if(Snmpv3StackDcptStubPtr->InPduWholeMsgBuf.wholeMsgHead != NULL)
	{
		TCPIP_HEAP_Free(snmpv3PktProcessingMemPntr.snmpHeapMemHandler, Snmpv3StackDcptStubPtr->InPduWholeMsgBuf.wholeMsgHead);
		Snmpv3StackDcptStubPtr->InPduWholeMsgBuf.wholeMsgLen.Val = 0;
		Snmpv3StackDcptStubPtr->InPduWholeMsgBuf.wholeMsgHead=NULL;
		Snmpv3StackDcptStubPtr->InPduWholeMsgBuf.snmpMsgHead=NULL;
	}

	if(Snmpv3StackDcptStubPtr->SecurtyPrimtvesOfIncmngPdu.securityEngineID!=NULL)
	{
		TCPIP_HEAP_Free(snmpv3PktProcessingMemPntr.snmpHeapMemHandler, Snmpv3StackDcptStubPtr->SecurtyPrimtvesOfIncmngPdu.securityEngineID);
		Snmpv3StackDcptStubPtr->SecurtyPrimtvesOfIncmngPdu.securityEngineID=0x00;
		Snmpv3StackDcptStubPtr->SecurtyPrimtvesOfIncmngPdu.securityEngineIDLen=0x00;
	}

	if(Snmpv3StackDcptStubPtr->SecurtyPrimtvesOfIncmngPdu.securityName != NULL)
	{ 
		TCPIP_HEAP_Free(snmpv3PktProcessingMemPntr.snmpHeapMemHandler, Snmpv3StackDcptStubPtr->SecurtyPrimtvesOfIncmngPdu.securityName);
		Snmpv3StackDcptStubPtr->SecurtyPrimtvesOfIncmngPdu.securityName=NULL;
	}

	if(Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf.head != NULL)
	{
		TCPIP_HEAP_Free(snmpv3PktProcessingMemPntr.snmpHeapMemHandler, Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf.head);
		Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf.length=0;
        Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf.head = NULL;
	}

	if(Snmpv3StackDcptStubPtr->TrapScopdPduRespnsBuf.head != NULL)
	{
		TCPIP_HEAP_Free(snmpv3PktProcessingMemPntr.snmpHeapMemHandler, Snmpv3StackDcptStubPtr->TrapScopdPduRespnsBuf.head);
		Snmpv3StackDcptStubPtr->TrapScopdPduRespnsBuf.length = 0;
		Snmpv3StackDcptStubPtr->TrapScopdPduRespnsBuf.head = NULL;
	}
	
	if(Snmpv3StackDcptStubPtr->OUTPduWholeMsgBuf.wholeMsgHead != NULL)
	{
		TCPIP_HEAP_Free(snmpv3PktProcessingMemPntr.snmpHeapMemHandler,Snmpv3StackDcptStubPtr->OUTPduWholeMsgBuf.wholeMsgHead);
		Snmpv3StackDcptStubPtr->OUTPduWholeMsgBuf.wholeMsgHead=0x00;
		Snmpv3StackDcptStubPtr->OUTPduWholeMsgBuf.wholeMsgLen.Val=0x00;
		Snmpv3StackDcptStubPtr->OUTPduWholeMsgBuf.snmpMsgHead = NULL;
		Snmpv3StackDcptStubPtr->OUTPduWholeMsgBuf.msgAuthParamOffsetOutWholeMsg = NULL;
		Snmpv3StackDcptStubPtr->OUTPduWholeMsgBuf.scopedPduOffset =  NULL;			
	}
	
}

/****************************************************************************
  Function:
	bool SNMPv3ProcessV3MsgData(PDU_INFO* pduDbPtr)
	
  Summary:
  	This routine processes the snmpv3 request and parallely creates the response pdu.
	
  Description:
  	Once the received pdu is validated as Snmpv3 pdu, it is forwarded for 
  	processing to this routine. This rotuine handles Get, Get_Next, Get_Bulk,
  	Set request and creates appropriate response as Get_Response. 
  	This routine will decide on whether the request pdu should be processed
  	or be discarded. 
  	
  Precondition:
	The received udp packet is varified as valid SNMPv3 request.
		
  Parameters:
  	pduDbPtr  - Pointer to received pdu information database
  	
  Return Values:
	true 	- If the snmp request processing is successful.
	false	- If the processing failed else the processing is not completed.
	
  Remarks:
	None
 ***************************************************************************/
bool SNMPv3ProcessV3MsgData(PDU_INFO* pduDbPtr)
{
	uint8_t 			Getbulk_N=0,Getbulk_M=0,Getbulk_R=0;/*Refer RFC 3416 Section "4.2.3" GetBulkRequest-PDU*/
	uint8_t			OIDValue[OID_MAX_LEN];
	uint8_t			OIDlen=0;
	uint8_t			oidLookUpRet=0;
	uint8_t			noOfVarbindreq=0xFF;
	uint8_t tempBuf[4];
	uint8_t tempCntr=0;
	uint8_t* tempPtr=NULL;
	uint8_t* outBufPtr=NULL;
	uint8_t			varIndex=0;
	uint8_t 			repeatCntr=0,varBindCntr=0;
	uint8_t			succesor=0,tempRet=0xFF;
	uint16_t	 		pduLenOffset=0;
	uint16_t 	 		pduLength=0;
	uint16_t 			errorStatusOffset=0;
	uint16_t 			errorIndexOffset=0;
	uint16_t 			varBindHeaderLen=0;
	uint16_t			previousGetBufpos=0;
	uint16_t			varBindHeaderOffset=0;
	uint16_t			varBindHeaderOffset_2=0;
	uint16_t			varBindHeaderOffset_3=0;
	uint16_t			contextNameOffset=0;	
	uint16_t		 	varStructLenOffset=0;
	uint16_t			maxRepeatationOffset=0;
	uint16_t			tempOffset=0;						
	TCPIP_UINT16_VAL		varPairLen = {0x0};
	TCPIP_UINT32_VAL 		tempVal={0};
	static SNMPV3MSGDATA	*dynScopedBufPtr=NULL;
	OID_INFO		OIDInfo;  
	SNMP_ERR_STATUS errorStatus;
	uint8_t intfIdx;
    bool bSnmpv3GenError= false;
    bool bSnmpV3GetBulkResError = false;		
	enum 
	{
		SM_PKT_STRUCT_LEN_OFFSET=0u,
		SM_RESPONSE_PDU_LEN_OFFSET,
		SM_ERROR_STATUS_OFFSET,
		SM_ERROR_INDEX_OFFSET,
		SM_FIND_NO_OF_REQUESTED_VARBINDS,
		SM_FIND_NO_OF_RESPONSE_VARBINDS,
		SM_VARBIND_STRUCT_OFFSET,
		SM_VARSTRUCT_LEN_OFFSET,
		SM_POPULATE_REQ_OID,
		SM_FIND_OID_IN_MIB,
		SM_NON_REPETITIONS,
		SM_MAX_REPETITIONS
	}smSnmp=SM_PKT_STRUCT_LEN_OFFSET;

	
	SNMP_PROCESSING_MEM_INFO_PTRS snmpPktProcsMemPtrsInfo; 
	SNMP_STACK_DCPT_STUB * snmpStkDcptMemStubPtr=0;		
						
	SNMPGetPktProcessingDynMemStubPtrs(&snmpPktProcsMemPtrsInfo);
	snmpStkDcptMemStubPtr=snmpPktProcsMemPtrsInfo.snmpStkDynMemStubPtr;

	dynScopedBufPtr = &Snmpv3StackDcptStubPtr->ScopedPduRespnsBuf;
	intfIdx = ((TCPIP_NET_IF*)TCPIP_STACK_GetDefaultNet())->netIfIx;
	

	
	while(1)
	{
		switch(smSnmp)
		{
	
			// Before each variables are processed, prepare necessary header.
				case SM_PKT_STRUCT_LEN_OFFSET:
	
				varPairLen.Val=0x0000;
				
				if(SNMPv3MsgProcessingModelProcessPDU(SNMP_RESPONSE_PDU)!= SNMP_NO_ERR)
				{
					SNMPv3FreeDynAllocMem();
					return false;
				}
				if(SNMPv3UserSecurityModelProcessPDU(SNMP_RESPONSE_PDU)!= SNMP_NO_ERR)
				{
					SNMPv3FreeDynAllocMem();
					return false;
				}
				contextNameOffset = dynScopedBufPtr->length;
				if(SNMPv3ScopedPduProcessing(SNMP_RESPONSE_PDU)!= SNMP_NO_ERR)
				{
					SNMPv3FreeDynAllocMem();
					return false;
				}
				
				dynScopedBufPtr = &Snmpv3StackDcptStubPtr->ScopedPduRespnsBuf;					
				dynScopedBufPtr->head = Snmpv3StackDcptStubPtr->ScopedPduRespnsBuf.head;
				dynScopedBufPtr->length = Snmpv3StackDcptStubPtr->ScopedPduRespnsBuf.length;
				
				pduDbPtr->pduType = SNMPv3GetProcessBuffData(Snmpv3StackDcptStubPtr->ScopedPduRequstBuf,++Snmpv3StackDcptStubPtr->ScopedPduDataPos);
				pduLength = SNMPv3GetProcessBuffData(Snmpv3StackDcptStubPtr->ScopedPduRequstBuf,++Snmpv3StackDcptStubPtr->ScopedPduDataPos);
				if(pduLength == 0x81)
				{
					pduLength = SNMPv3GetProcessBuffData(Snmpv3StackDcptStubPtr->ScopedPduRequstBuf,++Snmpv3StackDcptStubPtr->ScopedPduDataPos);
				}
				else if(pduLength == 0x82)
				{
					pduLength = SNMPv3GetProcessBuffData(Snmpv3StackDcptStubPtr->ScopedPduRequstBuf,++Snmpv3StackDcptStubPtr->ScopedPduDataPos);
					pduLength = SNMPv3GetProcessBuffData(Snmpv3StackDcptStubPtr->ScopedPduRequstBuf,++Snmpv3StackDcptStubPtr->ScopedPduDataPos);
				}					
			
				varBindHeaderOffset = dynScopedBufPtr->length;
				previousGetBufpos = Snmpv3StackDcptStubPtr->ScopedPduDataPos;
				/*if(SNMPv3GetProcessBuffData(Snmpv3StackDcptStubPtr->ScopedPduRequstBuf,previousGetBufpos+0x0E)==0x0)
				{
					noOfVarbindreq = 0;
				}*/
				smSnmp++;
			case SM_RESPONSE_PDU_LEN_OFFSET:
				// The exact Reponse will be updated with varBindHeaderOffset
				// After reading no of varbinds from SNMPv3FindOIDsFrmIncmingV3Req
				if(!noOfVarbindreq)
					SNMPv3CopyDataToProcessBuff(REPORT_RESPONSE,dynScopedBufPtr);
				else
					SNMPv3CopyDataToProcessBuff(GET_RESPONSE,dynScopedBufPtr);			
				// Since we don't know length of this response, use placeholders until
				pduLenOffset = dynScopedBufPtr->length;
				SNMPv3CopyDataToProcessBuff(0x82,dynScopedBufPtr);				
				SNMPv3CopyDataToProcessBuff(0,dynScopedBufPtr);
				SNMPv3CopyDataToProcessBuff(0,dynScopedBufPtr);
				
				if(SNMPv3CheckIfValidInt(&tempVal.Val) != true)
				{					
					SNMPv3FreeDynAllocMem();
					return false;
				}
				else
				{
					// Put original request back.
					SNMPv3CopyDataToProcessBuff(ASN_INT,dynScopedBufPtr);	// Int type.
					SNMPv3CopyDataToProcessBuff(4,dynScopedBufPtr);		// To simplify logic, always use 4 byte long requestID
					SNMPv3CopyDataToProcessBuff(tempVal.v[3],dynScopedBufPtr); // Start MSB
					SNMPv3CopyDataToProcessBuff(tempVal.v[2],dynScopedBufPtr);
					SNMPv3CopyDataToProcessBuff(tempVal.v[1],dynScopedBufPtr);
					SNMPv3CopyDataToProcessBuff(tempVal.v[0],dynScopedBufPtr);
				}

				smSnmp++;
			case SM_ERROR_STATUS_OFFSET :
				/*update pduDbPtr structure for error index and eroor status 
				and non repeators and max repeators */
				if(pduDbPtr->pduType != GET_BULK_REQUEST)
				{
					// ignore error index and error status but update pduDBptr
					tempVal.Val = 0;
					if(! SNMPv3CheckIfValidInt(&tempVal.Val))
						return false;
					pduDbPtr->errorStatus = tempVal.Val;
					tempVal.Val = 0;
					if(! SNMPv3CheckIfValidInt(&tempVal.Val))
						return false;
					pduDbPtr->erroIndex = tempVal.Val;
				}
				else
				{
					// update max repeators and non repeators
					tempVal.Val = 0;
					if(SNMPv3CheckIfValidInt(&tempVal.Val) == true)
						pduDbPtr->nonRepeators = tempVal.Val;
					else
					{						
						SNMPv3FreeDynAllocMem();
						return false;
					}
					tempVal.Val = 0;
					if(SNMPv3CheckIfValidInt(&tempVal.Val) == true)
						pduDbPtr->maxRepetitions = tempVal.Val;
					else
					{						
						SNMPv3FreeDynAllocMem();
						return false;
					}
					
				}
				
				// Put error status.
				// Since we do not know error status, put place holder until we know it...
				SNMPv3CopyDataToProcessBuff(ASN_INT,dynScopedBufPtr);				// Int type
				SNMPv3CopyDataToProcessBuff(1,dynScopedBufPtr);					// One byte long.
				errorStatusOffset = dynScopedBufPtr->length;
				SNMPv3CopyDataToProcessBuff(0,dynScopedBufPtr);					// Placeholder.
				smSnmp++;
	
			case SM_ERROR_INDEX_OFFSET :
	
				// Similarly put error index.
				SNMPv3CopyDataToProcessBuff(ASN_INT,dynScopedBufPtr);				// Int type
				SNMPv3CopyDataToProcessBuff(1,dynScopedBufPtr);					// One byte long
				errorIndexOffset = dynScopedBufPtr->length;
				SNMPv3CopyDataToProcessBuff(0,dynScopedBufPtr);					// Placeholder.
	
				smSnmp++;
	
			case SM_FIND_NO_OF_REQUESTED_VARBINDS:

				varBindHeaderOffset_2 = dynScopedBufPtr->length;
				// Decode variable binding structure
				if ( SNMPv3CheckIfv3ValidStructure(&varBindHeaderLen) == false)
				{
					noOfVarbindreq = 0;						
				}
				else	//Find number of OIDs/varbinds's data requested in received PDU.
					noOfVarbindreq = SNMPv3FindOIDsFrmIncmingV3Req(varBindHeaderLen);
				
				tempOffset = dynScopedBufPtr->length;
				dynScopedBufPtr->length = varBindHeaderOffset;
				if(!noOfVarbindreq)
				{
					SNMPv3CopyDataToProcessBuff(REPORT_RESPONSE,dynScopedBufPtr);
				}
				else
				{
					SNMPv3CopyDataToProcessBuff(GET_RESPONSE,dynScopedBufPtr);	
				}				
				dynScopedBufPtr->length = tempOffset;
								
				SNMPv3CopyDataToProcessBuff(STRUCTURE,dynScopedBufPtr);
				varBindHeaderOffset_3 = dynScopedBufPtr->length;
				SNMPv3CopyDataToProcessBuff(0x82,dynScopedBufPtr);
				SNMPv3CopyDataToProcessBuff(0,dynScopedBufPtr);				
				SNMPv3CopyDataToProcessBuff(0,dynScopedBufPtr);
				if(noOfVarbindreq == 0)
				{
					SNMPv3ConstructReportPdu(dynScopedBufPtr);
					break;
				}
				smSnmp++;	
	
			case SM_FIND_NO_OF_RESPONSE_VARBINDS:
				//Calculate number of variables to be responded for the received request
				Getbulk_N = noOfVarbindreq; Getbulk_M=0; Getbulk_R=0;
				if((pduDbPtr->snmpVersion == (uint8_t)SNMP_V3) && 
					(pduDbPtr->pduType == GET_BULK_REQUEST))
				{
					if((pduDbPtr->nonRepeators) <= noOfVarbindreq)
					{
						Getbulk_N = pduDbPtr->nonRepeators;
					}	
					Getbulk_M = pduDbPtr->maxRepetitions;
	
					if((noOfVarbindreq - Getbulk_N)>=0u)
						Getbulk_R = noOfVarbindreq-Getbulk_N;
					
					noOfVarbindreq = Getbulk_N + (Getbulk_M * Getbulk_R);//Refer RFC 3416 
					
					if(Getbulk_N == 0)
					{
						smSnmp=SM_MAX_REPETITIONS;
						break;
					}
				}	
				//noOfVarbindreq = Getbulk_N + (Getbulk_M * Getbulk_R);//Refer RFC 3416 
	
				smSnmp++;
			case SM_VARSTRUCT_LEN_OFFSET:
				if(noOfVarbindreq == 0)
					break;
				
				if(Getbulk_N!= 0u) // decreament non repeators.
					Getbulk_N--;
				else if(Getbulk_M > 0) // jump to max repeatations
				{
					smSnmp = SM_MAX_REPETITIONS;
					break;
				}
				
				varIndex++;
				if ( SNMPv3CheckIfv3ValidStructure(&varBindHeaderLen) == false)
				{
					SNMPv3SetErrorStatus(errorStatusOffset,errorIndexOffset,SNMP_GEN_ERR,varIndex,dynScopedBufPtr);
                    bSnmpv3GenError = true;
                    break;
				}
				smSnmp++;
			case SM_POPULATE_REQ_OID:
				for(OIDlen=0;OIDlen<sizeof(OIDValue);OIDlen++)
					OIDValue[OIDlen]=0;
				OIDlen=0;
				if(SNMPv3CheckIfv3ValidOID(OIDValue,&OIDlen) == false)
				{
					SNMPv3SetErrorStatus(errorStatusOffset,errorIndexOffset,SNMP_GEN_ERR,varIndex,dynScopedBufPtr);
                    bSnmpv3GenError = true;
                    break;
				}
				
				// For Get & Get-Next, value must be NULL.
				if ( pduDbPtr->pduType != (uint8_t)SET_REQUEST )
				{
					if ( !SNMPv3CheckIfv3ASNNull() )
					{						
						SNMPv3SetErrorStatus(errorStatusOffset,errorIndexOffset,SNMP_GEN_ERR,varIndex,dynScopedBufPtr);
                        bSnmpv3GenError = true;
                        break;
					}
				}
				noOfVarbindreq--;
				smSnmp++;
			
			case SM_FIND_OID_IN_MIB:
			
				/* Search for the requested OID in the MIB database with the agent.*/
				
				
				//Searching the requested OID in the MIB database 
				oidLookUpRet = SNMPSearchOIDInMgmtInfoBase(pduDbPtr,OIDValue, OIDlen, &OIDInfo);	
				if(SNMPv3CopyDataToProcessBuff(STRUCTURE,dynScopedBufPtr) != true)
                {    
                    bSnmpV3GetBulkResError = true;
                    break;
                }
				varStructLenOffset= dynScopedBufPtr->length;
				if(SNMPv3CopyDataToProcessBuff(0,dynScopedBufPtr)!= true)
                {    
                    bSnmpV3GetBulkResError = true;
                    break;
                }
		
				// ASN OID data type
				if(SNMPv3CopyDataToProcessBuff(ASN_OID,dynScopedBufPtr)!= true)
                {    
                    bSnmpV3GetBulkResError = true;
                    break;
                }
		
				/* send the error code for SNMPv3 version for GET request and SET - request.
				As the follwing code is only for the get and set response, so SNMPv3CopyDataToProcessBuff is not
				under the buffer over flow check.
				*/
				if(oidLookUpRet != (uint8_t)true && (pduDbPtr->pduType != GET_NEXT_REQUEST) &&
					(pduDbPtr->pduType != GET_BULK_REQUEST))
				{
					if(snmpStkDcptMemStubPtr->appendZeroToOID)
						SNMPv3CopyDataToProcessBuff(OIDlen+1,dynScopedBufPtr);//for appending "0"
					else 
						SNMPv3CopyDataToProcessBuff(OIDlen,dynScopedBufPtr);//do not append "0"		
					pduLength = 0;							
					//Put OID
					while( OIDlen-- )
						SNMPv3CopyDataToProcessBuff(OIDValue[pduLength++],dynScopedBufPtr);//do not append "0"		
					
					if(snmpStkDcptMemStubPtr->appendZeroToOID)
					{
						SNMPv3CopyDataToProcessBuff(0x00,dynScopedBufPtr);//Appending '0' to OID in response
					}
					if(( pduDbPtr->snmpVersion == (uint8_t)SNMP_V3)
							&& (pduDbPtr->pduType == SNMP_GET))
					{
						SNMPv3CopyDataToProcessBuff(oidLookUpRet,dynScopedBufPtr);//Appending '0' to OID in response
						SNMPv3CopyDataToProcessBuff(0x0,dynScopedBufPtr);//Appending '0' to OID in response
					}
					tempOffset = dynScopedBufPtr->length;
					pduLength = dynScopedBufPtr->length-(varStructLenOffset+1);
					dynScopedBufPtr->length = varStructLenOffset;
					SNMPv3CopyDataToProcessBuff(pduLength,dynScopedBufPtr);
					dynScopedBufPtr->length = tempOffset;
					//Reset to state machine to access the next oid in request
					smSnmp=SM_VARSTRUCT_LEN_OFFSET;
					break;	
				}
				smSnmp++;
			
				//return false;
				
				case SM_NON_REPETITIONS:
				
					/*	Variables in get,get_next,set and get_bulk ( non repetition variables)
						of snmp request are processed in this part of the state machine.*/

					if(pduDbPtr->pduType == SNMP_SET)
					{
						uint8_t templen=OIDlen;
						uint8_t *ptroid=OIDValue;	
						//to validate the REC ID is present or not
	
						if(SNMPIdRecrdValidation(pduDbPtr,&OIDInfo,OIDValue,OIDlen) != true)
						{
							/*if(pduDbPtr->snmpVersion == (uint8_t)SNMP_V1)
					       		*errorStatus = SNMP_NO_SUCH_NAME;
							else if ((pduDbPtr->snmpVersion == (uint8_t)SNMP_V2C) ||
											(pduDbPtr->snmpVersion == (uint8_t)SNMP_V3))*/

							 /*if the variable binding's name specifies a
						     * variable which does not exist and could not ever be
						     * created, then the value of the Response-PDU's error-
						     * status field is set to `noCreation', and the value of its
						     * error-index field is set to the index of the failed
						     * variable binding.
						     */
							errorStatus = SNMP_NO_CREATION;
							
							return false;
						}
						
						if(snmpStkDcptMemStubPtr->appendZeroToOID)
							SNMPv3CopyDataToProcessBuff(OIDlen+1,dynScopedBufPtr);//for appending "0"
						else 
							SNMPv3CopyDataToProcessBuff(OIDlen,dynScopedBufPtr);//do not append "0"
						
						//Put OID
						while( templen-- )
							SNMPv3CopyDataToProcessBuff(*ptroid++,dynScopedBufPtr);//do not append "0" 	
						
						if(snmpStkDcptMemStubPtr->appendZeroToOID)
						{
							SNMPv3CopyDataToProcessBuff(0x00,dynScopedBufPtr);//Appending '0' to OID in response
						}
						//Now process the SET command
						tempRet = SNMPProcessSetVar(pduDbPtr,&OIDInfo, &errorStatus);
				
						if ( errorStatus != SNMP_NO_ERR )
						{
							//SET var command failed. Update the error status.
							SNMPv3SetErrorStatus(errorStatusOffset, \
										   errorIndexOffset, \
										   errorStatus, \
										   varIndex,dynScopedBufPtr); \
				
						}	
						
					}/*Get-Next-rquest also calls the SNMPProcessGetVar for 0the instance.  */
					else if((pduDbPtr->pduType == SNMP_GET) ||
						((pduDbPtr->pduType == SNMP_V2C_GET_BULK) && (oidLookUpRet == true)))
					{	
						uint8_t templen=OIDlen;
						uint8_t *ptroid=OIDValue;	

						//to validate the REC ID is present or not
						if(SNMPIdRecrdValidation(pduDbPtr,&OIDInfo,OIDValue,OIDlen) != true)
						{
							return false;
						}
						if(snmpStkDcptMemStubPtr->appendZeroToOID)
						{
							if(SNMPv3CopyDataToProcessBuff(OIDlen+1,dynScopedBufPtr)!= true) //for appending "0"
                            {    
                                bSnmpV3GetBulkResError = true;
                                break;
                            }
						}
						else 
						{
							if(SNMPv3CopyDataToProcessBuff(OIDlen,dynScopedBufPtr) != true)//do not append "0"
                            {    
                                bSnmpV3GetBulkResError = true;
                                break;
                            }
						}
						
						//Put OID
						while( templen-- )
						{
							if(SNMPv3CopyDataToProcessBuff(*ptroid++,dynScopedBufPtr)!= true)//do not append "0" 	
                            {    
                                bSnmpV3GetBulkResError = true;
                                break;
                            }
						}
						
						if(snmpStkDcptMemStubPtr->appendZeroToOID)
						{
							if(SNMPv3CopyDataToProcessBuff(0x00,dynScopedBufPtr)!= true)//Appending '0' to OID in response
                            {    
                                bSnmpV3GetBulkResError = true;
                                break;
                            }
						}

						tempRet = SNMPProcessGetVar(&OIDInfo,false,pduDbPtr);				
					}	
					else if((pduDbPtr->pduType == SNMP_GET_NEXT)||
						((pduDbPtr->pduType == SNMP_V2C_GET_BULK) && (oidLookUpRet != true)))
					{			
						tempRet = SNMPProcessGetNextVar(&OIDInfo,pduDbPtr);
						if(tempRet ==0)
						{
							uint8_t templen=OIDlen;
							uint8_t *ptroid=OIDValue;	
							
							if(snmpStkDcptMemStubPtr->appendZeroToOID)
							{
								if(SNMPv3CopyDataToProcessBuff(OIDlen+1,dynScopedBufPtr) != true)//for appending "0"
                                {    
                                    bSnmpV3GetBulkResError = true;
                                    break;
                                }
							}
							else 
							{
								if(SNMPv3CopyDataToProcessBuff(OIDlen,dynScopedBufPtr)!= true)//do not append "0"
                                {    
                                    bSnmpV3GetBulkResError = true;
                                    break;
                                }
							}
							
							//Put OID
							while( templen-- )
							{
								if(SNMPv3CopyDataToProcessBuff(*ptroid++,dynScopedBufPtr)!= true)//do not append "0" 
                                {    
                                    bSnmpV3GetBulkResError = true;
                                    break;
                                }
							}
							
							if(snmpStkDcptMemStubPtr->appendZeroToOID)
							{
								if(SNMPv3CopyDataToProcessBuff(0x00,dynScopedBufPtr)!= true)//Appending '0' to OID in response
                                {    
                                    bSnmpV3GetBulkResError = true;
                                    break;
                                }
							}
						}
					}
				
				
					/*	If the request command processing is failed, update
						the error status, index accordingly and response pdu.*/ 
					if(tempRet == 0u &&(pduDbPtr->pduType != SNMP_SET))
					{
						if(dynScopedBufPtr->length >= dynScopedBufPtr->maxlength)
                        {    
                            bSnmpV3GetBulkResError = true;
                            break;
                        }
						if(((pduDbPtr->pduType == SNMP_GET_NEXT)|| (pduDbPtr->pduType == SNMP_V2C_GET_BULK))&&pduDbPtr->snmpVersion == (uint8_t)SNMP_V3)
						{
							if(SNMPv3CopyDataToProcessBuff(SNMP_END_OF_MIB_VIEW,dynScopedBufPtr)!= true)
                            {    
                                bSnmpV3GetBulkResError = true;
                                break;
                            }
							if(SNMPv3CopyDataToProcessBuff(0,dynScopedBufPtr)!= true)
                            {    
                                bSnmpV3GetBulkResError = true;
                                break;
                            }
							//if get bulk response reaches END of mIB view break from the loop.
							noOfVarbindreq = 0;
							Getbulk_N = 0u; 					
						}
				
					}
					dynScopedBufPtr->length = Snmpv3StackDcptStubPtr->ScopedPduRespnsBuf.length;
					tempOffset = dynScopedBufPtr->length;
					pduLength = dynScopedBufPtr->length-(varStructLenOffset+1);
					dynScopedBufPtr->length = varStructLenOffset;
					SNMPv3CopyDataToProcessBuff(pduLength,dynScopedBufPtr);
					dynScopedBufPtr->length = tempOffset;

					/* to avoid Dynamic out buffer crash   we need to calculate buffer
					availability .Approximatly the next variable bind length should be less than 30.*/
					
					if(dynScopedBufPtr->length>= dynScopedBufPtr->maxlength)
					{
						noOfVarbindreq = 0;
						Getbulk_N = 0u;						
						break;
					}
					/*	Decide on the number of Non repetition variables remained to 
						be processed, decide the course of state machine.*/
					if((pduDbPtr->pduType==GET_BULK_REQUEST) && (pduDbPtr->snmpVersion == (uint8_t)SNMP_V3)
														&&( Getbulk_N == 0u))
					{					
						smSnmp=SM_MAX_REPETITIONS;
					}
					else
					{
						smSnmp=SM_VARSTRUCT_LEN_OFFSET;
					}
				
					snmpStkDcptMemStubPtr->getZeroInstance = false;
					break;
				case SM_MAX_REPETITIONS:
					maxRepeatationOffset = Snmpv3StackDcptStubPtr->ScopedPduDataPos;
					/*Process each variable in request as Get_Next for 
					  Getbulk_M (Max_repetition) times */
					for(repeatCntr=0;repeatCntr<Getbulk_M;repeatCntr++)
					{
						Snmpv3StackDcptStubPtr->ScopedPduDataPos = maxRepeatationOffset;
						
						//Process every veriable in the request.
						for(varBindCntr=0;varBindCntr<Getbulk_R;varBindCntr++)
						{
							if(noOfVarbindreq != 0)
							{
								noOfVarbindreq--;
							}
							else
							{
								break;
							}
							
							if(varBindCntr==0u)
							{
								varIndex=(noOfVarbindreq - Getbulk_R);
							}
				
							varIndex++;

							if(SNMPv3CopyDataToProcessBuff(STRUCTURE,dynScopedBufPtr)!= true)
                            {    
                                bSnmpV3GetBulkResError = true;
                                break;
                            }
							varStructLenOffset= dynScopedBufPtr->length;
							if(SNMPv3CopyDataToProcessBuff(0,dynScopedBufPtr)!= true)
                            {    
                                bSnmpV3GetBulkResError = true;
                                break;
                            }
							succesor=repeatCntr;
				
							// Decode variable length structure
							if(SNMPv3CheckIfv3ValidStructure(&varBindHeaderLen) == false)
							{
								SNMPv3SetErrorStatus(errorStatusOffset,errorIndexOffset,SNMP_GEN_ERR,varIndex,dynScopedBufPtr);
                                bSnmpv3GenError = true;
                                break;
							}
							// Decode next object
							if ( !SNMPv3CheckIfv3ValidOID(OIDValue, &OIDlen) )
							{
								SNMPv3SetErrorStatus(errorStatusOffset,errorIndexOffset,SNMP_GEN_ERR,varIndex,dynScopedBufPtr);
                                bSnmpv3GenError = true;
                                break;
							}
							
							// For Get & Get-Next, value must be NULL.
							if ( pduDbPtr->pduType != (uint8_t)SET_REQUEST )
								if ( !SNMPv3CheckIfv3ASNNull() )
									break;
				
							oidLookUpRet = SNMPSearchOIDInMgmtInfoBase(pduDbPtr,OIDValue, OIDlen, &OIDInfo);
							if(oidLookUpRet == SNMP_END_OF_MIB_VIEW)
							{
								tempRet = SNMPGetNextLeaf(&OIDInfo);
							}
							if(oidLookUpRet == false)
							{
								uint8_t templen=OIDlen;
								uint8_t *ptroid=OIDValue;	
								
								if(SNMPv3CopyDataToProcessBuff(ASN_OID,dynScopedBufPtr)!= true)
                                {    
                                    bSnmpV3GetBulkResError = true;
                                    break;
                                }
								if(snmpStkDcptMemStubPtr->appendZeroToOID)
								{
									if(SNMPv3CopyDataToProcessBuff(OIDlen+1,dynScopedBufPtr)!= true)
                                    {    
                                        bSnmpV3GetBulkResError = true;
                                        break;
                                    }
									OIDlen += 1;
								}
								else 
									if(SNMPv3CopyDataToProcessBuff(OIDlen,dynScopedBufPtr) != true)
                                    {    
                                        bSnmpV3GetBulkResError = true;
                                        break;
                                    }
				
								//Put OID
								while( templen-- )
									if(SNMPv3CopyDataToProcessBuff(*ptroid++,dynScopedBufPtr) != true)
                                    {    
                                        bSnmpV3GetBulkResError = true;
                                        break;
                                    }
								if(snmpStkDcptMemStubPtr->appendZeroToOID)
									if(SNMPv3CopyDataToProcessBuff(0x0,dynScopedBufPtr) != true)
                                    {    
                                        bSnmpV3GetBulkResError = true;
                                        break;
                                    }
								if(SNMPv3CopyDataToProcessBuff(SNMP_END_OF_MIB_VIEW,dynScopedBufPtr) != true)
                                {    
                                    bSnmpV3GetBulkResError = true;
                                    break;
                                }
								if(SNMPv3CopyDataToProcessBuff(0x0,dynScopedBufPtr)!= true)
                                {    
                                    bSnmpV3GetBulkResError = true;
                                    break;
                                }

								noOfVarbindreq = 0;

							}
							else if(tempRet != 0)//if(oidLookUpRet != SNMP_END_OF_MIB_VIEW)
							{
								tempRet = SNMPProcessGetBulkVar(&OIDInfo, &OIDValue[0],&OIDlen,&succesor,pduDbPtr);
							}
							if ( tempRet == 0u )
							{
								uint8_t templen=OIDlen;
								uint8_t *ptroid=OIDValue;	
								if(SNMPv3CopyDataToProcessBuff(ASN_OID,dynScopedBufPtr) != true)
                                {    
                                    bSnmpV3GetBulkResError = true;
                                    break;
                                }
								if(snmpStkDcptMemStubPtr->appendZeroToOID)
								{
									if(SNMPv3CopyDataToProcessBuff(OIDlen+1,dynScopedBufPtr)!= true) //for appending "0"
                                    {    
                                        bSnmpV3GetBulkResError = true;
                                        break;
                                    }
									OIDlen += 1;
								}
								else 
									if(SNMPv3CopyDataToProcessBuff(OIDlen,dynScopedBufPtr)!=  true)
                                    {    
                                        bSnmpV3GetBulkResError = true;
                                        break;
                                    }
				
								//Put OID
								while( templen-- )
									if(SNMPv3CopyDataToProcessBuff(*ptroid++,dynScopedBufPtr) != true)
                                    {    
                                        bSnmpV3GetBulkResError = true;
                                        break;
                                    }
				
								/*Do send back the Same OID if get_next is EndOfMibView. Do not
								  append zero to this OID*/
								
								if(SNMPv3CopyDataToProcessBuff(SNMP_END_OF_MIB_VIEW,dynScopedBufPtr) != true)
                                {    
                                    bSnmpV3GetBulkResError = true;
                                    break;
                                }
								if(SNMPv3CopyDataToProcessBuff(0x0,dynScopedBufPtr) != true)
                                {    
                                    bSnmpV3GetBulkResError = true;
                                    break;
                                }
				
								if(snmpStkDcptMemStubPtr->appendZeroToOID)
									if(SNMPv3CopyDataToProcessBuff(0x0,dynScopedBufPtr) != true)
                                    {    
                                        bSnmpV3GetBulkResError = true;
                                        break;
                                    }
								noOfVarbindreq = 0;
								
							}
							
						
							dynScopedBufPtr = &Snmpv3StackDcptStubPtr->ScopedPduRespnsBuf;
							tempOffset = dynScopedBufPtr->length;
							pduLength = dynScopedBufPtr->length -(varStructLenOffset+1);
							dynScopedBufPtr->length = varStructLenOffset;
							SNMPv3CopyDataToProcessBuff(pduLength,dynScopedBufPtr);
							dynScopedBufPtr->length = tempOffset;

							/* if length dynamic buffer length increases more than the allocated memory 
							then break from the loop.*/
							if(dynScopedBufPtr->length>= dynScopedBufPtr->maxlength)
							{
								noOfVarbindreq = 0;
								Getbulk_N = 0u; 					
                                bSnmpV3GetBulkResError = true;
                                break;
							}
							tempRet = 0xFF;
						}//for(varBindCntr=0;varBindCntr<Getbulk_R;varBindCntr++)						
					}//for(repeatCntr=0;repeatCntr<Getbulk_M;repeatCntr++)
					/* check length*/
				break;	

			default:				
				SNMPv3FreeDynAllocMem();
				return false;
		}
		if(noOfVarbindreq == 0)
		{
			break;
		}
	}

	if(bSnmpV3GetBulkResError && (dynScopedBufPtr->length >= dynScopedBufPtr->maxlength))
	{
		if(pduDbPtr->pduType == SNMP_V2C_GET_BULK)
		{
			pduLength = dynScopedBufPtr->length - (varStructLenOffset-1);
			dynScopedBufPtr->length = dynScopedBufPtr->length - pduLength;			
		}
		else
		{
			dynScopedBufPtr->length = varBindHeaderOffset_3;
			SNMPv3CopyDataToProcessBuff(0,dynScopedBufPtr);
			SNMPv3SetErrorStatus(errorStatusOffset,errorIndexOffset,SNMP_TOO_BIG,varIndex,dynScopedBufPtr); 
		}
	}


	/* pass the data to wire*/
	{
		uint16_t	 	scopedpduHeaderOffset = 0;
		TCPIP_UINT16_VAL 	totalPdulength = {0};
		TCPIP_UINT16_VAL	scoped_pdu_len_1 = {0}; // context data length
		TCPIP_UINT16_VAL	scoped_pdu_len_2 = {0}; // pdu response length
		TCPIP_UINT16_VAL	scoped_pdu_len_3 = {0}; //variable binding header with varbinds
		uint16_t 		i=0;
		//uint32_t       scopedPduHeadPtr=0;
		SNMPV3MSGDATA tempScopedData;
		
		scopedpduHeaderOffset = dynScopedBufPtr->length;
		tempScopedData = Snmpv3StackDcptStubPtr->ScopedPduRespnsBuf;
		
		// update length for variable binds
		scoped_pdu_len_3.Val = (dynScopedBufPtr->length-3)-varBindHeaderOffset_3;
		dynScopedBufPtr->length = varBindHeaderOffset_3+1;
		SNMPv3CopyDataToProcessBuff(scoped_pdu_len_3.v[1],dynScopedBufPtr);
		SNMPv3CopyDataToProcessBuff(scoped_pdu_len_3.v[0],dynScopedBufPtr);
		dynScopedBufPtr->length = scopedpduHeaderOffset;
		
		//update the length for get response pduLenOffset 
		scoped_pdu_len_2.Val = (dynScopedBufPtr->length-3)-pduLenOffset;
		dynScopedBufPtr->length = pduLenOffset+1;
		SNMPv3CopyDataToProcessBuff(scoped_pdu_len_2.v[1],dynScopedBufPtr);
		SNMPv3CopyDataToProcessBuff(scoped_pdu_len_2.v[0],dynScopedBufPtr);
		dynScopedBufPtr->length = scopedpduHeaderOffset;

		scoped_pdu_len_1.Val = dynScopedBufPtr->length-4;
		if((scoped_pdu_len_1.Val >= 0x80) && (scoped_pdu_len_1.Val <= 0xFF))
		{// total scoped pdu length decreamented by 1
			dynScopedBufPtr->length = contextNameOffset+1;
			//if(Snmpv3StackDcptStubPtr->SnmpSecurityLevel>>1 & 0x01)
			//	SNMPv3CopyDataToProcessBuff(0x04,dynScopedBufPtr);
			SNMPv3CopyDataToProcessBuff(0x30,dynScopedBufPtr);
			SNMPv3CopyDataToProcessBuff(0x81,dynScopedBufPtr);
			SNMPv3CopyDataToProcessBuff(scoped_pdu_len_1.Val,dynScopedBufPtr);
			dynScopedBufPtr->length = scopedpduHeaderOffset;
			tempScopedData.head++;
			tempScopedData.length--;
		}
		else if((scoped_pdu_len_1.Val > 0xFF) && (scoped_pdu_len_1.Val < 0xFFFF))
		{			
			dynScopedBufPtr->length = contextNameOffset;
			SNMPv3CopyDataToProcessBuff(0x30,dynScopedBufPtr);
			SNMPv3CopyDataToProcessBuff(0x82,dynScopedBufPtr);
			SNMPv3CopyDataToProcessBuff(scoped_pdu_len_1.v[1],dynScopedBufPtr);
			SNMPv3CopyDataToProcessBuff(scoped_pdu_len_1.v[0],dynScopedBufPtr);
			dynScopedBufPtr->length = scopedpduHeaderOffset;
			//Snmpv3StackDcptStubPtr->ScopedPduRespnsBuf.head++;
			//Snmpv3StackDcptStubPtr->ScopedPduRespnsBuf.length--;
		}
		else
		{// total scoped pdu length decreamented by 2
			dynScopedBufPtr->length = contextNameOffset+2;
			SNMPv3CopyDataToProcessBuff(0x30,dynScopedBufPtr);
			//SNMPv3CopyDataToProcessBuff(0x81,dynScopedBufPtr);
			SNMPv3CopyDataToProcessBuff(scoped_pdu_len_1.Val,dynScopedBufPtr);
			dynScopedBufPtr->length = scopedpduHeaderOffset;
			tempScopedData.head=tempScopedData.head+2;
			tempScopedData.length=tempScopedData.length-2;
		}


		//dynScopedBufPtr = &Snmpv3StackDcptStubPtr->ScopedPduRespnsBuf;
		totalPdulength.Val = tempScopedData.length + \
							 Snmpv3StackDcptStubPtr->PduHeaderBuf.length + \
							 3; // asn_int+len+version

		if((Snmpv3StackDcptStubPtr->SnmpSecurityLevel & 0x02)==0x02)
		{
			tempPtr=tempBuf;
			*tempPtr++=0X04;
			if((tempScopedData.length >= 0x80) && (tempScopedData.length <= 0xFF))
			{
				*tempPtr++=0x81;
				*tempPtr=tempScopedData.length;
				tempCntr=3; //0x04(encrypted pkt),0x81,len
			}
			else if((tempScopedData.length > 0xFF) && (tempScopedData.length < 0xFFFF))
			{			
				*tempPtr++=0x82;
				*tempPtr++=tempScopedData.length>>8;
				*tempPtr=tempScopedData.length;
				tempCntr=4; //0x04(encrypted pkt),0x81,len_1,len_0
			}
			else
			{
				*tempPtr=tempScopedData.length;
				tempCntr=2; //0x04(encrypted pkt),len
			}
		}

		Snmpv3StackDcptStubPtr->OUTPduWholeMsgBuf.wholeMsgLen.Val=(totalPdulength.Val+tempCntr/*0x04,0x82,len_1,len_0*/);
		Snmpv3StackDcptStubPtr->OUTPduWholeMsgBuf.wholeMsgHead=
			(uint8_t*)(TCPIP_HEAP_Calloc(snmpPktProcsMemPtrsInfo.snmpHeapMemHandler,1,(size_t)Snmpv3StackDcptStubPtr->OUTPduWholeMsgBuf.wholeMsgLen.Val+4+16));
		if(Snmpv3StackDcptStubPtr->OUTPduWholeMsgBuf.wholeMsgHead == NULL)
			return false;
		outBufPtr=Snmpv3StackDcptStubPtr->OUTPduWholeMsgBuf.wholeMsgHead;	


		//Start Writing to the outPut Buffer

		*outBufPtr++=STRUCTURE;

		totalPdulength.Val+=tempCntr;
							 
		if((totalPdulength.Val >= 0x80) && (totalPdulength.Val <= 0xFF))
		{
			*outBufPtr++=0x81;
			*outBufPtr++=totalPdulength.Val;
		}
		else if((totalPdulength.Val > 0xFF) && (totalPdulength.Val < 0xFFFF))
		{			
			*outBufPtr++=0x82;
			*outBufPtr++=totalPdulength.v[1];
			*outBufPtr++=totalPdulength.v[0];
		}
		else
			*outBufPtr++=totalPdulength.Val;

		*outBufPtr++=ASN_INT;
		*outBufPtr++=0x1;		
		*outBufPtr++=SNMP_V3;

		Snmpv3StackDcptStubPtr->OUTPduWholeMsgBuf.msgAuthParamOffsetOutWholeMsg=(uint8_t*)(outBufPtr+Snmpv3StackDcptStubPtr->PduHeaderBuf.msgAuthParamOffset);
		//put global snmpv3 msg header 
		for(i=0;i<Snmpv3StackDcptStubPtr->PduHeaderBuf.length;i++)
		{
			*outBufPtr++=Snmpv3StackDcptStubPtr->PduHeaderBuf.head[i];
		}

		//Copy Scoped PDU to the Out Buffer
		if((Snmpv3StackDcptStubPtr->SnmpSecurityLevel & 0x02)==0x02) //Encrypted message	
		{
			//Copy Packet Auth indicator, length
			for(i=0;i<tempCntr;i++) 
			{
				*outBufPtr++=tempBuf[i];

			}
		}
		Snmpv3StackDcptStubPtr->OUTPduWholeMsgBuf.scopedPduOffset=outBufPtr;
		Snmpv3StackDcptStubPtr->OUTPduWholeMsgBuf.scopedPduStructLen=tempScopedData.length;

		i=0;
		*outBufPtr++=tempScopedData.head[i++];//0x30

		if(tempScopedData.head[1] == 0x81)
		{
			*outBufPtr++=tempScopedData.head[i++];//0x81
			*outBufPtr++=tempScopedData.head[i++];//len_0
		}
		else if(tempScopedData.head[1] == 0x82)
		{
			*outBufPtr++=tempScopedData.head[i++]; //0x82
			*outBufPtr++=tempScopedData.head[i++]; //len_1
			*outBufPtr++=tempScopedData.head[i++]; //len_0
		}
		else
			*outBufPtr++=tempScopedData.head[i++];//len_o
		
		// send context id and context name and the get response 
		// Authentication and privacy data packet will be sent from here onwards
		for(;i<(tempScopedData.length);i++)
		{
			*outBufPtr++=tempScopedData.head[i];
		}


		/*Encrypt the Response to the messgae originator*/
		if((Snmpv3StackDcptStubPtr->SnmpSecurityLevel & 0x02)==0x02) //Encrypted message
		{
			 /*Rxed SNMPv3 message is encrypted. Hence Response should be encrypted*/
			 Snmpv3StackDcptStubPtr->OUTPduWholeMsgBuf.wholeMsgLen.Val = outBufPtr-Snmpv3StackDcptStubPtr->OUTPduWholeMsgBuf.wholeMsgHead;
			 /*If user privacy protocol is AES*/
			if(SNMPv3AESEncryptResponseScopedPdu(&Snmpv3StackDcptStubPtr->OUTPduWholeMsgBuf) != SNMPV3_MSG_PRIV_PASS)
				return SNMPV3_MSG_PRIV_FAIL;
		
			/*If user privacy Protocol is DES*/
			//snmpV3DESDecryptRxedScopedPdu();
		}

		/* Authenticate the whole message to be transmitted*/
		if((Snmpv3StackDcptStubPtr->SnmpSecurityLevel & 0x01)==0x01) //Authenticatd message
		{
			 /*Rxed SNMPv3 message is Authenticated.Send authenticatin parameters for the Response*/
			Snmpv3StackDcptStubPtr->OUTPduWholeMsgBuf.wholeMsgLen.Val = outBufPtr-Snmpv3StackDcptStubPtr->OUTPduWholeMsgBuf.wholeMsgHead;
			 /*If user authentication is HAMC-MD5-96*/
			if(SNMPv3AuthenticateTxPduForDataIntegrity(&Snmpv3StackDcptStubPtr->OUTPduWholeMsgBuf)!=SNMPV3_MSG_AUTH_PASS)
				return SNMPV3_MSG_AUTH_FAIL;

			tempPtr = outBufPtr;
			outBufPtr=Snmpv3StackDcptStubPtr->OUTPduWholeMsgBuf.msgAuthParamOffsetOutWholeMsg;
			for(i=0;i<Snmpv3StackDcptStubPtr->SnmpOutMsgAuthParmLen;i++)
				*outBufPtr++=Snmpv3StackDcptStubPtr->SnmpOutMsgAuthParaStrng[i];
			outBufPtr = tempPtr;
		}
			
		Snmpv3StackDcptStubPtr->OUTPduWholeMsgBuf.wholeMsgLen.Val = outBufPtr-Snmpv3StackDcptStubPtr->OUTPduWholeMsgBuf.wholeMsgHead;
		outBufPtr=Snmpv3StackDcptStubPtr->OUTPduWholeMsgBuf.wholeMsgHead;
		
		for(i=0;i < Snmpv3StackDcptStubPtr->OUTPduWholeMsgBuf.wholeMsgLen.Val;i++)
		{	
			SNMPPutByteToUDPTxBuffer(snmpPktProcsMemPtrsInfo.snmpDcptPtr[intfIdx].skt,*(outBufPtr+i));
		}

		//if(Snmpv3StackDcptStubPtr->ScopedPduRespnsBuf.head != NULL)
		if(Snmpv3StackDcptStubPtr->OUTPduWholeMsgBuf.wholeMsgHead != NULL)
		{
			TCPIP_HEAP_Free(snmpPktProcsMemPtrsInfo.snmpHeapMemHandler,Snmpv3StackDcptStubPtr->OUTPduWholeMsgBuf.wholeMsgHead);
			Snmpv3StackDcptStubPtr->OUTPduWholeMsgBuf.wholeMsgHead=0x00;
			Snmpv3StackDcptStubPtr->OUTPduWholeMsgBuf.wholeMsgLen.Val=0x00;
			Snmpv3StackDcptStubPtr->OUTPduWholeMsgBuf.snmpMsgHead = NULL;
			Snmpv3StackDcptStubPtr->OUTPduWholeMsgBuf.msgAuthParamOffsetOutWholeMsg = NULL;
			Snmpv3StackDcptStubPtr->OUTPduWholeMsgBuf.scopedPduOffset =  NULL;			
		}

	}
	return true;
}


/****************************************************************************
  Function:
	static bool SNMPv3CheckIfValidInt(uint32_t* val)
	
  Summary:
  	Verifies variable datatype as int and retrieves its value.

  Description:
  	This routine populates and validates the received variable for the
  	data type as "ASN_INT" and the data length for max 4 bytes.
  	This rotuine only refers to the incoming snmpv3 request dynamically 
	allocated 	memory buffer 'ScopedPduRequstBuf' .
  	
  	 		 		  	
  Precondition:
	None
	
  Parameters:
  	val - Pointer to memory where int var value will be stored.
 
  ReturnValues:
	true	- If valid integer type and value is received.
	false	- Other than integer data type and value received .
	
  Remarks:
	None.
***************************************************************************/
static bool SNMPv3CheckIfValidInt(uint32_t* val)
{
	TCPIP_UINT32_VAL tempData;
    uint8_t	tempLen=0;

    
    // Get variable type
    if (!IS_ASN_INT(SNMPv3GetProcessBuffData(Snmpv3StackDcptStubPtr->ScopedPduRequstBuf,++Snmpv3StackDcptStubPtr->ScopedPduDataPos)))
        return false;

	// Integer length of more than 32-bit is not supported.
	tempLen = SNMPv3GetProcessBuffData(Snmpv3StackDcptStubPtr->ScopedPduRequstBuf,++Snmpv3StackDcptStubPtr->ScopedPduDataPos);
    if ( tempLen > 4u)
        return false;
    

    tempData.Val = 0;
    while( tempLen-- )
        tempData.v[tempLen] = SNMPv3GetProcessBuffData(Snmpv3StackDcptStubPtr->ScopedPduRequstBuf,
        													++Snmpv3StackDcptStubPtr->ScopedPduDataPos);

    *val = tempData.Val;

    return true;
}


/****************************************************************************
  Function:
	static bool SNMPv3CheckIfValidIntDataType(uint8_t * wholeMsgPtr,uint16_t* pos, uint32_t* val )
	
  Summary:
  	Verifies variable datatype as int and retrieves its value.

  Description:
  	This routine populates and validates the received variable for the
  	data type as "ASN_INT" and the data length for max 4 bytes.
  	 		 		  	
  Precondition:
	None
	
  Parameters:
  	wholeMsgPtr - Pointer to memory where int var value is be stored.
 	pos - position in the memory buffer where data taype to be varified is stored
 	val - Pointer to memory where int var value will be stored.
 	
  ReturnValues:
	true	- If valid integer type and value is received.
	false	- Other than integer data type and value received .
	
  Remarks:
	None.
***************************************************************************/

static bool SNMPv3CheckIfValidIntDataType(uint8_t * wholeMsgPtr,uint16_t* pos, uint32_t* val )
{
	TCPIP_UINT32_VAL tempData;
    uint8_t	tempLen=0;

	uint8_t* lclWholeMsgPtr;
	
	lclWholeMsgPtr=wholeMsgPtr;

    
    // Get variable type
    if (!IS_ASN_INT(SNMPv3GetWholeMsgBufferData(lclWholeMsgPtr,pos)))
        return false;

	// Integer length of more than 32-bit is not supported.
	tempLen = SNMPv3GetWholeMsgBufferData(lclWholeMsgPtr,pos);
    if ( tempLen > 4u)
        return false;
    

    tempData.Val = 0;
    while( tempLen-- )
        tempData.v[tempLen] = SNMPv3GetWholeMsgBufferData(lclWholeMsgPtr,pos);

    *val = tempData.Val;

    return true;
}

/****************************************************************************
  Function:
	static bool SNMPv3CheckIfv3ValidStructure(uint16_t* dataLen)
	
  Summary:
  	Decode variable length structure.

  Description:
	This routine is used  to verify whether the received varbind is of type
	STRUCTURE and to find out the variable binding structure length.
	This rotuine only refers to the incoming snmpv3 request dynamically 
	allocated 	memory buffer 'ScopedPduRequstBuf' .
  	
  Precondition:
	SNMPProcessHeader() is called.	
	
  Parameters:
  	datalen	- Pointer to memory to store OID structure length.
 
  Return Values:
	true	- If valid Structure data type and value is received.
	false	- If variable data structure is not type STRUCTURE. 	

  Remarks:
	None.
***************************************************************************/
static bool SNMPv3CheckIfv3ValidStructure(uint16_t* dataLen)
{
    TCPIP_UINT32_VAL tempLen;

    if ( !IS_STRUCTURE(SNMPv3GetProcessBuffData(Snmpv3StackDcptStubPtr->ScopedPduRequstBuf,++Snmpv3StackDcptStubPtr->ScopedPduDataPos)) )
        return false;

	tempLen.Val = SNMPv3GetProcessBuffData(Snmpv3StackDcptStubPtr->ScopedPduRequstBuf,++Snmpv3StackDcptStubPtr->ScopedPduDataPos);
    if ( tempLen.Val == 0x81) 
    {
        tempLen.Val = SNMPv3GetProcessBuffData(Snmpv3StackDcptStubPtr->ScopedPduRequstBuf,++Snmpv3StackDcptStubPtr->ScopedPduDataPos);
    }
	else if(tempLen.Val == 0x82)
	{
        tempLen.v[1] = SNMPv3GetProcessBuffData(Snmpv3StackDcptStubPtr->ScopedPduRequstBuf,++Snmpv3StackDcptStubPtr->ScopedPduDataPos);		
        tempLen.v[0] = SNMPv3GetProcessBuffData(Snmpv3StackDcptStubPtr->ScopedPduRequstBuf,++Snmpv3StackDcptStubPtr->ScopedPduDataPos);
	}
	else if(tempLen.Val == 0)
		return false;
    // Since we are using UDP as our transport and UDP are not fragmented,
    // this structure length cannot be more than 1500 bytes.
    // As a result, we will only use lower uint16_t of length value.
    *dataLen = tempLen.Val;

    return true;
}


/****************************************************************************
  Function:
	static bool SNMPv3CheckIfValidV3StructAnd4ByteDataLen(uint8_t* wholeMsgPtr,uint16_t* pos, uint16_t* dataLen )
	
  Summary:
  	Decode variable length structure.

  Description:
  	This routine populates and validates the received variable for the
  	data type as "STRUCTURE" and the data length for max 4 bytes.
  	 		 		  	
  Precondition:
	None
	
  Parameters:
  	wholeMsgPtr - Pointer to memory where int var value is be stored.
 	pos - position in the memory buffer where data taype to be varified is stored
 	val - Pointer to memory where int var value will be stored.
 	
  ReturnValues:
	true	- If valid integer type and value is received.
	false	- Other than integer data type and value received .
	
  Remarks:
	None.
***************************************************************************/
static bool SNMPv3CheckIfValidV3StructAnd4ByteDataLen(uint8_t* wholeMsgPtr,uint16_t* pos, uint16_t* dataLen )
{
    TCPIP_UINT32_VAL tempLen;

    if ( !IS_STRUCTURE(SNMPv3GetWholeMsgBufferData(wholeMsgPtr,pos)) )
        return false;


	tempLen.Val = SNMPv3GetWholeMsgBufferData(wholeMsgPtr,pos);
    if ( tempLen.Val == 0x81) 
    {
        tempLen.Val = SNMPv3GetWholeMsgBufferData(wholeMsgPtr,pos);
    }
	else if(tempLen.Val == 0x82)
	{
        tempLen.v[1] = SNMPv3GetWholeMsgBufferData(wholeMsgPtr,pos);		
        tempLen.v[0] = SNMPv3GetWholeMsgBufferData(wholeMsgPtr,pos);
	}
	else if(tempLen.Val == 0)
		return false;
    // Since we are using UDP as our transport and UDP are not fragmented,
    // this structure length cannot be more than 1500 bytes.
    // As a result, we will only use lower uint16_t of length value.
    *dataLen = tempLen.Val;

    return true;
}


/****************************************************************************
  Function:
	static uint8_t SNMPv3FindOIDsFrmIncmingV3Req(uint16_t pdulen)
	
  Summary:
  	Finds number of varbinds in the varbind list received in a SNMPv3 pdu.
  	
  Description:
  	This routine is used to find the number of OIDs requested in the received
  	snmp pdu.   	
  	
  Precondition	:
	SNMPProcessVariables() is called.	
		
  Parameters:
	pdulen		-	Length of snmp pdu request received. 
    
  Return Values:
	varCount	-	Number of OIDs found in a pdu request. 
	
  Remarks:
  	None.
  	
***************************************************************************/
static uint8_t SNMPv3FindOIDsFrmIncmingV3Req(uint16_t pdulen)
{
uint8_t varCount=0;
uint16_t prevUDPRxOffset;
uint16_t varBindLen;
uint16_t snmpPduLen;
 	
	snmpPduLen=pdulen;

	prevUDPRxOffset=Snmpv3StackDcptStubPtr->ScopedPduDataPos;

	while(snmpPduLen)
	{
		
		if(!SNMPv3CheckIfv3ValidStructure(&varBindLen))
			return false;

		Snmpv3StackDcptStubPtr->ScopedPduDataPos = Snmpv3StackDcptStubPtr->ScopedPduDataPos + varBindLen;
		varCount++;
		snmpPduLen=snmpPduLen
					-1      //  1   byte for STRUCTURE identifier
					-1  //  1  byte for varbind length 
					-varBindLen;
		
	}

	Snmpv3StackDcptStubPtr->ScopedPduDataPos=prevUDPRxOffset;

	return varCount;
}


/****************************************************************************
  Function:
	static void SNMPv3ConstructReportPdu(SNMPV3MSGDATA *dynScopedBufPtr)
	
  Summary:
  	Constructs the report pdu infomration for the Report Pdu.
  	
  Description:
  	The SNMPv3 PDU exchange starts with the agent sending a report pdu on 
  	reception of any Get_Request PDU for SNMPv3 request. 
  	This routine froms the report pdu for response to the requesting entity.
  	
  Precondition	:
	SNMPProcessVariables() is called and a valid SNMPv3 request is received. 
		
  Parameters:
	dynScopedBufPtr	- pointer to the response buffer memory where the 'report' response 
					   to be savced for transmission. 
    
  Return Values:
	None
	
  Remarks:
  	None.
  	
***************************************************************************/
static void SNMPv3ConstructReportPdu(SNMPV3MSGDATA *dynScopedBufPtr)
{
	uint8_t	usmStatEngineIds[]={43,6,1,6,3,15,1,1,4,0};
	uint8_t	reportPduLenOffset=0;	
	uint8_t	usmLen=0,i=0;
	uint16_t	varbindPairOffset1=0;
	

	SNMPv3CopyDataToProcessBuff(STRUCTURE,dynScopedBufPtr);
	varbindPairOffset1 = dynScopedBufPtr->length;
	SNMPv3CopyDataToProcessBuff(0,dynScopedBufPtr);

/* put  usm OID */
	SNMPv3CopyDataToProcessBuff(ASN_OID,dynScopedBufPtr);
	usmLen = sizeof(usmStatEngineIds);
	SNMPv3CopyDataToProcessBuff(usmLen,dynScopedBufPtr);
	while(usmLen--)
		SNMPv3CopyDataToProcessBuff(usmStatEngineIds[i++],dynScopedBufPtr);

/* put engine ID stat value */	
	SNMPv3CopyDataToProcessBuff(SNMP_COUNTER32,dynScopedBufPtr);
	SNMPv3CopyDataToProcessBuff(4,dynScopedBufPtr);
	SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->UsmStatsEngineID.v[3],dynScopedBufPtr);
	SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->UsmStatsEngineID.v[2],dynScopedBufPtr);
	SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->UsmStatsEngineID.v[1],dynScopedBufPtr);
	SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->UsmStatsEngineID.v[0],dynScopedBufPtr);
	
	reportPduLenOffset = dynScopedBufPtr->length;
	
	usmLen = dynScopedBufPtr->length - (varbindPairOffset1+1) ;
	dynScopedBufPtr->length = varbindPairOffset1;
	SNMPv3CopyDataToProcessBuff(usmLen,dynScopedBufPtr);
	
	dynScopedBufPtr->length = reportPduLenOffset;
	
	
}


/****************************************************************************
  Function:
	static bool SNMPv3CheckIfv3ValidOID(uint8_t* oid, uint8_t* len)
	
  Summary:
  	Populates OID type, length and oid string from the received pdu.

  Description:
	In this routine, OID data type "ASN_OID" is verified in the received pdu.
	If the data type is matched, then only var bind is processed. OID length
	and OID is populated. The max OID length can be 15. 
  	
  Precondition:
	ProcessVariabels() is called.
	
  Parameters:
  	oid - Pointer to memory to store the received OID string
  	len	- Pointer to memory to store OID length
 
  Return Values:
	true	- If value type is ASN_OID and oid length not more than 15. 
	false	- Otherwise.
	
  Remarks:
	None.
***************************************************************************/
static bool SNMPv3CheckIfv3ValidOID(uint8_t* oid, uint8_t* len)
{
    uint8_t tempLen=0;

    // Fetch and verify that this is OID.
    if ( !IS_OID(SNMPv3GetProcessBuffData(Snmpv3StackDcptStubPtr->ScopedPduRequstBuf,++Snmpv3StackDcptStubPtr->ScopedPduDataPos)) )
        return false;

   tempLen = SNMPv3GetProcessBuffData(Snmpv3StackDcptStubPtr->ScopedPduRequstBuf,++Snmpv3StackDcptStubPtr->ScopedPduDataPos);

    // Make sure that OID length is within our capability.
    if (tempLen > (uint8_t)OID_MAX_LEN )
        return false;

    *len = tempLen;

	while( tempLen-- )
	{
       *oid++ = SNMPv3GetProcessBuffData(Snmpv3StackDcptStubPtr->ScopedPduRequstBuf,++Snmpv3StackDcptStubPtr->ScopedPduDataPos);
	}
	*oid=0xff;
    return true;
}

/****************************************************************************
  Function:
	static bool SNMPv3CheckIfv3ASNNull(void)
	
  Summary:
  	Verifies the value type as ASN_NULL.

  Description:
  	For Get,Get_Next,Get_Bulk snmp reuest, the var bind the value data type 
  	should be ASN_NULL and value field must be NULL and . This routine
  	verifies the data type and value fields in the received requests.
  	The SET request, the value data type can not be ASN_NULL,
  	otherwise the snmp request is not processed.
  	This rotuine only refers to the incoming snmpv3 request dynamically 
	allocated 	memory buffer 'ScopedPduRequstBuf' .

  Precondition:
	None
	
  Parameters:
  	None
 
  Returns Values
	true	- If value type is ASN_NULL and value is NULL. 
	false	- If data type and value is other than ASN_NULL and NULL resp.
	
  Remarks:
	None.
***************************************************************************/
static bool SNMPv3CheckIfv3ASNNull(void)
{
	uint8_t a;

	a = SNMPv3GetProcessBuffData(Snmpv3StackDcptStubPtr->ScopedPduRequstBuf,++Snmpv3StackDcptStubPtr->ScopedPduDataPos);

	if (!IS_ASN_NULL(a))
			return false;

    // Fetch and verify that length value is zero.
    return (SNMPv3GetProcessBuffData(Snmpv3StackDcptStubPtr->ScopedPduRequstBuf,++Snmpv3StackDcptStubPtr->ScopedPduDataPos) == 0u );
}


/****************************************************************************
  Function:
	static void SNMPv3SetErrorStatus(uint16_t errorStatusOffset,
                           uint16_t errorIndexOffset,
                           SNMP_ERR_STATUS errorStatus,
                           uint8_t errorIndex,SNMPV3MSGDATA *dynScopedPduPutBuf)
  Summary:
  	Set snmpv3 error status in the response pdu. 
  	
  Description:
  	This routine processes the received snmp Get request pdu for the 
  	variable binding in the request and also creates the response pdu.
  	
  Precondition:
	SNMPProcessVariables() is called.	
	
  Parameters:
    errorStatusOffset - Offset to update error status in Response Tx pdu 	
    errorIndexOffset  - Offset to update error index
    errorStatus		  - Snmp process error to be updated in response.	
    errorIndex		  - Index of the request varbind in the var bind list 
    					for which error status is to be updated. 					
    dynScopedPduPutBuf -  dynamic snmpv3 scoped pdu buffer
  Returns:
	None.
	
  Remarks:
  	None.
***************************************************************************/
static void SNMPv3SetErrorStatus(uint16_t errorStatusOffset,
                           uint16_t errorIndexOffset,
                           SNMP_ERR_STATUS errorStatus,
                           uint8_t errorIndex,SNMPV3MSGDATA *dynScopedPduPutBuf)
{
    uint16_t prevOffset;

    prevOffset = dynScopedPduPutBuf->length;
	dynScopedPduPutBuf->length = errorStatusOffset;
    SNMPv3CopyDataToProcessBuff(errorStatus,dynScopedPduPutBuf);

    
	dynScopedPduPutBuf->length = errorIndexOffset;
    SNMPv3CopyDataToProcessBuff(errorIndex,dynScopedPduPutBuf);

	dynScopedPduPutBuf->length = prevOffset;
}

/****************************************************************************
  Function:
	static uint8_t SNMPv3CheckIfValidAuthStructure(uint16_t* dataLen)
	
  Summary:
  	Decode variable length structure.

  Description:
	This routine is used  to verify whether the received varbind is of type
	STRUCTURE and to find out the variable binding structure length.
  	This rotuine only refers to the incoming snmpv3 request dynamically 
	allocated   memory buffer 'InPduWholeMsgBuf' .

  Precondition:
	SNMPProcessHeader() is called.	
	
  Parameters:
  	datalen	- Pointer to memory to store OID structure length.
 
  Return Values:
	headrbytes	- Variable binding length.
	false		- If variable data structure is not type STRUCTURE. 	

  Remarks:
	None.
***************************************************************************/

static uint8_t SNMPv3CheckIfValidAuthStructure(uint16_t* dataLen)
{
    TCPIP_UINT16_VAL tempLen;
    uint8_t headerBytes;
	uint8_t authStructure=0;
	uint16_t tempPos;
	
	uint8_t tempData;
	
	tempPos=Snmpv3StackDcptStubPtr->SnmpMsgBufSeekPos;
	authStructure = SNMPv3GetWholeMsgBufferData(Snmpv3StackDcptStubPtr->InPduWholeMsgBuf.snmpMsgHead,&tempPos);
	

    if ( !IS_STRUCTURE(authStructure) && !IS_SNMPV3_AUTH_STRUCTURE(authStructure) )
        return false;

	
    // Initialize length value.
    tempLen.Val = 0;
    headerBytes = 0;

    tempData = SNMPv3GetWholeMsgBufferData(Snmpv3StackDcptStubPtr->InPduWholeMsgBuf.snmpMsgHead,&tempPos);
    tempLen.v[0] = tempData;
    if ( tempData & 0x80 )
    {
        tempData &= 0x7F;

        // We do not support any length byte count of more than 2
        // i.e. total length value must not be more than 16-bit.
        if ( tempData > 2u )
            return false;

        // Total length bytes are 0x80 itself plus tempData.
        headerBytes = tempData + 1;

        // Get upto 2 bytes of length value.
        while( tempData-- )
            tempLen.v[tempData] = SNMPv3GetWholeMsgBufferData(Snmpv3StackDcptStubPtr->InPduWholeMsgBuf.snmpMsgHead,&tempPos);
    }
    else
        headerBytes = 1;

	 if ( !headerBytes )
      return false;

	Snmpv3StackDcptStubPtr->InPduWholeMsgBuf.scopedPduAuthStructVal=authStructure;
	Snmpv3StackDcptStubPtr->InPduWholeMsgBuf.scopedPduStructLen=tempLen.Val;
	
    headerBytes++;

    // Since we are using UDP as our transport and UDP are not fragmented,
    // this structure length cannot be more than 1500 bytes.
    // As a result, we will only use lower uint16_t of length value.
    *dataLen = tempLen.Val;

	Snmpv3StackDcptStubPtr->SnmpMsgBufSeekPos=tempPos;
    return headerBytes;
}


#if defined(SNMP_V1_V2_TRAP_WITH_SNMPV3)&& !defined(SNMP_TRAP_DISABLED)
static uint8_t gSNMPV3TrapSecurityLevel = NO_REPORT_NO_PRIVACY_NO_AUTH;
#define INVALID_INDEX 0xFF

/****************************************************************************
  Function:
	uint8_t SNMPv3GetUserIndxFromUsmUserDB(uint8_t targetIndex)
	
  Summary:
  	Routine to find the index of the user name in the user data base table. 
  	
  Description:
  	There are two different data base tables defined with SNMPv3 stack,
  	like 'UserInfoDataBase' and 'Snmpv3TrapConfigData'.
	It returns the index of the user name which matches to the trap target 
	user name within the user data base. 

  Precondition:
	Trap notification event is triggred and the trap send flag is enabled.
	
  Parameters:
  	targetIndex -index of the 'Snmpv3TrapConfigData' table to match the 
  			     'userSecurityName' with the user data base  
 
  Return Values:
	INVALID_INDEX - if the trap target user name does not match.
	uint8_t - Byte value fo the index matched

  Remarks:
	None.
***************************************************************************/
uint8_t SNMPv3GetUserIndxFromUsmUserDB(uint8_t targetIndex)
{
	uint8_t *userSecurityName=NULL;
	uint8_t userDBsecurityLevel=0;
	uint8_t trapSecurityLevel=0; 
	uint8_t userTrapSecLen=0;
	uint8_t *userTrapSecurityName=NULL;
	uint8_t i=0;

	trapSecurityLevel = SNMPv3GetTrapSecurityLevel((STD_BASED_SNMPV3_SECURITY_LEVEL)Snmpv3StackDcptStubPtr->Snmpv3TrapConfigData[targetIndex].securityLevelType);
	if(trapSecurityLevel == INVALID_MSG)
		return INVALID_INDEX;

	for(i=0;i<SNMPV3_USM_MAX_USER;i++)
	{
		userSecurityName = Snmpv3StackDcptStubPtr->UserInfoDataBase[i].userName;
		userDBsecurityLevel = SNMPv3GetSecurityLevel(i);
		userTrapSecLen = strlen((char *)Snmpv3StackDcptStubPtr->Snmpv3TrapConfigData[targetIndex].userSecurityName);
		userTrapSecurityName = Snmpv3StackDcptStubPtr->Snmpv3TrapConfigData[targetIndex].userSecurityName;
		
		if(userTrapSecLen != Snmpv3StackDcptStubPtr->UserInfoDataBase[i].userNameLength)
			continue;
		if(strncmp((char *)userTrapSecurityName,(char *)userSecurityName,userTrapSecLen) == 0)
		{
			if(trapSecurityLevel == userDBsecurityLevel)
				return i;
			else
				continue;
		}
	}

	return INVALID_INDEX;
}


/****************************************************************************
  Function:
	bool SNMPv3CmprTrapSecNameAndSecLvlWithUSMDb(
				uint8_t tragetIndex,
				uint8_t userTrapSecLen,
				uint8_t *userTrapSecurityName,
				STD_BASED_SNMPV3_SECURITY_LEVEL securityLevel)
	
  Summary:
  	Routine to find the index of the user name in the user data base table. 
  	
  Description:
  	There are two different data base tables defined with SNMPv3 stack,
  	like 'UserInfoDataBase' and 'Snmpv3TrapConfigData'.
	This routine is used to validte the trap user security level setting with
	SET request.
	

  Precondition:
	SET operation would be allowed if the USM security conditions and
	user security name in the request is matched to one of the user security 
	name stored in the usm user database.
	
  Parameters:
  	targetIndex -index of the 'Snmpv3TrapConfigData' table to match the 
  			     'userSecurityName' with the user data base  
	userTrapSecLen - user sec name length in the SET request
	userTrapSecurityName - pointer to user sec name in the SET request
	securityLevel - trap security level to be SET on the agent
 
  Return Values:
	true - if the trap target user sec level setting is successful
	FLASE - If the SET failed due to non matching of the security parameters

  Remarks:
	None.
***************************************************************************/
bool SNMPv3CmprTrapSecNameAndSecLvlWithUSMDb(uint8_t tragetIndex,uint8_t userTrapSecLen,
								uint8_t *userTrapSecurityName,STD_BASED_SNMPV3_SECURITY_LEVEL securityLevel)
{

	uint8_t *userSecurityName=NULL;
	uint8_t userDBsecurityLevel=0;
	uint8_t trapSecurityLevel=0; 
	uint8_t i=0;

	trapSecurityLevel = SNMPv3GetTrapSecurityLevel(securityLevel);
	if(trapSecurityLevel == INVALID_MSG)
		return false;

	for(i=0;i<SNMPV3_USM_MAX_USER;i++)
	{
		userSecurityName = Snmpv3StackDcptStubPtr->UserInfoDataBase[i].userName;
		userDBsecurityLevel = SNMPv3GetSecurityLevel(i);
		if(userTrapSecLen != Snmpv3StackDcptStubPtr->UserInfoDataBase[i].userNameLength)
			continue;
		if(strncmp((char *)userTrapSecurityName,(char *)userSecurityName,userTrapSecLen) == 0)
		{
			if(trapSecurityLevel == userDBsecurityLevel)
				return true;
			else
				continue;
		}
	}

	return false;
}


/****************************************************************************
  Function:
	uint8_t SNMPv3GetTrapSecurityLevel(STD_BASED_SNMPV3_SECURITY_LEVEL securityLevel)
	
  Summary:
  	Routine to find the report, auth and privacy flags settings in the TRAP. 
  	
  Description:
  	This routine to find the report, auth and privacy flags setting for the trap to be 
  	generated. The message flags octet's least significant three bits: 
  	Reportable, PrivFlag, AuthFlag forms different secuity level combinations.
	

  Precondition:
	None
	
  Parameters:
  	securityLevel -trap security level to be compared for getting the agent's security 
  				level settings 
  Return Values:
	NO_REPORT_NO_PRIVACY_NO_AUTH - No authentication, no encryption
	NO_REPORT_NO_PRIVACY_BUT_AUTH_PROVIDED - authentication but no encryption
  	NO_REPORT_PRIVACY_AND_AUTH_PROVIDED - authentication and encryption 
  	INVALID_MSG - if security level doesn't match any of the above

  Remarks:
	None.
***************************************************************************/
uint8_t SNMPv3GetTrapSecurityLevel(STD_BASED_SNMPV3_SECURITY_LEVEL securityLevel)
{
	uint8_t tempSecurityLevel=0xFF;
	
	switch(securityLevel)
	{
		case NO_AUTH_NO_PRIV:
			tempSecurityLevel =  NO_REPORT_NO_PRIVACY_NO_AUTH;
			break;
		case AUTH_NO_PRIV:
			tempSecurityLevel = NO_REPORT_NO_PRIVACY_BUT_AUTH_PROVIDED;
			break;
		case AUTH_PRIV:
			tempSecurityLevel = NO_REPORT_PRIVACY_AND_AUTH_PROVIDED;
			break;
		default:
			return INVALID_MSG;
	}
	return tempSecurityLevel;
}

/****************************************************************************
  Function:
	bool SNMPv3TrapMsgHeaderPDU(unsigned int targetIndex)
	
  Summary:
  	TRAP PDU message header construction. 
  	
  Description:
  	This routine forms the message header for the SNMPv3 trap PDU 
  	to be originated from this agent. 
  	
  Precondition:
	TRAP event is triggered.
	
  Parameters:
  	targetIndex -index of the 'Snmpv3TrapConfigData' table's security user name 
  			     for which the TRAP PDU message header to constructed.
  			     
  Return Values:
	INVALID_INDEX - if the 'targetIndex' does not match to any of the users configured 
					with the agent in 'Snmpv3TrapConfigData'.
	true - The trap message header generation is successful.
	false -The trap message header generation failed.

  Remarks:
	None.
***************************************************************************/
bool SNMPv3TrapMsgHeaderPDU(unsigned int targetIndex)
{
	uint8_t putCntr=0;	
	uint8_t *ptr=NULL;
	bool retBuf=true;
	uint8_t snmpv3MsgGlobalHeaderlength=0;
	uint16_t snmpv3Headerlength=0;
	uint16_t snmpv3MsgAuthHeaderLength=0;
	uint8_t tempData=0;
	uint16_t msgHeaderOffset1=0;
	uint16_t msgHeaderOffset2=0;
	uint16_t tempMsgHeaderOffset=0;
	uint8_t  USM_Index=0,temp_index=0;

	
	SNMP_PROCESSING_MEM_INFO_PTRS snmpPktProcsMemPtrsInfo; 
	SNMPGetPktProcessingDynMemStubPtrs(&snmpPktProcsMemPtrsInfo);
	
	temp_index = Snmpv3StackDcptStubPtr->SecurtyPrimtvesOfIncmngPdu.securityNameLength ;
	USM_Index = SNMPv3GetUserIndxFromUsmUserDB(targetIndex);
	if(USM_Index != INVALID_INDEX)
	{
		Snmpv3StackDcptStubPtr->SecurtyPrimtvesOfIncmngPdu.securityNameLength = strlen((char *)Snmpv3StackDcptStubPtr->Snmpv3TrapConfigData[targetIndex].userSecurityName);
	}
	snmpv3Headerlength = MSGGLOBAL_HEADER_LEN(snmpv3MsgGlobalHeaderlength)+
						  MSG_AUTHORITATIVE_HEADER_LEN(snmpv3MsgAuthHeaderLength);

	// update the IN pdu trap security name size
	Snmpv3StackDcptStubPtr->SecurtyPrimtvesOfIncmngPdu.securityNameLength = temp_index;	
	
	ptr = Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf.head = 
		(uint8_t *)(TCPIP_HEAP_Calloc(snmpPktProcsMemPtrsInfo.snmpHeapMemHandler,1,(size_t)snmpv3Headerlength+1));
	//ptr = Snmpv3StackDcptStubPtr->PduHeaderBuf.head = (uint8_t *)(malloc(0x1));
	if(ptr == NULL)
		return false;
	
	Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf.length = 0;
	Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf.maxlength = snmpv3Headerlength+1;

	/*Msg Processing Model PDU header */
	/*ID + Msg Size + Msg Flag + Security Model */
	//message header
	SNMPv3CopyDataToProcessBuff(STRUCTURE,&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf);
	SNMPv3CopyDataToProcessBuff(MSGGLOBAL_HEADER_LEN(snmpv3MsgGlobalHeaderlength)-2,&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf);

	//Put "msgID" type ASN_INT of length 4 bytes	
	SNMPv3CopyDataToProcessBuff(ASN_INT,&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf);		
	SNMPv3CopyDataToProcessBuff(0x04,&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf);
	SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->IncmngSnmpPduMsgID.v[3],&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf);
	SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->IncmngSnmpPduMsgID.v[2],&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf);
	SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->IncmngSnmpPduMsgID.v[1],&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf);
	SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->IncmngSnmpPduMsgID.v[0],&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf);

	//Put "msgMaxSize"  type ASN_INT of length 4 bytes
	SNMPv3CopyDataToProcessBuff(ASN_INT,&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf);
	SNMPv3CopyDataToProcessBuff(0x04,&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf);
	SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->SnmpEngnMaxMsgSize.v[3],&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf);
	SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->SnmpEngnMaxMsgSize.v[2],&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf);
	SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->SnmpEngnMaxMsgSize.v[1],&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf);
	SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->SnmpEngnMaxMsgSize.v[0],&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf);

	//Put "msgFlags"  type octet_string 
	SNMPv3CopyDataToProcessBuff(OCTET_STRING,&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf);
	SNMPv3CopyDataToProcessBuff(0x01,&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf);
	SNMPv3CopyDataToProcessBuff(gSNMPV3TrapSecurityLevel&0x03,&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf);//Rsponse "msgFlags" value as Reportable, Encrypted and Authenticated Bits are not set. 

	//Put "msgSecurityModel"	
	SNMPv3CopyDataToProcessBuff(ASN_INT,&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf);
	SNMPv3CopyDataToProcessBuff(0x01,&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf);
	SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->Snmpv3TrapConfigData[targetIndex].securityModelType,&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf);



	/*User Security Module pdu header 
	Authoritative Engin ID + Authoritative Boots + Authoritative Engine Time+ 
	User name + Authentication parameters + Privacy Parameter 
	*/
	SNMPv3CopyDataToProcessBuff(OCTET_STRING,&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf);  //Security Parameter string		
	msgHeaderOffset1 = Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf.length;
	SNMPv3CopyDataToProcessBuff(MSG_AUTHORITATIVE_HEADER_LEN(snmpv3MsgAuthHeaderLength)-2,&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf);
	SNMPv3CopyDataToProcessBuff(STRUCTURE,&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf);
	msgHeaderOffset2 = Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf.length;
	SNMPv3CopyDataToProcessBuff(MSG_AUTHORITATIVE_HEADER_LEN(snmpv3MsgAuthHeaderLength)-4,&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf);
	
	//Put "msgAuthoritiveEngineID"	
	SNMPv3CopyDataToProcessBuff(OCTET_STRING,&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf);	
	SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->SnmpEngnIDLength,&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf); //Integer Length
	putCntr = 0;
	for(;putCntr<Snmpv3StackDcptStubPtr->SnmpEngnIDLength;putCntr++)
	{
		SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->SnmpEngineID[putCntr],&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf);
	}

	//Put "msgAuthoritiveEngineBoots" 
	SNMPv3CopyDataToProcessBuff(ASN_INT,&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf);	
	SNMPv3CopyDataToProcessBuff(0x04,&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf);
	SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->SnmpEngineBoots>>24,&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf);
	SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->SnmpEngineBoots>>16,&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf);
	SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->SnmpEngineBoots>>8,&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf);
	SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->SnmpEngineBoots,&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf);
	
	//Put "msgAuthoritiveEngineTime" 
	SNMPv3GetAuthEngineTime();
	SNMPv3CopyDataToProcessBuff(ASN_INT,&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf);	
	SNMPv3CopyDataToProcessBuff(0x04,&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf); 
	SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->SnmpEngineTime.v[3],&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf);
	SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->SnmpEngineTime.v[2],&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf);
	SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->SnmpEngineTime.v[1],&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf);
	SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->SnmpEngineTime.v[0],&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf);
	
	putCntr = 0;

	//Put "msgUserName" 
	SNMPv3CopyDataToProcessBuff(OCTET_STRING,&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf);	
	tempData = strlen((char *)Snmpv3StackDcptStubPtr->Snmpv3TrapConfigData[targetIndex].userSecurityName);
	SNMPv3CopyDataToProcessBuff(tempData,&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf);	
	if(tempData != 0)
	{
		ptr= Snmpv3StackDcptStubPtr->Snmpv3TrapConfigData[targetIndex].userSecurityName;

		for(;putCntr<tempData;putCntr++)
		{
			SNMPv3CopyDataToProcessBuff(ptr[putCntr],&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf);
		}
	}
	

	putCntr = 0;
	
	SNMPv3UsmOutMsgAuthParam(Snmpv3StackDcptStubPtr->UserInfoDataBase[USM_Index].userHashType);
	//SNMPv3UsmOutMsgAuthParam(SNMPV3_HAMC_MD5);

	//Put "msgAuthenticationParameters" 
	SNMPv3CopyDataToProcessBuff(OCTET_STRING,&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf);
	if((gSNMPV3TrapSecurityLevel &0x01) == 0x01)
	{
		SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->SnmpOutMsgAuthParmLen,&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf); //Not supported with the Alpha Release.

		Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf.msgAuthParamOffset=Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf.length;

		for(;putCntr<Snmpv3StackDcptStubPtr->SnmpOutMsgAuthParmLen;putCntr++)
			SNMPv3CopyDataToProcessBuff(0x0,&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf);
	}
	else
	{
		SNMPv3CopyDataToProcessBuff(0x0,&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf); //Not supported with the Alpha Release.
	}
	
	putCntr = 0;
	SNMPv3USMOutMsgPrivParam();
	
	//Put "msgPrivacyParameters" 
	SNMPv3CopyDataToProcessBuff(OCTET_STRING,&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf); 
	if((gSNMPV3TrapSecurityLevel&0x02) == 0x02)
	{
		SNMPv3USMOutMsgPrivParam();
		SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->SnmpOutMsgPrivParmLen,&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf); //Not supported with the Alpha Release
		for(;putCntr<Snmpv3StackDcptStubPtr->SnmpOutMsgPrivParmLen;putCntr++)
			retBuf = SNMPv3CopyDataToProcessBuff(Snmpv3StackDcptStubPtr->SnmpOutMsgPrvParmStrng[putCntr],&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf);
	}
	else
	{
		SNMPv3CopyDataToProcessBuff(0x0,&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf); //Not supported with the Alpha Release.
	}


	tempMsgHeaderOffset = Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf.length;
	Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf.length = msgHeaderOffset2;
	SNMPv3CopyDataToProcessBuff((tempMsgHeaderOffset-msgHeaderOffset2)-1,&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf); 
	Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf.length = tempMsgHeaderOffset;
	
	tempMsgHeaderOffset = Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf.length;
	Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf.length = msgHeaderOffset1;
	SNMPv3CopyDataToProcessBuff((tempMsgHeaderOffset-msgHeaderOffset1)-1,&Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf);
	Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf.length = tempMsgHeaderOffset;
	
	return true;
}


/****************************************************************************
  Function:
	uint8_t SNMPv3TrapScopedpdu(SNMP_ID var, SNMP_VAL val, SNMP_INDEX index,
							   uint8_t targetIndex)
	
  Summary:
  	TRAP PDU scoped pdu header construction. 
  	
  Description:
  	This routine forms the trap scoped pdu header for the SNMPv3 trap PDU 
  	to be originated from this agent. Scoped pdu comprises of 
  	msg data : - <contextEngineID><context name>[<data> == <pdutype><request id>
	<error status><error index><varbinds>
  	
  Precondition:
	TRAP event is triggered.
	
  Parameters:
  	var - var id of the variable whose value to be sent in the trap pdu
  	val - value of the variable
  	index - index of the variable in the multiple variable bind scenario
  	targetIndex -index of the 'Snmpv3TrapConfigData' table's security user name 
  			     for which the TRAP PDU message header to constructed.
 
   Return Values:
	true - The trap scoped pdu header generation is successful.
	false -The trap scoped pdu header generation failed.

  Remarks:
	None.
***************************************************************************/

uint8_t SNMPv3TrapScopedpdu(SNMP_ID var, SNMP_VAL val, SNMP_INDEX index,uint8_t targetIndex)
{	
	uint8_t 			*ptr=NULL;
	uint8_t			contextName[]="";
	uint8_t			contextEngId[]="";
	uint8_t			count=0;
	uint8_t			OIDLen=0; 
    uint8_t 			len=0;
    uint8_t 			OIDValue[OID_MAX_LEN];
#ifdef SNMP_STACK_USE_V2_TRAP
	uint8_t 			snmptrap_oids[]  = {0x2b,6,1,6,3,1,1,4,1 }; /* len=10 */
	uint8_t			sysUpTime_oids[] = {0x2b,6,1,2,1,1,3}; /* len = 8 */
	int 			i=0;
#endif
	uint16_t			contextIDlen=0;
	uint16_t			contextNameLength=0;
	uint16_t			snmpv3Headerlength=0,snmpv3MsgGlobalHeaderlength=0;
	uint16_t			snmpv3MsgAuthHeaderLength=0;
	SNMPV3MSGDATA	*dynTrapScopedPduBuf;
	static uint16_t		pduStructLenOffset=0;
	uint16_t			varPairStructLenOffset=0;
	static uint16_t		varBindStructLenOffset=0;
	uint16_t			tempOffset=0;
    OID_INFO		rec;
	SNMP_DATA_TYPE_INFO	dataTypeInfo;
	TCPIP_UINT16_VAL		varBindLen = {0};
	uint8_t  			USM_Index=0,temp_index=0;
	

	SNMP_PROCESSING_MEM_INFO_PTRS snmpPktProcsMemPtrsInfo; 
	SNMP_STACK_DCPT_STUB * snmpStkDcptMemStubPtr=0;		
	
	SNMPGetPktProcessingDynMemStubPtrs(&snmpPktProcsMemPtrsInfo);
	snmpStkDcptMemStubPtr=snmpPktProcsMemPtrsInfo.snmpStkDynMemStubPtr;
	
	if(Snmpv3StackDcptStubPtr->TrapScopdPduRespnsBuf.head == NULL)
	{
		
		temp_index = Snmpv3StackDcptStubPtr->SecurtyPrimtvesOfIncmngPdu.securityNameLength ;
		USM_Index = SNMPv3GetUserIndxFromUsmUserDB(targetIndex);
		if(USM_Index != INVALID_INDEX)
		{
			Snmpv3StackDcptStubPtr->SecurtyPrimtvesOfIncmngPdu.securityNameLength = strlen((char *)Snmpv3StackDcptStubPtr->Snmpv3TrapConfigData[targetIndex].userSecurityName);
		}
		snmpv3Headerlength = MSGGLOBAL_HEADER_LEN(snmpv3MsgGlobalHeaderlength)+
									  MSG_AUTHORITATIVE_HEADER_LEN(snmpv3MsgAuthHeaderLength);
		// update the IN pdu trap security name size
		Snmpv3StackDcptStubPtr->SecurtyPrimtvesOfIncmngPdu.securityNameLength = temp_index;
		
		ptr = Snmpv3StackDcptStubPtr->TrapScopdPduRespnsBuf.head = 
			(uint8_t*)(TCPIP_HEAP_Calloc(snmpPktProcsMemPtrsInfo.snmpHeapMemHandler,1,(size_t)(SNMP_MAX_MSG_SIZE - snmpv3Headerlength)+1));
		if(ptr == NULL)
		{
			return SNMP_ACTION_UNKNOWN;
		}
		Snmpv3StackDcptStubPtr->TrapScopdPduRespnsBuf.length = 0;
		Snmpv3StackDcptStubPtr->TrapScopdPduRespnsBuf.maxlength = (SNMP_MAX_MSG_SIZE - snmpv3Headerlength)+1;

		 //Start collecting the plaint text Scoped PDU data byte from the incoming PDU.
		 //Check if the plain text scoped pdu data bytes are binded in ASN structure format 

		dynTrapScopedPduBuf = &Snmpv3StackDcptStubPtr->TrapScopdPduRespnsBuf;
		SNMPv3CopyDataToProcessBuff(STRUCTURE,dynTrapScopedPduBuf); // First item to Response buffer is packet structure
		SNMPv3CopyDataToProcessBuff(0x82,dynTrapScopedPduBuf); 
		SNMPv3CopyDataToProcessBuff(0,dynTrapScopedPduBuf);
		SNMPv3CopyDataToProcessBuff(0,dynTrapScopedPduBuf);

		//Collect context engine id
		SNMPv3CopyDataToProcessBuff(OCTET_STRING,dynTrapScopedPduBuf);
		// populate context Engine id to contextEngId
		contextIDlen = strlen((char*)contextEngId);
		if(contextIDlen == 0)
		{			
			SNMPv3CopyDataToProcessBuff(0,dynTrapScopedPduBuf);
		}
		else
		{
			//copy context engine id from a local buffer			
			SNMPv3CopyDataToProcessBuff(contextIDlen,dynTrapScopedPduBuf);
			while(contextIDlen--)
				SNMPv3CopyDataToProcessBuff(contextEngId[count++],dynTrapScopedPduBuf);
		}

		//Check and collect "contextName" 
		SNMPv3CopyDataToProcessBuff(OCTET_STRING,dynTrapScopedPduBuf);
		contextNameLength = strlen((char*)contextName);
		count = 0;
		if(contextNameLength == 0)
		{
			SNMPv3CopyDataToProcessBuff(0,dynTrapScopedPduBuf);
		}
		else
		{
			SNMPv3CopyDataToProcessBuff(contextNameLength,dynTrapScopedPduBuf);
			while(contextNameLength--)
				SNMPv3CopyDataToProcessBuff(contextName[count++],dynTrapScopedPduBuf);
		}	


#ifdef SNMP_STACK_USE_V2_TRAP
		//TRAP Version type.  
		SNMPv3CopyDataToProcessBuff(SNMP_V2_TRAP,dynTrapScopedPduBuf);
		pduStructLenOffset = dynTrapScopedPduBuf->length;
		SNMPv3CopyDataToProcessBuff(0x82,dynTrapScopedPduBuf);
		SNMPv3CopyDataToProcessBuff(0,dynTrapScopedPduBuf);
		SNMPv3CopyDataToProcessBuff(0,dynTrapScopedPduBuf);

		//put Request ID for the trapv2 as 1 
		SNMPv3CopyDataToProcessBuff(ASN_INT,dynTrapScopedPduBuf);// To simplify logic, always use 4 byte long requestID
		SNMPv3CopyDataToProcessBuff(4,dynTrapScopedPduBuf);
		SNMPv3CopyDataToProcessBuff(0,dynTrapScopedPduBuf);
		SNMPv3CopyDataToProcessBuff(0,dynTrapScopedPduBuf);
		SNMPv3CopyDataToProcessBuff(0,dynTrapScopedPduBuf);
		SNMPv3CopyDataToProcessBuff(1,dynTrapScopedPduBuf);

		// Put error status.
		SNMPv3CopyDataToProcessBuff(ASN_INT,dynTrapScopedPduBuf);// Int type
		SNMPv3CopyDataToProcessBuff(1,dynTrapScopedPduBuf); // One byte long.
		SNMPv3CopyDataToProcessBuff(0,dynTrapScopedPduBuf); // Placeholder.

		// Similarly put error index.
		SNMPv3CopyDataToProcessBuff(ASN_INT,dynTrapScopedPduBuf);// Int type
		SNMPv3CopyDataToProcessBuff(1,dynTrapScopedPduBuf); // One byte long.
		SNMPv3CopyDataToProcessBuff(0,dynTrapScopedPduBuf); // Placeholder.

		// Variable binding structure header
		SNMPv3CopyDataToProcessBuff(STRUCTURE,dynTrapScopedPduBuf);
		varBindStructLenOffset = dynTrapScopedPduBuf->length;
		SNMPv3CopyDataToProcessBuff(0x82,dynTrapScopedPduBuf);
		SNMPv3CopyDataToProcessBuff(0,dynTrapScopedPduBuf);
		SNMPv3CopyDataToProcessBuff(0,dynTrapScopedPduBuf);

		// Create variable name-pair structure
		SNMPv3CopyDataToProcessBuff(STRUCTURE,dynTrapScopedPduBuf);
		varPairStructLenOffset = dynTrapScopedPduBuf->length;
		SNMPv3CopyDataToProcessBuff(0,dynTrapScopedPduBuf);

		// Set 1st varbind object i,e sysUpTime.0 time stamp for the snmpv2 trap
		// Get complete notification variable OID string.

		SNMPv3CopyDataToProcessBuff(ASN_OID,dynTrapScopedPduBuf);
		OIDLen = (uint8_t)sizeof(sysUpTime_oids);
		SNMPv3CopyDataToProcessBuff((uint8_t)(OIDLen)+1,dynTrapScopedPduBuf);
		ptr = sysUpTime_oids;
		while( OIDLen-- )
			SNMPv3CopyDataToProcessBuff(*ptr++,dynTrapScopedPduBuf);

		//1st varbind	 and this is a scalar object so index = 0
		SNMPv3CopyDataToProcessBuff(0,dynTrapScopedPduBuf);

		// Time stamp
		SNMPv3CopyDataToProcessBuff(SNMP_TIME_TICKS,dynTrapScopedPduBuf);
		SNMPv3CopyDataToProcessBuff(4,dynTrapScopedPduBuf);
		SNMPv3CopyDataToProcessBuff(snmpStkDcptMemStubPtr->SNMPNotifyInfo.timestamp.v[3],dynTrapScopedPduBuf);
		SNMPv3CopyDataToProcessBuff(snmpStkDcptMemStubPtr->SNMPNotifyInfo.timestamp.v[2],dynTrapScopedPduBuf);
		SNMPv3CopyDataToProcessBuff(snmpStkDcptMemStubPtr->SNMPNotifyInfo.timestamp.v[1],dynTrapScopedPduBuf);
		SNMPv3CopyDataToProcessBuff(snmpStkDcptMemStubPtr->SNMPNotifyInfo.timestamp.v[0],dynTrapScopedPduBuf);

		tempOffset = dynTrapScopedPduBuf->length;
		//set the snmp time varbind trap offset 
		dynTrapScopedPduBuf->length = varPairStructLenOffset;

		/*// SNMP time stamp varbind length
		OIDLen = 2							// 1st varbind header 
		   + (uint8_t)sizeof(sysUpTime_oids)
		   + 1						   // index byte
		   + 6 ;						// time stamp */
		   
		SNMPv3CopyDataToProcessBuff((tempOffset-varPairStructLenOffset)-1,dynTrapScopedPduBuf);
		//set the previous TX offset
		dynTrapScopedPduBuf->length = tempOffset;
					
		// Set 2nd varbind object i,e snmpTrapOID.0 for the snmpv2 trap
		// Get complete notification variable OID string.

		// Create variable name-pair structure
		SNMPv3CopyDataToProcessBuff(STRUCTURE,dynTrapScopedPduBuf);
		varPairStructLenOffset = dynTrapScopedPduBuf->length;
		SNMPv3CopyDataToProcessBuff(0,dynTrapScopedPduBuf);

		// Copy OID string into PDU.
		SNMPv3CopyDataToProcessBuff(ASN_OID,dynTrapScopedPduBuf);
		OIDLen = (uint8_t)sizeof(snmptrap_oids);
		SNMPv3CopyDataToProcessBuff((uint8_t)(OIDLen)+1,dynTrapScopedPduBuf);

		ptr = snmptrap_oids;
		while( OIDLen-- )
		SNMPv3CopyDataToProcessBuff(*ptr++,dynTrapScopedPduBuf);

		//2nd varbind  and this is a scalar object so index = 0
		SNMPv3CopyDataToProcessBuff(0,dynTrapScopedPduBuf);

		if (!SNMPFindOIDStringByID(snmpStkDcptMemStubPtr->SNMPNotifyInfo.trapIDVar, &rec, OIDValue, &OIDLen) )
		{
			SYS_FS_close(Snmpv3FileDescrptr);
			UDPClose(snmpStkDcptMemStubPtr->SNMPNotifyInfo.socket);
			return false;
		}
		SNMPv3CopyDataToProcessBuff(ASN_OID,dynTrapScopedPduBuf);
		len = OIDLen;
		SNMPv3CopyDataToProcessBuff(OIDLen,dynTrapScopedPduBuf);
		for(i=0;i<len;i++)
		{
			SNMPv3CopyDataToProcessBuff(OIDValue[i],dynTrapScopedPduBuf);
		}
		tempOffset = dynTrapScopedPduBuf->length;
		//set the snmp varbind trap offset
		dynTrapScopedPduBuf->length = varPairStructLenOffset;
		// Snmp trap varbind length 
		/*OIDLen = 2					 // Agent ID header bytes
			+ (uint8_t)sizeof(snmptrap_oids)
			+ 1 					   // index byte
			+ 2 					 // header
			+ agentIDLen;				 // Agent ID bytes				  */
		SNMPv3CopyDataToProcessBuff((tempOffset-varPairStructLenOffset)-1,dynTrapScopedPduBuf);
		//set the previous TX offset
		dynTrapScopedPduBuf->length = tempOffset;
#else
		// Put PDU type.  SNMP agent's response is always GET RESPONSE
		SNMPv3CopyDataToProcessBuff(TRAP,dynTrapScopedPduBuf);
		pduStructLenOffset = dynTrapScopedPduBuf->length;
		SNMPv3CopyDataToProcessBuff(0,dynTrapScopedPduBuf);
	
		// Get complete OID string from snmp.bib.
		if(!SNMPFindOIDStringByID(snmpStkDcptMemStubPtr->SNMPNotifyInfo.agentIDVar,&rec, OIDValue, &OIDLen))
		{
			return false;
		}
	
		if(!rec.nodeInfo.Flags.bIsAgentID )
		{
			return false;
		}


		SYS_FS_lseek(Snmpv3FileDescrptr, rec.hData, SEEK_SET);

		
		SNMPv3CopyDataToProcessBuff(ASN_OID,dynTrapScopedPduBuf);
		SYS_FS_read(Snmpv3FileDescrptr,&len,1);

		OIDLen = len;
		SNMPv3CopyDataToProcessBuff(len,dynTrapScopedPduBuf);
		while( len-- )
		{
			uint8_t c;
			SYS_FS_read(Snmpv3FileDescrptr,&c,1);

			SNMPv3CopyDataToProcessBuff(c,dynTrapScopedPduBuf);
		}
	
		// This agent's IP address.
		SNMPv3CopyDataToProcessBuff(SNMP_IP_ADDR,dynTrapScopedPduBuf);
		SNMPv3CopyDataToProcessBuff(4,dynTrapScopedPduBuf);
		SNMPv3CopyDataToProcessBuff(((TCPIP_NET_IF*)(snmpStkDcptMemStubPtr->SNMPNotifyInfo.snmpTrapInf))->netIPAddr.v[0],dynTrapScopedPduBuf);
		SNMPv3CopyDataToProcessBuff(((TCPIP_NET_IF*)(snmpStkDcptMemStubPtr->SNMPNotifyInfo.snmpTrapInf))->netIPAddr.v[1],dynTrapScopedPduBuf);
		SNMPv3CopyDataToProcessBuff(((TCPIP_NET_IF*)(snmpStkDcptMemStubPtr->SNMPNotifyInfo.snmpTrapInf))->netIPAddr.v[2],dynTrapScopedPduBuf);
		SNMPv3CopyDataToProcessBuff(((TCPIP_NET_IF*)(snmpStkDcptMemStubPtr->SNMPNotifyInfo.snmpTrapInf))->netIPAddr.v[3],dynTrapScopedPduBuf);
	
		// Geberic/Enterprise Trap code
		 SNMPv3CopyDataToProcessBuff(ASN_INT,dynTrapScopedPduBuf);
		 SNMPv3CopyDataToProcessBuff(1,dynTrapScopedPduBuf);
		 SNMPv3CopyDataToProcessBuff(snmpStkDcptMemStubPtr->gGenericTrapNotification,dynTrapScopedPduBuf); 
	
		// Specific Trap code
		SNMPv3CopyDataToProcessBuff(ASN_INT,dynTrapScopedPduBuf);
		SNMPv3CopyDataToProcessBuff(1,dynTrapScopedPduBuf);
		SNMPv3CopyDataToProcessBuff(snmpStkDcptMemStubPtr->SNMPNotifyInfo.notificationCode,dynTrapScopedPduBuf);
	
		// Time stamp
		SNMPv3CopyDataToProcessBuff(SNMP_TIME_TICKS,dynTrapScopedPduBuf);
		SNMPv3CopyDataToProcessBuff(4,dynTrapScopedPduBuf);
		SNMPv3CopyDataToProcessBuff(snmpStkDcptMemStubPtr->SNMPNotifyInfo.timestamp.v[3],dynTrapScopedPduBuf);
		SNMPv3CopyDataToProcessBuff(snmpStkDcptMemStubPtr->SNMPNotifyInfo.timestamp.v[2],dynTrapScopedPduBuf);
		SNMPv3CopyDataToProcessBuff(snmpStkDcptMemStubPtr->SNMPNotifyInfo.timestamp.v[1],dynTrapScopedPduBuf);
		SNMPv3CopyDataToProcessBuff(snmpStkDcptMemStubPtr->SNMPNotifyInfo.timestamp.v[0],dynTrapScopedPduBuf);
	
		// Variable binding structure header
		SNMPv3CopyDataToProcessBuff(0x30,dynTrapScopedPduBuf);
		varBindStructLenOffset = dynTrapScopedPduBuf->length;
		SNMPv3CopyDataToProcessBuff(0,dynTrapScopedPduBuf);
	
		// Create variable name-pair structure
		SNMPv3CopyDataToProcessBuff(0x30,dynTrapScopedPduBuf);
		varPairStructLenOffset = dynTrapScopedPduBuf->length;
		SNMPv3CopyDataToProcessBuff(0,dynTrapScopedPduBuf);
		 
		// Get complete notification variable OID string.
		if ( !SNMPFindOIDStringByID(var, &rec, OIDValue, &OIDLen) )
		{
			return false;
		}
	
		// Copy OID string into packet.
		SNMPv3CopyDataToProcessBuff(ASN_OID,dynTrapScopedPduBuf);
		SNMPv3CopyDataToProcessBuff((uint8_t)(OIDLen+1),dynTrapScopedPduBuf);
		len = OIDLen;
		ptr = OIDValue;
		while( len-- )
			SNMPv3CopyDataToProcessBuff(*ptr++,dynTrapScopedPduBuf);
		SNMPv3CopyDataToProcessBuff(index,dynTrapScopedPduBuf);
	
		// Encode and Copy actual data bytes
		if ( !SNMPGetDataTypeInfo(rec.dataType, &dataTypeInfo) )
		{
			return false;
		}
	
		SNMPv3CopyDataToProcessBuff(dataTypeInfo.asnType,dynTrapScopedPduBuf);
	
	
		//Modified to Send trap even for  dataTypeInfo.asnType= ASCII_STRING, 
		//where dataTypeInfo.asnLen=0xff
		if ( dataTypeInfo.asnLen == 0xff )
		{
			dataTypeInfo.asnLen=0x4;
			val.dword=0;
		}	
		len = dataTypeInfo.asnLen;
		SNMPv3CopyDataToProcessBuff(len,dynTrapScopedPduBuf);
		while( len-- )
			SNMPv3CopyDataToProcessBuff(val.v[len],dynTrapScopedPduBuf);
	
		tempOffset = dynTrapScopedPduBuf->length;
		dynTrapScopedPduBuf->length = varPairStructLenOffset;
		varBindLen.Val = (tempOffset - varPairStructLenOffset)-1;
		SNMPv3CopyDataToProcessBuff(varBindLen.v[0],dynTrapScopedPduBuf);
		
		dynTrapScopedPduBuf->length = varBindStructLenOffset;
		varBindLen.Val = (tempOffset - varBindStructLenOffset)-1;
		SNMPv3CopyDataToProcessBuff(varBindLen.v[0],dynTrapScopedPduBuf);
	
		dynTrapScopedPduBuf->length = pduStructLenOffset;
		varBindLen.Val = (tempOffset - pduStructLenOffset)-1;
		SNMPv3CopyDataToProcessBuff(varBindLen.v[0],dynTrapScopedPduBuf);
	
		dynTrapScopedPduBuf->length = 1;
		SNMPv3CopyDataToProcessBuff(0x82,dynTrapScopedPduBuf);
		varBindLen.Val = tempOffset - 4; // equal to tempOffset - dynTrapScopedPduBuf->length
		SNMPv3CopyDataToProcessBuff(varBindLen.v[1],dynTrapScopedPduBuf);
		SNMPv3CopyDataToProcessBuff(varBindLen.v[0],dynTrapScopedPduBuf);
	
		dynTrapScopedPduBuf->length = tempOffset;
		
		pduStructLenOffset = 0;
		varBindStructLenOffset = 0;
		return true;
	
#endif
	}
	else
	{
		dynTrapScopedPduBuf = &Snmpv3StackDcptStubPtr->TrapScopdPduRespnsBuf;
	}
	
	// Create variable name-pair structure
	SNMPv3CopyDataToProcessBuff(STRUCTURE,dynTrapScopedPduBuf);
	varPairStructLenOffset = dynTrapScopedPduBuf->length;
	SNMPv3CopyDataToProcessBuff(0,dynTrapScopedPduBuf);
	/* to send generic trap trap */
	if(snmpStkDcptMemStubPtr->gGenericTrapNotification != ENTERPRISE_SPECIFIC)
	{
		ptr = (uint8_t*)SNMPResolveGenTrapCodeToTrapOID(snmpStkDcptMemStubPtr->gGenericTrapNotification,&OIDLen);
		if(ptr == NULL)
		{
			//SYS_FS_close(Snmpv3FileDescrptr);
			return false;
		}
		// Copy OID string into PDU.
		SNMPv3CopyDataToProcessBuff(ASN_OID,dynTrapScopedPduBuf);
		SNMPv3CopyDataToProcessBuff((uint8_t)(OIDLen)+1,dynTrapScopedPduBuf);
		while( OIDLen-- )
		SNMPv3CopyDataToProcessBuff(*ptr++,dynTrapScopedPduBuf);

		//2nd varbind  and this is a scalar object so index = 0
		SNMPv3CopyDataToProcessBuff(0,dynTrapScopedPduBuf);
		if ( !SNMPFindOIDStringByID(snmpStkDcptMemStubPtr->SNMPNotifyInfo.agentIDVar, &rec, OIDValue, &OIDLen) )
		{
			SYS_FS_close(Snmpv3FileDescrptr);
			//UDPClose(snmpStkDcptMemStubPtr->SNMPNotifyInfo.socket);
			return false;
		}
		if ( !rec.nodeInfo.Flags.bIsAgentID )
		{
			return false;
		}


		SYS_FS_lseek(Snmpv3FileDescrptr, rec.hData, SEEK_SET);

		SNMPv3CopyDataToProcessBuff(ASN_OID,dynTrapScopedPduBuf);
		SYS_FS_read(Snmpv3FileDescrptr,&len,1);
		OIDLen = len;
		SNMPv3CopyDataToProcessBuff(OIDLen,dynTrapScopedPduBuf);
		while( OIDLen-- )
		{
			uint8_t c;
			SYS_FS_read(Snmpv3FileDescrptr,&c,1);
			SNMPv3CopyDataToProcessBuff(c,dynTrapScopedPduBuf);
		}
		tempOffset = dynTrapScopedPduBuf->length;
		//set the snmp varbind trap offset
		dynTrapScopedPduBuf->length = varPairStructLenOffset;
		/*OIDLen = 2 					 // Agent ID header bytes
			+ (uint8_t)sizeof(snmptrap_oids)
			+ 1 					   // index byte
			+ 2 					 // header
			+ agentIDLen;				 // Agent ID bytes				  */
		
		SNMPv3CopyDataToProcessBuff((tempOffset-varPairStructLenOffset)-1,dynTrapScopedPduBuf);
		//set the previous TX offset
		dynTrapScopedPduBuf->length = tempOffset;
		// len = OIDLen;
	}
	else
	{
		// Get complete notification variable OID string.
		if ( !SNMPFindOIDStringByID(var, &rec, OIDValue, &OIDLen) )
		{
			return false;
		}		
		ptr = OIDValue;
	
		// Copy OID string into packet.
		SNMPv3CopyDataToProcessBuff(ASN_OID,dynTrapScopedPduBuf);
		SNMPv3CopyDataToProcessBuff((uint8_t)(OIDLen+1),dynTrapScopedPduBuf);
		len = OIDLen;
		while( len-- )
			SNMPv3CopyDataToProcessBuff(*ptr++,dynTrapScopedPduBuf);
		SNMPv3CopyDataToProcessBuff(index,dynTrapScopedPduBuf);

		// Encode and Copy actual data bytes
		if ( !SNMPGetDataTypeInfo(rec.dataType, &dataTypeInfo) )
		{
			return false;
		}
		SNMPv3CopyDataToProcessBuff(dataTypeInfo.asnType,dynTrapScopedPduBuf);
	     //Modified to Send trap even for  dataTypeInfo.asnType= ASCII_STRING, 
		//where dataTypeInfo.asnLen=0xff
		if ( dataTypeInfo.asnLen == 0xff )
		{
			uint8_t *asciiStr= (uint8_t *)val.dword;
			int k=0;
			dataTypeInfo.asnLen=strlen((char *)asciiStr);
			len = dataTypeInfo.asnLen;
			//val.dword=0;
			SNMPv3CopyDataToProcessBuff(len,dynTrapScopedPduBuf);
			for(k=0;k<len;k++)
				SNMPv3CopyDataToProcessBuff(asciiStr[k],dynTrapScopedPduBuf);
		}
		else
		{
		len = dataTypeInfo.asnLen;
	
			SNMPv3CopyDataToProcessBuff(len,dynTrapScopedPduBuf);
		while( len-- )
				SNMPv3CopyDataToProcessBuff(val.v[len],dynTrapScopedPduBuf);
		}
	  
		/*len	 = dataTypeInfo.asnLen	// data bytes count
			 + 1                    // Length byte
			 + 1                    // Data type byte
			 + OIDLen               // OID bytes
			 + 2                    // OID header bytes
			 + 1;		            // index byte */
		tempOffset = dynTrapScopedPduBuf->length;
		dynTrapScopedPduBuf->length = varPairStructLenOffset;
		SNMPv3CopyDataToProcessBuff((tempOffset-varPairStructLenOffset)-1,dynTrapScopedPduBuf);
		dynTrapScopedPduBuf->length = tempOffset;
	} 
	//set the previous TX offset
#if defined(SNMP_STACK_USE_V2_TRAP)
	if(snmpStkDcptMemStubPtr->gSetTrapSendFlag == true)
	{
		//SYS_FS_close(Snmpv3FileDescrptr);
		return true;
	}
#endif
	tempOffset = dynTrapScopedPduBuf->length;
	dynTrapScopedPduBuf->length = varBindStructLenOffset;
	SNMPv3CopyDataToProcessBuff(0x82,dynTrapScopedPduBuf);
	varBindLen.Val = (tempOffset - varBindStructLenOffset)-3;
	SNMPv3CopyDataToProcessBuff(varBindLen.v[1],dynTrapScopedPduBuf);
	SNMPv3CopyDataToProcessBuff(varBindLen.v[0],dynTrapScopedPduBuf);

	dynTrapScopedPduBuf->length = pduStructLenOffset;
	SNMPv3CopyDataToProcessBuff(0x82,dynTrapScopedPduBuf);
	varBindLen.Val = (tempOffset - pduStructLenOffset)-3;
	SNMPv3CopyDataToProcessBuff(varBindLen.v[1],dynTrapScopedPduBuf);
	SNMPv3CopyDataToProcessBuff(varBindLen.v[0],dynTrapScopedPduBuf);

	dynTrapScopedPduBuf->length = 1;
	SNMPv3CopyDataToProcessBuff(0x82,dynTrapScopedPduBuf);
	varBindLen.Val = tempOffset - 4; // equal to tempOffset - dynTrapScopedPduBuf->length
	SNMPv3CopyDataToProcessBuff(varBindLen.v[1],dynTrapScopedPduBuf);
	SNMPv3CopyDataToProcessBuff(varBindLen.v[0],dynTrapScopedPduBuf);

	dynTrapScopedPduBuf->length = tempOffset;

	
    pduStructLenOffset = 0;
    varBindStructLenOffset = 0;
	return true;

}

/****************************************************************************
  Function:
	bool SNMPv3Notify(SNMP_ID var, SNMP_VAL val, SNMP_INDEX index,uint8_t targetIndex)

  Summary:
  	Creates and Sends SNMPv3 TRAP pdu.
	
  Description:
	This function creates SNMPv3 trap PDU and sends it to previously specified
	remoteHost.
	       
  Precondition:
	TRAP event is triggered.

  Parameters:
	var     - SNMP var ID that is to be used in notification
	val     - Value of var. Only value of uint8_t, uint16_t or uint32_t can be sent.
	index   - Index of var. If this var is a single,index would be 0, or else 
			  if this var Is a sequence, index could be any value 
			  from 0 to 127
	targetIndex -index of the 'Snmpv3TrapConfigData' table's security user name 
  			     for which the TRAP PDU message header to constructed. 

  Return Values:
	true	-	if SNMP notification was successful sent.
	   			This does not guarantee that remoteHost recieved it.
	false	-	Notification sent failed.
  			    This would fail under following contions:
  			    1) Given SNMP_BIB_FILE does not exist in file system
  			    2) Given var does not exist.
  			    3) Previously given agentID does not exist
  			    4) Data type of given var is unknown - only
  			       possible if file system itself was corrupted.
 	SNMPV3_MSG_PRIV_FAIL -encryption of the trap msg failed
	SNMPV3_MSG_AUTH_FAIL - HAMC of the trap msg failed

  Remarks:
	None
 ***************************************************************************/
 
bool SNMPv3Notify(SNMP_ID var, SNMP_VAL val, SNMP_INDEX index,uint8_t targetIndex)
{
	TCPIP_UINT16_VAL	totaltrapLen={0};
	uint16_t		i=0;
	uint8_t		USM_Index=0;

	SNMP_PROCESSING_MEM_INFO_PTRS snmpPktProcsMemPtrsInfo; 
	SNMP_STACK_DCPT_STUB * snmpStkDcptMemStubPtr=0; 	
						
	SNMPGetPktProcessingDynMemStubPtrs(&snmpPktProcsMemPtrsInfo);
	snmpStkDcptMemStubPtr=snmpPktProcsMemPtrsInfo.snmpStkDynMemStubPtr;

	//validate the trap user security name , message processing model 
	//, security model and the security level
	//if(validateTrapSecurityNameAndSecuityLevel(tragetIndex) != true)
	//	return false;
	gSNMPV3TrapSecurityLevel = SNMPv3GetTrapSecurityLevel(Snmpv3StackDcptStubPtr->Snmpv3TrapConfigData[targetIndex].securityLevelType);
	if( gSNMPV3TrapSecurityLevel == INVALID_MSG)
	{
		SNMPv3FreeDynAllocMem();
		UDPClose(snmpStkDcptMemStubPtr->SNMPNotifyInfo.socket);
		snmpStkDcptMemStubPtr->SNMPNotifyInfo.socket =  INVALID_UDP_SOCKET;
		
		return false;
	}

	Snmpv3FileDescrptr = SYS_FS_open((const char*)SNMP_BIB_FILE_NAME,0);
	
	if ( Snmpv3FileDescrptr == RETURN_FAILED) //Invalid handle
	{
		UDPClose(snmpStkDcptMemStubPtr->SNMPNotifyInfo.socket);
		snmpStkDcptMemStubPtr->SNMPNotifyInfo.socket =  INVALID_UDP_SOCKET;
		SNMPv3FreeDynAllocMem();
		return false;
	}

	if(Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf.head == NULL)
	{
		if(SNMPv3TrapMsgHeaderPDU(targetIndex)!= true)
		{
			SYS_FS_close(Snmpv3FileDescrptr);
			UDPClose(snmpStkDcptMemStubPtr->SNMPNotifyInfo.socket);
			snmpStkDcptMemStubPtr->SNMPNotifyInfo.socket =  INVALID_UDP_SOCKET;
			SNMPv3FreeDynAllocMem();
			return false;
		}		
	}
	if(SNMPv3TrapScopedpdu(var,val,index,targetIndex) != true)
	{
		SYS_FS_close(Snmpv3FileDescrptr);
		UDPClose(snmpStkDcptMemStubPtr->SNMPNotifyInfo.socket);
		snmpStkDcptMemStubPtr->SNMPNotifyInfo.socket =  INVALID_UDP_SOCKET;
		SNMPv3FreeDynAllocMem();
		return false;
	}
#if defined(SNMP_STACK_USE_V2_TRAP)
	if(snmpStkDcptMemStubPtr->gSetTrapSendFlag == true)
	{
		SYS_FS_close(Snmpv3FileDescrptr);
		return true;
	}
#endif
	USM_Index = SNMPv3GetUserIndxFromUsmUserDB(targetIndex);
	if(USM_Index == INVALID_INDEX)
	{
		SNMPv3FreeDynAllocMem();
		SYS_FS_close(Snmpv3FileDescrptr);
		UDPClose(snmpStkDcptMemStubPtr->SNMPNotifyInfo.socket);
		snmpStkDcptMemStubPtr->SNMPNotifyInfo.socket =  INVALID_UDP_SOCKET;
		return false;
	}
#if 1
	{
		uint8_t tempBuf[4];
		uint8_t tempCntr=0;
		uint8_t* tempPtr=NULL;
		uint8_t* outBufPtr=NULL;
		//uint16_t  tempScopedPduLen=0;
		

		totaltrapLen.Val = Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf.length + Snmpv3StackDcptStubPtr->TrapScopdPduRespnsBuf.length+3;
		//tempScopedPduLen = Snmpv3StackDcptStubPtr->TrapScopdPduRespnsBuf.length-4; // 4 == STRUCTURE+0x82+len1+len2
		if((gSNMPV3TrapSecurityLevel & 0x02)==0x02) //Encrypted message	
		{
			tempPtr=tempBuf;
			*tempPtr++=0X04;
			if((Snmpv3StackDcptStubPtr->TrapScopdPduRespnsBuf.length >= 0x80) && (Snmpv3StackDcptStubPtr->TrapScopdPduRespnsBuf.length <= 0xFF))
			{
				*tempPtr++=0x81;
				*tempPtr=Snmpv3StackDcptStubPtr->TrapScopdPduRespnsBuf.length;
				tempCntr=3; //0x04(encrypted pkt),0x81,len
			}
			else if((Snmpv3StackDcptStubPtr->TrapScopdPduRespnsBuf.length > 0xFF) && (Snmpv3StackDcptStubPtr->TrapScopdPduRespnsBuf.length < 0xFFFF))
			{			
				*tempPtr++=0x82;
				*tempPtr++=Snmpv3StackDcptStubPtr->TrapScopdPduRespnsBuf.length>>8;
				*tempPtr=Snmpv3StackDcptStubPtr->TrapScopdPduRespnsBuf.length;
				tempCntr=4; //0x04(encrypted pkt),0x81,len_1,len_0
			}
			else
			{
				*tempPtr=Snmpv3StackDcptStubPtr->TrapScopdPduRespnsBuf.length;
				tempCntr=2; //0x04(encrypted pkt),len
			}
		}

		Snmpv3StackDcptStubPtr->TrapOUTPduWholeMsgBuf.wholeMsgLen.Val=(totaltrapLen.Val+tempCntr/*0x04,0x82,len_1,len_0*/);
		Snmpv3StackDcptStubPtr->TrapOUTPduWholeMsgBuf.wholeMsgHead=
			(uint8_t*)(TCPIP_HEAP_Calloc(snmpPktProcsMemPtrsInfo.snmpHeapMemHandler,1,(size_t)Snmpv3StackDcptStubPtr->TrapOUTPduWholeMsgBuf.wholeMsgLen.Val+4+16));
		if(Snmpv3StackDcptStubPtr->TrapOUTPduWholeMsgBuf.wholeMsgHead == NULL)
		{
			SYS_FS_close(Snmpv3FileDescrptr);
			UDPClose(snmpStkDcptMemStubPtr->SNMPNotifyInfo.socket);
			snmpStkDcptMemStubPtr->SNMPNotifyInfo.socket =  INVALID_UDP_SOCKET;
			SNMPv3FreeDynAllocMem();
			return false;
		}

		outBufPtr=Snmpv3StackDcptStubPtr->TrapOUTPduWholeMsgBuf.wholeMsgHead;	


		//Start Writing to the outPut Buffer

		*outBufPtr++=STRUCTURE;

		totaltrapLen.Val+=tempCntr;
					 
		if((totaltrapLen.Val >= 0x80) && (totaltrapLen.Val <= 0xFF))
		{
			*outBufPtr++=0x81;
			*outBufPtr++=totaltrapLen.Val;
		}
		else if((totaltrapLen.Val > 0xFF) && (totaltrapLen.Val < 0xFFFF))
		{			
			*outBufPtr++=0x82;
			*outBufPtr++=totaltrapLen.v[1];
			*outBufPtr++=totaltrapLen.v[0];
		}
		else
			*outBufPtr++=totaltrapLen.Val;

		*outBufPtr++=ASN_INT;
		*outBufPtr++=0x1;		
		*outBufPtr++=SNMP_V3;

		Snmpv3StackDcptStubPtr->TrapOUTPduWholeMsgBuf.msgAuthParamOffsetOutWholeMsg=(uint8_t*)(outBufPtr+Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf.msgAuthParamOffset);
		//put global snmpv3 msg header 
		for(i=0;i<Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf.length;i++)
		{
			*outBufPtr++=Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf.head[i];
		}

		if (Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf.head!=NULL)
		{	
			TCPIP_HEAP_Free(snmpPktProcsMemPtrsInfo.snmpHeapMemHandler,Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf.head);
			Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf.length=0x00;
			Snmpv3StackDcptStubPtr->TrapMsgHeaderBuf.head=NULL;
		}


		//Copy Scoped PDU to the Out Buffer
		if((gSNMPV3TrapSecurityLevel & 0x02)==0x02) //Encrypted message	
		{	//Copy Packet Auth indicator, length
			for(i=0;i<tempCntr;i++) 
			{
				*outBufPtr++=tempBuf[i];
			}
		}
		Snmpv3StackDcptStubPtr->TrapOUTPduWholeMsgBuf.scopedPduOffset=outBufPtr;
		Snmpv3StackDcptStubPtr->TrapOUTPduWholeMsgBuf.scopedPduStructLen=Snmpv3StackDcptStubPtr->TrapScopdPduRespnsBuf.length;

		i=0;
		*outBufPtr++=Snmpv3StackDcptStubPtr->TrapScopdPduRespnsBuf.head[i++];//0x30

		if(Snmpv3StackDcptStubPtr->TrapScopdPduRespnsBuf.head[1] == 0x81)
		{
			*outBufPtr++=Snmpv3StackDcptStubPtr->TrapScopdPduRespnsBuf.head[i++];//0x81
			*outBufPtr++=Snmpv3StackDcptStubPtr->TrapScopdPduRespnsBuf.head[i++];//len_0
		}
		else if(Snmpv3StackDcptStubPtr->TrapScopdPduRespnsBuf.head[1] == 0x82)
		{
			*outBufPtr++=Snmpv3StackDcptStubPtr->TrapScopdPduRespnsBuf.head[i++]; //0x82
			*outBufPtr++=Snmpv3StackDcptStubPtr->TrapScopdPduRespnsBuf.head[i++]; //len_1
			*outBufPtr++=Snmpv3StackDcptStubPtr->TrapScopdPduRespnsBuf.head[i++]; //len_0
		}
		else
			*outBufPtr++=Snmpv3StackDcptStubPtr->TrapScopdPduRespnsBuf.head[i++];//len_o

		// send context id and context name and the get response 
		// Authentication and privacy data packet will be sent from here onwards
		for(;i<(Snmpv3StackDcptStubPtr->TrapScopdPduRespnsBuf.length);i++)
		{
			*outBufPtr++=Snmpv3StackDcptStubPtr->TrapScopdPduRespnsBuf.head[i];
		}

		/*Encrypt the Response to the messgae originator*/
		if((gSNMPV3TrapSecurityLevel & 0x02)==0x02) //Encrypted message
		{
			uint8_t temp_usm_index = 0;
			/*Rxed SNMPv3 message is encrypted. Hence Response should be encrypted*/

			/*If user privacy protocol is AES*/
			temp_usm_index = Snmpv3StackDcptStubPtr->UserInfoDataBaseIndx; 
			Snmpv3StackDcptStubPtr->UserInfoDataBaseIndx = USM_Index;
			if(SNMPv3AESEncryptResponseScopedPdu(&Snmpv3StackDcptStubPtr->TrapOUTPduWholeMsgBuf) != SNMPV3_MSG_PRIV_PASS)
			{
				Snmpv3StackDcptStubPtr->UserInfoDataBaseIndx = temp_usm_index;
				SYS_FS_close(Snmpv3FileDescrptr);
				UDPClose(snmpStkDcptMemStubPtr->SNMPNotifyInfo.socket);
				snmpStkDcptMemStubPtr->SNMPNotifyInfo.socket =  INVALID_UDP_SOCKET;
				SNMPv3FreeDynAllocMem();
				return SNMPV3_MSG_PRIV_FAIL;
			}
			Snmpv3StackDcptStubPtr->UserInfoDataBaseIndx = temp_usm_index;
			/*If user privacy Protocol is DES*/
			//snmpV3DESDecryptRxedScopedPdu();
		}

		/* Authenticate the whole message to be transmitted*/
		if((gSNMPV3TrapSecurityLevel & 0x01)==0x01) //Authenticatd message
		{
			uint8_t temp_usm_index = 0;
			/*Rxed SNMPv3 message is Authenticated.Send authenticatin parameters for the Response*/
			Snmpv3StackDcptStubPtr->TrapOUTPduWholeMsgBuf.wholeMsgLen.Val = outBufPtr - Snmpv3StackDcptStubPtr->TrapOUTPduWholeMsgBuf.wholeMsgHead;
			/*If user authentication is HAMC-MD5-96*/
			temp_usm_index = Snmpv3StackDcptStubPtr->UserInfoDataBaseIndx; 
			Snmpv3StackDcptStubPtr->UserInfoDataBaseIndx = USM_Index;
			if(SNMPv3AuthenticateTxPduForDataIntegrity(&Snmpv3StackDcptStubPtr->TrapOUTPduWholeMsgBuf)!=SNMPV3_MSG_AUTH_PASS)
			{
				Snmpv3StackDcptStubPtr->UserInfoDataBaseIndx = temp_usm_index;
				SYS_FS_close(Snmpv3FileDescrptr);
				UDPClose(snmpStkDcptMemStubPtr->SNMPNotifyInfo.socket);
				snmpStkDcptMemStubPtr->SNMPNotifyInfo.socket =  INVALID_UDP_SOCKET;
				SNMPv3FreeDynAllocMem();
				return SNMPV3_MSG_AUTH_FAIL;
			}
			Snmpv3StackDcptStubPtr->UserInfoDataBaseIndx = temp_usm_index;
			tempPtr = outBufPtr;
			outBufPtr=Snmpv3StackDcptStubPtr->TrapOUTPduWholeMsgBuf.msgAuthParamOffsetOutWholeMsg;
			for(i=0;i<Snmpv3StackDcptStubPtr->SnmpOutMsgAuthParmLen;i++)
				*outBufPtr++=Snmpv3StackDcptStubPtr->SnmpOutMsgAuthParaStrng[i];
			outBufPtr = tempPtr;
		}

		
		SNMPProcessReqBuildRespnsDuplexInit(snmpStkDcptMemStubPtr->SNMPNotifyInfo.socket);

		//total length number of bytes need to be passed to the wire
		Snmpv3StackDcptStubPtr->TrapOUTPduWholeMsgBuf.wholeMsgLen.Val = outBufPtr - Snmpv3StackDcptStubPtr->TrapOUTPduWholeMsgBuf.wholeMsgHead;

		i=0;
		SNMPPutByteToUDPTxBuffer(snmpStkDcptMemStubPtr->SNMPNotifyInfo.socket,Snmpv3StackDcptStubPtr->TrapOUTPduWholeMsgBuf.wholeMsgHead[i++]);		//0x30

		if(Snmpv3StackDcptStubPtr->TrapOUTPduWholeMsgBuf.wholeMsgHead[i] == 0x81)
		{
			SNMPPutByteToUDPTxBuffer(snmpStkDcptMemStubPtr->SNMPNotifyInfo.socket,Snmpv3StackDcptStubPtr->TrapOUTPduWholeMsgBuf.wholeMsgHead[i++]);//0x81
			SNMPPutByteToUDPTxBuffer(snmpStkDcptMemStubPtr->SNMPNotifyInfo.socket,Snmpv3StackDcptStubPtr->TrapOUTPduWholeMsgBuf.wholeMsgHead[i++]);//len_0
		}
		else if(Snmpv3StackDcptStubPtr->TrapOUTPduWholeMsgBuf.wholeMsgHead[i] == 0x82)
		{
			SNMPPutByteToUDPTxBuffer(snmpStkDcptMemStubPtr->SNMPNotifyInfo.socket,Snmpv3StackDcptStubPtr->TrapOUTPduWholeMsgBuf.wholeMsgHead[i++]); //0x82
			SNMPPutByteToUDPTxBuffer(snmpStkDcptMemStubPtr->SNMPNotifyInfo.socket,Snmpv3StackDcptStubPtr->TrapOUTPduWholeMsgBuf.wholeMsgHead[i++]); //len_1
			SNMPPutByteToUDPTxBuffer(snmpStkDcptMemStubPtr->SNMPNotifyInfo.socket,Snmpv3StackDcptStubPtr->TrapOUTPduWholeMsgBuf.wholeMsgHead[i++]); //len_0
		}
		else
			SNMPPutByteToUDPTxBuffer(snmpStkDcptMemStubPtr->SNMPNotifyInfo.socket,Snmpv3StackDcptStubPtr->TrapOUTPduWholeMsgBuf.wholeMsgHead[i++]);//len_o
			
		for(;i<Snmpv3StackDcptStubPtr->TrapOUTPduWholeMsgBuf.wholeMsgLen.Val;i++)
		{	
			SNMPPutByteToUDPTxBuffer(snmpStkDcptMemStubPtr->SNMPNotifyInfo.socket,Snmpv3StackDcptStubPtr->TrapOUTPduWholeMsgBuf.wholeMsgHead[i]);
		}

		//if(Snmpv3StackDcptStubPtr->ScopedPduRespnsBuf.head != NULL)
		if(Snmpv3StackDcptStubPtr->TrapOUTPduWholeMsgBuf.wholeMsgHead != NULL)
		{
			TCPIP_HEAP_Free(snmpPktProcsMemPtrsInfo.snmpHeapMemHandler,Snmpv3StackDcptStubPtr->TrapOUTPduWholeMsgBuf.wholeMsgHead);
			Snmpv3StackDcptStubPtr->TrapOUTPduWholeMsgBuf.wholeMsgHead=0x00;
			Snmpv3StackDcptStubPtr->TrapOUTPduWholeMsgBuf.wholeMsgLen.Val=0x00;
			Snmpv3StackDcptStubPtr->TrapOUTPduWholeMsgBuf.snmpMsgHead = NULL;
			Snmpv3StackDcptStubPtr->TrapOUTPduWholeMsgBuf.msgAuthParamOffsetOutWholeMsg = NULL;
			Snmpv3StackDcptStubPtr->TrapOUTPduWholeMsgBuf.scopedPduOffset =	NULL;			
		}

		SYS_FS_close(Snmpv3FileDescrptr);
		UDPFlush(snmpStkDcptMemStubPtr->SNMPNotifyInfo.socket);
		UDPClose(snmpStkDcptMemStubPtr->SNMPNotifyInfo.socket);
		snmpStkDcptMemStubPtr->SNMPNotifyInfo.socket =  INVALID_UDP_SOCKET;

		SNMPv3FreeDynAllocMem();
		
	}
#endif
	return true;
}
#endif // end of #if defined(SNMP_V1_V2_TRAP_WITH_SNMPV3)


#endif // #if defined(TCPIP_STACK_USE_SNMPV3_SERVER)
