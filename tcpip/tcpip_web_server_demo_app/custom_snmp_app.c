/*******************************************************************************
  Application to Demo SNMP Server

  Summary:
    Support for SNMP module in Microchip TCP/IP Stack
    
  Description:
    - Implements the SNMP application
*******************************************************************************/

/*******************************************************************************
FileName:  CustomSNMPApp.c 
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

#define __CUSTOMSNMPAPP_C

#include "tcpip_config.h"

#if defined(TCPIP_STACK_USE_SNMP_SERVER)

#include "tcpip/tcpip.h"
#include "main_demo.h"
#include "tcpip/snmp.h"
#include "mib.h"




/****************************************************************************
  Section:
	Global Variables
  ***************************************************************************/

#ifdef TCPIP_STACK_USE_SNMPV3_SERVER
static uint8_t gSnmpv3UserSecurityName[SNMPV3_USER_SECURITY_NAME_LEN_MEM_USE];
#endif

/*
*  Default STACK_USE_SMIV2 is enabled . For Stack V5.31,  STACK_USE_SMIV2 should be disabled.
*/
#define STACK_USE_SMIV2

/* Update the Non record id OID value 
   which is part of CustomSnmpDemo.c file
*/
#define SNMP_MAX_NON_REC_ID_OID  3

/*
* NOTE - 
* gSnmpNonMibRecInfo[] is the list of static variable Parent OIDs which are not part of MIB.h file. 
* This structure is used to restrict access to static variables of SNMPv3 OIDs from SNMPv2c and SNMPv1 version. 
* With SNMPv3 all the OIDs accessible but when we are using SNMPv2c version , static variables of the SNMPv3 
* cannot be accessible with SNMPversion v2c.
* With V5.31 there was no MODULE-IDENTITY number in the SNMP.mib file. Now onwards we have supported SMIv2 
* standard and SNMP.mib has been updated with respect to SMIV2 standard and it also includes 
* MODULE-IDENTITY ( number 1)after ENTERPRISE-ID.
*/

/*
* This structure has been moved from snmp.c file to here. 
*/
#ifdef STACK_USE_SMIV2
/*
* With SMIv2 standard which includes MODULE-IDENTITY number with the existing OID string.
* For New snmp.mib file with SMIv2 standard
*/
/*
* ENTERPRISEID - 17095(Microchip) as per BER encoding standard 0x81,0x85,0x47
* Need to be modified with respect to customer enterprise ID 
*/

static SNMPNONMIBRECDINFO gSnmpNonMibRecInfo[SNMP_MAX_NON_REC_ID_OID] =
{
#ifdef TCPIP_STACK_USE_SNMPV3_SERVER		
	/* To restrict SNMPv3 Static Variable OID string which is not part of mib.h file, User is required to include here.*More details in the above NOTE.*/
#endif			
	{{43,6,1,2,1,1},SNMP_V2C}, /* Max matching Subids of the iso+org (43),dod(6),internet(1),mgmt(2),MIB2(1),system(1) tree*/	
	{{43,6,1,4,1,0x81,0x85,0x47,0x1,1},SNMP_V2C}, 
	/*Max matching Subids of the iso+org (43),dod(6),internet(1),private(4),ENTERPRISE(17095),MODULE-IDENTITY(1),product tree*/			
	
};
/*
 * if snmp.mib file doesnot have MODULE-IDENTITY number then this is the following structure should be used
 */

#else 

/*
* OLD snmp.mib file with SMIv1 standard 
*/

static SNMPNONMIBRECDINFO gSnmpNonMibRecInfo[SNMP_MAX_NON_REC_ID_OID] =
{
#ifdef TCPIP_STACK_USE_SNMPV3_SERVER		
	{{43,6,1,4,1,0x81,0x85,0x47,6},SNMP_V3},  /* SNMPv3 PVT test MIB OID is not part of mib.h file */
#endif			
	{{43,6,1,2,1,1},SNMP_V2C}, /* Max matching Subids of the iso+org (43),dod(6),internet(1),mgmt(2),MIB2(1),system(1) tree*/	
	{{43,6,1,4,1,0x81,0x85,0x47,0x1},SNMP_V2C}, 
	/*Max matching Subids of the iso+org (43),dod(6),internet(1),private(4),ENTERPRISE(17095),product tree*/			
	
};

#endif /* STACK_USE_SMIV2 */


#if !defined(SNMP_TRAP_DISABLED)

/*Initialize trap table with no entries.*/
static SNMP_TRAP_INFO trapInfo;
#ifdef TCPIP_STACK_USE_IPV6
static IPV6_SNMP_TRAP_INFO ipv6TrapInfo;
static bool SNMPIPv6SendNotification(uint8_t receiverIndex, SNMP_ID var, SNMP_VAL val,uint8_t targetIndex);
#endif

static SYS_TICK SNMPGetTimeStamp(void);
static bool SNMPSendNotification(uint8_t receiverIndex, SNMP_ID var, SNMP_VAL val,uint8_t targetIndex);

static uint8_t 	gSendTrapSMstate=0;
static bool		gtrapSMStateUpdate=false;


#define MAX_TRY_TO_SEND_TRAP (30)

#if defined(SYS_OUT_ENABLE)
static uint8_t lcdMessage[16*2+1]="";
#endif
/****************************************************************************
  ===========================================================================
  Section:
	SNMP Routines
  ===========================================================================
  ***************************************************************************/

#ifdef TCPIP_STACK_USE_IPV6

/****************************************************************************
  Function:
	 bool SNMPIPv6SendNotification(uint8_t receiverIndex, SNMP_ID var, SNMP_VAL val,uint8_t targetIndex)
 
  Summary:			
	Prepare, validate remote node which will receive trap and send trap pdu.
		  
  Description:		
	This routine prepares the trap notification pdu, sends ARP and get
	remote device MAC address to which notification to sent, sends
	the notification. 
	Notofication state machine is getting updated if there is any ARP resolution failure for 
	a perticular trap destination address.
	
  PreCondition:
	SNMPTrapDemo() is called.
	
  parameters:
	 receiverIndex - The index to array where remote ip address is stored.	
	 var		   - SNMP var ID that is to be used in notification
	 val		   - Value of var. Only value of uint8_t, uint16_t or uint32_t can be sent.
	 
  Return Values:		  
	 true	-	If notification send is successful.
	 false	-	If send notification failed.
	 
  Remarks:
	 None.
 *************************************************************************/
static bool SNMPIPv6SendNotification(uint8_t receiverIndex, SNMP_ID var, SNMP_VAL val,uint8_t targetIndex)
{
	static enum { SM_PREPARE, SM_NOTIFY_WAIT } smState = SM_PREPARE;
	IP_MULTI_ADDRESS ipv6Address;
	static uint8_t tempRxIndex;
	static IPV6_ADDR tempIpv6Address;
	uint8_t 	retVal = 0;

#ifdef TCPIP_STACK_USE_SNMPV3_SERVER
	SNMPV3_PROCESSING_MEM_INFO_PTRS snmpv3PktProcessingMemPntr; 
	SNMPV3_STACK_DCPT_STUB * snmpv3EngnDcptMemoryStubPtr=0; 	
	
	SNMPv3GetPktProcessingDynMemStubPtrs(&snmpv3PktProcessingMemPntr);
						
	snmpv3EngnDcptMemoryStubPtr=snmpv3PktProcessingMemPntr.snmpv3StkProcessingDynMemStubPtr;
#endif 

	SNMP_PROCESSING_MEM_INFO_PTRS snmpPktProcsMemPtrsInfo; 
	SNMP_STACK_DCPT_STUB * snmpStkDcptMemStubPtr=0; 	
	
	SNMPGetPktProcessingDynMemStubPtrs(&snmpPktProcsMemPtrsInfo);
					
	snmpStkDcptMemStubPtr=snmpPktProcsMemPtrsInfo.snmpStkDynMemStubPtr;

		
	ipv6TrapInfo.Size = TRAP_TABLE_SIZE;

	// Convert local to network order.
	memcpy(&ipv6Address.v6Add,&ipv6TrapInfo.table[receiverIndex].IPv6Address,16);
	
	if(gtrapSMStateUpdate == true)
	{
		smState = SM_PREPARE;
		gtrapSMStateUpdate = false;
	}

	
	switch(smState)
	{
		case SM_PREPARE:
			gSendTrapSMstate = smState;
			tempRxIndex=receiverIndex;			
			// Convert local to network order.
			memcpy(&tempIpv6Address,&ipv6TrapInfo.table[receiverIndex].IPv6Address,16);
			SNMPNotifyPrepare(NULL,
							  ipv6TrapInfo.table[receiverIndex].community,
							  ipv6TrapInfo.table[receiverIndex].communityLen,
							  MICROCHIP,				   // Agent ID Var
							  snmpStkDcptMemStubPtr->gSpecificTrapNotification,   // Specifc Trap notification code
							  SNMPGetTimeStamp());
			
			smState = SM_NOTIFY_WAIT;

			break;
	
		case SM_NOTIFY_WAIT:
			gSendTrapSMstate = smState;
			if ( SNMPIsNotifyReady(&ipv6Address,IPV6_SNMP_TRAP) )
			{
#if defined(SNMP_V1_V2_TRAP_WITH_SNMPV3)	
				if(targetIndex == (SNMPV3_USM_MAX_USER-1))
				{
					smState = SM_PREPARE;
				}
				if((snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[targetIndex].messageProcessingModelType == SNMPV3_MSG_PROCESSING_MODEL)
					&& (snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[targetIndex].securityModelType == SNMPV3_USM_SECURITY_MODEL))
				{
					retVal = SNMPv3Notify(var, val, 0,targetIndex);
				}
				else if((snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[targetIndex].messageProcessingModelType == SNMPV2C_MSG_PROCESSING_MODEL)
					&& (snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[targetIndex].securityModelType == SNMPV2C_SECURITY_MODEL))
				{
					retVal = SNMPNotify(var, val, 0);
				}
				else if((snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[targetIndex].messageProcessingModelType == SNMPV1_MSG_PROCESSING_MODEL)
					&& (snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[targetIndex].securityModelType == SNMPV1_SECURITY_MODEL))
				{
					retVal = SNMPNotify(var, val, 0);
				}
				else 
				{					
									return true;
				}
#else
				smState = SM_PREPARE;
				retVal = SNMPNotify(var, val, 0);
#endif
				return retVal;
			}
			/* if trapInfo table address for a particular index is different comparing to the SM_PREPARE IP address
				then change the state to SM_PREPARE*/
			if((memcmp(&tempIpv6Address,&ipv6Address,16) != 0)&&
					(tempRxIndex == receiverIndex))
			{
				smState = SM_PREPARE;
			}
			/* Change state machine from SM_NOTIFY_WAIT to SM_PREPARE if incoming trap destination 
			index is different from the SM_PREPARE	trap destination index*/
			if(tempRxIndex != receiverIndex)
			{
				smState=SM_PREPARE;
			}
			
	}
	
	return false;
}


#endif

/****************************************************************************
  Function:
 	 bool SNMPSendNotification(uint8_t receiverIndex, SNMP_ID var, SNMP_VAL val,uint8_t targetIndex)
 
  Summary:			
	Prepare, validate remote node which will receive trap and send trap pdu.
 	  	  
  Description:		
    This routine prepares the trap notification pdu, sends ARP and get
	remote device MAC address to which notification to sent, sends
	the notification. 
	Notofication state machine is getting updated if there is any ARP resolution failure for 
	a perticular trap destination address.
	
  PreCondition:
	SNMPTrapDemo() is called.
 	
  parameters:
     receiverIndex - The index to array where remote ip address is stored.  
     var		   - SNMP var ID that is to be used in notification
	 val           - Value of var. Only value of uint8_t, uint16_t or uint32_t can be sent.
	 
  Return Values:          
 	 true	-	If notification send is successful.
 	 false	-	If send notification failed.
 	 
  Remarks:
     None.
 *************************************************************************/
static bool SNMPSendNotification(uint8_t receiverIndex, SNMP_ID var, SNMP_VAL val,uint8_t targetIndex)
{
    static enum { SM_PREPARE, SM_NOTIFY_WAIT } smState = SM_PREPARE;
    IP_MULTI_ADDRESS IPAddress;
	static uint8_t tempRxIndex;
	static IPV4_ADDR tempIpAddress;	
	uint8_t		retVal = 0;

#ifdef TCPIP_STACK_USE_SNMPV3_SERVER
	SNMPV3_PROCESSING_MEM_INFO_PTRS snmpv3PktProcessingMemPntr; 
	SNMPV3_STACK_DCPT_STUB * snmpv3EngnDcptMemoryStubPtr=0;		
	
	SNMPv3GetPktProcessingDynMemStubPtrs(&snmpv3PktProcessingMemPntr);
						
	snmpv3EngnDcptMemoryStubPtr=snmpv3PktProcessingMemPntr.snmpv3StkProcessingDynMemStubPtr;
#endif 

	SNMP_PROCESSING_MEM_INFO_PTRS snmpPktProcsMemPtrsInfo; 
	SNMP_STACK_DCPT_STUB * snmpStkDcptMemStubPtr=0; 	
	
	SNMPGetPktProcessingDynMemStubPtrs(&snmpPktProcsMemPtrsInfo);
					
	snmpStkDcptMemStubPtr=snmpPktProcsMemPtrsInfo.snmpStkDynMemStubPtr;

		
	trapInfo.Size = TRAP_TABLE_SIZE;

    // Convert local to network order.
    IPAddress.v4Add.v[0] = trapInfo.table[receiverIndex].IPAddress.v[3];
    IPAddress.v4Add.v[1] = trapInfo.table[receiverIndex].IPAddress.v[2];
    IPAddress.v4Add.v[2] = trapInfo.table[receiverIndex].IPAddress.v[1];
    IPAddress.v4Add.v[3] = trapInfo.table[receiverIndex].IPAddress.v[0];
	
	if(gtrapSMStateUpdate == true)
	{
		smState = SM_PREPARE;
		gtrapSMStateUpdate = false;
	}

	
    switch(smState)
    {
	    case SM_PREPARE:
			gSendTrapSMstate = smState;
			tempRxIndex=receiverIndex;			
			// Convert local to network order.
			tempIpAddress.v[0] = trapInfo.table[receiverIndex].IPAddress.v[3];
			tempIpAddress.v[1] = trapInfo.table[receiverIndex].IPAddress.v[2];
			tempIpAddress.v[2] = trapInfo.table[receiverIndex].IPAddress.v[1];
			tempIpAddress.v[3] = trapInfo.table[receiverIndex].IPAddress.v[0];
			
	        SNMPNotifyPrepare(&IPAddress,
	                          trapInfo.table[receiverIndex].community,
	                          trapInfo.table[receiverIndex].communityLen,
	                          MICROCHIP,            	   // Agent ID Var
	                          snmpStkDcptMemStubPtr->gSpecificTrapNotification,   // Specifc Trap notification code
	                          SNMPGetTimeStamp());
			
	        smState = SM_NOTIFY_WAIT;

	        break;
	
	    case SM_NOTIFY_WAIT:
			gSendTrapSMstate = smState;
	        if ( SNMPIsNotifyReady(&IPAddress,IPV4_SNMP_TRAP) )
	        {
#if defined(SNMP_V1_V2_TRAP_WITH_SNMPV3)	
				if(targetIndex == (SNMPV3_USM_MAX_USER-1))
				{
					smState = SM_PREPARE;
				}
				if((snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[targetIndex].messageProcessingModelType == SNMPV3_MSG_PROCESSING_MODEL)
					&& (snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[targetIndex].securityModelType == SNMPV3_USM_SECURITY_MODEL))
				{
			   		retVal = SNMPv3Notify(var, val, 0,targetIndex);
				}
				else if((snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[targetIndex].messageProcessingModelType == SNMPV2C_MSG_PROCESSING_MODEL)
					&& (snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[targetIndex].securityModelType == SNMPV2C_SECURITY_MODEL))
				{
					retVal = SNMPNotify(var, val, 0);
				}
				else if((snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[targetIndex].messageProcessingModelType == SNMPV1_MSG_PROCESSING_MODEL)
					&& (snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[targetIndex].securityModelType == SNMPV1_SECURITY_MODEL))
				{
					retVal = SNMPNotify(var, val, 0);
				}
				else 
				{					
                                    return true;
				}
#else
	            smState = SM_PREPARE;
				retVal = SNMPNotify(var, val, 0);
#endif
	            return retVal;
	        }
			/* if trapInfo table address for a particular index is different comparing to the SM_PREPARE IP address
				then change the state to SM_PREPARE*/
			if((tempIpAddress.Val != IPAddress.v4Add.Val) && 
					(tempRxIndex == receiverIndex))
			{
				smState = SM_PREPARE;
			}
			/* Change state machine from SM_NOTIFY_WAIT to SM_PREPARE if incoming trap destination 
			index is different from the SM_PREPARE  trap destination index*/
			if(tempRxIndex != receiverIndex)
			{
				smState=SM_PREPARE;
			}
			
    }
	
    return false;
}

#if defined(SNMP_STACK_USE_V2_TRAP) || defined(SNMP_V1_V2_TRAP_WITH_SNMPV3)
/**************************************************************************
  Function:
 	void SNMPV2TrapDemo(void)
 
  Summary:	
  	Send SNMP V2 notification with multiple varbinds.
 	  	  
  Description:		
	This routine sends a trap v2 pdu with multiple varbind variables
	for the predefined ip addresses with the agent. And as per RFC1905 
	the first two variable bindings in the varbind pdu list of an
   	SNMPv2-Trap-PDU are sysUpTime.0 and snmpTrapOID.0 respectively.
   	To support multiple varbind, user need to call SNMPSendNotification()
    for the first varbind variable and SNMPSendNotification() will do the 
    arp resolve and adds sysUpTime.0 and snmpTrapOID.0 variable to 
    the pdu. For the second varbind variable onwards user need to 
   	call only SNMPNotify().
	In this demo , snmpv2 trap includes ANALOG_POT0,PUSH_BUTTON and LED_D5
	variable bindings and TrapCommunity variable is being used as part of the fourth varbind of
	the TRAP PDU which is ASCII string format.
	and this trap can be generated by using Analog portmeter value.
	and SNMPv2-Trap-PDU will be generated only when pot meter reading exceeds 12.
	
	gSetTrapSendFlag Should be set to true when user is trying to send first 
	variable binding and gSetTrapSendFlag should be set to false before 
    sending the last variable binding.

	* if user is sending only one variable binding then 
	* gSetTrapSendFlag should be set to False.	
    * user can add more variable bindings.
    * For ASCII STR trap , argument VAL contains the pointer address of the string variable.
  PreCondition:
 	Application defined event occurs to send the trap.
 	
  parameters:
     None.
 
  Returns:          
 	 None.
 
  Remarks:
    This routine guides how to build a event generated trap notification.
 *************************************************************************/
void SNMPV2TrapDemo()
{
	static SYS_TICK tempTimerRead = 0;
	static uint8_t	trapIndex=0;
	static SNMP_VAL		analogPotVal;
	static uint8_t potReadLock = false;
	static uint8_t timeLock = false;
#if defined(SNMP_V1_V2_TRAP_WITH_SNMPV3)	
	static uint8_t userIndex=0;
#endif
	uint8_t		targetIndex = 0;
	uint8_t		retVal = 0;
	static bool netStartForTrap = true;
	static SNMP_TRAP_IP_ADDRESS_TYPE snmpTrapIpAddresstype = IPV4_SNMP_TRAP;


	SNMP_PROCESSING_MEM_INFO_PTRS snmpPktProcsMemPtrsInfo; 
	SNMP_STACK_DCPT_STUB * snmpStkDcptMemStubPtr=0;		
	
	SNMPV3_PROCESSING_MEM_INFO_PTRS snmpv3PktProcessingMemPntr; 
	SNMPV3_STACK_DCPT_STUB * snmpv3EngnDcptMemoryStubPtr=0;		
					
								
	SNMPGetPktProcessingDynMemStubPtrs(&snmpPktProcsMemPtrsInfo);
	snmpStkDcptMemStubPtr=snmpPktProcsMemPtrsInfo.snmpStkDynMemStubPtr;

	SNMPv3GetPktProcessingDynMemStubPtrs(&snmpv3PktProcessingMemPntr);
	snmpv3EngnDcptMemoryStubPtr=snmpv3PktProcessingMemPntr.snmpv3StkProcessingDynMemStubPtr;

	trapInfo.Size = TRAP_TABLE_SIZE;	
#ifdef TCPIP_STACK_USE_IPV6	
	ipv6TrapInfo.Size = TRAP_TABLE_SIZE;
#endif	
	if(timeLock==(uint8_t)false)
	{
		tempTimerRead=SYS_TICK_Get();
		timeLock=true;
	}
	
	/*
	*  select one of the interface and send trap to all the trap Host IP address as per trap table.Then select the next interface till it reach all the interface. 
	*/
	if(netStartForTrap)
	{
		/*
			Specify SNMPV2 specific trap ID Here. Which will help Ireasoning and other SNMP manager tools to 
			recognise the trap information and it will help the SNMP manager tool to decrypt the 
			trap information. 
		
			This ID is only related to trap ID. and this implementaion is only for TRAPv2 specific.
		*/
		snmpStkDcptMemStubPtr->SNMPNotifyInfo.trapIDVar = SNMP_DEMO_TRAP;

    	 snmpStkDcptMemStubPtr->SNMPNotifyInfo.snmpTrapInf = (TCPIP_NET_HANDLE*)SNMPUdpClientGetNet();
		
		if(!TCPIP_STACK_IsNetUp(snmpStkDcptMemStubPtr->SNMPNotifyInfo.snmpTrapInf))
		{
			return;
		}
		netStartForTrap = false;
	}


	for(;trapIndex<TRAP_TABLE_SIZE;trapIndex++)
	{	

		if(snmpTrapIpAddresstype == IPV4_SNMP_TRAP)
		{
			if(!trapInfo.table[trapIndex].Flags.bEnabled)
			{
				continue;
			}
		}
#ifdef TCPIP_STACK_USE_IPV6
		else
		{
			if(!ipv6TrapInfo.table[trapIndex].Flags.bEnabled)
			{
				continue;
			}
		}
#endif		
		//Read POT reading once and send trap to all configured recipient
		if(potReadLock ==(uint8_t)false)
		{
			analogPotVal.word= (uint16_t)ADC1BUF0;
			potReadLock    = true;                        
		}
	
		if(analogPotVal.word >12u)
		{
			/*
			 * prepare  and send multivarbind pdu using pot meter value. 
			 * for SNMP v2 trap sysUpTime.0 and SNMPv2TrapOID.0 are mandatory
			 * apart from these varbinds, push button and potmeter OID are included
			 * to this pdu.
			*/
			//snmpStkDcptMemStubPtr->gSpecificTrapNotification = 1; //expecting 1 should be the specific trap.
			//snmpStkDcptMemStubPtr->gGenericTrapNotification = ENTERPRISE_SPECIFIC;
			//snmpStkDcptMemStubPtr->gSetTrapSendFlag = true;
			// insert ANALOG_POT0 OID value and OID to the varbind pdu
			//set global flag gSetTrapSendFlag to true , it signifies that there are more than one 
			// variable need to be the part of SNMP v2 TRAP. 
			// if there is  only varbind variable to be the part of SNMP v2 trap, 
			// then user should set gSetTrapSendFlag to false.
			//snmpStkDcptMemStubPtr->gSetTrapSendFlag = false;
#if defined(SNMP_V1_V2_TRAP_WITH_SNMPV3)	
			for(targetIndex=userIndex;targetIndex<SNMPV3_USM_MAX_USER;targetIndex++)
			{
#endif
#if defined(SNMP_STACK_USE_V2_TRAP)
				snmpStkDcptMemStubPtr->gSpecificTrapNotification = 1; //expecting 1 should be the specific trap.
				snmpStkDcptMemStubPtr->gGenericTrapNotification = ENTERPRISE_SPECIFIC;
				snmpStkDcptMemStubPtr->gSetTrapSendFlag = true;
#endif
				if(snmpTrapIpAddresstype == IPV4_SNMP_TRAP)
				{
					retVal =SNMPSendNotification(trapIndex,ANALOG_POT0,analogPotVal,targetIndex);
					if((gSendTrapSMstate == 0x0) && (retVal == false)) // gSendTrapSMstate == SM_PREPARE
					{
						retVal = SNMPSendNotification(trapIndex, ANALOG_POT0, analogPotVal,targetIndex);
					}
				}
#ifdef TCPIP_STACK_USE_IPV6
				else
				{
					retVal =SNMPIPv6SendNotification(trapIndex,ANALOG_POT0,analogPotVal,targetIndex);
					if((gSendTrapSMstate == 0x0) && (retVal == false)) // gSendTrapSMstate == SM_PREPARE
					{
						retVal = SNMPIPv6SendNotification(trapIndex, ANALOG_POT0, analogPotVal,targetIndex);
					}
				}
#endif		/* #ifdef TCPIP_STACK_USE_IPV6 */		
#if defined(SNMP_V1_V2_TRAP_WITH_SNMPV3)	
				if((retVal==true)&& (targetIndex == (SNMPV3_USM_MAX_USER-1)))
				{				
					//trapIndex++;
				}
				else if(retVal == false)
#else				
				if(retVal == false)
#endif
				{
#if defined(SNMP_V1_V2_TRAP_WITH_SNMPV3) // to keep tarck of the usr index for SNMPv3
					userIndex = targetIndex;
#endif	
					if((SYS_TICK_Get() - SNMPGetTrapTime()> 1*SYS_TICK_TicksPerSecondGet()))
					{
						trapIndex++;
						gtrapSMStateUpdate = true;
						if(snmpStkDcptMemStubPtr->SNMPNotifyInfo.socket != INVALID_UDP_SOCKET)
						{
							UDPClose(snmpStkDcptMemStubPtr->SNMPNotifyInfo.socket);
							snmpStkDcptMemStubPtr->SNMPNotifyInfo.socket = INVALID_UDP_SOCKET;
						}
						return;
					}
					return ;
				}
				//prepare PUSH_BUTTON trap .for the next trap varbind we need to use snmp_notify instead of 
				// SNMPSendNotification(), because we have already prepared SNMP v2 trap header 
				//and arp has been resolved already.
				
				analogPotVal.byte = BUTTON0_IO;
#if defined(SNMP_V1_V2_TRAP_WITH_SNMPV3)	
				if((snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[targetIndex].messageProcessingModelType == SNMPV3_MSG_PROCESSING_MODEL)
					&& (snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[targetIndex].securityModelType == SNMPV3_USM_SECURITY_MODEL))
				{
			   		SNMPv3Notify(PUSH_BUTTON,analogPotVal,0,targetIndex);
				}
				else if((snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[targetIndex].messageProcessingModelType == SNMPV2C_MSG_PROCESSING_MODEL)
					&& (snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[targetIndex].securityModelType == SNMPV2C_SECURITY_MODEL))
				{
					SNMPNotify(PUSH_BUTTON,analogPotVal,0);
				}
				else if((snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[targetIndex].messageProcessingModelType == SNMPV1_MSG_PROCESSING_MODEL)
					&& (snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[targetIndex].securityModelType == SNMPV1_SECURITY_MODEL))
				{
					SNMPNotify(PUSH_BUTTON,analogPotVal,0);
				}
				else
				{						
                    continue;
				}
#else
				SNMPNotify(PUSH_BUTTON,analogPotVal,0);
#endif
				// if this is the last trap variable need to be the part of SNMP v2 Trap,
				// then we should disable gSetTrapSendFlag to false
				
				analogPotVal.byte = LED0_IO;
#if defined(SNMP_V1_V2_TRAP_WITH_SNMPV3)				
				SNMPv3Notify(LED_D5,analogPotVal,0,targetIndex);
				if((snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[targetIndex].messageProcessingModelType == SNMPV3_MSG_PROCESSING_MODEL)
					&& (snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[targetIndex].securityModelType == SNMPV3_USM_SECURITY_MODEL))
				{
					SNMPv3Notify(LED_D5,analogPotVal,0,targetIndex);
				}
				else if((snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[targetIndex].messageProcessingModelType == SNMPV2C_MSG_PROCESSING_MODEL)
					&& (snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[targetIndex].securityModelType == SNMPV2C_SECURITY_MODEL))
				{
					SNMPNotify(LED_D5,analogPotVal,0);
				}
				else if((snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[targetIndex].messageProcessingModelType == SNMPV1_MSG_PROCESSING_MODEL)
					&& (snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[targetIndex].securityModelType == SNMPV1_SECURITY_MODEL))
				{
					SNMPNotify(LED_D5,analogPotVal,0);
				}
#else
				SNMPNotify(LED_D5,analogPotVal,0);

#endif
// if this is the last trap variable need to be the part of SNMP  Trap v2,
// then we should disable gSetTrapSendFlag to false
#if defined(SNMP_STACK_USE_V2_TRAP)
				snmpStkDcptMemStubPtr->gSetTrapSendFlag = false;
#endif
#if defined(SNMP_V1_V2_TRAP_WITH_SNMPV3)				
				//ASCII String is the fouth varbind of the TRAP ODU
				// for this TRAP_COMMUNITy string has been used as the 
				//  Fourth varbind.
				{
					uint8_t asciiStr[] = {"ascii_str_trap"};
					analogPotVal.dword = (uint32_t)&asciiStr;
					if((snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[targetIndex].messageProcessingModelType == SNMPV3_MSG_PROCESSING_MODEL)
						&& (snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[targetIndex].securityModelType == SNMPV3_USM_SECURITY_MODEL))
					{
						SNMPv3Notify(TRAP_COMMUNITY,analogPotVal,0,targetIndex);
					}
					else if((snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[targetIndex].messageProcessingModelType == SNMPV2C_MSG_PROCESSING_MODEL)
						&& (snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[targetIndex].securityModelType == SNMPV2C_SECURITY_MODEL))
					{
						SNMPNotify(TRAP_COMMUNITY,analogPotVal,0);
					}
					else if((snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[targetIndex].messageProcessingModelType == SNMPV1_MSG_PROCESSING_MODEL)
						&& (snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[targetIndex].securityModelType == SNMPV1_SECURITY_MODEL))
					{
						SNMPNotify(TRAP_COMMUNITY,analogPotVal,0);
					}
				}
#else
				{
					uint8_t asciiStr[] = {"ascii_str_trap"};
					analogPotVal.dword = (uint32_t)&asciiStr;
					SNMPNotify(TRAP_COMMUNITY,analogPotVal,0);
				}
#endif

#if defined(SNMP_V1_V2_TRAP_WITH_SNMPV3)
				if(userIndex == SNMPV3_USM_MAX_USER-1)
				{
					userIndex = 0;
				}
			}
#endif			
			
		}
                potReadLock = false;
	}
        
	//Try for max 5 seconds to send TRAP, do not get block in while()
	if((SYS_TICK_Get()-tempTimerRead)>(5*SYS_TICK_TicksPerSecondGet()))
	{
		UDPDiscard(snmpStkDcptMemStubPtr->SNMPNotifyInfo.socket);
		
		timeLock = false;
		if(trapIndex>=TRAP_TABLE_SIZE)
		{
#ifdef TCPIP_STACK_USE_IPV6		
			if(snmpTrapIpAddresstype == IPV4_SNMP_TRAP)
			{
				snmpTrapIpAddresstype=IPV6_SNMP_TRAP;
			}
			else if(snmpTrapIpAddresstype == IPV6_SNMP_TRAP)
			{
				snmpTrapIpAddresstype=IPV4_SNMP_TRAP;
			}
#endif			
			netStartForTrap = true;
			trapIndex = 0;
		}
		analogPotVal.word = 0;
		gSendTrapSMstate = 0;
		return;
	}
}
#endif /* SNMP_STACK_USE_V2_TRAP  || SNMP_V1_V2_TRAP_WITH_SNMPV3 */

/**************************************************************************
  Function:
 	void SNMPTrapDemo(void)
 
  Summary:	
  	Send trap pdu demo application.
 	  	  
  Description:		
	This routine is used to send various trap events for the predefined ip addresses with the
	agent. Events like ANALOG POT value, PUSH_BUTTON and ASCII Str notification.
	ANALOG POT event -  When there is a POT value greater than 12 and for this ANALOG_POT0
	is used as a TRAP PDU variable for this event.
	PUSH BUTTON event -  BUTTON2_IO is the varaible used for this trap event.
	ASCII STR event - TRAP_COMMUNITY variable is used for this event and this event will occur when
	both LED1 and LED2 are blinking.       
  PreCondition:
 	Application defined event occurs to send the trap.
 	
  parameters:
     None.
 
  Returns:          
 	 None.
 
  Remarks:
    This routine guides how to build a event generated trap notification.
    The application should make use of SNMPSendTrap() routine to generate 
    and send trap.
 *************************************************************************/
void SNMPTrapDemo(void)
{
	static SYS_TICK TimerRead=0;
	static bool analogPotNotify = false,buttonPushNotify=false,asciiStrNotify=false;
	static uint8_t anaPotNotfyCntr=0,buttonPushNotfyCntr=0;
	static SNMP_VAL buttonPushval,analogPotVal;
	static uint8_t potReadLock=false,buttonLock=false;
	static uint8_t timeLock=false;
	static uint32_t maxTryToSendTrap=0,maxTryToSendTrap1=0;
	uint8_t		targetIndex;
	bool		retVal=true;
	static uint8_t	trapIndex=0;
#if defined(SNMP_V1_V2_TRAP_WITH_SNMPV3)	
	static uint8_t userIndex=0,userIndex1=0,userIndex2=0;
#endif
	static bool netStartForTrap = true;
	SNMP_PROCESSING_MEM_INFO_PTRS snmpPktProcsMemPtrsInfo; 
	SNMP_STACK_DCPT_STUB * snmpStkDcptMemStubPtr=0;		
	
	
	SNMPGetPktProcessingDynMemStubPtrs(&snmpPktProcsMemPtrsInfo);
						
	snmpStkDcptMemStubPtr=snmpPktProcsMemPtrsInfo.snmpStkDynMemStubPtr;

#if defined(SNMP_V1_V2_TRAP_WITH_SNMPV3) || defined(SNMP_STACK_USE_V2_TRAP)
	/*
		Specify SNMPV2 specific trap ID Here. Which will help Ireasoning and other SNMP manager tools to 
		recognise the trap information and it will help the SNMP manager tool to decrypt the 
		trap information. 
	
		This ID is only related to trap ID. and this implementaion is only for TRAPv2 specific.
	*/
	snmpStkDcptMemStubPtr->SNMPNotifyInfo.trapIDVar = SNMP_DEMO_TRAP;
#endif
	targetIndex = 0;

	
	trapInfo.Size = TRAP_TABLE_SIZE;
#ifdef TCPIP_STACK_USE_IPV6	
	ipv6TrapInfo.Size = TRAP_TABLE_SIZE;
#endif
	if(timeLock==(uint8_t)false)
	{
		TimerRead=SYS_TICK_Get();
		timeLock=true;
	}
	
	/*
	*  select one of the interface and send trap to all the trap Host IP address as per trap table.Then select the next interface till it reach all the interface. 
	*/
	if(netStartForTrap)
	{
		/*
			Specify SNMPV2 specific trap ID Here. Which will help Ireasoning and other SNMP manager tools to 
			recognise the trap information and it will help the SNMP manager tool to decrypt the 
			trap information. 
		
			This ID is only related to trap ID. and this implementaion is only for TRAPv2 specific.
		*/
#if defined(SNMP_STACK_USE_V2_TRAP) || defined(SNMP_V1_V2_TRAP_WITH_SNMPV3)
		snmpStkDcptMemStubPtr->SNMPNotifyInfo.trapIDVar = SNMP_DEMO_TRAP;
#endif

    	snmpStkDcptMemStubPtr->SNMPNotifyInfo.snmpTrapInf = (TCPIP_NET_HANDLE*)SNMPUdpClientGetNet();
		
		if(!TCPIP_STACK_IsNetUp(snmpStkDcptMemStubPtr->SNMPNotifyInfo.snmpTrapInf))
		{
			return;
		}
		netStartForTrap = false;
	}
	if(anaPotNotfyCntr >= trapInfo.Size)
	{
		anaPotNotfyCntr = 0;
		potReadLock=false;
		//analogPotNotify = false;
		analogPotNotify = true;
	}
	
	if(!analogPotNotify)
	{
		//Read POT reading once and send trap to all configured recipient
		if(potReadLock ==(uint8_t)false)
		{
            analogPotVal.word= (uint16_t)ADC1BUF0;
			
			//Avoids Reading POT for every iteration unless trap sent to each configured recipients 
			potReadLock=true; 
		}
		if(trapInfo.table[anaPotNotfyCntr].Flags.bEnabled)
		{
			if(analogPotVal.word >12u)
			{
#if defined(SNMP_V1_V2_TRAP_WITH_SNMPV3)	
				for(targetIndex=userIndex;targetIndex<SNMPV3_USM_MAX_USER;targetIndex++)
				{
#endif			
#if defined(SNMP_STACK_USE_V2_TRAP)
                                    snmpStkDcptMemStubPtr->gSpecificTrapNotification=POT_READING_MORE_512;
				    snmpStkDcptMemStubPtr->gGenericTrapNotification=ENTERPRISE_SPECIFIC;
				    snmpStkDcptMemStubPtr->gSetTrapSendFlag = false;
#endif
                                    retVal = SNMPSendNotification(anaPotNotfyCntr, ANALOG_POT0, analogPotVal,targetIndex);
                                    if((gSendTrapSMstate == 0x0) && (retVal == false)) // gSendTrapSMstate == SM_PREPARE
                                    {
                                            retVal = SNMPSendNotification(anaPotNotfyCntr, ANALOG_POT0, analogPotVal,targetIndex);
                                    }
#if defined(SNMP_V1_V2_TRAP_WITH_SNMPV3)	
					if((retVal == true) && (targetIndex == (SNMPV3_USM_MAX_USER-1)))
						anaPotNotfyCntr++;
					else if(retVal == false)
#else
					if(retVal == true)
						anaPotNotfyCntr++;
					else 
#endif						
					{
#if defined(SNMP_V1_V2_TRAP_WITH_SNMPV3) // to keep tarck of the usr index for SNMPv3
						userIndex = targetIndex;
#endif
						if((SYS_TICK_Get() - SNMPGetTrapTime()> 2*SYS_TICK_TicksPerSecondGet()))
						{
							anaPotNotfyCntr++;
							gtrapSMStateUpdate = true;
							if(snmpStkDcptMemStubPtr->SNMPNotifyInfo.socket != INVALID_UDP_SOCKET)
							{
								UDPClose(snmpStkDcptMemStubPtr->SNMPNotifyInfo.socket);
								snmpStkDcptMemStubPtr->SNMPNotifyInfo.socket = INVALID_UDP_SOCKET;
							}
							return;
						}
						return ;
					}
#if defined(SNMP_V1_V2_TRAP_WITH_SNMPV3)
					if(userIndex == SNMPV3_USM_MAX_USER-1)
						userIndex = 0;					
				}
#endif			
			}
		}
		else
			anaPotNotfyCntr++;
			
	}


	if(buttonPushNotfyCntr==trapInfo.Size)
	{
		buttonPushNotfyCntr = 0;
		buttonLock=false;
		buttonPushNotify = false;
	}


	if(buttonLock == (uint8_t)false)
	{
		if(BUTTON2_IO == 0u)
		{
			buttonPushNotify = true;
			buttonLock =true;
		}
	}

	if(buttonPushNotify)
	{			  
		buttonPushval.byte = 0;
		if ( trapInfo.table[buttonPushNotfyCntr].Flags.bEnabled )
		{
#if defined(SNMP_V1_V2_TRAP_WITH_SNMPV3)	
			for(targetIndex=userIndex1;targetIndex<SNMPV3_USM_MAX_USER;targetIndex++)
			{
#endif		
#if defined(SNMP_STACK_USE_V2_TRAP)
                                snmpStkDcptMemStubPtr->gSpecificTrapNotification=BUTTON_PUSH_EVENT;
				snmpStkDcptMemStubPtr->gGenericTrapNotification=ENTERPRISE_SPECIFIC;
				snmpStkDcptMemStubPtr->gSetTrapSendFlag = false;
#endif
				retVal =  SNMPSendNotification(buttonPushNotfyCntr, PUSH_BUTTON, buttonPushval,targetIndex);
				if((gSendTrapSMstate == 0x0) && (retVal == false)) // gSendTrapSMstate == SM_PREPARE
				{
					retVal = SNMPSendNotification(buttonPushNotfyCntr, PUSH_BUTTON, buttonPushval,targetIndex);
				}
#if defined(SNMP_V1_V2_TRAP_WITH_SNMPV3)	
				if((retVal==true)&& (targetIndex == (SNMPV3_USM_MAX_USER-1)))
#else
				if(retVal==true)
#endif									
					buttonPushNotfyCntr++;					
#if defined(SNMP_V1_V2_TRAP_WITH_SNMPV3)
				else
					userIndex1 = targetIndex;				
				if(userIndex1 == SNMPV3_USM_MAX_USER-1)
					userIndex1 = 0;
			}
#endif			
		}
		else
			buttonPushNotfyCntr++;
	}
/*
	ASCII String trap support . When LED2 and LED1 are on then ASCII string trap will be send.
	TrapCommunity(4) is used as a variable for both TRAPv1 and TRAPv2 PDU.
*/
	if((!LED1_IO && !LED2_IO) && (trapIndex >= TRAP_TABLE_SIZE))
	{
		asciiStrNotify = false;
	}
	else if((LED1_IO && LED2_IO) && (trapIndex < TRAP_TABLE_SIZE))
		asciiStrNotify = true;

	if(asciiStrNotify)
	{
		for(;trapIndex<TRAP_TABLE_SIZE;trapIndex++)
		{	
			uint8_t asciiStr[] = {"ascii_str_trap"};
			SNMP_VAL asciistrVal;
			asciistrVal.dword = (uint32_t)&asciiStr;
			
			if(!trapInfo.table[trapIndex].Flags.bEnabled)
			continue;		
			
			if(analogPotVal.word <= 12u)
				continue;

#if defined(SNMP_V1_V2_TRAP_WITH_SNMPV3)	
			for(targetIndex=userIndex2;targetIndex<SNMPV3_USM_MAX_USER;targetIndex++)
			{
#endif
#if defined(SNMP_STACK_USE_V2_TRAP)
				snmpStkDcptMemStubPtr->gSpecificTrapNotification=POT_READING_MORE_512;
				snmpStkDcptMemStubPtr->gGenericTrapNotification=ENTERPRISE_SPECIFIC;
				snmpStkDcptMemStubPtr->gSetTrapSendFlag = false;
#endif
				retVal =  SNMPSendNotification(trapIndex, TRAP_COMMUNITY, asciistrVal,targetIndex);
				if((gSendTrapSMstate == 0x0) && (retVal == false)) // gSendTrapSMstate == SM_PREPARE
				{
					retVal = SNMPSendNotification(trapIndex, TRAP_COMMUNITY, asciistrVal,targetIndex);
				}
#if defined(SNMP_V1_V2_TRAP_WITH_SNMPV3)	
				if((retVal==true)&& (targetIndex == (SNMPV3_USM_MAX_USER-1)))
				{
					//trapIndex++;
				}
				else if(retVal == false)
#else				
				if(retVal == false)
#endif					
				{
#if defined(SNMP_V1_V2_TRAP_WITH_SNMPV3) // to keep tarck of the usr index for SNMPv3
					userIndex2 = targetIndex;
#endif				
					if((SYS_TICK_Get() - SNMPGetTrapTime()> 2*SYS_TICK_TicksPerSecondGet()))
					{
						trapIndex++;
						gtrapSMStateUpdate = true;
						if(snmpStkDcptMemStubPtr->SNMPNotifyInfo.socket != INVALID_UDP_SOCKET)
						{
							UDPClose(snmpStkDcptMemStubPtr->SNMPNotifyInfo.socket);
							snmpStkDcptMemStubPtr->SNMPNotifyInfo.socket = INVALID_UDP_SOCKET;
						}
						return;
					}
					return ;
				}
					
#if defined(SNMP_V1_V2_TRAP_WITH_SNMPV3)
			if(userIndex2 == SNMPV3_USM_MAX_USER-1)
				userIndex2 = 0;
			}
#endif		
		}
	}

	//Try for max 5 seconds to send TRAP, do not get block in while()
	if((SYS_TICK_Get()-TimerRead)>(5*SYS_TICK_TicksPerSecondGet()))
	{

		netStartForTrap = true;
		trapIndex = 0;
		UDPDiscard(snmpStkDcptMemStubPtr->SNMPNotifyInfo.socket);
		buttonPushNotfyCntr = 0;
		buttonLock=false;
		buttonPushNotify = false;
		anaPotNotfyCntr = 0;
		potReadLock=false;
		analogPotNotify = false;
		timeLock=false;
		snmpStkDcptMemStubPtr->gSpecificTrapNotification=VENDOR_TRAP_DEFAULT;
		snmpStkDcptMemStubPtr->gGenericTrapNotification=ENTERPRISE_SPECIFIC;
		maxTryToSendTrap = 0;
		maxTryToSendTrap1 = 0;
		gSendTrapSMstate = 0;		
		return;
	}

}


/**************************************************************************
  Function:
 	void SNMPSendTrap(void)
 
  Summary:	
  	 Prepare, validate remote node which will receive trap and send trap pdu.
 	 	  
  Description:		
     This function is used to send trap notification to previously 
     configured ip address if trap notification is enabled. There are
     different trap notification code. The current implementation
     sends trap for authentication failure (4).
  
  PreCondition:
 	 If application defined event occurs to send the trap.
 
  parameters:
     None.
 
  Returns:          
 	 None.
 
  Remarks:
     This is a callback function called by the application on certain 
     predefined events. This routine only implemented to send a 
     authentication failure Notification-type macro with PUSH_BUTTON
     oid stored in snmp.bib. If the ARP is no resolved i.e. if 
     SNMPIsNotifyReady() returns false, this routine times 
     out in 5 seconds. This routine should be modified according to 
     event occured and should update corrsponding OID and notification
     type to the trap pdu.
 *************************************************************************/
void SNMPSendTrap(void)
{
	static uint8_t timeLock=false;
	static uint8_t receiverIndex=0; ///is application specific
	IP_MULTI_ADDRESS remHostIPAddress,* remHostIpAddrPtr;
	SNMP_VAL val;
	static SYS_TICK TimerRead;
#if defined(SNMP_V1_V2_TRAP_WITH_SNMPV3)	
	static uint8_t userIndex=0;
#endif
	static enum 
	{
		SM_PREPARE,
		SM_NOTIFY_WAIT 
	} smState = SM_PREPARE;
	static bool netStartForTrap = true;

#ifdef TCPIP_STACK_USE_SNMPV3_SERVER
		SNMPV3_PROCESSING_MEM_INFO_PTRS snmpv3PktProcessingMemPntr; 
		SNMPV3_STACK_DCPT_STUB * snmpv3EngnDcptMemoryStubPtr=0; 	
		
		SNMPv3GetPktProcessingDynMemStubPtrs(&snmpv3PktProcessingMemPntr);
							
		snmpv3EngnDcptMemoryStubPtr=snmpv3PktProcessingMemPntr.snmpv3StkProcessingDynMemStubPtr;
#endif 

	SNMP_PROCESSING_MEM_INFO_PTRS snmpPktProcsMemPtrsInfo; 
	SNMP_STACK_DCPT_STUB * snmpStkDcptMemStubPtr=0; 	
		
							
	SNMPGetPktProcessingDynMemStubPtrs(&snmpPktProcsMemPtrsInfo);
	snmpStkDcptMemStubPtr=snmpPktProcsMemPtrsInfo.snmpStkDynMemStubPtr;


#if defined(SNMP_V1_V2_TRAP_WITH_SNMPV3) || defined(SNMP_STACK_USE_V2_TRAP)
	/*
		Specify SNMPV2 specific trap ID Here. Which will help Ireasoning and other SNMP manager tools to 
		recognise the trap information and it will help the SNMP manager tool to decrypt the 
		trap information. 
	
		This ID is only related to trap ID. and this implementaion is only for TRAPv2 specific.
	*/
	snmpStkDcptMemStubPtr->SNMPNotifyInfo.trapIDVar = SNMP_DEMO_TRAP;
#endif
	if(netStartForTrap)
	{
		/*
			Specify SNMPV2 specific trap ID Here. Which will help Ireasoning and other SNMP manager tools to 
			recognise the trap information and it will help the SNMP manager tool to decrypt the 
			trap information. 
		
			This ID is only related to trap ID. and this implementaion is only for TRAPv2 specific.
		*/
#if defined(SNMP_STACK_USE_V2_TRAP) || defined(SNMP_V1_V2_TRAP_WITH_SNMPV3)
		snmpStkDcptMemStubPtr->SNMPNotifyInfo.trapIDVar = SNMP_DEMO_TRAP;
#endif

		snmpStkDcptMemStubPtr->SNMPNotifyInfo.snmpTrapInf = (TCPIP_NET_HANDLE*)SNMPUdpClientGetNet();
		
		if(!TCPIP_STACK_IsNetUp(snmpStkDcptMemStubPtr->SNMPNotifyInfo.snmpTrapInf))
		{
			return;
		}
		netStartForTrap = false;
	}
	
	trapInfo.Size = TRAP_TABLE_SIZE;


	if(trapInfo.table[receiverIndex].Flags.bEnabled)
	{
		remHostIPAddress.v4Add.v[0] = trapInfo.table[receiverIndex].IPAddress.v[3];
		remHostIPAddress.v4Add.v[1] = trapInfo.table[receiverIndex].IPAddress.v[2];
		remHostIPAddress.v4Add.v[2] = trapInfo.table[receiverIndex].IPAddress.v[1];
		remHostIPAddress.v4Add.v[3] = trapInfo.table[receiverIndex].IPAddress.v[0];
		remHostIpAddrPtr = &remHostIPAddress;
		if(timeLock==(uint8_t)false)
		{
			TimerRead=SYS_TICK_Get();
			timeLock=true;
		}
	}	
	else
	{
		receiverIndex++;
		if((receiverIndex == (uint8_t)TRAP_TABLE_SIZE))
		{
			receiverIndex=0;
			timeLock=false;
			snmpStkDcptMemStubPtr->gSendTrapFlag=false;	
			UDPDiscard(snmpStkDcptMemStubPtr->SNMPNotifyInfo.socket);
		}
		return;
		
	}
		
	switch(smState)
	{
	
		case SM_PREPARE:

			SNMPNotifyPrepare(remHostIpAddrPtr,trapInfo.table[receiverIndex].community,
						trapInfo.table[receiverIndex].communityLen,
						MICROCHIP,			  // Agent ID Var
						snmpStkDcptMemStubPtr->gSpecificTrapNotification,					  // Notification code.
						SNMPGetTimeStamp());
			smState++;
			break;
			
		case SM_NOTIFY_WAIT:
			if(SNMPIsNotifyReady(remHostIpAddrPtr,IPV4_SNMP_TRAP))
			{
				smState = SM_PREPARE;
		 		val.byte = 0;
				receiverIndex++;
#if defined(SNMP_V1_V2_TRAP_WITH_SNMPV3)
				uint8_t  targetIndex = 0;

				for(targetIndex=userIndex;targetIndex<SNMPV3_USM_MAX_USER;targetIndex++)
				{
					if((snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[targetIndex].messageProcessingModelType == SNMPV3_MSG_PROCESSING_MODEL)
						&& (snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[targetIndex].securityModelType == SNMPV3_USM_SECURITY_MODEL))
						SNMPv3Notify(snmpStkDcptMemStubPtr->gOIDCorrespondingSnmpMibID, val, 0,targetIndex);
					else if((snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[targetIndex].messageProcessingModelType == SNMPV2C_MSG_PROCESSING_MODEL)
						&& (snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[targetIndex].securityModelType == SNMPV2C_SECURITY_MODEL))
						SNMPNotify(snmpStkDcptMemStubPtr->gOIDCorrespondingSnmpMibID, val, 0);
					else if((snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[targetIndex].messageProcessingModelType == SNMPV1_MSG_PROCESSING_MODEL)
						&& (snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[targetIndex].securityModelType == SNMPV1_SECURITY_MODEL))
						SNMPNotify(snmpStkDcptMemStubPtr->gOIDCorrespondingSnmpMibID, val, 0);
				}
#else
				SNMPNotify(snmpStkDcptMemStubPtr->gOIDCorrespondingSnmpMibID, val, 0);
#endif
				//application has to decide on which SNMP var OID to send. Ex. PUSH_BUTTON	
            	smState = SM_PREPARE;
				UDPDiscard(snmpStkDcptMemStubPtr->SNMPNotifyInfo.socket);
				break;
			}
	}	
		
	//Try for max 5 seconds to send TRAP, do not get block in while()
	if((SYS_TICK_Get()-TimerRead)>(5*SYS_TICK_TicksPerSecondGet())|| (receiverIndex == (uint8_t)TRAP_TABLE_SIZE))
	{
		UDPDiscard(snmpStkDcptMemStubPtr->SNMPNotifyInfo.socket);
		smState = SM_PREPARE;
		receiverIndex=0;
		timeLock=false;
		snmpStkDcptMemStubPtr->gSendTrapFlag=false;		
		netStartForTrap = true;
		return;
	}

}

#endif

/*********************************************************************
  Function:
 	 uint8_t SNMPValidateCommunity(uint8_t* community)
 
  Summary:			
 	 Validates community name for access control. 
 
  Description:		
     This function validates the community name for the mib access to NMS.
 	 The snmp community name received in the request pdu is validated for
 	 read and write community names. The agent gives an access to the mib
 	 variables only if the community matches with the predefined values.
  	 This routine also sets a gloabal flag to send trap if authentication
 	 failure occurs.
  
  PreCondition:
 	 SNMPInit is already called.
 
  parameters:
     community - Pointer to community string as sent by NMS.
 
  Returns:          
 	 This routine returns the community validation result as 
  	 READ_COMMUNITY or WRITE_COMMUNITY or INVALID_COMMUNITY	
 
  Remarks:
     This is a callback function called by module. User application must 
  	 implement this function and verify that community matches with 
 	 predefined value. This validation occurs for each NMS request.
 ********************************************************************/
uint8_t SNMPValidateCommunity(uint8_t * community)
{
	uint8_t i;
	uint8_t *ptr;
	

	SNMP_PROCESSING_MEM_INFO_PTRS snmpPktProcsMemPtrsInfo; 
	SNMP_STACK_DCPT_STUB * snmpStkDcptMemStubPtr=0;		
						
	SNMPGetPktProcessingDynMemStubPtrs(&snmpPktProcsMemPtrsInfo);
	snmpStkDcptMemStubPtr=snmpPktProcsMemPtrsInfo.snmpStkDynMemStubPtr;
		
#if !defined(SNMP_TRAP_DISABLED)
	snmpStkDcptMemStubPtr->gSendTrapFlag=false;//global flag to send Trap
	snmpStkDcptMemStubPtr->gGenericTrapNotification=ENTERPRISE_SPECIFIC;
	snmpStkDcptMemStubPtr->gSpecificTrapNotification=VENDOR_TRAP_DEFAULT; // Vendor specific trap code
#endif
	/*
	If the community name is encrypted in the request from the Manager,
	agent required to decrypt it to match with the community it is
	configured for. The response from the agent should contain encrypted community 
	name using the same encryption algorithm which Manager used while
	making the request.
	*/ 		

	// Validate that community string is a legal size
	if(strlen((char*)community) <= SNMP_COMMUNITY_MAX_LEN)
	{
        
		// Search to see if this is a write community.  This is done before 
		// searching read communities so that full read/write access is 
		// granted if a read and write community name happen to be the same.
		for(i = 0; i < SNMP_MAX_COMMUNITY_SUPPORT; i++)
		{
			ptr = SNMPRetrieveWriteCommunity(i);
			if(ptr == NULL)
				continue;
			if(*ptr == 0x00u)
				continue;
			if(strncmp((char*)community, (char*)ptr, SNMP_COMMUNITY_MAX_LEN) == 0)
			{
				return WRITE_COMMUNITY;
			}
		}
		
		// Did not find in write communities, search read communities
		for(i = 0; i < SNMP_MAX_COMMUNITY_SUPPORT; i++)
		{
			ptr = SNMPRetrieveReadCommunity(i);
			if(ptr == NULL)
				continue;
			if(*ptr == 0x00u)
				continue;
			if(strncmp((char*)community, (char*)ptr, SNMP_COMMUNITY_MAX_LEN) == 0)
			{
				return READ_COMMUNITY;
			}
		}
		
	}
#if !defined(SNMP_TRAP_DISABLED)	
	// Could not find any matching community, set up to send a trap
	snmpStkDcptMemStubPtr->gSpecificTrapNotification=VENDOR_TRAP_DEFAULT;
	snmpStkDcptMemStubPtr->gGenericTrapNotification=AUTH_FAILURE;
	snmpStkDcptMemStubPtr->gSendTrapFlag=true;
#endif	
	return INVALID_COMMUNITY;
	
}

/*********************************************************************
  Function:
  bool SNMPIsValidSetLen(SNMP_ID var, uint8_t len,uint8_t index)

  Summary: 	
	Validates the set variable data length to data type.
	
  Description:
  	This routine is used to validate the dyanmic variable data length
  	to the variable data type. It is used when SET request is processed.
  	This is a callback function called by module. User application
  	must implement this function.
  	
  PreCondition:
  	SNMPProcessSetVar() is called.
 
  Parameters:  
  	var	-	Variable id whose value is to be set
  	len	-	Length value that is to be validated.
 
  Return Values:  
  	true  - if given var can be set to given len
    false - if otherwise.
 
  Remarks:
  	This function will be called for only dynamic variables that are
  	defined as ASCII_STRING and OCTET_STRING (i.e. data length greater
  	than 4 bytes)
 ********************************************************************/
bool SNMPIsValidSetLen(SNMP_ID var, uint8_t len,uint8_t index)
{
#ifdef TCPIP_STACK_USE_SNMPV3_SERVER
	SNMPV3_PROCESSING_MEM_INFO_PTRS snmpv3PktProcessingMemPntr; 
	SNMPV3_STACK_DCPT_STUB * snmpv3EngnDcptMemoryStubPtr=0;		
					
	SNMPv3GetPktProcessingDynMemStubPtrs(&snmpv3PktProcessingMemPntr);
					
	snmpv3EngnDcptMemoryStubPtr=snmpv3PktProcessingMemPntr.snmpv3StkProcessingDynMemStubPtr;
#endif 

    switch(var)
    {
    case TRAP_COMMUNITY:
        if ( len < (uint8_t)TRAP_COMMUNITY_MAX_LEN)
            return true;
        break;
#ifdef TCPIP_STACK_USE_IPV6
	case IPV6_TRAP_COMMUNITY:
        if ( len < (uint8_t)TRAP_COMMUNITY_MAX_LEN)
            return true;
        break;
	case IPV6_TRAP_RECEIVER_IP:
		if(len == sizeof(IPV6_ADDR))
			return true;
		break;
#endif		
#ifdef TCPIP_STACK_USE_SNMPV3_SERVER			
		case USER_SECURITY_NAME:
			if(len <= USER_SECURITY_NAME_LEN)
			{
				memset(gSnmpv3UserSecurityName,'\0',USER_SECURITY_NAME_LEN);
				return true;
			}
			break;
		case USM_AUTH_KEY:
			if(len == AUTH_LOCALIZED_PASSWORD_KEY_LEN)
			{
				memset(snmpv3EngnDcptMemoryStubPtr->UserInfoDataBase[index].userAuthPswdLoclizdKey,'\0',AUTH_LOCALIZED_PASSWORD_KEY_LEN);
				return true;
			}
			break;
		case USM_PRIV_KEY:
			if(len == PRIV_LOCALIZED_PASSWORD_KEY_LEN)
			{
				memset(snmpv3EngnDcptMemoryStubPtr->UserInfoDataBase[index].userPrivPswdLoclizdKey,'\0',PRIV_LOCALIZED_PASSWORD_KEY_LEN);
				return true;
			}
			break;
		#if defined(SNMP_V1_V2_TRAP_WITH_SNMPV3)&& !defined(SNMP_TRAP_DISABLED)
				case SNMP_TARGET_SECURITY_NAME :			// 43.6.1.4.1.17095.5.1.1.4: READWRITE ASCII_STRING.
					if(len <= USER_SECURITY_NAME_LEN)
					{
						memset(gSnmpv3UserSecurityName,'\0',USER_SECURITY_NAME_LEN);
						return true;
					}
					break;
		#endif	/*defined(SNMP_V1_V2_TRAP_WITH_SNMPV3)&& !defined(SNMP_TRAP_DISABLED) */	
		
#endif /* TCPIP_STACK_USE_SNMPV3_SERVER */

#if defined(SYS_OUT_ENABLE)
    case LCD_DISPLAY:
        if ( len < sizeof(lcdMessage) + 1 )
            return true;
        break;
#endif /* defined(SYS_OUT_ENABLE) */
    }
    return false;
}

/*********************************************************************
  Function:  
 	bool SNMPSetVar(SNMP_ID var, SNMP_INDEX index,
                                   uint8_t ref, SNMP_VAL val)
 
  Summary:
  	This routine Set the mib variable with the requested value.
 
  Description:
  	This is a callback function called by module for the snmp
  	SET request.User application must modify this function 
 	for the new variables address.

  Precondition:
 	SNMPProcessVariables() is called.
 	
  Parameters:        
    var	-	Variable id whose value is to be set

    ref -   Variable reference used to transfer multi-byte data
            0 if first byte is set otherwise nonzero value to indicate
            corresponding byte being set.
            
    val -   Up to 4 byte data value.
            If var data type is uint8_t, variable
               value is in val->byte
            If var data type is uint16_t, variable
               value is in val->word
            If var data type is uint32_t, variable
               value is in val->dword.
            If var data type is IP_ADDRESS, COUNTER32,
               or GAUGE32, value is in val->dword
            If var data type is OCTET_STRING, ASCII_STRING
               value is in val->byte; multi-byte transfer
               will be performed to transfer remaining
               bytes of data.
 
  Return Values:  
  	true	-	if it is OK to set more byte(s).
    false	-	if otherwise.
 
  Remarks: 
  	This function may get called more than once depending on number 
	of bytes in a specific set request for given variable.
	only dynamic read-write variables needs to be handled.
********************************************************************/
bool SNMPSetVar(SNMP_ID var, SNMP_INDEX index, uint8_t ref, SNMP_VAL val)
{
#if defined(SNMP_V1_V2_TRAP_WITH_SNMPV3) && !defined(SNMP_TRAP_DISABLED)
	uint8_t tempUserNameLen = 0;
#endif /* defined(SNMP_V1_V2_TRAP_WITH_SNMPV3) && !defined(SNMP_TRAP_DISABLED) */

#ifdef TCPIP_STACK_USE_SNMPV3_SERVER	

	SNMPV3_PROCESSING_MEM_INFO_PTRS snmpv3PktProcessingMemPntr; 
	SNMPV3_STACK_DCPT_STUB * snmpv3EngnDcptMemoryStubPtr=0;		
					
	SNMPv3GetPktProcessingDynMemStubPtrs(&snmpv3PktProcessingMemPntr);
					
	snmpv3EngnDcptMemoryStubPtr=snmpv3PktProcessingMemPntr.snmpv3StkProcessingDynMemStubPtr;

#endif 
    switch(var)
    {
    case LED_D5:
        LED2_IO = val.byte;
        return true;

    case LED_D6:
        LED1_IO = val.byte;
        return true;
#if !defined(SNMP_TRAP_DISABLED)
    case TRAP_RECEIVER_IP:
        // Make sure that index is within our range.
        if ( index < trapInfo.Size )
        {
            // This is just an update to an existing entry.
            trapInfo.table[index].IPAddress.Val = val.dword;
            return true;
        }
        else if ( index < (uint8_t)TRAP_TABLE_SIZE )
        {
            // This is an addition to table.
            trapInfo.table[index].IPAddress.Val = val.dword;
            trapInfo.table[index].communityLen = 0;
            trapInfo.Size++;
            return true;
        }
        break;

    case TRAP_RECEIVER_ENABLED:
        // Make sure that index is within our range.
        if ( index < trapInfo.Size )
        {
            // Value of '1' means Enabled".
            if ( val.byte == 1u )
                trapInfo.table[index].Flags.bEnabled = 1;
            // Value of '0' means "Disabled.
            else if ( val.byte == 0u )
                trapInfo.table[index].Flags.bEnabled = 0;
            else
                // This is unknown value.
                return false;
            return true;
        }
        // Given index is more than our current table size.
        // If it is within our range, treat it as an addition to table.
        else if ( index < (uint8_t)TRAP_TABLE_SIZE )
        {
            // Treat this as an addition to table.
            trapInfo.Size++;
            trapInfo.table[index].communityLen = 0;
        }

        break;

    case TRAP_COMMUNITY:
        // Since this is a ASCII_STRING data type, SNMP will call with
        // SNMP_END_OF_VAR to indicate no more bytes.
        // Use this information to determine if we just added new row
        // or updated an existing one.
        if ( ref ==  SNMP_END_OF_VAR )
        {
            // Index equal to table size means that we have new row.
            if ( index == trapInfo.Size )
                trapInfo.Size++;

            // Length of string is one more than index.
            trapInfo.table[index].communityLen++;

            return true;
        }

        // Make sure that index is within our range.
        if ( index < trapInfo.Size )
        {
            // Copy given value into local buffer.
            trapInfo.table[index].community[ref] = val.byte;
            // Keep track of length too.
            // This may not be NULL terminate string.
            trapInfo.table[index].communityLen = (uint8_t)ref;
            return true;
        }
        break;
#ifdef TCPIP_STACK_USE_IPV6
	case IPV6_TRAP_RECEIVER_IP:
		// Make sure that index is within our range.
		if ( index < ipv6TrapInfo.Size )
		{
			// This is just an update to an existing entry.
			ipv6TrapInfo.table[index].IPv6Address.v[ref] = val.byte;
			return true;
		}
		break;
	
	case IPV6_TRAP_ENABLED:
		// Make sure that index is within our range.
		if ( index < ipv6TrapInfo.Size )
		{
			// Value of '1' means Enabled".
			if ( val.byte == 1u )
				ipv6TrapInfo.table[index].Flags.bEnabled = 1;
			// Value of '0' means "Disabled.
			else if ( val.byte == 0u )
				ipv6TrapInfo.table[index].Flags.bEnabled = 0;
			else
				// This is unknown value.
				return false;
			return true;
		}
		// Given index is more than our current table size.
		// If it is within our range, treat it as an addition to table.
		else if ( index < (uint8_t)TRAP_TABLE_SIZE )
		{
			// Treat this as an addition to table.
			ipv6TrapInfo.Size++;
			ipv6TrapInfo.table[index].communityLen = 0;
		}
	
		break;
	
	case IPV6_TRAP_COMMUNITY:
		// Since this is a ASCII_STRING data type, SNMP will call with
		// SNMP_END_OF_VAR to indicate no more bytes.
		// Use this information to determine if we just added new row
		// or updated an existing one.
		if ( ref ==  SNMP_END_OF_VAR )
		{
			// Index equal to table size means that we have new row.
			if ( index == ipv6TrapInfo.Size )
				ipv6TrapInfo.Size++;
	
			// Length of string is one more than index.
			ipv6TrapInfo.table[index].communityLen++;
	
			return true;
		}
	
		// Make sure that index is within our range.
		if ( index < ipv6TrapInfo.Size )
		{
			// Copy given value into local buffer.
			ipv6TrapInfo.table[index].community[ref] = val.byte;
			// Keep track of length too.
			// This may not be NULL terminate string.
			ipv6TrapInfo.table[index].communityLen = (uint8_t)ref;
			return true;
		}
		break;
#endif /* TCPIP_STACK_USE_IPV6 */	 
#endif /* !defined(SNMP_TRAP_DISABLED) */
#ifdef TCPIP_STACK_USE_SNMPV3_SERVER	
	case USM_AUTH_PROT:
		if(index>SNMPV3_USM_MAX_USER)
			return false;
		if(val.byte == hmacMD5Auth)
		{
			snmpv3EngnDcptMemoryStubPtr->UserInfoDataBase[index].userHashType = SNMPV3_HAMC_MD5;
		}
		else if(val.byte == hmacSHAAuth)
		{
			snmpv3EngnDcptMemoryStubPtr->UserInfoDataBase[index].userHashType = SNMPV3_HMAC_SHA1;
		}
		else if(val.byte == noAuthProtocol)
		{
			snmpv3EngnDcptMemoryStubPtr->UserInfoDataBase[index].userHashType = SNMPV3_NO_HMAC_AUTH;
		}
		else
			return false;

		SNMPv3USMAuthPrivPswdLocalization(index);
		SNMPv3ComputeHMACIpadOpadForAuthLoclzedKey(index);
		
		//gSnmpV3USMDataBase[index].userAuthType = val.byte;
 		return true;
		
	case USER_SECURITY_NAME:
		/* validate user security length*/
        // Since this is a ASCII_STRING data type, SNMP will call with
        // SNMP_END_OF_VAR to indicate no more bytes.
        // Use this information to determine if we just added new row
        // or updated an existing one.
        if ( ref ==  SNMP_END_OF_VAR )
        {
			//snmpV3UserDataBase[index].userName[ref] = '\0';
			// restrict the user security name "initial"
			if(strncmp((char*)gSnmpv3UserSecurityName,"initial",strlen((char*)gSnmpv3UserSecurityName))== 0)
				return false;
			snmpv3EngnDcptMemoryStubPtr->UserInfoDataBase[index].userNameLength = strlen((char*)gSnmpv3UserSecurityName);
			memset(snmpv3EngnDcptMemoryStubPtr->UserInfoDataBase[index].userName,'\0',USER_SECURITY_NAME_LEN);
			strncpy((char*)snmpv3EngnDcptMemoryStubPtr->UserInfoDataBase[index].userName,
				(char*)gSnmpv3UserSecurityName,strlen((char*)gSnmpv3UserSecurityName));
			snmpv3EngnDcptMemoryStubPtr->UserInfoDataBase[index].userNameLength = strlen((char*)snmpv3EngnDcptMemoryStubPtr->UserInfoDataBase[index].userName);
            return true;
        }
        // Make sure that index is within our range.
        if ( index < SNMPV3_USM_MAX_USER )
        {
            // Copy given value into local buffer.
            gSnmpv3UserSecurityName[ref]=val.byte;
            return true;
        }
		break;
	case USM_AUTH_KEY:
        if ( ref ==  SNMP_END_OF_VAR )
        {
			//snmpV3UserDataBase[index].userAuthPswdLoclizdKey[ref] = '\0';
			SNMPv3ComputeHMACIpadOpadForAuthLoclzedKey(index);
            return true;
        }
        // Make sure that index is within our range.
        if ( index < SNMPV3_USM_MAX_USER )
        {
            // Copy given value into local buffer.
            snmpv3EngnDcptMemoryStubPtr->UserInfoDataBase[index].userAuthPswdLoclizdKey[ref]=val.byte;
            return true;
        }
		break;
	case USM_PRIV_PROT:
		if(index>SNMPV3_USM_MAX_USER)
			return false;
		if((snmpv3EngnDcptMemoryStubPtr->UserInfoDataBase[index].userHashType == SNMPV3_NO_HMAC_AUTH) 
			&& (val.byte != noPrivProtocol))
		{
			return false;
		}
		if(val.byte == aesPrivProtocol)
		{
			snmpv3EngnDcptMemoryStubPtr->UserInfoDataBase[index].userPrivType = SNMPV3_AES_PRIV;
		}
		else if(val.byte == desPrivProtocol)
		{
			snmpv3EngnDcptMemoryStubPtr->UserInfoDataBase[index].userPrivType = SNMPV3_DES_PRIV;
		}
		else if(val.byte == noPrivProtocol)
		{
			snmpv3EngnDcptMemoryStubPtr->UserInfoDataBase[index].userPrivType = SNMPV3_NO_PRIV;
		}
		else
			return false;
		//gSnmpV3USMDataBase[index].userPrivType = val.byte;
		return true;
		
	case USM_PRIV_KEY:
        if ( ref ==  SNMP_END_OF_VAR )
        {
			//snmpV3UserDataBase[index].userPrivPswdLoclizdKey[ref] = '\0';
            return true;
        }
        // Make sure that index is within our range.
        if ( index < SNMPV3_USM_MAX_USER )
        {
            // Copy given value into local buffer.
            snmpv3EngnDcptMemoryStubPtr->UserInfoDataBase[index].userPrivPswdLoclizdKey[ref]=val.byte;
            return true;
        }
		break;
#if defined(SNMP_V1_V2_TRAP_WITH_SNMPV3) && !defined(SNMP_TRAP_DISABLED)
	case SNMP_TARGET_INDEX_ID : 		// 43.6.1.4.1.17095.5.1.1.1: READONLY uint8_t.
		break;
	case SNMP_TARGET_MP_MODEL : 		// 43.6.1.4.1.17095.5.1.1.2: READWRITE uint8_t.
		if(index < SNMPV3_USM_MAX_USER)
		{
			snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[index].messageProcessingModelType 
				= val.byte;
			return true;
		}
		break;
	case  SNMP_TARGET_SECURITY_MODEL :			// 43.6.1.4.1.17095.5.1.1.3: READWRITE uint8_t.
		{
			snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[index].securityModelType
				= val.byte;
			return true;
		}
		break;
	case SNMP_TARGET_SECURITY_NAME :			// 43.6.1.4.1.17095.5.1.1.4: READWRITE ASCII_STRING.
        if ( ref ==  SNMP_END_OF_VAR )
        {
        	UINT8 userIndex = 0;
			// restrict the user security name "initial"
			tempUserNameLen = strlen((char*)gSnmpv3UserSecurityName);
			if(strncmp((char*)gSnmpv3UserSecurityName,"initial",tempUserNameLen)== 0)
				return false;
			// check if the target security name is the part of the user security name table,
			// if target security name is not present in that table then return false.
			
			for(userIndex=0;userIndex<SNMPV3_USM_MAX_USER;userIndex++)
			{
				if(strncmp((char*)gSnmpv3UserSecurityName,(char*)snmpv3EngnDcptMemoryStubPtr->UserInfoDataBase[userIndex].userName,tempUserNameLen)== 0)
					break;
			}
			if(userIndex == SNMPV3_USM_MAX_USER)
				return false;
			
			memset(snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[index].userSecurityName,'\0',USER_SECURITY_NAME_LEN);
			strncpy((char*)snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[index].userSecurityName,
				(char*)gSnmpv3UserSecurityName,strlen((char*)gSnmpv3UserSecurityName));
            return true;
        }
        // Make sure that index is within our range.
        if ( index < SNMPV3_USM_MAX_USER )
        {
            // Copy given value into local buffer.
            gSnmpv3UserSecurityName[ref]=val.byte;
            return true;
        }
		break;
	case SNMP_TARGET_SECURITY_LEVEL :			// 43.6.1.4.1.17095.5.1.1.5: READWRITE uint8_t.
		{
			if(SNMPv3CmprTrapSecNameAndSecLvlWithUSMDb(index,strlen((char*)snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[index].userSecurityName),
				snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[index].userSecurityName,
				(STD_BASED_SNMPV3_SECURITY_LEVEL)val.byte)!= true)
				return false;
			snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[index].securityLevelType
				= val.byte;
			return true;
		}
		break;
#endif 	/* defined(SNMP_V1_V2_TRAP_WITH_SNMPV3) && !defined(SNMP_TRAP_DISABLED) */	
#endif  /* TCPIP_STACK_USE_SNMPV3_SERVER */
#if defined(SYS_OUT_ENABLE)
    case LCD_DISPLAY:      
	  // Copy all bytes until all bytes are transferred
	 if ( ref != SNMP_END_OF_VAR )
	 {
		 lcdMessage[ref] = val.byte;
		 lcdMessage[ref+1] = 0;
	 }
	 else
	 {
		 SYS_OUT_MESSAGE_LINE((char*)lcdMessage,2);
	 }      

     return true;
#endif		

    }

    return false;
}


/*********************************************************************
  Function:        
  	bool SNMPGetExactIndex(SNMP_ID var,SNMP_INDEX index)

  Summary:
  	To search for exact index node in case of a Sequence variable.
	
  Description:    
  	This is a callback function called by SNMP module.
    SNMP user must implement this function in user application and 
    provide appropriate data when called.  This function will only
    be called for OID variable of type sequence.
    
  PreCondition: 
  	None
 
  Parameters:
  	var		-	Variable id as per mib.h (input)
  	index      -	 Index of variable (input)
 
  Return Values:
  	true	-	 If the exact index value exists for given variable at given
                 index.
    false	-	 Otherwise.
 
  Remarks:
	  Only sequence index needs to be handled in this function.
 ********************************************************************/
bool SNMPGetExactIndex(SNMP_ID var, SNMP_INDEX *index)
{
    
    switch(var)
    {
#if !defined(SNMP_TRAP_DISABLED)
	    case TRAP_RECEIVER_ID:
	    case TRAP_RECEIVER_ENABLED:
		case TRAP_RECEIVER_IP:
		case TRAP_COMMUNITY:
	        // There is no next possible index if table itself is empty.
	        if(trapInfo.Size == 0u )
	            return false;
            if(*index == SNMP_INDEX_INVALID)
            {
                *index = 0;
            }

	        if(*index < trapInfo.Size)
	        {
	            return true;
	        }
	    break;
#ifdef TCPIP_STACK_USE_IPV6
    case IPV6_TRAP_RECEIVER_ID:
    case IPV6_TRAP_ENABLED:
	case IPV6_TRAP_RECEIVER_IP:
	case IPV6_TRAP_COMMUNITY:
        // There is no next possible index if table itself is empty.
        if ( ipv6TrapInfo.Size == 0u )
            return false;
        if(*index == SNMP_INDEX_INVALID)
        {
            *index = 0;
        }
        if(*index < ipv6TrapInfo.Size)
        {
            return true;
        }
    break;
#endif /* TCPIP_STACK_USE_IPV6*/
#endif /* !defined(SNMP_TRAP_DISABLED) */
#ifdef TCPIP_STACK_USE_SNMPV3_SERVER		
		case USM_INDEX_ID:
		case USM_AUTH_KEY:
		case USM_AUTH_PROT:
		case USER_SECURITY_NAME:
		case USM_PRIV_KEY:
		case USM_PRIV_PROT:            
            if(*index == SNMP_INDEX_INVALID)
            {
                *index = 0;
            }
	        if ( *index < SNMPV3_USM_MAX_USER)
	        {
	            return true;
	        }
		break;
#if defined(SNMP_V1_V2_TRAP_WITH_SNMPV3) && !defined(SNMP_TRAP_DISABLED)
		case SNMP_TARGET_INDEX_ID : 		// 43.6.1.4.1.17095.5.1.1.1: READONLY uint8_t.
		case SNMP_TARGET_MP_MODEL : 		// 43.6.1.4.1.17095.5.1.1.2: READWRITE uint8_t.
		case  SNMP_TARGET_SECURITY_MODEL :			// 43.6.1.4.1.17095.5.1.1.3: READWRITE uint8_t.
		case SNMP_TARGET_SECURITY_NAME :			// 43.6.1.4.1.17095.5.1.1.4: READWRITE ASCII_STRING.
		case SNMP_TARGET_SECURITY_LEVEL :			// 43.6.1.4.1.17095.5.1.1.5: READWRITE uint8_t.
		
            if(*index == SNMP_INDEX_INVALID)
            {
                *index = 0;
            }
	        if ( *index < SNMPV3_USM_MAX_USER)
	        {
	            return true;
	        }
		break;
#endif /*  defined(SNMP_V1_V2_TRAP_WITH_SNMPV3) && !defined(SNMP_TRAP_DISABLED) */
#endif /* TCPIP_STACK_USE_SNMPV3_SERVER */
    }
    return false;
}


/*********************************************************************
  Function:        
  	bool SNMPGetNextIndex(SNMP_ID var,SNMP_INDEX* index)

  Summary:
  	To search for next index node in case of a Sequence variable.
	
  Description:    
  	This is a callback function called by SNMP module.
    SNMP user must implement this function in user application and 
    provide appropriate data when called.  This function will only
    be called for OID variable of type sequence.
    
  PreCondition: 
  	None
 
  Parameters:
  	var		-	Variable id whose value is to be returned
  	index   -	Next Index of variable that should be transferred
 
  Return Values:
  	true	-	 If a next index value exists for given variable at given
                 index and index parameter contains next valid index.
    false	-	 Otherwise.
 
  Remarks:
	  Only sequence index needs to be handled in this function.
 ********************************************************************/
bool SNMPGetNextIndex(SNMP_ID var, SNMP_INDEX* index)
{
    SNMP_INDEX tempIndex;

    tempIndex = *index;

    switch(var)
    {
#if !defined(SNMP_TRAP_DISABLED)
    case TRAP_RECEIVER_ID:
	case TRAP_RECEIVER_ENABLED:
	case TRAP_RECEIVER_IP:
	case TRAP_COMMUNITY:
        // There is no next possible index if table itself is empty.
        if ( trapInfo.Size == 0u )
		{
            return false;
		}
        // INDEX_INVALID means start with first index.
        if ( tempIndex == (uint8_t)SNMP_INDEX_INVALID )
        {
            *index = 0;
            return true;
        }
        else if ( tempIndex < (trapInfo.Size-1) )
        {
            *index = tempIndex+1;
            return true;
		}
      	break;
#ifdef TCPIP_STACK_USE_IPV6
	case IPV6_TRAP_RECEIVER_ID:
	case IPV6_TRAP_ENABLED:
	case IPV6_TRAP_RECEIVER_IP:
	case IPV6_TRAP_COMMUNITY:
		// There is no next possible index if table itself is empty.
		if ( ipv6TrapInfo.Size == 0u )
		{
			return false;
		}
		// INDEX_INVALID means start with first index.
		if ( tempIndex == (uint8_t)SNMP_INDEX_INVALID )
		{
			*index = 0;
			return true;
		}
		else if ( tempIndex < (ipv6TrapInfo.Size-1) )
		{
			*index = tempIndex+1;
			return true;
		}
#endif		
		break;
#endif /* !defined(SNMP_TRAP_DISABLED) */
#ifdef TCPIP_STACK_USE_SNMPV3_SERVER	
	case USM_INDEX_ID:
	case USM_AUTH_KEY:
	case USM_AUTH_PROT:
	case USER_SECURITY_NAME:
	case USM_PRIV_KEY:
	case USM_PRIV_PROT:
        if ( tempIndex == (uint8_t)SNMP_INDEX_INVALID )
        {
            *index = 0;
            return true;
        }
        else if ( tempIndex < (SNMPV3_USM_MAX_USER-1) )
        {
            *index = tempIndex+1;
            return true;
        }
        break;
#if defined(SNMP_V1_V2_TRAP_WITH_SNMPV3) && !defined(SNMP_TRAP_DISABLED)
	case SNMP_TARGET_INDEX_ID : 		// 43.6.1.4.1.17095.5.1.1.1: READONLY uint8_t.
	case SNMP_TARGET_MP_MODEL : 		// 43.6.1.4.1.17095.5.1.1.2: READWRITE uint8_t.
	case SNMP_TARGET_SECURITY_MODEL :			// 43.6.1.4.1.17095.5.1.1.3: READWRITE uint8_t.
	case SNMP_TARGET_SECURITY_NAME :			// 43.6.1.4.1.17095.5.1.1.4: READWRITE ASCII_STRING.
	case SNMP_TARGET_SECURITY_LEVEL :			// 43.6.1.4.1.17095.5.1.1.5: READWRITE uint8_t.
        if ( tempIndex == (uint8_t)SNMP_INDEX_INVALID )
        {
            *index = 0;
            return true;
        }
        else if ( tempIndex < (SNMPV3_USM_MAX_USER-1) )
        {
            *index = tempIndex+1;
            return true;
        }
	break;
#endif	/* defined(SNMP_V1_V2_TRAP_WITH_SNMPV3) && !defined(SNMP_TRAP_DISABLED) */
#endif /* TCPIP_STACK_USE_SNMPV3_SERVER */
    }
    return false;
}


/*********************************************************************
  Function:
  	bool SNMPGetVar(SNMP_ID var, SNMP_INDEX index,uint8_t* ref, SNMP_VAL* val)
                                   
  Summary:
  	Used to Get/collect OID variable information.

  Description:
 	This is a callback function called by SNMP module. SNMP user must 
 	implement this function in user application and provide appropriate
 	data when called.
   	
  PreCondition:
  	None
 
  parameters:
  	var		-	Variable id whose value is to be returned
    index   -	Index of variable that should be transferred
    ref     -   Variable reference used to transfer
              	multi-byte data
                It is always SNMP_START_OF_VAR when very
                first byte is requested.
                Otherwise, use this as a reference to
                keep track of multi-byte transfers.
    val     -	Pointer to up to 4 byte buffer.
                If var data type is uint8_t, transfer data
                  in val->byte
                If var data type is uint16_t, transfer data in
                  val->word
                If var data type is uint32_t, transfer data in
                  val->dword
                If var data type is IP_ADDRESS, transfer data
                  in val->v[] or val->dword
                If var data type is COUNTER32, TIME_TICKS or
                  GAUGE32, transfer data in val->dword
                If var data type is ASCII_STRING or OCTET_STRING
                  transfer data in val->byte using multi-byte
                  transfer mechanism.
 
  Return Values:
  	true	-	If a value exists for given variable at given index.
    false 	-	Otherwise.
 
  Remarks:
 	None.
 ********************************************************************/
bool SNMPGetVar(SNMP_ID var, SNMP_INDEX index, uint8_t* ref, SNMP_VAL* val)
{
    uint8_t myRef;
   static uint8_t AN0String[8];

   #ifdef TCPIP_STACK_USE_SNMPV3_SERVER	

   SNMPV3_PROCESSING_MEM_INFO_PTRS snmpv3PktProcessingMemPntr; 
   SNMPV3_STACK_DCPT_STUB * snmpv3EngnDcptMemoryStubPtr=0;	   
						   
   SNMPv3GetPktProcessingDynMemStubPtrs(&snmpv3PktProcessingMemPntr);
						   
   snmpv3EngnDcptMemoryStubPtr=snmpv3PktProcessingMemPntr.snmpv3StkProcessingDynMemStubPtr;

	#endif 
    // Convert potentiometer result into ASCII string
    uitoa((uint16_t) ADC1BUF0, AN0String);

    myRef = *ref;

    switch(var)
    {
    case SYS_UP_TIME:
    {
	 
        SYS_TICK dw10msTicks;
        dw10msTicks = (SYS_TICK_Get() * 100ull)/SYS_TICK_TicksPerSecondGet();

        val->dword = dw10msTicks;
        return true;
    }    

    case LED_D5:
        val->byte = LED2_IO;
        return true;

    case LED_D6:
        val->byte = LED1_IO;
        return true;

    case PUSH_BUTTON:
        // There is only one button - meaning only index of 0 is allowed.
        val->byte = BUTTON0_IO;
        return true;

    case ANALOG_POT0:
        val->word = atoi((char*)AN0String);
        return true;
#if !defined(SNMP_TRAP_DISABLED)
    case TRAP_RECEIVER_ID:
        if ( index < trapInfo.Size )
        {
            val->byte = index;
            return true;
        }
        break;

    case TRAP_RECEIVER_ENABLED:
        if ( index < trapInfo.Size )
        {
            val->byte = trapInfo.table[index].Flags.bEnabled;
            return true;
        }
        break;

    case TRAP_RECEIVER_IP:
        if ( index < trapInfo.Size )
        {
            val->dword = trapInfo.table[index].IPAddress.Val;
            return true;
        }
        break;

    case TRAP_COMMUNITY:
        if ( index < trapInfo.Size )
        {
            if ( trapInfo.table[index].communityLen == 0u )
                *ref = SNMP_END_OF_VAR;
            else
            {
                val->byte = trapInfo.table[index].community[myRef];

                myRef++;

                if ( myRef == trapInfo.table[index].communityLen )
                    *ref = SNMP_END_OF_VAR;
                else
                    *ref = myRef;
            }
            return true;
        }
        break;
#ifdef	TCPIP_STACK_USE_IPV6
	case IPV6_TRAP_RECEIVER_ID:
		if ( index < ipv6TrapInfo.Size )
		{
			val->byte = index;
			return true;
		}
		break;

	case IPV6_TRAP_ENABLED:
		if ( index < ipv6TrapInfo.Size )
		{
			val->byte = ipv6TrapInfo.table[index].Flags.bEnabled;
			return true;
		}
		break;

	case IPV6_TRAP_RECEIVER_IP:
		if ( index < ipv6TrapInfo.Size )
		{
			val->byte = ipv6TrapInfo.table[index].IPv6Address.v[myRef];

			myRef++;

			if ( myRef == 16 )
				*ref = SNMP_END_OF_VAR;
			else
				*ref = myRef;
			return true;
		}
		break;

	case IPV6_TRAP_COMMUNITY:
		if ( index < ipv6TrapInfo.Size )
		{
			if ( ipv6TrapInfo.table[index].communityLen == 0u )
				*ref = SNMP_END_OF_VAR;
			else
			{
				val->byte = ipv6TrapInfo.table[index].community[myRef];

				myRef++;

				if ( myRef == ipv6TrapInfo.table[index].communityLen )
					*ref = SNMP_END_OF_VAR;
				else
					*ref = myRef;
			}
			return true;
		}
		break;
#endif /* TCPIP_STACK_USE_IPV6 */


#endif /* !defined(SNMP_TRAP_DISABLED) */
#ifdef TCPIP_STACK_USE_SNMPV3_SERVER	
	case ENGINE_ID:
		if(snmpv3EngnDcptMemoryStubPtr->SnmpEngnIDLength == 0u)
			*ref = SNMP_END_OF_VAR;
		else
		{
			val->byte = snmpv3EngnDcptMemoryStubPtr->SnmpEngineID[myRef];
			
			myRef++;
			
			if ( myRef == snmpv3EngnDcptMemoryStubPtr->SnmpEngnIDLength )
				*ref = SNMP_END_OF_VAR;
			else
				*ref = myRef;
		}
		return true;
	case ENGINE_BOOT:
		val->dword = (uint32_t)snmpv3EngnDcptMemoryStubPtr->SnmpEngineBoots;
		return true;
		
	case ENGINE_TIME:
		val->dword = (uint32_t)snmpv3EngnDcptMemoryStubPtr->SnmpEngineTime.Val;
		return true;
		
	case ENGINE_MAX_MSG:
		val->dword = (uint16_t)snmpv3EngnDcptMemoryStubPtr->SnmpEngnMaxMsgSize.Val; 
		return true;

	case USM_INDEX_ID:
        if ( index < SNMPV3_USM_MAX_USER)
        {
            val->byte = index;
            return true;
        }
        break;
	case USM_AUTH_PROT:
		if(index < SNMPV3_USM_MAX_USER)
		{
			if(snmpv3EngnDcptMemoryStubPtr->UserInfoDataBase[index].userHashType == SNMPV3_HAMC_MD5)
				val->byte = hmacMD5Auth;
			else if(snmpv3EngnDcptMemoryStubPtr->UserInfoDataBase[index].userHashType == SNMPV3_HMAC_SHA1)
				val->byte = hmacSHAAuth;
			else
				val->byte = noAuthProtocol;				
		}
		else
			return false;
		return true;
	case USM_PRIV_PROT:
		// code change is required 
		if(index < SNMPV3_USM_MAX_USER)
		{
			if(snmpv3EngnDcptMemoryStubPtr->UserInfoDataBase[index].userPrivType == SNMPV3_AES_PRIV)
				val->byte = aesPrivProtocol;
			else if(snmpv3EngnDcptMemoryStubPtr->UserInfoDataBase[index].userPrivType == SNMPV3_DES_PRIV)
				val->byte = desPrivProtocol;
			else
				val->byte = noPrivProtocol; 			
		}
		else
			return false;
		return true;
	case USER_SECURITY_NAME:
		if(index < SNMPV3_USM_MAX_USER)
		{
            if ( snmpv3EngnDcptMemoryStubPtr->UserInfoDataBase[index].userNameLength == 0u )
                *ref = SNMP_END_OF_VAR;
            else
            {
                val->byte = snmpv3EngnDcptMemoryStubPtr->UserInfoDataBase[index].userName[myRef];

                myRef++;

                if ( myRef == snmpv3EngnDcptMemoryStubPtr->UserInfoDataBase[index].userNameLength )
                    *ref = SNMP_END_OF_VAR;
                else
                    *ref = myRef;
            }
            return true;
		}
		break;
	case USM_AUTH_KEY:
		if(index < SNMPV3_USM_MAX_USER)
		{
            val->byte = snmpv3EngnDcptMemoryStubPtr->UserInfoDataBase[index].userAuthPswdLoclizdKey[myRef];

            myRef++;

            if ( myRef == AUTH_LOCALIZED_PASSWORD_KEY_LEN)
                *ref = SNMP_END_OF_VAR;
            else
                *ref = myRef;
            return true;
		}
		break;
	case USM_PRIV_KEY:
		// code change is required / for temp- same auth passwd string is returned.
		if(index < SNMPV3_USM_MAX_USER)
		{            
            val->byte = snmpv3EngnDcptMemoryStubPtr->UserInfoDataBase[index].userPrivPswdLoclizdKey[myRef];

            myRef++;

            if ( myRef == PRIV_LOCALIZED_PASSWORD_KEY_LEN)
                *ref = SNMP_END_OF_VAR;
            else
                *ref = myRef;
            return true;
		}
		break;
#if defined(SNMP_V1_V2_TRAP_WITH_SNMPV3) && !defined(SNMP_TRAP_DISABLED)
	case SNMP_TARGET_INDEX_ID :			// 43.6.1.4.1.17095.5.1.1.1: READONLY uint8_t.
		if(index < SNMPV3_USM_MAX_USER)
		{
			val->byte = index;
			return true;
		}
		break;
	case SNMP_TARGET_MP_MODEL :			// 43.6.1.4.1.17095.5.1.1.2: READWRITE uint8_t.		
		if(index < SNMPV3_USM_MAX_USER)
		{
			val->byte = snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[index].messageProcessingModelType;
			return true;
		}
		break;
	case  SNMP_TARGET_SECURITY_MODEL :			// 43.6.1.4.1.17095.5.1.1.3: READWRITE uint8_t.
		if(index < SNMPV3_USM_MAX_USER)
		{
			val->byte = snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[index].securityModelType;
			return true;
		}
		break;
	case SNMP_TARGET_SECURITY_NAME : 			// 43.6.1.4.1.17095.5.1.1.4: READWRITE ASCII_STRING.
		if(index < SNMPV3_USM_MAX_USER)
		{
            if ( strlen((char*)snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[index].userSecurityName) == 0u )
                *ref = SNMP_END_OF_VAR;
			else
			{
	            val->byte = snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[index].userSecurityName[myRef];

	            myRef++;

	            if ( myRef == strlen((char*)snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[index].userSecurityName) )
	                *ref = SNMP_END_OF_VAR;
	            else
	                *ref = myRef;
			}
            return true;
		}
		break;
	case SNMP_TARGET_SECURITY_LEVEL :			// 43.6.1.4.1.17095.5.1.1.5: READWRITE uint8_t.
		if(index < SNMPV3_USM_MAX_USER)
		{
			val->byte = snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[index].securityLevelType;
			return true;
		}
		break;
#endif /* defined(SNMP_V1_V2_TRAP_WITH_SNMPV3) && !defined(SNMP_TRAP_DISABLED)*/
#endif /* TCPIP_STACK_USE_SNMPV3_SERVER */
#if defined(SYS_OUT_ENABLE)
    case LCD_DISPLAY:
		strncpy((char*)lcdMessage, (char*)SYS_OUT_GET_LCD_MESSAGE(), sizeof(lcdMessage)-1);
		if ( lcdMessage[0] == 0u )
            myRef = SNMP_END_OF_VAR;
        else
        {
            val->byte = lcdMessage[myRef++];
            if ( lcdMessage[myRef] == 0u )
                myRef = SNMP_END_OF_VAR;
        }

        *ref = myRef;
        return true;
        break;
#endif /* SYS_OUT_ENABLE */
    }

    return false;
}


/*********************************************************************
  Function:
  	bool SNMPIdRecrdValidation(PDU_INFO * pduPtr,OID_INFO *var,uint8_t * oidValuePtr,uint8_t oidLen)
                                   
  Summary:
  	Used to Restrict the access dynamic and non dynamic OID string for A perticular SNMP Version.

  Description:
 	This is a callback function called by SNMP module. SNMP user must 
 	implement this function as per SNMP version. One need to add the new SNMP
 	MIB IDs hereas per SNMP version.
 	e.g - SYS_UP_TIME (250) is common for V1/V2/V3
 	ENGINE_ID - is the part of V3, So put the all the SNMPv3 var ids within 
 	Macro TCPIP_STACK_USE_SNMPV3_SERVER.   
 	
  PreCondition:
  	None
 
  parameters:
  	var		-	Variable rec whose record id need to be validated
  	oidValuePtr - OID Value
  	oidLen - oidValuePtr length
    
  Return Values:
  	true	-	If a Var ID exists .
    	false 	-	Otherwise.
 
  Remarks:
 	None.
 ********************************************************************/
bool SNMPIdRecrdValidation(PDU_INFO * pduPtr,OID_INFO *var,uint8_t * oidValuePtr,uint8_t oidLen)
{
	
	int i=0,j=0;
	int len=0;
	bool flag=false;
	uint8_t size=0;

	if(var == NULL)
		return false;
	
	if(!var->nodeInfo.Flags.bIsIDPresent)
	{
		if(oidValuePtr == NULL)
			return false;
		
		for(i=0; i< SNMP_MAX_NON_REC_ID_OID; i++)
		{
			if((pduPtr->snmpVersion != SNMP_V3) && 
				(gSnmpNonMibRecInfo[i].version == SNMP_V3))
				continue;
			
			size = strlen((char*)gSnmpNonMibRecInfo[i].oidstr);
			if(size == 0)
				continue;
			if( size <= oidLen)
				len = size;
			else
				continue;

			// find the first unmatching byte
			while(len--)
			{
				if(gSnmpNonMibRecInfo[i].oidstr[j] != oidValuePtr[j])
				{
					flag = false;
					j=0;
					break;
				}
				else
				{
					flag = true;
					j++;
				}
			}
			if(flag == true)
			{
				return true;
			}
		}			
		return false;
	}
	switch(var->id)
	{
		case MICROCHIP:
		case SYS_UP_TIME:
		case LED_D5:
	    case LED_D6:
	    case PUSH_BUTTON:
	   	case ANALOG_POT0:
#if !defined(SNMP_TRAP_DISABLED)	   	
	    case TRAP_RECEIVER_ID:
	    case TRAP_RECEIVER_ENABLED:
	    case TRAP_RECEIVER_IP:
	    case TRAP_COMMUNITY:
#ifdef TCPIP_STACK_USE_IPV6
		case IPV6_TRAP_RECEIVER_ID:
		case IPV6_TRAP_ENABLED:
		case IPV6_TRAP_RECEIVER_IP:
		case IPV6_TRAP_COMMUNITY:	
#endif			
#endif /* !defined(SNMP_TRAP_DISABLED) */
#if defined(SYS_OUT_ENABLE)
		case LCD_DISPLAY:
#endif /* SYS_OUT_ENABLE */

		return true;
	}
	
#ifdef TCPIP_STACK_USE_SNMPV3_SERVER
	if(pduPtr->snmpVersion == SNMP_V3)
	{
		if(!var->nodeInfo.Flags.bIsIDPresent)
			return true;
		
		switch(var->id)
		{

			case ENGINE_ID:
			case ENGINE_BOOT:
			case ENGINE_TIME:
			case ENGINE_MAX_MSG:
			case USM_INDEX_ID:
			case USM_AUTH_PROT:
			case USM_PRIV_PROT:
			case USER_SECURITY_NAME:
			case USM_AUTH_KEY:
			case USM_PRIV_KEY:
#if defined(SNMP_V1_V2_TRAP_WITH_SNMPV3) && !defined(SNMP_TRAP_DISABLED)
			case SNMP_TARGET_INDEX_ID :			// 43.6.1.4.1.17095.5.1.1.1: READONLY uint8_t.
			case SNMP_TARGET_MP_MODEL :			// 43.6.1.4.1.17095.5.1.1.2: READWRITE uint8_t.		
			case  SNMP_TARGET_SECURITY_MODEL :			// 43.6.1.4.1.17095.5.1.1.3: READWRITE uint8_t.
			case SNMP_TARGET_SECURITY_NAME : 			// 43.6.1.4.1.17095.5.1.1.4: READWRITE ASCII_STRING.
			case SNMP_TARGET_SECURITY_LEVEL :			// 43.6.1.4.1.17095.5.1.1.5: READWRITE uint8_t.
		
#endif /*SNMP_V1_V2_TRAP_WITH_SNMPV3 */
			return true;
		}
	}
#endif /* TCPIP_STACK_USE_SNMPV3_SERVER */

    return false;
}


#if !defined(SNMP_TRAP_DISABLED)
/*********************************************************************
  Function:
  	static SYS_TICK SNMPGetTimeStamp(void)
                                   
  Summary:
	Obtains the current Tick value for the SNMP time stamp.

  Description:
	This function retrieves the absolute time measurements for 
	SNMP time stamp in tens of milliseconds.

  PreCondition:
  	None
 
  parameters:
  	None
 
  Return Values:
  	timeStamp - SYS_TICK timevalue
 
  Remarks:
 	None.
 ********************************************************************/
static SYS_TICK SNMPGetTimeStamp(void)
{
    SYS_TICK    timeStamp;
    timeStamp = (SYS_TICK_Get() * 100ull)/SYS_TICK_TicksPerSecondGet();

    return timeStamp;

}
#endif /* !defined(SNMP_TRAP_DISABLED)*/
#endif	//#if defined(TCPIP_STACK_USE_SNMP_SERVER)
