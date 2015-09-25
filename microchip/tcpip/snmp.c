/*******************************************************************************
  Simple Network Management Protocol (SNMP) Version 1 Agent
  Simple Network Management Protocol (SNMP) Version 2 community based Agent

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Provides SNMP API for doing stuff
    -Reference: RFC 1157 (for SNMP V1)
                RFC 3416 (for SNMPv2C)
*******************************************************************************/

/*******************************************************************************
FileName:   SNMP.c
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

#define __SNMP_C

#include "tcpip_private.h"

#if defined(TCPIP_STACK_USE_SNMP_SERVER)


#include "tcpip/snmp.h"
#include "mib.h"


#ifdef TCPIP_STACK_USE_SNMPV3_SERVER
#include "tcpip/snmpv3.h"
#endif

#define TCPIP_THIS_MODULE_ID    TCPIP_MODULE_SNMP_SERVER

#include "sys_fs_config.h"


/****************************************************************************
  Section:
	Global Variables
  ***************************************************************************/
static uint16_t SNMPTxOffset;	//Snmp udp buffer tx offset
static uint16_t SNMPRxOffset;

static SNMP_STATUS SNMPStatus;	//MIB file access status

static reqVarErrStatus snmpReqVarErrStatus;


#ifdef TCPIP_STACK_USE_SNMPV3_SERVER
static uint16_t msgSecrtyParamLenOffset;
static int	SNMPV3InitCount=0;
#endif 


static const void*  SnmpStackMemH = 0;        // memory handle

static SNMP_STACK_DCPT_STUB*  SnmpStackDcptMemStubPtr=0;

static SNMPV3_STACK_DCPT_STUB* Snmpv3StkStubPtr=0;

static SNMPV1V2_REQUEST_WHOLEMSG snmpV1V2cIncomingMsgBufPtr;

static TCPIP_SNMP_DCPT* snmpDcpt= 0;
    

//ASN format datatype for snmp v1 and v2c
static const SNMP_DATA_TYPE_INFO dataTypeTable[] =
{
    { ASN_INT,           1       }, //INT8_VAL          
    { ASN_INT,           2       }, //INT16_VAL         
    { ASN_INT,           4       }, //INT32_VAL         
    { OCTET_STRING,      0xff    }, //BYTE_ARRAY        
    { OCTET_STRING,      0xff    }, //ASCII_ARRAY       
    { SNMP_IP_ADDR,      4       }, //IPADDRESS        
    { SNMP_COUNTER32,    4       }, //COUNTER32         
    { SNMP_TIME_TICKS,   4       }, //TIME_TICKS_VAL    
    { SNMP_GAUGE32,      4       }, //GAUTE32           
    { ASN_OID,           0xff    }  //OID_VAL           
};

static int snmpInitCount = 0;      // SNMP module initialization count

static int32_t snmpFileDescrptr;


/****************************************************************************
  Section:
	Function Prototypes
  ***************************************************************************/

static void SNMPPutDataArrayToUDPTxBuffer(UDP_SOCKET socket,SNMPBUFFERDATA *snmpPutData);
static uint8_t SNMPFindOIDsInRequest(uint16_t pdulen);
static SNMP_ACTION SNMPProcessHeader(PDU_INFO * pduDbPtr, char* community, uint8_t* len);
static bool SNMPProcessGetSetHeader(PDU_INFO * pduDbPtr);
static bool SNMPProcessVariables(PDU_INFO * pduDbPtr,char* community, uint8_t len);


static bool SNMPCheckIfValidOID(uint8_t* oid, uint8_t* len);
static bool SNMPCheckIfValidCommunityString(char* community, uint8_t* len);
static bool SNMPCheckIfValidPDU(SNMP_ACTION* pdu);
static bool SNMPCheckIsASNNull(void);
static bool SNMPGetOIDStringByAddr(OID_INFO* rec, uint8_t* oidString, uint8_t* len);
static void SNMPReadMIBRecord(uint32_t h, OID_INFO* rec);
static void SNMPSetErrorStatus(uint16_t errorStatusOffset, uint16_t errorIndexOffset,SNMP_ERR_STATUS errorStatus,uint8_t errorIndex,SNMPBUFFERDATA *snmpPutTxData);
static bool SNMPCheckIfPvtMibObjRequested(uint8_t* OIDValuePtr);
static void SNMPStkCrtDynMemForStkDcptr(const TCPIP_STACK_MODULE_CTRL* const stackCtrl);
static void SNMPInitializeStackDescriptors(const TCPIP_STACK_MODULE_CTRL* const stackCtrl);
static bool SNMPLoadDefaultUserConfig(void);

#ifdef TCPIP_STACK_USE_SNMPV3_SERVER
extern	void SNMPv3Init(TCPIP_NET_IF* netIf);
#endif /* TCPIP_STACK_USE_SNMPV3_SERVER */







/****************************************************************************
  ===========================================================================
  Section:
	SNMP v1 and v2c Agent Routines
  ===========================================================================
  ***************************************************************************/

/****************************************************************************
  Function:
    bool SNMPInit(void)

  Summary:
    Initialize SNMP module internals.

  Description:
  	This function initializes the Snmp agent. One udp socket is intialized 
  	and opened at port 161. Agent will receive and transmit all the snmp 
  	pdus on this udp socket. 
  	
  Precondition:
	At least one UDP socket must be available. UDPInit() is already called.

  Parameters:
	None

  Returns:
	None
    
  Remarks:
	This function is called only once during lifetime of the application.
	One UDP socket will be used.
 ***************************************************************************/
bool SNMPInit(const TCPIP_STACK_MODULE_CTRL* const stackCtrl,
              const SNMP_MODULE_GONFIG* snmpData)
{
	TCPIP_SNMP_DCPT *pDcpt;
	
    if(stackCtrl->stackAction == TCPIP_STACK_ACTION_IF_UP)
    {   
    	// interface restart
        return true;
    }

    if(snmpInitCount == 0)
    {   		
		SNMPStkCrtDynMemForStkDcptr(stackCtrl);
		
		snmpDcpt = (TCPIP_SNMP_DCPT*)TCPIP_HEAP_Calloc(stackCtrl->memH,  2, sizeof(TCPIP_SNMP_DCPT));
		if(snmpDcpt == 0)
		{	// failed
			return false;
		}

		
		SNMPInitializeStackDescriptors(stackCtrl);
		
		
      SNMPStatus.Val = 0;

      if(SNMPLoadDefaultUserConfig()!= true)
	  	return false;
   }

	
    pDcpt = snmpDcpt + stackCtrl->netIx;
    pDcpt->sm = SNMP_HOME;
    pDcpt->skt = INVALID_UDP_SOCKET;

    snmpInitCount++;
    return true;
}

/****************************************************************************
  Function:
    void SNMPDeInit(void)

  Summary:
    DeInitialize SNMP module internals.

  Description:
  	This function deinitializes the Snmp agent. Closes the UDP socket if all
    interfaces are down.		
  	
  Precondition:
	None.

  Parameters:
	None

  Returns:
	None
    
  Remarks:
	This function may be called many times during lifetime of the application.
 ***************************************************************************/
void SNMPDeInit(const TCPIP_STACK_MODULE_CTRL* const stackCtrl)
{
    if(stackCtrl->stackAction == TCPIP_STACK_ACTION_DEINIT)
    {   // whole stack is going down
        if(snmpInitCount > 0)
        {   // we're up and running
            if(--snmpInitCount == 0)
            {   // all closed
                // release resources
                if (snmpDcpt->skt != INVALID_UDP_SOCKET)
                {
                    UDPClose(snmpDcpt->skt);
                    snmpDcpt->skt = INVALID_UDP_SOCKET;
                }
                if(snmpDcpt != 0)
                {
                   TCPIP_HEAP_Free(stackCtrl->memH, snmpDcpt);
                   snmpDcpt = 0;
                }
           }
        }
    }
}


void SNMPStkCrtDynMemForStkDcptr(const TCPIP_STACK_MODULE_CTRL* const stackCtrl)
{
	SnmpStackMemH=stackCtrl->memH;


	SnmpStackDcptMemStubPtr = (SNMP_STACK_DCPT_STUB *)TCPIP_HEAP_Malloc(SnmpStackMemH, (sizeof(SNMP_STACK_DCPT_STUB)
	
	#ifdef TCPIP_STACK_USE_SNMPV3_SERVER

								+ sizeof(SNMPV3_STACK_DCPT_STUB)
								#endif

								));

        Snmpv3StkStubPtr = (SNMPV3_STACK_DCPT_STUB*) (((unsigned long int)(SnmpStackDcptMemStubPtr)) + sizeof(SNMP_STACK_DCPT_STUB));
    

}



void SNMPGetPktProcessingDynMemStubPtrs( SNMP_PROCESSING_MEM_INFO_PTRS * dynMemInfoPtr)
{

	dynMemInfoPtr->snmpHeapMemHandler=SnmpStackMemH;
	dynMemInfoPtr->snmpStkDynMemStubPtr=SnmpStackDcptMemStubPtr;
	dynMemInfoPtr->snmpDcptPtr=snmpDcpt;
}


void SNMPv3GetPktProcessingDynMemStubPtrs( SNMPV3_PROCESSING_MEM_INFO_PTRS * dynMemInfoPtr)
{

	dynMemInfoPtr->snmpHeapMemHandler=SnmpStackMemH;
	dynMemInfoPtr->snmpv3StkProcessingDynMemStubPtr=Snmpv3StkStubPtr;
}



static void SNMPInitializeStackDescriptors(const TCPIP_STACK_MODULE_CTRL* const stackCtrl)
{

#ifdef TCPIP_STACK_USE_SNMPV3_SERVER
	 SNMPV3_PROCESSING_MEM_INFO_PTRS snmpv3PktProcessingMemPntr; 
	 SNMPV3_STACK_DCPT_STUB * snmpv3EngnDcptMemoryStubPtr=0;
#if defined(SNMP_V1_V2_TRAP_WITH_SNMPV3) && !defined(SNMP_TRAP_DISABLED)
         uint8_t usmTrapIndex=0;
	 static const char * const cTrapUserSecurityName[] = SNMPV3_TRAP_USER_SECURITY_NAME_DB;
	 static const uint8_t const eTrapMsgProcessModelType[] = SNMPV3_TRAP_MSG_PROCESS_MODEL_DB;
	 static const uint8_t const  eTrapSecurityModelType[] = SNMPV3_TRAP_SECURITY_MODEL_TYPE_DB;
	 static const uint8_t const  eTrapSecurityLevelType[] = SNMPV3_TRAP_SECURITY_LEVEL_DB;
#endif
	SNMPv3GetPktProcessingDynMemStubPtrs(&snmpv3PktProcessingMemPntr);
	snmpv3EngnDcptMemoryStubPtr=snmpv3PktProcessingMemPntr.snmpv3StkProcessingDynMemStubPtr;


#if defined(SNMP_V1_V2_TRAP_WITH_SNMPV3)&& !defined(SNMP_TRAP_DISABLED)
	//snmv3 global database for trap table
	for(;usmTrapIndex<SNMPV3_USM_MAX_USER;usmTrapIndex++)
	{
		memcpy(snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[usmTrapIndex].userSecurityName,cTrapUserSecurityName[usmTrapIndex]
								,strlen(cTrapUserSecurityName[usmTrapIndex]));
		snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[usmTrapIndex].messageProcessingModelType = eTrapMsgProcessModelType[usmTrapIndex];
		snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[usmTrapIndex].securityModelType = eTrapSecurityModelType[usmTrapIndex];
		snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[usmTrapIndex].securityLevelType = eTrapSecurityLevelType[usmTrapIndex];
	}
	#endif

	snmpv3EngnDcptMemoryStubPtr->SnmpInMsgAuthParamLen=12;
	snmpv3EngnDcptMemoryStubPtr->SnmpInMsgPrivParmLen=8;
	snmpv3EngnDcptMemoryStubPtr->SnmpOutMsgAuthParmLen=12;
	snmpv3EngnDcptMemoryStubPtr->SnmpOutMsgPrivParmLen=8;

#endif 

#if !defined(SNMP_TRAP_DISABLED)
	SnmpStackDcptMemStubPtr->gSendTrapFlag=false;//global flag to send Trap
	SnmpStackDcptMemStubPtr->gOIDCorrespondingSnmpMibID=MICROCHIP;
	SnmpStackDcptMemStubPtr->gGenericTrapNotification=ENTERPRISE_SPECIFIC;
	SnmpStackDcptMemStubPtr->gSpecificTrapNotification=VENDOR_TRAP_DEFAULT; // Vendor specific trap code
#endif

#if defined(SNMP_STACK_USE_V2_TRAP) && !defined(SNMP_TRAP_DISABLED)
	//if gSetTrapSendFlag == false then the last varbind variable for 
	//multiple varbind variable pdu structure or if there is only varbind variable send.
	// if gSetTrapSendFlag == true, then v2 trap pdu is expecting more varbind variable.
	SnmpStackDcptMemStubPtr->gSetTrapSendFlag = false;
#endif /* SNMP_STACK_USE_V2_TRAP */

}


/****************************************************************************
  Function:
	bool SNMPPutDataToProcessBuff(uint8_t val ,SNMPBUFFERDATA *putbuf)
	
  Summary:
  	Copies uint8_t data to dynamically allocated memory buffer.

  Description:
	The SNMPv1 and v2c stack implementation uses dynamically allocated memory buffer for
	processing of request and response packets. This routine copies the uint8_t data to the 
	allocated buffer and updates the offset length couter. 
		  	 		 		  	
  Precondition:
	The SNMPv1 and v2c stack has sucessfully allocated dynamic memory buffer from the Heap
   	
  Parameters:
  	val: uint8_t value to be written to the buffer
  	putbuf: pointer to the dynamically allocated buffer to which the 'val' to be written 
  	
  Return Values:
	true: if successfully write to the buffer
	false: failure in writing to the buffer
	
  Remarks:
  	This routine is used by the SNMP stack. If required to be used by the application
  	code, valid pointers should be passed to this routine. 
  	
***************************************************************************/
bool SNMPPutDataToProcessBuff(uint8_t val ,SNMPBUFFERDATA *putbuf)
{
	if(putbuf->length < SNMP_MAX_MSG_SIZE)
	{
		putbuf->head[putbuf->length++] = (uint8_t)val;
		return true;
	}
	else
	{
		return false;
	}

}

/****************************************************************************
  Function:
	bool SNMPGetProcessBuffData(SNMPBUFFERDATA getbuf,uint16_t pos)
	
  Summary:
  	Reads uint8_t data from dynamically allocated memory buffer.

  Description:
	The SNMPstack implementation uses dynamically allocated memory buffer for
	processing of request and response packets. This routine reads the uint8_t data from 
	the allocated buffer at the positions (offset) provided.
		  	 		 		  	
  Precondition:
	The SNMPv1 and v2c stack has sucessfully allocated dynamic memory buffer from the Heap
   	
  Parameters:
  	getbuf: Structure from where to read the data byte.
  	pos: position in the buffer from which the data to be read 
  	
  Return Values:
	uint8_t: 1 byte value read
	
  Remarks:
  	The read position offset is required to be provided every time the routine is called.
  	This API do not increment the buffer read offset automatically, everytime it is called. 
  	
***************************************************************************/	
uint8_t SNMPGetProcessBuffData(SNMPBUFFERDATA getbuf,uint16_t pos)
{
	return (uint8_t)(getbuf.head[pos]);
}


/****************************************************************************
  Function:
	bool SNMPTask(TCPIP_NET_IF* pNetIf)

  Summary:
	Polls for every snmp pdu received.

  Description:
	Handle incoming SNMP requests as well as any outgoing SNMP 
	responses and timeout conditions.
	
  Precondition:
	SNMPInit() is already called.

  Parameters:
	None

  Return Values:
	true	-	If SNMP module has finished with a state
	false	-	If a state has not been finished.
	
  Remarks:
	None
 ***************************************************************************/
bool SNMPTask(TCPIP_NET_IF* pNetIf)
{
    char community[SNMP_COMMUNITY_MAX_LEN];
    uint8_t communityLen=0;
  	PDU_INFO pduInfoDB; //received pdu information database
    bool lbReturn=true;
    int        netIx;
	void *SnmpBufptr;
    UDP_SOCKET     s;


	if(!pNetIf)
    {
        return false;
    }
    else
	{
        netIx = _TCPIPStackNetIx(pNetIf);
	}

	s = snmpDcpt[netIx].skt;

    
	switch(snmpDcpt[netIx].sm)
	{
		case SNMP_HOME:
			// Open a SNMP agent socket 
			s = UDPOpenServer(IP_ADDRESS_TYPE_ANY, SNMP_AGENT_PORT, 0);
			if(s == INVALID_UDP_SOCKET)
			{
				return false;
			}
			snmpDcpt[netIx].skt = s;
			UDPSocketSetNet(s, pNetIf);
			
// intialize SNMP v3 with a proper pNetIf parameters 			
#ifdef TCPIP_STACK_USE_SNMPV3_SERVER
			if( SNMPV3InitCount == 0)
	{
				SNMPv3Init(pNetIf);
				SNMPV3InitCount++;
	}
#endif
			
			snmpDcpt[netIx].sm=SNMP_LISTEN;
			break;

		case SNMP_LISTEN:
			// Do nothing if no data is waiting
			if(!UDPIsGetReady(s))
			{
				return false;
			}
    // As we process SNMP variables, we will prepare response on-the-fly
    // creating full duplex transfer.
    // Current MAC layer does not support full duplex transfer, so
    // SNMP needs to manage its own full duplex connection.
    // Prepare for full duplex transfer.
			if(!SNMPProcessReqBuildRespnsDuplexInit(s))
    {
				return false;
    }
			
	communityLen = 0;	// Suppress C30 warning: 'communityLen' may be used uninitialized in this function
			pduInfoDB.pduType = SNMPProcessHeader(&pduInfoDB,community, &communityLen);
			// We received a discovery request, reply when we can
			snmpDcpt[netIx].sm=SNMP_PROCESS;
			//break;
		case SNMP_PROCESS:
			
	if(pduInfoDB.snmpVersion != SNMP_V3)
	{
		if ( pduInfoDB.pduType == SNMP_ACTION_UNKNOWN )
				{
					snmpDcpt[netIx].sm = SNMP_PACKET_DISCARD;
					
					break;
	}
			
				if ( !SNMPProcessGetSetHeader(&pduInfoDB))
				{
					snmpDcpt[netIx].sm = SNMP_PACKET_DISCARD;
					break;
				}
			}
			
    // Open MIB file.
    SNMPStatus.Flags.bIsFileOpen = false;
		
	
	snmpFileDescrptr= SYS_FS_open((const char*)SNMP_BIB_FILE_NAME,0);
		
	
    if(snmpFileDescrptr != RETURN_FAILED)
    {
       SNMPStatus.Flags.bIsFileOpen = true;
    }
			
    if(pduInfoDB.snmpVersion != SNMP_V3) // if(SNMP_V1, SNMP_V2C)
    {
            SnmpBufptr = TCPIP_HEAP_Calloc(SnmpStackMemH,1,(size_t)SNMP_MAX_MSG_SIZE);
            if(SnmpBufptr)
				{
					SnmpStackDcptMemStubPtr->outPduBufData.head = (uint8_t *)SnmpBufptr;
    }
				lbReturn = SNMPProcessVariables(&pduInfoDB,community, communityLen);


				if(snmpV1V2cIncomingMsgBufPtr.wholeMsgHead != 0)
				{
					TCPIP_HEAP_Free(SnmpStackMemH, snmpV1V2cIncomingMsgBufPtr.wholeMsgHead);
					snmpV1V2cIncomingMsgBufPtr.wholeMsgLen.Val=0;
					snmpV1V2cIncomingMsgBufPtr.wholeMsgHead=0;
					snmpV1V2cIncomingMsgBufPtr.snmpMsgHead=0;
					
				}
			}
			
#ifdef TCPIP_STACK_USE_SNMPV3_SERVER
    else
    {
					lbReturn = SNMPv3ProcessV3MsgData(&pduInfoDB);
    }
#endif
	
			if(SNMPStatus.Flags.bIsFileOpen)
    {
			   SYS_FS_close(snmpFileDescrptr);
    }
			
			if(lbReturn == false)
			{
				if(SnmpStackDcptMemStubPtr->outPduBufData.head !=0x00)
				{
					TCPIP_HEAP_Free(SnmpStackMemH, SnmpStackDcptMemStubPtr->outPduBufData.head);
				}
				SnmpStackDcptMemStubPtr->outPduBufData.length = 0;
#ifdef TCPIP_STACK_USE_SNMPV3_SERVER
				SNMPv3FreeDynAllocMem();
#endif 
				snmpDcpt[netIx].sm = SNMP_PACKET_DISCARD;
				break;
			}
			
			if(SnmpStackDcptMemStubPtr->gSendTrapFlag==(uint8_t)false)
			{
				UDPFlush(s);
			}
			snmpDcpt[netIx].sm=SNMP_PACKET_DISCARD;
			//break;
		case SNMP_PACKET_DISCARD:

			if(snmpV1V2cIncomingMsgBufPtr.wholeMsgHead != 0)
			{
				TCPIP_HEAP_Free(SnmpStackMemH, snmpV1V2cIncomingMsgBufPtr.wholeMsgHead);
				snmpV1V2cIncomingMsgBufPtr.wholeMsgLen.Val=0;
				snmpV1V2cIncomingMsgBufPtr.wholeMsgHead=0;
				snmpV1V2cIncomingMsgBufPtr.snmpMsgHead=0;
				
			}
			UDPDiscard(s);
			// clear v1/v2c outbuf dynamic buffer data
			if(SnmpStackDcptMemStubPtr->outPduBufData.head !=0x00)
			{
				TCPIP_HEAP_Free(SnmpStackMemH, SnmpStackDcptMemStubPtr->outPduBufData.head);
				SnmpStackDcptMemStubPtr->outPduBufData.head=0x00;
				SnmpStackDcptMemStubPtr->outPduBufData.length = 0;
			}
			
#ifdef TCPIP_STACK_USE_SNMPV3_SERVER
			SNMPv3FreeDynAllocMem();
#endif 
			snmpDcpt[netIx].sm = SNMP_LISTEN;

			break;
	
	}

    return true;
}


#if !defined(SNMP_TRAP_DISABLED)
static SYS_TICK  snmpTrapTimer=0;

/****************************************************************************
  Function:
	void SNMPNotifyPrepare(IP_MULTI_ADDRESS* remoteHost,
                           char* community,
                           uint8_t communityLen,
                           SNMP_ID agentIDVar,
                           uint8_t notificationCode,
                           uint32_t timestamp)

  Summary:
	Collects trap notification info and send ARP to remote host.

  Description:
	This function prepares SNMP module to send SNMP trap notification
	to remote host. It sends ARP request to remote host to learn remote
	host MAC address.
	
  Precondition:
	SNMPInit() is already called.

  Parameters:
	remoteHost  - pointer to remote Host IP address
	community   - Community string to use to notify
	communityLen- Community string length
	agentIDVar  - System ID to use identify this agent
	notificaitonCode - Notification Code to use
	timestamp   - Notification timestamp in 100th of second.

  Returns:
	None
		
  Remarks:
	This is first of series of functions to complete SNMP notification.
 ***************************************************************************/
void SNMPNotifyPrepare(IP_MULTI_ADDRESS* remoteHost,
                        char* community,
                        uint8_t communityLen,
                        SNMP_ID agentIDVar,
                        uint8_t notificationCode,
                        uint32_t timestamp )
{
	uint8_t intfIdx = 0;
	static IP_MULTI_ADDRESS* remHostIpAddrPtr;
	remHostIpAddrPtr = remoteHost;

	intfIdx = ((TCPIP_NET_IF*)TCPIP_STACK_GetDefaultNet())->netIfIx; 
    strcpy(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.community, community);
    SnmpStackDcptMemStubPtr->SNMPNotifyInfo.communityLen = communityLen;

    SnmpStackDcptMemStubPtr->SNMPNotifyInfo.agentIDVar = agentIDVar;
    SnmpStackDcptMemStubPtr->SNMPNotifyInfo.notificationCode = notificationCode;

    SnmpStackDcptMemStubPtr->SNMPNotifyInfo.timestamp.Val = timestamp;
	SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket = INVALID_UDP_SOCKET;

//    ARPResolve(remHostIpAddrPtr);
}


/****************************************************************************
  Function:
	bool SNMPIsNotifyReady(IP_MULTI_ADDRESS* remoteHost,SNMP_TRAP_IP_ADDRESS_TYPE eTrapMultiAddressType)

  Summary:
	Resolves given remoteHost IP address into MAC address.
	
  Description:
  	This function resolves given remoteHost IP address into MAC address using 
  	ARP module. If remoteHost is not aviailable, this function would never 
  	return true. Application must implement timeout logic to handle 
  	"remoteHost not avialable" situation.
	
  Precondition:
	SNMPNotifyPrepare() is already called.

  Parameters:
	remoteHost  - Pointer to remote Host IP address
      
  Return Values:
	true	-	If remoteHost IP address is resolved and 
				SNMPNotify may be called.
    false	-	If remoteHost IP address is not resolved.
    
  Remarks:
	This would fail if there were not UDP socket to open.
 ***************************************************************************/
bool SNMPIsNotifyReady(IP_MULTI_ADDRESS* remoteHost,SNMP_TRAP_IP_ADDRESS_TYPE eTrapMultiAddressType)
{
//	uint8_t intfIdx = 0;
    IP_MULTI_ADDRESS remoteAddress;
	if(eTrapMultiAddressType == IPV4_SNMP_TRAP)
	{
		remoteAddress.v4Add.Val = remoteHost->v4Add.Val;
	}
	else
	{
		memcpy(&remoteAddress.v6Add,&remoteHost->v6Add,16);
	}
	
	if(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket == INVALID_UDP_SOCKET)
	{
		if(eTrapMultiAddressType == IPV4_SNMP_TRAP)
		{
			SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket = UDPOpenClient(IP_ADDRESS_TYPE_IPV4,SNMP_NMS_PORT,&remoteAddress);
		}
		else
		{
			SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket = UDPOpenClient(IP_ADDRESS_TYPE_IPV6,SNMP_NMS_PORT,&remoteAddress);
		}

        if(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket != INVALID_UDP_SOCKET)
		{
			UDPSocketSetNet(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket,SnmpStackDcptMemStubPtr->SNMPNotifyInfo.snmpTrapInf);
			snmpTrapTimer = SYS_TICK_Get();
		}
		else
		{
			return false;
		}
	}


	if(UDPIsOpened(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket)== true)
	{		
		snmpTrapTimer = SYS_TICK_Get();
		return true;
	}
	else  
    {
		return false;
	}

	return true;

}

SYS_TICK SNMPGetTrapTime(void)
{
	return snmpTrapTimer;
}

/****************************************************************************
  Function:
	uint8_t *SNMPResolveGenTrapCodeToTrapOID(uint8_t generic_trap_code,uint8_t *len)
  Summary:
	Resolves generic trap code to generic trap OID.
	
  Description:
  	This function resolves given generic trap code  to generic trap OID.
	
  Precondition:
	SNMPNotifyPrepare() is already called.

  Parameters:
	generic_trap_code  - GENERIC_TRAP_NOTIFICATION_TYPE
	len 	- generic trap OID length
      
  Return Values:
	uint8_t *- TRAP OID
    
  Remarks:
	This would fail if generic_trap_code is not coming under 
	GENERIC_TRAP_NOTIFICATION_TYPE 
 ***************************************************************************/

uint8_t *SNMPResolveGenTrapCodeToTrapOID(uint8_t generic_trap_code,uint8_t *len)
{
    static  uint8_t gen_trap_oid[] = {0x2b,6,1,6,3,1,1,5,1};
/*	
	static  uint8_t cold_trap_oid[] = {0x2b,6,1,6,3,1,1,5,1};
    static  uint8_t warm_start_oid = {0x2b,6,1,6,3,1,1,5,2};
    static  uint8_t auth_fail_oid  = {0x2b,6,1,6,3,1,1,5,5};
    static  uint8_t linkdown_oid   = {0x2b,6,1,6,3,1,1,5,3};
    static  uint8_t linkup_oid     = {0x2b,6,1,6,3,1,1,5,4};
*/
	static uint8_t snmptrap_oids[]  = {0x2b,6,1,6,3,1,1,4,1 }; 

	*len = sizeof(gen_trap_oid);
    switch (generic_trap_code) 
	{
      case COLD_START:
	  	  gen_trap_oid[*len-1] = 1;
        break;

      case WARM_START:
        gen_trap_oid[*len-1] = 2;
        break;
      case LINK_UP:
           gen_trap_oid[*len-1] = 4;
        break;
      case LINK_DOWN:
          gen_trap_oid[*len-1] = 3;
        break;
      case AUTH_FAILURE:
          gen_trap_oid[*len-1] = 5;
        break;
	case ENTERPRISE_SPECIFIC:
		*len = sizeof(snmptrap_oids);
		return snmptrap_oids;
      default:
          return NULL;

    } /* switch (generic_trap_code) */

    return gen_trap_oid;

} /* end getSnmpV2TrapOid() */



/****************************************************************************
  Function:
	bool SNMPNotify(SNMP_ID var,SNMP_VAL val,SNMP_INDEX index)

  Summary:
  	Creates and Sends TRAP pdu.
	
  Description:
	This function creates SNMP V2 Trap PDU and sends it to previously specified
	remoteHost.
	
	snmpv1 trap pdu:
       | PDU-type | enterprise | agent-addr | generic-trap | specific-trap |
       | time-stamp | varbind-list |

       The v1 enterprise is mapped directly to SNMPv2TrapOID.0
	SNMP v2 trap pdu:
       version (0 or 1) | community | SNMP-PDU |pdu-type | request-id | error-status 
       |err-index |varbinds

       The first two variables (in varbind-list) of snmpv2 are: sysUpTime.0 and
        SNMPv2TrapOID.0

        Generic Trap OID is used as the varbind for authentication failure.

  Precondition:
 	SNMPIsNotifyReady() is already called and returned true.

  Parameters:
	var     - SNMP var ID that is to be used in notification
	val     - Value of var. Only value of uint8_t, uint16_t or uint32_t can be sent.
	index   - Index of var. If this var is a single,index would be 0, or else 
			  if this var Is a sequence, index could be any value 
			  from 0 to 127
	  
  Return Values:
	true	-	if SNMP notification was successful sent.
	   			This does not guarantee that remoteHost recieved it.
	false	-	Notification sent failed.
  			    This would fail under following contions:
  			    1) Given SNMP_BIB_FILE does not exist in file system
  			    2) Given var does not exist.
  			    3) Previously given agentID does not exist
  			    4) Data type of given var is unknown - 
  			       possible if file system itself was corrupted.

  Remarks:
	This would fail if there were not UDP socket to open.
 ***************************************************************************/
#if defined(SNMP_STACK_USE_V2_TRAP)

bool SNMPNotify(SNMP_ID var, SNMP_VAL val, SNMP_INDEX index)
{
	char* pCommunity;
	uint8_t len;
	uint8_t OIDValue[OID_MAX_LEN];
	uint8_t OIDLen;
	static uint32_t varbindlen = 0;
	uint8_t agentIDLen;
	uint8_t* pOIDValue;
	static uint16_t packetStructLenOffset = 0;
	static uint16_t pduStructLenOffset = 0;
	static uint16_t varBindStructLenOffset = 0;
	static uint16_t varPairStructLenOffset = 0;
	static uint16_t prevOffset = 0;
	uint16_t tempOffset = 0;
	OID_INFO rec;
	SNMP_DATA_TYPE_INFO dataTypeInfo;
	uint8_t	snmptrap_oids[]  = {0x2b,6,1,6,3,1,1,4,1 }; /* len=10 */
	uint8_t	sysUpTime_oids[] = {0x2b,6,1,2,1,1,3}; /* len = 8 */
	TCPIP_UINT16_VAL trapVarBindLen={0};
	int i=0;
	SNMPBUFFERDATA *snmpTrapPutData=NULL;
	uint8_t intfIdx;

		
	snmpFileDescrptr = SYS_FS_open((const char*)SNMP_BIB_FILE_NAME,0);

	
	intfIdx = ((TCPIP_NET_IF*)TCPIP_STACK_GetDefaultNet())->netIfIx; 
	
	
	if ( snmpFileDescrptr == RETURN_FAILED ) //INVALID_HANDLE
	{
		UDPClose(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket);
		SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket = INVALID_UDP_SOCKET;
		return false;
	}
	
	if((packetStructLenOffset == 0)&&(pduStructLenOffset==0))
	{
		if(!SNMPProcessReqBuildRespnsDuplexInit(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket))
        {        	
			UDPClose(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket);
			SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket = INVALID_UDP_SOCKET;
            return false;
        }
		SnmpStackDcptMemStubPtr->trapPduOutBufData.head = TCPIP_HEAP_Calloc(SnmpStackMemH,1,(size_t)SNMP_MAX_MSG_SIZE);
		if(!SnmpStackDcptMemStubPtr->trapPduOutBufData.head)
		{
			UDPClose(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket);
			SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket = INVALID_UDP_SOCKET;
			return false;
		}
		SnmpStackDcptMemStubPtr->trapPduOutBufData.length = 0;
		snmpTrapPutData = &SnmpStackDcptMemStubPtr->trapPduOutBufData;
		prevOffset =  snmpTrapPutData->length;
		
		len = SnmpStackDcptMemStubPtr->SNMPNotifyInfo.communityLen;
		pCommunity = SnmpStackDcptMemStubPtr->SNMPNotifyInfo.community;

		SNMPPutDataToProcessBuff(STRUCTURE,snmpTrapPutData);			// First item is packet structure
		SNMPPutDataToProcessBuff(0x82,snmpTrapPutData);
		packetStructLenOffset = snmpTrapPutData->length;
		SNMPPutDataToProcessBuff(0,snmpTrapPutData);
		SNMPPutDataToProcessBuff(0,snmpTrapPutData);

		// Put SNMP version info.
		SNMPPutDataToProcessBuff(ASN_INT,snmpTrapPutData);				// Int type.
		SNMPPutDataToProcessBuff(1,snmpTrapPutData);					// One byte long value.
		SNMPPutDataToProcessBuff(SNMP_V2C,snmpTrapPutData); 		  // v2

		//len = strlen(community);	// Save community length for later use.
		SNMPPutDataToProcessBuff(OCTET_STRING,snmpTrapPutData); 		// Octet string type.
		SNMPPutDataToProcessBuff(len,snmpTrapPutData);					// community string length
		while( len-- )					// Copy entire string.
			SNMPPutDataToProcessBuff(*(pCommunity++),snmpTrapPutData);

		//TRAP Version type.  
		SNMPPutDataToProcessBuff(SNMP_V2_TRAP,snmpTrapPutData);
		SNMPPutDataToProcessBuff(0x82,snmpTrapPutData);
		pduStructLenOffset = snmpTrapPutData->length;
		SNMPPutDataToProcessBuff(0,snmpTrapPutData);
		SNMPPutDataToProcessBuff(0,snmpTrapPutData);

		//put Request ID for the trapv2 as 1 
		SNMPPutDataToProcessBuff(ASN_INT,snmpTrapPutData);	// Int type.
		SNMPPutDataToProcessBuff(4,snmpTrapPutData);		// To simplify logic, always use 4 byte long requestID
		SNMPPutDataToProcessBuff(0,snmpTrapPutData); 
		SNMPPutDataToProcessBuff(0,snmpTrapPutData); 
		SNMPPutDataToProcessBuff(0,snmpTrapPutData); 
		SNMPPutDataToProcessBuff(1,snmpTrapPutData); 

		// Put error status.
		SNMPPutDataToProcessBuff(ASN_INT,snmpTrapPutData);				// Int type
		SNMPPutDataToProcessBuff(1,snmpTrapPutData);					// One byte long.
		SNMPPutDataToProcessBuff(0,snmpTrapPutData);					// Placeholder.

		// Similarly put error index.
		SNMPPutDataToProcessBuff(ASN_INT,snmpTrapPutData);				// Int type
		SNMPPutDataToProcessBuff(1,snmpTrapPutData);					// One byte long
		SNMPPutDataToProcessBuff(0,snmpTrapPutData);					// Placeholder.

		// Variable binding structure header
		SNMPPutDataToProcessBuff(0x30,snmpTrapPutData);
		SNMPPutDataToProcessBuff(0x82,snmpTrapPutData);
		varBindStructLenOffset = snmpTrapPutData->length; //SNMPTxOffset;
		SNMPPutDataToProcessBuff(0,snmpTrapPutData);
		SNMPPutDataToProcessBuff(0,snmpTrapPutData);

		// Create variable name-pair structure
		SNMPPutDataToProcessBuff(0x30,snmpTrapPutData);
		varPairStructLenOffset = snmpTrapPutData->length; //SNMPTxOffset;
		SNMPPutDataToProcessBuff(0,snmpTrapPutData);

		// Set 1st varbind object i,e sysUpTime.0 time stamp for the snmpv2 trap
		// Get complete notification variable OID string.

		SNMPPutDataToProcessBuff(ASN_OID,snmpTrapPutData);
		OIDLen = (uint8_t)sizeof(sysUpTime_oids);
		SNMPPutDataToProcessBuff((uint8_t)(OIDLen)+1,snmpTrapPutData);
		pOIDValue = sysUpTime_oids;
		while( OIDLen-- )
			SNMPPutDataToProcessBuff(*pOIDValue++,snmpTrapPutData);

		//1st varbind	 and this is a scalar object so index = 0
		SNMPPutDataToProcessBuff(0,snmpTrapPutData);

		// Time stamp
		SNMPPutDataToProcessBuff(SNMP_TIME_TICKS,snmpTrapPutData);
		SNMPPutDataToProcessBuff(4,snmpTrapPutData);
		SNMPPutDataToProcessBuff(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.timestamp.v[3],snmpTrapPutData);
		SNMPPutDataToProcessBuff(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.timestamp.v[2],snmpTrapPutData);
		SNMPPutDataToProcessBuff(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.timestamp.v[1],snmpTrapPutData);
		SNMPPutDataToProcessBuff(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.timestamp.v[0],snmpTrapPutData);

		tempOffset = snmpTrapPutData->length; 
		//set the snmp time varbind trap offset 
		snmpTrapPutData->length = varPairStructLenOffset;

		// SNMP time stamp varbind length
		OIDLen = 2							// 1st varbind header 
		   + (uint8_t)sizeof(sysUpTime_oids)
		   + 1						   // index byte
		   + 6 ;						// time stamp
		   
		SNMPPutDataToProcessBuff(OIDLen,snmpTrapPutData);
		//set the previous TX offset
		snmpTrapPutData->length = tempOffset;
		varbindlen += OIDLen // varbind length
					+ 2;  // varbind type(30) and length of individual varbind pdu
					
		// Set 2nd varbind object i,e snmpTrapOID.0 for the snmpv2 trap
		// Get complete notification variable OID string.

		// Create variable name-pair structure
		SNMPPutDataToProcessBuff(0x30,snmpTrapPutData);
		varPairStructLenOffset = snmpTrapPutData->length; //SNMPTxOffset;
		SNMPPutDataToProcessBuff(0,snmpTrapPutData);

		// Copy OID string into PDU.
		SNMPPutDataToProcessBuff(ASN_OID,snmpTrapPutData);
		OIDLen = (uint8_t)sizeof(snmptrap_oids);
		SNMPPutDataToProcessBuff((uint8_t)(OIDLen)+1,snmpTrapPutData);

		pOIDValue = snmptrap_oids;
		while( OIDLen-- )
		SNMPPutDataToProcessBuff(*pOIDValue++,snmpTrapPutData);

		//2nd varbind  and this is a scalar object so index = 0
		SNMPPutDataToProcessBuff(0,snmpTrapPutData);
		if ( !SNMPFindOIDStringByID(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.trapIDVar, &rec, OIDValue, &OIDLen) )
		{
			SYS_FS_close(snmpFileDescrptr);
			UDPClose(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket);
			TCPIP_HEAP_Free(SnmpStackMemH,SnmpStackDcptMemStubPtr->trapPduOutBufData.head);
			return false;
		}
		SNMPPutDataToProcessBuff(ASN_OID,snmpTrapPutData);
		agentIDLen = OIDLen;
		len  = OIDLen;
		SNMPPutDataToProcessBuff(agentIDLen,snmpTrapPutData);
		for(i=0;i<len;i++)
		{
			SNMPPutDataToProcessBuff(OIDValue[i],snmpTrapPutData);
		}
		tempOffset = snmpTrapPutData->length;
		//set the snmp varbind trap offset
		snmpTrapPutData->length = varPairStructLenOffset;
		// Snmp trap varbind length 
		OIDLen = 2					 // Agent ID header bytes
			+ (uint8_t)sizeof(snmptrap_oids)
			+ 1 					   // index byte
			+ 2 					 // header
			+ agentIDLen;				 // Agent ID bytes				  
		SNMPPutDataToProcessBuff(OIDLen,snmpTrapPutData);

		//set the previous TX offset
		snmpTrapPutData->length = tempOffset;
		varbindlen += OIDLen // varbind length
					+ 2;	 // varbind type(30) and length of individual varbind pdu
	
	}
	else
	{ // collect the last varbind offset value.
		snmpTrapPutData = &SnmpStackDcptMemStubPtr->trapPduOutBufData;
		snmpTrapPutData->length = varPairStructLenOffset;
	}
	
	// Create variable name-pair structure
	SNMPPutDataToProcessBuff(0x30,snmpTrapPutData);
	SNMPPutDataToProcessBuff(0x82,snmpTrapPutData);
	varPairStructLenOffset = snmpTrapPutData->length;
	SNMPPutDataToProcessBuff(0,snmpTrapPutData);
	SNMPPutDataToProcessBuff(0,snmpTrapPutData);
	/* to send generic trap trap */
	if(SnmpStackDcptMemStubPtr->gGenericTrapNotification != ENTERPRISE_SPECIFIC)
	{
		pOIDValue = SNMPResolveGenTrapCodeToTrapOID(SnmpStackDcptMemStubPtr->gGenericTrapNotification,&OIDLen);			
		if(pOIDValue == NULL)
		{
			SYS_FS_close(snmpFileDescrptr);
			UDPClose(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket);			
			TCPIP_HEAP_Free(SnmpStackMemH,SnmpStackDcptMemStubPtr->trapPduOutBufData.head);
			return false;
		}
		// Copy OID string into PDU.
		SNMPPutDataToProcessBuff(ASN_OID,snmpTrapPutData);
		SNMPPutDataToProcessBuff((uint8_t)(OIDLen)+1,snmpTrapPutData);
		while( OIDLen-- )
		SNMPPutDataToProcessBuff(*pOIDValue++,snmpTrapPutData);

		//2nd varbind  and this is a scalar object so index = 0
		SNMPPutDataToProcessBuff(0,snmpTrapPutData);
		// for microchip , SnmpStackDcptMemStubPtr->SNMPNotifyInfo.agentIDVar == MICROCHIP
		if ( !SNMPFindOIDStringByID(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.agentIDVar, &rec, OIDValue, &OIDLen) )
		{
			SYS_FS_close(snmpFileDescrptr);
			UDPClose(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket);
			TCPIP_HEAP_Free(SnmpStackMemH,SnmpStackDcptMemStubPtr->trapPduOutBufData.head);
			return false;
		}
		if ( !rec.nodeInfo.Flags.bIsAgentID )
		{
			SYS_FS_close(snmpFileDescrptr);
			UDPClose(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket);
			TCPIP_HEAP_Free(SnmpStackMemH,SnmpStackDcptMemStubPtr->trapPduOutBufData.head);
			return false;
		}


		SYS_FS_lseek(snmpFileDescrptr,rec.hData, SEEK_SET);

		SNMPPutDataToProcessBuff(ASN_OID,snmpTrapPutData);


		SYS_FS_read(snmpFileDescrptr,(uint8_t*)&len,1);
		

		agentIDLen = len;
		SNMPPutDataToProcessBuff(agentIDLen,snmpTrapPutData);
		while( len-- )
		{
			uint8_t c;
			SYS_FS_read(snmpFileDescrptr,(uint8_t*)&c,1);
			
			SNMPPutDataToProcessBuff(c,snmpTrapPutData);
		}
		tempOffset = snmpTrapPutData->length;
		//set the snmp varbind trap offset
		snmpTrapPutData->length = varPairStructLenOffset;
		// Snmp trap varbind length 
		trapVarBindLen.Val = 2					 // Agent ID header bytes
			+ (uint8_t)sizeof(snmptrap_oids)
			+ 1 					   // index byte
			+ 2 					 // header
			+ agentIDLen;				 // Agent ID bytes				  
		SNMPPutDataToProcessBuff(trapVarBindLen.v[1],snmpTrapPutData);
		SNMPPutDataToProcessBuff(trapVarBindLen.v[0],snmpTrapPutData);
		len = trapVarBindLen.Val;
	}
	else
	{
		// Get complete notification variable OID string.
		if ( !SNMPFindOIDStringByID(var, &rec, OIDValue, &OIDLen) )
		{
			SYS_FS_close(snmpFileDescrptr);
			UDPClose(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket);
			TCPIP_HEAP_Free(SnmpStackMemH,SnmpStackDcptMemStubPtr->trapPduOutBufData.head);
			return false;
		}
		
		pOIDValue = OIDValue;
	
		// Copy OID string into packet.
		SNMPPutDataToProcessBuff(ASN_OID,snmpTrapPutData);
		SNMPPutDataToProcessBuff((uint8_t)(OIDLen+1),snmpTrapPutData);
		len = OIDLen;
		while( len-- )
		{
			SNMPPutDataToProcessBuff(*pOIDValue++,snmpTrapPutData);
		}
		SNMPPutDataToProcessBuff(index,snmpTrapPutData);

		// Encode and Copy actual data bytes
		if ( !SNMPGetDataTypeInfo(rec.dataType, &dataTypeInfo) )
		{
			SYS_FS_close(snmpFileDescrptr);
			UDPClose(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket);
			TCPIP_HEAP_Free(SnmpStackMemH,SnmpStackDcptMemStubPtr->trapPduOutBufData.head);
			return false;
		}
		SNMPPutDataToProcessBuff(dataTypeInfo.asnType,snmpTrapPutData);
		 //Modified to Send trap even for  dataTypeInfo.asnType= ASCII_STRING, 
		//where dataTypeInfo.asnLen=0xff
		if ( dataTypeInfo.asnLen == 0xff )
		{
			uint8_t *asciiStr= (uint8_t *)val.dword;
			int k=0;
			dataTypeInfo.asnLen=strlen((char *)asciiStr);
			len = dataTypeInfo.asnLen;			
			SNMPPutDataToProcessBuff(len,snmpTrapPutData);
			for(k=0;k<len;k++)
			{
				SNMPPutDataToProcessBuff(asciiStr[k],snmpTrapPutData);
			}
		}
		else
		{
			len = dataTypeInfo.asnLen;	
			SNMPPutDataToProcessBuff(len,snmpTrapPutData);
			while( len-- )
			{
				SNMPPutDataToProcessBuff(val.v[len],snmpTrapPutData);
			}
		}
	  
		trapVarBindLen.Val = dataTypeInfo.asnLen	// data bytes count
			 + 1					// Length byte
			 + 1					// Data type byte
			 + OIDLen				// OID bytes
			 + 2					// OID header bytes
			 + 1;					// index byte
		tempOffset = snmpTrapPutData->length;
		snmpTrapPutData->length = varPairStructLenOffset;
		SNMPPutDataToProcessBuff(trapVarBindLen.v[1],snmpTrapPutData);
		SNMPPutDataToProcessBuff(trapVarBindLen.v[0],snmpTrapPutData);						 
	} 
	//set the previous TX offset
	snmpTrapPutData->length = tempOffset;
	varPairStructLenOffset = tempOffset;
	
	varbindlen += trapVarBindLen.Val // length of varbind
				+4; // varbind type(30) and 0x82 , lenght1 and length2 of individual varbind pdu
	if(SnmpStackDcptMemStubPtr->gSetTrapSendFlag == true)
	{
		SYS_FS_close(snmpFileDescrptr);
		return true;
	}
	trapVarBindLen.Val = varbindlen;
	snmpTrapPutData->length = varBindStructLenOffset;
	SNMPPutDataToProcessBuff(trapVarBindLen.v[1],snmpTrapPutData);
	SNMPPutDataToProcessBuff(trapVarBindLen.v[0],snmpTrapPutData);
	trapVarBindLen.Val = varbindlen
	+ 4 				   //  Variable Binding structure header(0x30,0x82,length1,length2)
	+ 12;					// req , error and error status for SNMPv2

	snmpTrapPutData->length = pduStructLenOffset;
	SNMPPutDataToProcessBuff(trapVarBindLen.v[1],snmpTrapPutData);
	SNMPPutDataToProcessBuff(trapVarBindLen.v[0],snmpTrapPutData);


	trapVarBindLen.Val = trapVarBindLen.Val 						  // PDU struct length
	+ 4 							// PDU trap header
	+ SnmpStackDcptMemStubPtr->SNMPNotifyInfo.communityLen			 // Community string bytes
	+ 2 							// Community header bytes
	+ 3;							// SNMP version bytes


	snmpTrapPutData->length = packetStructLenOffset;
	SNMPPutDataToProcessBuff(trapVarBindLen.v[1],snmpTrapPutData);
	SNMPPutDataToProcessBuff(trapVarBindLen.v[0],snmpTrapPutData);

	snmpTrapPutData->length = tempOffset;

// after setting all the offset values, initialize all static variables to 0.
	packetStructLenOffset = 0;
	pduStructLenOffset = 0;
	varBindStructLenOffset = 0;
	varPairStructLenOffset = 0;
	prevOffset = 0;
	varbindlen = 0;

	SNMPPutDataArrayToUDPTxBuffer(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket,snmpTrapPutData);
	
	SYS_FS_close(snmpFileDescrptr);
	UDPFlush(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket);
	UDPClose(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket);
	TCPIP_HEAP_Free(SnmpStackMemH,SnmpStackDcptMemStubPtr->trapPduOutBufData.head);
	SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket = INVALID_UDP_SOCKET;
	snmpTrapPutData = NULL;
	return true;
}


#else /* SNMP_STACK_USE_V2_TRAP */

/****************************************************************************
  Function:
	bool SNMPNotify(SNMP_ID var,SNMP_VAL val,SNMP_INDEX index)

  Summary:
  	Creates and Sends TRAP pdu.
	
  Description:
	This function creates SNMP trap PDU and sends it to previously specified
	remoteHost.
	snmpv1 trap pdu:
       | PDU-type | enterprise | agent-addr | generic-trap | specific-trap |
       | time-stamp | varbind-list |

       The v1 enterprise is mapped directly to SNMPv2TrapOID.0
       
  Precondition:
	SNMPIsNotifyReady() is already called and returned true.

  Parameters:
	var     - SNMP var ID that is to be used in notification
	val     - Value of var. Only value of uint8_t, uint16_t or uint32_t can be sent.
	index   - Index of var. If this var is a single,index would be 0, or else 
			  if this var Is a sequence, index could be any value 
			  from 0 to 127
	  
  Return Values:
	true	-	if SNMP notification was successful sent.
	   			This does not guarantee that remoteHost recieved it.
	false	-	Notification sent failed.
  			    This would fail under following contions:
  			    1) Given SNMP_BIB_FILE does not exist in file system
  			    2) Given var does not exist.
  			    3) Previously given agentID does not exist
  			    4) Data type of given var is unknown - 
  			       possible if file system itself was corrupted.

  Remarks:
	This would fail if there were not UDP socket to open.
 ***************************************************************************/
bool SNMPNotify(SNMP_ID var, SNMP_VAL val, SNMP_INDEX index)
{
	char* pCommunity;
    uint8_t len;
    uint8_t OIDValue[OID_MAX_LEN];
    uint8_t OIDLen;
    uint8_t agentIDLen;
    uint8_t* pOIDValue;
    uint16_t packetStructLenOffset;
    uint16_t pduStructLenOffset;
    uint16_t varBindStructLenOffset;
    uint16_t varPairStructLenOffset;
    uint16_t prevOffset;
    OID_INFO rec;
	SNMP_DATA_TYPE_INFO dataTypeInfo;
    
	SNMPBUFFERDATA *snmpTrapPutData=NULL;

	snmpFileDescrptr = SYS_FS_open((const char*)SNMP_BIB_FILE_NAME,0);

    if ( snmpFileDescrptr == RETURN_FAILED )//INVALID_HANDLE
    {
        UDPClose(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket);
        return false;
    }

    if(!SNMPProcessReqBuildRespnsDuplexInit(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket))
    {
        return false;
    }

    len = SnmpStackDcptMemStubPtr->SNMPNotifyInfo.communityLen;
    pCommunity = SnmpStackDcptMemStubPtr->SNMPNotifyInfo.community;
	
	SnmpStackDcptMemStubPtr->trapPduOutBufData.head = TCPIP_HEAP_Calloc(SnmpStackMemH,1,(size_t)SNMP_MAX_MSG_SIZE);
	if(!SnmpStackDcptMemStubPtr->trapPduOutBufData.head)
	{
        UDPClose(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket);
		return false;
	}
	SnmpStackDcptMemStubPtr->trapPduOutBufData.length = 0;
	snmpTrapPutData = &SnmpStackDcptMemStubPtr->trapPduOutBufData;
	
    SNMPPutDataToProcessBuff(STRUCTURE,snmpTrapPutData);            // First item is packet structure
    packetStructLenOffset = snmpTrapPutData->length;
    SNMPPutDataToProcessBuff(0,snmpTrapPutData);

    // Put SNMP version info.
    SNMPPutDataToProcessBuff(ASN_INT,snmpTrapPutData);              // Int type.
    SNMPPutDataToProcessBuff(1,snmpTrapPutData);                    // One byte long value.

	//Application has to decide which snmp version has to be 
	//updated to the notification pdu.
    SNMPPutDataToProcessBuff(SNMP_V1,snmpTrapPutData);              // v1.
    

    //len = strlen(community);  // Save community length for later use.
    SNMPPutDataToProcessBuff(OCTET_STRING,snmpTrapPutData);         // Octet string type.
    SNMPPutDataToProcessBuff(len,snmpTrapPutData);                  // community string length
    while( len-- )                  // Copy entire string.
        SNMPPutDataToProcessBuff(*(pCommunity++),snmpTrapPutData);

    // Put PDU type.  SNMP agent's response is always GET RESPONSE
    SNMPPutDataToProcessBuff(TRAP,snmpTrapPutData);
    pduStructLenOffset = snmpTrapPutData->length;
    SNMPPutDataToProcessBuff(0,snmpTrapPutData);

    // Get complete OID string from file snmp.bib.
    if ( !SNMPFindOIDStringByID(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.agentIDVar,
                           &rec, OIDValue, &agentIDLen) )
    {
        SYS_FS_close(snmpFileDescrptr);
        UDPClose(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket);
		TCPIP_HEAP_Free(SnmpStackMemH,SnmpStackDcptMemStubPtr->trapPduOutBufData.head);
        return false;
    }

    if ( !rec.nodeInfo.Flags.bIsAgentID )
    {
        SYS_FS_close(snmpFileDescrptr);
        UDPClose(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket);
		TCPIP_HEAP_Free(SnmpStackMemH,SnmpStackDcptMemStubPtr->trapPduOutBufData.head);
        return false;
    }

	SYS_FS_lseek(snmpFileDescrptr, rec.hData, SEEK_SET);

    SNMPPutDataToProcessBuff(ASN_OID,snmpTrapPutData);
	SYS_FS_read(snmpFileDescrptr,(uint_8*)&len,1);
    agentIDLen = len;
    SNMPPutDataToProcessBuff(len,snmpTrapPutData);
    while( len-- )
    {
	    uint8_t c;
		SYS_FS_read(snmpFileDescrptr,&c,1);
        SNMPPutDataToProcessBuff(c,snmpTrapPutData);
    }

    
   // pNetIf = UDPSocketGetNet(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket);
    // This agent's IP address.
    SNMPPutDataToProcessBuff(SNMP_IP_ADDR,snmpTrapPutData);
    SNMPPutDataToProcessBuff(4,snmpTrapPutData);
    SNMPPutDataToProcessBuff(((TCPIP_NET_IF*)(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.snmpTrapInf))->netIPAddr.v[0],snmpTrapPutData);
    SNMPPutDataToProcessBuff(((TCPIP_NET_IF*)(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.snmpTrapInf))->netIPAddr.v[1],snmpTrapPutData);
    SNMPPutDataToProcessBuff(((TCPIP_NET_IF*)(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.snmpTrapInf))->netIPAddr.v[2],snmpTrapPutData);
    SNMPPutDataToProcessBuff(((TCPIP_NET_IF*)(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.snmpTrapInf))->netIPAddr.v[3],snmpTrapPutData);

	// Geberic/Enterprise Trap code
	 SNMPPutDataToProcessBuff(ASN_INT,snmpTrapPutData);
	 SNMPPutDataToProcessBuff(1,snmpTrapPutData);
	 SNMPPutDataToProcessBuff(SnmpStackDcptMemStubPtr->gGenericTrapNotification,snmpTrapPutData); 

	// Specific Trap code
    SNMPPutDataToProcessBuff(ASN_INT,snmpTrapPutData);
    SNMPPutDataToProcessBuff(1,snmpTrapPutData);
    SNMPPutDataToProcessBuff(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.notificationCode,snmpTrapPutData);

    // Time stamp
    SNMPPutDataToProcessBuff(SNMP_TIME_TICKS,snmpTrapPutData);
    SNMPPutDataToProcessBuff(4,snmpTrapPutData);
    SNMPPutDataToProcessBuff(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.timestamp.v[3],snmpTrapPutData);
    SNMPPutDataToProcessBuff(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.timestamp.v[2],snmpTrapPutData);
    SNMPPutDataToProcessBuff(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.timestamp.v[1],snmpTrapPutData);
    SNMPPutDataToProcessBuff(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.timestamp.v[0],snmpTrapPutData);

    // Variable binding structure header
    SNMPPutDataToProcessBuff(0x30,snmpTrapPutData);
    varBindStructLenOffset = snmpTrapPutData->length;
    SNMPPutDataToProcessBuff(0,snmpTrapPutData);

    // Create variable name-pair structure
    SNMPPutDataToProcessBuff(0x30,snmpTrapPutData);
    varPairStructLenOffset = snmpTrapPutData->length;
	 SNMPPutDataToProcessBuff(0,snmpTrapPutData);
	 
    // Get complete notification variable OID string.
    if ( !SNMPFindOIDStringByID(var, &rec, OIDValue, &OIDLen) )
    {
        SYS_FS_close(snmpFileDescrptr);
        UDPClose(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket);
		TCPIP_HEAP_Free(SnmpStackMemH,SnmpStackDcptMemStubPtr->trapPduOutBufData.head);
        return false;
    }

    // Copy OID string into packet.
    SNMPPutDataToProcessBuff(ASN_OID,snmpTrapPutData);
    SNMPPutDataToProcessBuff((uint8_t)(OIDLen+1),snmpTrapPutData);
    len = OIDLen;
    pOIDValue = OIDValue;
    while( len-- )
        SNMPPutDataToProcessBuff(*pOIDValue++,snmpTrapPutData);
    SNMPPutDataToProcessBuff(index,snmpTrapPutData);

    // Encode and Copy actual data bytes
    if ( !SNMPGetDataTypeInfo(rec.dataType, &dataTypeInfo) )
    {
        SYS_FS_close(snmpFileDescrptr);
        UDPClose(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket);
		TCPIP_HEAP_Free(SnmpStackMemH,SnmpStackDcptMemStubPtr->trapPduOutBufData.head);
        return false;
    }

    SNMPPutDataToProcessBuff(dataTypeInfo.asnType,snmpTrapPutData);


	//Modified to Send trap even for  dataTypeInfo.asnType= ASCII_STRING, 
	//where dataTypeInfo.asnLen=0xff
	if ( dataTypeInfo.asnLen == 0xff )
	{
		dataTypeInfo.asnLen=0x4;
		val.dword=0;
	}

    len = dataTypeInfo.asnLen;
    SNMPPutDataToProcessBuff(len,snmpTrapPutData);
    while( len-- )
        SNMPPutDataToProcessBuff(val.v[len],snmpTrapPutData);

    len = dataTypeInfo.asnLen           // data bytes count
         + 1                            // Length byte
         + 1                            // Data type byte
         + OIDLen                       // OID bytes
         + 2                            // OID header bytes
         + 1;                           // index byte

    prevOffset = snmpTrapPutData->length;
    snmpTrapPutData->length = varPairStructLenOffset;
    SNMPPutDataToProcessBuff(len,snmpTrapPutData);

    len += 2;                           // Variable Binding structure header
    snmpTrapPutData->length = varBindStructLenOffset;
    SNMPPutDataToProcessBuff(len,snmpTrapPutData);

    len = len
        + 2                             // Var bind struct header
        + 6                             // 6 bytes of timestamp
        + 3                             // 3 bytes of trap code
        + 3                             // 3 bytes of notification code
        + 6                             // 6 bytes of agnent IP address
        + agentIDLen                    // Agent ID bytes
        + 2;                                // Agent ID header bytes
    snmpTrapPutData->length = pduStructLenOffset;
    SNMPPutDataToProcessBuff(len,snmpTrapPutData);

    len = len                           // PDU struct length
        + 2                             // PDU header
        + SnmpStackDcptMemStubPtr->SNMPNotifyInfo.communityLen            // Community string bytes
        + 2                             // Community header bytes
        + 3;                            // SNMP version bytes
    snmpTrapPutData->length = packetStructLenOffset;
    SNMPPutDataToProcessBuff(len,snmpTrapPutData);

    snmpTrapPutData->length = prevOffset;
// after setting all the offset values, initialize all static variables to 0.
	packetStructLenOffset = 0;
	pduStructLenOffset = 0;
	varBindStructLenOffset = 0;
	varPairStructLenOffset = 0;
	prevOffset = 0;

	SNMPPutDataArrayToUDPTxBuffer(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket,snmpTrapPutData);
    SYS_FS_close(snmpFileDescrptr);
    UDPFlush(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket);
    UDPClose(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket);
	TCPIP_HEAP_Free(SnmpStackMemH,SnmpStackDcptMemStubPtr->trapPduOutBufData.head);
    return true;
}

#endif 

TCPIP_NET_HANDLE SNMPUdpClientGetNet(void)
{
    static int                 		netIx=0;
    static TCPIP_NET_IF*         pNetIf=NULL;
    
	if(netIx >= TCPIP_STACK_NetworksNo())
	{
		netIx = 0;
		pNetIf = NULL;		
		
	}
	if(pNetIf == NULL)
	{
		pNetIf = (TCPIP_NET_IF*)TCPIP_STACK_GetDefaultNet();
		netIx++;
		
		return pNetIf;
	}
	for( ; netIx < TCPIP_STACK_NetworksNo(); )
	{
		++pNetIf;
		netIx++;

		if(pNetIf != NULL)
		{
			return pNetIf;
		}
		else
		{
			pNetIf = (TCPIP_NET_IF*)TCPIP_STACK_GetDefaultNet();
		}	
	}

    return pNetIf;

}
#endif // Code removed when SNMP_TRAP_DISABLED


/****************************************************************************
  Function:
	SNMP_ACTION SNMPProcessHeader(PDU_INFO* pduDbPtr,
							  char* community, uint8_t* len)

  Summary:
  	Validates the received udp packet Snmp header. 
	
  Description:
  	Collects PDU_INFO (SNMP pdu information database),community name,
  	community length and length of data payload. 
	This function validates the received udp packet for these different 
	variables of snmp pdu. The sequence in which these elements
	are received is important. The validation is done for the agent
	processing capabilities and the max UDP packet length as UDP packets
	can not be fragmented.
		
  Precondition:
	UDPIsGetReady(SNMPAgentSocket) is called in SNMPTask(),
	it check if there is any packet on SNMP Agent socket,
    should return true.
    
  Parameters:
  	 pduDbPtr  - Pointer to received pdu information database
	 community - Pointer to var storing, community string in rxed pdu
	 len	   - Pointer to var storing, community string length rxed in pdu	
	 
  Return Values:
	SNMP_ACTION - Snmp request pdu type.
	
  Remarks:
	The received pdu will be processed only if this routine returns the
	pdu type else the pdu is discarded as not Snmp pdu.
 ***************************************************************************/


static SNMP_ACTION SNMPProcessHeader(PDU_INFO* pduDbPtr, char* community, uint8_t* len)
{
    TCPIP_UINT32_VAL tempLen; 
	SNMP_ACTION pdu=0;   
	uint8_t snmpMsgBuf[7];/* 0x30,0x81/0x82/length,0xlen,0xlen,0x02,0x01,0x03(Snmp Version)*/
	TCPIP_UINT16_VAL snmpMsgLen;
	uint16_t retlen,tempCntr=0;
	uint8_t* tempPtr;
	uint8_t* inDataPtr;

	uint8_t intfIdx;
		
#ifdef TCPIP_STACK_USE_SNMPV3_SERVER	
	uint8_t extraMemReqdFor16BytesBlocks;

	SNMPV3_PROCESSING_MEM_INFO_PTRS snmpv3PktProcessingMemPntr; 
    SNMPV3_STACK_DCPT_STUB * snmpv3EngnDcptMemoryStubPtr=0; 	 
	
	SNMPv3GetPktProcessingDynMemStubPtrs(&snmpv3PktProcessingMemPntr);
							 
	snmpv3EngnDcptMemoryStubPtr=snmpv3PktProcessingMemPntr.snmpv3StkProcessingDynMemStubPtr;
#endif

	TCPIP_NET_IF* pNetIf = (TCPIP_NET_IF*)TCPIP_STACK_GetDefaultNet();
	intfIdx =  pNetIf->netIfIx; 

	tempPtr=snmpMsgBuf;
	// set the readFromSnmpBuf flag to true when we are reading from UDP SNMP socket buffer
	snmpDcpt[intfIdx].readFromSnmpBuf = false;
	
	//Get complete StructureOF var binds info
	*snmpMsgBuf = SNMPGetByteFromUDPRxBuff(snmpDcpt[intfIdx].skt);
	tempCntr+=1;
	
	if(!IS_STRUCTURE(*snmpMsgBuf))
		 return SNMP_ACTION_UNKNOWN;

	 // Retrieve structure length
    retlen= SNMPCheckIfValidLength(&tempLen.w[0]);
    if ( !retlen )
        return false;

	if(retlen == 1)
	{
		tempCntr+=1;
		*(snmpMsgBuf+1)=tempLen.w[0];	
	}
	else if( retlen== 2)
	{	
		tempCntr+=2;
		*(snmpMsgBuf+1)=0x81; //BER encoding
		*(snmpMsgBuf+2)=tempLen.w[0];
		
	} 
	else if(retlen== 3)
	{
		tempCntr+=3;
		*(snmpMsgBuf+1)=0x82;//BER encoding
		*(snmpMsgBuf+2)=tempLen.v[1];	
		*(snmpMsgBuf+3)=tempLen.v[0];	
	}
	else
	{
		return SNMP_ACTION_UNKNOWN;
	}
	snmpMsgLen.Val=tempLen.w[0];
	

	//Get snmp version info ASN_INT (1 Byte) + Length (1 Byte)+ snmp Version 1 Byte

	 if ( !SNMPCheckIfValidInt(&tempLen.Val) )
        return SNMP_ACTION_UNKNOWN;

	snmpMsgBuf[tempCntr]=ASN_INT;
		
	pduDbPtr->snmpVersion= tempLen.v[0];
    if ( (tempLen.v[0] != (uint8_t)SNMP_V1) && ( tempLen.v[0] != (uint8_t)SNMP_V2C )&&( tempLen.v[0] != (uint8_t)SNMP_V3 ))
        return SNMP_ACTION_UNKNOWN;

	snmpMsgBuf[tempCntr+1]=0x01;
	snmpMsgBuf[tempCntr+2]=tempLen.v[0];

#ifdef TCPIP_STACK_USE_SNMPV3_SERVER


	//Valid snmp packet in the SNMP UDP Socket received 
	if(pduDbPtr->snmpVersion == (uint8_t)SNMP_V3)
	{
		
		//Allocate dynamic memory to store the received SNMPv3 Whole Message.
		snmpv3EngnDcptMemoryStubPtr->InPduWholeMsgBuf.wholeMsgLen.Val=(snmpMsgLen.Val+tempCntr);
		extraMemReqdFor16BytesBlocks=16-(snmpv3EngnDcptMemoryStubPtr->InPduWholeMsgBuf.wholeMsgLen.Val%16);

		snmpv3EngnDcptMemoryStubPtr->InPduWholeMsgBuf.wholeMsgHead=(uint8_t*)(TCPIP_HEAP_Calloc(SnmpStackMemH,1,(size_t)snmpv3EngnDcptMemoryStubPtr->InPduWholeMsgBuf.wholeMsgLen.Val+extraMemReqdFor16BytesBlocks+16));

		if(snmpv3EngnDcptMemoryStubPtr->InPduWholeMsgBuf.wholeMsgHead == NULL)
		{
			return false;
		}
		
		snmpv3EngnDcptMemoryStubPtr->InPduWholeMsgBuf.snmpMsgHead=snmpv3EngnDcptMemoryStubPtr->InPduWholeMsgBuf.wholeMsgHead+tempCntr+3/*snmp Version info 0x02,0x01,0x03*/;
		snmpv3EngnDcptMemoryStubPtr->InPduWholeMsgBuf.snmpMsgLen.Val=snmpMsgLen.Val-3/*snmp Version info 0x02,0x01,0x03*/;


		//copy the WholeMsg structure info and snmp version info to dynamic mem from the UDP buffer. 
		//(WholeMsg is required to authenticate the received snmp pdu )//RFC 3414.
		tempCntr=tempCntr+3/*snmp Version info 0x02,0x01,0x03*/;
		inDataPtr=snmpv3EngnDcptMemoryStubPtr->InPduWholeMsgBuf.wholeMsgHead;
		while((tempCntr--)!=0)
		{
			*inDataPtr++=*tempPtr++;
		}

		inDataPtr=snmpv3EngnDcptMemoryStubPtr->InPduWholeMsgBuf.snmpMsgHead;
		tempCntr=snmpv3EngnDcptMemoryStubPtr->InPduWholeMsgBuf.snmpMsgLen.Val;

		while(tempCntr--)
		{
			*inDataPtr++=SNMPGetByteFromUDPRxBuff(snmpDcpt[intfIdx].skt);
	
		}
		
		pdu=SNMPv3MsgProcessingModelProcessPDU(SNMP_REQUEST_PDU);
		pdu=SNMPv3UserSecurityModelProcessPDU(SNMP_REQUEST_PDU);
		pdu=SNMPv3ScopedPduProcessing(SNMP_REQUEST_PDU);


		//Complete SNMPv3 data payload (Encrypted or as plain text) is received

		 //Check if received SNMPv3 message is Authenticated
		if((snmpv3EngnDcptMemoryStubPtr->SnmpSecurityLevel & 0x01)==0x01)
		{		
				//Message is authenticated
				if(SNMPv3AuthenticateRxedPduForDataIntegrity(&snmpv3EngnDcptMemoryStubPtr->InPduWholeMsgBuf)
					!= SNMPV3_MSG_AUTH_PASS)
				return SNMPV3_MSG_AUTH_FAIL;
		}	

		//Check if received SNMPv3 message is Encrypted.
		if((snmpv3EngnDcptMemoryStubPtr->SnmpSecurityLevel & 0x02)==0x02) 
		{
			 //Message is encrypted. Decrypt the message for processing
			 //user privacy protocol is AES

			if(SNMPv3AESDecryptRxedScopedPdu() != SNMPV3_MSG_PRIV_PASS)
				return SNMPV3_MSG_PRIV_FAIL;
		}	
	}

	else 
#endif	
	{

		snmpV1V2cIncomingMsgBufPtr.wholeMsgLen.Val=(snmpMsgLen.Val+tempCntr);
		snmpV1V2cIncomingMsgBufPtr.wholeMsgHead=(uint8_t*)(TCPIP_HEAP_Calloc(SnmpStackMemH,1,(size_t)snmpV1V2cIncomingMsgBufPtr.wholeMsgLen.Val+1));

		if(snmpV1V2cIncomingMsgBufPtr.wholeMsgHead == NULL)
		{
			return false;
		}
		
		snmpV1V2cIncomingMsgBufPtr.snmpMsgHead=snmpV1V2cIncomingMsgBufPtr.wholeMsgHead+tempCntr+3/*snmp Version info 0x02,0x01,0x03*/;
		snmpV1V2cIncomingMsgBufPtr.snmpMsgLen.Val=snmpMsgLen.Val-3/*snmp Version info 0x02,0x01,0x03*/;
		
		
		//copy the WholeMsg structure info and snmp version info to dynamic mem from the UDP buffer.
				// To copy the SNMP message payload to the message buffer
		tempCntr=tempCntr+3/*snmp Version info 0x02,0x01,0x<snmpversion>*/;
		inDataPtr=snmpV1V2cIncomingMsgBufPtr.wholeMsgHead;

		
		while((tempCntr--)!=0)
		{
			*inDataPtr++=*tempPtr++;
		}
		
		inDataPtr=snmpV1V2cIncomingMsgBufPtr.snmpMsgHead;
		tempCntr=snmpV1V2cIncomingMsgBufPtr.snmpMsgLen.Val;

		while(tempCntr--)
		{
			*inDataPtr++=SNMPGetByteFromUDPRxBuff(snmpDcpt[intfIdx].skt);		
		}
		
		// set the readFromSnmpBuf flag to true when we are reading from UDP SNMP socket buffer
		snmpDcpt[intfIdx].readFromSnmpBuf = true;
		SNMPRxOffset = 0;
	if((tempLen.v[0] == (uint8_t)SNMP_V1)||(tempLen.v[0] == (uint8_t)SNMP_V2C))
	{

	    // This function populates response as it processes community string.
		    if ( !SNMPCheckIfValidCommunityString(community, len) )
	        return SNMP_ACTION_UNKNOWN;

    	// Fetch and validate pdu type.  
	    	if ( !SNMPCheckIfValidPDU(&pdu) )
        return SNMP_ACTION_UNKNOWN;

		pduDbPtr->pduType = pdu;

		//Get_Bulk_Request is not defined in SNMP V1, hence discard udp request packet	
		if(pduDbPtr->snmpVersion==(uint8_t)SNMP_V1 && pduDbPtr->pduType == GET_BULK_REQUEST)
			return SNMP_ACTION_UNKNOWN;
	
	    // Ask main application to verify community name against requested pdu type.
	    if(SNMPValidateCommunity((uint8_t *)community)==(uint8_t)INVALID_COMMUNITY)
	        return SNMP_ACTION_UNKNOWN;
	}
	}
    return pdu;
}

/****************************************************************************
  Function:
	bool SNMPProcessGetSetHeader(PDU_INFO* pduDbPtr)
	
  Summary:
  	Validates the received udp packet Get/Set request header. 
	
  Description:
	All the variables of snmp pdu request header are validated for their
	data types. Collects request_id for the snmp request pdu. Fetch,validates
	error status,error index and discard as they are need not to be processed
	as received in request pdu. Collects non repeaters and max repeaters
	values in case of Get_Bulk request.  
  	
  Precondition:
	SNMPProcessHeader() is called and returns pdu type and do not returns 
	SNMP_ACTION_UNKNOWN
    
  Parameters:
  	pduDbPtr  - Pointer to received pdu information database.
  	
  Return Values:
	true  - If the received request header is validated and passed.
	false - If rxed request header is not valid.
	
  Remarks:
	The request pdu will be processed only if this routine returns true
 ***************************************************************************/
static bool SNMPProcessGetSetHeader(PDU_INFO* pduDbPtr)
{
    TCPIP_UINT32_VAL tempData;

    // Fetch and save request ID.
    if ( SNMPCheckIfValidInt(&tempData.Val) )
         pduDbPtr->requestID.Val = tempData.Val;
    else
        return false;

	if((pduDbPtr->snmpVersion == (uint8_t)SNMP_V1 || pduDbPtr->snmpVersion == (uint8_t)SNMP_V2C /*|| pduDbPtr->snmpVersion == (uint8_t)SNMP_V3*/) &&(pduDbPtr->pduType != GET_BULK_REQUEST))
	{
	    // Fetch and discard error status
	    if ( !SNMPCheckIfValidInt(&tempData.Val) )
	        return false;

	    // Fetch and disacard error index
	    return SNMPCheckIfValidInt(&tempData.Val);
	}
	else if((pduDbPtr->snmpVersion == (uint8_t)SNMP_V2C /*|| pduDbPtr->snmpVersion == (uint8_t)SNMP_V3*/ )&& pduDbPtr->pduType == GET_BULK_REQUEST )
	{
		// Fetch non-repeaters value
		if ( SNMPCheckIfValidInt(&tempData.Val) )
			 pduDbPtr->nonRepeators=tempData.v[0];
		else
			return false;
			
		// Fetch Max-repetitions value
		if(SNMPCheckIfValidInt(&tempData.Val))
			 pduDbPtr->maxRepetitions=(uint8_t)tempData.v[0];
		else
			return false;
	}
	else 
		return false;
	
	return true;
}


/****************************************************************************
  Function:
	bool SNMPProcessVariables(PDU_INFO* pduDbPtr,uint8_t* community, uint8_t len)
	
  Summary:
  	This routine processes the snmp request and parallely creates the 
  	response pdu.
	
  Description:
  	Once the received pdu is validated as Snmp pdu, it is forwarded for 
  	processing to this routine. This rotuine handles Get, Get_Next, Get_Bulk,
  	Set request and creates appropriate response as Get_Response. 
  	This routine will decide on whether the request pdu should be processed
  	or be discarded.
	
  Precondition:
	The received udp packet is varified as SNMP request.
	SNMPProcessHeader() and SNMPProcessGetSetHeader() returns but false.
	
  Parameters:
  	pduDbPtr  - Pointer to received pdu information database
    community - Pointer to var, storing community string in rxed pdu
	len	   	  - Pointer to var, storing community string length rxed in pdu
	
  Return Values:
	true 	- If the snmp request processing is successful.
	false	- If the processing failed else the processing is not completed.
	
  Remarks:
	None
 ***************************************************************************/


static bool SNMPProcessVariables(PDU_INFO* pduDbPtr,char* community, uint8_t len)
{	
 	uint8_t getbulkOverFlowFlag = false;
    uint8_t temp =0;
    uint8_t OIDValue[OID_MAX_LEN];
    uint8_t OIDLen=0;
   	uint8_t varIndex =0;
	uint8_t errorIndex;
	uint8_t communityLen=0,commRetVal=0;
	uint8_t noOfOIDsInReq=0,tempNonRepeators=0,noOfVarToBeInResponse=0;
	uint8_t repeatCntr,varBindCntr;
	uint8_t Getbulk_N=0,Getbulk_M=0,Getbulk_R=0;/*Refer RFC 3416 Section "4.2.3" GetBulkRequest-PDU*/
	uint8_t oidLookUpRet=0;
	uint8_t templen=0;
	uint8_t successor=0;// 'I'th lexicographic successor 	 
	uint8_t *ptemp;
	uint8_t *ptroid;
	uint8_t *rxedCommunityName;
	uint16_t varBindStructOffset=0;
   	uint16_t tempTxOffset=0;
    uint16_t oidOffset=0;
    uint16_t prevOffset=0;
	uint16_t packetStructLenOffset=0;
	//uint16_t snmpV3ScopedPduOffset=0;
    uint16_t pduLenOffset=0;
    uint16_t errorStatusOffset=0;
    uint16_t errorIndexOffset=0;    
    uint16_t varStructLenOffset=0;	
	uint16_t prevSnmpRxOffset=0;	
    TCPIP_UINT16_VAL varBindingLen={0};
    TCPIP_UINT16_VAL tempLen={0};
    TCPIP_UINT16_VAL varPairLen={0};
    static TCPIP_UINT16_VAL varBindLen={0};
	OID_INFO OIDInfo;  
    SNMP_ERR_STATUS errorStatus;
#ifdef TCPIP_STACK_USE_SNMPV3_SERVER	
    uint8_t agentIDLen;
    OID_INFO rec;
//	uint8_t OIDValue[OID_MAX_LEN];
	TCPIP_UINT16_VAL tempByteCntr;
#endif
    bool bSnmpGenError = false;
    
    bool bSnmpSendPkt = false;
	TCPIP_UINT16_VAL bytesAdded2Pdu;
	uint8_t intfIdx;
	SNMPBUFFERDATA *snmpPutData=NULL;	
	static enum 
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

	snmpReqVarErrStatus.noSuchInstanceErr=0x0000;
	snmpReqVarErrStatus.noSuchNameErr=0x0000;
	snmpReqVarErrStatus.noSuchObjectErr=0x0000;
	snmpReqVarErrStatus.endOfMibViewErr=0x0000;

	rxedCommunityName=(uint8_t *)community;
	/* Locate the start offset of the TX PDU */
	//tempTxOffset = _SNMPGetTxOffset();
	tempTxOffset = SNMPTxOffset;
	OIDLen = 0;
	varBindLen.Val=0x0000;
	SnmpStackDcptMemStubPtr->getZeroInstance = false;
	bytesAdded2Pdu.Val=0x00;
	
	intfIdx = ((TCPIP_NET_IF*)TCPIP_STACK_GetDefaultNet())->netIfIx; 
	
	while(1)
	{
		switch(smSnmp)
		{

		// Before each variables are processed, prepare necessary header.

		case SM_PKT_STRUCT_LEN_OFFSET:
			snmpPutData = &SnmpStackDcptMemStubPtr->outPduBufData;

			varPairLen.Val=0x0000;
			
			SNMPPutDataToProcessBuff(STRUCTURE,snmpPutData);  // first item in snmp packet
			SNMPPutDataToProcessBuff(0x82,snmpPutData);

			// Since we do not know length of structure at this point, use
			// placeholder bytes that will be replaced with actual value.

			packetStructLenOffset = snmpPutData->length;
			SNMPPutDataToProcessBuff(0,snmpPutData);
			SNMPPutDataToProcessBuff(0,snmpPutData);

			// Put SNMP version info - only v1.0 is supported.
			SNMPPutDataToProcessBuff(ASN_INT,snmpPutData);              // Int type.
			SNMPPutDataToProcessBuff(1,snmpPutData);                    // One byte long value.
			SNMPPutDataToProcessBuff(pduDbPtr->snmpVersion,snmpPutData);              // v1.0.

			// Put community string
			communityLen = len;             // Save community length for later use.
			SNMPPutDataToProcessBuff(OCTET_STRING,snmpPutData);         // Octet string type.
			SNMPPutDataToProcessBuff(len,snmpPutData);                  // community string length
			while( len-- )                  // Copy entire string.
				SNMPPutDataToProcessBuff(*community++,snmpPutData);
			
			smSnmp++;

			//return false;
		case SM_RESPONSE_PDU_LEN_OFFSET:

			// Put PDU type.  SNMP agent's response is always GET RESPONSE
			SNMPPutDataToProcessBuff(GET_RESPONSE,snmpPutData);
			
			// Since we don't know length of this response, use placeholders until
			// we know for sure...
			SNMPPutDataToProcessBuff(0x82,snmpPutData); 
			pduLenOffset = snmpPutData->length;
			SNMPPutDataToProcessBuff(0,snmpPutData); // Be prepared for 2 byte-long length
			SNMPPutDataToProcessBuff(0,snmpPutData);

			// Put original request back.
			SNMPPutDataToProcessBuff(ASN_INT,snmpPutData);	// Int type.
			SNMPPutDataToProcessBuff(4,snmpPutData);		// To simplify logic, always use 4 byte long requestID
			SNMPPutDataToProcessBuff(pduDbPtr->requestID.v[3],snmpPutData); // Start MSB
			SNMPPutDataToProcessBuff(pduDbPtr->requestID.v[2],snmpPutData);
			SNMPPutDataToProcessBuff(pduDbPtr->requestID.v[1],snmpPutData);
			SNMPPutDataToProcessBuff(pduDbPtr->requestID.v[0],snmpPutData);

			smSnmp++;

			//return false;
			
		case SM_ERROR_STATUS_OFFSET :

			// Put error status.
			// Since we do not know error status, put place holder until we know it...
			SNMPPutDataToProcessBuff(ASN_INT,snmpPutData);              // Int type
			SNMPPutDataToProcessBuff(1,snmpPutData);                    // One byte long.
			errorStatusOffset = snmpPutData->length;
			SNMPPutDataToProcessBuff(0,snmpPutData);                    // Placeholder.
			smSnmp++;

		case SM_ERROR_INDEX_OFFSET :

			// Similarly put error index.
			SNMPPutDataToProcessBuff(ASN_INT,snmpPutData);              // Int type
			SNMPPutDataToProcessBuff(1,snmpPutData);                    // One byte long
			errorIndexOffset = snmpPutData->length;
			SNMPPutDataToProcessBuff(0,snmpPutData);                    // Placeholder.

			varIndex    = 0;
			errorIndex  = 0;
			errorStatus = SNMP_NO_ERR;

			smSnmp++;

		case SM_FIND_NO_OF_REQUESTED_VARBINDS:

			// Decode variable binding structure
			if ( !SNMPCheckIfValidSnmpStructure(&varBindingLen.Val) )
			return false;

			//Find number of OIDs/varbinds's data requested in received PDU.
			noOfOIDsInReq=SNMPFindOIDsInRequest(varBindingLen.Val);
			
			smSnmp++;	

			//return false;

		case SM_FIND_NO_OF_RESPONSE_VARBINDS:

			//Calulate number of variables to be responded for the received request
			Getbulk_N = noOfOIDsInReq; Getbulk_M=0; Getbulk_R=0;
			if(((pduDbPtr->snmpVersion == (uint8_t)SNMP_V2C)||
				((pduDbPtr->snmpVersion == (uint8_t)SNMP_V3))) && 
				(pduDbPtr->pduType == GET_BULK_REQUEST))
			{
				if((pduDbPtr->nonRepeators) <= noOfOIDsInReq)
				{
					Getbulk_N = pduDbPtr->nonRepeators;
				}

				Getbulk_M = pduDbPtr->maxRepetitions;

				if((noOfOIDsInReq - Getbulk_N)>=0u)
					Getbulk_R = noOfOIDsInReq-Getbulk_N;
			}

			tempNonRepeators = Getbulk_N;

			noOfVarToBeInResponse = Getbulk_N + (Getbulk_M * Getbulk_R);//Refer RFC 3416 

			smSnmp++;

			//return false;

		case SM_VARBIND_STRUCT_OFFSET:
	
			// Put variable binding response structure
			SNMPPutDataToProcessBuff(STRUCTURE,snmpPutData);
			SNMPPutDataToProcessBuff(0x82,snmpPutData);

			// Since we do not know data payload length, put place holder until we know it...
			varBindStructOffset = snmpPutData->length;
			SNMPPutDataToProcessBuff(0,snmpPutData);
			SNMPPutDataToProcessBuff(0,snmpPutData);

			varBindLen.Val = 0;

			smSnmp++;

			//return false;
			
		case SM_VARSTRUCT_LEN_OFFSET:

			/*	If the getbulk request is received with zero non-repeaters, process
				variable State Machine jumps to SM_MAX_REPETITIONS. Modify the Rx
				and Tx offset accordigly. */
			if(Getbulk_N==0u)
			{

			#ifdef TCPIP_STACK_USE_SNMPV3_SERVER
				if((pduDbPtr->snmpVersion == (uint8_t)SNMP_V3))
				{
					// Get complete OID string from snmp.bib file.
				    SNMPFindOIDStringByID(MICROCHIP,&rec, OIDValue, &agentIDLen);
				    
				    if ( rec.nodeInfo.Flags.bIsAgentID )
				    {
						SYS_FS_lseek(snmpFileDescrptr, rec.hData, SEEK_SET);
				    }

				    SNMPPutDataToProcessBuff(ASN_OID,snmpPutData);

					SYS_FS_read(snmpFileDescrptr,&len,1);
					
				    agentIDLen = len;
				    SNMPPutDataToProcessBuff(len,snmpPutData);
				    while( len-- )
				    {
					    uint8_t c;
						SYS_FS_read(snmpFileDescrptr,&c,1);
				        SNMPPutDataToProcessBuff(c,snmpPutData);
				    }
							
				}
				else
			#endif			
				{
			
					prevSnmpRxOffset=SNMPRxOffset;
					smSnmp=SM_MAX_REPETITIONS;
					varStructLenOffset = snmpPutData->length;
					snmpPutData->length=snmpPutData->length+4;
					break;
				}
			}
			
			/*
				Need to know what variable we are processing, so that in case
				if there is problem for that variable, we can put it in
				errorIndex location of SNMP packet.
			*/
			varIndex++;

			// Decode variable length structure
			temp = SNMPCheckIfValidSnmpStructure(&tempLen.Val);
			if ( !temp )
			{
				SNMPSetErrorStatus(errorStatusOffset,errorIndexOffset,SNMP_GEN_ERR,varIndex,snmpPutData);
                bSnmpGenError = true;
                break;
			}
			
			varBindingLen.Val -= tempLen.Val;
			varBindingLen.Val -= temp;

			varStructLenOffset = snmpPutData->length;

			if(pduDbPtr->pduType == GET_BULK_REQUEST )
			{
				snmpPutData->length=snmpPutData->length+4;
			}
			smSnmp++;

			//return false;

		case SM_POPULATE_REQ_OID:
			
			/* 	Populate received pdu for the requested OIDs and also create the 
				response pdu on the go.*/
			
			// Decode next object
			if ( !SNMPCheckIfValidOID(OIDValue, &OIDLen) )
			{
				SNMPSetErrorStatus(errorStatusOffset,errorIndexOffset,SNMP_GEN_ERR,varIndex,snmpPutData);
                bSnmpGenError = true;
                break;
			}

			// For Get & Get-Next, value must be NULL.
			if ( pduDbPtr->pduType != (uint8_t)SET_REQUEST )
			{
				if ( !SNMPCheckIsASNNull() )
				return false;
			}

			if(pduDbPtr->pduType != GET_BULK_REQUEST )
			{
				// Prepare response - original variable
				SNMPPutDataToProcessBuff(ASN_OID,snmpPutData);
				oidOffset = snmpPutData->length;
				SNMPPutDataToProcessBuff(OIDLen,snmpPutData);
				ptemp = OIDValue;
				temp = OIDLen;
				while( temp-- )
				SNMPPutDataToProcessBuff(*ptemp++,snmpPutData);
			}


			/* 
			   Match "rxedCommunityName" to "readCommunity" to authorize access
			   to private MIB objects.
			   As we start supporting the secured encrypted community transaction, 
			   rxed community string can be an encrypted string which the agent
			   need to decrypt and validate to autohrize access.
			   The agent should respond with encrypted community name.
			*/

			if((pduDbPtr->snmpVersion != (uint8_t)SNMP_V3))
			{	
				commRetVal=SNMPValidateCommunity(rxedCommunityName);
					
				smSnmp=SM_PKT_STRUCT_LEN_OFFSET;	// Start out assuming commRetVal == INVALID_COMMUNITY
				if(pduDbPtr->pduType == (uint8_t)SET_REQUEST)
				{	
					if(commRetVal==(uint8_t)WRITE_COMMUNITY)//If SET request, then "community==WRITE_COMMUNITY" is must.
					{
						smSnmp=SM_FIND_OID_IN_MIB;
					}
					
				}
				else 
				{	
					if(commRetVal!=(uint8_t)INVALID_COMMUNITY)//If any GET request, then "community!=INVALID_COMMUNITY" is must (community is WRITE_COMMUNITY or READ_COMMUNITY).
					{
						smSnmp=SM_FIND_OID_IN_MIB;
					}
					
				}
			
			}
#ifdef TCPIP_STACK_USE_SNMPV3_SERVER	
			else
			{
				smSnmp=SM_FIND_OID_IN_MIB;

			}
#endif

			//Verify if trying to access the private object
			//Application has to decide on what community name should allowed to
			//read write the private mib objects.

			if(SNMPCheckIfPvtMibObjRequested(OIDValue) && (smSnmp==SM_PKT_STRUCT_LEN_OFFSET) )
			{
				//If private mib object is requested and community do not match, 
				//generate authentication failure TRAP

				Getbulk_N=0;
				noOfVarToBeInResponse=0;
				smSnmp=SM_PKT_STRUCT_LEN_OFFSET;	

				//Searching the requested OID in the MIB database 
				oidLookUpRet = SNMPSearchOIDInMgmtInfoBase(pduDbPtr,OIDValue, OIDLen, &OIDInfo);	
				SnmpStackDcptMemStubPtr->gOIDCorrespondingSnmpMibID=OIDInfo.id;

				//_SNMPSetTxOffset(packetStructLenOffset-2);
				SNMPTxOffset=packetStructLenOffset-2;
				SnmpStackDcptMemStubPtr->gSpecificTrapNotification=VENDOR_TRAP_DEFAULT;
				SnmpStackDcptMemStubPtr->gGenericTrapNotification=AUTH_FAILURE;
				SnmpStackDcptMemStubPtr->gSendTrapFlag=true;	

			}
			/*else 
				smSnmp++;*/

			if(smSnmp==SM_PKT_STRUCT_LEN_OFFSET || smSnmp==SM_VARSTRUCT_LEN_OFFSET)
				break;	
				
			//return false;
			
		case SM_FIND_OID_IN_MIB:

			/* Search for the requested OID in the MIB database with the agent.*/
			
			if(Getbulk_N!= 0u)
				Getbulk_N--;
			
			if(Getbulk_N==0u)
				prevSnmpRxOffset=SNMPRxOffset;

			noOfVarToBeInResponse--;

			//Searching the requested OID in the MIB database 
			oidLookUpRet = SNMPSearchOIDInMgmtInfoBase(pduDbPtr,OIDValue, OIDLen, &OIDInfo);	

			if(oidLookUpRet != (uint8_t)true && (pduDbPtr->pduType != GET_NEXT_REQUEST) &&
				(pduDbPtr->pduType != GET_BULK_REQUEST))
			{
				snmpPutData->length = varStructLenOffset;

				// Put corresponding variable response structure
				SNMPPutDataToProcessBuff(STRUCTURE,snmpPutData);
				SNMPPutDataToProcessBuff(0x82,snmpPutData);
				
				varStructLenOffset= snmpPutData->length; 
				SNMPPutDataToProcessBuff(0x00,snmpPutData);//Place holder
				SNMPPutDataToProcessBuff(0x00,snmpPutData);

				// ASN OID data type
				templen=OIDLen;
				ptroid=OIDValue;	
				SNMPPutDataToProcessBuff(ASN_OID,snmpPutData);

				if(SnmpStackDcptMemStubPtr->appendZeroToOID)
					SNMPPutDataToProcessBuff(OIDLen+1,snmpPutData);//for appending "0"
				else 
					SNMPPutDataToProcessBuff(OIDLen,snmpPutData);//do not append "0"		

				//Put OID
				while( templen-- )
				SNMPPutDataToProcessBuff(*ptroid++,snmpPutData);

				if(SnmpStackDcptMemStubPtr->appendZeroToOID)
				{
					SNMPPutDataToProcessBuff(0x00,snmpPutData);//Appending '0' to OID in response
					varPairLen.Val += OIDLen+1+2; //Modify the response length
				}
				else 
					varPairLen.Val += OIDLen+2;
			
				//update and send the error status and the error index.
				if(pduDbPtr->snmpVersion == (uint8_t)SNMP_V1)
				{
					errorStatus = SNMP_NO_SUCH_NAME;
					SNMPSetErrorStatus(errorStatusOffset,errorIndexOffset,SNMP_NO_SUCH_NAME,varIndex,snmpPutData);
					SNMPPutDataToProcessBuff(ASN_NULL,snmpPutData);
					SNMPPutDataToProcessBuff(0,snmpPutData);	
				}
				else if(((pduDbPtr->snmpVersion == (uint8_t)SNMP_V2C)
						||( pduDbPtr->snmpVersion == (uint8_t)SNMP_V3))
						&& pduDbPtr->pduType != SET_REQUEST)
				{
					if(pduDbPtr->pduType == SNMP_GET)
					{
						SNMPPutDataToProcessBuff(oidLookUpRet,snmpPutData);
						SNMPPutDataToProcessBuff(0x00,snmpPutData);
						if(oidLookUpRet == SNMP_NO_SUCH_OBJ)
						{
							snmpReqVarErrStatus.noSuchObjectErr|=(0x0001 << varIndex);
						}
						else if(oidLookUpRet == SNMP_NO_SUCH_INSTANCE)
						{
							snmpReqVarErrStatus.noSuchInstanceErr|=(0x0001 << varIndex);
						}
					}
				}

				if(pduDbPtr->snmpVersion !=SNMP_V3)
				varPairLen.Val +=2 ;

				varBindLen.Val += 4	// Variable Pair STRUCTURE byte + 1 length byte.
				+ varPairLen.Val;

				//Now update the place holder for var pair length
				prevOffset = snmpPutData->length; 
				snmpPutData->length = varStructLenOffset;

				SNMPPutDataToProcessBuff(varPairLen.v[1],snmpPutData);
				SNMPPutDataToProcessBuff(varPairLen.v[0],snmpPutData);

				snmpPutData->length = prevOffset;
				varPairLen.Val=0x00;

				//Reset to state machine to access the next oid in request
				smSnmp=SM_VARSTRUCT_LEN_OFFSET;
				break;	
			}
			smSnmp++;

			//return false;

		case SM_NON_REPETITIONS:

			/* 	Variables in get,get_next,set and get_bulk ( non repetition variables)
				of snmp request are processed in this part of the state machine.*/
			
			//Save SnmpTxOffsetfor future uses.
			prevOffset = snmpPutData->length;
			snmpPutData->length = varStructLenOffset;

			//Start response variable binding with ASN STRUCTURE type.	
			SNMPPutDataToProcessBuff(STRUCTURE,snmpPutData);
			SNMPPutDataToProcessBuff(0x82,snmpPutData);

			varStructLenOffset= snmpPutData->length;
			SNMPPutDataToProcessBuff(0x00,snmpPutData); //place holder
			SNMPPutDataToProcessBuff(0x00,snmpPutData);

			SNMPPutDataToProcessBuff(ASN_OID,snmpPutData); 

			if(pduDbPtr->pduType == SNMP_SET)
            {
            	templen=OIDLen;
				ptroid=OIDValue;	
				//to validate the REC ID is present or not
	
				if(SNMPIdRecrdValidation(pduDbPtr,&OIDInfo,OIDValue,OIDLen) != true)
				{

					 /*if the variable binding's name specifies a
				     * variable which does not exist and could not ever be
				     * created, then the value of the Response-PDU's error-
				     * status field is set to `noCreation', and the value of its
				     * error-index field is set to the index of the failed
				     * variable binding.
				     */
					errorStatus = SNMP_NO_CREATION;
					smSnmp=SM_PKT_STRUCT_LEN_OFFSET;
					return false;
				}

				if(SnmpStackDcptMemStubPtr->appendZeroToOID)
					SNMPPutDataToProcessBuff(OIDLen+1,snmpPutData);//for appending "0"
				else 
					SNMPPutDataToProcessBuff(OIDLen,snmpPutData);//do not append "0"		

				//Put OID
				while( templen-- )
				SNMPPutDataToProcessBuff(*ptroid++,snmpPutData);

				if(SnmpStackDcptMemStubPtr->appendZeroToOID)
					SNMPPutDataToProcessBuff(0x00,snmpPutData);//Appending '0' to OID in response

				//Now process the SET command
                temp = SNMPProcessSetVar(pduDbPtr,&OIDInfo, &errorStatus);

				if ( errorStatus != SNMP_NO_ERR )
                {
	                //SET var command failed. Update the error status.
                    SNMPSetErrorStatus(errorStatusOffset,
                                   errorIndexOffset,
                                   errorStatus,
                                   varIndex,snmpPutData);

                }

				if(SnmpStackDcptMemStubPtr->appendZeroToOID)
					varPairLen.Val = OIDLen+1 +2   // OID name + header bytes
                                + temp;            // value bytes as put by SetVar
				else
					varPairLen.Val = OIDLen+2+temp;

			}
			else if((pduDbPtr->pduType == SNMP_GET)  ||
				((pduDbPtr->pduType == SNMP_V2C_GET_BULK) && (oidLookUpRet == true)))
			{	
				//to validate the REC ID is present or not
				if(SNMPIdRecrdValidation(pduDbPtr,&OIDInfo,OIDValue,OIDLen) != true)
				{
					smSnmp=SM_PKT_STRUCT_LEN_OFFSET;
					return false;
				}
				templen=OIDLen;
				ptroid=OIDValue;	
				
				if(SnmpStackDcptMemStubPtr->appendZeroToOID)
					SNMPPutDataToProcessBuff(OIDLen+1,snmpPutData);//for appending "0"
				else 
					SNMPPutDataToProcessBuff(OIDLen,snmpPutData);//do not append "0"

				//Put OID
				while( templen-- )
					SNMPPutDataToProcessBuff(*ptroid++,snmpPutData);
				
				if(SnmpStackDcptMemStubPtr->appendZeroToOID)
				{
					SNMPPutDataToProcessBuff(0x00,snmpPutData);//Appending '0' to OID in response
					varPairLen.Val = OIDLen + 2+1;
				}
				else 
					varPairLen.Val = OIDLen +2;
				
				
				//Now process the GET command
				temp=SNMPProcessGetVar(&OIDInfo,false,pduDbPtr);

			}	
			else if((pduDbPtr->pduType == SNMP_GET_NEXT)||
				((pduDbPtr->pduType == SNMP_V2C_GET_BULK) && (oidLookUpRet != true)))
			{			
				temp=SNMPProcessGetNextVar(&OIDInfo,pduDbPtr);

				//If Get Next command failed
				if(temp==0u)
				{
					templen=OIDLen;
					ptroid=OIDValue;
					
					if(SnmpStackDcptMemStubPtr->appendZeroToOID)
						SNMPPutDataToProcessBuff(OIDLen+1,snmpPutData);//for appending "0"
					else 
						SNMPPutDataToProcessBuff(OIDLen,snmpPutData);//do not append "0"

					//Put OID
					while( templen-- )
						SNMPPutDataToProcessBuff(*ptroid++,snmpPutData);
					
					if(SnmpStackDcptMemStubPtr->appendZeroToOID)
						SNMPPutDataToProcessBuff(0x00,snmpPutData);//Appending '0' to OID in response
				}
			}


			/*  If the request command processing is failed, update
				the error status, index accordingly and response pdu.*/ 
			if(temp == 0u &&(pduDbPtr->pduType != SNMP_SET))
			{
				if(pduDbPtr->snmpVersion == (uint8_t)SNMP_V1)
				{
					errorStatus = SNMP_NO_SUCH_NAME;
					SNMPSetErrorStatus(errorStatusOffset,errorIndexOffset,SNMP_NO_SUCH_NAME,
								   varIndex,snmpPutData);

				}

				SNMPPutDataToProcessBuff(ASN_NULL,snmpPutData);
				SNMPPutDataToProcessBuff(0,snmpPutData);

				if((pduDbPtr->pduType == SNMP_GET_NEXT|| pduDbPtr->pduType == SNMP_V2C_GET_BULK)&&pduDbPtr->snmpVersion == (uint8_t)SNMP_V2C)
				{
					snmpPutData->length=snmpPutData->length-2;
					SNMPPutDataToProcessBuff(SNMP_END_OF_MIB_VIEW,snmpPutData);
					SNMPPutDataToProcessBuff(0,snmpPutData);
				
				}

				if((pduDbPtr->pduType == SNMP_GET) || 
					((pduDbPtr->pduType == SNMP_V2C_GET_BULK) && (oidLookUpRet == true)))
				{
					temp = 2;
				}
				else if((pduDbPtr->pduType == SNMP_GET_NEXT)||
					((pduDbPtr->pduType == SNMP_V2C_GET_BULK) && (oidLookUpRet != true)))
				{
				     varPairLen.Val = OIDLen+1          // as put by GetNextVar()
                                     + 2                // OID header
                                     + 2;               // END_OF_MIB_VIEW bytes
				}


				/* 	Applications can make use of the below information 
					to find the error status for the given variable and to 
					build the logic arround. */
				snmpReqVarErrStatus.noSuchNameErr	 |=(0x0001 << varIndex);
				snmpReqVarErrStatus.noSuchObjectErr	 |=(0x0001 << varIndex);
				snmpReqVarErrStatus.noSuchInstanceErr|=(0x0001 << varIndex);
				snmpReqVarErrStatus.endOfMibViewErr	 |=(0x0001 << varIndex);
				
			}
			else if((pduDbPtr->pduType == SNMP_GET_NEXT)||
				((pduDbPtr->pduType == SNMP_V2C_GET_BULK) && (oidLookUpRet != true)))
			{
				if(SnmpStackDcptMemStubPtr->getZeroInstance)
					varPairLen.Val += temp+2;
				else
					varPairLen.Val = (temp + 2);
			}
	
			if((pduDbPtr->pduType == SNMP_GET) || 
				((pduDbPtr->pduType == SNMP_V2C_GET_BULK) && (oidLookUpRet == true)))
				varPairLen.Val += temp;   

			varBindLen.Val += 4	// Variable Pair STRUCTURE byte + 1 length byte.
			+ varPairLen.Val;

			//Update place holder
			prevOffset =  snmpPutData->length;
			snmpPutData->length = varStructLenOffset;
			SNMPPutDataToProcessBuff(varPairLen.v[1],snmpPutData);
			SNMPPutDataToProcessBuff(varPairLen.v[0],snmpPutData);

			snmpPutData->length = prevOffset;
			varStructLenOffset = snmpPutData->length;


			/* 	Decide on the number of Non repetition variables remained to 
				be processed, decide the course of state machine.*/
				
			if((pduDbPtr->pduType==GET_BULK_REQUEST) &&
			   ((pduDbPtr->snmpVersion == (uint8_t)SNMP_V2C)||
			    (pduDbPtr->snmpVersion == (uint8_t)SNMP_V3))&&( Getbulk_N == 0u))
			{
                if((varStructLenOffset - tempTxOffset) >= SNMP_MAX_MSG_SIZE)
                {
                    getbulkOverFlowFlag = true;
                    break;
                }
                else
                {
                    smSnmp=SM_MAX_REPETITIONS;
                }
			}
			else
                smSnmp=SM_VARSTRUCT_LEN_OFFSET;

			
			varPairLen.Val=0x00;
			SnmpStackDcptMemStubPtr->getZeroInstance = false;

			/* check length*/
			break;	

			//return false;

		case SM_MAX_REPETITIONS:

			/*Process each variable in request as Get_Next for 
			  Getbulk_M (Max_repetition) times */
			for(repeatCntr=0;repeatCntr<Getbulk_M;repeatCntr++)
			{
				SNMPRxOffset=prevSnmpRxOffset;

				//Process every veriable in the request.
				for(varBindCntr=0;varBindCntr<Getbulk_R;varBindCntr++)
				{
                    if(varBindCntr==0u)
                            varIndex=(noOfOIDsInReq-Getbulk_R);

                    varIndex++;

                    if((snmpReqVarErrStatus.endOfMibViewErr >> (tempNonRepeators+varBindCntr+1))&0x0001)
                    {
                        noOfVarToBeInResponse--;
                        temp = SNMPCheckIfValidSnmpStructure(&tempLen.Val);

                        if(varBindCntr!=Getbulk_R)
                        {
                            SNMPRxOffset=SNMPRxOffset+tempLen.Val;//2+OIDLen+2;
                        }
                        continue;
                    }

                    if(noOfVarToBeInResponse != 0)
                    {
                        noOfVarToBeInResponse--;
                    }
                    varPairLen.Val = 0;
                    prevOffset = snmpPutData->length;
                    snmpPutData->length = varStructLenOffset;
                    if(SNMPPutDataToProcessBuff(STRUCTURE,snmpPutData)!= true)
                    {
                        getbulkOverFlowFlag = true;
                        break;
                    }
                    if(SNMPPutDataToProcessBuff(0x82,snmpPutData)!= true)
                    {
                        getbulkOverFlowFlag = true;
                        break;
                    }
                    varStructLenOffset= snmpPutData->length;
                    if(SNMPPutDataToProcessBuff(0x00,snmpPutData)!= true)
                    {
                        getbulkOverFlowFlag = true;
                        break;
                    }
                    if(SNMPPutDataToProcessBuff(0x00,snmpPutData)!= true)
                    {
                        getbulkOverFlowFlag = true;
                        break;
                    }
                    successor=repeatCntr;
                    // Decode variable length structure
                    temp = SNMPCheckIfValidSnmpStructure(&tempLen.Val);
                    if ( !temp )
                            break;

                    // Decode next object
                    if ( !SNMPCheckIfValidOID(OIDValue, &OIDLen) )
                    {
                        SNMPSetErrorStatus(errorStatusOffset,errorIndexOffset,SNMP_GEN_ERR,varIndex,snmpPutData);
                        bSnmpGenError = true;
                        break;
                    }
                    templen=OIDLen;
                    ptroid=OIDValue;

                    // For Get & Get-Next, value must be NULL.
                    if ( pduDbPtr->pduType != (uint8_t)SET_REQUEST )
                        if (!SNMPCheckIsASNNull())
                        break;

                    oidLookUpRet = SNMPSearchOIDInMgmtInfoBase(pduDbPtr,OIDValue, OIDLen, &OIDInfo);
                    if(oidLookUpRet == SNMP_END_OF_MIB_VIEW)
                    {
                        temp = SNMPGetNextLeaf(&OIDInfo);
                    }
                    if(oidLookUpRet == false)
                    {
                        templen=OIDLen;
                        ptroid=OIDValue;
                        if(SNMPPutDataToProcessBuff(ASN_OID,snmpPutData)!= true)
                        {
                            getbulkOverFlowFlag = true;
                            break;
                        }

                        if(SnmpStackDcptMemStubPtr->appendZeroToOID)
                        {
                            if(SNMPPutDataToProcessBuff(OIDLen+1,snmpPutData)!= true)//for appending "0"
                            {
                                getbulkOverFlowFlag = true;
                                break;
                            }
                            OIDLen += 1;
                        }
                        else
                        {
                            if(SNMPPutDataToProcessBuff(OIDLen,snmpPutData)!= true)
                            {
                                getbulkOverFlowFlag = true;
                                break;
                            }
                        }

                        //Put OID
                        while( templen-- )
                        {
                            if(SNMPPutDataToProcessBuff(*ptroid++,snmpPutData)!= true)
                            {
                                getbulkOverFlowFlag = true;
                                break;
                            }
                        }

                        if(SnmpStackDcptMemStubPtr->appendZeroToOID)
                        {
                            if(SNMPPutDataToProcessBuff(0x00,snmpPutData)!= true)
                            {
                                getbulkOverFlowFlag = true;
                                break;
                            }
                        }

                        if(SNMPPutDataToProcessBuff(SNMP_END_OF_MIB_VIEW,snmpPutData)!= true)
                        {
                            getbulkOverFlowFlag = true;
                            break;
                        }
                        if(SNMPPutDataToProcessBuff(0x00,snmpPutData)!= true)
                        {
                            getbulkOverFlowFlag = true;
                            break;
                        }

                        //Start counting total number of bytes in this structure.
                        varPairLen.Val = OIDLen // as put by GetNextVar()
                         +2       // OID header
                         +2;      // endOfMibView bytes

                        snmpReqVarErrStatus.endOfMibViewErr	 |=(0x0001 << varIndex);
                    }
                    else if(temp != 0)//if(oidLookUpRet != SNMP_END_OF_MIB_VIEW)
                    {
                        temp = SNMPProcessGetBulkVar(&OIDInfo, &OIDValue[0],&OIDLen,&successor,pduDbPtr);
                    }
                    if ( temp == 0u )
                    {
                        templen=OIDLen;
                        ptroid=OIDValue;
                        if(SNMPPutDataToProcessBuff(ASN_OID,snmpPutData)!=true)
                        {
                            getbulkOverFlowFlag = true;
                            break;
                        }
                        if(SNMPPutDataToProcessBuff(OIDLen,snmpPutData)!= true)
                        {
                            getbulkOverFlowFlag = true;
                            break;
                        }

                        //Put OID
                        while( templen-- )
                        {
                            if(SNMPPutDataToProcessBuff(*ptroid++,snmpPutData)!= true)
                            {
                                getbulkOverFlowFlag = true;
                                break;
                            }
                        }

                        /*Do send back the Same OID if get_next is EndOfMibView. Do not
                          append zero to this OID*/

                        if(SNMPPutDataToProcessBuff(SNMP_END_OF_MIB_VIEW,snmpPutData)!=true)
                        {
                            getbulkOverFlowFlag = true;
                            break;
                         }
                        if(SNMPPutDataToProcessBuff(0x00,snmpPutData)!= true)
                        {
                            getbulkOverFlowFlag = true;
                            break;
                        }

                        snmpReqVarErrStatus.endOfMibViewErr	 |=(0x0001 << varIndex);

                        //Start counting total number of bytes in this structure.
                        varPairLen.Val = OIDLen  // as put by GetNextVar()
                             + 2     // OID header
                             + 2;    // endOfMibView byte.
                    }
                    else
                    {
                            varPairLen.Val = (temp + 2);        // + OID headerbytes
                    }

                    varBindLen.Val += 4	// Variable Pair STRUCTURE byte + 1 length byte.
                    + varPairLen.Val;

                    prevOffset = snmpPutData->length;
                    snmpPutData->length = varStructLenOffset;
                    if(SNMPPutDataToProcessBuff(varPairLen.v[1],snmpPutData)!=true)
                    {
                        getbulkOverFlowFlag = true;
                        break;
                    }
                    if(SNMPPutDataToProcessBuff(varPairLen.v[0],snmpPutData)!=true)
                    {
                        getbulkOverFlowFlag = true;
                        break;
                    }

                    snmpPutData->length = prevOffset;
                    varStructLenOffset = snmpPutData->length;
                    if((varStructLenOffset - tempTxOffset) > (SNMP_MAX_MSG_SIZE))
                    {
                        getbulkOverFlowFlag = true;
                        break;
                    }
					
						 
				}//for(varBindCntr=0;varBindCntr<Getbulk_R;varBindCntr++)
				
			}//for(repeatCntr=0;repeatCntr<Getbulk_M;repeatCntr++)
			
			break;
		}//end of switch(smSnmp)

		/*If all the variables are processed and the repsonse pdu is updated with 
		  the number of variable responses ought to be in the response; you are done
		  with the request pdu processing. Else continue to processing.*/
		if(Getbulk_N==0u && noOfVarToBeInResponse==0u)
		{	
 			smSnmp=SM_PKT_STRUCT_LEN_OFFSET;
			break;
		}

	}//end of while(1)		


	// Update the place holders with respective values.
		
	
	/* As per RFC 3416 - GET bULK Response - 4.2.3
	If the size of the message encapsulating the Response-PDU containing the 
	requested number of variable bindings would be greater than either a local
	constraint or the maximum message size of the originator, then the response
	is generated with a lesser number of variable bindings. This lesser number is
	the ordered set of variable bindings with some of the variable bindings at the
	end of the set removed, such that the size of the message encapsulating the
	Response-PDU is approximately equal to but no greater than either a local
	constraint or the maximum message size of the originator. Note that the 
	number of variable bindings removed has no relationship to the values of N, M, or R.*/
	if(getbulkOverFlowFlag && (pduDbPtr->pduType==GET_BULK_REQUEST))
	{
		snmpPutData->length = prevOffset;
		varBindLen.Val = varBindLen.Val ;
		UDPSetTxOffset(snmpDcpt[intfIdx].skt, snmpPutData->length);
	}
	
	prevOffset = snmpPutData->length; 
	/* GetRequest-PDU (As per RFC 3416 - SECTION - 4.2.1)
	During the process of any OID,variable binding fails due to invalid OID 
	or invalid OID type or invalid OID length etc, i,e other than "noSuchObject"
	or " noSuchInstance", then the Response-PDU is re-formatted with the same 
	values in its request-id and variable-bindings fields as the received 
    GetRequest-PDU , with the value of its error-status field set to "genErr", 
	
	GetNextRequest-PDU (As per RFC 3416 - SECTION - 4.2.2)
	During the process of any OID,variable binding fails due to invalid OID 
	or invalid OID type or invalid OID length etc, other than "endOfMibView" ,
	then the Response-PDU is re-formatted with the same values in its request-id and 
	variable-bindings fields as the received GetNextRequest-PDU,with the value of 
	its error-status field set to "genErr", and the value of its error-index
	field is set to the index of the failed variable binding. 

	The generated Response-PDU is then encapsulated into a message. If the size of the resultant 
	message is less than or equal to maximum message size of the originator, it is transmitted 
	to the originator of the GetNextRequest-PDU. 

	Otherwise, an alternate Response-PDU is generated. This alternate Response-PDU is formatted 
	with the same values in its request-id field as the received GetNextRequest-PDU, with the value 
	of its error-status field set to "tooBig", the value of its error-index field set to zero, and an empty 
	variable-bindings field.
	
	*/
	//calculate the number of bytes are the part of RESPONSE PDU 
	if(bSnmpGenError)
    {   
    	if(((prevOffset - tempTxOffset) > SNMP_MAX_MSG_SIZE) 
    		&& (pduDbPtr->pduType!=GET_BULK_REQUEST))
    	{
    		/* for snmpv2 (or snmpv3) by rfc3416 we return special
    		*   tooBig(1) response with empty variable-bindings field. 
    		* error status  = toobig(1) and error_index set to 0.
    		*/
    		
    		SNMPSetErrorStatus(errorStatusOffset,errorIndexOffset,SNMP_TOO_BIG,0,snmpPutData);
    		varBindLen.Val =  6 						// Request ID bytes (4+2)
    						+ 3 						// Error status 	(2+1)
    						+ 3;						// Error index		(2+1)
    		snmpPutData->length = pduLenOffset; 
    		SNMPPutDataToProcessBuff(varBindLen.v[1],snmpPutData);
    		SNMPPutDataToProcessBuff(varBindLen.v[0],snmpPutData);
    	
    		// varBindLen is reused as "packetLen".
    		varBindLen.Val = 3						// SNMP Version bytes
    						+ 2 + communityLen		// community string bytes
    						+ 4 					// PDU structure header bytes.
    						+ varBindLen.Val;
    	
    		snmpPutData->length  = packetStructLenOffset;
    		SNMPPutDataToProcessBuff(varBindLen.v[1],snmpPutData);
    		SNMPPutDataToProcessBuff(varBindLen.v[0],snmpPutData);
    		snmpPutData->length = varBindStructOffset-2;
    		UDPSetTxOffset(snmpDcpt[intfIdx].skt, varBindStructOffset-2);
            bSnmpSendPkt =  true;
    		
            SNMPPutDataArrayToUDPTxBuffer(snmpDcpt[intfIdx].skt,snmpPutData);
    		smSnmp = SM_PKT_STRUCT_LEN_OFFSET;
    		return true;
    	}    	
     }
	snmpPutData->length = varBindStructOffset;
	SNMPPutDataToProcessBuff(varBindLen.v[1],snmpPutData);
	SNMPPutDataToProcessBuff(varBindLen.v[0],snmpPutData);
	snmpPutData->length = prevOffset;
	

    // varBindLen is reused as "pduLen"
    varBindLen.Val = varBindLen.Val+4       // Variable Binding Strucure length
                + 6                         // Request ID bytes (4+2)
                + 3                         // Error status		(2+1)
                + 3;                        // Error index		(2+1)	
	prevOffset = snmpPutData->length; 

	snmpPutData->length = pduLenOffset;
	 SNMPPutDataToProcessBuff(varBindLen.v[1],snmpPutData);
	 SNMPPutDataToProcessBuff(varBindLen.v[0],snmpPutData);
	 snmpPutData->length = prevOffset;
	 

#ifdef TCPIP_STACK_USE_SNMPV3_SERVER	
	if(pduDbPtr->snmpVersion == SNMP_V3)
	{
		prevOffset = snmpPutData->length;

		snmpPutData->length = msgSecrtyParamLenOffset;
		SNMPPutDataToProcessBuff(varBindLen.v[0]+tempByteCntr.v[0],snmpPutData);
		snmpPutData->length = prevOffset;
	}
#endif

	// Update the place holders with respective values.
	if(pduDbPtr->snmpVersion != SNMP_V3)
	{
    	// varBindLen is reused as "packetLen".
    	varBindLen.Val = 3                      // SNMP Version bytes
                    + 2 + communityLen      // community string bytes
                    + 4                     // PDU structure header bytes.
                    + varBindLen.Val;
	}
#ifdef TCPIP_STACK_USE_SNMPV3_SERVER		
	else
	{
		varBindLen.Val = 3       // SNMP Version bytes
		 + 2                     // PDU structure header bytes.
         + varBindLen.Val
		 +bytesAdded2Pdu.Val;
			
	}
#endif

	prevOffset = snmpPutData->length; 

	snmpPutData->length = packetStructLenOffset;
	SNMPPutDataToProcessBuff(varBindLen.v[1],snmpPutData);
	SNMPPutDataToProcessBuff(varBindLen.v[0],snmpPutData);

	snmpPutData->length = prevOffset;

	SNMPPutDataArrayToUDPTxBuffer(snmpDcpt[intfIdx].skt,snmpPutData);
	
	smSnmp = SM_PKT_STRUCT_LEN_OFFSET;

     return true;
	
}

/****************************************************************************
  Function:
	uint8_t SNMPProcessGetNextVar(OID_INFO* rec,PDU_INFO* pduDbPtr)
	
  Summary:
  	Retrieves next node from the MIB database.  
  	
  Description:
  	This routine reads into the MIB stored with the agent .
  	It will search for the first lexicographic successor of the variable 
  	binding's name in the incoming GetNextRequest-PDU. If found, the 
  	corresponding variable binding's name and value fields in the Response 
  	pdu are set to the name and value of the located variable. If the 
  	lexicographic succesor is not found, the vlaue filed is set to
  	"endofMibView" and name field is retained as in request.
	  	
  Precondition:
	SNMPProcessVariables() is called.
	
  Parameters:
  	rec - Pointer to SNMP MIB object information for which next node 
  		  to be found
  	
  Return Values:
	temp.V[0]- Total number of bytes copied to response packet if succesful.	
	false	 - If End of MIB is reached or processing is failure.
	
  Remarks:
	None.
 ***************************************************************************/
uint8_t SNMPProcessGetNextVar(OID_INFO* rec,PDU_INFO* pduDbPtr)
{
    TCPIP_UINT16_VAL temp;
    uint8_t putBytes=0;
    OID_INFO indexRec;
    uint8_t *pOIDValue;
    uint8_t OIDValue[OID_MAX_LEN];
    uint8_t OIDLen;
    SNMP_INDEX_INFO indexInfo;
    MIB_INFO varNodeInfo;
    SNMP_ID varID;
    uint16_t OIDValOffset=0;
    uint16_t prevOffset;
    static uint8_t varDataType;
    static uint8_t indexBytes;
    uint8_t idLen = 1;
    uint8_t	dummyRead;

    SNMPBUFFERDATA *snmpPutData =  NULL;

#ifdef TCPIP_STACK_USE_SNMPV3_SERVER	
    SNMPV3MSGDATA	*dynPduBuf=NULL;

    SNMPV3_PROCESSING_MEM_INFO_PTRS snmpv3PktProcessingMemPntr;
    SNMPV3_STACK_DCPT_STUB * snmpv3EngnDcptMemoryStubPtr=0;

    SNMPv3GetPktProcessingDynMemStubPtrs(&snmpv3PktProcessingMemPntr);

    snmpv3EngnDcptMemoryStubPtr=snmpv3PktProcessingMemPntr.snmpv3StkProcessingDynMemStubPtr;
    dynPduBuf = &snmpv3EngnDcptMemoryStubPtr->ScopedPduRespnsBuf;
#endif

    snmpPutData = &SnmpStackDcptMemStubPtr->outPduBufData;
    temp.v[0] = 0;

    // Get next leaf only if this OID is a parent or a simple leaf node.
    if ( rec->nodeInfo.Flags.bIsParent ||
       (!rec->nodeInfo.Flags.bIsParent && !rec->nodeInfo.Flags.bIsSequence) )
    {
        if ( !SNMPGetNextLeaf(rec))
            return false;
    }

    // Get complete OID string from oid record.
    if ( !SNMPGetOIDStringByAddr(rec, OIDValue, &OIDLen))
    {
        return false;
    }
    
    //to validate the REC ID is present or not
    // do while loop till find find a valid entry.
    while(1)
    {
        if(SNMPIdRecrdValidation(pduDbPtr,rec,OIDValue,OIDLen) != true)
        {
            if(!SNMPGetNextLeaf(rec))
                    return false;
            else
            {
                // Get complete OID string from oid record.
                if ( !SNMPGetOIDStringByAddr(rec, OIDValue, &OIDLen))
                {
                    return false;
                }
            }
        }
        else
        {
            break;
        }
    }
    while(1)
    {
        if(!rec->nodeInfo.Flags.bIsSequence)
            break;
        // Need to fetch index information from MIB and prepare complete OID+
        // index response.
        varNodeInfo.Val = rec->nodeInfo.Val;
    
        // In this version, only 7-bit index is supported.
        SYS_FS_read(snmpFileDescrptr,&dummyRead,1);
        indexBytes = 0;
    
        SYS_FS_read(snmpFileDescrptr, &indexInfo.Val,1);
    
        // Fetch index ID.
        SYS_FS_read(snmpFileDescrptr, &idLen,1);
        
        if(idLen == 1)
        {
            uint8_t temp;
            SYS_FS_read(snmpFileDescrptr,(uint8_t*)&temp,1);
            
            indexRec.id = temp & 0xFF;
        }
        else if(idLen == 2)
        {
            uint8_t temp[2];
            SYS_FS_read(snmpFileDescrptr,temp,2);
            indexRec.id = 0;
            indexRec.id = temp[0] & 0xFF;
            indexRec.id <<= 8;
            indexRec.id |= temp[1] & 0xFF;
        }
    
        // Fetch index data type.
        indexRec.dataType = 0;
        SYS_FS_read(snmpFileDescrptr,(uint8_t*)&indexRec.dataType,1);
    
        indexRec.index = rec->index;
    
        // Check with application to see if there exists next index
        // for this index id.
        if (!SNMPGetNextIndex(indexRec.id, &indexRec.index))
        {
            if ( !SNMPGetNextLeaf(rec))
                return false;
            
            if (!SNMPGetOIDStringByAddr(rec, OIDValue, &OIDLen))
            {   
                return false;
            }
            if(SNMPIdRecrdValidation(pduDbPtr,rec,OIDValue,OIDLen) != true)
            continue;
        }
        else
        {
            break;
        }
    }

    // Copy complete OID string to create response packet.
    pOIDValue = OIDValue;	
    temp.v[0] = OIDLen;
    if(pduDbPtr->snmpVersion != SNMP_V3)
    {
        OIDValOffset = snmpPutData->length;
        //temp.v[0] = OIDLen;
        snmpPutData->length = OIDValOffset+1;
        while( temp.v[0]-- )
            SNMPPutDataToProcessBuff(*pOIDValue++,snmpPutData);
    }
#ifdef TCPIP_STACK_USE_SNMPV3_SERVER	
    else
    {
        OIDValOffset = dynPduBuf->length;
        //temp.v[0] = OIDLen;
        //dynPduBuf.length = OIDValOffset+1;	// offset for the OID length
        if(SNMPv3CopyDataToProcessBuff(0,dynPduBuf)!= true)
            return false;
        while( temp.v[0]-- )
        if(SNMPv3CopyDataToProcessBuff(*pOIDValue++,dynPduBuf) != true)
            return false;
    }
#endif

	//Put OID

    // Start counting number of bytes put - OIDLen is already counted.
    temp.v[0] = OIDLen;

    varDataType = rec->dataType;
    varID = rec->id;

    // If this is a simple OID, handle it as a GetVar command.
    if(!rec->nodeInfo.Flags.bIsSequence)
    {
    	if(pduDbPtr->snmpVersion != SNMP_V3)
        {
            // This is an addition to previously copied OID string.
            // This is index value of '0'.
            SNMPPutDataToProcessBuff(0,snmpPutData);
            temp.v[0]++;

            // Since we added one more byte to previously copied OID
            // string, we need to update OIDLen value.

            prevOffset = snmpPutData->length;
            snmpPutData->length = OIDValOffset;
            SNMPPutDataToProcessBuff(++OIDLen,snmpPutData);
            snmpPutData->length = prevOffset;

            // Now do Get on this simple variable.
            prevOffset = snmpPutData->length;
            putBytes = SNMPProcessGetVar(rec, false,pduDbPtr);
            if ( putBytes == 0u )
            {
                snmpPutData->length = prevOffset;
                SNMPPutDataToProcessBuff(ASN_NULL,snmpPutData);
                SNMPPutDataToProcessBuff(0,snmpPutData);
                putBytes = 2;
            }
        }
#ifdef TCPIP_STACK_USE_SNMPV3_SERVER	
        else
        {
            // This is index value of '0'.
            if(SNMPv3CopyDataToProcessBuff(0,dynPduBuf) != true)
                return false;
            temp.v[0]++;
            prevOffset = dynPduBuf->length;
            dynPduBuf->length = OIDValOffset;
            // Since we added one more byte to previously copied OID
            // string, we need to update OIDLen value.
            if(SNMPv3CopyDataToProcessBuff(++OIDLen,dynPduBuf)!= true)
                return false;
            dynPduBuf->length = prevOffset;

            // Now do Get on this simple variable.
            prevOffset = dynPduBuf->length;
            putBytes = SNMPProcessGetVar(rec, false,pduDbPtr);
            if(dynPduBuf->length >= dynPduBuf->maxlength)
                    return false;
            if(( putBytes == 0u ) && (pduDbPtr->snmpVersion == SNMP_V3))
            {
                dynPduBuf->length = prevOffset;
                if(SNMPv3CopyDataToProcessBuff(ASN_NULL,dynPduBuf)!= true)
                        return false;
                if(SNMPv3CopyDataToProcessBuff(0,dynPduBuf)!= true)
                        return false;
                putBytes = 2;
            }
        }
#endif
        temp.v[0] += putBytes; // SNMPProcessGetVar(rec, false,pduDbPtr);

         // Return with total number of bytes copied to response packet.
        return temp.v[0];
    }

    // Index is assumed to be dynamic, and leaf node.
    // mib2bib has already ensured that this was the case.
    indexRec.nodeInfo.Flags.bIsConstant = 0;
    indexRec.nodeInfo.Flags.bIsParent = 0;
    indexRec.nodeInfo.Flags.bIsSequence = 1;

    // Now handle this as simple GetVar.
    // Keep track of number of bytes added to OID.
    indexBytes += SNMPProcessGetVar(&indexRec, true,pduDbPtr);

    rec->index = indexRec.index;

    // These are the total number of bytes put so far as a result of this function.
    temp.v[0] += indexBytes;

    // These are the total number of bytes in OID string including index bytes.
    OIDLen += indexBytes;

	if(pduDbPtr->snmpVersion != SNMP_V3)
	{
	    // Since we added index bytes to previously copied OID
	    // string, we need to update OIDLen value.
	    prevOffset = snmpPutData->length ;		
		snmpPutData->length = OIDValOffset;
		
	    SNMPPutDataToProcessBuff(OIDLen,snmpPutData);
	    snmpPutData->length = prevOffset;
		
	}
#ifdef TCPIP_STACK_USE_SNMPV3_SERVER	
	else
	{
	    // Since we added index bytes to previously copied OID
	    // string, we need to update OIDLen value.
	    prevOffset = dynPduBuf->length;
	    dynPduBuf->length = OIDValOffset;
	    SNMPv3CopyDataToProcessBuff(OIDLen,dynPduBuf);
		dynPduBuf->length = prevOffset;
	}
#endif

    // Fetch actual value itself.
    // Need to restore original OID value.
    rec->nodeInfo.Val = varNodeInfo.Val;
    rec->id = varID;
    rec->dataType = varDataType;

    temp.v[0] += SNMPProcessGetVar(rec, false,pduDbPtr);
     return temp.v[0];
}


/****************************************************************************
  Function:
	uint8_t SNMPProcessGetBulkVar(OID_INFO* rec, uint8_t* oidValuePtr, 
						   uint8_t* oidLenPtr,uint8_t* successor)	

  Summary:
  	This routine process the SNMPv2c Get Bulk Request.
  	
  Description:
	SNMPProcessVariables() processes the received snmp request pdu for each of
	the variable binding in the variable binding list to produce a response 
	pdu. Depending on the number of the Max_repetitions for every variable
	in the list for which Bulk information is expected, SNMPProcessGetBulkVar()
	is executed. It searches for the next lexicographically ordered 
	successor for of the OID received in the request. For each of the 
	iterations upto max-repetitions, the next leaf node is searched in the
	MIB to that of the leaf node found in the last iteration, for the 
	corresponding variable binding.
 		  	
  Precondition:
	SNMPProcessVariables() is called.
	
  Parameters:
  	rec 		- Pointer to SNMP MIB variable object information OID 
  	oidValuePtr	- Pointer to new node OID found in MIB 
  	oidLenPtr	- Oid length
  	successor	- 'I'th lexicographic successor to be found value
  	  	
  Return Values:
	false 	  - If no lexicographic successor found
	temp.v[0] - Total number of bytes copied to response packet
	
  Remarks:
	None.
***************************************************************************/
uint8_t SNMPProcessGetBulkVar(OID_INFO* rec, uint8_t* oidValuePtr, uint8_t* oidLenPtr,uint8_t* successor,PDU_INFO* pduDbPtr)
{
    
    //uint8_t ref;
    uint8_t putBytes,cntr;
    uint8_t OIDLen;
    static uint8_t varDataType;
    static uint8_t indexBytes;
    uint8_t sequenceCnt=0;
    uint8_t sequenceRepeatCnt=0;
    bool lbNextLeaf;
    SNMP_ID varID;
    OID_INFO indexRec;
    SNMP_INDEX_INFO indexInfo;
    MIB_INFO varNodeInfo;
    uint16_t OIDValOffset;
    uint16_t varBindOffset=0;
    uint16_t prevOffset;
    TCPIP_UINT16_VAL temp;
    //static SNMP_VAL v;
    uint8_t idLen=1;
    uint8_t dummyRead;
	
	
#ifdef TCPIP_STACK_USE_SNMPV3_SERVER	
    SNMPV3MSGDATA	*dynPduBuf=NULL;
    SNMPV3_PROCESSING_MEM_INFO_PTRS snmpv3PktProcessingMemPntr;
    SNMPV3_STACK_DCPT_STUB * snmpv3EngnDcptMemoryStubPtr=0;

    SNMPv3GetPktProcessingDynMemStubPtrs(&snmpv3PktProcessingMemPntr);

    snmpv3EngnDcptMemoryStubPtr=snmpv3PktProcessingMemPntr.snmpv3StkProcessingDynMemStubPtr;
    dynPduBuf = &snmpv3EngnDcptMemoryStubPtr->ScopedPduRespnsBuf;
#endif

    SNMPBUFFERDATA *snmpPutData =  NULL;
    snmpPutData = &SnmpStackDcptMemStubPtr->outPduBufData;

    /* intialize the local variables to 0 */
    OIDLen=0;
    sequenceCnt=0;
    sequenceRepeatCnt=0;
    varID=0;
    OIDValOffset=0;
    varBindOffset=0;
    prevOffset=0;
    temp.Val=0;

    lbNextLeaf = false;
    temp.v[0] = 0;
    sequenceRepeatCnt=*successor;

    //Reach to the node for the expected iteration
    for(cntr=0;cntr<=*successor;cntr++)
    {
    // Get next leaf only if this OID is a parent or a simple leaf node.
        if((rec->nodeInfo.Flags.bIsParent)||
        (!rec->nodeInfo.Flags.bIsParent && !rec->nodeInfo.Flags.bIsSequence))
        {	/* to maintain the number of interations */
            sequenceCnt++;
            if(!SNMPGetNextLeaf(rec))
                return false;
        }
    }

    /* If request OID is a sequence variable, the below for loop retrives the
    expected instance for the OID. SequenceRepeatCnt starts with "0th instance" and
    increments to Max repeatations. Find the exact indexed OID in the request at first.
    If indexed OID is not available, then go for the next index.
    If the next index is not available , then go to the next leaf.
    */
    for(;sequenceCnt<=sequenceRepeatCnt;sequenceCnt++)
    {
        if(rec->nodeInfo.Flags.bIsSequence)
        {
            SNMPGetExactIndex(rec->id,&rec->index);
            if(!SNMPGetNextIndex(rec->id,&rec->index))
            {
                if(!SNMPGetNextLeaf(rec))
                    return false;
            }
        }
        else
        {
            if(!SNMPGetNextLeaf(rec))
                return false;
        }
    }

    // Get complete OID string from oid record.
    if(!SNMPGetOIDStringByAddr(rec, oidValuePtr, &OIDLen))
        return false;

    //to validate the REC ID is present or not
    while(1)
    {
        if(SNMPIdRecrdValidation(pduDbPtr,rec,oidValuePtr,OIDLen) != true)
        {
            if(!SNMPGetNextLeaf(rec))
                    return false;
            else
            {
                    // Get complete OID string from oid record.
                if(!SNMPGetOIDStringByAddr(rec, oidValuePtr, &OIDLen))
                    return false;
            }
        }
        else
        {
            break;
        }
    }
    // get exact index value when it is a sequence variable
    while(1)
    {
        if(!rec->nodeInfo.Flags.bIsSequence)
            break;
        // Need to fetch index information from MIB and prepare complete OID+
        // index response.
        varNodeInfo.Val = rec->nodeInfo.Val;

        // In this version, only 7-bit index is supported.
        SYS_FS_read(snmpFileDescrptr,&dummyRead,1);

        indexBytes = 0;
        // Fetch index ID.
        SYS_FS_read(snmpFileDescrptr,&indexInfo.Val,1);

        // Fetch index ID.
        SYS_FS_read(snmpFileDescrptr,&idLen,1);

        if(idLen == 1)
        {
            uint8_t temp;
    	    SYS_FS_read(snmpFileDescrptr,&temp,1);

    	    indexRec.id = temp & 0xFF;
        }
        else if(idLen == 2)
        {
            uint8_t temp[2];
    	    SYS_FS_read(snmpFileDescrptr,temp,2);
            indexRec.id = 0;
            indexRec.id = temp[0] & 0xFF;
            indexRec.id <<= 8;
            indexRec.id |= temp[1] & 0xFF;
        }
        // Fetch index data type.
        indexRec.dataType = 0;

        SYS_FS_read(snmpFileDescrptr,(uint8_t*)&indexRec.dataType,1);

        indexRec.index = rec->index;
         // Check with application to see if there exists next index
        // for this index id.
        if (!SNMPGetExactIndex(indexRec.id, &indexRec.index))
        {
            if ( !SNMPGetNextLeaf(rec))
                return false;
            
            if (!SNMPGetOIDStringByAddr(rec, oidValuePtr,&OIDLen))
            {   
                return false;
            }
            if(SNMPIdRecrdValidation(pduDbPtr,rec,oidValuePtr,OIDLen) != true)
            continue;
        }
        else
        {
            break;
        }
    }
	
#ifdef TCPIP_STACK_USE_SNMPV3_SERVER	
    if(pduDbPtr->snmpVersion == SNMP_V3)
    {
        varBindOffset = dynPduBuf->length;
        if(SNMPv3CopyDataToProcessBuff(ASN_OID,dynPduBuf) != true)
                return false;

        OIDValOffset = dynPduBuf->length;
        temp.v[0] = OIDLen;
        if(SNMPv3CopyDataToProcessBuff(0,dynPduBuf) != true)
                return false;
        //Put OID
        while( temp.v[0]-- )
        {
            if(SNMPv3CopyDataToProcessBuff(*oidValuePtr,dynPduBuf) != true)
                return false;
            oidValuePtr++;
        }
    }
    else
#endif
    {
        varBindOffset = snmpPutData->length;
        if(SNMPPutDataToProcessBuff(ASN_OID,snmpPutData)!= true)
                return false;

        OIDValOffset = snmpPutData->length;
        temp.v[0] = OIDLen;
        snmpPutData->length = OIDValOffset+1;
        //Put OID
        while( temp.v[0]-- )
        {
            if(SNMPPutDataToProcessBuff(*oidValuePtr,snmpPutData)!= true)
                    return false;
            oidValuePtr++;
        }
    }
	// Start counting number of bytes put - OIDLen is already counted.
    temp.v[0] =*oidLenPtr= OIDLen;

    varDataType = rec->dataType;
    varID = rec->id;

    // If this is a simple OID, handle it as a GetVar command.
    if (!rec->nodeInfo.Flags.bIsSequence)
    {
#ifdef TCPIP_STACK_USE_SNMPV3_SERVER	
    	if(pduDbPtr->snmpVersion == SNMP_V3)
    	{
            // This is an addition to previously copied OID string.
            // This is index value of '0'.
            if(SNMPv3CopyDataToProcessBuff(0,dynPduBuf) != true)
                return false;
            temp.v[0]++;

            // Since we added one more byte to previously copied OID
            // string, we need to update OIDLen value.
            prevOffset = dynPduBuf->length;
            dynPduBuf->length = OIDValOffset;
            if(SNMPv3CopyDataToProcessBuff(++OIDLen,dynPduBuf) != true)
                    return false;
            dynPduBuf->length = prevOffset;


            // Now do Get on this simple variable.
            prevOffset = dynPduBuf->length;
            putBytes = SNMPProcessGetVar(rec, false,pduDbPtr);
    	}
	else
#endif
        {
             // This is an addition to previously copied OID string.
            // This is index value of '0'.
            if(SNMPPutDataToProcessBuff(0,snmpPutData)!= true)
                    return false;
            temp.v[0]++;

            // Since we added one more byte to previously copied OID
            // string, we need to update OIDLen value.

            prevOffset = snmpPutData->length;
            snmpPutData->length = OIDValOffset;

            if(SNMPPutDataToProcessBuff(++OIDLen,snmpPutData)!= true)
                    return false;
            snmpPutData->length = prevOffset;
            // Now do Get on this simple variable.
            prevOffset = snmpPutData->length;
            putBytes = SNMPProcessGetVar(rec, false,pduDbPtr);
            if(putBytes == false)
                    return false;
        }
        temp.v[0] += putBytes; // SNMPProcessGetVar(rec, false,pduDbPtr);

            // Return with total number of bytes copied to response packet.
        return temp.v[0];
    }
    
    // Index is assumed to be dynamic, and leaf node.
    // mib2bib has already ensured that this was the case.
    indexRec.nodeInfo.Flags.bIsConstant = 0;
    indexRec.nodeInfo.Flags.bIsParent = 0;
    indexRec.nodeInfo.Flags.bIsSequence = 1;

    // Now handle this as simple GetVar.
    // Keep track of number of bytes added to OID.
    putBytes = SNMPProcessGetVar(&indexRec, true,pduDbPtr);
    if(putBytes == false)
	    return false;
    indexBytes += putBytes;

    rec->index = indexRec.index;

    // These are the total number of bytes put so far as a result of this function.
    temp.v[0] += indexBytes;

    // These are the total number of bytes in OID string including index bytes.
    OIDLen += indexBytes;

    if(pduDbPtr->snmpVersion != SNMP_V3)
    {
        // Since we added index bytes to previously copied OID
        // string, we need to update OIDLen value.
        prevOffset = snmpPutData->length;
        snmpPutData->length = OIDValOffset;
        SNMPPutDataToProcessBuff(OIDLen,snmpPutData);
        snmpPutData->length = prevOffset;
    }
    #ifdef TCPIP_STACK_USE_SNMPV3_SERVER
    else
    {
        // Since we added index bytes to previously copied OID
        // string, we need to update OIDLen value.
        prevOffset = dynPduBuf->length;
        dynPduBuf->length =OIDValOffset;
        SNMPv3CopyDataToProcessBuff(OIDLen,dynPduBuf);
        dynPduBuf->length = prevOffset;
    }
#endif
    // Fetch actual value itself.
    // Need to restore original OID value.
    rec->nodeInfo.Val = varNodeInfo.Val;
    rec->id = varID;
    rec->dataType = varDataType;

    temp.v[0] += SNMPProcessGetVar(rec, false,pduDbPtr);
	
    return temp.v[0];
}


/****************************************************************************
  Function:
	uint8_t SNMPSearchOIDInMgmtInfoBase(PDU_INFO* pduDbPtr,uint8_t* oid, uint8_t oidLen, OID_INFO* rec)
	
  Summary:
  	To search and validate whether the requested OID is in the MIB database.
  	
  Description:
	The MIB database is stored with the agent in binary mib format.
  	This is the binary mib format:
  	<oid, nodeInfo, [id], [SiblingOffset], [DistantSibling], [dataType],
  	[dataLen], [data], [{IndexCount, <IndexType>, <Index>, ...>]}, ChildNode
	variable bind name is a dotted string of oid. Every oid is a node in the
	MIB tree and have varied information. This routine on reception of the 
	snmp request, will search for every oid in the var name. This routine 
	will return information whether the requested var name is part of the 
	MIB tree data structre of this agent or not.
	 		  	
  Precondition:
	Valid snmp request with valid OID format is received.
	
  Parameters:
  	pduDbPtr	- Pointer to received snmp  pdu elements information 
  	oid			- Pointer to the string of OID to be searched
  	oidLen		- Oid length
  	rec			- Pointer to SNMP MIB variable object information  
  	  	
  Return Values:
	true	-	If the complete OID string is found in the mib
	false	-   If complete OID do not match. 
				Also different erros returned are
				SNMP_END_OF_MIB_VIEW
				SNMP_NO_SUCH_NAME
				SNMP_NO_SUCH_OBJ
				SNMP_NO_SUCH_INSTANCE
  Remarks:
	This routine works for the snmp mib storage format. It uses the file system
	APIs to read,search and collect information from the mib database. 
***************************************************************************/
uint8_t SNMPSearchOIDInMgmtInfoBase(PDU_INFO* pduDbPtr,uint8_t* oid, uint8_t oidLen, OID_INFO* rec)
{
	uint8_t idLen=1;
	uint8_t savedOID,tempSavedOID;
	uint8_t matchedCount;
	uint8_t snmpVer;
	uint8_t snmpReqType;
	uint8_t tempLen;
	uint8_t* reqOidPtr;
	uint8_t comapreOidWithSibling=false;

	TCPIP_UINT16_VAL tempData;
    uint32_t hNode,tempHNode;//    FS hNode;
	bool  bFoundIt=false;

	SnmpStackDcptMemStubPtr->appendZeroToOID=true;

	snmpVer=pduDbPtr->snmpVersion;
	snmpReqType=pduDbPtr->pduType;

    if(!SNMPStatus.Flags.bIsFileOpen )
	   return false;
	
	hNode = 0;
    matchedCount = oidLen;

	tempLen=oidLen;
	reqOidPtr=oid;

    while( 1 )
    {
    
		SYS_FS_lseek(snmpFileDescrptr, hNode, SEEK_SET);

        // Remember offset of this node so that we can find its sibling
        // and child data.

        rec->hNode = SYS_FS_lseek(snmpFileDescrptr,0,SEEK_CUR); // hNode;

        // Read OID byte.
		SYS_FS_read(snmpFileDescrptr,&savedOID,1);

		if(comapreOidWithSibling==(uint8_t)false)
		{
		   	tempSavedOID=savedOID;
			tempHNode=hNode;
		}

        // Read Node Info
		SYS_FS_read(snmpFileDescrptr,&rec->nodeInfo.Val,1);

	    // Next byte will be node id, if this is a leaf node with variable data.
        if(rec->nodeInfo.Flags.bIsIDPresent)
        {
			SYS_FS_read(snmpFileDescrptr,&idLen,1);
			
			if(idLen == 1)
			{
             	uint8_t temp;
				SYS_FS_read(snmpFileDescrptr,&temp,1);
							
                rec->id = temp & 0xFF;
                
			}
			else if(idLen == 2)
			{
				uint8_t temp[2];
				SYS_FS_read(snmpFileDescrptr,temp,2);
				rec->id = 0;
				rec->id = temp[0] & 0xFF;
				rec->id <<= 8;
				rec->id |= temp[1] & 0xFF;				
			}				
        }
	
        // Read sibling offset, if there is any.
        if(rec->nodeInfo.Flags.bIsSibling)
        {
			SYS_FS_read(snmpFileDescrptr,&tempData.v[0],1);
			SYS_FS_read(snmpFileDescrptr,&tempData.v[1],1);
            rec->hSibling = tempData.Val;
        }

        if ( savedOID != *reqOidPtr )
        {
        	/*if very first OID byte does not match, it may be because it is
              0, 1 or 2.  In that case declare that there is a match.
              The command processor would detect OID type and continue or reject
              this OID as a valid argument.*/
            if(matchedCount == oidLen)
            {
                bFoundIt =  true;
                break;
            }
			
			if(comapreOidWithSibling==(uint8_t)true && !rec->nodeInfo.Flags.bIsSibling)
            {
                bFoundIt =  false;
                break;
            }

			if ( rec->nodeInfo.Flags.bIsSibling )
            {
				SYS_FS_lseek(snmpFileDescrptr, tempData.Val, SEEK_SET);
	
				 hNode = SYS_FS_lseek(snmpFileDescrptr,0,SEEK_CUR);
				comapreOidWithSibling=true;
            }
            else
            {
                bFoundIt =  false;
                break;
            }
        }
        else
        {
	        // One more oid byte matched.
            matchedCount--;
            reqOidPtr++;

            // A node is said to be matched if last matched node is a leaf node
            // or all but last OID string is matched and last byte of OID is '0'.
            // i.e. single index.
            if ( !rec->nodeInfo.Flags.bIsParent )
            {
            	// Read and discard Distant Sibling info if there is any.
                if ( rec->nodeInfo.Flags.bIsDistantSibling )
                {	
					
					SYS_FS_read(snmpFileDescrptr,&tempData.v[0],1);

					SYS_FS_read(snmpFileDescrptr,&tempData.v[1],1);

					rec->hSibling = tempData.Val;
					
                }

		        rec->dataType = 0;

				SYS_FS_read(snmpFileDescrptr,(uint8_t*)&rec->dataType,1);

			    rec->hData = SYS_FS_lseek(snmpFileDescrptr,0,SEEK_CUR);

				if(snmpReqType==SNMP_GET && matchedCount == 0u)
				{
					SnmpStackDcptMemStubPtr->appendZeroToOID=false;
                    bFoundIt =  false;
                    break;
                 }
				else if(snmpReqType==(uint8_t)SNMP_GET 
					&& matchedCount == 1u && *reqOidPtr == 0x00u)
				{
					SnmpStackDcptMemStubPtr->appendZeroToOID=false;
                 }
				else if(snmpReqType==SNMP_GET_NEXT && matchedCount == 0u)
				{
					SnmpStackDcptMemStubPtr->appendZeroToOID=true;
					SnmpStackDcptMemStubPtr->getZeroInstance=true;
				}
				else if(snmpReqType==(uint8_t)SNMP_V2C_GET_BULK && matchedCount == 1u )
				{
					SnmpStackDcptMemStubPtr->appendZeroToOID=false;
				}
                bFoundIt =  true;
                break;
            }
            else if(matchedCount == 1u && *reqOidPtr == 0x00u)
	        {
	            SnmpStackDcptMemStubPtr->appendZeroToOID=false;

				if(rec->nodeInfo.Flags.bIsParent)
                {
                    bFoundIt =  false;
                    break;
                }

			}
            else if(matchedCount == 0u)
            {
	           	if(rec->nodeInfo.Flags.bIsParent || snmpReqType==SNMP_GET)
    			{
					SnmpStackDcptMemStubPtr->appendZeroToOID=false;
                    bFoundIt =  false;
                    break;
                 }
				 else 
                 {
                     bFoundIt =  true;
                     break;
                 }
			}
            else
            {	
				hNode = SYS_FS_lseek(snmpFileDescrptr,0,SEEK_CUR);

				// Try to match following child node.
                continue;
            }
        }
    }

    if(bFoundIt == true)
    {
    	// Convert index info from OID to regular value format.
      	rec->index = savedOID;

    	/*To Reach To The Next leaf Node */
        savedOID = *reqOidPtr;
    	
    	rec->indexLen = 1;

    	if(matchedCount ==1u)
    	{
    		rec->index = *reqOidPtr;
    	}
    	else if(matchedCount == 0u)
    	{
    		rec->index = 0;
    	}
    	else if ( matchedCount > 1u || savedOID & 0x80 /*In this version, we only support 7-bit index*/)
        {	
        	// Current instnace spans across more than 7-bit.
            rec->indexLen = 0xff;

    		if(snmpReqType==SNMP_GET && snmpVer==(uint8_t)SNMP_V1)
    		{
    			return SNMP_NO_SUCH_NAME;
    		}
    		else if(snmpReqType==SNMP_GET && snmpVer==(uint8_t)SNMP_V2C)
    		{
    			if(matchedCount== oidLen) //No OBJECT IDNETIFIER Prefix match
    				return SNMP_NO_SUCH_INSTANCE;
    			else
    				return SNMP_NO_SUCH_OBJ;
    		}

    		return false;
        }
        
        if(SnmpStackDcptMemStubPtr->getZeroInstance)
	    {
		   rec->index = SNMP_INDEX_INVALID;		
        }

    	return true;
    }
    else
    {

    	if(snmpReqType==SNMP_GET)
    	{
    		if(snmpVer==(uint8_t)SNMP_V1)
    			return SNMP_NO_SUCH_NAME;

    		else if(snmpVer==(uint8_t)SNMP_V2C)
    		{	
    			if(matchedCount== oidLen) //No OBJECT IDNETIFIER Prefix match
    				return SNMP_NO_SUCH_INSTANCE;
    			else
    				return SNMP_NO_SUCH_OBJ;
    		}
    	}
    	else if((snmpReqType==SNMP_GET_NEXT||snmpReqType==SNMP_V2C_GET_BULK) && snmpVer==(uint8_t)SNMP_V2C)
    	{
    		return SNMP_END_OF_MIB_VIEW;
    	}
    	
    }
    
    return false;
}	


/****************************************************************************
  Function:
	bool SNMPGetNextLeaf(OID_INFO* rec)
	
  Summary:
  	Searches for the next leaf node in the MIP tree.  	

  Description:
  	This routine searches for the next leaf node from the current node.
  	The input to this function is the node from where next leaf node
  	is to be located. The next leaf node will be a silbing else distant 
  	sibling or leaf node of next branch, if any present. The input parameter
  	var pointer will be updated with the newly found leaf node OID info.
 		 		  	
  Precondition:
	SNMPProcessGetBulkVar() else SNMPProcessGetNextVar() is called. 
	
  Parameters:
  	rec		- Pointer to SNMP MIB variable object information  

  Return Values:
	true	- If next leaf node is found.
	false	- There is no next leaf node.
	
  Remarks:
	None.
***************************************************************************/
bool SNMPGetNextLeaf(OID_INFO* rec)
{
    TCPIP_UINT16_VAL temp;
	uint8_t idLen=1; 	 

    // If current node is leaf, its next sibling (near or distant) is the next leaf.
    if ( !rec->nodeInfo.Flags.bIsParent )
    {
        // Since this is a leaf node, it must have at least one distant or near
        // sibling to get next sibling.
        if(rec->nodeInfo.Flags.bIsSibling ||
           rec->nodeInfo.Flags.bIsDistantSibling )
        {
            // Reposition at sibling.
            SYS_FS_lseek(snmpFileDescrptr, rec->hSibling, SEEK_SET);

            // Fetch node related information
        }
        // There is no sibling to this leaf.  This must be the very last node on the tree.
        else
        {
            //--SYS_FS_close();
            return false;
        }
    }

    while( 1 )
    {
        // Remember current offset for this node.

		rec->hNode = SYS_FS_lseek(snmpFileDescrptr,0,SEEK_CUR);

        // Read OID byte.
		SYS_FS_read(snmpFileDescrptr,&rec->oid,1);

        // Read Node Info
		SYS_FS_read(snmpFileDescrptr,&rec->nodeInfo.Val,1);

        // Next byte will be node id, if this is a leaf node with variable data.
       
        if ( rec->nodeInfo.Flags.bIsIDPresent )
	    {
			 // Fetch index ID.
            SYS_FS_read(snmpFileDescrptr,&idLen,1);
			 
            if(idLen == 1)
            {
                uint8_t temp;

                SYS_FS_read(snmpFileDescrptr,&temp,1);

                rec->id = temp & 0xFF;
            }
            else if(idLen == 2)
            {
                uint8_t temp[2];
                SYS_FS_read(snmpFileDescrptr,temp,2);
                rec->id = 0;
                rec->id = temp[0] & 0xFF;
                rec->id <<= 8;
                rec->id |= temp[1] & 0xFF;
            }
	    }

        // Fetch sibling offset, if there is any.
        if ( rec->nodeInfo.Flags.bIsSibling ||
             rec->nodeInfo.Flags.bIsDistantSibling )
        {
            SYS_FS_read(snmpFileDescrptr,&temp.v[0],1);
            SYS_FS_read(snmpFileDescrptr,&temp.v[1],1);
            rec->hSibling = temp.Val;
        }

        // If we have not reached a leaf yet, continue fetching next child in line.
        if ( rec->nodeInfo.Flags.bIsParent )
        {
            continue;
        }

        // Fetch data type.
        rec->dataType = 0;
        SYS_FS_read(snmpFileDescrptr,(uint8_t*)&rec->dataType,1);

        rec->hData = SYS_FS_lseek(snmpFileDescrptr,0,SEEK_CUR);

        // Since we just found next leaf in line, it will always have zero index
        // to it.
        rec->indexLen = 1;
        rec->index = 0;

        if (rec->nodeInfo.Flags.bIsSequence)
	    {
		   rec->index = SNMP_INDEX_INVALID;		
        }

        return true;
    }
    return false;
}


/****************************************************************************
  Function:
	bool SNMPCheckIfValidCommunityString(char* community, uint8_t* len)
	
  Summary:
  	Verifies for the community string datatype and the max 
  	community name and length, this agent can process.

  Description:
  	This routine populates and validates the community datatype, community
  	name and length from the received snmp request pdu. Community name is
  	used for accessing public and private memebrs of the mib.
  	 		 		  	
  Precondition:
	SNMPProcessHeader() is called. 
	
  Parameters:
  	community -	Pointer to memory where community string will be stored.
  	len		  - Pointer to memory where comunity length gets stored.

  Return Values:
	true	- If valid community received.
	false	- If community is not valid.
	
  Remarks:
	None.
***************************************************************************/
static bool SNMPCheckIfValidCommunityString(char* community, uint8_t* len)
{
    uint8_t tempData;
    uint8_t tempLen;
	uint8_t intfIdx;
	intfIdx = ((TCPIP_NET_IF*)TCPIP_STACK_GetDefaultNet())->netIfIx; 
	
    tempData = SNMPGetByteFromUDPRxBuff(snmpDcpt[intfIdx].skt);
    if ( !IS_OCTET_STRING(tempData) )
        return false;

    tempLen = SNMPGetByteFromUDPRxBuff(snmpDcpt[intfIdx].skt);
    *len    = tempLen;
    if ( tempLen > SNMP_COMMUNITY_MAX_LEN )
        return false;

    while( tempLen-- )
    {
        tempData = SNMPGetByteFromUDPRxBuff(snmpDcpt[intfIdx].skt);
        *community++ = tempData;
    }
    *community = '\0';
	 return true;
}


/****************************************************************************
  Function:
	bool SNMPCheckIfValidInt(uint32_t* val)
	
  Summary:
  	Verifies variable datatype as int and retrieves its value.

  Description:
  	This routine populates and validates the received variable for the
  	data type as "ASN_INT" and the data length for max 4 bytes.
  	 		 		  	
  Precondition:
	SNMPProcessHeader() or SNMPProcessGetSetHeader() is called. 
	
  Parameters:
  	val - Pointer to memory where int var value will be stored.
 
  ReturnValues:
	true	- If valid integer type and value is received.
	false	- Other than integer data type and value received .
	
  Remarks:
	None.
***************************************************************************/
bool SNMPCheckIfValidInt(uint32_t* val)
{
    TCPIP_UINT32_VAL tempData;
    TCPIP_UINT32_VAL tempLen;
	uint8_t intfIdx;
	intfIdx = ((TCPIP_NET_IF*)TCPIP_STACK_GetDefaultNet())->netIfIx; 
	
    tempLen.Val = 0;

    // Get variable type
    if ( !IS_ASN_INT(SNMPGetByteFromUDPRxBuff(snmpDcpt[intfIdx].skt)) )
        return false;

    if ( !SNMPCheckIfValidLength(&tempLen.w[0]) )
        return false;

    // Integer length of more than 32-bit is not supported.
    if ( tempLen.Val > 4u )
        return false;

    tempData.Val = 0;
    while( tempLen.v[0]-- )
        tempData.v[tempLen.v[0]] = SNMPGetByteFromUDPRxBuff(snmpDcpt[intfIdx].skt);

    *val = tempData.Val;

    return true;
}


/****************************************************************************
  Function:
	bool SNMPCheckIfValidPDU(SNMP_ACTION* pdu)
	
  Summary:
  	Verifies for the snmp request type.

  Description:
  	This routine populates and verifies for the received snmp request 
  	pdu type. 
  	 		 		  	
  Precondition:
	SNMPProcessHeader() is called. 
	
  Parameters:
  	val - Pointer to memory where received snmp request type is stored.
 
  Return Values:
	true	- If this snmp request can be processed by the agent. 
	false	- If the request can not be processed.
	
  Remarks:
	None.
***************************************************************************/
bool SNMPCheckIfValidPDU(SNMP_ACTION* pdu)
{
    uint8_t tempData;
    uint16_t tempLen;
	uint8_t intfIdx;
	intfIdx = ((TCPIP_NET_IF*)TCPIP_STACK_GetDefaultNet())->netIfIx; 

    // Fetch pdu data type
    tempData = SNMPGetByteFromUDPRxBuff(snmpDcpt[intfIdx].skt);
    if ( !IS_AGENT_PDU(tempData) )
        return false;

    *pdu = tempData;


	/* Now fetch pdu length.  We don't need to remember pdu length.
	   Do this to proceed to next pdu element of interest*/	
    return SNMPCheckIfValidLength(&tempLen);
}




/****************************************************************************
  Function:
	uint8_t SNMPCheckIfValidLength(uint16_t* len)
	
  Summary:
  	Retrieves the packet length and actual pdu length.

  Description:
	Checks current packet and returns total length value as well as 
	actual length bytes.We do not support any length byte count of more 
	than 2 i.e. total length value must not be more than 16-bit.
  	 		 		  	
  Precondition:
	None 
	
  Parameters:
  	len - Pointer to memory where actual length is stored.
 
  Return Values:
	lengthBytes	- Total length bytes are 0x80 itself plus tempData.
		
  Remarks:
	None.
***************************************************************************/
uint8_t SNMPCheckIfValidLength(uint16_t *len)
{
    uint8_t tempData;
    TCPIP_UINT16_VAL tempLen;
    uint8_t lengthBytes;
	uint8_t intfIdx;
	intfIdx = ((TCPIP_NET_IF*)TCPIP_STACK_GetDefaultNet())->netIfIx; 
    // Initialize length value.
    tempLen.Val = 0;
    lengthBytes = 0;

    tempData = SNMPGetByteFromUDPRxBuff(snmpDcpt[intfIdx].skt);
    tempLen.v[0] = tempData;
    if ( tempData & 0x80 )
    {
        tempData &= 0x7F;

        // We do not support any length byte count of more than 2
        // i.e. total length value must not be more than 16-bit.
        if ( tempData > 2u )
            return false;

        // Total length bytes are 0x80 itself plus tempData.
        lengthBytes = tempData + 1;

        // Get upto 2 bytes of length value.
        while( tempData-- )
            tempLen.v[tempData] = SNMPGetByteFromUDPRxBuff(snmpDcpt[intfIdx].skt);
    }
    else
        lengthBytes = 1;

    *len = tempLen.Val;

    return lengthBytes;
}


/****************************************************************************
  Function:
	bool SNMPCheckIsASNNull(void)

	
  Summary:
  	Verifies the value type as ASN_NULL.

  Description:
  	For Get,Get_Next,Get_Bulk snmp reuest, the var bind the value data type 
  	should be ASN_NULL and value field must be NULL and . This routine
  	verifies the data type and value fields in the received requests.
  	The SET request, the value data type can not be ASN_NULL,
  	otherwise the snmp request is not processed.

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
static bool SNMPCheckIsASNNull(void)
{
	uint8_t a;
	uint8_t intfIdx;
	intfIdx = ((TCPIP_NET_IF*)TCPIP_STACK_GetDefaultNet())->netIfIx; 
		
    // Fetch and verify that this is NULL data type.
    /*if ( !IS_ASN_NULL(SNMPGetByteFromUDPRxBuff()) )
        return false;*/

	a=SNMPGetByteFromUDPRxBuff(snmpDcpt[intfIdx].skt);

	if (!IS_ASN_NULL(a))
			return false;

    // Fetch and verify that length value is zero.
    return (SNMPGetByteFromUDPRxBuff(snmpDcpt[intfIdx].skt) == 0u );
}


/****************************************************************************
  Function:
	bool SNMPCheckIfValidOID(uint8_t* oid, uint8_t* len)
	
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
static bool SNMPCheckIfValidOID(uint8_t* oid, uint8_t* len)
{
    TCPIP_UINT32_VAL tempLen;
	uint8_t intfIdx;
	intfIdx = ((TCPIP_NET_IF*)TCPIP_STACK_GetDefaultNet())->netIfIx; 
	
    // Fetch and verify that this is OID.
    if ( !IS_OID(SNMPGetByteFromUDPRxBuff(snmpDcpt[intfIdx].skt)) )
        return false;

    // Retrieve OID length
    if ( !SNMPCheckIfValidLength(&tempLen.w[0]) )
        return false;

    // Make sure that OID length is within our capability.
    if ( tempLen.w[0] > (uint8_t)OID_MAX_LEN )
        return false;

    *len = tempLen.v[0];

	while( tempLen.v[0]-- )
	{
       *oid++ = SNMPGetByteFromUDPRxBuff(snmpDcpt[intfIdx].skt);
	}
	*oid=0xff;
    return true;
}

/****************************************************************************
  Function:
	uint8_t SNMPCheckIfValidSnmpStructure(uint16_t* dataLen)
	
  Summary:
  	Decode variable length structure.

  Description:
	This routine is used  to verify whether the received varbind is of type
	STRUCTURE and to find out the variable binding structure length.
  	
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
uint8_t SNMPCheckIfValidSnmpStructure(uint16_t* dataLen)
{
    TCPIP_UINT32_VAL tempLen;
    uint8_t headerBytes;
	uint8_t intfIdx;
	intfIdx = ((TCPIP_NET_IF*)TCPIP_STACK_GetDefaultNet())->netIfIx; 

		
    if ( !IS_STRUCTURE(SNMPGetByteFromUDPRxBuff(snmpDcpt[intfIdx].skt)) )
        return false;

    // Retrieve structure length
    headerBytes = SNMPCheckIfValidLength(&tempLen.w[0]);
    if ( !headerBytes )
        return false;

    headerBytes++;

    // Since we are using UDP as our transport and UDP are not fragmented,
    // this structure length cannot be more than 1500 bytes.
    // As a result, we will only use lower uint16_t of length value.
    *dataLen = tempLen.w[0];

    return headerBytes;
}

/****************************************************************************
  Function:
	bool SNMPProcessReqBuildRespnsDuplexInit(UDP_SOCKET socket)
	
  Summary:
  	Prepare for full duplex transfer.

  Description:
	As we process SNMP variables, we will prepare response on-the-fly
    creating full duplex transfer. SNMP stack manages its software simulated full 
    duplex connection. Prepare for full duplex transfer. Set the Tx and Rx 
    offset to start of the buffer.
  	
  Precondition:
	SNMPTask() is called.	
	
  Parameters:
	socket - An active udp socket for which tx and rx offset to be set.
 
  Returns:
	true if success,
    false otherwise.

  Remarks:
  	 This routine should be called for every new snmp packet received.
***************************************************************************/
bool SNMPProcessReqBuildRespnsDuplexInit(UDP_SOCKET socket)
{
    // In full duplex transfer, transport protocol must be ready to
    // accept new transmit packet.
    //if(UDPIsPutReady(socket))
    if(UDPIsTxPutReady(socket,SNMP_MAX_MSG_SIZE))
    {
        // Initialize buffer offsets.
        SNMPRxOffset = 0;
        SNMPTxOffset = 0;
        return true;
    }

    return false;

}


/****************************************************************************
  Function:
	void SNMPPutByteToUDPTxBuffer(uint8_t v)
	
  Summary:
  	Copy byte to tx buffer.

  Description:
	This function writes a single byte to the currently active UDP socket, 
	transmit buffer, while incrementing the buffer offset for the next write 
	operation.

  Precondition:
	SNMPTask() is called.	
	A active udp socket is availabe to tx from.
	
  Parameters:
	None.
 
  Returns:
	None.

  Remarks:
  	None.
***************************************************************************/
void SNMPPutByteToUDPTxBuffer(UDP_SOCKET socket,uint8_t v)
{
    UDPSetTxOffset(socket, SNMPTxOffset);

    UDPPut(socket, v);

    SNMPTxOffset++;
}

static void SNMPPutDataArrayToUDPTxBuffer(UDP_SOCKET socket,SNMPBUFFERDATA *snmpPutData)
{	
    UDPSetTxOffset(socket, snmpPutData->length-1);
	UDPPutArray(socket,snmpPutData->head,snmpPutData->length);
}

/****************************************************************************
  Function:
	uint8_t SNMPGetByteFromUDPRxBuff(void)
	
  Summary:
  	Read byte from snmp udp socket rx buffer.

  Description:
	This function reads a single byte from the currently active UDP socket, 
	receive buffer, while incrementing the buffer offset from where the next 
	byte will be read.
	
  Precondition:
	SNMPTask() is called.
	A active udp socket is available to read from.
	
  Parameters:
	None
 
  Returns:
	None.

  Remarks:
  	None.
***************************************************************************/
uint8_t SNMPGetByteFromUDPRxBuff(UDP_SOCKET skt)
{
    uint8_t v;
	uint8_t intfIdx = ((TCPIP_NET_IF*)TCPIP_STACK_GetDefaultNet())->netIfIx; 
	
	if(!snmpDcpt[intfIdx].readFromSnmpBuf)
	{
	    UDPSetRxOffset(skt, SNMPRxOffset++);
	    UDPGet(skt, &v);
	}
	else
	{
		v= (uint8_t)snmpV1V2cIncomingMsgBufPtr.snmpMsgHead[SNMPRxOffset++];
	}
    return v;
}


/****************************************************************************
  Function:
	bool SNMPFindOIDStringByID(SNMP_ID id, OID_INFO* info, 
						  uint8_t* oidString, uint8_t* len)
	
  Summary:
  	Get complete notification variable OID string from snmp.bib using var id.
  	
  Description:
  	This routine is called when a OID string is required to be searched
  	from snmp.bib using agent id. The string is saved with agent.  
  	TRAP pdu is send with this OID corresponding to the SNMP_ID used 
  	by the agent application to send the pdu.
	
  Precondition:
	SNMPNotify() is called.	
	
  Parameters:
	id			-	System ID to use identify this agent.
	info		-	Pointer to SNMP MIB variable object information
	oidString	-	Pointer to store the string of OID serached
	len			-	Oid length
 
  Return Values:
	true	-	If oid string is found for the variable id in snmp.bib.
	FLASE	-	Otherwise.

  Remarks:
  	This function is used only when TRAP is enabled.
***************************************************************************/
bool SNMPFindOIDStringByID(SNMP_ID id, OID_INFO* info, uint8_t* oidString, uint8_t* len)
{
    uint32_t hCurrent;

    hCurrent = 0;

    while (1)
    {
    	//Read in the Mib record for the oid info
        SNMPReadMIBRecord(hCurrent, info);

        if ( !info->nodeInfo.Flags.bIsParent )
        {
            if ( info->nodeInfo.Flags.bIsIDPresent )
            {
                if ( info->id == id )
                    return SNMPGetOIDStringByAddr(info, oidString, len);
            }

            if ( info->nodeInfo.Flags.bIsSibling ||
                 info->nodeInfo.Flags.bIsDistantSibling )

			{
				SYS_FS_lseek(snmpFileDescrptr, info->hSibling, SEEK_SET);
            }
			else
                break;

        }

	    hCurrent = SYS_FS_lseek(snmpFileDescrptr,0,SEEK_CUR);
    }
    return false;
}



/****************************************************************************
  Function:
	bool SNMPGetOIDStringByAddr(OID_INFO* rec, uint8_t* oidString, uint8_t* len)
	
  Summary:
  	Get OID string from snmp.bib using the node address.
  	
  Description:
  	This routine is called when a OID string is required to be searched
  	from snmp.bib using node address.
  	
  Precondition:
	None.
	
  Parameters:
	rec			-	Pointer to SNMP MIB variable object information
	oidString	-	Pointer to store the string of OID searched
	len			-	Oid length
 
  Return Values:
	true	-	If oid string is found.
	FLASE	-	Otherwise.

  Remarks:
  	None.
***************************************************************************/
bool SNMPGetOIDStringByAddr(OID_INFO* rec, uint8_t* oidString, uint8_t* len)
{
    uint32_t hTarget;
    uint32_t hCurrent;
    uint32_t hNext;
    OID_INFO currentMIB;
    uint8_t index;
    enum { SM_PROBE_SIBLING, SM_PROBE_CHILD } state;

    hCurrent = 0;


    hTarget = rec->hNode;//node address
    state = SM_PROBE_SIBLING;
    index = 0;

    while( 1 )
    {
        SNMPReadMIBRecord(hCurrent, &currentMIB);

        oidString[index] = currentMIB.oid;

        if ( hTarget == hCurrent )
        {
            *len = ++index;
            return true;
        }

        switch(state)
        {
        case SM_PROBE_SIBLING:
            if ( !currentMIB.nodeInfo.Flags.bIsSibling )
                state = SM_PROBE_CHILD;

            else
            {
                hNext = currentMIB.hSibling;

				SYS_FS_lseek(snmpFileDescrptr, hNext, SEEK_SET);

				hNext = SYS_FS_lseek(snmpFileDescrptr,0,SEEK_CUR);
				
                if ( hTarget >= hNext )
                {
                    hCurrent = hNext;
                    break;
                }
                else
                    state = SM_PROBE_CHILD;
            }

        case SM_PROBE_CHILD:
            if ( !currentMIB.nodeInfo.Flags.bIsParent )
                return false;

            index++;

            hCurrent = currentMIB.hChild;
            state = SM_PROBE_SIBLING;
            break;
        }
    }
    return false;
}


/****************************************************************************
  Function:
	void SNMPReadMIBRecord(uint32_t h, OID_INFO* rec)
	
  Summary:
  	Get OID string from snmp.bib using the node address.
  	
  Description:
  	This routine is called when a OID string is required to be searched
  	from snmp.bib using node address.
  	
  Precondition:
	SNMPFindOIDStringByID() or SNMPGetOIDStringByAddr() is called.
	
  Parameters:
	h		-	Node adderess whose oid is to be read.
	rec		-	Pointer to store SNMP MIB variable object information
  
  Returns:
	None.
	
  Remarks:
  	None.
***************************************************************************/
static void SNMPReadMIBRecord(uint32_t h, OID_INFO* rec)
{
    MIB_INFO nodeInfo;
    TCPIP_UINT16_VAL tempVal;
	uint8_t idLen=1;


	SYS_FS_lseek(snmpFileDescrptr, h, SEEK_SET);

    // Remember location of this record.
    rec->hNode = h;

    // Read OID
	SYS_FS_read(snmpFileDescrptr,&rec->oid,1);

    // Read nodeInfo
	SYS_FS_read(snmpFileDescrptr,&rec->nodeInfo.Val,1);
    nodeInfo = rec->nodeInfo;

    // Read id, if there is any: Only leaf node with dynamic data will have id.
    if ( nodeInfo.Flags.bIsIDPresent )
    {
    	 // Fetch index ID.
		SYS_FS_read(snmpFileDescrptr,&idLen,1);
	    if(idLen == 1)
	    {
	    	uint8_t temp=0;
			SYS_FS_read(snmpFileDescrptr,&temp,1);
			rec->id = temp & 0xFF;			
	    }
		else if(idLen == 2)
		{
			uint8_t temp[2];
			SYS_FS_read(snmpFileDescrptr,temp,2);
			rec->id = 0;
			rec->id = temp[0] & 0xFF;
			rec->id <<= 8;
			rec->id |= temp[1] & 0xFF;				
		}		
    }
    // Read Sibling offset if there is any - any node may have sibling
    if ( nodeInfo.Flags.bIsSibling )
    {
		SYS_FS_read(snmpFileDescrptr,&tempVal.v[0],1);
		SYS_FS_read(snmpFileDescrptr,&tempVal.v[1],1);
        rec->hSibling = tempVal.Val;
    }

    // All rest of the parameters are applicable to leaf node only.
    if ( nodeInfo.Flags.bIsParent )
	{
	   rec->hChild = SYS_FS_lseek(snmpFileDescrptr,0,SEEK_CUR);
    }
    else
    {
        if ( nodeInfo.Flags.bIsDistantSibling )
        {
            // Read Distant Sibling if there is any - only leaf node will have distant sibling
			SYS_FS_read(snmpFileDescrptr,&tempVal.v[0],1);
			SYS_FS_read(snmpFileDescrptr,&tempVal.v[1],1);
            rec->hSibling = tempVal.Val;
        }

        // Save data type for this node.
        rec->dataType = 0;
		SYS_FS_read(snmpFileDescrptr,(uint8_t*)&rec->dataType,1);

		rec->hData = SYS_FS_lseek(snmpFileDescrptr,0,SEEK_CUR);

    }

}

bool SNMPGetDataTypeInfo(SNMP_DATA_TYPE dataType, SNMP_DATA_TYPE_INFO *info )
{
    if ( dataType >= DATA_TYPE_UNKNOWN )
    {
		info->asnType   = 0x00;
	    info->asnLen    = 0x00;
		return false;
	}

    info->asnType   = dataTypeTable[dataType].asnType;
    info->asnLen    = dataTypeTable[dataType].asnLen;

    return true;
}


/****************************************************************************
  Function:
	uint8_t SNMPProcessSetVar(PDU_INFO* pduDbPtr,OID_INFO* rec, 
					   SNMP_ERR_STATUS* errorStatus)
	
  Summary:
  	Processes snmp Set request pdu.
  	
  Description:
  	This routine processes the received snmp set request pdu for the 
  	variable binding in the request and also creates the response pdu.
  	
  Precondition:
	SNMPProcessVariables() is called.
	
  Parameters:
    pduDbPtr	-   Received pdu information database pointer
    rec		  	-   Pointer to SNMP MIB variable object information
    errorStatus -   Pointer to update error status info

  Return Values:
	copiedBytes	- Number of bytes copied by this routine to the 
				  snmp pdu tx buffer.

  Remarks:
  	None.
***************************************************************************/
uint8_t SNMPProcessSetVar(PDU_INFO* pduDbPtr,OID_INFO* rec, SNMP_ERR_STATUS* errorStatus)
{
	uint8_t ref;
	uint8_t temp=0;
	uint8_t dataType=0;
	uint8_t dataLen=0;
	uint8_t copiedBytes=0;
    SNMP_ERR_STATUS errorCode;
    SNMP_DATA_TYPE_INFO actualDataTypeInfo;
    SNMP_VAL dataValue;
	uint8_t intfIdx;
	
	
	SNMPBUFFERDATA *snmpPutData = NULL;
	
	#ifdef TCPIP_STACK_USE_SNMPV3_SERVER	
		SNMPV3MSGDATA	*dynPduBuf=NULL;
		
		SNMPV3_PROCESSING_MEM_INFO_PTRS snmpv3PktProcessingMemPntr; 
		SNMPV3_STACK_DCPT_STUB * snmpv3EngnDcptMemoryStubPtr=0;  
							 
		SNMPv3GetPktProcessingDynMemStubPtrs(&snmpv3PktProcessingMemPntr);
							 
		snmpv3EngnDcptMemoryStubPtr=snmpv3PktProcessingMemPntr.snmpv3StkProcessingDynMemStubPtr;
		dynPduBuf = &snmpv3EngnDcptMemoryStubPtr->ScopedPduRespnsBuf;										 
    #endif

	intfIdx = ((TCPIP_NET_IF*)TCPIP_STACK_GetDefaultNet())->netIfIx; 
	snmpPutData = &SnmpStackDcptMemStubPtr->outPduBufData;
	
    // Start with no error.
    errorCode = SNMP_NO_ERR;
    copiedBytes = 0;

	

    // Non-leaf, Constant and ReadOnly node cannot be modified
    if(rec->nodeInfo.Flags.bIsParent   ||
       rec->nodeInfo.Flags.bIsConstant ||
       !rec->nodeInfo.Flags.bIsEditable )
    {	
        if(pduDbPtr->snmpVersion == (uint8_t)SNMP_V1)
       		errorCode = SNMP_NO_SUCH_NAME;
		else if ((pduDbPtr->snmpVersion == (uint8_t)SNMP_V2C) ||
						(pduDbPtr->snmpVersion == (uint8_t)SNMP_V3))
			errorCode = SNMP_NOT_WRITABLE;
	}

	if(pduDbPtr->snmpVersion != (uint8_t)SNMP_V3)
	{
	    dataType = SNMPGetByteFromUDPRxBuff(snmpDcpt[intfIdx].skt);
	    SNMPPutDataToProcessBuff(dataType,snmpPutData);
	}
	#ifdef TCPIP_STACK_USE_SNMPV3_SERVER	
	else
	{
	    dataType = SNMPv3GetProcessBuffData(snmpv3EngnDcptMemoryStubPtr->ScopedPduRequstBuf,++snmpv3EngnDcptMemoryStubPtr->ScopedPduDataPos);	    
		SNMPv3CopyDataToProcessBuff(dataType,dynPduBuf);
	}
	#endif
	copiedBytes++;

    // Get data type for this node.
    if ( !SNMPGetDataTypeInfo(rec->dataType, &actualDataTypeInfo) )
	{
		if(pduDbPtr->snmpVersion == (uint8_t)SNMP_V1)
        	errorCode = SNMP_BAD_VALUE;
		else if ((pduDbPtr->snmpVersion == (uint8_t)SNMP_V2C) ||
					(pduDbPtr->snmpVersion == (uint8_t) SNMP_V3))
			errorCode = SNMP_WRONG_TYPE;
	}

    // Make sure that received data type is same as what is declared
    // for this node.
    if ( dataType != actualDataTypeInfo.asnType )
	{	
        if(pduDbPtr->snmpVersion == (uint8_t)SNMP_V1)
        	errorCode = SNMP_BAD_VALUE;
		else if ((pduDbPtr->snmpVersion == (uint8_t)SNMP_V2C) ||
					(pduDbPtr->snmpVersion == (uint8_t) SNMP_V3))
			errorCode = SNMP_WRONG_TYPE;
	}

    // Make sure that received data length is within our capability.
	if(pduDbPtr->snmpVersion != (uint8_t) SNMP_V3)
	{
	    dataLen = SNMPGetByteFromUDPRxBuff(snmpDcpt[intfIdx].skt);
	    SNMPPutDataToProcessBuff(dataLen,snmpPutData);
	}
	#ifdef TCPIP_STACK_USE_SNMPV3_SERVER	
	else
	{
	    dataLen = SNMPv3GetProcessBuffData(snmpv3EngnDcptMemoryStubPtr->ScopedPduRequstBuf,++snmpv3EngnDcptMemoryStubPtr->ScopedPduDataPos);	    
		SNMPv3CopyDataToProcessBuff(dataLen,dynPduBuf);
	}
	#endif
    copiedBytes++;

    // Only max data length of 127 is supported.
    if ( dataLen > 0x7fu )
	{
		if(pduDbPtr->snmpVersion == (uint8_t)SNMP_V1)
        	errorCode = SNMP_BAD_VALUE;
		else if ((pduDbPtr->snmpVersion == (uint8_t)SNMP_V2C)||
				(pduDbPtr->snmpVersion == (uint8_t) SNMP_V3))
			errorCode = SNMP_WRONG_LENGTH;
	}

	
    // If this is a Simple variable and given index is other than '0',
    // it is considered bad value
    if ( !rec->nodeInfo.Flags.bIsSequence && rec->index != 0x00u ){
        errorCode = SNMP_NO_SUCH_NAME;}

    dataValue.dword = 0;
    ref = 0;

    // If data length is within 4 bytes, fetch all at once and pass it
    // to application.
    if ( actualDataTypeInfo.asnLen != 0xff )
    {
        // According to mib def., this data length for this data type/
        // must be less or equal to 4, if not, we don't know what this
        // is.
        if ( dataLen <= 4u )
        {
            // Now that we have verified data length, fetch them all
            // at once and save it in correct place.
            //dataLen--;

            while( dataLen-- )
            {
				if(pduDbPtr->snmpVersion != (uint8_t) SNMP_V3)
				{
	                temp = SNMPGetByteFromUDPRxBuff(snmpDcpt[intfIdx].skt);
	                // Copy same byte back to create response...
	                SNMPPutDataToProcessBuff(temp,snmpPutData);
				}
				#ifdef TCPIP_STACK_USE_SNMPV3_SERVER	
				else
				{
					temp = SNMPv3GetProcessBuffData(snmpv3EngnDcptMemoryStubPtr->ScopedPduRequstBuf,++snmpv3EngnDcptMemoryStubPtr->ScopedPduDataPos);		
					SNMPv3CopyDataToProcessBuff(temp,dynPduBuf);
				}
				#endif
				dataValue.v[dataLen] = temp;
                copiedBytes++;
            }


            // Pass it to application.
            if ( errorCode == SNMP_NO_ERR )
            {
                if(!SNMPSetVar(rec->id, rec->index, ref, dataValue))
				{
                   	if(pduDbPtr->snmpVersion == (uint8_t)SNMP_V1)
        				errorCode = SNMP_BAD_VALUE;
					else if ((pduDbPtr->snmpVersion == (uint8_t)SNMP_V2C) ||
							(pduDbPtr->snmpVersion == (uint8_t) SNMP_V3))
						errorCode = SNMP_WRONG_VALUE;
				}
            }
        }
        else
		{
            if(pduDbPtr->snmpVersion == (uint8_t)SNMP_V1)
        		errorCode = SNMP_BAD_VALUE;
			else if ((pduDbPtr->snmpVersion == (uint8_t)SNMP_V2C)||
				(pduDbPtr->snmpVersion == (uint8_t) SNMP_V3))
			{	
				if( rec->nodeInfo.Flags.bIsConstant)
					errorCode = SNMP_NOT_WRITABLE;
				else
					errorCode = SNMP_WRONG_LENGTH;
			}
		}	
    }
    else
    {
        // This is a multi-byte Set operation.
        // Check with application to see if this many bytes can be
        // written to current variable.      
     //   if ( !SNMPIsValidSetLen(rec->id, dataLen) )
 		if ( !SNMPIsValidSetLen(rec->id, dataLen,rec->index) )
		{
            if(pduDbPtr->snmpVersion == (uint8_t)SNMP_V1)
        		errorCode = SNMP_BAD_VALUE;
			else if ((pduDbPtr->snmpVersion == (uint8_t)SNMP_V2C)  ||
				(pduDbPtr->snmpVersion == (uint8_t) SNMP_V3))
			{
				if( rec->nodeInfo.Flags.bIsConstant)
					errorCode = SNMP_NOT_WRITABLE;
				else
					errorCode = SNMP_WRONG_LENGTH;
			}
		}
        // Even though there may have been error processing this
        // variable, we still need to reply with original data
        // so at least copy those bytes.
        while( dataLen-- )
        {
			if(pduDbPtr->snmpVersion != (uint8_t) SNMP_V3)
			{
	            dataValue.byte = SNMPGetByteFromUDPRxBuff(snmpDcpt[intfIdx].skt);
	            SNMPPutDataToProcessBuff(dataValue.byte,snmpPutData);
			}
			#ifdef TCPIP_STACK_USE_SNMPV3_SERVER	
			else
			{
				dataValue.byte = SNMPv3GetProcessBuffData(snmpv3EngnDcptMemoryStubPtr->ScopedPduRequstBuf,++snmpv3EngnDcptMemoryStubPtr->ScopedPduDataPos);		
				SNMPv3CopyDataToProcessBuff(dataValue.byte,dynPduBuf);
			}
			#endif
            copiedBytes++;

            // Ask applicaton to set this variable only if there was
            // no previous error.
            if ( errorCode == SNMP_NO_ERR )
            {
                if ( !SNMPSetVar(rec->id, rec->index, ref++, dataValue) )
				{
                    errorCode = SNMP_BAD_VALUE;
				}
            }
        }
        // Let application know about end of data transfer
        if ( errorCode == SNMP_NO_ERR )
		{
            if(!SNMPSetVar(rec->id, rec->index, (uint16_t)SNMP_END_OF_VAR, dataValue))
			{
                errorCode = SNMP_BAD_VALUE;
			}
		}
    }
    *errorStatus = errorCode;

    return copiedBytes;
}


/****************************************************************************
  Function:
	uint8_t SNMPProcessGetVar(OID_INFO* rec, bool bAsOID,PDU_INFO* pduDbPtr)
	
  Summary:
  	Processes snmp Get request pdu.
  	
  Description:
  	This routine processes the received snmp Get request pdu for the 
  	variable binding in the request and also creates the response pdu.
  	
  Precondition:
	SNMPProcessVariables() is called.
	
  Parameters:
    rec		 -   Pointer to SNMP MIB variable object information
    bAsOID	 -   Oid flag.

  Return Values:
	varLen	- Number of bytes put in response tx pdu		
	false	- If any of the elements of the request pdu validation fails.
	
  Remarks:
  	None.
***************************************************************************/
uint8_t SNMPProcessGetVar(OID_INFO* rec, bool bAsOID,PDU_INFO* pduDbPtr)
{
    uint8_t ref;
    uint8_t temp;
    uint8_t varLen;
    uint8_t dataType;
    uint16_t offset;
    uint16_t prevOffset;
    SNMP_VAL v;
    SNMP_DATA_TYPE_INFO dataTypeInfo;
	SNMPBUFFERDATA *snmpPutData = NULL;
	
	 
	#ifdef TCPIP_STACK_USE_SNMPV3_SERVER	

	SNMPV3MSGDATA	*dynPduBuf=NULL;
		SNMPV3_PROCESSING_MEM_INFO_PTRS snmpv3PktProcessingMemPntr; 
		SNMPV3_STACK_DCPT_STUB * snmpv3EngnDcptMemoryStubPtr=0;	 
						 
		SNMPv3GetPktProcessingDynMemStubPtrs(&snmpv3PktProcessingMemPntr);
						 
		snmpv3EngnDcptMemoryStubPtr=snmpv3PktProcessingMemPntr.snmpv3StkProcessingDynMemStubPtr;
		dynPduBuf = &snmpv3EngnDcptMemoryStubPtr->ScopedPduRespnsBuf;
		
	#endif
	
	snmpPutData = &SnmpStackDcptMemStubPtr->outPduBufData;
	
	offset = 0;	// Suppress C30 warning: 'offset' may be used uninitialized in this function
    v.dword   = 0;

    // Non-leaf node does not contain any data.
    if ( rec->nodeInfo.Flags.bIsParent )
        return false;

    // If current OID is Simple variable and index is other than .0
    // we don't Get this variable.
    if ( !rec->nodeInfo.Flags.bIsSequence )
    {
        // index of other than '0' is not invalid.
        if ( rec->index > 0u )
            return false;
    }

    dataType = rec->dataType;
    if ( !SNMPGetDataTypeInfo(dataType, &dataTypeInfo) )
        return false;

    if ( !bAsOID )
    {
		if(pduDbPtr->snmpVersion != (uint8_t) SNMP_V3)
		{
			if(SNMPPutDataToProcessBuff(dataTypeInfo.asnType,snmpPutData)!=true)
				return false;

	        offset = snmpPutData->length;//SNMPTxOffset;
	        if(SNMPPutDataToProcessBuff(dataTypeInfo.asnLen,snmpPutData)!=true)
				return false;
		}
		#ifdef TCPIP_STACK_USE_SNMPV3_SERVER	
		else
		{
			if(SNMPv3CopyDataToProcessBuff(dataTypeInfo.asnType,dynPduBuf) != true)
				return false;
	        offset = dynPduBuf->length;
			if(SNMPv3CopyDataToProcessBuff(dataTypeInfo.asnLen,dynPduBuf)!= true)
				return false;
		}
		#endif
    }

    if ( rec->nodeInfo.Flags.bIsConstant )
    {
        uint8_t c;

		SYS_FS_lseek(snmpFileDescrptr, rec->hData, SEEK_SET);
		SYS_FS_read(snmpFileDescrptr,&varLen,1);
        temp = varLen;
        while( temp-- )
	    {
			SYS_FS_read(snmpFileDescrptr,&c,1);
			if(pduDbPtr->snmpVersion != (uint8_t) SNMP_V3)
			{
            	if(SNMPPutDataToProcessBuff(c,snmpPutData)!= true)
					return false;
			}
			#ifdef TCPIP_STACK_USE_SNMPV3_SERVER	
			else
				if(SNMPv3CopyDataToProcessBuff(c,dynPduBuf) != true)
					return false;
			#endif	
        }
    }
    else
    {
        ref = SNMP_START_OF_VAR;
        v.dword = 0;
        varLen = 0;

        do
        {
            if ( SNMPGetVar(rec->id, rec->index, &ref, &v) )
            {
                if ( dataTypeInfo.asnLen != 0xff )
                {
                    varLen = dataTypeInfo.asnLen;

                    while( dataTypeInfo.asnLen )
                    {
						if(pduDbPtr->snmpVersion != (uint8_t) SNMP_V3)
						{
                      	 	if(SNMPPutDataToProcessBuff(v.v[--dataTypeInfo.asnLen],snmpPutData)!= true)
								return false;
						}
						#ifdef TCPIP_STACK_USE_SNMPV3_SERVER
						else
							if(SNMPv3CopyDataToProcessBuff(v.v[--dataTypeInfo.asnLen],dynPduBuf)!=true)
								return false;
						#endif
                    }

                    break;
                }
                else
                {
                    varLen++;
					if(pduDbPtr->snmpVersion != (uint8_t) SNMP_V3)
					{
						if(SNMPPutDataToProcessBuff(v.v[0],snmpPutData)!= true)
							return false;
					}
					#ifdef TCPIP_STACK_USE_SNMPV3_SERVER
					else
						if(SNMPv3CopyDataToProcessBuff(v.v[0],dynPduBuf) != true)
							return false;
					#endif
                }
            }
            else
            {
            	return false;
            }

        } while( ref != SNMP_END_OF_VAR );
    }

    if ( !bAsOID )
    {

		 if(pduDbPtr->snmpVersion != SNMP_V3)
		 {
		 	
	         prevOffset = snmpPutData->length; 
			 snmpPutData->length = offset;
	        SNMPPutDataToProcessBuff(varLen,snmpPutData);

	       snmpPutData->length = prevOffset;
		 }
		 #ifdef TCPIP_STACK_USE_SNMPV3_SERVER	
		 else
	 	{
	 		prevOffset = dynPduBuf->length;
			dynPduBuf->length = offset;
			SNMPv3CopyDataToProcessBuff(varLen,dynPduBuf);	
			dynPduBuf->length = prevOffset;
	 	}
		#endif
        varLen++;
        varLen++;
    }

    return varLen;
}


/****************************************************************************
  Function:
	void SNMPSetErrorStatus(uint16_t errorStatusOffset,
                           uint16_t errorIndexOffset,
                           SNMP_ERR_STATUS errorStatus,
                           uint8_t errorIndex)
  Summary:
  	Set snmp error status in the response pdu. 
  	
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
    
  Returns:
	None.
	
  Remarks:
  	None.
***************************************************************************/
void SNMPSetErrorStatus(uint16_t errorStatusOffset,
                           uint16_t errorIndexOffset,
                           SNMP_ERR_STATUS errorStatus,
                           uint8_t errorIndex,SNMPBUFFERDATA *snmpPutTxData)
{
    uint16_t prevOffset;

    prevOffset = snmpPutTxData->length;
	snmpPutTxData->length = errorStatusOffset;
    SNMPPutDataToProcessBuff(errorStatus,snmpPutTxData);

    
	snmpPutTxData->length = errorIndexOffset;
    SNMPPutDataToProcessBuff(errorIndex,snmpPutTxData);

	snmpPutTxData->length = prevOffset;
}


/****************************************************************************
  Function:
	uint8_t SNMPFindOIDsInRequest(uint16_t pdulen)
	
  Summary:
  	Finds number of varbinds in the varbind list received in a pdu.
  	
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
static uint8_t SNMPFindOIDsInRequest(uint16_t pdulen)
{
uint8_t  structureLen;
uint8_t varCount=0;
uint16_t prevUDPRxOffset;
uint16_t varBindLen;
uint16_t snmpPduLen;
 	
	snmpPduLen=pdulen;

	prevUDPRxOffset=SNMPRxOffset;

	while(snmpPduLen)
	{
		
		structureLen = SNMPCheckIfValidSnmpStructure(&varBindLen);
		if(!structureLen)
		  return false;
	//	if(!SNMPCheckIfValidSnmpStructure(&varBindLen))
		//	return false;

		SNMPRxOffset=SNMPRxOffset+varBindLen;
		varCount++;
		snmpPduLen=snmpPduLen 
			- structureLen // 1 byte for STRUCTURE identifier + 0x82 or ox81+1+1 byte(s) for varbind length 
			- varBindLen;
					//-1 //1 byte for STRUCTURE identifier
					//-1//1 byte for varbind length 
				//	-varBindLen;
	}

	SNMPRxOffset=prevUDPRxOffset;

	return varCount;
}

/****************************************************************************
  Function:
	uint8_t *SNMPRetrieveWriteCommunity(int index)
	
  Summary:
  	Get the writeCommunity String with Snmp index.
  	
  Description:
  	Get the writeCommunity String with Snmp index.
  	
  Precondition	:
	SNMPProcessVariables() is called.
			
  Parameters:
	index		-	SNMP_MAX_COMMUNITY_SUPPORT. 
    
  Return Values:
	uint8_t *	-	unsigned char community string  
	
  Remarks:
  	None.
***************************************************************************/

uint8_t*  SNMPRetrieveWriteCommunity(int index)
{
    return (uint8_t*)SnmpStackDcptMemStubPtr->snmpNetConfig.writeCommunity[index];
}


/****************************************************************************
  Function:
	uint8_t *SNMPRetrieveReadCommunity(int index)
	
  Summary:
  	Get the readCommunity String with Snmp index.
  	
  Description:
  	Get the readCommunity String with Snmp index.
  	
  Precondition	:
	SNMPProcessVariables() is called.
		
  Parameters:
	index		-	SNMP_MAX_COMMUNITY_SUPPORT. 
    
  Return Values:
	uint8_t *	-	unsigned char community string  
	
  Remarks:
  	None.
***************************************************************************/

uint8_t*  SNMPRetrieveReadCommunity(int index)
{
    return (uint8_t*)SnmpStackDcptMemStubPtr->snmpNetConfig.readCommunity[index];
}


/****************************************************************************
  Function:
	static bool SNMPLoadDefaultUserConfig(void)
	
  Summary:
  	Configure SNMP Default configuration.
  	
  Description:
   	Initialize SNMP default Communities and other snmp parameters.
  	
  Precondition	:
	This is called from SNMPInit().		
  Parameters:
	void    
  Return Values:
	bool	
  Remarks:
  	None.
***************************************************************************/

static bool SNMPLoadDefaultUserConfig(void)
{
	   uint8_t i;
	   static const char * const cReadCommunities[] = SNMP_READ_COMMUNITIES;
	   static const char * const cWriteCommunities[] = SNMP_WRITE_COMMUNITIES;
	   const char * strCommunity;

	   for(i = 0; i < SNMP_MAX_COMMUNITY_SUPPORT; i++)
	   {
		   // Get a pointer to the next community string
		   strCommunity = cReadCommunities[i];
		   if(i >= sizeof(cReadCommunities)/sizeof(cReadCommunities[0]))
			   strCommunity = "";

		   // Ensure we don't buffer overflow.	If your code gets stuck here, 
		   // it means your SNMP_COMMUNITY_MAX_LEN definition in tcpip_config.h 
		   // is either too small or one of your community string lengths 
		   // (SNMP_READ_COMMUNITIES) are too large.  Fix either.
		   if(strlen(strCommunity) >= sizeof(SnmpStackDcptMemStubPtr->snmpNetConfig.readCommunity[0]))
			   return false; 

		   // Copy string into tcpipNetIf
	   		strcpy((char*)SnmpStackDcptMemStubPtr->snmpNetConfig.readCommunity[i], strCommunity);

		   // Get a pointer to the next community string
		   strCommunity = cWriteCommunities[i];
		   if(i >= sizeof(cWriteCommunities)/sizeof(cWriteCommunities[0]))
			   strCommunity = "";

		   // Ensure we don't buffer overflow.	If your code gets stuck here, 
		   // it means your SNMP_COMMUNITY_MAX_LEN definition in tcpip_config.h 
		   // is either too small or one of your community string lengths 
		   // (SNMP_WRITE_COMMUNITIES) are too large.  Fix either.
		   if(strlen(strCommunity) >= sizeof(SnmpStackDcptMemStubPtr->snmpNetConfig.writeCommunity[0]))
			   return false; 

		   // Copy string into tcpipNetIf
	   		strcpy((char*)SnmpStackDcptMemStubPtr->snmpNetConfig.writeCommunity[i], strCommunity);
	   }
           return true;
}
/****************************************************************************
  Function:
	bool SNMPCheckIfPvtMibObjRequested(uint8_t* OIDValuePtr)
	
  Summary:
  	To find whether requested OID is only for private access.
  	
  Description:
  	This routine is used to find whether requested object belongs
  	to the private object group of the mib of agent. If yes, then
  	that mib object can be accessed only with private community 
  	(supported in SNMPv2c). 
  	
  Precondition	:
	SNMPProcessVariables() is called.	
		
  Parameters:
	OIDValuePtr	-	Pointer to memory stored with received OID.
    
  Return Values:
	true	-	If the requested object is of private branch of the mib.
	FLASE	-	If the requested object is publically accessible.
	
  Remarks:
  	None.
***************************************************************************/
static bool SNMPCheckIfPvtMibObjRequested(uint8_t* OIDValuePtr)
{
	uint8_t cnt=0;
	uint8_t pvtObjIdentifier[4]={0x2b,0x06/*dod*/,0x01/*internet*/,0x04/*private*/};

	while(cnt<4u)
	{
		//check whether requested oid is for pvt obj
		if(pvtObjIdentifier[cnt]== OIDValuePtr[cnt])
		{
			cnt++;
		}
		else
		{
			cnt=0;
			return false;
		}
		if(cnt == 0x04u)
			return true;
	}
	return false;

}






#endif //#if defined(TCPIP_STACK_USE_SNMP_SERVER)

