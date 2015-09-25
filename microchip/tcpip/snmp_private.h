/*******************************************************************************
  SNMP Stack private APIs

  Summary:
    SNMP Stack private API for Microchip TCP/IP Stack
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  ipv6_private.h 
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
#ifndef _SNMP_PRIVATE_H_
#define _SNMP_PRIVATE_H_


// Section:  SNMP specific variables
#define STRUCTURE               (0x30u)
#define ASN_INT                 (0x02u)
#define OCTET_STRING            (0x04u)
#define ASN_NULL                (0x05u)
#define ASN_OID                 (0x06u)


#define SNMP_IP_ADDR            (0x40)
#define SNMP_COUNTER32          (0x41)
#define SNMP_GAUGE32            (0x42)
#define SNMP_TIME_TICKS         (0x43)
#define SNMP_OPAQUE             (0x44)
#define SNMP_NSAP_ADDR          (0x45)



// Section:  SNMP v1 and v2c pdu types
#define GET_REQUEST             (0xa0)
#define GET_NEXT_REQUEST        (0xa1)
#define GET_RESPONSE            (0xa2)
#define SET_REQUEST             (0xa3)
#define TRAP                    (0xa4)
#define GET_BULK_REQUEST        (0xa5)
#define REPORT_RESPONSE			(0xa8)


// Section:  SNMP Udp ports
#define SNMP_AGENT_PORT     (161)
#define SNMP_NMS_PORT       (162)
#define AGENT_NOTIFY_PORT   (0xfffe)


// Section:  SNMP specific data validation
#define IS_STRUCTURE(a)         (a==STRUCTURE)
#define IS_ASN_INT(a)           (a==ASN_INT)
#define IS_OCTET_STRING(a)      (a==OCTET_STRING)
#define IS_OID(a)               (a==ASN_OID)
#define IS_ASN_NULL(a)          (a==ASN_NULL)
#define IS_GET_REQUEST(a)       (a==GET_REQUEST)
#define IS_GET_NEXT_REQUEST(a)  (a==GET_NEXT_REQUEST)
#define IS_GET_RESPONSE(a)      (a==GET_RESPONSE)
#define IS_SET_REQUEST(a)       (a==SET_REQUEST)
#define IS_TRAP(a)              (a==TRAP)
#define IS_AGENT_PDU(a)         (a==GET_REQUEST || \
                                 a==GET_NEXT_REQUEST || \
                                 a==SET_REQUEST || \
                                 a==SNMP_V2C_GET_BULK)
#define IS_SNMPV3_AUTH_STRUCTURE(a) (a==SNMPV3_ENCRYPTION)




/****************************************************************************
  Section:
	Data Structures and Enumerations
  ***************************************************************************/

// SNMPv1/v2c
typedef struct 
{	
	uint8_t *head;
	uint16_t length;
}SNMPBUFFERDATA;

// Section:  SNMP specific data tyes
typedef enum 
{
	INT8_VAL		= 0x00, 	//8 bit integer value
	INT16_VAL		= 0x01, 	//16 bit integer value
	INT32_VAL		= 0x02, 	//32 bit integer value
	BYTE_ARRAY		= 0x03, 	//Aray of bytes 
	ASCII_STRING	= 0x04, 	//Ascii string type
	IPADDRESS		= 0x05, 	//IP address variable
	COUNTER32		= 0x06, 	//32 bit counter variable
	TIME_TICKS_VAL	= 0x07, 	//Timer vakue counter variable	
	GAUGE32 		= 0x08, 	//32 bit guage variable
	OID_VAL 		= 0x09, 	//Object id value var
	DATA_TYPE_UNKNOWN			//Unknown data type
} SNMP_DATA_TYPE;


// Section:  SNMP specific mib file access information
typedef union
{
	struct
	{
		unsigned int bIsFileOpen : 1;  //MIB file access int flag
	} Flags;						
	uint8_t Val;						   //MIB file access byte flag
} SNMP_STATUS;


// Section:  SNMP OID index information
typedef union 
{
	struct
	{
		unsigned int bIsOID:1;	//value is OID/index int flag
	} Flags;
	uint8_t Val;					//value is OID/index byte flag
} SNMP_INDEX_INFO;



// Section:  SNMP reuested variable list error status information.
//Max variable in a request supported 15
typedef struct 
{
uint16_t noSuchObjectErr;		//Var list no such obj errors flags
uint16_t noSuchNameErr; 		//Var list no such name error 
uint16_t noSuchInstanceErr; 	//Var list no such instance error
uint16_t endOfMibViewErr;		//Var list end of mib view error
}reqVarErrStatus;  


//This is the SNMP OID variable id.
//This id is assigned via MIB file.  Only dynamic and AgentID
//variables can contian ID.  MIB2BIB utility enforces this
//rules when BIB was generated.
//typedef int SNMP_ID;

typedef uint32_t SNMP_ID;
typedef uint8_t SNMP_INDEX;


// Section:  ASN data type info
typedef struct 
{
	uint8_t asnType;	//ASN data type
	uint8_t asnLen;	//ASN data length
} SNMP_DATA_TYPE_INFO;


// Section:  SNMP object information
typedef union 
{
	struct
	{
		unsigned int bIsDistantSibling : 1; //Object have distant sibling node
		unsigned int bIsConstant : 1;		//Object is constant
		unsigned int bIsSequence : 1;		//Object is sequence
		unsigned int bIsSibling : 1;		//Sibling node flag

		unsigned int bIsParent : 1; 		//Node is parent flag	
		unsigned int bIsEditable : 1;		//Node is editable flag
		unsigned int bIsAgentID : 1;		//Node have agent id flag
		unsigned int bIsIDPresent : 1;		//Id present flag
	} Flags;
	uint8_t Val;								//MIB Obj info as byte value
} MIB_INFO;

// Section:  SNMP MIB variable object information
typedef struct 
{
	uint32_t			hNode;		//Node location in the mib		
	uint8_t			oid;		//Object Id
	MIB_INFO		nodeInfo;	//Node info
	SNMP_DATA_TYPE		dataType;	//Data type 
	SNMP_ID 		id; 		//Snmp Id	
	TCPIP_UINT16_VAL		dataLen;	//Data length	
	uint32_t			hData;		//Data
	uint32_t			hSibling;	//Sibling info
	uint32_t			hChild; 	//Child info
	uint8_t			index;		//Index of object
	uint8_t			indexLen;	//Index length
} OID_INFO;



// Section:  SNMP pdu information database 
typedef struct 
{
	TCPIP_UINT32_VAL		requestID;		//Snmp request id
	uint8_t			nonRepeators;	//# non repeaters in the request	
	uint8_t			maxRepetitions; //# max repeaters in the request 
	uint8_t			pduType;		//Snmp pdu type
	uint8_t			errorStatus;	//Pdu error status
	uint8_t			erroIndex;		//Pdu error Index
	uint8_t			snmpVersion;	//Snmp version
	uint16_t			pduLength;		//Pdu length
} PDU_INFO;



// Section:  SNMP specific errors
typedef enum 
{
    SNMP_NO_ERR = 0,			//Snmp no error
    SNMP_TOO_BIG,				//Value too big error
    SNMP_NO_SUCH_NAME,			//No such name in MIB error
    SNMP_BAD_VALUE,				//Not assignable value for the var error	
    SNMP_READ_ONLY,				//Read only variable, write not allowed err
    SNMP_GEN_ERR,				//Snmp gen error
    SNMP_NO_ACCESS,				//Access to modify or read not granted err 
    SNMP_WRONG_TYPE,			//Variable data type wrong error	
    SNMP_WRONG_LENGTH,			//Wrong data length error
    SNMP_WRONG_ENCODING,		//Wrong encoding error
    SNMP_WRONG_VALUE,			//Wrong value for the var type
    SNMP_NO_CREATION,			//No creationg error
    SNMP_INCONSISTENT_VAL,		//Inconsistent value error
    SNMP_RESOURCE_UNAVAILABE,	//Resource unavailbe error	
    SNMP_COMMIT_FAILED,			//Modification update failed error
    SNMP_UNDO_FAILED,			//Modification undo failed	
    SNMP_AUTH_ERROR,			//Authorization failed error
    SNMP_NOT_WRITABLE,			//Variable read only
    SNMP_INCONSISTENT_NAME,		//Inconsistent name
    SNMP_NO_SUCH_OBJ=128,		//No such object error	
    SNMP_NO_SUCH_INSTANCE=129,	//No such instance error
    SNMP_END_OF_MIB_VIEW=130	//Reached to end of mib error
} SNMP_ERR_STATUS;



//This is the list of SNMP action a remote NMS can perform.
//This inforamtion is passed to application via
//callback SNMPValidateCommunity().
//Application should validate the action for given community
//string.
typedef enum
{
    SNMP_GET            = 0xa0,	//Snmp GET identifier
    SNMP_GET_NEXT       = 0xa1, //Snmp GET_NEXT identifier
    SNMP_GET_RESPONSE   = 0xa2,	//Snmp GET_RESPONSE identifier
    SNMP_SET            = 0xa3,	//Snmp SET identifier
    SNMP_TRAP           = 0xa4,	//Snmp TRAP identifier
    SNMP_V2C_GET_BULK	= 0xa5,	//Snmp GET_BULK identifier
    SNMP_V2_TRAP		= 0xa7, //Snmp v2 Trap Identifier
    SNMPV3_ENCRYPTION	= 0x04,
    SNMP_ACTION_UNKNOWN = 0		//Snmp requested action unknown
} SNMP_ACTION;


typedef enum
{
	SNMP_RESPONSE_PDU=0x01,
	SNMP_REQUEST_PDU=0x02

}INOUT_SNMP_PDU;

typedef struct 
{	
	uint8_t* wholeMsgHead;
	uint8_t* snmpMsgHead;
	TCPIP_UINT16_VAL wholeMsgLen;
	TCPIP_UINT16_VAL snmpMsgLen;
}SNMPV1V2_REQUEST_WHOLEMSG;

typedef enum {
    SNMP_HOME = 0,
    SNMP_LISTEN,
    SNMP_PROCESS,
    SNMP_PACKET_DISCARD,
}TCPIP_SNMP_SM;



typedef struct
{
    TCPIP_SNMP_SM   sm;     // current status
    UDP_SOCKET      skt;    // associated socket
	bool readFromSnmpBuf;
}TCPIP_SNMP_DCPT;


#if !defined(SNMP_TRAP_DISABLED)
uint8_t *SNMPResolveGenTrapCodeToTrapOID(uint8_t generic_trap_code,uint8_t *len);
#endif

bool SNMPCheckIfValidInt(uint32_t* val);
bool SNMPGetNextLeaf(OID_INFO* rec);
bool SNMPIdRecrdValidation(PDU_INFO * pduPtr,OID_INFO *var,uint8_t * oidValuePtr,uint8_t oidLen);
bool SNMPFindOIDStringByID(SNMP_ID id, OID_INFO* info, uint8_t* oidString, uint8_t* len);
bool SNMPProcessReqBuildRespnsDuplexInit(UDP_SOCKET socket);
bool SNMPGetDataTypeInfo(SNMP_DATA_TYPE dataType, SNMP_DATA_TYPE_INFO *info );


uint8_t SNMPProcessGetVar(OID_INFO* rec, bool bAsOID,PDU_INFO* pduDbPtr);
uint8_t SNMPProcessGetNextVar(OID_INFO* rec,PDU_INFO* pduDbPtr);
uint8_t SNMPProcessSetVar(PDU_INFO* pduDbPtr,OID_INFO* rec, SNMP_ERR_STATUS* errorStatus);
uint8_t SNMPProcessGetBulkVar(OID_INFO* rec, uint8_t* oidValuePtr, uint8_t* oidLenPtr,uint8_t* successor,PDU_INFO* pduDbPtr);
uint8_t SNMPGetByteFromUDPRxBuff(UDP_SOCKET skt);
uint8_t SNMPCheckIfValidLength(uint16_t *len);
uint8_t SNMPCheckIfValidSnmpStructure(uint16_t* dataLen);
uint8_t SNMPSearchOIDInMgmtInfoBase(PDU_INFO* pduDbPtr,uint8_t* oid, uint8_t oidLen, OID_INFO* rec);


void SNMPPutByteToUDPTxBuffer(UDP_SOCKET socket,uint8_t v);

#endif 

