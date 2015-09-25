/*******************************************************************************
  File Transfer Protocol (FTP) Client

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    - Provides ability to remotely upload MPFS2 image (web pages) 
      to external EEPROM or external Flash memory
    - Reference: RFC 959
*******************************************************************************/

/*******************************************************************************
FileName:   FTP.c
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

#define __FTP_C

#include "tcpip_private.h"
#include "ftp_private.h"



#if defined(TCPIP_STACK_USE_FTP_SERVER)

#define TCPIP_THIS_MODULE_ID    TCPIP_MODULE_FTP_SERVER


// Each entry in following table must match with that of FTP_COMMAND enum.
static const char * const sTCPIPFTPCmdString[] =
{
    "USER",                         // TCPIP_FTP_CMD_USER
    "PASS",                         // TCPIP_FTP_CMD_PASS
    "QUIT",                         // TCPIP_FTP_CMD_QUIT
    "STOR",                         // TCPIP_FTP_CMD_STOR
    "PORT",                         // TCPIP_FTP_CMD_PORT
    "ABOR",                         // TCPIP_FTP_CMD_ABORT
    "PWD ",                         // TCPIP_FTP_CMD_PWD
    "CWD ",                         // TCPIP_FTP_CMD_CWD
    "TYPE",                         // TCPIP_FTP_CMD_TYPE
    "RETR",							// TCPIP_FTP_CMD_RETR // GET command
    "SIZE",							// TCPIP_FTP_CMD_SIZE
    "PASV",							//TCPIP_FTP_CMD_PASSIVE
    "NLST",							// TCPIP_FTP_CMD_NAME_LIST  // MGET command
    "EPSV",							// TCPIP_FTP_CMD_EXTND_PASSIVE
    "EPRT",							// TCPIP_FTP_CMD_EXTND_PORT
	"LIST",							// TCPIP_FTP_CMD_EXTND_LIST
};
#define TCPIP_FTP_CMD_TBL_SIZE  ( sizeof(sTCPIPFTPCmdString)/sizeof(sTCPIPFTPCmdString[0]) )



// Each entry in following table must match with TCPIP_FTP_RESP enum
static const char * const sTCPIPFTPRespStr[] =
{
    "220 Ready\r\n",                    // TCPIP_FTP_RESP_BANNER
    "331 Password required\r\n",        // TCPIP_FTP_RESP_USER_OK
    "230 Logged in\r\n",                // TCPIP_FTP_RESP_PASS_OK
    "221 Bye\r\n",                      // TCPIP_FTP_RESP_QUIT_OK
    "500 \r\n",                         // TCPIP_FTP_RESP_STOR_OK
    "502 Not implemented\r\n",          // TCPIP_FTP_RESP_UNKNOWN
    "530 Login required\r\n",           // TCPIP_FTP_RESP_LOGIN
    "150 Transferring data...\r\n",     // TCPIP_FTP_RESP_DATA_OPEN
    "125 File status okay; about to open data connection\r\n",                    	// TCPIP_FTP_RESP_DATA_READY
    "226 Transfer Complete\r\n",        // TCPIP_FTP_RESP_DATA_CLOSE
	"425 Can't create data socket.\r\n",// TCPIP_FTP_RESP_DATA_NO_SOCKET
	"257 \"/\" is current\r\n",         // TCPIP_FTP_RESP_PWD
    "200 Ok\r\n",                        // TCPIP_FTP_RESP_OK
    "550 Requested action not taken. File not found\r\n",			// TCPIP_FTP_RESP_FILE_NOT_EXIST
    "150 File status okay; about to open data connection\r\n", //TCPIP_FTP_RESP_FILE_IS_PRESENT
    "227 Entering passive mode ", // TCPIP_FTP_RESP_ENTER_PASV_MODE
    "226 Closing data connection. Requested file action successful.\r\n", //TCPIP_FTP_RESP_FILE_ACTION_SUCCESSFUL_CLOSING_DATA_CONNECTION,
    "213 ",
    "522 Extended PORT failure - Unknown network protocol\r\n", //TCPIP_FTP_RESP_EXTND_PORT_FAILURE
    "",  // TCPIP_FTP_RESP_NONE 
};




static TCPIP_FTP_DCPT* sTCPIPFTPDcpt = 0;

//static int32_t 			sTCPIPFTPFileHandle;
static uint8_t 			sTCPIPFTPServerCount=0;
static const void 		*sTCPIPFtpMemH = 0;  // memory handle

// Private helper functions.
static TCPIP_FTP_CMD TCPIP_FTP_Parse_Cmds(uint8_t *cmd);
static void TCPIP_FTP_Parse_CmdString(TCPIP_FTP_DCPT* pFTPDcpt);
static bool TCPIP_FTP_Execute_Cmds(TCPIP_FTP_CMD cmd, TCPIP_FTP_DCPT* pFTPDcpt);
static bool TCPIP_FTP_PutFile(TCPIP_FTP_DCPT* pFTPDcpt);
static bool TCPIP_FTP_Quit(TCPIP_FTP_DCPT* pFTPDcpt);
static bool TCPIP_FTP_GetFile(TCPIP_FTP_DCPT* pFTPDcpt, uint8_t *cFile);
static bool TCPIP_FTP_Execute_Get_Cmd(TCPIP_FTP_DCPT* pFTPDcpt, uint8_t *cFile);
static bool TCPIP_FTP_NameListCmd(TCPIP_FTP_DCPT* pFTPDcpt, uint8_t *cFile);
static bool TCPIP_FTP_LISTCmd(TCPIP_FTP_DCPT* pFTPDcpt);

#define TCPIP_FTP_PUT_ENABLED
#define mMIN(a, b)	((a<b)?a:b)


static const char TCPIP_FTP_USER_NAME[]    = "admin";
static const char TCPIP_FTP_USER_PASS[]    = "microchip";
static const char TCPIP_FTP_ANNONYMOUS_USER_NAME[]    = "anonymous";

/*********************************************************************
 * Function:        bool TCPIP_FTP_Verify(uint8_t* login,uint8_t* password)
 *
 * PreCondition:    TCP module is already initialized.
 *
 * Input:           None
 *
 * Output:          true is success
 *                  false otherwise
 *
 * Side Effects:    None
 *
 * Overview:        Initializes internal variables of FTP
 *
 * Note:
 ********************************************************************/

bool TCPIP_FTP_Verify(uint8_t* login,uint8_t* password)
{
    if ( strncmp((const char*)login, (const char*)TCPIP_FTP_ANNONYMOUS_USER_NAME, TCPIP_FTP_USER_NAME_LEN) == 0 )
		return true;

    if ( strncmp((const char*)login, (const char*)TCPIP_FTP_USER_NAME, TCPIP_FTP_USER_NAME_LEN) == 0 )
    {
        if ( strncmp((const char*)password, (const char*)TCPIP_FTP_USER_PASS, TCPIP_FTP_PASSWD_LEN) == 0)
            return true;
    }
    return false;
}
/*********************************************************************
 * Function:        bool TCPIP_FTP_ServerInit(const TCPIP_STACK_MODULE_CTRL* const stackData,
             const FTP_MODULE_GONFIG* ftpData)
 *
 * PreCondition:    TCP module is already initialized.
 *
 * Input:           None
 *
 * Output:          true is success
 *                  false otherwise
 *
 * Side Effects:    None
 *
 * Overview:        Initializes internal variables of FTP
 *
 * Note:
 ********************************************************************/
bool TCPIP_FTP_ServerInit(const TCPIP_STACK_MODULE_CTRL* const stackData,
             const FTP_MODULE_GONFIG* ftpData)
{
	TCPIP_FTP_DCPT *pDcpt = 0;

        if(stackData->stackAction == TCPIP_STACK_ACTION_IF_UP)
	{   // interface restart
	   return true;
	}

    // Check to see whether we have been brought up previously. If so, we will move on.
    if(sTCPIPFTPServerCount == 0)
    {
    	sTCPIPFtpMemH = stackData->memH;
        
        sTCPIPFTPDcpt = (TCPIP_FTP_DCPT*)TCPIP_HEAP_Calloc(sTCPIPFtpMemH,  stackData->nIfs, sizeof(TCPIP_FTP_DCPT));
        if(sTCPIPFTPDcpt == 0)
        {   // failed
            return false;
        }		
    }

	
	pDcpt = sTCPIPFTPDcpt+stackData->netIx;
	sTCPIPFTPDcpt[stackData->netIx].ftpCmdskt = TCPOpenServer(IP_ADDRESS_TYPE_IPV4, TCPIP_FTP_COMMAND_PORT,0);
	pDcpt->ftpFlag.val = 0;
	pDcpt->ftpDataPort = 0;
	pDcpt->ftpStringLen = 0;
	pDcpt->ftpDataskt = INVALID_SOCKET;
	pDcpt->ftpSm = TCPIP_FTP_SM_NOT_CONNECTED;
	pDcpt->ftpCommand = TCPIP_FTP_CMD_NONE;
	pDcpt->ftpCommandSm = TCPIP_FTP_CMD_SM_IDLE;
	pDcpt->callbackPos = 0x00u;

	sTCPIPFTPServerCount++;

    return true;
}

/*********************************************************************
 * Function:        void TCPIP_FTP_DeInit(const TCPIP_STACK_MODULE_CTRL* const stackData)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        DeInitializes internal variables of FTP
 *
 * Note:
 ********************************************************************/
void TCPIP_FTP_DeInit(const TCPIP_STACK_MODULE_CTRL* const stackData)
{
    // if(stackInit->stackAction == TCPIP_STACK_ACTION_DEINIT) // stack shut down
    // if(stackInit->stackAction == TCPIP_STACK_ACTION_IF_DOWN) // interface down
	int nIntf=0;
	
	if(stackData->stackAction == TCPIP_STACK_ACTION_DEINIT)
    {   // whole stack is going down
        if(sTCPIPFTPServerCount > 0)
        {   // we're up and running
            if(--sTCPIPFTPServerCount == 0)
            {   // all closed
                // release resources
                if(sTCPIPFTPDcpt != 0)
            	{   
			        TCPIP_HEAP_Free(sTCPIPFtpMemH, sTCPIPFTPDcpt);
			        sTCPIPFTPDcpt = 0;
			    }
                sTCPIPFtpMemH = 0;
            }
        }
    }
	
	for(nIntf=0;nIntf<stackData->nIfs;nIntf++)
	{
		TCPClose(sTCPIPFTPDcpt[nIntf].ftpCmdskt);
	}
}

/*********************************************************************
 * Function:        bool TCPIP_FTP_Server(TCPIP_NET_IF * pNetIf)
 *
 * PreCondition:    TCPIP_FTP_ServerInit must already be called.
 *
 * Input:           None
 *
 * Output:          Opened FTP connections are served.
 *
 * Side Effects:    None
 *
 * Overview:
 *
 * Note:            This function acts as a task (similar to one in
 *                  RTOS).  This function performs its task in
 *                  co-operative manner.  Main application must call
 *                  this function repeatdly to ensure all open
 *                  or new connections are served on time.
 ********************************************************************/
bool TCPIP_FTP_Server(TCPIP_NET_IF * pNetIf)
{
    uint8_t 			v;
    SYS_TICK 			currentTick;
	TCPIP_FTP_DCPT*     pFTPDcpt;

    if(!pNetIf)
    {
        return false;
    }

    pFTPDcpt = sTCPIPFTPDcpt + _TCPIPStackNetIx(pNetIf);
	
	if(!TCPIsConnected(pFTPDcpt->ftpCmdskt) )
	{
	   pFTPDcpt->ftpSm = TCPIP_FTP_SM_NOT_CONNECTED;
	   pFTPDcpt->ftpCommand = TCPIP_FTP_CMD_NONE;
	   pFTPDcpt->ftpFlag.val    = 0;
	   pFTPDcpt->ftpCommandSm = TCPIP_FTP_CMD_SM_IDLE;
	   if(pFTPDcpt->ftpDataskt != INVALID_SOCKET)
	   {
		   TCPClose(pFTPDcpt->ftpDataskt);
		   pFTPDcpt->ftpDataskt = INVALID_SOCKET;
	   }
	   return true;
	}

	if(TCPIsGetReady(pFTPDcpt->ftpCmdskt) )
	{
	   pFTPDcpt->ftpSysTicklastActivity = SYS_TICK_Get();

	   while( TCPGet(pFTPDcpt->ftpCmdskt, &v ) )
	   {
		   pFTPDcpt->ftpCmdString[pFTPDcpt->ftpStringLen++] = v;
		   if(pFTPDcpt->ftpStringLen == TCPIP_FTP_CMD_MAX_STRING_LEN)
			   pFTPDcpt->ftpStringLen = 0;
	   }


	   if ( v == '\n' )
	   {
		   pFTPDcpt->ftpCmdString[pFTPDcpt->ftpStringLen] = '\0';
		   pFTPDcpt->ftpStringLen = 0;
		   TCPIP_FTP_Parse_CmdString(pFTPDcpt);
		   pFTPDcpt->ftpCommand = TCPIP_FTP_Parse_Cmds(pFTPDcpt->ftp_argv[0]);
	   }
	}
	else if ( pFTPDcpt->ftpSm != TCPIP_FTP_SM_NOT_CONNECTED )
	{
	   currentTick = SYS_TICK_Get();
	   currentTick = currentTick - pFTPDcpt->ftpSysTicklastActivity;
	   if ( currentTick >= (TCPIP_FTP_TIMEOUT * SYS_TICK_TicksPerSecondGet()) )
	   {
		   pFTPDcpt->ftpSysTicklastActivity	   = SYS_TICK_Get();
		   pFTPDcpt->ftpCommand				   = TCPIP_FTP_CMD_QUIT;
		   pFTPDcpt->ftpSm					   = TCPIP_FTP_SM_CONNECTED;
	   }
	}
	switch(pFTPDcpt->ftpSm)
	{
		case TCPIP_FTP_SM_HOME:
		//	pFTPDcpt->ftpCmdskt = TCPOpenServer(IP_ADDRESS_TYPE_IPV4, TCPIP_FTP_CMD_PORT,0);
		   
    //No break statement
	    case TCPIP_FTP_SM_NOT_CONNECTED:
	        pFTPDcpt->ftpResponse = TCPIP_FTP_RESP_BANNER;
	        pFTPDcpt->ftpSysTicklastActivity = SYS_TICK_Get();
	        // No break - Continue...

	    case TCPIP_FTP_SM_RESPOND:
			// Make sure there is enough TCP TX FIFO space to put our response
	        if(TCPIsPutReady(pFTPDcpt->ftpCmdskt) < strlen(sTCPIPFTPRespStr[pFTPDcpt->ftpResponse]))
	        {
				return true;
	        }

			TCPPutString(pFTPDcpt->ftpCmdskt, (const uint8_t*)sTCPIPFTPRespStr[pFTPDcpt->ftpResponse]);
			TCPFlush(pFTPDcpt->ftpCmdskt);
			pFTPDcpt->ftpResponse = TCPIP_FTP_RESP_NONE;
			pFTPDcpt->ftpSm = TCPIP_FTP_SM_CONNECTED;
	        // No break - this will speed up little bit

	    case TCPIP_FTP_SM_CONNECTED:
	        if ( pFTPDcpt->ftpCommand != TCPIP_FTP_CMD_NONE )
	        {
	            if ( TCPIP_FTP_Execute_Cmds(pFTPDcpt->ftpCommand, pFTPDcpt) )
	            {
	                if ( pFTPDcpt->ftpResponse != TCPIP_FTP_RESP_NONE )
	                    pFTPDcpt->ftpSm = TCPIP_FTP_SM_RESPOND;
	                else if ( pFTPDcpt->ftpCommand == TCPIP_FTP_CMD_QUIT )
	                    pFTPDcpt->ftpSm = TCPIP_FTP_SM_NOT_CONNECTED;

	                pFTPDcpt->ftpCommand = TCPIP_FTP_CMD_NONE;
	                pFTPDcpt->ftpCommandSm = TCPIP_FTP_CMD_SM_IDLE;
	            }
	            else if ( pFTPDcpt->ftpResponse != TCPIP_FTP_RESP_NONE )
	            {
	                pFTPDcpt->ftpSm = TCPIP_FTP_SM_RESPOND;
	            }
	        }
	        break;
		case TCPIP_FTP_SM_USER_NAME:
		case TCPIP_FTP_SM_USER_PASS:
			break;

    }

    return true;
}

/*****************************************************************************
  Function:
	static bool TCPIP_FTP_Execute_Cmds(TCPIP_FTP_CMD cmd, TCPIP_FTP_DCPT* pFTPDcpt)

  Summary:
	Execute different FTP commands 

  Description:
	Execute different FTP commands . Like UserName,Password,QUIT,GET,MGET. But PUT command 
	is in place. It is not tested. FTp works on both Active and Passive mode.
  Precondition:
	None

  Parameters:
	cmd - FTP Command
	pFTPDcpt - FTP descriptor
  Returns:
  	true or false
  	
  Remarks:
	Users should not call this function directly.
  ***************************************************************************/
static bool TCPIP_FTP_Execute_Cmds(TCPIP_FTP_CMD cmd, TCPIP_FTP_DCPT* pFTPDcpt)
{
	
	int32_t		fp;
	TCP_SOCKET_INFO remoteSockInfo,dataSockInfo;
	char passiveMsg[64];
	uint32_t fileSize=0;
	char tempMsg[10];
	
    switch(cmd)
    {
	    case TCPIP_FTP_CMD_USER:
	        pFTPDcpt->ftpFlag.Bits.userSupplied = true;
	        pFTPDcpt->ftpFlag.Bits.loggedIn = false;
	        pFTPDcpt->ftpResponse = TCPIP_FTP_RESP_USER_OK;
            if(pFTPDcpt->ftp_argv[1] != 0)
            {
                strncpy((char*)pFTPDcpt->ftpUserName, (char*)pFTPDcpt->ftp_argv[1], TCPIP_FTP_USER_NAME_LEN);
            }
            else
            {
                pFTPDcpt->ftpUserName[0] = '\0';    // empty user name
            }
	        break;

	    case TCPIP_FTP_CMD_PASS:
	        if ( !pFTPDcpt->ftpFlag.Bits.userSupplied )
	            pFTPDcpt->ftpResponse = TCPIP_FTP_RESP_LOGIN;
	        else
	        {
	            if(TCPIP_FTP_Verify(pFTPDcpt->ftpUserName, pFTPDcpt->ftp_argv[1]))
	            {
	            	pFTPDcpt->ftpFlag.Bits.loggedIn = true;
	                pFTPDcpt->ftpResponse = TCPIP_FTP_RESP_PASS_OK;
	            }
	            else
	                pFTPDcpt->ftpResponse = TCPIP_FTP_RESP_LOGIN;
	        }
	        break;

	    case TCPIP_FTP_CMD_QUIT:
	        return TCPIP_FTP_Quit(pFTPDcpt);

	    case TCPIP_FTP_CMD_PORT:
	        pFTPDcpt->ftpDataPort = (uint8_t)atoi((char*)pFTPDcpt->ftp_argv[5]);
	        pFTPDcpt->ftpDataPort = 
				(pFTPDcpt->ftpDataPort << 8)|(uint8_t)atoi((char*)pFTPDcpt->ftp_argv[6]);
	        pFTPDcpt->ftpResponse = TCPIP_FTP_RESP_OK;
	        break;

	    case TCPIP_FTP_CMD_STOR:
	        return TCPIP_FTP_PutFile(pFTPDcpt);
	        
	    case TCPIP_FTP_CMD_PWD:
	    	pFTPDcpt->ftpResponse = TCPIP_FTP_RESP_PWD;
	    	break;

	    case TCPIP_FTP_CMD_CWD:
	    case TCPIP_FTP_CMD_TYPE:
	       	pFTPDcpt->ftpResponse = TCPIP_FTP_RESP_OK;
	    	break;

	    case TCPIP_FTP_CMD_ABORT:
	        pFTPDcpt->ftpResponse = TCPIP_FTP_RESP_OK;
	        if ( pFTPDcpt->ftpDataskt!= INVALID_SOCKET )
			{
				TCPClose(pFTPDcpt->ftpDataskt);
				pFTPDcpt->ftpDataskt = INVALID_SOCKET;
			}
	        break;
		case TCPIP_FTP_CMD_EXTND_LIST:
			return TCPIP_FTP_LISTCmd(pFTPDcpt);
			break;
		case TCPIP_FTP_CMD_RETR:
			
			if(pFTPDcpt->callbackPos == 0x00u)
			{
				fp = SYS_FS_open((const char*)pFTPDcpt->ftp_argv[1],0);

				if(fp == RETURN_FAILED)
				{// File not found, so abort
					pFTPDcpt->ftpResponse = TCPIP_FTP_RESP_FILE_NOT_EXIST;
					return true;
				}	
				SYS_FS_close(fp);
			}
			return TCPIP_FTP_Execute_Get_Cmd(pFTPDcpt, pFTPDcpt->ftp_argv[1]);
			
			break;
		case TCPIP_FTP_CMD_SIZE:			
			fp = SYS_FS_open((const char*)pFTPDcpt->ftp_argv[1],0);
			if(fp == RETURN_FAILED)
			{// File not found, so abort
				pFTPDcpt->ftpResponse = TCPIP_FTP_RESP_FILE_NOT_EXIST;
				return true;
			}
			pFTPDcpt->fileDescr = fp;
			fileSize = SYS_FS_fsize(fp);
			strcpy(tempMsg,"");			
			sprintf(tempMsg,"213 %u\r\n",fileSize);	
			
	        if(TCPIsPutReady(pFTPDcpt->ftpCmdskt) < strlen(tempMsg))
	        {
				pFTPDcpt->ftpResponse = TCPIP_FTP_RESP_NONE;				
				pFTPDcpt->ftpSm = TCPIP_FTP_SM_CONNECTED;
				SYS_FS_close(pFTPDcpt->fileDescr);
				return true;
	        }
			TCPPutString(pFTPDcpt->ftpCmdskt, (const uint8_t*)tempMsg);
			TCPFlush(pFTPDcpt->ftpCmdskt);
			pFTPDcpt->ftpResponse = TCPIP_FTP_RESP_NONE;
			pFTPDcpt->ftpSm = TCPIP_FTP_SM_CONNECTED;
			SYS_FS_close(fp);
			break;
			
		case TCPIP_FTP_CMD_PASV:			
			// create a server socket with a available port number and send this port number to the client with Response string.
			pFTPDcpt->ftpDataskt = TCPOpenServer(IP_ADDRESS_TYPE_IPV4, 0,0);
			// Make sure that a valid socket was available and returned
			// If not, return with an error
			if(pFTPDcpt->ftpDataskt == INVALID_SOCKET)
			{
	            pFTPDcpt->ftpResponse = TCPIP_FTP_RESP_DATA_NO_SOCKET;
	            return true;
			}
					
			// modify the response string.
			//response string should have server ip address and the new data port number.
			
			TCPGetSocketInfo(pFTPDcpt->ftpCmdskt, &remoteSockInfo);

			TCPGetSocketInfo(pFTPDcpt->ftpDataskt, &dataSockInfo);
			//prepare addtional message IPaddress + Port
			strcpy(passiveMsg,"");
			// add IPv4 address
			sprintf(passiveMsg,"227 Entering passive mode (%u,%u,%u,%u,%u,%u)\r\n",remoteSockInfo.localIPaddress.v4Add.v[0],
									remoteSockInfo.localIPaddress.v4Add.v[1],
									remoteSockInfo.localIPaddress.v4Add.v[2],
									remoteSockInfo.localIPaddress.v4Add.v[3],
									dataSockInfo.localPort>>8 & 0xFF,
									dataSockInfo.localPort & 0xFF);
			//strcat(&sTCPIPFTPRespStr[pFTPDcpt->ftpResponse],passiveMsg);
			pFTPDcpt->ftpDataPort = dataSockInfo.localPort;
			
	        if(TCPIsPutReady(pFTPDcpt->ftpCmdskt) < strlen(passiveMsg))
	        {
				pFTPDcpt->ftpResponse = TCPIP_FTP_RESP_NONE;				
				pFTPDcpt->ftpSm = TCPIP_FTP_SM_CONNECTED;
				return true;
	        }
			TCPPutString(pFTPDcpt->ftpCmdskt, (const uint8_t*)passiveMsg);
			TCPFlush(pFTPDcpt->ftpCmdskt);
			pFTPDcpt->ftpResponse = TCPIP_FTP_RESP_NONE;
			pFTPDcpt->ftpSm = TCPIP_FTP_SM_CONNECTED;
			pFTPDcpt->ftpFlag.Bits.pasvMode = true;
			break;
		case TCPIP_FTP_CMD_EPSV:
			// RFC 2428 , EPSV<space><net-prt>  or EPSV<space>ALL nned to be handled properly.
			// for Time being If there is any argument return Error code 522. dont start any data communication.
			if(pFTPDcpt->ftp_argc > 1)
			{
				pFTPDcpt->ftpResponse = TCPIP_FTP_RESP_EXTND_PORT_FAILURE;
				return true;
			}
			// create a server socket with a available port number and send this port number to the client with Response string.
			pFTPDcpt->ftpDataskt = TCPOpenServer(IP_ADDRESS_TYPE_IPV4, 0,0);
			// Make sure that a valid socket was available and returned
			// If not, return with an error
			if(pFTPDcpt->ftpDataskt == INVALID_SOCKET)
			{
	            pFTPDcpt->ftpResponse = TCPIP_FTP_RESP_DATA_NO_SOCKET;
	            return true;
			}
					
			// modify the response string.
			//response string should have server ip address and the new data port number.
			
			TCPGetSocketInfo(pFTPDcpt->ftpCmdskt, &remoteSockInfo);

			TCPGetSocketInfo(pFTPDcpt->ftpDataskt, &dataSockInfo);
			//prepare addtional message IPaddress + Port
			strcpy(passiveMsg,"");

			
			sprintf(passiveMsg,"229 Extended passive mode entered (|||%u|)\r\n",dataSockInfo.localPort);
			pFTPDcpt->ftpDataPort = dataSockInfo.localPort;
			
	        if(TCPIsPutReady(pFTPDcpt->ftpCmdskt) < strlen(passiveMsg))
	        {
				pFTPDcpt->ftpResponse = TCPIP_FTP_RESP_NONE;				
				pFTPDcpt->ftpSm = TCPIP_FTP_SM_CONNECTED;
				return true;
	        }
			TCPPutString(pFTPDcpt->ftpCmdskt, (const uint8_t*)passiveMsg);
			TCPFlush(pFTPDcpt->ftpCmdskt);
			pFTPDcpt->ftpResponse = TCPIP_FTP_RESP_NONE;
			pFTPDcpt->ftpSm = TCPIP_FTP_SM_CONNECTED;
			pFTPDcpt->ftpFlag.Bits.pasvMode = true;
			break;
		case TCPIP_FTP_CMD_EPRT:
			pFTPDcpt->adressFamilyProtocol = atoi((char *)pFTPDcpt->ftp_argv[1]);
	        pFTPDcpt->ftpDataPort = atoi((char *)pFTPDcpt->ftp_argv[3]);
			pFTPDcpt->ftpResponse = TCPIP_FTP_RESP_OK;
			break;
		case TCPIP_FTP_CMD_NLST:
			return TCPIP_FTP_NameListCmd(pFTPDcpt, pFTPDcpt->ftp_argv[1]);
			break;
	    default:
	        pFTPDcpt->ftpResponse = TCPIP_FTP_RESP_UNKNOWN;
			
	        break;
    }
    return true;
}

/*****************************************************************************
  Function:
	static bool TCPIP_FTP_Quit(TCPIP_FTP_DCPT* pFTPDcpt)

  Summary:
	Execute FTP QUIT command.

  Description:
	Execute FTP QUIT command.
  Precondition:
	None

  Parameters:
	pFTPDcpt - FTP descriptor
  Returns:
  	true or false
  	
  Remarks:
	Users should not call this function directly.
  ***************************************************************************/
static bool TCPIP_FTP_Quit(TCPIP_FTP_DCPT* pFTPDcpt)
{
    switch(pFTPDcpt->ftpCommandSm)
    {
    case TCPIP_FTP_CMD_SM_IDLE:
#if defined(TCPIP_FTP_PUT_ENABLED)
        if ( pFTPDcpt->ftpCommandSm == TCPIP_FTP_CMD_SM_RECEIVE )
        {
            SYS_FS_close(pFTPDcpt->fileDescr);
        }
#endif

        if ( pFTPDcpt->ftpDataskt != INVALID_SOCKET )
        {
#if defined(TCPIP_FTP_PUT_ENABLED)
			SYS_FS_close(pFTPDcpt->fileDescr);
#endif
            TCPClose(pFTPDcpt->ftpDataskt);
            pFTPDcpt->ftpCommandSm = TCPIP_FTP_CMD_SM_WAIT;
        }
        else
        {
            pFTPDcpt->ftpResponse = TCPIP_FTP_RESP_QUIT_OK;
            pFTPDcpt->ftpCommandSm = TCPIP_FTP_CMD_SM_WAIT_FOR_DISCONNECT;
        }
        break;

    case TCPIP_FTP_CMD_SM_WAIT:
        if ( !TCPIsConnected(pFTPDcpt->ftpDataskt) )
        {
            pFTPDcpt->ftpResponse = TCPIP_FTP_RESP_QUIT_OK;
            pFTPDcpt->ftpCommandSm = TCPIP_FTP_CMD_SM_WAIT_FOR_DISCONNECT;
        }
        break;

    case TCPIP_FTP_CMD_SM_WAIT_FOR_DISCONNECT:
        if ( TCPIsPutReady(pFTPDcpt->ftpCmdskt) )
        {
            if ( TCPIsConnected(pFTPDcpt->ftpCmdskt) )
                TCPDisconnect(pFTPDcpt->ftpCmdskt);
        }
        break;
	case TCPIP_FTP_CMD_SM_RECEIVE:
    case TCPIP_FTP_CMD_SM_SEND:

		break;

    }
    return false;
}

/*****************************************************************************
  Function:
	static bool TCPIP_FTP_PutFile(TCPIP_FTP_DCPT* pFTPDcpt)

  Summary:
	Execute FTP PUT command.

  Description:
	Execute FTP PUT command.
  Precondition:
	None

  Parameters:
	pFTPDcpt - FTP descriptor
  Returns:
  	true or false
  	
  Remarks:
	Users should not call this function directly.
  ***************************************************************************/
static bool TCPIP_FTP_PutFile(TCPIP_FTP_DCPT* pFTPDcpt)
{
    uint8_t v;
	TCP_SOCKET_INFO remoteSockInfo;

    switch(pFTPDcpt->ftpCommandSm)
    {
    case TCPIP_FTP_CMD_SM_IDLE:
        if ( !pFTPDcpt->ftpFlag.Bits.loggedIn )
        {
            pFTPDcpt->ftpResponse     = TCPIP_FTP_RESP_LOGIN;
            return true;
        }
        else
        {
        
       		if(pFTPDcpt->ftpFlag.Bits.pasvMode != true)
			{
				TCPGetSocketInfo(pFTPDcpt->ftpCmdskt, &remoteSockInfo);
	            pFTPDcpt->ftpDataskt   = 
					TCPOpenClient(IP_ADDRESS_TYPE_IPV4,pFTPDcpt->ftpDataPort,&remoteSockInfo.remoteIPaddress);

				// Make sure that a valid socket was available and returned
				// If not, return with an error
				if(pFTPDcpt->ftpDataskt == INVALID_SOCKET)
				{
		            pFTPDcpt->ftpResponse = TCPIP_FTP_RESP_DATA_NO_SOCKET;
		            return true;
				}
				//TCPModifySrcPort(pFTPDcpt->ftpDataskt,TCPIP_FTP_DATA_PORT);
				if(TCPBind(pFTPDcpt->ftpDataskt,IP_ADDRESS_TYPE_IPV4,TCPIP_FTP_DATA_PORT,&remoteSockInfo.localIPaddress) == false)
				{
		            pFTPDcpt->ftpResponse = TCPIP_FTP_RESP_DATA_NO_SOCKET;
					TCPClose(pFTPDcpt->ftpDataskt);
		            return true;
				} 
			}			
			pFTPDcpt->ftpResponse = TCPIP_FTP_RESP_FILE_IS_PRESENT;
			pFTPDcpt->ftpCommandSm = TCPIP_FTP_CMD_SM_WAIT;
        }
        break;

    case TCPIP_FTP_CMD_SM_WAIT:
        if ( TCPIsConnected(pFTPDcpt->ftpDataskt) )
        {
#if defined(TCPIP_FTP_PUT_ENABLED)
			//sTCPIPFTPFileHandle   = MPFSFormat();
#endif

			pFTPDcpt->ftpCommandSm    = TCPIP_FTP_CMD_SM_RECEIVE;
        }
        break;

    case TCPIP_FTP_CMD_SM_RECEIVE:
        if ( TCPIsGetReady(pFTPDcpt->ftpDataskt) )
        {
            // Reload timeout timer.
            pFTPDcpt->ftpSysTicklastActivity   = SYS_TICK_Get();
            while( TCPGet(pFTPDcpt->ftpDataskt, &v) )
            {
#if defined(TCPIP_FTP_PUT_ENABLED)
          //      MPFSPutArray(sTCPIPFTPFileHandle,&v,1);
          		SYS_CONSOLE_PRINT("%c",v);
#endif
            }
          //  MPFSPutEnd(true);
          SYS_CONSOLE_PRINT("MPFS END",0);

        }
        else if ( !TCPIsConnected(pFTPDcpt->ftpDataskt) )
        {
#if defined(TCPIP_FTP_PUT_ENABLED)
            SYS_FS_close(pFTPDcpt->fileDescr);
#endif
            TCPClose(pFTPDcpt->ftpDataskt);
            pFTPDcpt->ftpDataskt   = INVALID_SOCKET;
            pFTPDcpt->ftpResponse = TCPIP_FTP_RESP_DATA_CLOSE;
            return true;
        }
	case TCPIP_FTP_CMD_SM_WAIT_FOR_DISCONNECT:
	case TCPIP_FTP_CMD_SM_SEND:
		break;
    }
    return false;
}

/*****************************************************************************
  Function:
	static bool TCPIP_FTP_LISTCmd(TCPIP_FTP_DCPT* pFTPDcpt)

  Summary:
	Execute FTP LIST command.

  Description:
	Execute FTP LIST command.
  Precondition:
	None

  Parameters:
	pFTPDcpt - FTP descriptor
  Returns:
  	true or false
  	
  Remarks:
	Users should not call this function directly.
  ***************************************************************************/
static bool TCPIP_FTP_LISTCmd(TCPIP_FTP_DCPT* pFTPDcpt)
{
	uint16_t wCount=0;
	TCP_SOCKET_INFO remoteSockInfo;
	uint8_t fileNameList[100];

	switch(pFTPDcpt->ftpCommandSm)
	{
		case TCPIP_FTP_CMD_SM_IDLE:
			if(pFTPDcpt->ftpFlag.Bits.pasvMode != true)
			{
				//pFTPDcpt->ftpResponse     = TCPIP_FTP_RESP_DATA_OPEN;
				TCPGetSocketInfo(pFTPDcpt->ftpCmdskt, &remoteSockInfo);
	            pFTPDcpt->ftpDataskt   = 
					TCPOpenClient(IP_ADDRESS_TYPE_IPV4,pFTPDcpt->ftpDataPort,&remoteSockInfo.remoteIPaddress);

				// Make sure that a valid socket was available and returned
				// If not, return with an error
				if(pFTPDcpt->ftpDataskt == INVALID_SOCKET)
				{
		            pFTPDcpt->ftpResponse = TCPIP_FTP_RESP_DATA_NO_SOCKET;
		            return true;
				}
				if(TCPBind(pFTPDcpt->ftpDataskt,IP_ADDRESS_TYPE_IPV4,TCPIP_FTP_DATA_PORT,&remoteSockInfo.localIPaddress) == false)
				{
		            pFTPDcpt->ftpResponse = TCPIP_FTP_RESP_DATA_NO_SOCKET;					
					TCPClose(pFTPDcpt->ftpDataskt);
		            return true;
				} 
			}
				
			// check login 
			if ( !pFTPDcpt->ftpFlag.Bits.loggedIn )
			{
				pFTPDcpt->ftpResponse	 = TCPIP_FTP_RESP_LOGIN;
				TCPClose(pFTPDcpt->ftpDataskt);
				return true;
			}
			
			pFTPDcpt->ftpResponse = TCPIP_FTP_RESP_DATA_READY;
			pFTPDcpt->ftpCommandSm = TCPIP_FTP_CMD_SM_WAIT;
			break;
		case TCPIP_FTP_CMD_SM_WAIT:
			if ( TCPIsConnected(pFTPDcpt->ftpDataskt) )
			{
				pFTPDcpt->ftpCommandSm	 = TCPIP_FTP_CMD_SM_SEND;
			}
			break;
		case TCPIP_FTP_CMD_SM_SEND:
			wCount = TCPIsPutReady(pFTPDcpt->ftpDataskt);
			strcpy((char*)fileNameList,(char*)"");
			sprintf((char*)fileNameList,"List File System is Not working.\r\n");
			if(wCount > strlen((char *)fileNameList))
			{
				TCPPutArray(pFTPDcpt->ftpDataskt, fileNameList, 
										strlen((char *)fileNameList));
			}
			
			pFTPDcpt->ftpCommandSm	= TCPIP_FTP_CMD_SM_WAIT_FOR_DISCONNECT;
			break;

		// to avoid warning
		case TCPIP_FTP_CMD_SM_WAIT_FOR_DISCONNECT:			
			TCPClose(pFTPDcpt->ftpDataskt);
			pFTPDcpt->ftpResponse 	= TCPIP_FTP_RESP_FILE_ACTION_SUCCESSFUL_CLOSING_DATA_CONNECTION;			
            pFTPDcpt->ftpDataskt   = INVALID_SOCKET;
			return true;
		case TCPIP_FTP_CMD_SM_RECEIVE:
			break;
	}

	return false;

}

/*****************************************************************************
  Function:
	static bool TCPIP_FTP_NameListCmd(TCPIP_FTP_DCPT* pFTPDcpt, uint8_t *cFile)

  Summary:
	Execute FTP Named LIST command.

  Description:
	Execute FTP Named LIST command. 
  Precondition:
	None

  Parameters:
	pFTPDcpt - FTP descriptor
  Returns:
  	true or false
  	
  Remarks:
	Users should not call this function directly.
  ***************************************************************************/
static bool TCPIP_FTP_NameListCmd(TCPIP_FTP_DCPT* pFTPDcpt, uint8_t *cFile)
{
	int32_t fp;
	uint16_t wCount=0;
	TCP_SOCKET_INFO remoteSockInfo;
	uint8_t fileNameList[20];

	switch(pFTPDcpt->ftpCommandSm)
	{
		case TCPIP_FTP_CMD_SM_IDLE:
			if(pFTPDcpt->ftpFlag.Bits.pasvMode != true)
			{
				TCPGetSocketInfo(pFTPDcpt->ftpCmdskt, &remoteSockInfo);
	            pFTPDcpt->ftpDataskt   = 
					TCPOpenClient(IP_ADDRESS_TYPE_IPV4,pFTPDcpt->ftpDataPort,&remoteSockInfo.remoteIPaddress);

				// Make sure that a valid socket was available and returned
				// If not, return with an error
				if(pFTPDcpt->ftpDataskt == INVALID_SOCKET)
				{
		            pFTPDcpt->ftpResponse = TCPIP_FTP_RESP_DATA_NO_SOCKET;
		            return true;
				}
				//TCPModifySrcPort(pFTPDcpt->ftpDataskt,TCPIP_FTP_DATA_PORT);
				if(TCPBind(pFTPDcpt->ftpDataskt,IP_ADDRESS_TYPE_IPV4,TCPIP_FTP_DATA_PORT,&remoteSockInfo.localIPaddress) == false)
				{
		            pFTPDcpt->ftpResponse = TCPIP_FTP_RESP_DATA_NO_SOCKET;					
					TCPClose(pFTPDcpt->ftpDataskt);
		            return true;
				} 
			}
				
			// check login 
			if ( !pFTPDcpt->ftpFlag.Bits.loggedIn )
			{
				pFTPDcpt->ftpResponse	 = TCPIP_FTP_RESP_LOGIN;
				TCPClose(pFTPDcpt->ftpDataskt);
				return true;
			}
			
			pFTPDcpt->ftpResponse = TCPIP_FTP_RESP_DATA_READY;
			pFTPDcpt->ftpCommandSm = TCPIP_FTP_CMD_SM_WAIT;
			break;
		case TCPIP_FTP_CMD_SM_WAIT:
			if ( TCPIsConnected(pFTPDcpt->ftpDataskt) )
			{
				pFTPDcpt->ftpCommandSm	 = TCPIP_FTP_CMD_SM_SEND;
			}
			break;
		case TCPIP_FTP_CMD_SM_SEND:
			fp = SYS_FS_open((const char*)cFile,0);
			if(fp != RETURN_FAILED)
			{
				wCount = TCPIsPutReady(pFTPDcpt->ftpDataskt);
				strcpy((char*)fileNameList,(char*)"");
				sprintf((char*)fileNameList,"%s\r\n",(char *)pFTPDcpt->ftp_argv[1]);
				if(wCount > strlen((char *)fileNameList))
				{
					TCPPutArray(pFTPDcpt->ftpDataskt, fileNameList, 
											strlen((char *)fileNameList));
				}
				SYS_FS_close(fp);
			}
			
			pFTPDcpt->ftpCommandSm	= TCPIP_FTP_CMD_SM_WAIT_FOR_DISCONNECT;
			break;

		// to avoid warning
		case TCPIP_FTP_CMD_SM_WAIT_FOR_DISCONNECT:			
			TCPClose(pFTPDcpt->ftpDataskt);
			pFTPDcpt->ftpResponse 	= TCPIP_FTP_RESP_FILE_ACTION_SUCCESSFUL_CLOSING_DATA_CONNECTION;			
            pFTPDcpt->ftpDataskt   = INVALID_SOCKET;
			return true;
		case TCPIP_FTP_CMD_SM_RECEIVE:
			break;
	}

	return false;

}

/*****************************************************************************
  Function:
	static bool TCPIP_FTP_Execute_Get_Cmd(TCPIP_FTP_DCPT* pFTPDcpt, uint8_t *cFile)

  Summary:
	Execute FTP GET command.

  Description:
	Execute FTP GET command. 
  Precondition:
	None

  Parameters:
  	cFile -  File Name string
	pFTPDcpt - FTP descriptor
  Returns:
  	true or false
  	
  Remarks:
	Users should not call this function directly.
  ***************************************************************************/
static bool TCPIP_FTP_Execute_Get_Cmd(TCPIP_FTP_DCPT* pFTPDcpt, uint8_t *cFile)
{
   	TCP_SOCKET_INFO remoteSockInfo;	

    switch(pFTPDcpt->ftpCommandSm)
    {
    case TCPIP_FTP_CMD_SM_IDLE:
        if ( !pFTPDcpt->ftpFlag.Bits.loggedIn )
        {
            pFTPDcpt->ftpResponse     = TCPIP_FTP_RESP_LOGIN;
            return true;
        }
        else
        {
       		if(pFTPDcpt->ftpFlag.Bits.pasvMode != true)
			{
				TCPGetSocketInfo(pFTPDcpt->ftpCmdskt, &remoteSockInfo);
	            pFTPDcpt->ftpDataskt   = 
					TCPOpenClient(IP_ADDRESS_TYPE_IPV4,pFTPDcpt->ftpDataPort,&remoteSockInfo.remoteIPaddress);

				// Make sure that a valid socket was available and returned
				// If not, return with an error
				if(pFTPDcpt->ftpDataskt == INVALID_SOCKET)
				{
		            pFTPDcpt->ftpResponse = TCPIP_FTP_RESP_DATA_NO_SOCKET;
		            return true;
				}
				if(TCPBind(pFTPDcpt->ftpDataskt,IP_ADDRESS_TYPE_IPV4,TCPIP_FTP_DATA_PORT,&remoteSockInfo.localIPaddress) == false)
				{
		            pFTPDcpt->ftpResponse = TCPIP_FTP_RESP_DATA_NO_SOCKET;
					TCPClose(pFTPDcpt->ftpDataskt);
		            return true;
				} 
			}			
			pFTPDcpt->ftpResponse = TCPIP_FTP_RESP_FILE_IS_PRESENT;
			pFTPDcpt->ftpCommandSm = TCPIP_FTP_CMD_SM_WAIT;
        }
        break;

    case TCPIP_FTP_CMD_SM_WAIT:
        if ( TCPIsConnected(pFTPDcpt->ftpDataskt) )
        {
			pFTPDcpt->ftpCommandSm    = TCPIP_FTP_CMD_SM_SEND;
        }
        break;

    case TCPIP_FTP_CMD_SM_SEND:
		// Get/put as many bytes as possible
		if(TCPIP_FTP_GetFile(pFTPDcpt, cFile)== false)
		{
			pFTPDcpt->ftpResponse = TCPIP_FTP_RESP_FILE_ACTION_SUCCESSFUL_CLOSING_DATA_CONNECTION;
			pFTPDcpt->callbackPos = 0;			
            TCPClose(pFTPDcpt->ftpDataskt);
            pFTPDcpt->ftpDataskt   = INVALID_SOCKET;
			pFTPDcpt->ftpFlag.Bits.pasvMode = false;
			return true;
		}
		else if(pFTPDcpt->callbackPos == 0)
		{
            TCPClose(pFTPDcpt->ftpDataskt);
            pFTPDcpt->ftpDataskt   = INVALID_SOCKET;
            pFTPDcpt->ftpResponse = TCPIP_FTP_RESP_DATA_CLOSE;
			pFTPDcpt->ftpFlag.Bits.pasvMode = false;
            return true;
        }
	case TCPIP_FTP_CMD_SM_WAIT_FOR_DISCONNECT:
	case TCPIP_FTP_CMD_SM_RECEIVE:
		break;
    }
    return false;
}

/*****************************************************************************
  Function:
	static TCPIP_FTP_CMD TCPIP_FTP_Parse_Cmds(uint8_t *cmd)

  Summary:
	Parse FTP commands.

  Description:
	There are list of FTP commands within sTCPIPFTPCmdString and these are the FTP commands 
	are supported. This function is used to check if the Command is a valid FTP command.
  Precondition:
	None

  Parameters:
	cmd - FTP command
  Returns:
  	true or false
  	
  Remarks:
	Users should not call this function directly.
  ***************************************************************************/
static TCPIP_FTP_CMD TCPIP_FTP_Parse_Cmds(uint8_t *cmd)
{
    TCPIP_FTP_CMD i;

	for ( i = 0; i < (TCPIP_FTP_CMD)TCPIP_FTP_CMD_TBL_SIZE; i++ )
	{
		if ( !memcmp((void*)cmd, (const void*)sTCPIPFTPCmdString[i], strlen((char*)cmd)) )
			return i;
	}

	return TCPIP_FTP_CMD_UNKNOWN;
}

/*****************************************************************************
  Function:
	static void TCPIP_FTP_Parse_CmdString(TCPIP_FTP_DCPT* pFTPDcpt)

  Summary:
	Parse FTP input command string.

  Description:
	This function is used to parse FTP command string per interface and collect all the arguments
	using SM_FTP_PARSE_PARAM,SM_FTP_PARSE_SPACE. This function will parse both ',' and '|' 
	separated commands.

  Precondition:
	None

  Parameters:
	pFTPDcpt - FTP descriptor
  Returns:
  	None
  	
  Remarks:
	Users should not call this function directly.
  ***************************************************************************/
static void TCPIP_FTP_Parse_CmdString(TCPIP_FTP_DCPT* pFTPDcpt)
{
    uint8_t *p;
    uint8_t v;
    enum { SM_FTP_PARSE_PARAM, SM_FTP_PARSE_SPACE } smParseFTP;

    smParseFTP  = SM_FTP_PARSE_PARAM;
    p           = (uint8_t*)&pFTPDcpt->ftpCmdString[0];

    // Skip white blanks
    while( *p == ' ' )
    {
        p++;
    }

    pFTPDcpt->ftp_argv[0]  = (uint8_t*)p;
    pFTPDcpt->ftp_argc     = 1;

    while( (v = *p) )
    {
        switch(smParseFTP)
        {
        case SM_FTP_PARSE_PARAM:
            if ( v == ' ' || v == ',' || v=='|')
            {
                *p = '\0';
                smParseFTP = SM_FTP_PARSE_SPACE;
            }
            else if ( v == '\r' || v == '\n' )
                *p = '\0';
            break;

        case SM_FTP_PARSE_SPACE:
            if (( v != ' ' ) && (v != '|'))
            {
                pFTPDcpt->ftp_argv[pFTPDcpt->ftp_argc++] = (uint8_t*)p;
                smParseFTP = SM_FTP_PARSE_PARAM;
            }
            break;
        }
        p++;
        if(pFTPDcpt->ftp_argc == TCPIP_FTP_MAX_ARGS)
        	break;
    }
}

/*****************************************************************************
  Function:
	bool TCPIP_FTP_GetFile(TCPIP_FTP_DCPT* pFTPDcpt, uint8_t *cFile)

  Summary:
	Writes a file byte-for-byte to the currently loaded TCP socket.

  Description:
	Allows an entire file to be included as a dynamic variable, providing
	a basic templating system for HTML web pages.  This reduces unneeded
	duplication of visual elements such as headers, menus, etc.

	When pFTPDcpt->callbackPos is 0, the file is opened and as many bytes
	as possible are written.  The current position is then saved to 
	pFTPDcpt->callbackPos and the file is closed.  On subsequent calls, 
	reading begins at the saved location and continues.  Once the end of
	the input file is reached, pFTPDcpt->callbackPos is set back to 0 to 
	indicate completion.

  Precondition:
	None

  Parameters:
	cFile - the name of the file to be sent
	pFTPDcpt - FTP descriptor
  Returns:
  	None
  	
  Remarks:
	Users should not call this function directly.
  ***************************************************************************/
static bool TCPIP_FTP_GetFile(TCPIP_FTP_DCPT* pFTPDcpt, uint8_t *cFile)
{
	int16_t wCount, wLen;
	uint8_t data[64];
	int32_t fp;

	fp = pFTPDcpt->fileDescr;

	// Check if this is a first round call
	if(pFTPDcpt->callbackPos == 0x00u)
	{// On initial call, open the file and save its ID
		fp = SYS_FS_open((const char*)cFile,0);
		if(fp == RETURN_FAILED)
		{// File not found, so abort
			return false;
		}
		pFTPDcpt->fileDescr = fp;
		pFTPDcpt->callbackPos = 0;
	} 
	else if(pFTPDcpt->callbackPos != 0x00u)
	{// The file was already opened, so load up its ID and seek
		if(fp == RETURN_FAILED)
		{// No file handles available, so wait for now
			return false;
		}
		SYS_FS_lseek(fp,((TCPIP_UINT32_VAL*)&pFTPDcpt->callbackPos)->w[1],SEEK_SET);
	}
	
	// Get/put as many bytes as possible
	wCount = TCPIsPutReady(pFTPDcpt->ftpDataskt);	
	while(wCount > 0u)
	{
		wLen = SYS_FS_read(fp,data,mMIN(wCount, sizeof(data)));
		if(wLen == -1)
		{// If no bytes were read, an EOF was reached
			SYS_FS_close(fp);
			pFTPDcpt->callbackPos = 0x00;
			return true;
		}
		else
		{// Write the bytes to the socket
			TCPPutArray(pFTPDcpt->ftpDataskt, data, wLen);			
			wCount -= wLen;
		}
	}
	
	// Save the new address and close the file
	((TCPIP_UINT32_VAL*)&pFTPDcpt->callbackPos)->w[1] = SYS_FS_lseek(fp,0,SEEK_CUR);//MPFSTell(fp);
	
	return true;
}


#endif	// #if defined(TCPIP_STACK_USE_FTP_SERVER)

