/*******************************************************************************
  Telnet Server

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Provides Telnet services on TCP port 23
    -Reference: RFC 854
*******************************************************************************/

/*******************************************************************************
FileName:   Telnet.c
Copyright ?2012 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS?WITHOUT WARRANTY OF ANY KIND,
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

#define __TELNET_C

#include "tcpip_private.h"

#if defined(TCPIP_STACK_USE_TELNET_SERVER)

// Set up configuration parameter defaults if not overridden in 
// tcpip_config.h
#if !defined(TELNET_PORT)
    // Unsecured Telnet port
	#define TELNET_PORT			23
#endif

#if !defined(TELNETS_PORT)	
    // SSL Secured Telnet port (ignored if TCPIP_STACK_USE_SSL_SERVER is undefined)
	#define TELNETS_PORT		992	
#endif
#if !defined(MAX_TELNET_CONNECTIONS)
    // Maximum number of Telnet connections
	#define MAX_TELNET_CONNECTIONS	(2u)
#endif
#if !defined(TELNET_USERNAME)
    // Default Telnet user name
	#define TELNET_USERNAME		"admin"
#endif
#if !defined(TELNET_PASSWORD)
    // Default Telnet password
	#define TELNET_PASSWORD		"microchip"
#endif

#define TELNET_LINE_RETURN  "\r"
#define TELNET_LINE_FEED    "\n"
#define TELNET_LINE_TERM    TELNET_LINE_RETURN TELNET_LINE_FEED  

// limited set of supported telnet commands 
#define TELNET_CMD_IAC          "\xff"
#define TELNET_CMD_DONT         "\xfe"
#define TELNET_CMD_DO           "\xfd"
#define TELNET_CMD_WONT         "\xfc"
#define TELNET_CMD_WILL         "\xfb"

#define TELNET_CMD_IAC_CODE     '\xff'
#define TELNET_CMD_DONT_CODE    '\xfe'
#define TELNET_CMD_DO_CODE      '\xfd'
#define TELNET_CMD_WONT_CODE    '\xfc'
#define TELNET_CMD_WILL_CODE    '\xfb'



// limited set of supported telnet options
#define TELNET_OPT_SUPP_LOCAL_ECHO  "\x2d"      // suppress local echo


// connection display strings
//

// start up message
// 2J is clear screen, 31m is red, 1m is bold
// 0m is clear all attributes
#define TELNET_START_MSG                "\x1b[2J\x1b[31m\x1b[1m" \
                                "Microchip Telnet Server 1.1\x1b[0m\r\n" \
								"Login: "
//
// ask password message
#define TELNET_ASK_PASSWORD_MSG     "Password: " TELNET_CMD_IAC TELNET_CMD_DO TELNET_OPT_SUPP_LOCAL_ECHO        // ask Suppress Local Echo

// Access denied message/ failed logon
#define TELNET_FAIL_LOGON_MSG       TELNET_LINE_TERM "Access denied" TELNET_LINE_TERM TELNET_LINE_TERM      

// internal buffer overflow message
#define TELNET_BUFFER_OVFLOW_MSG    "Too much data. Aborted" TELNET_LINE_TERM

// Successful authentication message/log on OK
#define TELNET_LOGON_OK             TELNET_LINE_TERM "Logged in successfully" TELNET_LINE_TERM TELNET_LINE_TERM 

// welcome message
#define TELNET_WELCOME_MSG          TELNET_LINE_TERM "--- Telnet Console ---" TELNET_LINE_TERM \
                                                     "Type help for commands" TELNET_LINE_TERM ">"

// disconnect message
//#define TELNET_BYE_MSG            TELNET_LINE_TERM TELNET_LINE_TERM "Goodbye!" TELNET_LINE_TERM TELNET_LINE_TERM

// failure to register with the command processor
#define TELNET_FAIL_CMD_REGISTER    "Failed to connect to the command processor. Aborting!" TELNET_LINE_TERM

// buffering defines
#define TELNET_PRINT_BUFF           200     // internal print buffer
#define TELNET_LINE_BUFF            (80 +3) // assembled line buffer for password, authentication, etc
                                            // + extra room for \r\n
#define TELNET_SKT_MESSAGE_SPACE    80      // min space needed in the socket buffer for displaying messages


// machine state
typedef	enum
{
    SM_HOME = 0,
    SM_PRINT_LOGIN,
    SM_GET_LOGIN,
    SM_GET_PASSWORD,
    SM_GET_PASSWORD_BAD_LOGIN,
    SM_AUTHENTICATED,
    SM_CONNECTED
} TELNET_STATE;

typedef enum
{
    TELNET_MSG_LINE_PENDING,        // no line assembled yet
    TELNET_MSG_LINE_DONE,           // line assembled
    TELNET_MSG_LINE_OVFL            // line buffer capacity exceeded
}TELNET_MSG_LINE_RES;   // message line result
    
typedef struct
{
    TCP_SOCKET          telnetSkt;
    TELNET_STATE        telnetState;
    _CMDIO_DEV_NODE*    telnetIO;
}TELNET_DCPT;

static TELNET_DCPT      telnetDcpt[MAX_TELNET_CONNECTIONS];

static int              telnetInitCount = 0;      // TELNET module initialization count


// prototypes
static void TelnetMSG(const void* cmdIoParam, const char* str);
static void TelnetPRINT(const void* cmdIoParam, const char* format, ...);
static void TelnetPUTC(const void* cmdIoParam, char c);
static bool TelnetDATA_RDY(const void* cmdIoParam);
static char TelnetGETC(const void* cmdIoParam);

static void TelnetDeregister(TELNET_DCPT* pDcpt);
static void TelnetClose(TELNET_DCPT* pDcpt);

static TELNET_STATE TelnetCheckUser(TCP_SOCKET tSocket, TELNET_STATE tState);
static TELNET_STATE TelnetCheckLogon(TCP_SOCKET tSocket, TELNET_STATE tState);
static TELNET_MSG_LINE_RES TelnetCheckMessageLine(TCP_SOCKET tSkt, char* lineBuffer, int bufferSize, int* readBytes);
static char* TelnetSkipCommands(const char* strMsg);

static const _CMDIO_DEV_API telnetIOApi = 
{
    TelnetMSG,
    TelnetPRINT,
    TelnetPUTC,
    TelnetDATA_RDY,
    TelnetGETC
};



// implementation
//


bool TelnetInit(const TCPIP_STACK_MODULE_CTRL* const stackCtrl, const TELNET_MODULE_CONFIG* pTelConfig)
{

    int tIx;
    TELNET_DCPT* pDcpt;

    // if(stackCtrl->stackAction == TCPIP_STACK_ACTION_IF_UP)
    // interface restart


    // stack init

    if(telnetInitCount == 0)
    {   // first time we're run
        pDcpt = telnetDcpt;
        for(tIx = 0; tIx < sizeof(telnetDcpt)/sizeof(*telnetDcpt); tIx++, pDcpt++)
        {
            pDcpt->telnetSkt = INVALID_SOCKET;
            pDcpt->telnetState = SM_HOME;
        }

    }

    telnetInitCount++;


    return true;
}


void TelnetDeInit(const TCPIP_STACK_MODULE_CTRL* const stackCtrl)
{
    int tIx;

    // if(stackCtrl->stackAction == TCPIP_STACK_ACTION_DEINIT) // stack shut down
    // if(stackCtrl->stackAction == TCPIP_STACK_ACTION_IF_DOWN) // interface down

    
    if(stackCtrl->stackAction == TCPIP_STACK_ACTION_DEINIT)
    {   // whole stack is going down
        if(telnetInitCount > 0)
        {   // we're up and running
            if(--telnetInitCount == 0)
            {   // all closed
                for(tIx = 0; tIx < sizeof(telnetDcpt)/sizeof(*telnetDcpt); tIx++)
                {
                    TelnetClose(telnetDcpt + tIx);
                }
            }
        }
    }

}


static void TelnetDeregister(TELNET_DCPT* pDcpt)
{
    if (pDcpt->telnetIO != 0)
    {
        _SYS_CMDIO_DELETE(pDcpt->telnetIO);
        pDcpt->telnetIO = 0;
    }

}

static void TelnetClose(TELNET_DCPT* pDcpt)
{

    TelnetDeregister(pDcpt);

    if( pDcpt->telnetSkt != INVALID_SOCKET)
    {
        TCPClose(pDcpt->telnetSkt);
        pDcpt->telnetSkt = INVALID_SOCKET;
    }

    pDcpt->telnetState = SM_HOME;

}

// Telnet's PUTC
static void TelnetPUTC(const void* cmdIoParam, char c)
{
    TCP_SOCKET tSkt = (TCP_SOCKET)(int)cmdIoParam;
    if(tSkt != INVALID_SOCKET)
    {
        TCPPut(tSkt, (uint8_t)c);
        TCPFlush(tSkt);
    }
}

// Telnet's message	
static void TelnetMSG(const void* cmdIoParam, const char* str)
{
    TCP_SOCKET tSkt = (TCP_SOCKET)(int)cmdIoParam;
    if(tSkt != INVALID_SOCKET)
    {
        TCPPutString(tSkt, (const uint8_t*)str);
        TCPFlush(tSkt);
    }
}


// Telnet's print
static void TelnetPRINT(const void* cmdIoParam, const char* format, ...)
{
    va_list arg_list;
    char buff[TELNET_PRINT_BUFF];

    va_start(arg_list, format);
    vsnprintf(buff, TELNET_PRINT_BUFF, format, arg_list);
    va_end(arg_list);

    TelnetMSG(cmdIoParam, buff);
}


// Telnet's data ready
static bool TelnetDATA_RDY(const void* cmdIoParam)
{
    TCP_SOCKET tSkt = (TCP_SOCKET)(int)cmdIoParam;
    if(tSkt != INVALID_SOCKET)
    {
        return TCPIsGetReady(tSkt) != 0;
    }

    return false;
}

// Telnet's getc
static char TelnetGETC(const void* cmdIoParam)
{

    TCP_SOCKET tSkt = (TCP_SOCKET)(int)cmdIoParam;
    if(tSkt != INVALID_SOCKET)
    {
        uint8_t bData;
        if(TCPGet(tSkt, &bData))
        {
            return (char)bData;
        }
    }

    return 0;
}



/*********************************************************************
 * Function:        bool TelnetTask(TCPIP_NET_IF* pNetIf)
 *
 * PreCondition:    Stack is initialized()
 *
 * Input:           pNetIf  - network interface
 *
 * Output:          true if processing OK, false otherwise
 *
 * Side Effects:    None
 *
 * Overview:        Performs Telnet Server related tasks.  Contains
 *                  the Telnet state machine and state tracking
 *                  variables.
 *
 * Note:            None
 ********************************************************************/
bool TelnetTask(TCPIP_NET_IF* pNetIf)
{
    int         tIx;
    TELNET_DCPT* pDcpt;
    TCP_SOCKET	tSocket;
    TELNET_STATE tState;


    // Loop through each telnet session and process state changes and TX/RX data
    pDcpt = telnetDcpt;
    for(tIx = 0; tIx < sizeof(telnetDcpt)/sizeof(*telnetDcpt); tIx++, pDcpt++)
    {
        // Load up static state information for this session
        tSocket = pDcpt->telnetSkt;
        tState = pDcpt->telnetState;

        // Reset our state if the remote client disconnected from us
        if(tSocket != INVALID_SOCKET)
        {
            if(TCPWasReset(tSocket))
            {
                // Deregister IO and free its space
                TelnetDeregister(pDcpt);
                tState = SM_PRINT_LOGIN;
            }
        }

        // Handle session state
        switch(tState)
        {
            case SM_HOME:
                // Connect a socket to the remote TCP server
                tSocket = TCPOpenServer(IP_ADDRESS_TYPE_IPV4, TELNET_PORT, 0);

                // Abort operation if no TCP socket could be opened.
                // If this ever happens, you need to update your tcp_config.h
                if(tSocket == INVALID_SOCKET)
                {
                    break;
                }

                pDcpt->telnetSkt = tSocket;
                // Open an SSL listener if SSL server support is enabled
#if defined(TCPIP_STACK_USE_SSL_SERVER)
                TCPAddSSLListener(tSocket, TELNETS_PORT);
#endif

                tState++;
                break;

            case SM_PRINT_LOGIN:
#if defined(TCPIP_STACK_USE_SSL_SERVER)
                // Reject unsecured connections if TELNET_REJECT_UNSECURED is defined
#if defined(TELNET_REJECT_UNSECURED)
                if(!TCPIsSSL(tSocket))
                {
                    if(TCPIsConnected(tSocket))
                    {
                        TCPClose(tSocket);
                        break;
                    }	
                }
#endif

                // Don't attempt to transmit anything if we are still handshaking.
                if(TCPSSLIsHandshaking(tSocket))
                    break;
#endif

                // Make certain the socket can be written to
                if(TCPIsPutReady(tSocket) < TELNET_SKT_MESSAGE_SPACE)
                    break;

                // Place the application protocol data into the transmit buffer.
                TCPPutString(tSocket, (const uint8_t*)TELNET_START_MSG);

                // Send the packet
                TCPFlush(tSocket);
                tState++;

            case SM_GET_LOGIN:
                tState = TelnetCheckUser(tSocket, tState);
                break;

            case SM_GET_PASSWORD:
            case SM_GET_PASSWORD_BAD_LOGIN:

                tState = TelnetCheckLogon(tSocket, tState);
                break;

            case SM_AUTHENTICATED:
                if(TCPIsPutReady(tSocket) < TELNET_SKT_MESSAGE_SPACE)
                    break;

                TCPPutString(tSocket, (const uint8_t*)TELNET_WELCOME_MSG);
                tState++;

                TCPFlush(tSocket);

                // Register telnet as cmd IO device
                pDcpt->telnetIO = _SYS_CMDIO_ADD(&telnetIOApi, (const void*)(int)tSocket);
                if (pDcpt->telnetIO == 0)
                {
                    TCPPutString(tSocket, (const uint8_t*)TELNET_FAIL_CMD_REGISTER);
                    TCPDisconnect(tSocket);
                    tState = SM_PRINT_LOGIN;
                    break;
                }	

            case SM_CONNECTED:
                // Check if you're disconnected and de-register from the command processor

                break;
        }


        // Save session state back into the static array
        pDcpt->telnetState = tState;
    }

    return true;
}

static TELNET_STATE TelnetCheckUser(TCP_SOCKET tSkt, TELNET_STATE tState)
{
    int         avlblBytes;
    bool        userFound;
    char        *lineStr;
    TELNET_MSG_LINE_RES lineRes;


    char    userMessage[TELNET_LINE_BUFF];    // telnet confirmation message

    if(TCPIsPutReady(tSkt) < TELNET_SKT_MESSAGE_SPACE)
    {   // wait some more
        return tState;
    }

    lineRes = TelnetCheckMessageLine(tSkt, userMessage, sizeof(userMessage), &avlblBytes);

    if(lineRes == TELNET_MSG_LINE_PENDING)
    {   // wait some more
        return tState;
    }
    else if(lineRes == TELNET_MSG_LINE_OVFL)
    {
        TCPPutString(tSkt, (const uint8_t*)TELNET_BUFFER_OVFLOW_MSG);
        TCPPutString(tSkt, (const uint8_t*)TELNET_FAIL_LOGON_MSG);
        TCPDisconnect(tSkt);
        return SM_PRINT_LOGIN;	
    }

    // TELNET_MSG_LINE_DONE
    // ignore telnet commands/advertisments sent to us by the client
    // we do not support them!
    lineStr = TelnetSkipCommands(userMessage);
    // remove the line termination 
    lineStr = strtok(lineStr, TELNET_LINE_TERM);
    // find the user name
    if(lineStr && strcmp(lineStr, TELNET_USERNAME) == 0)
    {
        userFound = true;
    }
    else
    {
        userFound = false;
    }

    TCPGetArray(tSkt, 0, avlblBytes);  //  throw this line of data away
    // Print the password prompt
    TCPPutString(tSkt, (const uint8_t*)TELNET_ASK_PASSWORD_MSG);
    return userFound?SM_GET_PASSWORD:SM_GET_PASSWORD_BAD_LOGIN;

}

static TELNET_STATE TelnetCheckLogon(TCP_SOCKET tSkt, TELNET_STATE tState)
{
    int     avlblBytes;
    bool    sktDisconnect, sktOverflow;
    char*   lineStr;
    TELNET_MSG_LINE_RES lineRes;

    char    passMessage[TELNET_LINE_BUFF];    // telnet confirmation message


    if(TCPIsPutReady(tSkt) < TELNET_SKT_MESSAGE_SPACE)
    {   // wait some more
        return tState;
    }

    lineRes = TelnetCheckMessageLine(tSkt, passMessage, sizeof(passMessage), &avlblBytes);

    if(lineRes == TELNET_MSG_LINE_PENDING)
    {   // wait some more
        return tState;
    }

    sktDisconnect = sktOverflow = false;

    if(lineRes == TELNET_MSG_LINE_OVFL)
    {
        sktOverflow = true;
    }
    else
    {   // TELNET_MSG_LINE_DONE
        // ignore telnet commands/advertisments sent to us by the client
        // we do not support them!
        lineStr = TelnetSkipCommands(passMessage);
        // remove the line termination 
        lineStr = strtok(lineStr, TELNET_LINE_TERM);
        if(tState != SM_GET_PASSWORD || strcmp(lineStr, TELNET_PASSWORD) != 0)
        {   // failed
            sktDisconnect = true;
        }
    }

    if(sktOverflow)
    {
        TCPPutString(tSkt, (const uint8_t*)TELNET_BUFFER_OVFLOW_MSG);
    }

    if(sktOverflow || sktDisconnect)
    {
        TCPPutString(tSkt, (const uint8_t*)TELNET_FAIL_LOGON_MSG);
        TCPDisconnect(tSkt);
        return SM_PRINT_LOGIN;	
    }

    // success
    TCPGetArray(tSkt, 0, avlblBytes);  //  throw this line of data away
    // Print the authenticated prompt
    TCPPutString(tSkt, (const uint8_t*)TELNET_LOGON_OK);
    return SM_AUTHENTICATED;

}



// checks if a complete line is assembled
static TELNET_MSG_LINE_RES TelnetCheckMessageLine(TCP_SOCKET tSkt, char* lineBuffer, int bufferSize, int* readBytes)
{
    int     avlblBytes;
    char    *lineTerm;

    avlblBytes = TCPPeekArray(tSkt, (uint8_t*)lineBuffer, bufferSize - 1, 0);

    if(avlblBytes)
    {   // we need at least one terminator character
        // make sure we have a complete line
        lineBuffer[avlblBytes] = 0;
        lineTerm = strstr(lineBuffer, TELNET_LINE_RETURN);
        if(lineTerm == 0)
        {
            lineTerm = strstr(lineBuffer, TELNET_LINE_FEED);
        }

        if(lineTerm != 0)
        {
            *readBytes = avlblBytes;
            return TELNET_MSG_LINE_DONE;
        }

        // no end of line pressed yet
        if(avlblBytes == bufferSize - 1)
        {   // our buffer is full: overflowed
            return TELNET_MSG_LINE_OVFL;
        }
        // else wait some more
    }

    return TELNET_MSG_LINE_PENDING;

}

static char* TelnetSkipCommands(const char* strMsg)
{
    char c;
    while(true)
    {
        if(*strMsg != TELNET_CMD_IAC_CODE)
        {
            break;
        }
        // start of command
        c = *++strMsg;
        if(c == TELNET_CMD_IAC_CODE)
        {   // this is data, not command
            break;
        }
        // valid command sequence
        if(c == TELNET_CMD_DO_CODE || c == TELNET_CMD_DONT_CODE || c == TELNET_CMD_WILL_CODE || c == TELNET_CMD_WONT_CODE)
        {   // skip option character that follows
            strMsg += 2;
        }
        else
        {   // we don't support other commands for now
            break;
        }
    }

    return (char*)strMsg;

}

#endif	//#if defined(TCPIP_STACK_USE_TELNET_SERVER)
