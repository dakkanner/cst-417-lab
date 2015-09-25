/*******************************************************************************
  Generic TCP Client Example Application

  Summary:
    Module for Microchip TCP/IP Stack

  Description:
    -Implements an example HTTP client and should be used as a basis
     for creating new TCP client applications
    -Reference: None.  Hopefully AN833 in the future.
*******************************************************************************/

/*******************************************************************************
FileName:  ftp_client.c
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

// This is a demo code.  there is a file on FTP server:  ftp.microchip.com\test.txt.
// This code is a demo to download  file "test.txt" from FTP server "ftp.microchip.com"
// the username is "mrfupdates",  password is "mchp1234";

#include "tcpip_config.h"
#include "main_demo.h"

#if defined(TCPIP_STACK_USE_TCP) && defined(APP_USE_FTP_CLIENT_DEMO)
#include "tcpip/tcpip.h"
#include <system/system_userio.h>

typedef enum _FTPClt_RESPONSE
 {
     FTP_CLT_RESP_READY,
     FTP_CLT_RESP_USER,
     FTP_CLT_RESP_PASS,
     FTP_CLT_RESP_QUIT,
     FTP_CLT_RESP_STOR,
     FTP_CLT_RESP_UNKNOWN,
     FTP_CLT_RESP_LOGIN,
     FTP_CLT_RESP_DATA_OPEN,
     FTP_CLT_RESP_DATA_READY,
     FTP_CLT_RESP_DATA_CLOSE,
     FTP_CLT_RESP_DATA_NO_SOCKET,
     FTP_CLT_RESP_PWD,
     FTP_CLT_RESP_OK,
 
     FTP_RESP_NONE                          // This must always be the last
                                         // There is no corresponding string.
 } FTPClt_RESPONSE;
    

 const char *  FtpCltResponseString[] =
 {
     "220 ",        // FTP_CLT_RESP_READY    
     "331 ",        // FTP_CLT_RESP_USER  
     "230 ",        // FTP_CLT_RESP_PASS      
     "221 ",        // FTP_CLT_RESP_QUIT      
     "500 ",        // FTP_CLT_RESP_STOR
     "502 ",        // FTP_CLT_RESP_UNKNOWN      
     "530 ",        // FTP_CLT_RESP_LOGIN     
     "150 ",        // FTP_CLT_RESP_DATA_OPEN     
     "125 ",        // FTP_CLT_RESP_DATA_READY 
     "226 ",        // FTP_CLT_RESP_DATA_CLOSE 
     "425 ",        // FTP_CLT_RESP_DATA_NO_SOCKET 
     "257 ",        // FTP_CLT_RESP_PWD  
     "200 ",         // FTP_CLT_RESP_OK 

     ""             //FTP_RESP_NONE
 };

char *    FtpCltResponseDescription[] =
{
    "220--Ready",               // FTP_CLT_RESP_BANNER
    "331--Password required",   // FTP_CLT_RESP_USER 
    "230--Logged in",           // FTP_CLT_RESP_PASS
    "221--Bye",                 // FTP_CLT_RESP_QUIT
    "500--",                    // FTP_CLT_RESP_STOR
    "502--",                    // FTP_CLT_RESP_UNKNOWN
    "530--Login required",        // FTP_CLT_RESP_LOGIN 
    "150--Transferring data...", // FTP_CLT_RESP_DATA_OPEN
    "125--Done",                // FTP_CLT_RESP_DATA_READY
    "226--Transfer Complete",   // FTP_CLT_RESP_DATA_CLOSE
    "425--Can't create data socket", // FTP_CLT_RESP_DATA_NO_SOCKET 
    "257--",                   // FTP_CLT_RESP_PWD
    "200--Ok",                  // FTP_CLT_RESP_OK

    "Nothing"                  //FTP_RESP_NONE
};

// Defines the server to be accessed for this application
static const char ServerName[] =    "ftp.microchip.com";
// the server address
static IPV4_ADDR  serverIP;


// Defines the port to be accessed for this application
static WORD Port_FtpCltCmd = 21;
static WORD Port_FtpCltData = 59136;

static const char UserName[]="mrfupdates";
static const char PassWord[]="mchp1234";

static const char FileName[]="test.txt";

static TCP_SOCKET     Socket_FtpCltCmd  = INVALID_SOCKET;
static TCP_SOCKET    Socket_FtpCltData = INVALID_SOCKET;

static void AU_print_string(uint8_t *buf,uint8_t length)
{
    int i;
    for(i=0;i<length;i++) SYS_CONSOLE_PUTC(buf[i]);

}


static bool TcpClientCheckRespond(FTPClt_RESPONSE index)
{
    uint16_t w, lenB;
    uint8_t                vBuffer[32];
    w = TCPIsGetReady(Socket_FtpCltCmd);
    if(w == 0) return false;
    
    if(0xFFFFu == TCPFindArray(Socket_FtpCltCmd, (const uint8_t*)FtpCltResponseString[index], strlen(FtpCltResponseString[index]), 0, 0, FALSE)) return false;
    SYS_CONSOLE_MESSAGE("\r\n#");
    w = TCPIsGetReady(Socket_FtpCltCmd);
    while(w>0)
    {
        lenB = TCPGetArray(Socket_FtpCltCmd, vBuffer, ((w <= sizeof(vBuffer)) ? w : sizeof(vBuffer)));
        AU_print_string(vBuffer,lenB);
        w -= lenB;
    }
    SYS_CONSOLE_MESSAGE("\r\n@-- ");
    SYS_CONSOLE_MESSAGE(FtpCltResponseDescription[index]);
    SYS_CONSOLE_MESSAGE("\r\n");
    
    return true;
 
}

/*****************************************************************************
  Function:
    void FTPClient(void)

  Summary:

  Description:

  Precondition:
    TCP is initialized.

  Parameters:
    None

  Returns:
    None
  ***************************************************************************/
void FTPClient(void)
{
    uint16_t                w;
    DNS_RESULT          dnsRes;
    uint8_t                vBuffer[32];
    static TCPIP_NET_HANDLE    netH;
    static uint32_t        clientTimer;

    uint16_t lenB;

 

    static enum _FtpClientCmdState
    {
        SM_FTP_CLIENT_COMMAND_HOME = 0,

        SM_FTP_CLIENT_COMMAND_WAIT_DNS,
        SM_FTP_CLIENT_COMMAND_DNS_RESOLVED,
        SM_FTP_CLIENT_COMMAND_SOCKET_OBTAINED,
        SM_FTP_CLIENT_COMMAND_USRNAME,
        SM_FTP_CLIENT_COMMAND_PASSWORD1,
        SM_FTP_CLIENT_COMMAND_PASSWORD2,
        SM_FTP_CLIENT_COMMAND_LOGIN1,
        SM_FTP_CLIENT_COMMAND_DATA1,
        SM_FTP_CLIENT_COMMAND_DATA2,
        SM_FTP_CLIENT_COMMAND_DATA3,
        SM_FTP_CLIENT_COMMAND_DATA4,
        SM_FTP_CLIENT_COMMAND_DATA5,
        SM_FTP_CLIENT_COMMAND_QUIT1,
        SM_FTP_CLIENT_COMMAND_QUIT2,
        SM_FTP_CLIENT_COMMAND_DISCONNECT,
        SM_FTP_CLIENT_COMMAND_DONE
    } FtpClientCmdState = SM_FTP_CLIENT_COMMAND_DONE;
    static enum _FtpClientDataState
    {
        SM_FTP_CLIENT_DATA_HOME = 0,
        SM_FTP_CLIENT_DATA_WAIT,
        SM_FTP_CLIENT_DATA_DATA,
        SM_FTP_CLIENT_DATA_DISCONNECT,
        SM_FTP_CLIENT_DATA_DONE
    } FtpClientDataState = SM_FTP_CLIENT_DATA_DONE;

    switch(FtpClientCmdState)
    {

        case SM_FTP_CLIENT_COMMAND_HOME:

            SYS_CONSOLE_MESSAGE((const char*)"\r\n\r\nResolving the server name using Microchip DNS API...\r\n");
            netH = TCPIP_STACK_GetDefaultNet();

            if(DNSBeginUsage(netH) != DNS_RES_OK)
            {
                break;
            }
            DNSResolve(ServerName, DNS_TYPE_A);
            FtpClientCmdState++;
            break;

        case SM_FTP_CLIENT_COMMAND_WAIT_DNS:

            dnsRes = DNSIsResolved(ServerName, &serverIP);
            if(dnsRes == DNS_RES_PENDING)
            {   // ongoing operation;
                break;
            }
            else if(dnsRes < 0)
            {   // some DNS error occurred; retry
                SYS_CONSOLE_MESSAGE((const char*)"\r\n\r\nDNS name resolving failed...\r\n");
                TCPClose(Socket_FtpCltCmd);
                Socket_FtpCltCmd = INVALID_SOCKET;
                FtpClientCmdState = SM_FTP_CLIENT_COMMAND_HOME;
            }
            else
            {
                clientTimer = SYS_TICK_Get();
                FtpClientCmdState++;
            }
            DNSEndUsage(netH);
            break;

        case SM_FTP_CLIENT_COMMAND_DNS_RESOLVED:
            // Connect the socket to the remote FTP server
            Socket_FtpCltCmd = TCPOpenClient(IP_ADDRESS_TYPE_IPV4, Port_FtpCltCmd, (IP_MULTI_ADDRESS*)&serverIP);

            // Abort operation if no TCP socket could be opened.
            // If this ever happens, you need to update your tcp_config.h
            if(Socket_FtpCltCmd == INVALID_SOCKET)
            {   // retry
                break;
            }

            SYS_CONSOLE_MESSAGE((const char*)"\r\n\r\nConnecting using Microchip TCP API...\r\n");

            FtpClientCmdState++;
            clientTimer = SYS_TICK_Get();
            break;

        case SM_FTP_CLIENT_COMMAND_SOCKET_OBTAINED:

            // Wait for the remote server to accept our connection request
            if(!TCPIsConnected(Socket_FtpCltCmd))
            {
                // Time out if more than 5 seconds is spent in this state
                if((SYS_TICK_Get()-clientTimer) > 5 * SYS_TICK_TicksPerSecondGet() )
                {
                    // Close the socket so it can be used by other modules
                    TCPClose(Socket_FtpCltCmd);
                    Socket_FtpCltCmd = INVALID_SOCKET;
                    FtpClientCmdState--;
                    SYS_CONSOLE_MESSAGE((const char*)"\r\n\r\nFailed connecting to the remote server...\r\n");
                }
                break;
            }

            clientTimer = SYS_TICK_Get();
            if(true == TcpClientCheckRespond(FTP_CLT_RESP_READY/*"220 "*/))
            {
                SYS_CONSOLE_MESSAGE("@Receive:220 -- Ready\r\n");
                FtpClientCmdState = SM_FTP_CLIENT_COMMAND_USRNAME;
            }
            break;

        case SM_FTP_CLIENT_COMMAND_USRNAME:
            if(!TCPIsConnected(Socket_FtpCltCmd))
            {
                FtpClientCmdState = SM_FTP_CLIENT_COMMAND_DISCONNECT;
                break;
            }
            
            if(TCPIsPutReady(Socket_FtpCltCmd) < 30u)   break;            
            TCPPutString(Socket_FtpCltCmd, (const uint8_t*)"USER ");
            TCPPutString(Socket_FtpCltCmd, (const uint8_t*)UserName);            
            TCPPutString(Socket_FtpCltCmd, (const uint8_t*)"\r\n");
            // Send the packet
            TCPFlush(Socket_FtpCltCmd);
            FtpClientCmdState = SM_FTP_CLIENT_COMMAND_PASSWORD1;
            break;

        case SM_FTP_CLIENT_COMMAND_PASSWORD1:
            if(!TCPIsConnected(Socket_FtpCltCmd))
            {
                FtpClientCmdState = SM_FTP_CLIENT_COMMAND_DISCONNECT;
                break;
            }
            if(true == TcpClientCheckRespond(/*331 */FTP_CLT_RESP_USER))
            {
                FtpClientCmdState = SM_FTP_CLIENT_COMMAND_PASSWORD2;
            }
            break;
        case SM_FTP_CLIENT_COMMAND_PASSWORD2:
            if(!TCPIsConnected(Socket_FtpCltCmd))
            {
                FtpClientCmdState = SM_FTP_CLIENT_COMMAND_DISCONNECT;
                break;
            }
            if(TCPIsPutReady(Socket_FtpCltCmd) < 30u)
                break;

            TCPPutString(Socket_FtpCltCmd, (const uint8_t*)"PASS ");
            TCPPutString(Socket_FtpCltCmd, (const uint8_t*)PassWord);
            TCPPutString(Socket_FtpCltCmd, (const uint8_t*)"\r\n");
            TCPFlush(Socket_FtpCltCmd);
            FtpClientCmdState = SM_FTP_CLIENT_COMMAND_LOGIN1;
            break;
        case SM_FTP_CLIENT_COMMAND_LOGIN1:
            if(!TCPIsConnected(Socket_FtpCltCmd))
            {
                FtpClientCmdState = SM_FTP_CLIENT_COMMAND_DISCONNECT;
                break;
            }

            if(true == TcpClientCheckRespond(/*230*/FTP_CLT_RESP_PASS))
            {
                FtpClientCmdState = SM_FTP_CLIENT_COMMAND_DATA1;
            }

            break;
        case SM_FTP_CLIENT_COMMAND_DATA1:
        {

            IPV4_ADDR localIP;
            char buf_tmp[30] = {0};

            if(!TCPIsConnected(Socket_FtpCltCmd))
            {
                FtpClientCmdState = SM_FTP_CLIENT_COMMAND_DISCONNECT;
                break;
            }
            if(TCPIsPutReady(Socket_FtpCltCmd) < 30u)
                break;
            FtpClientDataState = SM_FTP_CLIENT_DATA_HOME;
            
            netH = TCPIP_STACK_GetDefaultNet();
            localIP.Val = TCPIP_STACK_NetAddress(netH);
            sprintf(buf_tmp,"%d,%d,%d,%d,%d,%d\r\n",localIP.v[0],localIP.v[1],localIP.v[2],localIP.v[3],
                                                    Port_FtpCltData >> 8, Port_FtpCltData & 0x00ff);
            TCPPutString(Socket_FtpCltCmd, (const uint8_t*)"PORT ");
            TCPPutString(Socket_FtpCltCmd, (const uint8_t*)buf_tmp);
            TCPFlush(Socket_FtpCltCmd);
            FtpClientCmdState = SM_FTP_CLIENT_COMMAND_DATA2;
        }
            break;
        case SM_FTP_CLIENT_COMMAND_DATA2:
            if(!TCPIsConnected(Socket_FtpCltCmd))
            {
                FtpClientCmdState = SM_FTP_CLIENT_COMMAND_DISCONNECT;
                break;
            }
            if(true == TcpClientCheckRespond(/*200*/FTP_CLT_RESP_OK))
            {
                FtpClientCmdState = SM_FTP_CLIENT_COMMAND_DATA3;
            }
            break;
        case SM_FTP_CLIENT_COMMAND_DATA3:
            if(!TCPIsConnected(Socket_FtpCltCmd))
            {
                FtpClientCmdState = SM_FTP_CLIENT_COMMAND_DISCONNECT;
                break;
            }
            if(TCPIsPutReady(Socket_FtpCltCmd) < 30u)
                break;
            TCPPutString(Socket_FtpCltCmd, (const uint8_t*)"RETR ");
            TCPPutString(Socket_FtpCltCmd, (const uint8_t*)FileName);
            TCPPutString(Socket_FtpCltCmd, (const uint8_t*)"\r\n");
            TCPFlush(Socket_FtpCltCmd);
            FtpClientCmdState = SM_FTP_CLIENT_COMMAND_DATA4;
            
            break;
        case SM_FTP_CLIENT_COMMAND_DATA4:
            if(!TCPIsConnected(Socket_FtpCltCmd))
            {
                FtpClientCmdState = SM_FTP_CLIENT_COMMAND_DISCONNECT;
                break;
            }            
            if(true == TcpClientCheckRespond(/*150*/FTP_CLT_RESP_DATA_OPEN))
            {
                FtpClientCmdState = SM_FTP_CLIENT_COMMAND_DATA5;
            }    
            
            break;
        case SM_FTP_CLIENT_COMMAND_DATA5:
            if(!TCPIsConnected(Socket_FtpCltCmd))
            {
                FtpClientCmdState = SM_FTP_CLIENT_COMMAND_DISCONNECT;
                break;
            }            
            if(true == TcpClientCheckRespond(/*226*/FTP_CLT_RESP_DATA_CLOSE))
            {
                FtpClientCmdState = SM_FTP_CLIENT_COMMAND_QUIT1;
            }    
            break;

        case SM_FTP_CLIENT_COMMAND_QUIT1:
            if(!TCPIsConnected(Socket_FtpCltCmd))
            {
                FtpClientCmdState = SM_FTP_CLIENT_COMMAND_DISCONNECT;
                break;
            }
            if(TCPIsPutReady(Socket_FtpCltCmd) < 30u)
                break;
            TCPPutString(Socket_FtpCltCmd, (const uint8_t*)"QUIT\r\n");
            TCPFlush(Socket_FtpCltCmd);
            FtpClientCmdState = SM_FTP_CLIENT_COMMAND_QUIT2;
            break;
        case SM_FTP_CLIENT_COMMAND_QUIT2:
            if(!TCPIsConnected(Socket_FtpCltCmd))
            {
                FtpClientCmdState = SM_FTP_CLIENT_COMMAND_DISCONNECT;
                break;
            }
            if(true == TcpClientCheckRespond(/*221*/FTP_CLT_RESP_QUIT))
            {
                FtpClientCmdState = SM_FTP_CLIENT_COMMAND_DISCONNECT;
            }

            break;

        case SM_FTP_CLIENT_COMMAND_DISCONNECT:
            SYS_CONSOLE_MESSAGE("\r\nClosed---\r\n");
            TCPClose(Socket_FtpCltCmd);
            Socket_FtpCltCmd = INVALID_SOCKET;
            FtpClientCmdState = SM_FTP_CLIENT_COMMAND_DONE;
            break;

        case SM_FTP_CLIENT_COMMAND_DONE:
            // Do nothing unless the user pushes BUTTON1 and wants to restart the whole connection/download process
            // On many boards, SYS_USERIO_BUTTON_0 is assigned to sw1
            // SYS_USERIO_BUTTON_1=sw2 and SYS_USERIO_BUTTON_2=sw3 and SYS_USERIO_BUTTON_3=sw4
            if(SYS_USERIO_ButtonGet((SYS_USERIO_BUTTON_3),SYS_USERIO_BUTTON_ASSERTED))
                FtpClientCmdState = SM_FTP_CLIENT_COMMAND_HOME;
            break;
    }
    // FTP Client DATA State
    switch(FtpClientDataState)
    {
    case SM_FTP_CLIENT_DATA_HOME:
        Socket_FtpCltData = TCPOpenServer(IP_ADDRESS_TYPE_IPV4, Port_FtpCltData, 0);
        if(Socket_FtpCltData == INVALID_SOCKET)
        {
            SYS_CONSOLE_MESSAGE("Can not create FTP Data Server\r\n");
            FtpClientDataState = SM_FTP_CLIENT_DATA_DONE;
        }
        FtpClientDataState = SM_FTP_CLIENT_DATA_WAIT;
        break;
    case SM_FTP_CLIENT_DATA_WAIT:
        if(TCPIsConnected(Socket_FtpCltData)) 
        {
            SYS_CONSOLE_MESSAGE("\r\n!FTP data connected\r\n");
            FtpClientDataState = SM_FTP_CLIENT_DATA_DATA;
        }
        break;
    case SM_FTP_CLIENT_DATA_DATA:
        if(!TCPIsConnected(Socket_FtpCltData))
        {
            FtpClientDataState = SM_FTP_CLIENT_DATA_DISCONNECT;
            break;
        }
        w = TCPIsGetReady(Socket_FtpCltData);
        while(w>0)
        {
            lenB = TCPGetArray(Socket_FtpCltData, vBuffer, ((w <= sizeof(vBuffer)) ? w : sizeof(vBuffer)));
            AU_print_string(vBuffer,lenB);
            w -= lenB;
        }
        break;
    case SM_FTP_CLIENT_DATA_DISCONNECT:
        TCPClose(Socket_FtpCltData);
        SYS_CONSOLE_MESSAGE("\r\nFTP Data Disconneted---\r\n");
        FtpClientDataState = SM_FTP_CLIENT_DATA_DONE;
        break;
    case SM_FTP_CLIENT_DATA_DONE:
        break;
    }
}

#endif  //  defined(APP_USE_FTP_CLIENT_DEMO)

