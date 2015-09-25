/*******************************************************************************
  MRF24W Driver Medium Access Control (MAC) Layer

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Provides access to MRF24W WiFi controller
    -Reference: MRF24W Data sheet, IEEE 802.11 Standard
*******************************************************************************/

/*******************************************************************************
FileName:  wf_update_firmware_tcpclient_24g.c 
Copyright © 2012 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software foradditional information regarding your rights and obligations.

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

/*
*********************************************************************************************************
*                                           INCLUDES                               
*********************************************************************************************************
*/

#include "tcpip_private.h"

#include "tcpip_mac_private.h"
#include "tcpip/tcpip_mac.h"
#include "tcpip/tcpip_mac_object.h"

#if defined(TCPIP_IF_MRF24W) 
#if defined(WF_UPDATE_FIRMWARE_TCPCLIENT_24G)
#include "tcpip/tcpip.h"
#include <system/system_userio.h>

#define WrittenToFlash  1
//
// This file is used in OTA (over-the-air) web client MRF24WG RF transceiver firmware update.
// Applicable to MRF24WG only.
// MRF24WG0M RTP RF Transceiver FW is 0x3107  ( a2patch_3107_1029.bin)
//
// in file tcp_config.h, please increase the RX size from 512 to 1536
//                                #define TCP_SOCKET_DEFAULT_RX_SIZE   1536 //512
//
// Defines the server to be accessed for this application
static  char ServerName[] = "www.microchip.com"; 
// Defines the port to be accessed for this application
static uint16_t ServerPort = 80;

// Enter RF transceiver FW file name here. Case-sensitive.
//static BYTE PatchName[]  = "/mrfupdates/a2patch_3107_1029.bin";
static BYTE PatchName[]  = "/mrfupdates/a2patch_3108_04222013_a.bin";
//static BYTE PatchName[]  = "/mrfupdates/a2patch_3108_04222013_b.bin";

// Username is mrfupdates , password is mchp1234
static BYTE Key_authorization[]="bXJmdXBkYXRlczptY2hwMTIzNA==" ; 



// the server address
static IPV4_ADDR  serverIP;

extern uint32_t ImageUpdate_Addr;
extern uint32_t ImageUpdate_Checksum ;
extern uint32_t ImageUpdate_Size ;
extern void AutoUpdate_Initialize(void);
extern void AutoUpdate_Completed(void);
extern void AutoUpdate_Restore(void);

//#define SendSetParamMsg_debug   SendSetParamMsg


static void AU_print_string(uint8_t *buf,uint8_t length)
{
    int i;
    for(i=0;i<length;i++) SYS_CONSOLE_PUTC(buf[i]);

}

static void AutoUpdate_SendToModule(uint8_t c, BOOL all)
{
    static uint8_t buf_module[36]={0};
    static uint8_t index_b = 0;
    if(all == TRUE)
    {
        if(index_b >0)
        {
            buf_module[0]=(ImageUpdate_Addr&0x00FF0000)>>16;
            buf_module[1]=(ImageUpdate_Addr&0x0000FF00)>>8;
            buf_module[2]=(ImageUpdate_Addr&0xFF);
            buf_module[3]=index_b;
            _SYS_CONSOLE_PRINT("!");
            SendSetParamMsg(PARAM_FLASH_WRITE, buf_module, index_b+4);
            ImageUpdate_Addr += index_b;
            index_b = 0;
        }
    }
    else
    {
        buf_module[index_b+4] = c;
        index_b ++;
        if(index_b ==32)
        {
            buf_module[0]=(ImageUpdate_Addr&0x00FF0000)>>16;
            buf_module[1]=(ImageUpdate_Addr&0x0000FF00)>>8;
            buf_module[2]=(ImageUpdate_Addr&0xFF);
            buf_module[3]=32;
            //_SYS_CONSOLE_PRINT(".");
            SendSetParamMsg(PARAM_FLASH_WRITE, buf_module, index_b+4);
            ImageUpdate_Addr += 32;
            index_b = 0;
        }
    }
}
/*****************************************************************************
  Function:
    void WF_FirmwareUpdate_TcpClient_24G(void)

  Summary:
     
  Description:

  Precondition:
    TCP is initialized.

  Parameters:
    None

  Returns:
      None
  ***************************************************************************/
void WF_FirmwareUpdate_TcpClient_24G(void)
{
    uint16_t                w;
    uint16_t                 j;
    DNS_RESULT          dnsRes;
    uint8_t             vBuffer[256];
    static TCPIP_NET_HANDLE    netH;
    static TCP_SOCKET    MySocket = INVALID_SOCKET;
    static SYS_TICK        Timer;
    static int SizeOfPackage=0;
    WORD lenA,lenB;
    static enum _AutoUpdateTCPExampleState
    {
        SM_UPDATE_HOME = 0,
        SM_UPDATE_WAIT_DNS,
        SM_UPDATE_DNS_RESOLVED,
        SM_UPDATE_SOCKET_OBTAINED,
        SM_UPDATE_GET_SIZE_1,
        SM_UPDATE_GET_SIZE_2,
        SM_UPDATE_SKIP_TO_DATA,
        SM_UPDATE_READ_DATA,
        SM_UPDATE_DISCONNECT,
        SM_UPDATE_DONE
    } AutoUpdateTCPExampleState = SM_UPDATE_DONE;

    switch(AutoUpdateTCPExampleState)
    {

        case SM_UPDATE_HOME:
            SYS_CONSOLE_MESSAGE((const char*)"\r\n\r\nResolving the server name using Microchip DNS API...\r\n");
            netH = TCPIP_STACK_GetDefaultNet();
            if(DNSBeginUsage(netH) != DNS_RES_OK)
            {
                break;
            }
            DNSResolve(ServerName, DNS_TYPE_A);
            AutoUpdateTCPExampleState++;
            break;

        case SM_UPDATE_WAIT_DNS:

            dnsRes = DNSIsResolved(ServerName, &serverIP);
            if(dnsRes == DNS_RES_PENDING)
            {   // ongoing operation;
                break;
            }
            else if(dnsRes < 0)
            {   // some DNS error occurred; retry
                SYS_CONSOLE_MESSAGE((const char*)"\r\n\r\nDNS name resolving failed...\r\n");
                TCPClose(MySocket);
                MySocket = INVALID_SOCKET;
                AutoUpdateTCPExampleState = SM_UPDATE_HOME;
            }
            else
            {
                Timer = SYS_TICK_Get();
                AutoUpdateTCPExampleState++;
            }
            DNSEndUsage(netH);
            break;

        case SM_UPDATE_DNS_RESOLVED:
            // Connect the socket to the remote TCP server
            MySocket = TCPOpenClient(IP_ADDRESS_TYPE_IPV4, ServerPort, (IP_MULTI_ADDRESS*)&serverIP);

            // Abort operation if no TCP socket could be opened.
            // If this ever happens, you need to update your tcp_config.h
            if(MySocket == INVALID_SOCKET)
            {   // retry
                break;
            }

            SYS_CONSOLE_MESSAGE(( char*)"\r\n\r\nUpdate using Microchip TCP API...\r\n");

            AutoUpdateTCPExampleState++;
            Timer = SYS_TICK_Get();
            break;

        case SM_UPDATE_SOCKET_OBTAINED:
            // Wait for the remote server to accept our connection request
            if(!TCPIsConnected(MySocket))
            {
                // Time out if more than 5 seconds is spent in this state
                if((SYS_TICK_Get()-Timer) > 5 * SYS_TICK_TicksPerSecondGet() )
                {
                    // Close the socket so it can be used by other modules
                    TCPClose(MySocket);
                    MySocket = INVALID_SOCKET;
                    AutoUpdateTCPExampleState--;
                    SYS_CONSOLE_MESSAGE((const char*)"\r\n\r\nFailed connecting to the remote server...\r\n");
                }
                break;
            }
#if WrittenToFlash
            AutoUpdate_Initialize();
#endif
            Timer = SYS_TICK_Get();

            // Make certain the socket can be written to
            if(TCPIsPutReady(MySocket) < 125u)
                break;


            TCPPutString(MySocket, ( BYTE*)"GET ");
            TCPPutString(MySocket, ( BYTE*)PatchName);
            TCPPutString(MySocket, ( BYTE*)" HTTP/1.1\r\nHost:");
            TCPPutString(MySocket, ( BYTE*)ServerName);
            TCPPutString(MySocket, ( BYTE*)"\r\nConnection: close\r\n");
            TCPPutString(MySocket, ( BYTE*)"Authorization: Basic ");
            TCPPutString(MySocket, ( BYTE*)Key_authorization);
            TCPPutString(MySocket, ( BYTE*)"\r\n\r\n");

            // Send the packet
            TCPFlush(MySocket);
            AutoUpdateTCPExampleState++;
            break;
        case SM_UPDATE_GET_SIZE_1:  //Looking for "Length:"
            if(!TCPIsConnected(MySocket))
            {
                AutoUpdateTCPExampleState = SM_UPDATE_DISCONNECT;
                break;
            }
            // 2. Check ,if there is data?
            w = TCPIsGetReady(MySocket);        
            if(w ==0) break;
            
            lenA = TCPFindArray(MySocket, ( BYTE*)"Length:", 7, 0, 0,FALSE);
            if(lenA != 0xffff)
            {// Found it, so remove all data up to and including
                while(lenA>0)
                {
                    lenB = TCPGetArray(MySocket, vBuffer, ((lenA <= sizeof(vBuffer)) ? lenA : sizeof(vBuffer)));
                    AU_print_string(vBuffer,lenB);
                    lenA -= lenB;
                }
                AutoUpdateTCPExampleState = SM_UPDATE_GET_SIZE_2;
                
                // No break if we found the "Length:"    
            }
            else
            {// Otherwise, remove as much as possible
            
                w -= 7;
                while(w>0)
                {
                    lenB = TCPGetArray(MySocket, vBuffer, ((w <= sizeof(vBuffer)) ? w : sizeof(vBuffer)));
                    AU_print_string(vBuffer,lenB);
                    w -= lenB;
                }
                break;
            }
        case SM_UPDATE_GET_SIZE_2:  //Looking for "\r\n",if yes, we get the size of package
            if(!TCPIsConnected(MySocket))
            {
                AutoUpdateTCPExampleState = SM_UPDATE_DISCONNECT;
                break;
            }
            // 2. Check ,if there is data?
            w = TCPIsGetReady(MySocket);        
            if(w ==0) break;
            lenA = TCPFindArray(MySocket, ( BYTE*)"\r\n", 2, 0, 0, FALSE);
            if(lenA != 0xffff)
            {// Found it, get the size
                lenA = TCPGetArray(MySocket, vBuffer, lenA+2);    
                AU_print_string(vBuffer,lenA);
                sscanf((const char*)vBuffer,"Length: %d\r\n",&SizeOfPackage);
                AutoUpdateTCPExampleState = SM_UPDATE_SKIP_TO_DATA;
                // No break if we found the size
            }
            else
            {// Otherwise, wait to next read
                break;
            }

        case SM_UPDATE_SKIP_TO_DATA: // look for "\r\n\r\n"
            // 1. Check to see if the remote node has disconnected from us or sent us any application data
            if(!TCPIsConnected(MySocket))
            {
                AutoUpdateTCPExampleState = SM_UPDATE_DISCONNECT;
                // Do not break;  We might still have data in the TCP RX FIFO waiting for us
            }

            // 2. Check ,if there is data?
            w = TCPIsGetReady(MySocket);        
            if(w ==0) break;
            lenA = TCPFindArray(MySocket, ( BYTE*)"\r\n\r\n", 4, 0, 0,FALSE);
    
            if(lenA != 0xffff)
            {// Found it, so remove all data up to and including
                while(lenA>0)
                {
                    lenB = TCPGetArray(MySocket, vBuffer, ((lenA <= sizeof(vBuffer)) ? lenA : sizeof(vBuffer)));
                    AU_print_string(vBuffer,lenB);
                    lenA -= lenB;
                }
                lenB = TCPGetArray(MySocket, vBuffer, 4);
                AU_print_string(vBuffer,lenB);
                AutoUpdateTCPExampleState = SM_UPDATE_READ_DATA;
                
                // No break if we found the header terminator    
            }
            else
            {// Otherwise, remove as much as possible
                
                w -= 4;
                while(w>0)
                {
                    lenB = TCPGetArray(MySocket, vBuffer, ((w <= sizeof(vBuffer)) ? w : sizeof(vBuffer)));
                    AU_print_string(vBuffer,lenB);
                    w -= lenB;
                }
                //
                break;
            }
                        
        case SM_UPDATE_READ_DATA:
            if(!TCPIsConnected(MySocket))
            {
                AutoUpdateTCPExampleState = SM_UPDATE_DISCONNECT;
                // Do not break;  We might still have data in the TCP RX FIFO waiting for us
            }
            // Get count of RX bytes waiting
            w = TCPIsGetReady(MySocket);
            j = sizeof(vBuffer)-1;
            if(w!=0){ SYS_CONSOLE_MESSAGE("-!\n!-Total=%u,Current=%ud, size=%u: ",SizeOfPackage,ImageUpdate_Size,w);}
            while(w)
            {
                int i;
                if(w < j){j = w; }
                vBuffer[j] = '\0';
                lenB= TCPGetArray(MySocket, vBuffer, j);
                
                w -= lenB;
                
                for(i=0;i<lenB;i++)
                {
                    //Size and CheckSum
                    if((ImageUpdate_Size % 4u) == 0u) ImageUpdate_Checksum += (uint32_t)vBuffer[i]<<24;
                    if((ImageUpdate_Size % 4u) == 1u) ImageUpdate_Checksum += (uint32_t)vBuffer[i]<<16;
                    if((ImageUpdate_Size % 4u) == 2u) ImageUpdate_Checksum += (uint32_t)vBuffer[i]<<8;
                    if((ImageUpdate_Size % 4u) == 3u) ImageUpdate_Checksum += (uint32_t)vBuffer[i];
                    ImageUpdate_Size ++;
                    
                    //{ SYS_CONSOLE_MESSAGE("%c",vBuffer[i]);}
#if WrittenToFlash                    
                    AutoUpdate_SendToModule(vBuffer[i],FALSE);
#endif
                }
                { SYS_CONSOLE_MESSAGE(",%d",lenB);}
                
            }
            break;

        case SM_UPDATE_DISCONNECT:
            // Close the socket so it can be used by other modules
            // For this application, we wish to stay connected, but this state will still get entered if the remote server decides to disconnect
            TCPClose(MySocket);
            MySocket = INVALID_SOCKET;
            AutoUpdateTCPExampleState = SM_UPDATE_DONE;
            //did we receive enough data? Is the size correct?
            if( SizeOfPackage == ImageUpdate_Size)
            {    
#if WrittenToFlash            
                AutoUpdate_SendToModule(0,TRUE);  //Send all data left in buffer
                AutoUpdate_Completed();
#endif
                SYS_CONSOLE_MESSAGE("update done...\r\n");
            }
            else   //We lost data, so cancel , and roll back
            {
                SYS_CONSOLE_MESSAGE("Revert begin...\r\n");
#if WrittenToFlash                
                AutoUpdate_Restore();
#endif
                SYS_CONSOLE_MESSAGE("Revert Done\r\n");
            }
            break;

        case SM_UPDATE_DONE:
            // Do nothing unless the user pushes BUTTON1 and wants to restart the whole connection/download process
            // On many boards, SYS_USERIO_BUTTON_0 is assigned to sw1
            // SYS_USERIO_BUTTON_1=sw2 and SYS_USERIO_BUTTON_2=sw3
            if(SYS_USERIO_ButtonGet((SYS_USERIO_BUTTON_2),SYS_USERIO_BUTTON_ASSERTED))
            {
                AutoUpdateTCPExampleState = SM_UPDATE_HOME;
            }
            break;
    }
}



#endif
#endif
