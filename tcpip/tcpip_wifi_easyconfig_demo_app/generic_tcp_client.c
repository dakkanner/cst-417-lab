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
FileName:  GenericTCPClient.c
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



//#if defined(TCPIP_STACK_USE_TCP) && defined(APP_USE_GENERIC_TCP_CLIENT_DEMO)

#define TEST_WITH_NO_DNS // For use in concert with SSLTEST within ssl.c

#include "tcpip_config.h"

#include "tcpip/tcpip.h"
#include <system/system_userio.h>

#define BUFFER_SIZE 512

#if defined(TCPIP_STACK_USE_TCP)

#ifdef TCPIP_STACK_USE_SSL_CLIENT
static uint16_t ServerPort = 443; //JBS
#else
static uint16_t ServerPort = 9760; //JBS
#endif

#define SERVER_ADDRESS 0x6901A8C0 //  192.168.1.105 Change to whatever server address actual is

// the server address
static IPV4_ADDR  serverIP;

// Defines the server to be accessed for this application

#ifdef GENERIC_SSLTEST
static char ServerName[] =    "MCHPSERVER";
#else
static uint8_t ServerName[] =   "www.google.com";
#endif

// Defines the URL to be requested by this HTTP client
static const uint8_t RemoteURL[] = "/?as_q=Microchip&as_sitesearch=microchip.com";


/*****************************************************************************
  Function:
    void GenericTCPClient(void)

  Summary:
    Implements a simple HTTP client (over TCP).

  Description:
    This function implements a simple HTTP client, which operates over TCP.
    The function is called periodically by the stack, and waits for BUTTON1
    to be pressed.  When the button is pressed, the application opens a TCP
    connection to an Internet search engine, performs a search for the word
    "Microchip" on "microchip.com", and prints the resulting HTML page to
    the UART.

    This example can be used as a model for many TCP and HTTP client
    applications.

  Precondition:
    TCP is initialized.

  Parameters:
    None

  Returns:
    None
  ***************************************************************************/
void GenericTCPClient(void)
{
    uint32_t             i;
    uint32_t            w;
    uint8_t             vBuffer[BUFFER_SIZE];
    static TCPIP_NET_HANDLE    netH;
    static uint32_t        clientTimer;
    static TCP_SOCKET    MySocket = INVALID_SOCKET;
#ifndef TEST_WITH_NO_DNS
    DNS_RESULT          dnsRes;
    static uint32_t nAttempts =0;
#endif
    static enum _GenericTCPExampleState
    {
        SM_HOME = 0,
#ifndef TEST_WITH_NO_DNS
        SM_WAIT_DNS,
#endif
        SM_DNS_RESOLVED,
        SM_SOCKET_OBTAINED,
        SM_PROCESS_DATA,
        SM_PROCESS_RESPONSE,
        SM_DISCONNECT,
        SM_DONE
    } GenericTCPExampleState = SM_DONE;

    switch(GenericTCPExampleState)
    {

        case SM_HOME:

            netH = TCPIP_STACK_GetDefaultNet();
#ifndef TEST_WITH_NO_DNS
            dnsRes = DNSBeginUsage(netH);
            if(dnsRes != DNS_RES_OK)
                break;
            DNSResolve(ServerName, DNS_TYPE_A);
            GenericTCPExampleState++;
#endif



#ifdef TEST_WITH_NO_DNS
            GenericTCPExampleState=SM_DNS_RESOLVED;
#endif
            break;

#ifndef TEST_WITH_NO_DNS
        case SM_WAIT_DNS:

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
                GenericTCPExampleState = SM_HOME;
                nAttempts++;
                if(nAttempts>8) // After 8 attempts give-up
                {
                    GenericTCPExampleState = SM_DONE;
                    nAttempts=0;
                }
            }
            else
            {
                clientTimer = SYS_TICK_Get();
                GenericTCPExampleState++;
            }
            DNSEndUsage(netH);
        break;
#endif

        case SM_DNS_RESOLVED:
#ifdef TEST_WITH_NO_DNS
    serverIP.Val= SERVER_ADDRESS;
#endif
            // Connect the socket to the remote TCP server
            MySocket = TCPOpenClient(IP_ADDRESS_TYPE_IPV4, ServerPort, (IP_MULTI_ADDRESS*)&serverIP);

			// Abort operation if no TCP socket could be opened.
			// If this ever happens, you need to update your tcp_config.h
            if(MySocket == INVALID_SOCKET)
            {   // retry
                break;
            }

            SYS_CONSOLE_MESSAGE((const char*)"\r\n\r\nConnecting using Microchip TCP API...\r\n");

            GenericTCPExampleState=SM_SOCKET_OBTAINED;
            clientTimer = SYS_TICK_Get();
        break;

        case SM_SOCKET_OBTAINED:

            // Wait for the remote server to accept our connection request
            if(!TCPIsConnected(MySocket))
            {
                // Time out if more than 5 seconds is spent in this state
                if((SYS_TICK_Get()-clientTimer) > 5 * SYS_TICK_TicksPerSecondGet() )
                {
                    // Close the socket so it can be used by other modules
                    TCPClose(MySocket);
                    MySocket = INVALID_SOCKET;
                    GenericTCPExampleState=SM_DNS_RESOLVED;
                    SYS_CONSOLE_MESSAGE((const char*)"\r\n\r\nFailed connecting to the remote server...\r\n");
                }
                break;
            }

            clientTimer = SYS_TICK_Get();
#ifdef TCPIP_STACK_USE_SSL_CLIENT
            if(!TCPStartSSLClient(MySocket,(uint8_t *)"MCHPCLIENT"))
                break;
#endif
             GenericTCPExampleState = SM_PROCESS_DATA;
        break;

        case SM_PROCESS_DATA:
#ifdef TCPIP_STACK_USE_SSL_CLIENT
            if (TCPSSLIsHandshaking(MySocket))
            {
                // Handshaking may fail if the SSL_RSA_CLIENT_SIZE is not large enough
                // for the server?s certificate
                if(SYS_TICK_Get()-clientTimer > 10*SYS_TICK_TicksPerSecondGet())
                {
                    // Close the socket so it can be used by other modules
                    TCPClose(MySocket);
                    MySocket = INVALID_SOCKET;
                    GenericTCPExampleState=SM_HOME;
                }
                break;
            }
#endif

            // Make certain the socket can be written to
            if(TCPIsPutReady(MySocket) < 125u)
                break;

            // Place the application protocol data into the transmit buffer.  For this example, we are connected to an HTTP server, so we'll send an HTTP GET request.
            TCPPutString(MySocket, (const uint8_t*)"GET ");
            TCPPutString(MySocket, RemoteURL);
            TCPPutString(MySocket, (const uint8_t*)" HTTP/1.0\r\nHost: ");
            TCPPutString(MySocket, (const uint8_t*)ServerName);
            TCPPutString(MySocket, (const uint8_t*)"\r\nConnection: close\r\n\r\n");

            // Send the packet
            TCPFlush(MySocket);
            GenericTCPExampleState=SM_PROCESS_RESPONSE;
        break;

        case SM_PROCESS_RESPONSE:
            // Check to see if the remote node has disconnected from us or sent us any application data
            // If application data is available, write it to the UART
            if(!TCPIsConnected(MySocket))
            {
                GenericTCPExampleState = SM_DISCONNECT;
                // Do not break;  We might still have data in the TCP RX FIFO waiting for us
            }

            // Get count of RX bytes waiting
            w = TCPIsGetReady(MySocket);

            // Obtian and print the server reply
            i = sizeof(vBuffer)-1;
            vBuffer[i] = '\0';
            while(w)
            {
                if(w < i)
                {
                    i = w;
                    vBuffer[i] = '\0';
                }
                w -= TCPGetArray(MySocket, vBuffer, i);

                SYS_CONSOLE_MESSAGE((char*)vBuffer);

                // SYS_CONSOLE_MESSAGE is a blocking call which will slow down the rest of the stack
                // if we shovel the whole TCP RX FIFO into the serial port all at once.
                // Therefore, let's break out after only one chunk most of the time.  The
                // only exception is when the remote node disconncets from us and we need to
                // use up all the data before changing states.
                if(GenericTCPExampleState == SM_PROCESS_RESPONSE)
                    break;
            }

        break;

        case SM_DISCONNECT:
            // Close the socket so it can be used by other modules
            // For this application, we wish to stay connected, but this state will still get entered if the remote server decides to disconnect
            TCPClose(MySocket);
            MySocket = INVALID_SOCKET;
            GenericTCPExampleState = SM_DONE;
        break;

        case SM_DONE:
            // Do nothing unless the user pushes BUTTON1 and wants to restart the whole connection/download process
            // On many boards, SYS_USERIO_BUTTON_0 is assigned to sw1
            // SYS_USERIO_BUTTON_1=sw2 and SYS_USERIO_BUTTON_2=sw3
            if(SYS_USERIO_ButtonGet((SYS_USERIO_BUTTON_1),SYS_USERIO_BUTTON_ASSERTED))
                GenericTCPExampleState = SM_HOME;
        break;
    }
}

#endif // #if defined(TCPIP_STACK_USE_TCP)
