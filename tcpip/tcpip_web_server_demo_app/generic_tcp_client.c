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
Copyright � 2012 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED �AS IS� WITHOUT WARRANTY OF ANY KIND,
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


#include "tcpip_config.h"
#include "main_demo.h"

#if defined(TCPIP_STACK_USE_TCP) && defined(APP_USE_GENERIC_TCP_CLIENT_DEMO)

#include "tcpip/tcpip.h"
#include <system/system_userio.h>

// enable the output of the server obtained data to a serial channel
// Note: this may dump a lot of data on the serial line and is disabled by default!
//#define GENERIC_TCP_CLIENT_ENABLE_UART_DUMP

// Defines the server to be accessed for this application
static char ServerName[] =    "www.google.com";

// the server address
static IPV4_ADDR  serverIP;

// Defines the port to be accessed for this application
static uint16_t ServerPort = 80;

// Defines the URL to be requested by this HTTP client
static const uint8_t RemoteURL[] = "/search?as_q=Microchip&as_sitesearch=microchip.com";


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
    uint8_t                 i;
    uint16_t                w;
    DNS_RESULT          dnsRes;
    uint8_t                vBuffer[9];
    static TCPIP_NET_HANDLE    netH;
    static uint32_t        clientTimer;
    static TCP_SOCKET    MySocket = INVALID_SOCKET;
    static uint32_t nAttempts =0;

    static enum _GenericTCPExampleState
    {
        SM_HOME = 0,
        SM_WAIT_DNS,
        SM_DNS_RESOLVED,
        SM_SOCKET_OBTAINED,
        SM_PROCESS_RESPONSE,
        SM_DISCONNECT,
        SM_DONE
    } GenericTCPExampleState = SM_DONE;

    //DBPRINTF("    Starting TCP client\n");

    switch(GenericTCPExampleState)
    {

        case SM_HOME:

            DBPRINTF("  SM_HOME\n");
            netH = TCPIP_STACK_GetDefaultNet();
            if(DNSBeginUsage(netH) != DNS_RES_OK)
            {
                break;
            }
            DNSResolve(ServerName, DNS_TYPE_A); 
            GenericTCPExampleState++;
            break;

        case SM_WAIT_DNS:

            DBPRINTF("  SM_WAIT_DNS\n");
            dnsRes = DNSIsResolved(ServerName, &serverIP);
            if(dnsRes == DNS_RES_PENDING)
            {   // ongoing operation;
                break;
            }
            else if(dnsRes < 0)
            {   // some DNS error occurred; retry
                DBPRINTF((const char*)"\r\n\r\nGeneric TCP client: DNS name resolving failed...\r\n");
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

        case SM_DNS_RESOLVED:
            DBPRINTF("  SM_DNS_RESOLVED\n");
            // Connect the socket to the remote TCP server
            MySocket = TCPOpenClient(IP_ADDRESS_TYPE_IPV4, ServerPort, (IP_MULTI_ADDRESS*)&serverIP);

			// Abort operation if no TCP socket could be opened.
			// If this ever happens, you need to update your tcp_config.h
            if(MySocket == INVALID_SOCKET)
            {   // retry
                break;
            }

            GenericTCPExampleState++;
            clientTimer = SYS_TICK_Get();
            break;

        case SM_SOCKET_OBTAINED:

            DBPRINTF("  SM_SOCKET_OBTAINED\n");
            // Wait for the remote server to accept our connection request
            if(!TCPIsConnected(MySocket))
            {
                // Time out if more than 5 seconds is spent in this state
                if((SYS_TICK_Get()-clientTimer) > 5 * SYS_TICK_TicksPerSecondGet() )
                {
                    // Close the socket so it can be used by other modules
                    TCPClose(MySocket);
                    MySocket = INVALID_SOCKET;
                    GenericTCPExampleState--;
                    DBPRINTF((const char*)"\r\n\r\nGeneric TCP client: Failed connecting to the remote server...\r\n");
                }
                break;
            }

            clientTimer = SYS_TICK_Get();

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
            GenericTCPExampleState++;
            break;

        case SM_PROCESS_RESPONSE:
            //DBPRINTF("  SM_PROCESS_RESPONSE\n");
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
#if defined(GENERIC_TCP_CLIENT_ENABLE_UART_DUMP)
                DBPRINTF((char*)vBuffer);
#endif
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
            DBPRINTF("  SM_DISCONNECT\n");
            // Close the socket so it can be used by other modules
            // For this application, we wish to stay connected, but this state will still get entered if the remote server decides to disconnect
            TCPClose(MySocket);
            MySocket = INVALID_SOCKET;
            GenericTCPExampleState = SM_DONE;
            break;

        case SM_DONE:
            //DBPRINTF("  SM_DONE\n");
            // Do nothing unless the user pushes BUTTON1 and wants to restart the whole connection/download process
            // On many boards, SYS_USERIO_BUTTON_0 is assigned to sw1
            // SYS_USERIO_BUTTON_1=sw2 and SYS_USERIO_BUTTON_2=sw3
            if(SYS_USERIO_ButtonGet((SYS_USERIO_BUTTON_1),SYS_USERIO_BUTTON_ASSERTED))
                GenericTCPExampleState = SM_HOME;
            break;
    }
}

#endif  //  defined(TCPIP_STACK_USE_GENERIC_TCP_CLIENT_EXAMPLE)

