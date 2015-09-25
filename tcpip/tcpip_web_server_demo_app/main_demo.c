/*******************************************************************************
  Main Application Entry Point and TCP/IP Stack Demo

  Summary:
    Module for Microchip TCP/IP Stack

  Description:
    -Demonstrates how to call and use the Microchip TCP/IP stack
    -Reference: Microchip TCP/IP Stack Help (tcpip Help.chm)
 *******************************************************************************/

/*******************************************************************************
FileName:  MainDemo.c
Copyright © 2012 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
7INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/

#define _SUPPRESS_PLIB_WARNING 1


#define TCPIP_STACK_MODULE_CONFIGURATION
#include <stdlib.h>     /* strtol */

// Include all headers for any enabled tcpip functions
#include "tcpip/tcpip.h"

#include "tcpip_modules_config.h"

// Include functions specific to this stack application
#include "main_demo.h"
#include <system/system_userio.h>

#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)

int stackNotifyCnt = 0;
const void* stackNotifyHandle;
TCPIP_EVENT stackTotEvents = 0;


#if defined(TCPIP_IF_MRF24W)
uint16_t mrf_txEvents = 0, mrf_txEventInfo = 0;
uint16_t mrf_mgmtEvents = 0, mrf_mgmtEventInfo = 0;
int mrf_EventCnt = 0;
#endif
#endif  // defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)


// Local functions prototypes

#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
static void StackNotification(const void* hIf, TCPIP_EVENT event, const void* nParam);
static void ProcessNotification(const void* hIf);
#if defined(TCPIP_IF_MRF24W)
extern void WF_ProcessEvent(uint16_t event, uint16_t eventInfo);
#endif
#endif  // defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)

void ICMPv6Callback(TCPIP_NET_HANDLE hNetIf, uint8_t type, IPV6_ADDR * localIP, IPV6_ADDR * remoteIP, void * header);

static enum _UDPServerState
{
    SM_HOME = 0,
    SM_LISTENING,
    SM_CLOSING,
} UDPServerState = SM_HOME;

static IPV4_ADDR remoteAddress;
static int port;
uint8_t outputMessage[512];


//
// Main application entry point.
//
int main(void)
{
    static IPV4_ADDR dwLastIP[sizeof (TCPIP_HOSTS_CONFIGURATION) / sizeof (*TCPIP_HOSTS_CONFIGURATION)];
    int i, nNets;
#if defined(SYS_USERIO_ENABLE)    
    static SYS_TICK startTick = 0;
    int32_t LEDstate=SYS_USERIO_LED_DEASSERTED;
#endif  // defined(SYS_USERIO_ENABLE)   
    TCPIP_NET_HANDLE netH;
    const char  *netName=0;
    const char  *netBiosName;
#if defined (TCPIP_STACK_USE_ZEROCONF_MDNS_SD)
    char mDNSServiceName[] = "MyWebServiceNameX ";     // base name of the service Must not exceed 16 bytes long
                                                       // the last digit will be incremented by interface
#endif  // defined (TCPIP_STACK_USE_ZEROCONF_MDNS_SD)

    // 192.168.0.101
    //remoteAddress.Val = 0x6500A8C0;
    // 192.168.0.12
    remoteAddress.Val = 0x1000A8C0;
    port = 9930;
    strcpy((char*)outputMessage, "Hola mundo");

    DBINIT();

    // perform system initialization
    if(!SYS_Initialize())
    {
        return 0
                ;
    }

    DBPRINTF("TCPIPStack Version: ");
    DBPRINTF((const void*)TCPIP_STACK_VERSION);
    DBPRINTF("\n");
    
    // Initialize the TCPIP stack
    if (!TCPIP_STACK_Init(TCPIP_HOSTS_CONFIGURATION, sizeof (TCPIP_HOSTS_CONFIGURATION) / sizeof (*TCPIP_HOSTS_CONFIGURATION),
            TCPIP_STACK_MODULE_CONFIG_TBL, sizeof (TCPIP_STACK_MODULE_CONFIG_TBL) / sizeof (*TCPIP_STACK_MODULE_CONFIG_TBL)))
    {
        return 0;
    }
 
    // Display the names associated with each interface
    // Perform mDNS registration if mDNS is enabled
    nNets = TCPIP_STACK_NetworksNo();
    for(i = 0; i < nNets; i++)
    {
        netH = TCPIP_STACK_IxToNet(i);
        netName = TCPIP_STACK_NetName(netH);
        netBiosName = TCPIP_STACK_NetBIOSName(netH);

#if defined(TCPIP_STACK_USE_NBNS)
        DBPRINTF("    Interface %s on host %s - NBNS enabled\r\n", netName, netBiosName);
#else
        DBPRINTF("    Interface %s on host %s - NBNS disabled\r\n", netName, netBiosName);
#endif  // defined(TCPIP_STACK_USE_NBNS)
    
    
#if defined (TCPIP_STACK_USE_ZEROCONF_MDNS_SD)
        mDNSServiceName[sizeof(mDNSServiceName) - 2] = '1' + i;
        mDNSServiceRegister( netH
            , mDNSServiceName                   // name of the service
            ,"_http._tcp.local"                 // type of the service
            ,80                                 // TCP or UDP port, at which this service is available
            ,((const BYTE *)"path=/index.htm")  // TXT info
            ,1                                  // auto rename the service when if needed
            ,NULL                               // no callback function
            ,NULL);                             // no application context
#endif //TCPIP_STACK_USE_ZEROCONF_MDNS_SD
    }

#if defined (TCPIP_STACK_USE_IPV6)
    TCPIP_ICMPV6_RegisterCallback(ICMPv6Callback);
#endif

#if defined(TCPIP_STACK_USE_ICMP_CLIENT)
    ICMPRegisterCallback(PingProcessIPv4);
#endif


#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
    TCPIP_NET_HANDLE hWiFi = TCPIP_STACK_NetHandle("MRF24W");
    if (hWiFi)
    {
        TCPIP_STACK_RegisterHandler(hWiFi, TCPIP_EV_RX_ALL | TCPIP_EV_TX_ALL | TCPIP_EV_RXTX_ERRORS, StackNotification, 0);
    }
#endif  // defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)



    // Now that all items are initialized, begin the co-operative
    // multitasking loop.  This infinite loop will continuously
    // execute all stack-related tasks, as well as your own
    // application's functions.  Custom functions should be added
    // at the end of this loop.
    // Note that this is a "co-operative mult-tasking" mechanism
    // where every task performs its tasks (whether all in one shot
    // or part of it) and returns so that other tasks can do their
    // job.
    // If a task needs very long time to do its job, it must be broken
    // down into smaller pieces so that other tasks can have CPU time.
    while (1)
    {
        SYS_Tasks();

#if defined(SYS_USERIO_ENABLE)    
        // Blink LED0 (right most one) every second.
        if (SYS_TICK_Get() - startTick >= SYS_TICK_TicksPerSecondGet() / 2ul)
        {
            startTick = SYS_TICK_Get();
            LEDstate ^= SYS_USERIO_LED_ASSERTED;
            SYS_USERIO_SetLED(SYS_USERIO_LED_0, LEDstate);
        }
#endif  // defined(SYS_USERIO_ENABLE)   

        // This task performs normal stack task including checking
        // for incoming packet, type of packet and calling
        // appropriate stack entity to process it.
        TCPIP_STACK_Task();

        // Process application specific tasks here.
        // For this demo app, this will include the Generic TCP
        // client and servers, and the SNMP, Ping, and SNMP Trap
        // demos.  Following that, we will process any IO from
        // the inputs on the board itself.
        // Any custom modules or processing you need to do should
        // go here.
#if defined(TCPIP_STACK_USE_TCP) && defined(APP_USE_GENERIC_TCP_CLIENT_DEMO) 
        GenericTCPClient();
#endif

#if defined(TCPIP_STACK_USE_TCP) && defined(APP_USE_GENERIC_TCP_SERVER_DEMO)
        GenericTCPServer();
#endif

#if defined(TCPIP_STACK_USE_SMTP_CLIENT) && defined(APP_USE_SMTP_CLIENT_DEMO)
        SMTPDemo();
#endif


#if (defined(TCPIP_STACK_USE_ICMP_CLIENT) || defined (TCPIP_STACK_USE_IPV6)) && defined(APP_USE_PING_DEMO)
        // use ping on the default interface
        PingDemoTask();
#endif

#if defined(TCPIP_STACK_USE_SNMP_SERVER) && !defined(SNMP_TRAP_DISABLED)

        // User should use one of the following SNMP demo
        // This routine demonstrates V1 or V2 trap formats with one variable binding.

		//SNMPTrapDemo(); //This function sends the both SNMP trap version1 and 2 type of notifications

		#if defined(SNMP_STACK_USE_V2_TRAP) || defined(SNMP_V1_V2_TRAP_WITH_SNMPV3)
		//This routine provides V2 format notifications with multiple (3) variable bindings
		//User should modify this routine to send v2 trap format notifications with the required varbinds.
		    SNMPV2TrapDemo(); //This function sends the SNMP trap version 2 type of notifications
		#endif

		 /* 
		 SNMPSendTrap() is used to send trap notification to previously configured ip address if trap notification is enabled. 
		 There are different trap notification code. The current implementation sends trap for authentication failure (4).
	  	 PreCondition: If application defined event occurs to send the trap. Declare a notification flag and update as the event occurs.
	  	 Uncomment the below function if the application requires.	

		 if(notification flag is updated by the application as a predefined event occured)
		{
			SNMPSendTrap();
		}

		*/

#endif 


#if defined(TCPIP_STACK_USE_BERKELEY_API) && defined(APP_USE_BERKELEY_API_DEMO)
        BerkeleyTCPClientDemo();
        BerkeleyTCPServerDemo();
        BerkeleyUDPClientDemo(0);
#endif

        // If the local IP address has changed (ex: due to DHCP lease change)
        // write the new IP address to the console display, UART, and Announce
        // service
        // We use the default interface
        for (i = 0; i < sizeof (TCPIP_HOSTS_CONFIGURATION) / sizeof (*TCPIP_HOSTS_CONFIGURATION); i++)
        {
            netH = TCPIP_STACK_NetHandle(TCPIP_HOSTS_CONFIGURATION[i].interface);
            if ((uint32_t) dwLastIP[i].Val != TCPIP_STACK_NetAddress(netH))
            {
                dwLastIP[i].Val = TCPIP_STACK_NetAddress(netH);

                DBPRINTF("Interface Name is: %s\r\n", TCPIP_HOSTS_CONFIGURATION[i].interface);
                DBPRINTF("New IP Address is: "); DisplayIPValue(dwLastIP[i]);
                DBPRINTF("\r\n");
            }
        }

        //TODO Place code here to poll buttons and perform specified task

        DoTheThing(netH);

#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
        if (stackNotifyCnt)
        {
            stackNotifyCnt = 0;
            ProcessNotification(stackNotifyHandle);
        }
#endif  // defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
    }
}


// Writes an IP address to the LCD display and the UART as available
void DisplayIPValue(IPV4_ADDR IPVal)
{
    char message[16 + 1];

    //   printf("%u.%u.%u.%u", IPVal.v[0], IPVal.v[1], IPVal.v[2], IPVal.v[3]);
#if defined (__dsPIC33E__) || defined (__PIC24E__)
    static uint8_t IPDigit[4]; /* Needs to be declared as static to avoid the array getting optimized by C30 v3.30 compiler for dsPIC33E/PIC24E.
              Otherwise the LCD displays corrupted IP address on Explorer 16. To be fixed in the future compiler release*/
#else
    uint8_t IPDigit[4];
#endif
    int i;

    strcpy(message, "");
    for (i = 0; i < sizeof (IPV4_ADDR); i++)
    {
        uitoa((uint16_t) IPVal.v[i], IPDigit);
        strcat(message, (char *) IPDigit);
        if (i < sizeof (IPV4_ADDR) - 1)
        {
            strcat(message, ".");
        }
    }
    SYS_OUT_MESSAGE_LINE(message, 1);
    DBPRINTF((char *) message);
}

#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)

static void StackNotification(const void* hIf, TCPIP_EVENT event, const void* nParam)
{
    stackNotifyCnt++;
    stackNotifyHandle = hIf;
    stackTotEvents |= event;
}

static void ProcessNotification(const void* hIf)
{
#if defined(TCPIP_IF_MRF24W)

    mrf_txEvents = MRF24W_GetTrafficEvents(&mrf_txEventInfo);
    mrf_mgmtEvents = MRF24W_GetMgmtEvents(&mrf_mgmtEventInfo);
    mrf_EventCnt++;
    if (mrf_mgmtEvents)
    {
        WF_ProcessEvent(mrf_mgmtEvents, mrf_mgmtEventInfo);
    }
#endif
}
#endif  // defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)


#if defined (TCPIP_STACK_USE_IPV6)

void ICMPv6Callback(TCPIP_NET_HANDLE hNetIf, uint8_t type, IPV6_ADDR * localIP, IPV6_ADDR * remoteIP, void * header)
{
    switch (((ICMPV6_HEADER_ERROR *) header)->vType)
    {
        case ICMPV6_ERROR_DEST_UNREACHABLE:

            break;
        case ICMPV6_ERROR_PACKET_TOO_BIG:

            break;
        case ICMPV6_ERROR_TIME_EXCEEDED:

            break;
        case ICMPV6_ERROR_PARAMETER_PROBLEM:

            break;
        case ICMPV6_INFO_ECHO_REQUEST:

            break;
        case ICMPV6_INFO_ECHO_REPLY:
            PingProcessIPv6((ICMPV6_HEADER_ECHO *) header, remoteIP, localIP);
            break;
    }
}
#endif

int DoTheThing(TCPIP_NET_HANDLE netH)
{
//
// Lab 05 Start
//
    int buttons_pressed;

    uint8_t i, j;
    uint16_t w, w2;
    uint8_t AppBuffer[512];
    uint16_t wMaxGet, wMaxPut, wCurrentChunk;

    static UDP_SOCKET sokt_serv;
    static UDP_SOCKET sokt_client;

    UDPSocketSetNet(sokt_serv, netH);
    UDPSetOptions(sokt_serv, UDP_OPTION_STRICT_NET, (void*)true);
    UDPSocketSetNet(sokt_client, netH);
    UDPSetOptions(sokt_client, UDP_OPTION_STRICT_NET, (void*)true);

    for(j = 0; j < 255; j++)
    {
        switch(UDPServerState)
        {
        case SM_HOME:
            // Allocate a socket for this server to listen and accept connections on
            sokt_serv = UDPOpenServer(IP_ADDRESS_TYPE_IPV4, port, NULL);
            if(sokt_serv == INVALID_SOCKET)
            {
                DBPRINTF("  Error: Can't open UDP server socket\n");
            }
            sokt_client = UDPOpenClient(IP_ADDRESS_TYPE_IPV4, port, (IP_MULTI_ADDRESS*)&remoteAddress);
            if(sokt_client == INVALID_SOCKET)
            {
                DBPRINTF("  Error: Can't open UDP client socket\n");
            }

            UDPFlush(sokt_serv);

            UDPServerState = SM_LISTENING;
            DBPRINTF("  State: Changing from HOME to LISTENING\n");

            DBPRINTF("      Destination IP: '%i.%i.%i.%i'\r\n", remoteAddress.v[0],
                remoteAddress.v[1], remoteAddress.v[2], remoteAddress.v[3]);

            DBPRINTF("      Output port: '%i'\r\n", port);

            DBPRINTF("      Output message: '");
            DBPRINTF((char*)outputMessage);
            DBPRINTF("'\r\n");
            break;

        case SM_LISTENING:
            // See if anyone is connected to us
            if(!UDPIsConnected(sokt_serv))
            {
                DBPRINTF("  Error: Server listening, but not connected\n");
                UDPServerState = SM_HOME;
                break;
            }

            // Read the current button press(es)
            buttons_pressed = mPORTDRead();
            // Check if button1 is low (pressed)
            //     Send "Hello world" message
            if ( (buttons_pressed & BIT_6) == 0)
            {
                static char helloWorld[] = "Hello world";

                DBPRINTF("  bit 6");
                while(UDPIsTxPutReady(sokt_client, sizeof(helloWorld)) < sizeof(helloWorld))
                    DBPRINTF(".");

                UDPPutString(sokt_client, (uint8_t*)helloWorld);
                UDPFlush(sokt_client);
                DBPRINTF("  Sent default message '");
                DBPRINTF(helloWorld);
                DBPRINTF("'\r\n");

                //Wait until the button is released
                while ((mPORTDRead() & BIT_6) == 0);
            }
            // Check if button2 is low (pressed)
            //     Send "Hello World"
            if ( (buttons_pressed & BIT_7) == 0)
            {
                char tempMessage[256] = "";
                DBPRINTF("Message to Send (128 chars max): ");
                DBGETS((uint8_t*)tempMessage, 128);
                DBPRINTF("\n");
                
                while(UDPIsTxPutReady(sokt_client, sizeof(tempMessage)) < sizeof(tempMessage))
                    DBPRINTF(".");

                UDPPutString(sokt_client, (uint8_t*)tempMessage);
                UDPFlush(sokt_client);
                DBPRINTF("  Sent custom basic message '");
                DBPRINTF(tempMessage);
                DBPRINTF("'\r\n");

                //Wait until the button is released
                while ((mPORTDRead() & BIT_7) == 0);
            }
            // Check if button3 is low (pressed)
            //     Send custom message
            if ( (buttons_pressed & BIT_13) == 0)
            {
                DBPRINTF("  Button 3 pressed (BIT_13)\r\n");
                while(UDPIsTxPutReady(sokt_client, sizeof(outputMessage)) < sizeof(outputMessage))
                    DBPRINTF(".");

                UDPPutString(sokt_client, outputMessage);
                UDPFlush(sokt_client);
                DBPRINTF("  Sent custom message '");
                DBPRINTF((char*)outputMessage);
                DBPRINTF("'\r\n");

                //Wait until the button is released
                while ((mPORTDRead() & BIT_13) == 0);
            }

            // Figure out how many bytes have been received and how many we can transmit.
            wMaxGet = UDPIsGetReady(sokt_serv);    // Get UDP RX FIFO byte count
            wMaxPut = UDPIsTxPutReady(sokt_serv, 32);    // Get UDP TX FIFO free space

            // Make sure we don't take more bytes out of the RX FIFO than we can put into the TX FIFO
            if(wMaxPut < wMaxGet)
            {
                wMaxGet = wMaxPut;
            }

            // Process all bytes that we can
            // This is implemented as a loop, processing up to sizeof(AppBuffer) bytes at a time.
            // This limits memory usage while maximizing performance.
            // Single byte Gets and Puts are a lot slower than multibyte GetArrays and PutArrays.
            wCurrentChunk = sizeof(AppBuffer);
            for(w = 0; w < wMaxGet; w += sizeof(AppBuffer))
            {
                // Make sure the last chunk, which will likely be smaller than sizeof(AppBuffer), is treated correctly.
                if(w + sizeof(AppBuffer) > wMaxGet)
                    wCurrentChunk = wMaxGet - w;

                // Transfer the data out of the UDP RX FIFO and into our local processing buffer.
                UDPGetArray(sokt_serv, AppBuffer, wCurrentChunk);

                // Perform the "ToUpper" operation on each data byte
                for(w2 = 0; w2 < wCurrentChunk; w2++)
                {
                    i = AppBuffer[w2];
                    if(i == '\e')   //escape
                    {
                        sokt_serv = SM_CLOSING;
                        DBPRINTF("  State: Changing from LISTENING to CLOSING\n");
                    }
                }

                DBPRINTF((char*)AppBuffer);
                DBPRINTF("\n");

                // Transfer the data out of our local processing buffer and into the UDP TX FIFO.
                UDPPutArray(sokt_serv, AppBuffer, wCurrentChunk);
            }

            // No need to perform any flush.  TCP data in TX FIFO will automatically
            // transmit itself after it accumulates for a while.  If you want to
            // decrease latency (at the expense of wasting network bandwidth on TCP
            // overhead), perform and explicit flush via the TCPFlush() API.
            break;

        case SM_CLOSING:
            // Close the socket connection.
            UDPClose(sokt_serv);

            UDPServerState = SM_HOME;
            DBPRINTF("  State: Changing from CLOSING to HOME\n");
            break;

        default:
            UDPServerState = SM_HOME;
            DBPRINTF("  State: Changing from unknown to HOME\n");
            break;
        }
    }
    return 0;
}

// Setter used by the web interface
int SetDestinationAddressFromString(char* addressIn)
{
    char octetS1[4] = "000";     // Most significant octet
    char octetS2[4] = "000";
    char octetS3[4] = "000";
    char octetS4[4] = "000";     // Least significant octet

    int octet1 = 0;
    int octet2 = 0;
    int octet3 = 0;
    int octet4 = 0;

    if(strlen(addressIn) != 15)
    {
        return 1;
    }
    else
    {
        octetS1[0] = addressIn[0];
        octetS1[1] = addressIn[1];
        octetS1[2] = addressIn[2];
        octetS1[3] = 0;

        octetS2[0] = addressIn[4];
        octetS2[1] = addressIn[5];
        octetS2[2] = addressIn[6];
        octetS2[3] = 0;

        octetS3[0] = addressIn[8];
        octetS3[1] = addressIn[9];
        octetS3[2] = addressIn[10];
        octetS3[3] = 0;

        octetS4[0] = addressIn[12];
        octetS4[1] = addressIn[13];
        octetS4[2] = addressIn[14];
        octetS4[3] = 0;

        octet1 = strtol(octetS1, NULL, 10);
        octet2 = strtol(octetS2, NULL, 10);
        octet3 = strtol(octetS3, NULL, 10);
        octet4 = strtol(octetS4, NULL, 10);

        // Most significant octet
        remoteAddress.v[0] = octet1;
        remoteAddress.v[1] = octet2;
        remoteAddress.v[2] = octet3;
        // Least significant octet
        remoteAddress.v[3] = octet4;

        DBPRINTF("New remote address is: %i.%i.%i.%i", octet1, octet2, octet3, octet4);
        DBPRINTF("\n");
        DBPRINTF("Resetting UDP server state\n");

        UDPServerState = SM_HOME;
    }
    return 0;
}

// Getter used by the web interface
void GetDestinationAddress(char* buffer)
{
    sprintf((char*)buffer, "%03d.%03d.%03d.%03d", remoteAddress.v[0], remoteAddress.v[1], remoteAddress.v[2], remoteAddress.v[3]);
}

// Setter used by the web interface
int SetPortFromString(char* portIn)
{
    port = strtol(portIn, NULL, 10);

    DBPRINTF("New port is: %i", port);
    DBPRINTF("\n");
    DBPRINTF("Resetting UDP server state\n");

    UDPServerState = SM_HOME;
    return 0;
}

// Getter used by the web interface
void GetPort(char* buffer)
{
    sprintf((char*)buffer, "%d", port);
}

// Setter used by the web interface
int SetMessageToSend(char* messageIn)
{
    strcpy((char*)outputMessage, messageIn);

    DBPRINTF("New message is: %s", (char*)outputMessage);
    DBPRINTF("\n");

    return strcmp((char*)outputMessage, messageIn);
}

// Getter used by the web interface
void GetMessage(char* buffer)
{
    strcpy(buffer, (char*)outputMessage);
}