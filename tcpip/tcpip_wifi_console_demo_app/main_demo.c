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
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
#define TCPIP_STACK_MODULE_CONFIGURATION
// Include all headers for any enabled tcpip functions
#include "tcpip/tcpip.h"
#include "wifi/wf_easy_config.h"
#include "system_config.h"

#include "tcpip_modules_config.h"
#include "wifi/mrf24w_mac.h"
#include "tcpip/ipv4.h"
#include "tcpip/ipv6.h"
#include "tcpip_mac_object.h"

#if defined (__C32__)
    #include <peripheral/timer.h>
    #include <peripheral/system.h>
    #include <peripheral/rtcc.h>
    #include <system_services.h>
#elif defined (__C30__)
	#include <spi.h>
	#include <timer.h>
#endif

// Include functions specific to this stack application
#include "main_demo.h"
#include <system/system_userio.h>

#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)

int stackNotifyCnt = 0;
const void* stackNotifyHandle;
TCPIP_EVENT stackTotEvents = 0;

#if defined(HOST_CM_TEST)
extern UINT8 g_event;
#endif

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


#if defined(HOST_CM_TEST)
    #include "TCPIP/udp.h"
    UDP_SOCKET socket1;
    UDP_SOCKET socket2;
    UINT32 tx_number=0;
    UINT32 timeudp=0;
    char str[1000];
    UINT16 cntstr;
    extern bool g_DhcpSuccessful;
#endif

//
// Main application entry point.
//
int main(void)
{

#if defined(HOST_CM_TEST)
    DWORD t1 = 0;
    char st[80];
    BOOL host_scan = FALSE;
    UINT16 scan_count = 0;
#endif

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

    // perform system initialization
    if(!SYS_Initialize())
    {
        return 0;
    }


    SYS_CONSOLE_MESSAGE("\r\n\n\n ---  TCPIP Demo Starts!  --- \r\n");
    SYS_OUT_MESSAGE("TCPIPStack " TCPIP_STACK_VERSION "  ""                ");

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
        SYS_CONSOLE_PRINT("    Interface %s on host %s - NBNS enabled\r\n", netName, netBiosName);
#else
        SYS_CONSOLE_PRINT("    Interface %s on host %s - NBNS disabled\r\n", netName, netBiosName);
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

#if defined(WF_UPDATE_FIRMWARE_UART_24G)
    extern bool    WF_FirmwareUpdate_Uart_24G(void);
    WF_FirmwareUpdate_Uart_24G();
#endif

  

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
#if defined(TCPIP_STACK_USE_TCP) && defined(APP_USE_FTP_CLIENT_DEMO)
        FTPClient();
#endif        
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

                SYS_CONSOLE_PRINT("Interface Name is: %s\r\n", TCPIP_HOSTS_CONFIGURATION[i].interface);
                SYS_CONSOLE_MESSAGE("New IP Address is: "); DisplayIPValue(dwLastIP[i]);
                SYS_CONSOLE_MESSAGE("\r\n");
            }
        }

#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
        if (stackNotifyCnt)
        {
            stackNotifyCnt = 0;
            ProcessNotification(stackNotifyHandle);
        }
#endif  // defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)

#if defined(WF_UPDATE_FIRMWARE_TCPCLIENT_24G)
    void WF_FirmwareUpdate_TcpClient_24G(void);
    WF_FirmwareUpdate_TcpClient_24G();
#endif //defined(WF_UPDATE_FIRMWARE_TCPCLIENT_24G)

#if defined(HOST_CM_TEST)
       switch (g_event)
        {
            case WF_EVENT_CONNECTION_PERMANENTLY_LOST:
            case WF_EVENT_CONNECTION_FAILED:
                g_event = 0xff;             // clear current event
                // if host scan is active, it can be forced inactive by connection/re-connection process
                // so just reset host scan state to inactive.
                host_scan = FALSE;          // host scan inactive
                SYS_CONSOLE_MESSAGE("Reconnecting....\r\n");
                WF_Connect();
                break;
            case WF_EVENT_CONNECTION_SUCCESSFUL:
                g_event = 0xff;             // clear current event
                // if host scan is active, it can be forced inactive by connection/re-connection process
                // so just reset host scan state to inactive.
                host_scan = FALSE;          // host scan inactive
                break;
            case WF_EVENT_SCAN_RESULTS_READY:
                g_event = 0xff;             // clear current event
                host_scan = FALSE;          // host scan inactive
                // Scan results are valid - OK to retrieve
                if (SCANCXT.numScanResults > 0)
                {
                    SCAN_SET_DISPLAY(SCANCXT.scanState);
                    SCANCXT.displayIdx = 0;
                    while (IS_SCAN_STATE_DISPLAY(SCANCXT.scanState))
                         WFDisplayScanMgr();
                }
                break;
           case WF_EVENT_CONNECTION_TEMPORARILY_LOST:
                // This event can happened when CM in module is enabled.
                g_event = 0xff;         // clear current event
                // if host scan is active, it can be forced inactive by connection/re-connection process
                // so just reset host scan state to inactive.
                host_scan = FALSE;      // host scan inactive
                break;
            default:
                //sprintf(st,"skip event = %d\r\n",g_event);
                //SYS_CONSOLE_MESSAGE(st);
                break;
        }

       if (g_DhcpSuccessful)
       {

       /* Send and Receive UDP packets */
        if(UDPIsOpened(socket1))
        {
            // UDP TX every 10 msec
            if(SYS_TICK_Get() - timeudp >= SYS_TICK_TicksPerSecondGet() / 100)
            {
                timeudp = SYS_TICK_Get();
                tx_number++;
                LED0_IO ^= 1;
                sprintf(str,"rem=%12lu",tx_number);
                for(cntstr=16;cntstr<999;cntstr++)
                    str[cntstr]=cntstr;
                str[999]=0;
                // Send tx_number (formatted in a string)
                if(UDPIsTxPutReady(socket1,1000)!=0)
                {
                    UDPPutString(socket1,(BYTE *)str);
                    UDPFlush(socket1);
                    SYS_CONSOLE_MESSAGE(".");
                }
            }

            // UDP RX tx_number of remote board
            if(UDPIsGetReady(socket1)!=0)
            {
                LED1_IO ^= 1;
                UDPGetArray(socket1,(BYTE *)str,1000);
                str[16]=0;
                //sprintf((char*)LCDText,"%sloc=%12lu",str,tx_number); // Write on EXP16 LCD local and remote TX number
                //strcpypgm2ram(LCDText,str);
                //LCDUpdate();
                SYS_CONSOLE_MESSAGE("Rx");

            }
        }

        // Do host scan
         if((SYS_TICK_Get() - t1) >= SYS_TICK_TicksPerSecondGet() * 20)
        {
            t1 = SYS_TICK_Get();
            if (!host_scan)             // allow host scan if currently inactive
            {
                sprintf(st,"%d Scanning ..... event = %d\r\n",++scan_count, g_event);
                SYS_CONSOLE_MESSAGE(st);
                host_scan = TRUE;       // host scan active
                WF_Scan(0xff);          // scan on all channels
            }
        }
       } // DHCP status
       
#endif  //HOST_CM_TEST


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
    SYS_CONSOLE_MESSAGE((char *) message);
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


