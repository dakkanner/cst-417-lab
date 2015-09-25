/*******************************************************************************
  TCP/IP commands implementation

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    TCPIP stack commands entities. 
    Note, this module is based on system command parser
*******************************************************************************/

/*******************************************************************************
FileName:   tcpip_commands.c
Copyright ©2012 released Microchip Technology Inc.  All rights
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

#include "tcpip_private.h"
#include "tcpip/tcpip.h"
#include "tcpip_modules_config.h"
#include "tcpip/wf_api.h"


#if defined(TCPIP_STACK_COMMAND_ENABLE)

#define TCPIP_THIS_MODULE_ID    TCPIP_MODULE_TCPIP_COMMAND
static int  initialNetIfs = 0;    // Backup interfaces number for stack restart
static TCPIP_HEAP_HANDLE       initialHeapH = 0;

#if defined(TCPIP_STACK_COMMANDS_STORAGE_ENABLE)

typedef struct
{
    size_t                  stgSize;        // size  + valid flag
    TCPIP_STACK_NET_IF_DCPT netDcptStg;     // configuration save
    uint8_t                 restoreBuff[sizeof(TCPIP_NETWORK_CONFIG) + 120]; // buffer to restore the configuration
}TCPIP_COMMAND_STG_DCPT;

static TCPIP_COMMAND_STG_DCPT*   pCmdStgDcpt = 0;   // store current interface configuration
static TCPIP_NETWORK_CONFIG*     pCmdNetConf = 0;   // create the array of configurations needed for stack initialization

static bool                     tcpipCmdPreserveSavedInfo = false; // do not discard the saved data

#endif  // defined(TCPIP_STACK_COMMANDS_STORAGE_ENABLE)

static int CommandAddressService(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv, TCPIP_STACK_ADDRESS_SERVICE_TYPE svcType);


static int CommandNetInfo(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv);
static int CommandSetDefaultInterface (_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv);
static int CommandDhcpOnOff(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv);
static int CommandDhcpSOnOff(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv);
static int CommandZcllOnOff(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv);
static int CommandSetIpAddress(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv);
static int CommandSetGatewayAddress(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv);
static int CommandSetPriDNSAddress(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv);
static int CommandSetBIOSName(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv);
static int CommandSetMACAddress(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv);
static int CommandNetworkOnOff(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv);
static int CommandStackOnOff(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv);
static int CommandHeapInfo(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv);

#if defined(TCPIP_STACK_USE_DHCP_SERVER)
static int CommandDhcpLeaseInfo(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv);
#endif  //  defined(TCPIP_STACK_USE_DHCP_SERVER)

// TCPIP stack command table
static const _SYS_CMD_DCPT    tcpipCmdTbl[]=
{
    {"netinfo",     CommandNetInfo,              ": Get network information"},
    {"defnet",      CommandSetDefaultInterface,  ": Set default network interface"},
    {"dhcp",        CommandDhcpOnOff,            ": Turn DHCP client on/off"},
    {"dhcps",       CommandDhcpSOnOff,           ": Turn DHCP server on/off"},
    {"zcll",        CommandZcllOnOff,            ": Turn ZCLL on/off"},
    {"setip",       CommandSetIpAddress,         ": Set IP address and mask"},
    {"setgw",       CommandSetGatewayAddress,    ": Set Gateway address"},
    {"setdns",      CommandSetPriDNSAddress,     ": Set DNS address"},
    {"setbios",     CommandSetBIOSName,          ": Set host's NetBIOS name"},
    {"setmac",      CommandSetMACAddress,        ": Set MAC address"},
    {"if",          CommandNetworkOnOff,         ": Bring an interface up/down"},
    {"stack",       CommandStackOnOff,           ": Stack turn on/off"},
    {"heapinfo",    CommandHeapInfo,             ": Check heap status"},
#if defined(TCPIP_STACK_USE_DHCP_SERVER)
    {"dhcpsinfo",	CommandDhcpLeaseInfo, 	     ": Display DHCP Server Lease Details" },
#endif  //  defined(TCPIP_STACK_USE_DHCP_SERVER)
};

bool TCPIPCommandsInit(const TCPIP_STACK_MODULE_CTRL* const stackCtrl, const TCPIPCMD_MODULE_CONFIG* const pCmdInit)
{
    if(stackCtrl->stackAction == TCPIP_STACK_ACTION_IF_UP)
    {   // interface restart
        return true;
    }


    // stack init
    initialNetIfs = stackCtrl->nIfs;
    initialHeapH = stackCtrl->memH;

    // create command group
    if (!_SYS_COMMAND_ADDGRP(tcpipCmdTbl, sizeof(tcpipCmdTbl)/sizeof(*tcpipCmdTbl), "tcpip", ": stack commands"))
    {
        SYS_ERROR(SYS_ERROR_ERROR, "Failed to create TCPIP Commands\r\n");
        return false;
    }

#if defined(TCPIP_STACK_COMMANDS_STORAGE_ENABLE)
    // get storage for interfaces configuration
    // cannot be taken from the TCPIP-HEAP because we need it persistent after
    // TCPIP_STACK_Deinit() is called!
    if(pCmdStgDcpt == 0 && pCmdNetConf == 0)
    {
        pCmdStgDcpt = (TCPIP_COMMAND_STG_DCPT*)SystemCalloc(initialNetIfs, sizeof(*pCmdStgDcpt));
        pCmdNetConf = (TCPIP_NETWORK_CONFIG*)SystemCalloc(initialNetIfs, sizeof(*pCmdNetConf));
        if(pCmdStgDcpt == 0 || pCmdNetConf == 0)
        {   // failure is not considered to be catastrophic
            SYS_ERROR(SYS_ERROR_WARN, "Failed to create TCPIP Commands Storage/Config\r\n");
        }
    }
#endif  // defined(TCPIP_STACK_COMMANDS_STORAGE_ENABLE)

    #if defined(TCPIP_IF_MRF24W) && defined(TCPIP_STACK_COMMANDS_WIFI_ENABLE)
    WIFICommandsInit();
    #endif

    return true;
}

void TCPIPCommandsDeInit(const TCPIP_STACK_MODULE_CTRL* const stackCtrl)
{
    // if(stackCtrl->stackAction == TCPIP_STACK_ACTION_DEINIT) // stack shut down
    // if(stackCtrl->stackAction == TCPIP_STACK_ACTION_IF_DOWN) // interface down

    if(stackCtrl->stackAction == TCPIP_STACK_ACTION_DEINIT)
    {   // whole stack is going down
#if defined(TCPIP_STACK_COMMANDS_STORAGE_ENABLE)
        if(tcpipCmdPreserveSavedInfo == false)
        {
            SystemFree(pCmdStgDcpt);
            SystemFree(pCmdNetConf);
            pCmdStgDcpt = 0;
            pCmdNetConf = 0;
        }
#endif  // defined(TCPIP_STACK_COMMANDS_STORAGE_ENABLE)

        // Note: no de-initialize of the initialHeapH is done
        // so that we can still check the heap even after the stack shutdown
        // Note: no release of the allocated _SYS_COMMAND_ADDGRP() is done
        // so that stack commands still work
    }
}

static int CommandNetInfo(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv)
{
    int i;
    TCPIP_NET_HANDLE netH;
    IPV4_ADDR ipAddr;
    const MAC_ADDR* pMac;
    TCPIP_NET_IF* pNetIf;
    TCPIP_STACK_MODULE macID;
    const char  *hostName, *msgAdd;
    const void* cmdIoParam = pCmdIO->cmdIoParam;
#if defined(TCPIP_STACK_USE_IPV6)
    IPV6_ADDR_STRUCT currIpv6Add;
    IPV6_ADDR_HANDLE prevHandle, nextHandle;
    char   addrBuff[44];
#else
    char   addrBuff[20];
#endif  // defined(TCPIP_STACK_USE_IPV6)

    if (argc > 2)
    {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Usage: netinfo\r\n");
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Ex: netinfo\r\n");
        return false;
    }

    for (i=0; i<initialNetIfs; i++)
    {
        netH = TCPIP_STACK_IxToNet(i);
        pNetIf = _TCPIPStackHandleToNet(netH);
        macID = _TCPIPStackNetMacId(pNetIf);
        (*pCmdIO->pCmdApi->print)(cmdIoParam, "---------- Interface <%s> ---------- \r\n", _TCPIPStackMACIdToString(macID));
        if(!TCPIP_STACK_IsNetUp(netH))
        {
            (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Interface is down\r\n");
            continue;
        }
        hostName = TCPIP_STACK_NetBIOSName(netH); 
#if defined(TCPIP_STACK_USE_NBNS)
        (*pCmdIO->pCmdApi->print)(cmdIoParam, "Host Name: %s - NBNS enabled\r\n", hostName);
#else
        (*pCmdIO->pCmdApi->print)(cmdIoParam, "Host Name: %s - NBNS disabled \r\n", hostName);
#endif  // defined(TCPIP_STACK_USE_NBNS)
        ipAddr.Val = TCPIP_STACK_NetAddress(netH);
        TCPIP_HELPER_IPAddressToString(&ipAddr, addrBuff, sizeof(addrBuff));
        (*pCmdIO->pCmdApi->print)(cmdIoParam, "IPv4 Address: %s\r\n", addrBuff);

        ipAddr.Val = TCPIP_STACK_NetMask(netH);
        TCPIP_HELPER_IPAddressToString(&ipAddr, addrBuff, sizeof(addrBuff));
        (*pCmdIO->pCmdApi->print)(cmdIoParam, "Mask: %s\r\n", addrBuff);

        ipAddr.Val = TCPIP_STACK_NetGatewayAddress(netH);
        TCPIP_HELPER_IPAddressToString(&ipAddr, addrBuff, sizeof(addrBuff));
        (*pCmdIO->pCmdApi->print)(cmdIoParam, "Gateway: %s\r\n", addrBuff);

        ipAddr.Val = TCPIP_STACK_NetPriDNSAddress(netH);
        TCPIP_HELPER_IPAddressToString(&ipAddr, addrBuff, sizeof(addrBuff));
        (*pCmdIO->pCmdApi->print)(cmdIoParam, "DNS: %s\r\n", addrBuff);

        pMac = (const MAC_ADDR*)TCPIP_STACK_NetMacAddress(netH);
        TCPIP_HELPER_MACAddressToString(pMac, addrBuff, sizeof(addrBuff));
        (*pCmdIO->pCmdApi->print)(cmdIoParam, "MAC Address: %s\r\n", addrBuff);

        // display IPv6 addresses
#if defined(TCPIP_STACK_USE_IPV6)
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "IPv6 Unicast addresses:\r\n");

        prevHandle = 0;
        do
        {
            nextHandle = TCPIP_STACK_NetIPv6AddressGet(netH, IPV6_ADDR_TYPE_UNICAST, &currIpv6Add, prevHandle);
            if(nextHandle)
            {   // have a valid address; display it
                TCPIP_HELPER_IPv6AddressToString(&currIpv6Add.address, addrBuff, sizeof(addrBuff));
                (*pCmdIO->pCmdApi->print)(cmdIoParam, "    %s\r\n", addrBuff);
                prevHandle = nextHandle;
            }
        }while(nextHandle != 0);

        if(prevHandle == 0)
        {   // no valid address
            (*pCmdIO->pCmdApi->msg)(cmdIoParam, "    Unknown\r\n");
        }
        
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "IPv6 Multicast addresses:\r\n");
        prevHandle = 0;
        do
        {
            nextHandle = TCPIP_STACK_NetIPv6AddressGet(netH, IPV6_ADDR_TYPE_MULTICAST, &currIpv6Add, prevHandle);
            if(nextHandle)
            {   // have a valid address; display it
                TCPIP_HELPER_IPv6AddressToString(&currIpv6Add.address, addrBuff, sizeof(addrBuff));
                (*pCmdIO->pCmdApi->print)(cmdIoParam, "    %s\r\n", addrBuff);
                prevHandle = nextHandle;
            }
        }while(nextHandle != 0);

        if(prevHandle == 0)
        {   // no valid address
            (*pCmdIO->pCmdApi->msg)(cmdIoParam, "    Unknown\r\n");
        }

#endif  // defined(TCPIP_STACK_USE_IPV6)

        if(DHCPIsEnabled(netH))
        {
            msgAdd = "dhcp";
        }
        else if(ZCLLIsEnabled(netH))
        {
            msgAdd = "zcll";
        }
        else if(DHCPServerIsEnabled(netH))
        {
            msgAdd = "dhcps";
        }
        else
        {
            msgAdd = "default IP address";
        }

        (*pCmdIO->pCmdApi->print)(cmdIoParam, "%s is ON\r\n", msgAdd);
    }
    return true;
}

#if defined(TCPIP_STACK_USE_DHCP_SERVER)
static int CommandDhcpLeaseInfo(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv)
{
    TCPIP_NET_HANDLE netH;
    DHCPS_LEASE_HANDLE  prevLease, nextLease;
    DHCPS_LEASE_ENTRY leaseEntry;
    char   addrBuff[20];
const void* cmdIoParam = pCmdIO->cmdIoParam;	

	if (argc != 2)
    {
		(*pCmdIO->pCmdApi->msg)(cmdIoParam, "Usage: dhcpsinfo <interface> \r\n");
		(*pCmdIO->pCmdApi->msg)(cmdIoParam, "Ex: dhcpsinfo PIC32INT \r\n");
		return false;
	}

    netH = TCPIP_STACK_NetHandle(argv[1]);
    if (netH == 0)
    {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Unknown interface specified \r\n");
        return false;
    }

	(*pCmdIO->pCmdApi->print)(cmdIoParam,"MAC Address		IPAddress		RemainingLeaseTime \r\n",0);

    prevLease = 0;
    do
    {
        nextLease = DHCPServerLeaseEntryGet(netH, &leaseEntry, prevLease);
		if(!nextLease)
		{
            (*pCmdIO->pCmdApi->print)(cmdIoParam, " \n\r No more entry present \r\n", 0);
		}
        if(nextLease)
        {   // valid info
            // display info
            TCPIP_HELPER_MACAddressToString(&leaseEntry.hwAdd, addrBuff, sizeof(addrBuff));
            (*pCmdIO->pCmdApi->msg)(cmdIoParam, addrBuff);
            TCPIP_HELPER_IPAddressToString(&leaseEntry.ipAddress, addrBuff, sizeof(addrBuff));
            (*pCmdIO->pCmdApi->print)(cmdIoParam, "	%s ", addrBuff);
            (*pCmdIO->pCmdApi->print)(cmdIoParam, "	%d Secs\r\n", leaseEntry.leaseTime/SYS_TICK_TicksPerSecondGet());

            prevLease = nextLease;
        }
    }while(nextLease != 0);


	return true;

}
#endif  //  defined(TCPIP_STACK_USE_DHCP_SERVER)

static int CommandSetDefaultInterface (_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv)
{
    TCPIP_NET_HANDLE netH;
    const void* cmdIoParam = pCmdIO->cmdIoParam;

    if (argc != 2)
    {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Usage: defnet <interface>\r\n");
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Ex: defnet PIC32INT\r\n");
        return false;
    }

    netH = TCPIP_STACK_NetHandle(argv[1]);
    if (netH == 0)
    {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Unknown interface specified \r\n");
        return false;
    }

    if(TCPIP_STACK_SetDefaultNet(netH))
    {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Default interface set successful\r\n");
    }
    else
    {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Operation not accepted\r\n");
    }

    return true;
}

static int CommandDhcpOnOff(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv)
{
    return CommandAddressService(pCmdIO, argc, argv, TCPIP_STACK_ADDRESS_SERVICE_DHCPC);
}

static int CommandDhcpSOnOff(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv)
{
    return CommandAddressService(pCmdIO, argc, argv, TCPIP_STACK_ADDRESS_SERVICE_DHCPS);
}

static int CommandZcllOnOff(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv)
{
    return CommandAddressService(pCmdIO, argc, argv, TCPIP_STACK_ADDRESS_SERVICE_ZCLL);
}

static int CommandAddressService(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv, TCPIP_STACK_ADDRESS_SERVICE_TYPE svcType)
{ 
    typedef bool(*addSvcFnc)(TCPIP_NET_HANDLE hNet);

    TCPIP_NET_HANDLE netH;
    addSvcFnc        addFnc;
    bool             addRes, svcEnable;
    const char       *msgOK, *msgFail;
    const void* cmdIoParam = pCmdIO->cmdIoParam;

    if (argc != 3)
    {
        (*pCmdIO->pCmdApi->print)(cmdIoParam, "Usage: %s <interface> <on/off> \r\n", argv[0]);
        (*pCmdIO->pCmdApi->print)(cmdIoParam, "Ex: %s PIC32INT on \r\n", argv[0]);
        return false;
    }

    netH = TCPIP_STACK_NetHandle(argv[1]);
    if (netH == 0)
    {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Unknown interface\r\n");
        return false;
    }

    if (memcmp(argv[2], "on", 2) == 0)
    {   // turning on a service
        svcEnable = true;
    }
    else if (memcmp(argv[2], "off", 2) == 0)
    {   // turning off a service
        svcEnable = false;
    }
    else
    {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Unknown option\r\n");
        return false;
    }

    switch(svcType)
    {
        case TCPIP_STACK_ADDRESS_SERVICE_DHCPC:
            addFnc = svcEnable?DHCPEnable:DHCPDisable;
            break;

        case TCPIP_STACK_ADDRESS_SERVICE_DHCPS:
            addFnc = svcEnable?DHCPServerEnable:DHCPServerDisable;
            break;

        case TCPIP_STACK_ADDRESS_SERVICE_ZCLL:
            addFnc = svcEnable?ZCLLEnable:ZCLLDisable;
            break;

        default:
            addFnc = 0;     // unknown service; shouldn't happen
            break;
    }

    if(addFnc)
    {
        msgOK   = svcEnable?"enabled":"disabled";
        msgFail = svcEnable?"enable":"disable";

        addRes = (*addFnc)(netH);
        
        if(addRes)
        {
            (*pCmdIO->pCmdApi->print)(cmdIoParam, "%s %s\r\n", argv[0], msgOK);
        }
        else
        {
            (*pCmdIO->pCmdApi->print)(cmdIoParam, "Failed to %s %s\r\n", msgFail, argv[0]);
        }
    }

    return true;
}

static int CommandSetIpAddress(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv)
{
    TCPIP_NET_HANDLE netH;
    IPV4_ADDR ipAddr, ipMask;
    IPV4_ADDR*  pMask;
    const void* cmdIoParam = pCmdIO->cmdIoParam;

    if (argc < 3)
    {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Usage: setip <interface> <x.x.x.x> <x.x.x.x>\r\n");
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Ex: setip PIC32INT 192.168.0.8 255.255.255.0 \r\n");
        return false;
    }

    netH = TCPIP_STACK_NetHandle(argv[1]);
    if (netH == 0)
    {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Unknown interface specified \r\n");
        return false;
    }

    if (!TCPIP_HELPER_StringToIPAddress(argv[2], &ipAddr))
    {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Invalid IP address string \r\n");
        return false;
    }

    if(argc > 3)
    {   // we have net mask too
        if (!TCPIP_HELPER_StringToIPAddress(argv[3], &ipMask))
        {
            (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Invalid IP mask string \r\n");
            return false;
        }
        pMask = &ipMask;
    }
    else
    {
        pMask = 0;
    }

    if(!TCPIP_STACK_SetNetAddress(netH, &ipAddr, pMask, true))
    {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Set ip address failed\r\n");
        return false;
    }

    return true;
}

static int CommandSetGatewayAddress(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv)
{
    TCPIP_NET_HANDLE netH;
    IPV4_ADDR ipGateway;
    const void* cmdIoParam = pCmdIO->cmdIoParam;

    if (argc != 3) {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Usage: setgw <interface> <x.x.x.x> \r\n");
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Ex: setgw PIC32INT 255.255.255.0 \r\n");
        return false;
    }

    netH = TCPIP_STACK_NetHandle(argv[1]);
    if (netH == 0) {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Unknown interface specified \r\n");
        return false;
    }

    if (!TCPIP_HELPER_StringToIPAddress(argv[2], &ipGateway)) {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Invalid IP address string \r\n");
        return false;
    }

    if(!TCPIP_STACK_SetNetGatewayAddress(netH, &ipGateway)) {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Set gateway address failed\r\n");
        return false;
    }

    return true;
}

static int CommandSetPriDNSAddress(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv)
{
    TCPIP_NET_HANDLE netH;
    IPV4_ADDR ipDNS;
    const void* cmdIoParam = pCmdIO->cmdIoParam;

    if (argc != 3) {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Usage: setdns <interface> <x.x.x.x> \r\n");
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Ex: setdns PIC32INT 255.255.255.0 \r\n");
        return false;
    }

    netH = TCPIP_STACK_NetHandle(argv[1]);
    if (netH == 0) {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Unknown interface specified \r\n");
        return false;
    }

    if (!TCPIP_HELPER_StringToIPAddress(argv[2], &ipDNS)) {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Invalid IP address string \r\n");
        return false;
    }

    if(!TCPIP_STACK_SetNetPriDNSAddress(netH, &ipDNS)) {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Set DNS address failed\r\n");
        return false;
    }

    return true;
}

static int CommandSetBIOSName(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv)
{
    TCPIP_NET_HANDLE netH;
    const char* msg;
    const void* cmdIoParam = pCmdIO->cmdIoParam;

    if (argc != 3)
    {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Usage: setbios <interface> <x.x.x.x> \r\n");
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Ex: setbios PIC32INT MCHPBOARD_29 \r\n");
        return false;
    }

    netH = TCPIP_STACK_NetHandle(argv[1]);
    if (netH == 0)
    {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Unknown interface specified \r\n");
        return false;
    }

    if(TCPIP_STACK_SetNetBIOSName(netH, argv[2]))
    {
        msg = "Set BIOS Name OK\r\n";
    }
    else
    {
        msg = "Set BIOS Name failed\r\n";
    }

    (*pCmdIO->pCmdApi->msg)(cmdIoParam, msg);
    return true;
}

static int CommandSetMACAddress(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv)
{
    TCPIP_NET_HANDLE netH;
    MAC_ADDR macAddr;
    const void* cmdIoParam = pCmdIO->cmdIoParam;

    if (argc != 3) {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Usage: setmac <interface> <x:x:x:x:x:x> \r\n");
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Ex: setmac PIC32INT aa:bb:cc:dd:ee:ff \r\n");
        return false;
    }

    netH = TCPIP_STACK_NetHandle(argv[1]);
    if (netH == 0) {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Unknown interface specified \r\n");
        return false;
    }

    (*pCmdIO->pCmdApi->print)(cmdIoParam, "argv[2]: %s\r\n", argv[2]);

    if (!TCPIP_HELPER_StringToMACAddress(argv[2], macAddr.v)) {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Invalid MAC address string \r\n");
        return false;
    }

    if(!TCPIP_STACK_SetNetMacAddress(netH, &macAddr)) {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Set MAC address failed\r\n");
        return false;
    }

    return true;
}


static int CommandNetworkOnOff(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv)
{
    bool res = false;
    TCPIP_NET_HANDLE netH;
#if defined(TCPIP_STACK_COMMANDS_STORAGE_ENABLE)
    TCPIP_COMMAND_STG_DCPT*   pDcpt;
    TCPIP_NETWORK_CONFIG*     pNetConf;
#endif  // defined(TCPIP_STACK_COMMANDS_STORAGE_ENABLE)
    uint16_t net_ix = 0;
    const void* cmdIoParam = pCmdIO->cmdIoParam;

    if (argc != 3)
    {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Usage: if <interface> <down/up> \r\n");
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Ex: if PIC32INT down \r\n");
        return false;
    }

    netH = TCPIP_STACK_NetHandle(argv[1]);

    if (netH == 0)
    {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Unknown interface specified \r\n");
        return false;
    }

    net_ix = TCPIP_STACK_NetIx(netH);

    if (memcmp(argv[2], "up", 2) == 0)
    {
        if(TCPIP_STACK_IsNetUp(netH))
        {
            (*pCmdIO->pCmdApi->msg)(cmdIoParam, "This interface already up\r\n");
            return true;
        }

#if defined(TCPIP_STACK_COMMANDS_STORAGE_ENABLE)
        pNetConf = 0;
        if(pCmdStgDcpt) 
        {
            // get the saved network configuration
            pDcpt = pCmdStgDcpt + net_ix;
            if(pDcpt->stgSize)
            {   // saved config is valid; restore
                pNetConf = TCPIP_STACK_NetConfigSet(&pDcpt->netDcptStg, pDcpt->restoreBuff, sizeof(pDcpt->restoreBuff), 0);
                (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Interface up: configuration restored\r\n");
            }
        }
        res = TCPIP_STACK_NetUp(netH, pNetConf);
#else
        res = TCPIP_STACK_NetUp(netH, 0);
#endif  // defined(TCPIP_STACK_COMMANDS_STORAGE_ENABLE)

    }
    else if (memcmp(argv[2], "down", 4) == 0)
    {
        if(TCPIP_STACK_IsNetUp(netH) == 0)
        {
            (*pCmdIO->pCmdApi->msg)(cmdIoParam, "This interface already down\r\n");
            return true;
        }

#if defined(TCPIP_STACK_COMMANDS_STORAGE_ENABLE)
        if(pCmdStgDcpt) 
        {
            // get the last network configuration so we use it when
            // restart the stack/interface 
            pDcpt = pCmdStgDcpt + net_ix;
            pDcpt->stgSize = TCPIP_STACK_NetConfigGet(netH, &pDcpt->netDcptStg, sizeof(pDcpt->netDcptStg), 0);

            (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Interface down: configuration saved\r\n");
        }
#endif  // defined(TCPIP_STACK_COMMANDS_STORAGE_ENABLE)

        res = TCPIP_STACK_NetDown(netH);
    } 
    else
    {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Wrong parameter specified \r\n");
        return false;
    }

    if (res == true)
    {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Operation successful!\r\n");
    }
    else
    {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Operation failed!\r\n");
    }

    return true;
}

static int CommandStackOnOff(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv)
{
#if defined(TCPIP_STACK_COMMANDS_STORAGE_ENABLE)
    TCPIP_NET_HANDLE netH;
    int              netIx;
    TCPIP_COMMAND_STG_DCPT  *pDcpt;
    TCPIP_NETWORK_CONFIG    *pCurrConf, *pDstConf;
#endif  // defined(TCPIP_STACK_COMMANDS_STORAGE_ENABLE)
    TCPIP_NETWORK_CONFIG    *pStackConf;
    const char              *msg;
    const void* cmdIoParam = pCmdIO->cmdIoParam;

    if (argc < 2)
    {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Usage: stack <up/down> <preserve>\r\n");
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Ex: stack down preserve\r\n");
        return false;
    }


    if (memcmp(argv[1], "up", 2) == 0)
    {
        pStackConf = 0;
#if defined(TCPIP_STACK_COMMANDS_STORAGE_ENABLE)
        if(pCmdStgDcpt != 0 && pCmdNetConf != 0) 
        {
            // get the saved network configuration
            pDcpt = pCmdStgDcpt + 0;
            pDstConf = pCmdNetConf + 0; 
            pCurrConf = 0;
            for (netIx = 0; netIx < initialNetIfs; netIx++)
            {
                if(pDcpt->stgSize)
                {   // saved config is valid; restore
                    pCurrConf = TCPIP_STACK_NetConfigSet(&pDcpt->netDcptStg, pDcpt->restoreBuff, sizeof(pDcpt->restoreBuff), 0);
                }
                else
                {   // don't have a config to restore
                    pCurrConf = 0;
                }

                if(pCurrConf == 0)
                {   // restore failed
                    break;
                }
                else
                {   // save into array for the stack initialization
                    memcpy(pDstConf, pCurrConf, sizeof(*pDstConf));
                }

                pDcpt++;
                pDstConf++;
            }

            if(pCurrConf)
            {   // success
                pStackConf = pCmdNetConf;
                msg = "Stack up: configuration restored\r\n";
            }
            else
            {
                msg = "Stack up: configuration restore failed\r\n";
            }

            (*pCmdIO->pCmdApi->msg)(cmdIoParam, msg);
        }
#endif  // defined(TCPIP_STACK_COMMANDS_STORAGE_ENABLE)

        (*pCmdIO->pCmdApi->print)(cmdIoParam, "Restarting the stack with %d interfaces\r\n", initialNetIfs);
        if (TCPIP_STACK_Init(pStackConf, initialNetIfs, 0, 0) == false)
        {
            msg = "Stack up failed\r\n";
        }
        else
        {
            msg = "Stack up succeeded\r\n";
        }
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, msg);
    }
    else if (memcmp(argv[1], "down", 4) == 0)
    {
#if defined(TCPIP_STACK_COMMANDS_STORAGE_ENABLE)
        tcpipCmdPreserveSavedInfo = false;
        if(argc == 3 && memcmp(argv[2], "preserve", strlen("preserve")) == 0)
        {
            if(pCmdStgDcpt) 
            {
                // get the last network configuration so we use it when
                // restart the stack/interface 
                pDcpt = pCmdStgDcpt + 0;
                for (netIx = 0; netIx < initialNetIfs; netIx++)
                {
                    netH = TCPIP_STACK_IxToNet(netIx);
                    pDcpt->stgSize = TCPIP_STACK_NetConfigGet(netH, &pDcpt->netDcptStg, sizeof(pDcpt->netDcptStg), 0);
                    pDcpt++;
                }

                (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Stack down: configuration saved\r\n");
                tcpipCmdPreserveSavedInfo = true;
            }
        }
#endif  // defined(TCPIP_STACK_COMMANDS_STORAGE_ENABLE)

        TCPIP_STACK_DeInit();
#if defined(TCPIP_STACK_COMMANDS_STORAGE_ENABLE)
        tcpipCmdPreserveSavedInfo = false;          // make sure it doesn't work the next time
#endif  // defined(TCPIP_STACK_COMMANDS_STORAGE_ENABLE)
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Stack down succeeded\r\n");
    }

    return true;
}

static int CommandHeapInfo(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv)
{
    int     ix, nTraces, nEntries;
    HEAP_TRACE_ENTRY    tEntry;
    const void* cmdIoParam = pCmdIO->cmdIoParam;

    if (argc != 1) {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Usage: heapinfo \r\n");
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Ex: heapinfo \r\n");
        return false;
    }

    (*pCmdIO->pCmdApi->print)(cmdIoParam, "Initial created heap size: %d Bytes\r\n", TCPIP_HEAP_Size(initialHeapH));
    (*pCmdIO->pCmdApi->print)(cmdIoParam, "Allocable block heap size: %d Bytes\r\n", TCPIP_HEAP_MaxSize(initialHeapH));
    (*pCmdIO->pCmdApi->print)(cmdIoParam, "All available heap size: %d Bytes\r\n", TCPIP_HEAP_FreeSize(initialHeapH));
    (*pCmdIO->pCmdApi->print)(cmdIoParam, "Last heap error: 0x%x\r\n", TCPIP_HEAP_LastError(initialHeapH));

    nTraces = TCPIP_HEAP_TraceGetEntriesNo(initialHeapH, true);
    if(nTraces)
    {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Trace info: \r\n");
        nEntries = TCPIP_HEAP_TraceGetEntriesNo(initialHeapH, false);
        for(ix = 0; ix < nEntries; ix++)
        {
            if(TCPIP_HEAP_TraceGetEntry(initialHeapH, ix, &tEntry))
            {
                (*pCmdIO->pCmdApi->print)(cmdIoParam, "Module: %4d, totAllocated: %5d, currAllocated: %5d, totFailed: %5d, maxFailed: %5d \r\n", tEntry.moduleId, tEntry.totAllocated, tEntry.currAllocated, tEntry.totFailed, tEntry.maxFailed);
            }
                    
        }
    }
    else
    {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "No Trace info exists.\r\n");
    }

    return true;
}

#endif    // defined(TCPIP_STACK_COMMAND_ENABLE)


