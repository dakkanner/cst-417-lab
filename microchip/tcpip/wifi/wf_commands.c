/*******************************************************************************
  MRF24W commands(based on system commander) implementation

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Provides access to MRF24W WiFi controller
    -Reference: MRF24W Data sheet, IEEE 802.11 Standard
*******************************************************************************/

/*******************************************************************************
FileName:   wf_commands.c
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

//==============================================================================
//                                  Includes
//==============================================================================

#include "tcpip_private.h"
#include <ctype.h>
#include "wf_config.h"


#if defined( TCPIP_STACK_COMMANDS_WIFI_ENABLE )
#include "wf_commands.h"
#if defined ( EZ_CONFIG_SCAN )
#include "wf_easy_config.h"
#endif /* EZ_CONFIG_SCAN */

// iwconfig control block
typedef struct
{
    uint8_t powerSaveState;  // power save state
    uint8_t connState;       // connection state
    bool    isIdle;          // true if connState is WF_CSTATE_NOT_CONNECTED
} t_wfIwconfigCb;

//==============================================================================
//                                  Constants
//==============================================================================

//==============================================================================
//                                  Macros
//==============================================================================
// used to simplify the many print statements in this module.  The '##' gets rid
// of the ',' before __VA_ARGS__ when there are no arguments
#define PRINTF(format, ...)    (*pCmdIO->pCmdApi->print)(pCmdIO->cmdIoParam, format, ## __VA_ARGS__)

//==============================================================================
//                                  Globals
//==============================================================================
uint8_t g_hibernate_state = WF_HB_NO_SLEEP;
uint8_t g_wakeup_notice = false;
t_wfIwconfigCb iwconfigCb;
static bool iwconfigCbInitialized = false;


//==============================================================================
//                                  Local Function Prototypes
//==============================================================================
static bool iwconfigUpdateCb(_CMDIO_DEV_NODE* pCmdIO);
static void iwconfigDisplayStatus(_CMDIO_DEV_NODE* pCmdIO);
static bool iwconfigSetSsid(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv);
static bool iwconfigSetMode(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv);
static bool iwconfigSetChannel(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv);
static bool iwconfigSetPower(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv);
static bool iwconfigSetRTS(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv);
static int  CommandIwConfig(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv);
static int  CommandIw_readConfig(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv);
static int  CommandIw_eraseConfig(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv);
static int  CommandIw_saveConfig(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv);

#if defined ( EZ_CONFIG_SCAN )
static void DisplayScanResult(_CMDIO_DEV_NODE* pCmdIO, uint16_t index, t_wfScanResult *p_scanResult);
#endif

// WIFI command table
static const _SYS_CMD_DCPT    wfCmdTbl[]=
{
    {"iwconfig",     CommandIwConfig,         ": Wifi mode config"},
    {"iw_readconf",  CommandIw_readConfig,    ": read file from storage"},
    {"iw_eraseconf", CommandIw_eraseConfig,   ": erase storage"},
    {"iw_saveconf",  CommandIw_saveConfig,    ": save storage"},
};


bool WIFICommandsInit(void)
{
    if (_SYS_COMMAND_ADDGRP(wfCmdTbl, sizeof(wfCmdTbl)/sizeof(*wfCmdTbl), "wifi", ": Wifi commands") == -1) 
    {
        return false;
    }
    
    return true;
}

static int CommandIwConfig(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv)
{
#if defined ( EZ_CONFIG_SCAN )
    uint16_t scanIndex;
    t_wfScanResult scanResult;
#endif

    if (argc > 3)
    {
        PRINTF("Usage: iwconfig <arg1> <arg2>\r\n");
        PRINTF("Ex: iwconfig ssid wfrouter\r\n");
        return false;
    }

    if (!g_hibernate_state && !iwconfigUpdateCb(pCmdIO) )
    {
        PRINTF("iwconfigCb structure set failed\r\n");
        return false;
    }

    // if user only typed in iwconfig with no other parameters
    if (argc == 1)
    {
        if (!g_hibernate_state)
        {
            iwconfigDisplayStatus(pCmdIO);
        }
        else
        {
            PRINTF("The Wi-Fi module is in hibernate mode - command failed.\r\n");
        }
        return false;
    }

    if ( (2 <= argc) && (strcmp((char*)argv[1], "wakeup") == 0) )
    {
        if (!g_wakeup_notice)
        {
            g_wakeup_notice = true;
        }
        PRINTF("The Wi-Fi module is awake.\r\n");
        return true;
    }

    if ( (2 <= argc) && (strcmp((char*)argv[1], "hibernate") == 0) )
    {
        if (!g_hibernate_state)
        {
            g_hibernate_state = WF_HB_ENTER_SLEEP;
            g_wakeup_notice = false;
        }
        PRINTF("Putting Wi-Fi module is in hibernate mode.\r\n");
        return true;
    }

    if (g_hibernate_state)
    {
        PRINTF("The Wi-Fi module is in hibernate mode - command failed.\r\n");
        return false;
    }

    if ( (2 <= argc) && (strcmp((char*)argv[1], "ssid") == 0) )
    {
        if (!g_hibernate_state)
        {
            return iwconfigSetSsid(pCmdIO, argc, argv);
        }
    }
    else if ( (2 <= argc) && (strcmp((char*)argv[1], "mode") == 0) )
    {
        if (!g_hibernate_state)
        {
            return iwconfigSetMode(pCmdIO, argc, argv);
        }
    }
    else if ( (2 <= argc) && (strcmp((char*)argv[1], "channel") == 0) )
    {
        if (!g_hibernate_state)
        {
            return iwconfigSetChannel(pCmdIO, argc, argv);
        }
    }
    else if ( (2 <= argc) && (strcmp((char*)argv[1], "power") == 0) )
    {
        if (!g_hibernate_state)
        {
            return iwconfigSetPower(pCmdIO, argc, argv);
        }
    }
    else if ( (2 <= argc) && (strcmp((char*)argv[1], "rts") == 0) )
    {
        if (!g_hibernate_state)
        {
            return iwconfigSetRTS(pCmdIO, argc, argv);
        }
    }

#if defined ( EZ_CONFIG_SCAN )
    else if ( (2 == argc) && (strcmp((char*)argv[1], "scan") == 0) )
    {
        if (!g_hibernate_state) {
            if (WFStartScan() == WF_SUCCESS)
            {
                PRINTF("Scan started ...\r\n");
                SCAN_SET_DISPLAY(SCANCXT.scanState);
                SCANCXT.displayIdx = 0;
            }
            else
            {
                PRINTF("Scan failed.\r\n");
            }
        }
        return false;
    }
    else if ( (2 <= argc) && (strcmp((char*)argv[1], "scanget") == 0) )
    {
        if (argc != 3)
        {
            PRINTF("Need scan index to display (1 - N)\r\n");
            return false;
        }

        if (g_ScanCtx.numScanResults == 0)
        {
            PRINTF("No scan results to display\r\n");
            return false;
        }

        scanIndex = atoi((char *)argv[2]);
        if ((scanIndex > 0) && (scanIndex <= g_ScanCtx.numScanResults))
        {
            WF_ScanGetResult(scanIndex-1, &scanResult);
            DisplayScanResult(pCmdIO, scanIndex-1, &scanResult);
        }
        else
        {
            PRINTF("Scan index must be between 1 and %d\r\n", g_ScanCtx.numScanResults);
            return false;
        }
    }
#endif /* EZ_CONFIG_SCAN */
    else
    {
        PRINTF("Unknown parameter\r\n");
        return false;
    }

    return true;
}


//==============================================================================
//                            Local Function prototype for iwconfig
//==============================================================================


/*****************************************************************************
 * FUNCTION: iwconfigSetCb
 *
 * RETURNS: true or false
 *
 * PARAMS:  None
 *
 * NOTES:   Updates the iwconfigCb structure
 *****************************************************************************/
static bool iwconfigUpdateCb(_CMDIO_DEV_NODE* pCmdIO)
{
    if ( !iwconfigCbInitialized ) // first time call of iwconfigUpdateCb
    {
        memset(&iwconfigCb, 0, sizeof(iwconfigCb));
        iwconfigCbInitialized = true;
    }

    WF_GetPowerSaveState(&iwconfigCb.powerSaveState);
    if (iwconfigCb.powerSaveState == WF_PS_HIBERNATE)
    {
        PRINTF("WF device hibernated\r\n");
        return false;
    }

    WF_ConnectionStateGet(&iwconfigCb.connState);

    if ((iwconfigCb.connState == WF_CSTATE_NOT_CONNECTED) || (iwconfigCb.connState == WF_CSTATE_CONNECTION_PERMANENTLY_LOST))
    {
        iwconfigCb.isIdle = true;
    }    
    else
    {
        iwconfigCb.isIdle = false;
    }    

    return true;
}

/*****************************************************************************
 * FUNCTION: iwconfigDisplayStatus
 *
 * RETURNS:    None
 *
 * PARAMS:    None
 *
 * NOTES:    Responds to the user invoking iwconfig with no parameters
 *****************************************************************************/
static void iwconfigDisplayStatus(_CMDIO_DEV_NODE* pCmdIO)
{
    uint8_t *p;
    uint8_t tmp;
    uint8_t connectionState;

    union
    {
        struct
        {
            uint8_t List[WF_CHANNEL_LIST_LENGTH];
            uint8_t Num;
        } Channel;

        uint8_t Domain;

        struct
        {
            uint8_t String[WF_MAX_SSID_LENGTH+1];
            uint8_t Len;
        } Ssid;

        struct
        {
            uint8_t NetworkType;
        } Mode;

        struct
        {
            uint16_t Threshold;
        } Rts;
    } ws; // workspace

    // channel
    WF_ChannelListGet(ws.Channel.List, &ws.Channel.Num);
    PRINTF("\tchannel:  ");
    p = ws.Channel.List;
    tmp = ws.Channel.Num;
    while ( --tmp > 0u )
    {
        PRINTF("%d,", *p);
        p++;
    }
    PRINTF("%d\r\n", *p);

    // domain
    WF_RegionalDomainGet(&ws.Domain);
    PRINTF("\tdomain:   ");
    if ( ws.Domain == WF_DOMAIN_FCC )
    {
        PRINTF("FCC");
    }
    else if ( ws.Domain == WF_DOMAIN_ETSI )
    {
        PRINTF("ETSI");
    }
    else if ( ws.Domain == WF_DOMAIN_JAPAN )
    {
        PRINTF("JAPAN or OTHER");
    }
    else
    {
        PRINTF("unknown");
    }
    PRINTF("\r\n");


    // rts
    WF_RtsThresholdGet(&ws.Rts.Threshold);
    PRINTF("\trts:      %d\r\n", ws.Rts.Threshold);

    // mode
    WF_ConnectionStateGet(&connectionState);
    WF_NetworkTypeGet(&ws.Mode.NetworkType);
    PRINTF("\tmode:     ");
    if (iwconfigCb.isIdle)
    {
        if (iwconfigCb.connState == WF_CSTATE_NOT_CONNECTED)
        {
            PRINTF("Idle");
        }
        else if (iwconfigCb.connState == WF_CSTATE_CONNECTION_PERMANENTLY_LOST)
        {
            PRINTF("Idle (connection permanently lost)");
        }
        else
        {
            PRINTF("idle (?)");
        } 
    }
    else
    {
        WF_NetworkTypeGet(&ws.Mode.NetworkType);
        if (ws.Mode.NetworkType == WF_NETWORK_TYPE_INFRASTRUCTURE)
        {
            if (iwconfigCb.connState == WF_CSTATE_CONNECTION_IN_PROGRESS)
            {
                PRINTF("Managed (connection in progress)");
            }
            else if (iwconfigCb.connState == WF_CSTATE_CONNECTED_INFRASTRUCTURE)
            {
                PRINTF("Managed (connected)");
            }
            else if (iwconfigCb.connState == WF_CSTATE_RECONNECTION_IN_PROGRESS)
            {
                PRINTF("Managed (reconnection in progress)");
            }
            else 
            {
                PRINTF("managed (?)");
            }    
        }
        else if (ws.Mode.NetworkType == WF_NETWORK_TYPE_ADHOC)
        {
            if (iwconfigCb.connState == WF_CSTATE_CONNECTION_IN_PROGRESS)
            {
                PRINTF("AdHoc (connection in progress)");
            }
            else if (iwconfigCb.connState == WF_CSTATE_CONNECTED_ADHOC)
            {
                PRINTF("AdHoc (connected)");
            }
            else if (iwconfigCb.connState == WF_CSTATE_RECONNECTION_IN_PROGRESS)
            {
                PRINTF("AdHoc (reconnection in progress)");
            }
            else 
            {
                PRINTF("AdHoc (?)");
            }    
    
            PRINTF("AdHoc");
        }
        else
        {
            PRINTF("Unknown");
        }
    }
    PRINTF("\r\n");

    // ssid
    WF_SsidGet(ws.Ssid.String, &ws.Ssid.Len);
    ws.Ssid.String[ws.Ssid.Len] = '\0';

    PRINTF("\tssid:     ");
    PRINTF((char *)ws.Ssid.String);
    PRINTF("\r\n");

    // power
    switch (iwconfigCb.powerSaveState)
    {
        case WF_PS_PS_POLL_DTIM_ENABLED:
            PRINTF("\tpwrsave:  enabled\r\n");
            PRINTF("\tdtim rx:  enabled");
            break;
        case WF_PS_PS_POLL_DTIM_DISABLED:
            PRINTF("\tpwrsave:  enabled\r\n");
            PRINTF("\tdtim rx:  disabled\r\n");
            break;
        case WF_PS_OFF:
            PRINTF("\tpwrsave:  disabled\r\n");
            break;
        default:
            PRINTF("\tpwrsave:  unknown %d\r\n", iwconfigCb.powerSaveState);
            break;
    }
}

static bool iwconfigSetSsid(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv)
{
    if (argc < 3)
    {
        PRINTF("Missing value for last parameter\r\n");
        return false;
    }

    if (argc > 3)
    {
        PRINTF("SSID may not contain space for this demo\r\n");
        return false;
    }

    WF_SsidSet((uint8_t *)argv[2], strlen((char*)argv[2]));

    return true;
}

static bool iwconfigSetMode(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv)
{
    uint8_t networkType;

    WF_NetworkTypeGet(&networkType);

    if ( (3 <= argc) && (strcmp((char*)argv[2], "idle") == 0) )
    {
        if ( iwconfigCb.isIdle )
        {
            PRINTF("Already in the idle mode\r\n");
        }
        else
        {
            WF_Disconnect();
        }
    }
    else if ( (3 <= argc) && (strcmp((char*)argv[2], "managed") == 0) )
    {
        if ( iwconfigCb.isIdle )
        {
            WF_NetworkTypeSet(WF_NETWORK_TYPE_INFRASTRUCTURE);
            WF_Connect();
        }
        else
        {
            WF_NetworkTypeGet(&networkType);
            if (networkType == WF_NETWORK_TYPE_INFRASTRUCTURE)
            {
                PRINTF("Already in the managed mode\r\n");
            }
            else
            {
                WF_Disconnect();
                WF_NetworkTypeSet(WF_NETWORK_TYPE_INFRASTRUCTURE);
                WF_Connect();
            }
        }
    }
    else if ( (3 <= argc) && (strcmp((char*)argv[2], "adhoc") == 0) )
    {
        if ( iwconfigCb.isIdle )
        {
            t_adHocNetworkContext adhocContext;

            WF_NetworkTypeSet(WF_NETWORK_TYPE_ADHOC);
            adhocContext.beaconPeriod = WF_DEFAULT_ADHOC_BEACON_PERIOD;
            adhocContext.hiddenSsid   = WF_DEFAULT_ADHOC_HIDDEN_SSID;
            adhocContext.mode = WF_ADHOC_CONNECT_THEN_START;
            WF_AdhocContextSet(&adhocContext);
            WF_ReconnectModeSet(3,                              // retry N times to start or join AdHoc network
                                WF_DO_NOT_ATTEMPT_TO_RECONNECT, // do not attempt to reconnect on deauth from station
                                40,                             // beacon timeout is 40 beacon periods
                                WF_ATTEMPT_TO_RECONNECT);       // reconnect on beacon timeout
            WF_Connect();
        }
        else
        {
            WF_NetworkTypeGet(&networkType);
            if (networkType == WF_NETWORK_TYPE_ADHOC)
            {
                PRINTF("Already in the adhoc mode\r\n");
            }
            else
            {
                WF_Disconnect();

                WF_NetworkTypeSet(WF_NETWORK_TYPE_ADHOC);
                WF_Connect();
            }
        }
    }
    else
    {
        PRINTF("Unknown parameter\r\n");
        return false;
    }

    return true;
}

static bool iwconfigSetChannel(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv)
{
    char *p1, *p2;
    char *p_channelList;
    uint8_t index = 0;
    uint16_t temp;
    uint8_t i;

    if (argc < 3)
    {
        PRINTF("No channel numbers entered\r\n");
        return false;
    }

    if ( !iwconfigCb.isIdle )
    {
        PRINTF("Channel(s) can only be set in idle mode\r\n");
        return false;
    }

    p_channelList = argv[2];
    p1 = p2 = p_channelList;

    if ( (3 <= argc) && (strcmp((char*)argv[2], "all") == 0) )
    {
        WF_ChannelListSet((uint8_t *)p_channelList, 0); // reset to domain default channel list
        return true;
    }

    do
    {
        if ( (p2 = strchr(p1, (int) ',')) != NULL )
        {
            *p2='\0';
            p2++;
        }

        temp = atoi(p1);
        if( temp == 0)
        {
            PRINTF("Invalid channel\r\n");
            return  false;
        }

        p1 = p2;
        p_channelList[index] = (uint8_t)temp;
        index++;

    } while (  p2 != NULL );

    // Validate channels against current Domain
    #if (WF_DEFAULT_DOMAIN == WF_DOMAIN_FCC) || (WF_DEFAULT_DOMAIN == WF_DOMAIN_IC)
        for (i=0; i<index; i++)
        {
            if (p_channelList[i] < 1 || p_channelList[i] > 11)
            {
                PRINTF("In valid channel for FCC domain\r\n");
                return false;
            }
        }
    #elif (WF_DEFAULT_DOMAIN == WF_DOMAIN_ETSI)
        for (i=0; i<index; i++)
        {
            if (p_channelList[i] < 1 || p_channelList[i] > 13)
            {
                PRINTF("In valid channel for ETSI domain\r\n");
                return false;
            }
        }
    #elif (WF_DEFAULT_DOMAIN == WF_DOMAIN_JAPAN) || (WF_DEFAULT_DOMAIN == WF_DOMAIN_OTHER)
        for (i=0; i<index; i++)
        {
            if (p_channelList[i] < 1 || p_channelList[i] > 14)
            {
                PRINTF("In valid channel for Japan or Other domain\r\n");
                return false;
            }
        }
    #elif (WF_DEFAULT_DOMAIN == WF_DOMAIN_SPAIN)
        for (i=0; i<index; i++)
        {
            if (p_channelList[i] < 10 || p_channelList[i] > 11)
            {
                PRINTF("In valid channel for Spain\r\n");
                return false;
            }
        }
    #elif (WF_DEFAULT_DOMAIN == WF_DOMAIN_FRANCE)
        for (i=0; i<index; i++)
        {
            if (p_channelList[i] < 10 || p_channelList[i] > 13)
            {
                PRINTF("In valid channel for France\r\n");
                return false;
            }
        }
    #elif (WF_DEFAULT_DOMAIN == WF_DOMAIN_JAPAN_A)
        for (i=0; i<index; i++)
        {
            if (p_channelList[i] < 14 || p_channelList[i] > 14)
            {
                PRINTF("In valid channel for Japan_A\r\n");
                return false;
            }
        }
    #elif (WF_DEFAULT_DOMAIN == WF_DOMAIN_JAPAN_B)
        for (i=0; i<index; i++)
        {
            if (p_channelList[i] < 1 || p_channelList[i] > 13)
            {
                PRINTF("In valid channel for Japan_B\r\n");
                return false;
            }
        }
    #endif

    WF_ChannelListSet((uint8_t *)p_channelList, index);

    return true;
}

static bool iwconfigSetPower(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv)
{
    t_psPollContext psPollContext;

    psPollContext.dtimInterval   = WF_DEFAULT_PS_DTIM_INTERVAL;
    psPollContext.listenInterval = WF_DEFAULT_PS_LISTEN_INTERVAL;
    psPollContext.useDtim        = true;

    if (argc < 3)
    {
        PRINTF("Missing value for last parameter\r\n");
        return false;
    }

    if ( (3 <= argc) && (strcmp((char*)argv[2], "reenable") == 0) )
    {    // reenable power saving
        WF_PsPollEnable(&psPollContext);
    }
    else if ( (3 <= argc) && (strcmp((char*)argv[2], "disable") == 0) )
    {    // disable power saving
        WF_PsPollDisable();
    }
    else if ( (3 <= argc) && (strcmp((char*)argv[2], "unicast") == 0) )
    {    // enable power saving but don't poll for DTIM
        psPollContext.useDtim = false;
        WF_PsPollEnable(&psPollContext);
    }
    else if ( (3 <= argc) && (strcmp((char*)argv[2], "all") == 0) )
    {    // enable power saving and poll for DTIM
        WF_PsPollEnable(&psPollContext);
    }
    else
    {
        PRINTF("Unknown parameter\r\n");
        return false;
    }

    return true;
}


static bool iwconfigSetRTS(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv)
{
    uint16_t rtsThreshold;

    if (argc < 3)
    {
        PRINTF("Missing value for last parameter\r\n");
        return false;
    }

    rtsThreshold = atoi(argv[2]);
    if(rtsThreshold > 255)
    {
        return  false;
    }

    WF_RtsThresholdSet(rtsThreshold);

    return true;
}


static int CommandIw_eraseConfig(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv)
{
    PRINTF("--CommandIw_erase--\r\n");
    WF_ConfigDataErase();
    return 0;
}

static int CommandIw_saveConfig(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv)
{
    TCPIP_NET_HANDLE netH;
    netH = TCPIP_STACK_NetHandle("MRF24W");
    PRINTF("--CommandIw_save--\r\n");

    WF_ConfigDataSave();
    return 0;
}
static int CommandIw_readConfig(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv)
{
    PRINTF("--CommandIw_read--\r\n");
    WF_ConfigDataPrint();
    return 1;
}

#if defined ( EZ_CONFIG_SCAN )
static void DisplayScanResult(_CMDIO_DEV_NODE* pCmdIO, uint16_t index, t_wfScanResult *p_scanResult)
{
    int i;
    uint32_t rate;

    PRINTF("\r\n======================\r\n");
    PRINTF("Scan Result  %d\r\n", index+1);

    // ssid
    p_scanResult->ssid[p_scanResult->ssidLen] = '\0'; // ensure string terminator
    PRINTF("SSID:     %s\r\n", p_scanResult->ssid);

    // bssid
    PRINTF("BSSID:    %02x %02x %02x %02x %02x %02x\r\n",
                          p_scanResult->bssid[0], p_scanResult->bssid[1],
                          p_scanResult->bssid[2], p_scanResult->bssid[3],
                          p_scanResult->bssid[4], p_scanResult->bssid[5]);

    // BSS Type
    PRINTF("BSS Type: ");
    if (p_scanResult->bssType == WF_NETWORK_TYPE_INFRASTRUCTURE)
    {
        PRINTF("Infrastructure\r\n");
    }
    else
    {
        PRINTF("AdHoc\r\n");
    }

    // channel
    PRINTF("Channel:  %d\r\n", p_scanResult->channel);

    // beacon period
    PRINTF("Beacon:   %d ms\r\n", p_scanResult->beaconPeriod);

    // basic rates
    PRINTF("Rates:    ");
    for (i = 0; i < p_scanResult->numRates; ++i)
    {
        rate = (p_scanResult->basicRateSet[i] & ~0x80) * 500000;
        if ((rate % 1000000) == 0)
        {
            PRINTF("%d, ", rate / 1000000);
        }
        else
        {
            PRINTF("%d.5, ", rate / 1000000);
        }
    }
    PRINTF("  (mbps)\r\n");

    // security
    PRINTF("Security: ");
    if ((p_scanResult->apConfig & 0x10) > 0)        // if privacy bit set
    {
        if ((p_scanResult->apConfig & 0x80) > 0)    // if WPA2 bit is set
        {
            PRINTF("WPA2\r\n");
        }
        else if ((p_scanResult->apConfig & 0x40) > 0)    // if WPA bit is set
        {
            PRINTF("WPA\r\n");
        }
        else
        {
            PRINTF("WEP\r\n");
        }
    }
    else
    {
        PRINTF("Open\r\n");
    }

    // preamble
    PRINTF("Preamble: ");
    if ((p_scanResult->apConfig & 0x20) > 0)
    {
        PRINTF("Long\r\n");
    }
    else
    {
         PRINTF("Short\r\n");
    }

    // rssi
    PRINTF("RSSI:     %d\r\n", p_scanResult->rssi);
}
#endif


/*****************************************************************************
 * FUNCTION: HexToBin
 *
 * RETURNS: binary value associated with ASCII hex input value
 *
 * PARAMS:  hexChar -- ASCII hex character
 *
 * NOTES:   Converts an input ascii hex character to its binary value.  Function
 *          does not error check; it assumes only hex characters are passed in.
 *****************************************************************************/
uint8_t HexToBin(uint8_t hexChar)
{
    if ((hexChar >= 'a') && (hexChar <= 'f'))
    {
        return (0x0a + (hexChar - 'a'));
    }
    else if ((hexChar >= 'A') && (hexChar <= 'F'))
    {
        return (0x0a + (hexChar - 'A'));
    }
    else //  ((hexChar >= '0') && (hexChar <= '9'))
    {
        return (0x00 + (hexChar - '0'));
    }

}

/*****************************************************************************
 * FUNCTION: ConvertASCIIHexToBinary
 *
 * RETURNS: true if conversion successful, else false
 *
 * PARAMS:  p_ascii   -- ascii string to be converted
 *          p_binary  -- binary value if conversion successful
 *
 * NOTES:   Converts an input ascii hex string to binary value (up to 32-bit value)
 *****************************************************************************/
static bool ConvertASCIIHexToBinary(char *p_ascii, uint16_t *p_binary)
{
    int8_t  i;
    uint32_t multiplier = 1;

    *p_binary = 0;

    // not allowed to have a string of more than 4 nibbles
    if (strlen((char*)p_ascii) > 8u)
    {
        return false;
    }

    // first, ensure all characters are a hex digit
    for (i = (uint8_t)strlen((char *)p_ascii) - 1; i >= 0 ; --i)
    {
        if (!isxdigit(p_ascii[i]))
        {
            return false;
        }
        *p_binary += multiplier * HexToBin(p_ascii[i]);
        multiplier *= 16;
    }

    return true;
}


bool convertAsciiToHexInPlace(char *p_string, uint8_t expectedHexBinSize )
{

    char  ascii_buffer[3];
    uint8_t  hex_binary_index = 0;
    char  *hex_string_start = p_string;
    uint16_t hex_buffer = 0;

    /* gobble up any hex prefix */
    if ( memcmp(hex_string_start, (const char*) "0x", 2) == 0 )
         hex_string_start+=2;

   if ( strlen( (char *) hex_string_start) != (expectedHexBinSize*2) )
      return false;

    while ( hex_binary_index < expectedHexBinSize )
    {

      memcpy ( ascii_buffer, (const char*) hex_string_start, 2 );
      ascii_buffer[2] = '\0';

      /* convert the hex string to a machine hex value */
      if ( !ConvertASCIIHexToBinary( ascii_buffer,&hex_buffer) )
        return false;

      p_string[hex_binary_index++] = (uint8_t) hex_buffer;

      hex_string_start +=2;

    }

    return true;
}

#endif /* TCPIP_STACK_COMMANDS_WIFI_ENABLE */

