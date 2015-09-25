/*******************************************************************************
  MRF24W Driver WiFi Console

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Provides access to MRF24W WiFi controller
    -Reference: MRF24W Data sheet, IEEE 802.11 Standard
*******************************************************************************/

/*******************************************************************************
FileName:   wf_debug_output.c
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

//============================================================================
// Includes
//============================================================================
#include "tcpip_private.h"

#if defined(TCPIP_IF_MRF24W)
#include "mrf24w_mac.h"
#include "wf_debug_output.h"
#include "system_config.h"

#if defined(SYS_CONSOLE_ENABLE)
const char *connectionFailureStrings[] = {
                                        "NULL",                                  /* 0 - not used */
                                        "NULL",                                  /* 1 - not used */
                                        "WF_JOIN_FAILURE",                       /* 2            */
                                        "WF_AUTHENTICATION_FAILURE",             /* 3            */
                                        "WF_ASSOCIATION_FAILURE",                /* 4            */
                                        "WF_WEP_HANDSHAKE_FAILURE",              /* 5            */
                                        "WF_PSK_CALCULATION_FAILURE",            /* 6            */
                                        "WF_PSK_HANDSHAKE_FAILURE",              /* 7            */
                                        "WF_ADHOC_JOIN_FAILURE",                 /* 8            */
                                        "WF_SECURITY_MISMATCH_FAILURE",          /* 9            */
                                        "WF_NO_SUITABLE_AP_FOUND_FAILURE",       /* 10           */
                                        "WF_RETRY_FOREVER_NOT_SUPPORTED_FAILURE",/* 11           */
                                        "WF_LINK_LOST",                          /* 12           */
                                        "",                                      /* 13           */
                                        "WF_RSN_MIXED_MODE_NOT_SUPPORTED",       /* 14           */
                                        "WF_RECV_DEAUTH",                        /* 15           */
                                        "WF_RECV_DISASSOC",                      /* 16           */
                                        "WF_WPS_FAILURE",                        /* 17           */
                                        "WF_P2P_FAILURE",                        /* 18           */
                                        "WF_LINK_DOWN"                           /* 19           */
                                       };           
                                       
const char *connectionLostStrings[] = {
                                        "Association Failure",                  /* 0 */
                                        "WF_BEACON_TIMEOUT",                    /* 1 */
                                        "WF_DEAUTH_RECEIVED",                   /* 2 */
                                        "WF_DISASSOCIATE_RECEIVED",             /* 3 */
                                        "WF_TKIP_MIC_FAILURE",                  /* 4 */
                                        "WF_LINK_DOWN"                          /* 5 */
                                    };  
                                                                  
const char *deauthDisssocReasonCodes[] = 
                                {
                                        "",                                     /* 0 */
                                        "WF_UNSPECIFIED",                       /* 1 */
                                        "WF_PREV_AUTH_NOT_VALID",               /* 2 */
                                        "WF_DEAUTH_LEAVING",                    /* 3 */
                                        "WF_DISASSOC_DUE_TO_INACTIVITY",        /* 4 */
                                        "WF_DISASSOC_AP_BUSY",                  /* 5 */
                                        "WF_CLASS2_FRAME_FROM_NONAUTH_STA",     /* 6 */
                                        "WF_CLASS3_FRAME_FROM_NONASSOC_STA",    /* 7 */
                                        "WF_DISASSOC_STA_HAS_LEFT",             /* 8 */
                                        "WF_STA_REQ_ASSOC_WITHOUT_AUTH",        /* 9 */
                                        "",                                     /* 10 */
                                        "",                                     /* 11 */
                                        "",                                     /* 12 */
                                        "WF_INVALID_IE",                        /* 13 */
                                        "WF_MIC_FAILURE",                       /* 14 */
                                        "WF_4WAY_HANDSHAKE_TIMEOUT",            /* 15 */
                                        "WF_GROUP_KEY_HANDSHAKE_TIMEOUT",       /* 16 */
                                        "WF_IE_DIFFERENT",                      /* 17 */
                                        "WF_INVALID_GROUP_CIPHER",              /* 18 */
                                        "WF_INVALID_PAIRWISE_CIPHER",           /* 19 */
                                        "WF_INVALID_AKMP",                      /* 20 */
                                        "WF_UNSUPP_RSN_VERSION",                /* 21 */
                                        "WF_INVALID_RSN_IE_CAP",                /* 22 */
                                        "WF_IEEE8021X_FAILED",                  /* 23 */
                                        "WF_CIPHER_SUITE_REJECTED"              /* 24 */
                                };
    
    const char *statusCodes[] = 
                                    {
                                        "",                                     /* 0 */
                                        "WF_UNSPECIFIED_FAILURE",               /* 1 */
                                        "",                                     /* 2 */
                                        "",                                     /* 3 */
                                        "",                                     /* 4 */
                                        "",                                     /* 5 */
                                        "",                                     /* 6 */
                                        "",                                     /* 7 */
                                        "",                                     /* 8 */
                                        "",                                     /* 9 */
                                        "WF_CAPS_UNSUPPORTED",                  /* 10 */
                                        "WF_REASSOC_NO_ASSOC",                  /* 11 */
                                        "WF_ASSOC_DENIED_UNSPEC",               /* 12 */
                                        "WF_NOT_SUPPORTED_AUTH_ALG",            /* 13 */
                                        "WF_UNKNOWN_AUTH_TRANSACTION",          /* 14 */
                                        "WF_CHALLENGE_FAIL",                    /* 15 */
                                        "WF_AUTH_TIMEOUT",                      /* 16 */
                                        "WF_AP_UNABLE_TO_HANDLE_NEW_STA",       /* 17 */
                                        "WF_ASSOC_DENIED_RATES",                /* 18 */
                                        "WF_ASSOC_DENIED_NOSHORTPREAMBLE",      /* 19 */
                                        "WF_ASSOC_DENIED_NOPBCC",               /* 20 */
                                        "WF_ASSOC_DENIED_NOAGILITY",            /* 21 */
                                        "",                                     /* 22 */
                                        "",                                     /* 23 */
                                        "",                                     /* 24 */
                                        "WF_ASSOC_DENIED_NOSHORTTIME",          /* 25 */
                                        "WF_ASSOC_DENIED_NODSSSOFDM",           /* 26 */
                                        "",                                     /* 27 */
                                        "",                                     /* 28 */
                                        "",                                     /* 29 */
                                        "",                                     /* 30 */
                                        "",                                     /* 31 */
                                        "",                                     /* 32 */
                                        "",                                     /* 33 */
                                        "",                                     /* 34 */
                                        "",                                     /* 35 */
                                        "",                                     /* 36 */
                                        "",                                     /* 37 */
                                        "",                                     /* 38 */
                                        "",                                     /* 39 */
                                        "WF_NOT_VALID_IE",                      /* 40 */
                                        "WF_NOT_VALID_GROUPCIPHER",             /* 41 */
                                        "WF_NOT_VALID_PAIRWISE_CIPHER",         /* 42 */
                                        "WF_NOT_VALID_AKMP",                    /* 43 */
                                        "WF_UNSUPPORTED_RSN_VERSION",           /* 44 */
                                        "WF_INVALID_RSN_IE_CAP",                /* 45 */
                                        "WF_CIPHER_SUITE_REJECTED",             /* 46 */
                                        "WF_TIMEOUT"                            /* 47 */
                                    };
       
    const char *wpsState[] = 
                                    {
                                        "NONE",
                                        "EAPOL_START",
                                        "EAP_REQ_IDENTITY",
                                        "EAP_RSP_IDENTITY",
                                        "EAP_WPS_START",
                                        "EAP_RSP_M1",
                                        "EAP_REQ_M2",
                                        "EAP_RSP_M3",
                                        "EAP_REQ_M4",
                                        "EAP_RSP_M5",
                                        "EAP_REQ_M6",
                                        "EAP_RSP_M7",
                                        "EAP_REQ_M8",
                                        "EAP_RSP_DONE",
                                        "EAP_FAILURE"
                                    };
        
    const char *wpsConfigErr[] =      {
                                        "NOERR",
                                        "SESSION_OVERLAPPED",
                                        "DECRYPT_CRC_FAILURE",
                                        "24G_NOT_SUPPORTED",
                                        "RETRY_FAILURE",
                                        "INVALID_MSG",
                                        "AUTH_FAILURE",
                                        "ASSOC_FAILURE",
                                        "MSG_TIMEOUT",
                                        "SESSION_TIMEOUT",
                                        "DEVPASSWD_AUTH_FAILURE",    /* 10 */
                                        "NO_CONN_TOREG",
                                        "MULTI_PBC_DETECTED",
                                        "EAP_FAILURE",
                                        "DEV_BUSY",
                                        "SETUP_LOCKED"
                                    };

    static const char *P2PErr[] = {
                           "WFD_SUCCESS",
                           "WFD_INFO_CURRENTLY_UNAVAILABLE",
                           "WFD_INCOMPATIBLE_PARAMS",
                           "WFD_LIMIT_REACHED",
                           "WFD_INVALID_PARAMS",
                           "WFD_UNABLE_TO_ACCOMMODATE",
                           "WFD_PREV_PROTOCOL_ERROR",
                           "WFD_NO_COMMON_CHANNELS",
                           "WFD_UNKNOWN_GROUP",
                           "",
                           "WFD_INCOMPATIBLE_PROV_METHOD",    /* 10 */
                           "WFD_REJECTED_BY_USER",
                           "WFD_NO_MEM",
                           "WFD_INVALID_ACTION",
                           "WFD_TX_FAILURE",
                           "WFD_TIME_OUT"
                };
            
    static const char *P2PState[] = {
                           "P2PSTIdle",
                        "P2PSTScan",
                        "P2PSTListen",
                        "P2PSTFind",
                        "P2PSTStartFormation",
                        "P2PSTGONegoReqDone",
                        "P2PSTGOWaitNegoReqDone",
                        "P2PSTWaitFormationDone",
                        "P2PSTInvite",
                        "P2PSTProvision",
                        "P2PSTClient"
                };

#if !defined(WF_EVENT_DRIVEN)
bool g_outputConnectionContext = false;
#endif



#endif /* SYS_CONSOLE_ENABLE */


//============================================================================
// Local Function Prototypes
//============================================================================
#if defined(SYS_CONSOLE_ENABLE)
static void OutputDeviceInfo(void);
static void OutputDomain(void);
static void OutputMacAddress(void);
static void OutputSsid(void);
static void OutputNetworkType(void);
static void OutputScanType(void);
static void OutputChannelList(void);
static void OutputBeaconTimeout(void);
static void OutputSecurityMode(void);
static void OutputPowerSaveMode(void);
static void OutputIpAddress(void);

static void OutputConnectionTempLostMsg(uint16_t eventInfo);
static void OutputConnectionFailedMsg(uint16_t eventInfo);
static void OutputConnectionPermLostMsg(uint16_t eventInfo);

static void OutputTxMode(void);

#endif /* SYS_CONSOLE_ENABLE */

#if defined(SYS_CONSOLE_ENABLE)


#if (WF_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_WPS_PIN)
static BOOL ValidatePinChecksum(unsigned long int pin);
#endif
   

/*****************************************************************************
  Function:
    void OutputDemoHeader(void);

  Summary:
    Outputs WiFi demo header to console.

  Description:
    Outputs the name of the demo, and various information items about how the 
    demo is configured.

  Precondition:
    None

  Parameters:
    None

  Returns:
    None

  Remarks:
    None
***************************************************************************/
void OutputDemoHeader(void)
{
    SYS_CONSOLE_MESSAGE("\r\n");
    #if defined(WF_CONSOLE_DEMO)
        SYS_CONSOLE_MESSAGE("=========================\r\n");
        SYS_CONSOLE_MESSAGE("*** WiFi Console Demo ***\r\n"); 
        SYS_CONSOLE_MESSAGE("=========================\r\n");        
    #elif defined(WIFI_TCPIP_WEB_SERVER_DEMO)
        SYS_CONSOLE_MESSAGE("===================================\r\n");
        SYS_CONSOLE_MESSAGE("*** WiFi TCP/IP Web Server Demo ***\r\n"); 
        SYS_CONSOLE_MESSAGE("===================================\r\n");        
    #elif defined(WF_EASY_CONFIG_DEMO)
        SYS_CONSOLE_MESSAGE("==========================\r\n");    
        SYS_CONSOLE_MESSAGE("*** WiFi EZConfig Demo ***\r\n"); 
        SYS_CONSOLE_MESSAGE("==========================\r\n");            
    #endif

    OutputDeviceInfo();
    OutputDomain();   
    OutputMacAddress();
    OutputSsid();   
    OutputNetworkType();
    OutputScanType();
    OutputChannelList();
    OutputBeaconTimeout();
    OutputSecurityMode();
    OutputTxMode();
    OutputPowerSaveMode();
    OutputIpAddress();
}    


static void OutputDeviceInfo(void)
{
    char buf[20];
    t_wfDeviceInfo deviceInfo;
    
    SYS_CONSOLE_MESSAGE("Device:          ");        
    
    WF_GetDeviceInfo(&deviceInfo);
    if (deviceInfo.romVersion >= 0x30)
    {
        sprintf(buf, "MRF24WG (0x%02X%02X)\r\n", deviceInfo.romVersion, deviceInfo.patchVersion);
        SYS_CONSOLE_MESSAGE(buf);
    }
    else
    {
        sprintf(buf, "MRF24WB (0x%02X%02X)\r\n", deviceInfo.romVersion, deviceInfo.patchVersion);
        SYS_CONSOLE_MESSAGE(buf);
    }        
}    

/*****************************************************************************
  Function:
    static void OutputDomain(void)

  Summary:
    Outputs WiFi domain string.

  Description:
    None

  Precondition:
    None

  Parameters:
    None

  Returns:
    None

  Remarks:
    None
***************************************************************************/
static void OutputDomain(void)
{
    uint8_t regionalDomain;

    SYS_CONSOLE_MESSAGE("Domain:          ");

    WF_RegionalDomainGet(&regionalDomain);

    switch (regionalDomain)
    {
        case WF_DOMAIN_FCC:
            SYS_CONSOLE_MESSAGE("FCC");
            break;
        case WF_DOMAIN_ETSI:
            SYS_CONSOLE_MESSAGE("ETSI");
            break;
        case WF_DOMAIN_JAPAN:
            SYS_CONSOLE_MESSAGE("JAPAN or OTHER");
            break;
        default:
            SYS_CONSOLE_MESSAGE("invalid domain");
    }

    SYS_CONSOLE_MESSAGE("\r\n");
}    

/*****************************************************************************
  Function:
    static void OutputMacAddress(void)

  Summary:
    Outputs WiFi MAC address

  Description:
    None

  Precondition:
    None

  Parameters:
    None

  Returns:
    None

  Remarks:
    None
***************************************************************************/
static void OutputMacAddress(void)
{
    uint8_t mac[6];
    int i;
    char buf[16];
    
    WF_MacAddressGet(mac);
    SYS_CONSOLE_MESSAGE("MAC:             ");
    for (i = 0; i < 6; ++i)
    {
        sprintf(buf, "%02X ", mac[i]);
        SYS_CONSOLE_MESSAGE(buf);
    }    
    SYS_CONSOLE_MESSAGE("\r\n");
}    

/*****************************************************************************
  Function:
    static void OutputSsid(void)

  Summary:
    Outputs Wi-Fi SSID

  Description:
    None

  Precondition:
    None

  Parameters:
    None

  Returns:
    None

  Remarks:
    None
***************************************************************************/
static void OutputSsid(void)
{
    TCPIP_NET_IF* p_config;
    
    p_config = (TCPIP_NET_IF*)GetNetworkConfig();
    
    SYS_CONSOLE_MESSAGE("SSID:            ");
    if (p_wifi_ConfigData->netSSID[0] == '\0')
    {
        SYS_CONSOLE_MESSAGE("(none)");
    }    
    else
    {
        SYS_CONSOLE_MESSAGE((char *)p_wifi_ConfigData->netSSID);
    }    
    SYS_CONSOLE_MESSAGE("\r\n");
}

/*****************************************************************************
  Function:
    static void OutputNetworkType(void)

  Summary:
    Outputs Wi-Fi network type in connection profile

  Description:
    None

  Precondition:
    None

  Parameters:
    None

  Returns:
    None

  Remarks:
    None
***************************************************************************/
static void OutputNetworkType(void)
{
    uint8_t networkType;

    SYS_CONSOLE_MESSAGE("Network Type:    ");

    WF_NetworkTypeGet(&networkType);

    if (networkType == WF_NETWORK_TYPE_ADHOC)
        SYS_CONSOLE_MESSAGE("AdHoc");
    else if (networkType == WF_NETWORK_TYPE_SOFT_AP)
	SYS_CONSOLE_MESSAGE("SoftAP");
    else if (networkType == WF_NETWORK_TYPE_P2P)
        SYS_CONSOLE_MESSAGE("P2P");
    else if (networkType == WF_NETWORK_TYPE_INFRASTRUCTURE)
        SYS_CONSOLE_MESSAGE("Infrastructure");
    else
        SYS_CONSOLE_MESSAGE("invalid");
    SYS_CONSOLE_MESSAGE("\r\n");
}    
/*****************************************************************************
  Function:
    static void OutputScanType(void)

  Summary:
    Outputs Wi-Fi Scan Type

  Description:
    None

  Precondition:
    None

  Parameters:
    None

  Returns:
    None

  Remarks:
    None
***************************************************************************/
static void OutputScanType(void)
{
    t_scanContext scanContext;

    SYS_CONSOLE_MESSAGE("Scan Type:       ");

    WF_ScanContextGet(&scanContext);

    if (scanContext.scanType == WF_PASSIVE_SCAN)
        SYS_CONSOLE_MESSAGE("Passive Scan");
    else if (scanContext.scanType == WF_ACTIVE_SCAN)
        SYS_CONSOLE_MESSAGE("Active Scan");
    else
        SYS_CONSOLE_MESSAGE("invalid");
    SYS_CONSOLE_MESSAGE("\r\n");
}

/*****************************************************************************
  Function:
    static OutputChannelList(void)

  Summary:
    Outputs Wi-Fi channel list

  Description:
    None

  Precondition:
    None

  Parameters:
    None

  Returns:
    None

  Remarks:
    None
***************************************************************************/
static void OutputChannelList(void)
{
    uint8_t channelList[WF_CHANNEL_LIST_LENGTH] = WF_DEFAULT_CHANNEL_LIST;
    uint8_t numChannels;
    char    buf[64];
    int     i;
      
    SYS_CONSOLE_MESSAGE("Channel List:    ");

    if (channelList[0] == '\0')
    {
        // Empty channel list - retrieve the default channels from module
        WF_ChannelListGet(channelList,&numChannels);
    }
    else
    {
        numChannels = sizeof(channelList);
    }

    i = 0;
    while (i < numChannels && channelList[i] != '\0')
    {
        sprintf(buf, "%d", channelList[i]);
        SYS_CONSOLE_MESSAGE(buf);
        i++;
        if (channelList[i] != '\0' && i < numChannels)
            SYS_CONSOLE_MESSAGE(", ");
    }
    SYS_CONSOLE_MESSAGE("\r\n");
}


/*****************************************************************************
  Function:
    static void OutputBeaconTimeout(void)

  Summary:
    Outputs Wi-Fi network connection Beacon timeout count and retry count

  Description:
    None

  Precondition:
    None

  Parameters:
    None

  Returns:
    None

  Remarks:
    None
***************************************************************************/
static void OutputBeaconTimeout(void)
{
    char    buf[16];
    uint8_t beaconTimeout;
    uint8_t retryCount, deauthAction, beaconTimeoutAction;
    


    WF_ReconnectModeGet(&retryCount, &deauthAction, &beaconTimeout, &beaconTimeoutAction);

    SYS_CONSOLE_MESSAGE("Beacon Timeout:  ");
    if (beaconTimeout == 0)
    {
        SYS_CONSOLE_MESSAGE("No Beacon timeout");
    }
    else if (beaconTimeout > 0 && beaconTimeout <= 255)
    {
        sprintf(buf, "%d", WF_DEFAULT_BEACON_TIMEOUT);
        SYS_CONSOLE_MESSAGE(buf);
    }
    else
    {
        SYS_CONSOLE_MESSAGE("invalid");
    }
    SYS_CONSOLE_MESSAGE("\r\n");

    SYS_CONSOLE_MESSAGE("Retry Count:     ");
    if (retryCount == WF_RETRY_FOREVER)
        SYS_CONSOLE_MESSAGE("255 (Retry Forever)");
    else if (retryCount > 0 && retryCount < WF_RETRY_FOREVER )
    {
        sprintf(buf, "%d", retryCount);
        SYS_CONSOLE_MESSAGE(buf);
    }
    else
        SYS_CONSOLE_MESSAGE("invalid");
    SYS_CONSOLE_MESSAGE("\r\n");
}

/*****************************************************************************
  Function:
    static void OutputSecurityMode(void)

  Summary:
    Outputs Wi-Fi network connection security mode

  Description:
    None

  Precondition:
    None

  Parameters:
    None

  Returns:
    None

  Remarks:
    None
***************************************************************************/
static void OutputSecurityMode(void)
{
    uint8_t     securityType;
    uint8_t     securityKey[80];
    uint8_t     securityKeyLength;
    uint8_t     keyType;
    uint8_t*    p;
    char        buf[64];
    int         i;

    
    SYS_CONSOLE_MESSAGE("Security:        ");
    
    WF_SecurityGet( &securityType,
                    securityKey,
                    &securityKeyLength);
    p = securityKey;

    switch (securityType)
    {
        case WF_SECURITY_OPEN:
            SYS_CONSOLE_MESSAGE("Open\r\n");
            break;
        case WF_SECURITY_WPA_WITH_KEY:
            SYS_CONSOLE_MESSAGE("WPA PSK with key\r\n");
             break;
        case WF_SECURITY_WPA2_WITH_KEY:
            SYS_CONSOLE_MESSAGE("WPA2 PSK with key\r\n");
            break;
        case WF_SECURITY_WPA_WITH_PASS_PHRASE:
            SYS_CONSOLE_MESSAGE("WPA PSK with pass phrase\r\n");
            break;
        case WF_SECURITY_WPA2_WITH_PASS_PHRASE:
            SYS_CONSOLE_MESSAGE("WPA2 PSK with pass phrase\r\n");
            break;
        case WF_SECURITY_WPA_AUTO_WITH_KEY:
            SYS_CONSOLE_MESSAGE("WPA PSK with key, auto-select\r\n");
            break;
        case WF_SECURITY_WPA_AUTO_WITH_PASS_PHRASE:
            SYS_CONSOLE_MESSAGE("WPA PSK with pass phrase, auto-select\r\n");
            break;
        case WF_SECURITY_WPS_PUSH_BUTTON:
            SYS_CONSOLE_MESSAGE("WPS push button method\r\n");
            break;
        case  WF_SECURITY_WPS_PIN:
            SYS_CONSOLE_MESSAGE("WPS PIN method\r\n");
            break;
        case WF_SECURITY_EAP:
            SYS_CONSOLE_MESSAGE("WPA Enterprise\r\n");
            break;
        case WF_SECURITY_WEP_40:
        case WF_SECURITY_WEP_104:
            if (securityType == WF_SECURITY_WEP_40)
                SYS_CONSOLE_MESSAGE("WEP40, ");
            else if (securityType == WF_SECURITY_WEP_104)
                SYS_CONSOLE_MESSAGE("WEP104, ");
            else
                SYS_CONSOLE_MESSAGE("invalid WEP mode, ");

            WF_WepKeyTypeGet(&keyType);

            if (keyType == WF_SECURITY_WEP_OPENKEY)
                SYS_CONSOLE_MESSAGE("Open Key\r\n");
            else if (keyType == WF_SECURITY_WEP_SHAREDKEY)
                SYS_CONSOLE_MESSAGE("Shared Key\r\n");
            else
                SYS_CONSOLE_MESSAGE("invalid key type\r\n");
 
            break;
        default:
            SYS_CONSOLE_MESSAGE("invalid security setting\r\n");
    }
    SYS_CONSOLE_MESSAGE("Security Key:    ");
    if (securityType == WF_SECURITY_WPS_PIN || securityType == WF_SECURITY_WPS_PUSH_BUTTON)
       SYS_CONSOLE_MESSAGE("Acquire from AP");
    else
    {
        // Security Length is not Null terminated.  So for WEP key we need to limit
        // the length appropriately.
        if (securityType == WF_SECURITY_WEP_40)
            securityKeyLength=5;
        else if (securityType == WF_SECURITY_WEP_104)
            securityKeyLength=13;

        for ( i=0; i < securityKeyLength ; i++ )
        {
            sprintf(buf,"%.2x", *p++);
            SYS_CONSOLE_MESSAGE(buf);
        }
    }
    SYS_CONSOLE_MESSAGE("\r\n");
}

/*****************************************************************************
  Function:
    static void OutputTxMode(void)

  Summary:
    Outputs Wi-Fi network mode

  Description:
    None

  Precondition:
    None

  Parameters:
    None

  Returns:
    None

  Remarks:
    None
***************************************************************************/
static void OutputTxMode(void)
{
    uint8_t txMode;

    SYS_CONSOLE_MESSAGE("Tx Mode:         ");

    WF_GetTxMode(&txMode);

    if (txMode == WF_TXMODE_G_RATES)
    {
        SYS_CONSOLE_MESSAGE("802.11bg mixed");
    }
    else if (txMode == WF_TXMODE_B_RATES)
    {
        SYS_CONSOLE_MESSAGE("802.11b only");
    }
    else if (txMode == WF_TXMODE_LEGACY_RATES)
    {
        SYS_CONSOLE_MESSAGE("802.11b (1 and 2 Mbps only)");
    }
    else
    {
        SYS_CONSOLE_MESSAGE("invalid network mode");
    }
    SYS_CONSOLE_MESSAGE("\r\n");
}

/*****************************************************************************
  Function:
    static void OutputPowerSaveMode(void)

  Summary:
    Outputs Wi-Fi network power save mode

  Description:
    None

  Precondition:
    None

  Parameters:
    None

  Returns:
    None

  Remarks:
    None
***************************************************************************/
static void OutputPowerSaveMode(void)
{
    SYS_CONSOLE_MESSAGE("Power Save:      ");
    #if (WF_DEFAULT_PS_POLL == WF_ENABLED)
    {
        SYS_CONSOLE_MESSAGE("Enabled\r\n");
        SYS_CONSOLE_MESSAGE("PS state:        Active\r\n");
    }
    #elif (WF_DEFAULT_PS_POLL == WF_DISABLED)
        SYS_CONSOLE_MESSAGE("Disabled\r\n");
    #endif
}

/*****************************************************************************
  Function:
    static OutputIpAddress(void)

  Summary:
    Outputs Wi-Fi network IP address

  Description:
    None

  Precondition:
    None

  Parameters: TCPIP_MAC_HANDLE hMac
    None

  Returns:
    None

  Remarks:
    None
***************************************************************************/
static void OutputIpAddress(void)
{
    TCPIP_NET_HANDLE netH;
    TCPIP_NET_IF*    pNetIf;
    
    netH   = TCPIP_STACK_NetHandle("MRF24W");
    pNetIf = _TCPIPStackHandleToNet(netH);

    SYS_CONSOLE_MESSAGE("IP Config:       ");
    if(pNetIf->Flags.bIsDHCPEnabled == 1)
    {
        SYS_CONSOLE_MESSAGE("dynamic\r\n");
    }
    else
    {
        SYS_CONSOLE_MESSAGE("static\n\r");
    }
}


    #if !defined(WF_EVENT_DRIVEN)
    void WF_OutputConnectionContext(void)
    {
        g_outputConnectionContext = true;  // set the flag here; can't invoke mgmt message from event handler
    }

    void _WF_OutputConnectionContext(void)
    {
        t_wfConnectContext context;
        int i;
        char buf[6];

        WF_ConnectContextGet(&context);
        SYS_CONSOLE_MESSAGE("  Connected BSSID  :  ");
        for (i = 0; i < 6; ++i)
        {
            if (i < 5)
            {
                sprintf(buf, "%02X:", context.bssid[i]);
                SYS_CONSOLE_MESSAGE(buf);
            }
            else
            {
                sprintf(buf, "%02X\r\n", context.bssid[i]);
                SYS_CONSOLE_MESSAGE(buf);
            }
        }

        SYS_CONSOLE_MESSAGE("            Channel:  ");
        sprintf(buf, "%d\r\n", context.channel);
        SYS_CONSOLE_MESSAGE(buf);
    }
    #endif // !WF_EVENT_DRIVEN


void WF_OutputConnectionDebugMsg(uint8_t event, uint16_t eventInfo)
{
    if (event == WF_EVENT_CONNECTION_TEMPORARILY_LOST)
    {
        OutputConnectionTempLostMsg(eventInfo);
    }
    else if (event == WF_EVENT_CONNECTION_FAILED)
    {
        OutputConnectionFailedMsg(eventInfo);
    }  
    else if (event == WF_EVENT_CONNECTION_PERMANENTLY_LOST)
    {
        OutputConnectionPermLostMsg(eventInfo);        
    }           
}

static void OutputConnectionTempLostMsg(uint16_t eventInfo)
{
    char buf[8];
    
    if (eventInfo & WF_MASK_DEAUTH_REASONCODE) 
    {
        SYS_CONSOLE_MESSAGE("Event: Connection Temporarily Lost  -- eventInfo = Received deauth : ");
        SYS_CONSOLE_MESSAGE(deauthDisssocReasonCodes[eventInfo & ~WF_MASK_DEAUTH_REASONCODE]);
    } 
    else if (eventInfo & WF_MASK_DISASSOC_REASONCODE) 
    {
        SYS_CONSOLE_MESSAGE("Event: Connection Temporarily Lost  -- eventInfo = Received disassoc : ");
        SYS_CONSOLE_MESSAGE(deauthDisssocReasonCodes[eventInfo & ~WF_MASK_DISASSOC_REASONCODE]);
    } 
    else 
    {
        SYS_CONSOLE_MESSAGE("Event: Connection Temporarily Lost  -- eventInfo = ");
        sprintf(buf, "%d, ", eventInfo);
        SYS_CONSOLE_MESSAGE(buf);
        SYS_CONSOLE_MESSAGE(connectionLostStrings[eventInfo]);
    }

    SYS_CONSOLE_MESSAGE("\r\n");
}

static void OutputConnectionFailedMsg(uint16_t eventInfo)
{   
    uint8_t status;
    uint8_t reason;
    
    status = (uint8_t)(eventInfo >> 8);
    reason = (uint8_t)(eventInfo & 0xff);
    if (status == WF_RECV_DEAUTH || status == WF_RECV_DISASSOC) 
    {
        SYS_CONSOLE_MESSAGE("Event: Connection Failed : ");
        SYS_CONSOLE_MESSAGE(connectionFailureStrings[status]);
        SYS_CONSOLE_MESSAGE(" : ");
        SYS_CONSOLE_MESSAGE(deauthDisssocReasonCodes[reason]);
    
    } 
    else if (status == WF_AUTHENTICATION_FAILURE || status == WF_ASSOCIATION_FAILURE) 
    {
        SYS_CONSOLE_MESSAGE("Event: Connection Failed : ");
        SYS_CONSOLE_MESSAGE(connectionFailureStrings[status]);
        SYS_CONSOLE_MESSAGE(" : ");
        SYS_CONSOLE_MESSAGE(statusCodes[reason]);
    } 
    else if (status == WF_WPS_FAILURE) 
    {
        SYS_CONSOLE_MESSAGE("Event: Connection Failed : ");
        SYS_CONSOLE_MESSAGE(connectionFailureStrings[status]);
        SYS_CONSOLE_MESSAGE(" : ");
        SYS_CONSOLE_MESSAGE(wpsState[reason >> 4]);
        SYS_CONSOLE_MESSAGE(" : ");
        SYS_CONSOLE_MESSAGE(wpsConfigErr[reason & 0x0f]);
    } 
    else if (status == WF_P2P_FAILURE) 
    {
        SYS_CONSOLE_MESSAGE("Event: Connection Failed : ");
        SYS_CONSOLE_MESSAGE(connectionFailureStrings[status]);
        SYS_CONSOLE_MESSAGE(" : ");
        SYS_CONSOLE_MESSAGE(P2PState[reason >> 4]);
        SYS_CONSOLE_MESSAGE(" : ");
        SYS_CONSOLE_MESSAGE(P2PErr[reason & 0x0f]);
    }
    else 
    {
        SYS_CONSOLE_MESSAGE("Event: Connection Failed : ");
        SYS_CONSOLE_MESSAGE(connectionFailureStrings[status]);
    }
    SYS_CONSOLE_MESSAGE("\r\n");
}    
 
static void OutputConnectionPermLostMsg(uint16_t eventInfo)
{
    char buf[8];
    
    if (eventInfo & WF_MASK_DEAUTH_REASONCODE) 
    {
        SYS_CONSOLE_MESSAGE("Event: Connection Permanently Lost  -- eventInfo = Received deauth : ");
        SYS_CONSOLE_MESSAGE(deauthDisssocReasonCodes[eventInfo & ~WF_MASK_DEAUTH_REASONCODE]);
    } 
    else if (eventInfo & WF_MASK_DISASSOC_REASONCODE) 
    {
        SYS_CONSOLE_MESSAGE("Event: Connection Permanently Lost  -- eventInfo = Received disassoc : ");
        SYS_CONSOLE_MESSAGE(deauthDisssocReasonCodes[eventInfo & ~WF_MASK_DISASSOC_REASONCODE]);
    } 
    else 
    {
        SYS_CONSOLE_MESSAGE("Event: Connection Permanently Lost  -- eventInfo = ");
        sprintf(buf, "%d, ", eventInfo);
        SYS_CONSOLE_MESSAGE(buf);
        SYS_CONSOLE_MESSAGE(connectionLostStrings[eventInfo]);
    }

    SYS_CONSOLE_MESSAGE("\r\n");
}    

#endif /* SYS_CONSOLE_ENABLE) */


void ValidateConfig(void)
{
    char buf[64];
  
    #if (   (WF_DEFAULT_NETWORK_TYPE == WF_NETWORK_TYPE_INFRASTRUCTURE) \
                                 &&                        \
            ((WF_DEFAULT_WIFI_SECURITY_MODE != WF_SECURITY_WPS_PIN) && (WF_DEFAULT_WIFI_SECURITY_MODE != WF_SECURITY_WPS_PUSH_BUTTON)))
        strcpy(buf, WF_DEFAULT_SSID_NAME);
        if (strlen(buf) == 0)
        {
            SYS_CONSOLE_MESSAGE("Fatal Error:  Must define an SSID for this configuration\r\n");
            SYS_ASSERT(false, "");            
        }    
    #endif
  
    #if (WF_DEFAULT_NETWORK_TYPE == WF_NETWORK_TYPE_P2P)
        uint8_t channelList[] = WF_DEFAULT_CHANNEL_LIST;
        strcpy(buf, WF_DEFAULT_SSID_NAME);
        if (strcmp(buf, "DIRECT-") != 0)
        {
            SYS_CONSOLE_MESSAGE("Fatal Error:  When in P2P the SSID must be 'DIRECT-'\r\n");
            SYS_ASSERT(false, "");
        } 
        
        if (sizeof(channelList) != 3)
        {
            SYS_CONSOLE_MESSAGE("Fatal Error: For P2P, channel list must be 1, 6, 11\r\n");
            SYS_ASSERT(false, "");
        }    
        
        if ((channelList[0] != 1) || (channelList[1] != 6) || (channelList[2] != 11))
        {
            SYS_CONSOLE_MESSAGE("Fatal Error: For P2P, channel list must be 1, 6, 11\r\n");
            SYS_ASSERT(false, "");
        }
            
    #endif
    
    #if (WF_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_WPS_PIN)
        strcpy(buf, WF_DEFAULT_SSID_NAME);
        if (strlen(buf) == 0)
        {
            SYS_CONSOLE_MESSAGE("Fatal Error: For WF_SECURITY_WPS_PIN, an SSID must be defined\r\n");
            SYS_ASSERT(false, "");
        }    

        #if !defined(WF_DEFAULT_WPS_PIN)
            SYS_CONSOLE_MESSAGE("Fatal Error: If security is WF_SECURITY_WPS_PIN then WF_DEFAULT_WPS_PIN must be defined\r\n");
            SYS_ASSERT(false, "");
        #endif


        {
            strcpy(buf, WF_DEFAULT_WPS_PIN);
            unsigned long int numericPin;
            
            if (strlen(buf) != 8)
            {
                SYS_CONSOLE_MESSAGE("Fatal Error: WF_DEFAULT_WPS_PIN must be exactly 8 digits\r\n");
                SYS_ASSERT(false, "");
            } 

        
            // validate the pin
            sscanf(buf, "%ld", &numericPin);
            if (!ValidatePinChecksum(numericPin))
            {
                SYS_CONSOLE_MESSAGE("Fatal Error: WF_DEFAULT_WPS_PIN has an invalid checksum\r\n");
                SYS_ASSERT(false, "");                    
            }    
        }    
    #endif
    
    #if (WF_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_WPS_PUSH_BUTTON) && (WF_DEFAULT_NETWORK_TYPE != WF_NETWORK_TYPE_P2P)
        strcpy(buf, WF_DEFAULT_SSID_NAME);
        if (strlen(buf) != 0)
        {
            SYS_CONSOLE_MESSAGE("Fatal Error: For WF_SECURITY_WPS_PUSH_BUTTON, WF_DEFAULT_SSID_NAME must be defined as \"\" (empty string)\r\n");
            SYS_ASSERT(false, "");
        }            
    #endif
    
   
    #if (WF_DEFAULT_NETWORK_TYPE != WF_NETWORK_TYPE_P2P) && \
        ((WF_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_WPS_PIN) || (WF_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_WPS_PUSH_BUTTON))
    {
        uint8_t channelList[] = WF_DEFAULT_CHANNEL_LIST;
        uint8_t i;
        
        #if (WF_DEFAULT_DOMAIN == WF_DOMAIN_FCC)
            if ((sizeof(channelList) != 11) && (sizeof(channelList) != 0))
            {
                SYS_CONSOLE_MESSAGE("Fatal Error: If using WPS under FCC must have all 11 channels in list\r\n");
                SYS_ASSERT(false, "");                    
            }    

            if (sizeof(channelList) == 11)
            {
                for (i = 1; i <= 11; ++i)
                {
                    if (channelList[i-1] != i)
                    {
                        SYS_CONSOLE_MESSAGE("Fatal Error: If using WPS under FCC must have channels 1-11 in list\r\n");
                        SYS_ASSERT(false, "");
                    }
                }
            }

        #elif (WF_DEFAULT_DOMAIN == WF_DOMAIN_ETSI)
            if ((sizeof(channelList) != 13) && (sizeof(channelList)!= 0))
            {
                SYS_CONSOLE_MESSAGE("Fatal Error: If using WPS under ETSI domain must have all 13 channels in list\r\n");
                SYS_ASSERT(false, "");                    
            }    

            if (sizeof(channelList) == 13)
            {
                for (i = 1; i <= 13; ++i)
                {
                    if (channelList[i-1] != i)
                    {
                        SYS_CONSOLE_MESSAGE("Fatal Error: If using WPS under ETSI domain must have channels 1-13 in list\r\n");
                        SYS_ASSERT(false, "");
                    }
                }
            }

        #elif (WF_DEFAULT_DOMAIN == WF_DOMAIN_JAPAN)
          if ((sizeof(channelList) != 14) && (sizeof(channelList) != 0))
            {
                SYS_CONSOLE_MESSAGE("Fatal Error: If using WPS under ETSI domain must have all 14 channels in list\r\n");
                SYS_ASSERT(false, "");                    
            }    

            if (sizeof(channelList) == 14)
            {
                for (i = 1; i <= 14; ++i)
                {
                    if (channelList[i-1] != i)
                    {
                        SYS_CONSOLE_MESSAGE("Fatal Error: If using WPS under JAPAN domain must have channels 1-14 in list\r\n");
                        SYS_ASSERT(false, "");
                    }
                }
            }
        #endif
    }
    #endif
    
}

 #if (WF_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_WPS_PIN)
 BOOL ValidatePinChecksum(unsigned long int pin)
 {
    unsigned long int accum = 0;
    accum += 3 * ((pin / 10000000) % 10);
    accum += 1 * ((pin / 1000000) % 10);
    accum += 3 * ((pin / 100000) % 10);
    accum += 1 * ((pin / 10000) % 10);
    accum += 3 * ((pin / 1000) % 10);
    accum += 1 * ((pin / 100) % 10);
    accum += 3 * ((pin / 10) % 10);
    accum += 1 * ((pin / 1) % 10);
    return (0 == (accum % 10));
}
#endif

#endif /* #if defined(TCPIP_IF_MRF24W) */

