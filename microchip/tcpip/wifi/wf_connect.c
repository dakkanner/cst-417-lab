/*******************************************************************************
  MRF24W Connection Support

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  wf_connect.c 
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

/*
*********************************************************************************************************
*                                           INCLUDES                               
*********************************************************************************************************
*/

#include "tcpip_private.h"

// Compile only for PIC32MX MRF24W MAC interface
#if defined(TCPIP_IF_MRF24W)


#include "tcpip/wf_api.h"
#include "wf_config.h"
#include "tcpip/dhcp.h"
#include "wf_debug_output.h"
#include "wf_easy_config.h"
#include "wf_config.h"
#include "wf_configData.h"

extern void WF_ReconnectModeSet(uint8_t retryCount, uint8_t deauthAction, uint8_t beaconTimeout, uint8_t beaconTimeoutAction);

#if defined(TCPIP_STACK_USE_DHCP_CLIENT)
extern void DhcpEventHandler(TCPIP_NET_HANDLE hNet, DHCP_EVENT_TYPE evType, const void* param);
#endif  // defined(TCPIP_STACK_USE_DHCP_CLIENT)

static void SetWepSecurity(void);
static void SetWpaSecurity(void);
static void SetWpsSecurity(void);
static void SetEapSecurity(void);

#if defined(DERIVE_KEY_FROM_PASSPHRASE_IN_HOST)
extern void WF_ConvPassphrase2Key(uint8_t key_len, uint8_t *key, uint8_t ssid_len, uint8_t *ssid);
#endif

#define GRATUITOUS_ARP_PERIOD 5

/*****************************************************************************
 * FUNCTION: Wifi_Connect
 *
 * RETURNS:  None
 *
 * PARAMS:   None
 *
 *  NOTES:   Connects to an 802.11 network.
 *****************************************************************************/
 void Wifi_Connect(void)
{
    uint8_t channelList[] = WF_DEFAULT_CHANNEL_LIST;
    t_scanContext scanContext;

#if (WF_DEFAULT_NETWORK_TYPE == WF_NETWORK_TYPE_SOFT_AP)
    uint8_t channelList_postscan[] = WF_DEFAULT_CHANNEL_LIST_POSTSCAN;
#endif

    WF_SsidSet(p_wifi_ConfigData->netSSID, p_wifi_ConfigData->SsidLength);
#if (WF_DEFAULT_NETWORK_TYPE == WF_NETWORK_TYPE_ADHOC) || (WF_DEFAULT_NETWORK_TYPE == WF_NETWORK_TYPE_SOFT_AP)
    WF_NetworkTypeSet(p_wifi_ConfigData->networkType);
#else
    WF_NetworkTypeSet(WF_DEFAULT_NETWORK_TYPE);
#endif

    if( p_wifi_ConfigData->networkType == WF_NETWORK_TYPE_SOFT_AP)
    {
        scanContext.scanType = WF_PASSIVE_SCAN;
    }
    else
    {
        scanContext.scanType = WF_ACTIVE_SCAN;
    }
    scanContext.minChannelTime = WF_DEFAULT_SCAN_MIN_CHANNEL_TIME;
    scanContext.maxChannelTime = WF_DEFAULT_SCAN_MAX_CHANNEL_TIME;
    scanContext.probeDelay     = WF_DEFAULT_SCAN_PROBE_DELAY;
    scanContext.scanCount      = WF_DEFAULT_SCAN_COUNT;
    WF_ScanContextSet(&scanContext);

#if (WF_DEFAULT_NETWORK_TYPE == WF_NETWORK_TYPE_SOFT_AP)
    if (((CFGCXT.type != WF_NETWORK_TYPE_SOFT_AP) && (CFGCXT.prevWLAN == WF_NETWORK_TYPE_SOFT_AP)) ||
        (p_wifi_ConfigData->networkType != WF_NETWORK_TYPE_SOFT_AP))
    {
        // putrsUART("\r\n\r\nWF_Connect: Channel list update when transitioning from SoftAP to non-SoftAP or NOT in SoftAP..\r\n\r\n");
        WF_ChannelListSet(channelList_postscan, sizeof(channelList_postscan));
    }
    else
    {
        WF_ChannelListSet(channelList, sizeof(channelList));
    }
#else // not network type not WF_NETWORK_TYPE_SOFT_AP
    WF_ChannelListSet(channelList, sizeof(channelList));
#endif // (WF_DEFAULT_NETWORK_TYPE == WF_NETWORK_TYPE_SOFT_AP)

    // The Retry Count parameter tells the WiFi Connection manager how many attempts to make when trying
    // to connect to an existing network.  In the Infrastructure case, the default is to retry forever so that
    // if the AP is turned off or out of range, the radio will continue to attempt a connection until the
    // AP is eventually back on or in range.  In the Adhoc case, the default is to retry 3 times since the
    // purpose of attempting to establish a network in the Adhoc case is only to verify that one does not
    // initially exist.  If the retry count was set to WF_RETRY_FOREVER in the AdHoc mode, an AdHoc network
    // would never be established.
#if (WF_DEFAULT_NETWORK_TYPE == WF_NETWORK_TYPE_ADHOC) || (WF_DEFAULT_NETWORK_TYPE == WF_NETWORK_TYPE_SOFT_AP)
    if(p_wifi_ConfigData->networkType == WF_NETWORK_TYPE_INFRASTRUCTURE)
    {
        WF_ReconnectModeSet(WF_RETRY_FOREVER,           // retry forever to connect to WiFi network
                            WF_ATTEMPT_TO_RECONNECT,    // reconnect on deauth from AP
                            40,                         // beacon timeout is 40 beacon periods
                            WF_ATTEMPT_TO_RECONNECT);   // reconnect on beacon timeout
    }
    else if((p_wifi_ConfigData->networkType == WF_NETWORK_TYPE_ADHOC) || (p_wifi_ConfigData->networkType == WF_NETWORK_TYPE_SOFT_AP))
    {
        WF_ReconnectModeSet(WF_DEFAULT_LIST_RETRY_COUNT,    // retry N times to start or join AdHoc network
                            WF_DO_NOT_ATTEMPT_TO_RECONNECT, // do not attempt to reconnect on deauth from station
                            40,                             // beacon timeout is 40 beacon periods
                            WF_ATTEMPT_TO_RECONNECT);       // reconnect on beacon timeout
    }
    else
    {
        SYS_CONSOLE_MESSAGE("Please compile with correct XX_RETRY_COUNT\r\n");
        while(1);
    }
#else // network type not WF_NETWORK_TYPE_ADHOC or WF_NETWORK_TYPE_SOFT_AP
    WF_ReconnectModeSet(WF_DEFAULT_LIST_RETRY_COUNT,// retry N times to start or join AdHoc network
                        WF_ATTEMPT_TO_RECONNECT,    // reconnect on deauth from AP
                        40,                         // beacon timeout is 40 beacon periods
                        WF_ATTEMPT_TO_RECONNECT);   // reconnect on beacon timeout

#endif // (WF_DEFAULT_NETWORK_TYPE == WF_NETWORK_TYPE_ADHOC) || (WF_DEFAULT_NETWORK_TYPE == WF_NETWORK_TYPE_SOFT_AP)


    // Set Tx Mode
    WF_SetTxMode(WF_DEFAULT_TX_MODE); // WF_TXMODE_G_RATES, WF_TXMODE_B_RATES, WF_TXMODE_LEGACY_RATES

    // Error check items specific to WPS Push Button mode
    #if (WF_DEFAULT_WIFI_SECURITY_MODE==WF_SECURITY_WPS_PUSH_BUTTON)
        SYS_ASSERT(strlen(p_wifi_ConfigData->netSSID) == 0, "");  // SSID must be empty when using WPS
        // To Do: fix this to work with different Domain and also empty channelList
        // SYS_ASSERT(sizeof(channelList)==11, "");      // must scan all channels for WPS
    #endif    

    #if (WF_DEFAULT_NETWORK_TYPE == WF_NETWORK_TYPE_P2P)
        SYS_ASSERT(strcmp((char *)p_wifi_ConfigData->netSSID, "DIRECT-") == 0, "");
        SYS_ASSERT(sizeof(channelList) == 3, "");
        SYS_ASSERT(channelList[0] == 1, "");
        SYS_ASSERT(channelList[1] == 6, "");
        SYS_ASSERT(channelList[2] == 11, "");
    #endif

    if (p_wifi_ConfigData->SecurityMode == WF_SECURITY_OPEN)
    {
        WF_SecurityOpenSet();
    }
    else if ((p_wifi_ConfigData->SecurityMode == WF_SECURITY_WEP_40) || (p_wifi_ConfigData->SecurityMode == WF_SECURITY_WEP_104))
    {
        SetWepSecurity();
    }
    else if ((p_wifi_ConfigData->SecurityMode == WF_SECURITY_WPA_AUTO_WITH_KEY)         ||
             (p_wifi_ConfigData->SecurityMode == WF_SECURITY_WPA_WITH_KEY)              ||
             (p_wifi_ConfigData->SecurityMode == WF_SECURITY_WPA_AUTO_WITH_PASS_PHRASE) ||
             (p_wifi_ConfigData->SecurityMode == WF_SECURITY_WPA_WITH_PASS_PHRASE)      ||
             (p_wifi_ConfigData->SecurityMode == WF_SECURITY_WPA2_WITH_KEY)             ||
             (p_wifi_ConfigData->SecurityMode == WF_SECURITY_WPA2_WITH_PASS_PHRASE))
    {
        SetWpaSecurity();
    }
    else if ((p_wifi_ConfigData->SecurityMode == WF_SECURITY_WPS_PIN) ||
            (p_wifi_ConfigData->SecurityMode == WF_SECURITY_WPS_PUSH_BUTTON))
    {
        SetWpsSecurity();
    }
    else if (p_wifi_ConfigData->SecurityMode == WF_SECURITY_EAP)
    {
        SetEapSecurity();
    }

#if WF_DEFAULT_PS_POLL == WF_ENABLED
    {
        t_psPollContext psPollContext;
        psPollContext.dtimInterval   = WF_DEFAULT_PS_DTIM_INTERVAL;
        psPollContext.listenInterval = WF_DEFAULT_PS_LISTEN_INTERVAL;
        psPollContext.useDtim        = true;
        WF_PsPollEnable(&psPollContext);
    }
#else    /* WF_DEFAULT_PS_POLL == WF_DISABLED */
    WF_PsPollDisable();
#endif    /* WF_DEFAULT_PS_POLL == WF_ENABLED */

#if defined(TCPIP_STACK_USE_GRATUITOUS_ARP)
    WF_StartGratuitousArp(GRATUITOUS_ARP_PERIOD);  // output a gratuitous arp every 5 seconds (after connection)
#endif

#if defined(SYS_CONSOLE_ENABLE)
     OutputDemoHeader();
#endif

// override reconnect mode if connection manager disabled
#if defined(DISABLE_MODULE_FW_CONNECT_MANAGER_IN_INFRASTRUCTURE)
    WF_ReconnectModeSet(0,                                  // report-only when connection lost (no reconnect)
                        WF_DO_NOT_ATTEMPT_TO_RECONNECT,     // report-only when deauth received (no reconnect)
                        40,                                 // set beacon timeout to 40 beacon periods
                        WF_DO_NOT_ATTEMPT_TO_RECONNECT);    // report only when beacon timeout occurs
#endif

#if 0
    WF_EnableDebugPrint(ENABLE_WPS_PRINTS | ENABLE_P2P_PRINTS);
#endif
    SYS_CONSOLE_MESSAGE("\r\nStart WiFi Connect . . .\r\n");

#if defined(TCPIP_STACK_USE_DHCP_CLIENT) && defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
    DHCPRegisterHandler(TCPIP_STACK_NetHandle("MRF24W"), DhcpEventHandler, NULL);
#endif  // defined(TCPIP_STACK_USE_DHCP_CLIENT)

    // start the WiFi connection process
    WF_Connect();
    #if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
    WifiAsyncSetEventPending(ASYNC_DHCP_CONFIG_PENDING);
    #endif
}

static void SetWepSecurity(void)
{
    t_wepContext wepContext;

    wepContext.wepSecurityType = p_wifi_ConfigData->SecurityMode;
    wepContext.wepKeyLength    = p_wifi_ConfigData->SecurityKeyLength;
    memcpy(wepContext.wepKey, p_wifi_ConfigData->SecurityKey, wepContext.wepKeyLength);
    wepContext.wepKeyType      = WF_DEFAULT_WEP_KEY_TYPE;  // TBD if MRF24WB supported and > 1209 need to modify
    WF_SecurityWepSet(&wepContext);
}

static void SetWpaSecurity(void)
{
    t_wpaContext wpaContext;

    WF_SetWPATimeout(WF_DEFAULT_WPA_TIMEOUT);

#if defined(__C32__) && defined(DERIVE_KEY_FROM_PASSPHRASE_IN_HOST)
    if (p_wifi_ConfigData->SecurityMode == WF_SECURITY_WPA_WITH_PASS_PHRASE          ||
        p_wifi_ConfigData->SecurityMode == WF_SECURITY_WPA2_WITH_PASS_PHRASE         ||
        p_wifi_ConfigData->SecurityMode == WF_SECURITY_WPA_AUTO_WITH_PASS_PHRASE)
    {
        WF_ConvPassphrase2Key(p_wifi_ConfigData->SecurityKeyLength, p_wifi_ConfigData->SecurityKey, p_wifi_ConfigData->SsidLength, p_wifi_ConfigData->netSSID);
        p_wifi_ConfigData->SecurityMode--;
        p_wifi_ConfigData->SecurityKeyLength = 32;
    }
#endif // defined(__C32__) && defined(DERIVE_KEY_FROM_PASSPHRASE_IN_HOST)

    wpaContext.wpaSecurityType   = p_wifi_ConfigData->SecurityMode;
    wpaContext.keyInfo.keyLength = p_wifi_ConfigData->SecurityKeyLength;
    memcpy(wpaContext.keyInfo.key, p_wifi_ConfigData->SecurityKey, wpaContext.keyInfo.keyLength);
    WF_SecurityWpaSet(&wpaContext);
}

static void SetWpsSecurity(void)
{
    t_wpsContext wpsContext;

    wpsContext.wpsSecurityType = p_wifi_ConfigData->SecurityMode;

    if (wpsContext.wpsSecurityType == WF_SECURITY_WPS_PUSH_BUTTON)
    {
        memset(wpsContext.wpsPin, 0x00, WF_WPS_PIN_LENGTH);
        wpsContext.wpsPinLength = 0;
    }
    else
    {
        memcpy(wpsContext.wpsPin, (const void*)WF_DEFAULT_WPS_PIN, WF_WPS_PIN_LENGTH);
        wpsContext.wpsPinLength = WF_WPS_PIN_LENGTH;
    }

    //WF_YieldPassphrase2Host();
    WF_SecurityWpsSet(&wpsContext);
}

static void SetEapSecurity(void)
{
    // TBD
}


#endif  // defined(TCPIP_IF_MRF24W)


