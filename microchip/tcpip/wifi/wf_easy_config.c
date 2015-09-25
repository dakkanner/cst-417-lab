/*******************************************************************************
  MRF24W Driver

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Provides access to MRF24W WiFi controller
    -Reference: MRF24W Data sheet, IEEE 802.11 Standard
*******************************************************************************/

/*******************************************************************************
FileName:   wf_easy_config.c
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

#include "tcpip_private.h"

#if defined(TCPIP_IF_MRF24W)



#include "tcpip/wf_api.h"
#include "wf_easy_config.h"
#include "wf_configData.h"
#include "wifi/wf_debug_output.h"
extern void WF_ReconnectModeSet(uint8_t retryCount, uint8_t deauthAction, uint8_t beaconTimeout, uint8_t beaconTimeoutAction);

#if defined ( EZ_CONFIG_SCAN )
t_wfScanCtx  g_ScanCtx;
#endif /* EZ_CONFIG_SCAN */

t_wfEasyConfigCtx g_easyConfigCtx;
#if WF_DEFAULT_NETWORK_TYPE == WF_NETWORK_TYPE_SOFT_AP
t_wfScanResult preScanResult[50];      //WF_PRESCAN  May change struct later for memory optimization
t_wfHibernate WF_hibernate;
#endif

#if defined(TCPIP_STACK_USE_EZ_CONFIG)
/* Easy Config Globals */


/* Easy Config Private Functions */
static int WFEasyConfigProcess(TCPIP_NET_IF* pNetIf);
static void EasyConfigConnect(void);
static void EasyConfigSetSecurity(void);
static void EasyConfigTimerHandler(SYS_TICK curSysTick);

typedef enum
{
    EZ_WAIT_FOR_START = 0,
    EZ_WAIT_FOR_DELAY = 1,

} t_EasyConfigStates;

void WFEasyConfigInit(void)
{
    CFGCXT.ssid[0] = 0;
    CFGCXT.security = WF_SECURITY_OPEN;
    CFGCXT.key[0] = 0;
    CFGCXT.defaultWepKey = WF_WEP_KEY_INVALID;
    CFGCXT.type = WF_NETWORK_TYPE_INFRASTRUCTURE;
    CFGCXT.cfg_state = cfg_stopped;   // delete later
    CFGCXT.state = EZ_WAIT_FOR_START;
    CFGCXT.isWifiNeedToConfigure = false;
}


// called from a system async event handler whenever g_easyConfigCtx.isWifiNeedToConfigure is true
void WiFi_EasyConfigTask(void)
{
    switch (g_easyConfigCtx.state)
    {
        case EZ_WAIT_FOR_START:
            SYS_CONSOLE_MESSAGE("EZ_WAIT_FOR_START\r\n");
            g_easyConfigCtx.isWifiNeedToConfigure = false;
            // first thing to do is delay one second so user can see on the web
            // page that disconnect is about occur in process of connecting to
            // designated AP.  So create and start timer.
            g_easyConfigCtx.timer = SYS_TICK_TimerCreate(EasyConfigTimerHandler);
            SYS_TICK_TimerSetRate(g_easyConfigCtx.timer, SYS_TICK_TicksPerSecondGet() * WF_EASY_CONFIG_DELAY_TIME);
            g_easyConfigCtx.state = EZ_WAIT_FOR_DELAY;
            break;

        case EZ_WAIT_FOR_DELAY:
            SYS_CONSOLE_MESSAGE("EZ_WAIT_FOR_DELAY\r\n");
            g_easyConfigCtx.isWifiNeedToConfigure = false;
            
            // delete timer now that delay has occurred
            SYS_TICK_TimerDelete(g_easyConfigCtx.timer);

            // connect to AP that user selected on web page
            EasyConfigConnect();
            g_easyConfigCtx.state = EZ_WAIT_FOR_START;

            break;
        }
}

static void EasyConfigTimerHandler(SYS_TICK curSysTick)
{
    curSysTick = curSysTick;  // avoid warning

    // This function being called from timer interrupt, so don't do any work
    // here, but, schedule the async event handler to call EasyConfigStateMachine
    g_easyConfigCtx.isWifiNeedToConfigure = true;
}


static void EasyConfigConnect(void)
{
    WF_Disconnect();    // break the connection supporting the web page

    if (g_easyConfigCtx.ssid)    // if AP has an SSID (not a hidden SSID)
    {
        WF_SsidSet(g_easyConfigCtx.ssid, strlen((char*)g_easyConfigCtx.ssid));
    }

    // Set security supported by designated AP
    EasyConfigSetSecurity();

    #if defined (EZ_CONFIG_STORE) 
        #if 0
            TCPIP_STORAGE_HANDLE hS;
            hS = TCPIPStorageOpen(0, 1);
            TCPIPStorageSaveIfConfig(hS, "MRF24W", true);
            TCPIPStorageClose(hS);
        #else
            WF_ConfigDataSave();
        #endif
    #endif // defined (EZ_CONFIG_STORE)

                /* Set wlan mode */
    WF_NetworkTypeSet(CFGCXT.type);

    #if defined(DISABLE_MODULE_FW_CONNECT_MANAGER_IN_INFRASTRUCTURE)
        WF_ReconnectModeSet(0,                                  // report-only when connection lost (no reconnect)
                            WF_DO_NOT_ATTEMPT_TO_RECONNECT,     // report-only when deauth received (no reconnect)
                            40,                                 // set beacon timeout to 40 beacon periods
                            WF_DO_NOT_ATTEMPT_TO_RECONNECT);    // report only when beacon timeout occurs
    #else
        //TCPIP_NET_IF* p_config= (TCPIP_NET_IF*)GetNetworkConfig();
        if (p_wifi_ConfigData->networkType == WF_NETWORK_TYPE_INFRASTRUCTURE)
        {
            WF_ReconnectModeSet(WF_RETRY_FOREVER,           // retry forever to connect to WiFi network
                                WF_ATTEMPT_TO_RECONNECT,    // reconnect on deauth from AP
                                40,                         // beacon timeout is 40 beacon periods
                                WF_ATTEMPT_TO_RECONNECT);   // reconnect on beacon timeout
        }
    #endif

#if WF_DEFAULT_NETWORK_TYPE == WF_NETWORK_TYPE_SOFT_AP
    // SoftAP: To allow redirection, need to hibernate before changing network type. Module
    //         FW has SoftAP flag and therefore hibernate mode is needed to clear this
    //         indication and allow proper network change. This should work for non-SoftAP,
    //         but these have not been tested yet.
    #if 0
        WF_hibernate.state = WF_HB_ENTER_SLEEP;
        WF_hibernate.wakeup_notice = false;
        //WFConsolePrintRomStr("SoftAP redirection: Put Wi-Fi module into hibernate mode.", true);

        DelayMs(200);

        WF_hibernate.wakeup_notice = true;
        //WFConsolePrintRomStr("Wakeup Wi-Fi module.", true);
    #else
        extern bool SoftAP_Redirection_Enable;
        SoftAP_Redirection_Enable = true;
    #endif
#else
    /* Kick off connection now... */
    WF_Connect();
#endif

}


static void EasyConfigSetSecurity(void)
{
    t_securityContext securityContext;

    switch ((uint8_t)CFGCXT.security)
    {
        case WF_SECURITY_OPEN: /* No security */
            WF_SecurityOpenSet();
            break;

        case WF_SECURITY_WPA_AUTO_WITH_PASS_PHRASE:
            if (CFGCXT.key)
            {
                securityContext.wpaContext.wpaSecurityType = WF_SECURITY_WPA_AUTO_WITH_PASS_PHRASE;
                securityContext.wpaContext.keyInfo.keyLength = strlen((char *)CFGCXT.key);
                memcpy(securityContext.wpaContext.keyInfo.key, CFGCXT.key, securityContext.wpaContext.keyInfo.keyLength);
                WF_SecurityWpaSet(&securityContext.wpaContext);
            }
            break;

        case WF_SECURITY_WPA_AUTO_WITH_KEY:
            if (CFGCXT.key)
            {
                securityContext.wpaContext.wpaSecurityType = WF_SECURITY_WPA_AUTO_WITH_KEY;
                securityContext.wpaContext.keyInfo.keyLength = 32;
                memcpy(securityContext.wpaContext.keyInfo.key, CFGCXT.key, 32);
                WF_SecurityWpaSet(&securityContext.wpaContext);
            }
            break;

        case WF_SECURITY_WEP_40:
            if (CFGCXT.key)
            {
                securityContext.wepContext.wepSecurityType = WF_SECURITY_WEP_40;
                securityContext.wepContext.wepKeyLength    = WF_WEP40_KEY_LENGTH;
                memset(CFGCXT.key, 0x00, WF_WEP40_KEY_LENGTH);
                memset(securityContext.wepContext.wepKey, 0x00, WF_WEP40_KEY_LENGTH);
                securityContext.wepContext.wepKeyType      = WF_SECURITY_WEP_OPENKEY;
                WF_SecurityWepSet(&securityContext.wepContext);
            }
            break;

        case WF_SECURITY_WEP_104:
            if (CFGCXT.key)
            {
                securityContext.wepContext.wepSecurityType = WF_SECURITY_WEP_104;
                securityContext.wepContext.wepKeyLength    = WF_WEP104_KEY_LENGTH;
                memset(CFGCXT.key, 0x00, WF_WEP104_KEY_LENGTH);
                memset(securityContext.wepContext.wepKey, 0x00, WF_WEP104_KEY_LENGTH);
                securityContext.wepContext.wepKeyType      = WF_SECURITY_WEP_OPENKEY;
                WF_SecurityWepSet(&securityContext.wepContext);
            }
            break;
    } // end switch
}

bool WiFi_EasyConfigTaskPending(void)
{
    return g_easyConfigCtx.isWifiNeedToConfigure;
}


void WFEasyConfigMgr(TCPIP_NET_IF* pNetIf)
{
    if (CFGCXT.isWifiNeedToConfigure)
    {
        if (WFEasyConfigProcess(pNetIf))
        {
            //Has been configured, clear flag
            CFGCXT.isWifiNeedToConfigure = false;
        }
    }
}

static int WFEasyConfigProcess(TCPIP_NET_IF* pNetIf)
{
#if 0  // should not be needed
    uint8_t ConnectionState;
#endif
    t_securityContext securityContext;
    
    #if defined (EZ_CONFIG_STALL)
        if (CFGCXT.cfg_state == cfg_stopped)
        {
            /* State machine just started get current time stamp */
            CFGCXT.cfg_state = cfg_stalled;
            CFGCXT.timeStart = SYS_TICK_Get();
            return false;
        }

        /* Wait for stall time to expire */
        if (CFGCXT.cfg_state == cfg_stalled)
        {
            SYS_TICK time = SYS_TICK_Get();
            if ((time - CFGCXT.timeStart) < (WF_EASY_CONFIG_DELAY_TIME * SYS_TICK_TicksPerSecondGet()))
            {
                return false;
            }
        }
    #endif //EZ_CONFIG_STALL

#if 0  // should not be needed
    /* We will re-use the current profile */
    WF_ConnectionStateGet(&ConnectionState);
#endif

    /* Need to disconnect */
    WF_Disconnect();

    /* Set SSID... */
    if (CFGCXT.ssid)
    {
        WF_SsidSet(CFGCXT.ssid, strlen((char*)CFGCXT.ssid));
    }

    /* Now deal with security... */
    switch ((uint8_t)CFGCXT.security)
    {
        case WF_SECURITY_OPEN: /* No security */
            WF_SecurityOpenSet();
            break; 

        case WF_SECURITY_WPA_AUTO_WITH_PASS_PHRASE:
            if (CFGCXT.key)
            {
                securityContext.wpaContext.wpaSecurityType = WF_SECURITY_WPA_AUTO_WITH_PASS_PHRASE;
                securityContext.wpaContext.keyInfo.keyLength = strlen((char *)CFGCXT.key);
                memcpy(securityContext.wpaContext.keyInfo.key, CFGCXT.key, securityContext.wpaContext.keyInfo.keyLength);
                WF_SecurityWpaSet(&securityContext.wpaContext);
            }
            break;

        case WF_SECURITY_WPA_AUTO_WITH_KEY:
            if (CFGCXT.key) 
            {
                securityContext.wpaContext.wpaSecurityType = WF_SECURITY_WPA_AUTO_WITH_KEY;
                securityContext.wpaContext.keyInfo.keyLength = 32;
                memcpy(securityContext.wpaContext.keyInfo.key, CFGCXT.key, 32);
                WF_SecurityWpaSet(&securityContext.wpaContext);
            }
            break;

        case WF_SECURITY_WEP_40:
            if (CFGCXT.key)
            {
                securityContext.wepContext.wepSecurityType = WF_SECURITY_WEP_40;
                securityContext.wepContext.wepKeyLength    = WF_WEP40_KEY_LENGTH;
                memset(CFGCXT.key, 0x00, WF_WEP40_KEY_LENGTH);
                memset(securityContext.wepContext.wepKey, 0x00, WF_WEP40_KEY_LENGTH);
                securityContext.wepContext.wepKeyType      = WF_SECURITY_WEP_OPENKEY;
                WF_SecurityWepSet(&securityContext.wepContext);
            }
            break;

        case WF_SECURITY_WEP_104:
            if (CFGCXT.key)
            {
                securityContext.wepContext.wepSecurityType = WF_SECURITY_WEP_104;
                securityContext.wepContext.wepKeyLength    = WF_WEP104_KEY_LENGTH;
                memset(CFGCXT.key, 0x00, WF_WEP104_KEY_LENGTH);
                memset(securityContext.wepContext.wepKey, 0x00, WF_WEP104_KEY_LENGTH);
                securityContext.wepContext.wepKeyType      = WF_SECURITY_WEP_OPENKEY;
                WF_SecurityWepSet(&securityContext.wepContext);
            }
            break;
    }
 
    #if defined (EZ_CONFIG_STORE) && defined(TCPIP_STACK_USE_STORAGE)
        #if 0
            TCPIP_STORAGE_HANDLE hS;
            hS = TCPIPStorageOpen(0, 1);
            TCPIPStorageSaveIfConfig(hS, "MRF24W", true);
            TCPIPStorageClose(hS);
        #else
            WF_ConfigDataSave();
        #endif
    #endif // defined (EZ_CONFIG_STORE)

    /* Set wlan mode */
    WF_NetworkTypeSet(CFGCXT.type);

#if defined(DISABLE_MODULE_FW_CONNECT_MANAGER_IN_INFRASTRUCTURE)
    WF_ReconnectModeSet(0,                                  // report-only when connection lost (no reconnect)
                        WF_DO_NOT_ATTEMPT_TO_RECONNECT,     // report-only when deauth received (no reconnect)
                        40,                                 // set beacon timeout to 40 beacon periods
                        WF_DO_NOT_ATTEMPT_TO_RECONNECT);    // report only when beacon timeout occurs
#endif
    //TCPIP_NET_IF* p_config= (TCPIP_NET_IF*)GetNetworkConfig();
    if (p_wifi_ConfigData->networkType == WF_NETWORK_TYPE_INFRASTRUCTURE)
    {
        WF_ReconnectModeSet(WF_RETRY_FOREVER,           // retry forever to connect to WiFi network
                            WF_ATTEMPT_TO_RECONNECT,    // reconnect on deauth from AP
                            40,                         // beacon timeout is 40 beacon periods
                            WF_ATTEMPT_TO_RECONNECT);   // reconnect on beacon timeout
    }

#if WF_DEFAULT_NETWORK_TYPE == WF_NETWORK_TYPE_SOFT_AP
    // SoftAP: To allow redirection, need to hibernate before changing network type. Module
    //         FW has SoftAP flag and therefore hibernate mode is needed to clear this
    //         indication and allow proper network change. This should work for non-SoftAP,
    //         but these have not been tested yet.
    #if 0
        WF_hibernate.state = WF_HB_ENTER_SLEEP;
        WF_hibernate.wakeup_notice = false;
        //WFConsolePrintRomStr("SoftAP redirection: Put Wi-Fi module into hibernate mode.", true);

        DelayMs(200);

        WF_hibernate.wakeup_notice = true;
        //WFConsolePrintRomStr("Wakeup Wi-Fi module.", true);
    #else
        extern bool SoftAP_Redirection_Enable;
        SoftAP_Redirection_Enable = true;
    #endif
#else
    /* Kick off connection now... */
    WF_Connect();
    #if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
        WifiAsyncSetEventPending(ASYNC_DHCP_CONFIG_PENDING); // configure DHCP after init complete
    #endif
#endif
    /* Change state and return true to show we are done! */
    CFGCXT.cfg_state = cfg_stopped;

    return true;
}
#endif /* TCPIP_STACK_USE_EZ_CONFIG */


#if defined ( EZ_CONFIG_SCAN )
void WFInitScan(void)
{
    SCANCXT.scanState = 0;
    SCANCXT.numScanResults = 0;
    SCANCXT.displayIdx = 0;
}

uint16_t WFStartScan(void)
{
    /* If scan already in progress bail out */
    if (IS_SCAN_IN_PROGRESS(SCANCXT.scanState))
    {
        return WF_ERROR_OPERATION_CANCELLED;
    }
    if (WF_Scan(WF_SCAN_ALL) != WF_SUCCESS)
    {
        return WF_ERROR_OPERATION_CANCELLED;
    }
   SCAN_SET_IN_PROGRESS(SCANCXT.scanState);

   return WF_SUCCESS;
}

uint16_t WFRetrieveScanResult(uint8_t Idx, t_wfScanResult *p_ScanResult)
{
    if (Idx >= SCANCXT.numScanResults)
        return WF_ERROR_INVALID_PARAM;

    WF_ScanGetResult(Idx, p_ScanResult);
    
    if(p_ScanResult->ssidLen < 32)
    	p_ScanResult->ssid[p_ScanResult->ssidLen] = 0; /* Terminate */

    return WF_SUCCESS;
}

void WFScanEventHandler(uint16_t scanResults)
{
    /* Cache number APs found in scan */
    SCANCXT.numScanResults = scanResults;

    /* Clear the scan in progress */
    SCAN_CLEAR_IN_PROGRESS(SCANCXT.scanState);
    SCAN_SET_VALID(SCANCXT.scanState);
}
#endif /* EZ_CONFIG_SCAN */

 void WFDisplayScanMgr(void)
{
    t_wfScanResult   bssDesc;
    char st[80];
    int i;
	
    if ((SCANCXT.numScanResults == 0)               ||
        (!IS_SCAN_STATE_DISPLAY(SCANCXT.scanState)) ||
        (IS_SCAN_IN_PROGRESS(SCANCXT.scanState))    ||
        (!IS_SCAN_STATE_VALID(SCANCXT.scanState)))
    {
       return;
    }

    WFRetrieveScanResult(SCANCXT.displayIdx, &bssDesc);

    /* Display SSID */
    sprintf(st,"%2d ",SCANCXT.displayIdx);
    SYS_CONSOLE_MESSAGE(st);
    SYS_CONSOLE_MESSAGE("SSID: ");
    for(i = 0; i < bssDesc.ssidLen; i++) st[i] = bssDesc.ssid[i];
    st[bssDesc.ssidLen]=0;
    SYS_CONSOLE_MESSAGE(st);
    SYS_CONSOLE_MESSAGE("\r\n");


    /* Display BSSID */
    SYS_CONSOLE_MESSAGE("   BSSID: ");
    for(i = 0; i < WF_BSSID_LENGTH; i++)
    {
        if (i < 5)
        {
            sprintf(st, "%02X:", bssDesc.bssid[i]);
            SYS_CONSOLE_MESSAGE(st);
        }
        else
        {
            sprintf(st, "%02X, ", bssDesc.bssid[i]);
            SYS_CONSOLE_MESSAGE(st);
        }
    }
    /* Display network mode*/
    if (bssDesc.bssType == WF_NETWORK_TYPE_INFRASTRUCTURE)
        SYS_CONSOLE_MESSAGE("Network Mode: Infra, ");
    else if (bssDesc.bssType == WF_NETWORK_TYPE_ADHOC)
        SYS_CONSOLE_MESSAGE("Network Mode: Adhoc, ");

    /* Display RSSI  & Channel */
    sprintf(st, "RSSI: %3u, Channel: %2u\r\n", bssDesc.rssi, bssDesc.channel);
    SYS_CONSOLE_MESSAGE(st);

    #if (WF_DEFAULT_NETWORK_TYPE == WF_NETWORK_TYPE_SOFT_AP)
        preScanResult[SCANCXT.displayIdx]= bssDesc;    // WF_PRESCAN
    #endif

    if (++SCANCXT.displayIdx == SCANCXT.numScanResults)  
    {
        SCAN_CLEAR_DISPLAY(SCANCXT.scanState);
        SCANCXT.displayIdx = 0;
        #if defined(CMD_PARSER)
            WFConsoleReleaseConsoleMsg();
        #endif
    }
}

#endif /* TCPIP_IF_MRF24W */
