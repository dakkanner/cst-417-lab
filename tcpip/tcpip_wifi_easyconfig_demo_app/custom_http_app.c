/*******************************************************************************
  Application to Demo HTTP2 Server

  Summary:
    Support for HTTP2 module in Microchip TCP/IP Stack
    
  Description:
    -Implements the application 
    -Reference: RFC 1002
*******************************************************************************/

/*******************************************************************************
FileName:   CustomHTTPApp.c
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

#define __CUSTOMHTTPAPP_C
#include <ctype.h>
#include "tcpip_config.h"

#if defined(TCPIP_STACK_USE_HTTP2_SERVER)

#include "tcpip/tcpip.h"
#include "../../microchip/tcpip/wifi/wf_easy_config.h"
#include "http2_config.h"

#include "main_demo.h"
#include "../tcpip/wifi/wf_configData.h"

#include <common/hashes.h>
extern t_wfScanCtx  g_ScanCtx;

uint8_t HexToBin(uint8_t hexChar);

/****************************************************************************
  Section:
    Definitions
  ***************************************************************************/
#if defined(TCPIP_STACK_USE_STORAGE) 
    #define HTTP_APP_USE_STORAGE    // this demo can use the storage functions
#endif  // defined(TCPIP_STACK_USE_STORAGE)


/****************************************************************************
  Section:
    Function Prototypes and Memory Globalizers
  ***************************************************************************/
#if defined(HTTP_USE_POST)
    #if defined(SYS_OUT_ENABLE)
        static HTTP_IO_RESULT HTTPPostLCD(HTTP_CONN_HANDLE connHandle);
    #endif
    #if defined(HTTP_APP_USE_MD5)
        static HTTP_IO_RESULT HTTPPostMD5(HTTP_CONN_HANDLE connHandle);
    #endif
    #if defined(TCPIP_STACK_USE_EZ_CONFIG)
        static HTTP_IO_RESULT HTTPPostWifiConfig(HTTP_CONN_HANDLE connHandle);
    #endif
    #if defined(HTTP_APP_USE_RECONFIG)
        static HTTP_IO_RESULT HTTPPostConfig(HTTP_CONN_HANDLE connHandle);
        #if defined(TCPIP_STACK_USE_SNMP_SERVER)
        static HTTP_IO_RESULT HTTPPostSNMPCommunity(HTTP_CONN_HANDLE connHandle);
        #endif
    #endif
    #if defined(HTTP_APP_USE_EMAIL) || defined(TCPIP_STACK_USE_SMTP_CLIENT)
        static HTTP_IO_RESULT HTTPPostEmail(HTTP_CONN_HANDLE connHandle);
    #endif
    #if defined(TCPIP_STACK_USE_DYNAMICDNS_CLIENT)
        static HTTP_IO_RESULT HTTPPostDDNSConfig(HTTP_CONN_HANDLE connHandle);
    #endif
#endif
extern bool convertAsciiToHexInPlace(char *p_string, uint8_t expectedHexBinSize );
// bss descriptor fetched from on-chip
static t_wfScanResult bssDesc;
static bool bssDescIsValid = false;

// RAM allocated for DDNS parameters
#if defined(TCPIP_STACK_USE_DYNAMICDNS_CLIENT)
    static uint8_t DDNSData[100];
#endif

// Sticky status message variable.
// This is used to indicated whether or not the previous POST operation was 
// successful.  The application uses these to store status messages when a 
// POST operation redirects.  This lets the application provide status messages
// after a redirect, when connection instance data has already been lost.
static bool lastSuccess = false;

// Stick status message variable.  See lastSuccess for details.
static bool lastFailure = false;
#if (WF_DEFAULT_NETWORK_TYPE == WF_NETWORK_TYPE_SOFT_AP)
extern t_wfScanResult preScanResult[50]; // WF_PRESCAN
#endif

/****************************************************************************
  Section:
    Authorization Handlers
  ***************************************************************************/
  
/*****************************************************************************
  Function:
    uint8_t HTTPNeedsAuth(HTTP_CONN_HANDLE connHandle, uint8_t* cFile)

  Internal:
      See documentation in the TCP/IP Stack API or HTTP2.h for details.
  ***************************************************************************/
#if defined(HTTP_USE_AUTHENTICATION)
uint8_t HTTPNeedsAuth(HTTP_CONN_HANDLE connHandle, uint8_t* cFile)
{
    // If the filename begins with the folder "protect", then require auth
    if(memcmp(cFile, (const void*)"protect", 7) == 0)
        return 0x00;        // Authentication will be needed later

    // If the filename begins with the folder "snmp", then require auth
    if(memcmp(cFile, (const void*)"snmp", 4) == 0)
        return 0x00;        // Authentication will be needed later

    #if defined(HTTP_MPFS_UPLOAD_REQUIRES_AUTH)
    if(memcmp(cFile, (const void*)"mpfsupload", 10) == 0)
        return 0x00;
    #endif
    // You can match additional strings here to password protect other files.
    // You could switch this and exclude files from authentication.
    // You could also always return 0x00 to require auth for all files.
    // You can return different values (0x00 to 0x79) to track "realms" for below.

    return 0x80;            // No authentication required
}
#endif

/*****************************************************************************
  Function:
    uint8_t HTTPCheckAuth(HTTP_CONN_HANDLE connHandle, uint8_t* cUser, uint8_t* cPass)

  Internal:
      See documentation in the TCP/IP Stack API or HTTP2.h for details.
  ***************************************************************************/
#if defined(HTTP_USE_AUTHENTICATION)
uint8_t HTTPCheckAuth(HTTP_CONN_HANDLE connHandle, uint8_t* cUser, uint8_t* cPass)
{
    if(strcmp((char *)cUser,(const char *)"admin") == 0
        && strcmp((char *)cPass, (const char *)"microchip") == 0)
        return 0x80;        // We accept this combination

    // You can add additional user/pass combos here.
    // If you return specific "realm" values above, you can base this
    //   decision on what specific file or folder is being accessed.
    // You could return different values (0x80 to 0xff) to indicate
    //   various users or groups, and base future processing decisions
    //   in HTTPExecuteGet/Post or HTTPPrint callbacks on this value.

    return 0x00;            // Provided user/pass is invalid
}
#endif

/****************************************************************************
  Section:
    GET Form Handlers
  ***************************************************************************/
  typedef union 
{
    uint16_t Val;
    uint8_t v[2];
} MCHP_UINT16_VAL;
/*****************************************************************************
  Function:
    HTTP_IO_RESULT HTTPExecuteGet(HTTP_CONN_HANDLE connHandle)

  Internal:
      See documentation in the TCP/IP Stack API or HTTP2.h for details.
  ***************************************************************************/
HTTP_IO_RESULT HTTPExecuteGet(HTTP_CONN_HANDLE connHandle)
{
    const uint8_t *ptr, *ptr1;
    uint8_t filename[20];
    uint8_t* httpDataBuff;
    uint8_t bssIdx;
    MCHP_UINT16_VAL bssIdxStr;
    // Load the file name
    // Make sure uint8_t filename[] above is large enough for your longest name
    MPFSGetFilename(HTTPCurConnectionFileGet(connHandle), filename, 20);

    httpDataBuff = HTTPCurConnectionDataBufferGet(connHandle);
    // If its the forms.htm page
    if(!memcmp(filename, "forms.htm", 9))
    {
        // Seek out each of the four LED strings, and if it exists set the LED states
        ptr = HTTPGetArg(httpDataBuff, (const uint8_t *)"led4");
        if(ptr)
            LED4_IO = (*ptr == '1');

        ptr = HTTPGetArg(httpDataBuff, (const uint8_t *)"led3");
        if(ptr)
            LED3_IO = (*ptr == '1');

        ptr = HTTPGetArg(httpDataBuff, (const uint8_t *)"led2");
        if(ptr)
            LED2_IO = (*ptr == '1');

        ptr = HTTPGetArg(httpDataBuff, (const uint8_t *)"led1");
        if(ptr)
            LED1_IO = (*ptr == '1');
    }

    // If it's the LED updater file
    else if(!memcmp(filename, "cookies.htm", 11))
    {
        // This is very simple.  The names and values we want are already in
        // the data array.  We just set the hasArgs value to indicate how many
        // name/value pairs we want stored as cookies.
        // To add the second cookie, just increment this value.
        // remember to also add a dynamic variable callback to control the printout.
        HTTPCurConnectionHasArgsSet(connHandle, 0x01);
    }


    // If it's the LED updater file
    else if(!memcmp(filename, "leds.cgi", 8))
    {
        // Determine which LED to toggle
        ptr = HTTPGetArg(httpDataBuff, (const uint8_t *)"led");

        // Toggle the specified LED
        switch(*ptr) {
            case '1':
                LED1_IO ^= 1;
                break;
            case '2':
                LED2_IO ^= 1;
                break;
            case '3':
                LED3_IO ^= 1;
                break;
            case '4':
                LED4_IO ^= 1;
                break;
            case '5':
                LED5_IO ^= 1;
                break;
            case '6':
                LED6_IO ^= 1;
                break;
            case '7':
                LED7_IO ^= 1;
                break;
        }

    }
    else if(!memcmp(filename, "scan.cgi", 8))
    {
        ptr = HTTPGetArg(httpDataBuff, (const uint8_t *)"scan");
        ptr1 = HTTPGetArg(httpDataBuff, (const uint8_t *)"getBss");
        
        if ((ptr != NULL) && (ptr1 == NULL))
        {
            bssDescIsValid = false;
            // scan request
            #if (WF_DEFAULT_NETWORK_TYPE == WF_NETWORK_TYPE_SOFT_AP)
            //TCPIP_NET_IF* p_config= (TCPIP_NET_IF*)GetNetworkConfig();
            if (p_wifi_ConfigData->networkType == WF_NETWORK_TYPE_SOFT_AP)
            {   // SoftAP: display pre-scan results before starting as SoftAP. SoftAP does not scan.
                SYS_CONSOLE_MESSAGE("HTTPExecuteGet: SoftAP ... bypass scan  ................. \r\n");
            }
            else
            {
                if (WFStartScan() == WF_SUCCESS)
                {
                    SYS_CONSOLE_MESSAGE("HTTPExecuteGet: WFStartScan success ................. \r\n");
                    SCAN_SET_DISPLAY(SCANCXT.scanState);
                    SCANCXT.displayIdx = 0;
                }
                //else
                //     putsUART("HTTPExecuteGet: WFStartScan fails ................. \r\n");
            }
            #else
            if (WFStartScan() == WF_SUCCESS)
            {
                //SCAN_SET_DISPLAY(SCANCXT.scanState);
                //SCANCXT.displayIdx = 0;
            }
            #endif
        }
        else if ((ptr == NULL) && (ptr1 != NULL))
        {
            // getBss request
            // use the value to get the nth bss stored on chip
            bssDescIsValid = false;
            bssIdxStr.v[1] = *ptr1;
            bssIdxStr.v[0] = *(ptr1+1);    
            bssIdx = hexatob(bssIdxStr.Val);

            #if (WF_DEFAULT_NETWORK_TYPE == WF_NETWORK_TYPE_SOFT_AP)
            //TCPIP_NET_IF* p_config= (TCPIP_NET_IF*)GetNetworkConfig();
            if (p_wifi_ConfigData->networkType == WF_NETWORK_TYPE_SOFT_AP)
            {   // SoftAP: display pre-scan results before starting as SoftAP. SoftAP does not scan.
                bssDesc = preScanResult[bssIdx];
                //putsUART("HTTPExecuteGet: SoftAP ... display pre-scan  ................. \r\n");
            }
            else
            {
            WFRetrieveScanResult(bssIdx, &bssDesc);
            }
            #else 
            WFRetrieveScanResult(bssIdx, &bssDesc);
            #endif
            bssDescIsValid = true;
        }
        else
        {
            // impossible to get here
        }                  
    }
    return HTTP_IO_DONE;
}


/****************************************************************************
  Section:
    POST Form Handlers
  ***************************************************************************/
#if defined(HTTP_USE_POST)

/*****************************************************************************
  Function:
    HTTP_IO_RESULT HTTPExecutePost(HTTP_CONN_HANDLE connHandle)

  Internal:
      See documentation in the TCP/IP Stack API or HTTP2.h for details.
  ***************************************************************************/
HTTP_IO_RESULT HTTPExecutePost(HTTP_CONN_HANDLE connHandle)
{
    // Resolve which function to use and pass along
    uint8_t filename[20];

    // Load the file name
    // Make sure uint8_t filename[] above is large enough for your longest name
    MPFSGetFilename(HTTPCurConnectionFileGet(connHandle), filename, sizeof(filename));

#if defined(SYS_OUT_ENABLE)
    if(!memcmp(filename, "forms.htm", 9))
        return HTTPPostLCD(connHandle);
#endif

#if defined(HTTP_APP_USE_MD5)
    if(!memcmp(filename, "upload.htm", 10))
        return HTTPPostMD5(connHandle);
#endif

#if defined(TCPIP_STACK_USE_EZ_CONFIG)
    if(!memcmp(filename, "configure.htm", 13))
        return HTTPPostWifiConfig(connHandle) /*HTTPPostConfig()*/;
#endif
#if defined(HTTP_APP_USE_RECONFIG)
    if(!memcmp(filename, "protect/config.htm", 18))
        return HTTPPostConfig(connHandle);
    #if defined(TCPIP_STACK_USE_SNMP_SERVER)
    else if(!memcmp(filename, "snmp/snmpconfig.htm", 19))
        return HTTPPostSNMPCommunity(connHandle);
    #endif
#endif

#if defined(TCPIP_STACK_USE_SMTP_CLIENT)
    if(!strcmp((char*)filename, "email/index.htm"))
        return HTTPPostEmail(connHandle);
#endif

#if defined(TCPIP_STACK_USE_DYNAMICDNS_CLIENT)
    if(!strcmp((char*)filename, "dyndns/index.htm"))
        return HTTPPostDDNSConfig(connHandle);
#endif

    return HTTP_IO_DONE;
}

/*****************************************************************************
  Function:
    static HTTP_IO_RESULT HTTPPostLCD(HTTP_CONN_HANDLE connHandle)

  Summary:
    Processes the LCD form on forms.htm

  Description:
    Locates the 'lcd' parameter and uses it to update the text displayed
    on the board's LCD display.

    This function has four states.  The first reads a name from the data
    string returned as part of the POST request.  If a name cannot
    be found, it returns, asking for more data.  Otherwise, if the name
    is expected, it reads the associated value and writes it to the LCD.
    If the name is not expected, the value is discarded and the next name
    parameter is read.

    In the case where the expected string is never found, this function
    will eventually return HTTP_IO_NEED_DATA when no data is left.  In that
    case, the HTTP2 server will automatically trap the error and issue an
    Internal Server Error to the browser.

  Precondition:
    None

  Parameters:
    None

  Return Values:
      HTTP_IO_DONE - the parameter has been found and saved
      HTTP_IO_WAITING - the function is pausing to continue later
      HTTP_IO_NEED_DATA - data needed by this function has not yet arrived
  ***************************************************************************/
#if defined(SYS_OUT_ENABLE)
static HTTP_IO_RESULT HTTPPostLCD(HTTP_CONN_HANDLE connHandle)
{
    uint8_t* cDest;
    uint8_t* httpDataBuff;

    #define SM_POST_LCD_READ_NAME        (0u)
    #define SM_POST_LCD_READ_VALUE        (1u)

    httpDataBuff = HTTPCurConnectionDataBufferGet(connHandle);
    switch(HTTPCurConnectionPostSmGet(connHandle))
    {
        // Find the name
        case SM_POST_LCD_READ_NAME:

            // Read a name
            if(HTTPReadPostName(connHandle, httpDataBuff, HTTP_MAX_DATA_LEN) == HTTP_READ_INCOMPLETE)
                return HTTP_IO_NEED_DATA;

            HTTPCurConnectionPostSmSet(connHandle, SM_POST_LCD_READ_VALUE);
            // No break...continue reading value

        // Found the value, so store the LCD and return
        case SM_POST_LCD_READ_VALUE:

            // If value is expected, read it to data buffer,
            // otherwise ignore it (by reading to NULL)
            if(!strcmp((char*)httpDataBuff, (const char*)"lcd"))
                cDest = httpDataBuff;
            else
                cDest = NULL;

            // Read a value string
            if(HTTPReadPostValue(connHandle, cDest, HTTP_MAX_DATA_LEN) == HTTP_READ_INCOMPLETE)
                return HTTP_IO_NEED_DATA;

            // If this was an unexpected value, look for a new name
            if(!cDest)
            {
                HTTPCurConnectionPostSmSet(connHandle, SM_POST_LCD_READ_NAME);
                break;
            }

                        SYS_OUT_MESSAGE((char*)cDest);

            // This is the only expected value, so callback is done
            strcpy((char*)httpDataBuff, "/forms.htm");
            HTTPCurConnectionStatusSet(connHandle, HTTP_REDIRECT);
            return HTTP_IO_DONE;
    }

    // Default assumes that we're returning for state machine convenience.
    // Function will be called again later.
    return HTTP_IO_WAITING;
}
#endif
/*****************************************************************************
  Function:
    static HTTP_IO_RESULT HTTPPostWifiConfig(HTTP_CONN_HANDLE connHandle)

  Summary:
    Processes the wifi config data

  Description:
    Accepts wireless configuration data from the www site and saves them to a
    structure to be applied by the ZG configuration manager.

    The following configurations are possible:
         i) Mode: adhoc or infrastructure
        ii) Security:
               - None
               - WPA/WPA2 passphrase
               - WPA/WPA2 pre-calculated key
               - WEP 64-bit
               - WEP 128-bit
       iii) Key material
    
    If an error occurs, such as data is invalid they will be redirected to a page
    informing the user of such results.  

    NOTE: This code for modified originally from HTTPPostWifiConfig as distributed
          by Microchip.

  Precondition:
    None

  Parameters:
    None

  Return Values:
    HTTP_IO_DONE - all parameters have been processed
    HTTP_IO_NEED_DATA - data needed by this function has not yet arrived
  ***************************************************************************/
#if defined(TCPIP_STACK_USE_EZ_CONFIG)
static HTTP_IO_RESULT HTTPPostWifiConfig(HTTP_CONN_HANDLE connHandle)
{
        UINT8 ssidLen;
        TCPIP_NET_HANDLE currNet;//, newNet;
        uint32_t byteCount;
        TCP_SOCKET sktHTTP;
        uint8_t* httpDataBuff = 0;
        byteCount = HTTPCurConnectionByteCountGet(connHandle);
        sktHTTP = HTTPCurConnectionSocketGet(connHandle);
        if(byteCount > TCPIsGetReady(sktHTTP) + TCPGetRxFIFOFree(sktHTTP))
            goto ConfigFailure;

        // current interface we're working on
        currNet = TCPSocketGetNet(sktHTTP);
        // ensure our static approach is working OK

        // Ensure that all data is waiting to be parsed.  If not, keep waiting for
        // all of it to arrive.
        if(TCPIsGetReady(sktHTTP) < byteCount)
            return HTTP_IO_NEED_DATA;

        httpDataBuff = HTTPCurConnectionDataBufferGet(connHandle);
        // Read all browser POST data
        while(HTTPCurConnectionByteCountGet(connHandle))
        {
            // Read a form field name
            if(HTTPReadPostName(connHandle, httpDataBuff, 6) != HTTP_READ_OK)
                goto ConfigFailure;

            // Read a form field value
            if(HTTPReadPostValue(connHandle, httpDataBuff + 6, HTTP_MAX_DATA_LEN-6-2) != HTTP_READ_OK)
                goto ConfigFailure;

            // Parse the value that was read
            if(!strcmp((char*)httpDataBuff, (const char*)"sec"))
            {// Read security type
                char   security_type[7];

                if (strlen((char*)(httpDataBuff+6)) > 6) /* Sanity check */
                    goto ConfigFailure;

                memcpy(security_type, (void*)(httpDataBuff+6), strlen((char*)(httpDataBuff+6)));
                security_type[strlen((char*)(httpDataBuff+6))] = 0; /* Terminate string */

                if (!strcmp((char*)security_type, (const char*)"no"))
                {
                    CFGCXT.security = WF_SECURITY_OPEN;
                    SYS_CONSOLE_MESSAGE("\r\n1.0 sec=open\r\n");
                }
                else if(!strcmp((char*)security_type, (const char*)"wpa"))
                {
                    CFGCXT.security = WF_SECURITY_WPA_AUTO_WITH_PASS_PHRASE;
                    SYS_CONSOLE_MESSAGE("\r\n1.1 WPA passphrase\r\n");
                }
                else if(!strcmp((char*)security_type, (const char*)"calc"))
                {    /* Pre-calculated key */
                    CFGCXT.security = WF_SECURITY_WPA_AUTO_WITH_KEY;
                }
                else if(!strcmp((char*)security_type, (const char*)"wep40"))
                {
                    CFGCXT.security = WF_SECURITY_WEP_40;
                    SYS_CONSOLE_MESSAGE("\r\n1.3 Select wep64 on www\r\n");
                }
                else if(!strcmp((char*)security_type, (const char*)"wep104"))
                {
                    CFGCXT.security = WF_SECURITY_WEP_104;
                    SYS_CONSOLE_MESSAGE("\r\n1.4 Select wep128 on www\r\n");
                }
                else
                {    //Security type no good  :-(
                    SYS_CONSOLE_MESSAGE("\r\n1.5 Unknown key type on www\r\n");
                    goto ConfigFailure;
                }
            }
            else if(!strcmp((char*)httpDataBuff, (const char*)"key"))
            {// Read new key material
                uint8_t key_size = 0, ascii_key = 0;
                switch ((uint8_t)CFGCXT.security) {
                    case WF_SECURITY_OPEN:     //keep compiler happy, nothing to do here!
                        break;
                    case WF_SECURITY_WPA_AUTO_WITH_PASS_PHRASE:  //wpa passphrase
                        //putrsUART((ROM char*)"\r\nPassphrase type of key! ");
                        ascii_key = 1;
                        key_size = strlen((char *)(httpDataBuff+6));
                        //between 8-63 characters, passphrase
                        if ((key_size < 8 ) || (key_size > 63))
                              goto ConfigFailure;
                        break;
                    case WF_SECURITY_WPA_AUTO_WITH_KEY: //wpa pre-calculated key!!!
                        key_size = 64;
                        break;
                    case WF_SECURITY_WEP_40:
                        key_size = 10; /* Assume hex size */
                        if (strlen((char *)(httpDataBuff+6)) == 5) {
                            key_size = 5;  /* ASCII key support */
                            ascii_key = 1;
                         }
                        CFGCXT.defaultWepKey = 0; /* Example uses only key idx 0 (sometimes called 1) */
                        break;
                    case WF_SECURITY_WEP_104:
                        key_size = 26; /* Assume hex size */
                        if (strlen((char *)(httpDataBuff+6)) == 13) {
                            key_size = 13;    /* ASCII key support */
                            ascii_key = 1;
                        }
                        CFGCXT.defaultWepKey = 0; /* Example uses only key idx 0 (sometimes called 1) */
                        break;
                }
                if (strlen((char *)(httpDataBuff+6)) != key_size) {
                    SYS_CONSOLE_MESSAGE("\r\nIncomplete key received\r\n");
                    goto ConfigFailure;
                }
                memcpy(CFGCXT.key, (void*)(httpDataBuff+6), key_size);
                CFGCXT.key[key_size] = 0; /* terminate string */
                CFGCXT.SecurityKeyLength = key_size;
                if (!ascii_key) {
                    //if ((cfg.security == sec_wep64) || (cfg.security == sec_wep128))
                        key_size /= 2;
                        //CFGCXT.SecurityKeyLength = key_size;   // yes Or No? Need more debug
                    if (!convertAsciiToHexInPlace((char *)&CFGCXT.key[0], key_size)) {
                        SYS_CONSOLE_MESSAGE("\r\nFailed to convert ASCII to hex!\r\n");
                        goto ConfigFailure;
                    }
                }
            }
            else if(!strcmp((char*)httpDataBuff, "ssid"))
            {// Get new ssid and make sure it is valid
                if(strlen((char*)(httpDataBuff+6)) < 33u)
                {
                    memcpy(CFGCXT.ssid, (void*)(httpDataBuff+6), strlen((char*)(httpDataBuff+6)));
                    CFGCXT.ssid[strlen((char*)(httpDataBuff+6))] = 0; /* Terminate string */

                    /* save current profile SSID for displaying later */
                    WF_SsidGet((UINT8*)&CFGCXT.prevSSID, &ssidLen);
                    CFGCXT.prevSSID[ssidLen] = 0;
                }
                else
                {    //Invalid SSID... fail :-(
                    goto ConfigFailure;
                }
            }
            else if(!strcmp((char*)httpDataBuff, (const char*)"wlan"))
            {// Get the wlan mode: adhoc or infrastructure
                char mode[6];
                if (strlen((char*)(httpDataBuff+6)) > 5) /* Sanity check */
                    goto ConfigFailure;

                memcpy(mode, (void*)(httpDataBuff+6), strlen((char*)(httpDataBuff+6)));
                mode[strlen((char*)(httpDataBuff+6))] = 0; /* Terminate string */
                if(!strcmp((char*)mode, (const char*)"infra"))
                {
                    SYS_CONSOLE_MESSAGE((const char*)"\r\nSetting mode to infrastructure!\r\n");
                    CFGCXT.type = WF_NETWORK_TYPE_INFRASTRUCTURE;
                }
                else if(!strcmp((char*)mode, "adhoc"))
                {
                    t_adHocNetworkContext adhocContext;

                    SYS_CONSOLE_MESSAGE((const char*)"\r\nSetting mode to adhoc!\r\n");
                    CFGCXT.type = WF_NETWORK_TYPE_ADHOC;

                    // always setup adhoc to attempt to connect first, then start
                    adhocContext.mode = WF_ADHOC_CONNECT_THEN_START;
                    adhocContext.beaconPeriod = WF_DEFAULT_ADHOC_BEACON_PERIOD;
                    adhocContext.hiddenSsid   = WF_DEFAULT_ADHOC_HIDDEN_SSID;
                    WF_AdhocContextSet(&adhocContext);
                }
                else
                {    //Mode type no good  :-(
                    //memset(LCDText, ' ', 32);
                    //putrsUART((ROM char*)"\r\nUnknown mode type on www! ");
                    //strcpy((char*)LCDText, (char*)mode);
                    //LCDUpdate();
                    SYS_CONSOLE_MESSAGE((const char*)"\r\nUnknown mode type on www! ");
                    goto ConfigFailure;
                }

                // save old WLAN mode
                WF_NetworkTypeGet(&CFGCXT.prevWLAN);
            }
        }

        /* Check if WPA hasn't been selected with adhoc, if it has we choke! */
        if ((CFGCXT.type == WF_NETWORK_TYPE_ADHOC) &&
          ((CFGCXT.security == WF_SECURITY_WPA_AUTO_WITH_PASS_PHRASE) || (CFGCXT.security == WF_SECURITY_WPA_AUTO_WITH_KEY)))
            goto ConfigFailure;

        /*
         * All parsing complete!  If we have got to here all data has been validated and
         * We can handle what is necessary to start the reconfigure process of the WiFi device
         */
#if defined ( EZ_CONFIG_STORE )
        /* Copy wifi cfg data to be committed to NVM */
        //TCPIP_NET_IF* p_config= (TCPIP_NET_IF*)GetNetworkConfig();
        strcpy((char *)p_wifi_ConfigData->netSSID, (char *)CFGCXT.ssid);
        p_wifi_ConfigData->SsidLength = strlen((char*)(CFGCXT.ssid));
        /* Going to set security type */
        p_wifi_ConfigData->SecurityMode = CFGCXT.security;
        /* Going to save the key, if required */
        if (CFGCXT.security != WF_SECURITY_OPEN) {
            uint8_t  key_size =0;

            switch ((uint8_t)CFGCXT.security) {
                case WF_SECURITY_WPA_AUTO_WITH_PASS_PHRASE:  //wpa passphrase
                    key_size = strlen((char*)(CFGCXT.key)); //ascii so use strlen
                    break;
                case WF_SECURITY_WPA_AUTO_WITH_KEY: //wpa pre-calculated key!!!
                    key_size = 32;
                    break;
                case WF_SECURITY_WEP_40:
                    key_size = 5;
                    break;
                case WF_SECURITY_WEP_104:
                    key_size = 13;
                    break;

            }

            memcpy(p_wifi_ConfigData->SecurityKey, CFGCXT.key, key_size);
            p_wifi_ConfigData->SecurityKey[strlen((char*)(CFGCXT.key))] = 0;
            p_wifi_ConfigData->SecurityKeyLength = key_size;

        }
        /* Going to save the network type */
        p_wifi_ConfigData->networkType = CFGCXT.type;
        p_wifi_ConfigData->dataValid = 1; /* Validate wifi configuration */
#endif // EZ_CONFIG_STORE

                strcpy((char*)httpDataBuff, "/reconnect.htm");
                //memcpy((void*)(httpDataBuff+14), TCPIP_STACK_NetBIOSName(newNet), 16);
               // httpDataBuff[14+16] = 0x00;    // Force null termination
               // for(i = 14; i < 14u+16u; i++)
                {
                //    if(httpDataBuff[i] == ' ')
        //    httpDataBuff[i] = 0x00;
                }
                HTTPCurConnectionStatusSet(connHandle, HTTP_REDIRECT);
                WF_START_EASY_CONFIG();
                WifiAsyncSetEventPending(ASYNC_EASY_CONFIG_PENDING);


        return HTTP_IO_DONE;


    ConfigFailure:
                lastFailure = true;
                if(httpDataBuff)
                {
                    strcpy((char*)httpDataBuff, "/error.htm");
                }
                HTTPCurConnectionStatusSet(connHandle, HTTP_REDIRECT);
        return HTTP_IO_DONE;
}

#endif    

/*****************************************************************************
  Function:
    static HTTP_IO_RESULT HTTPPostConfig(HTTP_CONN_HANDLE connHandle)

  Summary:
    Processes the configuration form on config/index.htm

  Description:
    Accepts configuration parameters from the form, saves them to a
    temporary location in RAM, then eventually saves the data to EEPROM or
    external Flash.

    When complete, this function redirects to config/reboot.htm, which will
    display information on reconnecting to the board.

    This function creates a shadow copy of a network info structure in
    RAM and then overwrites incoming data there as it arrives.  For each
    name/value pair, the name is first read to cur connection data[0:5].  Next, the
    value is read to newNetConfig.  Once all data has been read, the new
    network info structure is saved back to storage and the browser is redirected to
    reboot.htm.  That file includes an AJAX call to reboot.cgi, which
    performs the actual reboot of the machine.

    If an IP address cannot be parsed, too much data is POSTed, or any other
    parsing error occurs, the browser reloads config.htm and displays an error
    message at the top.

  Precondition:
    None

  Parameters:
    None

  Return Values:
      HTTP_IO_DONE - all parameters have been processed
      HTTP_IO_NEED_DATA - data needed by this function has not yet arrived
  ***************************************************************************/
#if defined(HTTP_APP_USE_RECONFIG)
static HTTP_IO_RESULT HTTPPostConfig(HTTP_CONN_HANDLE connHandle)
{
    // room to store the new network configuration/information
    // to be sure that always you have the right amount of space neede
    // dynamic allocation should be used
    uint8_t newNetInfo[200];
    uint8_t *ptr;
    uint8_t i;
    TCPIP_NET_HANDLE currNet, newNet;
    IPV4_ADDR newIPAddress, newMask;
    MAC_ADDR    myMACAddr;
#if defined(HTTP_APP_USE_STORAGE)
    int stgIx;
    TCPIP_STORAGE_HANDLE hS = 0;
#endif  // defined(HTTP_APP_USE_STORAGE)
    uint32_t byteCount;
    TCP_SOCKET sktHTTP;
    uint8_t* httpDataBuff = 0;

    
    // Check to see if the browser is attempting to submit more data than we
    // can parse at once.  This function needs to receive all updated
    // parameters and validate them all before committing them to memory so that
    // orphaned configuration parameters do not get written (for example, if a
    // static IP address is given, but the subnet mask fails parsing, we
    // should not use the static IP address).  Everything needs to be processed
    // in a single transaction.  If this is impossible, fail and notify the user.
    // As a web devloper, if you add parameters to the network info and run into this
    // problem, you could fix this by to splitting your update web page into two
    // seperate web pages (causing two transactional writes).  Alternatively,
    // you could fix it by storing a static shadow copy of network info someplace
    // in memory and using it instead of newNetInfo.  Lastly, you could
    // increase the TCP RX FIFO size for the HTTP server.  This will allow more
    // data to be POSTed by the web browser before hitting this limit.
    byteCount = HTTPCurConnectionByteCountGet(connHandle);
    sktHTTP = HTTPCurConnectionSocketGet(connHandle);
    if(byteCount > TCPIsGetReady(sktHTTP) + TCPGetRxFIFOFree(sktHTTP))
        goto ConfigFailure;

    // current interface we're working on
    currNet = TCPSocketGetNet(sktHTTP);
    // ensure our static approach is working OK
    if(sizeof(newNetInfo) < TCPIP_STACK_NetInfoSize(currNet))
    {
        goto ConfigFailure;
    }
    
    // create a copy of the existent network configuration
    newNet = TCPIP_STACK_CreateNetInfo(currNet, newNetInfo, sizeof(newNetInfo), true);
    if(newNet == 0)
    {
        goto ConfigFailure;
    }
    
    // Ensure that all data is waiting to be parsed.  If not, keep waiting for 
    // all of it to arrive.
    if(TCPIsGetReady(sktHTTP) < byteCount)
        return HTTP_IO_NEED_DATA;

        
    
    // Use current config in non-volatile memory as defaults
    #if defined(HTTP_APP_USE_STORAGE)
    hS = TCPIP_STORAGE_Open(0, true);
    if(hS != TCPIP_STORAGE_HANDLE_INVALID)
    {
        stgIx = TCPIP_STORAGE_FindEntry(hS, currNet);
        if(stgIx >= 0)
        {   // found a valid configuration stored
            TCPIP_STORAGE_ReadEntryIx(hS, stgIx, newNet);
        }
    }
    #endif  // defined(HTTP_APP_USE_STORAGE)

    // Start out assuming that DHCP is disabled.  This is necessary since the
    // browser doesn't submit this field if it is unchecked (meaning zero).
    // However, if it is checked, this will be overridden since it will be
    // submitted.
    DHCPDisable(newNet);

    httpDataBuff = HTTPCurConnectionDataBufferGet(connHandle);

    // Read all browser POST data
    while(HTTPCurConnectionByteCountGet(connHandle))
    {
        // Read a form field name
        if(HTTPReadPostName(connHandle, httpDataBuff, 6) != HTTP_READ_OK)
            goto ConfigFailure;

        // Read a form field value
        if(HTTPReadPostValue(connHandle, httpDataBuff + 6, HTTP_MAX_DATA_LEN-6-2) != HTTP_READ_OK)
            goto ConfigFailure;

        // Parse the value that was read
        if(!strcmp((char*)httpDataBuff, (const char*)"ip"))
        {// Read new static IP Address
            if(!TCPIP_HELPER_StringToIPAddress((char*)(httpDataBuff+6), &newIPAddress))
                goto ConfigFailure;

            TCPIP_STACK_SetNetAddress(newNet, &newIPAddress, 0, true);
        }
        else if(!strcmp((char*)httpDataBuff, (const char*)"gw"))
        {// Read new gateway address
            if(!TCPIP_HELPER_StringToIPAddress((char*)(httpDataBuff+6), &newIPAddress))
                goto ConfigFailure;

            TCPIP_STACK_SetNetGatewayAddress(newNet, &newIPAddress);
        }
        else if(!strcmp((char*)httpDataBuff, (const char*)"sub"))
        {// Read new static subnet
            if(!TCPIP_HELPER_StringToIPAddress((char*)(httpDataBuff+6), &newMask))
                goto ConfigFailure;

			TCPIP_STACK_SetNetAddress(newNet, 0, &newMask, true);
        }
        else if(!strcmp((char*)httpDataBuff, (const char*)"dns1"))
        {// Read new primary DNS server
            if(!TCPIP_HELPER_StringToIPAddress((char*)(httpDataBuff+6), &newIPAddress))
                goto ConfigFailure;
            TCPIP_STACK_SetNetPriDNSAddress(newNet, &newIPAddress);
        }
        else if(!strcmp((char*)httpDataBuff, (const char*)"dns2"))
        {// Read new secondary DNS server
            if(!TCPIP_HELPER_StringToIPAddress((char*)(httpDataBuff+6), &newIPAddress))
                goto ConfigFailure;
            TCPIP_STACK_SetNetSecondDNSAddress(newNet, &newIPAddress);
        }
        else if(!strcmp((char*)httpDataBuff, (const char*)"mac"))
        {
            // Read new MAC address
            uint16_t w;
            uint8_t i;

            ptr = httpDataBuff+6;

            for(i = 0; i < 12u; i++)
            {// Read the MAC address

                // Skip non-hex bytes
                while( *ptr != 0x00u && !(*ptr >= '0' && *ptr <= '9') && !(*ptr >= 'A' && *ptr <= 'F') && !(*ptr >= 'a' && *ptr <= 'f') )
                    ptr++;

                // MAC string is over, so zeroize the rest
                if(*ptr == 0x00u)
                {
                    for(; i < 12u; i++)
                        httpDataBuff[i] = '0';
                    break;
                }

                // Save the MAC byte
                httpDataBuff[i] = *ptr++;
            }

            // Read MAC Address, one byte at a time
            for(i = 0; i < 6u; i++)
            {
                ((uint8_t*)&w)[1] = httpDataBuff[i*2];
                ((uint8_t*)&w)[0] = httpDataBuff[i*2+1];
                myMACAddr.v[i] = hexatob(w);
            }
            TCPIP_STACK_SetNetMacAddress(newNet, &myMACAddr);
        }
        else if(!strcmp((char*)httpDataBuff, (const char*)"host"))
        {// Read new hostname
            TCPIP_STACK_SetNetBIOSName(newNet, (const char* )&httpDataBuff[6]);
        }
        else if(!strcmp((char*)httpDataBuff, (const char*)"dhcp"))
        {// Read new DHCP Enabled flag
            if(httpDataBuff[6] == '1')
                DHCPEnable(newNet);
        }
    }


    // All parsing complete!  Save new settings and force a reboot
#if defined(HTTP_APP_USE_STORAGE)
    if(hS != 0)
    {
        TCPIP_STORAGE_WriteEntry(hS, newNet, 0);
        TCPIP_STORAGE_Close(hS);   // release handle 
    }
#endif  // defined(HTTP_APP_USE_STORAGE)
    
    // Set the board to reboot and display reconnecting information
    strcpy((char*)httpDataBuff, "/protect/reboot.htm?");
    memcpy((void*)(httpDataBuff+20), TCPIP_STACK_NetBIOSName(newNet), 16);
    httpDataBuff[20+16] = 0x00;    // Force null termination
    for(i = 20; i < 20u+16u; i++)
    {
        if(httpDataBuff[i] == ' ')
            httpDataBuff[i] = 0x00;
    }
    HTTPCurConnectionStatusSet(connHandle, HTTP_REDIRECT);

    return HTTP_IO_DONE;


ConfigFailure:
    lastFailure = true;
    if(httpDataBuff)
    {
        strcpy((char*)httpDataBuff, "/protect/config.htm");
    }
    HTTPCurConnectionStatusSet(connHandle, HTTP_REDIRECT);

#if defined(HTTP_APP_USE_STORAGE)
    if(hS != 0)
    {
        TCPIP_STORAGE_Close(hS);   // release handle 
    }
#endif  // defined(HTTP_APP_USE_STORAGE)

    return HTTP_IO_DONE;
}

#if defined(TCPIP_STACK_USE_SNMP_SERVER)
static HTTP_IO_RESULT HTTPPostSNMPCommunity(HTTP_CONN_HANDLE connHandle)
{
    uint8_t vCommunityIndex;
    uint8_t *dest;
    uint8_t* httpDataBuff;
    TCP_SOCKET sktHTTP;

    #define SM_CFG_SNMP_READ_NAME    (0u)
    #define SM_CFG_SNMP_READ_VALUE    (1u)

    httpDataBuff = HTTPCurConnectionDataBufferGet(connHandle);
    sktHTTP = HTTPCurConnectionSocketGet(connHandle);
    switch(HTTPCurConnectionPostSmGet(connHandle))
    {
        case SM_CFG_SNMP_READ_NAME:
            // If all parameters have been read, end
            if(HTTPCurConnectionByteCountGet(connHandle) == 0u)
            {
#if defined(HTTP_APP_USE_STORAGE)
                TCPIP_STORAGE_HANDLE hS;
                hS = TCPIP_STORAGE_Open(0, false);
                if(hS != TCPIP_STORAGE_HANDLE_INVALID)
                {
                    TCPIP_STORAGE_SaveStackConfig(hS, false);
                    TCPIP_STORAGE_Close(hS);
                }
#endif /*HTTP_APP_USE_STORAGE */
                return HTTP_IO_DONE;
            }

            // Read a name
            if(HTTPReadPostName(connHandle, httpDataBuff, sizeof(httpDataBuff)-2) == HTTP_READ_INCOMPLETE)
                return HTTP_IO_NEED_DATA;

            // Move to reading a value, but no break
            HTTPCurConnectionPostSmSet(connHandle, SM_CFG_SNMP_READ_VALUE);

        case SM_CFG_SNMP_READ_VALUE:
            // Read a value
            if(HTTPReadPostValue(connHandle, httpDataBuff + 6, sizeof(httpDataBuff)-6-2) == HTTP_READ_INCOMPLETE)
                return HTTP_IO_NEED_DATA;

            // Default action after this is to read the next name, unless there's an error
            HTTPCurConnectionPostSmSet(connHandle, SM_CFG_SNMP_READ_NAME);

            // See if this is a known parameter and legal (must be null
            // terminator in 4th field name byte, string must no greater than
            // SNMP_COMMUNITY_MAX_LEN bytes long, and SNMP_MAX_COMMUNITY_SUPPORT
            // must not be violated.
            vCommunityIndex = httpDataBuff[3] - '0';
            if(vCommunityIndex >= SNMP_MAX_COMMUNITY_SUPPORT)
                break;
            if(httpDataBuff[4] != 0x00u)
                break;
            if(memcmp((void*)httpDataBuff, (const void*)"rcm", 3) == 0)
                dest = ((SNMP_NET_CONFIG*)TCPSocketGetNet(sktHTTP))->readCommunity[vCommunityIndex];
            else if(memcmp((void*)httpDataBuff, (const void*)"wcm", 3) == 0)
                dest = ((SNMP_NET_CONFIG*)TCPSocketGetNet(sktHTTP))->writeCommunity[vCommunityIndex];
            else
                break;
            if(strlen((char*)httpDataBuff + 6) > SNMP_COMMUNITY_MAX_LEN)
                break;

            // String seems valid, lets copy it to the network configuration
            strcpy((char*)dest, (char*)httpDataBuff+6);
            break;
    }

    return HTTP_IO_WAITING;        // Assume we're waiting to process more data
}
#endif //#if defined(TCPIP_STACK_USE_SNMP_SERVER)

#endif    // #if defined(HTTP_APP_USE_RECONFIG)

/*****************************************************************************
  Function:
    static HTTP_IO_RESULT HTTPPostMD5(HTTP_CONN_HANDLE connHandle)

  Summary:
    Processes the file upload form on upload.htm

  Description:
    This function demonstrates the processing of file uploads.  First, the
    function locates the file data, skipping over any headers that arrive.
    Second, it reads the file 64 bytes at a time and hashes that data.  Once
    all data has been received, the function calculates the MD5 sum and
    stores it in current connection data buffer.

    After the headers, the first line from the form will be the MIME
    separator.  Following that is more headers about the file, which we
    discard.  After another CRLFCRLF, the file data begins, and we read
    it 16 bytes at a time and add that to the MD5 calculation.  The reading
    terminates when the separator string is encountered again on its own
    line.  Notice that the actual file data is trashed in this process,
    allowing us to accept files of arbitrary size, not limited by RAM.
    Also notice that the data buffer is used as an arbitrary storage array
    for the result.  The ~uploadedmd5~ callback reads this data later to
    send back to the client.

  Precondition:
    None

  Parameters:
    None

  Return Values:
    HTTP_IO_DONE - all parameters have been processed
    HTTP_IO_WAITING - the function is pausing to continue later
    HTTP_IO_NEED_DATA - data needed by this function has not yet arrived
  ***************************************************************************/
#if defined(HTTP_APP_USE_MD5)
static HTTP_IO_RESULT HTTPPostMD5(HTTP_CONN_HANDLE connHandle)
{
    uint16_t lenA, lenB;
    static HASH_SUM md5;            // Assume only one simultaneous MD5
    TCP_SOCKET sktHTTP;
    uint8_t* httpDataBuff;

    #define SM_MD5_READ_SEPARATOR    (0u)
    #define SM_MD5_SKIP_TO_DATA        (1u)
    #define SM_MD5_READ_DATA        (2u)
    #define SM_MD5_POST_COMPLETE    (3u)

    sktHTTP = HTTPCurConnectionSocketGet(connHandle);
    switch(HTTPCurConnectionPostSmGet(connHandle))
    {
        // Just started, so try to find the separator string
        case SM_MD5_READ_SEPARATOR:
            // Reset the MD5 calculation
            MD5Initialize(&md5);

            // See if a CRLF is in the buffer
            lenA = TCPFindArray(sktHTTP, (const uint8_t*)"\r\n", 2, 0, 0, false);
            if(lenA == 0xffff)
            {//if not, ask for more data
                return HTTP_IO_NEED_DATA;
            }

            // If so, figure out where the last byte of data is
            // Data ends at CRLFseparator--CRLF, so 6+len bytes
            HTTPCurConnectionByteCountDec(connHandle, lenA + 6);

            // Read past the CRLF
            HTTPCurConnectionByteCountDec(connHandle,  TCPGetArray(sktHTTP, NULL, lenA+2));

            // Save the next state (skip to CRLFCRLF)
            HTTPCurConnectionPostSmSet(connHandle, SM_MD5_SKIP_TO_DATA);

            // No break...continue reading the headers if possible

        // Skip the headers
        case SM_MD5_SKIP_TO_DATA:
            // Look for the CRLFCRLF
            lenA = TCPFindArray(sktHTTP, (const uint8_t*)"\r\n\r\n", 4, 0, 0, false);

            if(lenA != 0xffff)
            {// Found it, so remove all data up to and including
                lenA = TCPGetArray(sktHTTP, NULL, lenA+4);
                HTTPCurConnectionByteCountDec(connHandle, lenA);
                HTTPCurConnectionPostSmSet(connHandle, SM_MD5_READ_DATA);
            }
            else
            {// Otherwise, remove as much as possible
                lenA = TCPGetArray(sktHTTP, NULL, TCPIsGetReady(sktHTTP) - 4);
                HTTPCurConnectionByteCountDec(connHandle, lenA);

                // Return the need more data flag
                return HTTP_IO_NEED_DATA;
            }

            // No break if we found the header terminator

        // Read and hash file data
        case SM_MD5_READ_DATA:
            // Find out how many bytes are available to be read
            httpDataBuff = HTTPCurConnectionDataBufferGet(connHandle);
            lenA = TCPIsGetReady(sktHTTP);
            lenB = HTTPCurConnectionByteCountGet(connHandle);
            if(lenA > lenB)
                lenA = lenB;

            while(lenA > 0u)
            {// Add up to 64 bytes at a time to the sum
                lenB = TCPGetArray(sktHTTP, httpDataBuff, (lenA < 64u)?lenA:64);
                HTTPCurConnectionByteCountDec(connHandle, lenB);
                lenA -= lenB;
                MD5AddData(&md5, httpDataBuff, lenB);
            }

            // If we've read all the data
            if(HTTPCurConnectionByteCountGet(connHandle) == 0u)
            {// Calculate and copy result data buffer for printout
                HTTPCurConnectionPostSmSet(connHandle, SM_MD5_POST_COMPLETE);
                MD5Calculate(&md5, httpDataBuff);
                return HTTP_IO_DONE;
            }

            // Ask for more data
            return HTTP_IO_NEED_DATA;
    }

    return HTTP_IO_DONE;
}
#endif // #if defined(HTTP_APP_USE_MD5)

/*****************************************************************************
  Function:
    static HTTP_IO_RESULT HTTPPostEmail(HTTP_CONN_HANDLE connHandle)

  Summary:
    Processes the e-mail form on email/index.htm

  Description:
    This function sends an e-mail message using the SMTP client and
    optionally encrypts the connection to the SMTP server using SSL.  It
    demonstrates the use of the SMTP client, waiting for asynchronous
    processes in an HTTP callback, and how to send e-mail attachments using
    the stack.

    Messages with attachments are sent using multipart/mixed MIME encoding,
    which has three sections.  The first has no headers, and is only to be
    displayed by old clients that cannot interpret the MIME format.  (The
    overwhelming majority of these clients have been obseleted, but the
    so-called "ignored" section is still used.)  The second has a few
    headers to indicate that it is the main body of the message in plain-
    text encoding.  The third section has headers indicating an attached
    file, along with its name and type.  All sections are separated by a
    boundary string, which cannot appear anywhere else in the message.

  Precondition:
    None

  Parameters:
    None

  Return Values:
    HTTP_IO_DONE - the message has been sent
    HTTP_IO_WAITING - the function is waiting for the SMTP process to complete
    HTTP_IO_NEED_DATA - data needed by this function has not yet arrived
  ***************************************************************************/
#if defined(TCPIP_STACK_USE_SMTP_CLIENT)
static HTTP_IO_RESULT HTTPPostEmail(HTTP_CONN_HANDLE connHandle)
{
    static uint8_t *ptrData;
    static uint8_t *szPort;
    static SMTP_POINTERS mySMTPClient;
    #if defined(TCPIP_STACK_USE_SSL_CLIENT)
    static uint8_t *szUseSSL;
    #endif
    uint16_t len, rem;
    uint8_t cName[8];
    uint8_t* httpDataBuff;
    TCP_SOCKET sktHTTP;

    #define SM_EMAIL_CLAIM_MODULE                (0u)
    #define SM_EMAIL_READ_PARAM_NAME            (1u)
    #define SM_EMAIL_READ_PARAM_VALUE            (2u)
    #define SM_EMAIL_PUT_IGNORED                (3u)
    #define SM_EMAIL_PUT_BODY                    (4u)
    #define SM_EMAIL_PUT_ATTACHMENT_HEADER        (5u)
    #define SM_EMAIL_PUT_ATTACHMENT_DATA_BTNS    (6u)
    #define SM_EMAIL_PUT_ATTACHMENT_DATA_LEDS    (7u)
    #define SM_EMAIL_PUT_ATTACHMENT_DATA_POT    (8u)
    #define SM_EMAIL_PUT_TERMINATOR                (9u)
    #define SM_EMAIL_FINISHING                    (10u)


    httpDataBuff = HTTPCurConnectionDataBufferGet(connHandle);
    sktHTTP = HTTPCurConnectionSocketGet(connHandle);
    switch(HTTPCurConnectionPostSmGet(connHandle))
    {
        case SM_EMAIL_CLAIM_MODULE:
            // Try to claim module
            if(SMTPBeginUsage())
            {// Module was claimed, so set up static parameters
                memset(&mySMTPClient, 0, sizeof(mySMTPClient));
                mySMTPClient.Subject = "Microchip TCP/IP Stack Status Update";
                mySMTPClient.From = "\"SMTP Service\" <mchpboard@picsaregood.com>";

                // The following two lines indicate to the receiving client that
                // this message has an attachment.  The boundary field *must not*
                // be included anywhere in the content of the message.  In real
                // applications it is typically a long random string.
                mySMTPClient.OtherHeaders = "MIME-version: 1.0\r\nContent-type: multipart/mixed; boundary=\"frontier\"\r\n";

                // Move our state machine forward
                ptrData = httpDataBuff;
                szPort = NULL;
                HTTPCurConnectionPostSmSet(connHandle, SM_EMAIL_READ_PARAM_NAME);
            }
            return HTTP_IO_WAITING;

        case SM_EMAIL_READ_PARAM_NAME:
            // Search for a parameter name in POST data
            if(HTTPReadPostName(connHandle, cName, sizeof(cName)) == HTTP_READ_INCOMPLETE)
                return HTTP_IO_NEED_DATA;

            // Try to match the name value
            if(!strcmp((char*)cName, (const char*)"server"))
            {// Read the server name
                mySMTPClient.Server = (char*)ptrData;
                HTTPCurConnectionPostSmSet(connHandle, SM_EMAIL_READ_PARAM_VALUE);
            }
            else if(!strcmp((char*)cName, (const char*)"port"))
            {// Read the server port
                szPort = ptrData;
                HTTPCurConnectionPostSmSet(connHandle, SM_EMAIL_READ_PARAM_VALUE);
            }
            #if defined(TCPIP_STACK_USE_SSL_CLIENT)
            else if(!strcmp((char*)cName, (const char*)"ssl"))
            {// Read the server port
                szUseSSL = ptrData;
                HTTPCurConnectionPostSmSet(connHandle, SM_EMAIL_READ_PARAM_VALUE);
            }
            #endif
            else if(!strcmp((char*)cName, (const char*)"user"))
            {// Read the user name
                mySMTPClient.Username = (char*)ptrData;
                HTTPCurConnectionPostSmSet(connHandle, SM_EMAIL_READ_PARAM_VALUE);
            }
            else if(!strcmp((char*)cName, (const char*)"pass"))
            {// Read the password
                mySMTPClient.Password = (char*)ptrData;
                HTTPCurConnectionPostSmSet(connHandle, SM_EMAIL_READ_PARAM_VALUE);
            }
            else if(!strcmp((char*)cName, (const char*)"to"))
            {// Read the To string
                mySMTPClient.To = (char*)ptrData;
                HTTPCurConnectionPostSmSet(connHandle, SM_EMAIL_READ_PARAM_VALUE);
            }
            else if(!strcmp((char*)cName, (const char*)"msg"))
            {// Done with headers, move on to the message
                // Delete paramters that are just null strings (no data from user) or illegal (ex: password without username)
                if(mySMTPClient.Server )
                    if(*mySMTPClient.Server == 0x00u)
                        mySMTPClient.Server = NULL;
                if(mySMTPClient.Username )
                    if(*mySMTPClient.Username == 0x00u)
                        mySMTPClient.Username = NULL;
                if(mySMTPClient.Password)
                    if((*mySMTPClient.Password == 0x00u) || (mySMTPClient.Username == NULL))
                        mySMTPClient.Password = NULL;

                // Decode server port string if it exists
                if(szPort)
                    if(*szPort)
                        mySMTPClient.ServerPort = (uint16_t)atol((char*)szPort);

                // Determine if SSL should be used
                #if defined(TCPIP_STACK_USE_SSL_CLIENT)
                if(szUseSSL)
                    if(*szUseSSL == '1')
                        mySMTPClient.UseSSL = true;
                #endif

                // Start sending the message
				SMTPSendMail(&mySMTPClient);
                HTTPCurConnectionPostSmSet(connHandle, SM_EMAIL_PUT_IGNORED);
                return HTTP_IO_WAITING;
            }
            else
            {// Don't know what we're receiving
                HTTPCurConnectionPostSmSet(connHandle, SM_EMAIL_READ_PARAM_VALUE);
            }

            // No break...continue to try reading the value

        case SM_EMAIL_READ_PARAM_VALUE:
            // Search for a parameter value in POST data
            rem = HTTP_MAX_DATA_LEN - (ptrData - httpDataBuff);
            if(HTTPReadPostValue(connHandle, ptrData, rem) == HTTP_READ_INCOMPLETE)
                return HTTP_IO_NEED_DATA;

            // Move past the data that was just read
            ptrData += strlen((char*)ptrData);
            if(ptrData < httpDataBuff + HTTP_MAX_DATA_LEN - 1)
                ptrData += 1;

            // Try reading the next parameter
            HTTPCurConnectionPostSmSet(connHandle, SM_EMAIL_READ_PARAM_NAME);
            return HTTP_IO_WAITING;

        case SM_EMAIL_PUT_IGNORED:
            // This section puts a message that is ignored by compatible clients.
            // This text will not display unless the receiving client is obselete
            // and does not understand the MIME structure.
            // The "--frontier" indicates the start of a section, then any
            // needed MIME headers follow, then two CRLF pairs, and then
            // the actual content (which will be the body text in the next state).

            // Check to see if a failure occured
            if(!SMTPIsBusy())
            {
                HTTPCurConnectionPostSmSet(connHandle, SM_EMAIL_FINISHING);
                return HTTP_IO_WAITING;
            }

            // See if we're ready to write data
            if(SMTPIsPutReady() < 90u)
                return HTTP_IO_WAITING;

            // Write the ignored text
            SMTPPutString("This is a multi-part message in MIME format.\r\n");
            SMTPPutString("--frontier\r\nContent-type: text/plain\r\n\r\n");
            SMTPFlush();

            // Move to the next state
            HTTPCurConnectionPostSmSet(connHandle, SM_EMAIL_PUT_BODY);

        case SM_EMAIL_PUT_BODY:
            // Write as much body text as is available from the TCP buffer
            // return HTTP_IO_NEED_DATA or HTTP_IO_WAITING
            // On completion, => PUT_ATTACHMENT_HEADER and continue

            // Check to see if a failure occurred
            if(!SMTPIsBusy())
            {
                HTTPCurConnectionPostSmSet(connHandle, SM_EMAIL_FINISHING);
                return HTTP_IO_WAITING;
            }

            // Loop as long as data remains to be read
            while(HTTPCurConnectionByteCountGet(connHandle))
            {
                // See if space is available to write
                len = SMTPIsPutReady();
                if(len == 0u)
                    return HTTP_IO_WAITING;

                // See if data is ready to be read
                rem = TCPIsGetReady(sktHTTP);
                if(rem == 0u)
                    return HTTP_IO_NEED_DATA;

                // Only write as much as we can handle
                if(len > rem)
                    len = rem;
                if(len > HTTP_MAX_DATA_LEN - 2)
                    len = HTTP_MAX_DATA_LEN - 2;

                // Read the data from HTTP POST buffer and send it to SMTP
                HTTPCurConnectionByteCountDec(connHandle, TCPGetArray(sktHTTP, httpDataBuff, len));
                httpDataBuff[len] = '\0';
                HTTPURLDecode(httpDataBuff);
                SMTPPutString((char*)httpDataBuff);
                SMTPFlush();
            }

            // We're done with the POST data, so continue
            HTTPCurConnectionPostSmSet(connHandle, SM_EMAIL_PUT_ATTACHMENT_HEADER);

        case SM_EMAIL_PUT_ATTACHMENT_HEADER:
            // This section writes the attachment to the message.
            // This portion generally will not display in the reader, but
            // will be downloadable to the local machine.  Use caution
            // when selecting the content-type and file name, as certain
            // types and extensions are blocked by virus filters.

            // The same structure as the message body is used.
            // Any attachment must not include high-bit ASCII characters or
            // binary data.  If binary data is to be sent, the data should
            // be encoded using Base64 and a MIME header should be added:
            // Content-transfer-encoding: base64

            // Check to see if a failure occurred
            if(!SMTPIsBusy())
            {
                HTTPCurConnectionPostSmSet(connHandle, SM_EMAIL_FINISHING);
                return HTTP_IO_WAITING;
            }

            // See if we're ready to write data
            if(SMTPIsPutReady() < 100u)
                return HTTP_IO_WAITING;

            // Write the attachment header
            SMTPPutString("\r\n--frontier\r\nContent-type: text/csv\r\nContent-Disposition: attachment; filename=\"status.csv\"\r\n\r\n");
            SMTPFlush();

            // Move to the next state
            HTTPCurConnectionPostSmSet(connHandle, SM_EMAIL_PUT_ATTACHMENT_DATA_BTNS);

        case SM_EMAIL_PUT_ATTACHMENT_DATA_BTNS:
            // The following states output the system status as a CSV file.

            // Check to see if a failure occurred
            if(!SMTPIsBusy())
            {
                HTTPCurConnectionPostSmSet(connHandle, SM_EMAIL_FINISHING);
                return HTTP_IO_WAITING;
            }

            // See if we're ready to write data
            if(SMTPIsPutReady() < 36u)
                return HTTP_IO_WAITING;

            // Write the header and button strings
            SMTPPutString("SYSTEM STATUS\r\n");
            SMTPPutString("Buttons:,");
            SMTPPut(BUTTON0_IO + '0');
            SMTPPut(',');
            SMTPPut(BUTTON1_IO + '0');
            SMTPPut(',');
            SMTPPut(BUTTON2_IO + '0');
            SMTPPut(',');
            SMTPPut(BUTTON3_IO + '0');
            SMTPPut('\r');
            SMTPPut('\n');
            SMTPFlush();

            // Move to the next state
            HTTPCurConnectionPostSmSet(connHandle, SM_EMAIL_PUT_ATTACHMENT_DATA_LEDS);

        case SM_EMAIL_PUT_ATTACHMENT_DATA_LEDS:
            // Check to see if a failure occurred
            if(!SMTPIsBusy())
            {
                HTTPCurConnectionPostSmSet(connHandle, SM_EMAIL_FINISHING);
                return HTTP_IO_WAITING;
            }

            // See if we're ready to write data
            if(SMTPIsPutReady() < 30u)
                return HTTP_IO_WAITING;

            // Write the header and button strings
            SMTPPutString("LEDs:,");
            SMTPPut(LED0_IO + '0');
            SMTPPut(',');
            SMTPPut(LED1_IO + '0');
            SMTPPut(',');
            SMTPPut(LED2_IO + '0');
            SMTPPut(',');
            SMTPPut(LED3_IO + '0');
            SMTPPut(',');
            SMTPPut(LED4_IO + '0');
            SMTPPut(',');
            SMTPPut(LED5_IO + '0');
            SMTPPut(',');
            SMTPPut(LED6_IO + '0');
            SMTPPut(',');
            SMTPPut(LED7_IO + '0');
            SMTPPut('\r');
            SMTPPut('\n');
            SMTPFlush();

            // Move to the next state
            HTTPCurConnectionPostSmSet(connHandle, SM_EMAIL_PUT_ATTACHMENT_DATA_POT);

        case SM_EMAIL_PUT_ATTACHMENT_DATA_POT:
            // Check to see if a failure occurred
            if(!SMTPIsBusy())
            {
                HTTPCurConnectionPostSmSet(connHandle, SM_EMAIL_FINISHING);
                return HTTP_IO_WAITING;
            }

            // See if we're ready to write data
            if(SMTPIsPutReady() < 16u)
                return HTTP_IO_WAITING;

            // Do the A/D conversion
            len = (uint16_t)ADC1BUF0;
            uitoa(len, (uint8_t*)&httpDataBuff[1]);

            // Write the header and button strings
            SMTPPutString("Pot:,");
            SMTPPutString((char*)(httpDataBuff+1));
            SMTPPut('\r');
            SMTPPut('\n');
            SMTPFlush();

            // Move to the next state
            HTTPCurConnectionPostSmSet(connHandle, SM_EMAIL_PUT_TERMINATOR);

        case SM_EMAIL_PUT_TERMINATOR:
            // This section finishes the message
            // This consists of two dashes, the boundary, and two more dashes
            // on a single line, followed by a CRLF pair to terminate the message.

            // Check to see if a failure occured
            if(!SMTPIsBusy())
            {
                HTTPCurConnectionPostSmSet(connHandle, SM_EMAIL_FINISHING);
                return HTTP_IO_WAITING;
            }

            // See if we're ready to write data
            if(SMTPIsPutReady() < 16u)
                return HTTP_IO_WAITING;

            // Write the ignored text
            SMTPPutString("--frontier--\r\n");
            SMTPPutDone();
            SMTPFlush();

            // Move to the next state
            HTTPCurConnectionPostSmSet(connHandle, SM_EMAIL_FINISHING);

        case SM_EMAIL_FINISHING:
            // Wait for status
            if(!SMTPIsBusy())
            {
                // Release the module and check success
                // Redirect the user based on the result
                if(SMTPEndUsage() == SMTP_SUCCESS)
                    lastSuccess = true;
                else
                    lastFailure = true;

                // Redirect to the page
                strcpy((char*)httpDataBuff, "/email/index.htm");
                HTTPCurConnectionStatusSet(connHandle, HTTP_REDIRECT);
                return HTTP_IO_DONE;
            }

            return HTTP_IO_WAITING;
    }

    return HTTP_IO_DONE;
}
#endif    // #if defined(TCPIP_STACK_USE_SMTP_CLIENT)

/****************************************************************************
  Function:
    HTTP_IO_RESULT HTTPPostDDNSConfig(HTTP_CONN_HANDLE connHandle)
    
  Summary:
    Parsing and collecting http data received from http form.

  Description:
    This routine will be excuted every time the Dynamic DNS Client
    configuration form is submitted.  The http data is received 
    as a string of the variables seperated by '&' characters in the TCP RX
    buffer.  This data is parsed to read the required configuration values, 
    and those values are populated to the global array (DDNSData) reserved 
    for this purpose.  As the data is read, DDNSPointers is also populated
    so that the dynamic DNS client can execute with the new parameters.
    
  Precondition:
     cur HTTP connection is loaded.

  Parameters:
    None.

  Return Values:
    HTTP_IO_DONE         -  Finished with procedure
    HTTP_IO_NEED_DATA    -  More data needed to continue, call again later
    HTTP_IO_WAITING     -  Waiting for asynchronous process to complete,
                            call again later
  ***************************************************************************/
#if defined(TCPIP_STACK_USE_DYNAMICDNS_CLIENT)
static HTTP_IO_RESULT HTTPPostDDNSConfig(HTTP_CONN_HANDLE connHandle)
{
    static uint8_t *ptrDDNS;
    uint8_t* httpDataBuff;
    uint8_t    smPost;

    #define SM_DDNS_START            (0u)
    #define SM_DDNS_READ_NAME        (1u)
    #define SM_DDNS_READ_VALUE        (2u)
    #define SM_DDNS_READ_SERVICE    (3u)
    #define SM_DDNS_DONE            (4u)

    #define DDNS_SPACE_REMAINING                (sizeof(DDNSData) - (ptrDDNS - DDNSData))

    httpDataBuff = HTTPCurConnectionDataBufferGet(connHandle);
    smPost = HTTPCurConnectionPostSmGet(connHandle);
    switch(smPost)
    {
        // Sets defaults for the system
        case SM_DDNS_START:
            ptrDDNS = DDNSData;
            DDNSSetService(0);
            DDNSClient.Host.szROM = NULL;
            DDNSClient.Username.szROM = NULL;
            DDNSClient.Password.szROM = NULL;
            DDNSClient.ROMPointers.Host = 0;
            DDNSClient.ROMPointers.Username = 0;
            DDNSClient.ROMPointers.Password = 0;
            HTTPCurConnectionPostSmSet(connHandle, ++smPost);

        // Searches out names and handles them as they arrive
        case SM_DDNS_READ_NAME:
            // If all parameters have been read, end
            if(HTTPCurConnectionByteCountGet(connHandle) == 0u)
            {
                HTTPCurConnectionPostSmSet(connHandle, SM_DDNS_DONE);
                break;
            }

            // Read a name
            if(HTTPReadPostName(connHandle, httpDataBuff, HTTP_MAX_DATA_LEN) == HTTP_READ_INCOMPLETE)
                return HTTP_IO_NEED_DATA;

            if(!strcmp((char *)httpDataBuff, (const char*)"service"))
            {
                // Reading the service (numeric)
                HTTPCurConnectionPostSmSet(connHandle, SM_DDNS_READ_SERVICE);
                break;
            }
            else if(!strcmp((char *)httpDataBuff, (const char*)"user"))
                DDNSClient.Username.szRAM = ptrDDNS;
            else if(!strcmp((char *)httpDataBuff, (const char*)"pass"))
                DDNSClient.Password.szRAM = ptrDDNS;
            else if(!strcmp((char *)httpDataBuff, (const char*)"host"))
                DDNSClient.Host.szRAM = ptrDDNS;

            // Move to reading the value for user/pass/host
            HTTPCurConnectionPostSmSet(connHandle, ++smPost);

        // Reads in values and assigns them to the DDNS RAM
        case SM_DDNS_READ_VALUE:
            // Read a name
            if(HTTPReadPostValue(connHandle, ptrDDNS, DDNS_SPACE_REMAINING) == HTTP_READ_INCOMPLETE)
                return HTTP_IO_NEED_DATA;

            // Move past the data that was just read
            ptrDDNS += strlen((char*)ptrDDNS);
            if(ptrDDNS < DDNSData + sizeof(DDNSData) - 1)
                ptrDDNS += 1;

            // Return to reading names
            HTTPCurConnectionPostSmSet(connHandle, SM_DDNS_READ_NAME);
            break;

        // Reads in a service ID
        case SM_DDNS_READ_SERVICE:
            // Read the integer id
            if(HTTPReadPostValue(connHandle, httpDataBuff, HTTP_MAX_DATA_LEN) == HTTP_READ_INCOMPLETE)
                return HTTP_IO_NEED_DATA;

            // Convert to a service ID
            DDNSSetService((uint8_t)atol((char*)httpDataBuff));

            // Return to reading names
            HTTPCurConnectionPostSmSet(connHandle, SM_DDNS_READ_NAME);
            break;

        // Sets up the DDNS client for an update
        case SM_DDNS_DONE:
            // Since user name and password changed, force an update immediately
            DDNSForceUpdate();

            // Redirect to prevent POST errors
            lastSuccess = true;
            strcpy((char*)httpDataBuff, "/dyndns/index.htm");
            HTTPCurConnectionStatusSet(connHandle, HTTP_REDIRECT);
            return HTTP_IO_DONE;
    }

    return HTTP_IO_WAITING;        // Assume we're waiting to process more data
}
#endif    // #if defined(TCPIP_STACK_USE_DYNAMICDNS_CLIENT)

#endif //(use_post)


/****************************************************************************
  Section:
    Dynamic Variable Callback Functions
  ***************************************************************************/

/*****************************************************************************
  Function:
    void HTTPPrint_varname(HTTP_CONN_HANDLE connHandle)

  Internal:
      See documentation in the TCP/IP Stack API or HTTP2.h for details.
  ***************************************************************************/

void HTTPPrint_builddate(HTTP_CONN_HANDLE connHandle)
{
    TCP_SOCKET sktHTTP;
    sktHTTP = HTTPCurConnectionSocketGet(connHandle);

    HTTPCurConnectionCallbackPosSet(connHandle, 0x01);
    if(TCPIsPutReady(sktHTTP) < strlen((const char*)__DATE__" "__TIME__))
        return;
 
    HTTPCurConnectionCallbackPosSet(connHandle, 0x00);
    TCPPutString(sktHTTP, (const void*)__DATE__" "__TIME__);
}

void HTTPPrint_version(HTTP_CONN_HANDLE connHandle)
{
    TCPPutString(HTTPCurConnectionSocketGet(connHandle), (const void*)TCPIP_STACK_VERSION);
}




const uint8_t HTML_UP_ARROW[] = "up";
const uint8_t HTML_DOWN_ARROW[] = "dn";

void HTTPPrint_btn(HTTP_CONN_HANDLE connHandle, uint16_t num)
{
    // Determine which button
    switch(num)
    {
        case 0:
            num = BUTTON0_IO;
            break;
        case 1:
            num = BUTTON1_IO;
            break;
        case 2:
            num = BUTTON2_IO;
            break;
        case 3:
            num = BUTTON3_IO;
            break;
        default:
            num = 0;
    }

    // Print the output
    TCPPutString(HTTPCurConnectionSocketGet(connHandle), (num?HTML_UP_ARROW:HTML_DOWN_ARROW));
}

void HTTPPrint_led(HTTP_CONN_HANDLE connHandle, uint16_t num)
{
    // Determine which LED
    switch(num)
    {
        case 0:
            num = LED0_IO;
            break;
        case 1:
            num = LED1_IO;
            break;
        case 2:
            num = LED2_IO;
            break;
        case 3:
            num = LED3_IO;
            break;
        case 4:
            num = LED4_IO;
            break;
        case 5:
            num = LED5_IO;
            break;
        case 6:
            num = LED6_IO;
            break;
        case 7:
            num = LED7_IO;
            break;

        default:
            num = 0;
    }

    // Print the output
    TCPPut(HTTPCurConnectionSocketGet(connHandle), (num?'1':'0'));
}

void HTTPPrint_ledSelected(HTTP_CONN_HANDLE connHandle, uint16_t num, uint16_t state)
{
    // Determine which LED to check
    switch(num)
    {
        case 0:
            num = LED0_IO;
            break;
        case 1:
            num = LED1_IO;
            break;
        case 2:
            num = LED2_IO;
            break;
        case 3:
            num = LED3_IO;
            break;
        case 4:
            num = LED4_IO;
            break;
        case 5:
            num = LED5_IO;
            break;
        case 6:
            num = LED6_IO;
            break;
        case 7:
            num = LED7_IO;
            break;

        default:
            num = 0;
    }

    // Print output if true and ON or if false and OFF
    if((state && num) || (!state && !num))
        TCPPutString(HTTPCurConnectionSocketGet(connHandle), (const uint8_t*)"SELECTED");
}

void HTTPPrint_pot(HTTP_CONN_HANDLE connHandle)
{
    uint8_t AN0String[8];
    uint16_t ADval;

    ADval = (uint16_t)ADC1BUF0;
    //ADval *= (uint16_t)10;
    //ADval /= (uint16_t)102;
    uitoa(ADval, (uint8_t*)AN0String);

       TCPPutString(HTTPCurConnectionSocketGet(connHandle), AN0String);
}

void HTTPPrint_lcdtext(HTTP_CONN_HANDLE connHandle)
{
// This function is no longer supported.
#if 0
    TCP_SOCKET sktHTTP = HTTPCurConnectionSocketGet(connHandle);

    // If just starting, set callbackPos
    uint16_t len;
    uint32_t callbackPos;

    // Determine how many bytes we can write
    len = TCPIsPutReady(sktHTTP);
    callbackPos = HTTPCurConnectionCallbackPosGet(connHandle);

    if(callbackPos == 0u)
        callbackPos = 32;

    // Write a byte at a time while we still can
    // It may take up to 12 bytes to write a character
    // (spaces and newlines are longer)
    while(len > 12u && callbackPos)
    {
        // After 16 bytes write a newline
        if(callbackPos == 16u)
            len -= TCPPutArray(sktHTTP, (const uint8_t*)"<br />", 6);

        if(LCDText[32-callbackPos] == ' ' || LCDText[32-callbackPos] == '\0')
            len -= TCPPutArray(sktHTTP, (const uint8_t*)"&nbsp;", 6);
        else
            len -= TCPPut(sktHTTP, LCDText[32-callbackPos]);

        callbackPos--;
    }
    HTTPCurConnectionCallbackPosSet(connHandle, callbackPos);
#endif
}

void HTTPPrint_hellomsg(HTTP_CONN_HANDLE connHandle)
{
    const uint8_t *ptr;
    TCP_SOCKET sktHTTP;

    ptr = HTTPGetArg(HTTPCurConnectionDataBufferGet(connHandle), (const uint8_t*)"name");

    sktHTTP = HTTPCurConnectionSocketGet(connHandle);
    // We omit checking for space because this is the only data being written
    if(ptr != NULL)
    {
        TCPPutString(sktHTTP, (const uint8_t*)"Hello, ");
        TCPPutString(sktHTTP, ptr);
    }

}

void HTTPPrint_cookiename(HTTP_CONN_HANDLE connHandle)
{
    const uint8_t *ptr;
    TCP_SOCKET sktHTTP;

    ptr = HTTPGetArg(HTTPCurConnectionDataBufferGet(connHandle), (const uint8_t*)"name");

    sktHTTP = HTTPCurConnectionSocketGet(connHandle);
    if(ptr)
        TCPPutString(sktHTTP, ptr);
    else
        TCPPutString(sktHTTP, (const uint8_t*)"not set");

}

void HTTPPrint_uploadedmd5(HTTP_CONN_HANDLE connHandle)
{
    uint8_t i;
    uint8_t* httpDataBuff;
    TCP_SOCKET sktHTTP;

    // Set a flag to indicate not finished
    HTTPCurConnectionCallbackPosSet(connHandle, 1);

    sktHTTP = HTTPCurConnectionSocketGet(connHandle);
    // Make sure there's enough output space
    if(TCPIsPutReady(sktHTTP) < 32u + 37u + 5u)
        return;

    // Check for flag set in HTTPPostMD5
#if defined(HTTP_APP_USE_MD5)
    if(HTTPCurConnectionPostSmGet(connHandle) != SM_MD5_POST_COMPLETE)
#endif
    {// No file uploaded, so just return
        TCPPutString(sktHTTP, (const uint8_t*)"<b>Upload a File</b>");
        HTTPCurConnectionCallbackPosSet(connHandle, 0);
        return;
    }

    TCPPutString(sktHTTP, (const uint8_t*)"<b>Uploaded File's MD5 was:</b><br />");
    httpDataBuff = HTTPCurConnectionDataBufferGet(connHandle);

    // Write a byte of the md5 sum at a time
    for(i = 0; i < 16u; i++)
    {
        TCPPut(sktHTTP, btohexa_high(httpDataBuff[i]));
        TCPPut(sktHTTP, btohexa_low(httpDataBuff[i]));
        if((i & 0x03) == 3u)
            TCPPut(sktHTTP, ' ');
    }

    HTTPCurConnectionCallbackPosSet(connHandle, 0x00);
}


void HTTPPrintIP(HTTP_CONN_HANDLE connHandle, IPV4_ADDR ip)
{
    uint8_t digits[4];
    uint8_t i;
    TCP_SOCKET sktHTTP;

    sktHTTP = HTTPCurConnectionSocketGet(connHandle);
    for(i = 0; i < 4u; i++)
    {
        if(i)
            TCPPut(sktHTTP, '.');
        uitoa(ip.v[i], digits);
        TCPPutString(sktHTTP, digits);
    }
}

void HTTPPrint_config_hostname(HTTP_CONN_HANDLE connHandle)
{
    TCP_SOCKET sktHTTP;
    sktHTTP = HTTPCurConnectionSocketGet(connHandle);
    TCPPutString(sktHTTP, (uint8_t*)TCPIP_STACK_NetBIOSName(TCPSocketGetNet(sktHTTP)));
}

void HTTPPrint_config_dhcpchecked(HTTP_CONN_HANDLE connHandle)
{
    TCP_SOCKET sktHTTP;
    sktHTTP = HTTPCurConnectionSocketGet(connHandle);
    if(DHCPIsEnabled(TCPSocketGetNet(sktHTTP)))
    {
        TCPPutString(sktHTTP, (const uint8_t*)"checked");
    }
}

void HTTPPrint_config_ip(HTTP_CONN_HANDLE connHandle)
{
    TCP_SOCKET sktHTTP;
    sktHTTP = HTTPCurConnectionSocketGet(connHandle);
    TCPIP_NET_HANDLE netH = TCPSocketGetNet(sktHTTP);
    IPV4_ADDR ipAddress;    
    
    ipAddress.Val = TCPIP_STACK_NetAddress(netH);    
    HTTPPrintIP(connHandle, ipAddress);
}

void HTTPPrint_config_gw(HTTP_CONN_HANDLE connHandle)
{
    TCP_SOCKET sktHTTP;
    sktHTTP = HTTPCurConnectionSocketGet(connHandle);
    TCPIP_NET_HANDLE netH = TCPSocketGetNet(sktHTTP);
    IPV4_ADDR gwAddress;    

    gwAddress.Val = TCPIP_STACK_NetGatewayAddress(netH);    
    
    HTTPPrintIP(connHandle, gwAddress);
}

void HTTPPrint_config_subnet(HTTP_CONN_HANDLE connHandle)
{
    TCP_SOCKET sktHTTP;
    sktHTTP = HTTPCurConnectionSocketGet(connHandle);
    TCPIP_NET_HANDLE netH = TCPSocketGetNet(sktHTTP);
    IPV4_ADDR ipMask;    

    ipMask.Val = TCPIP_STACK_NetMask(netH);    
    HTTPPrintIP(connHandle, ipMask);
}

void HTTPPrint_config_dns1(HTTP_CONN_HANDLE connHandle)
{
    TCP_SOCKET sktHTTP;
    sktHTTP = HTTPCurConnectionSocketGet(connHandle);
    TCPIP_NET_HANDLE netH = TCPSocketGetNet(sktHTTP);
    IPV4_ADDR priDnsAddr;    

    priDnsAddr.Val = TCPIP_STACK_NetPriDNSAddress(netH);    

    HTTPPrintIP(connHandle, priDnsAddr);
}

void HTTPPrint_config_dns2(HTTP_CONN_HANDLE connHandle)
{
    TCP_SOCKET sktHTTP;
    sktHTTP = HTTPCurConnectionSocketGet(connHandle);
    TCPIP_NET_HANDLE netH = TCPSocketGetNet(sktHTTP);
    IPV4_ADDR secondDnsAddr;    

    secondDnsAddr.Val = TCPIP_STACK_NetSecondDNSAddress(netH);    

    HTTPPrintIP(connHandle, secondDnsAddr);
}

void HTTPPrint_config_mac(HTTP_CONN_HANDLE connHandle)
{
    uint8_t i;
    TCP_SOCKET sktHTTP;
    TCPIP_NET_HANDLE hNet; 
    const uint8_t* pMacAdd;
    sktHTTP = HTTPCurConnectionSocketGet(connHandle);

    if(TCPIsPutReady(sktHTTP) < 18u)
    {//need 17 bytes to write a MAC
        HTTPCurConnectionCallbackPosSet(connHandle, 0x01);
        return;
    }

    hNet = TCPSocketGetNet(sktHTTP); 
    pMacAdd = TCPIP_STACK_NetMacAddress(hNet);
    // Write each byte
    for(i = 0; i < 6u; i++)
    {
        if(i)
            TCPPut(sktHTTP, ':');
        TCPPut(sktHTTP, btohexa_high(pMacAdd[i]));
        TCPPut(sktHTTP, btohexa_low(pMacAdd[i]));
    }

    // Indicate that we're done
    HTTPCurConnectionCallbackPosSet(connHandle, 0x00);
    return;
}


// SNMP Read communities configuration page
void HTTPPrint_read_comm(HTTP_CONN_HANDLE connHandle, uint16_t num)
{
    #if defined(TCPIP_STACK_USE_SNMP_SERVER)
    TCP_SOCKET sktHTTP;
    sktHTTP = HTTPCurConnectionSocketGet(connHandle);
    // Ensure no one tries to read illegal memory addresses by specifying
    // illegal num values.
    if(num >= SNMP_MAX_COMMUNITY_SUPPORT)
        return;

    // Send proper string
    TCPPutString(sktHTTP, ((SNMP_NET_CONFIG*)TCPSocketGetNet(sktHTTP))->readCommunity[num]);
    #endif
}

// SNMP Write communities configuration page
void HTTPPrint_write_comm(HTTP_CONN_HANDLE connHandle, uint16_t num)
{
    #if defined(TCPIP_STACK_USE_SNMP_SERVER)
    TCP_SOCKET sktHTTP;
    sktHTTP = HTTPCurConnectionSocketGet(connHandle);
    // Ensure no one tries to read illegal memory addresses by specifying
    // illegal num values.
    if(num >= SNMP_MAX_COMMUNITY_SUPPORT)
        return;

    // Send proper string
    TCPPutString(sktHTTP, ((SNMP_NET_CONFIG*)TCPSocketGetNet(sktHTTP))->writeCommunity[num]);
    #endif
}


void HTTPPrint_reboot(HTTP_CONN_HANDLE connHandle)
{
    // This is not so much a print function, but causes the board to reboot
    // when the configuration is changed.  If called via an AJAX call, this
    // will gracefully reboot the system and bring it back online immediately
    SYS_Reboot();
}

void HTTPPrint_rebootaddr(HTTP_CONN_HANDLE connHandle)
{// This is the expected address of the board upon rebooting
    TCPPutString(HTTPCurConnectionSocketGet(connHandle), HTTPCurConnectionDataBufferGet(connHandle));
}

void HTTPPrint_ddns_user(HTTP_CONN_HANDLE connHandle)
{
    #if defined(TCPIP_STACK_USE_DYNAMICDNS_CLIENT)
    uint32_t callbackPos;

    if(DDNSClient.ROMPointers.Username || !DDNSClient.Username.szRAM)
        return;

    callbackPos = HTTPCurConnectionCallbackPosGet(connHandle);
    if(callbackPos == 0x00u)
        callbackPos = (uint32_t)DDNSClient.Username.szRAM;
    callbackPos = (uint32_t)TCPPutString(HTTPCurConnectionSocketGet(connHandle), (uint8_t*)callbackPos);
    if(*(uint8_t*)callbackPos == '\0')
        callbackPos = 0x00;
    HTTPCurConnectionCallbackPosSet(connHandle, callbackPos);
    #endif
}

void HTTPPrint_ddns_pass(HTTP_CONN_HANDLE connHandle)
{
    #if defined(TCPIP_STACK_USE_DYNAMICDNS_CLIENT)
    uint32_t callbackPos;
    if(DDNSClient.ROMPointers.Password || !DDNSClient.Password.szRAM)
        return;

    callbackPos = HTTPCurConnectionCallbackPosGet(connHandle);

    if(callbackPos == 0x00u)
        callbackPos = (uint32_t)DDNSClient.Password.szRAM;
    callbackPos = (uint32_t)TCPPutString(HTTPCurConnectionSocketGet(connHandle), (uint8_t*)callbackPos);
    if(*(uint8_t*)callbackPos == '\0')
        callbackPos = 0x00;
    HTTPCurConnectionCallbackPosSet(connHandle, callbackPos);
    #endif
}

void HTTPPrint_ddns_host(HTTP_CONN_HANDLE connHandle)
{
    #if defined(TCPIP_STACK_USE_DYNAMICDNS_CLIENT)
    uint32_t callbackPos;
    if(DDNSClient.ROMPointers.Host || !DDNSClient.Host.szRAM)
        return;
    callbackPos = HTTPCurConnectionCallbackPosGet(connHandle);
    if(callbackPos == 0x00u)
        callbackPos = (uint32_t)DDNSClient.Host.szRAM;
    callbackPos = (uint32_t)TCPPutString(HTTPCurConnectionSocketGet(connHandle), (uint8_t*)callbackPos);
    if(*(uint8_t*)callbackPos == '\0')
        callbackPos = 0x00;
    HTTPCurConnectionCallbackPosSet(connHandle, callbackPos);
    #endif
}

extern const char * const ddnsServiceHosts[];
void HTTPPrint_ddns_service(HTTP_CONN_HANDLE connHandle, uint16_t i)
{
    #if defined(TCPIP_STACK_USE_DYNAMICDNS_CLIENT)
    if(!DDNSClient.ROMPointers.UpdateServer || !DDNSClient.UpdateServer.szROM)
        return;
    if((const char*)DDNSClient.UpdateServer.szROM == ddnsServiceHosts[i])
        TCPPutString(HTTPCurConnectionSocketGet(connHandle), (const uint8_t*)"selected");
    #endif
}


void HTTPPrint_ddns_status(HTTP_CONN_HANDLE connHandle)
{
    TCP_SOCKET sktHTTP;
    sktHTTP = HTTPCurConnectionSocketGet(connHandle);

    #if defined(TCPIP_STACK_USE_DYNAMICDNS_CLIENT)
    DDNS_STATUS s;
    s = DDNSGetLastStatus();
    if(s == DDNS_STATUS_GOOD || s == DDNS_STATUS_UNCHANGED || s == DDNS_STATUS_NOCHG)
        TCPPutString(sktHTTP, (const uint8_t*)"ok");
    else if(s == DDNS_STATUS_UNKNOWN)
        TCPPutString(sktHTTP, (const uint8_t*)"unk");
    else
        TCPPutString(sktHTTP, (const uint8_t*)"fail");
    #else
    TCPPutString(sktHTTP, (const uint8_t*)"fail");
    #endif
}

void HTTPPrint_ddns_status_msg(HTTP_CONN_HANDLE connHandle)
{
    TCP_SOCKET sktHTTP;
    sktHTTP = HTTPCurConnectionSocketGet(connHandle);
    if(TCPIsPutReady(sktHTTP) < 75u)
    {
        HTTPCurConnectionCallbackPosSet(connHandle, 0x01);
        return;
    }

    #if defined(TCPIP_STACK_USE_DYNAMICDNS_CLIENT)
    switch(DDNSGetLastStatus())
    {
        case DDNS_STATUS_GOOD:
        case DDNS_STATUS_NOCHG:
            TCPPutString(sktHTTP, (const uint8_t*)"The last update was successful.");
            break;
        case DDNS_STATUS_UNCHANGED:
            TCPPutString(sktHTTP, (const uint8_t*)"The IP has not changed since the last update.");
            break;
        case DDNS_STATUS_UPDATE_ERROR:
        case DDNS_STATUS_CHECKIP_ERROR:
            TCPPutString(sktHTTP, (const uint8_t*)"Could not communicate with DDNS server.");
            break;
        case DDNS_STATUS_INVALID:
            TCPPutString(sktHTTP, (const uint8_t*)"The current configuration is not valid.");
            break;
        case DDNS_STATUS_UNKNOWN:
            TCPPutString(sktHTTP, (const uint8_t*)"The Dynamic DNS client is pending an update.");
            break;
        default:
            TCPPutString(sktHTTP, (const uint8_t*)"An error occurred during the update.<br />The DDNS Client is suspended.");
            break;
    }
    #else
    TCPPutString(sktHTTP, (const uint8_t*)"The Dynamic DNS Client is not enabled.");
    #endif

    HTTPCurConnectionCallbackPosSet(connHandle, 0);
}

void HTTPPrint_smtps_en(HTTP_CONN_HANDLE connHandle)
{
    TCP_SOCKET sktHTTP;
    sktHTTP = HTTPCurConnectionSocketGet(connHandle);
    #if defined(TCPIP_STACK_USE_SSL_CLIENT)
        TCPPutString(sktHTTP, (const uint8_t*)"inline");
    #else
        TCPPutString(sktHTTP, (const uint8_t*)"none");
    #endif
}

void HTTPPrint_snmp_en(HTTP_CONN_HANDLE connHandle)
{
    TCP_SOCKET sktHTTP;
    sktHTTP = HTTPCurConnectionSocketGet(connHandle);
    #if defined(TCPIP_STACK_USE_SNMP_SERVER)
        TCPPutString(sktHTTP, (const uint8_t*)"none");
    #else
        TCPPutString(sktHTTP, (const uint8_t*)"block");
    #endif
}

void HTTPPrint_status_ok(HTTP_CONN_HANDLE connHandle)
{
    TCP_SOCKET sktHTTP;
    sktHTTP = HTTPCurConnectionSocketGet(connHandle);
    if(lastSuccess)
        TCPPutString(sktHTTP, (const uint8_t*)"block");
    else
        TCPPutString(sktHTTP, (const uint8_t*)"none");
    lastSuccess = false;
}

void HTTPPrint_status_fail(HTTP_CONN_HANDLE connHandle)
{
    TCP_SOCKET sktHTTP;
    sktHTTP = HTTPCurConnectionSocketGet(connHandle);
    if(lastFailure)
        TCPPutString(sktHTTP, (const uint8_t*)"block");
    else
        TCPPutString(sktHTTP, (const uint8_t*)"none");
    lastFailure = false;
}
void HTTPPrint_scan(HTTP_CONN_HANDLE connHandle)
{
    uint8_t scanInProgressString[4];
    TCP_SOCKET sktHTTP;
    sktHTTP = HTTPCurConnectionSocketGet(connHandle);
    uitoa(IS_SCAN_IN_PROGRESS(g_ScanCtx.scanState), scanInProgressString);
    TCPPutString(sktHTTP, scanInProgressString);
}

void HTTPPrint_fwver(HTTP_CONN_HANDLE connHandle)
{
    static bool firstTime = true;
    static t_wfDeviceInfo deviceInfo;
    uint8_t fwVerString[8];
    TCP_SOCKET sktHTTP;
    sktHTTP = HTTPCurConnectionSocketGet(connHandle);
    if (firstTime)
    {
        firstTime = false;
        WF_GetDeviceInfo(&deviceInfo);  // only call this once, not continually
    }    
    
    uitoa((deviceInfo.romVersion << 8) | deviceInfo.patchVersion, fwVerString);
    TCPPutString(sktHTTP, fwVerString);
}

void HTTPPrint_ssid(HTTP_CONN_HANDLE connHandle)
{
    static bool firstTime = true;
    static uint8_t ssidString[33];
    static uint8_t ssidLength;
    TCP_SOCKET sktHTTP;
    sktHTTP = HTTPCurConnectionSocketGet(connHandle);
    // we don't need to check the connection state as the only way this function
    // is called is from the webserver.  if the webserver is requesting this,
    // then you can infer that we should be connected to the network
    if (firstTime)
    {
        firstTime = false;
        WF_SsidGet(ssidString, &ssidLength);
    }
    TCPPutArray(sktHTTP, ssidString, ssidLength);
}    

void HTTPPrint_bssCount(HTTP_CONN_HANDLE connHandle)
{
    uint8_t bssCountString[4];
    TCP_SOCKET sktHTTP;
    sktHTTP = HTTPCurConnectionSocketGet(connHandle);
    uitoa(SCANCXT.numScanResults, bssCountString);
    TCPPutString(sktHTTP, bssCountString);
}

void HTTPPrint_valid(HTTP_CONN_HANDLE connHandle)
{
    uint8_t bssDescIsValidString[4];
    TCP_SOCKET sktHTTP;
    sktHTTP = HTTPCurConnectionSocketGet(connHandle);
    uitoa((uint8_t)bssDescIsValid, bssDescIsValidString);
    TCPPutString(sktHTTP, bssDescIsValidString);
}    

void HTTPPrint_name(HTTP_CONN_HANDLE connHandle)
{
    TCP_SOCKET sktHTTP;
    sktHTTP = HTTPCurConnectionSocketGet(connHandle);

    if (bssDescIsValid)
    {
        if(strlen((const char*)bssDesc.ssid)<32)
            TCPPutString(sktHTTP, bssDesc.ssid);
		else
		{
            uint8_t buf_tmp[33];
            int i;
            for(i=0;i<32;i++) buf_tmp[i] = bssDesc.ssid[i];
            buf_tmp[32] = 0;
            TCPPutString(sktHTTP, buf_tmp);
		}
    }
    else
    {
        TCPPutString(sktHTTP, (const uint8_t *)"0");
    }        

}    

void HTTPPrint_privacy(HTTP_CONN_HANDLE connHandle)
{
    TCP_SOCKET sktHTTP;
    sktHTTP = HTTPCurConnectionSocketGet(connHandle);

    uint8_t security = (bssDesc.apConfig & 0xd0) >> 4;
    uint8_t secString[4];
    
    if (bssDescIsValid)
    {
        uitoa(security, secString);
        TCPPutString(sktHTTP, secString);
    }
    else
    {
        TCPPutString(sktHTTP, (const uint8_t*)"0");
    }    
}    

void HTTPPrint_wlan(HTTP_CONN_HANDLE connHandle)
{
    TCP_SOCKET sktHTTP;
    sktHTTP = HTTPCurConnectionSocketGet(connHandle);
    uint8_t bssTypeString[4];
    if (bssDescIsValid)
    {
        uitoa(bssDesc.bssType, bssTypeString);
        TCPPutString(sktHTTP, bssTypeString);
    }
    else
    {    
        TCPPutString(sktHTTP, (const uint8_t*)"0");
    }    
}    

void HTTPPrint_strength(HTTP_CONN_HANDLE connHandle)
{
    uint8_t strVal;
    uint8_t strString[4];
    TCP_SOCKET sktHTTP;
    sktHTTP = HTTPCurConnectionSocketGet(connHandle);
    if (bssDescIsValid)
    {
        if (bssDesc.rssi < 121)
        {
            strVal = 1;
        }
        else if (bssDesc.rssi < 141)
        {
            strVal = 2;
        }
        else if (bssDesc.rssi < 161)
        {
            strVal = 3;
        }
        else
        {
            strVal = 4;
        }
        
        uitoa(strVal, strString);
        TCPPutString(sktHTTP, strString);
    }
    else
    {                
        TCPPutString(sktHTTP, (const uint8_t*)"0");
    }
}

void HTTPPrint_nextSSID(HTTP_CONN_HANDLE connHandle)
{
    TCP_SOCKET sktHTTP;
    sktHTTP = HTTPCurConnectionSocketGet(connHandle);
    TCPPutString(sktHTTP, (uint8_t*)"ToDo:nextSSID");//(uint8_t*)CFGCXT.ssid);
}
void HTTPPrint_prevSSID(HTTP_CONN_HANDLE connHandle)
{
    TCP_SOCKET sktHTTP;
    sktHTTP = HTTPCurConnectionSocketGet(connHandle);
    TCPPutString(sktHTTP, (uint8_t*)"ToDo: prevSSID");//(uint8_t*)CFGCXT.prevSSID);
}
void HTTPPrint_prevWLAN(HTTP_CONN_HANDLE connHandle)
{
    TCP_SOCKET sktHTTP;
    sktHTTP = HTTPCurConnectionSocketGet(connHandle);

    if (CFGCXT.prevWLAN == WF_NETWORK_TYPE_INFRASTRUCTURE)
    {
        TCPPutString(sktHTTP, (const uint8_t*)"infrastructure (BSS)");
    }

    else if (CFGCXT.prevWLAN == WF_NETWORK_TYPE_SOFT_AP)
    {
        TCPPutString(sktHTTP, (const uint8_t*)"softAP (BSS)");
    }
    else
    {
        TCPPutString(sktHTTP, (const uint8_t*)"adhoc (IBSS)");
    }
}
void HTTPPrint_nextWLAN(HTTP_CONN_HANDLE connHandle)
{
    TCP_SOCKET sktHTTP;
    sktHTTP = HTTPCurConnectionSocketGet(connHandle);

    if (CFGCXT.type == WF_NETWORK_TYPE_INFRASTRUCTURE)
    {
        TCPPutString(sktHTTP, (const uint8_t*)"infrastructure (BSS)");
    }
        
    else if (CFGCXT.type == WF_NETWORK_TYPE_ADHOC)
    {
        TCPPutString(sktHTTP, (const uint8_t*)"adhoc (IBSS)");
    }
    else if (CFGCXT.type == WF_NETWORK_TYPE_SOFT_AP)
    {     
        TCPPutString(sktHTTP, (const uint8_t*)"softAP (BSS)");
    }
    else
    {
        TCPPutString(sktHTTP, (const uint8_t*)"unknown");
    }
}

//==============================================================================
//                      Local Function Prototypes for iwpriv
//==============================================================================

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
#if 0
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
#endif

#if 0
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
#endif


#if 0  //This API has been defined in wf_commands.c
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
#endif
#endif
