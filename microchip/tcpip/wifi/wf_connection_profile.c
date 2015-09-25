/*******************************************************************************
  MRF24W Driver Connection Profile

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Provides access to MRF24W WiFi controller
    -Reference: MRF24W Data sheet, IEEE 802.11 Standard
*******************************************************************************/

/*******************************************************************************
FileName:   wf_connection_profile.c
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

#if defined(TCPIP_IF_MRF24W)

#include "wf_debug_output.h"
#include "wf_configData.h"

/*
*********************************************************************************************************
*                                           LOCAL DATA TYPES                               
*********************************************************************************************************
*/

/* header format for response to CP Get Element message */
typedef struct cpElementResponseStruct
{
    tMgmtMsgRxHdr  mgmtHdr;                /* normal 4-byte hdr for all mgmt responses */
    uint8_t        profileId;              // connection profile ID
    uint8_t        elementId;
    uint8_t        elementDataLength;
    /* element data follows */
} tCPElementResponseHdr;    

/*
*********************************************************************************************************
*                                           GLOBALS
*********************************************************************************************************
*/
static uint8_t g_networkType;

/*
*********************************************************************************************************
*                                           LOCAL FUNCTION PROTOTYPES                               
*********************************************************************************************************
*/

static void SetSecurity(uint8_t securityType,
                        uint8_t wepKeyIndex,
                        uint8_t *p_securityKey,
                        uint8_t securityKeyLength);

static void LowLevel_CPSetElement(uint8_t elementId, 
                                  uint8_t *p_elementData, 
                                  uint8_t elementDataLength);

static void LowLevel_CPGetElement(uint8_t elementId, 
                                  uint8_t *p_elementData, 
                                  uint8_t elementDataLength,
                                  uint8_t dataReadAction);

// creates a connection profile on MRF24WG that will be used forever
void WF_CPCreate(void)
{
    uint8_t  hdr[2];
    
    hdr[0] = WF_MGMT_REQUEST_TYPE;
    hdr[1] = WF_CP_CREATE_PROFILE_SUBTYPE;

    SendMgmtMsg(hdr,
                sizeof(hdr),
                NULL,            /* no data */
                0);              /* no data */
    
    /* wait for MRF24W management response, read data, free response after read */
    WaitForMgmtResponse(WF_CP_CREATE_PROFILE_SUBTYPE, FREE_MGMT_BUFFER);
}    


/*******************************************************************************
  Function:    
    void WF_SsidSet(uint8_t *p_ssid, uint8_t *p_ssidLength)

  Summary:
    Sets the SSID for the specified Connection Profile ID.    

  Description:
    Sets the SSID and SSID Length elements in the Connection Profile.  Note that
    if an Access Point can have either a visible or hidden SSID.  If an Access Point
    uses a hidden SSID then an active scan must be used (see scanType field in the 
    Connection Algorithm).

  Precondition:
    MACInit must be called first.

  Parameters:
    p_ssid - Pointer to the SSID string
    ssidLength - Number of bytes in the SSID

  Returns:
    None.
      
  Remarks:
    None.
  *****************************************************************************/
void WF_SsidSet(uint8_t *p_ssid,  uint8_t ssidLength)
{
    SYS_ASSERT(ssidLength <= WF_MAX_SSID_LENGTH, "");
    LowLevel_CPSetElement(WF_CP_ELEMENT_SSID,     /* Element ID                   */
                         (uint8_t *)p_ssid,       /* pointer to element data      */
                          ssidLength);            /* number of element data bytes */
    memcpy(p_wifi_ConfigData->netSSID,p_ssid,ssidLength);
    p_wifi_ConfigData->SsidLength=ssidLength;
    p_wifi_ConfigData->netSSID[ssidLength]=0x00;

}   
 
/*******************************************************************************
  Function:    
    void WF_SsidGet(uint8_t *p_ssid, uint8_t *p_ssidLength)

  Summary:
    Gets the SSID for the specified Connection Profile ID.    

  Description:
    Gets the SSID and SSID Length elements in the Connection Profile.

  Precondition:
    MACInit must be called first.

  Parameters:
    p_ssid - Pointer to the SSID string
    ssidLength - Pumber of bytes in the SSID

  Returns:
    None.
      
  Remarks:
    None.
  *****************************************************************************/
void WF_SsidGet(uint8_t *p_ssid, uint8_t *p_ssidLength)
{
    tCPElementResponseHdr  mgmtHdr;
    
    /* Request SSID, but don't have this function read data or free response buffer.       */
    LowLevel_CPGetElement(WF_CP_ELEMENT_SSID,     /* Element ID                            */
                          NULL,                   /* ptr to element data (not used here    */
                          0,                      /* num data bytes to read (not used here */ 
                          false);                 /* no read, leave response mounted       */

    /* At this point, management response is mounted and ready to be read.                 */
    /* Set raw index to 0, read normal 4 byte header plus the next 3 bytes, these will be: */
    /*   profile id             [4]                                                        */
    /*   element id             [5]                                                        */
    /*   element data length    [6]                                                        */
    RawRead(RAW_MGMT_RX_ID, 0, sizeof(tCPElementResponseHdr), (uint8_t *)&mgmtHdr);

    /* extract SSID length and write to caller */
    *p_ssidLength = mgmtHdr.elementDataLength;
    
    /* copy SSID name to callers buffer */
    RawRead(RAW_MGMT_RX_ID, sizeof(tCPElementResponseHdr), *p_ssidLength, p_ssid);
    
    /* free management buffer */
    DeallocateMgmtRxBuffer();
}

/*******************************************************************************
  Function:    
    void WF_BssidSet(uint8_t *p_bssid)

  Summary:
    Sets the BSSID for the specified Connection Profile ID.

  Description:
    Sets the BSSID element in a Connection Profile.

  Precondition:
    MACInit must be called first.

  Parameters:
    p_bssid - Pointer to the BSSID 

  Returns:
    None.
      
  Remarks:
    None.
  *****************************************************************************/
void WF_BssidSet(uint8_t *p_bssid)
{
    LowLevel_CPSetElement(WF_CP_ELEMENT_BSSID,   /* Element ID                   */
                          p_bssid,               /* pointer to element data      */
                          WF_BSSID_LENGTH);      /* number of element data bytes */
}   

/*******************************************************************************
  Function:    
    void WF_BssidGet(uint8_t *p_bssid)

  Summary:
    Gets the BSSID for the specified Connection Profile ID.

  Description:
    Gets the BSSID element in a Connection Profile.

  Precondition:
    MACInit must be called first.

  Parameters:
    p_bssid - Pointer to the BSSID 

  Returns:
    None.
      
  Remarks:
    None.
  *****************************************************************************/
void WF_BssidGet(uint8_t *p_bssid)
{
    LowLevel_CPGetElement(WF_CP_ELEMENT_BSSID,   /* Element ID                        */
                          p_bssid,               /* pointer to element data           */
                          WF_BSSID_LENGTH,       /* number of element data bytes      */
                          true);                 /* read data, free buffer after read */
}   

void DhcpConfigTask(void)
{
    TCPIP_NET_HANDLE netH;
    uint8_t networkType;

    networkType = GetWiFiNetworkType();

    netH = TCPIP_STACK_NetHandle("MRF24W");
    if(networkType == WF_NETWORK_TYPE_SOFT_AP)
    {
        DHCPDisable(netH);          //Client  Off
        DHCPServerEnable(netH);     //Server On
    }
    else if(networkType == WF_NETWORK_TYPE_INFRASTRUCTURE)
    {
        DHCPServerDisable(netH);    //Server Off
        DHCPEnable(netH);           //Client On
    }
    else if(networkType == WF_NETWORK_TYPE_ADHOC)
    {
        DHCPDisable(netH);           //Client Off
        DHCPServerEnable(netH);     //Server On
    }
}

/*******************************************************************************
  Function:    
    void WF_NetworkTypeSet(uint8_t networkType)

  Summary:
    Sets the network for the specified Connection Profile ID.

  Description:
    Sets the Network Type element a Connection Profile.  Allowable values are:
    * WF_NETWORK_TYPE_INFRASTRUCTURE
    * WF_NETWORK_TYPE_ADHOC

  Precondition:
    MACInit must be called first.

  Parameters:
    networkType - Type of network to create (infrastructure or adhoc)

  Returns:
    None.
      
  Remarks:
    None.
  *****************************************************************************/
void WF_NetworkTypeSet(uint8_t networkType)
{
    LowLevel_CPSetElement(WF_CP_ELEMENT_NETWORK_TYPE,   /* Element ID                   */
                          &networkType,                 /* pointer to element data      */
                          1);                           /* number of element data bytes */
    p_wifi_ConfigData->networkType = networkType;
    g_networkType = networkType;
}

uint8_t GetWiFiNetworkType(void)
{
    return g_networkType;
}

/*******************************************************************************
  Function:    
    void WF_NetworkTypeGet(uint8_t networkType)

  Summary:
    Gets the network for the specified Connection Profile ID.

  Description:
    Gets the Network Type element a Connection Profile.  Allowable values are:
    * WF_NETWORK_TYPE_INFRASTRUCTURE
    * WF_NETWORK_TYPE_ADHOC

  Precondition:
    MACInit must be called first.

  Parameters:
    networkType - Type of network to create (infrastructure or adhoc)

  Returns:
    None.
      
  Remarks:
    None.
  *****************************************************************************/ 
void WF_NetworkTypeGet(uint8_t *p_networkType)
{
    LowLevel_CPGetElement(WF_CP_ELEMENT_NETWORK_TYPE, /* element ID             */
                          p_networkType,              /* element data pointer   */
                          1,                          /* read one byte          */
                          true);                      /* read data, free buffer */
}

/*******************************************************************************
  Function:    
    void WF_CPSetWepKeyType(uint8_t wepKeyType)

  Summary:
    Sets the Wep key type

  Description:
     Sets the Wep key type

  Precondition:
    MACInit must be called first.

  Parameters:
    wepKeyType -- WF_SECURITY_WEP_SHAREDKEY or WF_SECURITY_WEP_OPENKEY (default)

  Returns:
    None.
      
  Remarks:
    None.
  *****************************************************************************/
void WF_CPSetWepKeyType(uint8_t wepKeyType)
{
    LowLevel_CPSetElement(WF_CP_ELEMENT_WEPKEY_TYPE,    /* Element ID                   */
                          &wepKeyType,                  /* pointer to element data      */
                          1);                           /* number of element data bytes */
    p_wifi_ConfigData->wepKeyType = wepKeyType;
}   

/*******************************************************************************
  Function:    
    void WF_WepKeyTypeGet(uint8_t *p_keyType)

  Summary:
    Gets the Wep Key type

  Description:
    Gets the Network Type element a Connection Profile.  Allowable values are:
    * WF_SECURITY_WEP_SHAREDKEY
    * WF_SECURITY_WEP_OPENKEY

  Precondition:
    MACInit must be called first.

  Parameters:
    networkType -- type of key for Wep security (shared key or open key)

  Returns:
    None.
      
  Remarks:
    None.
  *****************************************************************************/ 
void WF_WepKeyTypeGet(uint8_t *p_wepKeyType)
{
    LowLevel_CPGetElement(WF_CP_ELEMENT_WEPKEY_TYPE, /* element ID             */
                          p_wepKeyType,              /* element data pointer   */
                          1,                          /* read one byte          */
                          true);                      /* read data, free buffer */
}


/*******************************************************************************
  Function:    
    void WF_WPSCredentialsGet(t_wfWpsCred *p_cred)

  Summary:
    Gets the WPS credentials

  Description:
    Gets the WPS credentials after WPS completed

  Precondition:
    MACInit must be called first.

  Parameters:
    p_cred - Pointer to the credentials

  Returns:
    None.
      
  Remarks:
    None.
  *****************************************************************************/
void WF_WPSCredentialsGet(t_wfWpsCred *p_cred)
{
    LowLevel_CPGetElement(WF_CP_ELEMENT_READ_WPS_CRED,   /* Element ID                        */
                          (uint8_t *)p_cred,             /* pointer to element data           */
                          sizeof(*p_cred),               /* number of element data bytes      */
                          true);                         /* read data, free buffer after read */
}   

//============================================================================
void WF_SecurityOpenSet()
{
    SetSecurity(WF_SECURITY_OPEN, 0, NULL, 0);
}

//============================================================================
void WF_SecurityWepSet(t_wepContext* p_context)
{
    SetSecurity(p_context->wepSecurityType,
                WF_DEFAULT_WEP_KEY_INDEX,
                p_context->wepKey,
                p_context->wepKeyLength);
}

//============================================================================
void WF_SecurityWpaSet(t_wpaContext* p_context)
{
#if defined(WF_ERROR_CHECKING)
    uint32_t errorCode;

    errorCode = UdSetSecurityWpa(p_context);
    if (errorCode != UD_SUCCESS)
    {
        EventEnqueue(WF_EVENT_ERROR, errorCode);
        return;
    }
#endif /* WF_ERROR_CHECKING */

    SetSecurity(p_context->wpaSecurityType,
                0, // not used
                p_context->keyInfo.key,
                p_context->keyInfo.keyLength);
}

//============================================================================
void WF_SecurityWpsSet(t_wpsContext *p_context)
{
    SetSecurity(p_context->wpsSecurityType,
                0,
                p_context->wpsPin,
                p_context->wpsPinLength);

#if defined(WF_USE_HOST_WPA_KEY_CALCULATION)
    // if host wants the host to calculate a binary key from a possible WPS-PSK passphrase
    if (p_context->getPassPhrase == true)
    {
        // tell MRF24WG to send wpa-psk passphrase back to host (if AP using WPA passphrase)
        YieldPassPhraseToHost();

        // save pointer to passphrase info block
        g_p_wpaKeyInfo = p_context->p_keyInfo;
    }
#endif /* WF_USE_HOST_WPA_KEY_CALCULATION */
}


/*******************************************************************************
  Function:    
    void SetSecurity(uint8_t securityType,
                     uint8_t wepKeyIndex,
                     uint8_t *p_securityKey,
                     uint8_t securityKeyLength)

  Summary:
    Sets the security

  Description:
    Configures security

    <table>
    Security                                Key         Length
    --------                                ---         ------
    WF_SECURITY_OPEN                        N/A         N/A
    WF_SECURITY_WEP_40                      hex         4, 5 byte keys
    WF_SECURITY_WEP_104                     hex         4, 13 byte keys
    WF_SECURITY_WPA_WITH_KEY                hex         32 bytes
    WF_SECURITY_WPA_WITH_PASS_PHRASE        ascii       8-63 ascii characters
    WF_SECURITY_WPA2_WITH_KEY               hex         32 bytes
    WF_SECURITY_WPA2_WITH_PASS_PHRASE       ascii       8-63 ascii characters
    WF_SECURITY_WPA_AUTO_WITH_KEY           hex         32 bytes
    WF_SECURITY_WPA_AUTO_WITH_PASS_PHRASE   ascii       8-63 ascii characters
    </table>

  Precondition:
    MACInit must be called first.

  Parameters:
    securityType - Value corresponding to the security type desired.
    wepKeyIndex - 0 thru 3 (only used if security type is WF_SECURITY_WEP_40 or
                   WF_SECURITY_WEP_104)
    p_securityKey - Binary key or passphrase (not used if security is 
                     WF_SECURITY_OPEN)
    securityKeyLength - Number of bytes in p_securityKey (not used if security
                         is WF_SECURITY_OPEN)

  Returns:
    None.
      
  Remarks:
    None.
  *****************************************************************************/
static void SetSecurity(uint8_t securityType,
                        uint8_t wepKeyIndex,
                        uint8_t *p_securityKey,
                        uint8_t securityKeyLength)
{
    uint8_t  hdrBuf[7];
    uint8_t  *p_key;

    /* Write out header portion of msg */
    hdrBuf[0] = WF_MGMT_REQUEST_TYPE;           /* indicate this is a mgmt msg     */
    hdrBuf[1] = WF_CP_SET_ELEMENT_SUBTYPE;      /* mgmt request subtype            */     
    hdrBuf[2] = CPID;                           /* Connection Profile ID           */
    hdrBuf[3] = WF_CP_ELEMENT_SECURITY;         /* Element ID                      */
    
    /* Next to header bytes are really part of data, but need to put them in header      */
    /* bytes in order to prepend to security key                                         */
    hdrBuf[5] = securityType;                   
    hdrBuf[6] = wepKeyIndex;                     
    
    /* if security is open (no key) or WPS push button method */
    if (securityType == WF_SECURITY_OPEN            ||
        securityType == WF_SECURITY_WPS_PUSH_BUTTON ||
        securityType == WF_SECURITY_EAP)
    {
        hdrBuf[4]         = 2;      /* Only data is security type and wep index */ 
        p_key             = NULL;   
        securityKeyLength = 0;    

    } 
    /* else security is selected, so need to send key */   
    else
    {
        hdrBuf[4] = 2 + securityKeyLength;  /* data is security type + wep index + key */
        p_key     = p_securityKey;       
    }    
    
    SendMgmtMsg(hdrBuf,              /* msg header which includes the security type and WEP index)    */
                sizeof(hdrBuf),      /* msg header length                                             */
                p_key,               /* msg data (security key), can be NULL                          */
                securityKeyLength);  /* msg data length (will be 0 if p_securityKey is NULL)          */
    
    /* wait for mgmt response, free after it comes in, don't need data bytes */
    WaitForMgmtResponse(WF_CP_SET_ELEMENT_SUBTYPE, FREE_MGMT_BUFFER);

    p_wifi_ConfigData->SecurityMode = securityType;
    p_wifi_ConfigData->SecurityKeyLength = securityKeyLength;
    memcpy(p_wifi_ConfigData->SecurityKey,p_securityKey,securityKeyLength);
}   

/*******************************************************************************
  Function:    
    void WF_SecurityGet(uint8_t securityType,
                        uint8_t wepKeyIndex,
                        uint8_t *p_securityKey,
                        uint8_t securityKeyLength)

  Summary:
    Gets the security

  Description:
    Gets the security

    <table>
    Security                                Key         Length
    --------                                ---         ------
    WF_SECURITY_OPEN                        N/A         N/A
    WF_SECURITY_WEP_40                      hex         4, 5 byte keys
    WF_SECURITY_WEP_104                     hex         4, 13 byte keys
    WF_SECURITY_WPA_WITH_KEY                hex         32 bytes
    WF_SECURITY_WPA_WITH_PASS_PHRASE        ascii       8-63 ascii characters
    WF_SECURITY_WPA2_WITH_KEY               hex         32 bytes
    WF_SECURITY_WPA2_WITH_PASS_PHRASE       ascii       8-63 ascii characters
    WF_SECURITY_WPA_AUTO_WITH_KEY           hex         32 bytes
    WF_SECURITY_WPA_AUTO_WITH_PASS_PHRASE   ascii       8-63 ascii characters
    </table>

  Precondition:
    MACInit must be called first.

  Parameters:
    securityType       - Value corresponding to the security type desired.
    wepKeyIndex        - 0 thru 3 (only used if security type is WF_SECURITY_WEP_40 or
                          WF_SECURITY_WEP_104)
    p_securityKey      - Binary key or passphrase (not used if security is
                          WF_SECURITY_OPEN)
    securityKeyLength - Number of bytes in p_securityKey (not used if security
                         is WF_SECURITY_OPEN)

  Returns:
    None.
      
  Remarks:
    If security was initially set with a passphrase that the MRF24WG used to generate
    a binary key, this function returns the binary key, not the passphrase.
  *****************************************************************************/
void WF_SecurityGet(uint8_t *p_securityType,
                    uint8_t *p_securityKey,
                    uint8_t *p_securityKeyLength)
{
    tCPElementResponseHdr mgmtHdr;
    uint8_t keyLength;
    uint8_t wepKeyIndex;
    
    /* send request, wait for mgmt response, do not read and do not free up response buffer */
     LowLevel_CPGetElement(WF_CP_ELEMENT_SECURITY,        /* Element ID      */
                           NULL,                          /* do not read     */
                           0,                             /* do not read     */
                           false);                        /* do not read, do not free mgmt buffer */

    /* at this point, management response is mounted and ready to be read */

    /* At this point, management response is mounted and ready to be read.                 */
    /* Set raw index to 0, read normal 4 byte header plus the next 3 bytes, these will be: */
    /*   profile id             [4]                                                        */
    /*   element id             [5]                                                        */
    /*   element data length    [6]                                                        */
    RawRead(RAW_MGMT_RX_ID, 0, sizeof(tCPElementResponseHdr), (uint8_t *)&mgmtHdr);
    
    RawRead(RAW_MGMT_RX_ID,                     /* raw Id                     */
            sizeof(tCPElementResponseHdr) + 0,  /* index of security type [7] */
            1,                                  /* read one byte              */
            p_securityType);                    /* copy that byte here        */

    // read wep key index to increment pointer in raw buffer, but throw away, it is always 0
    RawRead(RAW_MGMT_RX_ID,                     /* raw Id                     */
            sizeof(tCPElementResponseHdr) + 1 , /* index of WEP key index [8] */
            1,                                  /* read one byte              */
            &wepKeyIndex);                     /* copy that byte here        */

    /* determine security key length and read if it is present */
    keyLength = mgmtHdr.elementDataLength - 2;
    if (keyLength > 0)
    {
        *p_securityKeyLength = keyLength;
        
        RawRead(RAW_MGMT_RX_ID,                     /* raw Id                  */
                sizeof(tCPElementResponseHdr) + 2,  /* index of first key byte */
                keyLength,                          /* number of bytes to read */
                p_securityKey);                     /* copy bytes here         */
        
    }
    /* no security key, so set key length param to 0 */
    else
    {
        *p_securityKeyLength = 0;
    }       
    
    /* free management buffer */
    DeallocateMgmtRxBuffer();
}    



void SetHiddenSsid(bool hiddenSsid)
{
    LowLevel_CPSetElement(WF_CP_ELEMENT_SSID_TYPE, // Element ID
                          (uint8_t *)&hiddenSsid,  // pointer to element data
                          1);                      // number of element data bytes
}

uint8_t GetHiddenSsid(void)
{
    uint8_t hidden;

    LowLevel_CPGetElement(WF_CP_ELEMENT_SSID_TYPE, /* element ID             */
                          &hidden,                 /* element data pointer   */
                          1,                       /* read one byte          */
                          true);                   /* read data, free buffer */
     return hidden;
}

// called from SetAdhocContext().  Error checking performed there
void SetAdHocMode(uint8_t mode)
{
    LowLevel_CPSetElement(WF_CP_ELEMENT_ADHOC_BEHAVIOR,  // Element ID
                          &mode,                         // pointer to element data
                          1);                            // number of element data bytes
}

uint8_t GetAdHocMode(void)
{
    uint8_t adhocMode;
    LowLevel_CPGetElement(WF_CP_ELEMENT_ADHOC_BEHAVIOR, /* element ID             */
                          &adhocMode,                   /* element data pointer   */
                          1,                            /* read one byte          */
                          true);                        /* read data, free buffer */
    return adhocMode;
}

/*******************************************************************************
  Function:    
    static void LowLevel_CPSetElement(uint8_t elementId, 
                                      uint8_t *p_elementData,
                                      uint8_t elementDataLength)

  Summary:
    Set an element of the connection profile on the MRF24W.

  Description:
    All Connection Profile 'Set Element' functions call this function to 
    construct the management message.  The caller must fix up any endian issues 
    prior to calling this function.

  Precondition:
    MACInit must be called first.

  Parameters:
    elementId - Element that is being set
    p_elementData - Pointer to element data
    elementDataLength - Number of bytes pointed to by p_elementData

  Returns:
    None.
      
  Remarks:
    None.
  *****************************************************************************/
static void LowLevel_CPSetElement(uint8_t elementId, 
                                  uint8_t *p_elementData, 
                                  uint8_t elementDataLength)
{
    uint8_t  hdrBuf[5];
    
    /* Write out header portion of msg */
    hdrBuf[0] = WF_MGMT_REQUEST_TYPE;       /* indicate this is a mgmt msg     */
    hdrBuf[1] = WF_CP_SET_ELEMENT_SUBTYPE;  /* mgmt request subtype            */     
    hdrBuf[2] = CPID;                       /* Connection Profile ID           */
    hdrBuf[3] = elementId;                  /* Element ID                      */
    hdrBuf[4] = elementDataLength;          /* number of bytes of element data */
    
    SendMgmtMsg(hdrBuf,              /* msg header        */
                sizeof(hdrBuf),      /* msg header length */
                p_elementData,       /* msg data          */
                elementDataLength);  /* msg data length   */
    
    /* wait for mgmt response, free after it comes in, don't need data bytes */
    WaitForMgmtResponse(WF_CP_SET_ELEMENT_SUBTYPE, FREE_MGMT_BUFFER);

}    

/*******************************************************************************
  Function:    
    static void LowLevel_CPGetElement(uint8_t elementId, 
                                      uint8_t *p_elementData, 
                                      uint8_t elementDataLength,
                                      uint8_t dataReadAction)

  Summary:
    Get an element of the connection profile on the MRF24W.

  Description:
    All Connection Profile 'Get Element' functions call this function to 
    construct the management message.  The caller must fix up any endian issues 
    prior to calling this function.

  Precondition:
    MACInit must be called first.

  Parameters:
    elementId - Element that is being read
    p_elementData - Pointer to where element data will be written
    elementDataLength - Number of element data bytes that will be read
    dataReadAction - If true then read data per paramters and free mgmt 
                      response buffer. If false then return after response 
                      received, do not read any data as the caller will do that, 
                      and don't free buffer, as caller will do that as well.

  Returns:
    None.
      
  Remarks:
    None.
  *****************************************************************************/
static void LowLevel_CPGetElement(uint8_t elementId, 
                                  uint8_t *p_elementData, 
                                  uint8_t elementDataLength,
                                  uint8_t dataReadAction)    /* true or false */
{
    uint8_t  hdrBuf[4];
      
    hdrBuf[0] = WF_MGMT_REQUEST_TYPE;       /* indicate this is a mgmt msg     */
    hdrBuf[1] = WF_CP_GET_ELEMENT_SUBTYPE;  /* mgmt request subtype            */     
    hdrBuf[2] = CPID;                       /* Connection Profile ID           */
    hdrBuf[3] = elementId;                  /* Element ID                      */

    SendMgmtMsg(hdrBuf,              /* msg header        */
                sizeof(hdrBuf),      /* msg header length */
                NULL,                /* msg data          */
                0);                  /* msg data length   */
  
    if (dataReadAction == (uint8_t)true)
    {
        /* wait for mgmt response, read desired data, and then free response buffer */
        WaitForMgmtResponseAndReadData(WF_CP_GET_ELEMENT_SUBTYPE, 
                                       elementDataLength,                   /* num data bytes to read                */
                                       sizeof(tCPElementResponseHdr),       /* index of first byte of element data   */
                                       p_elementData);                      /* where to write element data           */
    }
    else
    {
        /* wait for mgmt response, don't read any data bytes, do not release mgmt buffer */
        WaitForMgmtResponse(WF_CP_GET_ELEMENT_SUBTYPE, DO_NOT_FREE_MGMT_BUFFER);
    }                                                    
}
    

#endif /* TCPIP_IF_MRF24W */
