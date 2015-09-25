/*******************************************************************************
  MRF24W Driver Connection Algorithm

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Provides access to MRF24W WiFi controller
    -Reference: MRF24W Data sheet, IEEE 802.11 Standard
*******************************************************************************/

/*******************************************************************************
FileName:   wf_connection_algorithm.c
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

/*
*********************************************************************************************************
*                                           DEFINES                               
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                           LOCAL DATA TYPES                               
*********************************************************************************************************
*/

/* header format for response to CP Get Element message */
typedef struct caElementResponseStruct
{
    tMgmtMsgRxHdr   mgmtHdr;                /* normal 4-byte hdr for all mgmt responses */
    uint8_t           elementId;              /* index 4 */
    uint8_t           elementDataLength;      /* index 5 */
    /* element data follows */
} tCAElementResponseHdr;    


/*
*********************************************************************************************************
*                                           LOCAL GLOBAL VARIABLES                               
*********************************************************************************************************
*/

static uint8_t g_EventNotificationAction = WF_NOTIFY_ALL_EVENTS;


/*
*********************************************************************************************************
*                                           LOCAL FUNCTION PROTOTYPES                               
*********************************************************************************************************
*/

static void LowLevel_CASetElement(uint8_t elementId, 
                                  uint8_t *p_elementData, 
                                  uint8_t elementDataLength);

static void LowLevel_CAGetElement(uint8_t elementId, 
                                  uint8_t *p_elementData, 
                                  uint8_t elementDataLength,
                                  uint8_t dataReadAction);

static void SetEventNotificationMask(uint8_t eventNotificationBitMask);

extern void SetHiddenSsid(bool hiddenSsid);
extern uint8_t GetHiddenSsid(void);
extern void SetAdHocMode(uint8_t mode);
extern uint8_t GetAdHocMode(void);
                       


void WF_ScanContextSet(t_scanContext *p_context)
{
    uint16_t tmp;

    LowLevel_CASetElement(WF_CA_ELEMENT_SCANTYPE,           // Element ID
                          &p_context->scanType,             // pointer to element data
                          sizeof(p_context->scanType));     // number of element data bytes
                        

    LowLevel_CASetElement(WF_CA_ELEMENT_SCAN_COUNT,
                          &p_context->scanCount,
                          sizeof(p_context->scanCount));

    tmp = TCPIP_HELPER_htons(p_context->minChannelTime);
    LowLevel_CASetElement(WF_CA_ELEMENT_MIN_CHANNEL_TIME,
                          (uint8_t *)&tmp,
                          sizeof(tmp));

    tmp = TCPIP_HELPER_htons(p_context->minChannelTime);
    LowLevel_CASetElement(WF_CA_ELEMENT_MAX_CHANNEL_TIME,
                          (uint8_t *)&tmp,
                          sizeof(tmp));
    

    tmp = TCPIP_HELPER_htons(p_context->probeDelay);
    LowLevel_CASetElement(WF_CA_ELEMENT_PROBE_DELAY,
                          (uint8_t *)&tmp,
                          sizeof(tmp));
}

void WF_ScanContextGet(t_scanContext *p_context)
{
    uint16_t tmp;

    LowLevel_CAGetElement(WF_CA_ELEMENT_SCANTYPE,           // Element ID
                          &p_context->scanType,             // pointer to element data
                          sizeof(p_context->scanType),      // number of element data bytes
                          true);                            // delete mgmt msg after read

    LowLevel_CAGetElement(WF_CA_ELEMENT_SCAN_COUNT,
                          &p_context->scanCount,
                          sizeof(p_context->scanCount),
                          true);

    LowLevel_CAGetElement(WF_CA_ELEMENT_MIN_CHANNEL_TIME,
                          (uint8_t *)&tmp,
                          sizeof(tmp),
                          true);
    tmp = TCPIP_HELPER_htons(p_context->minChannelTime);

    LowLevel_CAGetElement(WF_CA_ELEMENT_MAX_CHANNEL_TIME,
                          (uint8_t *)&tmp,
                          sizeof(tmp),
                          true);
    tmp = TCPIP_HELPER_htons(p_context->minChannelTime);

    LowLevel_CAGetElement(WF_CA_ELEMENT_PROBE_DELAY,
                          (uint8_t *)&tmp,
                          sizeof(tmp),
                          true);
    tmp = TCPIP_HELPER_htons(p_context->probeDelay);
}


/*****************************************************************************
  Function:
    void WF_RssiSet(uint8_t rssi)

  Summary:
    Sets the RSSI threshold

  Description:
    Specifies the RSSI behavior when connecting.  This value is only used if
      1) The current Connection Profile has not defined an SSID or BSSID
      2) An SSID is defined in the current Connection Profile and multiple
         access points are discovered with the same SSID.
         
     Values:
       0      : Connect to the first network found
       1 - 254: Only connect to a network if the RSSI is greater than or equal to
                the specified value
       255:     Connect to the highest RSSI found

       Note that RSSI is a relative value with no units -- it is not correlated to dBm.
       
  Precondition:
    MACInit must be called first.

  Parameters:
    scanCount - Desired scan count
                 
  Returns:
    None
      
  Remarks:
    Default is 255
*****************************************************************************/
void WF_RssiSet(uint8_t rssi)
{
    LowLevel_CASetElement(WF_CA_ELEMENT_RSSI,  /* Element ID                   */
                          &rssi,                /* pointer to element data      */
                          sizeof(rssi));        /* number of element data bytes */
   
}       

/*****************************************************************************
  Function:
    void WF_RssiGet(uint8_t p_rssi)

  Summary:
    Gets the RSSI threshold

  Description:
    See WF_RssiSet.  Note that this function only retrieves the RSSI threshold
    used during the connection -- this is not the current RSSI of an existing connection.
    If it is desired to retrieve the current RSSI state then a scan must be performed and
    the scan result will contain the current RSSI state.

  Precondition:
    MACInit must be called first.

  Parameters:
    p_rssi - Pointer to where RSSI value is written
                 
  Returns:
    None
      
  Remarks:
    Default is 255
*****************************************************************************/
void WF_RssiGet(uint8_t *p_rssi)
{
    LowLevel_CAGetElement(WF_CA_ELEMENT_RSSI,      /* Element ID                   */
                          p_rssi,                  /* pointer to element data      */
                          1,                       /* number of element data bytes */
                         true);                    /* read data, free buffer       */
}          

      
/*******************************************************************************
  Function:
    void WF_CASetEventNotificationAction(uint8_t eventNotificationAction)

  Summary:
    Sets the WiFi events that the host wishes to be notified of.

  Description:
    Sets the Event Notification Action used by the Connection Algorithm.  The
    bit mask for the allowable entries is as follows:
    
    <table>
        Bit     Event
        ---     -----
        0       WF_NOTIFY_CONNECTION_ATTEMPT_SUCCESSFUL  
        1       WF_NOTIFY_CONNECTION_ATTEMPT_FAILED
        2       WF_NOTIFY_CONNECTION_TEMPORARILY_LOST
        3       WF_NOTIFY_CONNECTION_PERMANENTLY_LOST 
        4       WF_NOTIFY_CONNECTION_REESTABLISHED 
    </table>

  Precondition:
    MACInit must be called first.

  Parameters:
    eventNotificationAction - Bit mask indicating which events the host wants
                               to be notifed of.

  Returns:
    None.
      
  Remarks:
    None.
 *****************************************************************************/
void WF_CASetEventNotificationAction(uint8_t eventNotificationAction)
{    
    /* Remember what events application wants to be notified of.  The MRF24W will inform the WiFi driver    */
    /* of all events, but only events the application wants to see will ripple up to WF_ProcessEvent().     */
    SetEventNotificationMask(eventNotificationAction);
   
}    

/*******************************************************************************
  Function:    
    void WF_CAGetEventNotificationAction(uint8_t *p_eventNotificationAction)

  Summary:
    Reads the Connection Algorithm event notification action.

  Description:
    Gets the Event Notification Action used by the Connection Algorithm.  The
    value read back will be a bit mask that corresponds to the following table:

    <table>
        Bit     Event
        ---     -----
        0       WF_NOTIFY_CONNECTION_ATTEMPT_SUCCESSFUL  
        1       WF_NOTIFY_CONNECTION_ATTEMPT_FAILED
        2       WF_NOTIFY_CONNECTION_TEMPORARILY_LOST
        3       WF_NOTIFY_CONNECTION_PERMANENTLY_LOST 
        4       WF_NOTIFY_CONNECTION_REESTABLISHED 
    </table>

  Precondition:
    MACInit must be called first.

  Parameters:
    p_eventNotificationAction - Pointer to where returned value is written.

  Returns:
    None.
      
  Remarks:
    None.
 *****************************************************************************/
void WF_CAGetEventNotificationAction(uint8_t *p_eventNotificationAction)
{
    *p_eventNotificationAction = GetEventNotificationMask();
}  


/*******************************************************************************
  Function:
    void WF_ReconnectModeSet(uint8_t retryCount, uint8_t deauthAction, uint8_t beaconTimeout, uint8_t beaconTimeoutAction);

  Summary:
    Controls how the MRF24WG handles reconnection in the event of a beacon timeout
    or a deauthentication from the AP.

  Description:
     The host application has two basic options with respect to controlling how the
     MRF24WG handles a loss of WiFi connection.
        1) MRF24WG informs the host and automatically retries N times (or forever)
           to regain the connection
        2) MRF24WG simply informas the host application that the connection has
           been lost; it does not automatically try to regain the connection.
           Instead, it is up to the host to reestablish the connection.

  Parameters:
    retryCount    -- the number of times the MRF24WG should try to regain a connection:
                      0     -- Do not try to regain the connection (simply report event to host application)
                      1:254 -- number of times to try to regain the connection
                      255   -- Retry forever (WF_RETRY_FOREVER)

    deauthAction  -- WF_ATTEMPT_TO_RECONNECT or WF_DO_NOT_ATTEMPT_TO_RECONNECT

    beaconTimeOut -- Number of missed beacons before MRF24WG designates the
                     connection as lost:
                      0 -- MRF24WG will NOT monitor the beacon timeout condition
                            and will not indicate this condition to Host
                      1:255 -- number of missed beacons before connection declared lost

    beaconTimeoutAction -- WF_ATTEMPT_TO_RECONNECT or WF_DO_NOT_ATTEMPT_TO_RECONNECT

  Returns:
    None

  Remarks:
    If this function is not called, the MRF2WG default is the equivalent of:
        WF_SetReconnectMode(3, WF_ATTEMPT_TO_RECONNECT, 0, WF_DO_NOT_ATTEMPT_TO_RECONNECT);

    Examples of different scenarios are below.

    Example 1: MRF24WG should not do any connection retries and only report deauth events to host:
                 WF_SetReconnectMode(0, WF_DO_NOT_ATTEMPT_TO_RECONNECT, 0, WF_DO_NOT_ATTEMPT_TO_RECONNECT);

    Example 2: MRF24WG should not do any connection retries, but report deauth and beacon timeout events to host.
               Beacon timeout should be 5 beacon periods:
                 WF_SetReconnectMode(0, WF_DO_NOT_ATTEMPT_TO_RECONNECT, 5, WF_DO_NOT_ATTEMPT_TO_RECONNECT);

    Example 3: MRF24WG should ignore beacon timeouts, but attempt to reconnect 3 times if a deauth occurs:
                WF_SetReconnectMode(3, WF_ATTEMPT_TO_RECONNECT, 0, WF_DO_NOT_ATTEMPT_TO_RECONNECT);

    Example 4: MRF24WG should not do any connection retries if a deauth occcurs, but retry 3 times if a beacon
               timeout of 4 beacon periods occur:
                WF_SetReconnectMode(3, WF_DO_NOT_ATTEMPT_TO_RECONNECT, 4, WF_ATTEMPT_TO_RECONNECT);

    Example 5: MRF24WG should retry forever if either a deauth or beacon timeout occurs (beacon timeout is
               3 beacon periods):
                WF_SetReconnectMode(WF_RETRY_FOREVER, WF_ATTEMPT_TO_RECONNECT, 3, WF_ATTEMPT_TO_RECONNECT);

  *****************************************************************************/
void WF_ReconnectModeSet(uint8_t retryCount, uint8_t deauthAction, uint8_t beaconTimeout, uint8_t beaconTimeoutAction)
{
    LowLevel_CASetElement(WF_CA_ELEMENT_LIST_RETRY_COUNT,       // Element ID
                          &retryCount,                          // pointer to element data
                          sizeof(retryCount));                  // number of element data bytes

    LowLevel_CASetElement(WF_CA_ELEMENT_DEAUTH_ACTION,          // Element ID
                          &deauthAction,                        // pointer to element data
                          sizeof(deauthAction));                // number of element data bytes

    LowLevel_CASetElement(WF_CA_ELEMENT_BEACON_TIMEOUT,         // Element ID
                          &beaconTimeout,                       // pointer to element data
                          sizeof(beaconTimeout));               // number of element data bytes

    LowLevel_CASetElement(WF_CA_ELEMENT_BEACON_TIMEOUT_ACTION,  // Element ID
                          &beaconTimeoutAction,                 // pointer to element data
                          sizeof(beaconTimeoutAction));         // number of element data bytes
}

/*******************************************************************************
  Function:
    void WF_ReconnectModeGet(uint8_t *p_retryCount, uint8_t *p_deauthAction, uint8_t *p_beaconTimeout, uint8_t *p_beaconTimeoutAction);

  Summary:
    Gets the reconnection parameters

  Description:
     See WF_ReconnectModeSet()

  Parameters:
     See WF_ReconnectModeSet()

  Returns:
    None
*****************************************************************************/
void WF_ReconnectModeGet(uint8_t *p_retryCount, uint8_t *p_deauthAction, uint8_t *p_beaconTimeout, uint8_t *p_beaconTimeoutAction)
{
    LowLevel_CAGetElement(WF_CA_ELEMENT_LIST_RETRY_COUNT,       // Element ID
                          p_retryCount,                         // pointer to element data
                          1,                                    // number of element data bytes
                          true);                                // read data, free mgmt buf

    LowLevel_CAGetElement(WF_CA_ELEMENT_DEAUTH_ACTION,          // Element ID
                          p_deauthAction,                       // pointer to element data
                          1,                                    // number of element data bytes
                          true);                                // read data, free mgmt buf

    LowLevel_CAGetElement(WF_CA_ELEMENT_BEACON_TIMEOUT,         // Element ID
                          p_beaconTimeout,                      // pointer to element data
                          1,                                    // number of element data bytes
                          true);                                // read data, free mgmt buf

    LowLevel_CAGetElement(WF_CA_ELEMENT_BEACON_TIMEOUT_ACTION,  // Element ID
                          p_beaconTimeoutAction,                // pointer to element data
                          1,                                    // number of element data bytes
                          true);                                // read data, free mgmt buf
}

/*******************************************************************************
  Function:    
    void WF_ChannelListSet(uint8_t *p_channelList, uint8_t numChannels)

  Summary:
    Sets the channel list.

  Description:
    Sets the Channel List used by the Connection Algorithm.

  Precondition:
    MACInit must be called first. 

  Parameters:
    p_channelList - Pointer to channel list.
    numChannels   - Number of channels in p_channelList.  If set to 0, the
                     MRF24W will use all valid channels for the current 
                     regional domain.

  Returns:
    None.
      
  Remarks:
    None.
 *****************************************************************************/
void WF_ChannelListSet(uint8_t *p_channelList, uint8_t numChannels)
{
    LowLevel_CASetElement(WF_CA_ELEMENT_CHANNEL_LIST,  /* Element ID                   */
                          p_channelList,               /* pointer to element data      */
                          numChannels);                /* number of element data bytes */
}

/*******************************************************************************
  Function:    
    void WF_ChannelListGet(uint8_t *p_channelList, uint8_t *p_numChannels)

  Summary:
    Gets the channel list.

  Description:
    Gets the Channel List used by the Connection Algorithm.

  Precondition:
    MACInit must be called first.

  Parameters:
    p_channelList - Pointer to where channel list will be returned
    p_numChannels - Pointer to where number of channels in list will be 
                     returned

  Returns:
    None.
      
  Remarks:
    None.
 *****************************************************************************/
void WF_ChannelListGet(uint8_t *p_channelList, uint8_t *p_numChannels)
{
    tCAElementResponseHdr mgmtHdr;
    
    /* send request, wait for mgmt response, do not read and do not free up response buffer */
     LowLevel_CAGetElement(WF_CA_ELEMENT_CHANNEL_LIST,    /* Element ID      */
                           NULL,                          /* do not read     */
                           0,                             /* do not read     */
                           false);                        /* do not read, do not free mgmt buffer */

    /* at this point, management response is mounted and ready to be read */  

    /* read managment header */
    RawRead(RAW_MGMT_RX_ID, 0, sizeof(tCAElementResponseHdr), (uint8_t *)&mgmtHdr);
    
    /* extract data length (which will be channel list length) */
    *p_numChannels = mgmtHdr.elementDataLength;

    RawRead(RAW_MGMT_RX_ID, sizeof(tCAElementResponseHdr), *p_numChannels, p_channelList);
    
    /* free management buffer */
    DeallocateMgmtRxBuffer();
}

void WF_AdhocContextSet(t_adHocNetworkContext *p_context)
{
    uint16_t tmp;
    
    SetAdHocMode(p_context->mode);

    SetHiddenSsid(p_context->hiddenSsid);

    tmp = TCPIP_HELPER_htons(p_context->beaconPeriod);
    LowLevel_CASetElement(WF_CA_ELEMENT_BEACON_PERIOD,  // Element ID
                          (uint8_t *)&tmp,              // pointer to element data
                          sizeof(tmp));                 // number of element data bytes
}

void WF_AdhocContextGet(t_adHocNetworkContext *p_context)
{
    p_context->mode = GetAdHocMode();

    LowLevel_CAGetElement(WF_CA_ELEMENT_BEACON_PERIOD,        /* Element ID                   */
                         (uint8_t *)&p_context->beaconPeriod, /* pointer to element data      */
                          sizeof(p_context->beaconPeriod),    /* number of element data bytes */
                          true);                              /* read data, free buffer       */

    /* fix endianness before returning value */
    p_context->beaconPeriod = TCPIP_HELPER_ntohs(p_context->beaconPeriod);

    p_context->hiddenSsid = GetHiddenSsid();
}


/*******************************************************************************
  Function:    
    static void SetEventNotificationMask(uint8_t eventNotificationBitMask)

  Summary:
    Sets the event notification mask.

  Description:
    Sets the event notification mask for the Connection Algorithm.  Allowable
    values are:
    
    <table>
        Value   Event
        -----   -----
        0x01    WF_NOTIFY_CONNECTION_ATTEMPT_SUCCESSFUL
        0x02    WF_NOTIFY_CONNECTION_ATTEMPT_FAILED
        0x04    WF_NOTIFY_CONNECTION_TEMPORARILY_LOST
        0x08    WF_NOTIFY_CONNECTION_PERMANENTLY_LOST
        0x10    WF_NOTIFY_CONNECTION_REESTABLISHED
        0x1f    WF_NOTIFY_ALL_EVENTS
    </table>

  Precondition:
    MACInit must be called first.
      
  Parameters:
    eventNotificationBitMask - Bit mask defining which events the host will be
                               notified of.

  Returns:
    None.
      
  Remarks:
    None.
  *****************************************************************************/
static void SetEventNotificationMask(uint8_t eventNotificationBitMask)
{
    g_EventNotificationAction = eventNotificationBitMask;
}    

/*******************************************************************************
  Function:    
    uint8_t GetEventNotificationMask(void)

  Summary:
    Gets the event notification mask.

  Description:
    Gets the event notification mask for the Connection Algorithm.  Retruned
    values are:

    <table>
        Value   Event
        -----   -----
        0x01    WF_NOTIFY_CONNECTION_ATTEMPT_SUCCESSFUL
        0x02    WF_NOTIFY_CONNECTION_ATTEMPT_FAILED
        0x04    WF_NOTIFY_CONNECTION_TEMPORARILY_LOST
        0x08    WF_NOTIFY_CONNECTION_PERMANENTLY_LOST
        0x10    WF_NOTIFY_CONNECTION_REESTABLISHED
        0x1f    WF_NOTIFY_ALL_EVENTS
    </table>

  Precondition:
    MACInit must be called first.

  Parameters:
    None.

  Returns:
    A uint8_t of the event notification bit mask.
      
  Remarks:
    None.
  *****************************************************************************/
uint8_t GetEventNotificationMask(void)
{
    return g_EventNotificationAction;
}

void SetListenInterval(uint16_t listenInterval)
{

    /* correct endianness before sending message */
    listenInterval = TCPIP_HELPER_htons(listenInterval);
    LowLevel_CASetElement(WF_CA_ELEMENT_LISTEN_INTERVAL,    /* Element ID                   */
                         (uint8_t *)&listenInterval,        /* pointer to element data      */
                          sizeof(listenInterval));          /* number of element data bytes */
}

void SetDtimInterval(uint16_t dtimInterval)
{
    /* correct endianness before sending message */
    dtimInterval = TCPIP_HELPER_htons(dtimInterval);
    LowLevel_CASetElement(WF_CA_ELEMENT_DTIM_INTERVAL,    /* Element ID                   */
                          (uint8_t *)&dtimInterval,       /* pointer to element data      */
                          sizeof(dtimInterval));          /* number of element data bytes */
}

/*******************************************************************************
  Function:    
    static void LowLevel_CASetElement(uint8_t elementId, 
                                      uint8_t *p_elementData, 
                                      uint8_t elementDataLength)

  Summary:
    Set an element of the connection algorithm on the MRF24W.

  Description:
    Low-level function to send the appropriate management message to the
    MRF24W to set the Connection Algorithm element.
    
  Precondition:
    MACInit must be called first.

  Parameters:
    elementId - Element that is being set
    p_elementData - Pointer to element data
    elementDataLength - Number of bytes pointed to by p_elementData

  Returns:
    None.
    
  Remarks:
    All Connection Algorithm 'Set Element' functions call this function
    to construct the management message.  The caller must fix up any endian
    issues prior to calling this function.
 *****************************************************************************/
static void LowLevel_CASetElement(uint8_t elementId, 
                                  uint8_t *p_elementData, 
                                  uint8_t elementDataLength)
{
    uint8_t  hdrBuf[4];

    hdrBuf[0] = WF_MGMT_REQUEST_TYPE;           /* indicate this is a mgmt msg     */
    hdrBuf[1] = WF_CA_SET_ELEMENT_SUBTYPE;      /* mgmt request subtype            */     
    hdrBuf[2] = elementId;                      /* Element ID                      */
    hdrBuf[3] = elementDataLength;              /* number of bytes of element data */
    
    SendMgmtMsg(hdrBuf,
                sizeof(hdrBuf),
                p_elementData,
                elementDataLength);
    
    /* wait for mgmt response, free after it comes in, don't need data bytes */
    WaitForMgmtResponse(WF_CA_SET_ELEMENT_SUBTYPE, FREE_MGMT_BUFFER);
}   

/*******************************************************************************
  Function:    
    static void LowLevel_CAGetElement(uint8_t elementId, 
                                      uint8_t *p_elementData, 
                                      uint8_t elementDataLength,
                                      uint8_t dataReadAction)

  Summary:
    Get an element of the connection algorithm on the MRF24W.

  Description:
    Low-level function to send the appropriate management message to the
    MRF24W to get the Connection Algorithm element.

  Precondition:
    MACInit must be called first.

  Parameters:
    elementId - Element that is being read
    p_elementData - Pointer to where element data will be written
    elementDataLength - Number of element data bytes that will be read
    dataReadAction - If true then read data per paramters and free mgmt response buffer.
                      If false then return after response received, do not read any data as the 
                      caller will do that, and don't free buffer, as caller will do that as well.

  Returns:
    None.

  Remarks:
    All Connection Algorithm 'Get Element' functions call this function to 
    construct the management message.  The caller must fix up any endian issues
    after getting the data from this function.
 *****************************************************************************/
static void LowLevel_CAGetElement(uint8_t elementId, 
                                  uint8_t *p_elementData, 
                                  uint8_t elementDataLength,
                                  uint8_t dataReadAction)    /* true or false */
{
    uint8_t  hdrBuf[4];
            
    hdrBuf[0] = WF_MGMT_REQUEST_TYPE;       /* indicate this is a mgmt msg     */
    hdrBuf[1] = WF_CA_GET_ELEMENT_SUBTYPE;  /* mgmt request subtype            */     
    hdrBuf[2] = elementId;                  /* Element ID                      */
    hdrBuf[3] = 0;                          /* not used                        */

    SendMgmtMsg(hdrBuf,
                sizeof(hdrBuf),
                NULL,
                0);

    if (dataReadAction == (uint8_t)true)
    {
        /* wait for mgmt response, read desired data, and then free response buffer */
        WaitForMgmtResponseAndReadData(WF_CA_GET_ELEMENT_SUBTYPE, 
                                                    elementDataLength,                   /* num data bytes to read                */
                                                    sizeof(tCAElementResponseHdr),       /* index of first byte of element data   */
                                                    p_elementData);                      /* where to write element data           */
    }
    else
    {
        /* wait for mgmt response, don't read any data bytes, do not release mgmt buffer */
        WaitForMgmtResponse(WF_CA_GET_ELEMENT_SUBTYPE, DO_NOT_FREE_MGMT_BUFFER);
    }                                                    
}

  

#endif /* TCPIP_IF_MRF24W */

