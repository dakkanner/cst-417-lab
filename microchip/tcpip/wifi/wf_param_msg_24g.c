/*******************************************************************************
  MRF24W Driver Managment Set/Get Param messages (specific to the MRF24WG)

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Provides access to MRF24W WiFi controller
    -Reference: MRF24W Data sheet, IEEE 802.11 Standard
*******************************************************************************/

/*******************************************************************************
FileName:   wf_param_msg_24.g.c
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


/*
*********************************************************************************************************
*                                           DEFINES                               
*********************************************************************************************************
*/

#define MSG_PARAM_START_DATA_INDEX          (6)
#define MULTICAST_ADDRESS                   (6)
#define ADDRESS_FILTER_DEACTIVATE           (0)

#define ENABLE_MRF24W                       (1)

/*
*********************************************************************************************************
*                                           LOCAL DATA TYPES
*********************************************************************************************************
*/

typedef struct  multicastFilterMsgStruct
{
    uint8_t filterId;
    uint8_t filterAction;
    uint8_t macAddress[6];
    uint8_t macBitMask;
     
} tMulticastFilterMsg; 

/*
*********************************************************************************************************
*                                           LOCAL GLOBALS
*********************************************************************************************************
*/


/*******************************************************************************
  Function:    
    void WFEnableMRF24WMode(void)

  Summary:
    Must be called to configure the MRF24W for operations.

  Description:

  Precondition:
    MACInit must be called first.

  Parameters:
    None.

  Returns:
    None.
      
  Remarks:
    None.
 *****************************************************************************/
void WFEnableMRF24WMode(void)
{
    uint8_t buf[1] = {ENABLE_MRF24W};
    
    SendSetParamMsg(PARAM_MRF24W, buf, sizeof(buf)); 
}    

/*******************************************************************************
  Function:    
    void WFEnableBroadcastProbeResponse(void)

  Summary:
    This allows MRF24W to broadcast probe response in Adhoc mode

  Description:

  Precondition:
    MACInit must be called first.

  Parameters:
    None.

  Returns:
    None.
      
  Remarks:
    None.
  *****************************************************************************/

void WFEnableBroadcastProbeResponse(void)
{
    uint8_t buf[1] = {1};
    
    SendSetParamMsg(PARAM_BROADCAST_PROBE_RESPONSE, buf, sizeof(buf)); 
}   

#if 0
/*******************************************************************************
  Function:    
  WF_EnableDebugPrint(uint8_t option)

  Summary:
    Can be called to enable printfs for WPS & P2P

  Description:

  Precondition:
  MACInit must be called first.

  Parameters:
    option -- has option value to enable the printfs

  Returns:
    None.
      
  Remarks:
    None.
 *****************************************************************************/
void WF_EnableDebugPrint(uint8_t option)
{ 
    SendSetParamMsg(PARAM_ENABLE_DEBUG_PRINT, &option, sizeof(option)); 
}  
#endif // 0

/*******************************************************************************
  Function:    
  WF_SetLinkDownThreshold(uint8_t threshold)

  Summary:
    Sets number of consecutive WiFi Tx failures before link is considered down.
    
  Description:
    This function allows the application to set the number of MRF24W consecutive Tx failures
    before the connection failure event (WF_LINK_DOWN) is reported to the host application.  
    Range:
       0 -- disabled (default)
       1:255 -- number of consecutive Tx failures before connection failure event is reported

  Precondition:
  MACInit must be called first.

  Parameters:
    threshold --  0:      disabled (default)
                  1-255:  number of consecutive Tx failures before connection failure event is reported
  Returns:
    None.
      
  Remarks:
    None.
 *****************************************************************************/

void WF_SetLinkDownThreshold(uint8_t threshold)
{ 
    SendSetParamMsg(PARAM_LINK_DOWN_THRESHOLD, &threshold, sizeof(threshold)); 
}

/*******************************************************************************
  Function:    
  WF_SetStackVersion(uint8_t major, uint8_t minor)

  Summary:
    Can be called to notify stack version to RF FW

  Description:

  Precondition:
  MACInit must be called first.

  Parameters:
    major -- major version number
    minor -- minor version number

  Returns:
    None.
      
  Remarks:
    None.
 *****************************************************************************/

void WF_SetStackVersion(uint8_t major, uint8_t minor)
{ 
    uint16_t version;

    version = minor << 8;
    version |= major;
    
    SendSetParamMsg(PARAM_STACK_VERSION, (uint8_t *)&version, sizeof(version)); 
}  
  

/*******************************************************************************
  Function:    
  WF_SetWPATimeout(uint32_t timeout)

  Summary:
    Can be called to configure WPA transaction timeout

  Description:

  Precondition:
  MACInit must be called first.

  Parameters:
    timeout -- has transaction timeout value by us unit

  Returns:
    None.
      
  Remarks:
    None.
 *****************************************************************************/
void WF_SetWPATimeout(uint32_t timeout)
{ 
    SendSetParamMsg(PARAM_WPA_TIMEOUT, (uint8_t *)&timeout, sizeof(timeout)); 
}  

/*******************************************************************************
  Function:    
  void WF_GetWPATimeout(uint32_t *timeout)

  Summary:
    Retrives WPA transaction timeout value

  Description:

  Precondition:
  MACInit must be called first.

  Parameters:
    timeout -- pointer to location to store the timeout value

  Returns:
    None.
      
  Remarks:
    None.
 *****************************************************************************/
void WF_GetWPATimeout(uint32_t *timeout)
{ 
    SendGetParamMsg(PARAM_WPA_TIMEOUT, (uint8_t *)timeout, sizeof(*timeout)); 
}  

/*******************************************************************************
  Function:    
  WF_SetTxMode(uint8_t mode)

  Summary:
    Configures 802.11 Tx mode

  Description:

  Precondition:
    MACInit must be called first.

  Parameters:
    mode -- Tx mode value.  Choices are:
             WF_TXMODE_G_RATES (default) -- will use all 802.11g rates
             WF_TXMODE_B_RATES           -- will only use 802.11b rates
             WF_TXMODE_LEGACY_RATES      -- will only use 1 and 2 mbps rates 

  Returns:
    None.
      
  Remarks:
    None.
 *****************************************************************************/
void WF_SetTxMode(uint8_t mode)
{ 
    SendSetParamMsg(PARAM_TX_MODE, (uint8_t *)&mode, 1); 
}  

/*******************************************************************************
  Function:    
  void WF_GetTxMode(uint8_t *mode)

  Summary:
    Retrieves tx mode value

  Description:

  Precondition:
    MACInit must be called first.

  Parameters:
    p_mode -- pointer to location to store the mode value

  Returns:
    None.
      
  Remarks:
    None.
 *****************************************************************************/
void WF_GetTxMode(uint8_t *p_mode)
{ 
    SendGetParamMsg(PARAM_TX_MODE, p_mode, 1); 
}  

/*******************************************************************************
  Function:    
    void WF_GetDeviceInfo(t_wfDeviceInfo *p_deviceInfo)

  Summary:
    Retrieves WF device information

  Description:

  Precondition:
    MACInit must be called first.

  Parameters:
    p_deviceInfo - Pointer where device info will be written

  Returns:
    None.
      
  Remarks:
    None.
 *****************************************************************************/
void WF_GetDeviceInfo(t_wfDeviceInfo *p_deviceInfo)
{
    uint8_t  msgData[2];

    SendGetParamMsg(PARAM_SYSTEM_VERSION, msgData, sizeof(msgData));
    
    p_deviceInfo->deviceType   = 0xff;
    p_deviceInfo->romVersion   = msgData[0];
    p_deviceInfo->patchVersion = msgData[1];
    
    if (p_deviceInfo->romVersion == 0x12)
    {
        p_deviceInfo->deviceType = MRF24WB0M_DEVICE;         
    }       
    else if (p_deviceInfo->romVersion == 0x30 || p_deviceInfo->romVersion == 0x31)
    {
        p_deviceInfo->deviceType = MRF24WG0M_DEVICE;  /* need part number */               
    }   
    else
    {
        SYS_ASSERT(false, "");
    }     
}


/*******************************************************************************
  Function:    
    void WF_MacAddressSet(uint8_t *p_mac)

  Summary:
    Uses a different MAC address for the MRF24W

  Description:
    Directs the MRF24W to use the input MAC address instead of its
    factory-default MAC address.  This function does not overwrite the factory 
    default, which is in FLASH memory – it simply tells the MRF24W to use a
    different MAC.

  Precondition:
    MACInit must be called first.  Cannot be called when the MRF24W is in a
    connected state.

  Parameters:
    p_mac  - Pointer to 6-byte MAC that will be sent to MRF24W

  Returns:
    None.
      
  Remarks:
    None.
 *****************************************************************************/
void WF_MacAddressSet(uint8_t *p_mac)
{
    SendSetParamMsg(PARAM_MAC_ADDRESS, p_mac, WF_MAC_ADDRESS_LENGTH); 
}   

/*******************************************************************************
  Function:    
    void WF_MacAddressGet(uint8_t *p_mac)

  Summary:
    Retrieves the MRF24W MAC address

  Description:

  Precondition:
    MACInit must be called first.

  Parameters:
    p_mac  - Pointer where mac will be written (must point to a 6-byte buffer)

  Returns:
    None.
      
  Remarks:
    None.
 *****************************************************************************/
void WF_MacAddressGet(uint8_t *p_mac)
{
    SendGetParamMsg(PARAM_MAC_ADDRESS, p_mac, WF_MAC_ADDRESS_LENGTH);
}   

/*******************************************************************************
  Function:	
    void WF_SetMultiCastFilter(uint8_t multicastFilterId, 
                               uint8_t multicastAddress[6])

  Summary:
    Sets a multicast address filter using one of the two multicast filters.

  Description:
    This function allows the application to configure up to two Multicast 
    Address Filters on the MRF24W.  If two active multicast filters are set 
    up they are OR’d together – the MRF24W will receive and pass to the Host 
    CPU received packets from either multicast address.  
    The allowable values for the multicast filter are:
    * WF_MULTICAST_FILTER_1
    * WF_MULTICAST_FILTER_2

    By default, both Multicast Filters are inactive.

  Precondition:
  	MACInit must be called first.

  Parameters:
    multicastFilterId - WF_MULTICAST_FILTER_1 or WF_MULTICAST_FILTER_2
    multicastAddress  - 6-byte address (all 0xFF will inactivate the filter)

  Returns:
  	None.
  	
  Remarks:
  	None.
  *****************************************************************************/
void WF_SetMultiCastFilter(uint8_t multicastFilterId,
                           uint8_t multicastAddress[6])
{
    int i;
    bool deactivateFlag = true;
    uint8_t msgData[8];
    
    SYS_ASSERT( ((multicastFilterId == WF_MULTICAST_FILTER_1) || (multicastFilterId == WF_MULTICAST_FILTER_2)), "" );
    
    /* check if all 6 bytes of the address are 0xff, implying that the caller wants to deactivate */
    /* the multicast filter.                                                                      */
    for (i = 0; i < 6; ++i)
    {
        /* if any byte is not 0xff then a presume a valid multicast address */
        if (multicastAddress[i] != 0xff)
        {
            deactivateFlag = false;
            break;
        }    
    }  
    
    msgData[0] = multicastFilterId;     /* Address Compare Register number to use   */
    if (deactivateFlag)
    {
        msgData[1] = ADDRESS_FILTER_DEACTIVATE;
    }    
    else
    {
        msgData[1] = MULTICAST_ADDRESS;     /* type of address being used in the filter */  
    }
    memcpy(&msgData[2], (void *)multicastAddress, WF_MAC_ADDRESS_LENGTH);

    SendSetParamMsg(PARAM_COMPARE_ADDRESS, msgData, sizeof(msgData) ); 
}    

/*******************************************************************************
  Function:	
    void WF_GetMultiCastFilter(uint8_t multicastFilterId,
                               uint8_t multicastAddress[6])

  Summary:
    Gets a multicast address filter from one of the two multicast filters.

  Description:
    Gets the current state of the specified Multicast Filter.  

    Normally would call SendGetParamMsg, but this GetParam returns all 6 address 
    filters + 2 more bytes for a total of 48 bytes plus header. So, doing this 
    msg manually to not require a large stack allocation to hold all the data.                                                                                  
    
    Exact format of returned message is:                                                                
    [0]     -- always mgmt response (2)
    [1]     -- always WF_GET_PARAM_SUBTYPE (16)
    [2]     -- result (1 if successful)
    [3]     -- mac state (not used)
    [4]     -- data length (length of response data starting at index 6)
    [5]     -- not used
    [6-11]  -- Compare Address 0 address
    [12]    -- Compare Address 0 group
    [13]    -- Compare Address 0 type
    [14-19] -- Compare Address 1 address
    [20]    -- Compare Address 1 group
    [21]    -- Compare Address 1 type
    [22-27] -- Compare Address 2 address
    [28]    -- Compare Address 2 group
    [29]    -- Compare Address 2 type
    [30-35] -- Compare Address 3 address
    [36]    -- Compare Address 3 group
    [37]    -- Compare Address 3 type
    [38-43] -- Compare Address 4 address
    [44]    -- Compare Address 4 group
    [45]    -- Compare Address 4 type
    [46-51] -- Compare Address 5 address
    [52]    -- Compare Address 5 group
    [53]    -- Compare Address 5 type

  Precondition:
  	MACInit must be called first.

  Parameters:
    multicastFilterId - WF_MULTICAST_FILTER_1 or WF_MULTICAST_FILTER_2
    multicastAddress - 6-byte address

  Returns:
  	None.
  	
  Remarks:
  	None.
  *****************************************************************************/
void WF_GetMultiCastFilter(uint8_t multicastFilterId,
                           uint8_t multicastAddress[6])
{
    uint8_t  hdr[4];
    uint8_t  paramData[8];
    uint8_t  startIndex;

    SYS_ASSERT( ((multicastFilterId == WF_MULTICAST_FILTER_1) || (multicastFilterId == WF_MULTICAST_FILTER_2)), "" );

    
    hdr[0] = WF_MGMT_REQUEST_TYPE;
    hdr[1] = WF_GET_PARAM_SUBTYPE;
    hdr[2] = 0x00;                      /* MS 8 bits of param Id, always 0 */
    hdr[3] = PARAM_COMPARE_ADDRESS;     /* LS 8 bits of param ID           */
    
    SendMgmtMsg(hdr,             /* header           */
                sizeof(hdr),     /* size of header   */
                NULL,            /* no data          */
                0);              /* no data          */

    if (multicastFilterId == WF_MULTICAST_FILTER_1)
    {
        startIndex = 38; /* index of first byte of index 4 address filter */
    }    
    else
    {
        startIndex = 46; /* index of first byte of index 5 address filter */
    }    

	WaitForMgmtResponseAndReadData(WF_GET_PARAM_SUBTYPE,       /* expected subtype                           */ 
                                   sizeof(paramData),          /* num data bytes to read                     */
                                   startIndex,                 /* starting at this index                     */
                                   paramData);                 /* write the response data here               */
	
	
	memcpy((void *)multicastAddress, (void *)&paramData[0], 6);
}                                   

/*******************************************************************************
  Function:    
    void WF_MulticastSetConfig(t_wfMultiCastConfig *p_config);

  Summary:
    Sets a multicast address filter using one of the two multicast filters.

  Description:
    This function allows the application to configure up to two Multicast 
    Address Filters on the MRF24W.  If two active multicast filters are set
    up they are OR’d together – the MRF24W will receive and pass to the Host
    CPU received packets from either multicast address.  
    The allowable values in p_config are:
    
    filterId -- WF_MULTICAST_FILTER_1 or WF_MULTICAST_FILTER_2
    
    action   -- WF_MULTICAST_DISABLE_ALL (default) 
                   The Multicast Filter discards all received 
                   multicast messages – they will not be forwarded 
                   to the Host PIC.  The remaining fields in this 
                   structure are ignored. 
                   
                WF_MULTICAST_ENABLE_ALL
                   The Multicast Filter forwards all received multicast messages 
                   to the Host PIC. The remaining fields in this structure are 
                   ignored. 
                   
                WF_MULTICAST_USE_FILTERS
                   The MAC filter will be used and the remaining fields in this 
                   structure configure which Multicast messages are forwarded to 
                   the Host PIC.
                   
    macBytes -- Array containing the MAC address to filter on (using the destination 
                address of each incoming 802.11 frame).  Specific bytes with the 
                MAC address can be designated as ‘don’t care’ bytes.  See macBitMask.
                This field in only used if action = WF_MULTICAST_USE_FILTERS.
                
    macBitMask -- A byte where bits 5:0 correspond to macBytes[5:0].  If the bit is 
                  zero then the corresponding MAC byte must be an exact match for the 
                  frame to be forwarded to the Host PIC.  If the bit is one then the 
                  corresponding MAC byte is a ‘don’t care’ and not used in the 
                  Multicast filtering process.  This field in only used if 
                  action = WF_MULTICAST_USE_FILTERS.

    By default, both Multicast Filters are inactive.
    
    Example -- Filter on Multicast Address of 01:00:5e:xx:xx:xx where xx are don't care bytes.
                  p_config->filterId = WF_MULTICAST_FILTER_1 
                  
                                         [0] [1] [2] [3] [4] [5]
                  p_config->macBytes[] = 01, 00, 5e, ff, ff, ff  (0xff are the don't care bytes)
                  
                  p_config->macBitMask = 0x38 --> bits 5:3 = 1 (don't care on bytes 3,4,5)
                                              --> bits 2:0 = 0 (exact match required on bytes 0,1,2)
                  
  Precondition:
    MACInit must be called first.

  Parameters:
    p_config -- pointer to the multicast config structure.  See documentation.

  Returns:
    None.
      
  Remarks:
    None.
 *****************************************************************************/
 void WF_MulticastSetConfig(t_wfMultiCastConfig *p_config)
{
    tMulticastFilterMsg msg;
    uint8_t filterId;
    
    filterId = p_config->filterId;
    SYS_ASSERT((p_config->action <= WF_MULTICAST_USE_FILTERS), "");

    /* if want no multicast messages forwarded to the host PIC */ 
    if (p_config->action == WF_MULTICAST_DISABLE_ALL)
    {
        msg.filterId     = WF_MULTICAST_FILTER_1;
        msg.filterAction = ADDRESS_FILTER_DEACTIVATE;  
        memset(msg.macAddress, 0xff, WF_MAC_ADDRESS_LENGTH);   
        msg.macBitMask = 0x00;                                 /* don't care */
    }
    /* else if want all multicast messages forwarded to the host PIC */
    else if (p_config->action == WF_MULTICAST_ENABLE_ALL)    
    {
        msg.filterId     = WF_MULTICAST_FILTER_1;
        msg.filterAction = MULTICAST_ADDRESS;
        memcpy((void *)msg.macAddress, (void *)p_config->macBytes, WF_MAC_ADDRESS_LENGTH);
        msg.macBitMask = 0x3f;  /* don't care from host, but Ostrich needs to see this bitmask */
    }
    /* else if want a single multicast address or group of multicast addresses forwarded to Host PIC */
    else if (p_config->action == WF_MULTICAST_USE_FILTERS)
    {
        msg.filterId     = filterId;        
        msg.filterAction =  MULTICAST_ADDRESS;
        memcpy((void *)&msg.macAddress, (void *)p_config->macBytes, WF_MAC_ADDRESS_LENGTH);        
        msg.macBitMask = p_config->macBitMask; 
    }      
    /* invalid action */
    else
    {
        SYS_ASSERT(false, "");
    }    
   
    SendSetParamMsg(PARAM_COMPARE_ADDRESS, (uint8_t *)&msg, sizeof(msg) ); 
} 


/*******************************************************************************
  Function:    
    void WF_MulticastGetConfig(uint8_t filterId, t_wfMultiCastConfig *p_config);

  Summary:
    Gets a multicast address filter from one of the two multicast filters.

  Description:
    Gets the current state of the specified Multicast Filter. 

    Normally would call SendGetParamMsg, but this GetParam returns all 6 address 
    filters + 2 more bytes for a total of 48 bytes plus header. So, doing this 
    msg manually to not require a large stack allocation to hold all the data.                                                                                  
    
    Exact format of management message stored on device is:                                                                
    [0]     -- always mgmt response (2)
    [1]     -- always WF_GET_PARAM_SUBTYPE (16)
    [2]     -- result (1 if successful)
    [3]     -- mac state (not used)
    [4]     -- data length (length of response data starting at index 6)
    [5]     -- not used
    
    [6-11]  -- Compare Address 0 address
    [12]    -- Compare Address 0 group
    [13]    -- Compare Address 0 type
    [14]    -- Compare Address 0 macBitMask
    [15-17] -- Not used
    
    [18-23] -- Compare Address 1 address
    [24]    -- Compare Address 1 group
    [25]    -- Compare Address 1 type
    [26]    -- Compare Address 1 macBitMask
    [27-29] -- Not used
    
    [30-35] -- Compare Address 2 address
    [36]    -- Compare Address 2 group
    [37]    -- Compare Address 2 type
    [38]    -- Compare Address 2 macBitMask
    [39-41] -- Not used
    
    [42-47] -- Compare Address 3 address
    [48]    -- Compare Address 3 group
    [49]    -- Compare Address 3 type
    [50]    -- Compare Address 3 macBitMask
    [51-53] -- Not used
    
    [54-59] -- Compare Address 4 address
    [60]    -- Compare Address 4 group
    [61]    -- Compare Address 4 type
    [62]    -- Compare Address 4 macBitMask
    [63-65] -- Not used
    
    [66-71] -- Compare Address 5 address
    [72]    -- Compare Address 5 group
    [73]    -- Compare Address 5 type
    [74]    -- Compare Address 5 macBitMask
    [75-77] -- Not used

  Precondition:
    MACInit must be called first.  

  Parameters:
    filterId -- ID of filter being retrieved.  Must be:
                  WF_MULTICAST_FILTER_1 or WF_MULTICAST_FILTER_2
    
    p_config -- Pointer to config structure filled in by this function.

  Returns:
    None.
      
  Remarks:
    None.
 *****************************************************************************/
void WF_MulticastGetConfig(uint8_t filterId, t_wfMultiCastConfig *p_config)
{
    uint8_t  hdr[4];
    uint8_t  paramData[12];

    SYS_ASSERT( (filterId <= WF_MULTICAST_FILTER_16), "");

    hdr[0] = WF_MGMT_REQUEST_TYPE;
    hdr[1] = WF_GET_PARAM_SUBTYPE;
    hdr[2] = 0x00;                      /* MS 8 bits of param Id, always 0 */
    hdr[3] = PARAM_COMPARE_ADDRESS;     /* LS 8 bits of param ID           */
    
    SendMgmtMsg(hdr,             /* header              */
                sizeof(hdr),     /* size of header      */
                &filterId,       /* multicast filter id */
                1);              /* length is 1         */

    WaitForMgmtResponseAndReadData(WF_GET_PARAM_SUBTYPE,       /* expected subtype                           */ 
                                   sizeof(paramData),          /* num data bytes to read                     */
                                   MSG_PARAM_START_DATA_INDEX, /* starting at this index                     */
                                   paramData);                 /* write the response data here               */
    
    /* put param data into return structure */
    p_config->filterId = filterId;
    memcpy((void *)p_config->macBytes, (void *)&paramData[0], 6);
    p_config->action = paramData[7];
    p_config->macBitMask = paramData[8];
}                                   


/*******************************************************************************
  Function:    
    void WF_SetTxDataConfirm(uint8_t state)

  Summary:
    Enables or disables Tx data confirmation management messages.

  Description:
    Enables or disables the MRF24W Tx data confirm mgmt message.  Data
    confirms should always be disabled.

  Precondition:
    MACInit must be called first.

  Parameters:
    state - WF_DISABLED or WF_ENABLED

  Returns:
    None.

  Remarks:
    None.
 *****************************************************************************/
void WF_SetTxDataConfirm(uint8_t state)
{
    SendSetParamMsg(PARAM_CONFIRM_DATA_TX_REQ, &state, 1); 
}   

/*******************************************************************************
  Function:    
    void WF_GetTxDataConfirm(uint8_t *p_txDataConfirm)

  Summary:
    Retrives the current state of Tx data confirmation management messages.

  Description:

  Precondition:
    MACInit must be called first.

  Parameters:
    p_txDataConfirm - Pointer to location where Tx data confirmation state is

  Returns:
    None.
      
  Remarks:
    None.
 *****************************************************************************/
void WF_GetTxDataConfirm(uint8_t *p_txDataConfirm)
{    
    SendGetParamMsg(PARAM_CONFIRM_DATA_TX_REQ, p_txDataConfirm, 1);
}   


/*******************************************************************************
  Function:    
    void WF_RegionalDomainGet(uint8_t *p_regionalDomain)

  Summary:
    Retrieves the MRF24W Regional domain

  Description:
    Gets the regional domain on the MRF24W.  Allowable values are:
    * WF_DOMAIN_FCC     
    * WF_DOMAIN_IC      
    * WF_DOMAIN_ETSI    
    * WF_DOMAIN_SPAIN   
    * WF_DOMAIN_FRANCE  
    * WF_DOMAIN_JAPAN_A 
    * WF_DOMAIN_JAPAN_B

  Precondition:
    MACInit must be called first.

  Parameters:
    p_regionalDomain - Pointer where the regional domain value will be written

  Returns:
    None.
      
  Remarks:
    None.
 *****************************************************************************/
void WF_RegionalDomainGet(uint8_t *p_regionalDomain)
{
    SendGetParamMsg(PARAM_REGIONAL_DOMAIN, p_regionalDomain, 1);
}   

#if 0
/*******************************************************************************
  Function:    
    void WF_SetPromiscuousMode(bool action, uint8_t channel)

  Summary:
    Puts the WiFi MAC in or out of promiscuous mode

  Description:
    By default the MAC operates in 802.11 mode.  This function allows the application
    to put the MAC in promiscuous mode where it will send all packets received to the
    host.

  Precondition:
    MACInit must be called first.

  Parameters:
    action -- true to put device in promiscuous mode, false to take it out of 
              promiscuous mode.
    channel -- 802.11 channel to listen to (1 thru 11).  Only used if action is true               

  Returns:
    None.
      
  Remarks:
    None.
 *****************************************************************************/
#define WF_MASTER_STATE_802_11       (1)    /* default, MAC operates as 802.11 link layer        */
#define WF_MASTER_STATE_PROMISCUOUS  (2)    /* MAC operates in promiscuous mode and forwards all */
                                            /*  packets to host                                  */
#define WF_RADIO_ON                  (1)                                            
void WF_SetPromiscuousMode(bool action, uint8_t channel)
{
    uint8_t masterState;
    uint8_t buf[2];
    
    if (action == true)
    {
        /* enable the radio and select the desired channel */
        buf[0] = WF_RADIO_ON;
        buf[1] = channel;
        SendSetParamMsg(PARAM_ON_OFF_RADIO, buf, 2);
        
        masterState = WF_MASTER_STATE_PROMISCUOUS;
        SendSetParamMsg(PARAM_MASTER_STATE, &masterState, 1);    
    }
    else
    {
        masterState = WF_MASTER_STATE_802_11;
        SendSetParamMsg(PARAM_MASTER_STATE, &masterState, 1);    
    }       
}   
#endif 


/*******************************************************************************
  Function:    
    void WF_RtsThresholdSet(uint16_t rtsThreshold)

  Summary:
    Sets the RTS Threshold.

  Description:
    Sets the RTS/CTS packet size threshold for when RTS/CTS frame will be sent.  
    The default is 2347 bytes – the maximum for 802.11.  It is recommended that 
    the user leave the default at 2347 until they understand the performance and 
    power ramifications of setting it smaller.  Valid values are from 0 to
    WF_RTS_THRESHOLD_MAX (2347).

  Precondition:
    MACInit must be called first.

  Parameters:
    rtsThreshold - Value of the packet size threshold

  Returns:
    None.
      
  Remarks:
    None.
 *****************************************************************************/
void WF_RtsThresholdSet(uint16_t rtsThreshold)
{
    SYS_ASSERT((rtsThreshold <= WF_RTS_THRESHOLD_MAX), "");
    
     /* correct endianness before sending message */
    rtsThreshold = TCPIP_HELPER_htons(rtsThreshold);
    
    SendSetParamMsg(PARAM_RTS_THRESHOLD, (uint8_t *)&rtsThreshold, sizeof(rtsThreshold)); 
} 

/*******************************************************************************
  Function:    
    void WF_RtsThresholdGet(uint16_t *p_rtsThreshold)

  Summary:
    Gets the RTS Threshold

  Description:
    Gets the RTS/CTS packet size threshold.  

  Precondition:
    MACInit must be called first.

  Parameters:
    p_rtsThreshold - Pointer to where RTS threshold is written

  Returns:
    None.
      
  Remarks:
    None.
 *****************************************************************************/
void WF_RtsThresholdGet(uint16_t *p_rtsThreshold)
{
    SendGetParamMsg(PARAM_RTS_THRESHOLD, (uint8_t *)p_rtsThreshold, sizeof(uint16_t)); 

     /* correct endianness before sending message */
    *p_rtsThreshold = TCPIP_HELPER_htons(*p_rtsThreshold);
} 

/*******************************************************************************
  Function:    
    void WF_EnableSWMultiCastFilter(void)

  Summary:
    Forces the module FW to use software filter instead of hardware filter

  Description:
    This function allows the application to configure up to max 16 Multicast 
    Address Filters on the MRF24WG0MA/B. 
    
  Precondition:
    MACInit must be called first.

  Parameters:
    None.

  Returns:
    None.
      
  Remarks:
    None.
  *****************************************************************************/
void WF_EnableSWMultiCastFilter(void)
{
    UINT8 enable = 1;

    SendSetParamMsg(PARAM_USE_SW_MULTICAST_FILTER, (UINT8 *)&enable, sizeof(enable)); 
}

/*******************************************************************************
  Function:    
    void WF_GetMacStats(t_wfMacStats *p_macStats)

  Summary:
    Gets MAC statistics.  

  Description:

  Precondition:
    MACInit must be called first.

  Parameters:
    p_macStats - Pointer to where MAC statistics are written

  Returns:
    None.
      
  Remarks:
    None.
  *****************************************************************************/
void WF_GetMacStats(t_wfMacStats *p_macStats)
{
    uint32_t *p_value;
    uint8_t  numElements;
    uint8_t  i;
    
    SendGetParamMsg(PARAM_STAT_COUNTERS, (uint8_t *)p_macStats, sizeof(t_wfMacStats));
    
    /* calculate number of 32-bit counters in the stats structure and point to first element */
    numElements = sizeof(t_wfMacStats) / sizeof(uint32_t);
    p_value = (uint32_t *)p_macStats;
    
    /* correct endianness on all counters in structure */    
    for (i = 0; i < numElements; ++i)
    {
        *p_value = TCPIP_HELPER_ntohl(*p_value);
        ++p_value;        
    }    
}

/*******************************************************************************
  Function:
    void WF_YieldPassphrase2Host(void)

  Summary:
    Allows host to handle WPS WPA-PSK passphrase

  Description:
     Allows host to convert pass phrase to key in WPS WPA-PSK

  Precondition:
    MACInit must be called first.

  Parameters:
       None

  Returns:
    None.

  Remarks:
    None.
 *****************************************************************************/
void WF_YieldPassphrase2Host(void)
{
    UINT8 yield = 1;

    SendSetParamMsg(PARAM_YIELD_PASSPHRASE_TOHOST, (UINT8 *)&yield, sizeof(yield));
}

/*******************************************************************************
  Function:
    void WF_SetPSK(UINT8 *psk)

  Summary:
    Set PSK to module FW

  Description:
     Set PSK to module FW in WPS WPA-PSK

  Precondition:
    MACInit must be called first.

  Parameters:
       None

  Returns:
    None.

  Remarks:
    None.
 *****************************************************************************/
void WF_SetPSK(UINT8 *psk)
{
    SendSetParamMsg(PARAM_SET_PSK, (UINT8 *)psk, 32);
}

/*******************************************************************************
  Function:    
    void SendSetParamMsg(uint8_t paramType, 
                         uint8_t *p_paramData, 
                         uint8_t paramDataLength)

  Summary:
    Sends a SetParam Mgmt request to MRF24W and waits for response.

  Description:
    Index Set Param Request
    ----- -----------------
    0     type            (always 0x02 signifying a mgmt request)
    1     subtype         (always 0x10 signifying a Set Param Msg)
    2     param ID [msb]  (MS byte of parameter ID being requested, e.g. 
                           PARAM_SYSTEM_VERSION)
    3     param ID [lsb]  (LS byte of parameter ID being requested. e.g. 
                           PARAM_SYSTEM_VERSION)
    4     payload[0]      first byte of param data
    N     payload[n]      Nth byte of payload data
            
    Index  Set Param Response
    ------ ------------------
    0      type           (always 0x02 signifying a mgmt response)
    1      subtype        (always 0x10 signifying a Param Response Msg
    2      result         (1 if successful -- any other value indicates failure
    3      mac state      (not used)

  Precondition:
    MACInit must be called first.

  Parameters:
    paramType - Parameter type associated with the SetParam msg.
    p_paramData - pointer to parameter data
    paramDataLength - Number of bytes pointed to by p_paramData

  Returns:
    None.

  Remarks:
    None.
 *****************************************************************************/
void SendSetParamMsg(uint8_t paramType, 
                     uint8_t *p_paramData, 
                     uint8_t paramDataLength)
{
    uint8_t hdr[4];
    
    hdr[0] = WF_MGMT_REQUEST_TYPE;
    hdr[1] = WF_SET_PARAM_SUBTYPE;
    hdr[2] = 0x00;                      /* MS 8 bits of param Id, always 0 */
    hdr[3] = paramType;                 /* LS 8 bits of param ID           */

    SendMgmtMsg(hdr,               /* header            */
                sizeof(hdr),       /* size of header    */
                p_paramData,       /* param data        */
                paramDataLength);  /* param data length */

       /* wait for MRF24W management response; free response because not needed */
    WaitForMgmtResponse(WF_SET_PARAM_SUBTYPE, FREE_MGMT_BUFFER); 
}    

/*******************************************************************************
  Function:    
    void SendGetParamMsg(uint8_t paramType, 
                         uint8_t *p_paramData, 
                         uint8_t paramDataLength)

  Summary:
    Sends a GetParam Mgmt request to MRF24W and waits for response.

  Description:  
    After response is received the param data is read from message and written 
    to p_paramData.  It is up to the caller to fix up endianness.
     
    Index Get Param Request
    ----- -----------------
    0     type            (always 0x02 signifying a mgmt request)
    1     subtype         (always 0x10 signifying a Get Param Msg)
    2     param ID [msb]  (MS byte of parameter ID being requested, e.g. 
                           PARAM_SYSTEM_VERSION)
    3     param ID [lsb]  (LS byte of parameter ID being requested, e.g. 
                           PARAM_SYSTEM_VERSION)
           
    Index  Get Param Response
    ------ ------------------
    0      type           (always 0x02 signifying a mgmt response)
    1      subtype        (always 0x10 signifying a Param Response Msg
    2      result         (1 if successful -- any other value indicates failure
    3      mac state      (not used)
    4      data length    Length of response data starting at index 6 (in bytes)
    5      not used         
    6      Data[0]        first byte of returned parameter data
    N      Data[N]        Nth byte of param data

  Precondition:
    MACInit must be called first.

  Parameters:

  Returns:
    None.
      
  Remarks:
    None.
 *****************************************************************************/
void SendGetParamMsg(uint8_t paramType, 
                     uint8_t *p_paramData, 
                     uint8_t paramDataLength)
{
    uint8_t hdr[4];
    
    hdr[0] = WF_MGMT_REQUEST_TYPE;
    hdr[1] = WF_GET_PARAM_SUBTYPE;
    hdr[2] = 0x00;                      /* MS 8 bits of param Id, always 0 */
    hdr[3] = paramType;                 /* LS 8 bits of param ID           */
    
    SendMgmtMsg(hdr,             /* header           */
                sizeof(hdr),     /* size of header   */
                NULL,            /* no data          */
                0);              /* no data          */

    WaitForMgmtResponseAndReadData(WF_GET_PARAM_SUBTYPE,       /* expected subtype                           */ 
                                   paramDataLength,            /* num data bytes to read                     */
                                   MSG_PARAM_START_DATA_INDEX, /* data for GetParam always starts at index 6 */
                                   p_paramData);               /* write the response data here               */
}    
#endif /* TCPIP_IF_MRF24W */
