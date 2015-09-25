/*******************************************************************************
  MRF24W Driver Initialization

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Provides access to MRF24W WiFi controller
    -Reference: MRF24W Data sheet, IEEE 802.11 Standard
*******************************************************************************/

/*******************************************************************************
FileName:  wf_init.c 
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

/*
*********************************************************************************************************
*                                           INCLUDES                               
*********************************************************************************************************
*/

#include "tcpip_private.h"
#include "wf_mac.h"

#if defined(TCPIP_IF_MRF24W)
#include "wf_debug_output.h"
#include "wf_configData.h"
#if defined(TCPIP_STACK_USE_EZ_CONFIG)
#include "wf_easy_config.h"
#endif

/*
*********************************************************************************************************
*                                           DEFINES                               
*********************************************************************************************************
*/

#define EXPECTED_MRF24W_VERSION_NUMBER      (2)

/* This MAC address is the default MAC address used in tcpip_config.h.  If the */
/* user leaves this MAC address unchanged then the WiFi Driver will get the   */
/* unique MAC address from the MRF24W and have the stack use it.           */
#define MCHP_DEFAULT_MAC_ADDRESS_BYTE_1     (0x00)
#define MCHP_DEFAULT_MAC_ADDRESS_BYTE_2     (0x04)
#define MCHP_DEFAULT_MAC_ADDRESS_BYTE_3     (0xa3)
#define MCHP_DEFAULT_MAC_ADDRESS_BYTE_4     (0x00)
#define MCHP_DEFAULT_MAC_ADDRESS_BYTE_5     (0x00)
#define MCHP_DEFAULT_MAC_ADDRESS_BYTE_6     (0x00)

#if defined(WF_EVENT_DRIVEN)
// Initialize SM states
typedef enum
{
    I_INIT_BEGIN          = 0,
    I_CHIP_RESET          = 1,
    I_RAW_INIT            = 2,
    I_SEND_INIT_MGMT_MSGS = 4,
    I_CONNECT             = 5,
    I_INIT_COMPLETE       = 6

} t_initStates;

typedef enum 
{
    CR_BEGIN                = 0,
    CR_WAIT_HW_RESET        = 1,
    CR_WAIT_FIFO_BYTE_COUNT = 2,
} t_chipResetStates;

#endif // WF_EVENT_DRIVEN


/*
*********************************************************************************************************
*                                           LOCAL GLOBAL VARIABLES                               
*********************************************************************************************************
*/

/* This MAC address is the default MAC address used in tcpip_config.h.  If the */
/* user leaves this MAC address unchanged then the WiFi Driver will get the  */
/* unique MAC address from the MRF24W and have the stack use it.              */
static const uint8_t MchpDefaultMacAddress[WF_MAC_ADDRESS_LENGTH] = {0x00u, 0x04u, 0xA3u, 0x00u, 0x00u, 0x00u};

#if defined(WF_EVENT_DRIVEN)
static uint8_t g_initState;
static uint8_t g_chipResetState;
static bool    g_chipResetTimeout;
#endif // WF_EVENT_DRIVEN

/*
*********************************************************************************************************
*                                           LOCAL FUNCTION PROTOTYPES                              
*********************************************************************************************************
*/

 void WF_LibInitialize(TCPIP_NET_IF* pNetIf);

#if defined(WF_EVENT_DRIVEN)
int WF_InitStateMachine(void);
extern void ResetPll(void);
    #if defined(SYS_CONSOLE_ENABLE)
        extern void ValidateConfig(void);
    #endif
static int ChipResetStateMachine(void);
static void ChipResetTimeoutCallback(SYS_TICK currSysTick);
static void InitWiFiInterrupts(void);
static void HostInterrupt2RegInit(uint16_t hostIntMaskRegMask, uint8_t state);
static void HostInterruptRegInit(uint8_t hostIntrMaskRegMask,  uint8_t state);
#endif // WF_EVENT_DRIVEN




extern void SetAppPowerSaveMode(bool state);
extern void WF_CASetEventNotificationAction(uint8_t eventNotificationAction);
extern void WF_CPCreate(void);
void WiFiMemory_write(uint8_t *buf, uint32_t size);
void WiFiMemory_read(uint8_t *buf, uint32_t size);

void SetDhcpProgressState(void);

#if defined(WF_EVENT_DRIVEN)
/******************************************************************************
 * Function:        void WRF_InitStateMachine(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          success, failure, or still initializing (busy)
 *
 * Side Effects:    None
 *
 * Overview:        Called from MRF24W_MACInit().  Performs the WiFi driver
 *                  initialization, resets the MRF24WG, and configures MRF24WG
 *                  for operations.
 *
 * Note:            None
 *****************************************************************************/
int WF_InitStateMachine(void)
{
    int retCode = TCPIP_MAC_RES_PENDING;
    t_wfDeviceInfo deviceInfo;

    WiFiAsyncTask();    // check for and process events.  The async handler is
                        // not invoked during initialization at a system level, so
                        // it needs to be done here

    switch (g_initState)
    {
        //----------------------------------------
        case I_INIT_BEGIN:
        //----------------------------------------
            #if defined(SYS_CONSOLE_ENABLE)
            ValidateConfig();
            #endif
            if (!WF_ConfigDataLoad())
            {
                retCode = TCPIP_MAC_RES_INIT_FAIL;
            }

            /* initialize the SPI interface */
            if(!WF_SpiInit())
            {
                retCode = TCPIP_MAC_RES_INIT_FAIL;
            }

            /* Toggle the module into and then out of hibernate */
            WF_SetCE_N(WF_HIGH); /* disable module */
            WF_SetCE_N(WF_LOW);  /* enable module  */

            /* Toggle the module into and out of reset */
            WF_SetRST_N(WF_LOW);            // put module into reset
            WF_SetRST_N(WF_HIGH);           // take module out of of reset

            ResetPll();  // needed until PLL fix made in A2 silicon
            
            WifiAsyncClearAllEvents();
            g_chipResetState = CR_BEGIN;
            g_initState = I_CHIP_RESET;
            break;

        //----------------------------------------
        case I_CHIP_RESET:
        //----------------------------------------
            retCode = ChipResetStateMachine();
            // if chip reset complete
            if (retCode == TCPIP_MAC_RES_OK)
            {
                // TBD -- disable low power mode

                retCode = TCPIP_MAC_RES_PENDING;  // not done initializing, so set code back to busy
                RawResetInitStateMachine();
                g_initState = I_RAW_INIT;
            }
            break;

        //----------------------------------------
        case I_RAW_INIT:
        //----------------------------------------
            retCode = RawInitStateMachine();
            // if RAW init complete
            if (retCode == RAW_INIT_COMPLETE)
            {
                g_initState = I_SEND_INIT_MGMT_MSGS;
                retCode = TCPIP_MAC_RES_PENDING; // not done initializing, so still busy with init
            }
            // else if not still busy doing raw init then raw init failed
            else if (retCode != TCPIP_MAC_RES_PENDING)
            {
                // TBD: signal WiFi fatal error
                retCode = TCPIP_MAC_RES_INIT_FAIL;
            }
            break;

        //----------------------------------------
        case I_SEND_INIT_MGMT_MSGS:
        //----------------------------------------
            WFEnableMRF24WMode();
            WF_GetDeviceInfo(&deviceInfo);
            WF_CPCreate();
            WifiAsyncClearAllEvents();
            SYS_ASSERT(deviceInfo.romVersion == 0x30 || deviceInfo.romVersion == 0x31, "");

            /* send init messages to MRF24W */
            WF_LibInitialize(_TCPIPStackHandleToNet(GetNetworkConfig()));

            #if defined(TCPIP_STACK_USE_GRATUITOUS_ARP)
            InitGratuitousArp();
            #endif

            #if defined(TCPIP_STACK_USE_EZ_CONFIG)
            if(p_wifi_ConfigData->Flags.bWFEasyConfig)
            {
                WFEasyConfigInit();
            }
            #endif  // defined(TCPIP_STACK_USE_EZ_CONFIG)

            g_initState = I_CONNECT;
            break;
       
       //-----------------------------------------
       case I_CONNECT:
       //-----------------------------------------
#if 1
            #if (WF_DEFAULT_NETWORK_TYPE == WF_NETWORK_TYPE_SOFT_AP)
                WF_StartScanWhenBoot(GetNetworkConfig());
            #else
                Wifi_Connect();
                WifiAsyncSetEventPending(ASYNC_DHCP_CONFIG_PENDING); // configure DHCP after init complete
            #endif
#endif
            g_initState = I_INIT_COMPLETE; // init done, this state machine won't be called anymore, but
                                           // interrupt needs to know
            retCode = TCPIP_MAC_RES_OK;
            break;

        case I_INIT_COMPLETE:
            // should never reach this case
            SYS_ASSERT(false, "Invalid case");
            break;

    } // end switch

    return retCode;
}


bool isInitComplete(void)
{
    return g_initState == I_INIT_COMPLETE;
}

static void ChipResetTimeoutCallback(SYS_TICK currSysTick)
{
    currSysTick = currSysTick; // not used, but avoid warning
    g_chipResetTimeout = true;
}

static int ChipResetStateMachine(void)
{
    uint16_t value;
    static SystemTickHandle timer;
    int retCode = TCPIP_MAC_RES_PENDING;

    switch (g_chipResetState)
    {
        //----------------------------------------
        case CR_BEGIN:
        //----------------------------------------
            /* clear the power bit to disable low power mode on the MRF24W */
            Write16BitWFRegister(WF_PSPOLL_H_REG, 0x0000);

            /* Set HOST_RESET bit in register to put device in reset */
            Write16BitWFRegister(WF_HOST_RESET_REG, Read16BitWFRegister(WF_HOST_RESET_REG) | WF_HOST_RESET_MASK);

            /* Clear HOST_RESET bit in register to take device out of reset */
            Write16BitWFRegister(WF_HOST_RESET_REG, Read16BitWFRegister(WF_HOST_RESET_REG) & ~WF_HOST_RESET_MASK);

            /* after reset is started poll register to determine when HW reset has completed */
            g_chipResetTimeout = FALSE;
            timer = SYS_TICK_TimerCreate(ChipResetTimeoutCallback);
            SYS_TICK_TimerSetRate(timer, SYS_TICK_TicksPerSecondGet() * 3);  // one shot at 3 seconds
            g_chipResetState = CR_WAIT_HW_RESET;
            break;

        //----------------------------------------
        case CR_WAIT_HW_RESET:
        //----------------------------------------
            // if HW reset completed
            Write16BitWFRegister(WF_INDEX_ADDR_REG, WF_HW_STATUS_REG);
            value = Read16BitWFRegister(WF_INDEX_DATA_REG);
            if ((value & WF_HW_STATUS_NOT_IN_RESET_MASK) != 0)
            {
                // if value 0xffff then most likely SPI is not connected
                if (value == 0xffff)
                {
                    retCode = TCPIP_MAC_RES_INIT_FAIL;
                }
                // else successful HW reset, set timer and go to next state
                else
                {
                    SYS_TICK_TimerDelete(timer);  // get rid of previous timer
                    g_chipResetTimeout = FALSE;
                    timer = SYS_TICK_TimerCreate(ChipResetTimeoutCallback); // new timer
                    SYS_TICK_TimerSetRate(timer, SYS_TICK_TicksPerSecondGet() * 3);  // one shot at 3 seconds
                    g_chipResetState = CR_WAIT_FIFO_BYTE_COUNT;
                }
            }
            // else if timed out waiting for HW reset
            else if (g_chipResetTimeout)
            {
                retCode = TCPIP_MAC_RES_INIT_FAIL;
            }
            break;

        //----------------------------------------
        case CR_WAIT_FIFO_BYTE_COUNT:
        //----------------------------------------
            // if chip reset complete
            value = Read16BitWFRegister(WF_HOST_WFIFO_BCNT0_REG);
            if (value != 0)
            {
                SYS_TICK_TimerDelete(timer);  // get rid of timer
                InitWiFiInterrupts();
                retCode = TCPIP_MAC_RES_OK;
            }
            // else timed out
            else if (g_chipResetTimeout)
            {
                retCode = TCPIP_MAC_RES_INIT_FAIL;
            }
            break;

    } // end switch

    return retCode;
}


static void InitWiFiInterrupts(void)
{
    uint8_t  mask8;
    uint16_t mask16;

    /* disable the interrupts gated by the 16-bit host int register */
    HostInterrupt2RegInit(WF_HOST_2_INT_MASK_ALL_INT, (uint16_t)WF_INT_DISABLE);

    /* disable the interrupts gated the by main 8-bit host int register */
    HostInterruptRegInit(WF_HOST_INT_MASK_ALL_INT, WF_INT_DISABLE);

    /* Initialize the External Interrupt allowing the MRF24W to interrupt */
    /* the Host from this point forward.                                                       */
    WF_EintInit();
    WF_EintEnable();


    /* enable the following MRF24W interrupts in the INT1 8-bit register */
    mask8 = (WF_HOST_INT_MASK_FIFO_1_THRESHOLD |     /* Mgmt Rx Msg interrupt                  */
             WF_HOST_INT_MASK_FIFO_0_THRESHOLD |     /* Data Rx Msg interrupt                  */
             WF_HOST_INT_MASK_RAW_0_INT_0      |     /* RAW0 Move Complete (Data Rx) interrupt */
             WF_HOST_INT_MASK_RAW_1_INT_0      |     /* RAW1 Move Complete (Data Tx) interrupt */
             WF_HOST_INT_MASK_INT2);                 /* Interrupt 2 interrupt                  */
    HostInterruptRegInit(mask8, WF_INT_ENABLE);

    /* enable the following MRF24W interrupts in the INT2 16-bit register */
    mask16 = (WF_HOST_INT_MASK_RAW_2_INT_0     |    /* RAW2 Move Complete (Mgmt Rx) interrupt */
              WF_HOST_INT_MASK_RAW_3_INT_0     |    /* RAW3 Move Complete (Mgmt Tx) interrupt */
              WF_HOST_INT_MASK_RAW_4_INT_0     |    /* RAW4 Move Complete (Scratch) interrupt */
              WF_HOST_INT_MASK_RAW_5_INT_0     |    /* RAW5 Move Complete (Scratch) interrupt */
              WF_HOST_INT_MASK_MAIL_BOX_1_WRT);
    HostInterrupt2RegInit(mask16, WF_INT_ENABLE);
}

/*****************************************************************************
 * FUNCTION: HostInterrupt2RegInit
 *
 * RETURNS: N/A
 *
 * PARAMS:
 *      hostIntrMaskRegMask - The bit mask to be modified.
 *      state               - One of WF_INT_DISABLE, WF_INT_ENABLE where
 *                             Disable implies clearing the bits and enable sets the bits.
 *
 *
 *  NOTES: Initializes the 16-bit Host Interrupt register on the MRF24W with the
 *      specified mask value either setting or clearing the mask register
 *      as determined by the input parameter state.
 *****************************************************************************/
static void HostInterrupt2RegInit(uint16_t hostIntMaskRegMask,
                                  uint8_t  state)
{
    uint16_t int2MaskValue;

    /* Host Int Register is a status register where each bit indicates a specific event  */
    /* has occurred. In addition, writing a 1 to a bit location in this register clears  */
    /* the event.                                                                        */

    /* Host Int Mask Register is used to enable those events activated in Host Int Register */
    /* to cause an interrupt to the host                                                    */

    /* read current state of int2 mask reg */
    int2MaskValue = Read16BitWFRegister(WF_HOST_INTR2_MASK_REG);

    /* if caller is disabling a set of interrupts */
    if (state == WF_INT_DISABLE)
    {
        /* zero out that set of interrupts in the interrupt mask copy */
        int2MaskValue &= ~hostIntMaskRegMask;
    }
    /* else caller is enabling a set of interrupts */
    else
    {
        /* set to 1 that set of interrupts in the interrupt mask copy */
        int2MaskValue |= hostIntMaskRegMask;
    }

    /* write out new interrupt mask value */
    Write16BitWFRegister(WF_HOST_INTR2_MASK_REG, int2MaskValue);

    /* ensure that pending interrupts from those updated interrupts are cleared */
    Write16BitWFRegister(WF_HOST_INTR2_REG, hostIntMaskRegMask);

}


/*****************************************************************************
 * FUNCTION: HostInterruptRegInit
 *
 * RETURNS: N/A
 *
 * PARAMS:
 *      hostIntrMaskRegMask - The bit mask to be modified.
 *      state -  one of WF_EXINT_DISABLE, WF_EXINT_ENABLE where
 *                Disable implies clearing the bits and enable sets the bits.
 *
 *
 *  NOTES: Initializes the 8-bit Host Interrupt register on the MRF24W with the
 *      specified mask value either setting or clearing the mask register
 *      as determined by the input parameter state.  The process requires
 *      2 spi operations which are performed in a blocking fashion.  The
 *      function does not return until both spi operations have completed.
 *****************************************************************************/
static void HostInterruptRegInit(uint8_t hostIntrMaskRegMask,
                                 uint8_t state)
{
    uint8_t hostIntMaskValue;

    /* Host Int Register is a status register where each bit indicates a specific event  */
    /* has occurred. In addition, writing a 1 to a bit location in this register clears  */
    /* the event.                                                                        */

    /* Host Int Mask Register is used to enable those events activated in Host Int Register */
    /* to cause an interrupt to the host                                                    */

    /* read current state of Host Interrupt Mask register */
    hostIntMaskValue = Read8BitWFRegister(WF_HOST_MASK_REG);

    /* if caller is disabling a set of interrupts */
    if (state == WF_INT_DISABLE)
    {
        /* zero out that set of interrupts in the interrupt mask copy */
        hostIntMaskValue = (hostIntMaskValue & ~hostIntrMaskRegMask);
    }
    /* else caller is enabling a set of interrupts */
    else
    {
        /* set to 1 that set of interrupts in the interrupt mask copy */
        hostIntMaskValue = (hostIntMaskValue & ~hostIntrMaskRegMask) | hostIntrMaskRegMask;
    }

    /* write out new interrupt mask value */
    Write8BitWFRegister(WF_HOST_MASK_REG, hostIntMaskValue);

    /* ensure that pending interrupts from those updated interrupts are cleared */
    Write8BitWFRegister(WF_HOST_INTR_REG, hostIntrMaskRegMask);


}


#endif // WF_EVENT_DRIVEN

/*****************************************************************************
 * FUNCTION: WF_Init
 *
 * RETURNS: true if success
 *          false otherwise
 *
 * PARAMS:
 *      N/A.
 *
 *
 *  NOTES: This function must be called once prior to calling any other WF...
 *          functions.  This function initializes the Wifi Driver internal State.
 *          It also verifies functionality of the lower level SPI driver and
 *          connected hardware.
 *****************************************************************************/
bool WF_Init(const void* pNetIf)
{
#if !defined(WF_EVENT_DRIVEN)
    t_wfDeviceInfo deviceInfo;
#endif

#if 0
    // debug remove later
    PA6_IO = 0;
    SET_PA6_AS_OUTPUT();
 //   PA6_IO = 1;
 //   PA6_IO = 0;
#endif

#if !defined(WF_EVENT_DRIVEN)
    WF_ConfigDataLoad();
    if(!WFHardwareInit())
    {
        return false;
    }
    RawInit();


    WFEnableMRF24WMode();
    WF_GetDeviceInfo(&deviceInfo);
    WF_CPCreate();
    #if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
    WifiAsyncClearAllEvents();
    #endif

    SYS_ASSERT(deviceInfo.romVersion == 0x30 || deviceInfo.romVersion == 0x31, "");        
    
    /* send init messages to MRF24W */
    WF_LibInitialize((TCPIP_NET_IF*)pNetIf);
    

#if defined(TCPIP_STACK_USE_GRATUITOUS_ARP)
    InitGratuitousArp();
#endif
    
#endif // WF_EVENT_DRIVEN

    return true;
}
/*****************************************************************************
 * FUNCTION: WF_DeInit
 *
 * RETURNS: true if success
 *          false otherwise
 *
 * PARAMS:
 *      N/A.
 *
 *
 *  NOTES: This function save WiFi configure setting data into storage.
 *****************************************************************************/
bool WF_DeInit( void* pNetIf)
{
	return WF_ConfigDataSave();
}

/*****************************************************************************
 * FUNCTION: WF_LibInitialize
 *
 * RETURNS:  None
 *
 * PARAMS:   None
 *
 *  NOTES:   Performs initialization which is specific to the Microchip Demo code.
 *****************************************************************************/
 void WF_LibInitialize(TCPIP_NET_IF* pNetIf)
{
#if defined(TCPIP_STACK_USE_IPV6)
    uint8_t solicitedNodeMulticastMACAddr[] = {0x33, 0x33, 0xff, 0x00, 0x00, 0x00};
    uint8_t allNodesMulticastMACAddr[] = {0x33, 0x33, 0x00, 0x00, 0x00, 0x01};
    uint8_t i;
#endif // TCPIP_STACK_USE_IPV6


    /* Disable Tx Data confirms (from the MRF24W) */
    WF_SetTxDataConfirm(WF_DISABLED);

    // notify host on all events
    WF_CASetEventNotificationAction(WF_DEFAULT_EVENT_NOTIFICATION_LIST);

    /* if the user has left the default MAC address in tcpip_config.h unchanged then use */
    /* the unique MRF24W MAC address so prevent multiple devices from having the same   */
    /* MAC address.                                                                     */
    if ( memcmp((void *)pNetIf->netMACAddr.v, (void *)MchpDefaultMacAddress, WF_MAC_ADDRESS_LENGTH) == 0)
    {
        /* get the MRF24W MAC address and overwrite the MAC in pNetIf */
        WF_MacAddressGet((uint8_t *)pNetIf->netMACAddr.v);
    }
    /* else presume the user has a unique MAC address of their own that they wish to use */    
    else
    {
        // set MAC address with user-supplied MAC */
        WF_MacAddressSet((uint8_t *)pNetIf->netMACAddr.v);
    }

    InitPowerSaveTask();

    #if defined(TCPIP_STACK_USE_IPV6)
    for (i =0; i < 3; i++)
    {
        solicitedNodeMulticastMACAddr[3 + i] = pNetIf->netMACAddr.v[3 + i];
    }

        #if defined(ENABLE_SOFTWARE_MULTICAST_FILTER)
        WF_EnableSWMultiCastFilter();
        do {
            t_wfMultiCastConfig p_config;

            p_config.filterId = WF_MULTICAST_FILTER_1;
            p_config.action = WF_MULTICAST_USE_FILTERS;
            memcpy((void *)p_config.macBytes, (void *)solicitedNodeMulticastMACAddr, WF_MAC_ADDRESS_LENGTH);
            p_config.macBitMask = 0x3F;
            WF_MulticastSetConfig(&p_config);

            p_config.filterId = WF_MULTICAST_FILTER_2;
            p_config.action = WF_MULTICAST_USE_FILTERS;
            memcpy((void *)p_config.macBytes, (void *)allNodesMulticastMACAddr, WF_MAC_ADDRESS_LENGTH);
            p_config.macBitMask = 0x3F;
            WF_MulticastSetConfig(&p_config);
        } while (0);

        #else // not using software multicast filters
            WF_SetMultiCastFilter(WF_MULTICAST_FILTER_1, solicitedNodeMulticastMACAddr);
            WF_SetMultiCastFilter(WF_MULTICAST_FILTER_2, allNodesMulticastMACAddr);
        #endif // ENABLE_SOFTWARE_MULTICAST_FILTER
    #endif // TCPIP_STACK_USE_IPV6

}
/*****************************************************************************
 * FUNCTION: WF_ConfigDataSave
 *
 * RETURNS:  bool
 *
 * PARAMS:   TCPIP_NET_IF* pNetIf
 *
 *  NOTES:   write WiFi configure setting data into memory.  
                  such as ssid,security key,etc..
 *****************************************************************************/
bool WF_ConfigDataSave(void)
{
#if defined(MEDIA_STORAGE_EEPROM)
    WiFiMemory_write((uint8_t*)p_wifi_ConfigData, sizeof(t_wfConfigData));
#endif  // defined(MEDIA_STORAGE_EEPROM)
    return true;
}


/*****************************************************************************
 * FUNCTION: WF_ConfigDataLoad
 *
 * RETURNS:  bool
 *
 * PARAMS:   TCPIP_NET_IF* pNetIf
 *
 *  NOTES:   To configure WiFi setting, Read WiFi setting data from Storage, if failed, 
             then use default value such as ssid,security key,etc..
 *****************************************************************************/
bool WF_ConfigDataLoad(void)
{
    bool bVerifySuccess = true;
    WiFiMemory_read((uint8_t*)p_wifi_ConfigData,sizeof(t_wfConfigData));

    // verify
    if ((p_wifi_ConfigData->networkType == 0) || 
        ( p_wifi_ConfigData->SsidLength ==0)  ||
        (strlen((const char *)p_wifi_ConfigData->netSSID) != p_wifi_ConfigData->SsidLength))
    {
        bVerifySuccess = false;
    }

    if (bVerifySuccess == false)  // use default value
    {
        // Load the default SSID Name
        if(sizeof(WF_DEFAULT_SSID_NAME)-1 > sizeof(p_wifi_ConfigData->netSSID))
        {
            return false;
        }
        memcpy(p_wifi_ConfigData->netSSID, (const void*)WF_DEFAULT_SSID_NAME, sizeof(WF_DEFAULT_SSID_NAME)-1);
        p_wifi_ConfigData->SsidLength = sizeof(WF_DEFAULT_SSID_NAME) - 1;
        p_wifi_ConfigData->networkType = WF_DEFAULT_NETWORK_TYPE;
#if defined(TCPIP_STACK_USE_EZ_CONFIG)
        p_wifi_ConfigData->Flags.bWFEasyConfig = 1;
#endif  // defined(TCPIP_STACK_USE_EZ_CONFIG)

        p_wifi_ConfigData->SecurityMode = WF_DEFAULT_WIFI_SECURITY_MODE;

#if (WF_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_OPEN)
        memset(p_wifi_ConfigData->SecurityKey, 0x00, sizeof(p_wifi_ConfigData->SecurityKey));
        p_wifi_ConfigData->SecurityKeyLength = 0;

#elif WF_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_WEP_40
        memcpy(p_wifi_ConfigData->SecurityKey, (const void*)WF_DEFAULT_WEP_KEYS_40, sizeof(WF_DEFAULT_WEP_KEYS_40) - 1);
        p_wifi_ConfigData->SecurityKeyLength = sizeof(WF_DEFAULT_WEP_KEYS_40) - 1;

#elif WF_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_WEP_104
        memcpy(p_wifi_ConfigData->SecurityKey, (const void*)WF_DEFAULT_WEP_KEYS_104, sizeof(WF_DEFAULT_WEP_KEYS_104) - 1);
        p_wifi_ConfigData->SecurityKeyLength = sizeof(WF_DEFAULT_WEP_KEYS_104) - 1;

#elif (WF_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_WPA_WITH_KEY)         || \
        (WF_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_WPA2_WITH_KEY)      || \
        (WF_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_WPA_AUTO_WITH_KEY)
        memcpy(p_wifi_ConfigData->SecurityKey, (const void*)WF_DEFAULT_PSK, sizeof(WF_DEFAULT_PSK) - 1);
        p_wifi_ConfigData->SecurityKeyLength = sizeof(WF_DEFAULT_PSK) - 1;

#elif (WF_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_WPA_WITH_PASS_PHRASE)       || \
        (WF_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_WPA2_WITH_PASS_PHRASE)    || \
        (WF_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_WPA_AUTO_WITH_PASS_PHRASE)
        memcpy(p_wifi_ConfigData->SecurityKey, (const void*)WF_DEFAULT_PSK_PHRASE, sizeof(WF_DEFAULT_PSK_PHRASE) - 1);
        p_wifi_ConfigData->SecurityKeyLength = sizeof(WF_DEFAULT_PSK_PHRASE) - 1;

#elif (WF_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_WPS_PUSH_BUTTON)
    memset(p_wifi_ConfigData->SecurityKey, 0x00, sizeof(p_wifi_ConfigData->SecurityKey));
    p_wifi_ConfigData->SecurityKeyLength = 0;

#elif (WF_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_WPS_PIN)
    memcpy(p_wifi_ConfigData->SecurityKey, (const void*)WF_DEFAULT_WPS_PIN, sizeof(WF_DEFAULT_WPS_PIN) - 1);
    p_wifi_ConfigData->SecurityKeyLength = sizeof(WF_DEFAULT_WPS_PIN) - 1;

#else 
#error "No security defined"
#endif /* WF_DEFAULT_WIFI_SECURITY_MODE */

#if defined(ENABLE_STATIC_IP)
	p_wifi_ConfigData->Flags.bIsDHCPEnabled = false;
#else
	p_wifi_ConfigData->Flags.bIsDHCPEnabled = true;
#endif

    }
    return true;
}

#endif /* TCPIP_IF_MRF24W */

