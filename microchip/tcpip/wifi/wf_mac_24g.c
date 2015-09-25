/*******************************************************************************
  MRF24W Driver Medium Access Control (MAC) Layer

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Provides access to MRF24W WiFi controller
    -Reference: MRF24W Data sheet, IEEE 802.11 Standard
*******************************************************************************/

/*******************************************************************************
FileName:  wf_mac_24g.c 
Copyright © 2012 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software foradditional information regarding your rights and obligations.

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

#include "tcpip_mac_private.h"
#include "tcpip/tcpip_mac.h"
#include "tcpip/tcpip_mac_object.h"

#if defined(TCPIP_IF_MRF24W) 

#include "mrf24w_mac.h"
#include "wf_easy_config.h"
#include "wf_configData.h"



/*
*********************************************************************************************************
*                                           DEFINES                               
*********************************************************************************************************
*/

#define SNAP_VAL        (0xaa)
#define SNAP_CTRL_VAL   (0x03)
#define SNAP_TYPE_VAL   (0x00)

#define ETHER_IP        (0x00)
#define ETHER_ARP       (0x06)

#if defined( __PIC32MX__ )   
    #define IPL_MASK  ((uint32_t)0x3f << 10)
#endif

#define SNAP_SIZE (6)

#define ENC_PREAMBLE_SIZE    (sizeof(ENC_PREAMBLE))
#define ENC_PREAMBLE_OFFSET  (10)

#define WF_RX_PREAMBLE_SIZE   (sizeof(t_wfRxPreamble))
#define WF_TX_PREAMBLE_OFFSET (0)

#define WF_TX_PREAMBLE_SIZE   (sizeof(t_wfTxPreamble))

#define MCHP_DATA_PACKET_SIZE (4 + MAX_PACKET_SIZE + 4)

//============================================================================
//                                  Rx/Tx Buffer Constants
// Used to correlate former Ethernet packets to MRF24W packets.
//============================================================================
#define ENC_RX_BUF_TO_RAW_RX_BUF_ADJUSTMENT          ((RXSTART + ENC_PREAMBLE_SIZE)   - (ENC_PREAMBLE_OFFSET   + WF_RX_PREAMBLE_SIZE))
#define ENC_TX_BUF_TO_RAW_TX_BUF_ADJUSTMENT          ((TXSTART + WF_TX_PREAMBLE_SIZE) - (WF_TX_PREAMBLE_OFFSET + WF_TX_PREAMBLE_SIZE))
#define ENC_TCB_BUF_TO_RAW_SCRATCH_BUF_ADJUSTMENT    (BASE_TCB_ADDR)

//============================================================================
//                                  RAW Constants
//============================================================================
#define ENC_RD_PTR_ID               (0)
#define ENC_WT_PTR_ID               (1)

#if defined(WF_EVENT_DRIVEN)
// RAW Init state machine states
enum
{
    R_INIT_BEGIN                = 0,
    R_WAIT_FOR_SCRATCH_UNMOUNT  = 1,
    R_WAIT_FOR_SCRATCH_MOUNT    = 2
} t_rawInitStates;

typedef enum
{
    WAIT_FOR_DATA_RX_MOUNT   = 0,
    WAIT_FOR_DATA_RX_UNMOUNT = 1

} t_RxStates;

typedef enum
{
    DATA_TX_INACTIVE         = 0,
    WAIT_FOR_DATA_TX_MOUNT   = 1,
    WAIT_FOR_DATA_TX_SIGNAL  = 2
} t_TxStates;

#endif // WF_EVENT_DRIVEN

/*
*********************************************************************************************************
*                                           LOCAL DATA TYPES                               
*********************************************************************************************************
*/



#if defined( TCPIP_STACK_COMMANDS_WIFI_ENABLE )
extern uint8_t g_hibernate_state;
extern uint8_t g_wakeup_notice;
#endif


typedef struct
{
    uint8_t type;
    uint8_t subType;
} tRxPreamble;

/* A header appended at the start of all RX frames by the hardware */
typedef struct _ENC_PREAMBLE
{
    uint16_t        NextPacketPointer;
    RXSTATUS        StatusVector;
    MAC_ADDR        DestMACAddr;
    MAC_ADDR        SourceMACAddr;
    TCPIP_UINT16_VAL        Type;
} ENC_PREAMBLE;

typedef struct
{
    uint8_t           snap[SNAP_SIZE];
    MAC_ADDR        DestMACAddr;
    MAC_ADDR        SourceMACAddr;
    TCPIP_UINT16_VAL        Type;
} t_wfRxPreamble;


typedef struct
{
    uint8_t  reserved[4];
} t_wfTxPreamble;

#if defined(TCPIP_STACK_USE_GRATUITOUS_ARP)
typedef struct
{
    uint8_t          periodInSeconds;       // period in seconds betweeen each arp
    SystemTickHandle timerHandle;           // handle to timer handle
} t_gratuitousArpContext;
#endif // TCPIP_STACK_USE_GRATUITOUS_ARP


/*
*********************************************************************************************************
*                                           LOCAL GLOBAL VARIABLES                               
*********************************************************************************************************
*/
TCPIP_NET_IF    *g_pNetIf;
static uint8_t  g_encPtrRAWId[2];  // indexed by ENC_RD_PTR_ID (0) and ENC_WT_PTR_ID (1).  Values can be:
                                   //  RAW_DATA_RX_ID, RAW_DATA_TX_ID, RAW_SCRATCH_ID

// keeps track of ENC read and write indexes
static uint16_t g_encIndex[2];       // index 0 stores current ENC read index, index 1 stores current ENC write index


#if defined(WF_EVENT_DRIVEN)
static uint8_t  g_rawInitState;                         // state for Raw Init state machine
static uint8_t  g_rxDataState;                          // state for Data Rx state machine
static uint8_t  g_txDataState;                          // state for Data Tx state machine
static uint16_t g_rxByteCount;                          // byte count for Raw Data Rx message
static SystemTickHandle g_DataRxRawMoveTimer  = NULL;   // timer callback for Data Rx raw move complete
static SystemTickHandle g_DataTxRawMoveTimer  = NULL;   // timer callback for Data Tx raw move complete
static SystemTickHandle g_ScratchRawMoveTimer = NULL;   // timer callback for Scratch raw move complete
static bool g_rxDataRawMoveTimeout;                     // set to true when rx data raw move timeout occurs
static bool g_txDataRawMoveTimeout;                     // set to true when tx data raw move timeout occurs
static bool g_scratchRawMoveTimeout;                    // set to true when scratch raw move timeout occurs
#endif

#if defined(TCPIP_STACK_USE_GRATUITOUS_ARP)
t_gratuitousArpContext g_gratArpContext;
#endif

uint16_t g_rxBufferLength;
static uint16_t g_txPacketLength;

#if defined(SYS_DEBUG_ENABLE)
static bool   g_txBufferFlushed;
#endif

#if defined(SYS_CONSOLE_ENABLE)
    #if !defined(WF_EVENT_DRIVEN)
    extern bool g_outputConnectionContext;
    #endif
#endif

extern bool gHostScanNotAllowed;

#if !defined(WF_EVENT_DRIVEN)
    #if defined(SAVE_WPS_CREDENTIALS)
    extern     bool g_Get_WPSCredentials;
    #endif
#endif
/*
*********************************************************************************************************
*                                           LOCAL FUNCTION PROTOTYPES                               
*********************************************************************************************************
*/

static uint16_t MACIFService(void);
static void SyncENCPtrRAWState(uint8_t encPtrId, uint16_t encIndex);
extern uint16_t RawCalculateChecksum(uint16_t length);
extern     void WFEasyConfigMgr(TCPIP_NET_IF* pNetIf);

#if defined(TCPIP_STACK_USE_GRATUITOUS_ARP)
static void GratuitousArpHandler(SYS_TICK curSysTick);
#endif

#if WF_EVENT_DRIVEN
extern int WF_InitStateMachine(void);
static void RawInit(void);
static void RawMoveDataRxTimeoutHandler(SYS_TICK currSysTick);
static void RawMoveDataTxTimeoutHandler(SYS_TICK currSysTick);
static void RawMoveScratchTimeoutHandler(SYS_TICK currSysTick);
#endif

#if (WF_DEFAULT_NETWORK_TYPE == WF_NETWORK_TYPE_SOFT_AP)
static void SoftAP_ReDirection(TCPIP_MAC_HANDLE hMac);
#endif


#if defined(SYS_CONSOLE_ENABLE)
    extern void _WF_OutputConnectionContext(void);
#endif



uint8_t GetReadPtrRawWindow()
{
    return g_encPtrRAWId[ENC_RD_PTR_ID];
}

void SetNetworkConfig(TCPIP_NET_HANDLE netH)
{
    g_pNetIf = _TCPIPStackHandleToNet(netH);
}    

TCPIP_NET_HANDLE GetNetworkConfig(void)
{
    return g_pNetIf;
}    


/*****************************************************************************
  Function:
    void SyncENCPtrRAWState(uint8_t encPtrId)

  Summary:
    Synchronizes the Ethernet RAM index to the WiFi RAW window index.

  Description:
    Any time stack code changes the index within the 'logical' Ethernet RAM
    this function must be called to assure the RAW driver is synced up with
    where the stack code thinks it is within the Ethernet RAM.  This applies
    to reading/writing tx data, rx data, or tcb data.  
    
    This function is typically called right after g_encIndex[] is updated.

  Precondition:
    None

  Parameters:
    encPtrId -- ENC_RD_PTR_ID or ENC_WT_PTR_ID
                 
  Returns:
    None
      
  Remarks:
    None
*****************************************************************************/
static void SyncENCPtrRAWState(uint8_t encPtrId, uint16_t encIndex)
{
    uint8_t     rawId;
    uint16_t    rawIndex;
    SYS_TICK  startTickCount;
    SYS_TICK  maxAllowedTicks;

    EnsureWFisAwake();
    
    /* Save the the read or write index value.  Code below will convert it to a */
    /* Raw index                                                                */
    g_encIndex[encPtrId] = encIndex;

    /*-----------------------------------------------------------------------------*/    
    /* if the input read or write pointer is in the Rx section of the Ethernet RAM */
    /*-----------------------------------------------------------------------------*/
    if (g_encIndex[encPtrId] < TXSTART)
    {
        /* set the rawId */
        rawId = RAW_DATA_RX_ID;

        /* Convert encPtr index to Raw Index */
        rawIndex = g_encIndex[encPtrId] - ENC_RX_BUF_TO_RAW_RX_BUF_ADJUSTMENT;

        // encPtr[encPtrId] < (RXSTART + ENC_PREAMBLE_SIZE) is an error since we don't have
        // the same preamble as the ENC chip
        SYS_ASSERT( (g_encIndex[encPtrId] >= (RXSTART + ENC_PREAMBLE_SIZE)), "" );
        
    }
    /*----------------------------------------------------------------------------------*/    
    /* else if the input read or write pointer is in the Tx section of the Ethernet RAM */
    /*----------------------------------------------------------------------------------*/
    else if (g_encIndex[encPtrId] < BASE_TCB_ADDR)   
    {
        /* if the Tx data raw window has not yet been allocated (the stack is creating a new Tx data packet) */
        if (GetRawWindowState(RAW_DATA_TX_ID) != WF_RAW_DATA_MOUNTED)
        {
            /* WiFi chip may be in processs of transmitting a prior tx packet, so it is very possible that  */
            /* we won't be able to allocate a tx data packet instantly.  Set up timer and wait a reasonable */
            /* amount of time.                                                                              */
            maxAllowedTicks = SYS_TICK_TicksPerSecondGet() * 6;  /* 6 second timeout, needed if scan in progress */
            startTickCount = SYS_TICK_Get();
            while ( !AllocateDataTxBuffer(MCHP_DATA_PACKET_SIZE) )
            {
                /* If timed out than lock up -- something bad happened */
                if (SYS_TICK_Get() - startTickCount >= maxAllowedTicks)
                {
                    SYS_ASSERT(false, "");  /* timeout occurred */
                }
            } 
        } 

        /* set the rawId */
        rawId = RAW_DATA_TX_ID;

        /* convert enc Ptr index to raw index */
        rawIndex = g_encIndex[encPtrId] - ENC_TX_BUF_TO_RAW_TX_BUF_ADJUSTMENT;

        /* encPtr[encPtrId] < BASE_TX_ADDR is an error since we don't have the same  */
        /* pre-BASE_TX_ADDR or post tx buffer as the ENC chip                        */
        SYS_ASSERT((g_encIndex[encPtrId] >= BASE_TX_ADDR) && (g_encIndex[encPtrId] <= (BASE_TX_ADDR + MAX_PACKET_SIZE)), "");
    }
    /*----------------------------------------------------------------------------------*/    
    /* else input read or write pointer is in the TCB (Scratch) section of Ethernet RAM */
    /*----------------------------------------------------------------------------------*/
    else
    {
        /* set the raw Id, scratch is always mounted during intialization */
        rawId = RAW_SCRATCH_ID;

        /* convert Enc ptr index to raw index */
        rawIndex = g_encIndex[encPtrId] - ENC_TCB_BUF_TO_RAW_SCRATCH_BUF_ADJUSTMENT;
    }        
   
    /* set the raw index in the specified raw window */
    RawSetIndex(rawId, rawIndex);

    /* Update state variable so we know which raw window is being indexed by the read or write index.  */
    /* Will be either RAW_DATA_RX_ID, RAW_DATA_TX_ID, or RAW_SCRATCH_ID.                               */
    g_encPtrRAWId[encPtrId] = rawId;
    
}


#if defined(TCPIP_STACK_USE_GRATUITOUS_ARP)

// Called periodically when gratuitous arp is being used.  Called from timer interrupt, so
// don't do any work here, but signal an async event.  We don't want to send an
// arp from the timer interrupt!
static void GratuitousArpHandler(SYS_TICK curSysTick)
{
    curSysTick = curSysTick;  // avoid compiler warning

    // if connected and gratuitous arp has not been disabled
    if ( (WFisConnected()) && (g_gratArpContext.periodInSeconds > 0) )
    {
        #if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
        WifiAsyncSetEventPending(ASYNC_GRAT_ARP_PENDING);
        #endif
    }
}

// called by system async event handler
void WiFi_GratuitousArpTask(void)
{
    ARPResolve(g_pNetIf, &g_pNetIf->netIPAddr);
}

// called during init to create gratuitous ARP timer and callback reference
void InitGratuitousArp(void)
{
    // create timer that will be use for gratuitous arp
    g_gratArpContext.timerHandle = SYS_TICK_TimerCreate(GratuitousArpHandler);
    SYS_TICK_TimerSetRate(g_gratArpContext.timerHandle, 0);
}


// called by application to start gratuitous arp
void WF_StartGratuitousArp(uint8_t periodInSeconds)
{
    // configure the arp timer for the desired period
    SYS_TICK_TimerSetRate(g_gratArpContext.timerHandle, SYS_TICK_TicksPerSecondGet() * periodInSeconds);
    g_gratArpContext.periodInSeconds = periodInSeconds;
}

// called by application to stop a gratuitous arp
void WF_StopGratuitousArp(void)
{
    SYS_TICK_TimerSetRate(g_gratArpContext.timerHandle, 0);
    g_gratArpContext.periodInSeconds = 0;
}
#endif // TCPIP_STACK_USE_GRATUITOUS_ARP



#if defined(SAVE_WPS_CREDENTIALS)
enum
{
    AUTH_OPEN       = 0x01,
    AUTH_WPA_PSK    = 0x02,
    AUTH_SHARED     = 0x04,
    AUTH_WPA        = 0x08,
    AUTH_WPA2       = 0x10,
    AUTH_WPA2_PSK   = 0x20
};

enum
{
    ENC_NONE        = 0x01,
    ENC_WEP         = 0x02,
    ENC_TKIP        = 0x04,
    ENC_AES         = 0x08
};

enum
{
    WEP_SHORT_KEY_SIZE  = 5,
    WEP_LONG_KEY_SIZE   = 13
};

enum
{
    SECURITY_NONE,
    SECURITY_OPEN,
    SECURITY_SHARED_KEY40,
    SECURITY_SHARED_KEY104,
    SECURITY_OPEN_KEY40,
    SECURITY_OPEN_KEY104,
    SECURITY_WPA1_PSK_KEY,
    SECURITY_WPA1_PSK_PASS,
    SECURITY_WPA2_PSK_KEY,
    SECURITY_WPA2_PSK_PASS,
    SECURITY_WPAUTO_PSK_KEY,
    SECURITY_WPAUTO_PSK_PASS,
    SECURITY_WPA_ENTERPRISE,
    SECURITY_WPS_PIN,
    SECURITY_WPS_PSB,
};

enum
{
    WEP_KEYIDX_MAX = 4,
    MSK_MAX = 64,
    PIN_MAX = 8,
};

struct sec_wep40
{
    UINT8 key_idx;
    UINT8 key[WEP_KEYIDX_MAX][5];
};

struct sec_wep104
{
    UINT8 key_idx;
    UINT8 key[WEP_KEYIDX_MAX][13];
};

struct sec_wpa_psk
{
    UINT8 key_len;
    UINT8 key[MSK_MAX];
};

struct sec_wps
{
    UINT8 pin[PIN_MAX];
};

union sec_key
{
    struct sec_wep40 wep40;
    struct sec_wep104 wep104;
    struct sec_wpa_psk wpa_psk;
    struct sec_wps wps;
};


static UINT8 ConvAscii2Hex(UINT8 a)
{
    if (a >= '0' && a <= '9')
        return (UINT8)(a - 48);
    if (a >= 'a' && a <= 'f')
        return (UINT8)(a - 97 + 10);
    if (a >= 'A' && a <= 'F')
        return (UINT8)(a - 65 + 10);

    return '?';
}

static void ConvAsciiKey2Hex(UINT8 *key, UINT8 keyLen, UINT8 *hexKey)
{
    UINT8 i;

    for (i = 0; i < keyLen; i += 2)
    {
        hexKey[i / 2] = ConvAscii2Hex(key[i]) << 4;
        hexKey[i / 2] |= ConvAscii2Hex(key[i + 1]);
    }
}

static void ConfigWep(t_wfWpsCred *cred, UINT8 *secType, union sec_key *key)
{
    UINT8 i;
    UINT8 wep_key[WEP_LONG_KEY_SIZE];
    struct sec_wep40 *wep_ctx = (struct sec_wep40 *)key;
    UINT8 *keys = (UINT8 *)wep_ctx + 1;
    UINT8 key_len;

    if (cred->keyLen == WEP_SHORT_KEY_SIZE * 2)
    {
        *secType = WF_SECURITY_WEP_40;
        ConvAsciiKey2Hex(cred->netKey, cred->keyLen, wep_key);
        key_len = cred->keyLen / 2;
    }
    else if (cred->keyLen == WEP_SHORT_KEY_SIZE)
    {
        *secType = WF_SECURITY_WEP_40;
        memcpy(wep_key, cred->netKey, cred->keyLen);
        key_len = cred->keyLen;
    } 
    else if (cred->keyLen == WEP_LONG_KEY_SIZE * 2)
    {
        *secType = WF_SECURITY_WEP_104;
        ConvAsciiKey2Hex(cred->netKey, cred->keyLen, wep_key);
        key_len = cred->keyLen / 2;
    }
    else if (cred->keyLen == WEP_LONG_KEY_SIZE)
    {
        *secType = WF_SECURITY_WEP_104;
        memcpy(wep_key, cred->netKey, cred->keyLen);
        key_len = cred->keyLen;
    } 
    else
    {
        //WF_ASSERT(false);
    }

    for (i = 0; i < 4; i++)
    {
        memcpy(keys + i * key_len, wep_key, key_len);
    }

    wep_ctx->key_idx = cred->keyIdx - 1;
}

#if !defined(WF_EVENT_DRIVEN)
static void WF_SaveWPSCredentials(void)
#endif
#if defined(WF_EVENT_DRIVEN)
void WF_SaveWPSCredentials(void)
#endif
{
    t_wfWpsCred cred;
    union sec_key key;
    UINT8 *psk;
    static BOOL once = FALSE;

    if (!once)
    {
        WF_WPSCredentialsGet(&cred);
        memcpy((void *)(p_wifi_ConfigData->netSSID), (void *)cred.ssid, cred.ssidLen);
        p_wifi_ConfigData->SsidLength  = cred.ssidLen;

        switch (cred.authType)
        {
        case AUTH_OPEN:
            if (cred.encType == ENC_NONE)
            {
                p_wifi_ConfigData->SecurityMode = WF_SECURITY_OPEN;
            } 
            else if (cred.encType == ENC_WEP)
            {
                ConfigWep(&cred, &(p_wifi_ConfigData->SecurityMode), &key);
                if (p_wifi_ConfigData->SecurityMode == WF_SECURITY_WEP_40)
                {
                    memcpy((void *)p_wifi_ConfigData->SecurityKey, (void *)key.wep40.key, WEP_SHORT_KEY_SIZE * 4);
                    p_wifi_ConfigData->SecurityKeyLength = WEP_SHORT_KEY_SIZE * 4;
                } 
                else if (p_wifi_ConfigData->SecurityMode == WF_SECURITY_WEP_104)
                {
                    memcpy((void *)p_wifi_ConfigData->SecurityKey, (void *)key.wep104.key, WEP_LONG_KEY_SIZE * 4);
                    p_wifi_ConfigData->SecurityKeyLength = WEP_LONG_KEY_SIZE * 4;
                } 
                else
                {
                    //WF_ASSERT(false);
                }
            }
            break;

        case AUTH_SHARED:
            ConfigWep(&cred, &p_wifi_ConfigData->SecurityMode, &key);
            if (p_wifi_ConfigData->SecurityMode == WF_SECURITY_WEP_40)
            {
                memcpy((void *)p_wifi_ConfigData->SecurityKey, (void *)key.wep40.key, WEP_SHORT_KEY_SIZE * 4);
                p_wifi_ConfigData->SecurityKeyLength = WEP_SHORT_KEY_SIZE * 4;
            } 
            else if (p_wifi_ConfigData->SecurityMode == WF_SECURITY_WEP_104)
            {
                memcpy((void *)p_wifi_ConfigData->SecurityKey, (void *)key.wep104.key, WEP_LONG_KEY_SIZE * 4);
                p_wifi_ConfigData->SecurityKeyLength = WEP_LONG_KEY_SIZE * 4;
            } 
            else
            {
                //WF_ASSERT(false);
            }
            break;

        case AUTH_WPA_PSK:
        case AUTH_WPA2_PSK:
            psk = (UINT8 *)p_wifi_ConfigData->SecurityKey;;
            memset((void *)psk, 0x00, 64);
            if (cred.keyLen == 64)
            {
                p_wifi_ConfigData->SecurityMode = cred.authType == AUTH_WPA_PSK ?WF_SECURITY_WPA_WITH_KEY : WF_SECURITY_WPA2_WITH_KEY;
                p_wifi_ConfigData->SecurityKeyLength = 32;
                ConvAsciiKey2Hex(cred.netKey, cred.keyLen, psk);
            } 
            else if (cred.keyLen >= 8 && cred.keyLen < 64)
            {
                p_wifi_ConfigData->SecurityMode= cred.authType == AUTH_WPA_PSK ?WF_SECURITY_WPA_WITH_PASS_PHRASE : WF_SECURITY_WPA2_WITH_PASS_PHRASE;
                p_wifi_ConfigData->SecurityKeyLength = cred.keyLen;
                if (p_wifi_ConfigData->SecurityKeyLength > 8 && cred.netKey[p_wifi_ConfigData->SecurityKeyLength - 1] == '\0')
                {
                    --p_wifi_ConfigData->SecurityKeyLength;
                }
                memcpy(psk, cred.netKey, p_wifi_ConfigData->SecurityKeyLength);
            }
            break;

        default:
            //WF_ASSERT(false);
            break;
        } // end switch

        WF_ConfigDataSave();
        once = true;
    }
}
#endif /* SAVE_WPS_CREDENTIALS */

#if defined(SYS_CONSOLE_ENABLE)
    extern bool OutputConnectionContext;
#endif
    
/*****************************************************************************
 * FUNCTION: MRF24W_MACProcess
 *
 * RETURNS: None
 *
 * PARAMS:
 *          hMac - interface instance
 *
 *  NOTES: Called form main loop to support 802.11 operations
 *****************************************************************************/
void MRF24W_MACProcess(TCPIP_MAC_HANDLE hMac)
{
    #if defined( TCPIP_STACK_USE_EZ_CONFIG )
        WFEasyConfigMgr(_TCPIPStackMacToNet(hMac));
    #endif


    #if !defined(WF_EVENT_DRIVEN)
        #if defined(SYS_CONSOLE_ENABLE)
        if (g_outputConnectionContext == true)
        {
            g_outputConnectionContext = false;
            _WF_OutputConnectionContext();
        }
        #endif

        #if defined(SAVE_WPS_CREDENTIALS)
        if(g_Get_WPSCredentials == true)
        {
            g_Get_WPSCredentials = false;
            WF_SaveWPSCredentials();
        }
        #endif // SAVE_WPS_CREDENTIALS)

    #endif // !WF_EVENT_DRIVEN


    #if 0
    // checks if going into or coming out of hibernate mode
    CheckHibernate();
    #endif

    #if WF_DEFAULT_NETWORK_TYPE == WF_NETWORK_TYPE_SOFT_AP
    SoftAP_ReDirection(hMac);
    #endif
    
}

#if defined(SYS_CONSOLE_ENABLE)
    extern void ValidateConfig(void);
#endif

#if ( WF_DEFAULT_NETWORK_TYPE == WF_NETWORK_TYPE_SOFT_AP)
bool SoftAP_Redirection_Enable=false;
extern TCPIP_STACK_MODULE_CTRL  tcpip_stack_ctrl_data;
extern t_wfEasyConfigCtx g_easyConfigCtx;
extern void WF_CPCreate(void);
extern void WF_LibInitialize(TCPIP_NET_IF* pNetIf);
static void SoftAP_ReDirection(TCPIP_MAC_HANDLE hMac)
{
    int i;
    if(SoftAP_Redirection_Enable == true)
    {
        SoftAP_Redirection_Enable = false;

        p_wifi_ConfigData->networkType = CFGCXT.type;
        for(i=0;i<32;i++) p_wifi_ConfigData->netSSID[i] = CFGCXT.ssid[i];		
        p_wifi_ConfigData->netSSID[32] = 0;		
        p_wifi_ConfigData->SsidLength = strlen((const char*)p_wifi_ConfigData->netSSID);
        p_wifi_ConfigData->SecurityMode = CFGCXT.security;		
        p_wifi_ConfigData->defaultWepKey = CFGCXT.defaultWepKey;		
        for(i=0;i<64;i++) p_wifi_ConfigData->SecurityKey[i] = CFGCXT.key[i];		
        p_wifi_ConfigData->SecurityKeyLength = CFGCXT.SecurityKeyLength;
		
        TCPIP_NET_IF* pNetIf = _TCPIPStackMacToNet(hMac);
        {  // WF_Init(pNetIf); //  TBD -- need to modify this as WiFi init no longer exists           
            WFHardwareInit();            
            RawInit();            
            WFEnableMRF24WMode();            
            WF_CPCreate();            
            /* send init messages to MRF24W */            
            WF_LibInitialize((TCPIP_NET_IF*)pNetIf);        
        }
        Wifi_Connect();
        #if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
        WifiAsyncSetEventPending(ASYNC_DHCP_CONFIG_PENDING); // configure DHCP after init complete
    	#endif
    }
}

#endif
#if 0 /* TBD: need to figure out how to do this with unified stack */
static void CheckHibernate(void)
{
    #if defined( TCPIP_STACK_COMMANDS_WIFI_ENABLE )
    #endif
    
    #if defined( TCPIP_STACK_COMMANDS_WIFI_ENABLE )
         if (g_wakeup_notice && (g_hibernate_state == WF_HB_WAIT_WAKEUP)) 
         {
            SYS_TICK_MsDelay(200);
            g_hibernate_state = WF_HB_NO_SLEEP;
            StackInit();
            
            #if defined(WF_CONSOLE_DEMO)
            IperfAppInit();
            #endif
    
            Wifi_Connect();
        }
    #endif

    
    #if defined( TCPIP_STACK_COMMANDS_WIFI_ENABLE )
       wait_console_input:
    #endif
        
    #if defined(WF_CONSOLE) 
        WFConsoleProcess();
        #if defined( WF_CONSOLE_DEMO )
            if (g_hibernate_state == WF_HB_NO_SLEEP)
            {
                IperfAppCall();
            }    
            WFConsoleProcessEpilogue();
        #endif
    #endif


    #if defined( TCPIP_STACK_COMMANDS_WIFI_ENABLE )
        if (g_hibernate_state != WF_HB_NO_SLEEP) 
        {
            if (g_hibernate_state == WF_HB_ENTER_SLEEP) 
            {
                // if we are connected then disconnect before going into hibernate state
                WF_ConnectionStateGet(&state);
                if ((state == WF_CSTATE_CONNECTED_INFRASTRUCTURE) || (state == WF_CSTATE_CONNECTED_ADHOC))
                {
                    WF_Disconnect();
                }          

                WF_HibernateEnable();
                g_hibernate_state = WF_HB_WAIT_WAKEUP;
            }
            if (g_wakeup_notice) 
            {
                //continue;
            }    
            else
            {
                goto wait_console_input;
            }                
        }
    #endif        
}
#endif /* 0 */ 

 /******************************************************************************
 * Function:        void MRF24W_MACInit(TCPIP_NET_IF* pNetIf)
 *
 * PreCondition:    None
 *
 * Input:           pNetIf   - network interface
 *
 * Output:          TCPIP_MAC_RES_OK if initialization succeeded,
 *                  error code otherwise
 *
 * Side Effects:    None
 *
 * Overview:        MRF24W_MACInit sets up the PIC's SPI module and all the
 *                  registers in the MRF24W so that normal operation can
 *                  begin.
 *
 * Note:            When the system is using dynamically registered interrupts,
 *                  the WiFi interrupt handler should be registered here because
 *                  the WF_Init() enables the interrupts! 
 *****************************************************************************/
TCPIP_MAC_RES MRF24W_MACInit(TCPIP_NET_IF* pNetIf)
{
#if defined(WF_EVENT_DRIVEN)
     int retCode;
#endif

    // save the network interface handle
    SetNetworkConfig(pNetIf);

#if defined(WF_EVENT_DRIVEN)
    retCode = WF_InitStateMachine();
    return retCode;
#endif

#if !defined(WF_EVENT_DRIVEN)
    #if defined(SYS_CONSOLE_ENABLE)
    ValidateConfig();
    #endif
    
    if(!WF_Init(pNetIf))
    {
        return TCPIP_MAC_RES_INIT_FAIL;
    }


#if defined(TCPIP_STACK_USE_EZ_CONFIG)
    {
        if(p_wifi_ConfigData->Flags.bWFEasyConfig)
        {
            WFEasyConfigInit();
        }
    }
#endif  // defined(TCPIP_STACK_USE_EZ_CONFIG)




#endif // !WF_EVENT_DRIVEN
    return TCPIP_MAC_RES_OK;
}

#if defined(WF_EVENT_DRIVEN)
void RawResetInitStateMachine(void)
{
    g_rawInitState = R_INIT_BEGIN;
}

int RawInitStateMachine(void)
{
    int retCode = TCPIP_MAC_RES_PENDING;
    int status;
    uint16_t byteCount;


    switch (g_rawInitState)
    {
        //-------------------------------------
        case R_INIT_BEGIN:
        //-------------------------------------
            // create raw move timers
            CreateRawMoveTimers();

            // By default the firmware mounts Scratch to RAW 1 after reset.  If desired,
            // we can read the SysInfo data block from the Scratch.  We are not using this
            // data, so unmount the scratch from this RAW window.
            ScratchUnmount(RAW_ID_1);
            g_rawInitState = R_WAIT_FOR_SCRATCH_UNMOUNT;
            break;

        //-------------------------------------
        case R_WAIT_FOR_SCRATCH_UNMOUNT:
        //-------------------------------------
            // if raw move completed
            if (isRawMoveComplete(&status, &byteCount))
            {
                /* Mount scratch memory, index defaults to 0.  This will stay permanently mounted.   */
                /* If one needs to know, this function returns the number of bytes in scratch memory */
                ScratchMount(RAW_SCRATCH_ID);
                g_rawInitState = R_WAIT_FOR_SCRATCH_MOUNT;
            }
            // else if raw move not completed and it timed out
            else if (status == RM_TIMEOUT)
            {
                retCode = RAW_INIT_SCRATCH_UNMOUNT_FAIL;
            }
            break;

        //-------------------------------------
        case R_WAIT_FOR_SCRATCH_MOUNT:
        //-------------------------------------
            if (isRawMoveComplete(&status, &byteCount))
            {
                RawInit();  // complete raw init
                retCode = RAW_INIT_COMPLETE;
            }
            // else if raw move not completed and it timed out
            else if (status == RM_TIMEOUT)
            {
                retCode = RAW_INIT_SCRATCH_MOUNT_FAIL;
            }
            break;
    }

    return retCode;
}

/******************************************************************************
 * Function:        void RawInit(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Initializes the RAW window states.
 *
 * Note:            None
 *****************************************************************************/
static void RawInit(void)
{
    g_encPtrRAWId[ENC_RD_PTR_ID] = RAW_DATA_RX_ID;
    g_encIndex[ENC_RD_PTR_ID]    = BASE_TCB_ADDR;

    SetRawWindowState(RAW_DATA_TX_ID, WF_RAW_UNMOUNTED);

    g_encPtrRAWId[ENC_WT_PTR_ID]  = RAW_DATA_TX_ID;
    g_encIndex[ENC_WT_PTR_ID]     = BASE_TX_ADDR;                 // set tx encode ptr (index) to start of tx buf + 4 bytes

    SetRxDataPacketLength(0);                                     // current rx buffer length (none) is 0 bytes
    g_txPacketLength  = 0;                                        // current tx packet length (none) is 0 bytes
    #if defined(SYS_DEBUG_ENABLE)
    g_txBufferFlushed = true;                                     // tx buffer is flushed
    #endif

    // from ENC MAC init
    // encWrPtr is left pointing to BASE_TX_ADDR
    // encRdPtr is not initialized... we leave it pointing to BASE_TCB_ADDR

    g_RawIndexPastEnd = 0x00; /* no raw indexes have been set past end of raw window */

    g_txDataState = DATA_TX_INACTIVE;
}


#endif // WF_EVENT_DRIVEN

#if !defined(WF_EVENT_DRIVEN)
/******************************************************************************
 * Function:        void RawInit(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Initializes the RAW window states.
 *
 * Note:            None
 *****************************************************************************/
void RawInit(void)
{
    /* By default the firmware mounts Scratch to RAW 1 after reset.  If desired,    */
    /* we can read the SysInfo data block from the Scratch.  We are not using this  */
    /* data, so unmount the scratch from this RAW window.                           */
    ScratchUnmount(RAW_ID_1);

    /* Mount scratch memory, index defaults to 0.  This will stay permanently mounted.   */
    /* If one needs to know, this function returns the number of bytes in scratch memory */
    ScratchMount(RAW_SCRATCH_ID);
    
    g_encPtrRAWId[ENC_RD_PTR_ID] = RAW_DATA_RX_ID;
    g_encIndex[ENC_RD_PTR_ID]    = BASE_TCB_ADDR;

    SetRawWindowState(RAW_DATA_TX_ID, WF_RAW_UNMOUNTED);

    g_encPtrRAWId[ENC_WT_PTR_ID]  = RAW_DATA_TX_ID;
    g_encIndex[ENC_WT_PTR_ID]     = BASE_TX_ADDR;                 // set tx encode ptr (index) to start of tx buf + 4 bytes

    SetRxDataPacketLength(0);                                     // current rx buffer length (none) is 0 bytes
    g_txPacketLength  = 0;                                        // current tx packet length (none) is 0 bytes
    #if defined(SYS_DEBUG_ENABLE)
    g_txBufferFlushed = true;                                     // tx buffer is flushed
    #endif
    
    // from ENC MAC init
    // encWrPtr is left pointing to BASE_TX_ADDR
    // encRdPtr is not initialized... we leave it pointing to BASE_TCB_ADDR

    g_RawIndexPastEnd = 0x00; /* no raw indexes have been set past end of raw window */
}    
#endif // !WF_EVENT_DRIVEN




/******************************************************************************
 * Function:        bool MRF24W_MACCheckLink(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          true: If the PHY reports that a link partner is present
 *                        and the link has been up continuously since the last
 *                        call to MRF24W_MACCheckLink()
 *                  false: If the PHY reports no link partner, or the link went
 *                         down momentarily since the last call to MRF24W_MACCheckLink()
 *
 * Side Effects:    None
 *
 * Overview:        Returns the PHSTAT1.LLSTAT bit.
 *
 * Note:            None
 *****************************************************************************/
bool MRF24W_MACCheckLink(void)
{
    return ( WFisConnected() );
}

#if !defined(WF_EVENT_DRIVEN)
/******************************************************************************
 * Function:        bool MRF24W_MACIsTxReady(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          true: If no Ethernet transmission is in progress
 *                  false: If a previous transmission was started, and it has
 *                         not completed yet.  While false, the data in the
 *                         transmit buffer and the TXST/TXND pointers must not
 *                         be changed.
 *
 * Side Effects:    None
 *
 * Overview:        Returns the ECON1.TXRTS bit
 *
 * Note:            None
 *****************************************************************************/
bool MRF24W_MACIsTxReady(void)
{
    bool result;
    
    /* if raw tx data window is in use */
    if (GetRawWindowState(RAW_DATA_TX_ID) == WF_RAW_DATA_MOUNTED)
    {
        result = true;
    }    
    /* else raw tx data window is available */
    else
    {
        /* create the new tx buffer */
        result = AllocateDataTxBuffer(MCHP_DATA_PACKET_SIZE);
    }    

    return result;
}
#endif // !WF_EVENT_DRIVEN

#if defined(WF_EVENT_DRIVEN)
bool MRF24W_MACIsTxReady(void)
{
    bool result;

    // if the data tx task is inactive and ready for next tx data packet
    if (isDataTxTaskInactive())
    {
        // begin allocation of tx packet, but return false because Mac is not yet
        // ready for Tx
        AllocateDataTxBuffer(MCHP_DATA_PACKET_SIZE);
        result = false;
    }
    //
    else
    {
        result = false;
    }

    return result;
}
#endif // WF_EVENT_DRIVEN



void RawGetMgmtRxBuffer(uint16_t *p_numBytes)
{

    /* Mount the mgmt pool rx data, returns number of bytes in mgmt msg.  Read */
    /* index defaults to 0.                                                    */
    *p_numBytes = RawMountRxBuffer(RAW_MGMT_RX_ID);


}



/******************************************************************************
 * Function:        void MRF24W_MACDiscardRx(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Marks the last received packet (obtained using
 *                  MRF24W_MACGetHeader())as being processed and frees the buffer
 *                  memory associated with it
 *
 * Note:            It is safe to call this function multiple times between
 *                  MRF24W_MACGetHeader() calls.  Extra packets won't be thrown away
 *                  until MRF24W_MACGetHeader() makes it available.
 *****************************************************************************/
void MRF24W_MACDiscardRx(void)
{
    /* if we currently have a rx buffer mounted then deallocate it as stack no */
    /* longer needs it.                                                        */
    if ( GetRawWindowState(RAW_DATA_RX_ID) == WF_RAW_DATA_MOUNTED )
    {
        DeallocateDataRxBuffer();
    }
}

/*****************************************************************************
 * FUNCTION: MACIFService
 *
 *
 * RETURNS: Number of bytes in the Data Rx packet if one is received, else 0.
 *
 * PARAMS:  None
 *
 *  NOTES: Called by MACGetHeader() to see if any data packets have been received.
 *         If the MRF24W has received a data packet and the data packet is not
 *         a management data packet, then this function returns the number of
 *         bytes in the data packet. Otherwise it returns 0.
 *****************************************************************************/
static uint16_t MACIFService(void)
{
    uint16_t byteCount;
    tRxPreamble wfPreamble;

    /* check if external interrupt has signalled receipt of a data frame */
    if (!g_HostRAWDataPacketReceived)
    {
        byteCount = 0;  /* return 0 indicating no data rx frame received */
    }
    /* else external interrupt received indicating a data rx frame ready for TCP/IP stack */
    else
    {
        g_HostRAWDataPacketReceived = false; /* clear flag for next data packet */
    
        /* Mount Read FIFO to RAW Rx window.  Allows use of RAW engine to read rx data packet. */
        /* Function call returns number of bytes in the data packet.                           */
        byteCount = RawMountRxBuffer(RAW_DATA_RX_ID);
        SYS_ASSERT((byteCount > 0), "");  
        SetRxDataPacketLength(byteCount);
        
        /* now that buffer mounted it is safe to reenable interrupts, which were left disabled */
        /* in the WiFi interrupt handler.                                                      */
        WF_EintEnable();
    
        /* read the data frame internal preamble (type and subtype) to verify that we did, in   */
        /* fact, mount an Rx data packet.  This read auto-increments the raw index to the first */
        /* actual data byte in the frame.                                                       */
        RawGetByte(RAW_DATA_RX_ID, (uint8_t*)&wfPreamble, sizeof(tRxPreamble));
        SYS_ASSERT((wfPreamble.type == WF_DATA_RX_INDICATE_TYPE), "");
    }

    return byteCount;
}

 
/******************************************************************************
 * Function:        int MRF24W_MACGetHeader(MAC_ADDR *remote, uint16_t* type)
 *
 * PreCondition:    None
 *
 * Input:           *remote: Location to store the Source MAC address of the
 *                           received frame.
 *                  *type: Location of a uint16_t to store the constant
 *                         ETHERTYPE_UNKNOWN, ETHERTYPE_IPVx, or ETHERTYPE_ARP,
 *                         representing the contents of the Ethernet type
 *                         field.
 *
 * Output:          !0: If a packet of this size is waiting in the RX buffer.  The
 *                        remote, and type values are updated.
 *                  0: If a packet was not pending.  remote and type are
 *                         not changed.
 *
 * Side Effects:    Last packet is discarded if MRF24W_MACDiscardRx() hasn't already
 *                  been called.
 *
 * Overview:        None
 *
 * Note:            None
 *****************************************************************************/
int MRF24W_MACGetHeader(MAC_ADDR *remote, uint16_t* type)
{
    uint16_t len;
    t_wfRxPreamble header;
    
    /* stack no longer cares about currently mounted rx packet, so clear flag */
    ClearIndexOutOfBoundsFlag(RAW_DATA_RX_ID);

    /* if we currently have a rx buffer mounted then deallocate it as stack no */
    /* longer needs it.                                                        */
    if ( GetRawWindowState(RAW_DATA_RX_ID) == WF_RAW_DATA_MOUNTED )
    {
        DeallocateDataRxBuffer();
    }

    /* check WiFi chip to see if a data frame has been received.  If so, the length of the */
    /* data frame is returned. If no data packet received then 0 will be returned.         */
    len = MACIFService();
    if ( len == 0 )
    {
        return false;
    }

    /* read preamble header.  The RAW index was left at index 2 in the call to MACIFService, which */
    /* is the logical start of the received data frame LLC Header, also known as the SNAP header.  */
    RawRead(RAW_DATA_RX_ID, ENC_PREAMBLE_OFFSET, WF_RX_PREAMBLE_SIZE, (uint8_t *)&header);  

    /* as a sanity check verify that the expected bytes contain the SNAP header */
    if (!(header.snap[0] == SNAP_VAL        && 
          header.snap[1] == SNAP_VAL        &&
          header.snap[2] == SNAP_CTRL_VAL   &&
          header.snap[3] == SNAP_TYPE_VAL   && 
          header.snap[4] == SNAP_TYPE_VAL   && 
          header.snap[5] == SNAP_TYPE_VAL) )
    {
        /* if a vendor proprietary packet, throw away */
        DeallocateDataRxBuffer();
        return false;
    }
  

#if 0
/* this should no longer be needed */
    /* we can flush any saved RAW state now by saving and restoring the current rx buffer.  */
    PushRawWindow(RAW_DATA_RX_ID);
    PopRawWindow(RAW_DATA_RX_ID); 
#endif
    
    // set RAW pointer to 802.11 payload
    RawSetIndex(RAW_DATA_RX_ID, (ENC_PREAMBLE_OFFSET + WF_RX_PREAMBLE_SIZE));

    g_encPtrRAWId[ENC_RD_PTR_ID] = RAW_DATA_RX_ID;
    g_encIndex[ENC_RD_PTR_ID]    = RXSTART + sizeof(ENC_PREAMBLE);

    // The EtherType field, like most items transmitted on the Ethernet medium
    // are in big endian.
    header.Type.Val = TCPIP_HELPER_ntohs(header.Type.Val);

    // Return the Ethernet frame's Source MAC address field to the caller
    // This parameter is useful for replying to requests without requiring an
    // ARP cycle.
    memcpy((void*)remote->v, (void*)header.SourceMACAddr.v, sizeof(*remote));

    // Return a simplified version of the EtherType field to the caller
    *type = ETHERTYPE_UNKNOWN;
    if( (header.Type.Val == ETHERTYPE_IPV4) || (header.Type.Val == ETHERTYPE_IPV6) || 
        (header.Type.Val == ETHERTYPE_ARP) )
    {
        *type = header.Type.Val;
    }

    return true;
}

#define MAC_IP          (0x00u)
/******************************************************************************
 * Function:        void MRF24W_MACPutHeader(MAC_ADDR *remote, uint16_t type, uint16_t dataLen)
 *
 * PreCondition:    MRF24W_MACIsTxReady() must return true.
 *
 * Input:           *remote: Pointer to memory which contains the destination
 *                           MAC address (6 bytes)
 *                  type: The constant ETHERTYPE_ARP or ETHERTYPE_IPVx, defining
 *                        which value to write into the Ethernet header's type
 *                        field.
 *                  dataLen: Length of the Ethernet data payload
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        None
 *
 * Note:            Because of the dataLen parameter, it is probably
 *                  advantagous to call this function immediately before
 *                  transmitting a packet rather than initially when the
 *                  packet is first created.  The order in which the packet
 *                  is constructed (header first or data first) is not
 *                  important.
 *****************************************************************************/
void MRF24W_MACPutHeader(MAC_ADDR *remote, uint16_t type, uint16_t dataLen)
{
    uint8_t buf[14];
       
    #if defined(SYS_DEBUG_ENABLE)
    g_txBufferFlushed = false;
    #endif
    
    g_txPacketLength = dataLen + (uint16_t)sizeof(MAC_ETHERNET_HEADER) + WF_TX_PREAMBLE_SIZE;

    // Set the SPI write pointer to the beginning of the transmit buffer (post WF_TX_PREAMBLE_SIZE)
    SyncENCPtrRAWState(ENC_WT_PTR_ID, TXSTART + WF_TX_PREAMBLE_SIZE);

    /*  write the Ethernet destination address to buffer (6 bytes) */
    memcpy(&buf[0], (void *)remote, sizeof(*remote));
    /* write snap header to buffer (6 bytes) */
    buf[6] =  SNAP_VAL;         
    buf[7] =  SNAP_VAL;
    buf[8] =  SNAP_CTRL_VAL;
    buf[9] =  SNAP_TYPE_VAL;
    buf[10] = SNAP_TYPE_VAL;
    buf[11] = SNAP_TYPE_VAL;
    /* Write the appropriate Ethernet Type uint16_t for the protocol being used */
    buf[12] = (type >> 8) & 0xFF;                
    buf[13] = type & 0xFF;

    /* write buffer to RAW window */
    MRF24W_MACPutArray((uint8_t *)buf, sizeof(buf));
}



/******************************************************************************
 * Function:        void MRF24W_MACFlush(void)
 *
 * PreCondition:    A packet has been created by calling MRF24W_MACPutArray() and
 *                  MRF24W_MACPutHeader().
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        MRF24W_MACFlush causes the current TX packet to be sent out on
 *                  the Ethernet medium.  The hardware MAC will take control
 *                  and handle CRC generation, collision retransmission and
 *                  other details.
 *
 * Note:            After transmission completes (MRF24W_MACIsTxReady() returns true),
 *                  the packet can be modified and transmitted again by calling
 *                  MRF24W_MACFlush() again.  Until MRF24W_MACPutHeader() or MRF24W_MACPutArray() is
 *                  called (in the TX data area), the data in the TX buffer
 *                  will not be corrupted.
 *****************************************************************************/
void MRF24W_MACFlush(void)
{
   
    /* this function should not be called after the current tx buffer has been transmitted */
    SYS_ASSERT(!g_txBufferFlushed, "");

    #if defined(SYS_DEBUG_ENABLE)
    g_txBufferFlushed = true;
    #endif

    EnsureWFisAwake();
    SendRAWDataFrame(g_txPacketLength);
}

/******************************************************************************
 * Function:        void MRF24W_MACSetReadPtrInRx(uint16_t offset)
 *
 * PreCondition:    A packet has been obtained by calling MRF24W_MACGetHeader() and
 *                  getting a true result.
 *
 * Input:           offset: uint16_t specifying how many bytes beyond the Ethernet
 *                          header's type field to relocate the SPI read
 *                          pointer.
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        SPI read pointer are updated.  All calls to
 *                  MRF24W_MACGetArray() will use these new values.
 *
 * Note:            RXSTOP must be statically defined as being > RXSTART for
 *                  this function to work correctly.  In other uint16_ts, do not
 *                  define an RX buffer which spans the 0x1FFF->0x0000 memory
 *                  boundary.
 *****************************************************************************/
void MRF24W_MACSetReadPtrInRx(uint16_t offset)
{
    SyncENCPtrRAWState(ENC_RD_PTR_ID, RXSTART + sizeof(ENC_PREAMBLE) + offset);
}

TCPIP_MAC_PTR_TYPE MRF24W_MACGetReadPtrInRx(void)
{
    return g_encIndex[ENC_RD_PTR_ID];
}

/******************************************************************************
 * Function:        uint16_t MRF24W_MACSetWritePtr(uint16_t Address)
 *
 * PreCondition:    None
 *
 * Input:           Address: Address to seek to
 *
 * Output:          uint16_t: Old EWRPT location
 *
 * Side Effects:    None
 *
 * Overview:        SPI write pointer is updated.  All calls to
 *                  MRF24W_MACPutArray() will use this new value.
 *
 * Note:            None
 *****************************************************************************/
TCPIP_MAC_PTR_TYPE MRF24W_MACSetWritePtr(TCPIP_MAC_PTR_TYPE address)
{
    TCPIP_MAC_PTR_TYPE oldVal;

    oldVal = g_encIndex[ENC_WT_PTR_ID];

    SyncENCPtrRAWState(ENC_WT_PTR_ID, address);

    return oldVal;
}

/******************************************************************************
 * Function:        uint16_t MRF24W_MACSetReadPtr(uint16_t Address)
 *
 * PreCondition:    None
 *
 * Input:           Address: Address to seek to
 *
 * Output:          uint16_t: Old ERDPT value
 *
 * Side Effects:    None
 *
 * Overview:        SPI write pointer is updated.  All calls to
 *                  MRF24W_MACPutArray() will use this new value.
 *
 * Note:            None
 *****************************************************************************/
TCPIP_MAC_PTR_TYPE MRF24W_MACSetReadPtr(TCPIP_MAC_PTR_TYPE address)
{
    TCPIP_MAC_PTR_TYPE oldVal;

    oldVal = g_encIndex[ENC_RD_PTR_ID];

    SyncENCPtrRAWState(ENC_RD_PTR_ID, address);

    return oldVal;
}

/******************************************************************************
 * Function:        uint16_t MRF24W_MACSetBaseReadPtr(uint16_t Address)
 *
 * PreCondition:    None
 *
 * Input:           Address: Address to seek to
 *
 * Output:          uint16_t: Old ERDPT value
 *
 * Side Effects:    None
 *
 * Overview:        SPI write pointer is updated.  All calls to
 *                  MRF24W_MACPutArray() will use this new value.
 *
 * Note:            None
 *****************************************************************************/
TCPIP_MAC_PTR_TYPE MRF24W_MACSetBaseReadPtr(TCPIP_MAC_PTR_TYPE address)
{
    TCPIP_MAC_PTR_TYPE oldVal;

    oldVal = g_encIndex[ENC_RD_PTR_ID];

    SyncENCPtrRAWState(ENC_RD_PTR_ID, address);

    return oldVal;
}



/******************************************************************************
 * Function:        uint16_t MRF24W_MACCalcRxChecksum(TCPIP_MAC_HANDLE hMac, uint16_t offset, uint16_t len)
 *
 * PreCondition:    None
 *
 * Input:           offset  - Number of bytes beyond the beginning of the
 *                          Ethernet data (first byte after the type field)
 *                          where the checksum should begin
 *                  len     - Total number of bytes to include in the checksum
 *
 * Output:          16-bit checksum as defined by RFC 793.
 *
 * Side Effects:    None
 *
 * Overview:        This function performs a checksum calculation in the MAC
 *                  buffer itself
 *
 * Note:            None
 *****************************************************************************/
uint16_t MRF24W_MACCalcRxChecksum(TCPIP_MAC_HANDLE hMac, uint16_t offset, uint16_t len)
{
    uint16_t temp;
    uint16_t rdSave;

    // Add the offset requested by firmware plus the Ethernet header
    temp = RXSTART + sizeof(ENC_PREAMBLE) + offset;

    rdSave = g_encIndex[ENC_RD_PTR_ID];

    SyncENCPtrRAWState(ENC_RD_PTR_ID, temp);

    temp = TCPIP_Helper_CalcMACBufferIPChecksum(hMac, len);

    SyncENCPtrRAWState(ENC_RD_PTR_ID, rdSave);

    return temp;
}

/******************************************************************************
 * Function:        uint16_t MRF24W_MACCalcIPBufferChecksum(TCPIP_MAC_HANDLE hMac, uint16_t len)
 *
 * PreCondition:    None
 *
 * Input:          len     - Total number of bytes to include in the checksum
 *
 * Output:          16-bit checksum as defined by RFC 793.
 *
 * Side Effects:    None
 *
 * Overview:        This function performs a checksum calculation in the MAC
 *                  buffer pointed by the current value of the read pointer.
 *
 * Note:            None
 *****************************************************************************/
uint16_t MRF24W_MACCalcIPBufferChecksum(TCPIP_MAC_HANDLE hMac, uint16_t len)
{
    uint16_t checksum;

    // do checksum calculation via the checksum hardware engine
    checksum = TCPIP_HELPER_htons(RawCalculateChecksum(len));
        
    return checksum;

#if 0
    return TCPIP_Helper_CalcMACBufferIPChecksum(hMac, len);
#endif    
}


/******************************************************************************
 * Function:        uint16_t MRF24W_MACGetArray(uint8_t *val, uint16_t len)
 *
 * PreCondition:    SPI bus must be initialized (done in MRF24W_MACInit()).
 *                  ERDPT must point to the place to read from.
 *
 * Input:           *val: Pointer to storage location
 *                  len:  Number of bytes to read from the data buffer.
 *
 * Output:          uint8_t(s) of data read from the data buffer.
 *
 * Side Effects:    None
 *
 * Overview:        Burst reads several sequential bytes from the data buffer
 *                  and places them into local memory.  With SPI burst support,
 *                  it performs much faster than multiple single byte calls.
 *                  ERDPT is incremented after each byte
 *
 * Note:            None
 *****************************************************************************/
uint16_t MRF24W_MACGetArray(uint8_t *val, uint16_t len)
{
    uint16_t i = 0;
    uint8_t byte;


    if ( val )
    {
        RawGetByte(g_encPtrRAWId[ENC_RD_PTR_ID], val, len);
    }
    else
    {
        // Read the data
        while(i<len)
        {
            RawGetByte(g_encPtrRAWId[ENC_RD_PTR_ID], &byte, 1);
            i++;
        }
    }
    g_encIndex[ENC_RD_PTR_ID] += len;

    return len;
}//end MRF24W_MACGetArray

/******************************************************************************
 * Function:        void MRF24W_MACPutArray(uint8_t *val, uint16_t len)
 *
 * PreCondition:    SPI bus must be initialized (done in MRF24W_MACInit()).
 *                  EWRPT must point to the location to begin writing.
 *
 * Input:           *val: Pointer to source of bytes to copy.
 *                  len:  Number of bytes to write to the data buffer.
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        MRF24W_MACPutArray writes several sequential bytes to the
 *                  MRF24W RAM.  
 *                  EWRPT is incremented by len.
 *
 * Note:            None
 *****************************************************************************/
void MRF24W_MACPutArray(uint8_t *val, uint16_t len)
{
    RawSetByte(g_encPtrRAWId[ENC_WT_PTR_ID], val, len);

    g_encIndex[ENC_WT_PTR_ID] += len;
    
}//end MRF24W_MACPutArray



/******************************************************************************
 * Function:        void MRF24W_MACPowerDown(void)
 *
 * PreCondition:    SPI bus must be initialized (done in MRF24W_MACInit()).
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        MRF24W_MACPowerDown puts the MRF24W in low power sleep mode. In
 *                  sleep mode, no packets can be transmitted or received.
 *                  All MAC and PHY registers should not be accessed.
 *
 * Note:            If a packet is being transmitted while this function is
 *                  called, this function will block until it is it complete.
 *                  If anything is being received, it will be completed.
 *****************************************************************************/
void MRF24W_MACPowerDown(void)
{
}//end MRF24W_MACPowerDown


/******************************************************************************
 * Function:        void MRF24W_MACPowerUp(void)
 *
 * PreCondition:    SPI bus must be initialized (done in MRF24W_MACInit()).
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        MRF24W_MACPowerUp returns the MRF24W back to normal operation
 *                  after a previous call to MRF24W_MACPowerDown().  Calling this
 *                  function when already powered up will have no effect.
 *
 * Note:            If a link partner is present, it will take 10s of
 *                  milliseconds before a new link will be established after
 *                  waking up.  While not linked, packets which are
 *                  transmitted will most likely be lost.  MRF24W_MACCheckLink() can
 *                  be called to determine if a link is established.
 *****************************************************************************/
void MRF24W_MACPowerUp(void)
{
}//end MRF24W_MACPowerUp


#if defined(WF_EVENT_DRIVEN)

// creates timers used to check for timeouts on data tx/rx raw moves
void CreateRawMoveTimers(void)
{
    // if this is the first call to this function
    if ((g_DataRxRawMoveTimer == NULL) && (g_DataTxRawMoveTimer == NULL) && (g_ScratchRawMoveTimer == NULL))
    {
        // create the timers and put them into a stopped state
        g_DataRxRawMoveTimer  = SYS_TICK_TimerCreate(RawMoveDataRxTimeoutHandler);
        g_DataTxRawMoveTimer  = SYS_TICK_TimerCreate(RawMoveDataTxTimeoutHandler);
        g_ScratchRawMoveTimer = SYS_TICK_TimerCreate(RawMoveScratchTimeoutHandler);
        SYS_ASSERT((g_DataRxRawMoveTimer != NULL) || 
                   (g_DataTxRawMoveTimer != NULL) ||
                    g_ScratchRawMoveTimer != NULL, "raw timer creation failed");

        SYS_TICK_TimerSetRate(g_DataRxRawMoveTimer,  0);  // timer is stopped
        SYS_TICK_TimerSetRate(g_DataTxRawMoveTimer,  0);  // timer is stopped
        SYS_TICK_TimerSetRate(g_ScratchRawMoveTimer, 0);  // timer is stopped
    }
}

// called by RawMove()
void StartRawMoveTimer(uint16_t rawId)
{
    uint16_t timeout = SYS_TICK_TicksPerSecondGet() / 2; // 500ms

    if (rawId == RAW_DATA_RX_ID)
    {
        g_rxDataRawMoveTimeout = false;
        SYS_TICK_TimerSetRate(g_DataRxRawMoveTimer, timeout);
    }
    else if (rawId == RAW_DATA_TX_ID)
    {
        g_txDataRawMoveTimeout = false;
        SYS_TICK_TimerSetRate(g_DataTxRawMoveTimer, timeout);
    }
    else // must be RAW_SCRATCH_ID (only at init)
    {
        g_scratchRawMoveTimeout = false;
        SYS_TICK_TimerSetRate(g_ScratchRawMoveTimer, timeout);
    }
}

void StopRawMoveTimer(uint16_t rawId)
{
    if (rawId == RAW_DATA_RX_ID)
    {
        g_rxDataRawMoveTimeout = false;
        SYS_TICK_TimerSetRate(g_DataRxRawMoveTimer, 0);
    }
    else if (rawId == RAW_DATA_TX_ID)
    {
        g_txDataRawMoveTimeout = false;
        SYS_TICK_TimerSetRate(g_DataTxRawMoveTimer, 0);
    }
    else // must be RAW_SCRATCH_ID (only at init)
    {
        g_scratchRawMoveTimeout = false;
        SYS_TICK_TimerSetRate(g_ScratchRawMoveTimer, 0);
    }
}

bool isRawMoveTimeout(uint16_t rawId)
{
    if (rawId == RAW_DATA_RX_ID)
    {
        return (g_rxDataRawMoveTimeout);
    }
    else if (rawId == RAW_DATA_TX_ID)
    {
        return (g_txDataRawMoveTimeout);
    }
    else // must be RAW_SCRATCH_ID (only at init)
    {
        return (g_scratchRawMoveTimeout);
    }
}


// called from timer interrupt when timeout occurs
static void RawMoveDataRxTimeoutHandler(SYS_TICK currSysTick)
{
    currSysTick = currSysTick;      // avoid warning
    g_rxDataRawMoveTimeout = true;
    StopRawMoveTimer(RAW_DATA_RX_ID);
}

// called from timer interrupt when timeout occurs
static void RawMoveDataTxTimeoutHandler(SYS_TICK currSysTick)
{
    currSysTick = currSysTick;      // avoid warning
    g_txDataRawMoveTimeout = true;
    StopRawMoveTimer(RAW_DATA_TX_ID);
}

static void RawMoveScratchTimeoutHandler(SYS_TICK currSysTick)
{
    currSysTick = currSysTick;      // avoid warning
    g_scratchRawMoveTimeout = true;
    StopRawMoveTimer(RAW_SCRATCH_ID);
}

// Called from ProcessInterruptServiceResult() when Rx data interrupt detected;
// Raw mount of rx data packet has been started by that function.
void InitDataRxStateMachine(void)
{
    g_rxDataState = WAIT_FOR_DATA_RX_MOUNT;
}

// called by Raw Move function that mounted the Rx data packet
void SetRxDataByteCount(uint16_t byteCount)
{
    g_rxByteCount = byteCount;
}

// called when Raw move complete event occurs
void WiFi_DataRxTask(void)
{
    int status;
    uint16_t byteCount;

    switch (g_rxDataState)
    {
        case WAIT_FOR_DATA_RX_MOUNT:
            // this should always be true as the event is only triggered, and this
            // function should only be called after the Raw Move has completed.
            if (isRawMoveComplete(&status, &byteCount))
            {
                SYS_ASSERT(byteCount > 0, "Invalid Rx byte count");

                // read Rx packet into host buffer
                // TBD

                // begin raw unmount of data rx packet
                DeallocateDataRxBuffer();

                // signal stack that rx packet
                // TBD

                g_rxDataState = WAIT_FOR_DATA_RX_UNMOUNT;
            }
            // else Raw Move not yet complete (still waiting)
            else
            {
                SYS_ASSERT(status != RM_TIMEOUT, "Rx Raw Move timeout");
            }
            break;

        case WAIT_FOR_DATA_RX_UNMOUNT:
            if (isRawMoveComplete(&status, &byteCount))
            {
                // nothing to do; we are now ready to receive next data message
            }
            else
            {
                SYS_ASSERT(status != RM_TIMEOUT, "Invalid Raw Move timeout");
            }

            g_rxDataState = WAIT_FOR_DATA_RX_MOUNT;
            break;
    } // end switch
}

void InitDataTxStateMachine(void)
{
    g_txDataState = WAIT_FOR_DATA_TX_MOUNT;
}


bool isDataTxTaskInactive(void)
{
    return (g_txDataState == DATA_TX_INACTIVE);
}

void WiFi_DataTxTask(void)
{
    int status;
    uint16_t byteCount;

    switch (g_txDataState)
    {

        case DATA_TX_INACTIVE:
            SYS_ASSERT(false, "");  // should never be called when in this state
            break;

        case WAIT_FOR_DATA_TX_MOUNT:
            // if tx data buffer successfully mounted
            if (isRawMoveComplete(&status, &byteCount))
            {
                SYS_ASSERT(byteCount > 0, "Invalid Tx byte count");

                // TBD: after Adrian finishes mod to MAC layer here is where we
                //      would copy tx data packet from host memory to MRF24WG and
                //      and signal via raw move, that packet ready to transmit

                g_txDataState = WAIT_FOR_DATA_TX_SIGNAL;
            }
            else
            {
                SYS_ASSERT(status != RM_TIMEOUT, "Tx Raw Move timeout");
            }

        case WAIT_FOR_DATA_TX_SIGNAL:
            // if raw move complete that signals MRF24WG that there is a tx data
            // packet to transmit
            if (isRawMoveComplete(&status, &byteCount))
            {
                // TBD: signal stack OK to transmit next data packet

                g_txDataState = DATA_TX_INACTIVE; // ready for next tx data
            }
            else
            {
                SYS_ASSERT(status != RM_TIMEOUT, "Tx Raw Move timeout");
            }
            break;

    }
}

#endif // WF_EVENT_DRIVEN


#endif /* TCPIP_IF_MRF24W*/


