/*******************************************************************************
  MRF24W Driver API Interface

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Provides access to MRF24W WiFi controller
    -Reference: MRF24W Data sheet, IEEE 802.11 Standard
*******************************************************************************/

/*******************************************************************************
FileName:  wf_api.h 
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

#ifndef __WF_API_H_
#define __WF_API_H_


#include <stdint.h>
#include <stdbool.h>

#if 0
// debug, remove later
#define PA6_IO                LATAbits.LATA6
#define PA6_TRISTATE          TRISAbits.TRISA6
#define SET_PA6_AS_OUTPUT()   TRISAbits.TRISA6 = 0
#endif


/*
*********************************************************************************************************
*                                           DEFINES                               
*********************************************************************************************************
*/

// WiFi Driver Version Number
#define WF_HOST_DRIVER_VERSION_NUMBER           "3.0.0"

#define WF_MAX_SSID_LENGTH                      (32)
#define WF_BSSID_LENGTH                         (6)
#define WF_RETRY_FOREVER                        (255)
#define WF_RETRY_ADHOC                          (3)

#define WF_CHANNEL_LIST_LENGTH                  (14)
#define WF_MAX_SECURITY_KEY_LENGTH              (64)

#define WF_RTS_THRESHOLD_MAX                    (2347) /* maximum RTS threshold size in bytes */
#define WF_MAX_NUM_RATES                        (8)

/* Key size defines */
#define WF_MIN_WPA_PASS_PHRASE_LENGTH           (8)
#define WF_MIN_WPA_PASS_PHRASE_LENGTH           (8)
#define WF_MAX_WPA_PASS_PHRASE_LENGTH           (64)  // must include string terminator
#define WF_MAX_WPA2_PASS_PHRASE_LENGTH          (64)
#define WF_WPA_KEY_LENGTH                       (32)
#define WF_WPA2_KEY_LENGTH                      (32)

// WEP key lengths
#define WF_WEP40_KEY_LENGTH                     (20)        // 4 keys of 5 bytes each
#define WF_WEP104_KEY_LENGTH                    (52)        // 4 keys of 13 bytes each
#define WF_MAX_WEP_KEY_LENGTH                   (WF_WEP104_KEY_LENGTH)

// Wep key types
typedef enum
{
    WF_SECURITY_WEP_SHAREDKEY = 0,
    WF_SECURITY_WEP_OPENKEY   = 1
} t_wepKeyType;

//------------------------------------------------------------------------------
// These are error codes returned in the result field of a management response.
//------------------------------------------------------------------------------
typedef enum
{
    WF_SUCCESS                                              = 1,
    WF_ERROR_INVALID_SUBTYPE                                = 2,
    WF_ERROR_OPERATION_CANCELLED                            = 3,
    WF_ERROR_FRAME_END_OF_LINE_OCCURRED                     = 4,
    WF_ERROR_FRAME_RETRY_LIMIT_EXCEEDED                     = 5,
    WF_ERROR_EXPECTED_BSS_VALUE_NOT_IN_FRAME                = 6,
    WF_ERROR_FRAME_SIZE_EXCEEDS_BUFFER_SIZE                 = 7,
    WF_ERROR_FRAME_ENCRYPT_FAILED                           = 8,
    WF_ERROR_INVALID_PARAM                                  = 9,
    WF_ERROR_AUTH_REQ_ISSUED_WHILE_IN_AUTH_STATE            = 10,
    WF_ERROR_ASSOC_REQ_ISSUED_WHILE_IN_ASSOC_STATE          = 11,
    WF_ERROR_INSUFFICIENT_RESOURCES                         = 12,
    WF_ERROR_TIMEOUT_OCCURRED                               = 13,
    WF_ERROR_BAD_EXCHANGE_ENCOUNTERED_IN_FRAME_RECEPTION    = 14,
    WF_ERROR_AUTH_REQUEST_REFUSED                           = 15,
    WF_ERROR_ASSOCIATION_REQUEST_REFUSED                    = 16,
    WF_ERROR_PRIOR_MGMT_REQUEST_IN_PROGRESS                 = 17,
    WF_ERROR_NOT_IN_JOINED_STATE                            = 18,
    WF_ERROR_NOT_IN_ASSOCIATED_STATE                        = 19,
    WF_ERROR_NOT_IN_AUTHENTICATED_STATE                     = 20,
    WF_ERROR_SUPPLICANT_FAILED                              = 21,
    WF_ERROR_UNSUPPORTED_FEATURE                            = 22,
    WF_ERROR_REQUEST_OUT_OF_SYNC                            = 23,
    WF_ERROR_CP_INVALID_ELEMENT_TYPE                        = 24,
    WF_ERROR_CP_INVALID_PROFILE_ID                          = 25,
    WF_ERROR_CP_INVALID_DATA_LENGTH                         = 26,
    WF_ERROR_CP_INVALID_SSID_LENGTH                         = 27,
    WF_ERROR_CP_INVALID_SECURITY_TYPE                       = 28,
    WF_ERROR_CP_INVALID_SECURITY_KEY_LENGTH                 = 29,
    WF_ERROR_CP_INVALID_WEP_KEY_ID                          = 30,
    WF_ERROR_CP_INVALID_NETWORK_TYPE                        = 31,
    WF_ERROR_CP_INVALID_ADHOC_MODE                          = 32,
    WF_ERROR_CP_INVALID_SCAN_TYPE                           = 33,
    WF_ERROR_CP_INVALID_CP_LIST                             = 34,
    WF_ERROR_CP_INVALID_CHANNEL_LIST_LENGTH                 = 35,
    WF_ERROR_NOT_CONNECTED                                  = 36,
    WF_ERROR_ALREADY_CONNECTING                             = 37,
    WF_ERROR_DISCONNECT_FAILED                              = 38, // Disconnect failed. Disconnect is allowed only when module is in connected state
    WF_ERROR_NO_STORED_BSS_DESCRIPTOR                       = 39, // No stored scan results
    WF_ERROR_INVALID_MAX_POWER                              = 40,
    WF_ERROR_CONNECTION_TERMINATED                          = 41,
    WF_ERROR_HOST_SCAN_NOT_ALLOWED                          = 42, // Host Scan Failed. Host scan is allowed only in idle or connected state
    WF_ERROR_INVALID_WPS_PIN                                = 44  // WPS pin was invalid
} t_mgmtErrors;

#if 0
// other errors associated with WF_EVENT_ERROR
typedef enum
{
    //placeholder, add error events as needed

} t_wfErrorEvents;
#endif


#define WF_NOTIFY_CONNECTION_ATTEMPT_SUCCESSFUL     ((uint8_t)(0x01))  
#define WF_NOTIFY_CONNECTION_ATTEMPT_FAILED         ((uint8_t)(0x02))  
#define WF_NOTIFY_CONNECTION_TEMPORARILY_LOST       ((uint8_t)(0x04))  
#define WF_NOTIFY_CONNECTION_PERMANENTLY_LOST       ((uint8_t)(0x08))  
#define WF_NOTIFY_CONNECTION_REESTABLISHED          ((uint8_t)(0x10))  
#define WF_NOTIFY_ALL_EVENTS                        ((uint8_t)(0x1f))

// Tx Mode options
typedef enum
{
    WF_TXMODE_G_RATES      = 0,
    WF_TXMODE_B_RATES      = 1,
    WF_TXMODE_LEGACY_RATES = 2
} t_txMode;

// Multicast Filter ID's
// Infrastructure can use 2,3,4,5 and AdHoc can only use 4,5.  Use 4,5 which works for both.
typedef enum
{
    WF_MULTICAST_FILTER_1       = 4,
    WF_MULTICAST_FILTER_2       = 5,
    WF_MULTICAST_FILTER_3       = 6,
    WF_MULTICAST_FILTER_4       = 7,
    WF_MULTICAST_FILTER_5       = 8,
    WF_MULTICAST_FILTER_6       = 9,
    WF_MULTICAST_FILTER_7       = 10,
    WF_MULTICAST_FILTER_8       = 11,
    WF_MULTICAST_FILTER_9       = 12,
    WF_MULTICAST_FILTER_10      = 13,
    WF_MULTICAST_FILTER_11      = 14,
    WF_MULTICAST_FILTER_12      = 15,
    WF_MULTICAST_FILTER_13      = 16,
    WF_MULTICAST_FILTER_14      = 17,
    WF_MULTICAST_FILTER_15      = 18,
    WF_MULTICAST_FILTER_16      = 19
} t_multicastFilterId;

typedef enum
{
    WF_MULTICAST_DISABLE_ALL  = 0,
    WF_MULTICAST_ENABLE_ALL   = 1,
    WF_MULTICAST_USE_FILTERS  = 2
} t_multicastFilterAction;

#define WF_MASK_DEAUTH_REASONCODE       ((uint8_t)0x80)
#define WF_MASK_DISASSOC_REASONCODE     ((uint8_t)0x40)

#define WF_SCAN_ALL                     ((uint8_t)(0xff))

// Events that can be invoked in WF_ProcessEvent()
typedef enum
{
    WF_EVENT_CONNECTION_SUCCESSFUL          = 1,   // Connection attempt to network successful
    WF_EVENT_CONNECTION_FAILED              = 2,   // Connection attempt failed
    WF_EVENT_CONNECTION_TEMPORARILY_LOST    = 3,   // Connection lost; MRF24W attempting to reconnect
    WF_EVENT_CONNECTION_PERMANENTLY_LOST    = 4,   // Connection lost; MRF24W no longer trying to connect
    WF_EVENT_CONNECTION_REESTABLISHED       = 5,
    WF_EVENT_FLASH_UPDATE_SUCCESSFUL        = 6,   // Update to FLASH successful
    WF_EVENT_FLASH_UPDATE_FAILED            = 7,   // Update to FLASH failed
    WF_EVENT_KEY_CALCULATION_REQUEST        = 8,   // Key calculation is required
    WF_EVENT_SCAN_RESULTS_READY             = 9,   // scan results are ready
    WF_EVENT_IE_RESULTS_READY               = 10,  // IE data ready
    WF_EVENT_INVALID_WPS_PIN                = 12,  // Invalid WPS pin was entered
    WF_EVENT_SOFT_AP                        = 13,  // Client connection events
    WF_EVENT_DISCONNECT_DONE                = 14,  // Disconnect done event
    WF_EVENT_ERROR                          = 15   // WiFi error event occurred
} t_events;

// see t_scanContext (default values upon MRF24WG reset)
#define WF_DEFAULT_SCAN_COUNT                   (1)
#define WF_DEFAULT_SCAN_MIN_CHANNEL_TIME        (200)   // ms
#define WF_DEFAULT_SCAN_MAX_CHANNEL_TIME        (400)   // ms
#define WF_DEFAULT_SCAN_PROBE_DELAY             (20)    // us

// see t_psPollContext
#define WF_DEFAULT_PS_LISTEN_INTERVAL           ((uint16_t)1)       // 100ms multiplier, e.g. 1 * 100ms = 100ms
#define WF_DEFAULT_PS_DTIM_INTERVAL             ((uint16_t)2)       // number of beacon periods
#define WF_DEFAULT_PS_DTIM_ENABLED              true                // DTIM wake-up enabled (normally the case)

// see t_adHocNetworkContext
#define WF_DEFAULT_ADHOC_HIDDEN_SSID            false
#define WF_DEFAULT_ADHOC_BEACON_PERIOD          (100)   // ms
#define WF_DEFAULT_ADHOC_MODE                   WF_ADHOC_CONNECT_THEN_START

// see WF_SecurityWepSet() and t_WepContext
#define WF_DEFAULT_WEP_KEY_INDEX                (0)
#define WF_DEFAULT_WEP_KEY_TYPE                 WF_SECURITY_WEP_OPENKEY


// Ad Hoc modes
typedef enum adhocMode
{
    WF_ADHOC_CONNECT_THEN_START = 0,
    WF_ADHOC_CONNECT_ONLY       = 1,
    WF_ADHOC_START_ONLY         = 2
} t_adhocMode;


//----------------------------------------------------------
// Network Type defines (do not make these enumerated types)
//----------------------------------------------------------
#define WF_NETWORK_TYPE_INFRASTRUCTURE          (1)
#define WF_NETWORK_TYPE_ADHOC                   (2)
#define WF_NETWORK_TYPE_P2P                     (3)
#define WF_NETWORK_TYPE_SOFT_AP                 (4)

//-----------------------------------------------------------
// Security Type defines (do not make these enumerated types)
//-----------------------------------------------------------
#define WF_SECURITY_OPEN                        (0)
#define WF_SECURITY_WEP_40                      (1)
#define WF_SECURITY_WEP_104                     (2)
#define WF_SECURITY_WPA_WITH_KEY                (3)
#define WF_SECURITY_WPA_WITH_PASS_PHRASE        (4)
#define WF_SECURITY_WPA2_WITH_KEY               (5)
#define WF_SECURITY_WPA2_WITH_PASS_PHRASE       (6)
#define WF_SECURITY_WPA_AUTO_WITH_KEY           (7)
#define WF_SECURITY_WPA_AUTO_WITH_PASS_PHRASE   (8)
#define WF_SECURITY_WPS_PUSH_BUTTON             (9)
#define WF_SECURITY_WPS_PIN                     (10)
#define WF_SECURITY_EAP                         (11)

#define WF_WPS_PIN_LENGTH                       (8)         // 7 digits + checksum byte

//------------------
// Scan type defines
//------------------
typedef enum
{
    WF_ACTIVE_SCAN  = 1,
    WF_PASSIVE_SCAN = 2
} t_scanType;

//----------------------------------
// Beacon Timeout and Deauth defines
//----------------------------------
typedef enum
{
    WF_DO_NOT_ATTEMPT_TO_RECONNECT = 0,
    WF_ATTEMPT_TO_RECONNECT        = 1
} t_reconnectMode;


// Do not make this an enumerated type!
#define WF_DISABLED     (0)
#define WF_ENABLED      (1)


// eventInfo defines for WF_ProcessEvent(), case WF_EVENT_CONNECTION_FAILED.
// Also value for index 3 of WF_CONNECTION_FAILED_EVENT_SUBTYPE
typedef enum
{
    WF_JOIN_FAILURE                         = 2,
    WF_AUTHENTICATION_FAILURE               = 3,
    WF_ASSOCIATION_FAILURE                  = 4,
    WF_WEP_HANDSHAKE_FAILURE                = 5,
    WF_PSK_CALCULATION_FAILURE              = 6,
    WF_PSK_HANDSHAKE_FAILURE                = 7,
    WF_ADHOC_JOIN_FAILURE                   = 8,
    WF_SECURITY_MISMATCH_FAILURE            = 9,
    WF_NO_SUITABLE_AP_FOUND_FAILURE         = 10,
    WF_RETRY_FOREVER_NOT_SUPPORTED_FAILURE  = 11,
    WF_LINK_LOST                            = 12,
    WF_TKIP_MIC_FAILURE                     = 13,
    WF_RSN_MIXED_MODE_NOT_SUPPORTED         = 14,
    WF_RECV_DEAUTH                          = 15,
    WF_RECV_DISASSOC                        = 16,
    WF_WPS_FAILURE                          = 17,
    WF_P2P_FAILURE                          = 18
} t_evenInfo;

// Reason Codes
typedef enum
{
    WF_UNSPECIFIED                          = 1,
    WF_REASON_PREV_AUTH_NOT_VALID           = 2,
    WF_DEAUTH_LEAVING                       = 3,
    WF_DISASSOC_DUE_TO_INACTIVITY           = 4,
    WF_DISASSOC_AP_BUSY                     = 5,
    WF_CLASS2_FRAME_FROM_NONAUTH_STA        = 6,
    WF_CLASS3_FRAME_FROM_NONASSOC_STA       = 7,
    WF_DISASSOC_STA_HAS_LEFT                = 8,
    WF_STA_REQ_ASSOC_WITHOUT_AUTH           = 9,
    WF_INVALID_IE                           = 13,
    WF_MIC_FAILURE                          = 14,
    WF_4WAY_HANDSHAKE_TIMEOUT               = 15,
    WF_GROUP_KEY_HANDSHAKE_TIMEOUT          = 16,
    WF_IE_DIFFERENT                         = 17,
    WF_INVALID_GROUP_CIPHER                 = 18,
    WF_INVALID_PAIRWISE_CIPHER              = 19,
    WF_INVALID_AKMP                         = 20,
    WF_UNSUPP_RSN_VERSION                   = 21,
    WF_INVALID_RSN_IE_CAP                   = 22,
    WF_IEEE8021X_FAILED                     = 23,
    WF_CIPHER_SUITE_REJECTED                = 24
} t_reasonCode;

// eventInfo defines for WF_ProcessEvent(), case WF_EVENT_CONNECTION_TEMPORARILY_LOST
typedef enum
{
    WF_BEACON_TIMEOUT                       = 1,
    WF_DEAUTH_RECEIVED                      = 2,
    WF_DISASSOCIATE_RECEIVED                = 3
} t_eventConnTempLost;

// Status Codes
typedef enum
{
    WF_UNSPECIFIED_FAILURE                  = 1,
    WF_CAPS_UNSUPPORTED                     = 10,
    WF_REASSOC_NO_ASSOC                     = 11,
    WF_ASSOC_DENIED_UNSPEC                  = 12,
    WF_NOT_SUPPORTED_AUTH_ALG               = 13,
    WF_UNKNOWN_AUTH_TRANSACTION             = 14,
    WF_CHALLENGE_FAIL                       = 15,
    WF_AUTH_TIMEOUT                         = 16,
    WF_AP_UNABLE_TO_HANDLE_NEW_STA          = 17,
    WF_ASSOC_DENIED_RATES                   = 18,
    WF_ASSOC_DENIED_NOSHORTPREAMBLE         = 19,
    WF_ASSOC_DENIED_NOPBCC                  = 20,
    WF_ASSOC_DENIED_NOAGILITY               = 21,
    WF_ASSOC_DENIED_NOSHORTTIME             = 25,
    WF_ASSOC_DENIED_NODSSSOFDM              = 26,
    WF_S_INVALID_IE                         = 40,
    WF_S_INVALID_GROUPCIPHER                = 41,
    WF_S_INVALID_PAIRWISE_CIPHER            = 42,
    WF_S_INVALID_AKMP                       = 43,
    WF_UNSUPPORTED_RSN_VERSION              = 44,
    WF_S_INVALID_RSN_IE_CAP                 = 45,
    WF_S_CIPHER_SUITE_REJECTED              = 46,
    WF_TIMEOUT                              = 47
} t_statusCode;

// WiFi Device Types
typedef enum
{
    MRF24WB0M_DEVICE = 1,
    MRF24WG0M_DEVICE = 2
} t_deviceType;


// domain codes
typedef enum
{
    WF_DOMAIN_FCC   = 0, // Available Channels: 1 - 11
    WF_DOMAIN_ETSI  = 2, // Available Channels: 1 - 13
    WF_DOMAIN_JAPAN = 7, // Available Channels: 1 - 14
    WF_DOMAIN_OTHER = 7  // Available Channels: 1 - 14 
} t_domainCode;

// Power save states
typedef enum
{
    WF_PS_HIBERNATE             = 1,
    WF_PS_PS_POLL_DTIM_ENABLED  = 2,
    WF_PS_PS_POLL_DTIM_DISABLED = 3,
    WF_PS_OFF                   = 4
} t_powerSaveState;

// Hibernate states
typedef enum
{
    WF_HB_NO_SLEEP      = 0,
    WF_HB_ENTER_SLEEP   = 1,
    WF_HB_WAIT_WAKEUP   = 2
} t_hibernateState;

// Pin Level
typedef enum
{
    WF_LOW    = 0,
    WF_HIGH   = 1
} t_pinLevel;

// Connection states
typedef enum
{
    WF_CSTATE_NOT_CONNECTED                = 1,
    WF_CSTATE_CONNECTION_IN_PROGRESS       = 2,
    WF_CSTATE_CONNECTED_INFRASTRUCTURE     = 3,
    WF_CSTATE_CONNECTED_ADHOC              = 4,
    WF_CSTATE_RECONNECTION_IN_PROGRESS     = 5,
    WF_CSTATE_CONNECTION_PERMANENTLY_LOST  = 6
} t_connectionStates;


// eventInfo define for WF_ProcessEvent() when no additional info is supplied
#define WF_NO_ADDITIONAL_INFO                   ((uint16_t)0xffff)

#define ENABLE_WPS_PRINTS                       ((uint8_t)(1 << 0))
#define ENABLE_P2P_PRINTS                       ((uint8_t)(1 << 1))

typedef enum
{
    SOFTAP_EVENT_CONNECTED    = 0,
    SOFTAP_EVENT_DISCONNECTED = 1
} t_softAPEvent;

typedef enum
{
    SOFTAP_EVENT_LINK_LOST       = 0,
    SOFTAP_EVENT_RECEIVED_DEAUTH = 1
} t_softAPEventReason ;

typedef struct wfMacStatsStruct
{
    uint32_t MibWEPExcludeCtr;      // Number of frames received with the Protected Frame subfield of the Frame
                                    //  Control field set to zero and the value of dot11ExcludeUnencrypted causes
                                    //  that frame to be discarded.
    uint32_t MibTxBytesCtr;         // Total number of Tx bytes that have been transmitted
    uint32_t MibTxMulticastCtr;     // Number of frames successfully transmitted that had the multicast bit set
                                    //  in the destination MAC address
    uint32_t MibTxFailedCtr;        // Number of Tx frames that failed due to the number of transmits exceeding the retry count
    uint32_t MibTxRtryCtr;          // Number of times a transmitted frame needed to be retried
    uint32_t MibTxMultRtryCtr;      // Number of times a frame was successfully transmitted after more than one retransmission.
    uint32_t MibTxSuccessCtr;       // Number of Tx frames successfully transmitted.
    uint32_t MibRxDupCtr;           // Number of frames received where the Sequence Control field indicates a duplicate.
    uint32_t MibRxCtsSuccCtr;       // Number of CTS frames received in response to an RTS frame.
    uint32_t MibRxCtsFailCtr;       // Number of times an RTS frame was not received in response to a CTS frame.
    uint32_t MibRxAckFailCtr;       // Number of times an Ack was not received in response to a Tx frame.
    uint32_t MibRxBytesCtr;         // Total number of Rx bytes received.
    uint32_t MibRxFragCtr;          // Number of successful received frames (management or data)
    uint32_t MibRxMultCtr;          // Number of frames received with the multicast bit set in the destination MAC address.
    uint32_t MibRxFCSErrCtr;        // Number of frames received with an invalid Frame Checksum (FCS).
    uint32_t MibRxWEPUndecryptCtr;  // Number of frames received where the Protected Frame subfield of the Frame Control Field is set to
                                    //  one and the WEPOn value for the key mapped to the transmitter?s MAC address indicates the frame
                                    //  should not have been encrypted.
    uint32_t MibRxFragAgedCtr;      // Number of times that fragments ?aged out?, or were not received in the allowable time.
    uint32_t MibRxMICFailureCtr;    // Number of MIC failures that have occurred.
} t_wfMacStats;



// used in WF_GetDeviceInfo
typedef struct wfdeviceInfoStruct
{
    uint8_t  deviceType;    // MRF24W_DEVICE_TYPE
    uint8_t  romVersion;    // const version number
    uint8_t  patchVersion;  // patch version number
} t_wfDeviceInfo;

    
// used in WF_ConnectContextGet
typedef struct wfConnectContextStruct
{
    uint8_t  channel;    // channel number of current connection
    uint8_t  bssid[6];   // bssid of connected AP
} t_wfConnectContext;

// See WF_ScanContextSet()
typedef struct scanContext
{
    uint8_t  scanType;          // see t_scanType
    uint8_t  scanCount;
    uint16_t minChannelTime;    // ms
    uint16_t maxChannelTime;    // ms
    uint16_t probeDelay;        // us
} t_scanContext;

// See WF_PsPollEnable()
typedef struct psPollContext
{
    uint16_t listenInterval;    // Number of 100ms intervals between instances when
                                //  the MRF24W wakes up to receive buffered messages
                                //  from the network (1 = 100ms, 2 = 200ms, etc.)
    uint16_t dtimInterval;      // Number of DTIM intervals between instances when
                                //  the MRF24W wakes up to receive buffered messages
                                //  from the network.
    bool     useDtim;           // true if dtimInterval is being used, else false
} t_psPollContext;

// See SetAdhocContext()
typedef struct adHocNetworkContext
{
    uint8_t  mode;          // see t_adhoc mode
    bool     hiddenSsid;    // True if SSID should be hidden, else False (normally False)
    uint16_t beaconPeriod;  // beacon period, in ms
} t_adHocNetworkContext;

// See WF_SecurityWepSet()
typedef struct wepContext
{
    uint8_t wepSecurityType;                // WF_SECURITY_WEP_40 or WF_SECURITY_WEP_104
    uint8_t wepKey[WF_MAX_WEP_KEY_LENGTH];  // array containing WEP binary security key (4 5-byte keys for WEP-40, 4 13-byte keys for WEP-104)
    uint8_t wepKeyLength;                   // number of bytes pointed to by p_wepKey
    uint8_t wepKeyType;                     // WF_SECURITY_WEP_OPENKEY (default) or WF_SECURITY_WEP_SHAREDKEY
} t_wepContext;

// used in t_wpaContext and t_wpsContext
typedef struct
{
    uint8_t key[WF_MAX_WPA_PASS_PHRASE_LENGTH];  // binary key or passphrase
    uint8_t keyLength;                           // number of bytes in binary key (always 32) or passphrase
} t_wpaKeyInfo;

// See WF_SetSecurityWpa()
typedef struct wpaContext
{
    uint8_t wpaSecurityType; // WF_SECURITY_WPA_WITH_KEY, WF_SECURITY_WPA_WITH_PASS_PHRASE,
                             //  WF_SECURITY_WPA2_WITH_KEY, WF_SECURITY_WPA2_WITH_PASS_PHRASE
                             //  WF_SECURITY_WPA_AUTO_WITH_KEY, WF_SECURITY_WPA_AUTO_WITH_PASS_PHRASE
    t_wpaKeyInfo keyInfo;
} t_wpaContext;

// See WF_SetSecurityWps()
typedef struct wpsContext
{
    uint8_t wpsSecurityType;                    // WF_SECURITY_WPS_PUSH_BUTTON or WF_SECURITY_WPS_PIN
    uint8_t wpsPin[WF_WPS_PIN_LENGTH];          // if using WF_SECURITY_WPS_PIN then pointer to 8-digit pin
    uint8_t wpsPinLength;                       // should always be 8
    #if defined(WF_USE_HOST_WPA_KEY_CALCULATION)
        bool    getPassPhrase;                  // True if ASCII passphrase should be sent back to host
                                                //  so host can (more quickly) calculate binary key.  False
                                                //  if the MRF24WG should calculate the binary key

        t_wpaKeyInfo *p_keyInfo;                // pointer to where the Universal driver will
                                                // store passphrase info (must be global memory)
    #endif // WF_USE_HOST_WPA_KEY_CALCULATION 
} t_wpsContext;

// structure/union can be used in functions WF_SecurityWepSet, WF_SecurityWpaSet,
// and WF_SecurityWpsSet
typedef union
{
    t_wepContext wepContext;
    t_wpaContext wpaContext;
    t_wpsContext wpsContext;
} t_securityContext;

// Scan Result structure
typedef struct
{
    uint8_t      bssid[WF_BSSID_LENGTH]; // Network BSSID value
    uint8_t      ssid[WF_MAX_SSID_LENGTH]; // Network SSID value

    /*
      Access point configuration
      <table>
        Bit 7       Bit 6       Bit 5       Bit 4       Bit 3       Bit 2       Bit 1       Bit 0
        -----       -----       -----       -----       -----       -----       -----       -----
        WPA2        WPA         Preamble    Privacy     Reserved    Reserved    Reserved    IE
      </table>
      
      <table>
      IE        1 if AP broadcasting one or more Information Elements, else 0
      Privacy   0 : AP is open (no security)
                 1: AP using security,  if neither WPA and WPA2 set then security is WEP.
      Preamble  0: AP transmitting with short preamble
                 1: AP transmitting with long preamble
      WPA       Only valid if Privacy is 1.
                 0: AP does not support WPA
                 1: AP supports WPA
      WPA2      Only valid if Privacy is 1.
                 0: AP does not support WPA2
                 1: AP supports WPA2
      </table>
      */
    uint8_t      apConfig;
    uint8_t      reserved;
    uint16_t     beaconPeriod; // Network beacon interval          
    uint16_t     atimWindow;   // Only valid if bssType = WF_NETWORK_TYPE_INFRASTRUCTURE

    /*
      List of Network basic rates.  Each rate has the following format:
      
          Bit 7
      * 0: rate is not part of the basic rates set
      * 1: rate is part of the basic rates set

          Bits 6:0
      Multiple of 500kbps giving the supported rate.  For example, a value of 2 
      (2 * 500kbps) indicates that 1mbps is a supported rate.  A value of 4 in 
      this field indicates a 2mbps rate (4 * 500kbps).
      */
    uint8_t      basicRateSet[WF_MAX_NUM_RATES]; 
    uint8_t      rssi;          // Signal strength of received frame beacon or probe response
    uint8_t      numRates;      // Number of valid rates in basicRates
    uint8_t      DtimPeriod;    // Part of TIM element
    uint8_t      bssType;       // WF_NETWORK_TYPE_INFRASTRUCTURE or WF_NETWORK_TYPE_ADHOC
    uint8_t      channel;       // Channel number
    uint8_t      ssidLen;       // Number of valid characters in ssid

} t_wfScanResult;

typedef struct WFHibernate
{
    UINT8 state;
    UINT8 wakeup_notice;
} t_wfHibernate;

typedef struct wfMultiCastConfigStruct
{
    uint8_t filterId;
    uint8_t action;
    uint8_t macBytes[6];
    uint8_t macBitMask;

} t_wfMultiCastConfig;

typedef struct wfWpsCredStruct
{
        uint8_t  ssid[32];
        uint8_t  netKey[64];
        uint16_t authType;
        uint16_t encType;
        uint8_t  netIdx;
        uint8_t  ssidLen;
        uint8_t  keyIdx;
        uint8_t  keyLen;
        uint8_t  bssid[6];
} t_wfWpsCred;

typedef struct mgmtIndicateSoftAPEvent
{
    uint8_t event;
    uint8_t reason;
    uint8_t address[6];
} tMgmtIndicateSoftAPEvent;


/*
*********************************************************************************************************
*                                           FUNCTION PROTOTYPES                               
*********************************************************************************************************
*/

//-------------------------
// Initialization Functions
//-------------------------
bool WF_Init(const void* pNetIf);
bool WF_DeInit( void* pNetIf);

//----------------------------
// Network Selection Functions
//----------------------------
void WF_NetworkTypeSet(uint8_t networkType);
void WF_NetworkTypeGet(uint8_t *p_networkType);
void WF_SsidSet(uint8_t *p_ssid,  uint8_t ssidLength);
void WF_SsidGet(uint8_t *p_ssid, uint8_t *p_ssidLength);
void WF_AdhocContextSet(t_adHocNetworkContext *p_context);
void WF_BssidSet(uint8_t *p_bssid);
void WF_BssidGet(uint8_t *p_bssid);

//---------------------
// Connection Functions
//---------------------
void WF_ChannelListSet(uint8_t *p_channelList, uint8_t numChannels);
void WF_ChannelListGet(uint8_t *p_channelList, uint8_t *p_numChannels);
void WF_ReconnectModeSet(uint8_t retryCount, uint8_t deauthAction, uint8_t beaconTimeout, uint8_t beaconTimeoutAction);
void WF_ReconnectModeGet(uint8_t *p_retryCount, uint8_t *p_deauthAction, uint8_t *p_beaconTimeout, uint8_t *p_beaconTimeoutAction);
void Wifi_Connect(void);
void WF_Connect(void);    // low=-
uint16_t WF_Disconnect(void);
void WF_ConnectionStateGet(uint8_t *p_state);
void WF_ConnectContextGet(t_wfConnectContext *p_ctx);
void WF_SetLinkDownThreshold(uint8_t threshold);
void WF_SetTxMode(uint8_t mode);
void WF_GetTxMode(uint8_t *mode);
void WF_RssiSet(uint8_t rssi);
void WF_RssiGet(uint8_t *p_rssi);

//-------------------
// Security Functions
//-------------------
void WF_SecurityOpenSet(void);
void WF_SecurityWepSet(t_wepContext* p_context);
void WF_SecurityWpaSet(t_wpaContext* p_context);
void WF_SecurityWpsSet(t_wpsContext *p_context);

void WF_SecurityGet(uint8_t *p_securityType,
                      uint8_t *p_securityKey,
                      uint8_t *p_securityKeyLength);
void WF_SetWPATimeout(uint32_t timeout);
void WF_GetWPATimeout(uint32_t *timeout);
void WF_WepKeyTypeGet(uint8_t *p_wepKeyType);
void WF_WPSCredentialsGet(t_wfWpsCred *p_cred);
tMgmtIndicateSoftAPEvent * WF_GetSoftApEventInfo(void);
void WF_YieldPassphrase2Host(void); // used in WPS
void WF_SetPSK(UINT8 *psk);


//------------------
// Version functions
//------------------
void WF_GetDeviceInfo(t_wfDeviceInfo *p_deviceInfo);

//----------------------
// MAC Address Functions
//----------------------
void WF_MacAddressSet(uint8_t *p_mac);
void WF_MacAddressGet(uint8_t *p_mac);

//---------------------------
// Tx Power Control Functions
//---------------------------
void WF_TxPowerSetMax(int8_t maxTxPower);
void WF_TxPowerGetMax(int8_t *p_maxTxPower);
void WF_TxPowerGetFactoryMax(int8_t *p_factoryMaxTxPower);

//---------------------
// Power-save Functions
//---------------------
void WF_PsPollEnable(t_psPollContext *p_context);
void WF_PsPollDisable(void);
void WF_GetPowerSaveState(uint8_t *p_powerSaveState);
void WF_HibernateEnable(void);

//------------------------
// RTS Threshold Functions
//------------------------
void WF_RtsThresholdSet(uint16_t rtsThreshold);
void WF_RtsThresholdGet(uint16_t *p_rtsThreshold);

//--------------------------
// Regional Domain Functions
//--------------------------
void WF_RegionalDomainGet(uint8_t *p_regionalDomain);  

//--------------------
// Multicast Functions
//--------------------
void WF_MulticastSetConfig(t_wfMultiCastConfig *p_config);
void WF_EnableSWMultiCastFilter(void);
void WF_SetMultiCastFilter(uint8_t multicastFilterId, uint8_t multicastAddress[6]);
void WF_GetMultiCastFilter(uint8_t multicastFilterId, uint8_t multicastAddress[6]);

/* MAC Stats */
void WF_GetMacStats(t_wfMacStats *p_macStats);

//---------------
// Scan Functions
//---------------
void WF_ScanContextSet(t_scanContext *p_context);
void WF_ScanContextGet(t_scanContext *p_context);
uint16_t WF_Scan(bool scanAll);
void WF_ScanGetResult(uint8_t listIndex,  t_wfScanResult *p_scanResult);
void WF_StartScanWhenBoot(TCPIP_MAC_HANDLE hMac);

//-------------------------
// Gratuitous Arp Functions
//-------------------------
void WF_StartGratuitousArp(uint8_t periodInSeconds);
void WF_StopGratuitousArp(void);

//------------------------------
// External Interrupt Functions
//------------------------------
void WF_EintInit(void);
void WF_EintEnable(void);
void WF_EintDisable(void);
bool WF_EintIsDisabled(void);
bool WF_EintIsPending(void);

//---------------
// SPI Functions
//---------------
bool WF_SpiInit(void);
void WF_SpiEnableChipSelect(void);
void WF_SpiDisableChipSelect(void);
void WF_SpiTxRx(uint8_t   *p_txBuf, uint16_t  txLen, uint8_t   *p_rxBuf, uint16_t  rxLen);

//--------------------------------------
// Enable debug print message in module FW
//--------------------------------------
void WF_EnableDebugPrint(uint8_t option);


//--------------------------
// Event Handling Functions
//--------------------------
void MRF24W_SetUserEvents(uint8_t event, uint16_t eventInfo, bool isMgmt);
uint16_t MRF24W_GetTrafficEvents(uint16_t* pEventInfo);
uint16_t MRF24W_GetMgmtEvents(uint16_t* pEventInfo);


//--------------------------
// Configure Data Functions
//--------------------------
bool WF_ConfigDataLoad(void);
bool WF_ConfigDataSave(void);
bool WF_ConfigDataErase(void);
void WF_ConfigDataPrint(void);


#endif /* __WF_API_H_ */



