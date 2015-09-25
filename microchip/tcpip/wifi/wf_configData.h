

#ifndef __WF_MEMORY_H
#define __WF_MEMORY_H

typedef struct __attribute__((__packed__))
{
    uint8_t networkType;
    uint8_t netSSID[32];       // Wireless SSID
    uint8_t SsidLength;        // number of bytes in SSID
    uint8_t SecurityMode;      // WF_SECURITY_OPEN or one of the other security modes
    uint8_t defaultWepKey;     // WEP index value
    uint8_t SecurityKey[64];   // WiFi Security key, or passphrase.
    uint8_t SecurityKeyLength; // number of bytes in security key (can be 0)
    uint8_t wepKeyType;
    uint8_t dataValid;
    uint8_t saveSecurityInfo;  // Save 32-byte PSK
    union
    {
        struct
        {
            uint16_t bInterfaceEnabled : 1; // 0 when TCPIP_MAC_POWER_DOWN/TCPIP_MAC_POWER_LOW
            volatile uint16_t  bNewTcpipEventAvlbl: 1;  // event available flag
            uint16_t                              : 1;  // unused
            uint16_t bIPv6InConfig                : 1;
            uint16_t bIPv6Enabled                 : 1;
            uint16_t bWFEasyConfig                : 1;  // NOTE : This was not present in 5.36
            uint16_t bIsDHCPEnabled               : 1;
            uint16_t bInConfigMode                : 1;
            uint16_t localAndType                 : 2;  // type of AND operation to be performed for local/nonlocal detection
            uint16_t localOrType                  : 2;  // type of OR operation to be performed for local/nonlocal detection
            uint16_t reserved                     : 4;  // not used
        };
        uint16_t v;
    }
    Flags;                        // Flags structure
    IPV4_ADDR   netIPAddr;        // IP address; currently only one IP add per interface
    IPV4_ADDR   netMask;          // Subnet mask
    IPV4_ADDR   netGateway;       // Default Gateway
    IPV4_ADDR   PrimaryDNSServer; // Primary DNS Server
     uint8_t    NetBIOSName[16];  // NetBIOS name
} t_wfConfigData;


extern t_wfConfigData *p_wifi_ConfigData;

#endif // __WF_MEMORY_H

