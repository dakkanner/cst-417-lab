
#include <string.h>

#include "tcpip_private.h"

//#include "arp_private.h"
//#include "hash_fnv.h"
#include "hardware_config.h"
#include "sys_fs_config.h"



#include "system/drivers/drv_media.h"
#include "system/drivers/xeeprom.h"
#include "wf_configData.h"
// handle to the opened media


t_wfConfigData *p_wifi_ConfigData;


/****************************************************************************
  Section:
    Stack-Level Functions
  ***************************************************************************/
#if defined(MEDIA_STORAGE_EEPROM)
/*  ********************************************* 
#define WIFI_STORAGET_PARTITION_SIZE    (140)
This setting "WIFI_STORAGET_PARTITION_SIZE" is the memory size for WiFi setting. 
***************************************************/
void WiFiMemory_write(uint8_t *buf, uint32_t size)
{
    XEEBeginWrite(0);
    XEEWriteArray(buf, size);
}
void WiFiMemory_read(uint8_t *buf, uint32_t size)
{
    XEEReadArray(0,buf,size);
}
#else
void WiFiMemory_write(uint8_t *buf, uint32_t size){}
void WiFiMemory_read(uint8_t *buf, uint32_t size){}
#endif

t_wfConfigData wifi_ConfigData;
t_wfConfigData *p_wifi_ConfigData = &wifi_ConfigData;

void WF_ConfigDataPrint(void)
{
    char buf_t[50]; int i;

    sprintf(buf_t, "Size of Configdata:  %lu\r\n", sizeof(wifi_ConfigData));     SYS_CONSOLE_MESSAGE(buf_t);
    sprintf(buf_t, "networkType:         %d\r\n",  wifi_ConfigData.networkType);     SYS_CONSOLE_MESSAGE(buf_t);
    sprintf(buf_t, "netSSID:             %s\r\n",  wifi_ConfigData.netSSID);         SYS_CONSOLE_MESSAGE(buf_t);
    sprintf(buf_t, "SsidLength:          %d\r\n",  wifi_ConfigData.SsidLength);     SYS_CONSOLE_MESSAGE(buf_t);
    sprintf(buf_t, "SecurityMode:        %d\r\n",  wifi_ConfigData.SecurityMode);     SYS_CONSOLE_MESSAGE(buf_t);
    sprintf(buf_t, "SecurityKey:         %s\r\n",  wifi_ConfigData.SecurityKey);     SYS_CONSOLE_MESSAGE(buf_t);
    sprintf(buf_t, "SecurityKeyLength:   %d\r\n",  wifi_ConfigData.SecurityKeyLength);     SYS_CONSOLE_MESSAGE(buf_t);
    sprintf(buf_t, "DHCP mode:   %d\r\n",  wifi_ConfigData.Flags.bIsDHCPEnabled);     SYS_CONSOLE_MESSAGE(buf_t);
    sprintf(buf_t, "IP is:   %d.%d.%d.%d\r\n",  wifi_ConfigData.netIPAddr.v[0],
                                                wifi_ConfigData.netIPAddr.v[1],
                                                wifi_ConfigData.netIPAddr.v[2],
                                                wifi_ConfigData.netIPAddr.v[3]);
    SYS_CONSOLE_MESSAGE(buf_t);
    sprintf(buf_t, "netMask is:   %d.%d.%d.%d\r\n",  wifi_ConfigData.netMask.v[0],
                                                     wifi_ConfigData.netMask.v[1],
                                                     wifi_ConfigData.netMask.v[2],
                                                     wifi_ConfigData.netMask.v[3]);
    SYS_CONSOLE_MESSAGE(buf_t);
    sprintf(buf_t, "netGateway is:   %d.%d.%d.%d\r\n",  wifi_ConfigData.netGateway.v[0],
                                                wifi_ConfigData.netGateway.v[1],
                                                wifi_ConfigData.netGateway.v[2],
                                                wifi_ConfigData.netGateway.v[3]);
    SYS_CONSOLE_MESSAGE(buf_t);
    sprintf(buf_t, "PrimaryDNSServer is:   %d.%d.%d.%d\r\n",  wifi_ConfigData.PrimaryDNSServer.v[0],
                                                wifi_ConfigData.PrimaryDNSServer.v[1],
                                                wifi_ConfigData.PrimaryDNSServer.v[2],
                                                wifi_ConfigData.PrimaryDNSServer.v[3]);
    SYS_CONSOLE_MESSAGE(buf_t);
    SYS_CONSOLE_MESSAGE("NetBIOSName is: ");
    for(i=0;i<16;i++)
    {
        sprintf(buf_t, "%c",  wifi_ConfigData.NetBIOSName[i]);
        SYS_CONSOLE_MESSAGE(buf_t);
    }
    SYS_CONSOLE_MESSAGE("\r\n");

    SYS_CONSOLE_MESSAGE("IF->NetBIOSName is: ");
    TCPIP_NET_HANDLE netH;
    netH = TCPIP_STACK_NetHandle("MRF24W");
    TCPIP_NET_IF* pNetIf = (TCPIP_NET_IF*)netH;
    for(i=0;i<16;i++)
    {
        sprintf(buf_t, "%c",  pNetIf->NetBIOSName[i]);
        SYS_CONSOLE_MESSAGE(buf_t);
    }
    SYS_CONSOLE_MESSAGE("\r\n");
}


bool WF_ConfigDataErase(void)
{
    t_wfConfigData tmp_wifi_ConfigData;
    memset(&tmp_wifi_ConfigData,0x00,sizeof(t_wfConfigData));
    WiFiMemory_write((uint8_t*)&tmp_wifi_ConfigData,sizeof(t_wfConfigData));
    return true;
}




