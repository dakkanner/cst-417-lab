/*******************************************************************************
  Multiple MAC Module implementation for Microchip Stack

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:   tcpip_mac_object.c
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


#include "tcpip_private.h"

#include "tcpip_mac_private.h"
#include "tcpip/tcpip_mac_object.h"
#include "wifi/mrf24w_mac.h"



/************************************
 *  the PIC32 MAC parameterized interface implementation
 *************************************/


TCPIP_MAC_RES MACClose(TCPIP_MAC_HANDLE hMac)
{
    return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACClose)(hMac);
}
    
int MACGetHeader(TCPIP_MAC_HANDLE hMac, MAC_ADDR *remote, uint16_t* type)
{
    return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACGetHeader)(hMac, remote, type);
}
    
void MACSetReadPtrInRx(TCPIP_MAC_HANDLE hMac, uint16_t offset)
{
    (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACSetReadPtrInRx)(hMac, offset);
}
    
TCPIP_MAC_PTR_TYPE MACSetWritePtr(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_PTR_TYPE address)
{
    return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACSetWritePtr)(hMac, address);
}
 
TCPIP_MAC_PTR_TYPE MACGetReadPtrInRx(TCPIP_MAC_HANDLE hMac)
{
    return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACGetReadPtrInRx)(hMac);
}
   
TCPIP_MAC_PTR_TYPE MACSetReadPtr(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_PTR_TYPE address)
{
    return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACSetReadPtr)(hMac, address);
}

TCPIP_MAC_PTR_TYPE MACSetBaseReadPtr(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_PTR_TYPE address)
{
    return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACSetBaseReadPtr)(hMac, address);
}
    
uint16_t MACGetArray(TCPIP_MAC_HANDLE hMac, uint8_t *val, uint16_t len)
{
    return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACGetArray)(hMac, val, len);
}
    
void MACDiscardRx(TCPIP_MAC_HANDLE hMac)
{
    return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACDiscardRx)(hMac);
}
    
void MACPutHeader(TCPIP_MAC_HANDLE hMac, MAC_ADDR *remote, uint16_t type, uint16_t dataLen)
{
    (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACPutHeader)(hMac, remote, type, dataLen);
}
    
bool MACIsTxReady(TCPIP_MAC_HANDLE hMac)
{
    return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACIsTxReady)(hMac);
}
    
void MACPutArray(TCPIP_MAC_HANDLE hMac, uint8_t *val, uint16_t len)
{
    (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACPutArray)(hMac, val, len);
}
    
void MACFlush(TCPIP_MAC_HANDLE hMac)
{
    (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACFlush)(hMac);
}
    
bool MACCheckLink(TCPIP_MAC_HANDLE hMac)
{
    return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACCheckLink)(hMac);
}
    
TCPIP_MAC_PTR_TYPE MACGetTxBaseAddr(TCPIP_MAC_HANDLE hMac)
{
    return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACGetTxBaseAddr)(hMac);
}
    
uint16_t MACCalcRxChecksum(TCPIP_MAC_HANDLE hMac, uint16_t offset, uint16_t len)
{
    return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACCalcRxChecksum)(hMac, offset, len);
}
    
uint16_t MACCalcIPBufferChecksum(TCPIP_MAC_HANDLE hMac, uint16_t len)
{
    return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACCalcIPBufferChecksum)(hMac, len);
}
    
void MACSetRXHashTableEntry(TCPIP_MAC_HANDLE hMac, MAC_ADDR DestMACAddr)
{
    (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACSetRXHashTableEntry)(hMac, DestMACAddr);
}
    

bool MACPowerMode(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_POWER_MODE pwrMode)
{
    return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACPowerMode)(hMac, pwrMode);
}
    
#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
TCPIP_MAC_EVENT_RESULT MACEventSetNotifyEvents(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents)
{
        return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACEventSetNotifyEvents)(hMac, tcpEvGroup, tcpipEvents);
}

TCPIP_MAC_EVENT_RESULT MACEventClearNotifyEvents(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents)
{
        return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACEventClearNotifyEvents)(hMac, tcpEvGroup, tcpipEvents);
}

TCPIP_MAC_EVENT_RESULT MACEventAck(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents)
{
        return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACEventAck)(hMac, tcpEvGroup, tcpipEvents);
}

TCPIP_EVENT MACEventGetPending(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup)
{
        return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACEventGetPending)(hMac, tcpEvGroup);
}

TCPIP_MAC_EVENT_RESULT MACEventSetNotifyHandler(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, pMacEventF eventHandler, void* hParam)
{
        return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACEventSetNotifyHandler)(hMac, tcpEvGroup, eventHandler, hParam);
}


#endif  // (TCPIP_STACK_USE_EVENT_NOTIFICATION)




