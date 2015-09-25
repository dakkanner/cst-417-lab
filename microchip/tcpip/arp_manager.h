/*******************************************************************************
  ARP module manager header

  Summary:
    Stack internal definitions for ARP module
    
  Description:
    This file contains the stack internal API for the ARP module
*******************************************************************************/

/*******************************************************************************
FileName:   arp_manager.h
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

#ifndef _ARP_MANAGER_H_
#define _ARP_MANAGER_H_ 


// stack private API functions

bool            ARPInitialize(const TCPIP_STACK_MODULE_CTRL* const stackData
                             ,const ARP_MODULE_CONFIG* arpData);

void            ARPDeinitialize(const TCPIP_STACK_MODULE_CTRL* const stackData);
    
bool            ARPTaskPending(void);

void            ARPTask(void);

ARP_RESULT      ARPProcess(TCPIP_NET_IF* pIf);



// TCPIP_STACK_USE_ZEROCONF_LINK_LOCAL API specific Definitions
struct arp_app_callbacks {
    bool used;
    void (*ARPPkt_notify)( TCPIP_NET_IF*  pNetIf
                           ,uint32_t SenderIPAddr
                           ,uint32_t TargetIPAddr
                           ,MAC_ADDR* SenderMACAddr
                           ,MAC_ADDR* TargetMACAddr
                           ,uint8_t op_req);
};
int8_t ARPRegisterCallbacks(struct arp_app_callbacks *app);
bool ARPDeRegisterCallbacks(int8_t id);



#endif  // _ARP_MANAGER_H_ 

