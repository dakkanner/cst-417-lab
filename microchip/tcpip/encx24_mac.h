/*******************************************************************************
  ENC28J60 Driver Medium Access Control (MAC) 

  Summary:
    Layer Module  for Microchip TCP/IP Stack
  
  Description:
    PIC32 implementation for multiple MAC support
*******************************************************************************/

/*******************************************************************************
FileName:   enc28_mac.h
Copyright � 2012 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED �AS IS� WITHOUT WARRANTY OF ANY KIND,
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

#ifndef _ENCX24_MAC_H_ 
#define _ENCX24_MAC_H_ 

#include "tcpip/tcpip.h"
#include "tcpip/tcpip_mac.h"

// Compile only for ENC28J60 MAC interface
#if defined(TCPIP_IF_ENCX24J600)

#include "tcpip/tcpip_mac_object.h"

// ENCX24J600 MAC descriptor
//
typedef struct  
{
    const TCPIP_MAC_OBJECT* pObj;           // safe cast to TCPIP_MAC_DCPT   
    // specific ENCX24J600 MAC data 
    TCPIP_NET_IF*         pNetIf;         // interface we belong to    
    int					_linkPrev;		// last value of the link status                                 
    bool                isOpen;         // simple open status flag
                                        // just one hardware module supported for now
    // add other ENCX24J600 data here
}ENCX24_MAC_DCPT;


#endif

#endif

