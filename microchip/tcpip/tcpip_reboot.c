/*******************************************************************************
  Reboot Module

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Remotely resets the PIC
    -Reference: Internet Bootloader documentation
*******************************************************************************/

/*******************************************************************************
FileName:   Reboot.c
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

#define __REBOOT_C

#include "tcpip_private.h"

#if defined(TCPIP_STACK_USE_REBOOT_SERVER)

#define TCPIP_THIS_MODULE_ID    TCPIP_MODULE_REBOOT_SERVER

extern NODE_INFO remoteNode; 


static UDP_SOCKET*	rebootSockets = 0;
static int          rebootInitCount = 0;
static const void*  rebootMemH = 0;        // memory handle

/*****************************************************************************
  Function:
    bool TCPIP_RebootInit(const TCPIP_STACK_MODULE_CTRL* const stackCtrl, const REBOOT_MODULE_CONFIG* pRebootConfig);

  Summary:
	Resets the reboot server module for the specified interface.

  Description:
	Resets the reboot server module for the specified interface.

  Precondition:
	None

  Parameters:
	stackCtrl - pointer to stack structure specifying the interface to initialize

  Returns:
	None

  Remarks:
	This function should be called internally just once per interface 
    by the stack manager.
***************************************************************************/
bool TCPIP_RebootInit(const TCPIP_STACK_MODULE_CTRL* const stackCtrl, const void* pRebootConfig)
{
    int ix;
    
    if(stackCtrl->stackAction == TCPIP_STACK_ACTION_IF_UP)
    {   // interface restart
        return true;
    }

    // stack init
    
    if(rebootInitCount == 0)
    {   // first time we're run
        // store the memory allocation handle
        rebootMemH = stackCtrl->memH;
        
        rebootSockets = (UDP_SOCKET*)TCPIP_HEAP_Malloc(rebootMemH,  stackCtrl->nIfs * sizeof(UDP_SOCKET));
        if(rebootSockets == 0)
        {   // failed
            return false;
        }

        // initialize per interface
        for(ix = 0; ix < stackCtrl->nIfs; ix++)
        {
            rebootSockets[ix] = INVALID_UDP_SOCKET;
        }

    }
            
    // Reset state machine and flags to default values
    rebootSockets[stackCtrl->netIx] = INVALID_UDP_SOCKET;
    
    rebootInitCount++;
    return true;
}

/*****************************************************************************
  Function:
    bool TCPIP_RebootDeInit(const TCPIP_STACK_MODULE_CTRL* const stackCtrl);

  Summary:
	Turns off the reboot server module for the specified interface.

  Description:
	Closes the reboot server.

  Precondition:
	None

  Parameters:
	stackData - pointer to stack structure specifying the interface to deinitialize

  Returns:
	None

  Remarks:
	This function should be called internally just once per interface 
    by the stack manager.
***************************************************************************/
void TCPIP_RebootDeInit(const TCPIP_STACK_MODULE_CTRL* const stackCtrl)
{

    // if(stackCtrl->stackAction == TCPIP_STACK_ACTION_DEINIT) // stack shut down
    // if(stackCtrl->stackAction == TCPIP_STACK_ACTION_IF_DOWN) // interface down

    // one way or another this interface is going down
	if(rebootSockets[stackCtrl->netIx] != INVALID_UDP_SOCKET)
    {
        UDPClose(rebootSockets[stackCtrl->netIx]);
        rebootSockets[stackCtrl->netIx] = INVALID_UDP_SOCKET;
    }

    if(stackCtrl->stackAction == TCPIP_STACK_ACTION_DEINIT)
    {   // whole stack is going down
        if(rebootInitCount > 0)
        {   // we're up and running
            if(--rebootInitCount == 0)
            {   // all closed
                // release resources
                TCPIP_HEAP_Free(rebootMemH, rebootSockets);
                rebootSockets = 0;
                rebootMemH = 0;
            }
        }
    }

}


/*********************************************************************
 * Function:        bool TCPIP_RebootTask(TCPIP_NET_IF* pNetIf)
 *
 * PreCondition:    Stack is initialized()
 *
 * Input:           pNetIf   - interface 
 *
 * Output:          true if processed, false otherwise
 *
 * Side Effects:    None
 *
 * Overview:        Checks for incomming traffic on port 69.  
 *					Resets the PIC if a 'R' is received.
 *
 * Note:            This module is primarily for use with the 
 *					Ethernet bootloader.  By resetting, the Ethernet 
 *					bootloader can take control for a second and let
 *					a firmware upgrade take place.
 ********************************************************************/
bool TCPIP_RebootTask(TCPIP_NET_IF* pNetIf)
{
	struct
	{
		uint8_t vMACAddress[6];
		uint32_t dwIPAddress;
		uint16_t wChecksum;
	} BootloaderAddress;
    
    int                 netIx;
    UDP_SOCKET          rSkt;
    

    netIx = _TCPIPStackNetIx(pNetIf);

	
	if((rSkt = rebootSockets[netIx]) == INVALID_UDP_SOCKET)
    {
        rSkt = rebootSockets[netIx] = UDPOpenServer(IP_ADDRESS_TYPE_IPV4, REBOOT_PORT, 0);

        if(rSkt == INVALID_UDP_SOCKET)
        {
            return false;
        }
        UDPSocketSetNet(rSkt, pNetIf);
    }

	// Do nothing if no data is waiting
	if(!UDPIsGetReady(rSkt))
		return false;
    
	#if defined(REBOOT_SAME_SUBNET_ONLY)
    {
        // Respond only to name requests sent to us from nodes on the same subnet
     	if((remoteNode.IPAddr.Val & pNetIf->netMask.Val) != (pNetIf->netIPAddr.Val & pNetIf->netMask.Val))
        {
            UDPDiscard(rSkt);
            return false;
        }
    }
	#endif

	// Get our MAC address, IP address, and compute a checksum of them 
	memcpy((void*)&BootloaderAddress.vMACAddress[0], (void*)&pNetIf->netMACAddr.v[0], sizeof(pNetIf->netMACAddr));
	BootloaderAddress.dwIPAddress = pNetIf->netIPAddr.Val;
	BootloaderAddress.wChecksum = TCPIP_Helper_CalcIPChecksum((uint8_t*)&BootloaderAddress, sizeof(BootloaderAddress) - sizeof(BootloaderAddress.wChecksum), 0);
	
	// To enter the bootloader, we reset the system
	SYS_OUT_MESSAGE("Bootloader Reset");

	SYS_Reboot();

    return true;    // keep compiler happy
}

#endif //#if defined(TCPIP_STACK_USE_REBOOT_SERVER)

