/*******************************************************************************
  UDP Performance Test example

  Summary:
    UDP example for Microchip TCP/IP Stack
    
  Description:
    -Sends out dummy packets from const memory
    -Reference: None.  This is for testing/example only.
*******************************************************************************/

/*******************************************************************************
FileName:   udp_performance_test.c
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

#include "tcpip/tcpip.h"


typedef enum
{
    UDP_PERF_START,
    UDP_PERF_WAIT_DHCP,
    UDP_PERF_OPEN,
    UDP_PERF_BCAST,
    UDP_PERF_IDLE

}UDP_PERF_STATE;

// interface to operate on
static TCPIP_NET_HANDLE pPerfIf = 0;

// UDP socket to use
static UDP_SOCKET	perfSkt;

// UDP Port to use for packet destination
static UDP_PORT     perfPort = 9;

// current state
static UDP_PERF_STATE   udpPerfState = UDP_PERF_START;

// counter of the actual packets sent
static uint32_t pktCounter = 1;

// counter of how many packets to send
static uint32_t pktLimit = 1024;

// if true, the socket will be closed and re-opened at each iteration
// if false, the same socket will be used for all packets to be sent
static bool udpPerfKillSkt = false;

// timeout measurement
static SYS_TICK udpPerfTick = 0;

// message to be sent with each packet
// extra 4 bytes will be used for sending the pktCounter
// You can increase/decrease the number of characters in the message
// by adjusting this message
const char* udpPerfMessage [] = 
{
    "The quick brown fox tried to jump over the yellow dog.  Unfortunately, the yellow dog stood up while the fox was in mid-jump.  As a result, the two collided.  Then, the dog, being the omnivore that it is, ate the quick brown fox.  This line is 256 bytes.\r\n"
        "The quick brown fox tried to jump over the yellow dog.  Unfortunately, the yellow dog stood up while the fox was in mid-jump.  As a result, the two collided.  Then, the dog, being the omnivore that it is, ate the quick brown fox.  This line is 256 bytes.\r\n"
        "The quick brown fox tried to jump over the yellow dog.  Unfortunately, the yellow dog stood up while the fox was in mid-jump.  As a result, the two collided.  Then, the dog, being the omnivore that it is, ate the quick brown fox.  This line is 256 bytes.\r\n"
        "The quick brown fox tried to jump over the yellow dog.  Unfortunately, the yellow dog stood up while the fox was in mid-jump.  As a result, the two collided.  Then, the dog, being the omnivore that it is, ate the quick brown fox.  This line is 252b. \r\n"
};




/*****************************************************************************
  Function:
	void UDPPerformanceTask(TCPIP_NET_HANDLE pNetIf)

  Summary:
	Tests the transmit performance of the UDP module.

  Description:
	This function tests the transmit performance of the UDP module.  At boot,
	this module will transmit 1024 large UDP broadcast packets of 1024 bytes
	each.  Using a packet sniffer, one can determine how long this process 
	takes and calculate the transmit rate of the stack.  This function tests 
	true UDP performance in that it will open a socket, transmit one packet, 
	and close the socket for each loop.  After this initial transmission, the
	module can be re-enabled by holding button 3.
	
	This function is particularly useful after development to determine the
	impact of your application code on the stack's performance.  A before and
	after comparison will indicate if your application is unacceptably
	blocking the processor or taking too long to execute.

  Precondition:
	UDP is initialized.

  Parameters:
	pNetIf - network to run on

  Returns:
	None
  ***************************************************************************/
void UDPPerformanceTask(TCPIP_NET_HANDLE pNetIf)
{
    IPV4_ADDR   bcastAddr;


    if(udpPerfState == UDP_PERF_IDLE)
    {
        return; // nothing to do
    }
    else if (pPerfIf == 0)
    {
        pPerfIf = pNetIf;   // remember the interface to work on
    }
    else if(pNetIf != pPerfIf)
    {
        return; // not our job
    }



    switch(udpPerfState)
    {
        case UDP_PERF_START:
            // Suppress transmissions if we don't have an Ethernet link so our counter starts correctly at 0x00000001
            if(!TCPIP_STACK_IsNetLinked(pPerfIf))
            {
                return;
            }
            udpPerfState++;
            break;

        case UDP_PERF_WAIT_DHCP:

#if defined(TCPIP_STACK_USE_DHCP_CLIENT)

            // Wait until DHCP module is finished
            if(!DHCPIsBound(pPerfIf))
            {
                udpPerfTick = SYS_TICK_Get();
                return;
            }

            // Wait an additional half second after DHCP is finished to let the announce module and any other stack state machines to reach normal operation
            if(SYS_TICK_Get() - udpPerfTick < SYS_TICK_TicksPerSecondGet()/2)
                return;
#endif

            udpPerfState++;
            break;

        case UDP_PERF_OPEN:

            // Set the socket's destination to be a broadcast over our IP 
            // subnet
            bcastAddr.Val = TCPIP_STACK_NetBcastAddress(pPerfIf);

            // Open a UDP socket for outbound transmission
            perfSkt = UDPOpenClient(IP_ADDRESS_TYPE_IPV4, perfPort, (IP_MULTI_ADDRESS*)&bcastAddr);

            // Abort operation if no UDP sockets are available
            // If this ever happens, incrementing UDP_MAX_SOCKETS in 
            // udp_config.h may help (at the expense of more global memory 
            // resources).
            if(perfSkt == INVALID_UDP_SOCKET)
                return;

            // make sure we're working on the selected interface
            UDPSocketSetNet(perfSkt, pPerfIf);
            udpPerfState++;
            break;

        case UDP_PERF_BCAST:

            // Make certain the socket can be written to
            if(!UDPIsTxPutReady(perfSkt, sizeof(udpPerfMessage) + sizeof(pktCounter)))
            {
                break;
            }

            // Put counter value into first 4 bytes of the packet
            UDPPutArray(perfSkt, (uint8_t*)&pktCounter, sizeof(pktCounter));

            UDPPutArray(perfSkt, (const uint8_t*)udpPerfMessage, sizeof(udpPerfMessage));

            // Send the packet
            UDPFlush(perfSkt);

            if(++pktCounter == pktLimit)
            {   // we're done
                UDPClose(perfSkt);
                udpPerfState = UDP_PERF_IDLE;
            }
            else if(udpPerfKillSkt)
            {
                UDPClose(perfSkt);
                udpPerfState = UDP_PERF_OPEN;
            }
            // else continue sending packets
                
            break;

        default:
            break;  // shouldn't happen
    }
    
}

