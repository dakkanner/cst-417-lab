/*******************************************************************************
  Zero Configuration (Zeroconf) IPV4 Link Local Addressing
  Module for Microchip TCP/IP Stack

  Summary:

  Description:
*******************************************************************************/

/*******************************************************************************
FileName:   zero_conf_link_local.c
Copyright © 2012 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED ?AS IS? WITHOUT WARRANTY OF ANY KIND,
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
#include "zero_conf_link_local_private.h"


#if !defined(TCPIP_STACK_USE_ZEROCONF_LINK_LOCAL)

bool ZCLLEnable(TCPIP_NET_HANDLE hNet)
{
    return false;
}

bool ZCLLDisable(TCPIP_NET_HANDLE hNet)
{
    return false;
}

bool ZCLLIsEnabled(TCPIP_NET_HANDLE hNet)
{
    return false;
}


#else   // defined(TCPIP_STACK_USE_ZEROCONF_LINK_LOCAL)

#ifndef TCPIP_STACK_USE_ZEROCONF_MDNS_SD
  #error "TCPIP_STACK_USE_ZEROCONF_LINK_LOCAL requires TCPIP_STACK_USE_ZEROCONF_MDNS_SD"
#endif

#define TCPIP_THIS_MODULE_ID    TCPIP_MODULE_ZCLL

/**************** Global Declarations ***************/
static ZCLL_NET_HANDLE* phZCLL = 0;

static int              zcllInitCount = 0;      // ZCLL module initialization count

/***************** Forward Declarations **************/

static void     _ZCLLCleanup(const TCPIP_STACK_MODULE_CTRL* const stackCtrl);

static void     _ZCLLDisable(TCPIP_NET_IF* pNetIf);

static bool     _ZCLLEnable(TCPIP_NET_IF* pNetIf);



static void ZeroconfARPPktNotify ( TCPIP_NET_IF*  pNetIf
                                 ,uint32_t SenderIPAddr
                                 ,uint32_t TargetIPAddr
                                 ,MAC_ADDR* SenderMACAddr
                                 ,MAC_ADDR* TargetMACAddr
                                 ,uint8_t op_req);

static ARP_PKT_TYPE FindARPPktType(uint32_t SrcIPAddr
                                   ,uint32_t DestIPAddr
                                   ,uint8_t op_req);

static struct arp_app_callbacks callbacks =
{
   .ARPPkt_notify  = ZeroconfARPPktNotify,
};



static SYS_TICK zcll_rand(void)
{
   return SYS_TICK_Get();
}

/***************************************************************
  Function:
   void ARPAction(*IPV4_ADDR SrcIPAddr,*IPV4_ADDR DestIPAddr ,uint8_t op_req,ARP_STATE ARPAction)

  Summary:
     a).ZCLL_ARP_PROBE:
     Sends out the ARP-probe packet.
     b).ZCLL_ARP_CLAIM:
     Sends out the ARP-Claim packet.
     c).ZCLL_ARP_DEFEND:
         Sends out the ARP-Defend packet, when a address-conflict is detected.

  Description:


    a).ZCLL_ARP_PROBE:
   This function is used to send out the ARP-Probe packet to check
    the uniquness of selected IP-address in private space(169.254.x.x)

    This function makes use of ARPSendPkt User-API exposed by ARP
    module.

    ARP-Probe Packet:
     ARP-Request
     sender IP address: 0.0.0.0
     sender HW address: Self MAC address
     target IP address: <probe addr> (Chosen IP-address 169.254.x.x)
     target HW address: FF:FF:FF:FF:FF:FF

    b).ZCLL_ARP_CLAIM:
    This function is used to send out the ARP-Claim packet to finalize
    the uniquness of selected IP-address in private space(169.254.x.x).
    This claim packet is final-step in decision making of selected IP-
    address.

    This function makes use of ARPSendPkt User-API exposed by ARP
    module.

    ARP-Probe Packet:
     ARP-Request
     sender IP address: <claim addr> (Chosen IP-address 169.254.x.x)
     sender HW address: Self MAC address
     target IP address: <claim addr> (Chosen IP-address 169.254.x.x)
     target HW address: FF:FF:FF:FF:FF:FF

    c).ZCLL_ARP_DEFEND:
       This function is used to send out the ARP-Defend packet to defend
    the selected IP-address. When a conflicting ARP-packet (Probe or
    Claim) is observed on local network ARP-defend packet will be sent
    out to announe its authority of owning IP-address.

    This function makes use of ARPSendPkt User-API exposed by ARP
    module.

    ARP-Probe Packet:
     ARP-Response
     sender IP address: <claim addr> (Chosen IP-address 169.254.x.x)
     sender HW address: Self MAC address
     target IP address: <claim addr> (Chosen IP-address 169.254.x.x)
     target HW address: FF:FF:FF:FF:FF:FF

  Parameters:
   None

  Returns:
     None
  ***************************************************************/
 void ARPAction(TCPIP_NET_HANDLE hNet
              ,IPV4_ADDR *SrcIPAddr
              ,IPV4_ADDR *DestIPAddr
              ,uint8_t op_req
              ,ZCLL_ARP_STATE arp_action)
{
    bool rc;

    rc=ARPProbe(hNet, DestIPAddr, SrcIPAddr, op_req);

    if(rc == false)
    {
        switch (arp_action)
        {
            case ZCLL_ARP_PROBE:
                INFO_ZCLL_PRINT("ZCLL_ARP_PROBE: Error in sending out ARP-Probe pkt \n");
                break;
            case ZCLL_ARP_CLAIM:
                INFO_ZCLL_PRINT("ZCLL_ARP_CLAIM: Error in sending out ARP-Claim pkt \n");
                break;
            case ZCLL_ARP_DEFEND:
                INFO_ZCLL_PRINT("ZCLL_ARP_DEFEND: Error in sending out ARP-Defend pkt \n");
                break;
        }
    }

}

/***************************************************************
  Function:
   ARP_PKT_TYPE FindARPPktType (uint32_t SrcIPAddr, uint32_t DestIPAddr,
                                 uint8_t op_req)

  Summary:
   Finds the Type of ARP-Packet based on the Source IP-address,
    Destination IP-address and operation-request.

  Description:
   This function is used to find out the ARP-packet type. When ARP
    module passes up a ARP-packet to Zeroconf Link-Local module, it
    parses contents and finds the packet-type (like ARP-Probe, ARP-
    Claim, ARP-Defend, or generic ARP-request/response)

  Parameters:
   SrcIPAddr    - Source IP-Address
    DestIPAddr   - Destination IP-Address
    op_req       - Operation-Request (ARP-Request/Response)

  Returns:
     ARP_PKT_TYPE - Type of ARP-Packet (Probe, Claim, Defend or
                    generic ARP-request/response)
  ***************************************************************/
ARP_PKT_TYPE FindARPPktType (uint32_t SrcIPAddr, uint32_t DestIPAddr,uint8_t op_req)
{
    if(op_req == ARP_OPERATION_REQ)
    {
        if(SrcIPAddr == 0x0)
            return ARP_PROBE_TYPE;
        else if (SrcIPAddr == DestIPAddr)
            return ARP_CLAIM_TYPE;
        else
            return ARP_REQUEST_TYPE;

    }

    else if(op_req == ARP_OPERATION_RESP)
    {
        if(SrcIPAddr == DestIPAddr)
            return ARP_DEFEND_TYPE;
        else
            return ARP_RESPONSE_TYPE;
    }

    else
        return UNKNOWN_TYPE;
}

/***************************************************************
  Function:
static void ZeroconfARPPktNotify ( TCPIP_NET_IF*  pNetIf
                                 ,uint32_t SenderIPAddr
                                 ,uint32_t TargetIPAddr
                                 ,MAC_ADDR* SenderMACAddr
                                 ,MAC_ADDR* TargetMACAddr
                                 ,uint8_t op_req)

  Summary:
   Callback registered with ARP-Module. This gets invoked from ARP-
    module and runs in the same context.

  Description:
   This function is registered as a callback with ARP-module to get
    notified about incoming Packet-events. Based on the type of packet
    received and Link-Local current state, appropriate action will be
    taken. To find the type of ARP-Packet this function makes use of
    FindARPPktType routine.

    Primary purpose of this function is to decipher the ARP-Packet rxed
    and check whether its leading to a conflict with the selected IP-
    address.

    Two types of conflicts are defined: Probe-Conflict and Late-Conflict
    If the current state of Link-Local is Probe/Claim and a conflict is
    detected its called "Probe-Conflict"
    If the current state of Link-Local is Defend-state and a conflict is
    detected its called "Late-Conflict"

  Parameters:
   SenderIPAddr    - Sender IP-Address
    TargetIPAddr    - Target IP-Address
    SenderMACAddr   - Sender MAC-Address
    TargetMACAddr   - Target MAC-Address
    op_req          - Operation-Request (ARP-Request/Response)

  Returns:
     None
  ***************************************************************/
static void ZeroconfARPPktNotify ( TCPIP_NET_IF*  pNetIf
                                 ,uint32_t SenderIPAddr
                                 ,uint32_t TargetIPAddr
                                 ,MAC_ADDR* SenderMACAddr
                                 ,MAC_ADDR* TargetMACAddr
                                 ,uint8_t op_req)
{
    ARP_PKT_TYPE pkt_type;
    ZCLL_NET_HANDLE *hZcll;

    int i;

    pkt_type = FindARPPktType (SenderIPAddr, TargetIPAddr, op_req);

    if(pkt_type == UNKNOWN_TYPE)
        return; // Can't hit this

    i = _TCPIPStackNetIx(pNetIf);
    hZcll =(phZCLL+i);

    switch (hZcll->zcll_state)
    {
        case SM_ADDR_PROBE:
        case SM_ADDR_CLAIM:
            {
                switch(pkt_type)
                {
                    case ARP_PROBE_TYPE:
                    case ARP_CLAIM_TYPE:
                    case ARP_DEFEND_TYPE:
                        if(hZcll->temp_IP_addr.Val == TargetIPAddr ) // Probe-Conflict
                        {
                            if(memcmp(SenderMACAddr, &pNetIf->netMACAddr.v, 6))
                            {
                                hZcll->zcll_flags.probe_conflict =1;
                            }
                        }
                        break;

                    case ARP_RESPONSE_TYPE:
                        /* Some-body has been using probed addr
                         * We need to choose different Address */
                        if(hZcll->temp_IP_addr.Val == SenderIPAddr)
                        {
                            INFO_ZCLL_PRINT("IP address conflict. Please wait...\r\n");
                            SYS_OUT_MESSAGE("IP conflict!    Please wait...");
                            hZcll->zcll_flags.probe_conflict =1;
                        }
                        break;
                    default:
                        break;
                }
            }
            break;

        case SM_ADDR_DEFEND:
            if(pNetIf->netIPAddr.Val == SenderIPAddr)
            {
                if(memcmp(SenderMACAddr, &pNetIf->netMACAddr, 6))
                {
                    INFO_ZCLL_PRINT("Zero Conf is defending the IP address\r\n");
                    hZcll->zcll_flags.late_conflict = 1;
                }
            }
            break;

        default:
            break; // Nothing to do in other states
    }

}

static void _ZCLLCleanup(const TCPIP_STACK_MODULE_CTRL* const stackCtrl)
{
    TCPIP_HEAP_Free(stackCtrl->memH, phZCLL);  // free the allocated memory
    phZCLL = (ZCLL_NET_HANDLE*)0;

}

static void _ZCLLDisable(TCPIP_NET_IF* pNetIf)
{
    ZCLL_NET_HANDLE *hZcll;

    hZcll = (phZCLL+_TCPIPStackNetIx(pNetIf));
    ARPDeRegisterCallbacks(hZcll->arpRegId);
    hZcll->zcll_state = SM_INIT;   //  stop the ZCLL state machine
    pNetIf->Flags.bIsZcllEnabled = false;
}

static bool _ZCLLEnable(TCPIP_NET_IF* pNetIf)
{
    ZCLL_NET_HANDLE* hZcll;

    hZcll = (phZCLL+_TCPIPStackNetIx(pNetIf));
    hZcll->arpRegId = ARPRegisterCallbacks(&callbacks);
    if(hZcll->arpRegId <0)
    {
        hZcll->zcll_state = SM_INIT;   //  should already be in idle state
        SYS_ERROR(SYS_ERROR_ERROR, "_ZCLLEnable: ARP registration Failed!!! \r\n");
        return false;
    }

    pNetIf->Flags.bIsZcllEnabled = true;
    hZcll->zcll_state = SM_ADDR_INIT;
    return true;
}

bool ZCLLEnable(TCPIP_NET_HANDLE hNet)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNet(hNet);

    if(pNetIf == 0)
    {
        return false;
    }

    if(pNetIf->Flags.bIsZcllEnabled != 0)
    {   // already started
        return true;
    }

    if(_TCPIPStackAddressServiceCanStart(pNetIf, TCPIP_STACK_ADDRESS_SERVICE_ZCLL))
    { 
        return _ZCLLEnable(pNetIf);
    }

    return false;
}

bool ZCLLDisable(TCPIP_NET_HANDLE hNet)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNet(hNet);
    if(pNetIf)
    {
        if(pNetIf->Flags.bIsZcllEnabled != 0)
        {
            _ZCLLDisable(pNetIf);
            _TCPIPStackAddressServiceEvent(pNetIf, TCPIP_STACK_ADDRESS_SERVICE_ZCLL, TCPIP_STACK_ADDRESS_SERVICE_EVENT_USER_STOP);
        }
        return true;
    }

    return false;
}

bool ZCLLIsEnabled(TCPIP_NET_HANDLE hNet)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNet(hNet);
    if(pNetIf)
    {
        return pNetIf->Flags.bIsZcllEnabled != 0;
    }

    return false;
}


/***************************************************************
  Function:
   void ZeroconfLLDeinitialize(void)

  Summary:
   Deinitialization routine for Zeroconf Link-Local state-machine.

  Description:
    This is deinitialization function for Zeroconf Link-Local and
    invoked from deinitialization portion of Main-function.

  Parameters:
   None

  Returns:
     None
  ***************************************************************/
void ZeroconfLLDeinitialize(const TCPIP_STACK_MODULE_CTRL* const stackCtrl)
{
    // if(stackCtrl->stackAction == TCPIP_STACK_ACTION_DEINIT) // stack shut down
    // if(stackCtrl->stackAction == TCPIP_STACK_ACTION_IF_DOWN) // interface down

    // one way or another this interface is going down
    _ZCLLDisable(stackCtrl->pNetIf);

    if(stackCtrl->stackAction == TCPIP_STACK_ACTION_DEINIT)
    {   // whole stack is going down
        if(zcllInitCount > 0)
        {   // we're up and running
            if(--zcllInitCount == 0)
            {   // all closed
                // release resources
                _ZCLLCleanup(stackCtrl);
            }
        }
    }
}


/***************************************************************
  Function:
   void ZeroconfLLInitialize(void)

  Summary:
   Initialization routine for Zeroconf Link-Local state-machine.

  Description:
    This is initialization function for Zeroconf Link-Local and
    invoked from initialization portion of Main-function.

    This function registers with ARP-module to get notifications
    about the incoming packets. Checks whether the WiFi MAC is
    connected to an Access-Point or not.

  Parameters:
   None

  Returns:
     None
  ***************************************************************/
bool ZeroconfLLInitialize(const TCPIP_STACK_MODULE_CTRL* const stackCtrl, const ZCLL_MODULE_CONFIG* zeroData)
{
    if(stackCtrl->stackAction != TCPIP_STACK_ACTION_IF_UP)
    {   // stack init/restart
        if(zcllInitCount == 0)
        {   // 1st time we run

            phZCLL = (ZCLL_NET_HANDLE*)TCPIP_HEAP_Calloc(stackCtrl->memH, stackCtrl->nIfs, sizeof(ZCLL_NET_HANDLE));
            if(phZCLL == (ZCLL_NET_HANDLE*)0)
            {
                SYS_ERROR(SYS_ERROR_ERROR, "ZeroconfLLInitialize: Failed to allocate memory\r\n");
                return false;
            }
        }
        zcllInitCount++;
    }

    if(stackCtrl->pNetIf->Flags.bIsZcllEnabled != 0)
    {
        _ZCLLEnable(stackCtrl->pNetIf);
    }

    return true;
}

/***************************************************************
  Function:
   bool ZeroconfLLProcess(TCPIP_NET_IF* pNetIf)

  Summary:
   Initialization routine for Zeroconf Link-Local state-machine.

  Description:
    This is Polled from Main-Application & Designed to support
    co-operative multi-tasking. This needs to return to Main,
    if we have to wait for Longer durations.

    This is the main function for Zeroconf's Link-Local and takes
    the actions accoding to current-state and event-notifications
    from ARP-Module.

  Parameters:
   TCPIP_NET_IF* pNetIf

  Returns:
     true if processed, false otherwise
  ***************************************************************/
bool ZeroconfLLProcess(TCPIP_NET_IF* pNetIf)
{
    ZCLL_NET_HANDLE *hZcll;
    char zeroconf_dbg_msg[256];

    hZcll = (phZCLL+_TCPIPStackNetIx(pNetIf));


    if(hZcll->zcll_state == SM_INIT)
    {   // nothing to do in this state
        return false;
    }

    if(!_TCPIPStackIsNetLinked(pNetIf))
    {   // lost connection; re-start
        hZcll->zcll_state = SM_ADDR_INIT;
        _TCPIPStackAddressServiceEvent(pNetIf, TCPIP_STACK_ADDRESS_SERVICE_ZCLL, TCPIP_STACK_ADDRESS_SERVICE_EVENT_CONN_LOST);
    }

    switch(hZcll->zcll_state)
    {
        case SM_ADDR_INIT:
            /* Not yet seeded in init routine */
            /* setup random number generator
             * we key this off the MAC-48 HW identifier
             * the first 3 octets are the manufacturer
             * the next 3 the serial number
             * we'll use the last four for the largest variety
             */

            hZcll->conflict_count = 0;
            pNetIf->netIPAddr.Val = 0x0;

            hZcll->probe_count = 0;

            hZcll->zcll_state = SM_ADDR_PROBE;
            INFO_ZCLL_PRINT("ADDR_INIT --> ADDR_PROBE \r\n");

            // No break. Fall through

        case SM_ADDR_PROBE:

            switch ( zgzc_wait_for(&hZcll->random_delay, &hZcll->event_time, &hZcll->time_recorded) )
            {
                case ZGZC_STARTED_WAITING:

                    if (hZcll->probe_count == 0)
                    {
                        // First probe. Wait for [0 ~ PROBE_WAIT] seconds before sending the probe.

                        hZcll->random_delay = (zcll_rand() % (PROBE_WAIT * SYS_TICK_TicksPerSecondGet()));

                        DEBUG0_ZCLL_MESG(zeroconf_dbg_msg,"PROBE_WAIT Random Delay [%d]: %ld secs \r\n",
                                hZcll->probe_count,
                                hZcll->random_delay);
                    }
                    else if (hZcll->probe_count < PROBE_NUM)
                    {
                        // Subsequent probes. Wait for [PROBE_MIN ~ PROBE_MAX] seconds before sending the probe.

                        hZcll->random_delay = ( (zcll_rand() % ((PROBE_MAX-PROBE_MIN) * SYS_TICK_TicksPerSecondGet()) ) +
                                (PROBE_MIN * SYS_TICK_TicksPerSecondGet()) );

                        DEBUG0_ZCLL_MESG(zeroconf_dbg_msg,"PROBE Random Delay [%d]: %ld ticks \r\n",
                                hZcll->probe_count,
                                hZcll->random_delay);
                    }
                    else
                    {
                        // Completed PROBE_NUM of probes. Now wait for ANNOUNCE_WAIT seconds to determine if
                        // we can claim it.

                        hZcll->random_delay = (ANNOUNCE_WAIT * SYS_TICK_TicksPerSecondGet());
                        DEBUG0_ZCLL_MESG(zeroconf_dbg_msg,"ANNOUNCE_WAIT delay [%d]: %ld ticks\r\n",
                                hZcll->probe_count,
                                hZcll->random_delay /*SYS_TICK_TicksPerSecondGet() */);
                    }

                    DEBUG0_ZCLL_PRINT((char*)zeroconf_dbg_msg);

                    // Intentional fall-through

                case ZGZC_KEEP_WAITING:

                    // Not Completed the delay proposed
                    return true;
            }

            // Completed the delay required

            DEBUG0_ZCLL_MESG(zeroconf_dbg_msg,"   delay: %ld ticks " \
                    "completed \r\n", hZcll->random_delay);
            DEBUG0_ZCLL_PRINT((char *)zeroconf_dbg_msg);

            if(hZcll->zcll_flags.probe_conflict)
            {
                /* Conflict with selected address */
                INFO_ZCLL_PRINT("Probe Conflict-1 Detected. Need to select diff addr \r\n");
                hZcll->temp_IP_addr.Val = 0x0;

                hZcll->conflict_count++;
                pNetIf->netIPAddr.Val = 0x0;
            }
            else if((hZcll->conflict_count == 0) &&
                    hZcll->temp_IP_addr.Val      &&
                    pNetIf->netIPAddr.Val != 0x0 &&
                    (ARPIsResolved(pNetIf,&hZcll->temp_IP_addr, &hZcll->temp_MAC_addr)) )
            {
                if(!memcmp (&hZcll->temp_MAC_addr, &pNetIf->netMACAddr, 6) )
                {
                    DEBUG0_ZCLL_PRINT("SM_ADDR_PROBE: Resolved with our address only. " \
                            "Rare Case !!!! \r\n");
                }
                else
                {
                    /* Conflict with selected address */
                    INFO_ZCLL_PRINT("Probe Conflict-2 Detected. Need to select diff addr \r\n");
                    hZcll->temp_IP_addr.Val = 0x0;

                    hZcll->conflict_count++;
                    pNetIf->netIPAddr.Val = 0x0;
                }
            }

            if ((hZcll->zcll_flags.probe_conflict == 1) ||
                    (!hZcll->bDefaultIPTried))
            {
                /*
                 * Pick random IP address in IPv4 link-local range
                 * 169.254.1.0/16 is the allowed address range however
                 * 169.254.0.0/24 and 169.254.255.0/24 must be excluded,
                 * which removes 512 address from our 65535 candidates.
                 * That leaves us with 65023 (0xfdff).
                 * The link-local address must start with 169.254.#.#
                 * If it does not then assign it the default value of
                 169.254.1.2 and send out probe.
                 */
                hZcll->probe_count = 0;

                if(!hZcll->bDefaultIPTried)
                {
                    // First probe, and the default IP is a valid IPV4_SOFTAP_LLBASE address.
                    if (((!hZcll->bDefaultIPTried)          &&
                                (pNetIf->DefaultIPAddr.v[0] != 169)) ||
                            ((pNetIf->DefaultIPAddr.v[1] != 254) &&
                             (pNetIf->DefaultIPAddr.v[2] != 0)   &&
                             (pNetIf->DefaultIPAddr.v[3] != 255)))
                    {

                        WARN_ZCLL_MESG(zeroconf_dbg_msg,"\r\n%d.%d.%d.%d not a valid link local addess. "
                                "Autogenerating address.\r\n"
                                ,pNetIf->DefaultIPAddr.v[0],pNetIf->DefaultIPAddr.v[1]
                                ,pNetIf->DefaultIPAddr.v[2],pNetIf->DefaultIPAddr.v[3]);
                        WARN_ZCLL_PRINT((char *)zeroconf_dbg_msg);
                        // First probe, if the default IP is a valid IPv4 LL then use it.
                        hZcll->temp_IP_addr.Val = (IPV4_LLBASE | ((abs(zcll_rand()) % 0xfdff) ));
                        hZcll->bDefaultIPTried = 1;
                    }
                    else
                    {
                        hZcll->temp_IP_addr.Val = TCPIP_HELPER_ntohl(pNetIf->DefaultIPAddr.Val);
                    }

                    hZcll->bDefaultIPTried = 1;
                }

                INFO_ZCLL_MESG(zeroconf_dbg_msg,"Picked IP-Addr [%d]: %d.%d.%d.%d \r\n",
                        hZcll->probe_count,
                        hZcll->temp_IP_addr.v[3],hZcll->temp_IP_addr.v[2],
                        hZcll->temp_IP_addr.v[1],hZcll->temp_IP_addr.v[0]);
                INFO_ZCLL_PRINT((char *)zeroconf_dbg_msg);

                hZcll->temp_IP_addr.Val = TCPIP_HELPER_ntohl((uint32_t) hZcll->temp_IP_addr.Val);
            }

            if((hZcll->zcll_flags.probe_conflict == 1) || (hZcll->probe_count < PROBE_NUM))
            {

                hZcll->zcll_flags.probe_conflict = 0;

                ARPAction( pNetIf
                        , &pNetIf->netIPAddr
                        , &hZcll->temp_IP_addr
                        , ARP_OPERATION_REQ
                        , ZCLL_ARP_PROBE);
                hZcll->probe_count++;

                DEBUG0_ZCLL_MESG(zeroconf_dbg_msg, "Sending ARP [%d]\r\n", hZcll->probe_count);
                DEBUG0_ZCLL_PRINT((char *)zeroconf_dbg_msg);

                break;
            }

            // No conflict detected ...

            if(hZcll->probe_count >= PROBE_NUM)
            {
                hZcll->zcll_state = SM_ADDR_CLAIM;
                hZcll->bDefaultIPTried = 0;

                INFO_ZCLL_PRINT("ADDR_PROBE --> ADDR_CLAIM \r\n");

                return true;
            }

            break;

        case SM_ADDR_CLAIM:

            switch ( zgzc_wait_for( &hZcll->random_delay
                        ,&hZcll->event_time
                        ,&hZcll->time_recorded) )
            {
                case ZGZC_STARTED_WAITING:
                    if (hZcll->bDefaultIPTried == 0)
                    {
                        // First announcement is immediate. We have passed the ANNOUNCE_WAIT in
                        // PROBE state already.

                        hZcll->random_delay = 0;
                    }
                    else
                    {
                        // Subsequent announcements need to wait ANNOUNCE_INTERVAL seconds
                        // before sending the announcement.

                        hZcll->random_delay = (ANNOUNCE_INTERVAL * SYS_TICK_TicksPerSecondGet());
                    }
                    // Intentional fall-through

                case ZGZC_KEEP_WAITING:

                    // Not Completed the delay proposed
                    return true;
            }

            // Completed the delay required

            DEBUG0_ZCLL_MESG(zeroconf_dbg_msg,"ANNOUNCE delay: %ld ticks completed \r\n", hZcll->random_delay);
            DEBUG0_ZCLL_PRINT((char *)zeroconf_dbg_msg);

            if ( hZcll->bDefaultIPTried < ANNOUNCE_NUM )
            {
                ARPAction(pNetIf,&hZcll->temp_IP_addr,&hZcll->temp_IP_addr, ARP_OPERATION_REQ, ZCLL_ARP_CLAIM);
                (hZcll->bDefaultIPTried)++;

                DEBUG0_ZCLL_MESG(zeroconf_dbg_msg, "Sending ANNOUNCEMENT [%d]\r\n", hZcll->bDefaultIPTried);
                DEBUG0_ZCLL_PRINT((char *)zeroconf_dbg_msg);
            }
            else
            {
                // Claim it. Goto DEFEND state

                pNetIf->netIPAddr = hZcll->temp_IP_addr;
                pNetIf->netMask.Val = IPV4_LLBASE_MASK;
                hZcll->zcll_state = SM_ADDR_DEFEND;
                INFO_ZCLL_MESG(zeroconf_dbg_msg,"\r\n******** Taken IP-Addr: " \
                        "%d.%d.%d.%d ******** \r\n",
                        pNetIf->netIPAddr.v[0],pNetIf->netIPAddr.v[1],
                        pNetIf->netIPAddr.v[2],pNetIf->netIPAddr.v[3]);
                INFO_ZCLL_PRINT((char *)zeroconf_dbg_msg);
                INFO_ZCLL_PRINT("ADDR_CLAIM --> ADDR_DEFEND \r\n");
            }

            break;

        case SM_ADDR_DEFEND:

            if( hZcll->zcll_flags.late_conflict)
            {
                if (!hZcll->zcll_flags.defended)
                {
                    hZcll->zcll_flags.late_conflict = 0;
                    INFO_ZCLL_PRINT("CONFLICT DETECTED !!! \r\n");

                    INFO_ZCLL_PRINT("Defending the Self Address once \r\n");
                    ARPAction( pNetIf
                            ,&pNetIf->netIPAddr
                            ,&pNetIf->netIPAddr
                            ,ARP_OPERATION_RESP
                            ,ZCLL_ARP_DEFEND);

                    hZcll->zcll_flags.defended = true;
                }
                else
                {
                    // We are not allowed to defend another conflict during an active defended period

                    INFO_ZCLL_PRINT("Releasing the IP-Address because of multiple Conflicts \r\n");

                    hZcll->zcll_state = SM_ADDR_RELEASE;

                    hZcll->zcll_flags.defended = false;
                    hZcll->event_time = false;
                    hZcll->random_delay = false;

                    INFO_ZCLL_PRINT("ADDR_DEFEND --> ADDR_RELEASE \r\n");
                    break;
                }
            }

            if (hZcll->zcll_flags.defended)
            {
                switch ( zgzc_wait_for(&hZcll->random_delay, &hZcll->event_time, &hZcll->time_recorded) )
                {
                    case ZGZC_STARTED_WAITING:

                        hZcll->random_delay = (DEFEND_INTERVAL * SYS_TICK_TicksPerSecondGet());
                        DEBUG0_ZCLL_MESG(zeroconf_dbg_msg,"DEFEND_INTERVAL Delay : %ld ticks\r\n",
                                hZcll->random_delay/*SYS_TICK_TicksPerSecondGet() */);
                        DEBUG0_ZCLL_PRINT((char *)zeroconf_dbg_msg);

                        // Intentional fall-through

                    case ZGZC_KEEP_WAITING:

                        // Not Completed the delay proposed
                        return true;
                }

                // Completed the delay required

                DEBUG0_ZCLL_MESG(zeroconf_dbg_msg,"ANNOUNCE delay: %ld ticks " \
                        "completed \r\n", hZcll->random_delay);
                DEBUG0_ZCLL_PRINT((char *)zeroconf_dbg_msg);

                hZcll->zcll_flags.defended = false;
            }

            break;

        case SM_ADDR_RELEASE:

            INFO_ZCLL_PRINT("ADDR_RELEASE --> ADDR_INIT\r\n");

            pNetIf->netIPAddr.Val = 0x00;

            // Need New Addr
            hZcll->temp_IP_addr.Val = (IPV4_LLBASE | ((abs(zcll_rand()) % 0xfdff) ));
            hZcll->temp_IP_addr.Val = TCPIP_HELPER_ntohl((uint32_t) hZcll->temp_IP_addr.Val);

            hZcll->zcll_state = SM_ADDR_INIT;
            hZcll->time_recorded = false;
            hZcll->zcll_flags.defended      = false;
            hZcll->event_time    = false;
            break;

        default:
            break;
    }

    return true;
}

#endif  // defined(TCPIP_STACK_USE_ZEROCONF_LINK_LOCAL)

