/*******************************************************************************
  TCP/IP Stack Manager

  Summary:
    Module for Microchip TCP/IP Stack

  Description:
    -Handles internal RX packet pre-processing prior to dispatching
     to upper application layers.
    -Reference: AN833
*******************************************************************************/

/*******************************************************************************
FileName:   tcpip_manager.c
Copyright ©2012 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED ?AS IS?WITHOUT WARRANTY OF ANY KIND,
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

#define TCPIP_THIS_MODULE_ID    TCPIP_MODULE_TCPIP_MANAGER

#include "tcpip_mac_private.h"

#include "tcpip/tcpip_mac_object.h"

#include "tcpip_module_manager.h"

#define TCPIP_STACK_INIT_MESSAGE   "TCP/IP Stack Initialization "

#include "tcpip_notify.h"

NODE_INFO remoteNode;


static TCPIP_NET_IF* tcpipNetIf = 0;       // dynamically allocated

// Main default interfaces
typedef struct
{
    TCPIP_NET_IF* defaultNet;     // default network interface
}TCPIPDefaultIF;


static TCPIPDefaultIF tcpipDefIf = { 0 };

static volatile int    totTcpipEventsCnt = 0;
static volatile int    newTcpipTickAvlbl = 0;

static volatile int    newTcpipErrorEventCnt = 0;
static volatile int    newTcpipStackEventCnt = 0;


static void    TCPIPMacEventCB(void* hParam, TCPIP_EVENT event);

static uint8_t*    tcpip_heap = 0;          // the actual TCPIP heap

static TCPIP_STACK_MODULE_CTRL  tcpip_stack_ctrl_data = {0};

//
static SystemTickHandle    tcpip_stack_tickH = 0;      // tick handle

static const TCPIP_NETWORK_CONFIG tcpip_def_config = 
{
    TCPIP_NETWORK_DEFAULT_INTERFACE_NAME,
    TCPIP_NETWORK_DEFAULT_HOST_NAME,
    TCPIP_NETWORK_DEFAULT_MAC_ADDR,
    TCPIP_NETWORK_DEFAULT_IP_ADDRESS,
    TCPIP_NETWORK_DEFAULT_IP_MASK,
    TCPIP_NETWORK_DEFAULT_GATEWAY,
    TCPIP_NETWORK_DEFAULT_DNS,
    TCPIP_NETWORK_DEFAULT_SECOND_DNS,
    TCPIP_NETWORK_DEFAULT_POWER_MODE,
    (TCPIP_NETWORK_CONFIG_DHCP_CLIENT_ON),
};

static void TCPIP_STACK_TickHandler(SYS_TICK currSysTick);        // stack tick handler

static void ProcessTCPIPTickEvent(void);
static void ProcessTCPIPMacEvents(TCPIP_NET_IF* pNetIf, TCPIP_EVENT activeEvent);
static void ProcessTCPIPMacGeneric(TCPIP_NET_IF* pNetIf);
static void ProcessTCPIPMacErrorEvents(TCPIP_NET_IF* pNetIf, TCPIP_EVENT activeEvent);

static bool InitNetConfig(const TCPIP_NETWORK_CONFIG* pUsrConfig, int nNets);
static bool LoadDefaultConfig(const TCPIP_NETWORK_CONFIG* pUsrConfig, TCPIP_NET_IF* pNetIf);

#if defined(TCPIP_IF_MRF24W)
static bool TCPIP_STACK_VerifyPktIf(TCPIP_NET_IF* pNetIf, IPV4_ADDR* pktDestIP);
static bool TCPIP_STACK_FilterRxPkt(TCPIP_NET_IF* pNetIf, NODE_INFO* pPkt);
#endif  // defined(TCPIP_IF_MRF24W)

static const TCPIP_STACK_MODULE_CONFIG* TcpipStackFindModuleData(TCPIP_STACK_MODULE moduleId, const TCPIP_STACK_MODULE_CONFIG* pModConfig, int nModules);

static const TCPIP_STACK_MODULE_MAC_ENTRY* TcpipStackFindMacModule(TCPIP_STACK_MODULE moduleId);

static void  TCPIP_STACK_BringNetDown(TCPIP_STACK_MODULE_CTRL* stackCtrlData);
static bool  TCPIP_STACK_BringNetUp(TCPIP_STACK_MODULE_CTRL* stackCtrlData, const TCPIP_NETWORK_CONFIG* pNetConf, const TCPIP_STACK_MODULE_CONFIG* pModConfig, int nModules);

static void* NetConfigStringToBuffer(void** ppDstBuff, void* pSrcBuff, size_t* pDstSize, size_t* pCurrLen);


/*********************************************************************
 * Function:        bool TCPIP_STACK_Init(const TCPIP_NETWORK_CONFIG* pNetConf, int nNets, TCPIP_STACK_MODULE_CONFIG* pModConfig, int nModules)
 *
 * PreCondition:    None
 *
 * Input:           pNetConf      - pointer to an array of TCPIP_NETWORK_CONFIG to support
 *                  nNets       - number of network configurations in the array
 *                  pModConfig  - pointer to an array of TCPIP_STACK_MODULE_CONFIG
 *                  nModules    - number of modules to initialize
 *
 * Output:          true if Stack and its componets are initialized
 *                  false otherwise
 *
 * Overview:        The function initializes the stack.
 *                  If an error occurs, the SYS_ERROR() is called
 *                  and the function de-initialize itself and will return false.
 *
 * Side Effects:    None
 *
 * Note:            This function must be called before any of the
 *                  stack or its component routines are used.
 *
 *                  New TCPIP_NETWORK_CONFIG types should be added/removed at run time for implementations that support
 *                  dynamic network interface creation.
 *
 ********************************************************************/
bool TCPIP_STACK_Init(const TCPIP_NETWORK_CONFIG* pUsrConfig, int nNets, const TCPIP_STACK_MODULE_CONFIG* pModConfig, int nModules)
{
    int                     netIx, initFail;
    TCPIP_HEAP_HANDLE       heapH;
    TCPIP_NET_IF*           pIf;
    TCPIP_MAC_POWER_MODE  powerMode;

    if(tcpipNetIf != 0)
    {   // already up and running
        return true;
    }

    if(pUsrConfig == 0)
    {
        pUsrConfig = & tcpip_def_config;
        nNets = 1;  // TCPIP_NETWORK_INTERFACES_NO; hard coded!
    }
    else if(nNets == 0)
    {   // cannot run with no interface
        return false;
    }

    SYS_CONSOLE_MESSAGE(TCPIP_STACK_INIT_MESSAGE "Started \n\r");


    while(true)
    {
        initFail = 0;

        totTcpipEventsCnt = 0;

        newTcpipErrorEventCnt = 0;
        newTcpipStackEventCnt = 0;
        newTcpipTickAvlbl = 0;

        // start stack initialization

        memset(&tcpip_stack_ctrl_data, 0, sizeof(tcpip_stack_ctrl_data));

        tcpip_heap = (uint8_t*)SystemMalloc(TCPIP_STACK_DRAM_SIZE);

        heapH = TCPIP_HEAP_Create(tcpip_heap, TCPIP_STACK_DRAM_SIZE, 0, 0);     // get handle to the heap memory
        if(heapH == 0)
        {
            SystemFree(tcpip_heap);     // free the allocated memory
            tcpip_heap = 0;
            SYS_ERROR_PRINT(SYS_ERROR_ERROR, "Heap creation failed, size: %d\r\n", TCPIP_STACK_DRAM_SIZE);
            initFail = 1;
            break;
        }

        tcpipNetIf = (TCPIP_NET_IF*)TCPIP_HEAP_Calloc(heapH, nNets, sizeof(TCPIP_NET_IF)); // allocate for each network interface
        if(tcpipNetIf == 0)
        {   // failed
            SYS_ERROR(SYS_ERROR_ERROR, "Network configuration allocation failed\r\n");
            TCPIP_HEAP_Delete(heapH);     // destroy the heap
            SystemFree(tcpip_heap);     // free the allocated memory
            tcpip_heap = 0;
            initFail = 1;
            break;
        }

        tcpip_stack_ctrl_data.memH = heapH;
        tcpip_stack_ctrl_data.nIfs = nNets;
        tcpip_stack_ctrl_data.nMdls = nModules;

        // Seed the LFSRRand() function
        LFSRSeedRand(SYS_GENERATE_RANDOM_DWORD());

        if(!InitNetConfig(pUsrConfig, nNets))
        {
            SYS_ERROR(SYS_ERROR_ERROR, "Network configuration initialization failed\r\n");
            initFail = 1;   // failed the initialization
            break;
        }

        tcpipDefIf.defaultNet = 0;          // delete the old default
        // start per interface initializing
        tcpip_stack_ctrl_data.stackAction = TCPIP_STACK_ACTION_INIT;

        for(netIx = 0, pIf = tcpipNetIf; netIx < nNets; netIx++, pIf++)
        {
            // get the power mode
#if defined (__C30__)  
            powerMode = TCPIP_MAC_POWER_FULL;
#else
            powerMode = TCPIP_Helper_StringToPowerMode(pUsrConfig->powerMode);
#endif
            if(powerMode == TCPIP_MAC_POWER_NONE || powerMode != TCPIP_MAC_POWER_FULL)
            {   
                SYS_ERROR(SYS_ERROR_ERROR, "Stack Power Mode initialization fail\r\n");
                initFail = 1;
                break;
            }

            // set transient data
            tcpip_stack_ctrl_data.powerMode = powerMode;
            tcpip_stack_ctrl_data.pNetIf = pIf;
            tcpip_stack_ctrl_data.netIx = netIx;
            if(!TCPIP_STACK_BringNetUp(&tcpip_stack_ctrl_data, pUsrConfig, pModConfig, nModules))
            {
                initFail = 1;
                break;
            }

            // interface success
            // set the default interfaces
            if(tcpipDefIf.defaultNet == 0)
            {
                tcpipDefIf.defaultNet = pIf;    // set as the 1st valid interface
            }
        }

    // initializing the rest of the services
    // (that don't need interface specific initialization)

        if(!initFail)
        {
            tcpip_stack_tickH = SYS_TICK_TimerCreate(TCPIP_STACK_TickHandler);
            if(tcpip_stack_tickH)
            {
                SYS_TICK_TimerSetRate(tcpip_stack_tickH, ((SYS_TICK_ResolutionGet() * TCPIP_STACK_TICK_RATE) + 999 )/1000);
            }
            else
            {
                SYS_ERROR(SYS_ERROR_ERROR, "Stack tick registration failed\r\n");
                initFail = 1;
                break;
            }
        }

        break;
    }

    // initialization done
    if(!initFail)
    {
        size_t heapLeft;
        SYS_CONSOLE_MESSAGE(TCPIP_STACK_INIT_MESSAGE "Ended - success \n\r");
        // check the amount of heap left
        heapLeft = TCPIP_HEAP_FreeSize(heapH);
        if(heapLeft < TCPIP_STACK_DRAM_RUN_LIMIT)
        {
            SYS_ERROR_PRINT(SYS_ERROR_WARN, "Dynamic memory is low: %d\r\n", heapLeft);
        }
        return true;
    }


    SYS_CONSOLE_MESSAGE(TCPIP_STACK_INIT_MESSAGE "failed - Aborting! \n\r");
    TCPIP_STACK_DeInit();
    return false;

}

/*********************************************************************
 * Function:        bool TCPIP_STACK_BringNetUp(TCPIP_NET_IF* pNetIf, const TCPIP_NETWORK_CONFIG* pNetConf, const TCPIP_STACK_MODULE_CONFIG* pModConfig, int nModules)
 *
 * PreCondition:    None
 *
 * Input:           net interface to bring up
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function performs the initialization of a net interface
 *
 * Note:            None
 ********************************************************************/
static bool TCPIP_STACK_BringNetUp(TCPIP_STACK_MODULE_CTRL* stackCtrlData, const TCPIP_NETWORK_CONFIG* pNetConf, const TCPIP_STACK_MODULE_CONFIG* pModConfig, int nModules)
{
    TCPIP_NET_IF*           pNetIf;
    bool                    netUpFail;
    const void*             configData;
    const TCPIP_STACK_MODULE_CONFIG* pConfig;
    TCPIP_MAC_RES           macInitRes;

    netUpFail = false;
    pNetIf = stackCtrlData->pNetIf;
    // restore the dynamic interface data
    pNetIf->netIfIx = stackCtrlData->netIx;
    pNetIf->Flags.powerMode = stackCtrlData->powerMode;


    while(true)
    {
        // start stack MAC modules initialization
        const TCPIP_STACK_MODULE_MAC_ENTRY*  pMacEntry = TcpipStackFindMacModule(stackCtrlData->pNetIf->macId);

        if(pMacEntry == 0)
        {
            SYS_ERROR_PRINT(SYS_ERROR_ERROR, "%s MAC module not found\r\n", pNetConf->interface);
            netUpFail = 1;
            break;
        }
        // find MAC initialization data
        configData = 0;
        if (pModConfig != 0)
        {
            pConfig = TcpipStackFindModuleData(pMacEntry->moduleId, pModConfig, nModules);
            if(pConfig != 0)
            {
                configData = pConfig->configData;
            }
        }
        // init the MAC
        do
        {
            macInitRes = (*pMacEntry->initFunc)(stackCtrlData, configData);
        }while(macInitRes == TCPIP_MAC_RES_PENDING);


        if( macInitRes != TCPIP_MAC_RES_OK)
        {
            SYS_ERROR_PRINT(SYS_ERROR_ERROR, "%s MAC initialization failed\r\n", pNetConf->interface);
            netUpFail = 1;
            break;
        }
        // open the MAC
        if( (pNetIf->hIfMac = (*pMacEntry->openFunc)(pNetIf->macId)) == 0)
        {
            SYS_ERROR_PRINT(SYS_ERROR_ERROR, "%s MAC Open failed\r\n", pNetConf->interface);
            netUpFail = 1;
            break;
        }

        // start stack initialization per module
        int modIx;
        const TCPIP_STACK_MODULE_ENTRY*  pEntry = TCPIP_STACK_MODULE_ENTRY_TBL + 0;

        for(modIx = 0; modIx < sizeof(TCPIP_STACK_MODULE_ENTRY_TBL)/sizeof(*TCPIP_STACK_MODULE_ENTRY_TBL); modIx++)
        {
            configData = 0;
            if (pModConfig != 0)
            {
                pConfig = TcpipStackFindModuleData(pEntry->moduleId, pModConfig, nModules);
                if(pConfig != 0)
                {
                    configData = pConfig->configData;
                }
            }

            if(!pEntry->initFunc(stackCtrlData, configData))
            {
                SYS_ERROR_PRINT(SYS_ERROR_ERROR, "Module no: %d Initialization failed\r\n", pEntry->moduleId);
                netUpFail = 1;
                break;
            }
            pEntry++;
        }

        if(!netUpFail)
        {
#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
            TCPIP_MAC_EVENT_RESULT  evRes;

            // Stack can use one handler for all network interfaces, like in this case
            // Each time a notification is received, all interfaces are checked
            // Or, more efficient, use a handler per interface
            evRes = MACEventSetNotifyHandler(pNetIf->hIfMac, TCPIP_MAC_EVGROUP_ALL, TCPIPMacEventCB, pNetIf);

            if(evRes != TCPIP_MAC_EVRES_OK)
            {
                SYS_ERROR_PRINT(SYS_ERROR_ERROR, "%s MAC event notification registration failed\r\n", pNetConf->interface);
                netUpFail = 1;
                break;
            }

            evRes = MACEventSetNotifyEvents(pNetIf->hIfMac, TCPIP_MAC_EVGROUP_ALL,
                                            TCPIP_EV_RX_PKTPEND | TCPIP_EV_TX_DONE | TCPIP_EV_RXTX_ERRORS | TCPIP_EV_CONN_ALL);
            if(evRes != TCPIP_MAC_EVRES_OK)
            {
                SYS_ERROR_PRINT(SYS_ERROR_ERROR, "%s MAC event notification setting failed\r\n", pNetConf->interface);
                netUpFail = 1;
                break;
            }
#endif  // defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)

            // completed the MAC initialization

#if defined(TCPIP_IF_MRF24W)
            // Wi-Fi connect
            if(pNetIf->macId == TCPIP_MODULE_MAC_MRF24W)
            {
                MRF24W_MACConnect(pNetIf->hIfMac);
            }
#endif  // defined(TCPIP_IF_MRF24W)



        }

        break;
    }

    if(netUpFail)
    {
        return false;
    }


    pNetIf->Flags.bInterfaceEnabled = true;
    pNetIf->Flags.powerMode = stackCtrlData->powerMode;

    return true;

}

/*********************************************************************
 * Function:        void TCPIP_STACK_NetUp(TCPIP_NET_HANDLE netH, const TCPIP_NETWORK_CONFIG* pUsrConfig)
 *
 * PreCondition:    None
 *
 * Input:           net interface to bring up
 *
 * Output:          true if success
 *                  false if no such network or an error occurred
 *
 * Side Effects:    None
 *
 * Overview:        This function performs the de-initialization of a net interface
 *
 * Note:            None
 ********************************************************************/
bool TCPIP_STACK_NetUp(TCPIP_NET_HANDLE netH, const TCPIP_NETWORK_CONFIG* pUsrConfig)
{
    bool    success;
    TCPIP_MAC_POWER_MODE  powerMode;

    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNet(netH);

    if(pNetIf)
    {
        if(pNetIf->Flags.bInterfaceEnabled)
        {   // already up
            return true;
        }

        if(pUsrConfig == 0)
        {
            pUsrConfig = & tcpip_def_config;
        }

        // Before we load the default config, we should save what used to be the netIfIx
        // set transient data
        tcpip_stack_ctrl_data.pNetIf = pNetIf;
        tcpip_stack_ctrl_data.netIx = pNetIf->netIfIx;
        tcpip_stack_ctrl_data.stackAction = TCPIP_STACK_ACTION_IF_UP;

        if(!LoadDefaultConfig(pUsrConfig, pNetIf))
        {
            SYS_ERROR(SYS_ERROR_ERROR, "Default Flash Network configuration load failed\r\n");
            return false;
        }

#if defined (__C30__)  
        powerMode = TCPIP_MAC_POWER_FULL;
#else
        powerMode = TCPIP_Helper_StringToPowerMode(pUsrConfig->powerMode);
#endif
        if(powerMode == TCPIP_MAC_POWER_NONE || powerMode != TCPIP_MAC_POWER_FULL)
        {  
            SYS_ERROR(SYS_ERROR_ERROR, "Stack Power Mode initialization fail\r\n");
            return false;
        }

        tcpip_stack_ctrl_data.powerMode = powerMode;

        success = TCPIP_STACK_BringNetUp(&tcpip_stack_ctrl_data, pUsrConfig, 0, 0);
        if(success)
        {
            if(tcpipDefIf.defaultNet == 0)
            {   // make this the default interface
                tcpipDefIf.defaultNet = pNetIf;
            }
        }
        else
        {   // don't let the MAC hanging because of a module failure
            tcpip_stack_ctrl_data.stackAction = TCPIP_STACK_ACTION_IF_DOWN;
            tcpip_stack_ctrl_data.powerMode = TCPIP_MAC_POWER_DOWN;
            TCPIP_STACK_BringNetDown(&tcpip_stack_ctrl_data);
        }
        return success;
    }

    return false;

}
/*********************************************************************
 * Function:        void TCPIP_STACK_DeInit(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function performs the de-initialization of the TCPIP stack
 *
 * Note:            None
 ********************************************************************/
void TCPIP_STACK_DeInit(void)
{
    int         netIx;
    TCPIP_NET_IF* pIf;

    if(tcpipNetIf == 0)
    {   // already shut down
        return;
    }

    SYS_TICK_TimerDelete(tcpip_stack_tickH);
    tcpip_stack_tickH = 0;

    // set transient data
    tcpip_stack_ctrl_data.stackAction = TCPIP_STACK_ACTION_DEINIT;
    tcpip_stack_ctrl_data.powerMode = TCPIP_MAC_POWER_DOWN;

    for(netIx = 0, pIf = tcpipNetIf; netIx < tcpip_stack_ctrl_data.nIfs; netIx++, pIf++)
    {
        tcpip_stack_ctrl_data.pNetIf = pIf;
        tcpip_stack_ctrl_data.netIx = pIf->netIfIx;
        if(pIf->Flags.bInterfaceEnabled)
        {
            TCPIP_STACK_BringNetDown(&tcpip_stack_ctrl_data);
        }
#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
        TCPIP_NotificationRemoveAll(&pIf->registeredClients, tcpip_stack_ctrl_data.memH);
#endif
    }


    TCPIP_HEAP_Free(tcpip_stack_ctrl_data.memH, tcpipNetIf);
    TCPIP_HEAP_Delete(tcpip_stack_ctrl_data.memH);     // destry the heap
    tcpip_stack_ctrl_data.memH = 0;
    tcpipNetIf = 0;

    SystemFree(tcpip_heap);     // free the allocated memory
    tcpip_heap = 0;

    tcpip_stack_ctrl_data.nIfs = 0;
    tcpip_stack_ctrl_data.nMdls = 0;
}


/*********************************************************************
 * Function:        void TCPIP_STACK_BringNetDown(TCPIP_STACK_MODULE_CTRL* stackCtrlData)
 *
 * PreCondition:    None
 *
 * Input:           net interface to bring down
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function performs the de-initialization of a net interface
 *
 * Note:            None
 ********************************************************************/
static void TCPIP_STACK_BringNetDown(TCPIP_STACK_MODULE_CTRL* stackCtrlData)
{
    const TCPIP_STACK_MODULE_ENTRY*  pEntry;
    const TCPIP_STACK_MODULE_MAC_ENTRY*  pMacEntry;
    TCPIP_NET_IF* pNetIf;


    // Go to the last entry in the table
    pEntry = TCPIP_STACK_MODULE_ENTRY_TBL + sizeof(TCPIP_STACK_MODULE_ENTRY_TBL)/sizeof(*TCPIP_STACK_MODULE_ENTRY_TBL);
    do
    {
        pEntry--;
        pEntry->deInitFunc(stackCtrlData);
    }
    while (pEntry != TCPIP_STACK_MODULE_ENTRY_TBL);

    pNetIf = stackCtrlData->pNetIf;
    if(pNetIf->hIfMac)
    {
        MACClose(pNetIf->hIfMac);
        pNetIf->hIfMac = 0;
    }
    // kill the MAC
    pMacEntry = TcpipStackFindMacModule(stackCtrlData->pNetIf->macId);
    (*pMacEntry->deInitFunc)(stackCtrlData);

    pNetIf->Flags.bInterfaceEnabled = false;
    pNetIf->Flags.powerMode = stackCtrlData->powerMode;

}

/*********************************************************************
 * Function:        void TCPIP_STACK_NetDown(TCPIP_NET_HANDLE netH)
 *
 * PreCondition:    None
 *
 * Input:           net interface to bring down
 *
 * Output:          true if success
 *                  false if no such network
 *
 * Side Effects:    None
 *
 * Overview:        This function performs the de-initialization of a net interface
 *
 * Note:            None
 ********************************************************************/
bool TCPIP_STACK_NetDown(TCPIP_NET_HANDLE netH)
{
    int netIx;
    TCPIP_NET_IF *pIf, *pNewIf;
    TCPIP_NET_IF* pDownIf = _TCPIPStackHandleToNet(netH);

    if(pDownIf)
    {
        if(pDownIf->Flags.bInterfaceEnabled)
        {
            // set transient data
            tcpip_stack_ctrl_data.stackAction = TCPIP_STACK_ACTION_IF_DOWN;
            tcpip_stack_ctrl_data.powerMode = TCPIP_MAC_POWER_DOWN;
            tcpip_stack_ctrl_data.pNetIf = pDownIf;
            tcpip_stack_ctrl_data.netIx = pDownIf->netIfIx;

            if(tcpipDefIf.defaultNet == pDownIf)
            {   // since this interface is going down change the default interface
                pNewIf = 0;
                for(netIx = 0, pIf = tcpipNetIf; netIx < tcpip_stack_ctrl_data.nIfs; netIx++, pIf++)
                {
                    if(pIf != pDownIf && pIf->Flags.bInterfaceEnabled)
                    {   // select this one
                        pNewIf = pIf;
                        break;
                    }
                }
                tcpipDefIf.defaultNet = pNewIf;
            }

            TCPIP_STACK_BringNetDown(&tcpip_stack_ctrl_data);
        }
        return true;
    }

    return false;

}


/*********************************************************************
 * Function:        void TCPIP_STACK_Task(void)
 *
 * PreCondition:    TCPIP_STACK_Init() is already called.
 *
 * Input:           None
 *
 * Output:          Stack Finite-state Machine (FSM) is executed.
 *
 * Side Effects:    None
 *
 * Note:            This FSM checks for new incoming packets,
 *                  and routes it to appropriate stack components.
 *                  It also performs timed operations.
 *
 *                  This function must be called periodically to
 *                  ensure timely responses.
 ********************************************************************/
void TCPIP_STACK_Task(void)
{
    int                 netIx, modIx;
    TCPIP_NET_IF*       pNetIf;
    bool                tickEvent;
    const TCPIP_STACK_SYNC_MODULE_ENTRY*   pSyncEntry;
    const TCPIP_STACK_ASYNC_MODULE_ENTRY*  pAsyncEntry;

    if( newTcpipTickAvlbl)
    {
        ProcessTCPIPTickEvent();
        tickEvent = true;
    }
    else
    {
        tickEvent = false;
    }

#if !defined(TCPIP_STACK_USE_EVENT_NOTIFICATION) || defined(TCPIP_IF_MRF24W)
    totTcpipEventsCnt = 1;  // fake that we always have an event pending
                            // the MRF24W cannot really work in interrupt mode!
#endif

    if( totTcpipEventsCnt || tickEvent)
    {   // event pending
        TCPIP_EVENT activeEvents;

        totTcpipEventsCnt = 0;


        for(netIx = 0, pNetIf = tcpipNetIf; netIx < tcpip_stack_ctrl_data.nIfs; netIx++, pNetIf++)
        {
            if (!pNetIf->Flags.bInterfaceEnabled)
            {
                continue;
            }
            activeEvents =  pNetIf->activeEvents;

#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
            TCPIP_MAC_HANDLE activeMac = pNetIf->hIfMac;
            {
                TCPIP_EVENT macEvents = 0;
                if(pNetIf->Flags.bNewTcpipEventAvlbl)
                {
                    pNetIf->Flags.bNewTcpipEventAvlbl = 0;
                    macEvents = MACEventGetPending(activeMac, TCPIP_MAC_EVGROUP_ALL);
                }
#if defined(TCPIP_IF_MRF24W)
                else if(pNetIf->macId == TCPIP_MODULE_MAC_MRF24W)
                {
                    macEvents = TCPIP_EV_RX_PKTPEND|TCPIP_EV_TX_DONE;    // fake pending MAC events
                    // the MRF24W cannot really work in interrupt mode!
                }
#endif  // defined(TCPIP_IF_MRF24W)
                activeEvents |= macEvents;
            }
#else
            activeEvents |= TCPIP_EV_RX_PKTPEND|TCPIP_EV_TX_DONE;    // just fake pending events
#endif  // defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)

            // clear processed events
            pNetIf->activeEvents &= ~activeEvents;

            if(activeEvents || tickEvent)
            {
                if(pNetIf->Flags.bInterfaceEnabled)
                {
                    ProcessTCPIPMacGeneric(pNetIf);

                    if(activeEvents&(TCPIP_EV_RX_PKTPEND|TCPIP_EV_TX_DONE))
                    {
                        ProcessTCPIPMacEvents(pNetIf, activeEvents);
                        newTcpipStackEventCnt++;
                    }

                    // process the synchronous handlers
                    pSyncEntry = TCPIP_STACK_MODULE_SYNC_TBL + 0;
                    for(modIx = 0; modIx < sizeof(TCPIP_STACK_MODULE_SYNC_TBL)/sizeof(*TCPIP_STACK_MODULE_SYNC_TBL); modIx++)
                    {
                        (*pSyncEntry->syncHandler)(pNetIf);
                        pSyncEntry++;
                    }

                    if(activeEvents&TCPIP_EV_RXTX_ERRORS)
                    {    // some error has occurred
                        ProcessTCPIPMacErrorEvents(pNetIf, activeEvents);
                    }
                }

                if((activeEvents & TCPIP_EV_CONN_ALL) != 0)
                {
                    for(modIx = 0; modIx < sizeof(TCPIP_STACK_CONN_EVENT_TBL)/sizeof(*TCPIP_STACK_CONN_EVENT_TBL); modIx++)
                    {
                        (*TCPIP_STACK_CONN_EVENT_TBL[modIx])(pNetIf, activeEvents);
                    }
                }


#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
                activeEvents &= ~TCPIP_EV_CONN_ALL; // connection events generated internally so no need to be ack-ed
                if(activeEvents)
                {
                    MACEventAck(activeMac, TCPIP_MAC_EVGROUP_ALL, activeEvents);
                }
#endif  // defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
            }
        }
    }

    // process the asynchronous handlers
    pAsyncEntry = TCPIP_STACK_MODULE_ASYNC_TBL + 0;
    for(modIx = 0; modIx < sizeof(TCPIP_STACK_MODULE_ASYNC_TBL)/sizeof(*TCPIP_STACK_MODULE_ASYNC_TBL); modIx++)
    {
        if((*pAsyncEntry->asyncPending)())
        {
            (*pAsyncEntry->asyncHandler)();
        }
        pAsyncEntry++;
    }
}

// non event specific processing
static void ProcessTCPIPMacGeneric(TCPIP_NET_IF* pNetIf)
{
#if defined(TCPIP_IF_MRF24W)
            // Wi-Fi connect
            if(pNetIf->macId == TCPIP_MODULE_MAC_MRF24W)
            {
                MRF24W_MACProcess(pNetIf->hIfMac);
            }
#endif  // defined(TCPIP_IF_MRF24W)


#if defined(TCPIP_STACK_USE_DHCP_CLIENT)
    DHCPTask(pNetIf);
#endif

#if defined(TCPIP_STACK_USE_UDP)
    UDPTask(pNetIf);
#endif
}

static void ProcessTCPIPTickEvent(void)
{
    int netIx;
    TCPIP_NET_IF* pNetIf;
    bool    linkCurr;

    newTcpipTickAvlbl = 0;

    for(netIx = 0, pNetIf = tcpipNetIf; netIx < tcpip_stack_ctrl_data.nIfs; netIx++, pNetIf++)
    {
        if(pNetIf->Flags.bInterfaceEnabled)
        {
            linkCurr = MACCheckLink(pNetIf->hIfMac);     // check link status
            if(pNetIf->linkPrev != linkCurr)
            {   // link status changed
                // just call directly the CB, and do not involve the MAC notification mechanism
                TCPIPMacEventCB(pNetIf, linkCurr?TCPIP_EV_CONN_ESTABLISHED:TCPIP_EV_CONN_LOST);
                pNetIf->linkPrev = linkCurr;
            }
        }
    }

}

static void ProcessTCPIPMacEvents(TCPIP_NET_IF* pNetIf, TCPIP_EVENT activeEvent)
{

    uint16_t dataCount;
    IPV4_ADDR pktDestIP;
    uint16_t cFrameType;
    uint8_t cIPFrameType;

    // Process as many incomming packets as we can
    while(activeEvent & TCPIP_EV_RX_PKTPEND)
    {
        // if using the random module, generate entropy
        // Entropy is generated as follows:
        //    1.     For every packet received, the last byte of the remote
        //        MAC address and the four bytes of SYS_TICK_Get() are added
        //        to a SHA-1 Hash (the seed)
        //    2.    Every time a random byte is requested, the hash is
        //        calculated and the hash sum is hashed into the seed.
        //    3.    Up to 20 bytes are returned from this hash sum before
        //        a new hash is calculated.
        //    4.    Every time new entropy is added, the remaining random
        //        output is discarded and new random bytes will be
        //        generated upon future calls to SYS_RANDOM_GET().
        #if defined(TCPIP_STACK_USE_RANDOM)
            SYS_RANDOM_ADD(remoteNode.MACAddr.v[5]);
        #endif

        // We are about to fetch a new packet, make sure that the
        // UDP module knows that any old RX data it has laying
        // around will now be gone.
        #if defined(TCPIP_STACK_USE_UDP)
            UDPDiscardNet(pNetIf);
        #endif

        // Fetch a packet (throws old one away, if not thrown away
        // yet)
        if(!MACGetHeader(pNetIf->hIfMac, &remoteNode.MACAddr, &cFrameType))
        {
            break;
        }

#if defined(TCPIP_IF_MRF24W)
        if(pNetIf->macId == TCPIP_MODULE_MAC_MRF24W)
        {
            if(TCPIP_STACK_FilterRxPkt(pNetIf, &remoteNode))
            {
                continue;       // packet filtered out
            }
        }
#endif  // defined(TCPIP_IF_MRF24W)

        // Dispatch the packet to the appropriate handler
        switch(cFrameType)
        {
#if defined (TCPIP_STACK_USE_IPV4)
            case ETHERTYPE_ARP:
                ARPProcess(pNetIf);
                break;

            case ETHERTYPE_IPV4:
                if(!TCPIP_IPV4_GetHeader(pNetIf, &pktDestIP, &remoteNode, &cIPFrameType, &dataCount))
                    break;

                // check the packet arrived on the proper interface
#if defined(TCPIP_IF_MRF24W)
                if(pNetIf->macId == TCPIP_MODULE_MAC_MRF24W)
                {
                    if(!TCPIP_STACK_VerifyPktIf(pNetIf, &pktDestIP))
                    {
                        break;   // discard
                    }
                }
#endif  // defined(TCPIP_IF_MRF24W)

                #if defined(TCPIP_STACK_USE_ICMP_SERVER) || defined(TCPIP_STACK_USE_ICMP_CLIENT)
                if(cIPFrameType == IP_PROT_ICMP)
                {
                    ICMPProcess(pNetIf, &remoteNode, dataCount);
                    break;
                }
                #endif

                #if defined(TCPIP_STACK_USE_TCP)
                if(cIPFrameType == IP_PROT_TCP)
                {
                    TCPProcessIPv4(pNetIf, &remoteNode, &pktDestIP, dataCount);
                    break;
                }
                #endif

                #if defined(TCPIP_STACK_USE_UDP)
                if(cIPFrameType == IP_PROT_UDP)
                {
                    // Stop processing packets if we came upon a UDP frame with application data in it
                    if(UDPProcessIPv4(pNetIf, &remoteNode, &pktDestIP, dataCount))
                    {
                        return;
                    }
                }
                #endif

                break;
#endif  // defined (TCPIP_STACK_USE_IPV4)

#if defined(TCPIP_STACK_USE_IPV6)
            case ETHERTYPE_IPV6:
                if (pNetIf->Flags.bIPv6Enabled)
                {
                    if(TCPIP_IPV6_Process(pNetIf, &remoteNode.MACAddr))
                    {   // Stop processing packets if we came upon a UDP frame with application data in it
                        return;
                    }
                }
                break;
#endif  // defined (TCPIP_STACK_USE_IPV6)

            default:
                break;
        }
    }
}


static void    TCPIPMacEventCB(void* hParam, TCPIP_EVENT event)
{
    TCPIP_NET_IF* pNetIf = (TCPIP_NET_IF*)hParam;

    pNetIf->activeEvents |= event;
    pNetIf->Flags.bNewTcpipEventAvlbl = 1;
    totTcpipEventsCnt++;
#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
    TCPIP_EVENT_LIST_NODE* tNode;

    for(tNode = (TCPIP_EVENT_LIST_NODE*)pNetIf->registeredClients.head; tNode != 0; tNode = tNode->next)
    {
        if((tNode->evMask & event) != 0 )
        {   // trigger event
            (*tNode->handler)(pNetIf, event, tNode->hParam);
        }
    }

#endif  // defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)

}



/*******************************************************************************
  Function:
    void    TCPIP_STACK_TickHandler(SYS_TICK currSysTick)

  Summary:
    Stack tick handler.

  Description:
    This function is called from within the System Tick ISR.
    It provides the Stack tick processing.
    It will call the notification handler registered with SYS_TICK_TimerCreate


  Precondition:
   System Tick should have been initialized
   and the Stack tick handler should have been registered with the SYS_TICK_TimerCreate.

  Parameters:
    currSysTick   - current system tick value at the time of call

  Returns:
    None

  Remarks:
    None
*****************************************************************************/
static void TCPIP_STACK_TickHandler(SYS_TICK currSysTick)
{
        newTcpipTickAvlbl++;
}





static void ProcessTCPIPMacErrorEvents(TCPIP_NET_IF* pNetIf, TCPIP_EVENT activeEvent)
{
    newTcpipErrorEventCnt++;
}





/*********************************************************************
 * Function:        TCPIP_STACK_GetDefaultNet(void)
 *
 * PreCondition:    TCPIP stack should have been initialized by
 *                    TCPIP_STACK_Init()
 *
 * Input:           None
 *
 * Output:          The default net interface for multi-homed hosts
 *
 * Side Effects:    None
 *
 * Note:            Function to dynamically change the default interface
 *                  will be added.
 ********************************************************************/
TCPIP_NET_HANDLE TCPIP_STACK_GetDefaultNet(void)
{
    return tcpipDefIf.defaultNet;
}

// sets the default interface
// returns true if success,
// false if failed (the old interface does not change)
bool TCPIP_STACK_SetDefaultNet(TCPIP_NET_HANDLE netH)
{
    TCPIP_NET_IF* pNewIf = _TCPIPStackHandleToNet(netH);
    if(pNewIf && pNewIf->Flags.bInterfaceEnabled)
    {
        tcpipDefIf.defaultNet = pNewIf;
        return true;
    }

    return false;
}

/*********************************************************************
 * Function:        _TCPIPStackIpAddToNet(IPV4_ADDR* pIpAddress, bool useDefault)
 *
 * PreCondition:    TCPIP stack should have been initialized by
 *                    TCPIP_STACK_Init()
 *
 * Input:           pIpAddress - pointer to an IP address
 *
 *                  useDefault - when no interface is found,
 *                               if true: return the default interface
 *                               else return 0;
 *
 * Output:          Resolves a local IP address to a network interface.
 *
 *
 * Side Effects:    None
 *
 * Note:            None
 ********************************************************************/
TCPIP_NET_IF* _TCPIPStackIpAddToNet(IPV4_ADDR* pIpAddress, bool useDefault)
{
    TCPIP_NET_IF* pNetIf = 0;

    if(pIpAddress && pIpAddress->Val != 0)
    {
        pNetIf = _TCPIPStackNetByAddress(pIpAddress);
    }

    if(pNetIf == 0 && useDefault)
    {
        pNetIf = tcpipDefIf.defaultNet;
    }

    return pNetIf;
}

/*********************************************************************
 * Function:        _TCPIPStackIpAddFromNet(TCPIP_NET_IF* pNetIf, IPV4_ADDR* pIpAddress)
 *
 * PreCondition:    TCPIP stack should have been initialized by
 *                    TCPIP_STACK_Init()
 *
 * Input:           pNetIf  - network interface to check for
 *                            if 0 all interfaces are checked
 *                  pIpAddress - pointer to an IP address
 *
 * Output:          Resolves a local IP address to a network interface
 *                  to which it belongs
 *
 * Side Effects:    None
 *
 * Note:            None
 ********************************************************************/
TCPIP_NET_IF* _TCPIPStackIpAddFromNet(TCPIP_NET_IF* pNetIf, IPV4_ADDR* pIpAddress)
{
    int netIx;
    TCPIP_NET_IF* pIf;

    if(pIpAddress && pIpAddress->Val != 0)
    {
        for(netIx = 0, pIf = tcpipNetIf ; netIx < tcpip_stack_ctrl_data.nIfs; netIx++, pIf++)
        {
            if(pNetIf == 0 || pIf == pNetIf)
            {
                if(pIf->Flags.bInterfaceEnabled)
                {
                    if(((pIf->netIPAddr.Val & pIpAddress->Val) ^ pIf->netMask.Val) == 0)
                    {
                        return pIf;
                    }
                }
            }
        }
    }

    return 0;
}

/*********************************************************************
 * Function:        _TCPIPStackMacToNet(TCPIP_MAC_HANDLE hMac)
 *
 * PreCondition:    TCPIP stack should have been initialized by
 *                    TCPIP_STACK_Init()
 *
 * Input:           None
 *
 * Output:          Resolves a MAC Id to an tcpipNetIf entry.
 *
 *
 * Side Effects:    None
 *
 * Note:            The tcpipNetIf entries match 1 to 1 the network interfaces
 *                  A more efficient algorithm to find MAC<->tcpipNetIf entry correspondence
 *                  will be eventually added.
 ********************************************************************/
TCPIP_NET_IF* _TCPIPStackMacToNet(TCPIP_MAC_HANDLE hMac)
{
    int netIx;
    TCPIP_NET_IF* pNetIf;

    for(netIx = 0, pNetIf = tcpipNetIf; netIx < tcpip_stack_ctrl_data.nIfs; netIx++, pNetIf++)
    {
        if(pNetIf->hIfMac == hMac)
        {
            return pNetIf;
        }
    }


    return 0;
}

/*********************************************************************
 * Function:        _TCPIPStackMacToNetIx(TCPIP_MAC_HANDLE hMac)
 *
 * PreCondition:    TCPIP stack should have been initialized by
 *                    TCPIP_STACK_Init()
 *
 * Input:           None
 *
 * Output:          Resolves a MAC to an tcpipNetIf entry.
 *
 *
 * Side Effects:    None
 *
 * Note:            The tcpipNetIf entries match 1 to 1 the network interfaces
 *                  A more efficient algorithm to find MAC<->tcpipNetIf entry correspondence
 *                  will be eventually added.
 ********************************************************************/
int _TCPIPStackMacToNetIx(TCPIP_MAC_HANDLE hMac)
{
    int netIx;
    TCPIP_NET_IF* pNetIf;

    for(netIx = 0, pNetIf = tcpipNetIf; netIx < tcpip_stack_ctrl_data.nIfs; netIx++, pNetIf++)
    {
        if(pNetIf->hIfMac == hMac)
        {
            return netIx;
        }
    }


    return -1;
}

/*********************************************************************
 * Function:        int TCPIP_STACK_NetworksNo(void)
 *
 * PreCondition:    TCPIP stack should have been initialized by TCPIP_STACK_Init()
 *
 * Input:           None
 *
 * Output:          Number of network interfaces
 *
 * Side Effects:    None
 *
 * Note:            None
 ********************************************************************/
int TCPIP_STACK_NetworksNo(void)
{
    return tcpip_stack_ctrl_data.nIfs;
}

/*********************************************************************
 * Function:        TCPIP_STACK_IxToNet(int netIx)
 *
 * PreCondition:    TCPIP stack should have been initialized by
 *                    TCPIP_STACK_Init()
 *
 * Input:           None
 *
 * Output:          Resolves an index to an tcpipNetIf entry.
 *
 *
 * Side Effects:    None
 *
 * Note:            The tcpipNetIf entries match 1 to 1 the network interfaces
 ********************************************************************/
TCPIP_NET_HANDLE TCPIP_STACK_IxToNet(int netIx)
{
    if(netIx < tcpip_stack_ctrl_data.nIfs)
    {
        return tcpipNetIf + netIx;
    }

    return 0;
}

int  TCPIP_STACK_NetIx(TCPIP_NET_HANDLE hNet)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNet(hNet);
    return _TCPIPStackNetIx(pNetIf);
}

/*********************************************************************
 * Function:        _TCPIPStackMacIdToNet(TCPIP_STACK_MODULE macId)
 *
 * PreCondition:    TCPIP stack should have been initialized by
 *                    TCPIP_STACK_Init()
 *
 * Input:           None
 *
 * Output:          Resolves an MAC id to a Net entry.
 *
 *
 * Side Effects:    None
 *
 * Note:            In multi-homed hosts with multiple
 *                  interfaces of the same type,
 *                  the translation might not be unique.
 *                  The first match is returned!
 *
 ********************************************************************/
TCPIP_NET_IF* _TCPIPStackMacIdToNet(TCPIP_STACK_MODULE macId)
{
    TCPIP_NET_IF* pNetIf;

    if(macId != TCPIP_MODULE_MAC_NONE)
    {
        int netIx;
        for(netIx = 0, pNetIf = tcpipNetIf; netIx < tcpip_stack_ctrl_data.nIfs; netIx++, pNetIf++)
        {
            if(pNetIf->macId == macId)
            {
                return pNetIf;
            }
        }
    }


    return 0;

}

/*********************************************************************
 * Function:        TCPIP_NET_HANDLE TCPIP_STACK_NetHandle(const char* interface)
 *
 * PreCondition:    TCPIP stack should have been initialized by TCPIP_STACK_Init()
 *
 * Input:           interface - The names specified in tcpip_config.h::TCPIP_NETWORK_CONFIG.
 *
 * Output:          Resolves an interface name to a handle.
 *
 * Side Effects:    None
 *
 * Example:            TCPIP_NET_HANDLE hNet = TCPIP_STACK_NetHandle("PIC32INT\r\n");
 *
 * Note:            None
 ********************************************************************/
TCPIP_NET_HANDLE TCPIP_STACK_NetHandle(const char* interface)
{
    return _TCPIPStackMacIdToNet(_TCPIPStackStringToMACId(interface));
}


/*********************************************************************
 * Function:        const char* TCPIP_STACK_NetName(TCPIP_NET_HANDLE netH)
 *
 * PreCondition:    TCPIP stack should have been initialized by TCPIP_STACK_Init()
 *
 * Input:           netH - Interface handle to get the name of.
 *
 * Output:          it returns the name associated to that interface handle
 *                     returns 0 if no such name
 *
 * Side Effects:    None
 *
 * Example:            TCPIP_NET_HANDLE netH = TCPIP_STACK_IxToNet(0);
 *                     const char* netName = TCPIP_STACK_NetName(netH);
 *
 * Note:            None
 ********************************************************************/
const char* TCPIP_STACK_NetName(TCPIP_NET_HANDLE netH)
{
    TCPIP_NET_IF*  pNetIf = _TCPIPStackHandleToNet(netH);

    return pNetIf?_TCPIPStackMACIdToString(pNetIf->macId):0;
}




/*********************************************************************
 * Function:        _TCPIPStackNetByAddress(IPV4_ADDR* pIpAddress)
 *
 * PreCondition:    TCPIP stack should have been initialized by
 *                    TCPIP_STACK_Init()
 *
 * Input:           pointer to an IP address
 *
 * Output:          The network interface pointer to which this ip
 *                  address belongs to.
 *                  NULL if not one of our addresses.
 *
 * Side Effects:    None
 *
 * Note:            A single network interface can support multiple IP addresses.
 *                  For now this feature is not implemented/supported.
 *
 ********************************************************************/
TCPIP_NET_IF* _TCPIPStackNetByAddress(IPV4_ADDR* pIpAddress)
{
    int netIx;
    TCPIP_NET_IF* pIf;

    for(netIx = 0, pIf = tcpipNetIf ; netIx < tcpip_stack_ctrl_data.nIfs; netIx++, pIf++)
    {
        if(pIf->Flags.bInterfaceEnabled && pIf->netIPAddr.Val == pIpAddress->Val)
        {
            return pIf;
        }
    }


    return 0;
}

/*********************************************************************
 * Function:        uint32_t TCPIP_STACK_NetAddress(TCPIP_NET_HANDLE netH)
 *
 * PreCondition:    TCPIP stack should have been initialized by TCPIP_STACK_Init()
 *
 * Input:           netH - Interface handle to get address of.
 *
 * Output:          The IP address of an interface.
 *
 * Side Effects:    None
 *
 * Example:            TCPIP_NET_HANDLE netH = TCPIP_STACK_NetHandle("PIC32INT\r\n");
 *                     uint32_t ipAdd = TCPIP_STACK_NetAddress(netH);
 *
 * Note:            None
 ********************************************************************/
uint32_t TCPIP_STACK_NetAddress(TCPIP_NET_HANDLE netH)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNet(netH);
    return _TCPIPStackNetAddress(pNetIf);
}


#if defined(TCPIP_STACK_USE_IPV6)
IPV6_ADDR_HANDLE TCPIP_STACK_NetIPv6AddressGet(TCPIP_NET_HANDLE netH, IPV6_ADDR_TYPE addType, IPV6_ADDR_STRUCT* pAddStruct, IPV6_ADDR_HANDLE addHandle)
{
    IPV6_ADDR_STRUCT * addrNode;
    IPV6_INTERFACE_CONFIG*  pIpv6Config;

    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNet(netH);
    if(pNetIf == 0 || pNetIf->Flags.bInterfaceEnabled == 0)
    {
        return 0;
    }


    // Note: for both unicast and multicast addresses we start from unicast list
    // that's because we need to construct the solicited node multicas address
    // which is not currently stored in its own list!

    pIpv6Config = TCPIP_IPV6_GetInterfaceConfig(pNetIf);
    if(addHandle == 0)
    {   // start iteration through the list
        addrNode = (IPV6_ADDR_STRUCT *)pIpv6Config->listIpv6UnicastAddresses.head;
    }
    else
    {
        addrNode = ((IPV6_ADDR_STRUCT*)addHandle)->next;
    }

    if(addType == IPV6_ADDR_TYPE_UNICAST)
    {
        if(addrNode && pAddStruct)
        {
            memcpy(pAddStruct, addrNode, sizeof(*addrNode));
            pAddStruct->next = pAddStruct->prev = 0;
        }
        return addrNode;
    }

    if(addType == IPV6_ADDR_TYPE_MULTICAST)
    {
        if(addrNode == 0)
        {
            if(addHandle == 0 || ((IPV6_ADDR_STRUCT*)addHandle)->flags.type == IPV6_ADDR_TYPE_UNICAST)
            {   // either the unicast list is empty or finished the unicast list
                addrNode = (IPV6_ADDR_STRUCT *)pIpv6Config->listIpv6MulticastAddresses.head;
            }
        }

        if(addrNode != 0 && addrNode->flags.type == IPV6_ADDR_TYPE_UNICAST)
        {   // do not report the same solicited node address multiple times
            IPV6_ADDR_STRUCT * unicastHead = (IPV6_ADDR_STRUCT *)pIpv6Config->listIpv6UnicastAddresses.head;
            IPV6_ADDR_STRUCT * currAddress = unicastHead;
            while(currAddress != addrNode)
            {
                if(memcmp(addrNode->address.v + sizeof (IPV6_ADDR) - 3, currAddress->address.v + sizeof (IPV6_ADDR) - 3, 3) == 0)
                {   // address match; skip this one
                    addrNode = addrNode->next;
                    if(addrNode == 0)
                    {   // end of list
                        addrNode = (IPV6_ADDR_STRUCT *)pIpv6Config->listIpv6MulticastAddresses.head;
                        break;
                    }
                    else
                    {   // restart traversal
                        currAddress = unicastHead; 
                    }
                }
                else
                {
                    currAddress = currAddress->next;
                }
            }
        }


        if(addrNode && pAddStruct)
        {
            memcpy(pAddStruct, addrNode, sizeof(*addrNode));
            pAddStruct->next = pAddStruct->prev = 0;
            if(addrNode->flags.type == IPV6_ADDR_TYPE_UNICAST)
            {   // construct the solicited node multicast address
                memcpy(pAddStruct->address.v, IPV6_SOLICITED_NODE_MULTICAST.v, sizeof (IPV6_ADDR) - 3);
                pAddStruct->flags.type = IPV6_ADDR_TYPE_MULTICAST;
            }
        }
        return addrNode;
    }


    // no other address type supported
    return 0;
}
#endif  // defined(TCPIP_STACK_USE_IPV6)




bool TCPIP_STACK_SetNetAddress(TCPIP_NET_HANDLE netH, IPV4_ADDR* ipAddress, IPV4_ADDR* mask, bool setDefault)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNet(netH);
    if(pNetIf)
    {
        _TCPIPStackSetNetAddress(pNetIf, ipAddress, mask, setDefault);
    }

    return pNetIf != 0;
}
bool TCPIP_STACK_SetNetGatewayAddress(TCPIP_NET_HANDLE netH, IPV4_ADDR* ipAddress)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNet(netH);
    _TCPIPStackSetGatewayAddress(pNetIf, ipAddress);

    return pNetIf != 0;
}

uint32_t TCPIP_STACK_NetGatewayAddress(TCPIP_NET_HANDLE netH)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNet(netH);

    if(pNetIf && pNetIf->Flags.bInterfaceEnabled)
    {
        return pNetIf->netGateway.Val;
    }

    return 0;
}

uint32_t TCPIP_STACK_NetPriDNSAddress(TCPIP_NET_HANDLE netH)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNet(netH);

    if(pNetIf && pNetIf->Flags.bInterfaceEnabled)
    {
        return pNetIf->PrimaryDNSServer.Val;
    }

    return 0;
}

uint32_t TCPIP_STACK_NetSecondDNSAddress(TCPIP_NET_HANDLE netH)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNet(netH);

    if(pNetIf && pNetIf->Flags.bInterfaceEnabled)
    {
        return pNetIf->SecondaryDNSServer.Val;
    }

    return 0;
}

bool TCPIP_STACK_SetNetPriDNSAddress(TCPIP_NET_HANDLE netH, IPV4_ADDR* ipAddress)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNet(netH);
    _TCPIPStackSetPriDNSAddress(pNetIf, ipAddress);

    return pNetIf != 0;
}

bool TCPIP_STACK_SetNetSecondDNSAddress(TCPIP_NET_HANDLE netH, IPV4_ADDR* ipAddress)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNet(netH);
    _TCPIPStackSetSecondDNSAddress(pNetIf, ipAddress);

    return pNetIf != 0;
}

/*********************************************************************
 * Function:        uint32_t TCPIP_STACK_NetMask(TCPIP_NET_HANDLE netH)
 *
 * PreCondition:    None
 *
 * Input:           netH - Interface handle to get address of.
 *
 * Output:          if interface enabled then Value of subnet mask
 *                     else 0
 *
 * Example:         TCPIP_NET_HANDLE netH = TCPIP_STACK_NetHandle("PIC32INT\r\n");
 *                     uint32_t subMask = TCPIP_STACK_NetMask(netH);
 *
 * Note:            None
 ********************************************************************/
uint32_t TCPIP_STACK_NetMask(TCPIP_NET_HANDLE netH)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNet(netH);
    return _TCPIPStackNetMask(pNetIf);
}


bool TCPIP_STACK_SetNetMacAddress(TCPIP_NET_HANDLE netH, const MAC_ADDR* pAddr)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNet(netH);
    if(pNetIf)
    {
        memcpy(pNetIf->netMACAddr.v, pAddr->v, sizeof(pNetIf->netMACAddr));
        return true;
    }

    return false;
}

const char* TCPIP_STACK_NetBIOSName(TCPIP_NET_HANDLE netH)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNet(netH);
    if(pNetIf && pNetIf->Flags.bInterfaceEnabled)
    {
        return (const char*)pNetIf->NetBIOSName;
    }

    return 0;

}

bool TCPIP_STACK_SetNetBIOSName(TCPIP_NET_HANDLE netH, const char* biosName)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNet(netH);
    if(pNetIf)
    {
        memcpy(pNetIf->NetBIOSName, biosName, sizeof(pNetIf->NetBIOSName));
        TCPIP_Helper_FormatNetBIOSName(pNetIf->NetBIOSName);
        return true;
    }

    return false;
}

const uint8_t* TCPIP_STACK_NetMacAddress(TCPIP_NET_HANDLE netH)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNet(netH);
    return _TCPIPStackNetMacAddress(pNetIf);
}

bool TCPIP_STACK_SetLocalMasksType(TCPIP_NET_HANDLE netH, TCPIP_LOCAL_MASK_TYPE andType, TCPIP_LOCAL_MASK_TYPE orType)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNet(netH);

    if(pNetIf)
    {
        pNetIf->Flags.localAndType = andType;
        pNetIf->Flags.localOrType = orType;
        return true;
    }

    return false;
}

bool TCPIP_STACK_SetLocalMasks(TCPIP_NET_HANDLE netH, uint32_t andMask, uint32_t orMask)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNet(netH);

    if(pNetIf)
    {
        pNetIf->localAndMask.Val = andMask;
        pNetIf->localOrMask.Val = orMask;
        return true;
    }

    return false;
}

TCPIP_STACK_MODULE  TCPIP_STACK_NetMACId(TCPIP_NET_HANDLE netH)
{
    return _TCPIPStackNetMacId(_TCPIPStackHandleToNet(netH));
}

size_t TCPIP_STACK_ModuleGetConfig(TCPIP_STACK_MODULE modId, void* configBuff, size_t buffSize, size_t* pNeededSize)
{
    if(tcpipNetIf != 0)
    {   // we should be up and running for this

        int modIx;
        const TCPIP_STACK_GET_CONFIG_MODULE_ENTRY*  pCfgEntry = TCPIP_STACK_MODULE_GET_CONFIG_ENTRY_TBL + 0;
        for(modIx = 0; modIx < sizeof(TCPIP_STACK_MODULE_GET_CONFIG_ENTRY_TBL)/sizeof(*TCPIP_STACK_MODULE_GET_CONFIG_ENTRY_TBL); modIx++)
        {
            if(pCfgEntry->moduleId == modId)
            {   // found corresponding module
                return (*pCfgEntry->getConfig)(modId, configBuff, buffSize, pNeededSize);
            }
            pCfgEntry++;
        }
    }

    // not found
    return -1;
}

// all the parameters are returned without checking
// that the interface is enabled or not!
size_t TCPIP_STACK_NetConfigGet(TCPIP_NET_HANDLE netH, void* configStoreBuff, size_t configStoreSize, size_t* pNeededSize)
{
    TCPIP_NET_IF* pNetIf;
    TCPIP_STACK_NET_IF_DCPT* pNetStg;

    if(tcpipNetIf == 0 || (pNetIf = _TCPIPStackHandleToNet(netH))== 0)
    {   // we should be up and running and have a valid IF
        return -1;
    }

    if(pNeededSize == 0 && (configStoreBuff == 0 || configStoreSize == 0 ))
    {   // nothing to do
        return 0;
    }
    // store needed size
    if(pNeededSize)
    {
        *pNeededSize = sizeof(*pNetStg); 
    }

    if(configStoreBuff && configStoreSize >= sizeof(*pNetStg))
    {   // copy all the fields
        pNetStg = (TCPIP_STACK_NET_IF_DCPT*)configStoreBuff;

        // the TCPIP_STACK_NET_IF_DCPT has to be at the very beginning of the pNetIf !!!
        memcpy(pNetStg, &pNetIf->size, sizeof(*pNetStg));
        // update the size field
        pNetStg->size = sizeof(*pNetStg);

        return sizeof(*pNetStg);
    }

    return 0;
}

static void* NetConfigStringToBuffer(void** ppDstBuff, void* pSrcBuff, size_t* pDstSize, size_t* pCurrLen)
{
    size_t  currLen;

    *pCurrLen = (currLen = strlen(pSrcBuff) + 1);

    if(currLen && currLen <= *pDstSize)
    {
        void* pCopy = *ppDstBuff;
        memcpy(*ppDstBuff, pSrcBuff, currLen);
        *ppDstBuff += currLen;
        *pDstSize -= currLen;
        return pCopy;
    }
    else
    {   // stop copying
        *pDstSize = 0;
        return 0;
    }

}

// restores pNetConfig from configBuff
TCPIP_NETWORK_CONFIG*   TCPIP_STACK_NetConfigSet(void* configStoreBuff, void* netConfigBuff, size_t buffSize, size_t* pNeededSize)
{
    TCPIP_NETWORK_CONFIG* pNetConf;            
    TCPIP_STACK_NET_IF_DCPT* pNetStg = (TCPIP_STACK_NET_IF_DCPT*)configStoreBuff;

    if(configStoreBuff == 0 || (pNeededSize == 0 && netConfigBuff == 0 ))
    {   // nothing to do
        return 0;
    }

    // minimum sanity check
    if(pNetStg->size != sizeof(*pNetStg))
    {   // not valid config save?
        return 0;
    }

    if(buffSize < sizeof(*pNetConf))
    {   // not even enough room to start
        return 0;
    }

    char    tempBuff[50 + 1];   // buffer large enough to hold any string in a TCPIP_NETWORK_CONFIG!
    void*   pDstBuff;
    size_t  dstSize;
    size_t  totLen, currLen;
    
    // create at the very beginning of the buffer
    pNetConf = (TCPIP_NETWORK_CONFIG*)netConfigBuff;
    pDstBuff = pNetConf + 1;    // write area
    dstSize = buffSize - sizeof(*pNetConf);
    totLen = 0;
    tempBuff[sizeof(tempBuff) - 1] = '\0';   // always end properly
    
    // get each field
    strncpy(tempBuff, _TCPIPStackMACIdToString(pNetStg->macId), sizeof(tempBuff) - 1);
    pNetConf->interface = NetConfigStringToBuffer(&pDstBuff, tempBuff, &dstSize, &currLen);
    totLen += currLen;

    strncpy(tempBuff, (char*)pNetStg->NetBIOSName, sizeof(tempBuff) - 1);
    pNetConf->hostName = NetConfigStringToBuffer(&pDstBuff, tempBuff, &dstSize, &currLen);
    totLen += currLen;

    TCPIP_HELPER_MACAddressToString(&pNetStg->netMACAddr, tempBuff, sizeof(tempBuff) - 1);
    pNetConf->macAddr = NetConfigStringToBuffer(&pDstBuff, tempBuff, &dstSize, &currLen);
    totLen += currLen;

    TCPIP_HELPER_IPAddressToString(&pNetStg->netIPAddr, tempBuff, sizeof(tempBuff) - 1);
    pNetConf->ipAddr = NetConfigStringToBuffer(&pDstBuff, tempBuff, &dstSize, &currLen);
    totLen += currLen;

    TCPIP_HELPER_IPAddressToString(&pNetStg->netMask, tempBuff, sizeof(tempBuff) - 1);
    pNetConf->ipMask = NetConfigStringToBuffer(&pDstBuff, tempBuff, &dstSize, &currLen);
    totLen += currLen;

    TCPIP_HELPER_IPAddressToString(&pNetStg->netGateway, tempBuff, sizeof(tempBuff) - 1);
    pNetConf->gateway = NetConfigStringToBuffer(&pDstBuff, tempBuff, &dstSize, &currLen);
    totLen += currLen;

    TCPIP_HELPER_IPAddressToString(&pNetStg->PrimaryDNSServer, tempBuff, sizeof(tempBuff) - 1);
    pNetConf->priDNS = NetConfigStringToBuffer(&pDstBuff, tempBuff, &dstSize, &currLen);
    totLen += currLen;

    TCPIP_HELPER_IPAddressToString(&pNetStg->SecondaryDNSServer, tempBuff, sizeof(tempBuff) - 1);
    pNetConf->secondDNS = NetConfigStringToBuffer(&pDstBuff, tempBuff, &dstSize, &currLen);
    totLen += currLen;

    strncpy(tempBuff, TCPIP_HELPER_PowerModeToString(pNetStg->Flags.powerMode), sizeof(tempBuff) - 1);
    pNetConf->powerMode = NetConfigStringToBuffer(&pDstBuff, tempBuff, &dstSize, &currLen);
    totLen += currLen;

    // set the flags
    pNetConf->startFlags = 0; 
    if(pNetStg->Flags.bIsDHCPEnabled)
    {
        pNetConf->startFlags |= TCPIP_NETWORK_CONFIG_DHCP_CLIENT_ON; 
    }
    if(pNetStg->Flags.bIsDHCPSrvEnabled)
    {
        pNetConf->startFlags |= TCPIP_NETWORK_CONFIG_DHCP_SERVER_ON; 
    }
    if(pNetStg->Flags.bIsZcllEnabled)
    {
        pNetConf->startFlags |= TCPIP_NETWORK_CONFIG_ZCLL_ON; 
    }

    if(currLen != 0)
    {   // last field succeeded
        return pNetConf;
    }

    return 0;

}

TCPIP_STACK_MODULE _TCPIPStackStringToMACId(const char* str)
{

    if(str)
    {
        int ix;
        const TCPIP_STACK_MODULE_MAC_ENTRY* pEntry = TCPIP_STACK_MODULE_MAC_ENTRY_TBL;

        for(ix = 0; ix < sizeof(TCPIP_STACK_MODULE_MAC_ENTRY_TBL)/sizeof(*TCPIP_STACK_MODULE_MAC_ENTRY_TBL); ix++, pEntry++)
        {
            if(!strcmp(str, pEntry->interfaceName))
            {
                return pEntry->moduleId;
            }
        }
    }

    return TCPIP_MODULE_MAC_NONE;
}

const char* _TCPIPStackMACIdToString(TCPIP_STACK_MODULE moduleId)
{
    int ix;
    const TCPIP_STACK_MODULE_MAC_ENTRY* pEntry = TCPIP_STACK_MODULE_MAC_ENTRY_TBL;

    for(ix = 0; ix < sizeof(TCPIP_STACK_MODULE_MAC_ENTRY_TBL)/sizeof(*TCPIP_STACK_MODULE_MAC_ENTRY_TBL); ix++, pEntry++)
    {
        if(pEntry->moduleId == moduleId)
        {
            return pEntry->interfaceName;
        }
    }

    return 0;
}

// detects if an IP address is a local network
bool _TCPIPStackIsLocalNetwork(TCPIP_NET_IF* pNetIf, IPV4_ADDR destIpAdd)
{
    uint32_t currNetVal, destNetVal;
    uint32_t andMask, orMask;

    switch(pNetIf->Flags.localAndType)
    {
        case TCPIP_LOCAL_MASK_ZERO:
            andMask = 0;
            break;

        case TCPIP_LOCAL_MASK_ONE:
            andMask = 0xffffffff;
            break;

        case TCPIP_LOCAL_MASK_NET:
            andMask = pNetIf->netMask.Val;
            break;

        default:    // TCPIP_LOCAL_MASK_SET
            andMask = pNetIf->localAndMask.Val;
            break;
    }

    switch(pNetIf->Flags.localOrType)
    {
        case TCPIP_LOCAL_MASK_ZERO:
            orMask = 0;
            break;

        case TCPIP_LOCAL_MASK_ONE:
            orMask = 0xffffffff;
            break;

        case TCPIP_LOCAL_MASK_NET:
            orMask = pNetIf->netMask.Val;
            break;

        default:    // TCPIP_LOCAL_MASK_SET
            orMask = pNetIf->localOrMask.Val;
            break;
    }

    currNetVal  = (pNetIf->netIPAddr.Val & andMask) | orMask;
    destNetVal = (destIpAdd.Val & andMask) | orMask;

    return destNetVal == currNetVal; 
}

/*********************************************************************
 * Function:        TCPIP_STACK_NetBcastAddress(TCPIP_NET_HANDLE netH)
 *
 * PreCondition:    TCPIP stack should have been initialized by
 *                    TCPIP_STACK_Init()
 *
 * Input:           interface handle to get address of
 *
 * Output:          The broadcast IP address of an interface.
 *
 *
 * Side Effects:    None
 *
 * Note:           
 ********************************************************************/
uint32_t TCPIP_STACK_NetBcastAddress(TCPIP_NET_HANDLE netH)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNet(netH);
    if(pNetIf && pNetIf->Flags.bInterfaceEnabled)
    {
        return (pNetIf->netIPAddr.Val & pNetIf->netMask.Val) | ~pNetIf->netMask.Val;
    }

    return 0;


}

bool TCPIP_STACK_IsNetUp(TCPIP_NET_HANDLE netH)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNet(netH);
    return _TCPIPStackIsNetUp(pNetIf);
}

bool TCPIP_STACK_IsNetLinked(TCPIP_NET_HANDLE netH)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNet(netH);
    return _TCPIPStackIsNetLinked(pNetIf);
}

TCPIP_STACK_ADDRESS_SERVICE_TYPE _TCPIPStackAddressServiceSelect(TCPIP_NET_IF* pNetIf, TCPIP_NETWORK_CONFIG_FLAGS configFlags)
{
    // clear all the existing address service bits
    pNetIf->Flags.v &= ~TCPIP_STACK_ADDRESS_SERVICE_MASK;

    // Set up the address service on this interface
    // Priority (high to low): DHCPc, ZCLL, DHCPS, static IP address
#if defined(TCPIP_STACK_USE_DHCP_CLIENT)
    if((configFlags & TCPIP_NETWORK_CONFIG_DHCP_CLIENT_ON) != 0 )
    { 
        pNetIf->Flags.bIsDHCPEnabled = 1;
        return TCPIP_STACK_ADDRESS_SERVICE_DHCPC;
    }
#endif  // defined(TCPIP_STACK_USE_DHCP_CLIENT)

#if defined(TCPIP_STACK_USE_ZEROCONF_LINK_LOCAL)
    if((configFlags & TCPIP_NETWORK_CONFIG_ZCLL_ON) != 0 )
    { 
        pNetIf->Flags.bIsZcllEnabled = 1;
        return TCPIP_STACK_ADDRESS_SERVICE_ZCLL;
    }
#endif  // defined(TCPIP_STACK_USE_ZEROCONF_LINK_LOCAL)

#if defined(TCPIP_STACK_USE_DHCP_SERVER)
    if((configFlags & TCPIP_NETWORK_CONFIG_DHCP_SERVER_ON) != 0 )
    { 
        pNetIf->Flags.bIsDHCPSrvEnabled = 1;
        return TCPIP_STACK_ADDRESS_SERVICE_DHCPS;
    }
#endif  // defined(TCPIP_STACK_USE_DHCP_SERVER)

    // couldn't select an address service
    // use default/static
    return TCPIP_STACK_ADDRESS_SERVICE_NONE;

}


bool _TCPIPStackAddressServiceCanStart(TCPIP_NET_IF* pNetIf, TCPIP_STACK_ADDRESS_SERVICE_TYPE adSvcType)
{
    if(pNetIf)
    {   // enable a different address service only if there's not another one running
        // client has to stop a previos service (DHCP, ZCLL, etc.) in order to start another one
        return (pNetIf->Flags.v & TCPIP_STACK_ADDRESS_SERVICE_MASK) == TCPIP_STACK_ADDRESS_SERVICE_NONE;
    }

    return false;
}

void _TCPIPStackAddressServiceEvent(TCPIP_NET_IF* pNetIf, TCPIP_STACK_ADDRESS_SERVICE_TYPE adSvcType,
                                    TCPIP_STACK_ADDRESS_SERVICE_EVENT evType)
{
    typedef bool(*addSvcFnc)(TCPIP_NET_HANDLE hNet);
    addSvcFnc   addFnc;

    if(evType == TCPIP_STACK_ADDRESS_SERVICE_EVENT_CONN_LOST)
    {   // connection loss is considered a temporary event;
        // no need to disable a service
        // since we don't have network connectivity anyway
        return;
    }
    else if(adSvcType == TCPIP_STACK_ADDRESS_SERVICE_DHCPS)
    {   // if DHCP server was stopped/failed
        // we won't start another address service
        // the user will have to take a decision
        return;
    }

    // the DHCPc/ZCLL address service failed/stopped:
    // TCPIP_STACK_ADDRESS_SERVICE_EVENT_RUN_FAIL, TCPIP_STACK_ADDRESS_SERVICE_EVENT_USER_STOP
    //
    // make sure any running service is cleared
    pNetIf->Flags.v &= ~TCPIP_STACK_ADDRESS_SERVICE_MASK;
    addFnc = 0;
    if(adSvcType == TCPIP_STACK_ADDRESS_SERVICE_DHCPC)
    {   // the DHCP client has been stopped or failed
        // if possible we'll select ZCLL
        if((pNetIf->startFlags & TCPIP_NETWORK_CONFIG_ZCLL_ON) != 0)
        {   // OK, we can use ZCLL
            addFnc = ZCLLEnable;
        }
    }
    // else if (adSvcType == TCPIP_STACK_ADDRESS_SERVICE_ZCLL)
    // we'll select the default IP address


    if(addFnc)
    {
        if((*addFnc)(pNetIf) == true)
        {   // success
            return;
        }
    }

    // no other address service or it couldn't be started
    // select the default/static addresses
    _TCPIPStackAddressServiceSetDefault(pNetIf);
}

/*********************************************************************
 * Function:        TCPIP_STACK_VerifyPktIf((TCPIP_NET_IF* pNetIf, IPV4_ADDR* pktDestIP)
 *
 * PreCondition:    TCPIP stack should have been initialized by
 *                    TCPIP_STACK_Init()
 *
 * Input:           pNetIf      - interface receiveing a packet
 *                  pktDestIP   - address of packet destination IP
 *
 * Output:          true if valid packet
 *                  false otherwise.
 *
 *
 * Side Effects:    None
 *
 * Note:           None
 ********************************************************************/
#if defined(TCPIP_IF_MRF24W)
static bool TCPIP_STACK_VerifyPktIf(TCPIP_NET_IF* pNetIf, IPV4_ADDR* pktDestIP)
{
    if(
        _TCPIPStackIsAddressOfNet(pNetIf, pktDestIP) || // unicast to me
        _TCPIPStackIsBcastAddress(pNetIf, pktDestIP) || // net or limited bcast
        TCPIP_HELPER_IsMcastAddress(pktDestIP)                 // multicast
        )
    {
        return true;
    }

    return false;


}

/******************************************************************************
 * Function:        bool TCPIP_STACK_FilterRxPkt(TCPIP_NET_IF* pNetIf, NODE_INFO* pPkt)
 *
 * PreCondition:    None
 *
 * Input:           pNetIf - interface
 *                  pPkt  - packet address to check
 *
 * Output:          true if the packet has to be discarded,
 *                  false if the packet is valid for further processing  
 *
 * Side Effects:    None
 *
 * Overview:        Checks an incoming packet (pPkt) to decide if it needs
 *                  discarded.
 *
 * Note:            When using a WiFi module, filter out all incoming packets that have
 *                  the same source MAC address as our own MAC address.  This is to 
 *                  prevent receiving and passing our own broadcast packets up to other 
 *                  layers and avoid, for example, having our own gratuitous ARPs get 
 *                  answered by ourself.
 *****************************************************************************/
static bool TCPIP_STACK_FilterRxPkt(TCPIP_NET_IF* pNetIf, NODE_INFO* pPkt)
{
    if(memcmp((void*)&pPkt->MACAddr, (void*)&pNetIf->netMACAddr, 6) == 0u)
    {
        return true;    // my own interface address; filter it out!
    }

    return false;
}

#endif  // defined(TCPIP_IF_MRF24W)

/*********************************************************************
 * Function:        bool InitNetConfig(const TCPIP_NETWORK_CONFIG* pUsrConfig, int nNets)
 *
 * PreCondition:    MPFSInit() is already called.
 *
 * Input:           pUsrConfig  - pointer to user configurations
 *                  nNets       - number of networks configurations provided
 *
 * Output:          Write/Read non-volatile config variables.
 *
 * Side Effects:    None
 *
 * Overview:        None
 *
 * Note:            None
 ********************************************************************/
static bool InitNetConfig(const TCPIP_NETWORK_CONFIG* pUsrConfig, int nNets)
{
    int     ix;
    TCPIP_NET_IF* pNetConfig;
#if defined(_TCPIP_STACK_CHECK_STORAGE_VERSION)
    TCPIP_STORAGE_HANDLE hStg;      // tcpip storage handle
    int                  storageIx;
    TCPIP_UINT16_VAL     buildChecksum, stgChecksum;
    int                  stgLabelSize;
    bool                 stgWriteNeeded, stgWriteFail;
#endif  // defined(_TCPIP_STACK_CHECK_STORAGE_VERSION)


    for(ix =0, pNetConfig = tcpipNetIf; ix < nNets; ix++, pNetConfig++, pUsrConfig++)
    {
        if(!LoadDefaultConfig(pUsrConfig, pNetConfig))
        {
            SYS_ERROR(SYS_ERROR_ERROR, "Default Flash Network configuration load failed\r\n");
            return false;
        }
    }

#if defined(_TCPIP_STACK_CHECK_STORAGE_VERSION)
    // see if we have a storage to use
    //
    if(!TCPIP_STORAGE_Init(0) || (hStg = TCPIP_STORAGE_Open(0, true)) == TCPIP_STORAGE_HANDLE_INVALID)
    {
        SYS_ERROR(SYS_ERROR_WARN, "Failed to open the storage\r\n");
        return true;    // continue with the default configuration
    }


    stgWriteNeeded = stgWriteFail = false;
    // calculate the checksum of the current network settings
    // as specified by the current build
    buildChecksum.Val = TCPIP_Helper_CalcIPChecksum((uint8_t*)tcpipNetIf, sizeof(TCPIP_NET_IF)*nNets, 0);
    stgLabelSize = TCPIP_STORAGE_GetLabel(hStg, stgChecksum.v, sizeof(stgChecksum.Val));

    if(stgLabelSize != sizeof(stgChecksum.Val) || buildChecksum.Val != stgChecksum.Val)
    {   // checksum mismatch or different build/label, etc
        stgWriteNeeded = true;
        TCPIP_STORAGE_SetLabel(hStg, buildChecksum.v, sizeof(buildChecksum.Val));
    }

    // OK, valid storage; write/read entries
    for(ix =0, pNetConfig = tcpipNetIf; ix < nNets; ix++, pNetConfig++)
    {
        if(stgWriteNeeded ||  (storageIx = TCPIP_STORAGE_FindEntry(hStg, pNetConfig)) == -1)
        {   // either re-write needed or this interface is missing anyway from storage
            // save this entry
            if(!TCPIP_STORAGE_WriteEntry(hStg, pNetConfig, true))
            {   // storage save failed. abort
                stgWriteFail = true;
                break;
            }
        }
        else
        {   // the entry for this interface exists; load it
            TCPIP_STORAGE_ReadEntryIx(hStg, storageIx, pNetConfig);
            SYS_CONSOLE_PRINT("Stack Configuration loaded from storage: if %d\r\n", ix);
        }
    }


    // check the result
    if(stgWriteFail)
    {
        SYS_ERROR(SYS_ERROR_WARN, "Failed to save entries in the storage\r\n");
    }
    else if(stgWriteNeeded)
    {   // successfully saved for this network interface
        SYS_CONSOLE_MESSAGE("Stack Configuration Storage updated\r\n");
    }
    // If we get down here, it means the EEPROM/Flash has valid contents
    // and either matches the const defaults or previously matched and
    // was run-time reconfigured by the user.  In this case, we shall
    // use the contents loaded from EEPROM/Flash.

    TCPIP_STORAGE_Close(hStg);
#endif  // defined(_TCPIP_STACK_CHECK_STORAGE_VERSION)


    return true;
}


/*********************************************************************
 * Function:        bool LoadDefaultConfig(const TCPIP_NETWORK_CONFIG* pUsrConfig, TCPIP_NET_IF* pNetIf)
 *
 * PreCondition:    None
 *
 * Input:           pUsrConfig  - pointer to configurations to use
 *                  pNetIf     - network interface to default configure
 *
 * Output:          true if the default configuration sucessfully loaded,
 *                  false otherwise
 *
 * Side Effects:    None
 *
 * Overview:        Loads the default values (flash) for the network configuration
 *
 * Note:            None
 ********************************************************************/
static bool LoadDefaultConfig(const TCPIP_NETWORK_CONFIG* pUsrConfig, TCPIP_NET_IF* pNetIf)
{
    TCPIP_STACK_ADDRESS_SERVICE_TYPE startAddService;

    memset(pNetIf, 0, sizeof(*pNetIf));

    if(pUsrConfig->macAddr != 0)
    {
        TCPIP_HELPER_StringToMACAddress(pUsrConfig->macAddr, pNetIf->netMACAddr.v);
    }
    else
    {
        memset(pNetIf->netMACAddr.v, 0, sizeof(pNetIf->netMACAddr.v));
    }

    // store the default addresses
    TCPIP_HELPER_StringToIPAddress(pUsrConfig->ipAddr, &pNetIf->DefaultIPAddr);
    TCPIP_HELPER_StringToIPAddress(pUsrConfig->ipMask, &pNetIf->DefaultMask);

    TCPIP_HELPER_StringToIPAddress(pUsrConfig->gateway, &pNetIf->DefaultGateway);
    TCPIP_HELPER_StringToIPAddress(pUsrConfig->priDNS, &pNetIf->DefaultDNSServer);
    TCPIP_HELPER_StringToIPAddress(pUsrConfig->secondDNS, &pNetIf->DefaultDNSServer2);

    pNetIf->macId = _TCPIPStackStringToMACId(pUsrConfig->interface);
    if(pNetIf->macId == TCPIP_MODULE_MAC_NONE)
    {
        return false;   // no such MAC interface
    }

    // Load the NetBIOS Host Name
    memcpy(pNetIf->NetBIOSName, pUsrConfig->hostName, sizeof(tcpipNetIf[0].NetBIOSName));
    TCPIP_Helper_FormatNetBIOSName(pNetIf->NetBIOSName);

    // store start up flags
    pNetIf->startFlags = pUsrConfig->startFlags;

    // Set up the address service on this interface
    startAddService = _TCPIPStackAddressServiceSelect(pNetIf, pUsrConfig->startFlags);

    if(startAddService == TCPIP_STACK_ADDRESS_SERVICE_NONE)
    {   // couldn't start an address service; use the static values supplied
        _TCPIPStackAddressServiceSetDefault(pNetIf);
    }
    
    if( startAddService != TCPIP_STACK_ADDRESS_SERVICE_DHCPC)
    {   // will use the default DNS server
        pNetIf->PrimaryDNSServer.Val = pNetIf->DefaultDNSServer.Val;
        pNetIf->SecondaryDNSServer.Val = pNetIf->DefaultDNSServer2.Val;
    }
    else
    {   // The DHCPc will update these
        pNetIf->Flags.bIsDNSServerAuto = 1;
    }

    return true;
}

void _TCPIPStackAddressServiceSetDefault(TCPIP_NET_IF* pNetIf)
{
    pNetIf->netIPAddr.Val = pNetIf->DefaultIPAddr.Val;
    pNetIf->netMask.Val = pNetIf->DefaultMask.Val;
    pNetIf->netGateway.Val = pNetIf->DefaultGateway.Val;
}


#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)

// Stack external event notification support

TCPIP_EVENT TCPIP_STACK_GetPendingEvents(const void* h)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNet(h);
    if(pNetIf)
    {
        return MACEventGetPending(pNetIf->hIfMac, TCPIP_MAC_EVGROUP_ALL);
    }
    return TCPIP_EV_NONE;
}

TCPIP_EVENT_HANDLE    TCPIP_STACK_RegisterHandler(TCPIP_NET_HANDLE hNet, TCPIP_EVENT evMask, TCPIP_STACK_EVENT_HANDLER handler, const void* hParam)
{
    TCPIP_EVENT_LIST_NODE* newNode;
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNet(hNet);

    if(pNetIf && handler)
    {
        newNode = (TCPIP_EVENT_LIST_NODE*)TCPIP_NotificationAdd(&pNetIf->registeredClients, tcpip_stack_ctrl_data.memH, sizeof(*newNode));
        if(newNode)
        {
            newNode->handler = handler;
            newNode->hParam = hParam;
            newNode->evMask = evMask;
            newNode->pNetIf = pNetIf;
        }
        return newNode;
    }

    return 0;
}

bool TCPIP_STACK_DeRegisterHandler(TCPIP_EVENT_HANDLE hStack)
{
    TCPIP_EVENT_LIST_NODE* pNode = (TCPIP_EVENT_LIST_NODE*)hStack;
    if(pNode)
    {
        if(TCPIP_NotificationRemove((SGL_LIST_NODE*)pNode, &pNode->pNetIf->registeredClients,tcpip_stack_ctrl_data.memH))
        {
            return true;
        }
    }

    return false;

}


#endif  // defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)

static const TCPIP_STACK_MODULE_CONFIG* TcpipStackFindModuleData(TCPIP_STACK_MODULE moduleId, const TCPIP_STACK_MODULE_CONFIG* pModConfig, int nModules)
{
    while(nModules--)
    {
        if(pModConfig->moduleId == moduleId)
        {
            return pModConfig;
        }
        pModConfig++;
    }

    return 0;
}

static const TCPIP_STACK_MODULE_MAC_ENTRY* TcpipStackFindMacModule(TCPIP_STACK_MODULE moduleId)
{
    int modIx;
    const TCPIP_STACK_MODULE_MAC_ENTRY*  pMacEntry = TCPIP_STACK_MODULE_MAC_ENTRY_TBL + 0;
    for(modIx = 0; modIx < sizeof(TCPIP_STACK_MODULE_MAC_ENTRY_TBL)/sizeof(*TCPIP_STACK_MODULE_MAC_ENTRY_TBL); modIx++)
    {
        if(pMacEntry->moduleId == moduleId)
        {   // found corresponding MAC
            return pMacEntry;
        }
        pMacEntry++;
    }

    return 0;
}


int  _TCPIPStackNetIx(TCPIP_NET_IF* pNetIf)
{
    if(pNetIf)
    {
        return pNetIf->netIfIx;
    }
    return -1;
}


uint32_t  _TCPIPStackNetAddress(TCPIP_NET_IF* pNetIf)
{
    if(pNetIf && pNetIf->Flags.bInterfaceEnabled)
    {
        return pNetIf->netIPAddr.Val;
    }
    return 0;
}

void  _TCPIPStackSetNetAddress(TCPIP_NET_IF* pNetIf, IPV4_ADDR* ipAddress, IPV4_ADDR* mask, bool setDefault)
{
    if(pNetIf)
    {
        if(ipAddress)
        {
            pNetIf->netIPAddr.Val = ipAddress->Val;
            if(setDefault)
            {
                pNetIf->DefaultIPAddr.Val = ipAddress->Val;
            }
        }

        if(mask)
        {
            pNetIf->netMask.Val = mask->Val;
            if(setDefault)
            {
                pNetIf->DefaultMask.Val = mask->Val;
            }
        }
    }
}


uint32_t  _TCPIPStackNetMask(TCPIP_NET_IF* pNetIf)
{
    if(pNetIf && pNetIf->Flags.bInterfaceEnabled)
    {
        return pNetIf->netMask.Val;
    }
    return 0;
}

void  _TCPIPStackSetGatewayAddress(TCPIP_NET_IF* pNetIf, IPV4_ADDR* ipAddress)
{
    if(pNetIf)
    {
        pNetIf->netGateway.Val = ipAddress->Val;
    }
}
void  _TCPIPStackSetPriDNSAddress(TCPIP_NET_IF* pNetIf, IPV4_ADDR* ipAddress)
{
    if(pNetIf)
    {
        pNetIf->PrimaryDNSServer.Val = ipAddress->Val;
    }
}

void  _TCPIPStackSetSecondDNSAddress(TCPIP_NET_IF* pNetIf, IPV4_ADDR* ipAddress)
{
    if(pNetIf)
    {
        pNetIf->SecondaryDNSServer.Val = ipAddress->Val;
    }
}


bool  _TCPIPStackIsAddressOfNet( TCPIP_NET_IF* pNetIf, IPV4_ADDR* pIpAdd)
{
    if(pNetIf && pNetIf->Flags.bInterfaceEnabled)
    {
        return pNetIf->netIPAddr.Val == pIpAdd->Val;
    }
    return false;
}

bool  _TCPIPStackIsNetBcastAddress(TCPIP_NET_IF* pNetIf, IPV4_ADDR* pIpAdd)
{
    if(pNetIf && pNetIf->Flags.bInterfaceEnabled)
    {
       return (pIpAdd->Val == ((pNetIf->netIPAddr.Val & pNetIf->netMask.Val) | ~pNetIf->netMask.Val));
    }
    return false;
}

bool  _TCPIPStackIsBcastAddress(TCPIP_NET_IF* pNetIf, IPV4_ADDR* pIpAdd)
{
    if(pNetIf && pNetIf->Flags.bInterfaceEnabled)
    {
       return (TCPIP_HELPER_IsBcastAddress(pIpAdd) ||  _TCPIPStackIsNetBcastAddress(pNetIf, pIpAdd));
    }
    return false;
}

bool  _TCPIPStackIsNetUp(TCPIP_NET_IF* pNetIf)
{
    if(pNetIf && pNetIf->Flags.bInterfaceEnabled)
    {
        return true;
    }
    return false;
}


bool _TCPIPStackIsNetLinked(TCPIP_NET_IF* pNetIf)
{
    if(pNetIf && pNetIf->Flags.bInterfaceEnabled)
    {
        return pNetIf->linkPrev;
    }

    return false;
}

TCPIP_STACK_MODULE  _TCPIPStackNetMacId(TCPIP_NET_IF* pNetIf)
{
    return pNetIf?pNetIf->macId:TCPIP_MODULE_MAC_NONE;
}


TCPIP_MAC_HANDLE  _TCPIPStackNetToMac(TCPIP_NET_IF* pNetIf)
{
    return pNetIf?pNetIf->hIfMac:0;
}



const uint8_t*  _TCPIPStackNetMacAddress(TCPIP_NET_IF* pNetIf)
{
    if(pNetIf && pNetIf->Flags.bInterfaceEnabled)
    {
        return pNetIf->netMACAddr.v;
    }

    return 0;
}


