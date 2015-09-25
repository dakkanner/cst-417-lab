/*******************************************************************************
  Microchip TCP/IP Stack Definitions

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  tcpip_manager.h 
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

#ifndef __TCPIP_MANAGER_H_
#define __TCPIP_MANAGER_H_


// a network interface handle
typedef const void* TCPIP_NET_HANDLE;

// types of masks used in the detection of a local/nonlocal network
typedef enum
{
    TCPIP_LOCAL_MASK_ZERO,      // use an all zero network mask
    TCPIP_LOCAL_MASK_ONE,       // use an all ones network mask
    TCPIP_LOCAL_MASK_NET,       // use the current network mask
    TCPIP_LOCAL_MASK_SET,       // use the set value for the network mask
                                // there  are operations available that set
                                // the local/nonlocal detection network mask
}TCPIP_LOCAL_MASK_TYPE;

/*********************************************************************
 * Function:        bool TCPIP_STACK_Init(const TCPIP_NETWORK_CONFIG* pNetConf, int nNets, TCPIP_STACK_MODULE_CONFIG* pModConfig, int nModules)
 *
 * PreCondition:    None 
 *
 * Input:           pNetConf  	- pointer to an array of TCPIP_NETWORK_CONFIG to support
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
 ********************************************************************/
bool                TCPIP_STACK_Init(const TCPIP_NETWORK_CONFIG* pNetConf, int nNets, const TCPIP_STACK_MODULE_CONFIG* pModConfig, int nModules);

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
void                TCPIP_STACK_DeInit(void);

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
void                TCPIP_STACK_Task(void);

//interface access functions

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
int                 TCPIP_STACK_NetworksNo(void);

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
 * Example:			TCPIP_NET_HANDLE hNet = TCPIP_STACK_NetHandle("PIC32INT");
 *
 * Note:            None
 ********************************************************************/
TCPIP_NET_HANDLE    TCPIP_STACK_NetHandle(const char* interface);

/*********************************************************************
 * Function:        const char* TCPIP_STACK_NetName(TCPIP_NET_HANDLE netH)
 *
 * PreCondition:    TCPIP stack should have been initialized by TCPIP_STACK_Init()
 *
 * Input:           netH - Interface handle to get the name of.
 *
 * Output:          it returns the name associated to that interface handle 
 * 					returns 0 if no such name
 *               
 * Side Effects:    None
 *
 * Example:			TCPIP_NET_HANDLE netH = TCPIP_STACK_IxToNet(0);
 * 					const char* netName = TCPIP_STACK_NetName(netH);
 *
 * Note:            None
 ********************************************************************/
const char*         TCPIP_STACK_NetName(TCPIP_NET_HANDLE netH);

/*********************************************************************
 * Function:        uint32_t TCPIP_STACK_NetAddress(TCPIP_NET_HANDLE netH)
 *
 * PreCondition:    TCPIP stack should have been initialized by TCPIP_STACK_Init()
 *
 * Input:           netH - Interface handle to get address of.
 *
 * Output:          if interface is enabled then it returns the IP address of that interface 
 * 					else return 0
 *               
 * Side Effects:    None
 *
 * Example:			TCPIP_NET_HANDLE netH = TCPIP_STACK_NetHandle("PIC32INT");
 * 					uint32_t ipAdd = TCPIP_STACK_NetAddress(netH);
 *
 * Note:            None
 ********************************************************************/
uint32_t            TCPIP_STACK_NetAddress(TCPIP_NET_HANDLE netH);


/*********************************************************************
 * Function:        uint32_t TCPIP_STACK_NetGatewayAddress(TCPIP_NET_HANDLE netH)
 *
 * PreCondition:    TCPIP stack should have been initialized by TCPIP_STACK_Init()
 *
 * Input:           netH - Interface handle to get address of.
 *
 * Output:          if interface is enabled then it returns the gateway address of that interface 
 * 					else return 0
 *               
 * Side Effects:    None
 *
 * Example:			TCPIP_NET_HANDLE netH = TCPIP_STACK_NetHandle("PIC32INT");
 * 					uint32_t ipAdd = TCPIP_STACK_NetGatewayAddress(netH);
 *
 * Note:            None
 ********************************************************************/
uint32_t            TCPIP_STACK_NetGatewayAddress(TCPIP_NET_HANDLE netH);


/*********************************************************************
 * Function:        uint32_t TCPIP_STACK_NetPriDNSAddress(TCPIP_NET_HANDLE netH)
 *
 * PreCondition:    TCPIP stack should have been initialized by TCPIP_STACK_Init()
 *
 * Input:           netH - Interface handle to get the DNS address of.
 *
 * Output:          the primary DNS address if succes
 *                  false if not such interface or interface is down
 *               
 * Side Effects:    None
 *
 * Example:			TCPIP_NET_HANDLE netH = TCPIP_STACK_NetHandle("PIC32INT");
 * 					myIPAddress = TCPIP_STACK_NetPriDNSAddress(netH);
 *
 * Note:            None
 ********************************************************************/
uint32_t            TCPIP_STACK_NetPriDNSAddress(TCPIP_NET_HANDLE netH);


/*********************************************************************
 * Function:        uint32_t TCPIP_STACK_NetSecondDNSAddress(TCPIP_NET_HANDLE netH)
 *
 * PreCondition:    TCPIP stack should have been initialized by TCPIP_STACK_Init()
 *
 * Input:           netH - Interface handle to get the DNS address of.
 *
 * Output:          the secondary DNS address if succes
 *                  false if not such interface or interface is down
 *               
 * Side Effects:    None
 *
 * Example:			TCPIP_NET_HANDLE netH = TCPIP_STACK_NetHandle("PIC32INT");
 * 					myIPAddress = TCPIP_STACK_NetSecondDNSAddress(netH);
 *
 * Note:            None
 ********************************************************************/
uint32_t            TCPIP_STACK_NetSecondDNSAddress(TCPIP_NET_HANDLE netH);


/*********************************************************************
 * Function:        uint32_t TCPIP_STACK_NetMask(TCPIP_NET_HANDLE netH)
 *
 * PreCondition:    None
 *
 * Input:           netH - Interface handle to get mask of.
 *
 * Output:          if interface is enabled then it returns the IP address mask of that interface 
 * 					else return 0
 *
 * Example:         TCPIP_NET_HANDLE netH = TCPIP_STACK_NetHandle("PIC32INT");
 * 					uint32_t subMask = TCPIP_STACK_NetMask(netH);
 *
 * Note:            None
 ********************************************************************/
uint32_t            TCPIP_STACK_NetMask(TCPIP_NET_HANDLE netH);


/*********************************************************************
 * Function:        const char* TCPIP_STACK_NetBIOSName(TCPIP_NET_HANDLE netH)
 *
 * PreCondition:    None
 *
 * Input:           netH - Interface handle to get name of.
 *
 * Output:          if interface is enabled then it returns the NetBIOS name of that interface 
 * 					else return 0
 *
 * Example:         TCPIP_NET_HANDLE netH = TCPIP_STACK_NetHandle("PIC32INT");
 * 					const char* biosName = TCPIP_STACK_NetBIOSName(netH);
 *
 * Note:            None
 ********************************************************************/
const char*         TCPIP_STACK_NetBIOSName(TCPIP_NET_HANDLE netH);

/*********************************************************************
 * Function:        const MAC_ADDR* TCPIP_STACK_NetMacAddress(TCPIP_NET_HANDLE netH)
 *
 * PreCondition:    None
 *
 * Input:           netH - Interface handle to get the address of.
 *
 * Output:          if interface is enabled then it returns a constant pointer to the MAC address
 *                  of that interface 
 * 					else return 0
 *
 * Example:         TCPIP_NET_HANDLE netH = TCPIP_STACK_NetHandle("PIC32INT");
 * 					const MAC_ADDR* pAdd = TCPIP_STACK_NetMacAddress(netH);
 *
 * Note:            None
 ********************************************************************/
const uint8_t*     TCPIP_STACK_NetMacAddress(TCPIP_NET_HANDLE netH);


/*********************************************************************
 * Function:        uint32_t TCPIP_STACK_NetBcastAddress(TCPIP_NET_HANDLE netH)
 *
 * PreCondition:    TCPIP stack should have been initialized by TCPIP_STACK_Init()
 *
 * Input:           netH - Interface handle to get address of.
 *
 * Output:          if interface is enabled then it returns the broadcast IP address mask of that interface 
 * 					else return 0
 *
 * Side Effects:    None
 *
 * Note:	   	 	None
 ********************************************************************/
uint32_t            TCPIP_STACK_NetBcastAddress(TCPIP_NET_HANDLE netH);

/*********************************************************************
 * Function:        TCPIP_NET_HANDLE TCPIP_STACK_GetDefaultNet(void)
 *
 * PreCondition:    TCPIP stack should have been initialized by TCPIP_STACK_Init()
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
TCPIP_NET_HANDLE    TCPIP_STACK_GetDefaultNet(void);

/*********************************************************************
 * Function:        bool TCPIP_STACK_SetDefaultNet(TCPIP_NET_HANDLE netH)
 *
 * PreCondition:    None
 *
 * Input:           netH - Interface handle.
 *
 * Output:          true if success
 * 					false if failed (the old interface does not change)
 *
 * Side Effects:    None
 *
 * Note:            sets the default interface
 ********************************************************************/
bool                TCPIP_STACK_SetDefaultNet(TCPIP_NET_HANDLE netH);


/*********************************************************************
 * Function:        TCPIP_NET_HANDLE TCPIP_STACK_IxToNet(int netIx)
 *
 * PreCondition:    TCPIP stack should have been initialized by TCPIP_STACK_Init()
 *
 * Input:           netH - Interface handle.
 *
 * Output:          Resolves an index to an network handle.
 *               
 * Side Effects:    None
 *
 * Note:            None
 ********************************************************************/
TCPIP_NET_HANDLE    TCPIP_STACK_IxToNet(int netIx);

/*********************************************************************
 * Function:        int TCPIP_STACK_NetIx(TCPIP_NET_HANDLE hNet);
 *
 * PreCondition:    None
 *
 * Input:           hNet - Interface handle.
 *
 * Output:          Index of this entry in the stack network handles
 *
 * Side Effects:    None
 *
 * Note:            None
 ********************************************************************/
int                 TCPIP_STACK_NetIx(TCPIP_NET_HANDLE hNet);

/*********************************************************************
 * Function:        bool TCPIP_STACK_IsNetUp(TCPIP_NET_HANDLE hNet);
 *
 * PreCondition:    None
 *
 * Input:           hNet - Interface handle.
 *
 * Output:          true if interface exists and is enabled
 *                  false otherwise
 *
 * Side Effects:    None
 *
 * Note:            None
 ********************************************************************/
bool                TCPIP_STACK_IsNetUp(TCPIP_NET_HANDLE hNet);

/*********************************************************************
 * Function:        bool TCPIP_STACK_IsNetLinked(TCPIP_NET_HANDLE hNet);
 *
 * PreCondition:    None
 *
 * Input:           hNet - Interface handle.
 *
 * Output:          true if interface exists and the corresponding MAC is linked
 *                  false otherwise
 *
 * Side Effects:    None
 *
 * Note:            None
 ********************************************************************/
bool                TCPIP_STACK_IsNetLinked(TCPIP_NET_HANDLE hNet);

/*********************************************************************
 * Function:        bool TCPIP_STACK_NetUp(TCPIP_NET_HANDLE netH, const TCPIP_NETWORK_CONFIG* pUsrConfig)
 *
 * PreCondition:    None
 *
 * Input:           netH 		- Interface handle.
 * 				 	pUsrConfig  - pointer to a TCPIP_NETWORK_CONFIG for the interface initialization
 *
 * Output:          true if success
 *                  false if no such network or an error occurred
 *
 * Side Effects:    None
 *
 * Overview:        This function brings the desired interface up.
 *
 * Note:            None
 ********************************************************************/
bool                TCPIP_STACK_NetUp(TCPIP_NET_HANDLE netH, const TCPIP_NETWORK_CONFIG* pUsrConfig);

/*********************************************************************
 * Function:        bool TCPIP_STACK_NetDown(TCPIP_NET_HANDLE netH)
 *
 * PreCondition:    None
 *
 * Input:           netH - Interface handle.
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
bool                TCPIP_STACK_NetDown(TCPIP_NET_HANDLE netH);

/*********************************************************************
 * Function:        bool TCPIP_STACK_SetLocalMasksType(TCPIP_NET_HANDLE netH, TCPIP_LOCAL_MASK_TYPE andType, TCPIP_LOCAL_MASK_TYPE orType)
 *
 * PreCondition:    None
 *
 * Description:     This function sets the types of masks used in the stack decision of
 *                  a destination address being a local or nonlocal address.
 *                  For example, when a TCP connection is made, the advertised MSS
 *                  usually has different values for local versus nonlocal networks.
 *
 *                  In order to decide if a destination IP address (destAdd) is local to
 *                  a network interface (having the netAdd address) the following calculation is performed:
 *                  if( ((destAdd & andMask) | orMask) == ((netAdd & andMask) | orMask))
 *                      then the destination address is considered to be a local address.
 *                  else
 *                      the destination address is nonlocal.
 *
 *                  This function sets the types of masks used for the AND and OR operations, as follows:
 *                      - TCPIP_LOCAL_MASK_ZERO: use the all 0's mask (0x00000000)
 *                      - TCPIP_LOCAL_MASK_ONE: use the all 1's mask (0xffffffff)
 *                      - TCPIP_LOCAL_MASK_NET: use the current network mask
 *                      - TCPIP_LOCAL_MASK_SET: use a mask that's set by the application
 *                          There are operations to set a specific AND and OR masks
 *
 *                  Using different valuse for the AND and OR masks an application can select
 *                  various destination networks to be considered as local/nonlocal:
 *                  - only the own network
 *                  - any network
 *                  - no network
 *                  - specific (range of) networks
 *                  - etc.
 *
 * Input:           netH    - handle of the interface to use
 *                  andType - AND type of mask to use for local network detection 
 *                  orType  - OR type of mask to use for local network detection 
 *
 * Output:          if interface exists then this function will set
 *                  the mask types and return true.
 *                  otherwise it will return false 
 *                  
 * Example:         TCPIP_NET_HANDLE netH = TCPIP_STACK_NetHandle("PIC32INT");
 * 					TCPIP_STACK_SetLocalMasksType(netH, TCPIP_LOCAL_MASK_NET, TCPIP_LOCAL_MASK_ZERO);
 *
 * Note:            The default value for both AND and OR masks is set to
 *                          TCPIP_LOCAL_MASK_ZERO
 *                  so that any destination network is considered to be local!
 *
 ********************************************************************/
bool                TCPIP_STACK_SetLocalMasksType(TCPIP_NET_HANDLE netH, TCPIP_LOCAL_MASK_TYPE andType, TCPIP_LOCAL_MASK_TYPE orType);

/*********************************************************************
 * Function:        bool TCPIP_STACK_SetLocalMasks(TCPIP_NET_HANDLE netH, uint32_t andMask, uint32_t orMask)
 *
 * PreCondition:    None
 *
 * Description:     This function sets the masks used in the stack decision of
 *                  a destination address being a local or nonlocal address.
 *                  These masks will be used when the corresponding mask type is set to
 *                  TCPIP_LOCAL_MASK_SET.
 *
 * Input:           netH    - handle of the interface to use
 *                  andMask - AND mask to use for local network detection, big endian (BE) format 
 *                  orType  - OR mask to use for local network detection , BE format
 *
 * Output:          if interface exists then this function will set
 *                  the masks and return true.
 *                  otherwise it will return false 
 *                  
 * Example:         TCPIP_NET_HANDLE netH = TCPIP_STACK_NetHandle("PIC32INT");
 * 					TCPIP_STACK_SetLocalMasks(netH, 0x0101a0c0, 0x0);
 *
 * Note:            None
 ********************************************************************/
bool                TCPIP_STACK_SetLocalMasks(TCPIP_NET_HANDLE netH, uint32_t andMask, uint32_t orMask);

/*********************************************************************
 * Function:        TCPIP_STACK_MODULE  TCPIP_STACK_NetMACId(TCPIP_NET_HANDLE netH)
 *
 * PreCondition:    None
 *
 * Description:     This function returns the module ID of the MAC that's attached to the
 *                  specified network interface.
 *
 * Input:           netH    - handle of the interface to use
 *
 * Output:          a TCPIP_STACK_MODULE ID that belongs to the MAC of that network interface.
 *                  
 * Example:         TCPIP_NET_HANDLE netH = TCPIP_STACK_NetHandle("PIC32INT");
 * 					TCPIP_STACK_MODULE modId = TCPIP_STACK_NetMACId(netH);
 * 					if(modId == TCPIP_MODULE_MAC_PIC32INT)
 * 					{
 * 					    // an internal PIC32 MAC attached to this interface
 * 					}
 *
 *
 * Note:            None
 ********************************************************************/
TCPIP_STACK_MODULE  TCPIP_STACK_NetMACId(TCPIP_NET_HANDLE netH);

/*********************************************************************
 * Function:        size_t  TCPIP_STACK_ModuleGetConfig(TCPIP_STACK_MODULE modId, void* configBuff, size_t buffSize, size_t* pNeededSize)
 *
 * PreCondition:    None
 *
 * Description:     This function returns the current configuration data of the stack module
 *                  specified by the corresponding module ID.
 *
 * Input:           modId       - the ID that identifies the requested module
 *                  configBuff  - pointer to a buffer that will receive the configuration data
 *                                If this pointer is 0, just the pNeededSize will be updated
 *                  buffSize    - size of the provided buffer
 *                  pNeededSize - pointer to an address to store the number of bytes needed to store this module configuration data
 *                                Can be NULL if not needed.
 *
 *
 * Output:          number of bytes copied to the user buffer:
 *                      -1 if the module ID is invalid
 *                       0 if the configBuff is NULL or buffSize is less than required
 *                      >0 if the call succeeded and the configuration was copied 
 *
 *                  
 * Example:         uint8_t configBuffer[200];
 *                  size_t configSize;
 *                  size_t copiedSize;
 *                  copiedSize = TCPIP_STACK_ModuleGetConfig(TCPIP_MODULE_MAC_MRF24W, configBuffer, sizeof(configBuffer), &configSize);
 *
 *
 * Note:            None
 ********************************************************************/
size_t              TCPIP_STACK_ModuleGetConfig(TCPIP_STACK_MODULE modId, void* configBuff, size_t buffSize, size_t* pNeededSize);

/*********************************************************************
 * Function:        size_t TCPIP_STACK_NetConfigGet(TCPIP_NET_HANDLE netH, void* configStoreBuff, size_t configStoreSize, size_t* pNeededSize)
 *
 * PreCondition:    None
 *
 * Description:     This function dumps the current configuration data of the network interface
 *                  specified by the corresponding network handle into the supplied buffer.
 *
 * Input:           netH        - the handle that identifies the requested interface
 *                  configStoreBuff  - pointer to a buffer that will receive the current configuration data.
 *                                All the data that's needed to restore a TCPIP_NETWORK_CONFIG structure
 *                                is stored in this buffer.
 *                                Can be NULL if only the storage size is needed.
 *                  configStoreSize    - size of the supplied buffer
 *                  pNeededSize - pointer to store the size needed for storage;
 *                                Can be NULL if not needed
 *
 * Output:          -1 if the interface is invalid or the stack is not initialized
 *                   0 if no data is copied (no supplied buffer of buffer too small)
 *                  >0 for success, indicating the amount of data copied. 
 *                  
 * Example:         uint8_t currConfig[100];
 *                  size_t neededSize, result;
 *                  TCPIP_NET_HANDLE hNet = TCPIP_STACK_NetHandle("PIC32INT");
 *                  result = TCPIP_STACK_NetConfigGet(hNet, currConfig, sizeof(currConfig), &neededSize);
 *                  if(result > 0)
 *                  {   
 *                      // store the currConfig to some external storage
 *                  }
 *
 *
 * Note:            The function is a helper for retrieving the network configuration data.
 *                  Its companion function, TCPIP_STACK_NetConfigSet, restores
 *                  the TCPIP_NETWORK_CONFIG from the dump buffer. 
 *
 *                  Currently the data is saved in plain binary format 
 *                  into the supplied buffer.
 *                  However the application must not make use of this assumption
 *                  as it may change in the future releases
 *                  (some compression scheme may be implemented).
 ********************************************************************/
size_t              TCPIP_STACK_NetConfigGet(TCPIP_NET_HANDLE netH, void* configStoreBuff, size_t configStoreSize, size_t* pNeededSize);

/*********************************************************************
 * Function:        TCPIP_NETWORK_CONFIG* TCPIP_STACK_NetConfigSet(void* configStoreBuff, void* netConfigBuff, size_t buffSize, size_t* pNeededSize);
 *
 * PreCondition:    None
 *
 * Description:     This function restores data from a previously dump buffer and updates the supplied interface configuration.
 *                  All the data is recovered and constructed into the netConfigBuff (supposing this buffer is large enough).
 *                  If this operation succeeded, the netConfigBuff can be safely cast to a (TCPIP_NETWORK_CONFIG*).
 *
 *                  The structure of the netConfigBuff is as follows:
 *                  - a TCPIP_NETWORK_CONFIG sturcture is created at the very beginning of the buffer
 *                  - all the needed fields that are part of the TCPIP_NETWORK_CONFIG will be placed in the buffer itself.
 *
 *
 * Input:           configStoreBuff  - pointer to a buffer that received configuration data from a TCPIP_STACK_NetConfigGet call.
 *
 *                  netConfigBuff    - pointer to a buffer that will receive the TCPIP_NETWORK_CONFIG data
 *
 *                  buffSize    - size of the supplied netConfigBuff buffer
 *
 *                  pNeededSize - pointer to store the size needed for storage;
 *                                Can be NULL if not needed
 *
 * Output:          a valid TCPIP_NETWORK_CONFIG pointer (netConfigBuff) if the netConfigBuff is successfully updated
 *                  0 - if the netConfigBuff is not supplied or not big enough
 *
 *                  If supplied, the pNeededSize will be updated with the actual size that's needed for the netConfigBuff
 *                  
 * Example:         uint8_t currConfig[100];
 *                  uint8_t restoreBuff[100];
 *                  size_t neededSize, result;
 *                  TCPIP_NET_HANDLE hNet = TCPIP_STACK_NetHandle("PIC32INT");
 *                  result = TCPIP_STACK_NetConfigGet(hNet, currConfig, sizeof(currConfig), &neededSize);
 *                  if(result > 0)
 *                  {   
 *                      // store the currConfig buffer to some external storage (neededSize bytes needed)
 *
 *                      // later on restore the configuration
 *                      TCPIP_NETWORK_CONFIG* netConfig;
 *                      // extract the network configuration from the previously saved buffer 
 *                      netConfig = TCPIP_STACK_NetConfigSet(currConfig, restoreBuff, sizeof(restoreBuff), neededSize);
 *                      if(netConfig)
 *                      {
 *                          // use this netConfig to initialize a network interface
 *                          TCPIP_STACK_NetUp(hNet, netConfig);
 *                      }
 *                  }
 *
 *
 * Note:            The function is a helper for being able to restore the configuration data.
 *                  Its companion function, TCPIP_STACK_NetConfigGet, saves
 *                  the TCPIP_NETWORK_CONFIG to a dump buffer. 
 *
 ********************************************************************/
TCPIP_NETWORK_CONFIG*   TCPIP_STACK_NetConfigSet(void* configStoreBuff, void* netConfigBuff, size_t buffSize, size_t* pNeededSize);

/*********** IPv6 interface address functions ******************/

/*********************************************************************
 * Function:        IPV6_ADDR_HANDLE TCPIP_STACK_NetIPv6AddressGet(TCPIP_NET_HANDLE netH, IPV6_ADDR_TYPE addType, IPV6_ADDR_STRUCT* pAddStruct, IPV6_ADDR_HANDLE addHandle);
 *
 * PreCondition:    None
 *
 * Description:     This function allows the listing of the IPv6 addresses associated with an interface.
 *
 *
 * Input:           netH       - handle of the interface to retrieve the addresses for
 *
 *                  addType    - type of address to request:
 *                               IPV6_ADDR_TYPE_UNICAST and IPV6_ADDR_TYPE_MULTICAST supported for now
 *
 *                  pAddStruct  - structure provided by the user that will be filled
 *                                with corresponding IPV6_ADDR_STRUCT data
 *
 *                  addHandle - an address handle that allows iteration across multiple IPv6 addresses.
 *                              On the first call: has to be 0; it will begin the listing of the IPv6 addresses
 *                              On subsequent calls: has to be a handle previously returned by a call to this function
 *
 * Output:          - a non NULL  IPV6_ADDR_HANDLE if an valid IPv6 address was found and the pAddStruct structure was filled with data
 *                  - 0 if no other IPv6 exists or if the supplied IPV6_ADDR_HANDLE is invalid
 *
 *                  
 * Example:         IPV6_ADDR_STRUCT currAddr;
 *                  IPV6_ADDR_HANDLE currHandle;
 *                  TCPIP_NET_HANDLE hNet = TCPIP_STACK_NetHandle("PIC32INT");
 *                  char    ipv6AddBuff[44];
 *
 *                  currHandle = 0;
 *                  do
 *                  {
 *                      currHandle = TCPIP_STACK_NetIPv6AddressGet(netH, IPV6_ADDR_TYPE_UNICAST, &currAddr, currHandle);
 *                      if(currHandle)
 *                      {   // have a valid address; display it
 *                          TCPIP_HELPER_IPv6AddressToString(&currAddr.address, ipv6AddBuff, sizeof(ipv6AddBuff));
 *                      }
 *                  }while(currHandle != 0);
 *                      
 *
 *
 * Note:            None
 *
 ********************************************************************/
IPV6_ADDR_HANDLE        TCPIP_STACK_NetIPv6AddressGet(TCPIP_NET_HANDLE netH, IPV6_ADDR_TYPE addType, IPV6_ADDR_STRUCT* pAddStruct, IPV6_ADDR_HANDLE addHandle);


/*********** Set Parameter functions ******************/
/*
 * Important Note:
 * One should use extreme caution when using these functions to change the settings
 * of a running network interface.
 * Changing these parameters at runtime can lead to unexpected behavior
 * or loss of network connectivity.
 * The preferred way to change the parameters for a running interface is to do so 
 * as part of the network configuration passed at the stack initialization.
*/ 



/*********************************************************************
 * Function:        bool TCPIP_STACK_SetNetAddress(TCPIP_NET_HANDLE netH, IPV4_ADDR* ipAddress, IPV4_ADDR* mask, bool setDefault)
 *
 * PreCondition:    TCPIP stack should have been initialized by TCPIP_STACK_Init()
 *
 * Input:           netH - Interface handle to set address of.
 *                  ipAddress - IP address to set (could be NULL to set only the mask)
 *                  mask      - corresponding network mask to set (could be NULL to set only the IP address)
 *                  setDefault  - if true, the interface default address/mask is also set
 *
 * Output:          true if succes
 *                  false if not such interface
 *               
 * Side Effects:    None
 *
 * Example:			TCPIP_NET_HANDLE netH = TCPIP_STACK_NetHandle("PIC32INT");
 * 					TCPIP_STACK_SetNetAddress(netH, &myIPAddress);
 *
 * Note:            See Important Note above!
 *
 *                  This function sets the associated network IP address and/or mask.
 *                  If you're changing the network then it's preferred that you set
 *                  both these values simultaneously to avoid having the stack running
 *                  with a mismatch between its IP address and mask. 
 ********************************************************************/
bool            TCPIP_STACK_SetNetAddress(TCPIP_NET_HANDLE netH, IPV4_ADDR* ipAddress, IPV4_ADDR* mask, bool setDefault);

/*********************************************************************
 * Function:        bool TCPIP_STACK_SetNetGatewayAddress(TCPIP_NET_HANDLE netH, IPV4_ADDR* ipAddress)
 *
 * PreCondition:    TCPIP stack should have been initialized by TCPIP_STACK_Init()
 *
 * Input:           netH - Interface handle to set the gateway address of.
 *                  ipAddress - IP address to set
 *
 * Output:          true if succes
 *                  false if not such interface
 *               
 * Side Effects:    None
 *
 * Example:			TCPIP_NET_HANDLE netH = TCPIP_STACK_NetHandle("PIC32INT");
 * 					TCPIP_STACK_SetNetGatewayAddress(netH, &myIPAddress);
 *
 * Note:            See Important Note above!
 ********************************************************************/
bool            TCPIP_STACK_SetNetGatewayAddress(TCPIP_NET_HANDLE netH, IPV4_ADDR* ipAddress);

/*********************************************************************
 * Function:        bool TCPIP_STACK_SetNetPriDNSAddress(TCPIP_NET_HANDLE netH, IPV4_ADDR* ipAddress)
 *
 * PreCondition:    TCPIP stack should have been initialized by TCPIP_STACK_Init()
 *
 * Input:           netH - Interface handle to set the DNS address of.
 *                  ipAddress - IP address to set
 *
 * Output:          true if succes
 *                  false if not such interface
 *               
 * Side Effects:    None
 *
 * Example:			TCPIP_NET_HANDLE netH = TCPIP_STACK_NetHandle("PIC32INT");
 * 					TCPIP_STACK_SetNetPriDNSAddress(netH, &myIPAddress);
 *
 * Note:            See Important Note above!
 ********************************************************************/
bool            TCPIP_STACK_SetNetPriDNSAddress(TCPIP_NET_HANDLE netH, IPV4_ADDR* ipAddress);

/*********************************************************************
 * Function:        bool TCPIP_STACK_SetNetSecondDNSAddress(TCPIP_NET_HANDLE netH, IPV4_ADDR* ipAddress)
 *
 * PreCondition:    TCPIP stack should have been initialized by TCPIP_STACK_Init()
 *
 * Input:           netH - Interface handle to set the secondary DNS address of.
 *                  ipAddress - IP address to set
 *
 * Output:          true if succes
 *                  false if not such interface
 *               
 * Side Effects:    None
 *
 * Example:			TCPIP_NET_HANDLE netH = TCPIP_STACK_NetHandle("PIC32INT");
 * 					TCPIP_STACK_SetNetSecondDNSAddress(netH, &myIPAddress);
 *
 * Note:            See Important Note above!
 ********************************************************************/
bool            TCPIP_STACK_SetNetSecondDNSAddress(TCPIP_NET_HANDLE netH, IPV4_ADDR* ipAddress);

/*********************************************************************
 * Function:        bool TCPIP_STACK_SetNetBIOSName(TCPIP_NET_HANDLE netH, const char* biosName);
 *
 * PreCondition:    None
 *
 * Input:           netH - Interface handle to set name of.
 *
 * Output:          if interface exists then it sets the NetBIOS name of that interface 
 *                  and returns true
 * 					else return false
 *
 * Example:         TCPIP_NET_HANDLE netH = TCPIP_STACK_NetHandle("PIC32INT");
 * 					TCPIP_STACK_SetNetBIOSName(netH, myBiosName);
 *
 * Note:            See Important Note above!
 ********************************************************************/
bool                TCPIP_STACK_SetNetBIOSName(TCPIP_NET_HANDLE netH, const char* biosName);

/*********************************************************************
 * Function:        bool TCPIP_STACK_SetNetMacAddress(TCPIP_NET_HANDLE netH, const MAC_ADDR* pAddr)
 *
 * PreCondition:    None
 *
 * Input:           netH - Interface handle to set the address of.
 *
 * Output:          if interface is enabled then it returns a constant pointer to the MAC address
 *                  of that interface 
 * 					else return 0
 *
 * Example:         TCPIP_NET_HANDLE netH = TCPIP_STACK_NetHandle("PIC32INT");
 * 					TCPIP_STACK_SetNetMacAddress(netH, &myMacAddress);
 *
 * Note:            See Important Note above!
 ********************************************************************/
bool                TCPIP_STACK_SetNetMacAddress(TCPIP_NET_HANDLE netH, const MAC_ADDR* pAddr);



#endif  // __TCPIP_MANAGER_H_



