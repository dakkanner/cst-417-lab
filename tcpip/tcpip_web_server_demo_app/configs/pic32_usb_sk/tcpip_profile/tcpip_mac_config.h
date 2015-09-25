/*******************************************************************************
  MAC Configuration file

  Summary:
    configuration file
    
  Description:
    This file contains the MAC module configuration options
    
*******************************************************************************/

/*******************************************************************************
FileName:   tcpip_mac_config.h
Copyright © 2011 released Microchip Technology Inc.  All rights
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

#ifndef _MAC_CONFIG_H_
#define _MAC_CONFIG_H_

// =======================================================================
//   TCPIP_MODULE_MAC_PIC32INT - PIC32MX7XX/6XX MAC Layer Options
//   If not using a PIC32MX7XX/6XX device with internal MAC, ignore this section.
// =======================================================================

// set to 1 if you need to config the link to specific following parameters
// otherwise the default connection will be attempted
// depending on the selected PHY
#define	ETH_CFG_LINK		0		

// use auto negotiation
#define	ETH_CFG_AUTO		1

// use/advertise 10 Mbps capability
#define	ETH_CFG_10		1

// use/advertise 100 Mbps capability
#define	ETH_CFG_100		1

// use/advertise half duplex capability
#define	ETH_CFG_HDUPLEX		1

// use/advertise full duplex capability
#define	ETH_CFG_FDUPLEX		1

// use/advertise auto MDIX capability
#define	ETH_CFG_AUTO_MDIX	1

// use swapped MDIX. else normal MDIX
#define	ETH_CFG_SWAP_MDIX	1


// MAC Configuration parameters
// Note: These values are used as defaults
// The actual values passed in network_config.h
// take precedence!

// number of the TX descriptors to be created
#define EMAC_TX_DESCRIPTORS	2

// maximum size of an TX packet
// no TX packet should be larger than this limit
#define EMAC_TX_BUFF_SIZE       1500
#if defined(WF_CS_TRIS)
    #define MAX_PACKET_SIZE     (1514ul)
#endif

// number of the RX descriptors and RX buffers to be created
#define EMAC_RX_DESCRIPTORS	6

// size of a RX buffer. should be multiple of 16
// this is the size of all receive buffers processed by the ETHC
// The size should be enough to accomodate any network received packet
// If the packets are larger, they will have to take multiple RX buffers
// The current implementation does not handle this situation right now and 
// the packet is discarded.
#define	EMAC_RX_BUFF_SIZE	1536

/*
 * Reference only: TCPIP_MODULE_MAC_PIC32INT module configuration structure
 *
typedef struct
{
    int             txBuffSize;     // size of the corresponding TX buffer
    int             nTxDescriptors; // number of TX descriptors
    int             rxBuffSize;     // size of the corresponding RX buffer
    int             nRxDescriptors; // number of RX descriptors
}TCPIP_MODULE_MAC_PIC32INT_CONFIG;
*/

// This is a template of how the TCPIP_MODULE_MAC_PIC32INT module
// should be initialized and the parameters that it needs.
#ifdef TCPIP_STACK_MODULE_CONFIGURATION
const TCPIP_MODULE_MAC_PIC32INT_CONFIG macPIC32INTConfigData = 
{
    EMAC_TX_BUFF_SIZE,      // txBuffSize
    EMAC_TX_DESCRIPTORS,    // nTxDescriptors
    EMAC_RX_BUFF_SIZE,      // rxBuffSize
    EMAC_RX_DESCRIPTORS,    // nRxDescriptors
};
#else
extern const TCPIP_MODULE_MAC_PIC32INT_CONFIG macPIC32INTConfigData;
#endif  // TCPIP_STACK_MODULE_CONFIGURATION




// =======================================================================
//   TCPIP_MODULE_MAC_ENCJ60 - MAC Layer Options
//   If not using an external ENC28J60 MAC, ignore this section.
// =======================================================================

/*
 * Reference only: TCPIP_MODULE_MAC_ENCJ60 module configuration structure
 *
typedef struct
{
}TCPIP_MODULE_MAC_ENCJ60_CONFIG;
*/

// This is a template of how the TCPIP_MODULE_MAC_ENCJ60 module
// should be initialized and the parameters that it needs.
#ifdef TCPIP_STACK_MODULE_CONFIGURATION
const TCPIP_MODULE_MAC_ENCJ60_CONFIG macENCJ60ConfigData = 
{
};
#else
extern const TCPIP_MODULE_MAC_ENCJ60_CONFIG macENCJ60ConfigData;
#endif  // TCPIP_STACK_MODULE_CONFIGURATION

// =======================================================================
//   TCPIP_MODULE_MAC_ENCJ600 - MAC Layer Options
//   If not using an external ENC24J600 MAC, ignore this section.
// =======================================================================

/*
 * Reference only: TCPIP_MODULE_MAC_ENCJ600 module configuration structure
 *
typedef struct
{
}TCPIP_MODULE_MAC_ENCJ600_CONFIG;
*/

// This is a template of how the TCPIP_MODULE_MAC_ENCJ600 module
// should be initialized and the parameters that it needs.
#ifdef TCPIP_STACK_MODULE_CONFIGURATION
const TCPIP_MODULE_MAC_ENCJ600_CONFIG macENCJ600ConfigData = 
{
};
#else
extern const TCPIP_MODULE_MAC_ENCJ600_CONFIG macENCJ600ConfigData;
#endif  // TCPIP_STACK_MODULE_CONFIGURATION



// =======================================================================
//   TCPIP_MODULE_MAC_97J60 - MAC Layer Options
//   If not using an external 97J60 MAC, ignore this section.
// =======================================================================

/*
 * Reference only: TCPIP_MODULE_MAC_97J60 module configuration structure
 *
typedef struct
{
}TCPIP_MODULE_MAC_97J60_CONFIG;
*/

// This is a template of how the TCPIP_MODULE_MAC_97J60 module
// should be initialized and the parameters that it needs.
#ifdef TCPIP_STACK_MODULE_CONFIGURATION
const TCPIP_MODULE_MAC_97J60_CONFIG mac97J60ConfigData = 
{
};
#else
extern const TCPIP_MODULE_MAC_97J60_CONFIG mac97J60ConfigData;
#endif  // TCPIP_STACK_MODULE_CONFIGURATION


// =======================================================================
//   TCPIP_MODULE_MAC_MRF24W - MAC Layer Options
//   If not using an external MRF24W Wi-Fi MAC, ignore this section.
// =======================================================================

/*
 * Reference only: TCPIP_MODULE_MAC_MRF24W module configuration structure
 *
typedef struct
{
}TCPIP_MODULE_MAC_MRF24W_CONFIG;
*/

// This is a template of how the TCPIP_MODULE_MAC_MRF24W module
// should be initialized and the parameters that it needs.
#ifdef TCPIP_STACK_MODULE_CONFIGURATION
const TCPIP_MODULE_MAC_MRF24W_CONFIG macMRF24WConfigData = 
{
};
#else
extern const TCPIP_MODULE_MAC_MRF24W_CONFIG macMRF24WConfigData;
#endif  // TCPIP_STACK_MODULE_CONFIGURATION




#endif  // _MAC_CONFIG_H_


