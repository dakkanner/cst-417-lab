/*******************************************************************************
Hardware specific definitions

  Summary:
    
  Description:
    - PIC32 Ethernet Starter Kit
    - PIC32MX795F512L
    - Internal 10/100 Ethernet MAC with National DP83848 10/100 PHY
*******************************************************************************/

/*******************************************************************************
FileName: 	hardware_config.h
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

#ifndef __HARDWARE_PROFILE_H
#define __HARDWARE_PROFILE_H

#include <p32xxxx.h>
#include <plib.h>

// Define a macro describing this hardware set up (used in other files)
// PIC32 Ethernet Starter Kit (04-02146) with PIC32MX795F512L processor and National DP83848 10/100 PHY
#define PIC32_ENET_SK_DM320004


#define     BSP_BOARD_XTAL_FREQ		8000000 	// external Xtal, Hz
#define     BSP_SOSC_XTAL_FREQ        32768     // external SOSC Xtal, Hz

// Hardware IO pin mappings

// using the PIC32 internal MAC interface
#define TCPIP_IF_PIC32INT

// External DP83848 PHY configuration
#define	PHY_RMII				// external PHY runs in RMII mode
#define	PHY_CONFIG_ALTERNATE	// alternate configuration used
#define	PHY_ADDRESS			0x1	// the address of the National DP83848 PHY


// Hardware I/O pin mappings
// LEDs
#define LED0_TRIS			(TRISDbits.TRISD0)	// Ref LED1
#define LED0_IO				(LATDbits.LATD0)
#define LED1_TRIS			(TRISDbits.TRISD1)	// Ref LED2
#define LED1_IO				(LATDbits.LATD1)
#define LED2_TRIS			(TRISDbits.TRISD2)	// Ref LED3
#define LED2_IO				(LATDbits.LATD2)
#define LED3_TRIS			(LED2_TRIS)			// No such LED
#define LED3_IO				(LATDbits.LATD6)
#define LED4_TRIS			(LED2_TRIS)			// No such LED
#define LED4_IO				(LATDbits.LATD6)
#define LED5_TRIS			(LED2_TRIS)			// No such LED
#define LED5_IO				(LATDbits.LATD6)
#define LED6_TRIS			(LED2_TRIS)			// No such LED
#define LED6_IO				(LATDbits.LATD6)
#define LED7_TRIS			(LED2_TRIS)			// No such LED
#define LED7_IO				(LATDbits.LATD6)

#define LED_GET()			((uint8_t)LATD & 0x07)
#define LED_PUT(a)			do{LATD = (LATD & 0xFFF8) | ((a)&0x07);}while(0)

// Momentary push buttons
#define BUTTON0_TRIS		(TRISDbits.TRISD6)	// Ref SW1
#define	BUTTON0_IO			(PORTDbits.RD6)
#define BUTTON1_TRIS		(TRISDbits.TRISD7)	// Ref SW2
#define	BUTTON1_IO			(PORTDbits.RD7)
#define BUTTON2_TRIS		(TRISDbits.TRISD13)	// Ref SW3
#define	BUTTON2_IO			(PORTDbits.RD13)
#define BUTTON3_TRIS		(TRISDbits.TRISD13)	// No BUTTON3 on this board
#define	BUTTON3_IO			(1)

// Note, it is not possible to use a MRF24W 802.11 WiFi PICtail 
// Plus card with this starter kit.  The required interrupt signal, among 
// possibly other I/O pins aren't available on the Starter Kit board.

#include "system_config.h"
#include "sys_fs_config.h"

// define what type of storage we use
#define MEDIA_STORAGE_INTERNAL_FLASH

#if defined(MEDIA_STORAGE_MDD_SPI_SDMMC)
    #define SDMMC_SPI_CHN  1
    // Registers for the SPI module you want to use

    // Define the SPI frequency
    #define SD_SPI_FREQUENCY			(20000000)

    #if (SDMMC_SPI_CHN == 1)
    // Description: SD-SPI Chip Select Output bit
    #define SD_CS               LATBbits.LATB1
    // Description: SD-SPI Chip Select TRIS bit
    #define SD_CS_TRIS          TRISBbits.TRISB1

    // Description: SD-SPI Card Detect Input bit
    #define SD_CD               PORTFbits.RF0
    // Description: SD-SPI Card Detect TRIS bit
    #define SD_CD_TRIS          TRISFbits.TRISF0

    // Description: SD-SPI Write Protect Check Input bit
    #define SD_WE               PORTFbits.RF1
    // Description: SD-SPI Write Protect Check TRIS bit
    #define SD_WE_TRIS          TRISFbits.TRISF1
    // Tris pins for SCK/SDI/SDO lines

    // Tris pins for SCK/SDI/SDO lines
    #if defined(__32MX460F512L__) || defined(__32MX795F512L__)
        // Description: The TRIS bit for the SCK pin
        #define SPICLOCK_TRIS            TRISDbits.TRISD10
        // Description: The TRIS bit for the SDI pin
        #define SPIIN_TRIS               TRISCbits.TRISC4
        // Description: The TRIS bit for the SDO pin
        #define SPIOUT_TRIS              TRISDbits.TRISD0
    #else	// example: PIC32MX360F512L
        // Description: The TRIS bit for the SCK pin
        #define SPICLOCK_TRIS            TRISFbits.TRISF6
        // Description: The TRIS bit for the SDI pin
        #define SPIIN_TRIS               TRISFbits.TRISF7
        // Description: The TRIS bit for the SDO pin
        #define SPIOUT_TRIS              TRISFbits.TRISF8
    #endif

    #endif
#endif

#endif // #ifndef HARDWARE_PROFILE_H

