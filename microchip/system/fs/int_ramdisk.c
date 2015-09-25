/******************************************************************************
 
                Microchip Memory Disk Drive File System
 
 *****************************************************************************
  FileName:        int_ramdisk.c
  Dependencies:    See includes section.
  Processor:       PIC24/dsPIC33/PIC32 microcontrollers
  Compiler:        MPLAB(R) C30/C32 compilers are supported by this code
  Company:         Microchip Technology, Inc.
 
  Software License Agreement
 
  The software supplied herewith by Microchip Technology Incorporated
  (the �Company�) for its PICmicro� Microcontroller is intended and
  supplied to you, the Company�s customer, for use solely and
  exclusively on Microchip PICmicro Microcontroller products. The
  software is owned by the Company and/or its supplier, and is
  protected under applicable copyright laws. All rights are reserved.
  Any use in violation of the foregoing restrictions may subject the
  user to criminal sanctions under applicable laws, as well as to
  civil liability for the breach of the terms and conditions of this
  license.
 
  THIS SOFTWARE IS PROVIDED IN AN �AS IS� CONDITION. NO WARRANTIES,
  WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
  TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
  PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
  IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
  CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 
*****************************************************************************/
//DOM-IGNORE-BEGIN
/********************************************************************
 Change History:
  Rev    Description

********************************************************************/
//DOM-IGNORE-END

#include "compiler.h"
#include <stdbool.h>
#include "string.h"
#include "system/fs/fs_defs.h"
#include "hardware_config.h"
#include "sys_fs_config.h"

#if defined(MEDIA_STORAGE_MDD_INTERNAL_RAM) && defined(SYS_FS_FILESYSTEM_MDD)

#include "system/fs/int_ramdisk.h"

/******************************************************************************
 * Global Variables
 *****************************************************************************/

static unsigned char __attribute__ ((aligned(4))) RamDiskData[MDD_INTERNAL_RAM_TOTAL_DISK_SIZE*MEDIA_SECTOR_SIZE] = {0};

#define MASTER_BOOT_RECORD_ADDRESS &RamDiskData[0]

static MEDIA_INFORMATION mediaInformation;

/******************************************************************************
 * Prototypes
 *****************************************************************************/

/******************************************************************************
 * Function:        uint8_t MDD_IntRam_MediaDetect(void)
 *
 * PreCondition:    InitIO() function has been executed.
 *
 * Input:           void
 *
 * Output:          TRUE   - Card detected
 *                  FALSE   - No card detected
 *
 * Side Effects:    None
 *
 * Overview:        None
 *
 * Note:            None
 *****************************************************************************/
uint8_t MDD_IntRam_MediaDetect()
{
	return TRUE;
}//end MediaDetect

/******************************************************************************
 * Function:        uint16_t ReadSectorSize(void)
 *
 * PreCondition:    MediaInitialize() is complete
 *
 * Input:           void
 *
 * Output:          uint16_t - size of the sectors for this physical media.
 *
 * Side Effects:    None
 *
 * Overview:        None
 *
 * Note:            None
 *****************************************************************************/
uint16_t MDD_IntRam_ReadSectorSize(void)
{
    return MEDIA_SECTOR_SIZE;
}

/******************************************************************************
 * Function:        uint32_t ReadCapacity(void)
 *
 * PreCondition:    MediaInitialize() is complete
 *
 * Input:           void
 *
 * Output:          uint32_t - size of the "disk" - 1 (in terms of sector count).
 *                  Ex: In other words, this function returns the last valid 
 *                  LBA address that may be read/written to.
 *
 * Side Effects:    None
 *
 * Overview:        None
 *
 * Note:            None
 *****************************************************************************/
uint32_t MDD_IntRam_ReadCapacity(void)
{
    //The SCSI READ_CAPACITY command wants to know the last valid LBA address 
    //that the host is allowed to read or write to.  Since LBA addresses start
    //at and include 0, a return value of 0 from this function would mean the 
    //host is allowed to read and write the LBA == 0x00000000, which would be 
    //1 sector worth of capacity.
    //Therefore, the last valid LBA that the host may access is 
    //MDD_INTERNAL_FLASH_TOTAL_DISK_SIZE - 1.
        
    return (MDD_INTERNAL_RAM_TOTAL_DISK_SIZE - 1);
}

/******************************************************************************
 * Function:        uint8_t MDD_IntRam_InitIO(void)
 *
 * PreCondition:    None
 *
 * Input:           void
 *
 * Output:          TRUE   - Card initialized
 *                  FALSE   - Card not initialized
 *
 * Side Effects:    None
 *
 * Overview:        None
 *
 * Note:            None
 *****************************************************************************/
uint8_t MDD_IntRam_InitIO (void)
{
    return  TRUE;
}

/******************************************************************************
 * Function:        uint8_t MDD_IntRam_MediaInitialize(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          Returns a pointer to a MEDIA_INFORMATION structure
 *
 * Overview:        MediaInitialize initializes the media card and supporting variables.
 *
 * Note:            None
 *****************************************************************************/
MEDIA_INFORMATION * MDD_IntRam_MediaInitialize(void)
{
    mediaInformation.validityFlags.bits.sectorSize = TRUE;
    mediaInformation.sectorSize = MEDIA_SECTOR_SIZE;
    
	mediaInformation.errorCode = MEDIA_NO_ERROR;
	return &mediaInformation;
}//end MediaInitialize


/******************************************************************************
 * Function:        uint8_t MDD_IntRam_SectorRead(uint32_t sector_addr, uint8_t *buffer)
 *
 * PreCondition:    None
 *
 * Input:           sector_addr - Sector address, each sector contains 512-byte
 *                  buffer      - Buffer where data will be stored, see
 *                                'ram_acs.h' for 'block' definition.
 *                                'Block' is dependent on whether internal or
 *                                external memory is used
 *
 * Output:          Returns TRUE if read successful, false otherwise
 *
 * Side Effects:    None
 *
 * Overview:        SectorRead reads 512 bytes of data from the card starting
 *                  at the sector address specified by sector_addr and stores
 *                  them in the location pointed to by 'buffer'.
 *
 * Note:            The device expects the address field in the command packet
 *                  to be byte address. Therefore the sector_addr must first
 *                  be converted to byte address. This is accomplished by
 *                  shifting the address left 9 times.
 *****************************************************************************/

uint8_t MDD_IntRam_SectorRead(uint32_t sector_addr, uint8_t* buffer)
{
    //Error check.  Make sure the host is trying to read from a legitimate
    //address, which corresponds to the MSD volume (and not some other program
    //memory region beyond the end of the MSD volume).
    if(sector_addr >= MDD_INTERNAL_RAM_TOTAL_DISK_SIZE)
    {
        return FALSE;
    }   
    
    //Read a sector worth of data, and copy it to the specified RAM "buffer".
    memcpy
    (
        (void*)buffer,
        (void*)(MASTER_BOOT_RECORD_ADDRESS + (sector_addr * MEDIA_SECTOR_SIZE)),
        MEDIA_SECTOR_SIZE
    );

    return TRUE;
}//end SectorRead


/******************************************************************************
 * Function:        uint8_t MDD_IntRam_SectorWrite(uint32_t sector_addr, uint8_t *buffer, uint8_t allowWriteToZero)
 *
 * PreCondition:    None
 *
 * Input:           sector_addr - Sector address, each sector contains 512-byte
 *                  buffer      - Buffer where data will be read
 *                  allowWriteToZero - If true, writes to the MBR will be valid
 *
 * Output:          Returns TRUE if write successful, FALSE otherwise
 *
 * Side Effects:    None
 *
 * Overview:        SectorWrite sends 512 bytes of data from the location
 *                  pointed to by 'buffer' to the card starting
 *                  at the sector address specified by sector_addr.
 *
 * Note:            The sample device expects the address field in the command packet
 *                  to be byte address. Therefore the sector_addr must first
 *                  be converted to byte address. This is accomplished by
 *                  shifting the address left 9 times.
 *****************************************************************************/

uint8_t MDD_IntRam_SectorWrite(uint32_t sector_addr, uint8_t* buffer, uint8_t allowWriteToZero)
{
        uint8_t* dest;
        
        //First, error check the resulting address, to make sure the MSD host isn't trying 
        //to erase/program illegal LBAs that are not part of the designated MSD volume space.
        if(sector_addr >= MDD_INTERNAL_RAM_TOTAL_DISK_SIZE)
        {
            return FALSE;
        }  

        dest = (uint8_t*)(MASTER_BOOT_RECORD_ADDRESS + (sector_addr * MEDIA_SECTOR_SIZE));

        memcpy((void *)dest, (void *)buffer, MEDIA_SECTOR_SIZE);
  
    	return TRUE;
} //end SectorWrite


/******************************************************************************
 * Function:        uint8_t WriteProtectState(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          uint8_t    - Returns the status of the "write enabled" pin
 *
 * Side Effects:    None
 *
 * Overview:        Determines if the card is write-protected
 *
 * Note:            No write protect for internal RAM Disk
 *****************************************************************************/

uint8_t MDD_IntRam_WriteProtectState(void)
{
    return FALSE;
}

#endif
