/******************************************************************************
 *
 *               Microchip Memory Disk Drive File System
 *
 ******************************************************************************
 * FileName:        int_ramdisk.h
 * Dependencies:    
 * Processor:       PIC24/dsPIC30/dsPIC33
 * Compiler:        C30/C32
 * Company:         Microchip Technology, Inc.
 *
 * Software License Agreement
 *
 * The software supplied herewith by Microchip Technology Incorporated
 * (the 锟紺ompany锟? for its PICmicro锟?Microcontroller is intended and
 * supplied to you, the Company锟絪 customer, for use solely and
 * exclusively on Microchip PICmicro Microcontroller products. The
 * software is owned by the Company and/or its supplier, and is
 * protected under applicable copyright laws. All rights are reserved.
 * Any use in violation of the foregoing restrictions may subject the
 * user to criminal sanctions under applicable laws, as well as to
 * civil liability for the breach of the terms and conditions of this
 * license.
 *
 * THIS SOFTWARE IS PROVIDED IN AN 锟紸S IS锟?CONDITION. NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
*****************************************************************************/


/*************************************************************************/
/*  Note:  This file is included as a template of a header file for      */
/*         a new physical layer. It is designed to go with               */
/*         "FS Phys Interface Template.c"                               */
/*************************************************************************/

//DOM-IGNORE-BEGIN
/********************************************************************
 Change History:
  Rev            Description
  ----           -----------------------
********************************************************************/
//DOM-IGNORE-END

#ifndef _INTERNAL_RAM_H_
#define _INTERNAL_RAM_H_

#include "compiler.h"
#include <stdbool.h>
#include "string.h"
#include "system/fs/fs_defs.h"

#ifndef FALSE
    #define FALSE   0
#endif
#ifndef TRUE
    #define TRUE    !FALSE
#endif

/****************************************************************/
/*                    YOUR CODE HERE                            */
/* Add any defines here                                         */
/****************************************************************/
uint8_t MDD_IntRam_InitIO (void);
uint8_t MDD_IntRam_MediaDetect(void);
MEDIA_INFORMATION * MDD_IntRam_MediaInitialize(void);
uint8_t MDD_IntRam_SectorRead(uint32_t sector_addr, uint8_t* buffer);
uint8_t MDD_IntRam_SectorWrite(uint32_t sector_addr, uint8_t* buffer, uint8_t allowWriteToZero);
uint16_t MDD_IntRam_ReadSectorSize(void);
uint32_t MDD_IntRam_ReadCapacity(void);
uint8_t MDD_IntRam_WriteProtectState(void);

#if !defined(MDD_INTERNAL_RAM_MAX_NUM_FILES_IN_ROOT)
    #define MDD_INTERNAL_RAM_MAX_NUM_FILES_IN_ROOT 16
#endif

//---------------------------------------------------------------------------------------
//The size (in number of sectors) of the desired usable data portion of the MSD volume
//---------------------------------------------------------------------------------------
//Note: Windows 7 appears to require a minimum capacity of at least 13 sectors.
//Note2: Windows will not be able to format a drive if it is too small.  The reason
//for this, is that Windows will try to put a "heavyweight" (comparatively) filesystem
//on the drive, which will consume ~18kB of overhead for the filesystem.  If the total
//drive size is too small to fit the filesystem, then Windows will give an error.
//This also means that formatting the drive will "shrink" the usuable data storage
//area, since the default FAT12 filesystem implemented in the Files.c data tables is very
//lightweight, with very low overhead.
//Note3: It is important to make sure that no part of the MSD volume shares a flash
//erase page with the firmware program memory.  This can be done by using a custom
//modified linker script, or by carefully selecting the starting address and the
//total size of the MSD volume.  See also below code comments.
//Note4: It is also important to make sure that no part of the MSD volume shares
//an erase page with the erase page that contains the microcontroller's configuration
//bits (for microcontrollers that use flash for storing the configuration bits,
//see device datasheet). This can be accomplished by using a modified linker script,
//which protects the flash page with the configuration bits (if applicable), or,
//by carefully choosing the FILES_ADDRESS and MDD_INTERNAL_FLASH_DRIVE_CAPACITY,
//to make sure the MSD volume does extend into the erase page with the configuration
//bits.
#define MDD_INTERNAL_RAM_DRIVE_CAPACITY 64//14

//Note: If only 1 FAT sector is used, assuming 12-bit (1.5 byte) FAT entry size 
//(ex: FAT12 filesystem), then the total FAT entries that can fit in a single 512 
//byte FAT sector is (512 bytes) / (1.5 bytes/entry) = 341 entries.  This allows 
//the FAT table to reference up to 341*512 = ~174kB of space.  Therfore, more 
//FAT sectors are needed if creating an MSD volume bigger than this.
#define MDD_INTERNAL_RAM_NUM_RESERVED_SECTORS 1
#define MDD_INTERNAL_RAM_NUM_VBR_SECTORS 1
#define MDD_INTERNAL_RAM_NUM_FAT_SECTORS 1
#define MDD_INTERNAL_RAM_NUM_ROOT_DIRECTORY_SECTORS ((MDD_INTERNAL_RAM_MAX_NUM_FILES_IN_ROOT+15)/16) //+15 because the compiler truncates
#define MDD_INTERNAL_RAM_OVERHEAD_SECTORS (\
            MDD_INTERNAL_RAM_NUM_RESERVED_SECTORS + \
            MDD_INTERNAL_RAM_NUM_VBR_SECTORS + \
            MDD_INTERNAL_RAM_NUM_ROOT_DIRECTORY_SECTORS + \
            MDD_INTERNAL_RAM_NUM_FAT_SECTORS)

#define MDD_INTERNAL_RAM_TOTAL_DISK_SIZE (\
            MDD_INTERNAL_RAM_OVERHEAD_SECTORS+\
            MDD_INTERNAL_RAM_DRIVE_CAPACITY)
#define MDD_INTERNAL_RAM_PARTITION_SIZE (uint32_t)(MDD_INTERNAL_RAM_TOTAL_DISK_SIZE - 1)  //-1 is to exclude the sector used for the MBR

#define MDD_INTERNAL_RAM_TOTAL_SECTORS MDD_INTERNAL_RAM_TOTAL_DISK_SIZE

#define ERASE_BLOCK_SIZE MEDIA_SECTOR_SIZE
#define WRITE_BLOCK_SIZE MEDIA_SECTOR_SIZE
//---------------------------------------------------------
//Do some build time error checking
//---------------------------------------------------------
#if defined(__C30__)
    #if(MDD_INTERNAL_FLASH_TOTAL_DISK_SIZE % 2)
        #warning "MSD volume overlaps flash erase page with firmware program memory.  Please change your FSconfig.h settings to ensure the MSD volume cannot share an erase page with the firmware."
        //See code comments in FSconfig.h, and adjust the MDD_INTERNAL_FLASH_DRIVE_CAPACITY definition until the warning goes away.
    #endif
#endif

#if (MDD_INTERNAL_RAM_MAX_NUM_FILES_IN_ROOT>64)
    #if defined(__C30__)
        #error "PSV only allows 32KB of memory.  The drive options selected result in more than 32KB of data.  Please reduce the number of root directory entries possible"
    #endif
#endif

#if (MEDIA_SECTOR_SIZE != 512)
    #error "The current implementation of internal flash MDD only supports a media sector size of 512.  Please modify your selected value in the FSconfig.h file."
#endif

#if (MDD_INTERNAL_RAM_MAX_NUM_FILES_IN_ROOT != 16) && \
    (MDD_INTERNAL_RAM_MAX_NUM_FILES_IN_ROOT != 32) && \
    (MDD_INTERNAL_RAM_MAX_NUM_FILES_IN_ROOT != 48) && \
    (MDD_INTERNAL_RAM_MAX_NUM_FILES_IN_ROOT != 64)
    #error "Number of root file entries must be a multiple of 16.  Please adjust the definition in the FSconfig.h file."
#endif

#endif

