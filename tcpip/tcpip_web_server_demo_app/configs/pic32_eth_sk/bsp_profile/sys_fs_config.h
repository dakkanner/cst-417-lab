/*******************************************************************************

  Summary:

  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  sys_fs_config.h
Copyright ?2012 released Microchip Technology Inc.  All rights
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

#ifndef __SYS_FS_CONFIG_H_
#define __SYS_FS_CONFIG_H_


/*****************************************************************************
 * The system MPFS file system module
 * Use to enable the system MPFS file system functionality.
 *****************************************************************************/
#define SYS_FS_FILESYSTEM_MPFS

/*****************************************************************************
 * The system MDD file system module
 * Use to enable the system MDD file system functionality.
 *****************************************************************************/
//#define SYS_FS_FILESYSTEM_MDD

#if defined(SYS_FS_FILESYSTEM_MDD)
// Summary: A macro indicating whether Long File Name is supported
// Description: If this macro is disabled then only 8.3 format file name is enabled.
//              If this macro is enabled then long file names upto 256 characters are
//              supported.
#define	SUPPORT_LFN

// Summary: A macro indicating the maximum number of concurrently open files
// Description: The FS_MAX_FILES_OPEN #define is only applicable when dynamic memory allocation is not used (FS_DYNAMIC_MEM is not defined).
//              This macro defines the maximum number of open files at any given time.  The amount of RAM used by FSFILE objects will
//              be equal to the size of an FSFILE object multipled by this macro value.  This value should be kept as small as possible
//              as dictated by the application.  This will reduce memory usage.
#define FS_MAX_FILES_OPEN 	5


// Summary: A macro defining the size of a sector
// Description: The MEDIA_SECTOR_SIZE macro will define the size of a sector on the FAT file system.  This value must equal 512 bytes,
//              1024 bytes, 2048 bytes, or 4096 bytes.  The value of a sector will usually be 512 bytes.
#define MEDIA_SECTOR_SIZE 		512



/* *******************************************************************************************************/
/************** Compiler options to enable/Disable Features based on user's application ******************/
/* *******************************************************************************************************/


// Summary: A macro to enable/disable file search functions.
// Description: The ALLOW_FILESEARCH definition can be commented out to disable file search functions in the library.  This will
//              prevent the use of the FindFirst and FindNext functions and reduce code size.
#define ALLOW_FILESEARCH

// Summary: A macro to enable/disable write functionality
// Description: The ALLOW_WRITES definition can be commented out to disable all operations that write to the device.  This will
//              greatly reduce code size.
#define ALLOW_WRITES


// Summary: A macro to enable/disable format functionality
// Description: The ALLOW_FORMATS definition can be commented out to disable formatting functionality.  This will prevent the use of
//              the FSformat function.  If formats are enabled, write operations must also be enabled by uncommenting ALLOW_WRITES.
#define ALLOW_FORMATS

// Summary: A macro to enable/disable directory operations.
// Description: The ALLOW_DIRS definition can be commented out to disable all directory functionality.  This will reduce code size.
//              If directories are enabled, write operations must also be enabled by uncommenting ALLOW_WRITES in order to use
//              the FSmkdir or FSrmdir functions.
#define ALLOW_DIRS

// Summary: A macro to enable/disable PIC18 ROM functions.
// Description: The ALLOW_PGMFUNCTIONS definition can be commented out to disable all PIC18 functions that allow the user to pass string
//              arguments in ROM (denoted by the suffix -pgm).  Note that this functionality must be disabled when not using PIC18.
//#define ALLOW_PGMFUNCTIONS

// Summary: A macro to enable/disable the FSfprintf function.
// Description: The ALLOW_FSFPRINTF definition can be commented out to disable the FSfprintf function.  This will save code space.  Note that
//              if FSfprintf is enabled and the PIC18 architecture is used, integer promotions must be enabled in the Project->Build Options
//              menu.  Write operations must be enabled to use FSfprintf.
//#define ALLOW_FSFPRINTF

// Summary: A macro to enable/disable FAT32 support.
// Description: The SUPPORT_FAT32 definition can be commented out to disable support for FAT32 functionality.  This will save a small amount
//              of code space.
#define SUPPORT_FAT32


/**************************************************************************************************/
// Select a method for updating file timestamps
/**************************************************************************************************/

// Summary: A macro to enable RTCC based timestamp generation
// Description: The USEREALTIMECLOCK macro will configure the code to automatically
//              generate timestamp information for files from the RTCC module. The user
//              must enable and configure the RTCC module before creating or modifying
//              files.
#define USEREALTIMECLOCK

// Summary: A macro to enable manual timestamp generation
// Description: The USERDEFINEDCLOCK macro will allow the user to manually set
//              timestamp information using the SetClockVars function. The user will
//              need to set the time variables immediately before creating or closing a
//              file or directory.
//#define USERDEFINEDCLOCK

// Summary: A macro to enable don't-care timestamp generation
// Description: The INCREMENTTIMESTAMP macro will set the create time of a file to a
//              static value and increment it when a file is updated. This timestamp
//              generation method should only be used in applications where file times
//              are not necessary.
//#define INCREMENTTIMESTAMP

#ifndef USEREALTIMECLOCK
    #ifndef USERDEFINEDCLOCK
        #ifndef INCREMENTTIMESTAMP
            #error Please enable USEREALTIMECLOCK, USERDEFINEDCLOCK, or INCREMENTTIMESTAMP
        #endif
    #endif
#endif

// Summary: A macro indicating that FSFILE objects will be allocated dynamically
// Description: The FS_DYNAMIC_MEM macro will cause FSFILE objects to be allocated from a dynamic heap.
// If it is undefined, the file objects will be allocated using a static array.
#define FS_DYNAMIC_MEM


/*
 *******************************************************************************
 *
 *  Define what type of phisic storage we use for MDD
 *  Currently only SD card or internal RAM are 
 * 
 * *****************************************************************************
*/
#define MEDIA_STORAGE_MDD_INTERNAL_RAM
//#define MEDIA_STORAGE_MDD_SPI_SDMMC

// Function definitions
// Associate the physical layer functions with the correct physical layer
#if defined(MEDIA_STORAGE_MDD_SPI_SDMMC)     // SD-SPI.c and .h

    // Description: Function pointer to the Media Initialize Physical Layer function
    #define MDD_MediaInitialize     MDD_SDSPI_MediaInitialize

    // Description: Function pointer to the Media Detect Physical Layer function
    #define MDD_MediaDetect         MDD_SDSPI_MediaDetect

    // Description: Function pointer to the Sector Read Physical Layer function
    #define MDD_SectorRead          MDD_SDSPI_SectorRead

    // Description: Function pointer to the Sector Write Physical Layer function
    #define MDD_SectorWrite         MDD_SDSPI_SectorWrite

    // Description: Function pointer to the I/O Initialization Physical Layer function
    #define MDD_InitIO              MDD_SDSPI_InitIO

    // Description: Function pointer to the Media Shutdown Physical Layer function
    #define MDD_ShutdownMedia       MDD_SDSPI_ShutdownMedia

    // Description: Function pointer to the Write Protect Check Physical Layer function
    #define MDD_WriteProtectState   MDD_SDSPI_WriteProtectState

    // Description: Function pointer to the Read Capacity Physical Layer function
    #define MDD_ReadCapacity        MDD_SDSPI_ReadCapacity

    // Description: Function pointer to the Read Sector Size Physical Layer Function
    #define MDD_ReadSectorSize      MDD_SDSPI_ReadSectorSize

#elif defined(MEDIA_STORAGE_MDD_INTERNAL_RAM)
    // Description: Function pointer to the Media Initialize Physical Layer function
    #define MDD_MediaInitialize     MDD_IntRam_MediaInitialize

    // Description: Function pointer to the Media Detect Physical Layer function
    #define MDD_MediaDetect         MDD_IntRam_MediaDetect

    // Description: Function pointer to the Sector Read Physical Layer function
    #define MDD_SectorRead          MDD_IntRam_SectorRead

    // Description: Function pointer to the Sector Write Physical Layer function
    #define MDD_SectorWrite         MDD_IntRam_SectorWrite

    // Description: Function pointer to the I/O Initialization Physical Layer function
    #define MDD_InitIO              MDD_IntRam_InitIO

    // Description: Function pointer to the Media Shutdown Physical Layer function
    #define MDD_ShutdownMedia       MDD_IntRam_ShutdownMedia

    // Description: Function pointer to the Write Protect Check Physical Layer function
    #define MDD_WriteProtectState   MDD_IntRam_WriteProtectState

    // Description: Function pointer to the Read Capacity Physical Layer function
    #define MDD_ReadCapacity        MDD_IntRam_ReadCapacity

    // Description: Function pointer to the Read Sector Size Physical Layer Function
    #define MDD_ReadSectorSize      MDD_IntRam_ReadSectorSize

#elif defined(MEDIA_STORAGE_MDD_CF_PMP)       // CF-PMP.c and .h

    // Description: Function pointer to the Media Initialize Physical Layer function
    #define MDD_MediaInitialize     MDD_CFPMP_MediaInitialize

    // Description: Function pointer to the Media Detect Physical Layer function
    #define MDD_MediaDetect         MDD_CFPMP_MediaDetect

    // Description: Function pointer to the Sector Read Physical Layer function
    #define MDD_SectorRead          MDD_CFPMP_SectorRead

    // Description: Function pointer to the Sector Write Physical Layer function
    #define MDD_SectorWrite         MDD_CFPMP_SectorWrite

    // Description: Function pointer to the I/O Initialization Physical Layer function
    #define MDD_InitIO              MDD_CFPMP_InitIO

    // Description: Function pointer to the Media Shutdown Physical Layer function
    #define MDD_ShutdownMedia       MDD_CFPMP_ShutdownMedia

    // Description: Function pointer to the Write Protect Check Physical Layer function
    #define MDD_WriteProtectState   MDD_CFPMP_WriteProtectState

    // Description: Function pointer to the CompactFlash Wait Physical Layer function
    #define MDD_CFwait              MDD_CFPMP_CFwait

    // Description: Function pointer to the CompactFlash Write Physical Layer function
    #define MDD_CFwrite             MDD_CFPMP_CFwrite

    // Description: Function pointer to the CompactFlash Read Physical Layer function
    #define MDD_CFread              MDD_CFPMP_CFread

#elif defined(MEDIA_STORAGE_MDD_CF_IO)         // CF-Bit transaction.c and .h

    // Description: Function pointer to the Media Initialize Physical Layer function
    #define MDD_MediaInitialize     MDD_CFBT_MediaInitialize

    // Description: Function pointer to the Media Detect Physical Layer function
    #define MDD_MediaDetect         MDD_CFBT_MediaDetect

    // Description: Function pointer to the Sector Read Physical Layer function
    #define MDD_SectorRead          MDD_CFBT_SectorRead

    // Description: Function pointer to the Sector Write Physical Layer function
    #define MDD_SectorWrite         MDD_CFBT_SectorWrite

    // Description: Function pointer to the I/O Initialization Physical Layer function
    #define MDD_InitIO              MDD_CFBT_InitIO

    // Description: Function pointer to the Media Shutdown Physical Layer function
    #define MDD_ShutdownMedia       MDD_CFBT_ShutdownMedia

    // Description: Function pointer to the Write Protect Check Physical Layer function
    #define MDD_WriteProtectState   MDD_CFBT_WriteProtectState

    // Description: Function pointer to the CompactFlash Wait Physical Layer function
    #define MDD_CFwait              MDD_CFBT_CFwait

    // Description: Function pointer to the CompactFlash Write Physical Layer function
    #define MDD_CFwrite             MDD_CFBT_CFwrite

    // Description: Function pointer to the CompactFlash Read Physical Layer function
    #define MDD_CFread              MDD_CFBT_CFread

#elif defined(MEDIA_STORAGE_MDD_USB_MSD)              // USB host MSD library

    // Description: Function pointer to the Media Initialize Physical Layer function
    #define MDD_MediaInitialize     USBHostMSDSCSIMediaInitialize

    // Description: Function pointer to the Media Detect Physical Layer function
    #define MDD_MediaDetect         USBHostMSDSCSIMediaDetect

    // Description: Function pointer to the Sector Read Physical Layer function
    #define MDD_SectorRead          USBHostMSDSCSISectorRead

    // Description: Function pointer to the Sector Write Physical Layer function
    #define MDD_SectorWrite         USBHostMSDSCSISectorWrite

    // Description: Function pointer to the I/O Initialization Physical Layer function
    #define MDD_InitIO();

    // Description: Function pointer to the Media Shutdown Physical Layer function
    #define MDD_ShutdownMedia       USBHostMSDSCSIMediaReset

    // Description: Function pointer to the Write Protect Check Physical Layer function
    #define MDD_WriteProtectState   USBHostMSDSCSIWriteProtectState

#endif

#endif // SYS_FS_FILESYSTEM_MDD

/*****************************************************************************
 * The file seperator used in directory paths.
 *****************************************************************************/
#define SYS_FS_CHAR '/' // File seperator
#define SYS_FS_STRING "/"

#ifndef SYS_FS_MAX_DIR
#define SYS_FS_MAX_DIR   235 //Maximum length of directory component
#endif
#ifndef SYS_FS_MAX_FNAME
#define SYS_FS_MAX_FNAME 8   //Maximum length of filename component
#endif
#ifndef SYS_FS_MAX_DRIVE
#define SYS_FS_MAX_DRIVE SYS_FS_MAX_FNAME   //Maximum length of a drive, volume name or mount point
#endif
#ifndef SYS_FS_MAX_EXT
#define SYS_FS_MAX_EXT   4   //Maximum length of extension component (including '.')
#endif
#ifndef SYS_FS_MAX_PATH
#define SYS_FS_MAX_PATH 256  //Maximum length of full path (components above not to exceed)
#endif
#ifndef SYS_FS_MAX_DESCRIPTORS
#define SYS_FS_MAX_DESCRIPTORS 1000 // Per file system
#endif

#define SYS_FS_MOUNT_POINT "MyDrive1"
#define SYS_FS_DEFAULT_FS "mpfs2"
#define SYS_FS_DEFAULT_MEDIA DRV_MEDIA_INTERNAL_FLASH

#endif	// __SYS_FS_CONFIG_H_

