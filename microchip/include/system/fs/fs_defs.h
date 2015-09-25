/******************************************************************************
 *
 *               Microchip Memory Disk Drive File System
 *
 ******************************************************************************
 * FileName:        fs_defs.h
 * Dependencies:    GenericTypeDefs.h
 * Processor:       PIC18/PIC24/dsPIC30/dsPIC33/PIC32
 * Compiler:        C18/C30/C32
 * Company:         Microchip Technology, Inc.
 * Version:         1.2.4
 *
 * Software License Agreement
 *
 * The software supplied herewith by Microchip Technology Incorporated
 * (the “Company”) for its PICmicro® Microcontroller is intended and
 * supplied to you, the Company’s customer, for use solely and
 * exclusively on Microchip PICmicro Microcontroller products. The
 * software is owned by the Company and/or its supplier, and is
 * protected under applicable copyright laws. All rights are reserved.
 * Any use in violation of the foregoing restrictions may subject the
 * user to criminal sanctions under applicable laws, as well as to
 * civil liability for the breach of the terms and conditions of this
 * license.
 *
 * THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
*****************************************************************************/

#ifndef  _FSDEF__H
#define  _FSDEF__H

#include <stdint.h>
#include <stdbool.h>


// Summary: An enumeration used for various error codes.
// Description: The CETYPE enumeration is used to indicate different error conditions during device operation.
typedef enum _CETYPE
{
    CE_GOOD = 0,                    // No error
    CE_ERASE_FAIL,                  // An erase failed
    CE_NOT_PRESENT,                 // No device was present
    CE_NOT_FORMATTED,               // The disk is of an unsupported format
    CE_BAD_PARTITION,               // The boot record is bad
    CE_UNSUPPORTED_FS,              // The file system type is unsupported
    CE_INIT_ERROR,                  // An initialization error has occured
    CE_NOT_INIT,                    // An operation was performed on an uninitialized device
    CE_BAD_SECTOR_READ,             // A bad read of a sector occured
    CE_WRITE_ERROR,                 // Could not write to a sector
    CE_INVALID_CLUSTER,             // Invalid cluster value > maxcls
    CE_FILE_NOT_FOUND,              // Could not find the file on the device
    CE_DIR_NOT_FOUND,               // Could not find the directory
    CE_BAD_FILE,                    // File is corrupted
    CE_DONE,                        // No more files in this directory
    CE_COULD_NOT_GET_CLUSTER,       // Could not load/allocate next cluster in file
    CE_FILENAME_2_LONG,             // A specified file name is too long to use
    CE_FILENAME_EXISTS,             // A specified filename already exists on the device
    CE_INVALID_FILENAME,            // Invalid file name
    CE_DELETE_DIR,                  // The user tried to delete a directory with FSremove
    CE_DIR_FULL,                    // All root dir entry are taken
    CE_DISK_FULL,                   // All clusters in partition are taken
    CE_DIR_NOT_EMPTY,               // This directory is not empty yet, remove files before deleting
    CE_NONSUPPORTED_SIZE,           // The disk is too big to format as FAT16
    CE_WRITE_PROTECTED,             // Card is write protected
    CE_FILENOTOPENED,               // File not opened for the write
    CE_SEEK_ERROR,                  // File location could not be changed successfully
    CE_BADCACHEREAD,                // Bad cache read
    CE_CARDFAT32,                   // FAT 32 - card not supported
    CE_READONLY,                    // The file is read-only
    CE_WRITEONLY,                   // The file is write-only
    CE_INVALID_ARGUMENT,            // Invalid argument
    CE_TOO_MANY_FILES_OPEN,         // Too many files are already open
    CE_UNSUPPORTED_SECTOR_SIZE      // Unsupported sector size
} CETYPE;


// Summary: A macro indicating a dir entry was found
// Description: The FOUND macro indicates that a directory entry was found in the specified position
#define FOUND       0

// Summary: A macro indicating no dir entry was found
// Description: The NOT_FOUND macro indicates that the specified directory entry to load was deleted
#define NOT_FOUND   1

// Summary: A macro indicating that no more files were found
// Description: The NO_MORE macro indicates that there are no more directory entries to search for
#define NO_MORE     2



// Summary: A macro indicating the device is formatted with FAT12
// Description: The FAT12 macro is used to indicate that the file system on the device being accessed is a FAT12 file system.
#define FAT12       1

// Summary: A macro indicating the device is formatted with FAT16
// Description: The FAT16 macro is used to indicate that the file system on the device being accessed is a FAT16 file system.
#define FAT16       2

// Summary: A macro indicating the device is formatted with FAT32
// Description: The FAT32 macro is used to indicate that the file system on the device being accessed is a FAT32 file system.
#define FAT32       3



// Summary: A read-only attribute macro
// Description: A macro for the read-only attribute.  A file with this attribute should not be written to.  Note that this
//              attribute will not actually prevent a write to the file; that functionality is operating-system dependant.  The
//              user should take care not to write to a read-only file.
#define ATTR_READ_ONLY      0x01

// Summary: A hidden attribute macro
// Description: A macro for the hidden attribute.  A file with this attribute may be hidden from the user, depending on the
//              implementation of the operating system.
#define ATTR_HIDDEN         0x02

// Summary: A system attribute macro
// Description: A macro for the system attribute.  A file with this attribute is used by the operating system, and should not be
//              modified.  Note that this attribute will not actually prevent a write to the file.
#define ATTR_SYSTEM         0x04

// Summary: A volume attribute macro
// Description: A macro for the volume attribute.  If the first directory entry in the root directory has the volume attribute set,
//              the device will use the name in that directory entry as the volume name.
#define ATTR_VOLUME         0x08

// Summary: A macro for the attributes for a long-file name entry
// Description: A macro for the long-name attributes.  If a directory entry is used in a long-file name implementation, it will have
//              all four lower bits set.  This indicates that any software that does not support long file names should ignore that
//              entry.
#define ATTR_LONG_NAME      0x0f

// Summary: A directory attribute macro
// Description: A macro for the directory attribute.  If a directory entry has this attribute set, the file it points to is a directory-
//              type file, and will contain directory entries that point to additional directories or files.
#define ATTR_DIRECTORY      0x10

// Summary: An archive attribute macro
// Description: A macro for the archive attribute.  This attribute will indicate to some archiving programs that the file with this
//              attribute needs to be backed up.  Most operating systems create files with the archive attribute set.
#define ATTR_ARCHIVE        0x20

// Summary: A macro for all attributes
// Description: A macro for all attributes.  The search functions in this library require an argument that determines which attributes
//              a file is allowed to have in order to be found.  If ATTR_MASK is specified as this argument, any file may be found, regardless
//              of its attributes.
#define ATTR_MASK           0x3f



// Summary: A macro to indicate an empty FAT entry
// Description: The CLUSTER_EMPTY value is used to indicate that a FAT entry and it's corresponding cluster are available.
#define CLUSTER_EMPTY               0x0000 

// Summary: A macro to indicate the last cluster value for FAT12
// Description: The LAST_CLUSTER_FAT12 macro is used when reading the FAT to indicate that the next FAT12 entry for a file contains
//              the end-of-file value.
#define LAST_CLUSTER_FAT12          0xff8

// Summary: A macro to indicate the last cluster value for FAT16
// Description: The LAST_CLUSTER_FAT16 macro is used when reading the FAT to indicate that the next FAT16 entry for a file contains
//              the end-of-file value.
#define LAST_CLUSTER_FAT16          0xfff8

// Summary: A macro to indicate the last allocatable cluster for FAT12
// Description: The END_CLUSTER_FAT12 value is used as a comparison in FAT12 to determine that the firmware has reached the end of
//              the range of allowed allocatable clusters.
#define END_CLUSTER_FAT12           0xFF7

// Summary: A macro to indicate the last allocatable cluster for FAT16
// Description: The END_CLUSTER_FAT16 value is used as a comparison in FAT16 to determine that the firmware has reached the end of
//              the range of allowed allocatable clusters.
#define END_CLUSTER_FAT16           0xFFF7

// Summary: A macro to indicate the failure of the ReadFAT function
// Description: The CLUSTER_FAIL_FAT16 macro is used by the ReadFAT function to indicate that an error occured reading a FAT12 or FAT16
//              file allocation table.  Note that since '0xFFF8' is used for the last cluster return value in the FAT16 implementation
//              the end-of-file value '0xFFFF' can be used to indicate an error condition.
#define CLUSTER_FAIL_FAT16          0xFFFF



#ifdef SUPPORT_FAT32
    // Summary: A macro to indicate the last cluster value for FAT32
    // Description: The LAST_CLUSTER_FAT32 macro is used when reading the FAT to indicate that the next FAT32 entry for a file contains
    //              the end-of-file value.
    #define LAST_CLUSTER_FAT32      0x0FFFFFF8

    // Summary: A macro to indicate the last allocatable cluster for FAT32
    // Description: The END_CLUSTER_FAT32 value is used as a comparison in FAT32 to determine that the firmware has reached the end of
    //              the range of allowed allocatable clusters.
    #define END_CLUSTER_FAT32       0x0FFFFFF7

    // Summary: A macro to indicate the failure of the ReadFAT function
    // Description: The CLUSTER_FAIL_FAT32 macro is used by the ReadFAT function to indicate that an error occured reading a FAT32
    //              file allocation able.
    #define CLUSTER_FAIL_FAT32      0x0FFFFFFF

#endif

// Summary: A macro indicating the number of bytes in a directory entry.
// Description: The NUMBER_OF_BYTES_IN_DIR_ENTRY macro represents the number of bytes in one directory entry.  It is used to calculate
//              the number of sectors in the root directory based on information in the boot sector.
#define NUMBER_OF_BYTES_IN_DIR_ENTRY    32



// Summary: A macro for a deleted dir entry marker.
// Description: The DIR_DEL macro is used to mark a directory entry as deleted.  When a file is deleted, this value will replace the
//              first character in the file name, and will indicate that the file the entry points to was deleted.
#define DIR_DEL             0xE5

// Summary: A macro for the last dir entry marker.
// Description: The DIR_EMPTY macro is used to indicate the last entry in a directory.  Since entries in use cannot start with a 0 and
//              deleted entries start with the DIR_DEL character, a 0 will mark the end of the in-use or previously used group of
//              entries in a directory
#define DIR_EMPTY           0



// Summary: A macro used to indicate the length of an 8.3 file name
// Description: The DIR_NAMESIZE macro is used when validing the name portion of 8.3 filenames
#define DIR_NAMESIZE        8

// Summary: A macro used to indicate the length of an 8.3 file extension
// Description: The DIR_EXTENSION macro is used when validating the extension portion of 8.3 filenames
#define DIR_EXTENSION       3

// Summary: A macro used to indicate the length of an 8.3 file name and extension
// Description: The DIR_NAMECOMP macro is used when validating 8.3 filenames
#define DIR_NAMECOMP        (DIR_NAMESIZE+DIR_EXTENSION)



// Summary: A macro to write a byte to RAM
// Description: The RAMwrite macro is used to write a byte of data to a RAM array
#define RAMwrite( a, f, d) *(a+f) = d

// Summary: A macro to read a byte from RAM
// Description: The RAMread macro is used to read a byte of data from a RAM array
#define RAMread( a, f)  *(a+f)

// Summary: A macro to read a 16-bit word from RAM
// Description: The RAMreadW macro is used to read two bytes of data from a RAM array
#define RAMreadW( a, f) *(uint16_t *)(a+f)

// Summary: A macro to read a 32-bit word from RAM
// Description: The RAMreadD macro is used to read four bytes of data from a RAM array
#define RAMreadD( a, f) *(uint32_t *)(a+f)



#ifndef EOF
    // Summary: Indicates error conditions or end-of-file conditions
    // Description: The EOF macro is used to indicate error conditions in some function calls.  It is also used to indicate
    //              that the end-of-file has been reached.
    #define EOF             ((int)-1)
#endif



// Summary: A structure containing information about the device.
// Description: The DISK structure contains information about the device being accessed.
typedef struct
{ 
    uint8_t    *   buffer;         // Address of the global data buffer used to read and write file information
    uint32_t       firsts;         // Logical block address of the first sector of the FAT partition on the device
    uint32_t       fat;            // Logical block address of the FAT
    uint32_t       root;           // Logical block address of the root directory
    uint32_t       data;           // Logical block address of the data section of the device.
    uint16_t        maxroot;        // The maximum number of entries in the root directory.
    uint32_t       maxcls;         // The maximum number of clusters in the partition.
    uint32_t       sectorSize;     // The size of a sector in bytes
    uint32_t       fatsize;        // The number of sectors in the FAT
    uint8_t        fatcopy;        // The number of copies of the FAT in the partition
    uint8_t        SecPerClus;     // The number of sectors per cluster in the data region
    uint8_t        type;           // The file system type of the partition (FAT12, FAT16 or FAT32)
    uint8_t        mount;          // Device mount flag (TRUE if disk was mounted successfully, FALSE otherwise)
#if defined __PIC32MX__ || defined __C30__
} __attribute__ ((packed)) DISK;
#else
} DISK;
#endif

    // Summary: A 24-bit data type
    // Description: The SWORD macro is used to defined a 24-bit data type.  For 16+ bit architectures, this must be represented as
    //              an array of three bytes.
    typedef struct
    {
        unsigned char array[3];
    } __attribute__ ((packed)) SWORD;


// Summary: A structure containing the bios parameter block for a FAT12 file system (in the boot sector)
// Description: The _BPB_FAT12 structure provides a layout of the "bios parameter block" in the boot sector of a FAT12 partition.
typedef struct {
    SWORD BootSec_JumpCmd;          // Jump Command
    uint8_t  BootSec_OEMName[8];       // OEM name
    uint16_t BootSec_BPS;               // Number of bytes per sector
    uint8_t  BootSec_SPC;              // Number of sectors per cluster
    uint16_t BootSec_ResrvSec;          // Number of reserved sectors at the beginning of the partition
    uint8_t  BootSec_FATCount;         // Number of FATs on the partition
    uint16_t BootSec_RootDirEnts;       // Number of root directory entries
    uint16_t BootSec_TotSec16;          // Total number of sectors
    uint8_t  BootSec_MDesc;            // Media descriptor
    uint16_t BootSec_SPF;               // Number of sectors per FAT
    uint16_t BootSec_SPT;               // Number of sectors per track
    uint16_t BootSec_HeadCnt;           // Number of heads
    uint32_t BootSec_HiddenSecCnt;     // Number of hidden sectors
    uint32_t  BootSec_Reserved;        // Reserved space
    uint8_t  BootSec_DriveNum;         // Drive number
    uint8_t  BootSec_Reserved2;        // Reserved space
    uint8_t  BootSec_BootSig;          // Boot signature - equal to 0x29
    uint8_t  BootSec_VolID[4];         // Volume ID
    uint8_t  BootSec_VolLabel[11];     // Volume Label
    uint8_t  BootSec_FSType[8];        // File system type in ASCII. Not used for determination
#if defined __PIC32MX__ || defined __C30__
    } __attribute__ ((packed)) _BPB_FAT12;
#else
    } _BPB_FAT12;
#endif

// Summary: A structure containing the bios parameter block for a FAT16 file system (in the boot sector)
// Description: The _BPB_FAT16 structure provides a layout of the "bios parameter block" in the boot sector of a FAT16 partition.
typedef struct {
    SWORD BootSec_JumpCmd;          // Jump Command
    uint8_t  BootSec_OEMName[8];       // OEM name
    uint16_t  BootSec_BPS;              // Number of bytes per sector
    uint8_t  BootSec_SPC;              // Number of sectors per cluster
    uint16_t  BootSec_ResrvSec;         // Number of reserved sectors at the beginning of the partition
    uint8_t  BootSec_FATCount;         // Number of FATs on the partition
    uint16_t  BootSec_RootDirEnts;      // Number of root directory entries
    uint16_t  BootSec_TotSec16;         // Total number of sectors
    uint8_t  BootSec_MDesc;            // Media descriptor
    uint16_t  BootSec_SPF;              // Number of sectors per FAT
    uint16_t  BootSec_SPT;              // Number of sectors per track
    uint16_t  BootSec_HeadCnt;          // Number of heads
    uint32_t BootSec_HiddenSecCnt;     // Number of hidden sectors
    uint32_t BootSec_TotSec32;         // Total sector count (32 bits)
    uint8_t  BootSec_DriveNum;         // Drive number
    uint8_t  BootSec_Reserved;         // Reserved space
    uint8_t  BootSec_BootSig;          // Boot signature - equal to 0x29
    uint8_t  BootSec_VolID[4];         // Volume ID
    uint8_t  BootSec_VolLabel[11];     // Volume Label
    uint8_t  BootSec_FSType[8];        // File system type in ASCII. Not used for determination
#if defined __PIC32MX__ || defined __C30__
    } __attribute__ ((packed)) _BPB_FAT16;
#else
    } _BPB_FAT16;
#endif

// Summary: A structure containing the bios parameter block for a FAT32 file system (in the boot sector)
// Description: The _BPB_FAT32 structure provides a layout of the "bios parameter block" in the boot sector of a FAT32 partition.
typedef struct {
    SWORD BootSec_jmpBoot;          // Jump Command
    uint8_t  BootSec_OEMName[8];       // OEM name
    uint16_t BootSec_BytsPerSec;        // Number of bytes per sector
    uint8_t  BootSec_SecPerClus;       // Number of sectors per cluster
    uint16_t BootSec_RsvdSecCnt;        // Number of reserved sectors at the beginning of the partition
    uint8_t  BootSec_NumFATs;          // Number of FATs on the partition
    uint16_t BootSec_RootEntCnt;        // Number of root directory entries
    uint16_t BootSec_TotSec16;          // Total number of sectors
    uint8_t  BootSec_Media;            // Media descriptor
    uint16_t BootSec_FATSz16;           // Number of sectors per FAT
    uint16_t BootSec_SecPerTrk;         // Number of sectors per track
    uint16_t BootSec_NumHeads;          // Number of heads
    uint32_t BootSec_HiddSec;          // Number of hidden sectors
    uint32_t BootSec_TotSec32;         // Total sector count (32 bits)
    uint32_t BootSec_FATSz32;          // Sectors per FAT (32 bits)
    uint16_t BootSec_ExtFlags;          // Presently active FAT. Defined by bits 0-3 if bit 7 is 1.
    uint16_t BootSec_FSVers;            // FAT32 filesystem version.  Should be 0:0
    uint32_t BootSec_RootClus;         // Start cluster of the root directory (should be 2)
    uint16_t BootSec_FSInfo;            // File system information
    uint16_t BootSec_BkBootSec;         // Backup boot sector address.
    uint8_t  BootSec_Reserved[12];     // Reserved space
    uint8_t  BootSec_DrvNum;           // Drive number
    uint8_t  BootSec_Reserved1;        // Reserved space
    uint8_t  BootSec_BootSig;          // Boot signature - 0x29
    uint8_t  BootSec_VolID[4];         // Volume ID
    uint8_t  BootSec_VolLab[11];       // Volume Label
    uint8_t  BootSec_FilSysType[8];    // File system type in ASCII.  Not used for determination
#if defined __PIC32MX__ || defined __C30__
    } __attribute__ ((packed)) _BPB_FAT32;
#else
    } _BPB_FAT32;
#endif


// Description: A macro for the boot sector bytes per sector value offset
#define BSI_BPS            11

// Description: A macro for the boot sector sector per cluster value offset
#define BSI_SPC            13

// Description: A macro for the boot sector reserved sector count value offset
#define BSI_RESRVSEC       14

// Description: A macro for the boot sector FAT count value offset
#define BSI_FATCOUNT       16

// Description: A macro for the boot sector root directory entry count value offset
#define BSI_ROOTDIRENTS    17

// Description: A macro for the boot sector 16-bit total sector count value offset
#define BSI_TOTSEC16       19

// Description: A macro for the boot sector sectors per FAT value offset
#define BSI_SPF            22

// Description: A macro for the boot sector 32-bit total sector count value offset
#define BSI_TOTSEC32       32

// Description: A macro for the boot sector boot signature offset
#define BSI_BOOTSIG        38

// Description: A macro for the boot sector file system type string offset
#define BSI_FSTYPE         54

// Description: A macro for the boot sector 32-bit sector per FAT value offset
#define  BSI_FATSZ32       36

// Description: A macro for the boot sector start cluster of root directory value offset
#define  BSI_ROOTCLUS      44

// Description: A macro for the FAT32 boot sector boot signature offset
#define  BSI_FAT32_BOOTSIG 66

// Description: A macro for the FAT32 boot sector file system type string offset
#define  BSI_FAT32_FSTYPE  82



// Summary: A partition table entry structure.
// Description: The PTE_MBR structure contains values found in a partition table entry in the MBR of a device.
typedef struct
{
    uint8_t      PTE_BootDes;            // The boot descriptor (should be 0x00 in a non-bootable device)
    SWORD     PTE_FrstPartSect;       // The cylinder-head-sector address of the first sector of the partition
    uint8_t      PTE_FSDesc;             // The file system descriptor
    SWORD     PTE_LstPartSect;        // The cylinder-head-sector address of the last sector of the partition
    uint32_t     PTE_FrstSect;           // The logical block address of the first sector of the partition
    uint32_t     PTE_NumSect;            // The number of sectors in a partition
#if defined __PIC32MX__ || defined __C30__
    } __attribute__ ((packed)) PTE_MBR;
#else
    } PTE_MBR;
#endif


// Summary: A structure of the organization of a master boot record.
// Description: The _PT_MBR structure has the same form as a master boot record.  When the MBR is loaded from the device, it will
//              be cast as a _PT_MBR structure so the MBR elements can be accessed.
typedef struct
{
    uint8_t        ConsChkRtn[446];        // Boot code
    PTE_MBR     Partition0;             // The first partition table entry
    PTE_MBR     Partition1;             // The second partition table entry
    PTE_MBR     Partition2;             // The third partition table entry
    PTE_MBR     Partition3;             // The fourth partition table entry
    uint8_t        Signature0;             // MBR signature code - equal to 0x55
    uint8_t        Signature1;             // MBR signature code - equal to 0xAA
#if defined __PIC32MX__ || defined __C30__
}__attribute__((packed)) _PT_MBR;
#else
}_PT_MBR;
#endif

// Summary: A pointer to a _PT_MBR structure
// Description: The PT_MBR pointer points to a _PT_MBR structure.
typedef _PT_MBR *  PT_MBR;



// Summary: A structure of the organization of a boot sector.
// Description: The _BootSec structure has the same form as a boot sector.  When the boot sector is loaded from the device, it will
//              be cast as a _BootSec structure so the boot sector elements can be accessed.
typedef struct
{
    // A union of different bios parameter blocks
    union
    {
        _BPB_FAT32  FAT_32;
        _BPB_FAT16  FAT_16;
        _BPB_FAT12  FAT_12;
    }FAT;
    uint8_t    Reserved[512-sizeof(_BPB_FAT32)-2]; // Reserved space
    uint8_t    Signature0;         // Boot sector signature code - equal to 0x55
    uint8_t    Signature1;         // Boot sector signature code - equal to 0xAA
#if defined __PIC32MX__ || defined __C30__
    } __attribute__ ((packed)) _BootSec;
#else
    } _BootSec;
#endif

// Summary: A pointer to a _BootSec structure
// Description: The BootSec pointer points to a _BootSec structure.
typedef _BootSec * BootSec;



// Summary: A macro indicating the offset for the master boot record
// Description: FO_MBR is a macro that indicates the addresss of the master boot record on the device.  When the device is initialized
//              this sector will be read
#define FO_MBR          0L



// Summary: A macro for the first boot sector/MBR signature byte
// Description: The FAT_GOOD_SIGN_0 macro is used to determine that the first byte of the MBR or boot sector signature code is correct
#define FAT_GOOD_SIGN_0     0x55

// Summary: A macro for the second boot sector/MBR signature byte
// Description: The FAT_GOOD_SIGN_1 macro is used to determine that the second byte of the MBR or boot sector signature code is correct
#define FAT_GOOD_SIGN_1     0xAA


typedef struct 
{
    uint8_t    errorCode;
    union 
    {
        uint8_t    value;
        struct 
        {
            uint8_t    sectorSize  : 1;
            uint8_t    maxLUN      : 1;
        }   bits;
    } validityFlags;
    
    uint16_t    sectorSize;
    uint8_t    maxLUN;
} MEDIA_INFORMATION;

typedef enum
{
    MEDIA_NO_ERROR,                     // No errors
    MEDIA_DEVICE_NOT_PRESENT,           // The requested device is not present
    MEDIA_CANNOT_INITIALIZE             // Cannot initialize media
} MEDIA_ERRORS;                


#endif
