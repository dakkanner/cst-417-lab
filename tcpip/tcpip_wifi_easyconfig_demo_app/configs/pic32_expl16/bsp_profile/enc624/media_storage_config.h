/*******************************************************************************
Microchip storage media Configuration Header

  Summary:
    
  Description:

*******************************************************************************/

/*******************************************************************************
FileName: 	media_storage_config.h
Copyright ?2012 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS?WITHOUT WARRANTY OF ANY KIND,
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

#ifndef __MEDIA_STORAGE_PROFILE_H_
#define __MEDIA_STORAGE_PROFILE_H_

// =======================================================================
//   Media Storage configuration
// =======================================================================


// The media storage descriptor
// Describes the associated media type
// and some physical characteristics
typedef struct
{
    const char*     storageName;    // associated name of the storage
                                    // supported names: "eeprom", "spi_flash", "flash";
                                    // ("flash" refers to the internal program flash)
    DRV_MEDIA_TYPE  mediaType;      // what kind of media we're describing
                                    // see drv_media.h
    unsigned long   phyStartOffset; // lowest physical start address/offset on the media;
                                    // 0 means the very beginning of the media
    unsigned long   phySize;        // physical media size;
                                    // addressable storage range: [phyStartOffset, phyStartOffset + phySize).

}MEDIA_STORAGE_INFO;



// The media storage can be partitioned to be used by multiple services
// ( a file system, the tcpip configuration storage service, etc).
// This is the description of the media partitioning that will be used
// by different system services (file systems) or tcpip services, etc.

typedef struct
{
    const char*     partName;           // name that identifies the partition
                                        // valid names: "mpfs2", "tcpip_storage";
    unsigned long   partStartOffset;    // the absolute starting offset on that media                                    
    unsigned long   partSize;           // size of the partition, in bytes
}MEDIA_STORAGE_PARTITION_INFO;


// Finally the complete description of the media:
// physical characteristics + partition info
// Multiple such descriptions can be provided for multiple existent
// storage media.
// For now the TCPIP stack use only one storage media.
//
// Note: there is no runtime check when the media initialization
// routine is called to verify that the partitions are disjoint!
// The partTable should be constructed carefully!
//
typedef struct
{
    MEDIA_STORAGE_INFO                  storageInfo;    // description of the storage media
    unsigned long                       nPartitions;    // how many partitions we're describing
    // partition table
    const MEDIA_STORAGE_PARTITION_INFO** partTable;     // nPartitions entries array
}MEDIA_STORAGE_CONFIG;




// Definitions of some usual types of storages
// We can use these conveniently when providing
// the MEDIA_STORAGE_CONFIG structure.
#define MEDIA_STORAGE_INFO_EEPROM_25LC256           {   \
                                                    "eeprom",           /* storageName */      \
                                                    DRV_MEDIA_EEPROM,   /* mediaType */        \
                                                    0,                  /* phyStartOffset */   \
                                                    (256/8 * 1024),     /* phySize */          \
                                                    }
                                        
#define MEDIA_STORAGE_INFO_EEPROM_25LC1024          {   \
                                                    "eeprom",           /* storageName */      \
                                                    DRV_MEDIA_EEPROM,   /* mediaType */        \
                                                    0,                  /* phyStartOffset */   \
                                                    (1024/8 * 1024)     /* phySize */          \
                                                    }
                                        



#define MEDIA_STORAGE_INFO_SPI_FLASH_SST25VF016B    {   \
                                                    "spi_flash",        /* storageName */      \
                                                    DRV_MEDIA_SPI_FLASH,/* mediaType */        \
                                                    0,                  /* phyStartOffset */   \
                                                    (16/8 * 1024 *1024) /* phySize */          \
                                                    }
                                        

#define MEDIA_STORAGE_INFO_FLASH_PIC32              {   \
                                                    "flash",                    /* storageName */      \
                                                    DRV_MEDIA_INTERNAL_FLASH,   /* mediaType */        \
                                                    0x9d000000,                 /* phyStartOffset: k0 start */   \
                                                    512 * 1024,                 /* phySize: all k0 */     \
                                                    }

// Apparently the 16 bit version has to 
// implement specific flash memory access functions
// and it was discontinued from the v5 stack anyway
// Not supported for now

#define MEDIA_STORAGE_INFO_NOT_SUPPORTED            {   \
                                                    "",                         /* storageName */      \
                                                    DRV_MEDIA_UNKNOWN,          /* mediaType */        \
                                                    0,                          /* phyStartOffset */   \
                                                    0,                          /* partSize */     \
                                                    }



// The actual storage configuration for this bsp profile 

#if defined(SYSTEM_DRIVER_MODULE_CONFIGURATION)
// the partition table
//

//  TCPIP Configuration Storage Reserved Area
// 	The Number of bytes to be reserved for this partition.
// 	This partition will store host application configurations such as IP Address,
// 	MAC Address, and any other required variables.
//
// 	Note: Depending on the enabled services, features and interfaces 
// 	the storage needed for configuration increases.
// 	Adjust accordingly!
#define TCPIP_STORAGE_PARTITION_SIZE    (200)
const MEDIA_STORAGE_PARTITION_INFO MEDIA_STORAGE_TCPIP_STORAGE_PARTITION = 
{
    "tcpip_storage",                        // partName
    0,                                      // partStartOffset;
    TCPIP_STORAGE_PARTITION_SIZE,           // partSize; 
};

const MEDIA_STORAGE_PARTITION_INFO MEDIA_STORAGE_MPFS_IMAGE_PARTITION = 
{
    "mpfs2",                                // partName
    TCPIP_STORAGE_PARTITION_SIZE,           // partStartOffset;
    0x8000 - TCPIP_STORAGE_PARTITION_SIZE,  // partSize; use the whole remainder of the storage
};

const MEDIA_STORAGE_PARTITION_INFO* MEDIA_STORAGE_PARTITION_TABLE[] = 
{
    &MEDIA_STORAGE_TCPIP_STORAGE_PARTITION,
    &MEDIA_STORAGE_MPFS_IMAGE_PARTITION,
};

// Note: this is a parameter of the DRV_MEDIA_Initialize
// It has to be PERSISTENT because there is no extra storage associated
// in the drv_media to store this info!!!
// MEDIA_STORAGE_EEPROM, MEDIA_EEPROM_TYPE_25LC256 should be defined!
const MEDIA_STORAGE_CONFIG MEDIA_STORAGE_CONFIGURATION =
{
    //  MEDIA_STORAGE_INFO storageInfo;
    MEDIA_STORAGE_INFO_EEPROM_25LC256,

    // nPartitions
    sizeof(MEDIA_STORAGE_PARTITION_TABLE)/sizeof(*MEDIA_STORAGE_PARTITION_TABLE),

    //  MEDIA_STORAGE_PARTITION_INFO**    partTable;
    MEDIA_STORAGE_PARTITION_TABLE                       
};
#endif  // defined(SYSTEM_DRIVER_MODULE_CONFIGURATION)


// The maximum number of supported clients by the media driver
// Usually 1 client is needed for any created partition
// (since 2 clients cannot own the same partition simultaneously).
// Normally you need the File System client(s) and the TCPIP configuration client.
#define     MEDIA_STORAGE_DRV_MEDIA_CLIENTS_NO        2



#endif  // __MEDIA_STORAGE_PROFILE_H_

