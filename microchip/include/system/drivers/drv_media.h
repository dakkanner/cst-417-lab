/*******************************************************************************
  Media Storage Driver Header

  Summary:
    SUMMARY
    
  Description:
    Media storage driver is used by the File System implementation to
    access the media storage
*******************************************************************************/

/*******************************************************************************
FileName:  drv_media.h 
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

#ifndef _DRV_MEDIA_H_
#define _DRV_MEDIA_H_

#include <stdint.h>
#include <stdbool.h>

// NOTE: This is a very minimal implementation!
// Only one type of media is supported at one time!
// Only one client is supported at one time!

// common definitions

typedef const void* DRV_MEDIA_HANDLE;
#define             DRV_MEDIA_HANDLE_INVALID  (0)


typedef unsigned int                    DRV_MEDIA_MODULE_OBJ;
#define DRV_MEDIA_MODULE_OBJ_INVALID    ((DRV_MEDIA_MODULE_OBJ) -1 )


typedef enum
{
    DRV_MEDIA_UNKNOWN,          // invalid media type
    DRV_MEDIA_DEFAULT,          // default media;
                                // mainly for systems where only one media supported 
    DRV_MEDIA_EEPROM,           // EEPROM
    DRV_MEDIA_SPI_FLASH,        // SPI flash:  SST 25VF016B, etc
    DRV_MEDIA_INTERNAL_FLASH,   // on chip Flash
    DRV_MEDIA_MDD_FLASH, // MDD
}DRV_MEDIA_TYPE;

typedef enum
{
    /* Read */
    DRV_MEDIA_IO_INTENT_READ = 1 << 0 ,

    /* Write */
    DRV_MEDIA_IO_INTENT_WRITE = 1 << 1,

    /* Read and Write*/
    DRV_MEDIA_IO_INTENT_READWRITE = DRV_MEDIA_IO_INTENT_READ|DRV_MEDIA_IO_INTENT_WRITE,

    /* The driver will block and will return when the operation is complete */
    DRV_MEDIA_IO_INTENT_BLOCKING = 0 << 2,

    /* The driver will return immediately; Not supported */
    DRV_MEDIA_IO_INTENT_NONBLOCKING = 1 << 2,

    /* The driver will support only one client at a time */
    DRV_MEDIA_IO_INTENT_EXCLUSIVE  = 1 << 3 ,

    /* The driver will support multiple clients at a time; Not supported */
    DRV_MEDIA_IO_INTENT_SHARED = 0 << 3

}DRV_MEDIA_IO_INTENT;


typedef enum
{
    /* Up and running, ready to start new operations */
    DRV_MEDIA_CLIENT_STATUS_READY = 0,

    /* Operation in progress, unable to start a new one */
    DRV_MEDIA_CLIENT_STATUS_BUSY, 

    /* client Invalid */
    DRV_MEDIA_STATUS_INVALID = -1,

    /* bad parameter */
    DRV_MEDIA_STATUS_PARAM_ERROR = -2,

    /* not supported operation */
    DRV_MEDIA_STATUS_OPERATION_ERROR = -3,

    /* storage full */
    DRV_MEDIA_STATUS_EOS = -4,

    /* range error */
    DRV_MEDIA_STATUS_RANGE_ERR = -5,


    /* read/write error */
    DRV_MEDIA_STATUS_RW_ERR = -6,

} DRV_MEDIA_CLIENT_STATUS;


// mediaIndex is for supporting multiple media storages
// set to 0 for now
// the initData type is 
// const MEDIA_STORAGE_CONFIG* const
// it is not optional! (default data is not supported for now)
// It sets both the physical limits of the storage: [phyStartOffset, phyStartOffset);
// and the limits of each partition: [partStartOffset, partEndOffset);
//
// Note: there is no run time check (yet) that the partition areas do not overlap!
//       The partitions should be disjoint! 
DRV_MEDIA_MODULE_OBJ    DRV_MEDIA_Initialize( int mediaIndex, const void * const initData);

// de-initialize the driver
void                    DRV_MEDIA_Deinitialize( DRV_MEDIA_MODULE_OBJ object);

// gives access to a client to a specified partition on the media
// 
// Note: only one client per partition is allowed!
// Note: for now the number of the supported clients is build time defined
// by MEDIA_STORAGE_DRV_MEDIA_CLIENTS_NO
DRV_MEDIA_HANDLE        DRV_MEDIA_Open( DRV_MEDIA_TYPE type, const char* partName, const DRV_MEDIA_IO_INTENT ioIntent);

// releases a client's usage of a partition
void                    DRV_MEDIA_Close( const DRV_MEDIA_HANDLE handle);

// the total size allocated to the client on the client's partition
// It is the whole partition size unless the user decided to shrink it.
// client media size returned is = cliEndOffset - cliStartOffset;     
uint32_t                DRV_MEDIA_ClientSizeGet(const DRV_MEDIA_HANDLE handle);

// Can be used to shrink/extend the client range within a client partition
// However the partition limis cannot be exceeded!
// if cliSize == 0 then the maximum size is used
// client media addressable range will be set to [cliStartOffset, cliStartOffset + cliSize);
DRV_MEDIA_CLIENT_STATUS DRV_MEDIA_ClientRangeSet(const DRV_MEDIA_HANDLE handle, uint32_t cliStartOffset, uint32_t cliSize);

// current client status
DRV_MEDIA_CLIENT_STATUS DRV_MEDIA_ClientStatus( const DRV_MEDIA_HANDLE handle );

// Sets the current read offset
// The read offset is advanced by successive read operations.
// The read offset is a relative offset within the client partition space.
DRV_MEDIA_CLIENT_STATUS DRV_MEDIA_ReadSetOffset(DRV_MEDIA_HANDLE handle, uint32_t offset);

// reads a buffer from a client partition
int                     DRV_MEDIA_Read( const DRV_MEDIA_HANDLE handle, uint8_t *buffer,  const unsigned int numbytes);

// reads a byte from a client partition
uint8_t                 DRV_MEDIA_ReadByte( const DRV_MEDIA_HANDLE handle);

// Sets the current write offset
// The write offset is advanced by successive write operations.
// The write offset is a relative offset within the client partition space.
DRV_MEDIA_CLIENT_STATUS DRV_MEDIA_WriteSetOffset(DRV_MEDIA_HANDLE handle, uint32_t offset);


// writes a buffer to a client partition
int                     DRV_MEDIA_Write( const DRV_MEDIA_HANDLE handle,  const uint8_t *buffer,  const unsigned int numbytes );

// writes a byte to a client partition
int                     DRV_MEDIA_WriteByte( const DRV_MEDIA_HANDLE handle,  const uint8_t byte);

// ensures that all the data has been written to the parition
int                     DRV_MEDIA_WriteFlush( const DRV_MEDIA_HANDLE handle);



#endif  // _DRV_MEDIA_H_

