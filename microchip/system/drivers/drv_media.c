/*******************************************************************************
  Media Storage Driver Implementation

  Summary:
    SUMMARY
    
  Description:
    Media storage driver is used by the File System implementation to
    access the media storage
*******************************************************************************/

/*******************************************************************************
FileName:  drv_media.c 
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
#include "hardware_config.h"

#include "system/drivers/drv_media.h"

#include "system/drivers/xeeprom.h"
#include "system/drivers/spi_flash.h"

#include "media_storage_config.h"


// NOTE: This is a very minimal implementation!
// Only one type of media is supported at one time!
// The number of supported clients is a build time constant for now!

// types of supported storages
#if defined(MEDIA_STORAGE_EEPROM) || defined(MEDIA_STORAGE_SPIFLASH) || defined(MEDIA_STORAGE_INTERNAL_FLASH)


// sanity check that is one of the supported devices
#if defined(MEDIA_STORAGE_EEPROM)
    #if !(defined(MEDIA_EEPROM_TYPE_25LC256) || defined(MEDIA_EEPROM_TYPE_25LC1024))
        #error "MEDIA_EEPROM_TYPE_25LC256/MEDIA_EEPROM_TYPE_25LC1024 are the only EEPROM supported for now!"
    #endif
#elif defined(MEDIA_STORAGE_SPIFLASH)
    #if !(defined(MEDIA_SPIFLASH_TYPE_SST25VF016B))
        #error "MEDIA_SPIFLASH_TYPE_SST25VF016B is the only SPI Flash supported for now!"
    #endif
#endif

// data

typedef int     (*pMediaInit)(const MEDIA_STORAGE_CONFIG* pConfigData);     // init function		

typedef	int     (*pMediaRead)(uint32_t address, uint8_t *vData, uint32_t wLen);

typedef	int     (*pMediaWriteOffset)(uint32_t offset);
typedef int     (*pMediaWrite)(const uint8_t *vData, uint32_t wLen);
typedef	int     (*pMediaWriteByte)(const uint8_t byte);
typedef	int     (*pMediaWriteEnd)(void);


// functions supported by a storage media
typedef struct
{
    pMediaInit          init;
    pMediaRead          read;
    pMediaWriteOffset   writeOffset;
    pMediaWrite         write;
    pMediaWriteByte     writeByte;
    pMediaWriteEnd      writeEnd;
}MEDIA_ACCESS_FNC;    


typedef struct
{
    MEDIA_ACCESS_FNC        mediaAcc;       // the one and only accessed media
    DRV_MEDIA_TYPE          mediaType;      // the type of this media
    uint32_t                phyStartOffset; // lowest physical address/offset on the media; usually 0
    uint32_t                phyEndOffset;   // highest physical address/offset on the media;
                                            // physical media addressable range is [phyStartOffset, phyEndOffset)
                                            // physical media size = phyEndOffset - phyStartOffset;
    uint32_t                nPartitions;    // number of partitions
    const MEDIA_STORAGE_PARTITION_INFO** partTbl;    // persistent pointer to the partition table description!     
}DRV_MEDIA_DCPT;


typedef struct
{
    DRV_MEDIA_CLIENT_STATUS status;         // DRV_MEDIA_STATUS_INVALID means not busy 
    uint32_t                startOffset;    // lowest client absolute address/offset on the media; usually 0
    uint32_t                endOffset;      // highest client absolute address/offset on the media;
                                            // client media addressable range is [startOffset, endOffset) !!!
                                            // client media size = endOffset - startOffset;     
    uint32_t                rdOffset;       // current relative rd ptr
    uint32_t                wrOffset;       // current relative wr ptr
    const MEDIA_STORAGE_PARTITION_INFO* partInfo;   // currently acquired partition    
}DRV_MEDIA_CLIENT_DCPT;     // client info


#if defined(MEDIA_STORAGE_EEPROM)
    static  int     _XEEFlashInit(const MEDIA_STORAGE_CONFIG* pConfigData);  

    static DRV_MEDIA_DCPT   drvMediaDcpt =
    {
        {
            _XEEFlashInit,
            XEEReadArray,
            XEEBeginWrite,
            XEEWriteArray,
            XEEWrite,
            XEEEndWrite,
        },      // mediaAcc;
        DRV_MEDIA_EEPROM,       // mediaType;
        0,                      // phyStartOffset
        0,                      // phyEndOffset;
        0,                      // nPartitions
        0,                      // partTbl
    };   // the one and only descriptor

#elif defined(MEDIA_STORAGE_SPIFLASH)
    static int      _SPIFlashInit(const MEDIA_STORAGE_CONFIG* pConfigData);  

    static DRV_MEDIA_DCPT   drvMediaDcpt =
    {
        {
            _SPIFlashInit,
            SPIFlashReadArray,
            SPIFlashBeginWrite,
            SPIFlashWriteArray,
            SPIFlashWrite,
            SPIFlashEndWrite,
        },  // mediaAcc;
        DRV_MEDIA_SPI_FLASH,    // mediaType;
        0,                      // phyStartOffset
        0,                      // phyEndOffset;
        0,                      // nPartitions
        0,                      // partTbl
    };   // the one and only descriptor

#else
    // internal Flash functions
    #if defined(__C32__)
        static int      _FlashMediaInit(const MEDIA_STORAGE_CONFIG* pConfigData);  
        static	int     _FlashMediaRead(uint32_t address, uint8_t *vData, uint32_t wLen);

        static DRV_MEDIA_DCPT   drvMediaDcpt =
        {
            {
                _FlashMediaInit,
                _FlashMediaRead,
                // write operations are not supported yet
                0,
                0,
                0,
                0
            },
            DRV_MEDIA_INTERNAL_FLASH,   // mediaType;
            0,                          // phyStartOffset
            0,                          // phyEndOffset
            0,                          // nPartitions
            0,                          // partTbl
        };   // the one and only descriptor

    #else

        static DRV_MEDIA_DCPT   drvMediaDcpt =
        {
            {
                0,
                0,
                0,
                0,
                0,
                0
            },
            DRV_MEDIA_INTERNAL_FLASH,   // mediaType;
            0,                          // phyStartOffset;
            0,                          // phyEndOffset;
            0,                          // nPartitions
            0,                          // partTbl
        };   // the one and only descriptor
    #endif  // defined(__C32__)



#endif  // MEDIA_STORAGE_EEPROM | MEDIA_STORAGE_SPIFLASH | MEDIA_STORAGE_INTERNAL_FLASH


static DRV_MEDIA_DCPT*          pGblMediaDcpt = 0; // the one and only supported media descriptor

static DRV_MEDIA_CLIENT_DCPT    gblMediaClients[MEDIA_STORAGE_DRV_MEDIA_CLIENTS_NO] = { { 0 } };  // supported clients

DRV_MEDIA_MODULE_OBJ DRV_MEDIA_Initialize(  int mediaIndex, const void * const initData)
{
    int         ix;
    int         res;
    DRV_MEDIA_CLIENT_DCPT* pClient;
    const MEDIA_STORAGE_CONFIG* pConfigData = (const MEDIA_STORAGE_CONFIG*)initData;

    pGblMediaDcpt = 0;  // kill any previous instance
    pClient = gblMediaClients;
    for(ix = 0; ix < sizeof(gblMediaClients)/sizeof(*gblMediaClients); ix++)
    {
        memset(pClient, 0x0, sizeof(*pClient));
        pClient->status = DRV_MEDIA_STATUS_INVALID;
        pClient++;
    }


    while(drvMediaDcpt.mediaAcc.init != 0 && pConfigData != 0)
    {
        // verify that the supported media type is requested
        if(pConfigData->storageInfo.mediaType != drvMediaDcpt.mediaType && pConfigData->storageInfo.mediaType != DRV_MEDIA_DEFAULT)
        {   // not supported type
            break;
        }

        drvMediaDcpt.phyStartOffset = pConfigData->storageInfo.phyStartOffset;
        drvMediaDcpt.phyEndOffset = drvMediaDcpt.phyStartOffset + pConfigData->storageInfo.phySize;
        // check
        if(drvMediaDcpt.phyStartOffset > drvMediaDcpt.phyEndOffset)
        {   // wrong limits
            break;
        }

        // store
        drvMediaDcpt.nPartitions = pConfigData->nPartitions;
        drvMediaDcpt.partTbl = pConfigData->partTable;

        res = drvMediaDcpt.mediaAcc.init(pConfigData);
        if(res == 0)
        {
            pGblMediaDcpt = &drvMediaDcpt;
            return 0;
        }

        break;
    }


    return  DRV_MEDIA_MODULE_OBJ_INVALID;
}


void DRV_MEDIA_Deinitialize( DRV_MEDIA_MODULE_OBJ object)
{
    if(object == 0)
    {
        pGblMediaDcpt = 0;

        //
    }
}


DRV_MEDIA_HANDLE DRV_MEDIA_Open( DRV_MEDIA_TYPE type, const char* partName, const DRV_MEDIA_IO_INTENT ioIntent)
{
    int ix;
    DRV_MEDIA_CLIENT_DCPT* pClient;
    const MEDIA_STORAGE_PARTITION_INFO* pCliPart, *pCurrPart, **pPart;
    
    // make sure the requested partition is a valid one

    pCliPart = 0;
    pPart = pGblMediaDcpt->partTbl;
    for(ix = 0; ix < pGblMediaDcpt->nPartitions; ix++)
    {
        pCurrPart = *pPart++;
        if(strcmp(partName, pCurrPart->partName) == 0)
        {   // found
            pCliPart = pCurrPart;
            break;
        }
    }

    if(pCliPart == 0)
    {   // no such partition
        return DRV_MEDIA_HANDLE_INVALID;
    }

    // make sure that this partition is not in use
    // we don't allow multiple clients on the same partition!
    pClient = gblMediaClients;
    for(ix = 0; ix < sizeof(gblMediaClients)/sizeof(*gblMediaClients); ix++)
    {
        if(pClient->status != DRV_MEDIA_STATUS_INVALID && pClient->partInfo == pCliPart)
        {   // found used partition
            return DRV_MEDIA_HANDLE_INVALID;
        }
        pClient++;
    }

    if(type == DRV_MEDIA_DEFAULT)
    {
        type = pGblMediaDcpt->mediaType;
    }

    if(pGblMediaDcpt->mediaType == type)
    {   // type match
        if(pGblMediaDcpt != 0)
        {   // initialized
            if((ioIntent & DRV_MEDIA_IO_INTENT_NONBLOCKING) == 0)
            {   // no support for non blocking operations for now
                if((ioIntent & DRV_MEDIA_IO_INTENT_EXCLUSIVE) != 0)
                {   // no support for shared access for now
                    if( ((ioIntent & DRV_MEDIA_IO_INTENT_WRITE) == 0 || pGblMediaDcpt->mediaAcc.write != 0) &&
                            ((ioIntent & DRV_MEDIA_IO_INTENT_READ) == 0 || pGblMediaDcpt->mediaAcc.read != 0) )    
                    {   // make sure we support read/write requested operations
                        // get an available client
                        pClient = gblMediaClients;
                        for(ix = 0; ix < sizeof(gblMediaClients)/sizeof(*gblMediaClients); ix++)
                        {
                            if(pClient->status == DRV_MEDIA_STATUS_INVALID)
                            {   // found empty slot
                                memset(pClient, 0x0, sizeof(pClient));
                                pClient->partInfo = pCliPart;
                                pClient->startOffset = pCliPart->partStartOffset;
                                pClient->endOffset = pCliPart->partStartOffset + pCliPart->partSize;
                                pClient->status = DRV_MEDIA_CLIENT_STATUS_READY;
                                return pClient;
                            }
                            pClient++;
                        }
                    }
                }
            }
        }
    }

    return DRV_MEDIA_HANDLE_INVALID;
}

void DRV_MEDIA_Close( const DRV_MEDIA_HANDLE handle)
{
    DRV_MEDIA_CLIENT_DCPT* pClient = (DRV_MEDIA_CLIENT_DCPT*)handle;

    pClient->status = DRV_MEDIA_STATUS_INVALID;
}

DRV_MEDIA_CLIENT_STATUS DRV_MEDIA_ClientStatus( const DRV_MEDIA_HANDLE handle )
{
    DRV_MEDIA_CLIENT_DCPT* pClient = (DRV_MEDIA_CLIENT_DCPT*)handle;

    return pClient->status;
}

uint32_t DRV_MEDIA_ClientSizeGet(const DRV_MEDIA_HANDLE handle)
{
    DRV_MEDIA_CLIENT_DCPT* pClient = (DRV_MEDIA_CLIENT_DCPT*)handle;
    return pClient->endOffset - pClient->startOffset;
}

// if cliSize == 0 then the maximum size is used
DRV_MEDIA_CLIENT_STATUS DRV_MEDIA_ClientRangeSet(const DRV_MEDIA_HANDLE handle, uint32_t startOffset, uint32_t cliSize)
{
    uint32_t    endOffset, minOffset, maxOffset;

    DRV_MEDIA_CLIENT_DCPT* pClient = (DRV_MEDIA_CLIENT_DCPT*)handle;

    minOffset =  pClient->partInfo->partStartOffset;
    maxOffset =  minOffset + pClient->partInfo->partSize;

    if(cliSize == 0)
    {
        endOffset = maxOffset;
    }
    else
    {
        endOffset = startOffset + cliSize;
    }

    // check 
    while(startOffset >= minOffset && endOffset <= maxOffset)
    {   // within limits
    #if defined(MEDIA_STORAGE_SPIFLASH)
        if( pGblMediaDcpt->mediaType == DRV_MEDIA_SPI_FLASH && (startOffset & 0xffff) != 0 )
        {   // startOffset has to be on a 4KB boundary for the SPI Flash storage
            break;
        }
    #endif  // defined(MEDIA_STORAGE_SPIFLASH)


        pClient->startOffset = startOffset;
        pClient->endOffset = endOffset;
        return (pClient->status = DRV_MEDIA_CLIENT_STATUS_READY);
    }

    
    return (pClient->status = DRV_MEDIA_STATUS_RANGE_ERR);
}

// sets a read relative offset within  [startOffset, endOffset)
// The absolute media offset is: startOffset + offs
// and startOffset + offs < endOffset check is done!
DRV_MEDIA_CLIENT_STATUS DRV_MEDIA_ReadSetOffset(DRV_MEDIA_HANDLE handle, uint32_t offset)
{
    DRV_MEDIA_CLIENT_DCPT* pClient = (DRV_MEDIA_CLIENT_DCPT*)handle;
    

    if(pClient->startOffset + offset <= pClient->endOffset)
    {
        pClient->rdOffset = offset;
        pClient->status = DRV_MEDIA_CLIENT_STATUS_READY;
    }
    else
    {
        pClient->status = DRV_MEDIA_STATUS_PARAM_ERROR;
    }

    return pClient->status;
}

int DRV_MEDIA_Read( const DRV_MEDIA_HANDLE handle, uint8_t *buffer,  const unsigned int numbytes)
{
    unsigned int nBytes = 0;
    DRV_MEDIA_CLIENT_DCPT* pClient = (DRV_MEDIA_CLIENT_DCPT*)handle;

    if(pGblMediaDcpt->mediaAcc.read == 0)
    {
        pClient->status = DRV_MEDIA_STATUS_OPERATION_ERROR;
    }
    else if(numbytes)
    {
        uint32_t absOffset = pClient->startOffset + pClient->rdOffset;
        unsigned int avlblBytes = pClient->endOffset - absOffset;
        if(numbytes > avlblBytes)
        {
            nBytes = avlblBytes;
        }
        else
        {
            nBytes = numbytes;
        }

        if(nBytes)
        {
            if( (*pGblMediaDcpt->mediaAcc.read)(absOffset, buffer, nBytes) == 0)
            {
                pClient->rdOffset += nBytes;
                pClient->status = DRV_MEDIA_CLIENT_STATUS_READY;
            }
            else
            {
                pClient->status = DRV_MEDIA_STATUS_RW_ERR;
            }
        }
        else
        {
            pClient->status = DRV_MEDIA_STATUS_EOS;
        }
    }
    else
    {
        pClient->status = DRV_MEDIA_STATUS_PARAM_ERROR; 
    }


    return nBytes;

}

uint8_t DRV_MEDIA_ReadByte( const DRV_MEDIA_HANDLE handle)
{
    int     nBytes;
    uint8_t vByte; 

    nBytes = DRV_MEDIA_Read(handle, &vByte, 1);

    return nBytes?vByte:0;  // status should be set

}

// sets a write relative offset within  [startOffset, endOffset)
// The absolute media offset is: startOffset + offs
// and startOffset + offs < endOffset check is done!
DRV_MEDIA_CLIENT_STATUS DRV_MEDIA_WriteSetOffset(DRV_MEDIA_HANDLE handle, uint32_t offset)
{
    uint32_t    absOffset;

    DRV_MEDIA_CLIENT_DCPT* pClient = (DRV_MEDIA_CLIENT_DCPT*)handle;
    
    if(pGblMediaDcpt->mediaAcc.writeOffset == 0)
    {
        pClient->status = DRV_MEDIA_STATUS_OPERATION_ERROR;
    }
    else if( (absOffset = pClient->startOffset + offset) <= pClient->endOffset)
    {
        pClient->wrOffset = offset;
        (*pGblMediaDcpt->mediaAcc.writeOffset)(absOffset);
        pClient->status = DRV_MEDIA_CLIENT_STATUS_READY;
    }
    else
    {
        pClient->status = DRV_MEDIA_STATUS_PARAM_ERROR;
    }

    return pClient->status;
}


int DRV_MEDIA_Write( const DRV_MEDIA_HANDLE handle,  const uint8_t *buffer,  const unsigned int numbytes )
{
    unsigned int nBytes = 0;
    DRV_MEDIA_CLIENT_DCPT* pClient = (DRV_MEDIA_CLIENT_DCPT*)handle;

    if(pGblMediaDcpt->mediaAcc.write == 0)
    {
        pClient->status = DRV_MEDIA_STATUS_OPERATION_ERROR;
    }
    else if(numbytes)
    {
        unsigned int avlblBytes = pClient->endOffset - (pClient->startOffset + pClient->wrOffset);
        if(numbytes > avlblBytes)
        {
            nBytes = avlblBytes;
        }
        else
        {
            nBytes = numbytes;
        }

        if(nBytes)
        {
            (*pGblMediaDcpt->mediaAcc.write)(buffer, nBytes);
            pClient->wrOffset += nBytes;
            pClient->status = DRV_MEDIA_CLIENT_STATUS_READY;
        }
        else
        {
            pClient->status = DRV_MEDIA_STATUS_EOS;
        }
    }
    else
    {
        pClient->status = DRV_MEDIA_STATUS_PARAM_ERROR;
    }

    return nBytes;
}

int DRV_MEDIA_WriteByte( const DRV_MEDIA_HANDLE handle,  const uint8_t byte)
{
    unsigned int nBytes = 0;
    DRV_MEDIA_CLIENT_DCPT* pClient = (DRV_MEDIA_CLIENT_DCPT*)handle;

    if(pGblMediaDcpt->mediaAcc.writeByte == 0)
    {
        pClient->status = DRV_MEDIA_STATUS_OPERATION_ERROR;
    }
    else
    {
        if(pClient->startOffset + pClient->wrOffset < pClient->endOffset)
        {
            nBytes = 1;
        }

        if(nBytes)
        {
            (*pGblMediaDcpt->mediaAcc.writeByte)(byte);
            pClient->wrOffset ++;
            pClient->status = DRV_MEDIA_CLIENT_STATUS_READY;
        }
        else
        {
            pClient->status = DRV_MEDIA_STATUS_EOS;
        }
    }

    return nBytes;
    
}

int DRV_MEDIA_WriteFlush( const DRV_MEDIA_HANDLE handle)
{
    // DRV_MEDIA_CLIENT_DCPT* pClient = (DRV_MEDIA_CLIENT_DCPT*)handle;
    return (*pGblMediaDcpt->mediaAcc.writeEnd)();
}

// local media functions

#if defined(MEDIA_STORAGE_EEPROM)
static int _XEEFlashInit(const MEDIA_STORAGE_CONFIG* pConfigData)
{
	EEPROM_CS_IO = 1;
	EEPROM_CS_TRIS = 0;
    return XEEInit();

}
#elif defined(MEDIA_STORAGE_SPIFLASH)
static int _SPIFlashInit(const MEDIA_STORAGE_CONFIG* pConfigData)
{
    int ix;
    const MEDIA_STORAGE_PARTITION_INFO *pCurrPart, **pPart;

    if( (pConfigData->storageInfo.phyStartOffset & 0xffff) != 0 )
    {   // physical storage has to be on a 4KB boundary for the SPI Flash storage
        return -1;
    }

    // check that all partitions start on a 4 KB boundary
    pPart = pConfigData->partTable;
    for(ix = 0; ix < pConfigData->nPartitions; ix++)
    {
        pCurrPart = *pPart++;
        if( (pCurrPart->partStartOffset & 0xffff) != 0)
        {   // partition has to start on a 4KB boundary for the SPI Flash storage
            return -1;
        }
    }

	SPIFLASH_CS_IO = 1;
	SPIFLASH_CS_TRIS = 0;

    return SPIFlashInit();
}
#else
#if defined(__C32__)
static int _FlashMediaInit(const MEDIA_STORAGE_CONFIG* pConfigData)
{
    return 0;
}

static	int _FlashMediaRead(uint32_t address, uint8_t *vData, uint32_t wLen)
{
    memcpy(vData, (void*)address, wLen);
    return 0;
}
#endif  // defined(__C32__)
#endif  // #if defined(MEDIA_STORAGE_EEPROM) || defined(MEDIA_STORAGE_SPIFLASH) || defined(MEDIA_STORAGE_INTERNAL_FLASH)


#endif  // defined(MEDIA_STORAGE_EEPROM) || defined(MEDIA_STORAGE_SPIFLASH) || defined(MEDIA_STORAGE_INTERNAL_FLASH)

