/*******************************************************************************
  tcpip PIC32 MAC events implementation

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:   mac_events.c
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

#include "tcpip_private.h"


#include "system/drivers/drv_media.h"
#include "tcpip/tcpip_storage.h"

#if defined(TCPIP_STACK_USE_STORAGE)

// data saved into the NVM storage per interface
typedef struct __attribute__((__packed__))
{
    TCPIP_NET_STORAGE   entryData;          // the actual configuration data saved
                                            // differentiation per interfaces is done using the macId field.
}TCPIP_NVM_STORAGE_ENTRY;


// header structure for validating the TCPIP_NVM_STORAGE_ENTRY data storage in EEPROM/Flash
typedef struct __attribute__((__packed__))
{
	int16_t             hdrLength;          // Size of the header, in bytes
                                            // The length of the label is variable so is the header length
    uint16_t            hdrChecksum;        // header only checksum

	int16_t             nEntries;           // Number of configuration entries in the storage
	int16_t             entryLength;        // Length in bytes of a configuration entry
                                            // Note: all entries have to have the same size for
                                            // configuration to be valid!
	uint16_t            storageChecksum; 	// Checksum of the contents.
                                            // Validates the storage contents
#if _TCPIP_STORAGE_USER_LABEL_SIZE > 0
    uint8_t             storageLabel[_TCPIP_STORAGE_USER_LABEL_SIZE];    // size of the proprietary data field, bytes
#endif

   TCPIP_NVM_STORAGE_ENTRY    entry[0];           // actual storage
   
}NVM_STORAGE_HDR;

typedef void (*pStorageInit)(void);     // init function		
typedef	void (*pStorageReadArray)(uint32_t dwAddress, uint8_t *vData, uint32_t wLen);
typedef void (*pStorageBeginWrite)(uint32_t dwAddr);
typedef void (*pStorageWriteArray)(uint8_t *vData, uint32_t wLen);

typedef enum
{
    NVM_ST_FLAG_VALID   = 0x01,     // has been validated
    NVM_ST_FLAG_DIRTY   = 0x02,     // dirty, hasn't been flushed
    
}NVM_STORAGE_FLAGS;



// identifies a storage medium
typedef struct
{
    // storage dynamic data
    DRV_MEDIA_HANDLE        mediaH;     // handle to opened media
    NVM_STORAGE_FLAGS       flags;          // valid, hasn't been flushed, etc.
    NVM_STORAGE_HDR         sHdr;           // actual data storage header
}NVM_STORAGE_DCPT;    
    

// the one and only NVM storage descriptor
// for now we support only one
static NVM_STORAGE_DCPT  _nvmStorageDcpt = { 0 }; 

static NVM_STORAGE_DCPT*  _hStorage = 0;    // the one and only storage handle  


// local functions
// 
static bool _TCPIPStorageVerifyLoad(NVM_STORAGE_DCPT* sDcpt);
static int _TCPIPStorageFindEntryIx(NVM_STORAGE_DCPT* sDcpt, const TCPIP_NVM_STORAGE_ENTRY* pcEntry);
static bool _TCPIPStorageWriteEntry(NVM_STORAGE_DCPT* sDcpt, int entryIx, const TCPIP_NVM_STORAGE_ENTRY* pcEntry);
static void _TCPIPStorageUpdateHdr(NVM_STORAGE_DCPT* sDcpt);
static uint16_t _TCPIPStorageCheckSum(NVM_STORAGE_DCPT* sDcpt);

static __inline__ void __attribute__((always_inline)) _TCPIPStorageInit(NVM_STORAGE_DCPT* sDcpt)
{
    memset(sDcpt, 0x0, sizeof(*sDcpt));
    sDcpt->mediaH = DRV_MEDIA_HANDLE_INVALID;
    sDcpt->sHdr.hdrLength = sizeof(sDcpt->sHdr);
        
}
static __inline__ void __attribute__((always_inline)) _TCPIPStorageInvalidate(NVM_STORAGE_DCPT* sDcpt)
{
    sDcpt->flags &= ~NVM_ST_FLAG_VALID;
    sDcpt->sHdr.nEntries = sDcpt->sHdr.entryLength = 0;        
}

static __inline__ bool __attribute__((always_inline)) _TCPIPStorageIsValid(NVM_STORAGE_DCPT* sDcpt)
{
    return (sDcpt->flags & NVM_ST_FLAG_VALID) != 0;
}

static __inline__ void __attribute__((always_inline)) _TCPIPStorageValidate(NVM_STORAGE_DCPT* sDcpt)
{
    sDcpt->flags |= NVM_ST_FLAG_VALID;
}

static __inline__ void __attribute__((always_inline)) _TCPIPStorageReadEntry(NVM_STORAGE_DCPT* sDcpt, int offset, TCPIP_NVM_STORAGE_ENTRY* pEntry)
{
    DRV_MEDIA_ReadSetOffset(sDcpt->mediaH, offset);
    DRV_MEDIA_Read(sDcpt->mediaH, (uint8_t*)pEntry, sizeof(*pEntry));
}

static __inline__ bool __attribute__((always_inline)) _TCPIPStorageMatchEntries(const TCPIP_NVM_STORAGE_ENTRY* pE1, const TCPIP_NVM_STORAGE_ENTRY* pE2)
{
    return pE1->entryData.macId == pE2->entryData.macId;
}

// implementation
//


// inits a storage
// storageId will allow selection between multiple storages
// not supported right now
// returns false if storage could not be opened
bool TCPIP_STORAGE_Init(int storageId)
{
    if(_hStorage == 0)
    {
        _hStorage = &_nvmStorageDcpt;
        _TCPIPStorageInit(_hStorage);
    }
    return true;
}


// de-inits a storage
// storageId will allow selection between multiple storages
// not supported right now
// returns true if de-init possible, false if clients still having valid handles
bool TCPIP_STORAGE_DeInit(int storageId)
{
    if(_hStorage != 0)
    {
        if(_hStorage->mediaH == DRV_MEDIA_HANDLE_INVALID)
        {   // all/only client closed
            _TCPIPStorageInit(_hStorage);
            _hStorage = 0;
            return true;
        }
        return false;
    }
    return true;
}

// opens a storage and returns a handle
// storageId will allow selection between multiple storages
// not supported right now
// refresh instructs to read the storage (default).
// no refresh is useful for just getting a handle to erase, for example 
// return 0 if storage could not be opened
// NOTE: only one client supported for now!
TCPIP_STORAGE_HANDLE TCPIP_STORAGE_Open(int storageId, bool refresh)
{
    DRV_MEDIA_IO_INTENT     ioIntent;    
    DRV_MEDIA_HANDLE        mediaH;

    if(_hStorage)
    {
        if(_hStorage->mediaH == DRV_MEDIA_HANDLE_INVALID)
        {   // no client present
            ioIntent = DRV_MEDIA_IO_INTENT_READWRITE | DRV_MEDIA_IO_INTENT_BLOCKING | DRV_MEDIA_IO_INTENT_EXCLUSIVE;
            mediaH = DRV_MEDIA_Open(DRV_MEDIA_DEFAULT, "tcpip_storage", ioIntent);
            if(mediaH != DRV_MEDIA_HANDLE_INVALID)
            {
                _hStorage->mediaH = mediaH;
                if(refresh)
                {
                    TCPIP_STORAGE_Refresh(_hStorage);
                }
                return _hStorage;
            }
        }
    }

    return TCPIP_STORAGE_HANDLE_INVALID;
}

// closes the storage
void TCPIP_STORAGE_Close(TCPIP_STORAGE_HANDLE hS)
{
    NVM_STORAGE_DCPT* sDcpt = (NVM_STORAGE_DCPT*)hS;

    if(sDcpt->mediaH != DRV_MEDIA_HANDLE_INVALID)
    {   // release the media
        DRV_MEDIA_Close(sDcpt->mediaH);
        sDcpt->mediaH = DRV_MEDIA_HANDLE_INVALID;
    }
}

// copies the current label existent in the storage
// the less of TCPIP_STORAGE_USER_LABEL_SIZE and bufferSize is copied to the buffer
// returns the number of bytes copied
int TCPIP_STORAGE_GetLabel(TCPIP_STORAGE_HANDLE hS, uint8_t* buffer, uint16_t bufferSize)
{
    int nBytes = 0;
#if _TCPIP_STORAGE_USER_LABEL_SIZE > 0
    NVM_STORAGE_DCPT* sDcpt = (NVM_STORAGE_DCPT*)hS;
    
    nBytes = sizeof(sDcpt->sHdr.storageLabel);
    if(nBytes > bufferSize)
    {
        nBytes = bufferSize;
    }
    memcpy(buffer, sDcpt->sHdr.storageLabel, nBytes);
#endif

    return nBytes;
}

// returns the current label size
// this function is useful when variable length labels will be supported
// right now this feature is not implemented
// and the function returns a build time value 
int TCPIP_STORAGE_GetLabelSize(TCPIP_STORAGE_HANDLE hS)
{
#if _TCPIP_STORAGE_USER_LABEL_SIZE > 0
    NVM_STORAGE_DCPT* sDcpt = (NVM_STORAGE_DCPT*)hS;
    
    return sizeof(sDcpt->sHdr.storageLabel);
#endif

    return 0;
}


// sets the current label to be used in write or verification
// the less of TCPIP_STORAGE_USER_LABEL_SIZE and labelSize are used from pLabel (none if 0)
// returns the number of bytes copied
int TCPIP_STORAGE_SetLabel(TCPIP_STORAGE_HANDLE hS, const uint8_t* pLabel, int labelSize)
{
    int nBytes = 0;
#if _TCPIP_STORAGE_USER_LABEL_SIZE > 0
    NVM_STORAGE_DCPT* sDcpt = (NVM_STORAGE_DCPT*)hS;

    nBytes = sizeof(sDcpt->sHdr.storageLabel);
    if(nBytes > labelSize)
    {
        nBytes = labelSize;
    }
    memset(sDcpt->sHdr.storageLabel, 0, sizeof(sDcpt->sHdr.storageLabel));
    memcpy(sDcpt->sHdr.storageLabel, pLabel, nBytes);
#endif
    return nBytes;
}


// forces a storage refresh 
// repeats the action that is done at Open time, for the 1st caller
// returns true if a valid configuration found
bool TCPIP_STORAGE_Refresh(TCPIP_STORAGE_HANDLE hS)
{
    NVM_STORAGE_HDR stgHdr;
    unsigned int    nBytes;
    uint16_t        chksum1, chksum2;        
    NVM_STORAGE_DCPT* sDcpt = (NVM_STORAGE_DCPT*)hS;

    // read the header
    DRV_MEDIA_ReadSetOffset(sDcpt->mediaH, 0);
    nBytes = DRV_MEDIA_Read(sDcpt->mediaH, (uint8_t*)&stgHdr, sizeof(sDcpt->sHdr));
    if(nBytes == sizeof(sDcpt->sHdr))
    {   // sanity check
        if(stgHdr.hdrLength == sDcpt->sHdr.hdrLength)
        {   // size match
            chksum1 = stgHdr.hdrChecksum;
            stgHdr.hdrChecksum = 0;
            chksum2 = TCPIP_Helper_CalcIPChecksum((uint8_t*)&stgHdr, sizeof(stgHdr), 0);
            if(chksum1 == chksum2)
            {   // checksum match
                stgHdr.hdrChecksum = chksum1;
                if(stgHdr.nEntries > 0 && stgHdr.entryLength == sizeof(TCPIP_NVM_STORAGE_ENTRY))
                {   // so far so good: the storage contains records of this size/build
                    memcpy(&sDcpt->sHdr, &stgHdr, sizeof(stgHdr));
                    if(_TCPIPStorageVerifyLoad(sDcpt))
                    {
                        _TCPIPStorageValidate(sDcpt);
                        return true;
                    }
                }
            }
        }
    }
    
    _TCPIPStorageInvalidate(sDcpt);
    return false;
}



// returns true if valid storage: checksum OK and entries 
bool TCPIP_STORAGE_IsValid(TCPIP_STORAGE_HANDLE hS)
{
    return _TCPIPStorageIsValid((NVM_STORAGE_DCPT*)hS);
}

// returns the no of entries 
int TCPIP_STORAGE_Entries(TCPIP_STORAGE_HANDLE hS)
{
    NVM_STORAGE_DCPT* sDcpt = (NVM_STORAGE_DCPT*)hS;
    if(_TCPIPStorageIsValid(sDcpt))
    {
        return sDcpt->sHdr.nEntries;
    }

    return 0;
}

// returns the size of an entry
// Note: all the storage entries have the same size! 
int TCPIP_STORAGE_EntryLength(TCPIP_STORAGE_HANDLE hS)
{
    NVM_STORAGE_DCPT* sDcpt = (NVM_STORAGE_DCPT*)hS;
    if(_TCPIPStorageIsValid(sDcpt))
    {
        return sDcpt->sHdr.entryLength;
    }

    return 0;
}

// erases/initializes the storage, marks it invalid
void TCPIP_STORAGE_Erase(TCPIP_STORAGE_HANDLE hS)
{
    NVM_STORAGE_HDR eraseHdr;
    NVM_STORAGE_DCPT* sDcpt = (NVM_STORAGE_DCPT*)hS;

    memset(&eraseHdr, 0xff, sizeof(eraseHdr));
    DRV_MEDIA_WriteSetOffset(sDcpt->mediaH, 0);
    DRV_MEDIA_Write(sDcpt->mediaH, (uint8_t*)&eraseHdr, sizeof(eraseHdr));
    
    _TCPIPStorageInvalidate(sDcpt);     // no longer valid

}


// writes an entry and optionally verifies that everything is OK
// returns true if success, false otherwise
// The entries are processed by the interface (MAC) type. 
bool TCPIP_STORAGE_WriteEntry(TCPIP_STORAGE_HANDLE hS, TCPIP_NET_HANDLE netH, bool verify)
{
    int replaceIx, entryIx;
    TCPIP_NVM_STORAGE_ENTRY stgEntry;
    NVM_STORAGE_DCPT* sDcpt = (NVM_STORAGE_DCPT*)hS;
    
    if(_TCPIPStackHandleToNet(netH) == 0)
    {   // sanity check it's a valid interface
       return false;
    }
    
    if(_TCPIPStorageIsValid(sDcpt))
    {   // adding an entry
        if(sDcpt->sHdr.entryLength != sizeof(TCPIP_NVM_STORAGE_ENTRY))
        {   // make sure we don't write into the wrong storage
            // that hasn't been erased if it's inconsistent with current build/save
            return false;
        }
    }

    _TCPIPStackCpyNetIfToStorageEntry(&stgEntry.entryData, netH);
    replaceIx = _TCPIPStorageFindEntryIx(sDcpt, &stgEntry);
    if(replaceIx == -1)
    {   // non-existent; add tail new entry
        entryIx = sDcpt->sHdr.nEntries; 
    }
    else
    {   // existent entry, replace
        entryIx = replaceIx;
    }
    
    if(!_TCPIPStorageWriteEntry(sDcpt, entryIx, &stgEntry))
    {   // failed
        return false;
    }

    if(replaceIx == -1)
    {   // just added something
        sDcpt->sHdr.entryLength = sizeof(TCPIP_NVM_STORAGE_ENTRY); // just in case this is the 1st valid entry
        sDcpt->sHdr.nEntries++;
    }
    
    _TCPIPStorageUpdateHdr(sDcpt);
    
    if(verify)
    {
        return TCPIP_STORAGE_Refresh(sDcpt);
    }

    _TCPIPStorageValidate(sDcpt);      // assume is valid now
    return true;

}

// reads an entry with the required index into the network info pointed by netH.
// returns true if such index found
bool TCPIP_STORAGE_ReadEntryIx(TCPIP_STORAGE_HANDLE hS, int entryIx, TCPIP_NET_HANDLE netH)
{
    NVM_STORAGE_DCPT* sDcpt = (NVM_STORAGE_DCPT*)hS;

    if( entryIx < sDcpt->sHdr.nEntries)  
    {   // valid entry
        TCPIP_NVM_STORAGE_ENTRY stgEntry;
        int offset = sizeof(NVM_STORAGE_HDR) + entryIx * sizeof(TCPIP_NVM_STORAGE_ENTRY);
        _TCPIPStorageReadEntry(sDcpt, offset, &stgEntry);
        _TCPIPStackCpyStorageEntryToNetIf(netH, &stgEntry.entryData);
        return true;
    }
    return false;
}

// finds a reference entry 
// returns the entry index (0 -  TCPIP_STORAGE_Entries()) if such entry found and the MAC id matches
// or -1 if not found
// chkFlags tells what is checked to have a match
int TCPIP_STORAGE_FindEntry(TCPIP_STORAGE_HANDLE hS, TCPIP_NET_HANDLE refNetH)
{
    TCPIP_NVM_STORAGE_ENTRY stgEntry;
    _TCPIPStackCpyNetIfToStorageEntry(&stgEntry.entryData, refNetH);
    return _TCPIPStorageFindEntryIx((NVM_STORAGE_DCPT*)hS, &stgEntry);
}

// helper to store a network interface configuration
// on the external storage
// returns true if succeeded
// Note: has to be called only AFTER the stack was initialized
// i.e. the call to the TCPIP_STACK_Init()
// otherwise the TCPIP_STORAGE_Init() must be called
bool TCPIP_STORAGE_SaveNetConfig(TCPIP_STORAGE_HANDLE hS, TCPIP_NET_HANDLE netH, bool verify)
{
        return TCPIP_STORAGE_WriteEntry(hS, netH, verify);
}

// helper to store the whole stack configuration
// on the external storage
// returns true if succeeded
// Note: has to be called only AFTER the stack was initialized
// i.e. the call to the TCPIP_STACK_Init()
// otherwise the TCPIP_STORAGE_Init() must be called
bool TCPIP_STORAGE_SaveStackConfig(TCPIP_STORAGE_HANDLE hS, bool verify)
{
    int netIx;
    int netNo = TCPIP_STACK_NetworksNo();  // all existent interfaces

    for(netIx = 0; netIx < netNo; netIx++)
    {
        if(!TCPIP_STORAGE_WriteEntry(hS, TCPIP_STACK_IxToNet(netIx), false))
        {
            return false;
        }
    }

    if(verify)
    {
        return TCPIP_STORAGE_Refresh(hS);
    }

    return true;
}

// This is a helper function for initializing a network interface structure that 
// can be later on saved to storage.
// hNet -  master interface handle to use
// buffer - buffer to create network information into
// buffSize    - size of the buffer
// copyInfo    - if true, then the master interface info is copied to the created
//               network info
//               else the new network info is zeroed out  
// Returns: A network handle that can be used to set/get network interface
//          parameters.
//          0 if the supplied buffer is not large enough to hold the interface associated info
//          (see TCPIP_STORAGE_NetConfigSize()) or the network interface does not exist 


TCPIP_NET_HANDLE TCPIP_STORAGE_CreateNetConfig(TCPIP_NET_HANDLE hNet, uint8_t* buffer, int buffSize, bool copyInfo)
{
    TCPIP_NET_IF* pNewIf;
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNet(hNet);
    if(!pNetIf || buffSize < sizeof(*pNetIf) )
    {
        return 0;
    }

    pNewIf = (TCPIP_NET_IF*)buffer;
    
    if(copyInfo)
    {
        memcpy(pNewIf, pNetIf, sizeof(*pNetIf));
    }
    else
    {
        memset(pNewIf, 0x0, sizeof(*pNetIf));
    }
    
    // fake interface enabled so that all managing functions work OK
    pNewIf->Flags.bInterfaceEnabled = true;

    return pNewIf;
}

// Returns: The size in bytes required to store a complete set of the corresponding network interface
//          parameters.
//          0 if the supplied interface does not exist;
int TCPIP_STORAGE_NetConfigSize(TCPIP_NET_HANDLE hNet)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNet(hNet);
    if(pNetIf)
    {
        // constant for now; can change later
        return sizeof(*pNetIf);
    }

    return 0;
    
}

// local functions
// 

// checks consistency of a storage payload
// the header is supposed to be already validated 
static bool _TCPIPStorageVerifyLoad(NVM_STORAGE_DCPT* sDcpt)
{

    uint16_t chkSum = _TCPIPStorageCheckSum(sDcpt);

    return chkSum == sDcpt->sHdr.storageChecksum;
}

// finds the index of a specific configuration in the storage
// returns the entry ix or, if not found, -1 
static int _TCPIPStorageFindEntryIx(NVM_STORAGE_DCPT* sDcpt, const TCPIP_NVM_STORAGE_ENTRY* pcEntry)
{
    int ix;
    TCPIP_NVM_STORAGE_ENTRY   stgEntry;
    int offset = sizeof(NVM_STORAGE_HDR);
    
    for(ix = 0; ix < sDcpt->sHdr.nEntries; ix++)
    {
        _TCPIPStorageReadEntry(sDcpt, offset, &stgEntry);
        if(_TCPIPStorageMatchEntries(&stgEntry, pcEntry))
        {   // found it
            return ix;
        }
        offset += sizeof(TCPIP_NVM_STORAGE_ENTRY);
    }

    return -1;
}

// writes a (new) entry at the entryIx position
static bool _TCPIPStorageWriteEntry(NVM_STORAGE_DCPT* sDcpt, int entryIx, const TCPIP_NVM_STORAGE_ENTRY* pcEntry)
{
    int nBytes;
    int offset = sizeof(NVM_STORAGE_HDR) + entryIx * sizeof(TCPIP_NVM_STORAGE_ENTRY);
    
    if(DRV_MEDIA_WriteSetOffset(sDcpt->mediaH, offset) == DRV_MEDIA_CLIENT_STATUS_READY)
    {
        nBytes = DRV_MEDIA_Write(sDcpt->mediaH, (uint8_t*)pcEntry, sizeof(TCPIP_NVM_STORAGE_ENTRY));
        if(nBytes == sizeof(TCPIP_NVM_STORAGE_ENTRY))
        {
            return true;
        }
    }

    return false;
}

// writes the updated header info
// it assumes some entries have changed
static void _TCPIPStorageUpdateHdr(NVM_STORAGE_DCPT* sDcpt)
{    
    sDcpt->sHdr.storageChecksum = _TCPIPStorageCheckSum(sDcpt);
    sDcpt->sHdr.hdrChecksum = 0;
    sDcpt->sHdr.hdrChecksum = TCPIP_Helper_CalcIPChecksum((uint8_t*)&sDcpt->sHdr, sizeof(sDcpt->sHdr), 0);

    DRV_MEDIA_WriteSetOffset(sDcpt->mediaH, 0);
    DRV_MEDIA_Write(sDcpt->mediaH, (uint8_t*)&sDcpt->sHdr, sizeof(sDcpt->sHdr));

}

// returns the storage of a VALID header storage
static uint16_t _TCPIPStorageCheckSum(NVM_STORAGE_DCPT* sDcpt)
{
    uint8_t  chksumBuff[40] __attribute__((aligned));
                            // 2 multiple size buffer!
                            // the exact size does not matter, but it has to be even
                            // and aligned in order to correctly calculate the check sum
                            // 
    int storageSz = sDcpt->sHdr.nEntries * sizeof(TCPIP_NVM_STORAGE_ENTRY);

    int nBuffs = storageSz/sizeof(chksumBuff);
    int rBytes = storageSz - nBuffs * sizeof(chksumBuff);

    int ix;
    TCPIP_UINT32_VAL chkSum;
    int offset = sizeof(NVM_STORAGE_HDR);
    
    chkSum.Val = 0;
    
    for(ix = 0; ix < nBuffs; ix++)
    {
        DRV_MEDIA_ReadSetOffset(sDcpt->mediaH, offset);
        DRV_MEDIA_Read(sDcpt->mediaH, chksumBuff, sizeof(chksumBuff));
        offset += sizeof(chksumBuff);
        chkSum.Val += ~TCPIP_Helper_CalcIPChecksum(chksumBuff, sizeof(chksumBuff), 0);
    }

    if(rBytes)
    {
        DRV_MEDIA_ReadSetOffset(sDcpt->mediaH, offset);
        DRV_MEDIA_Read(sDcpt->mediaH, chksumBuff, rBytes);
        chkSum.Val += ~TCPIP_Helper_CalcIPChecksum(chksumBuff, rBytes, 0);
    }

    // Perform the end-around carry final adjustment
	chkSum.Val = (uint32_t)chkSum.w[0] + (uint32_t)chkSum.w[1];
    chkSum.Val = (uint32_t)chkSum.w[0] + (uint32_t)chkSum.w[1];
    chkSum.w[0] += chkSum.w[1];
    return ~chkSum.w[0];
}

#endif  // defined(TCPIP_STACK_USE_STORAGE)

