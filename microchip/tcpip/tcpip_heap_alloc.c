/*******************************************************************************
  TCPIP Heap Allocation Manager

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:   tcpip_heap_alloc.c
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

#include <string.h>
#include <stdlib.h>

#include "tcpip_private.h"

// definitions


// min heap alignment
// always power of 2
#if defined(__PIC32MX__)
typedef uint32_t _heap_Align;
#elif defined(__C30__)
typedef uint16_t _heap_Align;
#endif  // defined(__PIC32MX__) || defined(__C30__)


typedef union _tag_headNode
{
    _heap_Align x;
    struct
    {
        union _tag_headNode*	next;
        size_t			        units;
    };
}_headNode;

#define	_TCPIP_HEAP_MIN_BLK_USIZE_  	2	// avoid tiny blocks having units <=  value. 

#define	_TCPIP_HEAP_MIN_BLKS_           64	// efficiency reasons, the minimum heap size that can be handled. 


#if defined(TCPIP_STACK_DRAM_DEBUG_ENABLE) && defined(TCPIP_STACK_DRAM_TRACE_ENABLE) 
    #define _TCPIP_STACK_DRAM_TRACE_ENABLE
#else
    #undef  _TCPIP_STACK_DRAM_TRACE_ENABLE
#endif

typedef struct
{
    _headNode*      _heapHead;
    _headNode*      _heapTail;                  // head and tail pointers
    size_t          _heapUnits;                 // size of the heap, units
    size_t          _heapAllocatedUnits;        // how many units allocated out there
    TCPIP_HEAP_RES  _lastHeapErr;               // last error encountered
#if defined(TCPIP_STACK_DRAM_DEBUG_ENABLE) 
    TCPIP_HEAP_NO_MEM_HANDLER _noMemHandler;    // out of memory handler
    _headNode*   _heapStart;                    // debug checkpoint     
    _headNode*   _heapEnd;                      // debug checkpoint     
#endif
#if defined(_TCPIP_STACK_DRAM_TRACE_ENABLE)
    HEAP_TRACE_ENTRY _heapTraceTbl[TCPIP_STACK_DRAM_TRACE_SLOTS];
#endif
    // alignment padding
    // _headNode _heap[0];           // the heap itself, dynamically allocated
    
}HEAP_DCPT; // descriptor of a heap


// local data
//


#if defined(SYS_DEBUG_ENABLE) && defined(TCPIP_STACK_DRAM_DEBUG_ENABLE) 
static const char* _heapFailMessage = "Heap allocation of %d bytes failed in module %d line %d\r\n";
#endif  // defined(SYS_DEBUG_ENABLE) && defined(TCPIP_STACK_DRAM_DEBUG_ENABLE) 

// local prototypes
//
#if defined(_TCPIP_STACK_DRAM_TRACE_ENABLE)
    static HEAP_TRACE_ENTRY* TCPIP_HEAP_FindEntry(HEAP_DCPT* hDcpt, int moduleId, bool addNewSlot);
    static void TCPIP_HEAP_AddToEntry(HEAP_DCPT* hDcpt, int moduleId, size_t nBytes, void* ptr);
    static void TCPIP_HEAP_RemoveFromEntry(HEAP_DCPT* hDcpt, int moduleId, size_t nBytes);
#endif  // defined(_TCPIP_STACK_DRAM_TRACE_ENABLE)

static  HEAP_DCPT*  _TcpipHeapHandleToDcpt(TCPIP_HEAP_HANDLE heapH)
{
    HEAP_DCPT* hDcpt = (HEAP_DCPT*)heapH;
    if(hDcpt && hDcpt->_heapUnits != 0)
    {
        return hDcpt;
    }
    return 0;
}
    
TCPIP_HEAP_HANDLE TCPIP_HEAP_Create(uint8_t* heapBuffer, size_t buffSize, TCPIP_HEAP_FLAGS flags, TCPIP_HEAP_RES* pRes)
{
    HEAP_DCPT*      hDcpt;
    size_t          heapSize, heapUnits, headerSize;
    uint8_t*        heapStart;
    TCPIP_HEAP_RES  res;
    

    while(true)
    {
        hDcpt =0;

        if( heapBuffer == 0 )
        {
            res = TCPIP_HEAP_RES_BUFF_ERR;
            break;
        }

        if( ((size_t)heapBuffer & (sizeof(_heap_Align)-1)) != 0 )
        {
            res = TCPIP_HEAP_RES_BUFF_ALIGN_ERR;
            break;
        }

        headerSize = ((sizeof(HEAP_DCPT) + sizeof(_headNode) - 1) / sizeof(_headNode)) * sizeof(_headNode);

        heapSize = buffSize - headerSize;

        heapUnits = heapSize / sizeof(_headNode);           // adjust to multiple of heads

        if(heapUnits < _TCPIP_HEAP_MIN_BLKS_)
        {
            res = TCPIP_HEAP_RES_BUFF_SIZE_ERR;
            break;
        }

        heapStart = heapBuffer + headerSize; 
        hDcpt = (HEAP_DCPT*)heapBuffer;
        hDcpt->_heapHead = (_headNode*)heapStart;
        hDcpt->_heapHead->units = heapUnits;
        hDcpt->_heapHead->next = 0;
        hDcpt->_heapTail = hDcpt->_heapHead;
        hDcpt->_heapUnits = heapUnits;
        hDcpt->_heapAllocatedUnits = 0;
        hDcpt->_lastHeapErr = TCPIP_HEAP_RES_OK;
#ifdef TCPIP_STACK_DRAM_DEBUG_ENABLE
        hDcpt->_heapStart = hDcpt->_heapHead;
        hDcpt->_heapEnd = hDcpt->_heapHead + heapUnits;
        hDcpt->_noMemHandler = 0;
#endif

#if defined(_TCPIP_STACK_DRAM_TRACE_ENABLE)
        {
            int ix;
            for(ix = 0; ix < sizeof(hDcpt->_heapTraceTbl)/sizeof(*hDcpt->_heapTraceTbl); ix++)
            {
                hDcpt->_heapTraceTbl[ix].moduleId = -1;    // mark it as free
            }
        }
#endif
        res = TCPIP_HEAP_RES_OK;
        break;
    }

    if(pRes)
    {
        *pRes = res;
    }

    return hDcpt;
    
}

// deallocates the heap
// NOTE: no check is done if some blocks are still in use!
int TCPIP_HEAP_Delete(TCPIP_HEAP_HANDLE heapH)
{
    HEAP_DCPT*   hDcpt = _TcpipHeapHandleToDcpt(heapH);

    if(hDcpt == 0)
    {
        return 0;
    }
    
    if(hDcpt->_heapHead)
    {
    #ifdef TCPIP_STACK_DRAM_DEBUG_ENABLE
        if(hDcpt->_heapAllocatedUnits != 0 || hDcpt->_heapHead != hDcpt->_heapStart || hDcpt->_heapHead->units != hDcpt->_heapUnits)
    #else
        if(hDcpt->_heapAllocatedUnits != 0)
    #endif
        {
            hDcpt->_lastHeapErr = TCPIP_HEAP_RES_IN_USE; 
            return 0;  //  deallocating a heap not completely de-allocated or corrupted
        }
        
        // completely deallocated
        hDcpt->_heapUnits = 0;
        return 1;
    }
    
    hDcpt->_lastHeapErr = TCPIP_HEAP_RES_IN_USE; 
    return 0; 
}


void* _TCPIP_HEAP_Malloc(TCPIP_HEAP_HANDLE heapH, size_t nBytes)
{
	_headNode	*ptr,*prev;
	size_t      nunits;
    HEAP_DCPT*  hDcpt = _TcpipHeapHandleToDcpt(heapH);


	if(nBytes == 0 || hDcpt == 0)
	{
		return 0;
	}
    
	
	nunits=(nBytes+sizeof(_headNode)-1)/sizeof(_headNode)+1;	// allocate units   
	prev=0;
    
	for(ptr = hDcpt->_heapHead; ptr != 0; prev = ptr, ptr = ptr->next)
	{
		if(ptr->units >= nunits)
		{   // found block
			if(ptr->units-nunits <= _TCPIP_HEAP_MIN_BLK_USIZE_)
			{
				nunits=ptr->units;	// get the whole block
			}

            if(ptr->units == nunits)
			{   // exact match
				if(prev)
				{
					prev->next = ptr->next;
				}
				else
				{
					hDcpt->_heapHead = ptr->next;
                    prev = hDcpt->_heapHead;
				}

                if(hDcpt->_heapTail == ptr)
                {
                    hDcpt->_heapTail = prev;
                }
			}
			else
			{   // larger than we need
				ptr->units -= nunits;
				ptr += ptr->units;
				ptr->units = nunits;
			}

            hDcpt->_heapAllocatedUnits += nunits;
            return ptr+1;
		}
	}

    hDcpt->_lastHeapErr = TCPIP_HEAP_RES_NO_MEM;
    return 0;
}

void* _TCPIP_HEAP_Calloc(TCPIP_HEAP_HANDLE heapH, size_t nElems, size_t elemSize)
{
    void* pBuff = _TCPIP_HEAP_Malloc(heapH, nElems * elemSize);
    if(pBuff)
    {
        memset(pBuff, 0, nElems * elemSize);
    }

    return pBuff;

}

int _TCPIP_HEAP_Free(TCPIP_HEAP_HANDLE heapH, const void* pBuff)
{  
	_headNode	*hdr,*ptr;
    int         fail;
    size_t      freedUnits;
    HEAP_DCPT*  hDcpt = _TcpipHeapHandleToDcpt(heapH);

    if(pBuff == 0 || hDcpt == 0)
	{
        return 0;
    }

    ptr = (_headNode*)pBuff-1;
    
#ifdef TCPIP_STACK_DRAM_DEBUG_ENABLE
    if(ptr < hDcpt->_heapStart || ptr+ptr->units > hDcpt->_heapEnd)
    {
        hDcpt->_lastHeapErr = TCPIP_HEAP_RES_PTR_ERR;   // not one of our pointers!!!
        return 0;
    }
#endif
    
    freedUnits = ptr->units;
    
    fail = 0;
    
    if(!hDcpt->_heapHead)
    {
        ptr->next=0;
        hDcpt->_heapHead = hDcpt->_heapTail = ptr;
    }
    else if(ptr < hDcpt->_heapHead)
    {   // put it in front
        if(ptr+ptr->units == hDcpt->_heapHead)
        {   // compact
            ptr->units += hDcpt->_heapHead->units;
            ptr->next = hDcpt->_heapHead->next;
            if(hDcpt->_heapTail == hDcpt->_heapHead)
            {
                hDcpt->_heapTail = ptr;
            }
        }
        else
        {
            ptr->next = hDcpt->_heapHead;
        }
        hDcpt->_heapHead = ptr;    // new head
    }
    else if(ptr > hDcpt->_heapTail)
    {   // append tail
        if(hDcpt->_heapTail + hDcpt->_heapTail->units == ptr)
        {   // compact
            hDcpt->_heapTail->units += ptr->units;
        }
        else
        {
            hDcpt->_heapTail->next = ptr;
            ptr->next = 0;
            hDcpt->_heapTail = ptr;
        }
    }
    else
    {   // somewhere in the middle
        fail = 1;
        for(hdr = hDcpt->_heapHead; hdr != 0; hdr = hdr->next)
        {
            if(hdr<ptr && ptr<hdr->next)
            {   // found a place
                if(ptr+ptr->units == hdr->next)
                {   // compact right
                    ptr->units += hdr->next->units;
                    ptr->next = hdr->next->next;
                    if(hDcpt->_heapTail == hdr->next)
                    {
                        hDcpt->_heapTail = ptr;
                    }
                }
                else
                {
                #ifdef TCPIP_STACK_DRAM_DEBUG_ENABLE
                    if(ptr+ptr->units > hdr->next)
                    {
                        break;  // corrupted pointer!!!
                    }
                #endif
                    ptr->next = hdr->next;
                }
                
                // compact left
                if(hdr+hdr->units == ptr)
                {
                    hdr->units += ptr->units;
                    hdr->next = ptr->next;
                    if(hDcpt->_heapTail == ptr)
                    {
                        hDcpt->_heapTail = hdr;
                    }
                }
                else
                {
                #ifdef TCPIP_STACK_DRAM_DEBUG_ENABLE
                    if(hdr+hdr->units > ptr)
                    {
                        break;      // corrupted pointer!!!
                    }
                #endif
                    hdr->next = ptr;
                }

                fail = 0;   // everything OK
                break;                
            }
        }
        
    }

    if(fail)
    {
        hDcpt->_lastHeapErr = TCPIP_HEAP_RES_PTR_ERR;   // corrupted pointer!!!
        return 0;
    }
    
    hDcpt->_heapAllocatedUnits -= freedUnits;
    return freedUnits;
}


size_t TCPIP_HEAP_Size(TCPIP_HEAP_HANDLE heapH)
{
    HEAP_DCPT*  hDcpt = _TcpipHeapHandleToDcpt(heapH);

    return hDcpt ? hDcpt->_heapUnits * sizeof(_headNode) : 0;   
}

size_t TCPIP_HEAP_FreeSize(TCPIP_HEAP_HANDLE heapH)
{
    HEAP_DCPT*  hDcpt = _TcpipHeapHandleToDcpt(heapH);

    return hDcpt ? (hDcpt->_heapUnits - hDcpt->_heapAllocatedUnits) * sizeof(_headNode) : 0;   
}


size_t TCPIP_HEAP_MaxSize(TCPIP_HEAP_HANDLE heapH)
{
	_headNode	*ptr;
	size_t      max_nunits;
    HEAP_DCPT*  hDcpt = _TcpipHeapHandleToDcpt(heapH);
    
    if(hDcpt == 0)
    {
        return 0;
    }

    max_nunits = 0;
    
	for(ptr = hDcpt->_heapHead; ptr != 0; ptr = ptr->next)
    {
        if(ptr->units >= max_nunits)
        {   // found block
            max_nunits = ptr->units;
        }
    }
    
    return max_nunits * sizeof(_headNode);   

}


TCPIP_HEAP_RES TCPIP_HEAP_LastError(TCPIP_HEAP_HANDLE heapH)
{
    TCPIP_HEAP_RES  res;
    HEAP_DCPT*  hDcpt = _TcpipHeapHandleToDcpt(heapH);

    if(hDcpt)
    {
        res = hDcpt->_lastHeapErr;
        hDcpt->_lastHeapErr = TCPIP_HEAP_RES_OK;
    }
    else
    {
        res = 0;
    }

    return res;
}

#if defined(TCPIP_STACK_DRAM_DEBUG_ENABLE) 
void* _TCPIP_HEAP_MallocDebug(TCPIP_HEAP_HANDLE heapH, size_t nBytes, int moduleId, int lineNo)
{
    HEAP_DCPT*  hDcpt = _TcpipHeapHandleToDcpt(heapH);

    if(hDcpt == 0)
    {
        return 0;
    }

    void* ptr = _TCPIP_HEAP_Malloc(heapH, nBytes);
    if(ptr == 0)
    {
        SYS_ERROR_PRINT(SYS_ERROR_WARN, _heapFailMessage, nBytes, moduleId, lineNo);
        if(hDcpt->_noMemHandler != 0)
        {
            (*hDcpt->_noMemHandler)(heapH, nBytes, moduleId, lineNo);
        }
    }
#if defined(_TCPIP_STACK_DRAM_TRACE_ENABLE)
    TCPIP_HEAP_AddToEntry(hDcpt, moduleId, nBytes, ptr);
#endif  // defined(_TCPIP_STACK_DRAM_TRACE_ENABLE)
    return ptr;
}

void* _TCPIP_HEAP_CallocDebug(TCPIP_HEAP_HANDLE heapH, size_t nElems, size_t elemSize, int moduleId, int lineNo)
{
    HEAP_DCPT*  hDcpt = _TcpipHeapHandleToDcpt(heapH);
    if(hDcpt == 0)
    {
        return 0;
    }

    size_t nBytes = nElems * elemSize;
    void* ptr = _TCPIP_HEAP_Calloc(heapH, nElems, elemSize);

    if(ptr == 0)
    {
        SYS_ERROR_PRINT(SYS_ERROR_WARN, _heapFailMessage, nBytes, moduleId, lineNo);
        if(hDcpt->_noMemHandler != 0)
        {
            (*hDcpt->_noMemHandler)(heapH, nBytes, moduleId, lineNo);
        }
    }
#if defined(_TCPIP_STACK_DRAM_TRACE_ENABLE)
    TCPIP_HEAP_AddToEntry(hDcpt, moduleId, nBytes, ptr);
#endif  // defined(_TCPIP_STACK_DRAM_TRACE_ENABLE)
    return ptr;
}

int _TCPIP_HEAP_FreeDebug(TCPIP_HEAP_HANDLE heapH,  const void* pBuff, int moduleId)
{
    HEAP_DCPT*  hDcpt = _TcpipHeapHandleToDcpt(heapH);
    if(hDcpt == 0)
    {
        return 0;
    }
    int nUnits = _TCPIP_HEAP_Free(heapH, pBuff);

#if defined(_TCPIP_STACK_DRAM_TRACE_ENABLE)
    if(nUnits)
    {
        TCPIP_HEAP_RemoveFromEntry(hDcpt, moduleId, nUnits);
    }
#endif  // defined(_TCPIP_STACK_DRAM_TRACE_ENABLE)
    return nUnits;
}

TCPIP_HEAP_RES TCPIP_HEAP_SetNoMemHandler(TCPIP_HEAP_HANDLE heapH, TCPIP_HEAP_NO_MEM_HANDLER handler)
{
    HEAP_DCPT*  hDcpt = _TcpipHeapHandleToDcpt(heapH);

    if(hDcpt)
    {
        hDcpt->_noMemHandler = handler;
    }

    return TCPIP_HEAP_RES_OK;

}

#endif  // defined(TCPIP_STACK_DRAM_DEBUG_ENABLE) 

#if defined(_TCPIP_STACK_DRAM_TRACE_ENABLE)
static HEAP_TRACE_ENTRY* TCPIP_HEAP_FindEntry(HEAP_DCPT* hDcpt, int moduleId, bool addNewSlot)
{
    int ix;
    HEAP_TRACE_ENTRY    *freeEntry,*pEntry;

    freeEntry = 0;
    for(ix = 0, pEntry = hDcpt->_heapTraceTbl; ix < sizeof(hDcpt->_heapTraceTbl)/sizeof(*hDcpt->_heapTraceTbl); ix++, pEntry++)
    {
        if(pEntry->moduleId == moduleId)
        {
            return pEntry;
        }
        else if(addNewSlot && freeEntry == 0 && pEntry->moduleId < 0)
        {
            freeEntry = pEntry;
        }
    }

    if(freeEntry)
    {
        memset(freeEntry, 0x0, sizeof(*freeEntry));
        freeEntry->moduleId = moduleId;
    }

    return freeEntry;
}

static void TCPIP_HEAP_AddToEntry(HEAP_DCPT* hDcpt, int moduleId, size_t nBytes, void* ptr)
{
    HEAP_TRACE_ENTRY *pEntry = TCPIP_HEAP_FindEntry(hDcpt, moduleId, true);

    if(pEntry)
    {
        if(ptr)
        {   // successful
            _headNode	*hPtr = (_headNode*)ptr-1;
            nBytes = hPtr->units * sizeof(_headNode);
            pEntry->totAllocated += nBytes;
            pEntry->currAllocated += nBytes;
        }
        else
        {
            pEntry->totFailed += nBytes;
            if(nBytes > pEntry->maxFailed)
            {
                pEntry->maxFailed = nBytes;
            }
        }
    }
}

static void TCPIP_HEAP_RemoveFromEntry(HEAP_DCPT* hDcpt, int moduleId, size_t nUnits)
{
    HEAP_TRACE_ENTRY *pEntry = TCPIP_HEAP_FindEntry(hDcpt, moduleId, false);

    if(pEntry)
    {
        pEntry->currAllocated -= nUnits * sizeof(_headNode);
    }
}

bool TCPIP_HEAP_TraceGetEntry(TCPIP_HEAP_HANDLE heapH, int entryIx, HEAP_TRACE_ENTRY* tEntry)
{
    HEAP_TRACE_ENTRY *pEntry;
    HEAP_DCPT*  hDcpt = _TcpipHeapHandleToDcpt(heapH);

    if(hDcpt)
    {
        if(entryIx < sizeof(hDcpt->_heapTraceTbl)/sizeof(*hDcpt->_heapTraceTbl))
        {   // valid index
            pEntry = hDcpt->_heapTraceTbl + entryIx;
            if(pEntry->moduleId >= 0)
            {
                *tEntry = *pEntry;
                return true;
            }
        }
    }

    return false;
}

int TCPIP_HEAP_TraceGetEntriesNo(TCPIP_HEAP_HANDLE heapH, bool getUsed)
{
    HEAP_DCPT*  hDcpt = _TcpipHeapHandleToDcpt(heapH);

    if(hDcpt == 0)
    {
        return 0;
    }

    if(getUsed)
    {
        HEAP_TRACE_ENTRY *pEntry;
        int ix, nUsed;

        nUsed = 0;

        for(ix = 0, pEntry = hDcpt->_heapTraceTbl; ix < sizeof(hDcpt->_heapTraceTbl)/sizeof(*hDcpt->_heapTraceTbl); ix++, pEntry++)
        {
            if(pEntry->moduleId >= 0)
            {
                nUsed++;
            }
        }

        return nUsed;
    }

    return sizeof(hDcpt->_heapTraceTbl)/sizeof(*hDcpt->_heapTraceTbl);
}


#else

bool TCPIP_HEAP_TraceGetEntry(TCPIP_HEAP_HANDLE heapH, int entryIx, HEAP_TRACE_ENTRY* tEntry)
{
    return false;
}

int TCPIP_HEAP_TraceGetEntriesNo(TCPIP_HEAP_HANDLE heapH, bool used)
{
    return 0;
}

#endif  // defined(_TCPIP_STACK_DRAM_TRACE_ENABLE)


