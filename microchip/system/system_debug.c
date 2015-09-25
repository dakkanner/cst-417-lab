/*******************************************************************************

  Summary: System Debugging and messaging interface
    
  Description:

*******************************************************************************/

/*******************************************************************************
FileName:	system_debug.c
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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>



#include "system/system_services.h"
#include "system_config.h"
#include "system_debug_private.h"

#include "./drivers/lcd.h"

#if defined (__C32__)
#define DBAPPIO_HANDLE -1111
#endif

#if defined (__C32__)
void __attribute__((nomips16)) __sys_assert(bool smth)
{
		if((smth)==false)
		{
            __asm__ __volatile__ ("teq $0, $0");
		}
}
#elif defined (__C30__)
extern __inline__ void __attribute__((always_inline)) __sys_assert(bool smth)
{
		if((smth)==false)
		{
					while(1);
		}
}
#endif

typedef struct
{
    SYS_MODULE_ID       modId;      // module id
    int                 port_no;    // corresponding port on that module
    _SYS_CDEV_OBJ*      pObj;       // object implementing the functionality
    _SYS_CDEV_HANDLE    obHandle;    // handle to call the object as parameter
}_SYS_CDEV_ENTRY;   // entry describing a character device to be used for debug/console functionality


// objects implementing the character devices 
extern _SYS_CDEV_OBJ _usart_cdev_obj;       // usart        
#if ((defined (SYS_CONSOLE_ENABLE) || defined (SYS_DEBUG_ENABLE))) && defined(__PIC32MX__) && defined(PIC32_STARTER_KIT) && defined(__DEBUG)
extern _SYS_CDEV_OBJ _dbappio_cdev_obj;     // db_appio
#endif

#if defined(SYS_DEBUG_ENABLE) || defined (SYS_CONSOLE_ENABLE)
static _SYS_CDEV_ENTRY cdev_entry_tbl[] = 
{
    // modId                // port_no      // pObj             // obHandle
    {SYS_MODULE_UART_1,     0,              &_usart_cdev_obj,    0},
    {SYS_MODULE_UART_2,     1,              &_usart_cdev_obj,    0},
#if ((defined (SYS_CONSOLE_ENABLE) || defined (SYS_DEBUG_ENABLE))) && defined(__PIC32MX__) && defined(PIC32_STARTER_KIT) && defined(__DEBUG)
    {SYS_MODULE_DBAPPIO,    0,              &_dbappio_cdev_obj,    0},
#endif
};
#endif


#if defined(SYS_DEBUG_ENABLE) 
static _SYS_CDEV_ENTRY* debug_handle = 0;
#endif

#if defined(SYS_CONSOLE_ENABLE) 
static _SYS_CDEV_ENTRY* console_handle = 0;
#endif

#if defined(SYS_DEBUG_ENABLE) 
static const char* _sys_excep_tbl[16] = 
{
    "Int",
    "Reserved",
    "Reserved",
    "Reserved",
    "AdEL",
    "AdES",
    "IBE",
    "DBE",
    "Sys",
    "Bp",
    "RI",
    "CpU",
    "Ov",
    "Tr",
    "Reserved",
    "Reserved",
};
#endif  // defined(SYS_DEBUG_ENABLE) 


#if defined(SYS_DEBUG_ENABLE) || defined(SYS_CONSOLE_ENABLE)
static _SYS_CDEV_ENTRY* _SysGetCDevEntry(SYS_MODULE_ID modId)
{
    int ix;
    for(ix = 0; ix < sizeof(cdev_entry_tbl)/sizeof(*cdev_entry_tbl); ix++)
    {
        if(cdev_entry_tbl[ix].modId == modId)
        {
            return cdev_entry_tbl + ix;
        }
    }

    return 0;
}
#endif


/*********************************************************************
 * Function:        bool _SYS_DEBUG_INIT(int debug_port)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          It initializes the system debug module.
 * Side Effects:    None
 *
 * Overview:        The function initializes the system debug module.
 *
 * Note:            None
 ********************************************************************/
#if defined(SYS_DEBUG_ENABLE) 
bool _SYS_DEBUG_INIT(int debug_port)
{
    _SYS_CDEV_ENTRY* pEntry;
    bool init_res;

    pEntry = _SysGetCDevEntry(debug_port);

    if(pEntry == 0)
    {   // no such entry
        return false;
    }

    init_res = (*pEntry->pObj->init)(pEntry->port_no, SYS_DEBUG_BAUDRATE);

    // perform also the _SYS_DEBUG_OPEN() here so that the system calls
    // directly debug services without a handle
    if(init_res)
    {
        debug_handle = (_SYS_CDEV_ENTRY*)_SYS_DEBUG_OPEN(debug_port);
        return debug_handle != 0;
    }
    
    return false;
    
}    
#endif


/*********************************************************************
 * Function:        _SYS_CDEV_HANDLE _SYS_DEBUG_OPEN(int debug_port)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          It opens the system debug module.
 *
 * Side Effects:    None
 *
 * Overview:        The function opens the system debug module.
 *
 * Note:            None
 ********************************************************************/
#if defined(SYS_DEBUG_ENABLE) 
_SYS_CDEV_HANDLE _SYS_DEBUG_OPEN(int debug_port)
{
    _SYS_CDEV_ENTRY* pEntry;

    pEntry = _SysGetCDevEntry(debug_port);

    if(pEntry)
    {
        pEntry->obHandle = (*pEntry->pObj->open)(pEntry->port_no);
        if(pEntry->obHandle != 0)

        {
            return pEntry;
        }
    }

    return 0;

}
#endif




/*********************************************************************
 * Function:        void _SYS_ASSERT(int linenumber, const char *filename, const char *message)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          It displays a message to the system console and
 *                  halts the system 
 *
 * Side Effects:    None
 *
 * Overview:        This function performs an assert and halts the system
 *                  if the test expression is false.
 *
 * Note:            None
 ********************************************************************/
#if defined(SYS_DEBUG_ENABLE) 
void _SYS_ASSERT(int linenumber, const char* filename, const char* message)
{

    char buff[SYS_CONSOLE_BUFFER_LEN];
    snprintf(buff, SYS_CONSOLE_BUFFER_LEN, "\nSYS_ASSERT occurred in file %s on line %d. ", filename, linenumber);

    (debug_handle->pObj->print)(debug_handle->obHandle, buff);
    (debug_handle->pObj->print)(debug_handle->obHandle, message);

    for(;;);  // for internal debug testing, remove later
	__sys_assert(0);
}
#endif  // defined(SYS_DEBUG_ENABLE) 


/*********************************************************************
 * Function:        void _SYS_ERROR(int linenumber, const char *filename, const char *message)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          It displays an error message to the system console
 *
 * Side Effects:    None
 *
 * Overview:        The function wil display the message to the system console
 *
 * Note:            None
 ********************************************************************/
#if defined(SYS_DEBUG_ENABLE) 
void _SYS_ERROR(int linenumber, const char* filename, const char* message)
{
    char buff[SYS_CONSOLE_BUFFER_LEN];

    snprintf(buff, SYS_CONSOLE_BUFFER_LEN, "\nSYS_ERROR occurred in file %s on line %d. ", filename, linenumber);

    (debug_handle->pObj->print)(debug_handle->obHandle, buff);
    (debug_handle->pObj->print)(debug_handle->obHandle, message);

}
#endif  // defined(SYS_DEBUG_ENABLE) 



/*********************************************************************
 * Function:        void _SYS_ERROR_PRINT(SYS_ERROR_LEVEL level, 
 *										const char* format, ...)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          It displays a formatted message to the system debug port
 *
 * Side Effects:    None
 *
 * Overview:        The function wil display the message to the system debug port
 *
 * Note:            None
 ********************************************************************/
#if defined(SYS_DEBUG_ENABLE) 
void _SYS_ERROR_PRINT(SYS_ERROR_LEVEL level, const char* format, ...)
{
    char buff[SYS_CONSOLE_BUFFER_LEN];
    va_list args; 
    if( (level) < SYSTEM_CURRENT_ERROR_LEVEL){
        va_start( args, format ); 
        snprintf(buff, SYS_CONSOLE_BUFFER_LEN, "\nSYS_ERROR_PRINT issued: ");
        (debug_handle->pObj->print)(debug_handle->obHandle, buff);
        vsnprintf(buff, SYS_CONSOLE_BUFFER_LEN, format, args);
        (debug_handle->pObj->print)(debug_handle->obHandle, buff);
        va_end( args );	
    }
}
#endif  // defined(SYS_DEBUG_ENABLE) 


/*********************************************************************
 * Function:        void _SYS_PRINT(const char *format, ...)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          It displays a formatted message to the system console
 *
 * Side Effects:    None
 *
 * Overview:        The function wil display the message to the system console
 *
 * Note:            None
 ********************************************************************/
 #if defined(SYS_DEBUG_ENABLE) 
void _SYS_PRINT(const char* format, ...)
{	
    char buff[SYS_CONSOLE_BUFFER_LEN];
    va_list args; 
    va_start( args, format ); 

    snprintf(buff, SYS_CONSOLE_BUFFER_LEN, "\nSYS_PRINT issued:");
    (debug_handle->pObj->print)(debug_handle->obHandle, buff);
    vsnprintf(buff, SYS_CONSOLE_BUFFER_LEN, format, args);
    (debug_handle->pObj->print)(debug_handle->obHandle, buff);

    va_end( args );        
}
#endif


#if defined(SYS_DEBUG_ENABLE) 
void __attribute__((weak, nomips16)) _general_exception_handler (void)
{
	char buff[SYS_CONSOLE_BUFFER_LEN];
    uint32_t epc, cause;
    int cause_ix;
    const char* cause_str;

	snprintf(buff, SYS_CONSOLE_BUFFER_LEN, "\n\rSystem Exception Handler entered.\n\r");
    (debug_handle->pObj->print)(debug_handle->obHandle, buff);

    epc = _CP0_GET_EPC();
    cause = _CP0_GET_CAUSE();
    cause_ix = (cause & _CP0_CAUSE_EXCCODE_MASK) >> _CP0_CAUSE_EXCCODE_POSITION;
    if(cause_ix < sizeof(_sys_excep_tbl)/sizeof(*_sys_excep_tbl))
    {
        cause_str = _sys_excep_tbl[cause_ix];
    }
    else
    {
        cause_str = "Unidentified";
    }

	snprintf(buff, SYS_CONSOLE_BUFFER_LEN, "\n\rA %s type exception occurred at address: 0x%08x\n\r", cause_str, epc);
    (debug_handle->pObj->print)(debug_handle->obHandle, buff);

	snprintf(buff, SYS_CONSOLE_BUFFER_LEN, "\n\rSystem will halt!\n\r");
    (debug_handle->pObj->print)(debug_handle->obHandle, buff);

    // break into the debugger
  __asm__ volatile (" sdbbp 0");
}
#endif




/*********************************************************************
 * Function:        bool _SYS_CONSOLE_INIT(int console_port)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          It initializes the system console module.
 *
 * Side Effects:    None
 *
 * Overview:        The function initializes the system console module.
 *
 * Note:            None
 ********************************************************************/
#if defined(SYS_CONSOLE_ENABLE) 
bool _SYS_CONSOLE_INIT(int console_port)
{
    _SYS_CDEV_ENTRY* pEntry;
    bool init_res;

    pEntry = _SysGetCDevEntry(console_port);

    if(pEntry == 0)
    {   // no such entry
        return false;
    }

    init_res = (*pEntry->pObj->init)(pEntry->port_no, SYS_CONSOLE_BAUDRATE);

    // perform also the _SYS_CONSOLE_OPEN() here so that the system calls
    // directly debug services without a handle
    if(init_res)
    {
        console_handle = (_SYS_CDEV_ENTRY*)_SYS_CONSOLE_OPEN(console_port);
        return console_handle != 0;
    }
    
    return false;
 
}
#endif

/*********************************************************************
 * Function:        _SYS_CDEV_HANDLE _SYS_CONSOLE_OPEN(int console_port)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          It opens the system console module.
 *
 * Side Effects:    None
 *
 * Overview:        The function opens the system console module.
 *
 * Note:            None
 ********************************************************************/
#if defined(SYS_CONSOLE_ENABLE) 
_SYS_CDEV_HANDLE _SYS_CONSOLE_OPEN(int console_port)
{
    _SYS_CDEV_ENTRY* pEntry;

    pEntry = _SysGetCDevEntry(console_port);

    if(pEntry)
    {
        pEntry->obHandle = (*pEntry->pObj->open)(pEntry->port_no);
        if(pEntry->obHandle != 0)

        {
            return pEntry;
        }
    }

    return 0;

}
#endif

/*********************************************************************
 * Function:        void _SYS_CONSOLE_PRINT(const char *format, ...)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          It displays a formatted message to the system console
 *
 * Side Effects:    None
 *
 * Overview:        The function will display the formatted message 
 *					to the system console.
 *
 * Note:            None
 ********************************************************************/
#if defined(SYS_CONSOLE_ENABLE) 
void _SYS_CONSOLE_PRINT(const char* format, ...)
{
    va_list arg_list;
    char buff[SYS_CONSOLE_BUFFER_LEN];

    va_start(arg_list, format);
    vsnprintf(buff, SYS_CONSOLE_BUFFER_LEN, format, arg_list);
    (console_handle->pObj->print)(console_handle->obHandle, buff);
    va_end(arg_list);
}
#endif

/*********************************************************************
 * Function:        void _SYS_CONSOLE_MESSAGE(const char* msg)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          It displays a message to the system console
 *
 * Side Effects:    None
 *
 * Overview:        The function will display the message to the 
 *					system console.
 *
 * Note:            None
 ********************************************************************/
#if defined(SYS_CONSOLE_ENABLE) 
void _SYS_CONSOLE_MESSAGE(const char* msg)
{
    (console_handle->pObj->print)(console_handle->obHandle, msg);
}
#endif

/*********************************************************************
 * Function:		char _SYS_CONSOLE_DATA_RDY(void)
 *
 * PreCondition:	None
 *
 * Input:			None
 *
 * Output:			true or false
 *
 * Side Effects:	None
 *
 * Overview:		The function will return the status of console receive
 *					
 *
 * Note:			None
 ********************************************************************/
#if defined(SYS_CONSOLE_ENABLE) 
bool _SYS_CONSOLE_DATA_RDY()
{
    return (console_handle->pObj->isRdy)(console_handle->obHandle);
}
#endif


/*********************************************************************
 * Function:        char _SYS_CONSOLE_GETC(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          It gets a character from the system console
 *
 * Side Effects:    None
 *
 * Overview:        The function will display the message to the 
 *					system console.
 *
 * Note:            None
 ********************************************************************/
#if defined(SYS_CONSOLE_ENABLE) 
char _SYS_CONSOLE_GETC()
{
    return (console_handle->pObj->getc)(console_handle->obHandle);
}
#endif
	
/*********************************************************************
 * Function:		char _SYS_CONSOLE_PUTC(void)
 *
 * PreCondition:	None
 *
 * Input:			c
 *
 * Output:			It puts a character out to the system console
 *
 * Side Effects:	None
 *
 * Overview:		The function will display the character to the 
 *					system console.
 *
 * Note:			None
 ********************************************************************/
#if defined(SYS_CONSOLE_ENABLE) 
void _SYS_CONSOLE_PUTC(char c)
{
    return (console_handle->pObj->putc)(console_handle->obHandle, c);
}
#endif



/*********************************************************************
 * Function:        int _SYS_CONSOLE_GETLINE(char* line, int bufferLen)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          It gets a line from the system console
 *
 * Side Effects:    None
 *
 * Overview:        The function will display the message to the 
 *					system console.
 *
 * Note:            None
 ********************************************************************/
#if defined(SYS_CONSOLE_ENABLE) 
int _SYS_CONSOLE_GETLINE(char* line, int bufferLen)
{
    return (console_handle->pObj->gets)(console_handle->obHandle, line, bufferLen);
}
#endif

/*********************************************************************
 * Function:        int _SYS_CONSOLE_SCANF(const char *format, ...)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          It scans inputs from the system console
 *
 * Side Effects:    None
 *
 * Overview:        The function scans formatted inputs from the  
 *					system console.
 *
 * Note:            None
 ********************************************************************/
//#if defined(SYS_CONSOLE_ENABLE) 
//int _SYS_CONSOLE_SCANF(const char *format, ...){
//	int i=0;
//	char buff[SYS_CONSOLE_BUFFER_LEN];
//	va_list arg_list;
//	va_start(arg_list, format);
//	while(i < SYS_CONSOLE_BUFFER_LEN)
//		buff[i++]=CONSOLE_GETC(console_handle);			
//	
//	vsscanf(buff, format, arg_list);
//	va_end(arg_list);
//}
//#endif
//


/*********************************************************************
 * Function:        bool _SYS_OUT_INIT(int out_port)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          It initializes the system output module.
 *
 * Side Effects:    None
 *
 * Overview:        The function initializes the system output module.
 *
 * Note:            None
 ********************************************************************/
#if defined(SYS_OUT_ENABLE) 
bool _SYS_OUT_INIT(int out_port)
{
    LCD_INIT();
	return true;
}
#endif

/*********************************************************************
 * Function:        void _SYS_OUT_MESSAGE(const char* msg)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          It displays a message to the system output
 *
 * Side Effects:    None
 *
 * Overview:        The function will display the message to the
 *					system output.
 *
 * Note:            None
 ********************************************************************/
#if defined(SYS_OUT_ENABLE) 
void _SYS_OUT_MESSAGE(const char* msg)
{
	LCD_ERASE();
	_SYS_OUT_MESSAGE_OFFSET(msg, 0);
}
#endif

/*********************************************************************
 * Function:        void _SYS_OUT_MESSAGE_LINE(const char *msg, int line)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          It displays a message to the system output on the 
 *					specified line
 *
 * Side Effects:    None
 *
 * Overview:        The function will display the message to the
 *					system output on the specified line.
 *
 * Note:            None
 ********************************************************************/
#if defined(SYS_OUT_ENABLE) 
void _SYS_OUT_MESSAGE_LINE(const char *msg, int line)
{
	_SYS_OUT_MESSAGE_OFFSET(msg, line*LCD_CHARS_PER_LINE);
}
#endif

/*********************************************************************
 * Function:        void _SYS_OUT_MESSAGE_OFFSET(const char *msg, int offset)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          It displays a message to the system output starting  
 *					at the specified offset
 *
 * Side Effects:    None
 *
 * Overview:        The function will display the message to the
 *					system output on the specified offset.
 *
 * Note:            None
 ********************************************************************/
#if defined(SYS_OUT_ENABLE) 
void _SYS_OUT_MESSAGE_OFFSET(const char *msg, int offset)
{
	extern char LCDText[];
	strncpy((char*)LCDText, msg, LCD_CHARS_PER_LINE);
	LCD_UPDATE_OFFSET(offset);	
}
#endif

/*********************************************************************
 * Function:        
 * uint8_t *	SYS_OUT_GET_LCD_MESSAGE(void)
 
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          
 *
 * Side Effects:    None
 *
 * Overview:        The function will display the message to the
 *					system output on the specified offset.
 *
 * Note:            None
 ********************************************************************/
#if defined(SYS_OUT_ENABLE) 
char *  SYS_OUT_GET_LCD_MESSAGE(void)
{
	extern char LCDText[];
	return 	(char *)LCDText;
}
#endif



