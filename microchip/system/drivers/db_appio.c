/*******************************************************************************

  Summary:
    
  Description:

*******************************************************************************/

/*******************************************************************************
FileName: 	db_appio.c
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
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PzUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/

#include "system_config.h"

#if ((defined (SYS_CONSOLE_ENABLE) || defined (SYS_DEBUG_ENABLE))) && defined(__PIC32MX__) && defined(PIC32_STARTER_KIT) && defined(__DEBUG)

#include <sys/appio.h>
#include "system/system_services.h"


// prototypes
bool                DB_APPIO_INIT(int port_no, unsigned int baud_rate);
_SYS_CDEV_HANDLE    DB_APPIO_OPEN(int port_no);
void                DB_APPIO_PRINT(_SYS_CDEV_HANDLE d_handle, const char* str);
bool                DB_APPIO_DATA_RDY(_SYS_CDEV_HANDLE d_handle);
char                DB_APPIO_GETC(_SYS_CDEV_HANDLE d_handle);
int                 DB_APPIO_GETS(_SYS_CDEV_HANDLE d_handle, char* buff, int buff_len);
void                DB_APPIO_PUTC(_SYS_CDEV_HANDLE d_handle, char c);


// object implementing the DB_APPIO character device    
_SYS_CDEV_OBJ _dbappio_cdev_obj = 
{
    DB_APPIO_INIT,     // _SYS_CDEV_INIT  init;
    DB_APPIO_OPEN,     // _SYS_CDEV_OPEN  open;
    DB_APPIO_PRINT,    // _SYS_CDEV_PRINT print;
    DB_APPIO_PUTC,     // _SYS_CDEV_PUTC  putc;
    DB_APPIO_DATA_RDY, // _SYS_CDEV_DATA_RDY  isRdy;
    DB_APPIO_GETC,     // _SYS_CDEV_GETC      getc;;
    DB_APPIO_GETS,     // _SYS_CDEV_GETS      gets;
};


// *****************************************************************************
// *****************************************************************************
// Section: DB_APPIO Functions
// *****************************************************************************
// *****************************************************************************




bool DB_APPIO_INIT(int port_no, unsigned int baud_rate)
{
    
    if(port_no != 0)
    {   // only one instance supported
        return false;
    }
    
	DBINIT();
	return true;
}

_SYS_CDEV_HANDLE DB_APPIO_OPEN(int port_no)
{
    if(port_no != 0)
    {   // only one instance supported
        return 0;
    }
    
    return &_dbappio_cdev_obj;
}
	
void DB_APPIO_PRINT(_SYS_CDEV_HANDLE d_handle, const char* str)
{
    DBPUTS(str);
}

bool DB_APPIO_DATA_RDY(_SYS_CDEV_HANDLE d_handle)
{
    return false;
}

char DB_APPIO_GETC(_SYS_CDEV_HANDLE d_handle)
{
	unsigned char c;
	DBGETC(&c);
	return c;
}


int DB_APPIO_GETS(_SYS_CDEV_HANDLE d_handle, char* buff, int buff_len)
{
	DBGETS((unsigned char*)buff, buff_len);
	return buff_len;
}

void DB_APPIO_PUTC(_SYS_CDEV_HANDLE d_handle, char c)
{
	DBPUTC((unsigned char*)&c);
}



#endif  // ((defined (SYS_CONSOLE_ENABLE) || defined (SYS_DEBUG_ENABLE))) && defined(__PIC32MX__) && defined(PIC32_STARTER_KIT) && defined(__DEBUG)

