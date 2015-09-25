/*******************************************************************************

  Summary: System Debugging private messaging API
    
  Description:

*******************************************************************************/

/*******************************************************************************
FileName: 	system_debug_private.h
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


#ifndef _SYSTEM_DEBUG_PRIVATE_H_
#define _SYSTEM_DEBUG_PRIVATE_H_



// functions supported by a system character device
// 

// a system character device handle
typedef const void* _SYS_CDEV_HANDLE;

// init function
typedef bool (*_SYS_CDEV_INIT)(int port_no,  unsigned int init_param);

// open function
typedef _SYS_CDEV_HANDLE (*_SYS_CDEV_OPEN)(int port_no);
	
// print function
typedef void (*_SYS_CDEV_PRINT)(_SYS_CDEV_HANDLE u_handle, const char* str);

// put single char function
typedef void (*_SYS_CDEV_PUTC)(_SYS_CDEV_HANDLE u_handle, char c);

// data available
typedef bool (*_SYS_CDEV_DATA_RDY)(_SYS_CDEV_HANDLE u_handle);

// get single data
typedef char (*_SYS_CDEV_GETC)(_SYS_CDEV_HANDLE u_handle);

// get multiple data
typedef int (*_SYS_CDEV_GETS)(_SYS_CDEV_HANDLE u_handle, char* buff, int buff_len);


typedef struct
{
    // init function
    _SYS_CDEV_INIT  init;

    // open function
    _SYS_CDEV_OPEN  open;

    // print function
    _SYS_CDEV_PRINT print;

    // put single char function
    _SYS_CDEV_PUTC  putc;

    // data available
    _SYS_CDEV_DATA_RDY  isRdy;

    // get single data
    _SYS_CDEV_GETC      getc;

    // get multiple data
    _SYS_CDEV_GETS      gets;
}_SYS_CDEV_OBJ;     // object implementing a character device    


// error, and debug services
bool                _SYS_DEBUG_INIT(int debug_port);

_SYS_CDEV_HANDLE    _SYS_DEBUG_OPEN(int debug_port);

void                _SYS_ASSERT(int linenumber, const char *filename, const char *message);

void                _SYS_ERROR(int linenumber, const char *filename, const char *message);

void 		        _SYS_ERROR_PRINT(SYS_ERROR_LEVEL level, const char *format, ...);


// console services

bool                _SYS_CONSOLE_INIT(int console_port);

_SYS_CDEV_HANDLE    _SYS_CONSOLE_OPEN(int console_port);
    
void 		        _SYS_CONSOLE_MESSAGE(const char *message);

void                _SYS_CONSOLE_PRINT(const char *format, ...);

char 		        _SYS_CONSOLE_GETC(void);

void                _SYS_CONSOLE_PUTC(char c);

bool                _SYS_CONSOLE_DATA_RDY();

int 	        	_SYS_CONSOLE_GETLINE(char* buffer, int bufferLen);

int 		        _SYS_CONSOLE_SCANF(const char *format, ...);

// Output (LCD) services

bool                _SYS_OUT_INIT(int out_port);

void 	        	_SYS_OUT_MESSAGE(const char *msg);

void 		        _SYS_OUT_MESSAGE_LINE(const char *msg, int line);

void 		        _SYS_OUT_MESSAGE_OFFSET(const char *msg, int offset);
        
int 	        	_SYS_OUT_MESSAGE_LINE_COUNT(void);

int 		        _SYS_OUT_MESSAGE_LINE_LENGTH(void);
char *  SYS_OUT_GET_LCD_MESSAGE(void);







#endif  // _SYSTEM_DEBUG_PRIVATE_H_

