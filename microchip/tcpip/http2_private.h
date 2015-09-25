/*******************************************************************************
  HTTP Headers for Microchip TCP/IP Stack

Summary:

Description:
 *******************************************************************************/

/*******************************************************************************
FileName:  http2_private.h 
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

#ifndef __HTTP2_PRIVATE_H
#define __HTTP2_PRIVATE_H


/****************************************************************************
Section:
HTTP State Definitions
 ***************************************************************************/

// Basic HTTP Connection State Machine
typedef enum
{
    SM_HTTP_IDLE = 0u,				// Socket is idle
    SM_HTTP_PARSE_REQUEST,			// Parses the first line for a file name and GET args
    SM_HTTP_PARSE_HEADERS,			// Reads and parses headers one at a time
    SM_HTTP_AUTHENTICATE,			// Validates the current authorization state
    SM_HTTP_PROCESS_GET,			// Invokes user callback for GET args or cookies
    SM_HTTP_PROCESS_POST,			// Invokes user callback for POSTed data
    SM_HTTP_PROCESS_REQUEST,		// Begins the process of returning data
    SM_HTTP_SERVE_HEADERS,			// Sends any required headers for the response
    SM_HTTP_SERVE_COOKIES,			// Adds any cookies to the response
    SM_HTTP_SERVE_BODY,				// Serves the actual content
    SM_HTTP_SEND_FROM_CALLBACK,		// Invokes a dynamic variable callback
    SM_HTTP_DISCONNECT				// Disconnects the server and closes all files
} SM_HTTP2;



// Stores extended state data for each connection
typedef struct
{
    uint32_t byteCount;					// How many bytes have been read so far
    uint32_t nextCallback;					// Byte index of the next callback
    union
    {
        SYS_TICK    httpTick;           // watchdog timer
        uint32_t       callbackID;			// Callback ID to execute
    };
    uint32_t callbackPos;					// Callback position indicator
    uint8_t *ptrData;						// Points to first free byte in data
    uint8_t *ptrRead;						// Points to current read location
    const void*   mediaHndl;                // media handle;
    MPFS_HANDLE file;					// File pointer for the file being served
    MPFS_HANDLE offsets;				// File pointer for any offset info being used
    HTTP_STATUS httpStatus;				// Request method/status
    HTTP_FILE_TYPE fileType;			// File type to return with Content-Type
    SM_HTTP2 sm;						// Current connection state
    TCP_SOCKET socket;					// Socket being served
    uint8_t* data;              		// General purpose data buffer
    uint8_t hasArgs;						// True if there were get or cookie arguments
    uint8_t isAuthorized;					// 0x00-0x79 on fail, 0x80-0xff on pass
    uint8_t smPost;						// POST state machine variable
    const void* userData;               // user supplied data; not used by the HTTP module
} HTTP_CONN;





#endif // __HTTP2_PRIVATE_H

