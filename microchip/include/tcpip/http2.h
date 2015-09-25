/*******************************************************************************
  HTTP Headers for Microchip TCP/IP Stack

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  HTTP2.h 
Copyright � 2012 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED �AS IS� WITHOUT WARRANTY OF ANY KIND,
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

#ifndef __HTTP2_H_
#define __HTTP2_H_

#include "system/fs/mpfs2.h"


/****************************************************************************
  Section:
	Commands and Server Responses
  ***************************************************************************/

	//Supported Commands and Server Response Codes
	typedef enum
	{
	    HTTP_GET = 0u,					// GET command is being processed
	    HTTP_POST,						// POST command is being processed
	    HTTP_BAD_REQUEST,				// 400 Bad Request will be returned
		HTTP_UNAUTHORIZED,				// 401 Unauthorized will be returned
	    HTTP_NOT_FOUND,					// 404 Not Found will be returned
		HTTP_OVERFLOW,					// 414 Request-URI Too Long will be returned
		HTTP_INTERNAL_SERVER_ERROR,		// 500 Internal Server Error will be returned
		HTTP_NOT_IMPLEMENTED,			// 501 Not Implemented (not a GET or POST command)
        HTTP_REDIRECT,					// 302 Redirect will be returned
		HTTP_SSL_REQUIRED,				// 403 Forbidden is returned, indicating SSL is required

		HTTP_MPFS_FORM,					// Show the MPFS Upload form
		HTTP_MPFS_UP,					// An MPFS Upload is being processed
		HTTP_MPFS_OK,					// An MPFS Upload was successful
		HTTP_MPFS_ERROR,				// An MPFS Upload was not a valid image
		
	} HTTP_STATUS;

/****************************************************************************
  Section:
	HTTP Definitions
  ***************************************************************************/


	// Result states for execution callbacks
	typedef enum
	{
		HTTP_IO_DONE = 0u,	// Finished with procedure
		HTTP_IO_NEED_DATA,	// More data needed to continue, call again later
		HTTP_IO_WAITING		// Waiting for asynchronous process to complete, call again later
	} HTTP_IO_RESULT;

	// Result states for HTTPPostReadName and HTTPPostReadValue
	typedef enum
	{
		HTTP_READ_OK = 0u,		// Read was successful
		HTTP_READ_TRUNCATED,	// Buffer overflow prevented by truncating value
		HTTP_READ_INCOMPLETE	// Entire object is not yet in the buffer.  Try again later.
	} HTTP_READ_STATUS;

	// File type definitions
	typedef enum
	{
		HTTP_TXT = 0u,		// File is a text document
		HTTP_HTM,			// File is HTML (extension .htm)
		HTTP_HTML,			// File is HTML (extension .html)
		HTTP_CGI,			// File is HTML (extension .cgi)
		HTTP_XML,			// File is XML (extension .xml)
		HTTP_CSS,			// File is stylesheet (extension .css)
		HTTP_GIF,			// File is GIF image (extension .gif)
		HTTP_PNG,			// File is PNG image (extension .png)
		HTTP_JPG,			// File is JPG image (extension .jpg)
		HTTP_JAVA,			// File is java (extension .java)
		HTTP_WAV,			// File is audio (extension .wav)
		HTTP_UNKNOWN		// File type is unknown
	} HTTP_FILE_TYPE;

// HTTP connection identifier
// handle of a HTTP connection
typedef const void*     HTTP_CONN_HANDLE;

// HTTP module configuration flags
typedef enum
{
    HTTP_MODULE_FLAG_ADJUST_SKT_FIFOS    = 0x01, // adjust corresponding socket FIFO at run time
                                                 // improves throughput when the socket buffers are small
    HTTP_MODULE_FLAG_NO_DELAY            = 0x02, // create the http sockets with NO-DELAY option
                                                 // it will flush data as soon as possible
}HTTP_MODULE_FLAGS;

// HTTP module dynamic configuration data
typedef struct
{
    uint16_t    nConnections;   // number of http connections
    uint16_t    dataLen;        // size of the data buffer for reading cookie and GET/POST arguments (bytes)
    uint16_t    sktTxBuffSize;  // size of TX buffer for the associatted socket; leave 0 for default
    uint16_t    sktRxBuffSize;  // size of RX buffer for the associatted socket; leave 0 for default
    uint16_t    configFlags;    // a HTTP_MODULE_FLAGS value.

} HTTP_MODULE_CONFIG;

/****************************************************************************
  Section:
	Function Prototypes
  ***************************************************************************/

uint8_t*            HTTPURLDecode(uint8_t* cData);
const uint8_t*      HTTPGetArg(const uint8_t* cData, const uint8_t* cArg);

void                HTTPIncFile(HTTP_CONN_HANDLE connHandle, const uint8_t* cFile);
HTTP_READ_STATUS    HTTPReadPostName(HTTP_CONN_HANDLE connHandle, uint8_t* cData, uint16_t wLen);
HTTP_READ_STATUS    HTTPReadPostValue(HTTP_CONN_HANDLE connHandle, uint8_t* cData, uint16_t wLen);

MPFS_HANDLE	        HTTPCurConnectionFileGet(HTTP_CONN_HANDLE connHandle);
int 		        HTTPCurConnectionPostSmGet(HTTP_CONN_HANDLE connHandle);
void		        HTTPCurConnectionPostSmSet(HTTP_CONN_HANDLE connHandle, int);
uint8_t*            HTTPCurConnectionDataBufferGet(HTTP_CONN_HANDLE connHandle);
uint32_t            HTTPCurConnectionCallbackPosGet(HTTP_CONN_HANDLE connHandle);
void	            HTTPCurConnectionCallbackPosSet(HTTP_CONN_HANDLE connHandle, uint32_t);
void	            HTTPCurConnectionStatusSet(HTTP_CONN_HANDLE connHandle, HTTP_STATUS stat);
void	            HTTPCurConnectionHasArgsSet(HTTP_CONN_HANDLE connHandle, bool args);
uint32_t            HTTPCurConnectionByteCountGet(HTTP_CONN_HANDLE connHandle);
void          	    HTTPCurConnectionByteCountSet(HTTP_CONN_HANDLE connHandle, uint32_t);
void          	    HTTPCurConnectionByteCountDec(HTTP_CONN_HANDLE connHandle, uint32_t);
TCP_SOCKET          HTTPCurConnectionSocketGet(HTTP_CONN_HANDLE connHandle);
uint8_t             HTTPCurConnectionIsAuthorizedGet(HTTP_CONN_HANDLE connHandle);
void                HTTPCurConnectionIsAuthorizedSet(HTTP_CONN_HANDLE connHandle, uint8_t auth);
void          	    HTTPCurConnectionUserDataSet(HTTP_CONN_HANDLE connHandle, const void* uData);
const void*         HTTPCurConnectionUserDataGet(HTTP_CONN_HANDLE connHandle);

/*****************************************************************************
  Function:
	HTTP_READ_STATUS HTTPReadPostPair(HTTP_CONN_HANDLE connHandle, uint8_t* cData, uint16_t wLen)

  Summary:
	Reads a name and value pair from a URL encoded string in the TCP buffer.

  Description:
	Reads a name and value pair from a URL encoded string in the TCP buffer.
	This function is meant to be called from an HTTPExecutePost callback to
	facilitate easier parsing of incoming data.  This function also prevents
	buffer overflows by forcing the programmer to indicate how many bytes are
	expected.  At least 2 extra bytes are needed in cData over the maximum
	length of data expected to be read.

	This function will read until the next '&' character, which indicates the
	end of a value parameter.  It assumes that the front of the buffer is
	the beginning of the name paramter to be read.

	This function properly updates curHTTP.byteCount by decrementing it
	by the number of bytes read.  It also removes the delimiting '&' from
	the buffer.

	Once complete, two strings will exist in the cData buffer.  The first is
	the parameter name that was read, while the second is the associated
	value.

  Precondition:
	Front of TCP buffer is the beginning of a name parameter, and the rest of
	the TCP buffer contains a URL-encoded string with a name parameter
	terminated by a '=' character and a value parameter terminated by a '&'.

  Parameters:
	connHandle  - HTTP connection handle
	cData - where to store the name and value strings once they are read
	wLen - how many bytes can be written to cData

  Return Values:
	HTTP_READ_OK - name and value were successfully read
	HTTP_READ_TRUNCTATED - entire name and value could not fit in the buffer,
							so input was truncated and data has been lost
	HTTP_READ_INCOMPLETE - entire name and value was not yet in the buffer,
							so call this function again later to retrieve

  Remarks:
	This function is aliased to HTTPReadPostValue, since they effectively
	perform the same task.  The name is provided only for completeness.
  ***************************************************************************/
#define HTTPReadPostPair(connHandle, cData, wLen) HTTPReadPostValue(connHandle, cData, wLen)


/****************************************************************************
  Section:
	User-Implemented Callback Function Prototypes
  ***************************************************************************/

/*****************************************************************************
  Function:
	HTTP_IO_RESULT HTTPExecuteGet(HTTP_CONN_HANDLE connHandle)

  Summary:
	Processes GET form field variables and cookies.

  Description:
	This function is implemented by the application developer in
	CustomHTTPApp.c.  Its purpose is to parse the data received from
	URL parameters (GET method forms) and cookies and perform any
	application-specific tasks in response to these inputs.  Any
	required authentication has already been validated.

	When this function is called, curHTTP.data contains sequential
	name/value pairs of strings representing the data received.  In this
	format, HTTPGetArg can be used to search for
	specific variables in the input.  If data buffer space associated
	with this connection is required, curHTTP.data may be overwritten
	here once the application is done with the values.  Any data placed
	there will be available to future callbacks for this connection,
	including HTTPExecutePost and any HTTPPrint_varname dynamic
	substitutions.

	This function may also issue redirections by setting curHTTP.data
	to the destination file name or URL, and curHTTP.httpStatus to
	HTTP_REDIRECT.

	Finally, this function may set cookies.  Set curHTTP.data to a series
	of name/value string pairs (in the same format in which parameters
	arrive) and then set curHTTP.hasArgs equal to the number of cookie
	name/value pairs.  The cookies will be transmitted to the browser,
	and any future requests will have those values available in
	curHTTP.data.

  Precondition:
	None

  Parameters:
	connHandle  - HTTP connection handle

  Return Values:
	HTTP_IO_DONE - application is done processing
	HTTP_IO_NEED_DATA - this value may not be returned because more data
						will not become available
	HTTP_IO_WAITING - the application is waiting for an asynchronous
					  process to complete, and this function should be
					  called again later

  Remarks:
	This function is only called if variables are received via URL
	parameters or Cookie arguments.  This function may NOT write to the
	TCP buffer.

	This function may service multiple HTTP requests simultaneously.
	Exercise caution when using global or static variables inside this
	routine.  Use curHTTP.callbackPos or curHTTP.data for storage associated
	with individual requests.
  ***************************************************************************/
HTTP_IO_RESULT HTTPExecuteGet(HTTP_CONN_HANDLE connHandle);

/*****************************************************************************
  Function:
	HTTP_IO_RESULT HTTPExecutePost(HTTP_CONN_HANDLE connHandle)

  Summary:
	Processes POST form variables and data.

  Description:
	This function is implemented by the application developer in
	CustomHTTPApp.c.  Its purpose is to parse the data received from
	POST forms and perform any application-specific tasks in response
	to these inputs.  Any required authentication has already been
	validated before this function is called.

	When this function is called, POST data will be waiting in the
	TCP buffer.  curHTTP.byteCount will indicate the number of bytes
	remaining to be received before the browser request is complete.

	Since data is still in the TCP buffer, the application must call
	TCPGet or TCPGetArray in order to retrieve bytes.  When this is done,
	curHTTP.byteCount MUST be updated to reflect how many bytes now
	remain.  The functions TCPFind, TCPFindString, and TCPFindArray
    may be helpful to locate data in the TCP buffer.

	In general, data submitted from web forms via POST is URL encoded.
	The HTTPURLDecode function can be used to decode this information
	back to a standard string if required.  If data buffer space
	associated with this connection is required, curHTTP.data may be
	overwritten here once the application is done with the values.
	Any data placed there will be available to future callbacks for
	this connection,  including HTTPExecutePost and any
	HTTPPrint_varname dynamic substitutions.

	Whenever a POST form is processed it is recommended to issue a
	redirect back to the browser, either to a status page or to
	the same form page that was posted.  This prevents accidental
	duplicate submissions (by clicking refresh or back/forward) and
	avoids browser warnings about "resubmitting form data".  Redirects
	may be issued to the browser by setting curHTTP.data to the
	destination file or URL, and curHTTP.httpStatus to HTTP_REDIRECT.

	Finally, this function may set cookies.  Set curHTTP.data to a series
	of name/value string pairs (in the same format in which parameters
	arrive) and then set curHTTP.hasArgs equal to the number of cookie
	name/value pairs.  The cookies will be transmitted to the browser,
	and any future requests will have those values available in
	curHTTP.data.

  Precondition:
	None

  Parameters:
	connHandle  - HTTP connection handle

  Return Values:
	HTTP_IO_DONE - application is done processing
	HTTP_IO_NEED_DATA - more data is needed to continue, and this
						function should be called again later
	HTTP_IO_WAITING - the application is waiting for an asynchronous
					  process to complete, and this function should
					  be called again later

  Remarks:
	This function is only called when the request method is POST, and is
	only used when HTTP_USE_POST is defined.  This method may NOT write
	to the TCP buffer.

	This function may service multiple HTTP requests simultaneously.
	Exercise caution when using global or static variables inside this
	routine.  Use curHTTP.callbackPos or curHTTP.data for storage associated
	with individual requests.
  ***************************************************************************/
HTTP_IO_RESULT HTTPExecutePost(HTTP_CONN_HANDLE connHandle);

/*****************************************************************************
  Function:
	uint8_t HTTPNeedsAuth(HTTP_CONN_HANDLE connHandle, uint8_t* cFile)

  Summary:
	Determines if a given file name requires authentication

  Description:
	This function is implemented by the application developer in
	CustomHTTPApp.c.  Its function is to determine if a file being
	requested requires authentication to view.  The user name and password,
	if supplied, will arrive later with the request headers, and will be
	processed at that time.

	Return values 0x80 - 0xff indicate that authentication is not required,
	while values from 0x00 to 0x79 indicate that a user name and password
	are required before proceeding.  While most applications will only use a
	single value to grant access and another to require authorization, the
	range allows multiple "realms" or sets of pages to be protected, with
	different credential requirements for each.

	The return value of this function is saved as curHTTP.isAuthorized, and
	will be available to future callbacks, including HTTPCheckAuth and any
	of the HTTPExecuteGet, HTTPExecutePost, or HTTPPrint_varname callbacks.

  Precondition:
	None

  Parameters:
	connHandle  - HTTP connection handle
	cFile - the name of the file being requested

  Return Values:
	<= 0x79 - valid authentication is required
	>= 0x80 - access is granted for this connection

  Remarks:
	This function may NOT write to the TCP buffer.
  ***************************************************************************/
uint8_t HTTPNeedsAuth(HTTP_CONN_HANDLE connHandle, uint8_t* cFile);

/*****************************************************************************
  Function:
	uint8_t HTTPCheckAuth(HTTP_CONN_HANDLE connHandle, uint8_t* cUser, uint8_t* cPass)

  Summary:
	Performs validation on a specific user name and password.

  Description:
	This function is implemented by the application developer in
	CustomHTTPApp.c.  Its function is to determine if the user name and
	password supplied by the client are acceptable for this resource.

	The value of curHTTP.isAuthorized will be set to the previous return
	value of HTTPRequiresAuthorization.  This callback function can check
	this value to determine if only specific user names or passwords will
	be accepted for this resource.

	Return values 0x80 - 0xff indicate that the credentials were accepted,
	while values from 0x00 to 0x79 indicate that authorization failed.
	While most applications will only use a single value to grant access,
	flexibility is provided to store multiple values in order to
	indicate which user (or user's group) logged in.

	The return value of this function is saved as curHTTP.isAuthorized, and
	will be available to future callbacks, including any of the
	HTTPExecuteGet, HTTPExecutePost, or HTTPPrint_varname callbacks.

  Precondition:
	None

  Parameters:
	connHandle  - HTTP connection handle
	cUser - the user name supplied by the client
	cPass - the password supplied by the client

  Return Values:
	<= 0x79 - the credentials were rejected
	>= 0x80 - access is granted for this connection

  Remarks:
	This function is only called when an Authorization header is
	encountered.

	This function may NOT write to the TCP buffer.
  ***************************************************************************/
uint8_t HTTPCheckAuth(HTTP_CONN_HANDLE connHandle, uint8_t* cUser, uint8_t* cPass);

/*****************************************************************************
  Function:
	void HTTPPrint_varname(HTTP_CONN_HANDLE connHandle)
	void HTTPPrint_varname(HTTP_CONN_HANDLE connHandle, uint16_t wParam1)
	void HTTPPrint_varname(HTTP_CONN_HANDLE connHandle, uint16_t wParam1, uint16_t wParam2, ...)

  Summary:
	Inserts dynamic content into a web page

  Description:
	Functions in this style are implemented by the application developer in
	CustomHTTPApp.c.  These functions generate dynamic content to be
	inserted into web pages and other files returned by the HTTP2 server.

	Functions of this type are called when a dynamic variable is located
	in a web page.  (ie, ~varname~ )  The name between the tilde '~'
	characters is appended to the base function name.  In this example, the
	callback would be named HTTPPrint_varname.

	The function prototype is located in your project's http_print.h, which
	is automatically generated by the MPFS2 Utility.  The prototype will
	have uint16_t parameters included for each parameter passed in the dynamic
	variable.  For example, the variable "~myArray(2,6)~" will generate the
	prototype "void HTTPPrint_varname(uint16_t, uint16_t);".

	When called, this function should write its output directly to the TCP
	socket using any combination of TCPIsPutReady, TCPPut, TCPPutArray,
	TCPPutString, TCPPutArray, and TCPPutString.

	Before calling, the HTTP2 server guarantees that at least
	HTTP_MIN_CALLBACK_FREE bytes (defaults to 16 bytes) are free in the
	output buffer.  If the function is writing less than this amount, it
	should simply write the data to the socket and return.

	In situations where a function needs to write more this amount, it
	must manage its output state using curHTTP.callbackPos.  This value
	will be set to zero before the function is called.  If the function is
	managing its output state, it must set this to a non-zero value before
	returning.  Typically this is used to track how many bytes have been
	written, or how many remain to be written.  If curHTTP.callbackPos is
	non-zero, the function will be called again when more buffer space is
	available.  Once the callback completes, set this value back to zero
	to resume normal servicing of the request.

  Precondition:
	None

  Parameters:
	connHandle  - HTTP connection handle
	wParam1 - first parameter passed in the dynamic variable (if any)
	wParam2 - second parameter passed in the dynamic variable (if any)
	... - additional parameters as necessary

  Returns:
  	None

  Remarks:
	This function may service multiple HTTP requests simultaneously,
	especially when managing its output state.  Exercise caution when using
	global or static variables inside this routine.  Use curHTTP.callbackPos
	or curHTTP.data for storage associated with individual requests.
  ***************************************************************************/
void HTTPPrint_varname(HTTP_CONN_HANDLE connHandle,  uint16_t wParam1, uint16_t wParam2, ...);


#endif  // __HTTP2_H_

