/*******************************************************************************
  HyperText Transfer Protocol (HTTP) Server

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    - Serves dynamic pages to web browsers such as Microsoft Internet 
      Explorer, Mozilla Firefox, etc.
    - Reference: RFC 2616
*******************************************************************************/

/*******************************************************************************
FileName:   HTTP2.c
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

#if defined(TCPIP_STACK_USE_HTTP2_SERVER)

#define TCPIP_THIS_MODULE_ID    TCPIP_MODULE_HTTP_SERVER

#include "http2_private.h"
#include "http_print.h"
#include "system/drivers/drv_media.h"
#include <system/fs/sys_fs.h>


/****************************************************************************
  Section:
	String Constants
  ***************************************************************************/
	static const uint8_t HTTP_CRLF[] = "\r\n";	// New line sequence
	#define HTTP_CRLF_LEN	2				// Length of above string
		
/****************************************************************************
  Section:
	File and Content Type Settings
  ***************************************************************************/
	// File type extensions corresponding to HTTP_FILE_TYPE
	static const char * const httpFileExtensions[HTTP_UNKNOWN+1] =
	{
	    "txt",          // HTTP_TXT
	    "htm",          // HTTP_HTM
	    "html",         // HTTP_HTML
	    "cgi",          // HTTP_CGI
	    "xml",          // HTTP_XML
	    "css",          // HTTP_CSS
	    "gif",          // HTTP_GIF
	    "png",          // HTTP_PNG
	    "jpg",          // HTTP_JPG
	    "cla",          // HTTP_JAVA
	    "wav",          // HTTP_WAV
		"\0\0\0"		// HTTP_UNKNOWN
	};
	
	// Content-type strings corresponding to HTTP_FILE_TYPE
	static const char * const httpContentTypes[HTTP_UNKNOWN+1] =
	{
	    "text/plain",            // HTTP_TXT
	    "text/html",             // HTTP_HTM
	    "text/html",             // HTTP_HTML
	    "text/html",             // HTTP_CGI
	    "text/xml",              // HTTP_XML
	    "text/css",              // HTTP_CSS
	    "image/gif",             // HTTP_GIF
	    "image/png",             // HTTP_PNG
	    "image/jpeg",            // HTTP_JPG
	    "application/java-vm",   // HTTP_JAVA
	    "audio/x-wave",          // HTTP_WAV
		""						 // HTTP_UNKNOWN
	};
		
/****************************************************************************
  Section:
	Commands and Server Responses
  ***************************************************************************/

	// Initial response strings (Corresponding to HTTP_STATUS)
	static const char * const HTTPResponseHeaders[] =
	{
		"HTTP/1.1 200 OK\r\nConnection: close\r\n",
		"HTTP/1.1 200 OK\r\nConnection: close\r\n",
		"HTTP/1.1 400 Bad Request\r\nConnection: close\r\n\r\n400 Bad Request: can't handle Content-Length\r\n",
		"HTTP/1.1 401 Unauthorized\r\nWWW-Authenticate: Basic realm=\"Protected\"\r\nConnection: close\r\n\r\n401 Unauthorized: Password required\r\n",
		#if defined(HTTP_FILE_UPLOAD)
		"HTTP/1.1 404 Not found\r\nConnection: close\r\nContent-Type: text/html\r\n\r\n404: File not found<br>Use <a href=\"/" HTTP_FILE_UPLOAD "\">MPFS Upload</a> to program web pages\r\n",
		#else		
		"HTTP/1.1 404 Not found\r\nConnection: close\r\n\r\n404: File not found\r\n",
		#endif
		"HTTP/1.1 414 Request-URI Too Long\r\nConnection: close\r\n\r\n414 Request-URI Too Long: Buffer overflow detected\r\n",
		"HTTP/1.1 500 Internal Server Error\r\nConnection: close\r\n\r\n500 Internal Server Error: Expected data not present\r\n",
		"HTTP/1.1 501 Not Implemented\r\nConnection: close\r\n\r\n501 Not Implemented: Only GET and POST supported\r\n",
		"HTTP/1.1 302 Found\r\nConnection: close\r\nLocation: ",
		"HTTP/1.1 403 Forbidden\r\nConnection: close\r\n\r\n403 Forbidden: SSL Required - use HTTPS\r\n",

		#if defined(HTTP_FILE_UPLOAD)
		"HTTP/1.1 200 OK\r\nConnection: close\r\nContent-Type: text/html\r\n\r\n<html><body style=\"margin:100px\"><form method=post action=\"/" HTTP_FILE_UPLOAD "\" enctype=\"multipart/form-data\"><b>MPFS Image Upload</b><p><input type=file name=i size=40> &nbsp; <input type=submit value=\"Upload\"></form></body></html>",
		"",
		"HTTP/1.1 200 OK\r\nConnection: close\r\nContent-Type: text/html\r\n\r\n<html><body style=\"margin:100px\"><b>MPFS Update Successful</b><p><a href=\"/\">Site main page</a></body></html>",
		"HTTP/1.1 500 Internal Server Error\r\nConnection: close\r\nContent-Type: text/html\r\n\r\n<html><body style=\"margin:100px\"><b>MPFS Image Corrupt or Wrong Version</b><p><a href=\"/" HTTP_FILE_UPLOAD "\">Try again?</a></body></html>",
		#endif

	};
	
/****************************************************************************
  Section:
	Header Parsing Configuration
  ***************************************************************************/
	
	// Header strings for which we'd like to parse
	static const char * const HTTPRequestHeaders[] =
	{
		"Cookie:",
		"Authorization:",
		"Content-Length:"
	};
	
/****************************************************************************
  Section:
	HTTP Connection State Global Variables
  ***************************************************************************/
static HTTP_CONN*           httpConnCtrl = 0;        // all http connections
static uint8_t*             httpConnData = 0;       // http conenctions data space
static int                  httpConnNo = 0;         // number of HTTP connections
static int                  httpInitCount = 0;      // module init counter
static HTTP_MODULE_FLAGS    httpConfigFlags = 0;    // run time flags

static const HTTP_MODULE_CONFIG httpConfigDefault = 
{
    HTTP_MAX_CONNECTIONS,
    HTTP_MAX_DATA_LEN,
    HTTP_SKT_TX_BUFF_SIZE,
    HTTP_SKT_RX_BUFF_SIZE,
    HTTP_CONFIG_FLAGS,

};


/****************************************************************************
  Section:
	Function Prototypes
  ***************************************************************************/
	static void HTTPHeaderParseLookup(HTTP_CONN* pHttpCon, int i);
	#if defined(HTTP_USE_COOKIES)
	static void HTTPHeaderParseCookie(HTTP_CONN* pHttpCon);
	#endif
	#if defined(HTTP_USE_AUTHENTICATION)
	static void HTTPHeaderParseAuthorization(HTTP_CONN* pHttpCon);
	#endif
	#if defined(HTTP_USE_POST)
	static void HTTPHeaderParseContentLength(HTTP_CONN* pHttpCon);
	static HTTP_READ_STATUS HTTPReadTo(HTTP_CONN* pHttpCon, uint8_t delim, uint8_t* buf, uint16_t len);
	#endif
	
	static void HTTPProcess(HTTP_CONN* pHttpCon);
	static bool HTTPSendFile(HTTP_CONN* pHttpCon);

	#if defined(HTTP_FILE_UPLOAD)
	static HTTP_IO_RESULT HTTPMPFSUpload(HTTP_CONN* pHttpCon);
	#endif

	#define mMIN(a, b)	((a<b)?a:b)


#if defined(HTTP_FILE_UPLOAD)
    static bool     HTTPSetMediaWriteHeader(HTTP_CONN* pHttpCon, const uint8_t* buffer, unsigned int nBytes);
    static void     HTTPReleaseMedia(HTTP_CONN* pHttpCon);
#endif  // defined(HTTP_FILE_UPLOAD)


static void _HttpCleanup(const TCPIP_STACK_MODULE_CTRL* const stackCtrl)
{
    if(httpConnData)
    {
        TCPIP_HEAP_Free(stackCtrl->memH, httpConnData);
        httpConnData = 0;
    }
    if(httpConnCtrl)
    {
        TCPIP_HEAP_Free(stackCtrl->memH, httpConnCtrl);
        httpConnCtrl = 0;
    }

    httpConnNo = 0;
}



/*****************************************************************************
  Function:
	bool HTTPInit(const TCPIP_STACK_MODULE_CTRL* const stackCtrl, 
        const HTTP_MODULE_CONFIG* httpInitData)

  Summary:
	Initializes the HTTP server module.

  Description:
	Sets all HTTP sockets to the listening state, and initializes the
	state machine and file handles for each connection.  If SSL is
	enabled, opens a socket on that port as well.

  Precondition:
	TCP must already be initialized.

  Parameters:
	None

  Returns:
  	true if initialization succeeded,
    false otherwise
  	
  Remarks:
	This function is called only one during lifetime of the application.
  ***************************************************************************/
bool HTTPInit(const TCPIP_STACK_MODULE_CTRL* const stackCtrl,
              const HTTP_MODULE_CONFIG* httpInitData)
{
    bool        initFail;
    int         conn;
    HTTP_CONN*  pHttpCon;
    uint8_t*    pHttpData;

    if(stackCtrl->stackAction == TCPIP_STACK_ACTION_IF_UP)
    {   // interface restart
        return true;
    }

    // stack init
    initFail = false;
    while(httpInitCount == 0)
    {   // first time we're run

        if(httpInitData == 0)
        {
            httpInitData = &httpConfigDefault;
        }
        httpConnNo = httpInitData->nConnections;
        httpConfigFlags = httpInitData->configFlags;

        if(httpConnNo)
        {
            httpConnCtrl = (HTTP_CONN*)TCPIP_HEAP_Calloc(stackCtrl->memH, httpConnNo, sizeof(*httpConnCtrl));
            httpConnData = (uint8_t*)TCPIP_HEAP_Malloc(stackCtrl->memH, httpConnNo * httpInitData->dataLen);
            if(httpConnCtrl == 0 || httpConnData == 0)
            {   // failed
                SYS_ERROR(SYS_ERROR_ERROR, " HTTP: Dynamic allocation failed");
                initFail = true;
                break;
            }

            // initialize all connections
            pHttpCon = httpConnCtrl + 0;
            pHttpData = httpConnData;
            for(conn = 0; conn < httpConnNo ; conn++)
            {
                pHttpCon->sm = SM_HTTP_IDLE;
                pHttpCon->socket = TCPOpenServer(IP_ADDRESS_TYPE_ANY, HTTP_PORT, 0);
                if( pHttpCon->socket == INVALID_SOCKET)
                {   // failed to open the socket
                    SYS_ERROR(SYS_ERROR_ERROR, " HTTP: Socket creation failed");
                    initFail = true;
                    break;
                }
                
                // set socket options
                if((httpConfigFlags & HTTP_MODULE_FLAG_NO_DELAY) != 0)
                {
                    void* tcpForceFlush = (void*)1;
                    TCPSetOptions(pHttpCon->socket, TCP_OPTION_NODELAY, (void*)tcpForceFlush);
                }
                if(httpInitData->sktTxBuffSize != 0)
                {
                    void* tcpBuffSize = (void*)(unsigned int)httpInitData->sktTxBuffSize;
                    TCPSetOptions(pHttpCon->socket, TCP_OPTION_TX_BUFF, tcpBuffSize);
                }
                if(httpInitData->sktRxBuffSize != 0)
                {
                    void* tcpBuffSize = (void*)(unsigned int)httpInitData->sktRxBuffSize;
                    TCPSetOptions(pHttpCon->socket, TCP_OPTION_RX_BUFF, tcpBuffSize);
                }

#if defined(TCPIP_STACK_USE_SSL_SERVER)
                TCPAddSSLListener(pHttpCon->socket, HTTPS_PORT);
#endif

                // Save the default record (just invalid file handles)
                pHttpCon->file = MPFS_INVALID_HANDLE;
                pHttpCon->offsets = MPFS_INVALID_HANDLE;
                pHttpCon->data = pHttpData;
                pHttpCon++;
                pHttpData += httpInitData->dataLen;
            }
        }

        break;
    }

    if(initFail)
    {
        _HttpCleanup(stackCtrl);
        return false;
    }

    httpInitCount++;
    return true;    
}

/*****************************************************************************
  Function:
	bool HTTPDeInit(void)

  Summary:
	DeInitializes the HTTP server module.

  Description:
	Takes down all HTTP sockets, the state machine and file handles for 
	each connection.  If SSL is enabled, closes a socket on that port as well.

  Precondition:
	None

  Parameters:
	None

  Returns:
	None
  	
  Remarks:
	This function is called only once during lifetime of the application.
  ***************************************************************************/
void HTTPDeInit(const TCPIP_STACK_MODULE_CTRL* const stackCtrl)
{
    // if(stackCtrl->stackAction == TCPIP_STACK_ACTION_DEINIT) // stack shut down
    // if(stackCtrl->stackAction == TCPIP_STACK_ACTION_IF_DOWN) // interface down

    // one way or another this interface is going down
    HTTP_CONN* pHttpCon;
	int  conn;
	
    pHttpCon = httpConnCtrl + 0;
	for (conn = 0; conn < httpConnNo; conn++)
	{
		// Close the connections that were associated with this interface
		if (pHttpCon->socket != INVALID_SOCKET)
		{
			if (TCPSocketGetNet(pHttpCon->socket) == stackCtrl->pNetIf)
			{
				TCPClose(pHttpCon->socket);
				pHttpCon->socket = INVALID_SOCKET;
				pHttpCon->file = MPFS_INVALID_HANDLE;
				pHttpCon->offsets = MPFS_INVALID_HANDLE;
			}
		}
        pHttpCon++;
	}
	
    if(stackCtrl->stackAction == TCPIP_STACK_ACTION_DEINIT)
    {   // stack shut down
        if(httpInitCount > 0)
        {   // we're up and running
            if(--httpInitCount == 0)
            {   // all closed
                // release resources
                _HttpCleanup(stackCtrl);
            }
        }
    }
}

/*****************************************************************************
  Function:
	void HTTPServer(void)

  Summary:
	Performs periodic tasks for the HTTP2 module.

  Description:
	Browses through each open connection and attempts to process any
	pending operations.

  Precondition:
	HTTPInit() must already be called.

  Parameters:
	None

  Returns:
  	None
  	
  Remarks:
	This function acts as a task (similar to one in an RTOS).  It
	performs its task in a co-operative manner, and the main application
	must call this function repeatedly to ensure that all open or new
	connections are served in a timely fashion.
  ***************************************************************************/
void HTTPServer(void)
{
    HTTP_CONN* pHttpCon;
	int conn;

    pHttpCon = httpConnCtrl + 0;
	for(conn = 0; conn < httpConnNo; conn++)
	{
		if(pHttpCon->socket == INVALID_SOCKET)
			continue;
		
		// If a socket is disconnected at any time 
		// forget about it and return to idle state.
		// Must do this here, otherwise we will wait until a new
		// connection arrives, which causes problems with Linux and with SSL
		if(TCPWasReset(pHttpCon->socket))
		{
			pHttpCon->sm = SM_HTTP_IDLE;

			// Make sure any opened files are closed
			if(pHttpCon->file != MPFS_INVALID_HANDLE)
			{
				MPFSClose(pHttpCon->file);
				pHttpCon->file = MPFS_INVALID_HANDLE;
			}
			if(pHttpCon->offsets != MPFS_INVALID_HANDLE)
			{
				MPFSClose(pHttpCon->offsets);
				pHttpCon->offsets = MPFS_INVALID_HANDLE;
			}

            if((httpConfigFlags & HTTP_MODULE_FLAG_ADJUST_SKT_FIFOS) != 0)
            {
                // Adjust FIFO sizes to half and half.  Default state must remain
                // here so that SSL handshakes, if required, can proceed
                TCPAdjustFIFOSize(pHttpCon->socket, 1, 0, TCP_ADJUST_PRESERVE_RX);
            }
		}
		
		// Determine if this connection is eligible for processing
		if(pHttpCon->sm != SM_HTTP_IDLE || TCPIsGetReady(pHttpCon->socket))
		{
			HTTPProcess(pHttpCon);
		}
        pHttpCon++;
	}
}

/*****************************************************************************
  Function:
	static void HTTPProcess(HTTP_CONN* pHttpCon)

  Description:
	Performs any pending operations for the currently loaded HTTP connection.

  Precondition:
	HTTPInit() has been called.

  Parameters:
	None

  Returns:
  	None
  ***************************************************************************/
static void HTTPProcess(HTTP_CONN* pHttpCon)
{
    uint16_t lenA, lenB;
	int i;
	uint8_t c;
    bool isDone;
	uint8_t *ext;
	uint8_t buffer[HTTP_MAX_HEADER_LEN+1];

    do
    {
        isDone = true;

        switch(pHttpCon->sm)
        {

        case SM_HTTP_IDLE:

			// Check how much data is waiting
			lenA = TCPIsGetReady(pHttpCon->socket);

			// If a connection has been made, then process the request
            if(lenA)
            {// Clear out state info and move to next state
				pHttpCon->ptrData = pHttpCon->data;
				pHttpCon->sm = SM_HTTP_PARSE_REQUEST;
				pHttpCon->isAuthorized = 0xff;
				pHttpCon->hasArgs = false;
				pHttpCon->httpTick = SYS_TICK_Get() + HTTP_TIMEOUT*SYS_TICK_TicksPerSecondGet();
				pHttpCon->callbackPos = 0xffffffff;
				pHttpCon->byteCount = 0;
				#if defined(HTTP_USE_POST)
				pHttpCon->smPost = 0x00;
				#endif
				
                if((httpConfigFlags & HTTP_MODULE_FLAG_ADJUST_SKT_FIFOS) != 0)
                {
                    // Adjust the TCP FIFOs for optimal reception of 
                    // the next HTTP request from the browser
                    TCPAdjustFIFOSize(pHttpCon->socket, 1, 0, TCP_ADJUST_PRESERVE_RX | TCP_ADJUST_GIVE_REST_TO_RX);
                }
 			}
 			else
 				// Don't break for new connections.  There may be 
 				// an entire request in the buffer already.
 				break;

		case SM_HTTP_PARSE_REQUEST:

			// Verify the entire first line is in the FIFO
			if(TCPFind(pHttpCon->socket, '\n', 0, 0, false) == 0xffff)
			{// First line isn't here yet
				if(TCPGetRxFIFOFree(pHttpCon->socket) == 0u)
				{// If the FIFO is full, we overflowed
					pHttpCon->httpStatus = HTTP_OVERFLOW;
					pHttpCon->sm = SM_HTTP_SERVE_HEADERS;
					isDone = false;
				}
				if(SYS_TICK_Get() > pHttpCon->httpTick)
				{// A timeout has occurred
					TCPDisconnect(pHttpCon->socket);
					pHttpCon->sm = SM_HTTP_DISCONNECT;
					isDone = false;
				}
				break;
			}

			// Reset the watchdog timer
			pHttpCon->httpTick = SYS_TICK_Get() + HTTP_TIMEOUT*SYS_TICK_TicksPerSecondGet();

			// Determine the request method
			lenA = TCPFind(pHttpCon->socket, ' ', 0, 0, false);
			if(lenA > 5u)
				lenA = 5;
			TCPGetArray(pHttpCon->socket, pHttpCon->data, lenA+1);

		    if ( memcmp(pHttpCon->data, (const void*)"GET", 3) == 0)
			    pHttpCon->httpStatus = HTTP_GET;
			#if defined(HTTP_USE_POST)
		    else if ( memcmp(pHttpCon->data, (const void*)"POST", 4) == 0)
			    pHttpCon->httpStatus = HTTP_POST;
			#endif
		    else
			{// Unrecognized method, so return not implemented
		        pHttpCon->httpStatus = HTTP_NOT_IMPLEMENTED;
				pHttpCon->sm = SM_HTTP_SERVE_HEADERS;
				isDone = false;
				break;
			}

			// Find end of filename
			lenA = TCPFind(pHttpCon->socket, ' ', 0, 0, false);
			lenB = TCPFind(pHttpCon->socket, '?', 0, lenA, false);
			lenA = mMIN(lenA, lenB);
			
			// If the file name is too long, then reject the request
			if(lenA > HTTP_MAX_DATA_LEN - HTTP_DEFAULT_LEN - 1)
			{
				pHttpCon->httpStatus = HTTP_OVERFLOW;
				pHttpCon->sm = SM_HTTP_SERVE_HEADERS;
				isDone = false;
				break;
			}

			// Read in the filename and decode
			lenB = TCPGetArray(pHttpCon->socket, pHttpCon->data, lenA);
			pHttpCon->data[lenB] = '\0';
			HTTPURLDecode(pHttpCon->data);

			// Decode may have changed the string length - update it here
			lenB = strlen((char*)pHttpCon->data);

			// Check if this is an MPFS Upload
			#if defined(HTTP_FILE_UPLOAD)
			if(memcmp(&pHttpCon->data[1], HTTP_FILE_UPLOAD, sizeof(HTTP_FILE_UPLOAD)) == 0)
			{// Read remainder of line, and bypass all file opening, etc.
				#if defined(HTTP_USE_AUTHENTICATION)
				pHttpCon->isAuthorized = HTTPNeedsAuth(pHttpCon, &pHttpCon->data[1]);
				#endif
				if(pHttpCon->httpStatus == HTTP_GET)
					pHttpCon->httpStatus = HTTP_MPFS_FORM;
				else
					pHttpCon->httpStatus = HTTP_MPFS_UP;

				pHttpCon->sm = SM_HTTP_PARSE_HEADERS;
				isDone = false;
				break;
			}
			#endif
			
			// If the last character is a not a directory delimiter, then try to open the file
			// String starts at 2nd character, because the first is always a '/'
			if(pHttpCon->data[lenB-1] != '/')
                              pHttpCon->file = MPFSOpen(&pHttpCon->data[1]);

			// If the open fails, then add our default name and try again
			if(pHttpCon->file == MPFS_INVALID_HANDLE)
			{
				// Add the directory delimiter if needed
				if(pHttpCon->data[lenB-1] != '/')
					pHttpCon->data[lenB++] = '/';
				
				// Add our default file name			
				#if defined(TCPIP_STACK_USE_SSL_SERVER)
				if(TCPIsSSL(pHttpCon->socket))
				{
					strcpy((void*)&pHttpCon->data[lenB], HTTPS_DEFAULT_FILE);
					lenB += strlen(HTTPS_DEFAULT_FILE);
				}
				else
				#endif
				{
					strcpy((void*)&pHttpCon->data[lenB], HTTP_DEFAULT_FILE);
					lenB += strlen(HTTP_DEFAULT_FILE);
				}	
				// Try to open again
				pHttpCon->file = MPFSOpen(&pHttpCon->data[1]);
			}
			
			// Find the extension in the filename
			for(ext = pHttpCon->data + lenB-1; ext != pHttpCon->data; ext--)
				if(*ext == '.')
					break;
					
			// Compare to known extensions to determine Content-Type
			ext++;
			for(pHttpCon->fileType = HTTP_TXT; pHttpCon->fileType < HTTP_UNKNOWN; pHttpCon->fileType++)
				if(!stricmppgm2ram(ext, (const void*)httpFileExtensions[pHttpCon->fileType]))
					break;
			
			// Perform first round authentication (pass file name only)
			#if defined(HTTP_USE_AUTHENTICATION)
			pHttpCon->isAuthorized = HTTPNeedsAuth(pHttpCon, &pHttpCon->data[1]);
			#endif
			
			// If the file was found, see if it has an index
			if(pHttpCon->file != MPFS_INVALID_HANDLE &&
				(MPFSGetFlags(pHttpCon->file) & MPFS2_FLAG_HASINDEX) )
			{
				pHttpCon->offsets = MPFSOpenID(MPFSGetID(pHttpCon->file) + 1);
			}

			// Read GET args, up to buffer size - 1
			lenA = TCPFind(pHttpCon->socket, ' ', 0, 0, false);
			if(lenA != 0u)
			{
				pHttpCon->hasArgs = true;
				
				// Trash the '?'
				TCPGet(pHttpCon->socket, &c);

				// Verify there's enough space
				lenA--;
				if(lenA >= HTTP_MAX_DATA_LEN - 2)
				{
			        pHttpCon->httpStatus = HTTP_OVERFLOW;
					pHttpCon->sm = SM_HTTP_SERVE_HEADERS;
					isDone = false;
					break;
				}

				// Read in the arguments and '&'-terminate in anticipation of cookies
				pHttpCon->ptrData += TCPGetArray(pHttpCon->socket, pHttpCon->data, lenA);
				*(pHttpCon->ptrData++) = '&';

			}

			// Clear the rest of the line
			lenA = TCPFind(pHttpCon->socket, '\n', 0, 0, false);
			TCPGetArray(pHttpCon->socket, NULL, lenA + 1);

			// Move to parsing the headers
			pHttpCon->sm = SM_HTTP_PARSE_HEADERS;
			
			// No break, continue to parsing headers

		case SM_HTTP_PARSE_HEADERS:

			// Loop over all the headers
			while(1)
			{
				// Make sure entire line is in the FIFO
				lenA = TCPFind(pHttpCon->socket, '\n', 0, 0, false);
				if(lenA == 0xffff)
				{// If not, make sure we can receive more data
					if(TCPGetRxFIFOFree(pHttpCon->socket) == 0u)
					{// Overflow
						pHttpCon->httpStatus = HTTP_OVERFLOW;
						pHttpCon->sm = SM_HTTP_SERVE_HEADERS;
						isDone = false;
					}
					if(SYS_TICK_Get() > pHttpCon->httpTick)
					{// A timeout has occured
						TCPDisconnect(pHttpCon->socket);
						pHttpCon->sm = SM_HTTP_DISCONNECT;
						isDone = false;
					}
					break;
				}
				
				// Reset the watchdog timer
				pHttpCon->httpTick = SYS_TICK_Get() + HTTP_TIMEOUT*SYS_TICK_TicksPerSecondGet();
				
				// If a CRLF is immediate, then headers are done
				if(lenA == 1u)
				{// Remove the CRLF and move to next state
					TCPGetArray(pHttpCon->socket, NULL, 2);
					pHttpCon->sm = SM_HTTP_AUTHENTICATE;
					isDone = false;
					break;
				}
	
				// Find the header name, and use isDone as a flag to indicate a match
				lenB = TCPFind(pHttpCon->socket, ':', 0, lenA, false) + 2;
				isDone = false;
	
				// If name is too long or this line isn't a header, ignore it
				if(lenB > sizeof(buffer))
				{
					TCPGetArray(pHttpCon->socket, NULL, lenA+1);
					continue;
				}
				
				// Read in the header name
				TCPGetArray(pHttpCon->socket, buffer, lenB);
				buffer[lenB-1] = '\0';
				lenA -= lenB;
		
				// Compare header read to ones we're interested in
				for(i = 0; i < sizeof(HTTPRequestHeaders)/sizeof(HTTPRequestHeaders[0]); i++)
				{
					if(strcmp((char*)buffer, (const char *)HTTPRequestHeaders[i]) == 0)
					{// Parse the header and stop the loop
						HTTPHeaderParseLookup(pHttpCon, i);
						isDone = true;
						break;
					}
				}
				
				// Clear the rest of the line, and call the loop again
				if(isDone)
				{// We already know how much to remove unless a header was found
					lenA = TCPFind(pHttpCon->socket, '\n', 0, 0, false);
				}
				TCPGetArray(pHttpCon->socket, NULL, lenA+1);
			}
			
			break;

		case SM_HTTP_AUTHENTICATE:
		
			#if defined(HTTP_USE_AUTHENTICATION)
			// Check current authorization state
			if(pHttpCon->isAuthorized < 0x80)
			{// 401 error
				pHttpCon->httpStatus = HTTP_UNAUTHORIZED;
				pHttpCon->sm = SM_HTTP_SERVE_HEADERS;
				isDone = false;
				
				#if defined(HTTP_NO_AUTH_WITHOUT_SSL)
				if(!TCPIsSSL(pHttpCon->socket))
					pHttpCon->httpStatus = HTTP_SSL_REQUIRED;
				#endif

				break;
			}
			#endif

			// Parse the args string
			*pHttpCon->ptrData = '\0';
			pHttpCon->ptrData = HTTPURLDecode(pHttpCon->data);

			// If this is an MPFS upload form request, bypass to headers
			#if defined(HTTP_FILE_UPLOAD)
			if(pHttpCon->httpStatus == HTTP_MPFS_FORM)
			{
				pHttpCon->sm = SM_HTTP_SERVE_HEADERS;
				isDone = false;
				break;
			}
			#endif
			
			// Move on to GET args, unless there are none
			pHttpCon->sm = SM_HTTP_PROCESS_GET;
			if(!pHttpCon->hasArgs)
				pHttpCon->sm = SM_HTTP_PROCESS_POST;
			isDone = false;
			pHttpCon->hasArgs = false;
			break;

		case SM_HTTP_PROCESS_GET:

			// Run the application callback HTTPExecuteGet()
			if(HTTPExecuteGet(pHttpCon) == HTTP_IO_WAITING)
			{// If waiting for asynchronous process, return to main app
				break;
			}

			// Move on to POST data
			pHttpCon->sm = SM_HTTP_PROCESS_POST;

		case SM_HTTP_PROCESS_POST:

			#if defined(HTTP_USE_POST)
			
			// See if we have any new data
			if(TCPIsGetReady(pHttpCon->socket) == pHttpCon->callbackPos)
			{
				if(SYS_TICK_Get() > pHttpCon->httpTick)
				{// If a timeout has occured, disconnect
					TCPDisconnect(pHttpCon->socket);
					pHttpCon->sm = SM_HTTP_DISCONNECT;
					isDone = false;
					break;
				}
			}
			
			if(pHttpCon->httpStatus == HTTP_POST 
				#if defined(HTTP_FILE_UPLOAD)
				|| (pHttpCon->httpStatus >= HTTP_MPFS_UP && pHttpCon->httpStatus <= HTTP_MPFS_ERROR)
				#endif
				 )
			{
				// Run the application callback HTTPExecutePost()
				#if defined(HTTP_FILE_UPLOAD)
				if(pHttpCon->httpStatus >= HTTP_MPFS_UP && pHttpCon->httpStatus <= HTTP_MPFS_ERROR)
				{
					c = HTTPMPFSUpload(pHttpCon);
					if(c == (uint8_t)HTTP_IO_DONE)
					{
						pHttpCon->sm = SM_HTTP_SERVE_HEADERS;
						isDone = false;
						break;
					}
				}
				else
				#endif
				c = HTTPExecutePost(pHttpCon);
				
				// If waiting for asynchronous process, return to main app
				if(c == (uint8_t)HTTP_IO_WAITING)
				{// return to main app and make sure we don't get stuck by the watchdog
					pHttpCon->callbackPos = TCPIsGetReady(pHttpCon->socket) - 1;
					break;
				}
				else if(c == (uint8_t)HTTP_IO_NEED_DATA)
				{// If waiting for more data
					pHttpCon->callbackPos = TCPIsGetReady(pHttpCon->socket);
					pHttpCon->httpTick = SYS_TICK_Get() + HTTP_TIMEOUT*SYS_TICK_TicksPerSecondGet();
					
					// If more is expected and space is available, return to main app
					if(pHttpCon->byteCount > pHttpCon->callbackPos && TCPGetRxFIFOFree(pHttpCon->socket) != 0u)
						break;
					
					// Handle cases where application ran out of data or buffer space
					pHttpCon->httpStatus = HTTP_INTERNAL_SERVER_ERROR;
					pHttpCon->sm = SM_HTTP_SERVE_HEADERS;
					isDone = false;
					break;	
				}
			}
			#endif

			// We're done with POST
			pHttpCon->sm = SM_HTTP_PROCESS_REQUEST;
			// No break, continue to sending request

		case SM_HTTP_PROCESS_REQUEST:

			// Check for 404
            if(pHttpCon->file == MPFS_INVALID_HANDLE)
            {
                pHttpCon->httpStatus = HTTP_NOT_FOUND;
                pHttpCon->sm = SM_HTTP_SERVE_HEADERS;
                isDone = false;
                break;
            }

			// Set up the dynamic substitutions
			pHttpCon->byteCount = 0;
			if(pHttpCon->offsets == MPFS_INVALID_HANDLE)
            {// If no index file, then set next offset to huge
	            pHttpCon->nextCallback = 0xffffffff;
            }
            else
            {// Read in the next callback index
	            MPFSGetLong(pHttpCon->offsets, &(pHttpCon->nextCallback));
			}
			
			// Move to next state
			pHttpCon->sm = SM_HTTP_SERVE_HEADERS;

		case SM_HTTP_SERVE_HEADERS:

			// We're in write mode now:
            if((httpConfigFlags & HTTP_MODULE_FLAG_ADJUST_SKT_FIFOS) != 0)
            {
                // Adjust the TCP FIFOs for optimal transmission of 
                // the HTTP response to the browser
                TCPAdjustFIFOSize(pHttpCon->socket, 1, 0, TCP_ADJUST_GIVE_REST_TO_TX);
            }
				
			// Send headers
			TCPPutString(pHttpCon->socket, (const uint8_t*)HTTPResponseHeaders[pHttpCon->httpStatus]);
			
			// If this is a redirect, print the rest of the Location: header			   
			if(pHttpCon->httpStatus == HTTP_REDIRECT)
			{
				TCPPutString(pHttpCon->socket, pHttpCon->data);
				TCPPutString(pHttpCon->socket, (const uint8_t*)"\r\n\r\n304 Redirect: ");
				TCPPutString(pHttpCon->socket, pHttpCon->data);
				TCPPutString(pHttpCon->socket, (const uint8_t*)HTTP_CRLF);
			}

			// If not GET or POST, we're done
			if(pHttpCon->httpStatus != HTTP_GET && pHttpCon->httpStatus != HTTP_POST)
			{// Disconnect
				pHttpCon->sm = SM_HTTP_DISCONNECT;
				break;
			}

			// Output the content type, if known
			if(pHttpCon->fileType != HTTP_UNKNOWN)
			{
				TCPPutString(pHttpCon->socket, (const uint8_t*)"Content-Type: ");
				TCPPutString(pHttpCon->socket, (const uint8_t*)httpContentTypes[pHttpCon->fileType]);
				TCPPutString(pHttpCon->socket, HTTP_CRLF);
			}
			
			// Output the gzip encoding header if needed
			if(MPFSGetFlags(pHttpCon->file) & MPFS2_FLAG_ISZIPPED)
			{
				TCPPutString(pHttpCon->socket, (const uint8_t*)"Content-Encoding: gzip\r\n");
			}
						
			// Output the cache-control
			TCPPutString(pHttpCon->socket, (const uint8_t*)"Cache-Control: ");
			if(pHttpCon->httpStatus == HTTP_POST || pHttpCon->nextCallback != 0xffffffff)
			{// This is a dynamic page or a POST request, so no cache
				TCPPutString(pHttpCon->socket, (const uint8_t*)"no-cache");
			}
			else
			{// This is a static page, so save it for the specified amount of time
				TCPPutString(pHttpCon->socket, (const uint8_t*)"max-age=");
				TCPPutString(pHttpCon->socket, (const uint8_t*)HTTP_CACHE_LEN);
			}
			TCPPutString(pHttpCon->socket, HTTP_CRLF);
			
			// Check if we should output cookies
			if(pHttpCon->hasArgs)
				pHttpCon->sm = SM_HTTP_SERVE_COOKIES;
			else
			{// Terminate the headers
				TCPPutString(pHttpCon->socket, HTTP_CRLF);
				pHttpCon->sm = SM_HTTP_SERVE_BODY;
			}
	
			// Move to next stage
			isDone = false;
			break;

		case SM_HTTP_SERVE_COOKIES:

			#if defined(HTTP_USE_COOKIES)
			// If the TX FIFO runs out of space, the client will never get CRLFCRLF
			// Avoid writing huge cookies - keep it under a hundred bytes max

			// Write cookies one at a time as space permits
			for(pHttpCon->ptrRead = pHttpCon->data; pHttpCon->hasArgs != 0u; pHttpCon->hasArgs--)
			{
				// Write the header
				TCPPutString(pHttpCon->socket, (const uint8_t*)"Set-Cookie: ");

				// Write the name, URL encoded, one character at a time
				while((c = *(pHttpCon->ptrRead++)))
				{
					if(c == ' ')
						TCPPut(pHttpCon->socket, '+');
					else if(c < '0' || (c > '9' && c < 'A') || (c > 'Z' && c < 'a') || c > 'z')
					{
						TCPPut(pHttpCon->socket, '%');
						TCPPut(pHttpCon->socket, btohexa_high(c));
						TCPPut(pHttpCon->socket, btohexa_low(c));
					}
					else
						TCPPut(pHttpCon->socket, c);
				}
				
				TCPPut(pHttpCon->socket, '=');
				
				// Write the value, URL encoded, one character at a time
				while((c = *(pHttpCon->ptrRead++)))
				{
					if(c == ' ')
						TCPPut(pHttpCon->socket, '+');
					else if(c < '0' || (c > '9' && c < 'A') || (c > 'Z' && c < 'a') || c > 'z')
					{
						TCPPut(pHttpCon->socket, '%');
						TCPPut(pHttpCon->socket, btohexa_high(c));
						TCPPut(pHttpCon->socket, btohexa_low(c));
					}
					else
						TCPPut(pHttpCon->socket, c);
				}
				
				// Finish the line
				TCPPutString(pHttpCon->socket, HTTP_CRLF);

			}
			#endif

			// We're done, move to next state
			TCPPutString(pHttpCon->socket, HTTP_CRLF);
			pHttpCon->sm = SM_HTTP_SERVE_BODY;

		case SM_HTTP_SERVE_BODY:

			isDone = false;

			// Try to send next packet
			if(HTTPSendFile(pHttpCon))
			{// If EOF, then we're done so close and disconnect
				MPFSClose(pHttpCon->file);
				pHttpCon->file = MPFS_INVALID_HANDLE;
				pHttpCon->sm = SM_HTTP_DISCONNECT;
				isDone = true;
			}
			
			// If the TX FIFO is full, then return to main app loop
			if(TCPIsPutReady(pHttpCon->socket) == 0u)
				isDone = true;
            break;

		case SM_HTTP_SEND_FROM_CALLBACK:

			isDone = true;

			// Check that at least the minimum bytes are free
			if(TCPIsPutReady(pHttpCon->socket) < HTTP_MIN_CALLBACK_FREE)
				break;

			// Fill TX FIFO from callback
			HTTPPrint(pHttpCon, pHttpCon->callbackID);
			
			if(pHttpCon->callbackPos == 0u)
			{// Callback finished its output, so move on
				isDone = false;
				pHttpCon->sm = SM_HTTP_SERVE_BODY;
			}// Otherwise, callback needs more buffer space, so return and wait
			
			break;

		case SM_HTTP_DISCONNECT:
			// Make sure any opened files are closed
			if(pHttpCon->file != MPFS_INVALID_HANDLE)
			{
				MPFSClose(pHttpCon->file);
				pHttpCon->file = MPFS_INVALID_HANDLE;
			}
			if(pHttpCon->offsets != MPFS_INVALID_HANDLE)
			{
				MPFSClose(pHttpCon->offsets);
				pHttpCon->offsets = MPFS_INVALID_HANDLE;
			}

			TCPDisconnect(pHttpCon->socket);
            pHttpCon->sm = SM_HTTP_IDLE;
            break;
		}
	} while(!isDone);

}


/*****************************************************************************
  Function:
	static bool HTTPSendFile(HTTP_CONN* pHttpCon)

  Description:
	Serves up the next chunk of curHTTP's file, up to:
  a) the available TX FIFO space or
  b) the next callback index, whichever comes first.

  Precondition:
	pHttpCon->file and pHttpCon->offsets have both been opened for reading.

  Parameters:
	None

  Return Values:
	true - the end of the file was reached and reading is done
	false - more data remains to be read
  ***************************************************************************/
static bool HTTPSendFile(HTTP_CONN* pHttpCon)
{
	uint16_t numBytes, len;
	uint8_t c, data[64];
	
	// Determine how many bytes we can read right now
	len = TCPIsPutReady(pHttpCon->socket);
	numBytes = mMIN(len, pHttpCon->nextCallback - pHttpCon->byteCount);
	
	// Get/put as many bytes as possible
	pHttpCon->byteCount += numBytes;
	while(numBytes > 0u)
	{
		len = MPFSGetArray(pHttpCon->file, data, mMIN(numBytes, sizeof(data)));
		if(len == 0u)
			return true;
		else
			TCPPutArray(pHttpCon->socket, data, len);
		numBytes -= len;
	}
	
	// Check if a callback index was reached
	if(pHttpCon->byteCount == pHttpCon->nextCallback)
	{
		// Update the state machine
		pHttpCon->sm = SM_HTTP_SEND_FROM_CALLBACK;
		pHttpCon->callbackPos = 0;

		// Read past the variable name and close the MPFS
		MPFSGet(pHttpCon->file, NULL);
		do
		{
			if(!MPFSGet(pHttpCon->file, &c))
				break;
			pHttpCon->byteCount++;
		} while(c != '~');
		pHttpCon->byteCount++;
		
		// Read in the callback address and next offset
		MPFSGetLong(pHttpCon->offsets, &(pHttpCon->callbackID));
		if(!MPFSGetLong(pHttpCon->offsets, &(pHttpCon->nextCallback)))
		{
			pHttpCon->nextCallback = 0xffffffff;
			MPFSClose(pHttpCon->offsets);
			pHttpCon->offsets = MPFS_INVALID_HANDLE;
		}
	}

    // We are not done sending a file yet...
    return false;
}

/*****************************************************************************
  Function:
	static void HTTPHeaderParseLookup(HTTP_CONN* pHttpCon, int i)

  Description:
	Calls the appropriate header parser based on the index of the header
	that was read from the request.

  Precondition:
	None

  Parameters:
	i - the index of the string found in HTTPRequestHeaders

  Return Values:
	true - the end of the file was reached and reading is done
	false - more data remains to be read
  ***************************************************************************/
static void HTTPHeaderParseLookup(HTTP_CONN* pHttpCon, int i)
{
	// i corresponds to an index in HTTPRequestHeaders
	
	#if defined(HTTP_USE_COOKIES)
	if(i == 0u)
	{
		HTTPHeaderParseCookie(pHttpCon);
		return;
	}
	#endif
	
	#if defined(HTTP_USE_AUTHENTICATION)	
	if(i == 1u)
	{
		HTTPHeaderParseAuthorization(pHttpCon);
		return;
	}
	#endif
	
	#if defined(HTTP_USE_POST)
	if(i == 2u)
	{
		HTTPHeaderParseContentLength(pHttpCon);
		return;
	}
	#endif
}

/*****************************************************************************
  Function:
	static void HTTPHeaderParseAuthorization(HTTP_CONN* pHttpCon)

  Summary:
	Parses the "Authorization:" header for a request and verifies the
	credentials.

  Description:
	Parses the "Authorization:" header for a request.  For example, 
	"BASIC YWRtaW46cGFzc3dvcmQ=" is decoded to a user name of "admin" and
	a password of "password".  Once read, HTTPCheckAuth is called from
	CustomHTTPApp.c to determine if the credentials are acceptable.

	The return value of HTTPCheckAuth is saved in pHttpCon->isAuthorized for
	later use by the application.

  Precondition:
	None

  Parameters:
	None

  Returns:
	None

  Remarks:
	This function is ony available when HTTP_USE_AUTHENTICATION is defined.
  ***************************************************************************/
#if defined(HTTP_USE_AUTHENTICATION)
static void HTTPHeaderParseAuthorization(HTTP_CONN* pHttpCon)
{
    uint16_t len;
    uint8_t buf[40];
	uint8_t *ptrBuf;
	
	// If auth processing is not required, return
	if(pHttpCon->isAuthorized & 0x80)
		return;

	// Clear the auth type ("BASIC ")
	TCPGetArray(pHttpCon->socket, NULL, 6);

	// Find the terminating CRLF and make sure it's a multiple of four
	len = TCPFindArray(pHttpCon->socket, HTTP_CRLF, HTTP_CRLF_LEN, 0, 0, false);
	len += 3;
	len &= 0xfc;
	len = mMIN(len, sizeof(buf)-4);
	
	// Read in 4 bytes at a time and decode (slower, but saves RAM)
	for(ptrBuf = buf; len > 0u; len-=4, ptrBuf+=3)
	{
		TCPGetArray(pHttpCon->socket, ptrBuf, 4);
		TCPIP_Helper_Base64Decode(ptrBuf, 4, ptrBuf, 3);
	}

	// Null terminate both, and make sure there's at least two terminators
	*ptrBuf = '\0';
	for(len = 0, ptrBuf = buf; len < sizeof(buf); len++, ptrBuf++)
		if(*ptrBuf == ':')
			break;
	*(ptrBuf++) = '\0';
	
	// Verify credentials
	pHttpCon->isAuthorized = HTTPCheckAuth(pHttpCon, buf, ptrBuf);

	return;
}
#endif

/*****************************************************************************
  Function:
	static void HTTPHeaderParseCookie(HTTP_CONN* pHttpCon)

  Summary:
	Parses the "Cookie:" headers for a request and stores them as GET
	variables.

  Description:
	Parses the "Cookie:" headers for a request.  For example, 
 	"Cookie: name=Wile+E.+Coyote; order=ROCKET_LAUNCHER" is decoded to 
	"name=Wile+E.+Coyote&order=ROCKET_LAUNCHER&" and stored as any other 
	GET variable in pHttpCon->data.

	The user application can easily access these values later using the
	HTTPGetArg() and HTTPGetArg() functions.

  Precondition:
	None

  Parameters:
	None

  Returns:
	None

  Remarks:
	This function is ony available when HTTP_USE_COOKIES is defined.
  ***************************************************************************/
#if defined(HTTP_USE_COOKIES)
static void HTTPHeaderParseCookie(HTTP_CONN* pHttpCon)
{
	uint16_t lenA, lenB;

	// Verify there's enough space
	lenB = TCPFindArray(pHttpCon->socket, HTTP_CRLF, HTTP_CRLF_LEN, 0, 0, false);
	if(lenB >= (uint16_t)(pHttpCon->data + HTTP_MAX_DATA_LEN - pHttpCon->ptrData - 2))
	{// If not, overflow
		pHttpCon->httpStatus = HTTP_OVERFLOW;
		pHttpCon->sm = SM_HTTP_SERVE_HEADERS;
		return;
	}

	// While a CRLF is not immediate, grab a cookie value
	while(lenB != 0u)
	{
		// Look for a ';' and use the shorter of that or a CRLF
		lenA = TCPFind(pHttpCon->socket, ';', 0, 0, false);
		
		// Read to the terminator
		pHttpCon->ptrData += TCPGetArray(pHttpCon->socket, pHttpCon->ptrData, mMIN(lenA, lenB));
		
		// Insert an & to anticipate another cookie
		*(pHttpCon->ptrData++) = '&';
		
		// If semicolon, trash it and whitespace
		if(lenA < lenB)
		{
			TCPGet(pHttpCon->socket, NULL);
			while(TCPFind(pHttpCon->socket, ' ', 0, 0, false) == 0u)
				TCPGet(pHttpCon->socket, NULL);
		}
		
		// Find the new distance to the CRLF
		lenB = TCPFindArray(pHttpCon->socket, HTTP_CRLF, HTTP_CRLF_LEN, 0, 0, false);
	}

	return;

}
#endif

/*****************************************************************************
  Function:
	static void HTTPHeaderParseContentLength(HTTP_CONN* pHttpCon)

  Summary:
	Parses the "Content-Length:" header for a request.

  Description:
	Parses the "Content-Length:" header to determine how many bytes of
	POST data to expect after the request.  This value is stored in 
	pHttpCon->byteCount.

  Precondition:
	None

  Parameters:
	None

  Returns:
	None

  Remarks:
	This function is ony available when HTTP_USE_POST is defined.
  ***************************************************************************/
#if defined(HTTP_USE_POST)
static void HTTPHeaderParseContentLength(HTTP_CONN* pHttpCon)
{
	uint16_t len;
	uint8_t buf[10];

	// Read up to the CRLF (max 9 bytes or ~1GB)
	len = TCPFindArray(pHttpCon->socket, HTTP_CRLF, HTTP_CRLF_LEN, 0, 0, false);
	if(len >= sizeof(buf))
	{
		pHttpCon->httpStatus = HTTP_BAD_REQUEST;
		pHttpCon->byteCount = 0;
		return;
	}	
	len = TCPGetArray(pHttpCon->socket, buf, len);
	buf[len] = '\0';
	
	pHttpCon->byteCount = atol((char*)buf);
}
#endif

/*****************************************************************************
  Function:
	uint8_t* HTTPURLDecode(uint8_t* cData)

  Summary:
	Parses a string from URL encoding to plain-text.

  Description:
	Parses a string from URL encoding to plain-text.  The following
	conversions are made: ‘=’ to ‘\0’, ‘&’ to ‘\0’, ‘+’ to ‘ ‘, and
	“%xx” to a single hex byte.
 
	After completion, the data has been decoded and a null terminator
	signifies the end of a name or value.  A second null terminator (or a
	null name parameter) indicates the end of all the data.

  Precondition:
	The data parameter is null terminated and has at least one extra
	byte free.

  Parameters:
	cData - The string which is to be decoded in place.

  Returns:
	A pointer to the last null terminator in data, which is also the
	first free byte for new data.

  Remarks:
	This function is called by the stack to parse GET arguments and 
	cookie data.  User applications can use this function to decode POST
	data, but first need to verify that the string is null-terminated.
  ***************************************************************************/
uint8_t* HTTPURLDecode(uint8_t* cData)
{
	uint8_t *pRead, *pWrite;
	uint16_t wLen;
	uint8_t c;
	uint16_t hex;
	 
	// Determine length of input
	wLen = strlen((char*)cData);
	 
	// Read all characters in the string
	for(pRead = pWrite = cData; wLen != 0u; )
	{
		c = *pRead++;
		wLen--;
		
		if(c == '=' || c == '&')
			*pWrite++ = '\0';
		else if(c == '+')
			*pWrite++ = ' ';
		else if(c == '%')
		{
			if(wLen < 2u)
				wLen = 0;
			else
			{
				((uint8_t*)&hex)[1] = *pRead++;
				((uint8_t*)&hex)[0] = *pRead++;
				wLen--;
				wLen--;
				*pWrite++ = hexatob(hex);
			}
		}
		else
			*pWrite++ = c;
	}
	
	// Double null terminate the last value
	*pWrite++ = '\0';
	*pWrite = '\0';
	
	return pWrite;
}

/*****************************************************************************
  Function:
	const uint8_t* HTTPGetArg(const uint8_t* cData, const uint8_t* cArg)

  Summary:
	Locates a form field value in a given data array.

  Description:
	Searches through a data array to find the value associated with a
	given argument.  It can be used to find form field values in data
	received over GET or POST.
	
	The end of data is assumed to be reached when a null name parameter is
	encountered.  This requires the string to have an even number of 
	null-terminated strings, followed by an additional null terminator.

  Precondition:
	The data array has a valid series of null terminated name/value pairs.

  Parameters:
	data - the buffer to search
	arg - the name of the argument to find

  Returns:
	A pointer to the argument value, or NULL if not found.
  ***************************************************************************/
const uint8_t* HTTPGetArg(const uint8_t* cData, const uint8_t* cArg)
{
	// Search through the array while bytes remain
	while(*cData != '\0')
	{ 
		// Look for arg at current position
		if(!strcmp((const char*)cArg, (const char*)cData))
		{// Found it, so return parameter
			return cData + strlen((const char*)cArg) + 1;
		}
		
		// Skip past two strings (NUL bytes)
		cData += strlen((const char*)cData) + 1;
		cData += strlen((const char*)cData) + 1;
	}
	 	
	// Return NULL if not found
	return NULL;
}


/*****************************************************************************
  Function:
	HTTP_READ_STATUS HTTPReadPostName(HTTP_CONN_HANDLE connHandle, uint8_t* cData, uint16_t wLen)

  Summary:
	Reads a name from a URL encoded string in the TCP buffer.

  Description:
	Reads a name from a URL encoded string in the TCP buffer.  This function
	is meant to be called from an HTTPExecutePost callback to facilitate
	easier parsing of incoming data.  This function also prevents buffer
	overflows by forcing the programmer to indicate how many bytes are
	expected.  At least 2 extra bytes are needed in cData over the maximum
	length of data expected to be read.
	
	This function will read until the next '=' character, which indicates the
	end of a name parameter.  It assumes that the front of the buffer is
	the beginning of the name paramter to be read.
	
	This function properly updates pHttpCon->byteCount by decrementing it
	by the number of bytes read.  It also removes the delimiting '=' from
	the buffer.

  Precondition:
	Front of TCP buffer is the beginning of a name parameter, and the rest of
	the TCP buffer contains a URL-encoded string with a name parameter 
	terminated by a '=' character.

  Parameters:
	connHandle  - HTTP connection handle
	cData - where to store the name once it is read
	wLen - how many bytes can be written to cData

  Return Values:
	HTTP_READ_OK - name was successfully read
	HTTP_READ_TRUNCTATED - entire name could not fit in the buffer, so the
							value was truncated and data has been lost
	HTTP_READ_INCOMPLETE - entire name was not yet in the buffer, so call
							this function again later to retrieve
  ***************************************************************************/
#if defined(HTTP_USE_POST)
HTTP_READ_STATUS HTTPReadPostName(HTTP_CONN_HANDLE connHandle, uint8_t* cData, uint16_t wLen)
{
	HTTP_READ_STATUS status;
    HTTP_CONN* pHttpCon = (HTTP_CONN*)connHandle;
	
	status = HTTPReadTo(pHttpCon, '=', cData, wLen);

	// Decode the data (if not reading to null or blank) and return
	if(cData && *cData)
    {
		HTTPURLDecode(cData);
    }
	return status;
}	
#endif

/*****************************************************************************
  Function:
	HTTP_READ_STATUS HTTPReadPostValue(HTTP_CONN_HANDLE connHandle, uint8_t* cData, uint16_t wLen)

  Summary:
	Reads a value from a URL encoded string in the TCP buffer.

  Description:
	Reads a value from a URL encoded string in the TCP buffer.  This function
	is meant to be called from an HTTPExecutePost callback to facilitate
	easier parsing of incoming data.  This function also prevents buffer
	overflows by forcing the programmer to indicate how many bytes are
	expected.  At least 2 extra bytes are needed in cData above the maximum
	length of data expected to be read.
	
	This function will read until the next '&' character, which indicates the
	end of a value parameter.  It assumes that the front of the buffer is
	the beginning of the value paramter to be read.  If pHttpCon->byteCount
	indicates that all expected bytes are in the buffer, it assumes that 
	all remaining data is the value and acts accordingly.
	
	This function properly updates pHttpCon->byteCount by decrementing it
	by the number of bytes read.  The terminating '&' character is also 
	removed from the buffer.
	
  Precondition:
	Front of TCP buffer is the beginning of a name parameter, and the rest of
	the TCP buffer contains a URL-encoded string with a name parameter 
	terminated by a '=' character.

  Parameters:
	connHandle  - HTTP connection handle
	cData - where to store the value once it is read
	wLen - how many bytes can be written to cData

  Return Values:
	HTTP_READ_OK - value was successfully read
	HTTP_READ_TRUNCTATED - entire value could not fit in the buffer, so the
							value was truncated and data has been lost
	HTTP_READ_INCOMPLETE - entire value was not yet in the buffer, so call
							this function again later to retrieve
  ***************************************************************************/
#if defined(HTTP_USE_POST)
HTTP_READ_STATUS HTTPReadPostValue(HTTP_CONN_HANDLE connHandle, uint8_t* cData, uint16_t wLen)
{
	HTTP_READ_STATUS status;
    HTTP_CONN* pHttpCon = (HTTP_CONN*)connHandle;
	
	// Try to read the value
	status = HTTPReadTo(pHttpCon, '&', cData, wLen);
	
	// If read was incomplete, check if we're at the end
	if(status == HTTP_READ_INCOMPLETE)
	{
		// If all data has arrived, read all remaining data
		if(pHttpCon->byteCount == TCPIsGetReady(pHttpCon->socket))
			status = HTTPReadTo(pHttpCon, '\0', cData, wLen);
	}
		
	// Decode the data (if not reading to null or blank) and return
	if(cData && *cData)
		HTTPURLDecode(cData);
	return status;
}	
#endif

/*****************************************************************************
  Function:
	static HTTP_READ_STATUS HTTPReadTo(HTTP_CONN* pHttpCon, uint8_t cDelim, uint8_t* cData, uint16_t wLen)

  Summary:
	Reads to a buffer until a specified delimiter character.

  Description:
	Reads from the TCP buffer to cData until either cDelim is reached, or
	until wLen - 2 bytes have been read.  The value read is saved to cData and 
	null terminated.  (wLen - 2 is used so that the value can be passed to
	HTTPURLDecode later, which requires a null terminator plus one extra free
	byte.)
	
	The delimiter character is removed from the buffer, but not saved to 
	cData. If all data cannot fit into cData, it will still be removed from 
	the buffer but will not be saved anywhere.

	This function properly updates pHttpCon->byteCount by decrementing it
	by the number of bytes read. 

  Precondition:
	None

  Parameters:
  	cDelim - the character at which to stop reading, or NULL to read to
  			 the end of the buffer
	cData - where to store the data being read
	wLen - how many bytes can be written to cData

  Return Values:
	HTTP_READ_OK - data was successfully read
	HTTP_READ_TRUNCTATED - entire data could not fit in the buffer, so the
							data was truncated and data has been lost
	HTTP_READ_INCOMPLETE - delimiter character was not found
  ***************************************************************************/
#if defined(HTTP_USE_POST)
static HTTP_READ_STATUS HTTPReadTo(HTTP_CONN* pHttpCon, uint8_t cDelim, uint8_t* cData, uint16_t wLen)
{
	HTTP_READ_STATUS status;
	uint16_t wPos;
	
	// Either look for delimiter, or read all available data
	if(cDelim)
		wPos = TCPFind(pHttpCon->socket, cDelim, 0, 0, false);
	else
		wPos = TCPIsGetReady(pHttpCon->socket);
	
	// If not found, return incomplete
	if(wPos == 0xffff)
		return HTTP_READ_INCOMPLETE;
	
	// Read the value
	if(wLen < 2u && cData != NULL)
	{// Buffer is too small, so read to NULL instead
		pHttpCon->byteCount -= TCPGetArray(pHttpCon->socket, NULL, wPos);
		status = HTTP_READ_TRUNCATED;
	}
	else if(cData == NULL)
	{// Just remove the data
		pHttpCon->byteCount -= TCPGetArray(pHttpCon->socket, NULL, wPos);
		status = HTTP_READ_OK;
	}
	else if(wPos > wLen - 2)
	{// Read data, but truncate at max length
		pHttpCon->byteCount -= TCPGetArray(pHttpCon->socket, cData, wLen - 2);
		pHttpCon->byteCount -= TCPGetArray(pHttpCon->socket, NULL, wPos - (wLen - 2));
		cData[wLen - 2] = '\0';
		status = HTTP_READ_TRUNCATED;
	}
	else
	{// Read the data normally
		pHttpCon->byteCount -= TCPGetArray(pHttpCon->socket, cData, wPos);
		cData[wPos] = '\0';
		status = HTTP_READ_OK;
	}
	
	// Remove the delimiter
	if(cDelim)
		pHttpCon->byteCount -= TCPGet(pHttpCon->socket, NULL);
	
	return status;
}	
#endif

/*****************************************************************************
  Function:
	HTTP_IO_RESULT HTTPMPFSUpload(HTTP_CONN* pHttpCon)

  Summary:
	Saves a file uploaded via POST as the new MPFS image in EEPROM or 
	external Flash.

  Description:
	Allows the MPFS image in EEPROM or external Flash to be updated via a 
	web page by accepting a file upload and storing it to the external memory.

  Precondition:
	None

  Parameters:
	None

  Return Values:
	HTTP_IO_DONE - on success
	HTTP_IO_NEED_DATA - if more data is still expected

  Remarks:
	This function is only available when MPFS uploads are enabled and
	the MPFS image is stored in EEPROM.

  Internal:
	After the headers, the first line from the form will be the MIME
	separator.  Following that is more headers about the file, which
	are discarded.  After another CRLFCRLF pair the file data begins,
	which is read 16 bytes at a time and written to external memory.
  ***************************************************************************/
#if defined(HTTP_FILE_UPLOAD)
static HTTP_IO_RESULT HTTPMPFSUpload(HTTP_CONN* pHttpCon)
{
	uint8_t c[16];
	uint16_t lenA, lenB;
	
	switch(pHttpCon->httpStatus)
	{
		// New upload, so look for the CRLFCRLF
		case HTTP_MPFS_UP:
		
			lenA = TCPFindArray(pHttpCon->socket, (const uint8_t*)"\r\n\r\n", 4, 0, 0, false);
		
			if(lenA != 0xffff)
			{// Found it, so remove all data up to and including
				lenA = TCPGetArray(pHttpCon->socket, NULL, lenA);
				pHttpCon->byteCount -= lenA;
				
				// Make sure first 6 bytes are also in
				if(TCPIsGetReady(pHttpCon->socket) < (4u + 6u) )
				{
					lenA++;
					return HTTP_IO_NEED_DATA;
				}
				
				// Make sure it's an MPFS of the correct version
				lenA = TCPGetArray(pHttpCon->socket, c, 10);
				pHttpCon->byteCount -= lenA;
                pHttpCon->mediaHndl = DRV_MEDIA_HANDLE_INVALID;

				if(memcmp(c, (const void*)"\r\n\r\nMPFS\x02\x01", 10) == 0)
                {   // Read as Ver 2.1
                    // Format the Media storage and put 6 byte tag
                    if(HTTPSetMediaWriteHeader(pHttpCon, c + 4, 6))
                    {   // success
                        pHttpCon->httpStatus = HTTP_MPFS_OK;
                        return HTTP_IO_WAITING;
                    }
                }

				// Version is wrong or media error
                HTTPReleaseMedia(pHttpCon);
                pHttpCon->httpStatus = HTTP_MPFS_ERROR;
				return HTTP_IO_WAITING;
			}
			else
			{// Otherwise, remove as much as possible
				lenA = TCPGetArray(pHttpCon->socket, NULL, TCPIsGetReady(pHttpCon->socket) - 4);
				pHttpCon->byteCount -= lenA;
			}
			
			break;
		
		// Received file is invalid
		case HTTP_MPFS_ERROR:
			pHttpCon->byteCount -= TCPIsGetReady(pHttpCon->socket);
			TCPDiscard(pHttpCon->socket);
			if(pHttpCon->byteCount < 100u || pHttpCon->byteCount > 0x80000000u)
			{// If almost all data was read, or if we overflowed, then return
				pHttpCon->sm = SM_HTTP_SERVE_HEADERS;
				return HTTP_IO_DONE;
			}
			break;
		
		// File is verified, so write the data
		case HTTP_MPFS_OK:
			// Determine how much to read
			lenA = TCPIsGetReady(pHttpCon->socket);
			if(lenA > pHttpCon->byteCount)
				lenA = pHttpCon->byteCount;
				
			while(lenA > 0u)
			{
				lenB = TCPGetArray(pHttpCon->socket, c, mMIN(lenA,16u));
				pHttpCon->byteCount -= lenB;
				lenA -= lenB;
                DRV_MEDIA_Write(pHttpCon->mediaHndl, c, lenB);
			}
				
			// If we've read all the data
			if(pHttpCon->byteCount == 0u)
			{
				DRV_MEDIA_WriteFlush(pHttpCon->mediaHndl);
                HTTPReleaseMedia(pHttpCon);
				pHttpCon->sm = SM_HTTP_SERVE_HEADERS;
				return HTTP_IO_DONE;
			}
			
		// Other states are not valid here
		default:
			break;
	}
		
	// Ask for more data
	return HTTP_IO_NEED_DATA;
	
}

// prepares the media storage for writing and writes
// the supplied header
// returns true if success, false otherwise
// pHttpCon->mediaHndl stores the media handle
static bool HTTPSetMediaWriteHeader(HTTP_CONN* pHttpCon, const uint8_t* buffer, unsigned int nBytes)
{
    DRV_MEDIA_IO_INTENT ioIntent;    

    MPFSDeinit();

    ioIntent = DRV_MEDIA_IO_INTENT_WRITE | DRV_MEDIA_IO_INTENT_BLOCKING | DRV_MEDIA_IO_INTENT_EXCLUSIVE;
    pHttpCon->mediaHndl = DRV_MEDIA_Open(DRV_MEDIA_DEFAULT, "mpfs2", ioIntent);
    if(pHttpCon->mediaHndl != DRV_MEDIA_HANDLE_INVALID)
    {   // success
        DRV_MEDIA_WriteSetOffset(pHttpCon->mediaHndl, 0);
        if(DRV_MEDIA_Write(pHttpCon->mediaHndl, buffer, nBytes) == nBytes)
        {   // success
            return true;
        }
    }

    return false;
}

static void HTTPReleaseMedia(HTTP_CONN* pHttpCon)
{
    if(pHttpCon->mediaHndl != DRV_MEDIA_HANDLE_INVALID)
    {
        DRV_MEDIA_Close(pHttpCon->mediaHndl);

        pHttpCon->mediaHndl = DRV_MEDIA_HANDLE_INVALID;
    }


    MPFSInit();
}

#endif

/*****************************************************************************
  Function:
	void HTTPIncFile(HTTP_CONN_HANDLE connHandle, const uint8_t* cFile)

  Summary:
	Writes a file byte-for-byte to the currently loaded TCP socket.

  Description:
	Allows an entire file to be included as a dynamic variable, providing
	a basic templating system for HTML web pages.  This reduces unneeded
	duplication of visual elements such as headers, menus, etc.

	When pHttpCon->callbackPos is 0, the file is opened and as many bytes
	as possible are written.  The current position is then saved to 
	pHttpCon->callbackPos and the file is closed.  On subsequent calls, 
	reading begins at the saved location and continues.  Once the end of
	the input file is reached, pHttpCon->callbackPos is set back to 0 to 
	indicate completion.

  Precondition:
	None

  Parameters:
	cFile - the name of the file to be sent

  Returns:
  	None
  	
  Remarks:
	Users should not call this function directly, but should instead add
	dynamic variables in the form of ~inc:filename.ext~ in their HTML code
	to include (for example) the file "filename.ext" at that specified
	location.  The MPFS2 Generator utility will handle the rest.
  ***************************************************************************/
void HTTPIncFile(HTTP_CONN_HANDLE connHandle, const uint8_t* cFile)
{
	uint16_t wCount, wLen;
	uint8_t data[64];
	MPFS_HANDLE fp;
    HTTP_CONN* pHttpCon = (HTTP_CONN*)connHandle;
	
	// Check if this is a first round call
	if(pHttpCon->callbackPos == 0x00u)
	{// On initial call, open the file and save its ID
		fp = MPFSOpen(cFile);

		if(fp == MPFS_INVALID_HANDLE)
		{// File not found, so abort
			return;
		}
		((TCPIP_UINT32_VAL*)&pHttpCon->callbackPos)->w[0] = MPFSGetID(fp);
	}
	else
	{// The file was already opened, so load up its ID and seek
		fp = MPFSOpenID(((TCPIP_UINT32_VAL*)&pHttpCon->callbackPos)->w[0]);
		if(fp == MPFS_INVALID_HANDLE)
		{// No file handles available, so wait for now
			return;
		}
		MPFSSeek(fp, ((TCPIP_UINT32_VAL*)&pHttpCon->callbackPos)->w[1], MPFS_SEEK_FORWARD);
	}
	
	// Get/put as many bytes as possible
	wCount = TCPIsPutReady(pHttpCon->socket);
	while(wCount > 0u)
	{
		wLen = MPFSGetArray(fp, data, mMIN(wCount, sizeof(data)));
		if(wLen == 0u)
		{// If no bytes were read, an EOF was reached
			MPFSClose(fp);
			pHttpCon->callbackPos = 0x00;
			return;
		}
		else
		{// Write the bytes to the socket
			TCPPutArray(pHttpCon->socket, data, wLen);
			wCount -= wLen;
		}
	}
	
	// Save the new address and close the file
	((TCPIP_UINT32_VAL*)&pHttpCon->callbackPos)->w[1] = MPFSTell(fp);
	MPFSClose(fp);
	
	return;
}


MPFS_HANDLE HTTPCurConnectionFileGet(HTTP_CONN_HANDLE connHandle)
{
    HTTP_CONN* pHttpCon = (HTTP_CONN*)connHandle;
	return pHttpCon->file;
}

int HTTPCurConnectionPostSmGet(HTTP_CONN_HANDLE connHandle)
{
    HTTP_CONN* pHttpCon = (HTTP_CONN*)connHandle;
	return pHttpCon->smPost;
}

void HTTPCurConnectionPostSmSet(HTTP_CONN_HANDLE connHandle, int stat)
{
    HTTP_CONN* pHttpCon = (HTTP_CONN*)connHandle;
	pHttpCon->smPost = stat;
}

uint8_t* HTTPCurConnectionDataBufferGet(HTTP_CONN_HANDLE connHandle)
{
    HTTP_CONN* pHttpCon = (HTTP_CONN*)connHandle;
	return pHttpCon->data;
}

uint32_t HTTPCurConnectionCallbackPosGet(HTTP_CONN_HANDLE connHandle)
{
    HTTP_CONN* pHttpCon = (HTTP_CONN*)connHandle;
	return pHttpCon->callbackPos;
}

void HTTPCurConnectionCallbackPosSet(HTTP_CONN_HANDLE connHandle, uint32_t pos)
{
    HTTP_CONN* pHttpCon = (HTTP_CONN*)connHandle;
	pHttpCon->callbackPos = pos;
}

void HTTPCurConnectionStatusSet(HTTP_CONN_HANDLE connHandle, HTTP_STATUS stat)
{
    HTTP_CONN* pHttpCon = (HTTP_CONN*)connHandle;
	pHttpCon->httpStatus = stat;
}

void HTTPCurConnectionHasArgsSet(HTTP_CONN_HANDLE connHandle, bool args)
{
    HTTP_CONN* pHttpCon = (HTTP_CONN*)connHandle;
	pHttpCon->hasArgs = args;
}

uint32_t HTTPCurConnectionByteCountGet(HTTP_CONN_HANDLE connHandle)
{
    HTTP_CONN* pHttpCon = (HTTP_CONN*)connHandle;
	return pHttpCon->byteCount;
}

void HTTPCurConnectionByteCountSet(HTTP_CONN_HANDLE connHandle, uint32_t byteCount)
{
    HTTP_CONN* pHttpCon = (HTTP_CONN*)connHandle;
	pHttpCon->byteCount = byteCount;
}

void HTTPCurConnectionByteCountDec(HTTP_CONN_HANDLE connHandle, uint32_t byteCount)
{
    HTTP_CONN* pHttpCon = (HTTP_CONN*)connHandle;
	pHttpCon->byteCount -= byteCount;
}

TCP_SOCKET HTTPCurConnectionSocketGet(HTTP_CONN_HANDLE connHandle)
{
    HTTP_CONN* pHttpCon = (HTTP_CONN*)connHandle;
	return pHttpCon->socket;
}

uint8_t HTTPCurConnectionIsAuthorizedGet(HTTP_CONN_HANDLE connHandle)
{
    HTTP_CONN* pHttpCon = (HTTP_CONN*)connHandle;
	return pHttpCon->isAuthorized;
}

void HTTPCurConnectionIsAuthorizedSet(HTTP_CONN_HANDLE connHandle, uint8_t auth)
{
    HTTP_CONN* pHttpCon = (HTTP_CONN*)connHandle;
	pHttpCon->isAuthorized = auth;
}

void HTTPCurConnectionUserDataSet(HTTP_CONN_HANDLE connHandle, const void* uData)
{
    HTTP_CONN* pHttpCon = (HTTP_CONN*)connHandle;
    pHttpCon->userData = uData;
}

const void* HTTPCurConnectionUserDataGet(HTTP_CONN_HANDLE connHandle)
{
    HTTP_CONN* pHttpCon = (HTTP_CONN*)connHandle;
    return pHttpCon->userData;
}


#endif	// defined(TCPIP_STACK_USE_HTTP2_SERVER)

