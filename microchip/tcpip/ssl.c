/*******************************************************************************
  SSLv3 Protocol Client and Server Implementation

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    - Implements an SSL layer supporting both client and server
      operation for any given TCP socket.
*******************************************************************************/

/*******************************************************************************
FileName:   SSL.c
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

#define __SSL_C

#include "tcpip_private.h"

#if defined(TCPIP_STACK_USE_SSL_SERVER) || defined(TCPIP_STACK_USE_SSL_CLIENT)

#include "ssl_private.h"

#define TCPIP_THIS_MODULE_ID    TCPIP_MODULE_SSL

/****************************************************************************
  Section:
	SSL Connection State Global Variables
  ***************************************************************************/

typedef struct
{
    SSL_STUB            ssl_Stub;		    // The current SSL stub
    SSL_KEYS            ssl_Keys;		    // The current SSL session
    SSL_BUFFER          ssl_Buffer;         // SBox and RSA storage, accessed by RSA
    HASH_SUM            ssl_Hash;           // Hash storage
    SSL_SESSION         ssl_Session;        // Current session data
    SSL_SESSION_STUB    ssl_SessionStubs[SSL_MAX_SESSIONS];    // 8 byte session stubs
    uint8_t             ssl_Storage[SSL_RESERVED_MEMORY];
    uint8_t*            ssl_ptrHS;          // Used in buffering handshake results

    uint8_t             ssl_StubID;			// Which SSL_STUB is loaded
    uint8_t             ssl_KeysID;			// Which SSL_KEYS are loaded
    uint8_t             ssl_BufferID;		// Which buffer is loaded
    uint8_t             ssl_HashID;			// Which hash is loaded

    uint8_t             ssl_SessionID;		// Which session is loaded
    uint8_t             ssl_RSAStubID;		// Which stub is using RSA, if any
    uint16_t            ssl_isStubUsed;     // Indicates which stubs are in use

    uint16_t            ssl_isHashUsed;		// Indicates which hashes are in use
    uint16_t            ssl_isBufferUsed;   // Indicates which buffers are in use

    bool                ssl_SessionUpdated;	// Whether or not it has been updated

}SSL_DCPT;      // SSL layer descriptor

static SSL_DCPT*        sslDcpt = 0;        // The one and only SSL descriptor

static TCPIP_NET_IF*    sslSyncIf = 0;                      // last SSL interface index that was synced
static int              sslInitCount = 0;                   // SSL module initialization count


static int              sslConnCount = 0;           // quick counter of the open SSL connections

extern const uint16_t   SSL_CERT_LEN;	// RSA public certificate length		?
extern const uint8_t    SSL_CERT[];		// RSA public certificate data			?


/****************************************************************************
  Section:
	Function Prototypes
  ***************************************************************************/
	// Section: Cryptographic Calculation Functions
	static RSA_STATUS SSLRSAOperation(void);
	static void GenerateHashRounds(uint8_t num, uint8_t* rand1, uint8_t* rand2);
	static void CalculateFinishedHash(uint8_t hashID, bool fromClient, uint8_t *result);
	static void GenerateSessionKeys(void);

	// Section: Ethernet Buffer RAM Management
	static void SSLStubSync(TCPIP_NET_IF* pIf, uint8_t id);
	static bool SSLStubAlloc(TCPIP_NET_IF* currIf);
	static void SSLStubFree(uint8_t id);
	static void SSLKeysSync(uint8_t id);
	static void SSLHashSync(uint8_t id);
	static void SSLHashAlloc(uint8_t *id);
	static void SSLHashFree(uint8_t *id);
	static void SSLBufferSync(uint8_t id);
	static void SSLBufferAlloc(uint8_t *id);
	static void SSLBufferFree(uint8_t *id);
	static uint8_t SSLSessionNew(void);
	static void SSLSessionSync(uint8_t id);

	static void SSLSave(uint8_t *srcAdd, uint8_t* dstAddr, uint16_t len);
	static void SSLLoad(uint8_t *destAddr, uint8_t* srcAddr, uint16_t len);
	
	// Section: Handshake Hash and I/O Functions
	static void HSStart(void);
	static void HSEnd(void);
	static uint16_t HSGet(TCP_SOCKET skt, uint8_t *b);
	static uint16_t HSGetWord(TCP_SOCKET skt, uint16_t *w);
	static uint16_t HSGetArray(TCP_SOCKET skt, uint8_t *data, uint16_t len);
	static uint16_t HSPut(TCP_SOCKET skt, uint8_t b);
	static uint16_t HSPutWord(TCP_SOCKET skt, uint16_t w);
	static uint16_t HSPutArray(TCP_SOCKET skt, const uint8_t *data, uint16_t len);
	
	// Section: Client messages
	#if defined(TCPIP_STACK_USE_SSL_CLIENT)
		static uint8_t SSLSessionMatchIP(IPV4_ADDR ip);
		static void SSLTxClientHello(TCP_SOCKET hTCP);
		static void SSLRxServerHello(TCP_SOCKET hTCP);
		static void SSLRxServerCertificate(TCP_SOCKET hTCP);
		static void SSLTxClientKeyExchange(TCP_SOCKET hTCP);
	#endif
	
	// Section: Server messages
	#if defined(TCPIP_STACK_USE_SSL_SERVER)
		static uint8_t SSLSessionMatchID(uint8_t* SessionID);
		static void SSLRxAntiqueClientHello(TCP_SOCKET hTCP);
		static void SSLRxClientHello(TCP_SOCKET hTCP);
		static void SSLTxServerHello(TCP_SOCKET hTCP);
		static void SSLTxServerCertificate(TCP_SOCKET hTCP);
		static void SSLTxServerHelloDone(TCP_SOCKET hTCP);
		static void SSLRxClientKeyExchange(TCP_SOCKET hTCP);
	#endif
	
	// Section: Client and server messages
	static void SSLTxCCSFin(TCP_SOCKET hTCP);
	static void SSLRxCCS(TCP_SOCKET hTCP);
	static void SSLRxFinished(TCP_SOCKET hTCP);
	static void SSLRxAlert(TCP_SOCKET hTCP);

/****************************************************************************
  Section:
	Macros and Definitions
  ***************************************************************************/
	#define mMIN(a, b)	((a<b)?a:b)

	#define SSL_RSA_EXPORT_WITH_ARCFOUR_40_MD5	0x0003u
	#define SSL_RSA_WITH_ARCFOUR_128_MD5		0x0004u


#if SSL_MULTIPLE_INTERFACES
    #define     dcptSSL(netIx)          (sslDcpt + netIx)
#else
    #define     dcptSSL(netIx)          (sslDcpt + 0)
#endif


// SSL stub addresss
#define	SSL_STUB_ADDR(pDcpt, ix)    (pDcpt->ssl_Storage + SSL_STUB_SIZE * (ix))

// Base address for SSL keys
#define SSL_KEYS_ADDR(pDcpt, ix)    (pDcpt->ssl_Storage + SSL_STUB_SPACE + SSL_KEYS_SIZE * (ix))

// SSL hash addresss
#define SSL_HASH_ADDR(pDcpt, ix)    (pDcpt->ssl_Storage + SSL_STUB_SPACE + SSL_KEYS_SPACE + SSL_HASH_SIZE * (ix))

// SSL buffer addresss
#define SSL_BUFFER_ADDR(pDcpt, ix)  (pDcpt->ssl_Storage + SSL_STUB_SPACE + SSL_KEYS_SPACE + SSL_HASH_SPACE + SSL_BUFFER_SIZE * (ix))


// SSL session addresss
#define SSL_SESSION_ADDR(pDcpt, ix) (pDcpt->ssl_Storage + SSL_STUB_SPACE + SSL_KEYS_SPACE + SSL_HASH_SPACE + SSL_BUFFER_SPACE + SSL_SESSION_SIZE * (ix))




/****************************************************************************
  ===========================================================================
  Section:
	SSL Management Functions
  ===========================================================================
  ***************************************************************************/

/*****************************************************************************
  Function:
	void SSLInit(void)

  Description:
	Initializes the SSL engine.

  Precondition:
	None

  Parameters:
	None

  Returns:
  	None
  	
  Remarks:
	This function is called only one during lifetime of the application.
  ***************************************************************************/
bool SSLInit(const TCPIP_STACK_MODULE_CTRL* const stackCtrl,
             const SSL_MODULE_CONFIG* sslData)
{
    SSL_DCPT*     pDcpt;

    if(stackCtrl->stackAction == TCPIP_STACK_ACTION_IF_UP)
    {   // interface restart
        return true;
    }

    // (stackInit->stackAction == TCPIP_STACK_ACTION_INIT)   // stack init

    if(sslInitCount == 0)
    {   // first time we're run
#if SSL_MULTIPLE_INTERFACES
        sslDcpt = (SSL_DCPT*)TCPIP_HEAP_Malloc(stackCtrl->memH, stackCtrl->nIfs * sizeof(*sslDcpt));
#else
        sslDcpt = (SSL_DCPT*)TCPIP_HEAP_Malloc(stackCtrl->memH, sizeof(*sslDcpt));
#endif

        if(sslDcpt == 0)
        {   // failed
            return false;
        }

        sslSyncIf  = 0;
        sslConnCount = 0;
    }

    // per interface init
	// Set all resources to unused
    pDcpt = sslDcpt +  stackCtrl->netIx;

	pDcpt->ssl_isStubUsed = 0;
	pDcpt->ssl_isHashUsed = 0;
	pDcpt->ssl_isBufferUsed = 0;
	for(pDcpt->ssl_SessionID = 0; pDcpt->ssl_SessionID < SSL_MAX_SESSIONS; pDcpt->ssl_SessionID++)
	{
		pDcpt->ssl_SessionStubs[pDcpt->ssl_SessionID].tag.Val = 0;
	}

	// Indicate that nothing is loaded
	pDcpt->ssl_HashID = SSL_INVALID_ID;
	pDcpt->ssl_StubID = SSL_INVALID_ID;
	pDcpt->ssl_SessionID = SSL_INVALID_ID;
	pDcpt->ssl_KeysID = SSL_INVALID_ID;
	pDcpt->ssl_BufferID = SSL_INVALID_ID;
	pDcpt->ssl_SessionUpdated = false;
	pDcpt->ssl_RSAStubID = SSL_INVALID_ID;

    sslInitCount++;
	
	return true;
}	

/*****************************************************************************
  Function:
	void SSLDeInit(void)

  Description:
	DeInitializes the SSL engine.

  Precondition:
	None

  Parameters:
	None

  Returns:
  	None
  	
  Remarks:
	This function may be called several 
  ***************************************************************************/
void SSLDeInit(const TCPIP_STACK_MODULE_CTRL* const stackCtrl)
{
    // if(stackInit->stackAction == TCPIP_STACK_ACTION_DEINIT) // stack shut down
    // if(stackInit->stackAction == TCPIP_STACK_ACTION_IF_DOWN) // interface down

    // one way or another this interface is going down

    if(stackCtrl->stackAction == TCPIP_STACK_ACTION_DEINIT)
    {   // whole stack is going down
        if(sslInitCount > 0)
        {   // we're up and running
            if(--sslInitCount == 0)
            {   // all closed
                // release resources
                TCPIP_HEAP_Free(stackCtrl->memH, sslDcpt);
                sslDcpt = 0;
                sslConnCount = 0;
            }
        }
    }

}


void SSLTask(void)
{
	TCP_SOCKET hTCP;
    int        nSkts;
    uint8_t    sslStubID;

    nSkts = TCPSocketsNo();
	for(hTCP = 0; hTCP < nSkts; hTCP++)
    {
        sslStubID = TcpSocketGetSSLId(hTCP);
        if(sslStubID != SSL_INVALID_ID)
        {
            SSLPeriodic(hTCP, sslStubID);
            TcpSSLMessageTransmit(hTCP);
        }
    }
}

// Note: signaling that attention is needed whenever a SSL connection is open
// is pretty demanding.
// However, because the SSL has to advance the RSA state machine it needs
// to be called whenever there's a chance.
bool SSLTaskPending(void)
{
    return sslConnCount != 0;
}



/*****************************************************************************
  Function:
	void SSLPeriodic(TCP_SOCKET hTCP, uint8_t id)

  Summary:
	Performs any periodic tasks for the SSL module.

  Description:
	This function performs periodic tasks for the SSL module.  This includes
	processing for RSA operations.

  Precondition:
	SSL has already been initialized.

  Parameters:
	hTCP - the socket for which to perform periodic functions
	id - the SSL stub to use
	
  Returns:
  	None
  	
  ***************************************************************************/
void SSLPeriodic(TCP_SOCKET hTCP, uint8_t id)
{
	// Sync the SSL Stub
    TCPIP_NET_IF* currIf = (TCPIP_NET_IF*)TCPSocketGetNet(hTCP);
    SSL_DCPT*     pDcpt = dcptSSL(_TCPIPStackNetIx(currIf));

    SSLStubSync(currIf, id);
	
	// For new sessions, try to claim a session
	if(pDcpt->ssl_Stub.Flags.bNewSession && pDcpt->ssl_Stub.idSession == SSL_INVALID_ID)
	{
		pDcpt->ssl_Stub.idSession = SSLSessionNew();
	}
	
	// If RSA is in progress, do some RSA work
	if(pDcpt->ssl_Stub.Flags.bRSAInProgress)
	{			
		if(SSLRSAOperation() == RSA_DONE)
		{// Move on with the connection
			pDcpt->ssl_Stub.Flags.bRSAInProgress = 0;

			// For clients, request the CKE message
			#if defined(TCPIP_STACK_USE_SSL_CLIENT)
			if(!pDcpt->ssl_Stub.Flags.bIsServer)
				TCPRequestSSLMessage(hTCP, SSL_CLIENT_KEY_EXCHANGE);
			#endif
			
			// For servers, copy the decoded message to the session data
			#if defined(TCPIP_STACK_USE_SSL_SERVER)
			if(pDcpt->ssl_Stub.Flags.bIsServer)
			{
				// Copy over the pre-master secret
				SSLSessionSync(pDcpt->ssl_Stub.idSession);
				memcpy((void*)pDcpt->ssl_Session.masterSecret, (void*)&pDcpt->ssl_Buffer.full[(SSL_RSA_KEY_SIZE/8)-48], 48);
												
				// Generate the Master Secret
				SSLKeysSync(pDcpt->ssl_StubID);
				SSLBufferSync(SSL_INVALID_ID);
				GenerateHashRounds(3, pDcpt->ssl_Keys.Remote.random, pDcpt->ssl_Keys.Local.random);
				memcpy(pDcpt->ssl_Session.masterSecret, (void*)pDcpt->ssl_Buffer.hashRounds.temp, 48);
				
				// Note the new session data and release RSA engine
				pDcpt->ssl_SessionUpdated = true;
				RSAEndUsage(currIf);
				pDcpt->ssl_RSAStubID = SSL_INVALID_ID;
			}
			
			// Continue receiving the CCS and Finished messages
			TCPSSLHandleIncoming(hTCP);
			#endif
		}
	}
}

/*****************************************************************************
  Function:
	uint8_t SSLStartSession(TCP_SOCKET hTCP, uint8_t * buffer, uint8_t supDataType)

  Description:
	Begins a new SSL session for the given TCP connection.

  Precondition:
	SSL has been initialized and hTCP is connected.

  Parameters:
	hTCP - the socket to begin the SSL connection on
	buffer - pointer to a supplementary data buffer
	supDataType - type of supplementary data to store
	
  Return Values:
  	SSL_INVALID_ID - insufficient SSL resources to start a new connection
  	others - the allocated SSL stub ID
  ***************************************************************************/
uint8_t SSLStartSession(TCP_SOCKET hTCP, void * buffer, uint8_t supDataType)
{
	uint8_t    i;
    TCPIP_NET_IF* currIf = (TCPIP_NET_IF*)TCPSocketGetNet(hTCP);
    SSL_DCPT*     pDcpt = dcptSSL(_TCPIPStackNetIx(currIf));
	
	// Allocate a stub for use, or fail
	if(!SSLStubAlloc(currIf))
    {
		return SSL_INVALID_ID;
    }

    
	// Clear stub state
	pDcpt->ssl_Stub.wRxBytesRem = 0;
	pDcpt->ssl_Stub.wRxHsBytesRem = 0;
	pDcpt->ssl_Stub.Flags.w = 0x0000;
	
	// Clear any allocations
	pDcpt->ssl_Stub.idSession = SSL_INVALID_ID;
	pDcpt->ssl_Stub.idRxHash = SSL_INVALID_ID;
	pDcpt->ssl_Stub.idMD5 = SSL_INVALID_ID;
	pDcpt->ssl_Stub.idSHA1 = SSL_INVALID_ID;
	pDcpt->ssl_Stub.idRxBuffer = SSL_INVALID_ID;
	pDcpt->ssl_Stub.idTxBuffer = SSL_INVALID_ID;
	pDcpt->ssl_Stub.requestedMessage = SSL_NO_MESSAGE;
	pDcpt->ssl_Stub.dwTemp.Val = 0;
	pDcpt->ssl_Stub.supplementaryBuffer = buffer;
    pDcpt->ssl_Stub.supplementaryDataType = supDataType;

	// Allocate handshake hashes for use, or fail
	SSLHashAlloc(&pDcpt->ssl_Stub.idMD5);
	SSLHashAlloc(&pDcpt->ssl_Stub.idSHA1);
	if(pDcpt->ssl_Stub.idMD5 == SSL_INVALID_ID || pDcpt->ssl_Stub.idSHA1 == SSL_INVALID_ID)
	{
		SSLHashFree(&pDcpt->ssl_Stub.idMD5);
		SSLHashFree(&pDcpt->ssl_Stub.idSHA1);
		SSLStubFree(pDcpt->ssl_StubID);
		return SSL_INVALID_ID;
	}
	
	// Initialize the handshake hashes
	SSLHashSync(pDcpt->ssl_Stub.idSHA1);
	SHA1Initialize(&pDcpt->ssl_Hash);
	SSLHashSync(pDcpt->ssl_Stub.idMD5);
	MD5Initialize(&pDcpt->ssl_Hash);
	
	// Set up Local.random (4 byte UTC time, 28 bytes random)
	SSLKeysSync(pDcpt->ssl_StubID);
	#if defined(TCPIP_STACK_USE_SNTP_CLIENT)
	{
		TCPIP_UINT32_VAL temp;

		temp.Val = SNTPGetUTCSeconds();
		pDcpt->ssl_Keys.Local.random[0] = temp.v[3];
		pDcpt->ssl_Keys.Local.random[1] = temp.v[2];
		pDcpt->ssl_Keys.Local.random[2] = temp.v[1];
		pDcpt->ssl_Keys.Local.random[3] = temp.v[0];
		i = 4;
	}
	#else
		i = 0;
	#endif
	while(i < 32u)
		pDcpt->ssl_Keys.Local.random[i++] = SYS_RANDOM_GET();
		
    sslConnCount++;
	// Return the ID
	return pDcpt->ssl_StubID;
}

/*****************************************************************************
  Function:
	void SSLTerminate(TCP_SOCKET hTCP, uint8_t id)

  Description:
	Terminates an SSL connection and releases allocated resources.

  Precondition:
	None

  Parameters:
	hTCP - the socket to terminate the SSL connection on
	id - the SSL stub ID to terminate
	
  Returns:
  	None
  ***************************************************************************/
void SSLTerminate(TCP_SOCKET hTCP, uint8_t id)
{
	// Sync in the right stub
    TCPIP_NET_IF* currIf = (TCPIP_NET_IF*)TCPSocketGetNet(hTCP);
    SSL_DCPT*     pDcpt = dcptSSL(_TCPIPStackNetIx(currIf));

    SSLStubSync(currIf, id);
	
	// If no CloseNotify, then invalidate the session so it cannot resume
	// ( This restriction is not presently enforced.  IE incorrectly
	//   completes the handshake, then disconnects without a CloseNotify
	//   when it decides to prompt the user whether or not to accept a 
	//   unverifiable certificate. )
	//if(!pDcpt->ssl_Stub.Flags.bCloseNotify)
	//{
	//	pDcpt->ssl_SessionStubs[pDcpt->ssl_Stub.idSession].tag.Val = 0;
	//}	
	
	// Free up resources
	SSLBufferFree(&pDcpt->ssl_Stub.idRxBuffer);
	SSLBufferFree(&pDcpt->ssl_Stub.idTxBuffer);
	SSLHashFree(&pDcpt->ssl_Stub.idMD5);
	SSLHashFree(&pDcpt->ssl_Stub.idSHA1);
	SSLHashFree(&pDcpt->ssl_Stub.idRxHash);
	SSLStubFree(id);
	if(pDcpt->ssl_RSAStubID == id)
	{
		pDcpt->ssl_RSAStubID = SSL_INVALID_ID;
		RSAEndUsage(currIf);
	}
    sslConnCount--;
	
}

/****************************************************************************
  ===========================================================================
  Section:
	SSL Record Processing Functions
  ===========================================================================
  ***************************************************************************/

/*****************************************************************************
  Function:
	uint16_t SSLRxRecord(TCP_SOCKET hTCP, uint8_t id)

  Summary:
	Receives an SSL record.

  Description:
	Reads at most one SSL Record header from the TCP stream and determines what
	to do with the rest of the data.  If not all of the data is available for 
	the record, then the function returns and future call(s) to SSLRxRecord() 
	will process the remaining data until the end of the record is reached.  
	If this call process data from a past record, the next record will not be 
	started until the next call.

  Precondition:
	The specified SSL stub is initialized and the TCP socket is connected.

  Parameters:
	hTCP - The TCP socket from which to read
	id - The active SSL stub ID
	
  Returns:
  	uint16_t indicating the number of data bytes there were decrypted but left in 
  	the stream.

  Remarks:
  	SSL record headers, MAC footers, and symetric cipher block padding (if any) 
  	will be extracted from the TCP stream by this function.  Data will be 
  	decrypted but left in the stream.
  ***************************************************************************/
uint16_t SSLRxRecord(TCP_SOCKET hTCP, uint8_t id)
{	
	uint8_t temp[32];
	uint16_t wLen;
    
    TCPIP_NET_IF* currIf = (TCPIP_NET_IF*)TCPSocketGetNet(hTCP);
    SSL_DCPT*     pDcpt = dcptSSL(_TCPIPStackNetIx(currIf));
	
	SSLStubSync(currIf, id);
	
	// Don't do anything for terminated connections
	if(pDcpt->ssl_Stub.Flags.bDone)
		return 0;

	// If this is a new record, then read the header
	// When bytes remain, a message is not yet fully read, so
	// the switch statement will continue handling the data
	if(pDcpt->ssl_Stub.wRxBytesRem == 0u)
	{
		// See if we expect a MAC
		if(pDcpt->ssl_Stub.Flags.bExpectingMAC)
		{// Receive and verify the MAC
			if(TCPIsGetReady(hTCP) < 16u)
				return 0;
				
			// Read the MAC
			TCPGetArray(hTCP, temp, 16);
			
			// Calculate the expected MAC
			SSLBufferSync(pDcpt->ssl_Stub.idRxBuffer);
			SSLKeysSync(id);
			SSLHashSync(pDcpt->ssl_Stub.idRxHash);
			
			ARCFOURCrypt(&pDcpt->ssl_Keys.Remote.app.cryptCtx, temp, 16);
			SSLMACCalc(pDcpt->ssl_Keys.Remote.app.MACSecret, &temp[16]);
			
			// MAC no longer expected
			pDcpt->ssl_Stub.Flags.bExpectingMAC = 0;
			
			// Verify the MAC
			if(memcmp((void*)temp, (void*)&temp[16], 16) != 0)
			{// MAC fails
				TCPRequestSSLMessage(hTCP, SSL_ALERT_BAD_RECORD_MAC);
				return 0;
			}
		}	
		
		// Check if a new header is available
		// Also ignore data if SSL is terminated
		if(TCPIsGetReady(hTCP) < 5u)
			return 0;
		
		// Read the record type (uint8_t)
		TCPGet(hTCP, &pDcpt->ssl_Stub.rxProtocol);
		
		#if defined(TCPIP_STACK_USE_SSL_SERVER)
		// Check if we've received an SSLv2 ClientHello message
		// Client-only implementations don't need to deal with this
		if((pDcpt->ssl_Stub.rxProtocol & 0x80) == 0x80)
		{
			// After MSB, next 15 bits are the length
			((uint8_t*)&pDcpt->ssl_Stub.wRxBytesRem)[1] = pDcpt->ssl_Stub.rxProtocol & 0x7F;
			TCPGet(hTCP, ((uint8_t*)&pDcpt->ssl_Stub.wRxBytesRem));
			
			// Tell the handshaker what message to expect
			pDcpt->ssl_Stub.wRxHsBytesRem = pDcpt->ssl_Stub.wRxBytesRem;
			pDcpt->ssl_Stub.rxProtocol = SSL_HANDSHAKE;
			pDcpt->ssl_Stub.rxHSType = SSL_ANTIQUE_CLIENT_HELLO;
		}
		
		// Otherwise, this is a normal SSLv3 message
		// Read the rest of the record header and proceed normally
		else
		#endif
		{
			// Read version (uint16_t, currently ignored)
			TCPGet(hTCP, NULL);
			TCPGet(hTCP, NULL);
	
			// Read length (uint16_t)
			TCPGet(hTCP, ((uint8_t*)&pDcpt->ssl_Stub.wRxBytesRem)+1);
			TCPGet(hTCP, ((uint8_t*)&pDcpt->ssl_Stub.wRxBytesRem));
			
			// Determine if a MAC is expected
			if(pDcpt->ssl_Stub.Flags.bRemoteChangeCipherSpec)
			{
				pDcpt->ssl_Stub.Flags.bExpectingMAC = 1;
				pDcpt->ssl_Stub.wRxBytesRem -= 16;
							
				// Set up the MAC
				SSLKeysSync(pDcpt->ssl_StubID);
				SSLHashSync(pDcpt->ssl_Stub.idRxHash);
				SSLMACBegin(pDcpt->ssl_Keys.Remote.app.MACSecret, 
					pDcpt->ssl_Keys.Remote.app.sequence++, 
					pDcpt->ssl_Stub.rxProtocol, pDcpt->ssl_Stub.wRxBytesRem);
			}
		}
		
	}
	
	// See if data is ready that needs decryption
	wLen = TCPIsGetReady(hTCP);

	// Decrypt and MAC if necessary
	if(pDcpt->ssl_Stub.Flags.bRemoteChangeCipherSpec && wLen)
	{// Need to decrypt the data
		
		// Only decrypt up to end of record
		if(wLen > pDcpt->ssl_Stub.wRxBytesRem)
			wLen = pDcpt->ssl_Stub.wRxBytesRem;
						
		// Prepare for decryption
		SSLKeysSync(id);
		SSLBufferSync(pDcpt->ssl_Stub.idRxBuffer);
		SSLHashSync(pDcpt->ssl_Stub.idRxHash);

		// Decrypt application data to proper location, non-app in place
		TCPSSLDecryptMAC(hTCP, &pDcpt->ssl_Keys.Remote.app.cryptCtx, wLen);
	}
	
	// Determine what to do with the rest of the data
	switch(pDcpt->ssl_Stub.rxProtocol)
	{
		case SSL_HANDSHAKE:
			SSLRxHandshake(hTCP, pDcpt->ssl_StubID);
			break;
			
		case SSL_CHANGE_CIPHER_SPEC:
			SSLRxCCS(hTCP);
			break;
			
		case SSL_APPLICATION:
			// Data was handled above
			// Just note that it's all been read
			pDcpt->ssl_Stub.wRxBytesRem -= wLen;
			return wLen;
		
		case SSL_ALERT:
			SSLRxAlert(hTCP);
			break;
	}
	
	return 0;
}

/*****************************************************************************
  Function:
	void SSLTxRecord(TCP_SOCKET hTCP, uint8_t id, uint8_t txProtocol)

  Summary:
	Transmits an SSL record.

  Description:
	Transmits all pending data in the TCP TX buffer as an SSL record using
	the specified protocol.  This function transparently encrypts and MACs
	the data if there is an active cipher spec.

  Precondition:
	The specified SSL stub is initialized and the TCP socket is connected.

  Parameters:
	hTCP - The TCP socket with data waiting to be transmitted
	id - The active SSL stub ID
	txPortocol - The SSL protocol number to attach to this record
	
  Returns:
  	None
  ***************************************************************************/
void SSLTxRecord(TCP_SOCKET hTCP, uint8_t id, uint8_t txProtocol)
{
	TCPIP_UINT16_VAL wLen;
	uint8_t hdr[5];
    TCPIP_NET_IF* currIf = (TCPIP_NET_IF*)TCPSocketGetNet(hTCP);
    SSL_DCPT*     pDcpt = dcptSSL(_TCPIPStackNetIx(currIf));
	
	SSLStubSync(currIf, id);
	
	// If stub is done, prevent writing data
	if(pDcpt->ssl_Stub.Flags.bDone)
		return;
	
	// Determine how many bytes are ready to write
	wLen.Val = TCPSSLGetPendingTxSize(hTCP);
	if(wLen.Val == 0u)
		return;
	
	// Determine if a MAC is required
	if(pDcpt->ssl_Stub.Flags.bLocalChangeCipherSpec)
	{// Perform the encryption and MAC
		// Sync needed data
		SSLKeysSync(pDcpt->ssl_StubID);
		SSLHashSync(SSL_INVALID_ID);
		SSLBufferSync(pDcpt->ssl_Stub.idTxBuffer);
		
		// Start the MAC calculation
		SSLMACBegin(pDcpt->ssl_Keys.Local.app.MACSecret, 
			pDcpt->ssl_Keys.Local.app.sequence, txProtocol, wLen.Val);
		pDcpt->ssl_Keys.Local.app.sequence++;
		
		// Get ready to send
		TCPSSLInPlaceMACEncrypt(hTCP, &pDcpt->ssl_Keys.Local.app.cryptCtx,
				pDcpt->ssl_Keys.Local.app.MACSecret, wLen.Val);
		
		// Add MAC length to the data length
		wLen.Val += 16;
	}
	
	// Prepare the header
	hdr[0] = txProtocol;
	hdr[1] = SSL_VERSION_HI;
	hdr[2] = SSL_VERSION_LO;
	hdr[3] = wLen.v[1];
	hdr[4] = wLen.v[0];
	
	// Put the record header and send the data
	TCPSSLPutRecordHeader(hTCP, hdr, true);
	
}

/*****************************************************************************
  Function:
	void SSLStartPartialRecord(TCP_SOCKET hTCP, uint8_t id, uint8_t txProtocol,
								 uint16_t wLen)

  Summary:
	Begins a long SSL record.

  Description:
	This function allows messages longer than the TCP buffer to be sent,
	which is frequently the case for the Certificate handshake message.  The
	final message length is required to be known in order to transmit the
	header.  Once called, SSLFlushPartialRecord and SSLFinishPartialRecord
	must be called to write remaining data, finalize, and prepare for a new
	record.

  Precondition:
	The specified SSL stub is initialized and the TCP socket is connected.

  Parameters:
	hTCP - The TCP socket with data waiting to be transmitted
	id - The active SSL stub ID
	txPortocol - The SSL protocol number to attach to this record
	wLen - The length of all the data to be sent
	
  Returns:
  	None
  
  Remarks:
	Partial messages do not support the current cipher spec, so this can
	only be used during the handshake procedure.
  ***************************************************************************/
#if defined(TCPIP_STACK_USE_SSL_SERVER)
void SSLStartPartialRecord(TCP_SOCKET hTCP, uint8_t id, uint8_t txProtocol, uint16_t wLen)
{
	uint8_t hdr[5];
    TCPIP_NET_IF* currIf = (TCPIP_NET_IF*)TCPSocketGetNet(hTCP);
	
	SSLStubSync(currIf, id);
	
	// Prepare the header
	hdr[0] = txProtocol;
	hdr[1] = SSL_VERSION_HI;
	hdr[2] = SSL_VERSION_LO;
	hdr[3] = wLen >> 8;
	hdr[4] = wLen;
	
	// Put the record header and send the data
	TCPSSLPutRecordHeader(hTCP, hdr, false);
	
}
#endif

/*****************************************************************************
  Function:
	void SSLTxMessage(TCP_SOCKET hTCP, uint8_t id, uint8_t msg)

  Summary:
	Transmits an SSL message.

  Description:
	This function transmits a specific SSL message for handshakes and alert
	messages.  Supported messages are listed in SSL_MESSAGES.

  Precondition:
	The specified SSL stub is initialized and the TCP socket is connected.

  Parameters:
	hTCP - The TCP socket with data waiting to be transmitted
	id - The active SSL stub ID
	msg - One of the SSL_MESSAGES types to send
	
  Returns:
  	None
  ***************************************************************************/
void SSLTxMessage(TCP_SOCKET hTCP, uint8_t id, uint8_t msg)
{
    TCPIP_NET_IF* currIf = (TCPIP_NET_IF*)TCPSocketGetNet(hTCP);
    SSL_DCPT*     pDcpt = dcptSSL(_TCPIPStackNetIx(currIf));
	
    SSLStubSync(currIf, id);
	
	// Don't do anything for terminated connections
	if(pDcpt->ssl_Stub.Flags.bDone)
		return;
	
	// Transmit the requested message
	switch(msg)
	{
		#if defined(TCPIP_STACK_USE_SSL_CLIENT)
		case SSL_CLIENT_HELLO:
			SSLTxClientHello(hTCP);
			break;
		case SSL_CLIENT_KEY_EXCHANGE:
			SSLTxClientKeyExchange(hTCP);
			break;
		#endif
		
		#if defined(TCPIP_STACK_USE_SSL_SERVER)
		case SSL_SERVER_HELLO:
			SSLTxServerHello(hTCP);
			break;
		case SSL_CERTIFICATE:
			SSLTxServerCertificate(hTCP);
			break;
		case SSL_SERVER_HELLO_DONE:
			SSLTxServerHelloDone(hTCP);
			break;
		#endif
		
		case SSL_CHANGE_CIPHER_SPEC:
			SSLTxCCSFin(hTCP);
			break;
			
		// Handle all alert messages
		default:
			if((msg & 0x80) != 0x80)
				break;
			
			// Make sure we can write the message
			if(TCPIsPutReady(hTCP) < 2u)
				break;
			
			// Select FATAL or WARNING
			if(msg == SSL_ALERT_CLOSE_NOTIFY)
			{
				TCPPut(hTCP, SSL_ALERT_WARNING);
				pDcpt->ssl_Stub.Flags.bCloseNotify = 1;
			}
			else
				TCPPut(hTCP, SSL_ALERT_FATAL);
			
			// Put the message byte
			TCPPut(hTCP, msg - 0x80);
			
			// Flush the message
			SSLTxRecord(hTCP, pDcpt->ssl_StubID, SSL_ALERT);
			TCPRequestSSLMessage(hTCP, SSL_NO_MESSAGE);
			
			// Mark session as terminated
			pDcpt->ssl_Stub.Flags.bDone = 1;
	}
	
}

/*****************************************************************************
  Function:
	void SSLRxHandshake(TCP_SOCKET hTCP, uint8_t id)

  Summary:
	Receives a handshake message.

  Description:
	This function receives handshake messages, reads the handshake header,
	and passes the data off to the appropriate handshake parser. 

  Precondition:
	The specified SSL stub is initialized and the TCP socket is connected.
	Also requires that rxBytesRem has been populated and the current SSL stub
	has been synced into memory.

  Parameters:
	hTCP - The TCP socket to read a handshake message from
	id - The active SSL stub ID
	
  Returns:
  	None
  ***************************************************************************/
void SSLRxHandshake(TCP_SOCKET hTCP, uint8_t id)
{
	uint16_t wLen;
    TCPIP_NET_IF* currIf = (TCPIP_NET_IF*)TCPSocketGetNet(hTCP);
    SSL_DCPT*     pDcpt = dcptSSL(_TCPIPStackNetIx(currIf));
    
	SSLStubSync(currIf, id);   
	
	
	// Start reading handshake data
	HSStart();
	
	// If this is a new handshake message, read the header
	// If the message has already been started, there will
	// still be bytes remaining and the switch statement will 
	// handle the rest.
	if(pDcpt->ssl_Stub.wRxHsBytesRem == 0u)
	{
		// Make sure entire header is in the buffer
		if(TCPIsGetReady(hTCP) < 4u)
			return;
		
		// Read the message type (uint8_t)
		HSGet(hTCP, &pDcpt->ssl_Stub.rxHSType);
		
		// Read the length (3 BYTES)
		HSGet(hTCP, NULL);
		HSGetWord(hTCP, &wLen);
		pDcpt->ssl_Stub.wRxHsBytesRem = wLen;
	}
	
	// Determine what to do with the rest of the data
	switch(pDcpt->ssl_Stub.rxHSType)
	{
		#if defined(TCPIP_STACK_USE_SSL_CLIENT)
		case SSL_SERVER_HELLO:
			SSLRxServerHello(hTCP);
			break;

		case SSL_CERTIFICATE:
			SSLRxServerCertificate(hTCP);
			break;
			
		case SSL_SERVER_HELLO_DONE:
			// This message contains no data
			// Record that message was received
			pDcpt->ssl_Stub.Flags.bServerHelloDone = 1;
			break;
		#endif
		
		#if defined(TCPIP_STACK_USE_SSL_SERVER)
		case SSL_ANTIQUE_CLIENT_HELLO:
			SSLRxAntiqueClientHello(hTCP);
			break;

		case SSL_CLIENT_HELLO:
			SSLRxClientHello(hTCP);
			break;
			
		case SSL_CLIENT_KEY_EXCHANGE:
			SSLRxClientKeyExchange(hTCP);
			break;
		#endif
		
		case SSL_FINISHED:
			SSLRxFinished(hTCP);
			break;
	}
	
	// End reading handshake data
	HSEnd();
	
}	


/****************************************************************************
  ===========================================================================
  Section:
	SSL Message Processing Functions
  ===========================================================================
  ***************************************************************************/

/*********************************************************************
 * Function:        uint8_t SSLTxClientHello(TCP_SOCKET hTCP)
 *
 * PreCondition:    Enough space is available in hTCP to write the
 *					entire message. 
 *
 * Input:           hTCP - the TCP Socket to send the message to
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Transmits the ClientHello message to initiate a
 *					new SSL session with the server.
 *
 * Note:            None
 ********************************************************************/
#if defined(TCPIP_STACK_USE_SSL_CLIENT)
static void SSLTxClientHello(TCP_SOCKET hTCP)
{	
    TCP_SOCKET_INFO sktInfo;
    SSL_DCPT*     pDcpt = dcptSSL(_TCPIPStackNetIx(sslSyncIf));

	// Restart the handshake hasher
	HSStart();
	
	// Indicate that we're the client
	//pDcpt->ssl_Stub.Flags.bIsServer = 0;  // This is the default already
	
	// Make sure enough space is available to transmit
	if(TCPIsPutReady(hTCP) < 100u)
		return;
	
	// Look for a valid session to reuse
        TCPGetSocketInfo(hTCP, &sktInfo);
	pDcpt->ssl_Stub.idSession = SSLSessionMatchIP(sktInfo.remoteIPaddress.v4Add);
	pDcpt->ssl_Stub.Flags.bNewSession = (pDcpt->ssl_Stub.idSession == SSL_INVALID_ID);
	
	// If none is found, generate a new one
	if(pDcpt->ssl_Stub.Flags.bNewSession)
	{
		pDcpt->ssl_Stub.idSession = SSLSessionNew();
		if(pDcpt->ssl_Stub.idSession == SSL_INVALID_ID)
		{// No free sessions, so abort
			return;
		}

		// Mark session as using this IP
		memcpy((void*)&pDcpt->ssl_SessionStubs[pDcpt->ssl_Stub.idSession].tag.v[0],
				(void*)&sktInfo.remoteIPaddress.v4Add, 4);
	}

	// Send handshake message header (hashed)
	HSPut(hTCP, SSL_CLIENT_HELLO);
	HSPut(hTCP, 0x00);				
	HSPut(hTCP, 0x00);				// Message length is 40 bytes,
	if(pDcpt->ssl_Stub.Flags.bNewSession)	// plus 32 more if a session
		HSPut(hTCP, 43);			// ID is being included.
	else
		HSPut(hTCP, 43+32);
	
	// Send 
	HSPut(hTCP, SSL_VERSION_HI);
	HSPut(hTCP, SSL_VERSION_LO);
	
	// Put Client.Random
	HSPutArray(hTCP, pDcpt->ssl_Keys.Local.random, 32);
	
	// Put Session ID
	if(pDcpt->ssl_Stub.Flags.bNewSession)
	{// Send no session ID
		HSPut(hTCP, 0x00);
	}
	else
	{// Send the requested Session ID
		SSLSessionSync(pDcpt->ssl_Stub.idSession);
		HSPut(hTCP, 0x20);
		HSPutArray(hTCP, pDcpt->ssl_Session.sessionID, 32);
	}
	
	// Put Cipher Suites List
	HSPutWord(hTCP, 0x0004);
	HSPutWord(hTCP, SSL_RSA_WITH_ARCFOUR_128_MD5);
	HSPutWord(hTCP, SSL_RSA_EXPORT_WITH_ARCFOUR_40_MD5);
	
	// Put Compression Methods List (just null)
	HSPut(hTCP, 0x01);
	HSPut(hTCP, 0x00);
	
	// End the handshake and save the hash
	HSEnd();
	
	// Send record
	SSLTxRecord(hTCP, pDcpt->ssl_StubID, SSL_HANDSHAKE);
	
	// Record that message was sent
	TCPRequestSSLMessage(hTCP, SSL_NO_MESSAGE);
	pDcpt->ssl_Stub.Flags.bClientHello = 1;

}
#endif

/*********************************************************************
 * Function:        uint8_t SSLRxClientHello(TCP_SOCKET hTCP)
 *
 * PreCondition:    Handshake hasher is started, and SSL has a stub
 *					assigned. 
 *
 * Input:           hTCP - the TCP Socket to send the message to
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Receives the ClientHello message, initiating a
 *					new SSL session with a client
 *
 * Note:            None
 ********************************************************************/
#if defined(TCPIP_STACK_USE_SSL_SERVER)
static void SSLRxClientHello(TCP_SOCKET hTCP)
{
	uint16_t w;
	uint8_t c, *ptrID;
    SSL_DCPT*     pDcpt = dcptSSL(_TCPIPStackNetIx(sslSyncIf));
	
	// Make sure entire message is ready
	if(TCPIsGetReady(hTCP) < pDcpt->ssl_Stub.wRxHsBytesRem)
		return;
		
	// Verify handshake message sequence
	if(pDcpt->ssl_Stub.Flags.bClientHello)
		TCPRequestSSLMessage(hTCP, SSL_ALERT_HANDSHAKE_FAILURE);
	
	// Indicate that we're the server
	pDcpt->ssl_Stub.Flags.bIsServer = 1;
	
	// Read the version again
	HSGetWord(hTCP, &w);
	// Ignore the version here.  It must be at least 3.0 to receive this type
	// of message, and Safari 3.1 sends 0x0301 (TLS 1.0) even when the last 
	// connection was only 0x0300 (SSL 3.0)
		
	// Make sure the session keys are synced
	SSLKeysSync(pDcpt->ssl_StubID);
	
	// Read the Client.Random array
	HSGetArray(hTCP, pDcpt->ssl_Keys.Remote.random, 32);
	
	// Read the Session ID length
	HSGet(hTCP, &c);
	
	// Read the Session ID if it exists
	pDcpt->ssl_Stub.Flags.bNewSession = true;
	if(c > 0u)
	{
		// Note where it will be stored in RAM
		ptrID = pDcpt->ssl_ptrHS;
		HSGetArray(hTCP, NULL, c);
		
		// Try to match it with a known session
		pDcpt->ssl_Stub.idSession = SSLSessionMatchID(ptrID);
		if(pDcpt->ssl_Stub.idSession != SSL_INVALID_ID)
			pDcpt->ssl_Stub.Flags.bNewSession = false;
	}
	
	// If we we're starting a new session, try to obtain a free one
	if(pDcpt->ssl_Stub.Flags.bNewSession)
		pDcpt->ssl_Stub.idSession = SSLSessionNew();
	
	// Read CipherSuites length
	HSGetWord(hTCP, &w);
	
	// Check for an acceptable CipherSuite
	// Right now we just ignore this and assume support for 
	// SSL_RSA_WITH_ARCFOUR_128_MD5.  If we request this suite later 
	// and it isn't supported, the client will kill the connection.
	HSGetArray(hTCP, NULL, w);
	
	// Read the Compression Methods length
	HSGet(hTCP, &c);
	
	// Check for an acceptable Compression Method
	// Right now we just ignore this and assume support for
	// NULL_COMPRESSION.  If we request this later and the client
	// doesn't really support it, they'll kill the connection.
	HSGetArray(hTCP, NULL, c);
	
	// For TLS compatibility, we must ignore further bytes in ClientHello.
	// FF2+ may send "extensions" ad other things we don't support
	HSGetArray(hTCP, NULL, pDcpt->ssl_Stub.wRxBytesRem);
	
	// Mark message as received and request a ServerHello
	pDcpt->ssl_Stub.Flags.bClientHello = 1;
	TCPRequestSSLMessage(hTCP, SSL_SERVER_HELLO);

}
#endif

/*********************************************************************
 * Function:        uint8_t SSLRxAntiqueClientHello(TCP_SOCKET hTCP)
 *
 * PreCondition:    Handshake hasher is started, and SSL has a stub
 *					assigned.
 *
 * Input:           hTCP - the TCP Socket to send the message to
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Receives the SSLv2 ClientHello message, initiating
 *					a new SSL session with a client
 *
 * Note:            This is the only SSLv2 message we support, and
 *					is provided for browsers seeking backwards
 *					compatibility.  Connections must be upgraded to
 *					SSLv3.0 immediately following, otherwise the 
 *					connection will fail.
 ********************************************************************/
#if defined(TCPIP_STACK_USE_SSL_SERVER)
static void SSLRxAntiqueClientHello(TCP_SOCKET hTCP)
{
	uint16_t suiteLen, idLen, randLen;
	uint8_t c;
    SSL_DCPT*     pDcpt = dcptSSL(_TCPIPStackNetIx(sslSyncIf));
	
	// Make sure entire message is ready
	if(TCPIsGetReady(hTCP) < pDcpt->ssl_Stub.wRxHsBytesRem)
		return;
		
	// Verify handshake message sequence
	if(pDcpt->ssl_Stub.Flags.bClientHello)
		TCPRequestSSLMessage(hTCP, SSL_ALERT_HANDSHAKE_FAILURE);
		
	// Indicate that we're the server
	pDcpt->ssl_Stub.Flags.bIsServer = 1;
	
	// Make sure the session keys are synced
	SSLKeysSync(pDcpt->ssl_StubID);
	
	// Read and verify the handshake message type
	HSGet(hTCP, &c);
	if(c != 0x01u)
	{// This message is not supported, so handshake fails
		TCPRequestSSLMessage(hTCP, SSL_ALERT_HANDSHAKE_FAILURE);
		return;
	}
	
	// Read and verify the version
	HSGet(hTCP, &c);
	if(c != SSL_VERSION_HI)
	{// Version is too low, so handshake fails
		TCPRequestSSLMessage(hTCP, SSL_ALERT_HANDSHAKE_FAILURE);
		return;
	}
	HSGet(hTCP, &c);	// Ignore low byte of version number
	
	// Read the CipherSuite length
	HSGetWord(hTCP, &suiteLen);
	
	// Read Session ID Length
	HSGetWord(hTCP, &idLen);
	
	// Read Challenge (Client.Random) length
	HSGetWord(hTCP, &randLen);
		
	// Check for an acceptable CipherSuite
	// Right now we just ignore this and assume support for 
	// SSL_RSA_WITH_ARCFOUR_128_MD5.  If we request this suite later 
	// and it isn't supported, the client will kill the connection.
	HSGetArray(hTCP, NULL, suiteLen);
	
	// Read the SessionID
	// SSLv3 clients will send a v3 ClientHello when resuming, so
	// this is always a new session.  Therefore, ignore the ID
	HSGetArray(hTCP, NULL, idLen);
	
	// Obtain a new session
	pDcpt->ssl_Stub.idSession = SSLSessionNew();
	pDcpt->ssl_Stub.Flags.bNewSession = 1;
	
	// Read Client.Random
	// This needs to be 32 bytes, so zero-pad the left side
	for(c = 0; c < 32 - randLen; c++)
		pDcpt->ssl_Keys.Remote.random[c] = 0;
	HSGetArray(hTCP, &pDcpt->ssl_Keys.Remote.random[c], randLen);
	
	// Mark message as received and request a ServerHello
	TCPRequestSSLMessage(hTCP, SSL_SERVER_HELLO);
	pDcpt->ssl_Stub.Flags.bClientHello = 1;
}
#endif

/*********************************************************************
 * Function:        void SSLRxServerHello(TCP_SOCKET hTCP)
 *
 * PreCondition:    pDcpt->ssl_Stub is synchronized and HSStart() has been
 *					called.
 *
 * Input:           hTCP - the TCP Socket to read from
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Receives the ServerHello from the remote server
 *
 * Note:            None
 ********************************************************************/
#if defined(TCPIP_STACK_USE_SSL_CLIENT)
static void SSLRxServerHello(TCP_SOCKET hTCP)
{
	uint8_t b, sessionID[32];
	uint16_t w;
    SSL_DCPT*     pDcpt = dcptSSL(_TCPIPStackNetIx(sslSyncIf));
		
	// Make sure entire message is ready
	if(TCPIsGetReady(hTCP) < pDcpt->ssl_Stub.wRxHsBytesRem)
		return;
		
	// Verify handshake message sequence
	if(!pDcpt->ssl_Stub.Flags.bClientHello || pDcpt->ssl_Stub.Flags.bServerHello)
		TCPRequestSSLMessage(hTCP, SSL_ALERT_HANDSHAKE_FAILURE);
	
	// Make sure correct session and key set are loaded
	SSLKeysSync(pDcpt->ssl_StubID);
	
	// Read Version (2)
	HSGetWord(hTCP, NULL);
	
	// Read Server.Random (32)
	HSGetArray(hTCP, pDcpt->ssl_Keys.Remote.random, 32);
	
	// Read Session ID Length (byte)
	HSGet(hTCP, &b);
	
	// Read Session ID (if any)
	SSLSessionSync(pDcpt->ssl_Stub.idSession);
	if(b != 0u)
	{
		HSGetArray(hTCP, sessionID, b);

		// If reusing a session, check if our session ID was accepted
		if(!pDcpt->ssl_Stub.Flags.bNewSession &&
			memcmp((void*)pDcpt->ssl_Session.sessionID, (void*)sessionID, 32) == 0)
		{// Session restart was accepted
			// Nothing to do here...move along
		}
		else
		{// This is a new session
			memcpy((void*)pDcpt->ssl_Session.sessionID, (void*)sessionID, 32);

			// Reset the RxServerCertificate state machine
			pDcpt->ssl_Stub.dwTemp.v[0] = RX_SERVER_CERT_START;
		}
	}
	else
	{
		// Session is non-resumable, so invalidate its tag
		pDcpt->ssl_SessionStubs[pDcpt->ssl_Stub.idSession].tag.Val = 0;
	}
	
	// Read and verify Cipher Suite (uint16_t)
	HSGetWord(hTCP, &w);
	if(w != SSL_RSA_WITH_ARCFOUR_128_MD5)
		TCPRequestSSLMessage(hTCP, SSL_ALERT_HANDSHAKE_FAILURE);
	
	// Read and verify Compression Method (uint8_t)
	HSGet(hTCP, &b);
	if(b != 0x00u)
		TCPRequestSSLMessage(hTCP, SSL_ALERT_HANDSHAKE_FAILURE);
	
	// Note that message was received
	pDcpt->ssl_Stub.Flags.bServerHello = 1;
	
	// Note that we updated session data
    pDcpt->ssl_SessionUpdated = true;
}
#endif

/*********************************************************************
 * Function:        uint8_t SSLTxServerHello(TCP_SOCKET hTCP)
 *
 * PreCondition:    None
 *
 * Input:           hTCP - the TCP Socket to send the message to
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Transmits the ServerHello message.
 *
 * Note:            None
 ********************************************************************/
#if defined(TCPIP_STACK_USE_SSL_SERVER)
static void SSLTxServerHello(TCP_SOCKET hTCP)
{
	uint8_t i;
    SSL_DCPT*     pDcpt = dcptSSL(_TCPIPStackNetIx(sslSyncIf));
	
	// Only continue if the session has been obtained
	if(pDcpt->ssl_Stub.idSession == SSL_INVALID_ID)
		return;
	
	// Make sure enough space is available to transmit
	if(TCPIsPutReady(hTCP) < 78u)
		return;
	
	// Restart the handshake hasher
	HSStart();
	
	// Sync the session and keys
	SSLKeysSync(pDcpt->ssl_StubID);
	SSLSessionSync(pDcpt->ssl_Stub.idSession);
	
	// If this session is new, generate an ID
	if(pDcpt->ssl_Stub.Flags.bNewSession)
	{
		for(i = 0; i < 32u; i++)
			pDcpt->ssl_Session.sessionID[i] = SYS_RANDOM_GET();
        pDcpt->ssl_SessionUpdated = true;
		
		// Tag this session identifier
		memcpy((void*)&pDcpt->ssl_SessionStubs[pDcpt->ssl_Stub.idSession].tag.v[1],
			(void*)(pDcpt->ssl_Session.sessionID), 3);
	}

	// Send handshake message header (hashed)
	HSPut(hTCP, SSL_SERVER_HELLO);
	HSPut(hTCP, 0x00);				
	HSPut(hTCP, 0x00);				
	HSPut(hTCP, 70);
	
	// Send the version number
	HSPut(hTCP, SSL_VERSION_HI);
	HSPut(hTCP, SSL_VERSION_LO);
	
	// Put Server.Random
	#if defined(TCPIP_STACK_USE_SNTP_CLIENT)
	{
		TCPIP_UINT32_VAL temp;
		
		temp.Val = SNTPGetUTCSeconds();
		pDcpt->ssl_Keys.Local.random[0] = temp.v[3];
		pDcpt->ssl_Keys.Local.random[1] = temp.v[2];
		pDcpt->ssl_Keys.Local.random[2] = temp.v[1];
		pDcpt->ssl_Keys.Local.random[3] = temp.v[0];
		i = 4;
	}
	#else
		i = 0;
	#endif
	while(i < 32u)
		pDcpt->ssl_Keys.Local.random[i++] = SYS_RANDOM_GET();
	HSPutArray(hTCP, pDcpt->ssl_Keys.Local.random, 32);
	
	// Put Session ID
	HSPut(hTCP, 0x20);
	HSPutArray(hTCP, pDcpt->ssl_Session.sessionID, 32);
	
	// Put Cipher Suites
	HSPutWord(hTCP, SSL_RSA_WITH_ARCFOUR_128_MD5);
	
	// Put Compression Method (just null)
	HSPut(hTCP, 0x00);
	
	// End the handshake and save the hash
	HSEnd();
	
	// Send record
	SSLTxRecord(hTCP, pDcpt->ssl_StubID, SSL_HANDSHAKE);
	
	// Record that message was sent and request the next message
	pDcpt->ssl_Stub.Flags.bServerHello = 1;
	TCPRequestSSLMessage(hTCP, SSL_NO_MESSAGE);
	if(pDcpt->ssl_Stub.Flags.bNewSession)
		TCPRequestSSLMessage(hTCP, SSL_CERTIFICATE);
	else
		TCPRequestSSLMessage(hTCP, SSL_CHANGE_CIPHER_SPEC);
	
	// Set up to transmit certificate
	pDcpt->ssl_Stub.dwTemp.Val = SSL_CERT_LEN;
}
#endif

/*********************************************************************
 * Function:        void SSLRxServerCertificate(TCP_SOCKET hTCP)
 *
 * PreCondition:    pDcpt->ssl_Stub is synchronized and HSStart() has been
 *					called.
 *
 * Input:           hTCP - the TCP Socket to read from
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Receives ServerCertificate from the remote server,
 *					locates the public key information, and executes
 *					RSA operation.
 *
 * Note:            This shortcuts full parsing of the certificate by
 *					just finding the Public Key Algorithm identifier
 *					for RSA.  From there, the following ASN.1 struct
 *					is the public key.  That struct consists of the
 *					value for N, followed by the value for E.
 ********************************************************************/
#if defined(TCPIP_STACK_USE_SSL_CLIENT)
static void SSLRxServerCertificate(TCP_SOCKET hTCP)
{
	uint16_t len;
	uint8_t i, e[3];
	uint16_t data_length;   // number of key bytes read from certificate
	uint8_t length_bytes;  // decoded length value
	uint8_t index;         // temp index

    SSL_DCPT*     pDcpt = dcptSSL(_TCPIPStackNetIx(sslSyncIf));
	
	// Verify handshake message sequence
	if(!pDcpt->ssl_Stub.Flags.bServerHello || pDcpt->ssl_Stub.Flags.bServerCertificate)
		TCPRequestSSLMessage(hTCP, SSL_ALERT_HANDSHAKE_FAILURE);
	
	// Check state machine variable
	switch(pDcpt->ssl_Stub.dwTemp.v[0]) {

		case RX_SERVER_CERT_START:
			// Find RSA Public Key Algorithm identifier
			len = TCPFindArray(hTCP, (const uint8_t*)"\x2A\x86\x48\x86\xF7\x0D\x01\x01\x01", 9, 0, 0, false);
			
			if(len == 0xFFFFu)
			{// If not found, clear all but 10 bytes and return to wait

                data_length = TCPIsGetReady(hTCP);
                if(data_length > 10)
                {
				    HSGetArray(hTCP, NULL, data_length-10);
                }
				return;
			}
			
			// Otherwise, read it and move on
			HSGetArray(hTCP, NULL, len + 9);
			pDcpt->ssl_Stub.dwTemp.v[0]++;
					
		case RX_SERVER_CERT_FIND_KEY:
			// Search for beginning of struct
            data_length = TCPIsGetReady(hTCP);

			len = TCPFind(hTCP, 0x30, 0, 0, false);
			
			if(len == 0xFFFFu)
			{// Not found, so clear and return
				HSGetArray(hTCP, NULL, data_length);
                return;
			}
			
			// Clear up through the 0x30
			HSGetArray(hTCP, NULL, len + 1);
			
			// Increment and continue
			pDcpt->ssl_Stub.dwTemp.v[0]++;
		
		case RX_SERVER_CERT_FIND_N:
            // Make sure tag and length bytes are ready, plus one more
            len = TCPIsGetReady(hTCP);
			if(len < 11u)
				return;
			
			// Read 1 or 2 length bytes (ignore)
			HSGet(hTCP, &i);
            data_length = 0;
			if(i & 0x80)
            {
                length_bytes = i & 0x7F;
                for(index=0;index<length_bytes;index++)
                {
				HSGet(hTCP, &i);
    				data_length = (data_length<<8)+i;
                }
            }
            else
                data_length = i;

		
            // Read until 0x02  (should be the next byte)
            i = 0;
            while((i != 2) && (len > 0))
            {
                len--;
                HSGet(hTCP, &i);
            }
            if(len == 0)    // abort if 0x02 is not found
            {
        		TCPRequestSSLMessage(hTCP, SSL_ALERT_HANDSHAKE_FAILURE);
				return;
            }

			// Read 1 or 2 length bytes to pDcpt->ssl_Stub.dwTemp.v[1]
			// The next byte tells us how many bytes are in the length structure if it's MSB is set
			HSGet(hTCP, &i);
			data_length = 0;
			if(i & 0x80)
			{
    			length_bytes = i & 0x7F;
    			for(index=0;index<length_bytes;index++)
    			{
				HSGet(hTCP, &i);
    				data_length = (data_length<<8)+i;
    			}
        	}
            else
                data_length = i; // fix for single byte length < 0x80

			
            pDcpt->ssl_Stub.dwTemp.w[1] = data_length;
			// If there's one odd byte, it's a leading zero that we don't need
			if(pDcpt->ssl_Stub.dwTemp.w[1] & 0x01)
			{
				HSGet(hTCP, NULL);
				pDcpt->ssl_Stub.dwTemp.w[1]--;
			}
			
			// The max modulus we support is 2048 bits
			if(pDcpt->ssl_Stub.dwTemp.w[1] > SSL_RSA_CLIENT_SIZE/8)
			{
				TCPRequestSSLMessage(hTCP, SSL_ALERT_HANDSHAKE_FAILURE);
				pDcpt->ssl_Stub.dwTemp.w[1] = SSL_RSA_CLIENT_SIZE/8;
			}

			// Increment and continue
			pDcpt->ssl_Stub.dwTemp.v[0]++;
            // fall through to RX_SERVER_CERT_READ_N 
		
		case RX_SERVER_CERT_READ_N:
			// Make sure pDcpt->ssl_Stub.dwTemp.w[1] bytes are ready
            data_length = TCPIsGetReady(hTCP);
			if(data_length < pDcpt->ssl_Stub.dwTemp.w[1])
				return;
			
			// N will be stored in pDcpt->ssl_Buffer, which is currently in use
			// for handshaking.  We can stop the handshake hashing, read 
			// and hash this data, then resume more handshake hashing
			HSEnd();
			
			// Claim an SSL Buffer for RSA operations
			SSLBufferAlloc(&pDcpt->ssl_Stub.idRxBuffer);
			if(pDcpt->ssl_Stub.idRxBuffer == SSL_INVALID_ID)
				return;
				
			// Make sure we can claim RSA Engine
			if(!RSABeginEncrypt(sslSyncIf, pDcpt->ssl_Stub.dwTemp.w[1]))
				return;
			pDcpt->ssl_RSAStubID = pDcpt->ssl_StubID;
			
			// Read N to proper location
			HSGetArray(hTCP, pDcpt->ssl_Buffer.full, pDcpt->ssl_Stub.dwTemp.w[1]);
			
            if (pDcpt->ssl_Stub.supplementaryDataType == SSL_SUPPLEMENTARY_DATA_CERT_PUBLIC_KEY)
            {
                SSL_PKEY_INFO * tmpPKeyPtr = ((SSL_PKEY_INFO *)pDcpt->ssl_Stub.supplementaryBuffer);
                tmpPKeyPtr->pub_size_bytes = pDcpt->ssl_Stub.dwTemp.w[1];
                if (tmpPKeyPtr->pub_size_bytes <= sizeof (tmpPKeyPtr->pub_key))
                    memcpy (&tmpPKeyPtr->pub_key[0], pDcpt->ssl_Buffer.full, tmpPKeyPtr->pub_size_bytes);
            }


			// Hash what we just read
			SSLHashSync(pDcpt->ssl_Stub.idSHA1);
			HashAddData(&pDcpt->ssl_Hash, pDcpt->ssl_Buffer.full, pDcpt->ssl_Stub.dwTemp.w[1]);
			SSLHashSync(pDcpt->ssl_Stub.idMD5);
			HashAddData(&pDcpt->ssl_Hash, pDcpt->ssl_Buffer.full, pDcpt->ssl_Stub.dwTemp.w[1]);
			
			// Generate { SSL_VERSION random[46] } as pre-master secret & save
			SSLSessionSync(pDcpt->ssl_Stub.idSession);
			pDcpt->ssl_Session.masterSecret[0] = SSL_VERSION_HI;
			pDcpt->ssl_Session.masterSecret[1] = SSL_VERSION_LO;
			for(i = 2; i < 48u; i++)
				pDcpt->ssl_Session.masterSecret[i] = SYS_RANDOM_GET();
            pDcpt->ssl_SessionUpdated = true;
			
			// Set RSA engine to use this data and key
			RSASetData(sslSyncIf, pDcpt->ssl_Session.masterSecret, 48, RSA_BIG_ENDIAN);
			RSASetN(pDcpt->ssl_Buffer.full, RSA_BIG_ENDIAN);
			RSASetResult(pDcpt->ssl_Buffer.full+pDcpt->ssl_Stub.dwTemp.w[1], RSA_BIG_ENDIAN);
			
			// Start a new hash
			HSStart();
			
			// Increment and continue
			pDcpt->ssl_Stub.dwTemp.v[0]++;
			
		case RX_SERVER_CERT_READ_E: // A server message recieved by the client
			// Make sure 5 bytes are ready
			if(TCPIsGetReady(hTCP) < 5u)
				return;

			// Read 0x02
			HSGet(hTCP, NULL);
			
			// Read 1 length byte to temp
			HSGet(hTCP, &i);
			if(i > 3u)
				i = 3;
			
			// Read E to temp
			HSGetArray(hTCP, e, i);

                        if (pDcpt->ssl_Stub.supplementaryDataType == SSL_SUPPLEMENTARY_DATA_CERT_PUBLIC_KEY)
                        {
                            SSL_PKEY_INFO * tmpPKeyPtr = ((SSL_PKEY_INFO *)pDcpt->ssl_Stub.supplementaryBuffer);
                            if (i <= sizeof (tmpPKeyPtr->pub_e))
                                memcpy (&tmpPKeyPtr->pub_e[0], e, i);
                        }

			// Set RSA engine to encrypt with E
			RSASetE(e, i, RSA_BIG_ENDIAN);
			
			// Increment and continue
			pDcpt->ssl_Stub.dwTemp.v[0]++;
		
		case RX_SERVER_CERT_CLEAR:
			// Clear up to pDcpt->ssl_Stub.wRxHsBytesRem from hTCP
			len = TCPIsGetReady(hTCP);
			if(len > pDcpt->ssl_Stub.wRxHsBytesRem)
				len = pDcpt->ssl_Stub.wRxHsBytesRem;
			HSGetArray(hTCP, NULL, len);
			
			// If we're done, kick off the RSA encryption next
			if(pDcpt->ssl_Stub.wRxHsBytesRem == 0u)
			{
				// Set periodic function to do RSA operation
				pDcpt->ssl_Stub.Flags.bRSAInProgress = 1;
				pDcpt->ssl_SessionStubs[pDcpt->ssl_Stub.idSession].lastUsed += SSL_RSA_LIFETIME_EXTENSION;
				
				// Note that we've received this message
				pDcpt->ssl_Stub.Flags.bServerCertificate = 1;	
			}
			
			break;
	}
}
#endif

/*********************************************************************
 * Function:        uint8_t SSLTxServerCertificate(TCP_SOCKET hTCP)
 *
 * PreCondition:    None
 *
 * Input:           hTCP - the TCP Socket to send the message to
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Transmits the Certificate message with the 
 *					server's specified public key certificate.
 *
 * Note:            Certificate is defined in CustomSSLCert.c.
 *					This function requires special handling for
 *					partial records because the certificate will 
 *					likely be larger than the TCP buffer, and SSL
 *					handshake messages are constrained to fit in a
 *					single SSL handshake record
 ********************************************************************/
#if defined(TCPIP_STACK_USE_SSL_SERVER)
static void SSLTxServerCertificate(TCP_SOCKET hTCP)
{
	uint16_t len;
	const uint8_t* loc;
    SSL_DCPT*     pDcpt = dcptSSL(_TCPIPStackNetIx(sslSyncIf));
	
	// Restart the handshake hasher
	HSStart();
	
	// See how much we can write
	len = TCPIsPutReady(hTCP);
	
	// If full certificate remains, write the headers
	if(pDcpt->ssl_Stub.dwTemp.Val == SSL_CERT_LEN)
	{
		// Make sure we can send all headers plus one byte
		if(len < 11u)
			return;
		
		// Transmit the handshake headers
		HSPut(hTCP, SSL_CERTIFICATE);
		HSPut(hTCP, 0x00);
		HSPutWord(hTCP, SSL_CERT_LEN + 3 + 3);
		
		// Send length of all certificates
		HSPut(hTCP, 0x00);
		HSPutWord(hTCP, SSL_CERT_LEN + 3);
		
		// Send length of this (only) certificate
		HSPut(hTCP, 0x00);
		HSPutWord(hTCP, SSL_CERT_LEN);
		
		// Put in the record header and begin the partial record sending
		SSLStartPartialRecord(hTCP, pDcpt->ssl_StubID, SSL_HANDSHAKE, SSL_CERT_LEN + 3 + 3 + 4);
		
		// Update free space
		len -= 10;
	}
	
	// Figure out where to start, and how much to send
	loc = SSL_CERT + (SSL_CERT_LEN - pDcpt->ssl_Stub.dwTemp.Val);
	if(pDcpt->ssl_Stub.dwTemp.Val < len)
		len = pDcpt->ssl_Stub.dwTemp.Val;
		
	// Write the bytes
	HSPutArray(hTCP, loc, len);
	pDcpt->ssl_Stub.dwTemp.Val -= len;
	
	// End the handshake and save the hash
	HSEnd();
	
	// Send record
	SSLFlushPartialRecord(hTCP);
		
	// Check if entire certificate was sent
	if(pDcpt->ssl_Stub.dwTemp.Val == 0u)
	{
		// Finish the partial record
		SSLFinishPartialRecord(hTCP);
		
		// Record that message was sent and request a Certificate
		TCPRequestSSLMessage(hTCP, SSL_NO_MESSAGE);
		TCPRequestSSLMessage(hTCP, SSL_SERVER_HELLO_DONE);
		pDcpt->ssl_Stub.Flags.bServerCertificate = 1;
	}
}
#endif

/*********************************************************************
 * Function:        uint8_t SSLTxServerHelloDone(TCP_SOCKET hTCP)
 *
 * PreCondition:    None
 *
 * Input:           hTCP - the TCP Socket to send the message to
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Transmits the ServerHelloDone message.
 *
 * Note:            None
 ********************************************************************/
#if defined(TCPIP_STACK_USE_SSL_SERVER)
static void SSLTxServerHelloDone(TCP_SOCKET hTCP)
{
    SSL_DCPT*     pDcpt = dcptSSL(_TCPIPStackNetIx(sslSyncIf));

	// Make sure enough space is available to transmit
	if(TCPIsPutReady(hTCP) < 4u)
		return;
	
	// Restart the handshake hasher
	HSStart();
	
	// Send handshake message header (hashed)
	HSPut(hTCP, SSL_SERVER_HELLO_DONE);
	HSPut(hTCP, 0x00);				
	HSPut(hTCP, 0x00);				
	HSPut(hTCP, 0x00);

	// Message has no content, so we're done
	
	// End the handshake and save the hash
	HSEnd();
	
	// Send record
	SSLTxRecord(hTCP, pDcpt->ssl_StubID, SSL_HANDSHAKE);
	
	// Record that message was sent
	TCPRequestSSLMessage(hTCP, SSL_NO_MESSAGE);
	pDcpt->ssl_Stub.Flags.bServerHelloDone = 1;
}
#endif

/*********************************************************************
 * Function:        void SSLTxClientKeyExchange(TCP_SOCKET hTCP)
 *
 * PreCondition:    pDcpt->ssl_Stub is synchronized, pDcpt->ssl_Stub.dwTemp.v[1]
 *					contains the length of the public key, and 
 *					the RxBuffer contains the encrypted pre-master
 *					secret at address 0x80.
 *
 * Input:           hTCP - the TCP Socket to write the message to
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Transmits the encrypted pre-master secret to the
 *					server and requests the Change Cipher Spec.  Also
 *					generates the Master Secret from the pre-master
 *					secret that was used.
 *
 * Note:            None
 ********************************************************************/
#if defined(TCPIP_STACK_USE_SSL_CLIENT)
static void SSLTxClientKeyExchange(TCP_SOCKET hTCP)
{
	uint16_t len;
    SSL_DCPT*     pDcpt = dcptSSL(_TCPIPStackNetIx(sslSyncIf));
	
	// Load length of modulus from RxServerCertificate
	len = pDcpt->ssl_Stub.dwTemp.w[1];
	
	// Make sure there's len+9 bytes free
	if(TCPIsPutReady(hTCP) < len + 9)
		return;
	
	// Start the handshake processor
	HSStart();

	// Send handshake message header (hashed)
	HSPut(hTCP, SSL_CLIENT_KEY_EXCHANGE);
	HSPut(hTCP, 0x00);				
	HSPut(hTCP, (len>>8)&0xFF);				// Message length is (length of key) bytes
	HSPut(hTCP, len&0xFF);
	
	// Suspend the handshake hasher and load the buffer
	HSEnd();
	SSLBufferSync(pDcpt->ssl_Stub.idRxBuffer);	

	// Send encrypted pre-master secret
	TCPPutArray(hTCP, (uint8_t*) pDcpt->ssl_Buffer.full + len, len);
	
	// Free the RSA Engine
	RSAEndUsage(sslSyncIf);
	pDcpt->ssl_RSAStubID = SSL_INVALID_ID;

	// Hash what we just sent
	SSLHashSync(pDcpt->ssl_Stub.idSHA1);
	HashAddData(&pDcpt->ssl_Hash, pDcpt->ssl_Buffer.full + len, len);
	SSLHashSync(pDcpt->ssl_Stub.idMD5);
	HashAddData(&pDcpt->ssl_Hash, pDcpt->ssl_Buffer.full + len, len);
	
	// Generate the Master Secret
	SSLKeysSync(pDcpt->ssl_StubID);
	SSLSessionSync(pDcpt->ssl_Stub.idSession);
	GenerateHashRounds(3, pDcpt->ssl_Keys.Local.random, pDcpt->ssl_Keys.Remote.random);
	memcpy(pDcpt->ssl_Session.masterSecret, (void*)pDcpt->ssl_Buffer.hashRounds.temp, 48);
    pDcpt->ssl_SessionUpdated = true;

	
	// Free the buffer with the encrypted pre-master secret
	SSLBufferFree(&pDcpt->ssl_Stub.idRxBuffer);
	
	// Restart the handshaker
	HSStart();
	
	// Send the record
	SSLTxRecord(hTCP, pDcpt->ssl_StubID, SSL_HANDSHAKE);
	
	// Request a Change Cipher Spec and Finished message
	TCPRequestSSLMessage(hTCP, SSL_NO_MESSAGE);
	TCPRequestSSLMessage(hTCP, SSL_CHANGE_CIPHER_SPEC);
	
	// Note that this message was sent
	pDcpt->ssl_Stub.Flags.bClientKeyExchange = 1;
}
#endif

/*********************************************************************
 * Function:        void SSLRxClientKeyExchange(TCP_SOCKET hTCP)
 *
 * PreCondition:    pDcpt->ssl_Stub is synchronized and HSStart() has been
 *					called.
 *
 * Input:           hTCP - the TCP Socket to read from
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Receives the ClientKeyExchange message and begins
 *                  the decryption process.
 *
 * Note:            None
 ********************************************************************/
#if defined(TCPIP_STACK_USE_SSL_SERVER)
static void SSLRxClientKeyExchange(TCP_SOCKET hTCP)
{
	uint16_t wKeyLength;
    SSL_DCPT*     pDcpt = dcptSSL(_TCPIPStackNetIx(sslSyncIf));
	
	// Make sure entire message is ready
	if(TCPIsGetReady(hTCP) < pDcpt->ssl_Stub.wRxHsBytesRem)
		return;
	
	// Verify handshake message sequence
	if(!pDcpt->ssl_Stub.Flags.bServerHello || pDcpt->ssl_Stub.Flags.bClientKeyExchange)
		TCPRequestSSLMessage(hTCP, SSL_ALERT_HANDSHAKE_FAILURE);
	
	// Obtain a buffer to use
	SSLBufferAlloc(&pDcpt->ssl_Stub.idRxBuffer);
	if(pDcpt->ssl_Stub.idRxBuffer == SSL_INVALID_ID)
		return;
	
	// Claim the RSA engine
	if(!RSABeginDecrypt(sslSyncIf))
		return;
	pDcpt->ssl_RSAStubID = pDcpt->ssl_StubID;
	
	// Read the data
	wKeyLength = pDcpt->ssl_Stub.wRxHsBytesRem;
	HSEnd();
	HSStart();
	HSGetArray(hTCP, NULL, wKeyLength);
	HSEnd();
	RSASetData(sslSyncIf, pDcpt->ssl_Buffer.full, wKeyLength, RSA_BIG_ENDIAN);








/*
the pre-master key is located in pDcpt->ssl_Buffer.full. RSASetData() populates it into rsaDcpt->X.
That is all fine. The next step would be for server RSA to decrypt it using its private key. Where does the
decryption take place and how is the private key obtained. It takes place in the SM_RSA_DECRYPT_START
case statements of rsa.c::RSAStep() which calls _RSAModExpROM().
 * The private key is composed of eliments SSL_dP and SSL_dQ, which are eliments of the Chinese Remainder
 * Theorum. These elements are hardcoded in custom_ssl_cert.c
 
 NOTE: After _RSAModExpROM() is complete, the decrypted pre-master value will be in rsaDcpt->X.
 * The next step is to find out how SSL uses rsaDcpt->X to create the symetric key and respond
 * back to the client to let it know that it recieved the ClientKeyExchange
 * message (facilitaed via bClientKeyExchange below).



*/







        
	pDcpt->ssl_BufferID = pDcpt->ssl_Stub.idRxBuffer;
	
	// Note that message was received
	pDcpt->ssl_Stub.Flags.bClientKeyExchange = 1;
	
	// Kick off the RSA decryptor
	pDcpt->ssl_Stub.Flags.bRSAInProgress = 1;
	pDcpt->ssl_SessionStubs[pDcpt->ssl_Stub.idSession].lastUsed += SSL_RSA_LIFETIME_EXTENSION;
	
}
#endif

/*********************************************************************
 * Function:        void SSLTxCCSFin(TCP_SOCKET hTCP)
 *
 * PreCondition:    pDcpt->ssl_Stub is synchronized, and the current session
 *					has a valid pre-master secret to use.
 *
 * Input:           hTCP - the TCP Socket to write the message to
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:    Generates the session keys from the master secret,
 *		then allocates and generates the encryption 
 *		context.  Once processing is complete, transmits
 *		the Change Cipher Spec message and the Finished
 *		handshake message to the server.
 *
 * Note:            None
 ********************************************************************/
static void SSLTxCCSFin(TCP_SOCKET hTCP)
{
	uint8_t data[20];
    SSL_DCPT*     pDcpt = dcptSSL(_TCPIPStackNetIx(sslSyncIf));
	
	// Make sure enough space is available for both
	if(TCPIsPutReady(hTCP) < 68u)
		return;

	// Sync up the session
	SSLSessionSync(pDcpt->ssl_Stub.idSession);
	SSLKeysSync(pDcpt->ssl_StubID);
	SSLBufferSync(SSL_INVALID_ID);
			
	// Send the CCS (not a handshake message)
	TCPPut(hTCP, 1);
	SSLTxRecord(hTCP, pDcpt->ssl_StubID, SSL_CHANGE_CIPHER_SPEC);
	pDcpt->ssl_Stub.Flags.bLocalChangeCipherSpec = 1;
	
	// If keys are not ready, generate them
	if(!pDcpt->ssl_Stub.Flags.bKeysValid)
	{
		// Obtain two full buffers for the Sboxes
		SSLBufferAlloc(&pDcpt->ssl_Stub.idTxBuffer);
		SSLBufferAlloc(&pDcpt->ssl_Stub.idRxBuffer);
		if(pDcpt->ssl_Stub.idTxBuffer == SSL_INVALID_ID || pDcpt->ssl_Stub.idRxBuffer == SSL_INVALID_ID)
			return;

		// Generate the keys
		SSLHashSync(SSL_INVALID_ID);
		GenerateSessionKeys();
		pDcpt->ssl_Stub.Flags.bKeysValid = 1;
	}
	
	// Reset the sequence counters
	pDcpt->ssl_Keys.Local.app.sequence = 0;	
	
	// Start the handshake data processor
	HSStart();

	// First, write the handshake header
	HSPut(hTCP, SSL_FINISHED);
	HSPut(hTCP, 0x00);
	HSPut(hTCP, 0x00);
	HSPut(hTCP, 0x24);

	// Calculate the Finished hashes
	CalculateFinishedHash(pDcpt->ssl_Stub.idMD5, !pDcpt->ssl_Stub.Flags.bIsServer, data);
	HSPutArray(hTCP, data, 16);
	CalculateFinishedHash(pDcpt->ssl_Stub.idSHA1, !pDcpt->ssl_Stub.Flags.bIsServer, data);
	HSPutArray(hTCP, data, 20);	

	// Hash this message to the handshake hash
	HSEnd();

	// Send the record
	SSLTxRecord(hTCP, pDcpt->ssl_StubID, SSL_HANDSHAKE);
	
	// Update the connection state
	TCPRequestSSLMessage(hTCP, SSL_NO_MESSAGE);
	pDcpt->ssl_Stub.Flags.bLocalFinished = 1;
	
	// If complete, note that
	if(pDcpt->ssl_Stub.Flags.bRemoteFinished)
	{
		TCPSSLHandshakeComplete(hTCP);
		SSLHashFree(&pDcpt->ssl_Stub.idMD5);
		SSLHashFree(&pDcpt->ssl_Stub.idSHA1);
	}

}

/*********************************************************************
 * Function:        void SSLRxCCS(TCP_SOCKET hTCP)
 *
 * PreCondition:    pDcpt->ssl_Stub is synchronized.
 *
 * Input:           hTCP - the TCP Socket to read from
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Receives a ChangeCipherSpec from the remote server
 *
 * Note:            None
 ********************************************************************/
static void SSLRxCCS(TCP_SOCKET hTCP)
{
    SSL_DCPT*     pDcpt = dcptSSL(_TCPIPStackNetIx(sslSyncIf));
	// Only proceed if RSA is done
	if(pDcpt->ssl_Stub.Flags.bRSAInProgress)
		return;
	
	// Verify handshake message sequence
	if(!pDcpt->ssl_Stub.Flags.bClientHello || !pDcpt->ssl_Stub.Flags.bServerHello)
		TCPRequestSSLMessage(hTCP, SSL_ALERT_HANDSHAKE_FAILURE);
	
	// Allocate a hash for MACing data
	SSLHashAlloc(&pDcpt->ssl_Stub.idRxHash);

	// Make sure entire message is ready and an RX hash is allocated
	if(TCPIsGetReady(hTCP) < pDcpt->ssl_Stub.wRxBytesRem 
		|| pDcpt->ssl_Stub.idRxHash == SSL_INVALID_ID)
		return;
	
	// If keys are not ready, generate them
	if(!pDcpt->ssl_Stub.Flags.bKeysValid)
	{
		// Sync up the session
		SSLSessionSync(pDcpt->ssl_Stub.idSession);
		SSLKeysSync(pDcpt->ssl_StubID);
		SSLBufferSync(SSL_INVALID_ID);

		// Obtain two full buffers for the Sboxes
		SSLBufferAlloc(&pDcpt->ssl_Stub.idTxBuffer);
		SSLBufferAlloc(&pDcpt->ssl_Stub.idRxBuffer);
		if(pDcpt->ssl_Stub.idTxBuffer == SSL_INVALID_ID || pDcpt->ssl_Stub.idRxBuffer == SSL_INVALID_ID)
			return;

		// Generate the keys
		SSLHashSync(SSL_INVALID_ID);
		GenerateSessionKeys();
		pDcpt->ssl_Stub.Flags.bKeysValid = 1;
	}
	
	// Read the CCS message (ignoring its contents)
	pDcpt->ssl_Stub.wRxBytesRem -= TCPGetArray(hTCP, NULL, pDcpt->ssl_Stub.wRxBytesRem);
	
	// Note that message was received
	SSLKeysSync(pDcpt->ssl_StubID);
	pDcpt->ssl_Keys.Remote.app.sequence = 0;
	pDcpt->ssl_Stub.Flags.bRemoteChangeCipherSpec = 1;
}

/*********************************************************************
 * Function:        void SSLRxFinished(TCP_SOCKET hTCP)
 *
 * PreCondition:    pDcpt->ssl_Stub is synchronized and HSStart() has been
 *					called.
 *
 * Input:           hTCP - the TCP Socket to read from
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Receives the Finished message from remote node
 *
 * Note:            None
 ********************************************************************/
static void SSLRxFinished(TCP_SOCKET hTCP)
{
	uint8_t rxHash[20], expectedHash[20];
    SSL_DCPT*     pDcpt = dcptSSL(_TCPIPStackNetIx(sslSyncIf));
	
	// Make sure entire message is ready
	if(TCPIsGetReady(hTCP) < pDcpt->ssl_Stub.wRxHsBytesRem)
		return;
	
	// Verify handshake message sequence
	if(!pDcpt->ssl_Stub.Flags.bRemoteChangeCipherSpec)
		TCPRequestSSLMessage(hTCP, SSL_ALERT_HANDSHAKE_FAILURE);
	
	// Make sure correct session and key set are loaded
	SSLSessionSync(pDcpt->ssl_Stub.idSession);
	SSLKeysSync(pDcpt->ssl_StubID);
	
	// Read md5_sum to temporary location
	HSGetArray(hTCP, rxHash, 16);
	
	// Calculate expected MD5 hash
	CalculateFinishedHash(pDcpt->ssl_Stub.idMD5, pDcpt->ssl_Stub.Flags.bIsServer, expectedHash);	
	if(memcmp((void*)rxHash, (void*)expectedHash, 16) != 0)
	{// Handshake hash fails
		TCPRequestSSLMessage(hTCP, SSL_ALERT_HANDSHAKE_FAILURE);
	}
	
	// Read sha_sum to temporary location
	HSGetArray(hTCP, rxHash, 20);
	
	// Calculate expected SHA-1 hash	
	CalculateFinishedHash(pDcpt->ssl_Stub.idSHA1, pDcpt->ssl_Stub.Flags.bIsServer, expectedHash);
	if(memcmp((void*)rxHash, (void*)expectedHash, 20) != 0)
	{// Handshake hash fails
		TCPRequestSSLMessage(hTCP, SSL_ALERT_HANDSHAKE_FAILURE);
	}
	
	// Note that message was received
	pDcpt->ssl_Stub.Flags.bRemoteFinished = 1;
	
	// If complete, note that, otherwise, request our own CCS message
	if(pDcpt->ssl_Stub.Flags.bLocalFinished)
	{
		TCPSSLHandshakeComplete(hTCP);
		SSLHashFree(&pDcpt->ssl_Stub.idMD5);
		SSLHashFree(&pDcpt->ssl_Stub.idSHA1);
	}
	else
		TCPRequestSSLMessage(hTCP, SSL_CHANGE_CIPHER_SPEC);

}

/*********************************************************************
 * Function:        void SSLRxAlert(TCP_SOCKET hTCP)
 *
 * PreCondition:    pDcpt->ssl_Stub is synchronized
 *
 * Input:           hTCP - the TCP Socket to read from
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Receives an alert message and decides what to do
 *
 * Note:            None
 ********************************************************************/
static void SSLRxAlert(TCP_SOCKET hTCP)
{
	uint8_t bLevel, bDesc;
    SSL_DCPT*     pDcpt = dcptSSL(_TCPIPStackNetIx(sslSyncIf));
	
	// Make sure entire message is ready
	if(TCPIsGetReady(hTCP) < pDcpt->ssl_Stub.wRxBytesRem)
		return;
	
	// Read the alert message
	TCPGet(hTCP, &bLevel);
	TCPGet(hTCP, &bDesc);
	pDcpt->ssl_Stub.wRxBytesRem -= 2;
	
	// Determine what to do
	switch(bLevel)
	{
		case SSL_ALERT_WARNING:
			// Check if a close notify was received
			if(bDesc + 0x80 == SSL_ALERT_CLOSE_NOTIFY)
				pDcpt->ssl_Stub.Flags.bCloseNotify = 1;
			
			// We don't support non-fatal warnings besides CloseNotify,
			// so the connection is always done now.  When the TCP 
			// session closes, the resources will be cleaned up.
			
			// No break here:
			// Session is terminated, so still mark Done below
			
		case SSL_ALERT_FATAL:
			// Mark session as terminated
			pDcpt->ssl_Stub.Flags.bDone = 1;			
	}
	
}

/****************************************************************************
  ===========================================================================
  Section:
	SSL Key Processing Functions
  ===========================================================================
  ***************************************************************************/

/*********************************************************************
 * Function:        RSA_STATUS SSLRSAOperation(void)
 *
 * PreCondition:    The RSA Module has been secured, an RSA operation
 *					is pending, pDcpt->ssl_Stub.wRxHsBytesRem is the value of
 *					pDcpt->ssl_Stub.wRxBytesRem after completion, and 
 *					pDcpt->ssl_Stub.wRxBytesRem is the value of 
 *					pDcpt->ssl_Stub.rxProtocol after completion.  Also requires
 *					pDcpt->ssl_Stub to be synchronized.
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Pauses connection processing until RSA calculation
 *					is complete.
 *
 * Note:            This function exists outside of the handshaking 
 *					functions so that the system does not incur the
 *					expense of resuming and suspending handshake 
 *					hashes.
 ********************************************************************/
static RSA_STATUS SSLRSAOperation(void)
{
    SSL_DCPT*     pDcpt = dcptSSL(_TCPIPStackNetIx(sslSyncIf));

	SSLBufferSync(pDcpt->ssl_Stub.idRxBuffer);
		
	// Call RSAStep to perform some RSA processing
	return RSAStep(sslSyncIf);
}

/*********************************************************************
 * Function:        void GenerateHashRounds(uint8_t num, uint8_t* rand1,
 *					uint8_t* rand2)
 *
 * PreCondition:    The SSL buffer is allocated for temporary usage
 *                  and the data (pre-master secrete)to run rounds on is in
 *                  pDcpt->ssl_Session.masterSecret
 *
 * Input:           num   - how many rounds to compute
 *                          rand1 - the first random data block to use
 *                          rand2 - the second random data block to use
 *
 * Output:          None
 *
 * Side Effects:    Destroys the SSL Buffer space
 *
 * Overview:        Generates hash rounds to find either the
 *					Master Secret or the Key Block.
 *
 * Note:            This function will overflow the buffer after 7
 *					rounds, but in practice num = 3 or num = 4.
 ********************************************************************/
static void GenerateHashRounds(uint8_t num, uint8_t* rand1, uint8_t* rand2)
{
	uint8_t i, j, c, *res;
    SSL_DCPT*     pDcpt = dcptSSL(_TCPIPStackNetIx(sslSyncIf));

	
	c = 'A';
	res = pDcpt->ssl_Buffer.hashRounds.temp;
	
	for(i = 1; i <= num; i++, c++, res += 16)
	{
		SHA1Initialize(&pDcpt->ssl_Buffer.hashRounds.hash);
		for(j = 0; j < i; j++)
                   HashAddData(&pDcpt->ssl_Buffer.hashRounds.hash, &c, 1);

		HashAddData(&pDcpt->ssl_Buffer.hashRounds.hash, pDcpt->ssl_Session.masterSecret, 48);
		HashAddData(&pDcpt->ssl_Buffer.hashRounds.hash, rand1, 32);
		HashAddData(&pDcpt->ssl_Buffer.hashRounds.hash, rand2, 32);
		SHA1Calculate(&pDcpt->ssl_Buffer.hashRounds.hash, pDcpt->ssl_Buffer.hashRounds.sha_hash);
		MD5Initialize(&pDcpt->ssl_Buffer.hashRounds.hash);
		HashAddData(&pDcpt->ssl_Buffer.hashRounds.hash, pDcpt->ssl_Session.masterSecret, 48);
		HashAddData(&pDcpt->ssl_Buffer.hashRounds.hash, pDcpt->ssl_Buffer.hashRounds.sha_hash, 20);
		MD5Calculate(&pDcpt->ssl_Buffer.hashRounds.hash, res);
	}	
}

/*********************************************************************
 * Function:        void GenerateSessionKeys(void)
 *
 * PreCondition:    The SSL buffer is allocated for temporary usage,
 *		    session keys are synced, and the TX and RX buffers
 *		    are allocated for S-boxes.
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    Destroys the SSL Buffer Space
 *
 * Overview:        Generates the session write keys and MAC secrets
 *
 * Note:            None
 ********************************************************************/
static void GenerateSessionKeys(void)
{
	// This functionality differs slightly for client and server operations
    SSL_DCPT*     pDcpt = dcptSSL(_TCPIPStackNetIx(sslSyncIf));

	#if defined(TCPIP_STACK_USE_SSL_SERVER)
	if(pDcpt->ssl_Stub.Flags.bIsServer)
	{
		// Generate the key expansion block
		GenerateHashRounds(4, pDcpt->ssl_Keys.Local.random, pDcpt->ssl_Keys.Remote.random);
		memcpy(pDcpt->ssl_Keys.Remote.app.MACSecret, (void*)pDcpt->ssl_Buffer.hashRounds.temp, 16);
		memcpy(pDcpt->ssl_Keys.Local.app.MACSecret, (void*)pDcpt->ssl_Buffer.hashRounds.temp+16, 16);
	
		// Save write keys elsewhere temporarily
		SSLHashSync(SSL_INVALID_ID);
		memcpy(&pDcpt->ssl_Hash, (void*)pDcpt->ssl_Buffer.hashRounds.temp+32, 32);
	
		// Generate ARCFOUR Sboxes
		SSLBufferSync(pDcpt->ssl_Stub.idRxBuffer);
		pDcpt->ssl_Keys.Remote.app.cryptCtx.Sbox = pDcpt->ssl_Buffer.full;
		ARCFOURInitialize(&(pDcpt->ssl_Keys.Remote.app.cryptCtx), (uint8_t*)(&pDcpt->ssl_Hash), 16);
		SSLBufferSync(pDcpt->ssl_Stub.idTxBuffer);
		pDcpt->ssl_Keys.Local.app.cryptCtx.Sbox = pDcpt->ssl_Buffer.full;
		ARCFOURInitialize(&(pDcpt->ssl_Keys.Local.app.cryptCtx), (uint8_t*)(&pDcpt->ssl_Hash)+16, 16);
		
		return;
	}
	#endif
	
	#if defined(TCPIP_STACK_USE_SSL_CLIENT)
	// Generate the key expansion block
	GenerateHashRounds(4, pDcpt->ssl_Keys.Remote.random, pDcpt->ssl_Keys.Local.random);
	memcpy(pDcpt->ssl_Keys.Local.app.MACSecret, (void*)pDcpt->ssl_Buffer.hashRounds.temp, 16);
	memcpy(pDcpt->ssl_Keys.Remote.app.MACSecret, (void*)pDcpt->ssl_Buffer.hashRounds.temp+16, 16);

	// Save write keys elsewhere temporarily
	SSLHashSync(SSL_INVALID_ID);
	memcpy(&pDcpt->ssl_Hash, (void*)pDcpt->ssl_Buffer.hashRounds.temp+32, 32);

	// Generate ARCFOUR Sboxes
	SSLBufferSync(pDcpt->ssl_Stub.idTxBuffer);
	pDcpt->ssl_Keys.Local.app.cryptCtx.Sbox = pDcpt->ssl_Buffer.full;
	ARCFOURInitialize(&(pDcpt->ssl_Keys.Local.app.cryptCtx), (uint8_t*)(&pDcpt->ssl_Hash), 16);
	SSLBufferSync(pDcpt->ssl_Stub.idRxBuffer);
	pDcpt->ssl_Keys.Remote.app.cryptCtx.Sbox = pDcpt->ssl_Buffer.full;
	ARCFOURInitialize(&(pDcpt->ssl_Keys.Remote.app.cryptCtx), (uint8_t*)(&pDcpt->ssl_Hash)+16, 16);
	#endif
	
}

/*********************************************************************
 * Function:        static void CalculateFinishedHash(uint8_t hashID,
 *									bool fromClient, uint8_t *result)
 *
 * PreCondition:    hashID has all handshake data hashed so far and
 *					the current session is synced in.
 *
 * Input:           hashID     - the hash sum to use
 *					fromClient - true if client is sender
 *					result     - where to store results
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:		Calculates the handshake hash over the data.
 *					hashID can be either MD5 or SHA-1, and this
 *					function will calculate accordingly.
 *
 * Note:            None
 ********************************************************************/
static void CalculateFinishedHash(uint8_t hashID, bool fromClient, uint8_t *result)
{
	uint8_t i;
    SSL_DCPT*     pDcpt = dcptSSL(_TCPIPStackNetIx(sslSyncIf));

	// Load the hash, but make sure updates aren't saved
	SSLHashSync(hashID);
	pDcpt->ssl_HashID = SSL_INVALID_ID;
	
	// Sync the session data so masterSecret is available
	SSLSessionSync(pDcpt->ssl_Stub.idSession);
	
	// Hash in the sender phrase & master secret
	if(fromClient)
		HashAddData(&pDcpt->ssl_Hash, (const uint8_t*)"CLNT", 4);
	else
		HashAddData(&pDcpt->ssl_Hash, (const uint8_t*)"SRVR", 4);
	HashAddData(&pDcpt->ssl_Hash, pDcpt->ssl_Session.masterSecret, 48);
	
	// Hash in the pad1
	i = 6;
	if(pDcpt->ssl_Hash.hashType == HASH_SHA1)
		i--;
	for(; i > 0u; i--)
		HashAddData(&pDcpt->ssl_Hash, (const uint8_t*)"\x36\x36\x36\x36\x36\x36\x36\x36", 8);
	
	// Calculate the inner hash result and store, start new hash
	if(pDcpt->ssl_Hash.hashType == HASH_MD5)
	{
		MD5Calculate(&pDcpt->ssl_Hash, result);
		MD5Initialize(&pDcpt->ssl_Hash);
	}
	else
	{
		SHA1Calculate(&pDcpt->ssl_Hash, result);
		SHA1Initialize(&pDcpt->ssl_Hash);
	}
	
	// Hash in master secret
	HashAddData(&pDcpt->ssl_Hash, pDcpt->ssl_Session.masterSecret, 48);
	
	// Hash in pad2
	i = 6;
	if(pDcpt->ssl_Hash.hashType == HASH_SHA1)
		i--;
	for(; i > 0u; i--)
		HashAddData(&pDcpt->ssl_Hash, (const uint8_t*)"\x5c\x5c\x5c\x5c\x5c\x5c\x5c\x5c", 8);
	
	// Hash in the inner hash result and calculate
	if(pDcpt->ssl_Hash.hashType == HASH_MD5)
	{
		HashAddData(&pDcpt->ssl_Hash, result, 16);
		MD5Calculate(&pDcpt->ssl_Hash, result);
	}
	else
	{
		HashAddData(&pDcpt->ssl_Hash, result, 20);
		SHA1Calculate(&pDcpt->ssl_Hash, result);
	}
	
}


/****************************************************************************
  ===========================================================================
  Section:
	SSL Memory Management Functions
  ===========================================================================
  ***************************************************************************/


/*********************************************************************
 * Function:        static void SSLStubAlloc(TCPIP_NET_IF* currIf)
 *
 * PreCondition:    None
 *
 * Inputs:          currIf    - interface for this SSL connection
 *
 * Outputs:         None
 *
 * Returns:			true if stub was allocated, false otherwise
 *
 * Side Effects:    None
 *
 * Overview:        Allocates a stub for use.
 *
 * Note:            None
 ********************************************************************/
static bool SSLStubAlloc(TCPIP_NET_IF* currIf)
{
	uint8_t i, mask;
    SSL_DCPT*     pOldDcpt;
    SSL_DCPT*     pDcpt = dcptSSL(_TCPIPStackNetIx(currIf));

    
	// Search for a free stub
	for(i = 0; i != SSL_MAX_CONNECTIONS; i++)
	{
        mask = 1 << i;
		if(!(pDcpt->ssl_isStubUsed & mask))
		{// Stub is free, so claim it
			pDcpt->ssl_isStubUsed |= mask;
			
			// Save stub currently in RAM
            if(sslSyncIf != 0)
            {   // old interface valid
                pOldDcpt = dcptSSL(_TCPIPStackNetIx(sslSyncIf));
                if(pOldDcpt->ssl_StubID != SSL_INVALID_ID)
                {
                    SSLSave((uint8_t*)&pOldDcpt->ssl_Stub, SSL_STUB_ADDR(pOldDcpt, pOldDcpt->ssl_StubID), SSL_STUB_SIZE);
                }
            }
            
			// Switch to new stub and return
			pDcpt->ssl_StubID = i;
            sslSyncIf = currIf;
			return true;
		}
	}
	
	// No stub was found to be free
	return false;
	
}

/*********************************************************************
 * Function:        static void SSLStubFree(uint8_t id)
 *
 * PreCondition:    None
 *
 * Inputs:          id      - the stub ID to free
 *
 * Outputs:         None
 *
 * Returns:			None
 *
 * Side Effects:    None
 *
 * Overview:        Specified stub is released
 *
 * Note:            Interface should be sync-ed!
 ********************************************************************/
static void SSLStubFree(uint8_t id)
{
	// If ID is not valid
	if(id >= SSL_MAX_CONNECTIONS)
		return;
	
    SSL_DCPT*     pDcpt = dcptSSL(_TCPIPStackNetIx(sslSyncIf));

	// If currently in RAM, mark as unused
	if(pDcpt->ssl_StubID == id)
		pDcpt->ssl_StubID = SSL_INVALID_ID;

	// Release the stub
	pDcpt->ssl_isStubUsed &= ~(1 << id);
}

/*********************************************************************
 * Function:        static void SSLStubSync(TCPIP_NET_IF* currIf, uint8_t id)
 *
 * PreCondition:    None
 *
 * Input:           currIf - interface for this SSL connection 
 *                  id     - the stub ID to sync to RAM
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Specified stub is loaded to RAM.  Only loads if
 *					necessary, and saves any current stub before
 *					switching.
 *
 * Note:            None
 ********************************************************************/
static void SSLStubSync(TCPIP_NET_IF* currIf, uint8_t id)
{
    SSL_DCPT*     pOldDcpt;
    SSL_DCPT*     pDcpt = dcptSSL(_TCPIPStackNetIx(currIf));
    
	// Check if already loaded
	if(sslSyncIf == currIf && pDcpt->ssl_StubID == id)
    {
		return;     // everything already in sync
    }
    
	// Save old stub
	if(sslSyncIf != 0)
    {   // old interface valid
        pOldDcpt = dcptSSL(_TCPIPStackNetIx(sslSyncIf));
        if(pOldDcpt->ssl_StubID != SSL_INVALID_ID)
        {   // old id valid
		    SSLSave((uint8_t*)&pOldDcpt->ssl_Stub, SSL_STUB_ADDR(pOldDcpt, pOldDcpt->ssl_StubID), SSL_STUB_SIZE);
        }
    }
	
	// Load new stub
	SSLLoad((uint8_t*)&pDcpt->ssl_Stub, SSL_STUB_ADDR(pDcpt, id), SSL_STUB_SIZE);
	pDcpt->ssl_StubID = id;
    sslSyncIf = currIf;
}

/*********************************************************************
 * Function:        static void SSLKeysSync(uint8_t id)
 *
 * PreCondition:    None
 *
 * Input:           id - the key set ID to sync to RAM
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Specified key set is loaded to RAM.  Only loads if
 *					necessary, and saves any current key set before
 *					switching.
 *
 * Note:            Interface should be sync-ed!
 ********************************************************************/
static void SSLKeysSync(uint8_t id)
{
    SSL_DCPT*     pDcpt = dcptSSL(_TCPIPStackNetIx(sslSyncIf));


	// Check if already loaded
	if(pDcpt->ssl_KeysID == id)
		return;
	
	// Save old stub
	if(pDcpt->ssl_KeysID != SSL_INVALID_ID)
		SSLSave((uint8_t*)&pDcpt->ssl_Keys, SSL_KEYS_ADDR(pDcpt, pDcpt->ssl_KeysID), SSL_KEYS_SIZE);
	
	// Load new stub
	SSLLoad((uint8_t*)&pDcpt->ssl_Keys, SSL_KEYS_ADDR(pDcpt, id), SSL_KEYS_SIZE);
	pDcpt->ssl_KeysID = id;
}

/*********************************************************************
 * Function:        static void SSLHashAlloc(uint8_t *id)
 *
 * PreCondition:    None
 *
 * Inputs:          id - Where to store the allocated ID
 *
 * Outputs:         id - Allocated hash ID, or SSL_INVALID_ID if 
 *							none available
 *
 * Side Effects:    None
 *
 * Overview:        Allocates a hash for use.
 *
 * Note:            Interface should be sync-ed!
 ********************************************************************/
static void SSLHashAlloc(uint8_t *id)
{
	uint8_t i, mask;
	
	// If already allocated, just load it up
	if(*id != SSL_INVALID_ID)
	{
		SSLHashSync(*id);
		return;
	}
		
    SSL_DCPT*     pDcpt = dcptSSL(_TCPIPStackNetIx(sslSyncIf));

	// Search for a free hash
	for(i = 0; i != SSL_MAX_HASHES; i++)
	{
        mask = 1 << i;
		if(!(pDcpt->ssl_isHashUsed & mask))
		{// Hash is free, so claim it
			pDcpt->ssl_isHashUsed |= mask;
			
			// Save hash currently in RAM
			if(pDcpt->ssl_HashID != SSL_INVALID_ID)
				SSLSave((uint8_t*)&pDcpt->ssl_Hash, SSL_HASH_ADDR(pDcpt, pDcpt->ssl_HashID), SSL_HASH_SIZE);

			// Switch to new hash and return
			pDcpt->ssl_HashID = i;
			*id = i;
			return;
		}
	}
}

/*********************************************************************
 * Function:        static void SSLHashFree(uint8_t *id)
 *
 * PreCondition:    None
 *
 * Inputs:           id - the hash ID to free
 *
 * Outputs:          id - SSL_INVALID_ID
 *
 * Side Effects:    None
 *
 * Overview:        Specified hash is released
 *
 * Note:            None
 ********************************************************************/
static void SSLHashFree(uint8_t *id)
{
    uint8_t hashId = *id;
  	// Nothing to do for invalid hashes
	if(hashId > SSL_MAX_HASHES)
		return;
	
    SSL_DCPT*     pDcpt = dcptSSL(_TCPIPStackNetIx(sslSyncIf));
	// Purge from RAM if not used
	if(pDcpt->ssl_HashID == hashId)
		pDcpt->ssl_HashID = SSL_INVALID_ID;

	// Release the hash
	pDcpt->ssl_isHashUsed &= ~(1 << hashId);
	*id = SSL_INVALID_ID;
}

/*********************************************************************
 * Function:        static void SSLHashSync(uint8_t id)
 *
 * PreCondition:    None
 *
 * Input:           id - the hash ID to sync to RAM
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Specified hash is loaded to RAM.  Only loads if
 *					necessary, and saves any current hash before
 *					switching.
 *
 * Note:            Interface should be sync-ed!
 ********************************************************************/
static void SSLHashSync(uint8_t id)
{
    SSL_DCPT*     pDcpt = dcptSSL(_TCPIPStackNetIx(sslSyncIf));
    
    // Check if already loaded
	if(pDcpt->ssl_HashID == id)
		return;
	
	// Save old hash
	if(pDcpt->ssl_HashID != SSL_INVALID_ID)
		SSLSave((uint8_t*)&pDcpt->ssl_Hash, SSL_HASH_ADDR(pDcpt, pDcpt->ssl_HashID), SSL_HASH_SIZE);
	
	// Load new hash if not requesting a temporary hash
	if(id != SSL_INVALID_ID)
		SSLLoad((uint8_t*)&pDcpt->ssl_Hash, SSL_HASH_ADDR(pDcpt, id), SSL_HASH_SIZE);

	pDcpt->ssl_HashID = id;
}

/*********************************************************************
 * Function:        static void SSLBufferAlloc(uint8_t *id)
 *
 * PreCondition:    None
 *
 * Input:           id - Where to store the allocated ID
 *
 * Output:          id - Allocated buffer ID, or SSL_INVALID_ID if 
 *							none available
 *
 * Side Effects:    None
 *
 * Overview:        Allocates a buffer for use.
 *
 * Note:            Interface should be sync-ed!
 ********************************************************************/
static void SSLBufferAlloc(uint8_t *id)
{
	uint8_t i, mask;

	// If already allocated, just load it up
	if(*id != SSL_INVALID_ID)
	{
		SSLBufferSync(*id);
		return;
	}
	
    SSL_DCPT*     pDcpt = dcptSSL(_TCPIPStackNetIx(sslSyncIf));
	// Search for a free buffer
	for(i = 0; i != SSL_MAX_BUFFERS; i++)
	{
        mask = 1 << i;
		if(!(pDcpt->ssl_isBufferUsed & mask))
		{// Buffer is free, so claim it
			pDcpt->ssl_isBufferUsed |= mask;
			
			// Save buffer currently in RAM
			if(pDcpt->ssl_BufferID != SSL_INVALID_ID)
            {
                SSLSave((uint8_t*)&pDcpt->ssl_Buffer, SSL_BUFFER_ADDR(pDcpt, pDcpt->ssl_BufferID), SSL_BUFFER_SIZE);
            }

			// Switch to new buffer and return
			pDcpt->ssl_BufferID = i;
			*id = i;
			return;
		}
	}
}

/*********************************************************************
 * Function:        static void SSLBufferFree(uint8_t *id)
 *
 * PreCondition:    None
 *
 * Inputs:           id - the buffer ID to free
 *
 * Outputs:          id - SSL_INVALID_ID
 *
 * Side Effects:    None
 *
 * Overview:        Specified buffer is released
 *
 * Note:            None
 ********************************************************************/
static void SSLBufferFree(uint8_t *id)
{
    uint8_t buffId = *id;
	// Nothing to do for invalid hashes
	if(buffId > SSL_MAX_BUFFERS)
		return;

    SSL_DCPT*     pDcpt = dcptSSL(_TCPIPStackNetIx(sslSyncIf));
	// Purge from RAM if not used
	if(pDcpt->ssl_BufferID == buffId)
		pDcpt->ssl_BufferID = SSL_INVALID_ID;\
		
	// Release the buffer
	pDcpt->ssl_isBufferUsed &= ~(1 << buffId);
	*id = SSL_INVALID_ID;
}

/*********************************************************************
 * Function:        static void SSLBufferSync(uint8_t id)
 *
 * PreCondition:    None
 *
 * Input:           id - the buffer ID to sync to RAM
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Specified buffer is loaded to RAM.  Only loads if
 *					necessary, and saves any current buffer before
 *					switching.
 *
 * Note:            Interface should be sync-ed!
 ********************************************************************/
static void SSLBufferSync(uint8_t id)
{
    
    SSL_DCPT*     pDcpt = dcptSSL(_TCPIPStackNetIx(sslSyncIf));

	// Check if already loaded
	if(pDcpt->ssl_BufferID == id)
		return;
	
	// Save old buffer
	if(pDcpt->ssl_BufferID != SSL_INVALID_ID)
        SSLSave((uint8_t*)&pDcpt->ssl_Buffer, SSL_BUFFER_ADDR(pDcpt, pDcpt->ssl_BufferID), SSL_BUFFER_SIZE);

	// Load new buffer if not requesting temporary space
	if(id != SSL_INVALID_ID)
		SSLLoad((uint8_t*)&pDcpt->ssl_Buffer, SSL_BUFFER_ADDR(pDcpt, id), SSL_BUFFER_SIZE);

	pDcpt->ssl_BufferID = id;
}

/*********************************************************************
 * Function:        static uint8_t SSLSessionNew(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          Allocated Session ID, or SSL_INVALID_ID if none available
 *
 * Side Effects:    None
 *
 * Overview:        Finds space for a new SSL session
 *
 * Note:            Interface should be sync-ed!
 ********************************************************************/
static uint8_t SSLSessionNew(void)
{
	uint8_t id, oldestID;
	SYS_TICK now, age, oldest;
    SSL_DCPT*     pDcpt = dcptSSL(_TCPIPStackNetIx(sslSyncIf));
	
	// Set up the search
	oldestID = SSL_INVALID_ID;
	oldest = SSL_MIN_SESSION_LIFETIME * SYS_TICK_TicksPerSecondGet(); // convert to ticks
	now = SYS_TICK_Get();
		
	// Search for a free session
	for(id = 0; id != SSL_MAX_SESSIONS; id++)
	{
		if(pDcpt->ssl_SessionStubs[id].tag.Val == 0u)
		{// Unused session, so claim immediately
			break;
		}
		
		// Check how old this session is
		age = now - pDcpt->ssl_SessionStubs[id].lastUsed;
		if(age > oldest)
		{// This is now the oldest one
			oldest = age;
			oldestID = id;
		}
	}
	
	// Check if we can claim a session
	if(id == SSL_MAX_SESSIONS && oldestID != SSL_INVALID_ID)
		id = oldestID;
	
	// If a valid ID was found, claim it
	if(id < SSL_MAX_SESSIONS)
	{
		// Save old one if needed
		if(pDcpt->ssl_SessionUpdated)
        {
            SSLSave((uint8_t*)&pDcpt->ssl_Session, SSL_SESSION_ADDR(pDcpt, pDcpt->ssl_SessionID), SSL_SESSION_SIZE);
        }
		// Set up the new session
		pDcpt->ssl_SessionID = id;
		pDcpt->ssl_SessionStubs[id].lastUsed = now;
        pDcpt->ssl_SessionUpdated = true;
        
		return id;
	}
	
	return SSL_INVALID_ID;
}

/*********************************************************************
 * Function:        static uint8_t SSLSessionMatchID(uint8_t* SessionID)
 *
 * PreCondition:    None
 *
 * Input:           SessionID - the session identifier to match
 *
 * Output:          The matched session ID, or SSL_INVALID_ID if not found
 *
 * Side Effects:    None
 *
 * Overview:        Locates a cached SSL session for reuse.  Syncs 
 *                  found session into RAM.
 *
 * Note:            None
 ********************************************************************/
#if defined(TCPIP_STACK_USE_SSL_SERVER)
static uint8_t SSLSessionMatchID(uint8_t* SessionID)
{
	uint8_t i;
    SSL_DCPT*     pDcpt = dcptSSL(_TCPIPStackNetIx(sslSyncIf));
	
	for(i = 0; i < SSL_MAX_SESSIONS; i++)
	{
		// Check if tag matches the ID
		if(pDcpt->ssl_SessionStubs[i].tag.v[0] == 0u &&
			!memcmp((void*)&pDcpt->ssl_SessionStubs[i].tag.v[1], (void*)SessionID, 3) )
		{
			// Found a partial match, so load it to memory
			SSLSessionSync(i);
			
			// Verify complete match
			if(memcmp((void*)pDcpt->ssl_Session.sessionID, (void*)SessionID, 32) != 0)
				continue;
			
			// Mark it as being used now
			pDcpt->ssl_SessionStubs[i].lastUsed = SYS_TICK_Get();
			
			// Return this session for use
			return i;
		}
	}
	
	return SSL_INVALID_ID;

}
#endif

/*********************************************************************
 * Function:        static uint8_t SSLSessionMatchHost(IPV4_ADDR ip)
 *
 * PreCondition:    None
 *
 * Input:           ip - the host session to match
 *
 * Output:          The matched session ID, or SSL_INVALID_ID if not found
 *
 * Side Effects:    None
 *
 * Overview:        Locates a cached SSL session for reuse
 *
 * Note:            None
 ********************************************************************/
#if defined(TCPIP_STACK_USE_SSL_CLIENT)
static uint8_t SSLSessionMatchIP(IPV4_ADDR ip)
{
	uint8_t i;
    SSL_DCPT*     pDcpt = dcptSSL(_TCPIPStackNetIx(sslSyncIf));
               	
	for(i = 0; i < SSL_MAX_SESSIONS; i++)
	{
		// Check if tag matches the IP
		if(!memcmp((void*)&pDcpt->ssl_SessionStubs[i].tag.v[0], (void*)&ip, 4))
		{
			// Found a match, so load it to memory
			SSLSessionSync(i);
			
			// Mark it as being used now
			pDcpt->ssl_SessionStubs[i].lastUsed = SYS_TICK_Get();
			
			// Return this session for use
			return i;
		}
	}
	
	// No match so return invalid
	return SSL_INVALID_ID;
}
#endif

/*********************************************************************
 * Function:        static void SSLSessionSync(uint8_t id)
 *
 * PreCondition:    None
 *
 * Input:           id - the session ID to sync to RAM
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Specified session is loaded to RAM.  Only loads if
 *					necessary, and saves any current session before
 *					switching if it has been updated.
 *
 * Note:            Interface should be sync-ed!
 ********************************************************************/
static void SSLSessionSync(uint8_t id)
{
    SSL_DCPT*     pDcpt = dcptSSL(_TCPIPStackNetIx(sslSyncIf));
    
	// Check if already loaded
	if(pDcpt->ssl_SessionID == id)
		return;
	
	// Save old buffer
	if(pDcpt->ssl_SessionUpdated && pDcpt->ssl_SessionID != SSL_INVALID_ID)
		SSLSave((uint8_t*)&pDcpt->ssl_Session, SSL_SESSION_ADDR(pDcpt, pDcpt->ssl_SessionID), SSL_SESSION_SIZE);
	
	// Load new buffer
	SSLLoad((uint8_t*)&pDcpt->ssl_Session, SSL_SESSION_ADDR(pDcpt, id), SSL_SESSION_SIZE);

	pDcpt->ssl_SessionID = id;
	pDcpt->ssl_SessionUpdated = false;
}

/*********************************************************************
 * Function:        static void SSLSave(uint8_t *srcAdd, uint8_t* dstAddr, uint16_t len)
 *
 * PreCondition:    None
 *
 * Input:           srcAdd - source address in RAM
 *					dstAddr - destination address in RAM
 *					len		- number of bytes to copy
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Copies data from PIC RAM to PIC RAM
 *
 * Note:            None
 ********************************************************************/
static void SSLSave(uint8_t *srcAdd, uint8_t* dstAddr, uint16_t len)
{
    memcpy(dstAddr, srcAdd, len);
}

/*********************************************************************
 * Function:        static void SSLLoad(uint8_t *destAddr,
 *											uint8_t* srcAddr, uint16_t len)
 *
 * PreCondition:    None
 *
 * Input:           destAddr - destination address in RAM
 *					srcAddr - source address in RAM
 *					len		- number of bytes to copy
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Copies data from RAM to RAM
 *
 * Note:            None
 ********************************************************************/
static void SSLLoad(uint8_t *destAddr, uint8_t* srcAddr, uint16_t len)
{
    memcpy(destAddr, srcAddr, len);
}


/****************************************************************************
  ===========================================================================
  Section:
	SSL Handshake Hashing Functions
  ===========================================================================
  ***************************************************************************/

/*********************************************************************
 * Function:        static void HSStart()
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:		Sets up the buffer to store data for handshake
 *					hash tracking
 *
 * Note:            Interface should be sync-ed!
 ********************************************************************/
static void HSStart()
{
    SSL_DCPT*     pDcpt = dcptSSL(_TCPIPStackNetIx(sslSyncIf));
	// Allocate temporary storage and set the pointer to it
	SSLBufferSync(SSL_INVALID_ID);
	pDcpt->ssl_ptrHS = pDcpt->ssl_Buffer.full;
}

/*********************************************************************
 * Function:        static void HSEnd()
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:		Hashes the message contents into the Handshake
 *					hash structures and begins a new handshake hash.
 *
 * Note:            None
 ********************************************************************/
static void HSEnd()
{
	// Hash in the received data and reset the pointer
    SSL_DCPT*     pDcpt = dcptSSL(_TCPIPStackNetIx(sslSyncIf));

	if(pDcpt->ssl_HashID == pDcpt->ssl_Stub.idMD5)
	{
		HashAddData(&pDcpt->ssl_Hash, pDcpt->ssl_Buffer.full, pDcpt->ssl_ptrHS - pDcpt->ssl_Buffer.full);
		SSLHashSync(pDcpt->ssl_Stub.idSHA1);
		HashAddData(&pDcpt->ssl_Hash, pDcpt->ssl_Buffer.full, pDcpt->ssl_ptrHS - pDcpt->ssl_Buffer.full);
	}
	else
	{
		SSLHashSync(pDcpt->ssl_Stub.idSHA1);
		HashAddData(&pDcpt->ssl_Hash, pDcpt->ssl_Buffer.full, pDcpt->ssl_ptrHS - pDcpt->ssl_Buffer.full);
		SSLHashSync(pDcpt->ssl_Stub.idMD5);
		HashAddData(&pDcpt->ssl_Hash, pDcpt->ssl_Buffer.full, pDcpt->ssl_ptrHS - pDcpt->ssl_Buffer.full);
	}
	pDcpt->ssl_ptrHS = pDcpt->ssl_Buffer.full;
}

/*********************************************************************
 * Function:        static uint16_t HSGet(TCP_SOCKET skt, uint8_t *b)
 *
 * PreCondition:    None
 *
 * Input:           skt - socket to read data from
 *					b	- byte to read into
 *
 * Output:          Number of bytes read
 *
 * Side Effects:    None
 *
 * Overview:        Reads data from socket, transparently hashing it
 *					into the handshake hashes.
 *
 * Note:            None
 ********************************************************************/
static uint16_t HSGet(TCP_SOCKET skt, uint8_t *b)
{
	uint8_t c;

	// Must read to &c in case b is NULL (we still need to hash it)
	if(!TCPGet(skt, &c))
		return 0;
	
    SSL_DCPT*     pDcpt = dcptSSL(_TCPIPStackNetIx(sslSyncIf));

	*pDcpt->ssl_ptrHS = c;
	if(b)
		*b = c;
	pDcpt->ssl_ptrHS++;
	
	pDcpt->ssl_Stub.wRxBytesRem--;
	pDcpt->ssl_Stub.wRxHsBytesRem--;
	
	if(pDcpt->ssl_ptrHS > pDcpt->ssl_Buffer.full + 128)
	{
		HSEnd();
		HSStart();
	}

	return 1;
}

/*********************************************************************
 * Function:        static uint16_t HSGetWord(TCP_SOCKET skt, uint16_t *w)
 *
 * PreCondition:    None
 *
 * Input:           skt - socket to read data from
 *					w	- word to read into
 *
 * Output:          Number of bytes read
 *
 * Side Effects:    None
 *
 * Overview:        Reads data from socket, transparently hashing it
 *					into the handshake hashes.
 *
 * Note:            None
 ********************************************************************/
static uint16_t HSGetWord(TCP_SOCKET skt, uint16_t *w)
{
	if(w == NULL)
		return HSGet(skt, (uint8_t*)w) + HSGet(skt, (uint8_t*)w);
	else
		return HSGet(skt, (uint8_t*)w+1) + HSGet(skt, (uint8_t*)w);
}

/*********************************************************************
 * Function:        static uint16_t HSGetArray(TCP_SOCKET skt, 
 											 uint8_t *data, uint16_t len)
 *
 * PreCondition:    None
 *
 * Input:           skt  - socket to read data from
 *					data - array to read into, or NULL
 *					len  - number of bytes to read
 *
 * Output:          Number of bytes read
 *
 * Side Effects:    None
 *
 * Overview:        Reads data from socket, transparently hashing it
 *					into the handshake hashes.
 *
 * Note:            None
 ********************************************************************/
static uint16_t HSGetArray(TCP_SOCKET skt, uint8_t *data, uint16_t len)
{	
	uint16_t w;
    SSL_DCPT*     pDcpt = dcptSSL(_TCPIPStackNetIx(sslSyncIf));

/*	
	uint16_t wLenActual;
	uint16_t wChunkLen;

	wLenActual = 0;
	// Add all of this data to the running hash
	while(len)
	{
		wChunkLen = sizeof(pDcpt->ssl_Buffer.full) - (pDcpt->ssl_ptrHS - pDcpt->ssl_Buffer.full);
		if(wChunkLen == 0u)
		{
			HSEnd();
			wChunkLen = sizeof(pDcpt->ssl_Buffer.full);
		}	
		if(len < wChunkLen)
			wChunkLen = len;
		
		// Obtain data from TCP
		w = TCPGetArray(skt, pDcpt->ssl_ptrHS, wChunkLen);
		if(w == 0u)
			return wLenActual;
		
		// Copy data from hash area to *data area if a non-NULL pointer was 
		// provided
		if(data)
		{
			memcpy((void*)data, pDcpt->ssl_ptrHS, w);
			data += w;
		}
		
		// Update all tracking variables
		pDcpt->ssl_ptrHS += w;
		len -= w;
		wLenActual += w;
		pDcpt->ssl_Stub.wRxBytesRem -= w;
		pDcpt->ssl_Stub.wRxHsBytesRem -= w;
	}

	return wLenActual;
*/

	//if reading to NULL, we still have to hash
	if(!data)
	{
		uint16_t i, rem;
		for(i = 0; i < len; )
		{
			// Determine how much we can read
			rem = (pDcpt->ssl_Buffer.full + sizeof(pDcpt->ssl_Buffer.full) - 1) - pDcpt->ssl_ptrHS;
			if(rem > len - i)
				rem = len - i;

			// Read that much
			rem = TCPGetArray(skt, pDcpt->ssl_ptrHS, rem);
			pDcpt->ssl_Stub.wRxBytesRem -= rem;
			pDcpt->ssl_Stub.wRxHsBytesRem -= rem;
						
			// Hash what we've got
			pDcpt->ssl_ptrHS += rem;
			HSEnd();
			HSStart();
			
			i += rem;
			
			// Make sure we aren't in an infinite loop
			if(rem == 0u)
				break;
		}
		
		return i;
	}
	
	len = TCPGetArray(skt, data, len);
	w = len;
	
	memcpy(pDcpt->ssl_ptrHS, (void*)data, len);
	pDcpt->ssl_ptrHS += len;

	
//	// Add all of this data to the running hash
//	while(w)
//	{
//		uint16_t wChunkLen = sizeof(pDcpt->ssl_Buffer.full) - (pDcpt->ssl_ptrHS - pDcpt->ssl_Buffer.full);
//		if(wChunkLen == 0u)
//		{
//			HSEnd();
//			HSStart();
//			wChunkLen = sizeof(pDcpt->ssl_Buffer.full);
//		}	
//		if(w < wChunkLen)
//			wChunkLen = w;
//		memcpy(pDcpt->ssl_ptrHS, (void*)data, wChunkLen);
//		pDcpt->ssl_ptrHS += wChunkLen;
//		w -= wChunkLen;
//	}
	
	pDcpt->ssl_Stub.wRxBytesRem -= len;
	pDcpt->ssl_Stub.wRxHsBytesRem -= len;
	
	if(pDcpt->ssl_ptrHS > pDcpt->ssl_Buffer.full + (sizeof(pDcpt->ssl_Buffer.full)/2))
	{
		HSEnd();
		HSStart();
	}
	
	return len;
}

/*********************************************************************
 * Function:        static uint16_t HSPut(TCP_SOCKET skt, uint8_t b)
 *
 * PreCondition:    None
 *
 * Input:           skt  - socket to write data to
 *					b    - byte to write
 *
 * Output:          Number of bytes written
 *
 * Side Effects:    None
 *
 * Overview:        Writes data to socket, transparently hashing it
 *					into the handshake hashes.
 *
 * Note:            None
 ********************************************************************/
static uint16_t HSPut(TCP_SOCKET skt, uint8_t b)
{

	if(!TCPPut(skt, b))
		return 0;

    SSL_DCPT*     pDcpt = dcptSSL(_TCPIPStackNetIx(sslSyncIf));
    
	// Ensure we don't overflow the Hash buffer	
	if(sizeof(pDcpt->ssl_Buffer.full) - (pDcpt->ssl_ptrHS - pDcpt->ssl_Buffer.full) == 0u)
		HSEnd();
	
	// Add this byte of data to the running hash
	*pDcpt->ssl_ptrHS = b;
	pDcpt->ssl_ptrHS++;

	return 1;
}

/*********************************************************************
 * Function:        static uint16_t HSPutWord(TCP_SOCKET skt, uint16_t w)
 *
 * PreCondition:    None
 *
 * Input:           skt  - socket to write data to
 *					w    - word to write
 *
 * Output:          Number of bytes written
 *
 * Side Effects:    None
 *
 * Overview:        Writes data to socket, transparently hashing it
 *					into the handshake hashes.
 *
 * Note:            None
 ********************************************************************/
static uint16_t HSPutWord(TCP_SOCKET skt, uint16_t w)
{
	return HSPut(skt, (uint8_t)(w>>8)) + HSPut(skt, (uint8_t)w);
}

/*********************************************************************
 * Function:        static uint16_t HSPutArray(TCP_SOCKET skt,
 											uint8_t *data, uint8_t len)
 *
 * PreCondition:    None
 *
 * Input:           skt  - socket to write data to
 *					data - data to write
 *					len  - number of bytes to write
 *
 * Output:          Number of bytes written
 *
 * Side Effects:    None
 *
 * Overview:        Writes data to socket, transparently hashing it
 *					into the handshake hashes.
 *
 * Note:            None
 ********************************************************************/
static uint16_t HSPutArray(TCP_SOCKET skt, const uint8_t *data, uint16_t len)
{	
	uint16_t w;
    SSL_DCPT*     pDcpt = dcptSSL(_TCPIPStackNetIx(sslSyncIf));
	
	len = TCPPutArray(skt, data, len);
	w = len;

	// Add all of this data to the running hash
	while(w)
	{
		uint16_t wChunkLen = sizeof(pDcpt->ssl_Buffer.full) - (pDcpt->ssl_ptrHS - pDcpt->ssl_Buffer.full);
		if(wChunkLen == 0u)
		{
			HSEnd();
			wChunkLen = sizeof(pDcpt->ssl_Buffer.full);
		}	
		if(w < wChunkLen)
			wChunkLen = w;
		memcpy(pDcpt->ssl_ptrHS, (void*)data, wChunkLen);
		data += wChunkLen;
		pDcpt->ssl_ptrHS += wChunkLen;
		w -= wChunkLen;
	}
	
	return len;
}

/****************************************************************************
  ===========================================================================
  Section:
	SSL MAC Hashing Functions
  ===========================================================================
  ***************************************************************************/

/*********************************************************************
 * Function:        void SSLMACBegin(uint8_t *MACSecret, uint32_t seq, 
 *											uint8_t protocol, uint16_t len)
 *
 * PreCondition:    pDcpt->ssl_Hash is ready to be written
 *					(any pending data saved, nothing useful stored)
 *
 * Input:           MACSecret - the MAC write secret
 *					seq       - the sequence number for this message
 *					protocol  - the SSL_PROTOCOL for this message
 *					len       - the length of the message being MAC'd
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:		Begins a MAC calculation in pDcpt->ssl_Hash
 *
 * Note:            None
 ********************************************************************/
void SSLMACBegin(uint8_t *MACSecret, uint32_t seq, uint8_t protocol, uint16_t len)
{
	uint8_t i, temp[7];
    SSL_DCPT*     pDcpt = dcptSSL(_TCPIPStackNetIx(sslSyncIf));

	MD5Initialize(&pDcpt->ssl_Hash);

	// Form the temp array
	temp[0] = *((uint8_t*)&seq+3);
	temp[1] = *((uint8_t*)&seq+2);
	temp[2] = *((uint8_t*)&seq+1);
	temp[3] = *((uint8_t*)&seq+0);
	temp[4] = protocol;
	temp[5] = *((uint8_t*)&len+1);
	temp[6] = *((uint8_t*)&len+0);
		
	// Hash the initial data (secret, padding, seq, protcol, len)
	HashAddData(&pDcpt->ssl_Hash, MACSecret, 16);
	
	// Add in the padding
	for(i = 0; i < 6u; i++)
	{
		HashAddData(&pDcpt->ssl_Hash, (const uint8_t*)"\x36\x36\x36\x36\x36\x36\x36\x36", 8);
	}

	// Hash in the data length
	HashAddData(&pDcpt->ssl_Hash, (const uint8_t*)"\0\0\0\0", 4);
	HashAddData(&pDcpt->ssl_Hash, temp, 7);	
}

/*********************************************************************
 * Function:        void SSLMACAdd(uint8_t *data, uint16_t len)
 *
 * PreCondition:    pDcpt->ssl_Hash is ready to be written
 *					(any pending data saved, SSLMACBegin called)
 *
 * Input:           data - the data to add to the MAC
 *					len  - the length of data
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:		Adds more data to a MAC in progress
 *
 * Note:            None
 ********************************************************************/
void SSLMACAdd(uint8_t *data, uint16_t len)
{
    SSL_DCPT*     pDcpt = dcptSSL(_TCPIPStackNetIx(sslSyncIf));
	HashAddData(&pDcpt->ssl_Hash, data, len);
}

/*********************************************************************
 * Function:        static void SSLMACCalc(uint8_t *result)
 *
 * PreCondition:    pDcpt->ssl_Hash is ready to be written
 *					(any pending data saved, SSLMACBegin called)
 *
 * Input:           MACSecret - the MAC write secret
 *					result    - a 16 byte buffer to store result
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:		Finishes the MAC calculation, and places the
 *					result in the *result array
 *
 * Note:            None
 ********************************************************************/
void SSLMACCalc(uint8_t *MACSecret, uint8_t *result)
{
	uint8_t i;
    SSL_DCPT*     pDcpt = dcptSSL(_TCPIPStackNetIx(sslSyncIf));
	
	// Get inner hash result
	MD5Calculate(&pDcpt->ssl_Hash, result);
	
	// Begin outer hash
	MD5Initialize(&pDcpt->ssl_Hash);
	HashAddData(&pDcpt->ssl_Hash, MACSecret, 16);
	
	// Add in padding
	for(i = 0; i < 6u; i++)
	{
		HashAddData(&pDcpt->ssl_Hash, (const uint8_t*)"\x5c\x5c\x5c\x5c\x5c\x5c\x5c\x5c", 8);
	}
	
	// Hash in the previous result and calculate
	HashAddData(&pDcpt->ssl_Hash, result, 16);
	MD5Calculate(&pDcpt->ssl_Hash, result);	
}

uint8_t* SSLGetBuffer(int netIx)
{
    SSL_DCPT*     pDcpt = dcptSSL(netIx);
    return (uint8_t*)&pDcpt->ssl_Buffer;
}

#endif
