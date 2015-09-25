/*******************************************************************************
  RSA Algorithm Public Key Decryption Library

  Summary:
    Library for Microchip TCP/IP Stack
    
  Description:
    - Provides RSA private key decryption capabilities
    - Reference: PKCS #1
*******************************************************************************/

/*******************************************************************************
FileName:   RSA.c
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

#define __RSA_C

#include "tcpip_private.h"

#if (defined(TCPIP_STACK_USE_SSL_SERVER) || defined(TCPIP_STACK_USE_SSL_CLIENT)) // && !defined(TCPIP_IF_ENCX24J600)

#define TCPIP_THIS_MODULE_ID    TCPIP_MODULE_RSA

/****************************************************************************
  Section:
	Global RSA Variables
  ***************************************************************************/
typedef struct
{
    RSA_DATA_FORMAT outputFormat;	// Final output format for calculated data
    uint16_t keyLength;				// Length of the input key
    BIGINT X;						// BigInt to hold X
    BIGINT tmp;						// BigInt to hold temporary values

#if defined(TCPIP_STACK_USE_RSA_ENCRYPT)
    BIGINT E;			// BigInt to hold the exponent value E
    BIGINT N;			// BigInt to hold the modulus value N
    BIGINT Y;			// BigInt to hold the result of Y = X^E % N

    uint8_t rsaData[SSL_RSA_CLIENT_SIZE/8];	// Temporary space to store PKCS formatted data for encryption
    uint8_t eData[4];		// Temporary space to store E for encryption


#endif

#if defined(TCPIP_STACK_USE_RSA_DECRYPT)
    BIGINT P;		// BigInt to hold RSA prime P
    BIGINT Q;		// BigInt to hold RSA prime Q
    BIGINT dP;		// BigInt to hold CRT exponent dP
    BIGINT dQ;		// BigInt to hold CRT exponent dQ
    BIGINT qInv;	// BigInt to hold inverted Q value for CRT

#endif

    uint8_t rsaTemp[SSL_RSA_CLIENT_SIZE/4];	// Temporary data storage space for encryption

    SM_RSA rsaState[0]; 					// State machine variable
}RSA_DCPT;      // RSA module descriptor


static RSA_DCPT*    rsaDcpt = 0;    // the one and only RSA descriptor

static int          rsaInitCount = 0;      // RSA module initialization count

#if SSL_MULTIPLE_INTERFACES
    #define smRSA(netIx)        (rsaDcpt->rsaState[netIx])
#else
    #define smRSA(netIx)        (rsaDcpt->rsaState[0])
#endif  // SSL_MULTIPLE_INTERFACES


#if defined(TCPIP_STACK_USE_RSA_ENCRYPT)
	static bool _RSAModExp(BIGINT *y, BIGINT *x, BIGINT *e, BIGINT *n);
	
#endif

#if defined(TCPIP_STACK_USE_RSA_DECRYPT)
	extern const BIGINT_DATA_TYPE SSL_P[RSA_PRIME_WORDS], SSL_Q[RSA_PRIME_WORDS];
	extern const BIGINT_DATA_TYPE SSL_dP[RSA_PRIME_WORDS], SSL_dQ[RSA_PRIME_WORDS];
	extern const BIGINT_DATA_TYPE SSL_qInv[RSA_PRIME_WORDS];
	
    static bool _RSAModExp(BIGINT* y, BIGINT* x, BIGINT* e, BIGINT* n);
    #define _RSAModExpROM(y,x,e,n)	_RSAModExp(y,x,e,n)
	
#endif


/****************************************************************************
  Section:
	Function Implementations
  ***************************************************************************/

/*****************************************************************************
  Function:
	void RSAInit(void)

  Summary:
	Initializes the RSA engine

  Description:
	Call this function once at boot to configure the memories for RSA
	key storage and temporary processing space.

  Precondition:
	None

  Parameters:
	None

  Returns:
  	None
  	
  Remarks:
	This function is called only one during lifetime of the application.
  ***************************************************************************/
bool RSAInit(const TCPIP_STACK_MODULE_CTRL* const stackCtrl, const RSA_MODULE_GONFIG* const rsaConfig)
{

    if(stackCtrl->stackAction == TCPIP_STACK_ACTION_IF_UP)
    {   // interface restart
        return true;
    }

    // (stackCtrl->stackAction == TCPIP_STACK_ACTION_INIT)   // stack init
    
    if(rsaInitCount == 0)
    {   // first time we're run
#if SSL_MULTIPLE_INTERFACES
        rsaDcpt = (RSA_DCPT*)TCPIP_HEAP_Malloc(stackCtrl->memH, sizeof(*rsaDcpt) + stackCtrl->nIfs * sizeof(*rsaDcpt->rsaState));
#else
        rsaDcpt = (RSA_DCPT*)TCPIP_HEAP_Malloc(stackCtrl->memH, sizeof(*rsaDcpt) + sizeof(*rsaDcpt->rsaState));
#endif

        if(rsaDcpt == 0)
        {   // failed
            return false;
        }

        // initialize the data structures
#if defined(TCPIP_STACK_USE_RSA_DECRYPT)
        BigInt(&rsaDcpt->P, (BIGINT_DATA_TYPE*)SSL_P, RSA_PRIME_WORDS);
        BigInt(&rsaDcpt->Q, (BIGINT_DATA_TYPE*)SSL_Q, RSA_PRIME_WORDS);
        BigInt(&rsaDcpt->dP, (BIGINT_DATA_TYPE*)SSL_dP, RSA_PRIME_WORDS);
        BigInt(&rsaDcpt->dQ, (BIGINT_DATA_TYPE*)SSL_dQ, RSA_PRIME_WORDS);
        BigInt(&rsaDcpt->qInv, (BIGINT_DATA_TYPE*)SSL_qInv, RSA_PRIME_WORDS);
#endif

#if defined(TCPIP_STACK_USE_RSA_ENCRYPT)
        BigInt(&rsaDcpt->E, (BIGINT_DATA_TYPE*)rsaDcpt->eData, sizeof(rsaDcpt->eData)/sizeof(BIGINT_DATA_TYPE));
#endif

        BigInt(&rsaDcpt->tmp, (BIGINT_DATA_TYPE*)rsaDcpt->rsaTemp, sizeof(rsaDcpt->rsaTemp)/sizeof(BIGINT_DATA_TYPE));
    }

    // interface initialization
	smRSA(stackCtrl->netIx) = SM_RSA_IDLE;

    rsaInitCount++;
	return true;
}

/*****************************************************************************
  Function:
	void RSADeInit(void)

  Summary:
	DeInitializes the RSA engine

  Description:
	Call this function once at shutdown to free memories used for RSA
	key storage and temporary processing space.

  Precondition:
	None

  Parameters:
	None

  Returns:
  	None
  	
  Remarks:
	This function is called only one during lifetime of the application.
  ***************************************************************************/
void RSADeInit(const TCPIP_STACK_MODULE_CTRL* const stackCtrl)
{
    // if(stackCtrl->stackAction == TCPIP_STACK_ACTION_DEINIT) // stack shut down
    // if(stackCtrl->stackAction == TCPIP_STACK_ACTION_IF_DOWN) // interface down

    // one way or another this interface is going down
	smRSA(stackCtrl->netIx) = SM_RSA_IDLE;

    if(stackCtrl->stackAction == TCPIP_STACK_ACTION_DEINIT)
    {   // whole stack is going down
        if(rsaInitCount > 0)
        {   // we're up and running
            if(--rsaInitCount == 0)
            {   // all closed
                // release resources
                TCPIP_HEAP_Free(stackCtrl->memH, rsaDcpt);
                rsaDcpt = 0;
            }
        }
    }
}

/*****************************************************************************
  Function:
	bool RSABeginUsage(TCPIP_NET_HANDLE netH, RSA_OP op, uint8_t vKeyByteLen)

  Summary:
	Requests control of the RSA engine.

  Description:
	This function acts as a semaphore to gain control of the RSA engine.
	Call this function and ensure that it returns true before using
	any other RSA APIs.  When the RSA module is no longer needed, call
	RSAEndUsage to release the module back to the application.
	
	This function should not be called directly.  Instead, its macros
	RSABeginEncrypt and RSABeginDecrypt should be used to ensure that the
	specified option is enabled.

  Precondition:
	RSA has already been initialized.

  Parameters:
    netH - interface to work on
	op - one of the RSA_OP constants indicating encryption or decryption
	vKeyByteLen - For encryption, the length of the RSA key in bytes.  This
	              value is ignored otherwise.

  Return Values:
  	true - No RSA operation is in progress and the calling application has
  		   successfully taken ownership of the RSA module.
  	false - The RSA module is currently being used, so the calling 
  			application must wait.
  ***************************************************************************/
bool RSABeginUsage(TCPIP_NET_HANDLE netH, RSA_OP op, uint16_t vKeyByteLen)
{
    TCPIP_NET_IF * pIf =  _TCPIPStackHandleToNet(netH);
    if(pIf)
    {
#if SSL_MULTIPLE_INTERFACES
        int netIx = _TCPIPStackNetIx(pIf);
#endif  // SSL_MULTIPLE_INTERFACES

        if(smRSA(netIx) != SM_RSA_IDLE)
            return false;

        // Set up the state machine
        if(op == RSA_OP_ENCRYPT)
            smRSA(netIx) = SM_RSA_ENCRYPT_START;
        else if(op == RSA_OP_DECRYPT)
            smRSA(netIx) = SM_RSA_DECRYPT_START;
        else
            return false;

        rsaDcpt->keyLength = vKeyByteLen;

        return true;
    }

    return false;
}

/*****************************************************************************
  Function:
	void RSAEndUsage(TCPIP_NET_HANDLE netH)

  Summary:
	Releases control of the RSA engine.

  Description:
	This function acts as a semaphore to release control of the RSA engine.
	Call this function when your application is finished with the RSA
	module so that other applications may use it.
	
	This function should not be called directly.  Instead, its macros
	RSAEndEncrypt and RSAEndDecrypt should be used to ensure that the
	specified option is enabled.

  Precondition:
	RSA has already been initialized and RSABeginUsage has returned true.

  Parameters:
	pIf - network interface to work on

  Return Values:
  	None
  ***************************************************************************/
void RSAEndUsage(TCPIP_NET_HANDLE netH)
{
    TCPIP_NET_IF * pIf =  _TCPIPStackHandleToNet(netH);
    if(pIf)
    {
#if SSL_MULTIPLE_INTERFACES
        int netIx = _TCPIPStackNetIx(pIf);
#endif  // SSL_MULTIPLE_INTERFACES
        smRSA(netIx) = SM_RSA_IDLE;
    }
}

/*****************************************************************************
  Function:
	void RSASetData(TCPIP_NET_HANDLE netH, uint8_t* data, uint8_t len, RSA_DATA_FORMAT format)

  Summary:
	Indicates the data to be encrypted or decrypted.

  Description:
	Call this function to indicate what data is to be encrypted or decrypted.
	This function ensures that the data is PKCS #1 padded for encryption
	operations, and also normalizes the data to little-endian format
	so that it will be compatible with the BigInt libraries.

  Precondition:
	RSA has already been initialized and RSABeginUsage has returned true.

  Parameters:
	pIf - network interface to work on
	data - The data to be encrypted or decrypted
	len - The length of data
	format - One of the RSA_DATA_FORMAT constants indicating the endian-ness
			 of the input data.

  Return Values:
  	None
  
  Remarks:
  	For decryption operations, the calculation is done in place.  Thererore, 
  	the endian-ness of the input data may be modified by this function.  
  	Encryption operations may expand the input, so separate memory is 
  	allocated for the operation in that case.
  ***************************************************************************/
void RSASetData(TCPIP_NET_HANDLE netH, uint8_t* data, uint16_t len, RSA_DATA_FORMAT format)
{
    TCPIP_NET_IF * pIf =  _TCPIPStackHandleToNet(netH);
    if(pIf == 0)
    {
        return;
    }

#if SSL_MULTIPLE_INTERFACES
    int netIx = _TCPIPStackNetIx(pIf);
#endif  // SSL_MULTIPLE_INTERFACES

	#if defined(TCPIP_STACK_USE_RSA_ENCRYPT)
	if(smRSA(netIx) == SM_RSA_ENCRYPT_START)
	{
		// Initialize the BigInt wrappers
		BigInt(&rsaDcpt->X, (BIGINT_DATA_TYPE*)rsaDcpt->rsaData, len/sizeof(BIGINT_DATA_TYPE));
		
		// Copy in the data
		memcpy((void*)rsaDcpt->rsaData, (void*)data, len);
	
		// For big-endian, swap the data
		if(format == RSA_BIG_ENDIAN)
			BigIntSwapEndianness(&rsaDcpt->X);
		
		// Resize the big int to full size
		BigInt(&rsaDcpt->X, (BIGINT_DATA_TYPE*)rsaDcpt->rsaData, rsaDcpt->keyLength/sizeof(BIGINT_DATA_TYPE));
		
		// Pad the input data according to PKCS #1 Block 2
		if(len < rsaDcpt->keyLength-4)
		{
			rsaDcpt->rsaData[len++] = 0x00;
			while(len < rsaDcpt->keyLength-2)
			{
				do
				{
					rsaDcpt->rsaData[len] = SYS_RANDOM_GET();
				} while(rsaDcpt->rsaData[len] == 0x00u);
				len++;
			}
			rsaDcpt->rsaData[len++] = 0x02;

		}
	}
	#endif
	
	#if defined(TCPIP_STACK_USE_RSA_DECRYPT)
	if(smRSA(netIx) == SM_RSA_DECRYPT_START)
	{
		BigInt(&rsaDcpt->X, (BIGINT_DATA_TYPE*)data, len/sizeof(BIGINT_DATA_TYPE));
		
		// Correct and save endianness
		rsaDcpt->outputFormat = format;
		if(rsaDcpt->outputFormat == RSA_BIG_ENDIAN)
			BigIntSwapEndianness(&rsaDcpt->X);
	}
	#endif
}

/*****************************************************************************
  Function:
	void RSASetResult(uint8_t* data, RSA_DATA_FORMAT format)

  Summary:
	Indicates where the result should be stored.

  Description:
	Call this function to indicate where the result of an encryption 
	operation should be stored, and in what format.  The array indicated by 
	data should be the same size as the key being used for encryption.

  Precondition:
	RSA has already been initialized and RSABeginUsage has returned true.

  Parameters:
	data - Where to store the encryption result
	format - One of the RSA_DATA_FORMAT constants indicating the endian-ness
			 of the output data.

  Return Values:
  	None
  
  Remarks:
	Decryption shrinks the data, and the encrypted version is rarely needed
	after the operation, so decryption happens in place.  Therefore, this 
	function is not valid for decryption operations.
  ***************************************************************************/
#if defined(TCPIP_STACK_USE_RSA_ENCRYPT)
void RSASetResult(uint8_t* data, RSA_DATA_FORMAT format)
{	
	rsaDcpt->outputFormat = format;
	BigInt(&rsaDcpt->Y, (BIGINT_DATA_TYPE*)data, rsaDcpt->keyLength/sizeof(BIGINT_DATA_TYPE));
}
#endif

/*****************************************************************************
  Function:
	void RSASetE(uint8_t *data, uint8_t len, RSA_DATA_FORMAT format)

  Description:
	Sets the exponent for an encryption operation.

  Precondition:
	RSA has already been initialized and RSABeginUsage has returned true.

  Parameters:
	data - The exponent to use
	len - The length of the exponent
	format - One of the RSA_DATA_FORMAT constants indicating the endian-ness
			 of the exponent data

  Return Values:
  	None
  ***************************************************************************/
#if defined(TCPIP_STACK_USE_RSA_ENCRYPT)
void RSASetE(uint8_t* data, uint8_t len, RSA_DATA_FORMAT format)
{
	uint8_t size;
	
	BigIntZero(&rsaDcpt->E);
	size = (rsaDcpt->E.ptrMSBMax - rsaDcpt->E.ptrLSB + 1) * sizeof(BIGINT_DATA_TYPE);
	
	// Make sure we don't copy too much
	if(len > size)
		len = size;
	
	// For little-endian data, copy as is to the front
	if(format == RSA_LITTLE_ENDIAN)
	{
		memcpy((void*)rsaDcpt->eData, (void*)data, len);
	}

	// For big-endian data, copy to end, then swap
	else
	{
		memcpy((void*)&rsaDcpt->eData[size - len], (void*)data, len);
		BigIntSwapEndianness(&rsaDcpt->E);
	}
		
	rsaDcpt->E.bMSBValid = 0;

}
#endif

/*****************************************************************************
  Function:
	void RSASetN(uint8_t* data, RSA_DATA_FORMAT format)

  Description:
	Sets the modulus for an encryption operation.

  Precondition:
	RSA has already been initialized and RSABeginUsage has returned true.

  Parameters:
	data - The modulus to use
	format - One of the RSA_DATA_FORMAT constants indicating the endian-ness
			 of the exponent data

  Return Values:
  	None
  
  Remarks:
  	The modulus length must be identical to vKeyLenBytes as set
  	when the module was allocated by RSABeginEncrypt.
  ***************************************************************************/
#if defined(TCPIP_STACK_USE_RSA_ENCRYPT)
void RSASetN(uint8_t* data, RSA_DATA_FORMAT format)
{
	BigInt(&rsaDcpt->N, (BIGINT_DATA_TYPE*)data, rsaDcpt->keyLength/sizeof(BIGINT_DATA_TYPE));
	
	if(format == RSA_BIG_ENDIAN)
		BigIntSwapEndianness(&rsaDcpt->N);
}
#endif

/*****************************************************************************
  Function:
	RSA_STATUS RSAStep(TCPIP_NET_HANDLE netH)

  Summary:
	Performs non-blocking RSA calculations.

  Description:
	Call this function to process a portion of the pending RSA operation
	until RSA_DONE is returned.  This function performs small pieces of 
	work each time it is called, so it must be called repeatedly in order
	to complete an operation.  Performing small pieces at a time and then
	yielding to the main application allows long calculations to be
	performed in a non-blocking manner that fits with the co-operative
	multi-tasking model of the stack.
	
	Status updates are periodically returned.  For lengthy decryption
	operations, this helps prevent client time-outs by allowing the
	application to periodically transmit a few bytes if necessary.

  Precondition:
	RSA has already been initialized and RSABeginUsage has returned true.

  Parameters:
	pIf - network interface to work on

  Return Values:
  	RSA_WORKING - Calculation is in progress; no status update available.
  	RSA_FINISHED_M1 - This call has completed the decryption calculation of
  						the M1 CRT value.
  	RSA_FINISHED_M2 - This call has completed the decryption calculation of
  						the M2 CRT value.
  	RSA_DONE - The RSA operation is complete.
  ***************************************************************************/
RSA_STATUS RSAStep(TCPIP_NET_HANDLE netH)
{
    TCPIP_NET_IF * pIf =  _TCPIPStackHandleToNet(netH);
    if(pIf == 0)
    {
        return RSA_WORKING; 
    }

	#if defined(TCPIP_STACK_USE_RSA_DECRYPT)
	static BIGINT m1, m2;
    uint8_t*    sslBuffer;
	#endif
    int netIx = _TCPIPStackNetIx(pIf);
	
	switch(smRSA(netIx))
	{
		#if defined(TCPIP_STACK_USE_RSA_ENCRYPT)
		case SM_RSA_ENCRYPT_START:
			// Actually nothing to do here anymore
			smRSA(netIx) = SM_RSA_ENCRYPT;
			
		case SM_RSA_ENCRYPT:
			// Call ModExp until complete
			if(_RSAModExp(&rsaDcpt->Y, &rsaDcpt->X, &rsaDcpt->E, &rsaDcpt->N))
			{// Calculation is finished
				// Swap endian-ness if needed
				if(rsaDcpt->outputFormat == RSA_BIG_ENDIAN)
					BigIntSwapEndianness(&rsaDcpt->Y);
	
				// Indicate that we're finished
				smRSA(netIx) = SM_RSA_DONE;
				return RSA_DONE;
			}
			break;
		#endif
		
		#if defined(TCPIP_STACK_USE_RSA_DECRYPT)
               // Server is decripting a message (exlp Client Key Exchange) using its private key
		case SM_RSA_DECRYPT_START:
			// Set up the RSA Decryption memories
                sslBuffer = SSLGetBuffer(netIx);
			BigInt(&m1, (BIGINT_DATA_TYPE*)(sslBuffer+(SSL_RSA_KEY_SIZE/8)), RSA_PRIME_WORDS);
			BigInt(&m2, (BIGINT_DATA_TYPE*)(sslBuffer+(3*(SSL_RSA_KEY_SIZE/16))), RSA_PRIME_WORDS);
			
			smRSA(netIx) = SM_RSA_DECRYPT_FIND_M1;
			break;
		
		case SM_RSA_DECRYPT_FIND_M1:
			// m1 = c^rsaDcpt->dP % p
			if(_RSAModExpROM(&m1, &rsaDcpt->X, &rsaDcpt->dP, &rsaDcpt->P))
			{
				smRSA(netIx) = SM_RSA_DECRYPT_FIND_M2;
				return RSA_FINISHED_M1;
			}
			break;
	
		case SM_RSA_DECRYPT_FIND_M2:
			// m2 = c^rsaDcpt->dQ % q
			if(_RSAModExpROM(&m2, &rsaDcpt->X, &rsaDcpt->dQ, &rsaDcpt->Q))
			{
				smRSA(netIx) = SM_RSA_DECRYPT_FINISH;
				return RSA_FINISHED_M2;
			}
			break;
	
		case SM_RSA_DECRYPT_FINISH:
			// Almost done...finalize the CRT math
			
			if(BigIntCompare(&m1, &m2) > 0)
			{// if(m1 > m2)
				// m1 = m1 - m2
				BigIntSubtract(&m1, &m2);
			
				// rsaDcpt->tmp = m1 * rsaDcpt->qInv
				BigIntMultiply(&m1, &rsaDcpt->qInv, &rsaDcpt->tmp);
			
				// m1 = rsaDcpt->tmp % p
				BigIntMod(&rsaDcpt->tmp, &rsaDcpt->P);
				BigIntCopy(&m1, &rsaDcpt->tmp);
			}
			else 
			{// m1 < m2
				// rsaDcpt->tmp = m2
				BigIntCopy(&rsaDcpt->tmp, &m2);
		
				// m2 = m2 - m1
				BigIntSubtract(&m2, &m1);
				
				// m1 = rsaDcpt->tmp
				BigIntCopy(&m1, &rsaDcpt->tmp);
				
				// rsaDcpt->tmp = m2 * rsaDcpt->qInv
				BigIntMultiply(&m2, &rsaDcpt->qInv, &rsaDcpt->tmp);
				
				// m2 = m1
				BigIntCopy(&m2, &m1);
		
				// rsaDcpt->tmp = rsaDcpt->tmp % p
				BigIntMod(&rsaDcpt->tmp, &rsaDcpt->P);
		
				// m1 = rsaDcpt->P
				BigIntCopy(&m1, &rsaDcpt->P);
					
				// m1 = m1 - rsaDcpt->tmp;
				BigIntSubtract(&m1, &rsaDcpt->tmp);
			}

			// msg = m1 * q
			BigIntMultiply(&m1, &rsaDcpt->Q, &rsaDcpt->tmp);

			// rsaDcpt->tmp = m2 + rsaDcpt->tmp
			BigIntAdd(&rsaDcpt->tmp, &m2);	

			// Copy the decrypted value back to rsaDcpt->X
			BigIntCopy(&rsaDcpt->X, &rsaDcpt->tmp);

			// Swap endian-ness if needed
			if(rsaDcpt->outputFormat == RSA_BIG_ENDIAN)
				BigIntSwapEndianness(&rsaDcpt->X);

			// Indicate that we're finished
			smRSA(netIx) = SM_RSA_DONE;
			return RSA_DONE;
		#endif
		
		default:
			// Unknown state
			return RSA_DONE;
	}
	
	// Indicate that we're still working
	return RSA_WORKING;
	
}


/****************************************************************************
  Section:
	Fundamental RSA Operations
  ***************************************************************************/



/*****************************************************************************
  Function:
	static bool _RSAModExp(BIGINT* y, BIGINT* x, BIGINT* e, BIGINT* n)

  Summary:
	Performs a the base RSA operation y = x^e % n

  Description:
	This function solves y = x^e % n, the fundamental RSA calculation.
	Eight bits are processed with each call, allowing the function to 
	operate in a co-operative multi-tasking environment.

  Precondition:
	RSA has already been initialized and RSABeginUsage has returned true.

  Parameters:
	y - where the result should be stored
	x - the message value
	e - the exponent
	n - the modulus

  Return Values:
  	true - the operation is complete
  	false - more bits remain to be processed
  
  Remarks:
  	This function is not required on 8-bit platforms that do not need
  	encryption support.
  ***************************************************************************/
#if defined(TCPIP_STACK_USE_RSA_ENCRYPT) || defined(TCPIP_STACK_USE_RSA_DECRYPT)
static bool _RSAModExp(BIGINT* y, BIGINT* x, BIGINT* e, BIGINT* n)
{
	static uint8_t *pe = NULL, *pend = NULL;
	static uint8_t startBit;

	// This macro processes a single bit, either with or without debug output.
	// The C preprocessor will optimize this without the overhead of a function call.
	#if RSAEXP_DEBUG
		#define doBit(a)	SYS_MESSAGE("\r\n\r\n  * y = ");	\
							BigIntPrint(y); \
							BigIntSquare(y, &rsaDcpt->tmp); \
							SYS_MESSAGE("\r\nnow t = "); \
							BigIntPrint(&rsaDcpt->tmp); \
							SYS_MESSAGE("\r\n  % n = "); \
							BigIntPrint(n); \
							BigIntMod(&rsaDcpt->tmp, n); \
							BigIntCopy(y, &rsaDcpt->tmp); \
							SYS_MESSAGE("\r\nnow y = "); \
							BigIntPrint(y); \
							if(*pe & a) \
							{ \
								SYS_MESSAGE("\r\n\r\n!!* x = "); \
								BigIntPrint(x); \
								BigIntMultiply(y, x, &rsaDcpt->tmp); \
								SYS_MESSAGE("\r\nnow t = "); \
								BigIntPrint(&rsaDcpt->tmp); \
								SYS_MESSAGE("\r\n  % n = "); \
								BigIntPrint(n); \
								BigIntMod(&rsaDcpt->tmp, n); \
								BigIntCopy(y, &rsaDcpt->tmp); \
								SYS_MESSAGE("\r\nnow y = "); \
								BigIntPrint(y); \
							}
	#else
		#define doBit(a)	BigIntSquare(y, &rsaDcpt->tmp); \
							BigIntMod(&rsaDcpt->tmp, n); \
							BigIntCopy(y, &rsaDcpt->tmp); \
							if(*pe & a) \
							{ \
								BigIntMultiply(y, x, &rsaDcpt->tmp); \
								BigIntMod(&rsaDcpt->tmp, n); \
								BigIntCopy(y, &rsaDcpt->tmp); \
							}
	#endif

	// Determine if this is a new computation
	if(pe == pend)
	{// Yes, so set up the calculation
		
		// Set up *pe to point to the MSB in e, *pend to stop byte
		pe = (uint8_t*)e->ptrMSBMax + (sizeof(BIGINT_DATA_TYPE) - 1);
		pend = ((uint8_t*)e->ptrLSB) - 1;
		while(*pe == 0x00u)
		{
			pe--;
		
			// Handle special case where e is zero and n >= 2 (result y should be 1).  
			// If n = 1, then y should be zero, but this really special case isn't 
			// normally useful, so we shall not implement it and will return 1 
			// instead.
			if(pe == pend)
			{
				BigIntZero(y);
				*(y->ptrLSB) = 0x01;
				return true;
			}
		}

		// Start at the bit following the MSbit
		startBit = *pe;
		if(startBit & 0x80)
			startBit = 0x40;
		else if(startBit & 0x40)
			startBit = 0x20;
		else if(startBit & 0x20)
			startBit = 0x10;
		else if(startBit & 0x10)
			startBit = 0x08;
		else if(startBit & 0x08)
			startBit = 0x04;
		else if(startBit & 0x04)
			startBit = 0x02;
		else if(startBit & 0x02)
			startBit = 0x01;
		else
		{
			pe--;
			startBit = 0x80;
		}
		
		// Process that second bit now to save memory in rsaDcpt->Y
		// (first round squares the message (rsaDcpt->X).  Subsequent rounds square rsaDcpt->Y,
		//   which is at most half that size when running the CRT.)
		BigIntSquare(x, &rsaDcpt->tmp);
		BigIntMod(&rsaDcpt->tmp, n);
		BigIntCopy(y, &rsaDcpt->tmp);
		if(*pe & startBit)
		{// If bit is '1'
			BigIntMultiply(y, x, &rsaDcpt->tmp);
			BigIntMod(&rsaDcpt->tmp, n);
			BigIntCopy(y, &rsaDcpt->tmp);
		}
		
		// Move to the next bit as the startBit
		startBit >>= 1;
		if(!startBit)
		{
			pe--;
			startBit = 0x80;
		}

		// We are finished if e has only one or two set bits total, ex: e = 3
		if(pe == pend)
			return true;
	}
	
	// Process remaining bits in current byte
	switch(startBit)
	{
		case 0x80:
			doBit(0x80);
			startBit >>= 1;
		case 0x40:
			doBit(0x40);
			startBit >>= 1;
			break;
		case 0x20:
			doBit(0x20);
			startBit >>= 1;
		case 0x10:
			doBit(0x10);
			startBit >>= 1;
			break;
		case 0x08:
			doBit(0x08);
			startBit >>= 1;
		case 0x04:
			doBit(0x04);
			startBit >>= 1;
			break;
		case 0x02:
			doBit(0x02);
			startBit >>= 1;
		case 0x01:
			doBit(0x01);
			startBit = 0x80;
			pe--;
			if(pe == pend)
				return true;
	}

	return false;

}
#endif


#endif	//#if (defined(TCPIP_STACK_USE_SSL_SERVER) || defined(TCPIP_STACK_USE_SSL_CLIENT)) && !defined(ENC100_INTERFACE_MODE)
