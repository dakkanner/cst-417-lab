/*******************************************************************************
  RSA Public Key Encryption stack private API

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  rsa_manager.h 
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

#ifndef __RSA_MANAGER_H_
#define __RSA_MANAGER_H_

#define RSA_KEY_WORDS	(SSL_RSA_KEY_SIZE/BIGINT_DATA_SIZE)		// Represents the number of words in a key
#define RSA_PRIME_WORDS	(SSL_RSA_KEY_SIZE/BIGINT_DATA_SIZE/2)	// Represents the number of words in an RSA prime

/****************************************************************************
  Section:
	State Machines and Status Codes
  ***************************************************************************/

// State machine for RSA processes
typedef enum
{
	SM_RSA_IDLE = 0u,		// Data is being initialized by the application
	SM_RSA_ENCRYPT_START,	// Initial state for encryption processes; encryption is ready to begin
	SM_RSA_ENCRYPT,			// RSA encryption is proceeding
	SM_RSA_DECRYPT_START,	// Initial state for decryption processes; decryption is ready to begin
	SM_RSA_DECRYPT_FIND_M1, // First stage in the CRT decryption algorithm
	SM_RSA_DECRYPT_FIND_M2,	// Second stage in the CRT decryption algorithm
	SM_RSA_DECRYPT_FINISH,	// CRT values have been calculated, so finalize the operation
	SM_RSA_DONE				// RSA process is complete
} SM_RSA;



/****************************************************************************
  Section:
	Function Prototypes
  ***************************************************************************/

bool RSAInit(const TCPIP_STACK_MODULE_CTRL* const stackData, const RSA_MODULE_GONFIG* const rsaData);

void RSADeInit(const TCPIP_STACK_MODULE_CTRL* const stackData);



#endif  // __RSA_MANAGER_H_


