/*******************************************************************************
  SSLv3 Module Headers

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  SSL.h 
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

#ifndef __SSL_H
#define __SSL_H

// SSL layer configuration/initialization
typedef struct
{
} SSL_MODULE_CONFIG;

/****************************************************************************
  Section:
	Function Prototypes
  ***************************************************************************/

// *****************************************************************************
/*
  Function:
	bool TCPStartSSLClient(TCP_SOCKET hTCP, uint8_t* host)

  Summary:
	Begins an SSL client session.

  Description:
	This function escalates the current connection to an SSL secured 
	connection by initiating an SSL client handshake.

  Precondition:
	TCP is initialized and hTCP is already connected.

  Parameters:
	hTCP		- TCP connection to secure
	host		- Expected host name on certificate (currently ignored)

  Return Values:
	true 		- an SSL connection was initiated
	false 		- Insufficient SSL resources (stubs) were available

  Remarks:
	The host parameter is currently ignored and is not validated.
  */	
bool                TCPStartSSLClient(TCP_SOCKET hTCP, uint8_t* host);

// *****************************************************************************
/*
  Function:
	bool TCPStartSSLClientEx(TCP_SOCKET hTCP, uint8_t* host, void * buffer, uint8_t suppDataType)

  Summary:
	Begins an SSL client session.

  Description:
	This function escalates the current connection to an SSL secured 
	connection by initiating an SSL client handshake.

  Precondition:
	TCP is initialized and hTCP is already connected.

  Parameters:
	hTCP			- TCP connection to secure
	host			- Expected host name on certificate (currently ignored)
	buffer      	- Buffer for supplementary data return
	suppDataType 	- Type of supplementary data to copy

  Return Values:
	true 		- an SSL connection was initiated
	false 		- Insufficient SSL resources (stubs) were available

  Remarks:
	The host parameter is currently ignored and is not validated.
  */	
bool                TCPStartSSLClientEx(TCP_SOCKET hTCP, uint8_t* host, void * buffer, uint8_t suppDataType);

// *****************************************************************************
/*
  Function:
	bool TCPStartSSLServer(TCP_SOCKET hTCP)

  Summary:
	Begins an SSL server session.

  Description:
	This function sets up an SSL server session when a new connection is
	established on an SSL port.

  Precondition:
	TCP is initialized and hTCP is already connected.

  Parameters:
	hTCP		- TCP connection to secure

  Return Values:
	true		- an SSL connection was initiated
	false		- Insufficient SSL resources (stubs) were available
  */	
bool                TCPStartSSLServer(TCP_SOCKET hTCP);

// *****************************************************************************
/*
  Function:
	bool TCPAddSSLListener(TCP_SOCKET hTCP, uint16_t port)

  Summary:
	Listens for SSL connection on a specific port.

  Description:
	This function adds an additional listening port to a TCP connection.  
	Connections made on this alternate port will be secured via SSL.

  Precondition:
	TCP is initialized and hTCP is listening.

  Parameters:
	hTCP		- TCP connection to secure
	port		- SSL port to listen on

  Return Values:
	true		- SSL port was added.
	false		- The socket was not a listening socket.
  */	
bool                TCPAddSSLListener(TCP_SOCKET hTCP, uint16_t port);


// *****************************************************************************
/*
  Function:
	bool TCPSSLIsHandshaking(TCP_SOCKET hTCP)

  Summary:
	Determines if an SSL session is still handshaking.

  Description:
	Call this function after calling TCPStartSSLClient until false is
	returned.  Then your application may continue with its normal data
	transfer (which is now secured).
	
  Precondition:
	TCP is initialized and hTCP is connected.

  Parameters:
	hTCP		- TCP connection to check

  Return Values:
	true		- SSL handshake is still progressing
	false		- SSL handshake has completed
  */	
bool                TCPSSLIsHandshaking(TCP_SOCKET hTCP);

// *****************************************************************************
/*
  Function:
	bool TCPIsSSL(TCP_SOCKET hTCP)

  Summary:
	Determines if a TCP connection is secured with SSL.

  Description:
	Call this function to determine whether or not a TCP connection is 
	secured with SSL.
	
  Precondition:
	TCP is initialized and hTCP is connected.

  Parameters:
	hTCP		- TCP connection to check

  Return Values:
	true		- Connection is secured via SSL
	false		- Connection is not secured
  */	
bool                TCPIsSSL(TCP_SOCKET hTCP);

// *****************************************************************************
/*
	void TCPSSLHandshakeComplete(TCP_SOCKET hTCP)

  Summary:
	Clears the SSL handshake flag.

  Description:
	This function clears the flag indicating that an SSL handshake is
	complete.
	
  Precondition:
	TCP is initialized and hTCP is connected.

  Parameters:
	hTCP		- TCP connection to set

  Returns:
	None

  Remarks:
	This function should never be called by an application.  It is used 
	only by the SSL module itself.
  */	
void                TCPSSLHandshakeComplete(TCP_SOCKET hTCP);



#endif
