/*******************************************************************************
  Berekely Socket Distribution API Header File

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  berkeley_api.h 
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

#ifndef _BERKELEY_API_HEADER_FILE
#define _BERKELEY_API_HEADER_FILE

typedef int16_t SOCKET;   //Socket descriptor

#define AF_INET         2			// Internet Address Family - UDP, TCP, etc.

#define IP_ADDR_ANY     0u			// IP Address for server binding
#define INADDR_ANY      0x00000000u	// IP address for server binding.


#define SOCK_STREAM 100  //Connection based byte streams. Use TCP for the internet address family.
#define SOCK_DGRAM  110  //Connectionless datagram socket. Use UDP for the internet address family.
 
#define IPPROTO_IP      0   // Indicates IP pseudo-protocol.
#define IPPROTO_TCP     6   // Indicates TCP for the internet address family.
#define IPPROTO_UDP     17  // Indicates UDP for the internet address family.

#define SOCKET_ERROR            (-1) //Socket error
#define SOCKET_CNXN_IN_PROGRESS (-2) //Socket connection state.
#define SOCKET_DISCONNECTED     (-3) //Socket disconnected


#define INVALID_TCP_PORT   (0L)  //Invalid TCP port

struct in_addr
{
    union
   {
       struct { uint8_t s_b1,s_b2,s_b3,s_b4; } S_un_b; // IP address in Byte
       struct { uint16_t s_w1,s_w2; } S_un_w; //IP address in Word
       uint32_t S_addr; //IP address
   }S_un; //union of IP address
    
#define s_addr  S_un.S_addr //can be used for most tcp & ip code
#define s_host  S_un.S_un_b.s_b2 //host on imp
#define s_net   S_un.S_un_b.s_b1 // network
#define s_imp   S_un.S_un_w.s_w2 // imp
#define s_impno S_un.S_un_b.s_b4 // imp number
#define s_lh    S_un.S_un_b.s_b3 // logical host
}; // in_addr structure

struct __attribute__((__packed__)) sockaddr
{
    unsigned short   sa_family;   //address family
    char    sa_data[14];       //up to 14 bytes of direct address
}; //generic address structure for all address families

struct __attribute__((__packed__)) sockaddr_in
{
    short   sin_family; //Address family; must be AF_INET.
    uint16_t    sin_port;  //Internet Protocol (IP) port.
    struct  in_addr sin_addr; //IP address in network byte order.
    char    sin_zero[8];  //Padding to make structure the same size as SOCKADDR. 
}; //In the Internet address family

typedef struct sockaddr_in SOCKADDR_IN; //In the Internet address family
typedef struct sockaddr SOCKADDR;  // generic address structure for all address families

/*
 * Berkeley API module configuration structure
 */

typedef struct
{
} BERKELEY_MODULE_GONFIG;

SOCKET  socket( int af, int type, int protocol );
int     bind( SOCKET s, const struct sockaddr* name, int namelen );
int     listen( SOCKET s, int backlog );
SOCKET  accept( SOCKET s, struct sockaddr* addr, int* addrlen );
int     connect( SOCKET s, struct sockaddr* name, int namelen );
int     send( SOCKET s, const char* buf, int len, int flags );
int     sendto( SOCKET s, const char* buf, int len, int flags, const struct sockaddr* to, int tolen );
int     recv( SOCKET s, char* buf, int len, int flags );
int     recvfrom( SOCKET s, char* buf, int len, int flags, struct sockaddr* from, int* fromlen );
int     gethostname(char* name, int namelen);
int     closesocket( SOCKET s );

#endif  // _BERKELEY_API_HEADER_FILE


