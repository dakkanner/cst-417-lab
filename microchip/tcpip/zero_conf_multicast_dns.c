/*******************************************************************************
  Zero Configuration (Zeroconf) Multicast DNS and
  Service Discovery Module for Microchip TCP/IP Stack

  Summary:

  Description: The advantage of mDNS  DNS is that mDNS servers
               listen on a standardized multicast IP address of 224.0.0.251 for
               IPv4 and ff02::fb for IPv6 link-local addressing. Regular DNS
               can listen to any address that is assigned to them and as such
               can be difficult to find them.
*******************************************************************************/

/*******************************************************************************
FileName:   zero_conf_multicast_dns.c
Copyright � 2012 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED ?AS IS? WITHOUT WARRANTY OF ANY KIND,
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

#include <ctype.h>

#include "tcpip_private.h"

#if defined(TCPIP_STACK_USE_ZEROCONF_LINK_LOCAL) && defined(TCPIP_STACK_USE_ZEROCONF_MDNS_SD)

#define TCPIP_THIS_MODULE_ID    TCPIP_MODULE_MDNS

#include "zero_conf_link_local_private.h"
#include "zero_conf_helper.h"


extern void DisplayIPValue(IPV4_ADDR IPVal);

#define MDNS_PORT            5353
#define MAX_HOST_NAME_SIZE   32      //31+'\0'  Max Host name size
#define MAX_LABEL_SIZE       64      //63+'\0'  Maximum size allowed for a label. RFC 1035 (2.3.4) == 63
#define MAX_RR_NAME_SIZE     256   //255+'\0' Max Resource Recd Name size. RFC 1035 (2.3.4) == 255
#define MAX_SRV_TYPE_SIZE    32      //31+'\0'  eg. "_http._tcp.local". Max could be 255, but is an overkill.
#define MAX_SRV_NAME_SIZE    64      //63+'\0'  eg. "My Web server". Max could be 255, but is an overkill.
#define MAX_TXT_DATA_SIZE    128   //127+'\0' eg. "path=/index.htm"
#define RESOURCE_RECORD_TTL_VAL     3600 // Time-To-Live for a Resource-Record in seconds.

#define MAX_RR_NUM  4            // for A, PTR, SRV, and TXT  Max No.of Resource-Records/Service

/* Constants from mdns.txt (IETF Draft)*/
#define MDNS_PROBE_WAIT             750 // msecs  (initial random delay)
#define MDNS_PROBE_INTERVAL         250 // msecs (maximum delay till repeated probe)
#define MDNS_PROBE_NUM                3 //      (number of probe packets)
#define MDNS_MAX_PROBE_CONFLICT_NUM  30 // max num of conflicts before we insist and move on to announce ...
#define MDNS_ANNOUNCE_NUM             3 //      (number of announcement packets)
#define MDNS_ANNOUNCE_INTERVAL      250 // msecs (time between announcement packets)
#define MDNS_ANNOUNCE_WAIT          250 // msecs (delay before announcing)

/* Resource-Record Types from RFC-1035 */
/*
All RRs have the same top level format shown below:

  0  1  2  3  4 5  6  7  8  9  10 11 12 13 14 15
+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
|                                               |
/                                               /
/                    NAME                       /
|                                               |
+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
|                    TYPE                       |
+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
|                    CLASS                      |
+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
|                     TTL                       |
|                                               |
+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
|                    RDLENGTH                   |
+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--|
/                     RDATA                     /
/                                               /
+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
 */
typedef enum {
    QTYPE_A = 1,      //QUERY TYPE response = Address
    QTYPE_NS = 2,     //QUERY TYPE response = Authorative Name Server
    QTYPE_CNAME = 5,  //the canonical domain name for an alias
    QTYPE_PTR = 12,   // a domain name pointer
    QTYPE_TXT = 16,   // text strings
    QTYPE_SRV = 33,
    QTYPE_ANY = 255,
}MDNS_QTYPE;

/* Indexes in Resource-record list */
#define QTYPE_A_INDEX   0
#define QTYPE_PTR_INDEX 1
#define QTYPE_SRV_INDEX 2
#define QTYPE_TXT_INDEX 3

/* MDNS Message Fomrat, which is common
 * for Queries and Resource-Records. Taken
 * from RFC 1035
 */
/* MDNS Message Header Flags */
typedef union _MDNS_MSG_HEADER_FLAGS {

    struct {
      uint8_t        rcode:4;
      uint8_t        z:3;
        uint8_t        ra:1;
      uint8_t        rd:1;
      uint8_t        tc:1;
      uint8_t        aa:1;
      uint8_t        opcode:4;
        uint8_t        qr:1;
    }bits;
    uint16_t Val;
   uint8_t v[2];
} MDNS_MSG_HEADER_FLAGS;

/* MDNS Message-Header Format */
typedef struct _MDNS_MSG_HEADER
{
   TCPIP_UINT16_VAL query_id;
   MDNS_MSG_HEADER_FLAGS flags;
   TCPIP_UINT16_VAL nQuestions;
   TCPIP_UINT16_VAL nAnswers;
   TCPIP_UINT16_VAL nAuthoritativeRecords;
   TCPIP_UINT16_VAL nAdditionalRecords;
} MDNS_MSG_HEADER;

/* DNS-Query Format, which is prepended by
 * DNS-MESSAGE Header defined above */
struct question
{
    unsigned char *name;
    unsigned short int type, class;
};

/* DNS-Resource Record Format, which is
 * prepended by DNS-MESSAGE Header
 * defined above. This definition includes
 * all resource-record data formats, to have
 * small-memory foot print */

struct _mDNSProcessCtx_sd;// mdnsd_struct
struct _mDNSProcessCtx_common;

typedef struct _mDNSResourceRecord
{
    uint8_t            *name;
    TCPIP_UINT16_VAL   type;
    TCPIP_UINT16_VAL   class;
    TCPIP_UINT32_VAL   ttl;
    TCPIP_UINT16_VAL   rdlength;

   union {
      IPV4_ADDR ip;      // for A record

      struct {
         TCPIP_UINT16_VAL priority;
         TCPIP_UINT16_VAL weight;
         TCPIP_UINT16_VAL port;
      } srv;         // for SRV record
   };

   // DO NOT merge this into the union.
   uint8_t *rdata;      // for PTR, SRV and TXT records.

    /* House-Keeping Stuff */

   // pointer to the header Ctx of the process that "owns" this resource record.
   struct _mDNSProcessCtx_common *pOwnerCtx;

    uint8_t valid; /* indicates whether rr is valid */
   bool bNameAndTypeMatched;
   bool bResponseRequested;
   bool bResponseSuppressed;
} mDNSResourceRecord;

/* DNS-SD Specific Data-Structures */

typedef enum _MDNS_STATE
{
   MDNS_STATE_HOME = 0,
    MDNS_STATE_INTF_NOT_CONNECTED,
    MDNS_STATE_IPADDR_NOT_CONFIGURED,
   MDNS_STATE_NOT_READY,
   MDNS_STATE_INIT,
   MDNS_STATE_PROBE,
   MDNS_STATE_ANNOUNCE,
   MDNS_STATE_DEFEND,
} MDNS_STATE;

typedef enum _MDNS_RR_GROUP
{
   MDNS_RR_GROUP_QD, // Quuery count
   MDNS_RR_GROUP_AN, // Answer count
   MDNS_RR_GROUP_NS, // Authority record count
   MDNS_RR_GROUP_AR  // Addition Record Count
} MDNS_RR_GROUP;

typedef struct _mDNSResponderCtx
{
   mDNSResourceRecord   rr_list[MAX_RR_NUM];   // Our resource records.

   bool                 bLastMsgIsIncomplete;   // Last DNS msg was truncated
   TCPIP_UINT16_VAL     query_id;            // mDNS Query transaction ID
   IPV4_ADDR            prev_ipaddr;         // To keep track of changes in IP-addr
} mDNSResponderCtx;

typedef enum _MDNS_CTX_TYPE
{
   MDNS_CTX_TYPE_HOST = 0,
   MDNS_CTX_TYPE_SD
} MDNS_CTX_TYPE;

typedef struct _mDNSProcessCtx_common
{
   MDNS_CTX_TYPE   type;      // Is owner mDNS ("HOST") or mDNS-SD ("SD")?
   MDNS_STATE      state;      // PROBE, ANNOUNCE, DEFEND, ...

   uint8_t nProbeCount;
   uint8_t nProbeConflictCount;
    uint8_t nClaimCount;
    bool bProbeConflictSeen;
    bool bLateConflictSeen;

   bool bConflictSeenInLastProbe;
   uint8_t nInstanceId;

   SYS_TICK event_time;   // Internal Timer, to keep track of events
   uint8_t time_recorded; // Flag to indicate event_time is loaded
   SYS_TICK random_delay;

} mDNSProcessCtx_common;

typedef struct _mDNSProcessCtx_host
{
   mDNSProcessCtx_common common;

   mDNSResponderCtx *pResponderCtx;

   // other host name related info

   uint8_t szUserChosenHostName[MAX_HOST_NAME_SIZE];   // user chosen host name
   uint8_t szHostName[MAX_HOST_NAME_SIZE];               // mDNS chosen Host-Name

} mDNSProcessCtx_host;

typedef struct _mDNSProcessCtx_sd
{
   mDNSProcessCtx_common common;

   mDNSResponderCtx *pResponderCtx;

   // info specific to SD
    uint8_t srv_name[MAX_SRV_NAME_SIZE];
    uint8_t srv_type[MAX_SRV_TYPE_SIZE];
    uint8_t sd_qualified_name[MAX_RR_NAME_SIZE];
    uint8_t used; /* Spinlock to protect Race-cond. */

    uint8_t sd_auto_rename: 1,        /* Flag to show auto-Rename is enabled */
         sd_service_advertised: 1, /* Flag to show whether service is advertised */
       service_registered: 1;    /* Flag to indicate that user has registered this service */

    uint16_t sd_port; /* Port number in Local-sys where Service is being offered */
    uint8_t sd_txt_rec[MAX_TXT_DATA_SIZE];
    uint8_t sd_txt_rec_len;

    void (*sd_call_back)(char *, MDNSD_ERR_CODE , void *);
    void *sd_context;

} mDNSProcessCtx_sd;




/* DNS-SD State-Machine */



/* Multicast-DNS States defintion */

/************** Global Declarations ***************/
/* Remote Node info, which is Multicast-Node
 * whose IP-address is 224.0.0.251 & MAC-Address
 * is 01:00:5E:00:00:FB. Multicast-IP address for
 * mDNS is specified by mdns.txt (IETF). IP is
 * translated into Multicast-MAC address according
 * rules specified in Std.
 */


                         // mDNS Server/Client (Responder/Qurier)


/* Global declaration to support Message-Compression
 * defined in RFC-1035, Section 4.1.4 */



////////////////////////////////////
typedef enum {
      MDNS_RESPONDER_INIT,
      MDNS_RESPONDER_LISTEN
}MDNS_RESPONDER_TYPE;

typedef struct
{
    TCPIP_NET_IF*          mTcpIpNetIf;
    mDNSProcessCtx_host    mHostCtx;
    mDNSProcessCtx_sd      mSDCtx;
    mDNSResponderCtx       mResponderCtx;
    char                   CONST_STR_local[9];
    NODE_INFO              mDNSRemote;
    UDP_SOCKET             mDNS_socket;
    uint16_t               mDNS_offset;
    uint32_t               mDNS_responder_state;
} DNSDesc_t;



/* Forward declarations */
static void mDNSSetAddresses(DNSDesc_t *pDNSdesc);
static void mDNSResponder(DNSDesc_t *pDNSdesc);
static uint16_t mDNSDeCompress(uint16_t wPos
                                   ,uint8_t *pcString
                                   ,bool bFollowPtr
                                   ,uint8_t cElement
                                   ,uint8_t cDepth
                                   ,DNSDesc_t *pDNSdesc);
static size_t mDNSSDFormatServiceInstance(uint8_t *string, size_t strSize );
static MDNSD_ERR_CODE mDNSHostRegister( char *host_name,DNSDesc_t *pDNSdesc);
static void mDNSFillHostRecord(DNSDesc_t *pDNSdesc);
static void mDNSSDFillResRecords(mDNSProcessCtx_sd *sd,DNSDesc_t *pDNSdesc);
static void mDNSAnnounce(mDNSResourceRecord *pRR, DNSDesc_t *pDNSdesc);
static void mDNSProcessInternal(mDNSProcessCtx_common *pCtx, DNSDesc_t *pDNSdesc);

static DNSDesc_t *gDNSdesc;
static int  mDNSInitCount = 0;      // mDNS module initialization count

/////////////////////////////////////////




/************* Local String Functions ******************/



/***************************************************************
  Function:
   static uint8_t strcmp_local_ignore_case(uint8_t *string_1, uint8_t *string_2)

  Summary:
   Compares two strings by ignoring the case.

  Parameters:
   string_1 & string_2 - Two strings

  Returns:
    Zero: If two strings are equal.
    Non-Zero: If both strings are not equal or on error case
  **************************************************************/
static uint8_t strcmp_local_ignore_case(uint8_t *str_1, uint8_t *str_2)
{
    if(str_1 == NULL || str_2 == NULL)
    {
        WARN_MDNS_PRINT("strmcmp_local_ignore_case: String is NULL \r\n");
        return -1;
    }

    while(*str_1 && *str_2){
        if(*str_1 == *str_2 || (*str_1-32) == *str_2 ||
         *str_1 == (*str_2-32))
      {
         str_1++;
         str_2++;
            continue;
      }
      else
         return 1;

    }
    if(*str_1 == '\0' && *str_2 == '\0')
        return 0;
    else
        return 1;

}


static void
mDNSResetCounters(mDNSProcessCtx_common *pHeader, bool bResetProbeConflictCount)
{
   if (bResetProbeConflictCount)
   {
        pHeader->nProbeConflictCount = 0;
   }

   pHeader->nProbeCount = 0;
   pHeader->nClaimCount = 0;
   pHeader->bLateConflictSeen = false;
   pHeader->bProbeConflictSeen = false;
}

/***************************************************************
  Function:
   static void mDNSRename(uint8_t *str, uint8_t max_len)

  Summary:
   Renames a string with a numerical-extension.

  Description:
   This function is to rename host-name/Resource-Record Name,
    when a name-conflict is detected on local-network.
    For-Ex: "myhost" is chosen name and a conflict is detected
    this function renames as "myhost-2". Also ensures that string
    is properly formatted.

  Precondition:
   None

  Parameters:
   String - the string to be Renamed with Numerical-Extenstion.
    max_len - Maximum Length allowed for String

  Returns:
     None
  **************************************************************/
// strLabel:  the user registered name.
//            E.g., "Web Server", for service name (srv_name), or
//                 "My Host", for host name (taken from MY_DEFAULT_HOST_NAME)
// nLabelId:  instance number, to avoid conflict in the name space.
// strBase:   the base name for the appropriate name space.
//            E.g., "_http._tcp.local" for service name, or
//                 "local" for host name.
// strTarget: where the newly constructed fully-qualified-name will be stored.
// nMaxLen:   max length for the newly constructed label, which is the first portion of the
//            fully-qualified-name
//
// ("Web Server", 3, "_http._tcp.local", strTarget, 63) =>
//     stores "Web Server-3._http._tcp.local" to *strTarget.
// ("MyHost", 2, "local", strTarget, 63) =>
//     stores "MyHost-2.local" to *strTarget
//
static void mDNSRename(uint8_t *strLabel, uint8_t nLabelId, uint8_t *strBase, uint8_t *strTarget, uint8_t nMaxLen)
{
    size_t  targetLen;
   uint8_t n = nLabelId;
#define mDNSRename_ID_LEN 6
   uint8_t str_n[mDNSRename_ID_LEN]; //enough for "-255." + '\0'.
   uint8_t i = mDNSRename_ID_LEN - 1 ;

   str_n[i--] = 0;
   str_n[i--] = '.';

   // construct str_n from n
   while (i != 0)
   {
      str_n[i--] = '0'+ n%10;
      if (n < 10) break;
      n = n/10;
   }
   str_n[i] = '-';

    targetLen = strncpy_m((char*)strTarget, nMaxLen, 3, strLabel, &(str_n[i]), strBase);

#ifdef MDNS_WARN
   if ( targetLen == nMaxLen )
   {
      MDNS_WARN("mDNSRename: label too long - truncated\r\n");
   }
#endif

}

/***************************************************************
  Function:
   static void mDNSPutString(uint8_t* String)

  Summary:
   Writes a string to the Multicast-DNS socket.

  Description:
   This function writes a string to the Multicast-DNS socket,
    ensuring that it is properly formatted.

  Precondition:
   UDP socket is obtained and ready for writing.

  Parameters:
   String - the string to write to the UDP socket.

  Returns:
     None
  **************************************************************/
static void mDNSPutString(uint8_t* string, DNSDesc_t * pDNSdesc)
{
   uint8_t *right_ptr,*label_ptr;
   uint8_t label[MAX_LABEL_SIZE];
   uint8_t i;
   uint8_t len;

   right_ptr = string;

   while(1)
   {
        label_ptr = label;
        len = 0;
        while(*right_ptr)
        {
            i = *right_ptr;

            if(i == '.' || i == '/' ||
               i == ',' || i == '>' || i == '\\')
            {
             /* Formatted Serv-Instance will have '\.'
                * instead of just '.' */
                if(i == '\\')
                {
                    right_ptr++;
                }
                else
                    break;
            }
            *label_ptr++ = *right_ptr;
            len++;
            right_ptr++;
        }
        i = *right_ptr++;

      // Put the length and data
      // Also, skip over the '.' in the input string
      UDPPut(pDNSdesc->mDNS_socket, len);
      UDPPutArray(pDNSdesc->mDNS_socket, label, len);
      string =  right_ptr;

      if(i == 0x00u || i == '/' || i == ',' || i == '>')
         break;
   }

   // Put the string null terminator character
   UDPPut(pDNSdesc->mDNS_socket, 0x00);
}

/***************************************************************
  Function:
   static bool mDNSSendQuery(uint8_t* name, uint8_t record_type)

  Summary:
   Sends out a Multicast-DNS-Query to Multicast-Address
    through pDNSdesc->mDNS_socket.

  Description:
   This function is used in Probing-phase to check the
    uniqueness of chosen Host-Name/ Resoruce-Record-Name.
    Selected Name and Type of Query are put into
    Multicast UDP socket.

  Precondition:
   UDP socket (pDNSdesc->mDNS_socket) is obtained and ready for writing.

  Parameters:
   name - Chosen Host-Name/Resource-Record-Name, checking
           for uniqueness.
    type - Type of Query

  Returns:
     true - On Success
    false - On Failure (If UDP-Socket is invalid)
  **************************************************************/
//static bool mDNSSendQuery(uint8_t* name, uint8_t record_type, uint8_t cFlush, uint8_t probe_type)

/***************************************************************
  Function:
   static void mDNSProbe(uint8_t *name, MDNS_QTYPE q_type)

  Summary:
   Sends out Multicast-DNS probe packet with Host-name

  Description:
   This function is used to send out mDNS-probe packet for
    checking uniqueness of selected host-name. This function makes
    use of mDNSSendQuery to send out DNS-Query with chosen host-name
    to Multicast-Address.

    If any other machine is using same host-name, it responds with
    a reply and this host has to select different name.

  Precondition:
   None

  Parameters:
 *
 *

  Returns:
     None
  **************************************************************/
static bool mDNSProbe(mDNSProcessCtx_common *pCtx, DNSDesc_t *pDNSdesc)
{
   MDNS_MSG_HEADER mDNS_header;

    // Abort operation if no UDP sockets are available
   // If this ever happens, incrementing UDP_MAX_SOCKETS in
   // udp_config.h may help (at the expense of more global memory
   // resources).

   if(pDNSdesc->mDNS_socket == INVALID_UDP_SOCKET)
    {
        WARN_MDNS_PRINT("mDNSProbe: Opening UDP Socket Failed \r\n");
      return false;
    }

   // Make certain the socket can be written to
   //if(!UDPIsPutReady(pDNSdesc->mDNS_socket))
    if(!UDPIsTxPutReady(pDNSdesc->mDNS_socket,256)) //JBS
    {
        WARN_MDNS_PRINT("mDNSProbe: UDP Socket TX Busy \r\n");
      return false;
    }

    // Put DNS query here
   pDNSdesc->mResponderCtx.query_id.Val++;

   mDNS_header.query_id.Val = TCPIP_HELPER_htons(pDNSdesc->mResponderCtx.query_id.Val);   // User chosen transaction ID
   mDNS_header.flags.Val = 0;                              // Standard query with recursion
   mDNS_header.nQuestions.Val = TCPIP_HELPER_htons(((uint16_t)1u));               // 1 entry in the question section
   mDNS_header.nAnswers.Val = 0;                           // 0 entry in the answer section
   mDNS_header.nAuthoritativeRecords.Val = TCPIP_HELPER_htons(((uint16_t)1u));      // 1 entry in name server section
   mDNS_header.nAdditionalRecords.Val = 0;                     // 0 entry in additional records section

   // Put out the mDNS message header
   UDPPutArray(pDNSdesc->mDNS_socket, (uint8_t *) &mDNS_header, sizeof(MDNS_MSG_HEADER));

   // Start of the QD section
   switch (pCtx->type)
   {
   case MDNS_CTX_TYPE_HOST:
      mDNSPutString(pDNSdesc->mResponderCtx.rr_list[QTYPE_A_INDEX].name,pDNSdesc);
      break;

   case MDNS_CTX_TYPE_SD:
     // pDNSdesc->mResponderCtx.rr_list[QTYPE_SRV_INDEX].srv.port.Val=pDNSdesc->mSDCtx.sd_port;
      mDNSPutString(pDNSdesc->mResponderCtx.rr_list[QTYPE_SRV_INDEX].name,pDNSdesc);
      break;
   }

   UDPPut(pDNSdesc->mDNS_socket, 0x00);         // Type: Always QTYPE_ANY
   UDPPut(pDNSdesc->mDNS_socket, QTYPE_ANY);
//UDPPut(pDNSdesc->mDNS_socket, QTYPE_PTR);

   UDPPut(pDNSdesc->mDNS_socket, 0x80);         // Class: Cache-Flush
   UDPPut(pDNSdesc->mDNS_socket, 0x01);         //        IN (Internet)

   // Start of the NS section
   switch (pCtx->type)
   {
   case MDNS_CTX_TYPE_HOST:
      mDNSPutString(pDNSdesc->mResponderCtx.rr_list[QTYPE_A_INDEX].name,pDNSdesc);
      break;

   case MDNS_CTX_TYPE_SD:
     // pDNSdesc->mResponderCtx.rr_list[QTYPE_SRV_INDEX].srv.port.Val=pDNSdesc->mSDCtx.sd_port;
      mDNSPutString(pDNSdesc->mResponderCtx.rr_list[QTYPE_SRV_INDEX].name,pDNSdesc);
      break;
   }

   UDPPut(pDNSdesc->mDNS_socket, 0x00);      // Type: A or SRV

   switch (pCtx->type)
   {
   case MDNS_CTX_TYPE_HOST:
      UDPPut(pDNSdesc->mDNS_socket, QTYPE_A);
      break;

   case MDNS_CTX_TYPE_SD:
      UDPPut(pDNSdesc->mDNS_socket, QTYPE_SRV);
      break;
   }



   UDPPut(pDNSdesc->mDNS_socket, 0x00);      // Class: Cache-Flush bit MUST NOT be set
   UDPPut(pDNSdesc->mDNS_socket, 0x01);      //IN (Internet)

   UDPPut(pDNSdesc->mDNS_socket, 0x00);      // 0x00000078 Time To Live, 2 minutes
   UDPPut(pDNSdesc->mDNS_socket, 0x00);
   UDPPut(pDNSdesc->mDNS_socket, 0x00);
   UDPPut(pDNSdesc->mDNS_socket, 0x78);

   switch (pCtx->type)
   {
   case MDNS_CTX_TYPE_HOST:
      {
         UDPPut(pDNSdesc->mDNS_socket, pDNSdesc->mResponderCtx.rr_list[QTYPE_A_INDEX].rdlength.v[1]);
         UDPPut(pDNSdesc->mDNS_socket, pDNSdesc->mResponderCtx.rr_list[QTYPE_A_INDEX].rdlength.v[0]);

         UDPPut(pDNSdesc->mDNS_socket, pDNSdesc->mResponderCtx.rr_list[QTYPE_A_INDEX].ip.v[0]);
         UDPPut(pDNSdesc->mDNS_socket, pDNSdesc->mResponderCtx.rr_list[QTYPE_A_INDEX].ip.v[1]);
         UDPPut(pDNSdesc->mDNS_socket, pDNSdesc->mResponderCtx.rr_list[QTYPE_A_INDEX].ip.v[2]);
         UDPPut(pDNSdesc->mDNS_socket, pDNSdesc->mResponderCtx.rr_list[QTYPE_A_INDEX].ip.v[3]);

         break;
      }

   case MDNS_CTX_TYPE_SD:
      {
         UDPPut(pDNSdesc->mDNS_socket, pDNSdesc->mResponderCtx.rr_list[QTYPE_SRV_INDEX].rdlength.v[1]);
         UDPPut(pDNSdesc->mDNS_socket, pDNSdesc->mResponderCtx.rr_list[QTYPE_SRV_INDEX].rdlength.v[0]);
         UDPPut(pDNSdesc->mDNS_socket, pDNSdesc->mResponderCtx.rr_list[QTYPE_SRV_INDEX].srv.priority.v[1]);
         UDPPut(pDNSdesc->mDNS_socket, pDNSdesc->mResponderCtx.rr_list[QTYPE_SRV_INDEX].srv.priority.v[0]);
         UDPPut(pDNSdesc->mDNS_socket, pDNSdesc->mResponderCtx.rr_list[QTYPE_SRV_INDEX].srv.weight.v[1]);
         UDPPut(pDNSdesc->mDNS_socket, pDNSdesc->mResponderCtx.rr_list[QTYPE_SRV_INDEX].srv.weight.v[0]);
         UDPPut(pDNSdesc->mDNS_socket, pDNSdesc->mResponderCtx.rr_list[QTYPE_SRV_INDEX].srv.port.v[1]);
         UDPPut(pDNSdesc->mDNS_socket, pDNSdesc->mResponderCtx.rr_list[QTYPE_SRV_INDEX].srv.port.v[0]);

         mDNSPutString(pDNSdesc->mResponderCtx.rr_list[QTYPE_SRV_INDEX].rdata,pDNSdesc);

         break;
      }
   }

   mDNSSetAddresses(pDNSdesc);
   UDPFlush(pDNSdesc->mDNS_socket);

    return true;
}

/***************************************************************
  Function:
   static bool mDNSSendRR(struct mDNSResourceRecord *record,
                   uint8_t record_type, uint32_t ttl_val,uint16_t query_id)

  Summary:
   Sends out a Multicast-DNS-Answer (Resource-Record) to
    Multicast-Address through pDNSdesc->mDNS_socket (UDP Socket).

  Description:
   This function is used in Announce-phase & Defend-Phase.

    In announce-phase the Host-Name or Resource-Record (Service)
    will be announced in local-network, so that neighbors can
    detect new-service or update their caches with new host-name
    to IP-Address mapping.

    In Defend-Phase, when mDNSResponder receives a query for
    Host-name or Resounce-record for which this holds authority.

  Precondition:
   UDP socket (pDNSdesc->mDNS_socket) is obtained and ready for writing.

  Parameters:
   record - Resource-Record filled up with required info
    type   - Type of Res-Rec
    ttl_val - Time-To-Live value for Res-Record
    query_id - Query-ID for which this mDNS-answer (Res-Rec)
               corresponds to

  Returns:
     true - On Success
    false - On Failure (If UDP-Socket is invalid)
  **************************************************************/

static bool
mDNSSendRR(mDNSResourceRecord *pRecord
          ,uint16_t query_id
          ,uint8_t cFlush
          ,uint16_t nAnswersInMsg
          ,bool bIsFirstRR
          ,bool bIsLastRR
          ,DNSDesc_t *pDNSdesc)
{
    MDNS_MSG_HEADER mDNS_header;
    TCPIP_UINT32_VAL ttl;
    uint8_t rec_length;
    uint8_t record_type;
   //char zeroconf_dbg_msg[256];

   record_type = pRecord->type.Val;

   //DEBUG0_MDNS_MESG(zeroconf_dbg_msg, "tx RR: (%d)\r\n", record_type);
   //DEBUG0_MDNS_PRINT(zeroconf_dbg_msg);

    if(pDNSdesc->mDNS_socket == INVALID_UDP_SOCKET)
    {
        WARN_MDNS_PRINT("mDNSSendRR: Opening UDP Socket Failed \r\n");
      return false;
    }


   if (bIsFirstRR)
   {
      if(!UDPIsTxPutReady(pDNSdesc->mDNS_socket,sizeof(MDNS_MSG_HEADER)))
      {
          WARN_MDNS_PRINT("mDNSSendQuery: UDP Socket TX Busy \r\n");
        return false;
      }
      memset(&mDNS_header, 0, sizeof(MDNS_MSG_HEADER));

      mDNS_header.query_id.Val = TCPIP_HELPER_htons(query_id);

      mDNS_header.flags.bits.qr = 1; // this is a Response,
      mDNS_header.flags.bits.aa = 1; // and we are authoritative
      mDNS_header.flags.Val = TCPIP_HELPER_htons(mDNS_header.flags.Val);

      mDNS_header.nAnswers.Val = TCPIP_HELPER_htons(nAnswersInMsg);

      // Put out the mDNS message header
      UDPPutArray(pDNSdesc->mDNS_socket, (uint8_t *) &mDNS_header, sizeof(MDNS_MSG_HEADER));
   }

   ttl.Val = pRecord->ttl.Val;

   mDNSPutString(pRecord->name,pDNSdesc);

   UDPPut(pDNSdesc->mDNS_socket, 0x00); 
   UDPPut(pDNSdesc->mDNS_socket, record_type);      // Resource Record Type
    /* MSB of Upper-byte in Class field acts as
     * Cache-Flush bit to notify all Neighbors to
     * flush their caches and fill with this new
     * information */
   if (UDPSocketDcpt[pDNSdesc->mDNS_socket].remotePort == MDNS_PORT)
   {
      UDPPut(pDNSdesc->mDNS_socket, cFlush); 
      UDPPut(pDNSdesc->mDNS_socket, 0x01);      // Class
   }
   else
   {
      // Legacy/Unicast DNS response should not set the Cache-Flush bit.
      UDPPut(pDNSdesc->mDNS_socket, 0x00); 
      UDPPut(pDNSdesc->mDNS_socket, 0x01);         // Class
   }

    UDPPut(pDNSdesc->mDNS_socket, ttl.v[3]);      // Time To Live
    UDPPut(pDNSdesc->mDNS_socket, ttl.v[2]);
    UDPPut(pDNSdesc->mDNS_socket, ttl.v[1]);
    UDPPut(pDNSdesc->mDNS_socket, ttl.v[0]);

   switch (record_type)
   {
   case QTYPE_A:

        UDPPut(pDNSdesc->mDNS_socket, 0x00);   // 0x0004 Data length
        UDPPut(pDNSdesc->mDNS_socket, 0x04);
        UDPPut(pDNSdesc->mDNS_socket, pRecord->ip.v[0]);   // Put out IP address
        UDPPut(pDNSdesc->mDNS_socket, pRecord->ip.v[1]);
        UDPPut(pDNSdesc->mDNS_socket, pRecord->ip.v[2]);
        UDPPut(pDNSdesc->mDNS_socket, pRecord->ip.v[3]);
      break;

   case QTYPE_PTR:

        /* 2 bytes extra. One for Prefix Length for first-label.
         * Other one for NULL terminator */
        pRecord->rdlength.Val = strlen((char*)pRecord->rdata) + 2 ;

        UDPPut(pDNSdesc->mDNS_socket, pRecord->rdlength.v[1]);
        UDPPut(pDNSdesc->mDNS_socket, pRecord->rdlength.v[0]); // Res-Data Length. 0x4f

        mDNSPutString(((mDNSProcessCtx_sd *) (pRecord->pOwnerCtx))->sd_qualified_name,pDNSdesc); //0x97

      break;

   case QTYPE_SRV:

        /* 2 bytes extra. One for Prefix Length for first-label.
         * Other one for NULL terminator */
        pRecord->rdlength.Val = strlen((char*)pRecord->rdata) + 2;
        pRecord->rdlength.Val += 6;               // for priority, weight, and port

        UDPPut(pDNSdesc->mDNS_socket, pRecord->rdlength.v[1]);  // 0xee
        UDPPut(pDNSdesc->mDNS_socket, pRecord->rdlength.v[0]);      // Res-Data Length

        UDPPut(pDNSdesc->mDNS_socket, pRecord->srv.priority.v[1]);   // Put Priority
        UDPPut(pDNSdesc->mDNS_socket, pRecord->srv.priority.v[0]);
        UDPPut(pDNSdesc->mDNS_socket, pRecord->srv.weight.v[1]);
        UDPPut(pDNSdesc->mDNS_socket, pRecord->srv.weight.v[0]);
        UDPPut(pDNSdesc->mDNS_socket, pRecord->srv.port.v[1]);
        UDPPut(pDNSdesc->mDNS_socket, pRecord->srv.port.v[0]);

        mDNSPutString(pRecord->rdata,pDNSdesc); // 0x120


      break;

   case QTYPE_TXT:

        rec_length = strlen((char*)pRecord->rdata);

        pRecord->rdlength.Val = rec_length + 1;

        UDPPut(pDNSdesc->mDNS_socket, pRecord->rdlength.v[1]); // 0x178
        UDPPut(pDNSdesc->mDNS_socket, pRecord->rdlength.v[0]); // Res-Data Length
        UDPPut(pDNSdesc->mDNS_socket, pRecord->rdlength.Val-1); // As of now only single TXT string supported!!

        if(rec_length>0)
        {
           UDPPutArray(pDNSdesc->mDNS_socket, pRecord->rdata,rec_length); 
        }
      break;

   default:

        WARN_MDNS_PRINT("RR Type not supported \n");
   }

   if (bIsLastRR)
   {
       mDNSSetAddresses(pDNSdesc);
       UDPFlush(pDNSdesc->mDNS_socket);
   }

    return true;
}

/***************************************************************
  Function:
   size_t mDNSSDFormatServiceInstance(uint8_t *string, size_t strSize )

  Summary:
   Formats the Service-Instance name according to DNS-SD standard
    specification.

  Description:
   This function is used to format the Service-Instance name, if
    it contains 'dots' and 'backslashes'

    As the service-instance name will be merged with service-type &
    to distinguish the 'dots' seperating the service-type words and
    'dots' within service-instance name, the 'dots' within service-
    instance name will be replaced with '\.' in place of '.' Even the
    '\' are replaced with '\\'.

    When the resource-record containing service-instance name is
    pushed out, the formatted dots '\.' are sentout as '.' and the
    'dots' sperating the service-type & service-instances are replaced
    with length bytes, as specified in RFC 1035.

  Precondition:
   None

  Parameters:
   String - Service-Instance name to be formatted
    strSize - available size for the formatted string, not to be exceeded

  Returns:
     size of the formatted string
  **************************************************************/
static size_t mDNSSDFormatServiceInstance(uint8_t *string, size_t strSize )
{
   uint8_t *temp;
   uint8_t output[MAX_LABEL_SIZE];
   uint8_t i;
   uint8_t *right_ptr,*str_token;
   uint8_t len;

   temp = output;
   right_ptr = string;
   str_token = string;
   while(1)
   {
      do
      {
         i = *right_ptr++;
      } while((i != 0x00u) && (i != '\\') && (i != '.') );


      /* Prefix '\' for every occurance of '.' & '\' */
      len = (uint8_t)(right_ptr-str_token-1);

      memcpy(temp,str_token,len);
      temp += len;
      str_token +=  len;
      if(i == '.' || i == '\\')
      {
         *temp = '\\';
         temp++;
         *temp++ = i;
         str_token += 1;

      }
      else if(i == 0x00u || i == '/' || i == ',' || i == '>')
         break;

   }
   *temp++ = '\0';
   return strncpy_m((char*)string, strSize, 1, output);
}

/***************************************************************
  Function:
   void mDNSSDFillResRecords(mdnsd_struct *sd)

  Summary:
   Fills the resource-records with the information received from
    sd structure-instance, in which the information is filled from
    user input.

  Description:
   This function is used to fill the resource-records according to
    format specified in RFC 1035.

    In this context Service-Instance + Service-Type is called fully
    qualified name. For ex: Dummy HTTP Web-Server._http._tcp.local
    where Dummy HTTP Web-Server is Service-instance name
     and  _http._tcp.local is Service-Type

    Each service-instance that needs to be advertised contains three
    resource-reocrds.
    1) PTR Resource-Record: This is a shared record, with service-type
                           as rr-name and fully-qualified name as
                           rr-data.
    2) SRV Resource-Record: This is a unique record, with fully-
                            qualified name as rr-name and Host-name,
                            port-num as rr-data.
    3) TXT Resource-Record: This is a unique record, with fully-
                            qualified name as rr-name and additional
                            information as rr-data like default-page
                            name (For ex: "/index.htm")

  Precondition:
   None

  Parameters:
   sd - Service-Discovery structure instance for which Resource-
         records to be filled.

  Returns:
     None
  **************************************************************/
static void mDNSSDFillResRecords(mDNSProcessCtx_sd *sd,DNSDesc_t *pDNSdesc)
{
    size_t srv_name_len,srv_type_len, qual_len;
    mDNSResourceRecord *rr_list;
    //char zeroconf_dbg_msg[256];
    uint16_t serv_port;

    srv_name_len = strlen((char*)sd->srv_name);
    srv_type_len = strlen((char*)sd->srv_type);
    serv_port = pDNSdesc->mSDCtx.sd_port;

    memset(&(pDNSdesc->mResponderCtx.rr_list[QTYPE_PTR_INDEX]),0,(sizeof(mDNSResourceRecord)));
    memset(&(pDNSdesc->mResponderCtx.rr_list[QTYPE_SRV_INDEX]),0,(sizeof(mDNSResourceRecord)));
    memset(&(pDNSdesc->mResponderCtx.rr_list[QTYPE_TXT_INDEX]),0,(sizeof(mDNSResourceRecord)));


    /* Formatting Service-Instance name.
     * And preparing a fully qualified
     * Service-instance record . */


    strncpy((char*)sd->sd_qualified_name, (char*)sd->srv_name, sizeof(sd->sd_qualified_name));
    qual_len= mDNSSDFormatServiceInstance(sd->sd_qualified_name, sizeof(sd->sd_qualified_name));
    strncpy_m((char*)&sd->sd_qualified_name[qual_len], sizeof(sd->sd_qualified_name) - qual_len, 2, ".", sd->srv_type);
    sd->sd_port = pDNSdesc->mSDCtx.sd_port = serv_port;

    DEBUG_MDNS_MESG(zeroconf_dbg_msg,"Service Name and Type: %s \r\n",sd->sd_qualified_name);
    DEBUG_MDNS_PRINT(zeroconf_dbg_msg);


    /* Fill-up PTR Record */
    rr_list = &pDNSdesc->mResponderCtx.rr_list[QTYPE_PTR_INDEX];
    rr_list->type.Val = QTYPE_PTR;
    rr_list->name = (uint8_t *) (sd->srv_type);

    /* Res Record Name is
     * Service_Instance_name._srv-type._proto.domain */
   rr_list->rdata = (uint8_t *) (sd->sd_qualified_name);

    strncpy_m((char*)rr_list->rdata + srv_name_len, strlen((char*)sd->sd_qualified_name) - srv_name_len, 2, ".", sd->srv_type);

    /* 3 bytes extra. One for dot added between
     * Serv-Name and Serv-Type. One for length byte.
     * added for first-label in fully qualified name
     * Other one for NULL terminator */
    rr_list->rdlength.Val = srv_name_len+ srv_type_len + 3;
    rr_list->ttl.Val = RESOURCE_RECORD_TTL_VAL; /* Seconds. Not sure ! Need to check */
    rr_list->pOwnerCtx = (mDNSProcessCtx_common *) sd; /* Save back ptr */
    rr_list->valid = 1; /* Mark as valid */



      /* Fill-up SRV Record */
    rr_list = &pDNSdesc->mResponderCtx.rr_list[QTYPE_SRV_INDEX]; /* Move onto next entry */
    rr_list->name = (uint8_t *) (sd->sd_qualified_name);
    rr_list->type.Val = QTYPE_SRV;
    //CLASS???
    rr_list->ttl.Val = RESOURCE_RECORD_TTL_VAL;

    //rdlength is calculated/assigned last
    rr_list->srv.priority.Val = 0;
    rr_list->srv.weight.Val = 0;
    rr_list->srv.port.Val = pDNSdesc->mSDCtx.sd_port;

    /* Res Record Name is
     * Service_Instance_name._srv-type._proto.domain */
    rr_list->rdata = (uint8_t *) pDNSdesc->mHostCtx.szHostName;


    /* 2 bytes extra. One for Prefix Length for first-label.
     * Other one for NULL terminator */
   // then, add 6-byte extra: for priority, weight, and port

    rr_list->rdlength.Val = strlen((char*)rr_list->rdata)+2+6;
    rr_list->pOwnerCtx = (mDNSProcessCtx_common *) sd; /* Save back ptr */
    rr_list->valid = 1; /* Mark as valid */    




    /* Fill-up TXT Record with NULL data*/
    rr_list = &pDNSdesc->mResponderCtx.rr_list[QTYPE_TXT_INDEX]; /* Move onto next entry */
    rr_list->type.Val = QTYPE_TXT;
    rr_list->name = (uint8_t *) (sd->sd_qualified_name);

    /* Res Record data is what defined by the user */
    rr_list->rdata = (uint8_t *) (sd->sd_txt_rec);

    /* Extra byte for Length-Byte of TXT string */
    rr_list->rdlength.Val = pDNSdesc->mSDCtx.sd_txt_rec_len+1;
    rr_list->ttl.Val = RESOURCE_RECORD_TTL_VAL;
    rr_list->pOwnerCtx = (mDNSProcessCtx_common *) sd; /* Save back ptr */
    rr_list->valid = 1; /* Mark as valid */
}

MDNSD_ERR_CODE
mDNSServiceUpdate(TCPIP_NET_HANDLE netH, uint16_t port, const uint8_t* txt_record)
{
    mDNSProcessCtx_sd *sd;
    DNSDesc_t *pDNSdesc;
    TCPIP_NET_IF* pNetIf;

    pNetIf = _TCPIPStackHandleToNet(netH);
    if(pNetIf != 0)
    {
        pDNSdesc = gDNSdesc + _TCPIPStackNetIx(pNetIf);
        sd = &pDNSdesc->mSDCtx;

        if( sd->used)
        {
            sd->service_registered = 0;
            sd->sd_port = port;
            /* Update Port Value in SRV Resource-record */
            pDNSdesc->mResponderCtx.rr_list[QTYPE_SRV_INDEX].srv.port.Val = port;

            if(txt_record != NULL)
            {
                sd->sd_txt_rec_len = strncpy_m((char*)sd->sd_txt_rec, sizeof(sd->sd_txt_rec), 1, (uint8_t *) txt_record );

                /* Update Resource-records for this
                 * Service-instance, in MDNS-SD state-
                 * -machine */
                mDNSSDFillResRecords(sd,pDNSdesc);
                sd->common.state = MDNS_STATE_NOT_READY;
            }

            /* Notify MDNS Stack about Service-Registration
             * to get a time-slot for its own processing */
            sd->service_registered = 1;
            return MDNSD_SUCCESS;
        }
    }

    return MDNSD_ERR_INVAL;
}

MDNSD_ERR_CODE mDNSServiceDeRegister(TCPIP_NET_HANDLE netH)
{
    DNSDesc_t *pDNSdesc;
    mDNSProcessCtx_sd *sd;
    TCPIP_NET_IF* pNetIf;

    pNetIf = _TCPIPStackHandleToNet(netH);
    if(pNetIf != 0)
    {
        pDNSdesc = gDNSdesc + _TCPIPStackNetIx(pNetIf);
        sd = &pDNSdesc->mSDCtx;

        if(sd->used)
        {
            if(sd->sd_service_advertised == 1)
            {
                /* Send GoodBye Packet */
                pDNSdesc->mResponderCtx.rr_list[QTYPE_PTR_INDEX].ttl.Val = 0;
                pDNSdesc->mResponderCtx.rr_list[QTYPE_SRV_INDEX].ttl.Val = 0;
                pDNSdesc->mResponderCtx.rr_list[QTYPE_SRV_INDEX].ttl.Val = 0;

                mDNSSendRR(&pDNSdesc->mResponderCtx.rr_list[QTYPE_PTR_INDEX], 0, 0x00, 3, true,false,pDNSdesc);
                mDNSSendRR(&pDNSdesc->mResponderCtx.rr_list[QTYPE_SRV_INDEX], 0, 0x80, 3, false,false,pDNSdesc);
                mDNSSendRR(&pDNSdesc->mResponderCtx.rr_list[QTYPE_SRV_INDEX], 0, 0x80, 3, false,true,pDNSdesc);
            }
            /* Clear mSDCtx struct */
            sd->service_registered = 0;
            memset(sd,0,sizeof(mDNSProcessCtx_sd));
            return MDNSD_SUCCESS;
        }
    }

    return MDNSD_ERR_INVAL; /* Invalid Parameter */
}


MDNSD_ERR_CODE
mDNSServiceRegister( TCPIP_NET_HANDLE netH
                    ,const char *srv_name
                    ,const char *srv_type
                    ,uint16_t port
                    ,const uint8_t *txt_record
                    ,uint8_t auto_rename
                    ,void (*call_back)(char *name, MDNSD_ERR_CODE err, void *context)
                    ,void *context)
{
   DNSDesc_t *desc;
   TCPIP_NET_IF* pNetIf;

   if ( (srv_name == NULL) || (srv_type == NULL) || (txt_record == NULL) )
   {
       return MDNSD_ERR_INVAL; // Invalid Parameter
   }

    pNetIf = _TCPIPStackHandleToNet(netH);
    if(pNetIf != 0)
    {
        desc = gDNSdesc + _TCPIPStackNetIx(pNetIf);

        if(desc->mSDCtx.used)
        {
            return MDNSD_ERR_BUSY;
        }

        /* Clear the State-Machine */
        memset(&desc->mSDCtx,0,sizeof(mDNSProcessCtx_sd));
        desc->mSDCtx.used = 1; /* Mark it as used */
        desc->mSDCtx.sd_auto_rename = auto_rename;
        desc->mSDCtx.sd_port = port;
        desc->mSDCtx.sd_service_advertised = 0;

        strncpy((char*)desc->mSDCtx.srv_name
                , (char*)srv_name
                , sizeof(desc->mSDCtx.srv_name));

        strncpy((char*)desc->mSDCtx.srv_type
                , (char*)srv_type
                , sizeof(desc->mSDCtx.srv_type));

        desc->mSDCtx.sd_call_back = call_back;
        desc->mSDCtx.sd_context   = context;

        desc->mSDCtx.sd_txt_rec_len = strncpy_m((char*)desc->mSDCtx.sd_txt_rec
                ,sizeof(desc->mSDCtx.sd_txt_rec)
                ,1
                ,(uint8_t *) txt_record);

        /* Fill up Resource-records for this
         * Service-instance, in MDNS-SD state-
         * -machine */
        mDNSSDFillResRecords(&desc->mSDCtx,desc);

        desc->mSDCtx.common.type  = MDNS_CTX_TYPE_SD;
        desc->mSDCtx.common.state = MDNS_STATE_NOT_READY;
        desc->mSDCtx.common.nInstanceId = 0;

        /* Notify MDNS Stack about Service-Registration
         * to get a time-slot for its own processing */
        desc->mSDCtx.service_registered = 1;
        return MDNSD_SUCCESS;
    }

   return MDNSD_ERR_INVAL; // unknown interface
}

/***************************************************************
  Function:
   static void mDNSSDStateMachineReset(mdnsd_struct *sd)

  Summary:
   Resets DNS-SD state-machine

  Description:
   This function is used to reset all state-variables related
    to DNS-SD state-machine.

  Precondition:
   None

  Parameters:
   sd - Service Discovery structure instance

  Returns:
     None
  **************************************************************/

/***************************************************************
  Function:
   void mDNSSDProbe(mdnsd_struct *sd)

  Summary:
   Sends out Multicast-DNS SD probe-packet with chosen service-name
    The Query-type is of SRV.

  Description:
   This function is used to send out mDNS-probe packet for
    checking uniqueness of selected service-instance-name. This
    function makes use of mDNSSendQuery to send out DNS-Query with
    chosen service-name to Multicast-Address. The type of such query
    is QTYPE_SRV.

    If any other machine is using same service-name, it responds with
    a reply and this host has to select different service-name.

  Precondition:
   None

  Parameters:
   sd - Service Discovery structure instance

  Returns:
     None
  **************************************************************/


/***************************************************************
  Function:
   void mDNSSDAnnounce(mdnsd_struct *sd)

  Summary:
   Sends out Multicast-DNS SD packet with SRV Resource-Record.

  Description:
   This function is used to send out DNS-SD SRV resource-record
    Announce packet for announcing the service-name on local network.
    This function makes use of mDNSSendRR to send out DNS-Resource-
    Record with chosen service-name+service-type as rr-name and the
    host-name, port-number as rr-data.

    This announcement updates DNS-caches of neighbor machines on
    the local network.

  Precondition:
   None

  Parameters:
   sd - Service Discovery structure instance

  Returns:
     None
  **************************************************************/
static void mDNSAnnounce(mDNSResourceRecord *pRR, DNSDesc_t *pDNSdesc)
{
    if( false ==
      mDNSSendRR(pRR
                 ,0
                 ,0x80
                 ,1
                 ,true
                 ,true
                 ,pDNSdesc)
      )
   {
        WARN_MDNS_PRINT("mDNSAnnounce: Error in sending out Announce pkt \r\n");
   }
}


static uint16_t mDNSFetch(uint16_t wOffset, uint16_t wLen, uint8_t *pcString,DNSDesc_t *pDNSdesc)
{
   uint16_t rc;

   UDPSetRxOffset(pDNSdesc->mDNS_socket, wOffset);

   rc = UDPGetArray(pDNSdesc->mDNS_socket, pcString, wLen);

   return rc;
}


/***************************************************************
  Function:
   static uint16_t mDNSDeCompress(uint16_t wPos,
                               uint8_t *pcString,
                               bool bFollowPtr,
                               uint8_t cElement,
                               uint8_t cDepth)

  Summary:
   Read a string from a resource record, from the Multicast-DNS socket buffer.

  Description:
   This function reads a string to the Multicast-DNS socket,
    ensuring that it is properly formatted.

    String may be reconstructed through decompression if necessary.
   Decompression pointer traversal is done in place, recursively, in UDP's RxBuffer.

   cDepth represents the recursion depth, for debugging purpose.

   cElement represents the number of elements in the string. For example,
    "ezconfig._http._tcp.local" has 4 elements.

   bFollowPtr indicates if DNS compression offset needs to be followed. That is, if
   we should reconstruct a compressed string.

   The reconstructed string is placed in pcString, if it is not NULL.

   For DNS message compression format, see RFC 1035, section 4.1.4.

  Precondition:
   UDP socket is obtained and ready for writing.
    wPos correctly reflect the current position in the UDP RxBuffer.

  Parameters:
   String - the string to write to the UDP socket.

  Returns:
     Number of bytes in THIS resource record field (in RFC 1035's term, NAME or RDATA).
    UDP RxBuffer pointer is repositioned to the place right after THIS resource record field.

  **************************************************************/

static uint16_t mDNSDeCompress(uint16_t wPos
                                   ,uint8_t *pcString
                                   ,bool bFollowPtr
                                   ,uint8_t cElement
                                   ,uint8_t cDepth
                                   ,DNSDesc_t *pDNSdesc)
{
   uint16_t rr_len = 0; // As is in the packet. Could be in compressed format.
   uint16_t startOffset, endOffset, currOffset;
   uint8_t i, tmp;
   uint16_t offset_in_ptr;
   uint16_t len;
   uint8_t substr_len;
//   char zeroconf_dbg_msg[256];

   currOffset = startOffset = wPos;

   while (1)
   {
      rr_len++;
      if(!UDPGet(pDNSdesc->mDNS_socket, &substr_len))
         goto mDNSDeCompress_done;

      if(substr_len == 0u)
      {
         if (pcString)
         {
            *pcString++ = '\0';
         }
         goto mDNSDeCompress_done;
      }

      if((substr_len & 0xC0) == 0xC0)   // b'11 at MSb indicates compression ptr
      {
         offset_in_ptr = substr_len & 0x3F; // the rest of 6 bits is part of offset_in_ptr.
         offset_in_ptr = offset_in_ptr << 8;

         /* Remove label-ptr byte */
         rr_len++;
         UDPGet(pDNSdesc->mDNS_socket, &i);
         offset_in_ptr += i;

         if (bFollowPtr)
         {
            cDepth++;

            //DEBUG_MDNS_MESG(zeroconf_dbg_msg, "follow ptr: h'%X, ", offset_in_ptr);
            //DEBUG_MDNS_PRINT(zeroconf_dbg_msg);
            //DEBUG_MDNS_MESG(zeroconf_dbg_msg, "depth: %d, rr_len: %d\r\n", cDepth, rr_len);
            //DEBUG_MDNS_PRINT(zeroconf_dbg_msg);

            UDPSetRxOffset(pDNSdesc->mDNS_socket, offset_in_ptr);
            len = mDNSDeCompress(offset_in_ptr, pcString, bFollowPtr, cElement, cDepth,pDNSdesc);

            // compressed ptr is always the last element
            goto mDNSDeCompress_done;
         }

         goto mDNSDeCompress_done;
      }
      else
      {
         if (pcString)
         {
            if (cElement > 0)
            {
               // not the first element in name
               *pcString++ = '.';
            }

            UDPGetArray(pDNSdesc->mDNS_socket, pcString, substr_len);
            pcString += substr_len;
         }
         else
         {
            i = substr_len;
            while (i--)
            {
               UDPGet(pDNSdesc->mDNS_socket, &tmp);
            }
         }

         cElement++;
         rr_len += substr_len;
      }
   }

   mDNSDeCompress_done:

   endOffset = startOffset + rr_len;
   UDPSetRxOffset(pDNSdesc->mDNS_socket, endOffset);

   return rr_len;
}



static bool
mDNSTieBreaker(mDNSResourceRecord *their, mDNSResourceRecord *our)
{
   bool WeWonTheTieBreaker = true;
   uint8_t i;

   if (their->type.Val == QTYPE_A)
   {
      for (i = 0; i<= 3; i++)
      {
         if (their->ip.v[i] < our->ip.v[i])
         {
            WeWonTheTieBreaker = true;
            break;
         }
         else if (their->ip.v[i] > our->ip.v[i])
         {
            WeWonTheTieBreaker = false;
            break;
         }
      }
   }
   else if (their->type.Val == QTYPE_SRV)
   {
      if (their->srv.port.Val >= our->srv.port.Val)
      {
         WeWonTheTieBreaker = false;
      }
   }

   DEBUG0_MDNS_PRINT( (char *) (WeWonTheTieBreaker ? "   tie-breaker won\r\n" : "   tie-breaker lost\r\n") );

   return WeWonTheTieBreaker;
}


static uint8_t
mDNSProcessIncomingRR(MDNS_RR_GROUP     tag
                      ,MDNS_MSG_HEADER *pmDNSMsgHeader
                      ,uint16_t         idxGroup
                      ,uint16_t         idxRR
                      ,DNSDesc_t       *pDNSdesc)
{
   mDNSResourceRecord res_rec;
   uint8_t name[2 * MAX_RR_NAME_SIZE];
   uint8_t i,j;
   uint16_t len;
   uint8_t tmp;
   mDNSProcessCtx_common *pOwnerCtx;
   mDNSResourceRecord      *pMyRR;
   bool WeWonTheTieBreaker = false;
   bool bMsgIsAQuery;         // QUERY or RESPONSE ?
   bool bSenderHasAuthority;   // Sender has the authority ?
   // char zeroconf_dbg_msg[256];

   bMsgIsAQuery = (pmDNSMsgHeader->flags.bits.qr == 0);
   bSenderHasAuthority = (pmDNSMsgHeader->flags.bits.qr == 1);

   res_rec.name = name; // for temporary name storage.

   DEBUG0_MDNS_MESG(zeroconf_dbg_msg,"   rec [%d:%d]\t",idxGroup, idxRR);
   DEBUG0_MDNS_PRINT(zeroconf_dbg_msg);

   // NAME
   memset(name, 0, sizeof(name));
   len = mDNSDeCompress(pDNSdesc->mDNS_offset, name, true, 0, 0,pDNSdesc);
   pDNSdesc->mDNS_offset += len;

   // TYPE & CLASS
   UDPGet(pDNSdesc->mDNS_socket, &res_rec.type.v[1]);
   UDPGet(pDNSdesc->mDNS_socket, &res_rec.type.v[0]);
   UDPGet(pDNSdesc->mDNS_socket, &res_rec.class.v[1]);
   UDPGet(pDNSdesc->mDNS_socket, &res_rec.class.v[0]);
   pDNSdesc->mDNS_offset += 4;

   DEBUG0_MDNS_PRINT("Name: ");
   DEBUG0_MDNS_PRINT((char *) name);
   DEBUG0_MDNS_MESG(zeroconf_dbg_msg," Type: %d", res_rec.type.Val);
   DEBUG0_MDNS_PRINT((char*)zeroconf_dbg_msg);
   DEBUG0_MDNS_PRINT("\r\n");

   // Do the first round name check
   for (i = 0; i < MAX_RR_NUM; i++)
   {
      pDNSdesc->mResponderCtx.rr_list[i].bNameAndTypeMatched = false;

      if (
         !strcmp_local_ignore_case((void *)name, pDNSdesc->mResponderCtx.rr_list[i].name)
         &&
         ((res_rec.type.Val == QTYPE_ANY) ||
          (res_rec.type.Val == pDNSdesc->mResponderCtx.rr_list[i].type.Val))
         )
      {
         pDNSdesc->mResponderCtx.rr_list[i].bNameAndTypeMatched = true;
      }
      else if (
         (tag == MDNS_RR_GROUP_QD)
         &&
         !strcmp_local_ignore_case(name,(uint8_t *) "_services._dns-sd._udp.local")
         &&
         (res_rec.type.Val == QTYPE_PTR)
         )
      {
         pDNSdesc->mResponderCtx.rr_list[i].bNameAndTypeMatched = true;
      }
   }


   // Only AN, NS, AR records have extra fields
   if ( tag == MDNS_RR_GROUP_QD )
   {
      goto ReviewStage;
   }

   // Now retrieve those extra fields
   UDPGet(pDNSdesc->mDNS_socket, &res_rec.ttl.v[3]);      // Time to live
   UDPGet(pDNSdesc->mDNS_socket, &res_rec.ttl.v[2]);
   UDPGet(pDNSdesc->mDNS_socket, &res_rec.ttl.v[1]);
   UDPGet(pDNSdesc->mDNS_socket, &res_rec.ttl.v[0]);
   UDPGet(pDNSdesc->mDNS_socket, &res_rec.rdlength.v[1]);      // Response length
   UDPGet(pDNSdesc->mDNS_socket, &res_rec.rdlength.v[0]);
   pDNSdesc->mDNS_offset += 6;

   // The rest is record type dependent
   switch (res_rec.type.Val)
   {
   case QTYPE_A:
      UDPGet(pDNSdesc->mDNS_socket, &res_rec.ip.v[0]); // Read out IP address
      UDPGet(pDNSdesc->mDNS_socket, &res_rec.ip.v[1]);
      UDPGet(pDNSdesc->mDNS_socket, &res_rec.ip.v[2]);
      UDPGet(pDNSdesc->mDNS_socket, &res_rec.ip.v[3]);

      pDNSdesc->mDNS_offset += 4;

      //DEBUG_MDNS_MESG(zeroconf_dbg_msg, "     [A]: TTL=%d\r\n", res_rec.ttl.Val);
      //DEBUG_MDNS_PRINT((char*)zeroconf_dbg_msg);

      break;

   case QTYPE_PTR:

      memset(name, 0 , sizeof(name));
      len = mDNSDeCompress(pDNSdesc->mDNS_offset, name, true, 0, 0,pDNSdesc);
      pDNSdesc->mDNS_offset += len;

      //DEBUG_MDNS_MESG(zeroconf_dbg_msg, "     [PTR]: TTL=%d RDATA=%s\r\n", res_rec.ttl.Val,name);
      //DEBUG_MDNS_PRINT((char*)zeroconf_dbg_msg);

      break;

   case QTYPE_SRV:
      UDPGet(pDNSdesc->mDNS_socket, &res_rec.srv.priority.v[1]); // Put Priority, weight, port
      UDPGet(pDNSdesc->mDNS_socket, &res_rec.srv.priority.v[0]);
      UDPGet(pDNSdesc->mDNS_socket, &res_rec.srv.weight.v[1]);
      UDPGet(pDNSdesc->mDNS_socket, &res_rec.srv.weight.v[0]);
      UDPGet(pDNSdesc->mDNS_socket, &res_rec.srv.port.v[1]);
      UDPGet(pDNSdesc->mDNS_socket, &res_rec.srv.port.v[0]);

      pDNSdesc->mDNS_offset += 6;

      memset(name, 0 , sizeof(name));
      len = mDNSDeCompress(pDNSdesc->mDNS_offset, name, true, 0, 0,pDNSdesc);
      pDNSdesc->mDNS_offset += len;

      //DEBUG_MDNS_MESG(zeroconf_dbg_msg, "     [SRV]: TTL=%d RDATA=%s\r\n", res_rec.ttl.Val,name);
      //DEBUG_MDNS_PRINT((char*)zeroconf_dbg_msg);

      break;

   case QTYPE_TXT:

      i = res_rec.rdlength.Val;

      while (i--)
      {
         UDPGet(pDNSdesc->mDNS_socket, &tmp);
      }

      pDNSdesc->mDNS_offset += res_rec.rdlength.Val;

      DEBUG_MDNS_MESG(zeroconf_dbg_msg, "     [TXT]: (%d bytes)\r\n", res_rec.rdlength.Val);
      DEBUG_MDNS_PRINT((char*)zeroconf_dbg_msg);

      break;

   default:

      // Still needs to read it off

      i = res_rec.rdlength.Val;

      while (i--)
      {
         UDPGet(pDNSdesc->mDNS_socket, &tmp);
      }

      pDNSdesc->mDNS_offset += res_rec.rdlength.Val;

      DEBUG_MDNS_MESG(zeroconf_dbg_msg, "     [*]: (%d bytes)\r\n", res_rec.rdlength.Val);
      DEBUG_MDNS_PRINT((char*)zeroconf_dbg_msg);

      break;
   }

   // We now have all info about this received RR.

ReviewStage:

   // Do the second round
   for (i = 0; i < MAX_RR_NUM; i++)
   {
      pMyRR = &(pDNSdesc->mResponderCtx.rr_list[i]);
      pOwnerCtx = pDNSdesc->mResponderCtx.rr_list[i].pOwnerCtx;

      if ( (!pMyRR->bNameAndTypeMatched) || (pOwnerCtx == NULL) )
      {
         // do nothing
      }
      else if (
         bMsgIsAQuery &&
         (tag == MDNS_RR_GROUP_QD) &&
         (pOwnerCtx->state == MDNS_STATE_DEFEND)
         )
      {
         // Simple reply to an incoming DNS query.
         // Mark all of our RRs for reply.

         for (j = 0; j < MAX_RR_NUM; j++)
         {
            pDNSdesc->mResponderCtx.rr_list[j].bResponseRequested = true;
         }
      }
      else if (
         bMsgIsAQuery &&
         (tag == MDNS_RR_GROUP_AN) &&
         (pOwnerCtx->state == MDNS_STATE_DEFEND)
         )
      {
         // An answer in the incoming DNS query.
         // Look for possible duplicate (known) answers suppression.
         if ((((res_rec.type.Val == QTYPE_PTR) && (res_rec.ip.Val == pDNSdesc->mResponderCtx.rr_list[i].ip.Val))
             ||
            (!strcmp_local_ignore_case(name, pDNSdesc->mResponderCtx.rr_list[i].rdata)))
            &&
            (res_rec.ttl.Val > (pDNSdesc->mResponderCtx.rr_list[i].ttl.Val/2))
            )
         {
            pDNSdesc->mResponderCtx.rr_list[i].bResponseSuppressed = true;
            DEBUG_MDNS_PRINT("     rr suppressed\r\n");
         }
      }
      else if (
         bMsgIsAQuery &&
         (tag == MDNS_RR_GROUP_NS) &&
         ((pOwnerCtx->state == MDNS_STATE_PROBE) ||
          (pOwnerCtx->state == MDNS_STATE_ANNOUNCE))
         )
      {
         // Simultaneous probes by us and sender of this DNS query.
         // Mark as a conflict ONLY IF we lose the Tie-Breaker.

         WeWonTheTieBreaker = mDNSTieBreaker(&res_rec,
                                    &(pDNSdesc->mResponderCtx.rr_list[i]));

         if (!WeWonTheTieBreaker)
         {
            pOwnerCtx->bProbeConflictSeen = true;
            pOwnerCtx->nProbeConflictCount++;
         }

         UDPDiscard(pDNSdesc->mDNS_socket);

         return 0;
      }
      else if (
         !bMsgIsAQuery             &&
         bSenderHasAuthority       &&
         (tag == MDNS_RR_GROUP_AN) &&
         ((pOwnerCtx->state == MDNS_STATE_PROBE) ||
          (pOwnerCtx->state == MDNS_STATE_ANNOUNCE))
         )
      {
         // An authoritative DNS response to our probe/announcement.
         // Mark as a conflict. Effect a re-name, followed by a
         // re-probe.

         pOwnerCtx->bProbeConflictSeen = true;
         pOwnerCtx->nProbeConflictCount++;

         UDPDiscard(pDNSdesc->mDNS_socket);

         return 0;
      }
      else if(bMsgIsAQuery             &&
             (tag == MDNS_RR_GROUP_NS) &&
             (pOwnerCtx->state == MDNS_STATE_DEFEND)
         )
      {
         // A probe by the sender conflicts with our established record.
         // Need to defend our record. Effect a DNS response.

         INFO_MDNS_PRINT("Defending RR: \r\n");

         pMyRR->bResponseRequested = true;

         UDPDiscard(pDNSdesc->mDNS_socket);

         return 0;
      }
      else if (
         !bMsgIsAQuery &&
          bSenderHasAuthority &&
         (tag == MDNS_RR_GROUP_AN) &&
         (pMyRR->type.Val != QTYPE_PTR ) &&      // No one can claim authority on shared RR
         (pOwnerCtx->state == MDNS_STATE_DEFEND)
         )
      {
         // Sender claims that it also has the authority on
         // a unique (non-shared) record that we have already established authority.
         // Effect a re-probe.

         pOwnerCtx->bLateConflictSeen = true;

         UDPDiscard(pDNSdesc->mDNS_socket);

         return 0;
      }
   }
   return 0;
}


/***************************************************************
  Function:
   static void mDNSResponder(DNSDesc_t *pDNSdesc)

  Summary:
   Acts as Multicast-DNS responder & replies when it receives
    a query. Currenlty only supports IP_ADDRESS_TYPE_IPV4 addressing

  Description:
   This function is used as mDNS-Responder. On initialization of
    Multicast-DNS stack, this function Opens up pDNSdesc->mDNS_socket
    (UDP-Socket) for Mulitcast-Address (224.0.0.251).

    This function gets polled from mDNSProcess for every iteration.
    mDNSResponder constantly monitors the packets being sent to
    Multicast-Address, to check whether it is a conflict with
    its own host-name/resource-record names. It also verifies
    whether incoming query is for its own Host-name/Resource-
    Record, in which case it sends back a reply with corresponding
    Resource-Record.

  Precondition:
   UDP socket (pDNSdesc->mDNS_socket) is obtained and ready for writing.
    A UDP socket must be available before this function is called.
    UDP_MAX_SOCKETS may need to be increased if other modules use
    UDP sockets.

  Parameters:
   None

  Returns:
     None
  **************************************************************/
static void mDNSResponder(DNSDesc_t *pDNSdesc)
{
   MDNS_MSG_HEADER mDNS_header;
   uint16_t len;
   uint16_t i,j,count;
   uint16_t rr_count[4];
   MDNS_RR_GROUP rr_group[4];
   bool bMsgIsComplete;


   pDNSdesc->mDNS_offset = 0;

   if(pDNSdesc->mDNS_socket == INVALID_UDP_SOCKET)
   {
        pDNSdesc->mDNS_responder_state = MDNS_RESPONDER_INIT;
   }

   switch(pDNSdesc->mDNS_responder_state)
   {
      case MDNS_RESPONDER_INIT:


        //IP_MULTI_ADDRESS defAddr;
        //defAddr.v4Add.Val = TCPIP_STACK_NetAddress(TCPIP_STACK_GetDefaultNet());
        //pDNSdesc->mDNS_socket = UDPOpenServer(IP_ADDRESS_TYPE_IPV4, MDNS_PORT, &defAddr);
       // pDNSdesc->mDNS_socket = UDPOpenServer(IP_ADDRESS_TYPE_IPV4, MDNS_PORT, 0);

         if(pDNSdesc->mDNS_socket == INVALID_UDP_SOCKET)
         {
            WARN_MDNS_PRINT("mDNSResponder: Can't open Multicast-DNS UDP-Socket \r\n");
            return;
         }
         else
         {
             UDPSocketSetNet(pDNSdesc->mDNS_socket, TCPIP_STACK_GetDefaultNet());
             //UDPRemoteBind(pDNSdesc->mDNS_socket, IP_ADDRESS_TYPE_IPV4, MDNS_PORT, &Addr);
             //
                pDNSdesc->mDNS_responder_state = MDNS_RESPONDER_LISTEN ;
         }

            /* Called from mDNSInitialize. So return immediately */
            break;

      case MDNS_RESPONDER_LISTEN:

         // Do nothing if no data is waiting
         if(!UDPIsGetReady(pDNSdesc->mDNS_socket))
            return;

   //      if( UDPSocketDcpt[pDNSdesc->mDNS_socket].remotePort != MDNS_PORT )
   //      {
            // If the remote port (sender's src port)
            // is not MDNS_PORT (5353), then it is a multicast query
            // requesting a unicast response (even though the packet
            // was sent to the multicast group IP:MDNS_PORT ).
            // The response needs to be unicast, and sent
            // to sender:port, NOT to the multicast group IP:MDSN_PORT
            // (i.e., 224.0.0.251:5353).

            // See section 8.5, draft-cheshire-dnsext-multicastdns-08.txt.

            //This section is only of value when operating beyond local .link
        // which is not yet supported. For this reason this entire block is commented
         //out
       //  }
 //        else
         {
            /* Reset the Remote-node information in UDP-socket */
            UDPSocketDcpt[pDNSdesc->mDNS_socket].remotePort = MDNS_PORT;
            UDPSocketDcpt[pDNSdesc->mDNS_socket].localPort = MDNS_PORT;
         }

         // Retrieve the mDNS header
         len = mDNSFetch(0, sizeof(mDNS_header), (uint8_t *) &mDNS_header,pDNSdesc);
         mDNS_header.query_id.Val = TCPIP_HELPER_ntohs(mDNS_header.query_id.Val);
         mDNS_header.flags.Val = TCPIP_HELPER_ntohs(mDNS_header.flags.Val);
         mDNS_header.nQuestions.Val = TCPIP_HELPER_ntohs(mDNS_header.nQuestions.Val);
         mDNS_header.nAnswers.Val = TCPIP_HELPER_ntohs(mDNS_header.nAnswers.Val);
         mDNS_header.nAuthoritativeRecords.Val = TCPIP_HELPER_ntohs(mDNS_header.nAuthoritativeRecords.Val);
         mDNS_header.nAdditionalRecords.Val = TCPIP_HELPER_ntohs(mDNS_header.nAdditionalRecords.Val);

         pDNSdesc->mDNS_offset += len; // MUST BE 12

         if ( (mDNS_header.flags.bits.qr == 0) )
         {
            DEBUG0_MDNS_PRINT("rx QUERY \r\n");
         }
         else
         {
            DEBUG0_MDNS_PRINT("rx RESPONSE \r\n");
         }

         bMsgIsComplete = (mDNS_header.flags.bits.tc == 0);  // Message is not truncated.

         rr_count[0] = mDNS_header.nQuestions.Val;
         rr_group[0] = MDNS_RR_GROUP_QD;

         rr_count[1] = mDNS_header.nAnswers.Val;
         rr_group[1] = MDNS_RR_GROUP_AN;

         rr_count[2] = mDNS_header.nAuthoritativeRecords.Val;
         rr_group[2] = MDNS_RR_GROUP_NS;

         rr_count[3] = mDNS_header.nAdditionalRecords.Val;
         rr_group[3] = MDNS_RR_GROUP_AR;

         for (i = 0; i < MAX_RR_NUM; i++)
         {
            // Reset flags
            pDNSdesc->mResponderCtx.rr_list[i].bNameAndTypeMatched = false;

            if (pDNSdesc->mResponderCtx.bLastMsgIsIncomplete)
            {
               // Do nothing.
               // Whether a reply is needed is determined only when all parts
               // of the message are received.

               // Ideally, we want to verify that the current message is the
               // continuation of the previous message.
               // Don't have a cost-effective way to do this yet.
            }
            else
            {
               // Start of a new message

               pDNSdesc->mResponderCtx.rr_list[i].bResponseRequested = false;
               pDNSdesc->mResponderCtx.rr_list[i].bResponseSuppressed = false;
               pDNSdesc->mResponderCtx.rr_list[i].srv.port.Val=pDNSdesc->mSDCtx.sd_port;
            }
         }

         for (i=0; i<4; i++) // for all 4 groups: QD, AN, NS, AR
         {
            for(j=0; j < rr_count[i]; j++)      // RR_count = {#QD, #AN, #NS, #AR}
            {
               mDNSProcessIncomingRR(rr_group[i]
                                    ,&mDNS_header
                                    ,i
                                    ,j
                                    ,pDNSdesc);
            }
         }

         // Record the fact, for the next incoming message.
         pDNSdesc->mResponderCtx.bLastMsgIsIncomplete = (bMsgIsComplete == false);

         // Do not reply any answer if the current message is not the last part of
         // the complete message.
         // Future parts of the message may request some answers be suppressed.

         if (!bMsgIsComplete)
         {
            DEBUG0_MDNS_PRINT("   truncated msg.\r\n");
            return;
         }

         // Count all RRs marked as "reply needed".
         count = 0;
         for (i = 0; i < MAX_RR_NUM; i++)
         {
            if ((pDNSdesc->mResponderCtx.rr_list[i].pOwnerCtx != NULL) &&
               (pDNSdesc->mResponderCtx.rr_list[i].pOwnerCtx->state == MDNS_STATE_DEFEND) &&
               (pDNSdesc->mResponderCtx.rr_list[i].bResponseRequested == true) &&
               (pDNSdesc->mResponderCtx.rr_list[i].bResponseSuppressed == false)
               )
            {
               count++;
            }
         }

         // Send all RRs marked as "reply needed".

        mDNSSetAddresses(pDNSdesc);

         j = 1;
         for (i = 0; (count > 0) && (i < MAX_RR_NUM); i++)
         {
            if ((pDNSdesc->mResponderCtx.rr_list[i].pOwnerCtx != NULL) &&
               (pDNSdesc->mResponderCtx.rr_list[i].pOwnerCtx->state == MDNS_STATE_DEFEND) &&
               (pDNSdesc->mResponderCtx.rr_list[i].bResponseRequested == true) &&
               (pDNSdesc->mResponderCtx.rr_list[i].bResponseSuppressed == false) )
            {
               mDNSSendRR(&pDNSdesc->mResponderCtx.rr_list[i]
                         ,mDNS_header.query_id.Val
                         ,(pDNSdesc->mResponderCtx.rr_list[i].type.Val == QTYPE_PTR)?(0x00):(0x80) // flush, except for PTR; for Conformance Test.
                         ,count                           // MAX_RR_NUM answers;
                         ,(j==1)?true:false               // Is this the first RR?
                         ,(j==count)?true:false
                         ,pDNSdesc);         // Is this the last RR?

               j++;
            }
         }

         // end of MDNS_RESPONDER_LISTEN
         break;

      default:
         break;
   }

   return;
}




/***************************************************************
  Function:
   void void mDNSFillHostRecord(DNSDesc_t *pDNSdesc)

  Summary:


  Description:

  Precondition:
   None

  Parameters:
   None

  Returns:
     None
  **************************************************************/
static void mDNSFillHostRecord(DNSDesc_t *pDNSdesc)
{
   uint8_t i;

   // Fill the type A resource record
   pDNSdesc->mResponderCtx.rr_list[QTYPE_A_INDEX].name     = pDNSdesc->mHostCtx.szHostName;
   pDNSdesc->mResponderCtx.rr_list[QTYPE_A_INDEX].type.Val = QTYPE_A; // Query Type is Answer
   // CLASS??? 1=INternet, 255=Any class etc
   pDNSdesc->mResponderCtx.rr_list[QTYPE_A_INDEX].ttl.Val  = RESOURCE_RECORD_TTL_VAL;

   pDNSdesc->mResponderCtx.rr_list[QTYPE_A_INDEX].rdlength.Val = 4u; // 4-byte for IP address

   // Fill in the data for an A RR record (IP address)
   for (i=0; i<=3; i++)
      pDNSdesc->mResponderCtx.rr_list[QTYPE_A_INDEX].ip.v[i] = pDNSdesc->mTcpIpNetIf->netIPAddr.v[i];

   pDNSdesc->mResponderCtx.rr_list[QTYPE_A_INDEX].valid    = 1;
   pDNSdesc->mResponderCtx.rr_list[QTYPE_A_INDEX].pOwnerCtx = (mDNSProcessCtx_common *) &pDNSdesc->mHostCtx;
}



static MDNSD_ERR_CODE mDNSHostRegister( char *host_name,DNSDesc_t *pDNSdesc)
{

   memcpy((char*)pDNSdesc->mHostCtx.szUserChosenHostName
          , host_name
          , sizeof(pDNSdesc->mHostCtx.szUserChosenHostName));

   strncpy_m((char*)pDNSdesc->mHostCtx.szHostName
            , sizeof(pDNSdesc->mHostCtx.szHostName)
            , 3
            , pDNSdesc->mHostCtx.szUserChosenHostName
            , "."
            , pDNSdesc->CONST_STR_local);

   // strncpy does no always null terminate.
   pDNSdesc->mHostCtx.szUserChosenHostName[MAX_HOST_NAME_SIZE-1]=0;
   pDNSdesc->mHostCtx.szHostName[MAX_HOST_NAME_SIZE-1]=0;

   mDNSResetCounters((mDNSProcessCtx_common *) &pDNSdesc->mHostCtx, true);
   pDNSdesc->mHostCtx.common.type   = MDNS_CTX_TYPE_HOST;
   pDNSdesc->mHostCtx.common.state  = MDNS_STATE_INIT;
   pDNSdesc->mHostCtx.common.nInstanceId = 0;

   // Now create a QTYPE_A record for later use when answering queries.
   mDNSFillHostRecord(pDNSdesc);
   pDNSdesc->mResponderCtx.bLastMsgIsIncomplete = false;

   pDNSdesc->mHostCtx.common.state = MDNS_STATE_INIT;

   return MDNSD_SUCCESS;
}


// Returns true on success
bool mDNSInitialize( const TCPIP_STACK_MODULE_CTRL* const stackCtrl
                    ,const DNS_CLIENT_MODULE_CONFIG* dnsData)
{
    DNSDesc_t *desc;
    TCPIP_NET_IF*  pNetIf;
    char ServiceName[64];
    char*   hostName;
    int32_t n;

    if(stackCtrl->stackAction == TCPIP_STACK_ACTION_IF_UP)
    {   // interface restart
        return true;
    }

    // stack init

    if(mDNSInitCount == 0)
    {   // first time we're run

        // Alocate memory for a memory block of descriptors. Once descriptor per interface
        gDNSdesc = (DNSDesc_t*)TCPIP_HEAP_Calloc(stackCtrl->memH, stackCtrl->nIfs, sizeof(DNSDesc_t));
        if(gDNSdesc == (DNSDesc_t*)0)
        {
            SYS_ERROR(SYS_ERROR_ERROR, "mDNSInitialize: Failed to allocate memory\r\n");
            return false;
        }
    }


    pNetIf = stackCtrl->pNetIf;

    // Remove any trailing spaces from ther NetBiosName
    strncpy(ServiceName, (char*)pNetIf->NetBIOSName, sizeof(ServiceName) - 1);
    n = strlen((char*)pNetIf->NetBIOSName);
    ServiceName[n] = 0; // NULL Terminate in advance
    for( ; n >= 0; n--)
    {
        if(isalnum(ServiceName[n]) || ispunct(ServiceName[n]))
        {
            break;
        }
        else
        {
           ServiceName[n]=0;
        }
    }

    desc = gDNSdesc+ stackCtrl->netIx;
    desc->mTcpIpNetIf = pNetIf;
    desc->mResponderCtx.query_id.Val = 0;
    desc->mResponderCtx.prev_ipaddr.Val = desc->mTcpIpNetIf->netIPAddr.Val;

    strncpy(desc->CONST_STR_local,"local",6);

    hostName = 0;
    if(ServiceName[0] != 0)
    {
        hostName = ServiceName;
    }
#if defined MDNS_DEFAULT_HOST_NAME
    else
    {
        hostName = MDNS_DEFAULT_HOST_NAME;
    }
#endif  // defined MDNS_DEFAULT_HOST_NAME
    // Register the hostname with each descriptor
    if(hostName)
    {
        mDNSHostRegister(hostName, desc);
    }

    /* Initialize MDNS-Responder by opening up Multicast-UDP-Socket */
    desc->mHostCtx.common.state = MDNS_STATE_INIT;
    desc->mDNS_socket = UDPOpenServer(IP_ADDRESS_TYPE_IPV4, MDNS_PORT, 0);

    {
        // Register an RX MAC fitler for the IP multicast group 224.0.0.251,
        // which is mapped to 01:00:5E:00:00:FB
        MAC_ADDR mcast_addr = { {0x01, 0x00, 0x5E, 0x00, 0x00, 0xFB} };
        MACSetRXHashTableEntry(pNetIf->hIfMac, mcast_addr);
    }


    mDNSInitCount++;

    return true;
}

void mDNSDeinitialize(const TCPIP_STACK_MODULE_CTRL* const stackCtrl)
{
    // if(stackCtrl->stackAction == TCPIP_STACK_ACTION_DEINIT) // stack shut down
    // if(stackCtrl->stackAction == TCPIP_STACK_ACTION_IF_DOWN) // interface down

    // one way or another this interface is going down

    if(stackCtrl->stackAction == TCPIP_STACK_ACTION_DEINIT)
    {   // whole stack is going down
        if(mDNSInitCount > 0)
        {   // we're up and running
            if(--mDNSInitCount == 0)
            {   // all closed
                // release resources
                TCPIP_HEAP_Free(stackCtrl->memH, gDNSdesc);  // free the allocated memory
                gDNSdesc = 0;
            }
        }
    }

}


static void mDNSProcessInternal(mDNSProcessCtx_common *pCtx, DNSDesc_t *pDNSdesc)
{
   bool bIsHost = (((void *) pCtx) == ((void *) &pDNSdesc->mHostCtx));
   // char zeroconf_dbg_msg[256];

   switch(pCtx->state)
   {
      case MDNS_STATE_HOME:

         DEBUG_MDNS_PRINT("MDNS_STATE_HOME: Wrong state \r\n");
         break;

        case MDNS_STATE_NOT_READY:            // SD starts from here. SD only.

         if(pDNSdesc->mHostCtx.common.state != MDNS_STATE_DEFEND)
         {
                /* Multicast DNS is not ready */
                return;
         }
         else
         {
            /* Multicast DNS is ready now */
            pCtx->state = MDNS_STATE_INIT;
            pCtx->time_recorded = 0;
         }

            INFO_MDNS_PRINT("\r\nMDNS_STATE_NOT_READY --> MDNS_STATE_INIT \r\n");
         break;

      case MDNS_STATE_INTF_NOT_CONNECTED:      // HOST starts from here. HOST only.
           if(!_TCPIPStackIsNetLinked(pDNSdesc->mTcpIpNetIf))
               return;
         else
         {
            /* Interface is connected now */
            pCtx->state = MDNS_STATE_IPADDR_NOT_CONFIGURED;
            pCtx->time_recorded = 0;
         }

         // No break. Fall through

        case MDNS_STATE_IPADDR_NOT_CONFIGURED:   // HOST only.
        {
         // Wait until IP addr is configured ...
         if (pDNSdesc->mTcpIpNetIf->netIPAddr.Val == 0)
            break;

         pCtx->state = MDNS_STATE_INIT;
         pCtx->time_recorded = 0;

            INFO_MDNS_PRINT("MDNS_STATE_IPADDR_NOT_CONFIGURED --> MDNS_STATE_INIT \r\n");

         // No break. Fall through
        }

      case MDNS_STATE_INIT:
      {
         /* DEBUG_MDNS_MESG(zeroconf_dbg_msg,"MDNS_STATE_INIT \r\n");
         DEBUG_MDNS_PRINT((char*)zeroconf_dbg_msg); */

         pCtx->bConflictSeenInLastProbe = false;

         switch ( zgzc_wait_for(&(pCtx->random_delay), &(pCtx->event_time), &(pCtx->time_recorded)) )
         {
         case ZGZC_STARTED_WAITING:

            // Need to choose Random time between 0-MDNS_PROBE_WAIT msec

           // pCtx->random_delay = ((LFSRRand()% (MDNS_PROBE_WAIT) * (SYS_TICK_TicksPerSecondGet()/1000)));
           // DEBUG_MDNS_MESG(zeroconf_dbg_msg,"MDNS_PROBE_WAIT Random Delay: %ld ticks\r\n",
           //    pCtx->random_delay);
           // DEBUG_MDNS_PRINT((char*)zeroconf_dbg_msg);

            // Intentional fall-through

         case ZGZC_KEEP_WAITING:

            // Not Completed the delay proposed
            return;
         }

         // Completed the delay required

        /* DEBUG_MDNS_MESG(zeroconf_dbg_msg,"MDNS_PROBE_WAIT Random Delay: %ld ticks" \
            " Completed \r\n",
            pCtx->random_delay);
         DEBUG_MDNS_PRINT((char *)zeroconf_dbg_msg);
        */
         // Clear all counters
         mDNSResetCounters(pCtx, true);

         pCtx->state = MDNS_STATE_PROBE;
            INFO_MDNS_PRINT("MDNS_STATE_INIT --> MDNS_STATE_PROBE \r\n");

         // No break. Fall through
      }

      case MDNS_STATE_PROBE:
      case MDNS_STATE_ANNOUNCE:
      {
         //DEBUG_MDNS_PRINT("MDNS_STATE_PROBE \n");
         // or
         //DEBUG_MDNS_PRINT("MDNS_CLAIM \n");

         if(pCtx->bProbeConflictSeen)
         {
            pCtx->bConflictSeenInLastProbe = true;

            INFO_MDNS_PRINT("Conflict detected. Will rename\r\n");

            /* Conflict with selected name */
            pCtx->state = MDNS_STATE_PROBE;

            // Do not reset nProbeConflictCount if in PROBE state
            mDNSResetCounters(
               pCtx,
               (pCtx->state == MDNS_STATE_PROBE)?false:true
               );

            if ( bIsHost )
            {
               // Rename host name
               mDNSRename(pDNSdesc->mHostCtx.szUserChosenHostName
                         ,++(pDNSdesc->mHostCtx.common.nInstanceId)
                          ,(uint8_t *) pDNSdesc->CONST_STR_local
                          ,pDNSdesc->mHostCtx.szHostName
                          ,MAX_HOST_NAME_SIZE);

               INFO_MDNS_MESG(zeroconf_dbg_msg,"New host name : %s \r\n",pDNSdesc->mHostCtx.szHostName);
               INFO_MDNS_PRINT(zeroconf_dbg_msg);
            }
            else
            {
               // Rename service instance name
               if(pDNSdesc->mSDCtx.sd_auto_rename)
               {
                  mDNSRename(pDNSdesc->mSDCtx.srv_name
                            ,++pDNSdesc->mSDCtx.common.nInstanceId
                            ,pDNSdesc->mSDCtx.srv_type
                            ,pDNSdesc->mSDCtx.sd_qualified_name
                            ,MAX_LABEL_SIZE);

                  /* Reset Multicast-UDP socket */
                  UDPClose(pDNSdesc->mDNS_socket);
                  pDNSdesc->mDNS_socket = INVALID_UDP_SOCKET;
                  mDNSResponder(pDNSdesc);
               }
               else
               {
                  pDNSdesc->mSDCtx.service_registered = 0;

                  pDNSdesc->mSDCtx.used = 0;
                  if ( pDNSdesc->mSDCtx.sd_call_back != NULL)
                  {
                     pDNSdesc->mSDCtx.sd_call_back((char *)pDNSdesc->mSDCtx.srv_name,
                        MDNSD_ERR_CONFLICT,
                        pDNSdesc->mSDCtx.sd_context);
                  }
               }
            }
            break;
         }

SET_PROBE_ANNOUNCE_TIMER:

         switch ( zgzc_wait_for(&(pCtx->random_delay), &(pCtx->event_time), &(pCtx->time_recorded)) )
         {
         case ZGZC_STARTED_WAITING:

            if (pCtx->state == MDNS_STATE_PROBE)
            {
               if (((pCtx->nProbeCount >= MDNS_PROBE_NUM) && !pCtx->bConflictSeenInLastProbe) ||
                  (pCtx->nProbeConflictCount >= MDNS_MAX_PROBE_CONFLICT_NUM))
               {
                  /* Move onto Announce Step */
                  pCtx->state = MDNS_STATE_ANNOUNCE;
                  pCtx->bConflictSeenInLastProbe = false;

                  INFO_MDNS_PRINT("MDNS_STATE_PROBE --> MDNS_STATE_ANNOUNCE \r\n");

                  //Shall we mDNSResetCounters(pCtx, true)?
                  return;
               }
            }
            else
            {
               // We are in MDNS_STATE_ANNOUNCE

               if (pCtx->nClaimCount >= MDNS_ANNOUNCE_NUM)
               {
                  /* Finalize mDNS Host-name, Announced */
                  pCtx->state = MDNS_STATE_DEFEND;

                  if ( bIsHost )
                  {
                     INFO_MDNS_MESG(zeroconf_dbg_msg,"\r\n********* Taken Host-Name: %s ********* \r\n"
                                    ,pDNSdesc->mHostCtx.szHostName);

                     INFO_MDNS_PRINT((char *)zeroconf_dbg_msg);
                     INFO_MDNS_PRINT("MDNS_STATE_ANNOUNCE --> MDNS_STATE_DEFEND \r\n");
                  }
                  else
                  {
                     INFO_MDNS_PRINT("\r\nZeroConf: Service = ");
                     INFO_MDNS_PRINT((char*)pDNSdesc->mSDCtx.sd_qualified_name);
                     INFO_MDNS_PRINT("\r\n");

                     INFO_MDNS_MESG(zeroconf_dbg_msg,"\r\n******** Taken Service-Name: %s ********\r\n",
                        pDNSdesc->mSDCtx.sd_qualified_name);

                     INFO_MDNS_PRINT((char *)zeroconf_dbg_msg);
                     INFO_MDNS_PRINT("MDNS_STATE_ANNOUNCE --> MDNS_STATE_DEFEND \r\n");

                     mDNSSendRR(&pDNSdesc->mResponderCtx.rr_list[QTYPE_PTR_INDEX]
                               ,0
                               ,0x00
                               ,1
                               ,true
                               ,true
                               ,pDNSdesc); // This produces a bad PTR rec for MCHPDEMO.local

                     pDNSdesc->mSDCtx.sd_service_advertised = 1;
                     if (pDNSdesc->mSDCtx.sd_call_back != NULL)
                     {
                        pDNSdesc->mSDCtx.sd_call_back((char *)pDNSdesc->mSDCtx.srv_name
                                                     , MDNSD_SUCCESS
                                                     , pDNSdesc->mSDCtx.sd_context);
                     }
                  }
                  mDNSResetCounters(pCtx, true);

                  return;
               }
            }

            if (pCtx->state == MDNS_STATE_PROBE)
            {
               // Send out Probe packet
               mDNSProbe(pCtx,pDNSdesc);

               pCtx->nProbeCount++;
               pCtx->bConflictSeenInLastProbe = false;

               /* Need to set timeout for MDNS_PROBE_INTERVAL msec */
               if (pCtx->nProbeConflictCount < 9)  // less-than-10 is required to pass Bonjour Conformance test.
               {
                  pCtx->random_delay = ( MDNS_PROBE_INTERVAL * (SYS_TICK_TicksPerSecondGet()/1000));
               }
               else
               {
                  pCtx->random_delay = (SYS_TICK_TicksPerSecondGet());
               }

               //DEBUG_MDNS_MESG(
               //   zeroconf_dbg_msg,"MDNS_PROBE_INTERVAL Delay: %ld ticks (%d)\r\n",
               //   pCtx->random_delay, pCtx->nProbeCount);
               //DEBUG_MDNS_PRINT((char *)zeroconf_dbg_msg);

               return;
            }

            // We are in MDNS_STATE_ANNOUNCE

            /* Announce Name chosen on Local Network */

            mDNSAnnounce(&pDNSdesc->mResponderCtx.rr_list[(bIsHost?QTYPE_A_INDEX:QTYPE_SRV_INDEX)],pDNSdesc);

            pCtx->nClaimCount++;

            // Need to set timeout: ANNOUNCE_WAIT or INTERVAL

            if (pCtx->nClaimCount == 1)
            {
                    /* Setup a delay of MDNS_ANNOUNCE_WAIT before announcing */

               /* Need to wait for time MDNS_ANNOUNCE_WAIT msec */
                    pCtx->random_delay = ( MDNS_ANNOUNCE_WAIT * (SYS_TICK_TicksPerSecondGet()/1000));
            }
            else
            {
               pCtx->random_delay = (MDNS_ANNOUNCE_INTERVAL * (SYS_TICK_TicksPerSecondGet()/1000));
            }

            // Intenional fall-through

         case ZGZC_KEEP_WAITING:

            // Not Completed the delay proposed
            return;
         }

         // Completed the delay required
        // DEBUG_MDNS_MESG(zeroconf_dbg_msg,"Probe/Announce delay completed : %ld ticks\r\n",
         //           pCtx->random_delay);
         //DEBUG_MDNS_PRINT((char *)zeroconf_dbg_msg);

            /* Set the timer for next announce */
         goto SET_PROBE_ANNOUNCE_TIMER;
      }

      case MDNS_STATE_DEFEND:
      {
         //DEBUG_MDNS_PRINT("MDNS_STATE_DEFEND \n");
            /* On detection of Conflict Move back to PROBE step */

            if(pCtx->bLateConflictSeen)
         {
            /* Clear the Flag */
            pCtx->bLateConflictSeen = false;
            INFO_MDNS_PRINT("CONFLICT DETECTED !!! \r\n");
                INFO_MDNS_PRINT("Re-probing the Host-Name because of Conflict \r\n");
            pCtx->state = MDNS_STATE_INIT;
            pCtx->time_recorded = 0;

            INFO_MDNS_PRINT("MDNS_STATE_DEFEND --> MDNS_STATE_INIT \r\n");
            }
         else
               return;
      }

      default:
         break;
   }
}

bool mDNSProcess(TCPIP_NET_IF* pNetIf)
{
    DNSDesc_t *pDNSdesc;


    pDNSdesc = (gDNSdesc + _TCPIPStackNetIx(pNetIf));
    if(!_TCPIPStackIsNetLinked(pDNSdesc->mTcpIpNetIf))
        pDNSdesc->mHostCtx.common.state = MDNS_STATE_INTF_NOT_CONNECTED;

    if(pDNSdesc->mTcpIpNetIf->netIPAddr.Val == 0x00)
    {
        return false;
    }

    if (pDNSdesc->mTcpIpNetIf->netIPAddr.Val != pDNSdesc->mResponderCtx.prev_ipaddr.Val)
    {
        // IP address has been changed outside of Zeroconf.
        // Such change could be due to static IP assignment, or
        // a new dynamic IP lease.
        // Need to restart state-machine

        INFO_MDNS_PRINT("IP-Address change is detected \r\n");
        pDNSdesc->mResponderCtx.prev_ipaddr.Val = pDNSdesc->mTcpIpNetIf->netIPAddr.Val;
        pDNSdesc->mHostCtx.common.state = MDNS_STATE_IPADDR_NOT_CONFIGURED;

        // File in the host RR for the specified interface and mDNS desc
        mDNSFillHostRecord(pDNSdesc);
    }

    /* Poll mDNSResponder to allow it to check for
     * incoming mDNS Quries/Responses */
    mDNSResponder(pDNSdesc);

    if(pDNSdesc->mSDCtx.service_registered)
    {

        // Application has registered some services.
        // We now need to start the service probe/announce/defend process.
        if (pDNSdesc->mHostCtx.common.state != MDNS_STATE_DEFEND)
        {
            pDNSdesc->mSDCtx.common.state = MDNS_STATE_NOT_READY;
        }
        else
        {
            mDNSProcessInternal((mDNSProcessCtx_common *) &pDNSdesc->mSDCtx,pDNSdesc);
        }
    }
    mDNSProcessInternal((mDNSProcessCtx_common *) &pDNSdesc->mHostCtx,pDNSdesc);

    return true;
}



static void mDNSSetAddresses(DNSDesc_t *pDNSdesc)
{
   IP_MULTI_ADDRESS Addr;
   UDP_SOCKET_DCPT Skt;

   /* Open a UDP socket for inbound and outbound transmission
    * Since we expect to only receive multicast packets and
    * only send multicast packets the remote NodeInfo
    * parameter is initialized to Multicast-IP (224.0.0.251)
    * corresponding Multicast MAC-Address (01:00:5E:00:00:FB) */

    Addr.v4Add.v[0] = 0xE0;
    Addr.v4Add.v[1] = 0x0;
    Addr.v4Add.v[2] = 0x0;
    Addr.v4Add.v[3] = 0xFB;

    UDPSocketDcpt[pDNSdesc->mDNS_socket].remotePort=MDNS_PORT;
    Skt = UDPSocketDcpt[pDNSdesc->mDNS_socket];

    ARPIsResolved(pDNSdesc->mTcpIpNetIf
                     ,&Addr.v4Add
                     ,&(((IPV4_PACKET*)Skt.pTxPkt)->remoteMACAddr));

    UDPSetDestinationIPAddress(pDNSdesc->mDNS_socket // Now, destination address needs 2b multicast address
                              ,IP_ADDRESS_TYPE_IPV4
                              ,&Addr);

    Addr.v4Add=pDNSdesc->mTcpIpNetIf->netIPAddr;

    UDPSetSourceIPAddress(pDNSdesc->mDNS_socket
                              ,IP_ADDRESS_TYPE_IPV4
                              ,&Addr);

}

#endif //#if defined (TCPIP_STACK_USE_ZEROCONF_LINK_LOCAL) && defined(TCPIP_STACK_USE_ZEROCONF_MDNS_SD)





