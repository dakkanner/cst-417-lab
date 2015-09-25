/*******************************************************************************
  Simple Network Managment Protocol (SNMPv3) Configuration file

  Summary:
    SNMPv3 configuration file
    
  Description:
    This file contains the SNMPv3 module configuration options
    
*******************************************************************************/

/*******************************************************************************
FileName:   snmpv3_config.h
Copyright © 2011 released Microchip Technology Inc.  All rights
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

#ifndef _SNMPv3_CONFIG_H_
#define _SNMPv3_CONFIG_H_

// Client configuration options

#define SNMPV3_USM_MAX_USER	3

//SNMPv3 User Security Name length
#define USER_SECURITY_NAME_LEN (16)

//User security name length for memory validation
#define SNMPV3_USER_SECURITY_NAME_LEN_MEM_USE (USER_SECURITY_NAME_LEN+1)

//SNMPv3 Authentication Localized passwed key lenegth size
#define AUTH_LOCALIZED_PASSWORD_KEY_LEN	(20)

//SNMPv3 authentication localized Key length for memory validation
#define SNMPV3_AUTH_LOCALIZED_PASSWORD_KEY_LEN_MEM_USE (AUTH_LOCALIZED_PASSWORD_KEY_LEN+1)

// SNMPv3 Privacy Pasword key length size
#define PRIV_LOCALIZED_PASSWORD_KEY_LEN	(20)

//SNMPv3 privacy key length size for memory validation
#define SNMPV3_PRIV_LOCALIZED_PASSWORD_KEY_LEN_MEM_USE (PRIV_LOCALIZED_PASSWORD_KEY_LEN+1)



#define SNMPV3_USER_SECURITY_NAME_DB {"microchip","SnmpAdmin","root"}

#define SNMPV3_USER_AUTH_TYPE_DB	 {SNMPV3_HAMC_MD5,SNMPV3_HMAC_SHA1,SNMPV3_NO_HMAC_AUTH}

#define SNMPV3_USER_AUTH_PASSWD_DB	 {"auth12345","ChandlerUS",""}

#define SNMPV3_USER_PRIV_TYPE_DB	 {SNMPV3_AES_PRIV,SNMPV3_NO_PRIV,SNMPV3_NO_PRIV}

#define SNMPV3_USER_PRIV_PASSWD_DB   {"priv12345","",""}


#define SNMPV3_TRAP_USER_SECURITY_NAME_DB {"microchip","SnmpAdmin","root"}
#define SNMPV3_TRAP_MSG_PROCESS_MODEL_DB {SNMPV3_MSG_PROCESSING_MODEL,SNMPV3_MSG_PROCESSING_MODEL,SNMPV3_MSG_PROCESSING_MODEL}
#define SNMPV3_TRAP_SECURITY_MODEL_TYPE_DB {SNMPV3_USM_SECURITY_MODEL,SNMPV3_USM_SECURITY_MODEL,SNMPV3_USM_SECURITY_MODEL}
#define SNMPV3_TRAP_SECURITY_LEVEL_DB {AUTH_PRIV,AUTH_NO_PRIV,NO_AUTH_NO_PRIV}


#endif  // _SNMPv3_CONFIG_H_
