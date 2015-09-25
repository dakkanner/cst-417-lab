/*******************************************************************************
  FTP Server internal stack API

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  ftp_manager.h 
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

#ifndef __FTP_MANAGER_H_
#define __FTP_MANAGER_H_

/*********************************************************************
 * Function:        bool TCPIP_FTP_ServerInit(const TCPIP_STACK_MODULE_CTRL* const stackData,
             const FTP_MODULE_GONFIG* ftpData)
 *
 * PreCondition:    TCP module is already initialized.
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Initializes internal variables of Telnet
 *
 * Note:
 ********************************************************************/
bool TCPIP_FTP_ServerInit(const TCPIP_STACK_MODULE_CTRL* const stackData,
             const FTP_MODULE_GONFIG* ftpData);

/*********************************************************************
 * Function:        void TCPIP_FTP_DeInit(const TCPIP_STACK_MODULE_CTRL* const stackData)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        DeInitializes internal variables of Telnet
 *
 * Note:
 ********************************************************************/
void TCPIP_FTP_DeInit(const TCPIP_STACK_MODULE_CTRL* const stackData);


/*********************************************************************
 * Function:        bool TCPIP_FTP_Server(TCPIP_NET_IF * pNetIf)
 *
 * PreCondition:    TCPIPFTPServerInit() must already be called.
 *
 * Input:           None
 *
 * Output:          Opened Telnet connections are served.
 *
 * Side Effects:    None
 *
 * Overview:
 *
 * Note:            This function acts as a task (similar to one in
 *                  RTOS).  This function performs its task in
 *                  co-operative manner.  Main application must call
 *                  this function repeatdly to ensure all open
 *                  or new connections are served on time.
 ********************************************************************/
 bool TCPIP_FTP_Server(TCPIP_NET_IF * pNetIf);


#endif  // __FTP_MANAGER_H_

