/*******************************************************************************
  TCP/IP notifications Header file

  Company:
    Microchip Technology Incorported

  File Name:
   tcpip_notify.h

  Summary:
   TCPIP notifications mechanism header file

  Description:
    This source file contains the internal notifications API
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
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
//DOM-IGNORE-END

#ifndef __TCPIP_NOTIFY_H_
#define __TCPIP_NOTIFY_H_

#include <stdint.h>
#include <stdbool.h>

#include "link_list.h"
#include "tcpip_heap_alloc.h"


// *****************************************************************************
// *****************************************************************************
// Section: API definitions
// *****************************************************************************
// *****************************************************************************



// *****************************************************************************
/*
Function:
    SGL_LIST_NODE*      TCPIP_NotificationAdd(SINGLE_LIST* notifyList, TCPIP_HEAP_HANDLE heapH, size_t nBytes)

  Summary:
    Adds a new notification

  Description:
    Tries to create a new SGL_LIST_NODE and add it to the tail of the notifyList.

  Precondition:
    the module notifyList should have been initialized

  Parameters:
    notifyList  - address of the list where the new entry is to be added    
    heapH       - Heap to be used for adding the new node.
                  This could be module specific.
    nBytes      - size of the entry - module specific

  Returns:
    SGL_LIST_NODE pointer to the created node 
        On Success - Returns a valid pointer
        On Failure - null pointer if memory call failed

  Example:
     None

  Remarks:
    It is up to each module to set the specific data associated with this entry.
    This function only creates a new node and inserts it properly in the notification list.
*/
SGL_LIST_NODE*      TCPIP_NotificationAdd(SINGLE_LIST* notifyList, TCPIP_HEAP_HANDLE heapH, size_t nBytes);


// *****************************************************************************
/*
Function:
    bool      TCPIP_NotificationRemove(SGL_LIST_NODE* node, SINGLE_LIST* notifyList, TCPIP_HEAP_HANDLE heapH)

  Summary:
    Removes a notification 

  Description:
    Tries to remove a SGL_LIST_NODE from the notifyList.

  Precondition:
    the node should have been added to the notifyList with TCPIP_NotificationAdd()

  Parameters:
    node        - node to be deregistered
    heapH       - Heap to be used for freeing up memory
                  This could be module specific.
    notifyList  - address of the list from where the new entry is to be removed    

  Returns:
        true  - for success
        false - if failure

  Example:
     None

  Remarks:
    It is up to each module to remove/free the specific data associated with this entry.
    This function only removes the node from the notification list and then frees the associated memory
*/
bool      TCPIP_NotificationRemove(SGL_LIST_NODE* node, SINGLE_LIST* notifyList, TCPIP_HEAP_HANDLE heapH);

// *****************************************************************************
/*
Function:
    void      TCPIP_NotificationRemoveAll(SINGLE_LIST* notifyList, TCPIP_HEAP_HANDLE heapH)

  Summary:
    Removes all notifications from a list 

  Description:
    Tries to remove all notificationd from the notifyList.

  Precondition:
    the nodes should have been added to the notifyList with TCPIP_NotificationAdd()

  Parameters:
    heapH       - Heap to be used for freeing up memory
                  This could be module specific.
    notifyList  - address of the list from where the new entry is to be removed    

  Returns:
        None

  Example:
     None

  Remarks:
    It is up to each module to remove/free the specific data associated with this entry.
    This function only removes the node from the notification list and then frees the associated memory
*/
void      TCPIP_NotificationRemoveAll(SINGLE_LIST* notifyList, TCPIP_HEAP_HANDLE heapH);

#endif  // __TCPIP_NOTIFY_H_



