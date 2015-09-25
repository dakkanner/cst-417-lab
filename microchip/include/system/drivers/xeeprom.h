/*******************************************************************************
  External serial data EEPROM Access Defs.

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  XEEPROM.h 
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

#ifndef __XEEPROM_H_
#define __XEEPROM_H_

#include <stdint.h>
#include <stdbool.h>

typedef int XEE_RESULT;
#define     XEE_SUCCESS 0
#define     XEE_ERROR   1

XEE_RESULT  XEEInit(void);
XEE_RESULT  XEEBeginWrite(uint32_t address);
XEE_RESULT  XEEWrite(uint8_t val);
XEE_RESULT  XEEWriteArray(const uint8_t *val, uint32_t wLen);
XEE_RESULT  XEEEndWrite(void);
XEE_RESULT  XEEBeginRead(uint32_t address);
uint8_t     XEERead(void);
XEE_RESULT  XEEReadArray(uint32_t address, uint8_t *buffer, uint32_t length);
XEE_RESULT  XEEEndRead(void);
bool        XEEIsBusy(void);

#endif  // __XEEPROM_H_


