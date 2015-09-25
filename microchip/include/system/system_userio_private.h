/* 
 * File:   system_userio_private.h
 * Author: C15226
 *
 * Created on August 29, 2012, 2:56 PM
 */

#ifndef _SYSTEM_USERIO_PRIVATE_H_
#define	_SYSTEM_USERIO_PRIVATE_H_

#include <stdint.h>
#include <stdbool.h>


// *****************************************************************************
/* Function:
    bool SYS_USERIO_Initialize(const void * const initData )

  Summary:
    initialize the USERIO data elements

  Description:
    This routine is called to initialize the USERIO data elements. This is
 meant to only be invoked by SYS_USERIO framework or the ISP executive loop
 and not directly by the user defined application.

  Precondition:
  None

  Parameters:
    None

  Returns:
    true if success
    false otherwise
*/
bool SYS_USERIO_Initialize(const void * const initData);

void SYS_USERIO_Tasks(void);


#define SYS_USERIO_MAX_BUTTON       4       // Total number of buttons supported
#define SYS_USERIO_DEBOUNCE_TIME    30      // Minimum time in ms to wait. Zero will cause divide by zero error
                                            // legal values are 1-1000


#endif	/* _SYSTEM_USERIO_PRIVATE_H_ */

