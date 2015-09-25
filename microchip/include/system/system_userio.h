/* 
 * File:   system_userio.h
 * Author: C15226
 *
 * Created on August 29, 2012, 2:56 PM
 */

#ifndef _SYSTEM_USERIO_H_
#define	_SYSTEM_USERIO_H_

#include <hardware_config.h>
#include <stdint.h>
#include <stdbool.h>
#include "system/system_services.h"

// Asserted state of a LED
// User may need to change to match their board logic
typedef enum
{
    SYS_USERIO_LED_DEASSERTED,
    SYS_USERIO_LED_ASSERTED
}SYS_USERIO_LED_STATE;

// Asserted state of a button
// User may need to change to match their board logic
typedef enum
{
    SYS_USERIO_BUTTON_ASSERTED,
    SYS_USERIO_BUTTON_DEASSERTED,
}SYS_USERIO_BUTTON_STATE;


typedef enum
{
    SYS_USERIO_BUTTON_NONE  = 0,
    SYS_USERIO_BUTTON_0     = 0x1,
    SYS_USERIO_BUTTON_1     = 0x2,
    SYS_USERIO_BUTTON_2     = 0x4,
    SYS_USERIO_BUTTON_3     = 0x8,
    SYS_USERIO_BUTTON_4     = 0x10, 
    SYS_USERIO_BUTTON_5     = 0x20,
    SYS_USERIO_BUTTON_6     = 0x40,
    SYS_USERIO_BUTTON_7     = 0x80,
    SYS_USERIO_BUTTON_8     = 0x100,
    SYS_USERIO_BUTTON_9     = 0x200,
    SYS_USERIO_BUTTON_10    = 0x400,
    SYS_USERIO_BUTTON_11    = 0x800,
    SYS_USERIO_BUTTON_12    = 0x1000,
    SYS_USERIO_BUTTON_13    = 0x2000,
    SYS_USERIO_BUTTON_14    = 0x4000,
    SYS_USERIO_BUTTON_15    = 0x8000,
    SYS_USERIO_BUTTON_16    = 0x10000,
    SYS_USERIO_BUTTON_17    = 0x20000,
    SYS_USERIO_BUTTON_18    = 0x40000,
    SYS_USERIO_BUTTON_19    = 0x80000,
    SYS_USERIO_BUTTON_20    = 0x100000,
    SYS_USERIO_BUTTON_21    = 0x200000,
    SYS_USERIO_BUTTON_23    = 0x400000,
    SYS_USERIO_BUTTON_24    = 0x800000,
    SYS_USERIO_BUTTON_25    = 0x1000000,
    SYS_USERIO_BUTTON_26    = 0x2000000,
    SYS_USERIO_BUTTON_27    = 0x4000000,
    SYS_USERIO_BUTTON_28    = 0x8000000,
    SYS_USERIO_BUTTON_29    = 0x10000000,
    SYS_USERIO_BUTTON_30    = 0x20000000,
    SYS_USERIO_BUTTON_31    = 0x40000000,
    SYS_USERIO_BUTTON_ANY   = 0xFFFFFFFF

}SYS_USERIO_BUTTON_ID;


typedef enum
{
    SYS_USERIO_LED_NONE  = 0,
    SYS_USERIO_LED_0     = 0x1,
    SYS_USERIO_LED_1     = 0x2,
    SYS_USERIO_LED_2     = 0x4,
    SYS_USERIO_LED_3     = 0x8,
    SYS_USERIO_LED_4     = 0x10,
    SYS_USERIO_LED_5     = 0x20,
    SYS_USERIO_LED_6     = 0x40,
    SYS_USERIO_LED_7     = 0x80,
    SYS_USERIO_LED_8     = 0x100,
    SYS_USERIO_LED_9     = 0x200,
    SYS_USERIO_LED_10    = 0x400,
    SYS_USERIO_LED_11    = 0x800,
    SYS_USERIO_LED_12    = 0x1000,
    SYS_USERIO_LED_13    = 0x2000,
    SYS_USERIO_LED_14    = 0x4000,
    SYS_USERIO_LED_15    = 0x8000,
    SYS_USERIO_LED_16    = 0x10000,
    SYS_USERIO_LED_17    = 0x20000,
    SYS_USERIO_LED_18    = 0x40000,
    SYS_USERIO_LED_19    = 0x80000,
    SYS_USERIO_LED_20    = 0x100000,
    SYS_USERIO_LED_21    = 0x200000,
    SYS_USERIO_LED_23    = 0x400000,
    SYS_USERIO_LED_24    = 0x800000,
    SYS_USERIO_LED_25    = 0x1000000,
    SYS_USERIO_LED_26    = 0x2000000,
    SYS_USERIO_LED_27    = 0x4000000,
    SYS_USERIO_LED_28    = 0x8000000,
    SYS_USERIO_LED_29    = 0x10000000,
    SYS_USERIO_LED_30    = 0x20000000,
    SYS_USERIO_LED_31    = 0x40000000
} SYS_USERIO_LED_ID;



#define SYS_USERIO_BUSY -1 // A function is currently busy. Debounce logic is the main user.

typedef struct
{
    int32_t prevValue;
    int32_t timerActive;
    int32_t duration; // In milliseconds
    SYS_TICK startTick;
    SYS_TICK endTick;
}SYS_USERIO_DEBOUNCE_T;


typedef struct SYS_USER_StateMachine_tag
{
    int32_t b0,b1,b2,b3;
    int32_t led0,led1,led2,led3,led4,led5,led6,led7;
} SYS_USERIO_StateMachine;


// *****************************************************************************
/* Function:
    bool SYS_USERIO_ButtonGet(int32_t buttons, SYS_USERIO_BUTTON_STATE assertValue)

  Summary:
 Test the state of one or more buttons against a specified state.

  Description:
 If buttons is a single button ID , then the state of that button is compared
 against assertValue - which is SYS_USERIO_BUTTON_ASSERTED or SYS_USERIO_BUTTON_DEASSERTED.
 If buttons represent multiple buttons ORed together, then all buttons compared with assertValue.

  Precondition:
  SYS_Initialize() should have already been called

  Parameters:
 buttons one or more button ID identified within SYS_USERIO_BUTTON_ID
 and ORed together.
 assertValue - A value of SYS_USERIO_BUTTON_ASSERTED or SYS_USERIO_BUTTON_DEASSERTED

  Returns:
    If one or more buttons fail to match assertValue then false is returned.
 Otherwise true is returned.
*/
bool SYS_USERIO_ButtonGet(int32_t buttons, SYS_USERIO_BUTTON_STATE assertValue);


// *****************************************************************************
/* Function:
    SYS_USERIO_BUTTON_ID SYS_USERIO_ButtonGetBitMap( void )

  Summary:
    Returns a bit map representing all button states

  Description:
    Returns a bit map representing all button states. Each bit position
 within the mask is associated with a specific button, LSB=SYS_USERIO_BUTTON_0
 and bit MSB=SYS_USERIO_BUTTON_31

  Precondition:
  SYS_Initialize() and should have already been called

  Parameters:
 None

  Returns:
    An SYS_USERIO_BUTTON_ID representing a bitmap of all button states.
    SYS_USERIO_BUTTON_NONE if no button pressed.
*/
SYS_USERIO_BUTTON_ID SYS_USERIO_ButtonGetBitMap(void);



// *****************************************************************************
/* Function:
    SYS_USERIO_BUTTON_STATE SYS_USERIO_ButtonsTest(SYS_USERIO_BUTTON_ID dev)

  Summary:
 Evaluates a button state.

  Description:
 Used to obtain the state value of a specified button.

  Precondition:
  SYS_Initialize() and should have already been called

  Parameters:
 SYS_USERIO_BUTTON_ID dev  - A button ID of type SYS_USERIO_BUTTON_ID

  Returns:
    The state value of the device. 
    The meaning of the returned value is determined by the circuitry
    of the board set and can vary based on design.
*/
SYS_USERIO_BUTTON_STATE SYS_USERIO_ButtonsTest(SYS_USERIO_BUTTON_ID dev);


// *****************************************************************************
/* Function:
    void SYS_USERIO_SetLED(SYS_USERIO_LED_ID dev, SYS_USERIO_LED_STATE state)

  Summary:
 Sets dev to state.

  Description:
    Sets the current state of the specified LED to the specified value.

  Precondition:
  None

  Parameters:
    dev - An LED ID member defined from SYS_USERIO_LED_ID type
    stat - A  value of SYS_USERIO_LED_ASSERTED or SYS_USERIO_LED_DEASSERTED representing the desired
                   state of the LED.
  Returns:
    None
*/
void SYS_USERIO_SetLED(SYS_USERIO_LED_ID dev, SYS_USERIO_LED_STATE state);

// *****************************************************************************
/* Function:
    int32_t SYS_USERIO_DeviceDebounce(SYS_USERIO_DEBOUNCE_T *handle, uint32_t curVal))

  Summary:
 Provides deounce logic for a device state

  Description:
 This function can be called as often a required, each time providing the last
 state obtained from reading the device. The debounce logic will use the state
 value to determine if the device experienced a state change. Rapid changes over
 a specified amount of time are evaluated as a single state change.

  Precondition:
  SYS_Initialize() and should have already been called

  Parameters:
   SYS_USERIO_DEBOUNCE_T *handle - A device handle.
  uint32_t curVal - Last value read from the device.

  Returns:
    The state value of the device. The meaning of the returned value is
 determined by the circuitry of the board and can vary based on design.
 SYS_USERIO_BUSY if the debounce logic is still evaluating the state of device.
*/
int32_t SYS_USERIO_DeviceDebounce(SYS_USERIO_DEBOUNCE_T *handle, uint32_t curVal);



#endif	/* _SYSTEM_USERIO_H_ */

