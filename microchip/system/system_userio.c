/*******************************************************************************
  SYS USERIO Definition

  Company:
    Microchip Technology Incorported

  FileName:
    system_userio.c

  Summary:
    SYS USERIO services Implementation

  Description:
    The SYS USERIO services provides a simple interface to manage the USERIO modules
    on Microchip microcontrollers.  This file Implements the core interface
    routines for the SYS USERIO.
    While building the SYS USERIO from source, ALWAYS use this file in the build.
*******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <system/system_userio.h>
#include "system/system_userio_private.h"

#if defined(SYS_USERIO_ENABLE)


// *****************************************************************************
// *****************************************************************************
// Section: File Scope or Global Constants
// *****************************************************************************
// *****************************************************************************

#define SYS_USERIO_MS_ELLAPSED_TIME_TO_HZ(x) (1000/(x)) // convert time to frequency


// *****************************************************************************
// *****************************************************************************
// Section: Variable Definitions
// *****************************************************************************
// *****************************************************************************
static SYS_USERIO_StateMachine sys_io_sm; // state machine
static SYS_USERIO_DEBOUNCE_T hButtons[SYS_USERIO_MAX_BUTTON];



// *****************************************************************************
// *****************************************************************************
// Section: File scope Function Declarations
// *****************************************************************************
// *****************************************************************************
static void SYS_USERIO_ReadIO(void);


// *****************************************************************************
/* Function:
    bool SYS_USERIO_Initialize( const void * const initData )

  Summary:
    initialize the USERIO data elements

  Description:
    This routine is called to initialize the USERIO data elements.

  Precondition:
  None

  Parameters:
    None

  Returns:
    true if success
    false otherwise
*/
bool SYS_USERIO_Initialize(const void * const initData)
{
    int i;

    for(i=0;i<SYS_USERIO_MAX_BUTTON;i++)
    {
       hButtons[i].duration =  SYS_USERIO_DEBOUNCE_TIME;
       hButtons[i].startTick=0;
       hButtons[i].endTick=0;
       hButtons[i].timerActive=false;
    }

    sys_io_sm.b0=sys_io_sm.b1=sys_io_sm.b2=sys_io_sm.b3 = SYS_USERIO_BUTTON_DEASSERTED;

    sys_io_sm.led0=sys_io_sm.led1=sys_io_sm.led2=sys_io_sm.led3=-1;
    sys_io_sm.led4=sys_io_sm.led5=sys_io_sm.led6=sys_io_sm.led7=-1;


    return true;
}




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
void SYS_USERIO_SetLED(SYS_USERIO_LED_ID dev, SYS_USERIO_LED_STATE state)
{
    switch(dev)
    {
        case SYS_USERIO_LED_0:
          LED0_IO=state;
        break;
        case SYS_USERIO_LED_1:
          LED1_IO=state;
        break;
        case SYS_USERIO_LED_2:
          LED2_IO=state;
        break;
        case SYS_USERIO_LED_3:
          LED3_IO=state;
        break;
        case SYS_USERIO_LED_4:
          LED4_IO=state;
        break;
        case SYS_USERIO_LED_5:
          LED5_IO=state;
        break;
        case SYS_USERIO_LED_6:
          LED6_IO=state;
        break;
        case SYS_USERIO_LED_7:
          LED7_IO=state;
        break;
        default:
            break;
    }
}


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
  SYS_Initialize() and SYS_USERIO_Initialize() Should have already been called

  Parameters:
 None

  Returns:
    An SYS_USERIO_BUTTON_ID representing a bitmap of all button states.
*/
SYS_USERIO_BUTTON_ID SYS_USERIO_ButtonGetBitMap(void)
{
    SYS_USERIO_BUTTON_ID bitmap;

    bitmap = sys_io_sm.b0;
    bitmap |= sys_io_sm.b1<<1;
    bitmap |= sys_io_sm.b2<<2;
    bitmap |= sys_io_sm.b3<<3;
    return bitmap;
}





// *****************************************************************************
/* Function:
    bool SYS_USERIO_ButtonGet(int32_t buttons,SYS_USERIO_BUTTON_STATE assertValue)

  Summary:
 Test the state of one or more buttons against a specified state.

  Description:
 If buttons is a single button ID , then the state of that button is compared
 against assertValue - which is SYS_USERIO_BUTTON_ASSERTED or SYS_USERIO_BUTTON_DEASSERTED. 
 If buttons represent multiple buttons ORed together, then all buttons are compared
 to assertValue.


  Precondition:
  SYS_Initialize() and SYS_USERIO_Initialize() Should have already been called

  Additional Notes:
  There are only actually four buttons that are supported by the demo boards and
  for that reason this code only evaluate four bits of 'bitmap'. If more than
  4 buttons are actually reuired, the code within SYS_USERIO_ButtonGetBitMap()
  will have to be updated and the additional bits within the 'bitmap' variable
  below will have to be evualated.

  Parameters:
 buttons one or more button ID identified within SYS_USERIO_BUTTON_ID
 and ORed together.
 assertValue - A value of SYS_USERIO_BUTTON_ASSERTED or SYS_USERIO_BUTTON_DEASSERTED


  Returns:
    If one or more buttons fail to match assertValue then false is returned.
 Otherwise true is returned.
*/
bool SYS_USERIO_ButtonGet(int32_t buttons,SYS_USERIO_BUTTON_STATE assertValue)
{

    int32_t bitmap;
    int32_t a,b;

    bitmap = SYS_USERIO_ButtonGetBitMap();

    // bitmap holds the hardware states of the individual buttons.
    // buttons contains a set of buttonID ORed together. Since the logical
    // state of bitmap could be inverted due to the inverse logic of buttons,
    // the AND comparison could run into a case where it is comparing
    // inverted patterns. The value assertValue helps to identify referse logic
    
    // Check if any button is pressed
    if(buttons==SYS_USERIO_BUTTON_ANY)
    {
        if(assertValue==0) // Using reverse logic for button states
        {
           a= ~bitmap;     // Flip all bits for normal logic
           a &= 0x000f;    // Isolate the 4 button bit vals
           b= a & buttons; // Isolate the one that are 1 if any
           if(b)           // Are any of them 1?
             return true;
           else return false;
        }
        else
        {
           a= bitmap;
           a &= 0x000f;
           if(a)
             return true;
           else return false;
        }

    }
    else // Check only for a specific button
    {
        if(assertValue==0) // Using reverse logic for button states
        {
           if( ((~bitmap) & buttons ) == buttons)
             return true;
           else return false;
        }
        if(bitmap & buttons )
                return true;
        else return false;
    }
}



// *****************************************************************************
/* Function:
    SYS_USERIO_BUTTON_STATE SYS_USERIO_ButtonsTest(SYS_USERIO_BUTTON_ID dev)

  Summary:
 Evaluates the state of a specific button

  Description:
 Used to obtain the state value of a specified button.

  Precondition:
  SYS_Initialize() and SYS_USERIO_Initialize() Should have already been called

  Parameters:
 SYS_USERIO_BUTTON_ID dev  - A button ID of type SYS_USERIO_BUTTON_ID

  Returns:
    The state value of the device. The meaning of the returned value is
 determined by the circuitry of the board set and can vary based on design.
*/
SYS_USERIO_BUTTON_STATE SYS_USERIO_ButtonsTest(SYS_USERIO_BUTTON_ID dev)
{

    switch(dev)
    {
        case SYS_USERIO_BUTTON_0:
          return (SYS_USERIO_BUTTON_STATE)sys_io_sm.b0;
        break;
        case SYS_USERIO_BUTTON_1:
          return (SYS_USERIO_BUTTON_STATE)sys_io_sm.b1;
        break;
        case SYS_USERIO_BUTTON_2:
          return (SYS_USERIO_BUTTON_STATE)sys_io_sm.b2;
        break;
        case SYS_USERIO_BUTTON_3:
          return (SYS_USERIO_BUTTON_STATE)sys_io_sm.b3;
        break;
        default:
            return SYS_USERIO_BUTTON_DEASSERTED;
    }
}


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
  SYS_Initialize() and SYS_USERIO_Initialize() Should have already been called

  Parameters:
   SYS_USERIO_DEBOUNCE_T *handle - A device handle.
  uint32_t curVal - Last value read from the device.

  Returns:
    The state value of the device. The meaning of the returned value is
 determined by the circuitry of the board and can vary based on design.
 SYS_USERIO_BUSY if the debounce logic is still evaluating the state of device.
*/
int32_t SYS_USERIO_DeviceDebounce(SYS_USERIO_DEBOUNCE_T *handle, uint32_t curVal)
{
    if(handle->timerActive==false)
    {
       handle->timerActive = true;
       handle->prevValue   = curVal;
       handle->startTick   = SYS_TICK_Get();
    }

    if ((SYS_TICK_Get() - handle->startTick) >= (SYS_TICK_TicksPerSecondGet()
                         / SYS_USERIO_MS_ELLAPSED_TIME_TO_HZ(handle->duration)) )
    {
        handle->timerActive=false;
        if(handle->prevValue == curVal)
          return handle->prevValue;
        else
          return curVal; // Return the assert value
    }
    return SYS_USERIO_BUSY;  // Busy
}


// *****************************************************************************
/* Function:
    static void SYS_USERIO_ReadIO(void)

 Summary:
 Provides deounce logic for a device state

 Description:
 Reads the state values of the supported I/O. This is only meant to be called
 by the ISP executive routine and not by the user application.
 It should be called at the highest frequency tolerable in order to obtain
 the fastest response from the I/O

  Precondition:
  SYS_Initialize() and SYS_USERIO_Initialize() Should have already been called

  Parameters:
 None

  Returns:
 None
*/
static void SYS_USERIO_ReadIO(void)
{
    int32_t val;

    val = SYS_USERIO_DeviceDebounce(&hButtons[0], BUTTON0_IO);
    if(val != SYS_USERIO_BUSY) sys_io_sm.b0=val;

    val = SYS_USERIO_DeviceDebounce(&hButtons[1], BUTTON1_IO);
    if(val != SYS_USERIO_BUSY) sys_io_sm.b1=val;

    val = SYS_USERIO_DeviceDebounce(&hButtons[2], BUTTON2_IO);
    if(val != SYS_USERIO_BUSY) sys_io_sm.b2=val;

    val = SYS_USERIO_DeviceDebounce(&hButtons[3], BUTTON3_IO);
    if(val != SYS_USERIO_BUSY) sys_io_sm.b3=val;

    sys_io_sm.led0 = LED0_IO;
    sys_io_sm.led1 = LED1_IO;
    sys_io_sm.led2 = LED2_IO;
    sys_io_sm.led3 = LED3_IO;
    sys_io_sm.led4 = LED4_IO;
    sys_io_sm.led5 = LED5_IO;
    sys_io_sm.led6 = LED6_IO;
    sys_io_sm.led7 = LED7_IO;
}

void SYS_USERIO_Tasks(void)
{

   SYS_USERIO_ReadIO();

}

#endif  // defined(SYS_USERIO_ENABLE)

