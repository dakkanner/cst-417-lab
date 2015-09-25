/* In addition to providing debouce logic, this code also attepts to detect
 the intentions of a user pressing multiple buttons simulatniously. The logic
 accounts for the fact that multiple fingers do not always press and release
 multiple buttons in unison. When the intention of the user is that two or more
 fingers are to press multiple at the same time, what often happends is that
 one finger presses and release one button a few miliseconds ahead of the other.
 Without this logic, it is possible that the button pair would not be recognized
 as a set of simultaniously pressed buttons but a set of individual pressed buttons
 . This happend in example like smtp_demo.c where the user is required to press
 button 0 and button 1 at the same time in order to initiate the SMTP demo. If
 the user does not press each button at exactly the same time (even a few
 milisecondsapart could be problematic), then the surounding logic would not
 recognize a pair of buttons being pressed bu would instead recognize button0
 and then button1. This would result in the logic handler for button1 and button2
 (handlers for two other demos) instead of the single logic handler for SMTP
 */

#include <buttons.h>

uint16_t getTime(void)
{
    static uint16_t i=-1;
    i++; //Ellapsed time. Just fake it for now

    return i;
}




/*******************************
Private Interface
********************************/
uint16_t AssertMatch(hButton_group hBG, uint16_t Button)
{
    // Does the Current Buttons state match the assertion criteria
    switch(Button)
    {
        case BUTTON_0:
            if(BUTTON0_IO == (hBG.desiredButtonStates & Button))
              return 1;      // return true
            else return 0;   // return false
        break;
        case BUTTON_1:
            if(BUTTON1_IO == (hBG.desiredButtonStates & Button))
              return 1;      // return true
            else return 0;   // return false
        break;    
        case BUTTON_2:
            if(BUTTON2_IO == (hBG.desiredButtonStates & Button))
              return 1;      // return true
            else return 0;   // return false
        break;        
        case BUTTON_3:
            if(BUTTON3_IO == (hBG.desiredButtonStates & Button))
              return 1;      // return true
            else return 0;   // return false
        break;
        default:
            return 0;
    }
}



uint16_t SYS_IO_Buttons_Pressed (hButton_group *hBG)
{
      uint16_t returnAssert=0;

      if(hBG->ButtonGroup & BUTTON_0)  // If button 0 is part of the group
      {
         if(AssertMatch(*hBG,BUTTON_0))
         {
             if(!hBG->timerStarted)
             {
                 hBG->starTime=getTime();
                 hBG->timerStarted = 1;
             }
             hBG->AssertedButtons |= BUTTON_0;
         }
      }
      if(hBG->ButtonGroup & BUTTON_1)  // If button 1 is part of the group
      {
         if(AssertMatch(*hBG,BUTTON_1))
         {
             if(!hBG->timerStarted)
             {
                 hBG->starTime=getTime();
                 hBG->timerStarted = 1;
             }
             hBG->AssertedButtons |= BUTTON_1;
         }
      }
      if(hBG->ButtonGroup & BUTTON_2)  // If button 2 is part of the group
      {
         if(AssertMatch(*hBG,BUTTON_2))
         {
             if(!hBG->timerStarted)
             {
                 hBG->starTime=getTime();
                 hBG->timerStarted = 1;
             }
             hBG->AssertedButtons |= BUTTON_2;
         }
      }
      if( hBG->ButtonGroup & BUTTON_3)  // If button 3 is part of the group
      {
         if(AssertMatch(*hBG,BUTTON_3))
         {
             if(!hBG->timerStarted)
             {
                 hBG->starTime=getTime();
                 hBG->timerStarted = 1;
             }
             hBG->AssertedButtons |= BUTTON_3;
         }
      }

      if(!hBG->timerStarted)
          return 0;

   /*   if(hBG.timer_rollover) // If the required time has passed, then the
      {                      // assert conditions never completed. Reset.
         hBG->starTime = getTime(); // Reset the timer
         hBG->timer_rollover=0;
         hBG->timerStarted=0;
         hBG->AssertedButtons=0;
      }
   */


      // If the criteria was met for all buttons of the group
      if(hBG->AssertedButtons == hBG->ButtonGroup)
         returnAssert=1;
      else
      {      
          if( (getTime() - hBG->starTime) < hBG->delay)
          {
             return 0; // Time ellapsed criteria not met.
          }
      }

      hBG->AssertedButtons=0;

      // If we got here then all criteria was met to return a group assert
      hBG->starTime = getTime(); // Reset the timer
      hBG->timer_rollover=0;
      hBG->timerStarted=0;
      hBG->AssertedButtons=0;

      return returnAssert;
}

// assertCriteria is a bit mask indicating the state of each button
// required to assert the button group
hButton_group SYS_IO_CreateButtonGroup(uint16_t ButtonGroup
                                       ,uint16_t assertCriteria
                                       , long delay)
{
   hButton_group hBG;
   uint16_t i;

   hBG.ButtonGroup=ButtonGroup;
   hBG.desiredButtonStates=0;
   
    // Obtain the state values of all buttons
   hBG.AssertedButtons  = 0;

   // Obtain the desired trigger state of the buttons assigned to the button group
   i=ButtonGroup & BUTTON_0;
   if(i==BUTTON_0)// If button 0 is within the group
      hBG.desiredButtonStates  |=  (assertCriteria & BUTTON_0);

   i=ButtonGroup & BUTTON_1;
   if(i==BUTTON_1)// If button 1 is within the group
      hBG.desiredButtonStates  |=  (assertCriteria & BUTTON_1);

   i=ButtonGroup & BUTTON_2;
   if(i==BUTTON_2)// If button 2 is within the group
      hBG.desiredButtonStates  |=  (assertCriteria & BUTTON_2);

   i=ButtonGroup & BUTTON_3;
   if(i==BUTTON_3)// If button 3 is within the group
      hBG.desiredButtonStates  |=  (assertCriteria & BUTTON_3);

    hBG.delay = delay;
    hBG.timer_rollover = 0;
    hBG.timerStarted = 0;
    hBG.starTime = getTime();

    return hBG;
}





