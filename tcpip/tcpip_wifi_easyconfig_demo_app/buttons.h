/* 
 * File:   buttons.h
 * Author: C15226
 *
 * Created on August 24, 2012, 12:02 PM
 */

#ifndef BUTTONS_H
#define	BUTTONS_H

#include <stdint.h>
#include <hardware_profile.h>

/*******************************
Public Interface
********************************/
enum eButtoned_tag
{
    BUTTON_0=1,
    BUTTON_1=2,
    BUTTON_2=4,
    BUTTON_3=8,
    BUTTON_4=16,
    BUTTON_5=32,
    BUTTON_6=64,
    BUTTON_7=128
}  eButtones;

typedef struct hButton_tag
{
   uint16_t timerStarted;
   long starTime;
   long delay; // total time to wait for before evaluating the button group
   uint16_t timer_rollover;
   uint16_t ButtonGroup;
   uint16_t AssertedButtons;
   uint16_t desiredButtonStates;
} hButton_group;

uint16_t SYS_IO_Buttons_Pressed (hButton_group *hBG);
hButton_group SYS_IO_CreateButtonGroup(uint16_t ButtonGroup
                                       ,uint16_t assertCriteria
                                       , long delay);
#endif	/* BUTTONS_H */

