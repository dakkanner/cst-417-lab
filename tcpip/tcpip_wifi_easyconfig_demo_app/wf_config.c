/*******************************************************************************
  MRF24W Driver Customization

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Provides access to MRF24W WiFi controller
    -Reference: MRF24W Data sheet, IEEE 802.11 Standard
*******************************************************************************/

/*******************************************************************************
FileName:  WF_Config.c 
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

#include "hardware_config.h"

#if defined(TCPIP_IF_MRF24W)

/*==========================================================================*/
/*                                  INCLUDES                                */
/*==========================================================================*/
#include "tcpip/tcpip.h"
#include "wifi/wf_debug_output.h"

#if defined ( EZ_CONFIG_SCAN )
#include "../../microchip/tcpip/wifi/wf_easy_config.h"
#endif /* EZ_CONFIG_SCAN */

#if WF_DEFAULT_NETWORK_TYPE == WF_NETWORK_TYPE_SOFT_AP
extern UINT8 g_scan_done;

#endif

/*==========================================================================*/
/*                                  DEFINES                                 */
/*==========================================================================*/


/*****************************************************************************
 * FUNCTION: WF_ProcessEvent
*
* RETURNS:  None
*
* PARAMS:   event      -- event that occurred
*           eventInfo  -- additional information about the event.  Not all events
*                         have associated info, in which case this value will be
*                         set to WF_NO_ADDITIONAL_INFO (0xff)
*
*  NOTES:   Ror events that the application is not interested in simply leave the
*           case statement empty.
*
*           Customize this function as needed for your application.
 *****************************************************************************/
void WF_ProcessEvent(uint8_t event, uint16_t eventInfo)
{
    char buf[8];
  
    switch (event)
    {
        /*--------------------------------------*/
        case WF_EVENT_CONNECTION_SUCCESSFUL:
        /*--------------------------------------*/   
            SYS_CONSOLE_MESSAGE("Event: Connection Successful\r\n"); 
            #if defined(SYS_CONSOLE_ENABLE)
                WF_OutputConnectionContext();
            #endif
            break;
        
        /*--------------------------------------*/            
        case WF_EVENT_CONNECTION_FAILED:
        case WF_EVENT_CONNECTION_TEMPORARILY_LOST:
        case WF_EVENT_CONNECTION_PERMANENTLY_LOST:            
        /*--------------------------------------*/
            #if defined(SYS_CONSOLE_ENABLE)
            WF_OutputConnectionDebugMsg(event, eventInfo);
            #endif
            break; 
            
        /*--------------------------------------*/    
        case WF_EVENT_CONNECTION_REESTABLISHED:
        /*--------------------------------------*/
            SYS_CONSOLE_MESSAGE("Event: Connection Reestablished\r\n");
            #if defined(SYS_CONSOLE_ENABLE)
                WF_OutputConnectionContext();
            #endif
            
            break;
            
        /*--------------------------------------*/    
        case WF_EVENT_SCAN_RESULTS_READY:
        /*--------------------------------------*/  
            SYS_CONSOLE_MESSAGE("Event: Scan Results Ready,");
            sprintf(buf, "%d", eventInfo);
            SYS_CONSOLE_MESSAGE(buf);
            SYS_CONSOLE_MESSAGE("results\r\n");
            #if defined ( EZ_CONFIG_SCAN )
            WFScanEventHandler(eventInfo);
			#endif /* EZ_CONFIG_SCAN */

			#if (WF_DEFAULT_NETWORK_TYPE == WF_NETWORK_TYPE_SOFT_AP)
            g_scan_done = 1;    // WF_PRESCAN          
            #endif 
            break;

        /*--------------------------------------*/              
        case WF_EVENT_INVALID_WPS_PIN:
        /*--------------------------------------*/  
            SYS_CONSOLE_MESSAGE("Invalid WPS PIN; connection failed\r\n");
            break;
            
        case WF_EVENT_SOFT_AP:
            {
                tMgmtIndicateSoftAPEvent *p_softApEvent = WF_GetSoftApEventInfo();

                uint8_t *addr = p_softApEvent->address;
                uint8_t i;

                SYS_CONSOLE_MESSAGE("Event: SoftAP, ");
                if (p_softApEvent->event == SOFTAP_EVENT_CONNECTED)
                {
                    SYS_CONSOLE_MESSAGE("Connected, ");
                } 
                else if (p_softApEvent->event == SOFTAP_EVENT_DISCONNECTED)
                {
                    SYS_CONSOLE_MESSAGE("Disconnected, ");
                    if (p_softApEvent->reason == SOFTAP_EVENT_LINK_LOST)
                    {
                         SYS_CONSOLE_MESSAGE("Link Lost, ");
                    }
                    else if (p_softApEvent->reason == SOFTAP_EVENT_RECEIVED_DEAUTH)
                    {
                        SYS_CONSOLE_MESSAGE("ReceivedDeauth,");
                    }
                }

                for (i = 0; i < 6; ++i)
                {
                    if (i < 5)
                    {
                        sprintf(buf, "%2x:", addr[i]);
                    }
                    else
                    {
                        sprintf(buf, "%2x\r\n", addr[i]);
                    }
                    SYS_CONSOLE_MESSAGE(buf);
                }
            }
            break;

         case WF_EVENT_DISCONNECT_DONE:
            //SYS_CONSOLE_MESSAGE("Event: Disconnect complete\r\n");
            break;

        /*--------------------------------------*/
        case WF_EVENT_ERROR:
        /*--------------------------------------*/
            SYS_CONSOLE_PRINT("Event: WiFi error occurred (%d) See t_mgmtErrors and t_wfErrorEvents", eventInfo);
            break;


        default:
            SYS_ASSERT(false, "Unknown WiFi Event");  /* unknown event */
            break;
    }        
}    
  
 

#endif /* TCPIP_IF_MRF24W */

