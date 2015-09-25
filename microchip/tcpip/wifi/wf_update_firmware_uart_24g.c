/*******************************************************************************
  MRF24W Driver Medium Access Control (MAC) Layer

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Provides access to MRF24W WiFi controller
    -Reference: MRF24W Data sheet, IEEE 802.11 Standard
*******************************************************************************/

/*******************************************************************************
FileName:  wf_update_firmware_uart_24g.c 
Copyright © 2012 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software foradditional information regarding your rights and obligations.

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

/*
*********************************************************************************************************
*                                           INCLUDES                               
*********************************************************************************************************
*/


#include "tcpip_private.h"

#include "tcpip_mac_private.h"
#include "tcpip/tcpip_mac.h"
#include "tcpip/tcpip_mac_object.h"

#if defined(TCPIP_IF_MRF24W) 
#if defined(WF_UPDATE_FIRMWARE_UART_24G)

#define MAX_USER_RESPONSE_LEN   (20u)

#define XMODEM_SOH      0x01u
#define XMODEM_EOT      0x04u
#define XMODEM_ACK      0x06u
#define XMODEM_NAK      0x15u
#define XMODEM_CAN      0x18u
#define XMODEM_BLOCK_LEN 128u


extern void dbgPrintf( const char* format, ... ) ;
extern uint32_t ImageUpdate_Addr;
extern uint32_t ImageUpdate_Checksum;
extern uint32_t ImageUpdate_Size;
extern void AutoUpdate_Initialize(void);
extern void AutoUpdate_Completed(void);
extern void AutoUpdate_Restore(void);
void dbgPrintf( const char* format, ... )
{
}
static void XMODEM_SendToModule_subAPI(BYTE *buf)
{
    int i;
    uint8_t buf_module[36];
    buf_module[0]=(ImageUpdate_Addr&0x00FF0000)>>16;
    buf_module[1]=(ImageUpdate_Addr&0x0000FF00)>>8;
    buf_module[2]=(ImageUpdate_Addr&0xFF);
    buf_module[3]=32;
    for(i=0;i<32;i++) buf_module[i+4]=buf[i];
    SendSetParamMsg(PARAM_FLASH_WRITE, buf_module, 36);
    ImageUpdate_Addr += 32;
}


static void XMODEM_SendToModule(BYTE *xmodm_buf)
{
    int i;
    
    //  1. Calculate checksum
    for(i=0;i<128;i++)
    {
        if((ImageUpdate_Size % 4) == 0) ImageUpdate_Checksum += (uint32_t)xmodm_buf[i]<<24;
        if((ImageUpdate_Size % 4) == 1) ImageUpdate_Checksum += (uint32_t)xmodm_buf[i]<<16;
        if((ImageUpdate_Size % 4) == 2) ImageUpdate_Checksum += (uint32_t)xmodm_buf[i]<<8;
        if((ImageUpdate_Size % 4) == 3) ImageUpdate_Checksum += (uint32_t)xmodm_buf[i];
        ImageUpdate_Size ++;
    }
    // 2. send 128 bytes                
    XMODEM_SendToModule_subAPI(&xmodm_buf[0]);
    XMODEM_SendToModule_subAPI(&xmodm_buf[32]);
    XMODEM_SendToModule_subAPI(&xmodm_buf[64]);
    XMODEM_SendToModule_subAPI(&xmodm_buf[96]);
}
static BYTE tempData[XMODEM_BLOCK_LEN];

bool    WF_FirmwareUpdate_Uart_24G(void)
{
    enum SM_FIRMWARE_UPDATE
    {
        SM_FIRMWARE_UPDATE_SOH,
        SM_FIRMWARE_UPDATE_BLOCK,
        SM_FIRMWARE_UPDATE_BLOCK_CMP,
        SM_FIRMWARE_UPDATE_DATA,
        SM_FIRMWARE_UPDATE_CHECKSUM,
        SM_FIRMWARE_UPDATE_FINISH,
    } state;

    BYTE c;
   // MPFS_HANDLE handle;
    BOOL lbDone;
    BYTE blockLen=0;
    BOOL lResult = FALSE;
    BYTE BlockNumber=0, preBlockNum=0;
    BYTE checksum=0;
    
    SYS_TICK lastTick;
    SYS_TICK currentTick;
    state = SM_FIRMWARE_UPDATE_SOH;
    lbDone = FALSE;

    TCPIP_NET_HANDLE netH;
    netH = TCPIP_STACK_NetHandle("MRF24W");
    
    if( BUTTON3_IO == 1u) return FALSE;    
    printf("\n\rPress S2 (on Explorer16) to start the update.\n\r");
    while(BUTTON2_IO == 1u);
	printf("1\n");
    WF_Init(netH);//MACInit();
    printf("2\n");
    SYS_TICK_MsDelay(100);
	printf("3\n");
    AutoUpdate_Initialize();
    printf("I am ready, Please transfer firmware patch by XMODEM.\r\n"); 
    printf("If you press S3(On Explorwe16), I will stop update, and restore back to previous firmware.\r\n"); 
    lastTick = SYS_TICK_Get();
    do
    {
        currentTick = SYS_TICK_Get();
        if ( currentTick - lastTick >= (SYS_TICK_TicksPerSecondGet()*2) )
        {
            lastTick = SYS_TICK_Get();
            //todo jw  //while(BusyUART());
            _SYS_CONSOLE_PUTC(XMODEM_NAK); //WriteUART(XMODEM_NAK);
        }

    } while(!_SYS_CONSOLE_DATA_RDY()); //(!DataRdyUART());

    
    while(!lbDone)
    {
        if (BUTTON3_IO == 0u)   // If you want to cancel AutoUpdate, please press S3
        {
            printf("You press S3 button, revert begin...\r\n");
            AutoUpdate_Restore();
            printf("revert done\r\n");
            return FALSE;
        }
        if(_SYS_CONSOLE_DATA_RDY()) //(DataRdyUART())
        {
            c = _SYS_CONSOLE_GETC();//ReadUART();
            lastTick = SYS_TICK_Get();
        }
        else
        {
            // put some timeout to make sure  that we do not wait forever.
             currentTick = SYS_TICK_Get();
            if ( currentTick - lastTick >= (SYS_TICK_TicksPerSecondGet()*10) )
            {
                //if time out, copy old patch image from bank2 to bank1
                printf("timeout, revert begin...\r\n");
                AutoUpdate_Restore();
                printf("revert done\r\n");
                return FALSE;
            }
            continue;
        }
        //dbgPrintf("(%02x) ",c); 
        switch(state)
        {
        case SM_FIRMWARE_UPDATE_SOH:
            if(c == XMODEM_SOH)
            {
                state = SM_FIRMWARE_UPDATE_BLOCK;
                dbgPrintf("\r\n! ");
                checksum = c;
                lResult = TRUE;
            }
            else if ( c == XMODEM_EOT )
            {
                state = SM_FIRMWARE_UPDATE_FINISH;
                
                // todo jw://while(BusyUART());
                _SYS_CONSOLE_PUTC(XMODEM_ACK); //WriteUART(XMODEM_ACK);
                lbDone = TRUE;
            }  
            else
            {
                dbgPrintf("\n!error\n");
                while(1);
            }
            break;
        case SM_FIRMWARE_UPDATE_BLOCK:
            BlockNumber = c;
            dbgPrintf("BLK=%d ",BlockNumber);         
            checksum += c;
            state = SM_FIRMWARE_UPDATE_BLOCK_CMP;
            break;

        case SM_FIRMWARE_UPDATE_BLOCK_CMP:
            dbgPrintf("%d ",c);
            dbgPrintf("@:");
            //Judge: Is it correct ?
            if(c != (BlockNumber ^ 0xFF))
            {
                lResult = FALSE;
                dbgPrintf("\nBLOCK_CMP err: %x,%x\n", c, BlockNumber ^ 0xFF );
            }
            else 
            {
                if((BYTE)(preBlockNum+1) != BlockNumber)
                {
                    lResult = FALSE;
                    dbgPrintf("\nBLOCK  err %x %x\n",preBlockNum+1,BlockNumber);
                }
            }
            checksum += c;
            blockLen = 0;
            state = SM_FIRMWARE_UPDATE_DATA;
            break;
        case SM_FIRMWARE_UPDATE_DATA:
            // Buffer block data until it is over.
            tempData[blockLen++] = c;
            if ( blockLen == XMODEM_BLOCK_LEN )
            {
                state = SM_FIRMWARE_UPDATE_CHECKSUM;
            }
            checksum += c;
            
            break;
        case SM_FIRMWARE_UPDATE_CHECKSUM:
            dbgPrintf("Checksum=%x=%x ",checksum,c);
            if(checksum != c)
            {
                lResult = FALSE;
                dbgPrintf("\nchecksum  err\n");
            }
            XMODEM_SendToModule(tempData);
            // todo jw://while(BusyUART());
            if(lResult == TRUE)
            {
                _SYS_CONSOLE_PUTC(XMODEM_ACK);//WriteUART(XMODEM_ACK);
                preBlockNum++;
            }
            else
            {
                _SYS_CONSOLE_PUTC(XMODEM_NAK);//WriteUART(XMODEM_NAK);
            }
            state = SM_FIRMWARE_UPDATE_SOH;
            break;

        default:
            dbgPrintf("\n!error\n");
            while(1);
            break;
        }

    }
    
    AutoUpdate_Completed();

    return TRUE;
}

#endif
#endif
