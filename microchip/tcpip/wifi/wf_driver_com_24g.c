/*******************************************************************************
  MRF24W Driver Com Layer (specific to the MRF24WG)

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Provides access to MRF24W WiFi controller
    -Reference: MRF24W Data sheet, IEEE 802.11 Standard
*******************************************************************************/

/*******************************************************************************
FileName:   wf_driver_com_24g.c
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

/*
*********************************************************************************************************
*                                           INCLUDES                               
*********************************************************************************************************
*/

#include "tcpip_private.h"
#include "wf_mac.h"
#if defined(TCPIP_IF_MRF24W)

#include "wf_debug_output.h"


/*
*********************************************************************************************************
*                                           DEFINES      
*********************************************************************************************************
*/

// HOST_RESET_REG masks
#define HR_CPU_RST_N_MASK                     ((uint16_t)0x01 << 15)
#define HR_DISABLE_DOWNLOAD_SCRAMBLER_MASK    ((uint16_t)0x01 << 14)
#define HR_FORCE_CPU_CLK_FREEZE_MASK          ((uint16_t)0x01 << 13)
#define HR_HOST_ANA_SPI_EN_MASK               ((uint16_t)0x01 << 12)
#define HR_HOST_ANA_SPI_DIN_MASK              ((uint16_t)0x01 << 11)
#define HR_HOST_ANA_SPI_DOUT_MASK             ((uint16_t)0x01 << 10)
#define HR_HOST_ANA_SPI_CLK_MASK              ((uint16_t)0x01 << 9)
#define HR_HOST_ANA_SPI_CSN_MASK              ((uint16_t)0x07 << 6)   // 8:6
#define HR_RESERVED_2_MASK                    ((uint16_t)0x01 << 5)
#define HR_HOST_SPI_DISABLE_MASK              ((uint16_t)0x01 << 4)
#define HR_HOST_ENABLE_NEW_PROG_MASK          ((uint16_t)0x01 << 3)
#define HR_HOST_ENABLE_DOWNLOAD_MASK          ((uint16_t)0x01 << 2)
#define HR_HOST_FAST_RESET_MASK               ((uint16_t)0x01 << 1)
#define HR_HOST_RESET_MASK                    ((uint16_t)0x01 << 0)

// This block of defines needed to restart PLL
#define ANALOG_PORT_3_REG_TYPE           ((uint32_t)0x09)   /* 16-bit analog register in SPI Port 3                                          */
#define ANALOG_PORT_2_REG_TYPE           ((uint32_t)0x08)   /* 16-bit analog register in SPI Port 2                                          */
#define ANALOG_PORT_1_REG_TYPE           ((uint32_t)0x0a)   /* 16-bit analog register in SPI Port 1                                          */
#define ANALOG_PORT_0_REG_TYPE           ((uint32_t)0x0b)   /* 16-bit analog register in SPI Port 0                                          */

#define SPI_WRITE_MASK                          (uint8_t)0x00     // bit 0 = 0
#define SPI_READ_MASK                           (uint8_t)0x01     // bit 0 = 1
#define SPI_AUTO_INCREMENT_ENABLED_MASK         (uint8_t)0x00     // bit 1 = 0
#define SPI_AUTO_INCREMENT_DISABLED_MASK        (uint8_t)0x02     // bit 1 = 1

#define PLL9_REG   ((uint32_t)(9 * 2)) // SPI Port 3 Registers (Port 5 if going through Master SPI controller)
#define PLL8_REG   ((uint32_t)(8 * 2))
#define PLL7_REG   ((uint32_t)(7 * 2))
#define PLL6_REG   ((uint32_t)(6 * 2)) 
#define PLL5_REG   ((uint32_t)(5 * 2)) 
#define PLL4_REG   ((uint32_t)(4 * 2)) 
#define PLL3_REG   ((uint32_t)(3 * 2)) 
#define PLL2_REG   ((uint32_t)(2 * 2)) 
#define PLL1_REG   ((uint32_t)(1 * 2)) 
#define PLL0_REG   ((uint32_t)(0 * 2)) 
// end PLL block

#define OSC0_REG            ((uint32_t)(0 * 2)) 
#define OSC1_REG            ((uint32_t)(1 * 2)) 
#define OSC2_REG            ((uint32_t)(2 * 2)) 
#define PLDO_REG            ((uint32_t)(3 * 2))
#define BIAS_REG            ((uint32_t)(4 * 2))
#define ANALOG_SPARE_REG    ((uint32_t)(5 * 2))



/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES      
*********************************************************************************************************
*/

/* Functions that are called from the External Interrupt routine use these global */
/* variables instead of local variables to avoid stack corruption on CPU's that   */
/* that use overlay memory.  See note in WFEintHandler() function.                */
static uint8_t g_txBuf[3];
static uint8_t g_rxBuf[3];

static uint8_t g_HostIntSaved = 0;

// Keep these as static globals instead of local variables in the Eint Handler.
// If declared as local variables, causes stack corruption in PIC18
static uint8_t  g_EintHostIntRegValue;
static uint8_t  g_EintHostIntMaskRegValue;
static uint8_t  g_EintHostInt;

static bool             g_MgmtReadMsgReady;                  /* true if rx mgmt msg to process, else false              */

static bool g_EnablePowerSaveMode = false;

tRawMoveState RawMoveState;

extern bool g_WaitingForMgmtResponse;
bool g_DhcpSuccessful = false;
bool g_WiFiConnectionChanged = false;
bool g_WiFiConnection = false;
bool g_dhcpInProgress = false;

extern t_psPollContext g_savedPsPollContext;

/*
*********************************************************************************************************
*                                       LOCAL FUNCTION PROTOTYPES      
*********************************************************************************************************
*/

static void ProcessMgmtRxMsg(void);
static void ProcessInterruptServiceResult(void);
static uint8_t GetSpiPortWithBitBang(uint8_t regType);
static void WriteAnalogRegisterBitBang(uint8_t regType, uint16_t address, uint16_t value);
#if !defined(WF_EVENT_DRIVEN)
static void ResetPll(void);
#endif

#if defined(WF_EVENT_DRIVEN)
void ResetPll(void);
#endif


static bool isDhcpInProgress(void);

extern bool isSleepNeeded(void);
extern void SetSleepNeeded(void);
extern void ClearSleepNeeded(void);
extern bool GetAppPowerSaveMode(void);


uint8_t g_powerSaveState;



// called when an event occurs that may affect PS-Poll mode, and, the application
// has enabled power save mode.
void InitPowerSaveTask(void)
{
    g_DhcpSuccessful = false;

}

void PowerSaveTask(void)
{
    TCPIP_NET_IF *p_config;

    // if application has disabled power save mode
    if (GetAppPowerSaveMode() == false)
    {
        return;
    }

#if 0
    // if currently waiting for a mgmt msg response
    if (g_WaitingForMgmtResponse)
    {
        // run this task later so no mgmt msg conflict
        WifiAsyncSetEventPending(ASYNC_POWER_SAVE_PENDING);
        return;
    }
#endif

    p_config = (TCPIP_NET_IF *)GetNetworkConfig();;

    // if just got IP address from DHCP
    if ((p_config->Flags.bIsDHCPEnabled == true) && (g_DhcpSuccessful == true))
    {
        // now save to enable PS-Poll mode
        g_DhcpSuccessful = false;
        WF_PsPollEnable(&g_savedPsPollContext);
    }
    // else if just lost or regained connection to WiFi network
    else if (g_WiFiConnectionChanged)
    {
        g_WiFiConnectionChanged = false;

        // if lost connection
        if (g_WiFiConnection == false)
        {
           WFConfigureLowPowerMode(WF_LOW_POWER_MODE_OFF);
        }
        // else connected (or reconnected)
        else
        {
            // if not using DHCP
            if (p_config->Flags.bIsDHCPEnabled == false)
            {
                WF_PsPollEnable(&g_savedPsPollContext);
            }   
            // note: if using DHCP, another case will reenable PS-Poll mode
        }
    }
    // if application wants PS-Poll, but the WiFi driver temporarily disabled it 
    // to send a message
    else if (isSleepNeeded())
    {
        // if have not lost connection or started DCHP task in interim
        if ((g_WiFiConnection == true) && !isDhcpInProgress())
        {

            WFConfigureLowPowerMode(WF_LOW_POWER_MODE_ON);
        }
        // else wait for connection event or DHCP event before enabling PS-Poll

        ClearSleepNeeded();
    }
}

static bool isDhcpInProgress(void)
{
    return g_dhcpInProgress;
}


void SignalDHCPSuccessful(void)
{
    g_DhcpSuccessful = true;
    g_dhcpInProgress = false;

    // if application has requested PS-Poll mode
    if (GetAppPowerSaveMode() == true)
    {
        // run the power save task to restart PS-Poll mode now that DHCP completed
        WifiAsyncSetEventPending(ASYNC_POWER_SAVE_PENDING);
    }
}    


void SetDhcpProgressState(void)
{
    g_dhcpInProgress = true;
}    

void SignalWiFiConnectionChanged(bool state)
{
    g_WiFiConnectionChanged = true;
    g_WiFiConnection = state;
    // if application has requested PS-Poll mode
    if (GetAppPowerSaveMode() == true)
    {
        // run the power save task to restart PS-Poll mode now that DHCP completed
        WifiAsyncSetEventPending(ASYNC_POWER_SAVE_PENDING);
    }
}

void WiFi_EintTask(void)
{
    uint16_t len;

    ProcessInterruptServiceResult();

    if (g_MgmtReadMsgReady == true)
    {
        g_MgmtReadMsgReady = false;

        // mount raw mgmt msg and process it
        RawGetMgmtRxBuffer(&len);
        ProcessMgmtRxMsg();
        // reenable interrupts
        WF_EintEnable();
    }
}


/*****************************************************************************
 * FUNCTION: ProcessInterruptServiceResult
 *
 * RETURNS: N/A
 *
 * PARAMS:
 *      N/A
 *
 *
 *  NOTES: Processes EXINT from MRF24W. 
 *****************************************************************************/
static void ProcessInterruptServiceResult(void)
{
    uint8_t  hostIntRegValue;
    uint8_t  hostIntMaskRegValue;
    uint8_t  hostInt;

    /* read hostInt register to determine cause of interrupt */
    hostIntRegValue = Read8BitWFRegister(WF_HOST_INTR_REG);

    // OR in the saved interrupts during the time when we were waiting for raw complete, set by WFEintHandler()
    hostIntRegValue |= g_HostIntSaved;
    
    // done with the saved interrupts, clear variable
    g_HostIntSaved = 0;


    hostIntMaskRegValue  = Read8BitWFRegister(WF_HOST_MASK_REG);

    // AND the two registers together to determine which active, enabled interrupt has occurred
    hostInt = hostIntRegValue & hostIntMaskRegValue;

    // If received a level 2 interrupt (Raw move complete occurred or module assert or mgmt Rx)
    // handle module assert and mgmt rx (raw move complete already handled in lower-level
    // interrupt).
    if (hostInt & WF_HOST_INT_MASK_INT2)
    {
        uint16_t hostInt2;

        hostInt2 = Read16BitWFRegister(WF_HOST_INTR2_REG);
        if (hostInt2 & WF_HOST_INT_MASK_MAIL_BOX_1_WRT)
        {
            //SYS_CONSOLE_MESSAGE("Mail Box1 Event\r\n");
        }
		

        SYS_ASSERT(false,""); /* This should not happen. Other than the module assert, this interrupt is only used to     */
                              /* signify RAW Move Complete for RAW2/3/4.  This event should be cleared in EintHandler().  */

        /* clear this interrupt */
        Write16BitWFRegister(WF_HOST_INTR2_REG, WF_HOST_INT_MASK_INT2);
        //WF_EintEnable();
    }

    /* else if got a FIFO 1 Threshold interrupt (Management Fifo).  Mgmt Rx msg ready to proces. */
    else if((hostInt & WF_HOST_INT_MASK_FIFO_1_THRESHOLD) == WF_HOST_INT_MASK_FIFO_1_THRESHOLD)
    {
        /* clear this interrupt */
        Write8BitWFRegister(WF_HOST_INTR_REG, WF_HOST_INT_MASK_FIFO_1_THRESHOLD);
        
        // notify MAC state machine that management message needs to be processed
        g_MgmtReadMsgReady = true; 
    }
    // else if got a FIFO 0 Threshold Interrupt (Data Fifo)
    else if((hostInt & WF_HOST_INT_MASK_FIFO_0_THRESHOLD) == WF_HOST_INT_MASK_FIFO_0_THRESHOLD)
    {
        /* clear this interrupt */
        Write8BitWFRegister(WF_HOST_INTR_REG, WF_HOST_INT_MASK_FIFO_0_THRESHOLD);

#if !defined(WF_EVENT_DRIVEN)
        /* notify MAC state machine that data message needs to be processed */
        g_HostRAWDataPacketReceived = true;
#endif

#if defined(WF_EVENT_DRIVEN)
        // start the RAW mount of the Rx data packet
        RawMountRxDataBuffer();
#endif
    }
    // else got a Host interrupt that we don't handle
    else if(hostInt)
    {
        /* clear this interrupt */
        Write8BitWFRegister(WF_HOST_INTR_REG, hostInt);
        WF_EintEnable();
    }
    // we got a spurious interrupt (no bits set in register)
    else
    {
        /* spurious interrupt */
        WF_EintEnable();
    }
}


/*****************************************************************************
 * FUNCTION: Read8BitWFRegister
 *
 * RETURNS: register value
 *
 * PARAMS:
 *      regId -- ID of 8-bit register being read
 *
 *  NOTES: Reads WF 8-bit register
 *****************************************************************************/
uint8_t Read8BitWFRegister(uint8_t regId)
{
    g_txBuf[0] = regId | WF_READ_REGISTER_MASK;
    WF_SpiEnableChipSelect();
    
    WF_SpiTxRx(g_txBuf, 
              1,
              g_rxBuf,
              2);

    WF_SpiDisableChipSelect();
    
    return g_rxBuf[1];   /* register value returned in the second byte clocking */
}

/*****************************************************************************
 * FUNCTION: Write8BitWFRegister
 *
 * RETURNS: None
 *
 * PARAMS:
 *      regId -- ID of 8-bit register being written to
 *      value -- value to write
 *
 *  NOTES: Writes WF 8-bit register
 *****************************************************************************/
void Write8BitWFRegister(uint8_t regId, uint8_t value)
{
    g_txBuf[0] = regId | WF_WRITE_REGISTER_MASK;
    g_txBuf[1] = value;
    
    WF_SpiEnableChipSelect();

    WF_SpiTxRx(g_txBuf, 
              2,
              g_rxBuf,
              1);

    WF_SpiDisableChipSelect();
}

/*****************************************************************************
 * FUNCTION: Read16BitWFRegister
 *
 * RETURNS: register value
 *
 * PARAMS:
 *      regId -- ID of 16-bit register being read
 *
 *  NOTES: Reads WF 16-bit register
 *****************************************************************************/
uint16_t Read16BitWFRegister(uint8_t regId)
{
    g_txBuf[0] = regId | WF_READ_REGISTER_MASK;
    WF_SpiEnableChipSelect();
    
    WF_SpiTxRx(g_txBuf, 
              1,
              g_rxBuf,
              3);

    WF_SpiDisableChipSelect();
    
    return (((uint16_t)g_rxBuf[1]) << 8) | ((uint16_t)(g_rxBuf[2]));
}

/*****************************************************************************
 * FUNCTION: Write16BitWFRegister
 *
 * RETURNS: None
 *
 * PARAMS:
 *      regId -- ID of 16-bit register being written to
 *      value -- value to write
 *
 *  NOTES: Writes WF 16-bit register
 *****************************************************************************/
void Write16BitWFRegister(uint8_t regId, uint16_t value)
{
    g_txBuf[0] = regId | WF_WRITE_REGISTER_MASK;
    g_txBuf[1] = (uint8_t)(value >> 8);       /* MS byte being written     */
    g_txBuf[2] = (uint8_t)(value & 0x00ff);   /* LS byte being written     */
    
    WF_SpiEnableChipSelect();

    WF_SpiTxRx(g_txBuf, 
              3,
              g_rxBuf,
              1);

    WF_SpiDisableChipSelect();
}

/*****************************************************************************
 * FUNCTION: WriteWFArray
 *
 * RETURNS: None
 *
 * PARAMS:
 *      regId  -- Raw register being written to
 *      pBuf   -- pointer to array of bytes being written
 *      length -- number of bytes in pBuf
 *
 *  NOTES: Writes a data block to specified raw register
 *****************************************************************************/
void WriteWFArray(uint8_t regId, uint8_t *p_Buf, uint16_t length)
{
    g_txBuf[0] = regId;

    WF_SpiEnableChipSelect();

    /* output cmd byte */
    WF_SpiTxRx(g_txBuf, 
              1,
              g_rxBuf,
              1);

    /* output data array bytes */
    WF_SpiTxRx(p_Buf, 
              length,
              g_rxBuf,
              1);

    WF_SpiDisableChipSelect();
}

/*****************************************************************************
 * FUNCTION: ReadWFArray
 *
 * RETURNS: None
 *
 * PARAMS:
 *      regId  -- Raw register being read from
 *      pBuf   -- pointer where to write out bytes
 *      length -- number of bytes to read
 *
 *  NOTES: Reads a block of data from a raw register
 *****************************************************************************/
void ReadWFArray(uint8_t  regId, uint8_t *p_Buf, uint16_t length)
{
    WF_SpiEnableChipSelect();
    
    /* output command byte */
    g_txBuf[0] = regId | WF_READ_REGISTER_MASK;
    WF_SpiTxRx(g_txBuf, 
              1,
              g_rxBuf,
              1);

    /* read data array */
    WF_SpiTxRx(g_txBuf, 
              1,   /* garbage tx byte */
              p_Buf,
              length);

    WF_SpiDisableChipSelect();
}




/*****************************************************************************
 * FUNCTION: MRF24W_ChipReset
 *
 * RETURNS: N/A
 *
 * PARAMS:
 *      N/A
 *
 *
 *  NOTES: Performs the necessary SPI operations to cause the MRF24W to do a soft
 *         reset.
 *
 *         This function waits for the MRF24WG to complete its initialization before
 *         returning to the caller.  The largest part of the wait is for the MRF24WG
 *         to download any patch code in FLASH into its RAM.
 *****************************************************************************/
void MRF24W_ChipReset(void)
{
    uint16_t value;
    SYS_TICK timeoutPeriod;
    SYS_TICK startTickCount;

    /* clear the power bit to disable low power mode on the MRF24W */
    Write16BitWFRegister(WF_PSPOLL_H_REG, 0x0000);

    /* Set HOST_RESET bit in register to put device in reset */
    Write16BitWFRegister(WF_HOST_RESET_REG, Read16BitWFRegister(WF_HOST_RESET_REG) | WF_HOST_RESET_MASK);

    /* Clear HOST_RESET bit in register to take device out of reset */
    Write16BitWFRegister(WF_HOST_RESET_REG, Read16BitWFRegister(WF_HOST_RESET_REG) & ~WF_HOST_RESET_MASK);

    /* after reset is started poll register to determine when HW reset has completed */
    timeoutPeriod = SYS_TICK_TicksPerSecondGet() * 3;  /* 3000 ms */
    startTickCount = SYS_TICK_Get();  
    do
    {
        Write16BitWFRegister(WF_INDEX_ADDR_REG, WF_HW_STATUS_REG);
        value = Read16BitWFRegister(WF_INDEX_DATA_REG);
        if (SYS_TICK_Get() - startTickCount >= timeoutPeriod)
        {
            SYS_ASSERT(false, "");
        }   
    } while ( (value & WF_HW_STATUS_NOT_IN_RESET_MASK) == 0);

    
    /* if SPI not connected will read all 1's */
    SYS_ASSERT(value != 0xffff, "");

    /* now that chip has come out of HW reset, poll the FIFO byte count register     */
    /* which will be set to a non-zero value when the MRF24W initialization is   */
    /* complete.                                                                     */
    startTickCount = SYS_TICK_Get();
    do
    {
        value = Read16BitWFRegister(WF_HOST_WFIFO_BCNT0_REG);
        if (SYS_TICK_Get() - startTickCount >= timeoutPeriod)
        {
            SYS_ASSERT(false, "");
        } 
    } while (value == 0);
}


#if !defined(WF_EVENT_DRIVEN)
/*****************************************************************************
 * FUNCTION: HostInterrupt2RegInit
 *
 * RETURNS: N/A
 *
 * PARAMS:
 *      hostIntrMaskRegMask - The bit mask to be modified.
 *      state               - One of WF_INT_DISABLE, WF_INT_ENABLE where
 *                             Disable implies clearing the bits and enable sets the bits.
 *
 *
 *  NOTES: Initializes the 16-bit Host Interrupt register on the MRF24W with the
 *      specified mask value either setting or clearing the mask register
 *      as determined by the input parameter state. 
 *****************************************************************************/
static void HostInterrupt2RegInit(uint16_t hostIntMaskRegMask,
                                  uint8_t  state)
{
    uint16_t int2MaskValue;

    /* Host Int Register is a status register where each bit indicates a specific event  */
    /* has occurred. In addition, writing a 1 to a bit location in this register clears  */
    /* the event.                                                                        */

    /* Host Int Mask Register is used to enable those events activated in Host Int Register */
    /* to cause an interrupt to the host                                                    */

    /* read current state of int2 mask reg */
    int2MaskValue = Read16BitWFRegister(WF_HOST_INTR2_MASK_REG);

    /* if caller is disabling a set of interrupts */
    if (state == WF_INT_DISABLE)
    {
        /* zero out that set of interrupts in the interrupt mask copy */
        int2MaskValue &= ~hostIntMaskRegMask;
    }
    /* else caller is enabling a set of interrupts */
    else
    {
        /* set to 1 that set of interrupts in the interrupt mask copy */
        int2MaskValue |= hostIntMaskRegMask;
    }
    
    /* write out new interrupt mask value */
    Write16BitWFRegister(WF_HOST_INTR2_MASK_REG, int2MaskValue);

    /* ensure that pending interrupts from those updated interrupts are cleared */
    Write16BitWFRegister(WF_HOST_INTR2_REG, hostIntMaskRegMask);
    
}

/*****************************************************************************
 * FUNCTION: HostInterruptRegInit
 *
 * RETURNS: N/A
 *
 * PARAMS:
 *      hostIntrMaskRegMask - The bit mask to be modified.
 *      state -  one of WF_EXINT_DISABLE, WF_EXINT_ENABLE where
 *                Disable implies clearing the bits and enable sets the bits.
 *
 *
 *  NOTES: Initializes the 8-bit Host Interrupt register on the MRF24W with the
 *      specified mask value either setting or clearing the mask register
 *      as determined by the input parameter state.  The process requires
 *      2 spi operations which are performed in a blocking fashion.  The
 *      function does not return until both spi operations have completed.
 *****************************************************************************/
static void HostInterruptRegInit(uint8_t hostIntrMaskRegMask,
                                 uint8_t state)
{
    uint8_t hostIntMaskValue;

    /* Host Int Register is a status register where each bit indicates a specific event  */
    /* has occurred. In addition, writing a 1 to a bit location in this register clears  */
    /* the event.                                                                        */

    /* Host Int Mask Register is used to enable those events activated in Host Int Register */
    /* to cause an interrupt to the host                                                    */

    /* read current state of Host Interrupt Mask register */
    hostIntMaskValue = Read8BitWFRegister(WF_HOST_MASK_REG);

    /* if caller is disabling a set of interrupts */
    if (state == WF_INT_DISABLE)
    {
        /* zero out that set of interrupts in the interrupt mask copy */
        hostIntMaskValue = (hostIntMaskValue & ~hostIntrMaskRegMask);
    }
    /* else caller is enabling a set of interrupts */
    else
    {
        /* set to 1 that set of interrupts in the interrupt mask copy */
        hostIntMaskValue = (hostIntMaskValue & ~hostIntrMaskRegMask) | hostIntrMaskRegMask;
    }

    /* write out new interrupt mask value */
    Write8BitWFRegister(WF_HOST_MASK_REG, hostIntMaskValue);

    /* ensure that pending interrupts from those updated interrupts are cleared */
    Write8BitWFRegister(WF_HOST_INTR_REG, hostIntrMaskRegMask);
    
    
}
#endif // WF_EVENT_DRIVEN

/*****************************************************************************
 * FUNCTION: WFEintHandler
 *
 * RETURNS: N/A
 *
 * PARAMS:
 *      N/A
 *
 *
 *  NOTES: This function must be called once, each time an external interrupt
 *      is received from the WiFi device.   The WiFi Driver will schedule any
 *      subsequent SPI communication to process the interrupt.
 *
 * IMPORTANT NOTE: This function, and functions that are called by this function
 *                 must NOT use local variables.  The PIC18, or any other processor
 *                 that uses overlay memory will corrupt the logical stack within
 *                 overlay memory if the interrupt uses local variables.  
 *                 If local variables are used within an interrupt routine the toolchain 
 *                 cannot properly determine how not to overwrite local variables in 
 *                 non-interrupt releated functions, specifically the function that was 
 *                 interrupted.
 *****************************************************************************/
void WFEintHandler(void)
{
    /*--------------------------------------------------------*/    
    /* if driver is waiting for a RAW Move Complete interrupt */
    /*--------------------------------------------------------*/
    if (RawMoveState.waitingForRawMoveCompleteInterrupt)
    {
        /* read hostInt register and hostIntMask register to determine cause of interrupt */
        g_EintHostIntRegValue      = Read8BitWFRegister(WF_HOST_INTR_REG);
        g_EintHostIntMaskRegValue  = Read8BitWFRegister(WF_HOST_MASK_REG);
    
        // AND the two registers together to determine which active, enabled interrupt has occurred
        g_EintHostInt = g_EintHostIntRegValue & g_EintHostIntMaskRegValue;

        /* if a RAW0/RAW1 Move Complete interrupt occurred or a level 2 interrupt occurred, indicating */
        /* that a RAW2-5 Move Complete interrupt occurred                                           */
        if (g_EintHostInt & (WF_HOST_INT_MASK_RAW_0_INT_0 | WF_HOST_INT_MASK_RAW_1_INT_0 | WF_HOST_INT_MASK_INT2))
        {
#if defined(WF_EVENT_DRIVEN)
            // if got a Raw Move complete for Data Rx raw window
            if ((g_EintHostInt & WF_HOST_INT_MASK_RAW_0_INT_0) == WF_HOST_INT_MASK_RAW_0_INT_0)
            {
                // trigger event so WiFi_DataRxTask() will run 
                WifiAsyncSetEventPending(ASYNC_DATA_RX_PENDING);
            }

            // need this check for for Data Tx Raw Window (RAW1) because we unmount scratch window from RAW1
            // at initialization, and don't want false data tx trigger
            if (isInitComplete())
            {
                // if got a Raw Move complete for Data Tx window
                if ((g_EintHostInt & WF_HOST_INT_MASK_RAW_1_INT_0) == WF_HOST_INT_MASK_RAW_1_INT_0)
                {
                    // trigger event so WiFi_DataRxTask() will run
                    WifiAsyncSetEventPending(ASYNC_DATA_TX_PENDING);
                }
            }
#endif // WF_EVENT_DRIVEN
            /* save the copy of the active interrupts */
            RawMoveState.rawInterrupt = g_EintHostInt;
            RawMoveState.waitingForRawMoveCompleteInterrupt = false;
            
            /* if no other interrupts occurred other than a RAW0/RAW1/RAW2/RAW3/RAW4 Raw Move Complete */
            if((g_EintHostInt & ~(WF_HOST_INT_MASK_RAW_0_INT_0 | WF_HOST_INT_MASK_RAW_1_INT_0 | WF_HOST_INT_MASK_INT2)) == 0)
            {
                /* clear the RAW interrupts, re-enable interrupts, and exit */
                Write8BitWFRegister(WF_HOST_INTR_REG, (WF_HOST_INT_MASK_RAW_0_INT_0 | 
                                                       WF_HOST_INT_MASK_RAW_1_INT_0 | 
                                                       WF_HOST_INT_MASK_INT2));
                
                Write16BitWFRegister(WF_HOST_INTR2_REG, (WF_HOST_INT_MASK_RAW_2_INT_0 | 
                                                         WF_HOST_INT_MASK_RAW_3_INT_0 | 
                                                         WF_HOST_INT_MASK_RAW_4_INT_0 |
                                                         WF_HOST_INT_MASK_RAW_5_INT_0));

                WF_EintEnable();
                return;
            }
            /* else we got a RAW0/RAW1/RAW2/RAW3/RAW4/RAW5 Raw Move Complete interrupt, but, there is also at */
            /* least one other interrupt present                                                               */
            else
            {
                // save the other interrupts and clear them, along with the Raw Move Complete interrupts
                // keep interrupts disabled
                Write16BitWFRegister(WF_HOST_INTR2_REG, (WF_HOST_INT_MASK_RAW_2_INT_0   | 
                                                         WF_HOST_INT_MASK_RAW_3_INT_0   | 
                                                         WF_HOST_INT_MASK_RAW_4_INT_0   |
                                                         WF_HOST_INT_MASK_RAW_5_INT_0));
                
                g_HostIntSaved |= (g_EintHostInt & ~(WF_HOST_INT_MASK_RAW_0_INT_0 |
                                                     WF_HOST_INT_MASK_RAW_1_INT_0 |
                                                     WF_HOST_INT_MASK_INT2));
                
                Write8BitWFRegister(WF_HOST_INTR_REG, g_EintHostInt);
            }
        }
        /*--------------------------------------------------------------------------------------------------*/        
        /* else we did not get a 'RAW Move Complete' interrupt, but we did get at least one other interrupt */
        /*--------------------------------------------------------------------------------------------------*/
        else
        {
            g_HostIntSaved |= g_EintHostInt;
            Write8BitWFRegister(WF_HOST_INTR_REG, g_EintHostInt);
            WF_EintEnable();
        }
    }

    // Once we're in here, external interrupts have already been disabled so no need to call WF_EintDisable() in here
    #if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
    WifiAsyncSetEventPending(ASYNC_EINT_PENDING);
    #endif
}

void ReenablePowerSaveMode(void)
{
    g_EnablePowerSaveMode = true;
}

#if !defined(WF_EVENT_DRIVEN)
/*****************************************************************************
 * FUNCTION: WFHardwareInit
 *
 * RETURNS:  error code
 *
 * PARAMS:   None
 *
 *  NOTES:   Initializes CPU Host hardware interfaces (SPI, External Interrupt).
 *           Also resets the MRF24W.
 *****************************************************************************/
bool WFHardwareInit(void)
{
    uint8_t  mask8;
    uint16_t mask16;
    g_MgmtReadMsgReady = false;

    RawMoveState.rawInterrupt  = 0;
    RawMoveState.waitingForRawMoveCompleteInterrupt = false;   /* not waiting for RAW move complete */

    /* initialize the SPI interface */
    if(!WF_SpiInit())
    {
        return false;
    }
    
    /* Toggle the module into and then out of hibernate */
    WF_SetCE_N(WF_HIGH); /* disable module */
    WF_SetCE_N(WF_LOW);  /* enable module  */

    /* Toggle the module into and out of reset */
    WF_SetRST_N(WF_LOW);            // put module into reset
    WF_SetRST_N(WF_HIGH);           // take module out of of reset

    ResetPll();  // needed until PLL fix made in A2 silicon

    /* Soft reset the MRF24W (using SPI bus to write/read MRF24W registers */
    MRF24W_ChipReset();
    
    /* disable the interrupts gated by the 16-bit host int register */
    HostInterrupt2RegInit(WF_HOST_2_INT_MASK_ALL_INT, (uint16_t)WF_INT_DISABLE);
    
    /* disable the interrupts gated the by main 8-bit host int register */
    HostInterruptRegInit(WF_HOST_INT_MASK_ALL_INT, WF_INT_DISABLE);
    
    /* Initialize the External Interrupt allowing the MRF24W to interrupt */
    /* the Host from this point forward.                                                       */
    WF_EintInit();
    WF_EintEnable();
    
    
    /* enable the following MRF24W interrupts in the INT1 8-bit register */
    mask8 = (WF_HOST_INT_MASK_FIFO_1_THRESHOLD |     /* Mgmt Rx Msg interrupt                  */
             WF_HOST_INT_MASK_FIFO_0_THRESHOLD |     /* Data Rx Msg interrupt                  */
             WF_HOST_INT_MASK_RAW_0_INT_0      |     /* RAW0 Move Complete (Data Rx) interrupt */
             WF_HOST_INT_MASK_RAW_1_INT_0      |     /* RAW1 Move Complete (Data Tx) interrupt */
             WF_HOST_INT_MASK_INT2);                 /* Interrupt 2 interrupt                  */
    HostInterruptRegInit(mask8, WF_INT_ENABLE);

    /* enable the following MRF24W interrupts in the INT2 16-bit register */
    mask16 = (WF_HOST_INT_MASK_RAW_2_INT_0     |    /* RAW2 Move Complete (Mgmt Rx) interrupt */
              WF_HOST_INT_MASK_RAW_3_INT_0     |    /* RAW3 Move Complete (Mgmt Tx) interrupt */
              WF_HOST_INT_MASK_RAW_4_INT_0     |    /* RAW4 Move Complete (Scratch) interrupt */
              WF_HOST_INT_MASK_RAW_5_INT_0     |    /* RAW5 Move Complete (Scratch) interrupt */
              WF_HOST_INT_MASK_MAIL_BOX_1_WRT);
    HostInterrupt2RegInit(mask16, WF_INT_ENABLE);

     /* Disable PS-Poll mode */
    WFConfigureLowPowerMode(WF_LOW_POWER_MODE_OFF);

    return true;
}
#endif // WF_EVENT_DRIVEN


static void ProcessMgmtRxMsg(void)
{
    uint8_t msgType;

    /* read first byte from Mgmt Rx message (msg type) */
    RawRead(RAW_MGMT_RX_ID, 0, 1, &msgType);
    
    /* if not a management response or management indicate then fatal error */
    SYS_ASSERT( (msgType == WF_MGMT_CONFIRM_TYPE) || (msgType == WF_MGMT_INDICATE_TYPE), "" );

    if (msgType == WF_MGMT_CONFIRM_TYPE)
    {
        /* signal that a mgmt response has been received */
        SignalMgmtConfirmReceivedEvent();
    }
    else  /* must be WF_MGMT_INDICATE_TYPE */
    {
        /* handle the mgmt indicate */
        WFProcessMgmtIndicateMsg();
    }    
}



// When bit-banging, determines which SPI port to use based on the type of register we are accessing
static uint8_t GetSpiPortWithBitBang(uint8_t regType)
{
    if (regType == ANALOG_PORT_3_REG_TYPE)
    {
        return 2;
    }
    else if (regType == ANALOG_PORT_2_REG_TYPE)
    {
        return 3;
    }
    else if (regType == ANALOG_PORT_1_REG_TYPE)
    {
        return 1;
    }
    else if (regType == ANALOG_PORT_0_REG_TYPE)
    {
        return 0;
    }
    else
    {
        return 0xff; // should never happen
    }
}

static void WriteAnalogRegisterBitBang(uint8_t regType, uint16_t address, uint16_t value)
{
    uint8_t  spiPort;
    uint16_t hrVal;
    uint8_t  bitMask8;
    uint16_t bitMask16;
    uint8_t  i;
    uint8_t  regAddress;

    spiPort = GetSpiPortWithBitBang(regType);   // extract SPI port (0-3) from the register type

    // Enable the on-chip SPI and select the desired bank (0-3)
    hrVal = (HR_HOST_ANA_SPI_EN_MASK | (spiPort << 6));
    Write16BitWFRegister(WF_HOST_RESET_REG, hrVal);

    // create register address byte
    regAddress = (address << 2) | SPI_AUTO_INCREMENT_ENABLED_MASK | SPI_WRITE_MASK;

    // bit-bang the regAddress byte, MS bit to LS bit
    bitMask8 = 0x80;        // start with MS bit of byte being bit-banged out
    for (i = 0; i < 8; ++i)
    {
        hrVal &= ~(HR_HOST_ANA_SPI_DOUT_MASK | HR_HOST_ANA_SPI_CLK_MASK); // zero out DOUT and CLK

        // mask out ADDRESS bit being clocked and write to HOST_ANA_SPI_DOUT (bit 10) in HOST_RESET_REG with the HOST_ANA_SPI_CLK low
        hrVal |= (regAddress & bitMask8) << (3 + i);  // first time: bit 7 << 3, second time: bit 6 << 4, etc.
        Write16BitWFRegister(WF_HOST_RESET_REG, hrVal);

        // now toggle SPI clock high, on rising edge this bit is clocked out
        hrVal |= HR_HOST_ANA_SPI_CLK_MASK;
        Write16BitWFRegister(WF_HOST_RESET_REG, hrVal);

        bitMask8 >>= 1; //  # get to next bit in address byte
    }

    // bit bang data from MS bit to LS bit
    bitMask16 = 0x8000;        // start with MS bit of byte being bit-banged out
    for (i = 0; i < 16; ++i)
    {
        hrVal &= ~(HR_HOST_ANA_SPI_DOUT_MASK | HR_HOST_ANA_SPI_CLK_MASK); // zero out DOUT and CLK

        // mask in data bit being clock out and write to HOST_ANA_SPI_DOUT (bit 10) in HOST_RESET_REG with the HOST_ANA_SPI_CLK low
        if ((15 - i) >= 10) // bits 15:10 need to be right-shifted
        {
            hrVal |= (value & bitMask16) >> (5 - i);  // first time: bit 15 << 5, second time: bit  14 << 4, etc.
        }
        else // bits 10:0 need to be left-shifted
        {
            hrVal |= (value & bitMask16) << (i - 5);        // first time: bit 10 << 0, second time: bit  9 << 1, etc.
        }

        Write16BitWFRegister(WF_HOST_RESET_REG, hrVal);

        // now toggle SPI clock high, on rising edge this bit is clocked out
        hrVal |= HR_HOST_ANA_SPI_CLK_MASK;
        Write16BitWFRegister(WF_HOST_RESET_REG, hrVal);

        bitMask16 = bitMask16 >> 1;  // go to next bit in data byte
    }

    // Disable the on-chip SPI
    hrVal &= ~HR_HOST_ANA_SPI_EN_MASK;
    Write16BitWFRegister(WF_HOST_RESET_REG, hrVal);
}

#if !defined(WF_EVENT_DRIVEN)
static void ResetPll(void)
#endif
#if defined(WF_EVENT_DRIVEN)
void ResetPll(void)
#endif
{
    // shuttle Ostrich workaround (won't affect production parts)
    WriteAnalogRegisterBitBang(ANALOG_PORT_3_REG_TYPE, PLL0_REG, 0x8021);
    WriteAnalogRegisterBitBang(ANALOG_PORT_3_REG_TYPE, PLL0_REG, 0x6021);
    
    // production Ostrich workaround (won't affect shuttle parts)
    WriteAnalogRegisterBitBang(ANALOG_PORT_1_REG_TYPE, OSC0_REG, 0x6b80);
    WriteAnalogRegisterBitBang(ANALOG_PORT_1_REG_TYPE, BIAS_REG, 0xc000);

}    



#endif /* TCPIP_IF_MRF24W */




