/***********************************************************************************
FILE NAME   : SerialCommManagement.C
DESCRIPTION : Serial Communication Management drivers

Copyright   : 2006 Renesas Technology Corporation.
          All Rights Reserved

***********************************************************************************/

/***********************************************************************************
Revision History
DD.MM.YYYY OSO-UID Description
07.07.2006 RSO-PIS First Release
***********************************************************************************/

/**********************************************************************************
System Includes
***********************************************************************************/

/**********************************************************************************
User Includes
***********************************************************************************/
#include "srl_comm_management.h"


/************************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
************************************************************************************/
#define SRL_COMM_BUFF_SIZE_MAX      50


#define SRL_COMM_GET_RESOURCE       DISABLE_IRQ
#define SRL_COMM_RELEASE_RESOURCE   ENABLE_IRQ


/************************************************************************************
*************************************************************************************
* Private prototypes
*************************************************************************************
************************************************************************************/


/************************************************************************************
*************************************************************************************
* Private type definitions
*************************************************************************************
************************************************************************************/
typedef struct
{
  uint8_t readIdx;
  uint8_t writeIdx;
  uint8_t nAvailableBytes;
  uint8_t len;
  uint8_t getlen;
  uint8_t data[SRL_COMM_BUFF_SIZE_MAX];

} gSrlCommBuff_t;


/************************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
************************************************************************************/
gSrlCommBuff_t  gSrlCommBuffRx = {0,0,0,0,0,{0}};
static bool_t gDataAvailable = FALSE;

#ifdef TEST_MODE_1_SRL0
uint16_t nRx=0;
#endif


/************************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/

/**********************************************************************************
Function Name:
Description:
Parameters:   none
Return value:   none
***********************************************************************************/
void SrlCommManagmntInit(void)
{
  SrlCommManagmntFlushBuffRx();
  UartInitialize();

}
/**********************************************************************************
End of function
***********************************************************************************/

/**********************************************************************************
Function Name:
Description:
Parameters:   none
Return value:   none
***********************************************************************************/
void SrlCommManagmntFlushBuffRx( void )
{
  SRL_COMM_GET_RESOURCE;

  gDataAvailable = FALSE;
  gSrlCommBuffRx.len = 0;
  gSrlCommBuffRx.getlen = FALSE;
  gSrlCommBuffRx.readIdx = 0;
  gSrlCommBuffRx.writeIdx = 0;
  gSrlCommBuffRx.nAvailableBytes = 0;

  SRL_COMM_RELEASE_RESOURCE;

}
/***********************************************************************************
End of function SrlCommManagmntFlushBuffRx
***********************************************************************************/

/**********************************************************************************
Function Name:
Description:
Parameters:   none
Return value:   none
***********************************************************************************/
SrlCommErr_t SrlCommManagmntWriteData(uint8_t *pBuff, uint8_t bufferSize)
{
  uint32_t u32loop;
  UartErr_t uartErr;


  if (pBuff == NULL)
    return gSrlCommErrNullPointer_c;

  if (bufferSize > SRL_COMM_BUFF_SIZE_MAX)
    return gSrlCommErrInvalidSize_c;

  // Nothing to send
  if (bufferSize == 0)
    return gSrlCommErrNoError_c;

  // OK to send.
  uartErr = UartWriteData (pBuff, bufferSize);

  /* add a tempo to avoid sending 2 successive uart messages */
  for(u32loop=0;u32loop<0x05FF;u32loop++);

  if(uartErr != gUartErrNoError_c)
    return gSrlCommErrWriteError_c;

  return gSrlCommErrNoError_c;
}
/***********************************************************************************
End of function SrlCommManagmntWriteData
***********************************************************************************/

/**********************************************************************************
Function Name:
Description:
Parameters:   none
Return value:   none
***********************************************************************************/
SrlCommErr_t SrlCommManagmntReadData(uint8_t *pBuff, uint8_t bufferSize, uint8_t *nDataRcvd)
{
  uint8_t i;

  if (pBuff == NULL || nDataRcvd == NULL)
    return gSrlCommErrNullPointer_c;

  if (bufferSize > SRL_COMM_BUFF_SIZE_MAX || bufferSize == 0)
    return gSrlCommErrInvalidSize_c;

  // Disable serial interrupt to avoid memory access sharing problem.
  //SRL_COMM_GET_RESOURCE;
  if(!gDataAvailable)
  {
    *nDataRcvd = 0;
    return gSrlCommErrNoError_c;
  }

  // Wait for the serial interrupt subroutine to be finished
  // Is there enough available data ?
  if (gSrlCommBuffRx.nAvailableBytes < bufferSize)
  {
    // There is no enough data.
    *nDataRcvd = gSrlCommBuffRx.nAvailableBytes;
    // Enable serial interrupt
    //SRL_COMM_RELEASE_RESOURCE;

    return gSrlCommErrNoError_c;
  }
  else
  {
    *nDataRcvd = bufferSize;

    // Refresh Global Rx Buffer
    gSrlCommBuffRx.nAvailableBytes -= bufferSize;
    if(!gSrlCommBuffRx.nAvailableBytes)
      gDataAvailable = FALSE;

    // Copy Rx Buffer to pBuffer Data
    for(i=0;i<bufferSize;i++)
    {
      // Copy one byte
      pBuff[i] = gSrlCommBuffRx.data[ gSrlCommBuffRx.readIdx ];

      // Increment the data index and roll over if necessary
      gSrlCommBuffRx.readIdx = (gSrlCommBuffRx.readIdx + 1) % SRL_COMM_BUFF_SIZE_MAX;

    };
  };

  // Enable serial interrupt
  //SRL_COMM_RELEASE_RESOURCE;

  return gSrlCommErrNoError_c;
}
/***********************************************************************************
End of function SrlCommManagmntReadData
***********************************************************************************/

/*****************************************************************************
Name:    UART Receive Interrupt Routine
Parameters:  none
Returns:     none
Description: Interrupt routine for UART receive
             Reads character received and stores in Buff
*****************************************************************************/
void EUSART0_RX_IRQHandler(void)
{
  uint16_t u16RxData;
  if(!gSrlCommBuffRx.getlen)
  {
#ifdef TEST_MODE_1_SRL0
    nRx++;
#endif
    SRL_COMM_RECEIVE_DATA( u16RxData );
    gSrlCommBuffRx.len = u16RxData & 0x00FF;

    if( u16RxData & 0xF000 )
    {
      u16RxData &= 0x00FF;
    }

    if(gSrlCommBuffRx.len)
    {
      gSrlCommBuffRx.nAvailableBytes = gSrlCommBuffRx.len;
      gSrlCommBuffRx.getlen = TRUE;
    }

  }
  else
  {
    SRL_COMM_RECEIVE_DATA( u16RxData );
    gSrlCommBuffRx.data[gSrlCommBuffRx.writeIdx] = u16RxData & 0x00FF;
    if( u16RxData & 0xF000 )
    {
      u16RxData &= 0x00FF;
    }

    gSrlCommBuffRx.writeIdx = (gSrlCommBuffRx.writeIdx + 1) % SRL_COMM_BUFF_SIZE_MAX;

    gSrlCommBuffRx.len--;
    if(!gSrlCommBuffRx.len)
    {
      gSrlCommBuffRx.getlen = FALSE;
      gDataAvailable = TRUE;
    }
  }
  EUSART_IntClear(EUSART0,EUSART_IF_RXFL);
}
/***********************************************************************************
End of ISR function U1rec_ISR
***********************************************************************************/

