/**********************************************************************************
FILE NAME       Uart.c
DESCRIPTION:    Main routine for UART sample code

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
#include "uart.h"
#include "em_cmu.h"
#include "em_eusart.h"
#include "pin_config.h"

/******************************************************************************
*******************************************************************************
* Private Macros
*******************************************************************************
******************************************************************************/
#define UART_TRANFERT_DATA_8_BITS_MODE    5         /**< transfer data is 8 bits long. */


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


/************************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
************************************************************************************/


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

/************************************************************************************
* UartInitialize function
*
* Initializes settings to use UART bus interface.
************************************************************************************/
void UartInitialize(void)
{
  DISABLE_IRQ;
  CMU_ClockEnable(cmuClock_EUSART0,true);
  CMU_ClockEnable(cmuClock_GPIO,true);

  EUSART_UartInit_TypeDef init = EUSART_UART_INIT_DEFAULT_HF;
  init.parity = eusartOddParity;
  init.stopbits = eusartStopbits2;

  GPIO_PinModeSet(COM_RF_UART_RX_PORT,COM_RF_UART_RX_PIN, gpioModeInputPull,1);
  GPIO_PinModeSet(COM_RF_UART_TX_PORT,COM_RF_UART_TX_PIN, gpioModePushPull,1);

  GPIO->EUSARTROUTE[0].TXROUTE = (COM_RF_UART_TX_PORT << _GPIO_USART_TXROUTE_PORT_SHIFT) |
                                 (COM_RF_UART_TX_PIN << _GPIO_USART_TXROUTE_PIN_SHIFT);

  GPIO->EUSARTROUTE[0].RXROUTE = (COM_RF_UART_RX_PORT << _GPIO_EUSART_RXROUTE_PORT_SHIFT) |
                                   (COM_RF_UART_RX_PIN << _GPIO_EUSART_RXROUTE_PIN_SHIFT);

  // Enable RX and TX signals now that they have been routed
  GPIO->EUSARTROUTE[0].ROUTEEN = GPIO_EUSART_ROUTEEN_RXPEN | GPIO_EUSART_ROUTEEN_TXPEN;

  EUSART_UartInitHf(EUSART0,&init);
  NVIC_SetPriority(EUSART0_RX_IRQn,0);

  NVIC_ClearPendingIRQ(EUSART0_RX_IRQn);
  NVIC_EnableIRQ(EUSART0_RX_IRQn);
  EUSART_IntEnable(EUSART0,EUSART_IF_RXFL);
  ENABLE_IRQ;
}
/**********************************************************************************
End of function
***********************************************************************************/

/*****************************************************************************
Name:        UartWriteData
Parameters:  msg -> the message to output
Returns:     none
Description: The following sends a message
*****************************************************************************/
UartErr_t UartWriteData (uint8_t *msg, uint8_t nData)
{
  char i;

  if (msg == NULL)
    return gUartErrNullPointer_c;

  if (nData == 0)
    return gUartErrNoError_c;

  /* This loop reads in the mess and puts it in the UART 0 transmit buffer */
  for (i=0; i<nData; i++)
  {
      EUSART_Tx(EUSART0,*(msg + i));
  }

  return gUartErrNoError_c;
}

/***********************************************************************************
End of function text_write
***********************************************************************************/
