/***********************************************************************************
FILE NAME   : Uart.h
DESCRIPTION : Uart header file.

Copyright   : 2006 Renesas Technology Corporation.
          All Rights Reserved

***********************************************************************************/

/***********************************************************************************
Revision History
DD.MM.YYYY OSO-UID Description
07.07.2006 RSO-PIS First Release
***********************************************************************************/

#ifndef UART_H_INCLUDED
#define UART_H_INCLUDED

/***********************************************************************************
System Includes
***********************************************************************************/

/***********************************************************************************
User Includes
***********************************************************************************/
/* sfr_r827.h provides a structure to access all of the device registers. */
#include "em_eusart.h"
#include "uartdrv.h"
#include "sl_uartdrv_instances.h"
/* RSKR8C27def.h provides common defines for widely used items. */
#include "rskR8C27def.h"


/************************************************************************************
*************************************************************************************
* Public macros
*************************************************************************************
************************************************************************************/
#define BAUD_RATE_2400    2400
#define BAUD_RATE_4800    4800
#define BAUD_RATE_9600    9600
#define BAUD_RATE_19200   19200
#define BAUD_RATE_38400   38400
#define BAUD_RATE_57600   57600
#define BAUD_RATE_65000   65000
#define BAUD_RATE_115200  115200

#define BAUD_RATE_DEFAULT BAUD_RATE_65000

/* read data into Receive Data Register */
#define UART0_RECEIVE_DATA(Data8)    (Data8 = EUSART_Rx(EUSART0))

/* select transfer clock rate */

/************************************************************************************
*************************************************************************************
* Public type definitions
*************************************************************************************
************************************************************************************/
typedef enum
{
    gUartErrNoError_c = 0,
    gUartErrNullPointer_c,
    gUartErrInvalidSize_c,
    /*
  gUartErrNoAvailableBytes_c,
  gUartErrUartAlreadyOpen_c,
    gUartErrUartNotOpen_c,
    gUartErrNoCallbackDefined_c,
    gUartErrReadOngoing_c,
    gUartErrWriteOngoing_c,
    gUartErrInvalidClock_c,
    gUartErrInvalidBaudrate_c,
    gUartErrInvalidParity_c,
    gUartErrInvalidStop_c,
    gUartErrInvalidCTS_c,
    gUartErrInvalidThreshold_c,
    gUartErrWrongUartNumber_c,
    */
  gUartErrMax_c
} UartErr_t;

/************************************************************************************
*************************************************************************************
* Public prototypes
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

/***********************************************************************************
Function Prototypes
***********************************************************************************/

UartErr_t UartWriteData (uint8_t *msg, uint8_t nData);

/************************************************************************************
* Name :  UartInitialize  *//**
* @brief  Initializes settings to use UART bus interface.
* @param  .
* @return .
************************************************************************************/
void UartInitialize(void);



#endif /* UART_H_INCLUDED */
