/***********************************************************************************
FILE NAME   : Srl_Comm_Management.H
DESCRIPTION : Serial Communication Management header file

Copyright   : 2006 Renesas Technology Corporation.
          All Rights Reserved

***********************************************************************************/

/***********************************************************************************
Revision History
DD.MM.YYYY OSO-UID Description
07.07.2006 RSO-PIS First Release
***********************************************************************************/


#ifndef SRL_COMM_MANAGEMENT_H_INCLUDED
#define SRL_COMM_MANAGEMENT_H_INCLUDED

/***********************************************************************************
System Includes
***********************************************************************************/

/***********************************************************************************
User Includes
***********************************************************************************/
/* sfr_r827.h provides a structure to access all of the device registers. */
// #include "sfr_r827.h"
/* RSKR8C27def.h provides common defines for widely used items. */
#include "rskR8C27def.h"

#include "uart.h"


/************************************************************************************
*************************************************************************************
* Public macros
*************************************************************************************
************************************************************************************/
#define SRL_COMM_DATA_SIZE_MAX      31    /**< Maximal size of the Serial Communication Transmit Buffer. */


#define SRL_COMM_TRANSMIT_DATA(Data8) UART0_TRANSMIT_DATA(Data8)
#define SRL_COMM_RECEIVE_DATA(Data8)  UART0_RECEIVE_DATA(Data8)

/************************************************************************************
*************************************************************************************
* Public type definitions
*************************************************************************************
************************************************************************************/
/** This data type enumerates the UART API calls return values */
typedef enum
{
    gSrlCommErrNoError_c = 0,
    gSrlCommErrNullPointer_c,
    gSrlCommErrInvalidSize_c,
    gSrlCommErrWriteError_c,
    /*
  gSrlCommErrNoAvailableBytes_c,

  gSrlCommErrUartAlreadyOpen_c,
    gSrlCommErrUartNotOpen_c,
    gSrlCommErrNoCallbackDefined_c,
    gSrlCommErrReadOngoing_c,
    gSrlCommErrWriteOngoing_c,
    gSrlCommErrInvalidClock_c,
    gSrlCommErrInvalidBaudrate_c,
    gSrlCommErrInvalidParity_c,
    gSrlCommErrInvalidStop_c,
    gSrlCommErrInvalidCTS_c,
    gSrlCommErrInvalidThreshold_c,
    gSrlCommErrWrongUartNumber_c,
    */
  gSrlCommErrMax_c
} SrlCommErr_t;


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
extern void SrlCommManagmntInit(void);
extern void SrlCommManagmntFlushBuffRx(void);
extern SrlCommErr_t SrlCommManagmntWriteData(uint8_t *pBuff, uint8_t bufferSize);
extern SrlCommErr_t SrlCommManagmntReadData(uint8_t *pBuff, uint8_t bufferSize, uint8_t *nDataRcvd);


#endif    /*  SRL_COMM_MANAGEMENT_H_INCLUDED  */
