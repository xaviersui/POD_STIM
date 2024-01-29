/***********************************************************************************
FILE NAME   : spi.h
DESCRIPTION : SPI header file.

Copyright   : 2006 Renesas Technology Corporation.
          All Rights Reserved

***********************************************************************************/

/***********************************************************************************
Revision History
DD.MM.YYYY OSO-UID Description
07.07.2006 RSO-PIS First Release
***********************************************************************************/

#ifndef SPI_H_INCLUDED
#define SPI_H_INCLUDED
#include "sl_spidrv_instances.h"
#include "spidrv.h"
/***********************************************************************************
Macro defines
***********************************************************************************/
#define TR_DATA_LNGTH   5       /* Received Data Length */

#define DATA_BITS_8   1
#define DATA_BITS_16  2

#define SPI_WAIT_TRANSFER_FINISH(remain) {\
  do\
      {\
        SPIDRV_GetTransferStatus(sl_spidrv_usart_CAN_BIO_SPI_handle, NULL, &remain);\
      }\
    while(remain != 0);\
    }\

/**********************************************************************************
Global variables
***********************************************************************************/
extern unsigned char TR_Buff[TR_DATA_LNGTH];        /* Receive Buffer */

/***********************************************************************************
Function Prototypes
***********************************************************************************/
/* Declaration of function prototype */
void spi_master_TrData(unsigned char * Data8, unsigned short length);
                    /* spi master Data Transmission function */
void spi_master_ReData(unsigned char * Data8, unsigned short length);
                    /* spi master Data Reception function */
//void spi_transmit2(unsigned char* data, unsigned short length); /* transmit 8-bits data via spi */;
#endif    /*  SPI_H_INCLUDED  */
